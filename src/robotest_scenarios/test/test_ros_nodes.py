# Copyright 2026 Hasan Ahmed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Focused in-process ROS graph adapter tests."""

import hashlib
import inspect
import json
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import robotest_scenarios.scenario_controller as scenario_controller
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.exceptions import ParameterImmutableException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from robotest_scenarios.artifacts import canonical_json_bytes, write_canonical_json
from robotest_scenarios.constants import (
    ACTION_STATUS_TOPIC,
    CONTROL_ARM_REQUEST_MAX_BYTES,
    CONTROL_ARM_SCHEMA_VERSION,
    CONTROL_COMMAND_PERIOD_NS,
    CONTROL_COMMAND_PROGRESS_MAX_BYTES,
    CONTROL_COMMAND_PROGRESS_SCHEMA_VERSION,
    CONTROL_COMMAND_REQUIRED_SUBSCRIPTION_COUNT,
    CONTROL_REVERSE_MPS,
    ExitCode,
)
from robotest_scenarios.contact_control_driver import (
    ContactControlApp,
    ContactControlNode,
    _decode_arm_request,
    _decode_command_progress,
    _read_regular_nonsymlink,
)
from robotest_scenarios.contact_evidence import load_coverage_manifest
from robotest_scenarios.errors import (
    InfrastructureError,
    ProtocolError,
    ScenarioFailureError,
    WallTimeoutError,
)
from robotest_scenarios.models import load_scenario
from robotest_scenarios.provenance import contact_control_arm_protocol_sha256
from robotest_scenarios.scenario_controller import ScenarioControllerApp, ScenarioControllerNode
from ros_gz_interfaces.msg import Contact, Contacts
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

REPOSITORY = Path(__file__).resolve().parents[3]


def _clock(stamp_ns: int) -> Clock:
    message = Clock()
    message.clock.sec, message.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
    return message


def _local_clock_endpoint_gids(node: scenario_controller.Node) -> tuple[bytes, ...]:
    return tuple(
        bytes(info.endpoint_gid)
        for info in node.get_subscriptions_info_by_topic('/clock')
        if info.node_name == node.get_name() and info.node_namespace == node.get_namespace()
    )


def _status(raw_uuid: bytes, stamp_ns: int, status_code: int) -> GoalStatusArray:
    item = GoalStatus()
    item.goal_info.goal_id.uuid = list(raw_uuid)
    item.goal_info.stamp.sec, item.goal_info.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    item.status = status_code
    message = GoalStatusArray()
    message.status_list = [item]
    return message


def _init_ros() -> None:
    rclpy.init(args=['--ros-args', '-r', '__ns:=/robotest'])


def _install_cleanup_drain_clock(
    app: object,
    monkeypatch: pytest.MonkeyPatch,
    *,
    on_spin: object | None = None,
) -> SimpleNamespace:
    clock = SimpleNamespace(now_ns=4_000_000_000, spin_count=0)
    monkeypatch.setattr(time, 'monotonic_ns', lambda: clock.now_ns)

    def spin_once(*, timeout_s: float = 0.02, check_fatal: bool = True) -> None:
        assert check_fatal is False
        clock.spin_count += 1
        clock.now_ns += max(1, round(timeout_s * 1_000_000_000))
        if on_spin is not None:
            assert callable(on_spin)
            on_spin(clock.spin_count)

    monkeypatch.setattr(app, '_spin_once', spin_once)
    return clock


class _FakeCommandPublisher:
    def __init__(self) -> None:
        self.matched_subscription_count = 1
        self.published: list[object] = []

    def get_subscription_count(self) -> int:
        return self.matched_subscription_count

    def publish(self, message: object) -> None:
        self.published.append(message)


class _ArmTestNode:
    def __init__(self, acknowledgment_path: Path) -> None:
        self.acknowledgment_path = acknowledgment_path
        self.clock_sample_count = 8
        self.clock_seen = True
        self.control_started_stamp_ns = None
        self.current_sim_stamp_ns = 2_000_000_000
        self.command_publisher = _FakeCommandPublisher()
        self.cleanup_fatal_error = None
        self.fatal_error = None
        self.first_qualifying_contact = None
        self.motion_armed = False

    @staticmethod
    def resolve_topic_name(name: str) -> str:
        return f'/robotest/{name}'

    def arm_motion(self) -> None:
        assert self.acknowledgment_path.is_file()
        assert not self.acknowledgment_path.is_symlink()
        self.motion_armed = True

    def start_forward(self) -> None:
        assert self.motion_armed
        self.control_started_stamp_ns = self.current_sim_stamp_ns


def _arm_test_app(tmp_path: Path) -> ContactControlApp:
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='arm-test',
        wall_timeout_s=30.0,
        service_timeout_s=2.0,
        raw_ros_args=[],
    )
    app.node = _ArmTestNode(app.armed_path)
    app.ready_steady_ns = time.monotonic_ns()
    app.ready_sha256 = 'a' * 64
    return app


def _arm_request(app: ContactControlApp, *, ready_sha256: str | None = None) -> dict[str, object]:
    return {
        'action': 'start_positive_control_motion',
        'arm_protocol_sha256': contact_control_arm_protocol_sha256(),
        'arm_requested_steady_ns': time.monotonic_ns(),
        'producer': 'robotest_phase3/benchmark_runner',
        'ready_sha256': app.ready_sha256 if ready_sha256 is None else ready_sha256,
        'run_id': app.run_id,
        'runtime_gate_sha256': 'b' * 64,
        'schema_version': CONTROL_ARM_SCHEMA_VERSION,
    }


def _command_progress(
    app: ContactControlApp,
    *,
    observed_steady_ns: int | None = None,
    retained_command_count: int = 1,
    run_id: str | None = None,
    stamp_ns: int | None = None,
) -> dict[str, object]:
    return {
        'angular_z_rad_s': 0.0,
        'linear_x_m_s': 0.0,
        'linear_y_m_s': 0.0,
        'observed_steady_ns': (
            time.monotonic_ns() if observed_steady_ns is None else observed_steady_ns
        ),
        'producer': 'robotest_metrics/metrics_collector',
        'public_topic': '/robotest/cmd_vel',
        'retained_command_count': retained_command_count,
        'run_id': app.run_id if run_id is None else run_id,
        'schema_version': CONTROL_COMMAND_PROGRESS_SCHEMA_VERSION,
        'stamp_ns': app._node.current_sim_stamp_ns if stamp_ns is None else stamp_ns,
    }


def test_contact_graph_accepts_pinned_jazzy_rmw_endpoint_gid_size() -> None:
    assert ContactControlApp._endpoint_gid_is_valid('ab' * 16)


@pytest.mark.parametrize('value', ['', 'ab' * 15, 'ab' * 17, 'ab' * 24, 'gg' * 16])
def test_contact_graph_rejects_invalid_rmw_endpoint_gids(value: str) -> None:
    assert not ContactControlApp._endpoint_gid_is_valid(value)


def _spawn_test_app(tmp_path: Path, *, run_id: str) -> ContactControlApp:
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id=run_id,
        wall_timeout_s=30.0,
        service_timeout_s=2.0,
        raw_ros_args=[],
    )
    app.wall_asset = tmp_path / 'wall.sdf'
    app.wall_asset_sha256 = 'a' * 64
    return app


def test_contact_spawn_success_returns_complete_ready_and_result_projection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _spawn_test_app(tmp_path, run_id='spawn-projection-test')
    sequences = iter((41, 42))
    future = SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(success=True),
    )
    app.node = SimpleNamespace(
        _next_sequence=lambda: next(sequences),
        current_sim_stamp_ns=5_000_000_000,
        resolve_topic_name=lambda name: name,
        spawn_client=SimpleNamespace(call_async=lambda _request: future),
    )
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: app.wall_deadline)

    spawn = app._spawn_wall()
    expected_keys = {
        'attempt_count',
        'error',
        'request_sequence',
        'request_stamp_ns',
        'response_sequence',
        'response_stamp_ns',
        'success',
    }
    assert set(spawn) == expected_keys
    assert spawn == app.setup_evidence['spawn']
    assert spawn is not app.setup_evidence['spawn']
    assert spawn['error'] is None

    app.observed_start = {'sim_stamp_ns': 4_900_000_000}
    observed_wall = {'stamp_ns': 5_000_000_000}
    app.setup_evidence['observed_robot_start'] = app.observed_start
    app.setup_evidence['observed_wall'] = observed_wall
    app._write_ready(spawn, observed_wall)
    ready = json.loads(app.ready_path.read_text(encoding='utf-8'))
    assert ready['spawn'] == app.setup_evidence['spawn']
    assert set(ready['spawn']) == expected_keys

    spawn['attempt_count'] = 999
    assert app.setup_evidence['spawn']['attempt_count'] == 1


def test_contact_spawn_waits_for_one_delayed_response_until_operational_deadline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _spawn_test_app(tmp_path, run_id='delayed-spawn-response-test')
    app.wall_deadline = 30.0
    clock = SimpleNamespace(now=0.0)
    call_count = 0
    sequences = iter((51, 52))
    future = SimpleNamespace(
        done=lambda: clock.now >= 3.0,
        result=lambda: SimpleNamespace(success=True),
    )

    def call_async(_request: object) -> object:
        nonlocal call_count
        call_count += 1
        return future

    app.node = SimpleNamespace(
        _next_sequence=lambda: next(sequences),
        current_sim_stamp_ns=5_000_000_000,
        spawn_client=SimpleNamespace(call_async=call_async),
    )
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: app.wall_deadline)
    monkeypatch.setattr(app, '_spin_once', lambda: setattr(clock, 'now', clock.now + 1.1))
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic',
        lambda: clock.now,
    )

    spawn = app._spawn_wall()

    assert clock.now > app.service_timeout_s
    assert call_count == 1
    assert spawn['success'] is True
    assert spawn['attempt_count'] == 1


def test_contact_spawn_does_not_send_after_service_deadline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _spawn_test_app(tmp_path, run_id='spawn-pre-send-deadline-test')
    clock = SimpleNamespace(now=10.0)
    send_count = 0

    def next_sequence() -> int:
        clock.now = 10.2
        return 41

    def call_async(_request: object) -> object:
        nonlocal send_count
        send_count += 1
        return SimpleNamespace()

    app.node = SimpleNamespace(
        _next_sequence=next_sequence,
        current_sim_stamp_ns=5_000_000_000,
        spawn_client=SimpleNamespace(call_async=call_async),
    )
    app.wall_deadline = 100.0
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: 10.1)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    with pytest.raises(InfrastructureError, match='service unavailable before deadline'):
        app._spawn_wall()

    assert send_count == 0
    assert app.setup_evidence['spawn']['request_sequence'] == 41
    assert isinstance(app.setup_evidence['spawn']['error'], str)


def test_scenario_spawn_does_not_send_after_service_deadline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=0.1,
        raw_ros_args=[],
    )
    app.actor_asset = tmp_path / 'actor.sdf'
    clock = SimpleNamespace(now=10.0)
    send_count = 0

    def next_sequence() -> int:
        clock.now = 10.2
        return 41

    def call_async(_request: object) -> object:
        nonlocal send_count
        send_count += 1
        return SimpleNamespace()

    app.node = SimpleNamespace(
        _next_sequence=next_sequence,
        current_sim_stamp_ns=5_000_000_000,
        spawn_client=SimpleNamespace(call_async=call_async),
    )
    app.wall_deadline = 100.0
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: 10.1)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    with pytest.raises(InfrastructureError, match='service unavailable before deadline'):
        app._spawn_actor()

    assert send_count == 0
    assert app.spawn_evidence['request_sequence'] == 41
    assert isinstance(app.spawn_evidence['error'], str)


def test_scenario_spawn_rejects_response_observed_after_service_deadline(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=0.1,
        raw_ros_args=[],
    )
    app.actor_asset = tmp_path / 'actor.sdf'
    clock = SimpleNamespace(now=10.0)

    def response_done() -> bool:
        clock.now = 10.2
        return True

    app.node = SimpleNamespace(
        _next_sequence=iter((41,)).__next__,
        current_sim_stamp_ns=5_000_000_000,
        spawn_client=SimpleNamespace(
            call_async=lambda _request: SimpleNamespace(
                done=response_done,
                result=lambda: SimpleNamespace(success=True),
            )
        ),
    )
    app.wall_deadline = 100.0
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: 10.1)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    with pytest.raises(InfrastructureError, match='response timed out'):
        app._spawn_actor()

    assert app.actor_spawn_request_sent is True
    assert 'response timed out' in str(app.spawn_evidence['error'])


@pytest.mark.parametrize('app_kind', ['contact', 'scenario'])
def test_spawn_call_exception_retains_conservative_cleanup_obligation(
    app_kind: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    send_count = 0
    delete_call_count = 0
    sequences = iter((41, 43))

    def call_async(_request: object) -> object:
        nonlocal send_count
        send_count += 1
        raise RuntimeError('post-send client failure')

    if app_kind == 'contact':
        app = _spawn_test_app(tmp_path, run_id='ambiguous-contact-spawn-test')
        app.node = SimpleNamespace(
            _next_sequence=lambda: next(sequences),
            current_sim_stamp_ns=5_000_000_000,
            delete_client=object(),
            spawn_client=SimpleNamespace(call_async=call_async),
        )
        attempted_attr = 'wall_spawn_request_send_attempted'
        sent_attr = 'wall_spawn_request_sent'
        spawn = app._spawn_wall
        cleanup = app._cleanup_wall
        cleanup_error = 'rejected contact wall cleanup'
    else:
        document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
        app = ScenarioControllerApp(
            document=document,
            output_path=tmp_path / 'result.json',
            ready_path=tmp_path / 'ready.json',
            identity={},
            wall_timeout_s=30.0,
            service_timeout_s=2.0,
            raw_ros_args=[],
        )
        app.actor_asset = tmp_path / 'actor.sdf'
        app.node = SimpleNamespace(
            _next_sequence=lambda: next(sequences),
            current_sim_stamp_ns=5_000_000_000,
            delete_client=object(),
            spawn_client=SimpleNamespace(call_async=call_async),
        )
        attempted_attr = 'actor_spawn_request_send_attempted'
        sent_attr = 'actor_spawn_request_sent'
        spawn = app._spawn_actor
        cleanup = app._cleanup_actor
        cleanup_error = 'rejected actor cleanup'

    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: app.wall_deadline)

    with pytest.raises(InfrastructureError, match='request could not be sent'):
        spawn()

    assert send_count == 1
    assert getattr(app, attempted_attr) is True
    assert getattr(app, sent_attr) is False
    assert app.cleanup['proof'] == {'kind': 'spawn_request_send_attempted_cleanup_pending'}
    app.node.count_publishers = lambda _topic: (_ for _ in ()).throw(
        RuntimeError('cleanup reached')
    )

    def reject_delete(*_args: object, **_kwargs: object) -> tuple[object, int, int]:
        nonlocal delete_call_count
        delete_call_count += 1
        return SimpleNamespace(success=False), 42, 5_100_000_000

    monkeypatch.setattr(app, '_call_service', reject_delete)
    with pytest.raises(InfrastructureError, match=cleanup_error):
        cleanup()
    assert delete_call_count == 1
    assert app.delete_attempt_count == 1


def test_contact_spawn_timeout_runs_reserved_exactly_once_cleanup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = SimpleNamespace(now=100.0, now_ns=100_000_000_000)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic',
        lambda: clock.now,
    )
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic_ns',
        lambda: clock.now_ns,
    )
    app = _spawn_test_app(tmp_path, run_id='spawn-wall-deadline-test')
    assert app.wall_deadline == 125.0
    assert app.complete_wall_deadline == 130.0
    spawn_call_count = 0
    delete_call_count = 0
    future = SimpleNamespace(done=lambda: False)

    def spawn_call_async(_request: object) -> object:
        nonlocal spawn_call_count
        spawn_call_count += 1
        return future

    class FakeNode:
        def __init__(self) -> None:
            self.cleanup_started = False
            self.clock_seen = False
            self.current_sim_stamp_ns = 5_000_000_000
            self.delete_client = object()
            self.post_delete_entity_latest_sim_stamp_ns = None
            self.post_delete_entity_message_count = 0
            self.post_delete_wall_pose_count = 0
            self.post_delete_wall_pose_first_sequence = None
            self.post_delete_wall_pose_first_sim_stamp_ns = None
            self.post_delete_wall_pose_latest_sequence = None
            self.post_delete_wall_pose_latest_sim_stamp_ns = None
            self.delete_response_sequence = None
            self.sequence = 60
            self.spawn_client = SimpleNamespace(call_async=spawn_call_async)

        def _next_sequence(self) -> int:
            self.sequence += 1
            return self.sequence

        def begin_cleanup(self) -> None:
            self.cleanup_started = True

        @staticmethod
        def count_publishers(_topic: str) -> int:
            return 1

        @staticmethod
        def destroy_node() -> None:
            return None

    class FakeExecutor:
        @staticmethod
        def add_node(_node: object) -> None:
            return None

        @staticmethod
        def remove_node(_node: object) -> None:
            return None

        @staticmethod
        def shutdown(*, timeout_sec: float) -> None:
            assert timeout_sec == 1.0

    node = FakeNode()
    executor = FakeExecutor()
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.ContactControlNode',
        lambda _manifest: node,
    )
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.SingleThreadedExecutor',
        lambda: executor,
    )
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.rclpy.init', lambda **_: None)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.rclpy.try_shutdown', lambda: None
    )
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: app.wall_deadline)

    def spin_to_operational_deadline(
        timeout_s: float = 0.02,
        *,
        check_fatal: bool = True,
    ) -> None:
        if node.delete_response_sequence is not None:
            assert check_fatal is False
            clock.now_ns += max(1, round(timeout_s * 1_000_000_000))
            return
        clock.now += 5.0
        app._require_wall_budget()

    def call_delete_once(*_args: object, **_kwargs: object) -> tuple[object, int, int]:
        nonlocal delete_call_count
        delete_call_count += 1
        assert app.cleanup_window_active
        assert app.wall_deadline == 130.0
        assert clock.now == 125.0
        node.sequence = 62
        return SimpleNamespace(success=True), 62, node.current_sim_stamp_ns

    def prove_absence(*_args: object, **_kwargs: object) -> None:
        node.current_sim_stamp_ns += 250_000_000
        node.post_delete_entity_latest_sequence = node._next_sequence()
        node.post_delete_entity_latest_sim_stamp_ns = node.current_sim_stamp_ns
        node.post_delete_entity_message_count += 1

    def result(error: object) -> tuple[dict[str, object], int]:
        assert isinstance(error, InfrastructureError)
        assert str(error) == 'scenario/spawn_entity response timed out'
        assert app.cleanup['actor_absent'] is True
        return {}, 22

    monkeypatch.setattr(app, '_spin_once', spin_to_operational_deadline)
    monkeypatch.setattr(app, '_prepare', lambda: app._spawn_wall())
    monkeypatch.setattr(app, '_wait_for_arm', lambda: pytest.fail('arm must not run'))
    monkeypatch.setattr(app, '_run_control', lambda: pytest.fail('motion must not run'))
    monkeypatch.setattr(app, '_call_service', call_delete_once)
    monkeypatch.setattr(app, '_wait_for', prove_absence)
    monkeypatch.setattr(app, '_result', result)
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.load_schema', lambda _path: {})
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.write_canonical_json',
        lambda *_args, **_kwargs: None,
    )

    assert app.run() == 22

    assert spawn_call_count == 1
    assert delete_call_count == 1
    assert node.cleanup_started
    assert app.spawn_attempt_count == 1
    assert app.delete_attempt_count == 1
    assert app.cleanup_window_active
    assert app.setup_evidence['spawn']['error'] == (
        'spawn response did not complete before the reserved cleanup window'
    )


def test_contact_arm_waits_for_request_and_strictly_newer_positive_clock(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    spin_count = 0

    def spin_once() -> None:
        nonlocal spin_count
        spin_count += 1
        if spin_count == 1:
            write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)
        elif spin_count == 2:
            node.clock_sample_count += 1
        elif spin_count == 3:
            node.clock_sample_count += 1
            node.current_sim_stamp_ns += 1
        elif spin_count == 4:
            node.command_publisher.matched_subscription_count = (
                CONTROL_COMMAND_REQUIRED_SUBSCRIPTION_COUNT
            )
        elif spin_count == 5:
            write_canonical_json(
                app.command_progress_path,
                _command_progress(app),
                write_sidecar=False,
            )

    monkeypatch.setattr(app, '_spin_once', spin_once)
    app._wait_for_arm()

    assert spin_count == 5
    assert node.motion_armed
    acknowledgment_bytes = app.armed_path.read_bytes()
    acknowledgment = json.loads(acknowledgment_bytes)
    assert acknowledgment_bytes == canonical_json_bytes(acknowledgment)
    assert acknowledgment['arm_observed_clock_sample_count'] == 8
    assert acknowledgment['armed_clock_sample_count'] == 10
    assert acknowledgment['arm_observed_sim_stamp_ns'] == 2_000_000_000
    assert acknowledgment['armed_sim_stamp_ns'] == 2_000_000_001
    assert acknowledgment['arm_request_sha256'] == app.arm_request_sha256
    probe = acknowledgment['command_delivery_probe']
    assert probe['matched_subscription_count'] == 2
    assert probe['required_subscription_count'] == 2
    assert (
        acknowledgment['arm_observed_steady_ns']
        <= probe['match_observed_steady_ns']
        <= probe['probe_publish_started_steady_ns']
        <= probe['probe_publish_returned_steady_ns']
        <= acknowledgment['armed_steady_ns']
    )
    assert (
        probe['probe_publish_started_steady_ns']
        <= probe['collector_progress_observed_steady_ns']
        <= acknowledgment['armed_steady_ns']
    )
    assert abs(probe['collector_progress_stamp_ns'] - probe['probe_sim_stamp_ns']) <= 100_000_000
    assert len(node.command_publisher.published) == 1
    safe_zero = node.command_publisher.published[0]
    assert safe_zero.linear.x == safe_zero.linear.y == safe_zero.angular.z == 0.0
    assert app.armed_acknowledgment_sha256 == hashlib.sha256(acknowledgment_bytes).hexdigest()

    node.first_qualifying_contact = {'sim_stamp_ns': node.current_sim_stamp_ns}
    app._drive_until_contact()
    assert (
        acknowledgment['armed_steady_ns']
        <= app.first_nonzero_publish_started_steady_ns
        <= app.first_nonzero_publish_returned_steady_ns
    )


def test_contact_arm_rejects_more_than_two_command_subscriptions(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.command_publisher.matched_subscription_count = 3
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)

    def fresh_clock() -> None:
        node.clock_sample_count += 1
        node.current_sim_stamp_ns += 1

    monkeypatch.setattr(app, '_spin_once', fresh_clock)
    with pytest.raises(InfrastructureError, match='more than two matched subscriptions'):
        app._wait_for_arm()

    assert node.command_publisher.published == []
    assert app.armed_acknowledgment is None
    assert not node.motion_armed


def test_contact_arm_rejects_stale_command_progress_before_probe(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.command_publisher.matched_subscription_count = 2
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)
    write_canonical_json(
        app.command_progress_path,
        _command_progress(app),
        write_sidecar=False,
    )

    def fresh_clock() -> None:
        node.clock_sample_count += 1
        node.current_sim_stamp_ns += 1

    monkeypatch.setattr(app, '_spin_once', fresh_clock)
    with pytest.raises(ProtocolError, match='command progress path is not fresh'):
        app._wait_for_arm()

    assert node.command_publisher.published == []
    assert not node.motion_armed


@pytest.mark.parametrize(
    ('updates', 'message'),
    [
        ({'run_id': 'wrong-run'}, 'belongs to another run'),
        ({'retained_command_count': 2}, 'not the first command'),
        ({'linear_x_m_s': 0.05}, 'did not retain safe zero'),
        ({'public_topic': '/wrong/cmd_vel'}, 'topic is not frozen'),
    ],
)
def test_contact_arm_rejects_wrong_command_progress(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    updates: dict[str, object],
    message: str,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.command_publisher.matched_subscription_count = 2
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)
    spin_count = 0

    def spin_once() -> None:
        nonlocal spin_count
        spin_count += 1
        if spin_count == 1:
            node.clock_sample_count += 1
            node.current_sim_stamp_ns += 1
        elif node.command_publisher.published:
            progress = _command_progress(app)
            progress.update(updates)
            write_canonical_json(
                app.command_progress_path,
                progress,
                write_sidecar=False,
            )

    monkeypatch.setattr(app, '_spin_once', spin_once)
    with pytest.raises(ProtocolError, match=message):
        app._wait_for_arm()

    assert len(node.command_publisher.published) == 1
    assert app.armed_acknowledgment is None
    assert not node.motion_armed


@pytest.mark.parametrize('sim_delta_ns', [-100_000_001, 100_000_001])
def test_contact_arm_rejects_command_progress_outside_absolute_sim_bracket(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    sim_delta_ns: int,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.command_publisher.matched_subscription_count = 2
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)
    spin_count = 0

    def spin_once() -> None:
        nonlocal spin_count
        spin_count += 1
        if spin_count == 1:
            node.clock_sample_count += 1
            node.current_sim_stamp_ns += 1
        elif node.command_publisher.published:
            write_canonical_json(
                app.command_progress_path,
                _command_progress(app, stamp_ns=node.current_sim_stamp_ns + sim_delta_ns),
                write_sidecar=False,
            )

    monkeypatch.setattr(app, '_spin_once', spin_once)
    with pytest.raises(ProtocolError, match='missed its 100 ms sim bracket'):
        app._wait_for_arm()

    assert not node.motion_armed
    assert not app.armed_path.exists()


def test_command_progress_accepts_callback_before_publish_return_and_lagging_clock(
    tmp_path: Path,
) -> None:
    app = _arm_test_app(tmp_path)
    progress = _command_progress(
        app,
        observed_steady_ns=150,
        stamp_ns=app._node.current_sim_stamp_ns - 100_000_000,
    )

    assert app._validate_command_progress(
        progress,
        probe_publish_started_steady_ns=100,
        probe_sim_stamp_ns=app._node.current_sim_stamp_ns,
        progress_read_complete_steady_ns=250,
    ) == (app._node.current_sim_stamp_ns - 100_000_000, 150)


def test_contact_arm_rejects_progress_that_precedes_probe_start(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.command_publisher.matched_subscription_count = 2
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)
    spin_count = 0

    def spin_once() -> None:
        nonlocal spin_count
        spin_count += 1
        if spin_count == 1:
            node.clock_sample_count += 1
            node.current_sim_stamp_ns += 1
        elif node.command_publisher.published:
            write_canonical_json(
                app.command_progress_path,
                _command_progress(app, observed_steady_ns=1),
                write_sidecar=False,
            )

    monkeypatch.setattr(app, '_spin_once', spin_once)
    with pytest.raises(ProtocolError, match='steady ordering is invalid'):
        app._wait_for_arm()

    assert not node.motion_armed
    assert not app.armed_path.exists()


def test_command_progress_decoder_requires_exact_canonical_schema() -> None:
    progress = {
        'angular_z_rad_s': 0.0,
        'linear_x_m_s': 0.0,
        'linear_y_m_s': 0.0,
        'observed_steady_ns': 2,
        'producer': 'robotest_metrics/metrics_collector',
        'public_topic': '/robotest/cmd_vel',
        'retained_command_count': 1,
        'run_id': 'control-1',
        'schema_version': 1,
        'stamp_ns': 1,
    }
    canonical = canonical_json_bytes(progress)
    assert _decode_command_progress(canonical) == progress
    with pytest.raises(ProtocolError, match='exact canonical'):
        _decode_command_progress(json.dumps(progress).encode('utf-8'))
    with pytest.raises(ProtocolError, match='fields do not match'):
        _decode_command_progress(canonical_json_bytes({**progress, 'extra': 1}))
    with pytest.raises(ProtocolError, match='byte bound'):
        _decode_command_progress(b' ' * (CONTROL_COMMAND_PROGRESS_MAX_BYTES + 1))


def test_command_progress_reader_rejects_symlink(tmp_path: Path) -> None:
    target = tmp_path / 'progress-target.json'
    write_canonical_json(
        target,
        {
            'angular_z_rad_s': 0.0,
            'linear_x_m_s': 0.0,
            'linear_y_m_s': 0.0,
            'observed_steady_ns': 2,
            'producer': 'robotest_metrics/metrics_collector',
            'public_topic': '/robotest/cmd_vel',
            'retained_command_count': 1,
            'run_id': 'control-1',
            'schema_version': 1,
            'stamp_ns': 1,
        },
        write_sidecar=False,
    )
    progress_path = tmp_path / 'command-progress.json'
    progress_path.symlink_to(target)
    with pytest.raises(ProtocolError, match='regular non-symlink'):
        _read_regular_nonsymlink(
            progress_path,
            maximum_bytes=CONTROL_COMMAND_PROGRESS_MAX_BYTES,
            label='contact-control command progress',
        )


def test_contact_arm_timeout_cannot_arm_or_publish(tmp_path: Path) -> None:
    app = _arm_test_app(tmp_path)

    def timeout() -> None:
        raise WallTimeoutError('deadline elapsed')

    app._spin_once = timeout
    with pytest.raises(WallTimeoutError, match='deadline elapsed'):
        app._wait_for_arm()
    assert not app._node.motion_armed
    assert app.arm_request is None
    assert not app.armed_path.exists()


def test_contact_arm_fresh_clock_callback_cannot_cross_wall_deadline(tmp_path: Path) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)

    class CrossingExecutor:
        @staticmethod
        def spin_once(*, timeout_sec: float) -> None:
            assert timeout_sec > 0.0
            node.clock_sample_count += 1
            node.current_sim_stamp_ns += 1
            app.wall_deadline = time.monotonic()

    app.executor = CrossingExecutor()

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._wait_for_arm()

    assert app.arm_request is not None
    assert app.armed_acknowledgment is None
    assert not app.armed_path.exists()
    assert not node.motion_armed
    assert app.first_nonzero_publish_started_steady_ns is None


def test_contact_arm_ack_commit_crossing_deadline_is_retained_without_arming(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)

    def spin_once() -> None:
        node.clock_sample_count += 1
        node.current_sim_stamp_ns += 1
        node.command_publisher.matched_subscription_count = 2
        if node.command_publisher.published and not app.command_progress_path.exists():
            write_canonical_json(
                app.command_progress_path,
                _command_progress(app),
                write_sidecar=False,
            )

    def crossing_write(path: Path, value: object, **kwargs: object) -> str:
        digest = write_canonical_json(path, value, **kwargs)
        if path == app.armed_path:
            app.wall_deadline = time.monotonic()
        return digest

    monkeypatch.setattr(app, '_spin_once', spin_once)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.write_canonical_json',
        crossing_write,
    )

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._wait_for_arm()

    assert app.armed_path.is_file()
    assert app.armed_acknowledgment is not None
    assert (
        app.armed_acknowledgment_sha256 == hashlib.sha256(app.armed_path.read_bytes()).hexdigest()
    )
    assert not node.motion_armed
    assert app.first_nonzero_publish_started_steady_ns is None


def test_contact_first_nonzero_rechecks_wall_deadline_after_ack(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    write_canonical_json(app.arm_path, _arm_request(app), write_sidecar=False)

    def spin_once() -> None:
        node.clock_sample_count += 1
        node.current_sim_stamp_ns += 1
        node.command_publisher.matched_subscription_count = 2
        if node.command_publisher.published and not app.command_progress_path.exists():
            write_canonical_json(
                app.command_progress_path,
                _command_progress(app),
                write_sidecar=False,
            )

    monkeypatch.setattr(app, '_spin_once', spin_once)
    app._wait_for_arm()
    assert app.armed_path.is_file()
    assert node.motion_armed

    app.wall_deadline = time.monotonic()
    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._drive_until_contact()

    assert node.control_started_stamp_ns is None
    assert app.first_nonzero_publish_started_steady_ns is None
    assert app.first_nonzero_publish_returned_steady_ns is None


def test_contact_wait_predicate_cannot_cross_wall_deadline(tmp_path: Path) -> None:
    app = _arm_test_app(tmp_path)
    predicate_calls = 0

    def crossing_predicate() -> bool:
        nonlocal predicate_calls
        predicate_calls += 1
        app.wall_deadline = time.monotonic()
        return True

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._wait_for(crossing_predicate, reason='predicate crossed deadline')

    assert predicate_calls == 1


def test_contact_reverse_rechecks_wall_deadline_before_nonzero(tmp_path: Path) -> None:
    app = _arm_test_app(tmp_path)
    published_commands: list[tuple[float, str]] = []

    def publish_command(linear_x: float, *, phase: str) -> dict[str, int]:
        published_commands.append((linear_x, phase))
        return {'sim_stamp_ns': app._node.current_sim_stamp_ns}

    app._node.publish_command = publish_command
    app.wall_deadline = time.monotonic()

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._reverse_and_release()

    assert published_commands == []


def test_contact_repeated_forward_rechecks_deadline_after_spin_postcheck(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    node.motion_armed = True
    app.armed_acknowledgment = {}
    app.armed_acknowledgment_sha256 = 'a' * 64
    app.wall_deadline = 10.0
    monotonic_values = iter((9.0, 9.0, 9.0, 10.0))
    published_commands: list[tuple[float, str]] = []

    def publish_command(linear_x: float, *, phase: str) -> dict[str, int]:
        published_commands.append((linear_x, phase))
        return {'sim_stamp_ns': node.current_sim_stamp_ns}

    def spin_once() -> None:
        node.current_sim_stamp_ns += CONTROL_COMMAND_PERIOD_NS
        app._require_wall_budget()

    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic',
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(node, 'publish_command', publish_command, raising=False)
    monkeypatch.setattr(app, '_spin_once', spin_once)

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._drive_until_contact()

    assert node.control_started_stamp_ns == 2_000_000_000
    assert published_commands == []


def test_contact_repeated_reverse_rechecks_deadline_after_spin_postcheck(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _arm_test_app(tmp_path)
    node = app._node
    app.wall_deadline = 10.0
    monotonic_values = iter((9.0, 9.0, 9.0, 10.0))
    published_commands: list[tuple[float, str]] = []

    def publish_command(linear_x: float, *, phase: str) -> dict[str, int]:
        published_commands.append((linear_x, phase))
        return {'sim_stamp_ns': node.current_sim_stamp_ns}

    def spin_once() -> None:
        node.current_sim_stamp_ns += CONTROL_COMMAND_PERIOD_NS
        app._require_wall_budget()

    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic',
        lambda: next(monotonic_values),
    )
    monkeypatch.setattr(node, 'publish_command', publish_command, raising=False)
    monkeypatch.setattr(app, '_spin_once', spin_once)

    with pytest.raises(WallTimeoutError, match='operational steady-wall deadline elapsed'):
        app._reverse_and_release()

    assert published_commands == [(CONTROL_REVERSE_MPS, 'REVERSE')]


def test_contact_arm_rejects_canonical_request_with_wrong_ready_binding(tmp_path: Path) -> None:
    app = _arm_test_app(tmp_path)
    write_canonical_json(
        app.arm_path,
        _arm_request(app, ready_sha256='c' * 64),
        write_sidecar=False,
    )
    with pytest.raises(ProtocolError, match='ready artifact'):
        app._wait_for_arm()
    assert not app._node.motion_armed
    assert not app.armed_path.exists()


def test_contact_arm_rejects_noncanonical_oversized_and_symlink_payloads(
    tmp_path: Path,
) -> None:
    app = _arm_test_app(tmp_path)
    request = _arm_request(app)
    app.arm_path.write_text(json.dumps(request), encoding='utf-8')
    with pytest.raises(ProtocolError, match='exact canonical'):
        app._wait_for_arm()

    with pytest.raises(ProtocolError, match='byte bound'):
        _decode_arm_request(b' ' * (CONTROL_ARM_REQUEST_MAX_BYTES + 1))

    app.arm_path.unlink()
    target = tmp_path / 'arm-target.json'
    write_canonical_json(target, request, write_sidecar=False)
    app.arm_path.symlink_to(target)
    with pytest.raises(ProtocolError, match='regular non-symlink'):
        _read_regular_nonsymlink(
            app.arm_path,
            maximum_bytes=CONTROL_ARM_REQUEST_MAX_BYTES,
        )


def test_contact_node_blocks_nonzero_prearm_but_allows_fail_safe_zero() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        zero = node.publish_command(0.0, phase='FAIL_SAFE_ZERO')
        assert zero['linear_x'] == 0.0
        with pytest.raises(ProtocolError, match='preceded the armed acknowledgment'):
            node.publish_command(0.05, phase='FORWARD')
        node.arm_motion()
        forward = node.publish_command(0.05, phase='FORWARD')
        assert forward['linear_x'] == 0.05
        with pytest.raises(ProtocolError, match='exactly once'):
            node.arm_motion()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_controller_binds_only_one_post_ready_uuid() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node._on_clock(_clock(1_000_000_000))
        assert not node.status_message_seen
        node.mark_ready()
        first = uuid.UUID('00000000-0000-0000-0000-000000000001').bytes
        second = uuid.UUID('00000000-0000-0000-0000-000000000002').bytes
        node._on_status(_status(first, 1_100_000_000, GoalStatus.STATUS_ACCEPTED))
        assert node.status_message_seen
        assert node.bound_uuid == first
        assert node.accepted_goal_stamp_ns == 1_100_000_000
        with pytest.raises(ProtocolError, match='more than one'):
            node._on_status(_status(second, 1_200_000_000, GoalStatus.STATUS_ACCEPTED))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_controller_fuses_one_exact_clock_reader_before_guarded_evidence(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observed_ros_stamps: list[int] = []
    original_on_clock = ScenarioControllerNode._on_clock

    def on_clock_after_time_source(node: ScenarioControllerNode, message: Clock) -> None:
        observed_ros_stamps.append(int(node.get_clock().now().nanoseconds))
        original_on_clock(node, message)

    monkeypatch.setattr(ScenarioControllerNode, '_on_clock', on_clock_after_time_source)
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        clock_subscriptions = [item for item in node.subscriptions if item.topic_name == '/clock']
        assert clock_subscriptions == [node.clock_subscription]
        clock_subscription = node.clock_subscription
        clock_endpoint_gids = _local_clock_endpoint_gids(node)
        assert len(clock_endpoint_gids) == 1
        clock_qos = node.clock_subscription.qos_profile
        assert clock_qos.history == HistoryPolicy.KEEP_LAST
        assert clock_qos.depth == 1
        assert clock_qos.reliability == ReliabilityPolicy.BEST_EFFORT
        assert clock_qos.durability == DurabilityPolicy.VOLATILE
        assert node.get_parameter('use_sim_time').value is True
        rejected = node.set_parameters(
            [scenario_controller.Parameter('use_sim_time', value=False)]
        )[0]
        assert rejected.successful is False
        assert 'read-only' in rejected.reason
        assert node.get_parameter('use_sim_time').value is True
        unset = node.set_parameters(
            [
                scenario_controller.Parameter(
                    'use_sim_time',
                    type_=scenario_controller.Parameter.Type.NOT_SET,
                )
            ]
        )[0]
        assert unset.successful is False
        assert 'read-only' in unset.reason
        assert node.get_parameter('use_sim_time').value is True
        assert node.clock_subscription is clock_subscription
        assert [item for item in node.subscriptions if item.topic_name == '/clock'] == [
            clock_subscription
        ]
        assert _local_clock_endpoint_gids(node) == clock_endpoint_gids
        assert 'create_subscription(\n            Clock' not in inspect.getsource(
            ScenarioControllerNode.__init__
        )

        node.clock_subscription.callback(_clock(1_000_000_000))
        node.clock_subscription.callback(_clock(1_050_000_000))
        node.clock_subscription.callback(_clock(1_000_000_000))

        assert observed_ros_stamps == [1_000_000_000, 1_050_000_000, 1_000_000_000]
        assert node.get_clock().now().nanoseconds == 1_000_000_000
        assert node.clock_first_stamp_ns == 1_000_000_000
        assert node.clock_sample_count == 2
        assert node.clock_max_gap_ns == 50_000_000
        assert node.clock_regression_count == 1
        assert isinstance(node.fatal_error, ProtocolError)
        assert str(node.fatal_error) == 'simulation clock regressed'
        assert node.protocol_error_count == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_use_sim_time_cannot_be_undeclared() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        assert node.describe_parameter('use_sim_time').read_only is True
        with pytest.raises(ParameterImmutableException, match='use_sim_time'):
            node.undeclare_parameter('use_sim_time')
        assert node.get_parameter('use_sim_time').value is True
        assert [item for item in node.subscriptions if item.topic_name == '/clock'] == [
            node.clock_subscription
        ]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_clock_fusion_fails_closed_on_endpoint_or_qos_mismatch() -> None:
    exact_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
    exact = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=exact_qos,
        callback=lambda _: None,
    )

    with pytest.raises(RuntimeError, match='exactly one /clock subscription; found 0'):
        scenario_controller._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[]), '/clock', lambda _: None
        )
    with pytest.raises(RuntimeError, match='exactly one /clock subscription; found 2'):
        scenario_controller._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[exact, exact]), '/clock', lambda _: None
        )

    wrong_qos = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT),
        callback=lambda _: None,
    )
    with pytest.raises(RuntimeError, match=r'BEST_EFFORT KEEP_LAST\(1\) VOLATILE'):
        scenario_controller._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[wrong_qos]), '/clock', lambda _: None
        )

    wrong_type = SimpleNamespace(
        topic_name='/clock',
        msg_type=object,
        qos_profile=exact_qos,
        callback=lambda _: None,
    )
    with pytest.raises(RuntimeError, match='rosgraph_msgs/msg/Clock'):
        scenario_controller._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[wrong_type]), '/clock', lambda _: None
        )


def test_scenario_clock_fusion_does_not_cross_time_source_failure_boundary() -> None:
    events: list[str] = []
    exact_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)

    def failed_time_source(_message: Clock) -> None:
        events.append('time_source')
        raise RuntimeError('time source failed')

    subscription = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=exact_qos,
        callback=failed_time_source,
    )
    scenario_controller._fuse_time_source_clock_subscription(
        SimpleNamespace(subscriptions=[subscription]),
        '/clock',
        lambda _message: events.append('evidence'),
    )

    with pytest.raises(RuntimeError, match='time source failed'):
        subscription.callback(Clock())
    assert events == ['time_source']


def test_scenario_constructor_destroys_node_when_clock_fusion_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destroyed_names: list[str] = []
    original_destroy_node = scenario_controller.Node.destroy_node

    def fail_fusion(*_args: object) -> None:
        raise RuntimeError('clock fusion failed')

    def record_destroy(node: scenario_controller.Node) -> None:
        destroyed_names.append(node.get_name())
        original_destroy_node(node)

    monkeypatch.setattr(scenario_controller, '_fuse_time_source_clock_subscription', fail_fusion)
    monkeypatch.setattr(scenario_controller.Node, 'destroy_node', record_destroy)
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    try:
        with pytest.raises(RuntimeError, match='clock fusion failed'):
            ScenarioControllerNode(document)
        assert destroyed_names == ['scenario_controller']
    finally:
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    'executor_failure',
    ('construction', 'add', 'add_and_shutdown_false', 'add_and_shutdown_raise'),
)
def test_scenario_app_cleans_node_after_executor_setup_failure(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    executor_failure: str,
) -> None:
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=1.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    calls: list[str] = []

    class FakeNode:
        def __init__(self) -> None:
            self.bound_uuid = None
            self.context = object()
            self.terminal_status = None

        @staticmethod
        def begin_cleanup() -> None:
            calls.append('begin_cleanup')

        @staticmethod
        def destroy_node() -> None:
            calls.append('destroy_node')

    node = FakeNode()

    class FakeExecutor:
        def __init__(self, *, context: object) -> None:
            assert context is node.context
            calls.append('executor_constructed')
            if executor_failure == 'construction':
                raise RuntimeError('executor construction failed')

        @staticmethod
        def add_node(added_node: object) -> bool:
            assert added_node is node
            calls.append('add_node')
            return False

        @staticmethod
        def remove_node(_removed_node: object) -> None:
            pytest.fail('a refused node must not be removed')

        @staticmethod
        def shutdown(*, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            calls.append('executor_shutdown')
            if executor_failure == 'add_and_shutdown_raise':
                raise RuntimeError('shutdown exploded')
            return executor_failure != 'add_and_shutdown_false'

    monkeypatch.setattr(scenario_controller, 'ScenarioControllerNode', lambda _document: node)
    monkeypatch.setattr(scenario_controller, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(scenario_controller.rclpy, 'init', lambda **_: calls.append('rclpy_init'))
    monkeypatch.setattr(
        scenario_controller.rclpy,
        'try_shutdown',
        lambda: calls.append('rclpy_shutdown'),
    )
    monkeypatch.setattr(app, '_workflow', lambda: pytest.fail('workflow must not start'))

    assert app.run() == 22
    expected = ['rclpy_init', 'executor_constructed', 'begin_cleanup']
    if executor_failure != 'construction':
        expected.insert(2, 'add_node')
        expected.append('executor_shutdown')
    expected.extend(('destroy_node', 'rclpy_shutdown'))
    assert calls == expected
    if executor_failure.startswith('add_and_shutdown'):
        failure = (
            'executor shutdown failed: shutdown exploded'
            if executor_failure.endswith('raise')
            else 'executor shutdown did not complete'
        )
        assert f'scenario_controller: teardown warning: {failure}' in capsys.readouterr().err


def test_scenario_run_normalizes_cleanup_exception_and_still_finalizes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s1_baseline.yaml'))
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    events: list[str] = []

    class FakeNode:
        def __init__(self) -> None:
            self.bound_uuid = None
            self.context = object()
            self.terminal_status = None

        @staticmethod
        def begin_cleanup() -> None:
            events.append('begin_cleanup')

        @staticmethod
        def destroy_node() -> None:
            events.append('destroy_node')

    class FakeExecutor:
        def __init__(self, *, context: object) -> None:
            assert context is node.context

        @staticmethod
        def add_node(added_node: object) -> bool:
            assert added_node is node
            return True

        @staticmethod
        def remove_node(removed_node: object) -> None:
            assert removed_node is node
            events.append('remove_node')

        @staticmethod
        def shutdown(*, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            events.append('shutdown')
            return True

    def result(error: object) -> tuple[dict[str, object], int]:
        assert isinstance(error, InfrastructureError)
        assert 'scenario controller cleanup failed' in str(error)
        events.append('result')
        return {}, int(ExitCode.INFRASTRUCTURE_ERROR)

    node = FakeNode()
    monkeypatch.setattr(scenario_controller, 'ScenarioControllerNode', lambda _document: node)
    monkeypatch.setattr(scenario_controller, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(scenario_controller.rclpy, 'init', lambda **_: events.append('init'))
    monkeypatch.setattr(
        scenario_controller.rclpy,
        'try_shutdown',
        lambda: events.append('rclpy_shutdown'),
    )
    monkeypatch.setattr(app, '_workflow', lambda: events.append('workflow'))
    monkeypatch.setattr(
        app,
        '_cleanup_actor',
        lambda: (_ for _ in ()).throw(RuntimeError('cleanup transport failed')),
    )
    monkeypatch.setattr(app, '_result', result)
    monkeypatch.setattr(scenario_controller, 'load_schema', lambda _path: {})
    monkeypatch.setattr(
        scenario_controller,
        'write_canonical_json',
        lambda *_args, **_kwargs: events.append('write_result'),
    )

    assert app.run() == int(ExitCode.INFRASTRUCTURE_ERROR)
    assert events == [
        'init',
        'workflow',
        'begin_cleanup',
        'result',
        'write_result',
        'remove_node',
        'shutdown',
        'destroy_node',
        'rclpy_shutdown',
    ]


@pytest.mark.parametrize('executor_shutdown_outcome', ('false', 'none', 'raise'))
def test_scenario_app_rejects_incomplete_executor_shutdown(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    executor_shutdown_outcome: str,
) -> None:
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=1.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    calls: list[str] = []

    class FakeNode:
        def __init__(self) -> None:
            self.context = object()

        @staticmethod
        def begin_cleanup() -> None:
            calls.append('begin_cleanup')

        @staticmethod
        def destroy_node() -> None:
            calls.append('destroy_node')

    node = FakeNode()

    class FakeExecutor:
        def __init__(self, *, context: object) -> None:
            assert context is node.context
            calls.append('executor_constructed')

        @staticmethod
        def add_node(added_node: object) -> bool:
            assert added_node is node
            calls.append('add_node')
            return True

        @staticmethod
        def remove_node(removed_node: object) -> None:
            assert removed_node is node
            calls.append('remove_node')

        @staticmethod
        def shutdown(*, timeout_sec: float) -> bool | None:
            assert timeout_sec == 1.0
            calls.append('executor_shutdown')
            if executor_shutdown_outcome == 'raise':
                raise RuntimeError('shutdown exploded')
            if executor_shutdown_outcome == 'false':
                return False
            return None

    monkeypatch.setattr(scenario_controller, 'ScenarioControllerNode', lambda _document: node)
    monkeypatch.setattr(scenario_controller, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(scenario_controller.rclpy, 'init', lambda **_: calls.append('rclpy_init'))
    monkeypatch.setattr(
        scenario_controller.rclpy,
        'try_shutdown',
        lambda: calls.append('rclpy_shutdown'),
    )
    monkeypatch.setattr(app, '_workflow', lambda: calls.append('workflow'))
    monkeypatch.setattr(app, '_cleanup_actor', lambda: calls.append('cleanup_actor'))

    def result(error: object) -> tuple[dict[str, object], int]:
        assert error is None
        calls.append('result')
        return {}, 0

    monkeypatch.setattr(app, '_result', result)
    monkeypatch.setattr(scenario_controller, 'package_schema_path', lambda _name: Path('schema'))
    monkeypatch.setattr(scenario_controller, 'load_schema', lambda _path: {})
    monkeypatch.setattr(
        scenario_controller,
        'write_canonical_json',
        lambda *_args, **_kwargs: calls.append('write_result'),
    )

    failure = (
        'executor shutdown failed: shutdown exploded'
        if executor_shutdown_outcome == 'raise'
        else 'executor shutdown did not complete'
    )
    with pytest.raises(InfrastructureError, match=failure):
        app.run()
    assert calls == [
        'rclpy_init',
        'executor_constructed',
        'add_node',
        'workflow',
        'begin_cleanup',
        'cleanup_actor',
        'result',
        'write_result',
        'remove_node',
        'executor_shutdown',
        'destroy_node',
        'rclpy_shutdown',
    ]


def test_scenario_controller_rejects_post_terminal_status_regression() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node._on_clock(_clock(1_000_000_000))
        node.mark_ready()
        goal_uuid = uuid.UUID('00000000-0000-0000-0000-000000000001').bytes
        node._on_status(_status(goal_uuid, 1_100_000_000, GoalStatus.STATUS_ACCEPTED))
        node._on_status(_status(goal_uuid, 1_100_000_000, GoalStatus.STATUS_SUCCEEDED))
        with pytest.raises(ProtocolError, match='transition is impossible'):
            node._on_status(_status(goal_uuid, 1_100_000_000, GoalStatus.STATUS_EXECUTING))
        assert node.status.invalid_count == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_controller_rejects_observed_active_goal_before_ready() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node._on_clock(_clock(1_000_000_000))
        goal_uuid = uuid.UUID('00000000-0000-0000-0000-000000000001').bytes
        node._on_status(_status(goal_uuid, 900_000_000, GoalStatus.STATUS_EXECUTING))
        with pytest.raises(InfrastructureError, match='active before readiness'):
            node.mark_ready()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_ready_prerequisites_require_endpoint_not_idle_status_message() -> None:
    ready_client = SimpleNamespace(service_is_ready=lambda: True)
    publisher_counts = {ACTION_STATUS_TOPIC: 1}
    node = SimpleNamespace(
        cancel_client=ready_client,
        clock_seen=True,
        count_publishers=lambda topic: publisher_counts.get(topic, 1),
        current_sim_stamp_ns=1_000_000_000,
        delete_client=ready_client,
        set_pose_client=ready_client,
        spawn_client=ready_client,
        status_message_seen=False,
    )
    app = object.__new__(ScenarioControllerApp)
    app.document = SimpleNamespace(has_actor=False)
    app.node = node

    assert app._ready_prerequisites()
    publisher_counts[ACTION_STATUS_TOPIC] = 0
    assert not app._ready_prerequisites()


def test_contact_node_stops_on_exact_manifest_pair() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        command_qos = node.command_publisher.qos_profile
        assert command_qos.history == HistoryPolicy.KEEP_LAST
        assert command_qos.depth == 1
        assert command_qos.reliability == ReliabilityPolicy.RELIABLE
        assert command_qos.durability == DurabilityPolicy.VOLATILE
        assert node.resolve_topic_name('cmd_vel') == '/robotest/cmd_vel'
        assert (
            node.resolve_topic_name('validation/contacts') == manifest.public_contact_snapshot_topic
        )
        node._on_clock(_clock(1_000_000_000))
        ground_truth = Odometry()
        ground_truth.header.frame_id = 'world'
        ground_truth.header.stamp.sec = 1
        ground_truth.pose.pose.position.y = -3.5
        ground_truth.pose.pose.orientation.w = 1.0
        node._on_ground_truth(ground_truth)
        assert node.latest_ground_truth is not None
        node.arm_motion()
        node.start_forward()
        node._on_clock(_clock(1_050_000_000))
        contact = Contact()
        contact.collision1.name = manifest.chassis_collision
        contact.collision2.name = manifest.expected_control_pair[0]
        if contact.collision2.name == manifest.chassis_collision:
            contact.collision2.name = manifest.expected_control_pair[1]
        message = Contacts()
        message.header.stamp.sec = 1
        message.header.stamp.nanosec = 50_000_000
        message.contacts = [contact]
        node._on_contacts(message)
        assert node.first_qualifying_contact is not None
        assert node.exact_pair_snapshot_record_count == 1
        assert node.stop_latency_ns == 0
        assert node.commands.items[-1]['linear_x'] == 0.0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_discards_pre_positive_clock_contacts_then_seeds() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        support_pair = tuple(next(iter(manifest.support_exclusions)))
        message = Contacts()
        message.header.stamp.sec = 1
        contact = Contact()
        contact.collision1.name, contact.collision2.name = support_pair
        message.contacts = [contact]

        node._on_contacts(message)
        node._on_clock(_clock(0))
        node._on_contacts(message)
        assert node.pre_clock_contact_message_count == 2
        assert node.contact_snapshot_count == 0

        node._on_clock(_clock(1_000_000_000))
        node._on_contacts(message)
        assert node.pre_clock_contact_message_count == 2
        assert node.contact_snapshot_count == 1
        assert node.contact_snapshots.quality()['invalid_count'] == 0
        assert node.protocol_error_count == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_result_skips_final_graph_audit_after_primary_error(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    primary = ProtocolError('primary callback failure')

    def unexpected_graph_audit() -> None:
        raise AssertionError('final graph audit must not replace a primary error')

    monkeypatch.setattr(app, '_enforce_graph_isolation', unexpected_graph_audit)
    monkeypatch.setattr(
        app,
        '_result_without_graph',
        lambda error: ({'reason': str(error)}, int(error.exit_code)),
    )

    result, exit_code = app._result(primary)
    assert result['reason'] == 'primary callback failure'
    assert exit_code == int(primary.exit_code)


def test_contact_graph_missing_source_requires_two_observations_spanning_100ms(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    counts = {
        'validation/contacts': 1,
        'validation/scenario_entity_poses': 0,
        'validation/ground_truth': 1,
    }
    app.node = SimpleNamespace(count_publishers=lambda topic: counts[topic])
    app.last_publisher_count = 0
    app.forbidden_nodes = []
    app.last_source_publisher_counts = {}
    app.source_publisher_missing_since_ns = {
        'contacts': None,
        'entity_pose': None,
        'ground_truth': None,
    }
    app.source_publisher_missing_observation_count = 0
    audit_count = 0
    now_ns = 1_000_000_000

    def audit_contact_graph() -> None:
        nonlocal audit_count
        audit_count += 1

    monkeypatch.setattr(app, '_graph_state', lambda: (1, []))
    monkeypatch.setattr(app, '_audit_contact_graph', audit_contact_graph)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic_ns',
        lambda: now_ns,
    )

    assert app._enforce_graph_isolation() is False
    assert audit_count == 1
    counts['validation/scenario_entity_poses'] = 4
    assert app._enforce_graph_isolation() is True
    assert audit_count == 2
    assert all(value is None for value in app.source_publisher_missing_since_ns.values())

    counts['validation/ground_truth'] = 0
    now_ns += 1_000_000
    assert app._enforce_graph_isolation() is False
    now_ns += 99_999_999
    assert app._enforce_graph_isolation() is False
    now_ns += 1
    with pytest.raises(InfrastructureError, match='ground_truth'):
        app._enforce_graph_isolation()
    assert app.last_source_publisher_counts == {
        'contacts': 1,
        'entity_pose': 4,
        'ground_truth': 0,
    }


def test_contact_graph_contamination_is_immediately_fatal(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    app.node = SimpleNamespace(count_publishers=lambda _topic: 1)
    app.last_publisher_count = 0
    app.forbidden_nodes = []
    app.last_source_publisher_counts = {}
    app.source_publisher_missing_since_ns = {
        'contacts': None,
        'entity_pose': None,
        'ground_truth': None,
    }
    app.source_publisher_missing_observation_count = 0
    monkeypatch.setattr(app, '_graph_state', lambda: (2, []))

    with pytest.raises(InfrastructureError, match='requires one publisher'):
        app._enforce_graph_isolation()

    monkeypatch.setattr(app, '_graph_state', lambda: (1, ['controller_server']))
    with pytest.raises(InfrastructureError, match='Nav2/collision-monitor'):
        app._enforce_graph_isolation()

    def fail_contact_graph() -> None:
        raise InfrastructureError('contact topology changed')

    app.node = SimpleNamespace(
        count_publishers=lambda topic: 0 if topic == 'validation/scenario_entity_poses' else 1
    )
    monkeypatch.setattr(app, '_graph_state', lambda: (1, []))
    monkeypatch.setattr(app, '_audit_contact_graph', fail_contact_graph)
    with pytest.raises(InfrastructureError, match='contact topology changed'):
        app._enforce_graph_isolation()


def test_contact_graph_startup_requires_100ms_stability(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    app.source_graph_stable_since_ns = None
    now_ns = 2_000_000_000
    graph_ready = True
    monkeypatch.setattr(app, '_enforce_graph_isolation', lambda: graph_ready)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.time.monotonic_ns',
        lambda: now_ns,
    )

    assert app._startup_graph_is_stable() is False
    now_ns += 99_999_999
    assert app._startup_graph_is_stable() is False
    now_ns += 1
    assert app._startup_graph_is_stable() is True

    graph_ready = False
    assert app._startup_graph_is_stable() is False
    assert app.source_graph_stable_since_ns is None


def test_contact_graph_endpoint_gid_cannot_change_between_audits(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    app.contact_graph_audit_count = 0
    app.contact_graph_first_snapshot = None
    app.contact_graph_last_snapshot = None
    app.contact_graph_first_sha256 = None
    app.contact_graph_last_sha256 = None

    def snapshot(gid: str) -> dict[str, list[dict[str, str]]]:
        return {
            'private_raw_publishers': [{'endpoint_gid': gid}],
            'private_raw_subscribers': [{'endpoint_gid': gid}],
            'public_snapshot_publishers': [{'endpoint_gid': gid}],
        }

    snapshots = iter((snapshot('aa' * 16), snapshot('bb' * 16)))
    monkeypatch.setattr(app, '_contact_graph_snapshot', lambda: next(snapshots))
    monkeypatch.setattr(app, '_contact_graph_is_exact', lambda _snapshot: True)

    app._audit_contact_graph()
    with pytest.raises(InfrastructureError, match='endpoint identity changed'):
        app._audit_contact_graph()
    assert app.contact_graph_audit_count == 1


def test_contact_node_validates_whole_snapshot_before_stop_and_rejects_nonrobot() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        node.arm_motion()
        node.start_forward()
        exact = Contact()
        exact.collision1.name, exact.collision2.name = manifest.expected_control_pair
        nonrobot = Contact()
        nonrobot.collision1.name = 'box::link::collision'
        nonrobot.collision2.name = 'wall::link::collision'
        message = Contacts()
        message.header.stamp.sec = 1
        message.header.stamp.nanosec = 10_000_000
        message.contacts = [exact, nonrobot]
        with pytest.raises(ProtocolError, match='impossible non-robot pair'):
            node._on_contacts(message)
        assert node.first_qualifying_contact is None
        assert all(command['phase'] != 'CONTACT_STOP' for command in node.commands.items)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_stops_immediately_then_settles_future_snapshot_latency() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        node.arm_motion()
        node.start_forward()
        contact = Contact()
        contact.collision1.name, contact.collision2.name = manifest.expected_control_pair
        message = Contacts()
        message.header.stamp.sec = 1
        message.header.stamp.nanosec = 50_000_000
        message.contacts = [contact]
        node._on_contacts(message)
        assert node.commands.items[-1]['phase'] == 'CONTACT_STOP'
        assert node.stop_latency_ns is None
        assert node.pending_stop_contact_stamp_ns == 1_050_000_000
        node._on_clock(_clock(1_080_000_000))
        assert node.stop_latency_ns == 30_000_000
        assert node.stop_latency_clock_stamp_ns == 1_080_000_000
        assert node.first_qualifying_contact['stop_latency_upper_bound_ns'] == 30_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ('stop_latency_ns', 'misses_deadline'),
    [(100_000_000, False), (100_000_001, True)],
)
def test_contact_node_stop_latency_100ms_boundary(
    stop_latency_ns: int,
    misses_deadline: bool,
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        node.arm_motion()
        node.start_forward()
        node._on_clock(_clock(1_000_000_000 + stop_latency_ns))
        contact = Contact()
        contact.collision1.name, contact.collision2.name = manifest.expected_control_pair
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [contact]

        if misses_deadline:
            with pytest.raises(ScenarioFailureError, match=r'missed its 0\.10 s deadline'):
                node._on_contacts(message)
        else:
            node._on_contacts(message)

        assert node.commands.items[-1]['phase'] == 'CONTACT_STOP'
        assert node.commands.items[-1]['linear_x'] == 0.0
        assert node.stop_latency_ns == stop_latency_ns
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_retains_prepare_skew_but_readiness_waits_for_catch_up(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_220_000_001))
        wheel = next(name for name in manifest.robot_collisions if 'left_wheel' in name)
        support = Contact()
        support.collision1.name = wheel
        support.collision2.name = 'ground_plane::ground_link::ground_collision'
        delayed = Contacts()
        delayed.header.stamp.sec = 1
        delayed.contacts = [support]

        node._on_contacts(delayed)

        assert node.contact_snapshots.invalid_count == 0
        assert node.contact_snapshots.accepted_count == 1
        assert node.contact_snapshots.items[-1]['delivery_clock_offset_ns'] == 220_000_001

        ready_client = SimpleNamespace(service_is_ready=lambda: True)
        node.latest_ground_truth = {'sim_stamp_ns': 1_220_000_001}
        node.spawn_client = ready_client
        node.delete_client = ready_client
        monkeypatch.setattr(node, 'count_publishers', lambda _topic: 1)
        app = object.__new__(ContactControlApp)
        app.node = node
        app.manifest = manifest
        app._graph_state = lambda: (1, [])
        app._contact_graph_snapshot = lambda: {}
        app._contact_graph_is_exact = lambda _snapshot: True

        assert not app._base_prerequisites()

        caught_up = Contacts()
        caught_up.header.stamp.sec = 1
        caught_up.header.stamp.nanosec = 10_000_000
        caught_up.contacts = [support]
        node._on_contacts(caught_up)

        assert app._base_prerequisites()
        assert node.contact_snapshots.accepted_count == 2
        assert node.contact_snapshots.items[-1]['delivery_clock_offset_ns'] == 210_000_001
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ('phase', 'motion_active'),
    [('FORWARD', True), ('RELEASE', False)],
)
def test_contact_node_rejects_snapshot_delivery_after_220ms_once_control_started(
    phase: str,
    motion_active: bool,
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_220_000_001))
        node.arm_motion()
        node.start_forward()
        node.phase = phase
        node.motion_active = motion_active
        wheel = next(name for name in manifest.robot_collisions if 'left_wheel' in name)
        support = Contact()
        support.collision1.name = wheel
        support.collision2.name = 'ground_plane::ground_link::ground_collision'
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [support]
        with pytest.raises(ProtocolError, match='delivery skew exceeded 220 ms'):
            node._on_contacts(message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_prepare_revalidates_base_prerequisites_immediately_before_ready() -> None:
    app = object.__new__(ContactControlApp)
    app.setup_evidence = {
        'observed_robot_start': None,
        'observed_wall': None,
    }
    app.graph_gate_active = False
    events: list[str] = []

    def base_prerequisites() -> bool:
        return True

    def startup_graph_is_stable() -> bool:
        return True

    def wait_for(predicate: object, *, reason: str) -> None:
        del reason
        if predicate is base_prerequisites:
            events.append('wait_base')
        elif predicate is startup_graph_is_stable:
            events.append('wait_graph_stable')
        else:
            raise AssertionError('unexpected preparation predicate')

    app._resolve_wall_asset = lambda: events.append('resolve')
    app._base_prerequisites = base_prerequisites
    app._startup_graph_is_stable = startup_graph_is_stable
    app._wait_for = wait_for
    app._spawn_wall = lambda: events.append('spawn') or {'success': True}
    app._observe_wall = lambda _spawn: events.append('observe_wall') or {'stamp_ns': 1}
    app._verify_robot_start = lambda: events.append('verify_start') or {'sim_stamp_ns': 1}

    def write_ready(_spawn: object, _observed_wall: object) -> None:
        assert app.setup_evidence['observed_wall'] == {'stamp_ns': 1}
        assert app.setup_evidence['observed_robot_start'] == {'sim_stamp_ns': 1}
        events.append('write_ready')

    app._write_ready = write_ready

    app._prepare()

    assert events == [
        'resolve',
        'wait_base',
        'wait_graph_stable',
        'spawn',
        'observe_wall',
        'verify_start',
        'wait_base',
        'write_ready',
    ]


def test_contact_node_rejects_public_frame_and_snapshot_gap() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        wheel = next(name for name in manifest.robot_collisions if 'left_wheel' in name)
        support = Contact()
        support.collision1.name = wheel
        support.collision2.name = 'ground_plane::ground_link::ground_collision'
        framed = Contacts()
        framed.header.stamp.sec = 1
        framed.header.frame_id = 'world'
        framed.contacts = [support]
        with pytest.raises(ProtocolError, match='frame_id'):
            node._on_contacts(framed)

        first = Contacts()
        first.header.stamp.sec = 1
        first.contacts = [support]
        node._on_contacts(first)
        late = Contacts()
        late.header.stamp.sec = 1
        late.header.stamp.nanosec = 220_000_001
        late.contacts = [support]
        with pytest.raises(ProtocolError, match='220 ms'):
            node._on_contacts(late)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('record_count', [0, 17])
def test_contact_node_rejects_public_snapshot_record_cardinality(record_count: int) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        wheel = next(name for name in manifest.robot_collisions if 'left_wheel' in name)
        support = Contact()
        support.collision1.name = wheel
        support.collision2.name = 'ground_plane::ground_link::ground_collision'
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [support for _ in range(record_count)]
        with pytest.raises(ProtocolError, match='between one and 16'):
            node._on_contacts(message)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_retains_maximal_snapshot_record_prefix() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_000_000_000))
        node.contact_snapshot_records.capacity = 1
        wheel = next(name for name in manifest.robot_collisions if 'left_wheel' in name)
        contact = Contact()
        contact.collision1.name = wheel
        contact.collision2.name = 'ground_plane::ground_link::ground_collision'
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [contact, contact]
        with pytest.raises(ProtocolError, match='prefix buffer overflowed'):
            node._on_contacts(message)
        assert node.snapshot_contact_record_count == 2
        assert node.snapshot_contact_accepted_count == 1
        assert node.snapshot_contact_overflow_count == 1
        assert len(node.contact_snapshot_records.items) == 1
        assert node.contact_snapshot_records.items[0]['disposition'] == 'support_ground_excluded'
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_deduplicates_static_wall_pose_state() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node.spawn_request_sequence = 1
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.header.stamp.sec = 1
        transform.child_frame_id = 'phase3_contact_control_wall'
        transform.transform.translation.x = 0.7
        transform.transform.translation.y = -3.5
        transform.transform.translation.z = 0.4
        transform.transform.rotation.w = 1.0
        message = TFMessage(transforms=[transform])
        for _ in range(1_100):
            node._on_entity_poses(message)
        assert node.actor_state.ingress_count == 1_100
        assert node.actor_state.accepted_count == 1_100
        assert len(node.actor_state.items) == 1
        assert len(node.wall_poses) == 1
        assert node.actor_state.overflow_count == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_node_uses_permanent_pose_heartbeat_after_wall_delete() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node.delete_response_sequence = node._next_sequence()
        node.delete_response_stamp_ns = 1_000_000_000
        node.spawn_request_sequence = 1
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.header.stamp.sec = 1
        transform.header.stamp.nanosec = 200_000_000
        transform.child_frame_id = 'unrelated_actor'
        transform.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_entity_message_count == 0
        assert node.post_delete_entity_latest_sim_stamp_ns is None

        transform.header.stamp.nanosec = 300_000_000
        transform.child_frame_id = 'ground_plane'
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_entity_message_count == 1
        assert node.post_delete_entity_latest_sequence is not None
        assert node.post_delete_entity_latest_sim_stamp_ns == 1_300_000_000
        assert node.post_delete_wall_pose_count == 0

        transform.child_frame_id = 'phase3_contact_control_wall'
        transform.header.stamp.sec = 0
        transform.header.stamp.nanosec = 900_000_000
        transform.transform.translation.x = 0.7
        transform.transform.translation.y = -3.5
        transform.transform.translation.z = 0.4
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_wall_pose_count == 1
        assert node.post_delete_wall_pose_first_sequence is not None
        assert node.post_delete_wall_pose_first_sequence > node.post_delete_entity_latest_sequence
        assert node.post_delete_wall_pose_first_sim_stamp_ns == 900_000_000
        assert node.post_delete_wall_pose_latest_sequence == (
            node.post_delete_wall_pose_first_sequence
        )
        assert node.post_delete_wall_pose_latest_sim_stamp_ns == 900_000_000

        transform.header.stamp.nanosec = 200_000_000
        transform.child_frame_id = 'ground_plane'
        with pytest.raises(ProtocolError, match='heartbeat stamp regressed'):
            node._on_entity_poses(TFMessage(transforms=[transform]))

        transform.header.frame_id = 'map'
        transform.header.stamp.sec = 1
        transform.header.stamp.nanosec = 400_000_000
        with pytest.raises(ProtocolError, match='heartbeat has an invalid frame'):
            node._on_entity_poses(TFMessage(transforms=[transform]))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_node_uses_permanent_pose_heartbeat_after_actor_delete() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node.delete_response_sequence = node._next_sequence()
        node.delete_response_stamp_ns = 2_000_000_000
        node.spawn_request_sequence = 1
        node.spawn_request_stamp_ns = 1_900_000_000
        transform = TransformStamped()
        transform.header.frame_id = 'robotest_lab'
        transform.header.stamp.sec = 2
        transform.header.stamp.nanosec = 200_000_000
        transform.child_frame_id = 'unrelated_actor'
        transform.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_entity_message_count == 0
        assert node.post_delete_entity_latest_sim_stamp_ns is None

        transform.header.stamp.nanosec = 300_000_000
        transform.child_frame_id = 'ground_plane'
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_entity_message_count == 1
        assert node.post_delete_entity_latest_sequence is not None
        assert node.post_delete_entity_latest_sim_stamp_ns == 2_300_000_000
        assert node.post_delete_pose_count == 0

        transform.child_frame_id = document.actor_name
        transform.header.stamp.sec = 1
        transform.header.stamp.nanosec = 950_000_000
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert node.post_delete_pose_count == 1
        assert node.post_delete_pose_first_sequence is not None
        assert node.post_delete_pose_first_sequence > node.post_delete_entity_latest_sequence
        assert node.post_delete_pose_first_sim_stamp_ns == 1_950_000_000
        assert node.post_delete_pose_latest_sequence == node.post_delete_pose_first_sequence
        assert node.post_delete_pose_latest_sim_stamp_ns == 1_950_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_cleanup_retains_successful_response_when_quiet_wait_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-proof-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True

    def fail_wait(*_args: object, **_kwargs: object) -> None:
        raise ScenarioFailureError('quiet wait failed')

    try:
        node.current_sim_stamp_ns = 1_000_000_000
        monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
        monkeypatch.setattr(
            app,
            '_call_service',
            lambda *_args, **_kwargs: (SimpleNamespace(success=True), 7, 900_000_000),
        )
        monkeypatch.setattr(app, '_wait_for', fail_wait)
        with pytest.raises(ScenarioFailureError, match='quiet wait failed'):
            app._cleanup_wall()
        assert app.cleanup['delete_attempt_count'] == 1
        assert app.cleanup['delete_success'] is True
        assert app.cleanup['actor_absent'] is False
        assert app.cleanup['proof']['kind'] == (
            'successful_blocking_delete_response_cleanup_anchor_pending'
        )
        assert app.cleanup['proof']['response_stamp_ns'] == 1_000_000_000
        assert app.cleanup['proof']['quiet_start_sim_stamp_ns'] is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_retains_successful_response_when_quiet_wait_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=1.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True

    def fail_wait(*_args: object, **_kwargs: object) -> None:
        raise ScenarioFailureError('quiet wait failed')

    try:
        node.current_sim_stamp_ns = 2_000_000_000
        monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
        monkeypatch.setattr(
            app,
            '_call_service',
            lambda *_args, **_kwargs: (SimpleNamespace(success=True), 9, 1_900_000_000),
        )
        monkeypatch.setattr(app, '_wait_for', fail_wait)
        with pytest.raises(ScenarioFailureError, match='quiet wait failed'):
            app._cleanup_actor()
        assert app.cleanup['delete_attempt_count'] == 1
        assert app.cleanup['delete_success'] is True
        assert app.cleanup['actor_absent'] is False
        assert app.cleanup['proof']['kind'] == (
            'successful_blocking_delete_response_cleanup_anchor_pending'
        )
        assert app.cleanup['proof']['response_stamp_ns'] == 2_000_000_000
        assert app.cleanup['proof']['quiet_start_sim_stamp_ns'] is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_cleanup_rejected_response_retains_response_boundary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-reject-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)

    def reject(*_args: object, **kwargs: object) -> tuple[object, int, int]:
        evidence = kwargs['evidence']
        assert isinstance(evidence, dict)
        evidence['request_sequence'] = 7
        evidence['request_stamp_ns'] = 900_000_000
        node.sequence = 7
        return SimpleNamespace(success=False), 7, 900_000_000

    monkeypatch.setattr(app, '_call_service', reject)
    try:
        with pytest.raises(InfrastructureError, match='rejected contact wall cleanup'):
            app._cleanup_wall()
        proof = app.cleanup['proof']
        assert proof['kind'] == 'delete_response_did_not_prove_absence'
        assert proof['failure_stage'] == 'response_rejected'
        assert proof['error'] == 'scenario/delete_entity rejected contact wall cleanup'
        assert proof['request_sequence'] == 7
        assert proof['request_stamp_ns'] == 900_000_000
        assert proof['response_sequence'] > proof['request_sequence']
        assert proof['response_stamp_ns'] == 1_000_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_rejected_response_retains_response_boundary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)

    def reject(*_args: object, **kwargs: object) -> tuple[object, int, int]:
        evidence = kwargs['evidence']
        assert isinstance(evidence, dict)
        evidence['request_sequence'] = 9
        evidence['request_stamp_ns'] = 1_900_000_000
        node.sequence = 9
        return SimpleNamespace(success=False), 9, 1_900_000_000

    monkeypatch.setattr(app, '_call_service', reject)
    try:
        with pytest.raises(InfrastructureError, match='rejected actor cleanup'):
            app._cleanup_actor()
        proof = app.cleanup['proof']
        assert proof['kind'] == 'delete_response_did_not_prove_absence'
        assert proof['failure_stage'] == 'response_rejected'
        assert proof['error'] == 'scenario/delete_entity rejected actor cleanup'
        assert proof['request_sequence'] == 9
        assert proof['request_stamp_ns'] == 1_900_000_000
        assert proof['response_sequence'] > proof['request_sequence']
        assert proof['response_stamp_ns'] == 2_000_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_cleanup_retains_pending_proof_when_dds_drain_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-drain-failure-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 7, 900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        stamp_ns = 1_200_000_000 if wait_count == 1 else 1_500_000_000
        node.current_sim_stamp_ns = stamp_ns
        node.post_delete_entity_latest_sequence = node._next_sequence()
        node.post_delete_entity_latest_sim_stamp_ns = stamp_ns
        node.post_delete_entity_message_count += 1
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)
    monkeypatch.setattr(time, 'monotonic_ns', lambda: 4_000_000_000)

    def fail_drain(*_args: object, **_kwargs: object) -> None:
        raise ScenarioFailureError('drain failed')

    monkeypatch.setattr(app, '_spin_once', fail_drain)
    try:
        with pytest.raises(ScenarioFailureError, match='drain failed'):
            app._cleanup_wall()
        proof = app.cleanup['proof']
        assert app.cleanup['actor_absent'] is False
        assert proof['kind'] == 'successful_blocking_delete_response_cleanup_drain_pending'
        assert proof['dds_drain_start_steady_ns'] == 4_000_000_000
        assert proof['dds_drain_complete_steady_ns'] is None
        assert proof['dds_drain_spin_count'] == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_retains_pending_proof_when_dds_drain_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    node.spawn_request_sequence = 1
    node.spawn_request_stamp_ns = 1_800_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 9, 1_900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        stamp_ns = 2_200_000_000 if wait_count == 1 else 2_500_000_000
        node.current_sim_stamp_ns = stamp_ns
        node.post_delete_entity_latest_sequence = node._next_sequence()
        node.post_delete_entity_latest_sim_stamp_ns = stamp_ns
        node.post_delete_entity_message_count += 1
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)
    monkeypatch.setattr(time, 'monotonic_ns', lambda: 5_000_000_000)

    def fail_drain(*_args: object, **_kwargs: object) -> None:
        raise ScenarioFailureError('drain failed')

    monkeypatch.setattr(app, '_spin_once', fail_drain)
    try:
        with pytest.raises(ScenarioFailureError, match='drain failed'):
            app._cleanup_actor()
        proof = app.cleanup['proof']
        assert app.cleanup['actor_absent'] is False
        assert proof['kind'] == 'successful_blocking_delete_response_cleanup_drain_pending'
        assert proof['dds_drain_start_steady_ns'] == 5_000_000_000
        assert proof['dds_drain_complete_steady_ns'] is None
        assert proof['dds_drain_spin_count'] == 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ('deliver_terminal_during_drain', 'source_disappears', 'final_query_expires'),
    [(False, False, False), (True, False, False), (False, True, False), (False, False, True)],
)
def test_contact_cleanup_drains_callbacks_after_source_spanned_quiet_interval(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    deliver_terminal_during_drain: bool,
    source_disappears: bool,
    final_query_expires: bool,
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-terminal-pose-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    node.sequence = 7
    node.spawn_request_sequence = 1
    wall_clock = SimpleNamespace(now=9.0)
    app.wall_deadline = 10.0
    monkeypatch.setattr(time, 'monotonic', lambda: wall_clock.now)
    publisher_query_count = 0

    def count_publishers(_topic: str) -> int:
        nonlocal publisher_query_count
        publisher_query_count += 1
        if final_query_expires and publisher_query_count > 1:
            wall_clock.now = app.wall_deadline
        return 0 if source_disappears and publisher_query_count > 1 else 4

    monkeypatch.setattr(node, 'count_publishers', count_publishers)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 7, 900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.transform.rotation.w = 1.0
        stamps = (
            [1_200_000_000, 1_500_000_000, 1_600_000_000, 1_900_000_000]
            if deliver_terminal_during_drain
            else [1_200_000_000, 1_500_000_000]
        )
        stamp_ns = stamps[wait_count - 1]
        transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        transform.child_frame_id = 'ground_plane'
        node.current_sim_stamp_ns = stamp_ns
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)

    def drain_callback(spin_count: int) -> None:
        if spin_count != 1 or not deliver_terminal_during_drain:
            return
        wall = TransformStamped()
        wall.header.frame_id = 'world'
        wall.header.stamp.sec = 1
        wall.header.stamp.nanosec = 200_000_000
        wall.child_frame_id = 'phase3_contact_control_wall'
        wall.transform.translation.x = 0.7
        wall.transform.translation.y = -3.5
        wall.transform.translation.z = 0.4
        wall.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[wall]))

    drain_clock = _install_cleanup_drain_clock(
        app,
        monkeypatch,
        on_spin=drain_callback,
    )
    try:
        if final_query_expires:
            with pytest.raises(WallTimeoutError, match='deadline elapsed'):
                app._cleanup_wall()
            assert app.cleanup['actor_absent'] is False
            assert (
                app.cleanup['proof']['kind']
                == 'successful_blocking_delete_response_cleanup_drain_pending'
            )
            return
        if source_disappears:
            with pytest.raises(ScenarioFailureError, match='source publisher was missing'):
                app._cleanup_wall()
            assert app.cleanup['actor_absent'] is False
            assert (
                app.cleanup['proof']['kind']
                == 'successful_blocking_delete_response_cleanup_drain_pending'
            )
            assert 'pose_source_publishers_after' not in app.cleanup['proof']
            return
        app._cleanup_wall()
        assert wait_count == (4 if deliver_terminal_during_drain else 2)
        assert drain_clock.spin_count == (4 if deliver_terminal_during_drain else 3)
        assert app.cleanup['actor_absent'] is True
        proof = app.cleanup['proof']
        assert proof['kind'] == 'successful_blocking_delete_and_bounded_pose_absence'
        assert proof['post_delete_pose_count'] == int(deliver_terminal_during_drain)
        expected_stamp = 1_200_000_000 if deliver_terminal_during_drain else None
        assert proof['post_delete_pose_first_sim_stamp_ns'] == expected_stamp
        assert proof['post_delete_pose_latest_sim_stamp_ns'] == expected_stamp
        if deliver_terminal_during_drain:
            assert (
                proof['post_delete_pose_source_latest_sequence']
                > proof['post_delete_pose_latest_sequence']
            )
        expected_quiet_start = 1_600_000_000 if deliver_terminal_during_drain else 1_200_000_000
        assert proof['quiet_start_sim_stamp_ns'] == expected_quiet_start
        assert proof['quiet_until_sim_stamp_ns'] == expected_quiet_start + 250_000_000
        assert proof['quiet_restart_count'] == int(deliver_terminal_during_drain)
        assert proof['dds_drain_grace_ns'] == 50_000_000
        assert (
            proof['dds_drain_complete_steady_ns'] - proof['dds_drain_start_steady_ns']
            >= (proof['dds_drain_grace_ns'])
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_cleanup_restarts_quiet_window_after_newer_pose(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-persistent-pose-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    node.sequence = 7
    node.spawn_request_sequence = 1
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 7, 900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'world'
        heartbeat.header.stamp.sec = 1
        stamp_ns = {
            1: 1_200_000_000,
            2: 1_500_000_000,
            3: 1_600_000_000,
            4: 1_900_000_000,
        }[wait_count]
        heartbeat.header.stamp.sec, heartbeat.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        node.current_sim_stamp_ns = stamp_ns
        node._on_entity_poses(TFMessage(transforms=[heartbeat]))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)

    def drain_callback(spin_count: int) -> None:
        if spin_count != 1:
            return
        wall = TransformStamped()
        wall.header.frame_id = 'world'
        wall.header.stamp.sec = 1
        wall.header.stamp.nanosec = 300_000_000
        wall.child_frame_id = 'phase3_contact_control_wall'
        wall.transform.translation.x = 0.7
        wall.transform.translation.y = -3.5
        wall.transform.translation.z = 0.4
        wall.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[wall]))

    _install_cleanup_drain_clock(app, monkeypatch, on_spin=drain_callback)
    try:
        app._cleanup_wall()
        assert wait_count == 4
        assert app.cleanup['actor_absent'] is True
        assert app.cleanup['proof']['quiet_restart_count'] == 1
        assert app.cleanup['proof']['quiet_start_sim_stamp_ns'] == 1_600_000_000
        assert app.cleanup['proof']['quiet_until_sim_stamp_ns'] == 1_850_000_000
        assert app.cleanup['proof']['post_delete_pose_count'] == 1
        assert app.cleanup['proof']['post_delete_pose_latest_sim_stamp_ns'] == 1_300_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ('deliver_terminal_during_drain', 'source_disappears', 'final_query_expires'),
    [(False, False, False), (True, False, False), (False, True, False), (False, False, True)],
)
def test_scenario_cleanup_drains_callbacks_after_source_spanned_quiet_interval(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    deliver_terminal_during_drain: bool,
    source_disappears: bool,
    final_query_expires: bool,
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    node.sequence = 9
    node.spawn_request_sequence = 1
    node.spawn_request_stamp_ns = 1_800_000_000
    wall_clock = SimpleNamespace(now=9.0)
    app.wall_deadline = 10.0
    monkeypatch.setattr(time, 'monotonic', lambda: wall_clock.now)
    publisher_query_count = 0

    def count_publishers(_topic: str) -> int:
        nonlocal publisher_query_count
        publisher_query_count += 1
        if final_query_expires and publisher_query_count > 1:
            wall_clock.now = app.wall_deadline
        return 0 if source_disappears and publisher_query_count > 1 else 4

    monkeypatch.setattr(node, 'count_publishers', count_publishers)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 9, 1_900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        transform = TransformStamped()
        transform.header.frame_id = 'robotest_lab'
        transform.transform.rotation.w = 1.0
        stamps = (
            [2_200_000_000, 2_500_000_000, 2_600_000_000, 2_900_000_000]
            if deliver_terminal_during_drain
            else [2_200_000_000, 2_500_000_000]
        )
        stamp_ns = stamps[wait_count - 1]
        transform.header.stamp.sec, transform.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        transform.child_frame_id = 'ground_plane'
        node.current_sim_stamp_ns = stamp_ns
        node._on_entity_poses(TFMessage(transforms=[transform]))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)

    def drain_callback(spin_count: int) -> None:
        if spin_count != 1 or not deliver_terminal_during_drain:
            return
        actor = TransformStamped()
        actor.header.frame_id = 'robotest_lab'
        actor.header.stamp.sec = 2
        actor.header.stamp.nanosec = 200_000_000
        actor.child_frame_id = document.actor_name
        actor.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[actor]))

    drain_clock = _install_cleanup_drain_clock(
        app,
        monkeypatch,
        on_spin=drain_callback,
    )
    try:
        if final_query_expires:
            with pytest.raises(WallTimeoutError, match='escape timeout elapsed'):
                app._cleanup_actor()
            assert app.cleanup['actor_absent'] is False
            assert (
                app.cleanup['proof']['kind']
                == 'successful_blocking_delete_response_cleanup_drain_pending'
            )
            return
        if source_disappears:
            with pytest.raises(ScenarioFailureError, match='source publisher was missing'):
                app._cleanup_actor()
            assert app.cleanup['actor_absent'] is False
            assert (
                app.cleanup['proof']['kind']
                == 'successful_blocking_delete_response_cleanup_drain_pending'
            )
            assert 'pose_source_publishers_after' not in app.cleanup['proof']
            return
        app._cleanup_actor()
        assert wait_count == (4 if deliver_terminal_during_drain else 2)
        assert drain_clock.spin_count == (4 if deliver_terminal_during_drain else 3)
        assert app.cleanup['actor_absent'] is True
        proof = app.cleanup['proof']
        assert proof['kind'] == 'successful_blocking_delete_and_bounded_pose_absence'
        assert proof['post_delete_pose_count'] == int(deliver_terminal_during_drain)
        expected_stamp = 2_200_000_000 if deliver_terminal_during_drain else None
        assert proof['post_delete_pose_first_sim_stamp_ns'] == expected_stamp
        assert proof['post_delete_pose_latest_sim_stamp_ns'] == expected_stamp
        if deliver_terminal_during_drain:
            assert (
                proof['post_delete_pose_source_latest_sequence']
                > proof['post_delete_pose_latest_sequence']
            )
        expected_quiet_start = 2_600_000_000 if deliver_terminal_during_drain else 2_200_000_000
        assert proof['quiet_start_sim_stamp_ns'] == expected_quiet_start
        assert proof['quiet_until_sim_stamp_ns'] == expected_quiet_start + 250_000_000
        assert proof['quiet_restart_count'] == int(deliver_terminal_during_drain)
        assert proof['dds_drain_grace_ns'] == 50_000_000
        assert (
            proof['dds_drain_complete_steady_ns'] - proof['dds_drain_start_steady_ns']
            >= (proof['dds_drain_grace_ns'])
        )
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_restarts_quiet_window_after_newer_pose(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    node.sequence = 9
    node.spawn_request_sequence = 1
    node.spawn_request_stamp_ns = 1_800_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 9, 1_900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'robotest_lab'
        heartbeat.header.stamp.sec = 2
        stamp_ns = {
            1: 2_200_000_000,
            2: 2_500_000_000,
            3: 2_600_000_000,
            4: 2_900_000_000,
        }[wait_count]
        heartbeat.header.stamp.sec, heartbeat.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        node.current_sim_stamp_ns = stamp_ns
        node._on_entity_poses(TFMessage(transforms=[heartbeat]))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)

    def drain_callback(spin_count: int) -> None:
        if spin_count != 1:
            return
        actor = TransformStamped()
        actor.header.frame_id = 'robotest_lab'
        actor.header.stamp.sec = 2
        actor.header.stamp.nanosec = 300_000_000
        actor.child_frame_id = document.actor_name
        actor.transform.rotation.w = 1.0
        node._on_entity_poses(TFMessage(transforms=[actor]))

    _install_cleanup_drain_clock(app, monkeypatch, on_spin=drain_callback)
    try:
        app._cleanup_actor()
        assert wait_count == 4
        assert app.cleanup['actor_absent'] is True
        assert app.cleanup['proof']['quiet_restart_count'] == 1
        assert app.cleanup['proof']['quiet_start_sim_stamp_ns'] == 2_600_000_000
        assert app.cleanup['proof']['quiet_until_sim_stamp_ns'] == 2_850_000_000
        assert app.cleanup['proof']['post_delete_pose_count'] == 1
        assert app.cleanup['proof']['post_delete_pose_latest_sim_stamp_ns'] == 2_300_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_cleanup_persistent_pose_activity_exhausts_bounded_observation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-persistent-deadline-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    node.sequence = 7
    node.spawn_request_sequence = 1
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 7, 900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, *, reason: str, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        if wait_count == 3:
            node.current_sim_stamp_ns = 2_000_000_000
            raise ScenarioFailureError(reason)
        transforms: list[TransformStamped] = []
        heartbeat_stamp_ns = 1_200_000_000 if wait_count == 1 else 1_500_000_000
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'world'
        heartbeat.header.stamp.sec, heartbeat.header.stamp.nanosec = divmod(
            heartbeat_stamp_ns, 1_000_000_000
        )
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        transforms.append(heartbeat)
        if wait_count == 2:
            wall = TransformStamped()
            wall.header.frame_id = 'world'
            wall.header.stamp.sec = 1
            wall.header.stamp.nanosec = 300_000_000
            wall.child_frame_id = 'phase3_contact_control_wall'
            wall.transform.rotation.w = 1.0
            transforms.append(wall)
        node.current_sim_stamp_ns = heartbeat_stamp_ns
        node._on_entity_poses(TFMessage(transforms=transforms))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)
    try:
        with pytest.raises(ScenarioFailureError, match='heartbeat anchor'):
            app._cleanup_wall()
        proof = app.cleanup['proof']
        assert app.cleanup['actor_absent'] is False
        assert proof['kind'].endswith('cleanup_anchor_pending')
        assert proof['observation_deadline_sim_stamp_ns'] == 2_000_000_000
        assert proof['quiet_restart_count'] == 1
        assert proof['post_delete_pose_count'] == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_persistent_pose_activity_exhausts_bounded_observation(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    node.sequence = 9
    node.spawn_request_sequence = 1
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_call_service',
        lambda *_args, **_kwargs: (SimpleNamespace(success=True), 9, 1_900_000_000),
    )
    wait_count = 0

    def wait_for(predicate: object, *, reason: str, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        if wait_count == 3:
            node.current_sim_stamp_ns = 3_000_000_000
            raise ScenarioFailureError(reason)
        transforms: list[TransformStamped] = []
        heartbeat_stamp_ns = 2_200_000_000 if wait_count == 1 else 2_500_000_000
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'robotest_lab'
        heartbeat.header.stamp.sec, heartbeat.header.stamp.nanosec = divmod(
            heartbeat_stamp_ns, 1_000_000_000
        )
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        transforms.append(heartbeat)
        if wait_count == 2:
            actor = TransformStamped()
            actor.header.frame_id = 'robotest_lab'
            actor.header.stamp.sec = 2
            actor.header.stamp.nanosec = 300_000_000
            actor.child_frame_id = document.actor_name
            actor.transform.rotation.w = 1.0
            transforms.append(actor)
        node.current_sim_stamp_ns = heartbeat_stamp_ns
        node._on_entity_poses(TFMessage(transforms=transforms))
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)
    try:
        with pytest.raises(ScenarioFailureError, match='heartbeat anchor'):
            app._cleanup_actor()
        proof = app.cleanup['proof']
        assert app.cleanup['actor_absent'] is False
        assert proof['kind'].endswith('cleanup_anchor_pending')
        assert proof['observation_deadline_sim_stamp_ns'] == 3_000_000_000
        assert proof['quiet_restart_count'] == 1
        assert proof['post_delete_pose_count'] == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
@pytest.mark.parametrize(
    ('boundary', 'expected_stage', 'expected_send_count'),
    [
        ('readiness_service_deadline', 'service_wait', 0),
        ('readiness_wall_deadline', 'service_wait', 0),
        ('pre_send_service_deadline', 'service_wait', 0),
        ('response_service_deadline', 'response_timeout', 1),
    ],
)
def test_service_transactions_never_cross_hard_deadlines(
    app_type: type[object],
    boundary: str,
    expected_stage: str,
    expected_send_count: int,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(app_type)
    clock = SimpleNamespace(now=10.0)
    send_count = 0

    def next_sequence() -> int:
        if boundary == 'pre_send_service_deadline':
            clock.now = 10.2
        return 41

    app.node = SimpleNamespace(
        _next_sequence=next_sequence,
        cleanup_fatal_error=None,
        current_sim_stamp_ns=2_000_000_000,
    )
    app.service_timeout_s = 0.1 if boundary != 'readiness_wall_deadline' else 10.0
    app.wall_deadline = 10.1 if boundary == 'readiness_wall_deadline' else 100.0
    if app_type is ContactControlApp:
        app.cleanup_window_active = True
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    def service_is_ready() -> bool:
        if boundary.startswith('readiness_'):
            clock.now = 10.2
        return True

    def response_done() -> bool:
        if boundary == 'response_service_deadline':
            clock.now = 10.2
        return True

    def call_async(_request: object) -> object:
        nonlocal send_count
        send_count += 1
        return SimpleNamespace(done=response_done, result=lambda: SimpleNamespace(success=True))

    client = SimpleNamespace(service_is_ready=service_is_ready, call_async=call_async)
    if boundary in {'pre_send_service_deadline', 'response_service_deadline'}:
        monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: 10.1)
    evidence = {
        'error': None,
        'failure_stage': None,
        'kind': 'delete_request_pending',
        'request_sequence': None,
        'request_stamp_ns': None,
        'response_sequence': None,
        'response_stamp_ns': None,
    }

    with pytest.raises((InfrastructureError, WallTimeoutError)):
        app._call_service(
            client,
            object(),
            name='scenario/delete_entity',
            check_fatal=False,
            evidence=evidence,
        )

    assert send_count == expected_send_count
    assert evidence['kind'] == 'delete_transaction_failed'
    assert evidence['failure_stage'] == expected_stage
    assert isinstance(evidence['error'], str) and evidence['error']


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
@pytest.mark.parametrize(
    ('failure_mode', 'expected_stage'),
    [
        ('service_wait', 'service_wait'),
        ('service_wait_exception', 'service_wait'),
        ('request_send', 'request_send'),
        ('response_wait_spin', 'response_wait'),
        ('response_wait_ready_check', 'response_wait'),
        ('response_timeout', 'response_timeout'),
        ('response_future', 'response_future'),
        ('response_null', 'response_null'),
    ],
)
def test_delete_service_failures_retain_transaction_evidence(
    app_type: type[object],
    failure_mode: str,
    expected_stage: str,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(app_type)
    sequence = iter([41])
    app.node = SimpleNamespace(
        _next_sequence=lambda: next(sequence),
        current_sim_stamp_ns=2_000_000_000,
    )
    app.service_timeout_s = 0.1
    app.wall_deadline = 100.0
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(app, '_spin_once', lambda **_kwargs: setattr(clock, 'now', 10.2))
    if failure_mode == 'service_wait':
        monkeypatch.setattr(
            app,
            '_wait_service',
            lambda *_args, **_kwargs: (_ for _ in ()).throw(
                InfrastructureError('delete service unavailable')
            ),
        )
    elif failure_mode == 'service_wait_exception':
        monkeypatch.setattr(
            app,
            '_wait_service',
            lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError('readiness failed')),
        )
    else:
        monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: app.wall_deadline)

    if failure_mode == 'request_send':
        client = SimpleNamespace(
            call_async=lambda _request: (_ for _ in ()).throw(RuntimeError('send failed'))
        )
    elif failure_mode == 'response_wait_spin':
        monkeypatch.setattr(
            app,
            '_spin_once',
            lambda **_kwargs: (_ for _ in ()).throw(RuntimeError('response wait spin failed')),
        )
        client = SimpleNamespace(call_async=lambda _request: SimpleNamespace(done=lambda: False))
    elif failure_mode == 'response_wait_ready_check':
        client = SimpleNamespace(
            call_async=lambda _request: SimpleNamespace(
                done=lambda: (_ for _ in ()).throw(RuntimeError('ready check failed'))
            )
        )
    elif failure_mode == 'response_timeout':
        client = SimpleNamespace(call_async=lambda _request: SimpleNamespace(done=lambda: False))
    elif failure_mode == 'response_future':
        client = SimpleNamespace(
            call_async=lambda _request: SimpleNamespace(
                done=lambda: True,
                result=lambda: (_ for _ in ()).throw(RuntimeError('future failed')),
            )
        )
    else:
        client = SimpleNamespace(
            call_async=lambda _request: SimpleNamespace(done=lambda: True, result=lambda: None)
        )
    evidence = {
        'error': None,
        'failure_stage': None,
        'kind': 'delete_request_pending',
        'request_sequence': None,
        'request_stamp_ns': None,
        'response_sequence': None,
        'response_stamp_ns': None,
    }

    with pytest.raises(InfrastructureError):
        app._call_service(
            client,
            object(),
            name='scenario/delete_entity',
            check_fatal=False,
            evidence=evidence,
        )

    assert evidence['kind'] == 'delete_transaction_failed'
    assert evidence['failure_stage'] == expected_stage
    assert isinstance(evidence['error'], str) and evidence['error']
    assert evidence['response_sequence'] is None
    assert evidence['response_stamp_ns'] is None
    if failure_mode in {'service_wait', 'service_wait_exception'}:
        assert evidence['request_sequence'] is None
    else:
        assert evidence['request_sequence'] == 41
        assert evidence['request_stamp_ns'] == 2_000_000_000


def test_contact_cleanup_preserves_service_wait_failure_transaction(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='cleanup-service-wait-test',
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.wall_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_wait_service',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            InfrastructureError('delete service unavailable')
        ),
    )
    try:
        with pytest.raises(InfrastructureError, match='delete service unavailable'):
            app._cleanup_wall()
        assert app.cleanup['actor_absent'] is False
        assert app.cleanup['delete_success'] is None
        assert app.cleanup['proof']['kind'] == 'delete_transaction_failed'
        assert app.cleanup['proof']['failure_stage'] == 'service_wait'
        assert app.cleanup['proof']['request_sequence'] is None
        assert app.cleanup['proof']['response_sequence'] is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_preserves_service_wait_failure_transaction(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 1_000_000_000
    monkeypatch.setattr(node, 'count_publishers', lambda _topic: 4)
    monkeypatch.setattr(
        app,
        '_wait_service',
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            InfrastructureError('delete service unavailable')
        ),
    )
    try:
        with pytest.raises(InfrastructureError, match='delete service unavailable'):
            app._cleanup_actor()
        assert app.cleanup['actor_absent'] is False
        assert app.cleanup['delete_success'] is None
        assert app.cleanup['proof']['kind'] == 'delete_transaction_failed'
        assert app.cleanup['proof']['failure_stage'] == 'service_wait'
        assert app.cleanup['proof']['request_sequence'] is None
        assert app.cleanup['proof']['response_sequence'] is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_scenario_cleanup_retains_valid_pending_proof_when_source_query_fails(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    app = ScenarioControllerApp(
        document=document,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        identity={},
        wall_timeout_s=30.0,
        service_timeout_s=1.0,
        raw_ros_args=[],
    )
    app.node = node
    app.actor_spawn_request_sent = True
    node.current_sim_stamp_ns = 2_000_000_000
    node.sequence = 9
    node.spawn_request_sequence = 1
    node.spawn_request_stamp_ns = 1_800_000_000
    publisher_query_count = 0

    def count_publishers(_topic: str) -> int:
        nonlocal publisher_query_count
        publisher_query_count += 1
        if publisher_query_count == 1:
            raise RuntimeError('graph query failed')
        return 4

    monkeypatch.setattr(node, 'count_publishers', count_publishers)
    delete_call_count = 0

    def delete(*_args: object, **_kwargs: object) -> tuple[object, int, int]:
        nonlocal delete_call_count
        delete_call_count += 1
        return SimpleNamespace(success=True), 9, 1_900_000_000

    monkeypatch.setattr(app, '_call_service', delete)
    wait_count = 0

    def wait_for(predicate: object, **_kwargs: object) -> None:
        nonlocal wait_count
        wait_count += 1
        stamp_ns = 2_200_000_000 if wait_count == 1 else 2_500_000_000
        node.current_sim_stamp_ns = stamp_ns
        node.post_delete_entity_latest_sequence = node._next_sequence()
        node.post_delete_entity_latest_sim_stamp_ns = stamp_ns
        node.post_delete_entity_message_count += 1
        assert callable(predicate) and predicate()

    monkeypatch.setattr(app, '_wait_for', wait_for)
    _install_cleanup_drain_clock(app, monkeypatch)
    try:
        with pytest.raises(InfrastructureError, match='pre-delete query failed'):
            app._cleanup_actor()
        assert delete_call_count == 1
        assert app.cleanup['actor_absent'] is False
        assert app.cleanup['delete_attempt_count'] == 1
        assert app.cleanup['delete_success'] is True
        assert (
            app.cleanup['proof']['kind']
            == 'successful_blocking_delete_response_cleanup_drain_pending'
        )
        assert app.cleanup['proof']['pose_source_publishers_before'] == 0
        assert 'pose_source_publishers_after' not in app.cleanup['proof']
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_cleanup_callback_protocol_errors_remain_fatal_during_contact_cleanup() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config/collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node.fatal_error = ScenarioFailureError('primary control failure')
        node.begin_cleanup()
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'invalid'
        heartbeat.header.stamp.sec = 1
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        node.entity_pose_subscription.callback(TFMessage(transforms=[heartbeat]))
        assert isinstance(node.cleanup_fatal_error, ProtocolError)

        app = object.__new__(ContactControlApp)
        app.node = node
        app.executor = SimpleNamespace(spin_once=lambda **_kwargs: None)
        app.wall_deadline = time.monotonic() + 10.0
        app.cleanup_window_active = True
        app.graph_gate_active = False
        with pytest.raises(ProtocolError, match='invalid frame'):
            app._spin_once(check_fatal=False)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_cleanup_callback_protocol_errors_remain_fatal_during_scenario_cleanup() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios/phase3_s2_static_obstacle.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node.fatal_error = ScenarioFailureError('primary scenario failure')
        node.begin_cleanup()
        heartbeat = TransformStamped()
        heartbeat.header.frame_id = 'invalid'
        heartbeat.header.stamp.sec = 1
        heartbeat.child_frame_id = 'ground_plane'
        heartbeat.transform.rotation.w = 1.0
        node.entity_pose_subscription.callback(TFMessage(transforms=[heartbeat]))
        assert isinstance(node.cleanup_fatal_error, ProtocolError)

        app = object.__new__(ScenarioControllerApp)
        app.node = node
        app.executor = SimpleNamespace(spin_once=lambda **_kwargs: None)
        app.wall_deadline = time.monotonic() + 10.0
        with pytest.raises(ProtocolError, match='invalid frame'):
            app._spin_once(check_fatal=False)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
def test_cleanup_wait_rejects_prelatched_error_before_true_predicate(
    app_type: type[object],
) -> None:
    app = object.__new__(app_type)
    app.node = SimpleNamespace(
        cleanup_fatal_error=ProtocolError('prelatched cleanup failure'),
        clock_seen=True,
        current_sim_stamp_ns=1,
    )
    app.wall_deadline = time.monotonic() + 10.0
    if app_type is ContactControlApp:
        app.cleanup_window_active = True

    with pytest.raises(ProtocolError, match='prelatched cleanup failure'):
        app._wait_for(lambda: True, reason='must not return', check_fatal=False)


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
def test_cleanup_wait_fails_at_exact_sim_deadline_without_extra_spin(
    app_type: type[object],
) -> None:
    app = object.__new__(app_type)
    app.node = SimpleNamespace(
        cleanup_fatal_error=None,
        clock_seen=True,
        current_sim_stamp_ns=1_000_000_000,
    )
    app.executor = SimpleNamespace(
        spin_once=lambda **_kwargs: pytest.fail('deadline must be checked before another spin')
    )
    app.wall_deadline = time.monotonic() + 10.0
    if app_type is ContactControlApp:
        app.cleanup_window_active = True

    with pytest.raises(ScenarioFailureError, match='exact deadline'):
        app._wait_for(
            lambda: False,
            reason='exact deadline',
            sim_deadline_ns=1_000_000_000,
            check_fatal=False,
        )


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
def test_cleanup_wait_accepts_satisfied_predicate_at_exact_sim_deadline(
    app_type: type[object],
) -> None:
    app = object.__new__(app_type)
    app.node = SimpleNamespace(
        cleanup_fatal_error=None,
        clock_seen=True,
        current_sim_stamp_ns=1_000_000_000,
    )
    app.executor = SimpleNamespace(
        spin_once=lambda **_kwargs: pytest.fail('satisfied boundary must not spin')
    )
    app.wall_deadline = time.monotonic() + 10.0
    if app_type is ContactControlApp:
        app.cleanup_window_active = True

    app._wait_for(
        lambda: True,
        reason='exact satisfied deadline',
        sim_deadline_ns=1_000_000_000,
        check_fatal=False,
    )


@pytest.mark.parametrize('app_type', [ContactControlApp, ScenarioControllerApp])
def test_cleanup_wait_rejects_satisfied_predicate_past_sim_deadline(
    app_type: type[object],
) -> None:
    app = object.__new__(app_type)
    app.node = SimpleNamespace(
        cleanup_fatal_error=None,
        clock_seen=True,
        current_sim_stamp_ns=1_000_000_001,
    )
    app.executor = SimpleNamespace(
        spin_once=lambda **_kwargs: pytest.fail('expired deadline must not spin')
    )
    app.wall_deadline = time.monotonic() + 10.0
    if app_type is ContactControlApp:
        app.cleanup_window_active = True

    with pytest.raises(ScenarioFailureError, match='past deadline'):
        app._wait_for(
            lambda: True,
            reason='past deadline',
            sim_deadline_ns=1_000_000_000,
            check_fatal=False,
        )


def test_scenario_spin_rechecks_wall_deadline_after_executor_callback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = SimpleNamespace(now=9.0)
    app = object.__new__(ScenarioControllerApp)
    app.node = SimpleNamespace(cleanup_fatal_error=None, fatal_error=None)

    def cross_deadline(*, timeout_sec: float) -> None:
        assert timeout_sec == pytest.approx(0.02)
        clock.now = 10.0

    app.executor = SimpleNamespace(spin_once=cross_deadline)
    app.wall_deadline = 10.0
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    with pytest.raises(WallTimeoutError, match='steady-wall escape timeout elapsed'):
        app._spin_once(check_fatal=False)


def test_scenario_wait_predicate_cannot_cross_wall_deadline(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = SimpleNamespace(now=9.0)
    app = object.__new__(ScenarioControllerApp)
    app.node = SimpleNamespace(
        cleanup_fatal_error=None,
        clock_seen=True,
        current_sim_stamp_ns=1,
    )
    app.executor = SimpleNamespace(
        spin_once=lambda **_kwargs: pytest.fail('expired predicate must not spin')
    )
    app.wall_deadline = 10.0
    monkeypatch.setattr(time, 'monotonic', lambda: clock.now)

    def cross_deadline() -> bool:
        clock.now = 10.0
        return True

    with pytest.raises(WallTimeoutError, match='steady-wall escape timeout elapsed'):
        app._wait_for(cross_deadline, reason='must not pass', check_fatal=False)


def test_contact_final_graph_observation_precedes_cleanup_and_result_is_spin_free() -> None:
    run_source = inspect.getsource(ContactControlApp.run)
    result_source = inspect.getsource(ContactControlApp._result)

    assert run_source.index('self._finalize_graph_observation()') < run_source.index(
        'self.node.begin_cleanup()'
    )
    assert run_source.index('self.node.begin_cleanup()') < run_source.index('self._cleanup_wall()')
    assert '_wait_for(' not in result_source


def test_contact_run_calls_final_graph_before_cleanup(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    app = _spawn_test_app(tmp_path, run_id='final-graph-order-test')
    events: list[str] = []

    class FakeNode:
        def begin_cleanup(self) -> None:
            events.append('begin_cleanup')

        @staticmethod
        def destroy_node() -> None:
            return None

    class FakeExecutor:
        @staticmethod
        def add_node(_node: object) -> None:
            return None

        @staticmethod
        def remove_node(_node: object) -> None:
            return None

        @staticmethod
        def shutdown(*, timeout_sec: float) -> None:
            assert timeout_sec == 1.0

    node = FakeNode()
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.ContactControlNode',
        lambda _manifest: node,
    )
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.SingleThreadedExecutor',
        FakeExecutor,
    )
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.rclpy.init', lambda **_: None)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.rclpy.try_shutdown', lambda: None
    )
    monkeypatch.setattr(app, '_prepare', lambda: None)
    monkeypatch.setattr(app, '_wait_for_arm', lambda: None)
    monkeypatch.setattr(app, '_run_control', lambda: None)
    monkeypatch.setattr(app, '_begin_cleanup_window', lambda: events.append('cleanup_window'))
    monkeypatch.setattr(app, '_best_effort_zero', lambda: events.append('zero'))
    monkeypatch.setattr(
        app,
        '_finalize_graph_observation',
        lambda: events.append('final_graph'),
    )
    monkeypatch.setattr(app, '_cleanup_wall', lambda: events.append('delete_cleanup'))
    monkeypatch.setattr(app, '_result', lambda _error: ({}, 0))
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.load_schema', lambda _path: {})
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.write_canonical_json',
        lambda *_args, **_kwargs: events.append('write_result'),
    )

    assert app.run() == 0
    assert events == [
        'cleanup_window',
        'zero',
        'final_graph',
        'begin_cleanup',
        'delete_cleanup',
        'write_result',
    ]


def test_contact_run_normalizes_final_graph_exception_and_still_cleans_up(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _spawn_test_app(tmp_path, run_id='final-graph-failure-test')
    events: list[str] = []

    class FakeNode:
        def begin_cleanup(self) -> None:
            events.append('begin_cleanup')

        @staticmethod
        def destroy_node() -> None:
            events.append('destroy_node')

    class FakeExecutor:
        @staticmethod
        def add_node(_node: object) -> None:
            return None

        @staticmethod
        def remove_node(_node: object) -> None:
            events.append('remove_node')

        @staticmethod
        def shutdown(*, timeout_sec: float) -> None:
            assert timeout_sec == 1.0
            events.append('shutdown')

    def result(error: object) -> tuple[dict[str, object], int]:
        assert isinstance(error, InfrastructureError)
        assert 'final contact-control graph observation failed' in str(error)
        events.append('result')
        return {}, int(ExitCode.INFRASTRUCTURE_ERROR)

    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.ContactControlNode',
        lambda _manifest: FakeNode(),
    )
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.SingleThreadedExecutor',
        FakeExecutor,
    )
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.rclpy.init', lambda **_: None)
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.rclpy.try_shutdown',
        lambda: events.append('rclpy_shutdown'),
    )
    monkeypatch.setattr(app, '_prepare', lambda: None)
    monkeypatch.setattr(app, '_wait_for_arm', lambda: None)
    monkeypatch.setattr(app, '_run_control', lambda: None)
    monkeypatch.setattr(app, '_begin_cleanup_window', lambda: events.append('cleanup_window'))
    monkeypatch.setattr(app, '_best_effort_zero', lambda: events.append('zero'))
    monkeypatch.setattr(
        app,
        '_finalize_graph_observation',
        lambda: (_ for _ in ()).throw(RuntimeError('graph API failed')),
    )
    monkeypatch.setattr(app, '_cleanup_wall', lambda: events.append('delete_cleanup'))
    monkeypatch.setattr(app, '_result', result)
    monkeypatch.setattr('robotest_scenarios.contact_control_driver.load_schema', lambda _path: {})
    monkeypatch.setattr(
        'robotest_scenarios.contact_control_driver.write_canonical_json',
        lambda *_args, **_kwargs: events.append('write_result'),
    )

    assert app.run() == int(ExitCode.INFRASTRUCTURE_ERROR)
    assert events == [
        'cleanup_window',
        'zero',
        'begin_cleanup',
        'delete_cleanup',
        'result',
        'write_result',
        'remove_node',
        'shutdown',
        'destroy_node',
        'rclpy_shutdown',
    ]


def test_contact_control_phase_boundaries_publish_only_the_new_phase(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = object.__new__(ContactControlApp)
    app.wall_deadline = time.monotonic() + 30.0
    commands: list[dict[str, object]] = []

    def publish_command(linear_x: float, *, phase: str) -> dict[str, object]:
        record = {
            'angular_z': 0.0,
            'linear_x': linear_x,
            'phase': phase,
            'sim_stamp_ns': node.current_sim_stamp_ns,
        }
        commands.append(record)
        return record

    node = SimpleNamespace(
        contact_snapshot_count=0,
        current_sim_stamp_ns=1_000_000_000,
        phase='HOLD',
        publish_command=publish_command,
        qualified_release_snapshot=None,
        release_required_through_stamp_ns=None,
        stop_command_stamp_ns=1_000_000_000,
        stop_latency_clock_stamp_ns=1_000_000_000,
        tracker=SimpleNamespace(active_start_ns=None),
    )
    app.node = node
    app.hold_complete_stamp_ns = None
    app.reverse_start_stamp_ns = None
    app.final_zero_stamp_ns = None
    app.release_complete_stamp_ns = None
    app.release_observed_clock_stamp_ns = None
    app.contact_clock_bracket = None
    app.release_contact_snapshot_start_count = None
    app.manifest = SimpleNamespace(contact_snapshot_max_clock_lag_ns=220_000_000)

    monkeypatch.setattr(
        app,
        '_spin_once',
        lambda: setattr(node, 'current_sim_stamp_ns', 2_000_000_000),
    )
    app._hold_zero()
    assert commands == []
    assert app.hold_complete_stamp_ns == 2_000_000_000

    commands.clear()
    node.current_sim_stamp_ns = 3_000_000_000

    def wait_for_release(predicate: object, **_kwargs: object) -> None:
        node.qualified_release_snapshot = {
            'sim_stamp_ns': node.release_required_through_stamp_ns + 2_000_000
        }
        node.current_sim_stamp_ns = node.qualified_release_snapshot['sim_stamp_ns']
        assert predicate()

    monkeypatch.setattr(
        app,
        '_spin_once',
        lambda: setattr(node, 'current_sim_stamp_ns', 4_000_000_000),
    )
    monkeypatch.setattr(app, '_wait_for', wait_for_release)
    app._reverse_and_release()
    assert [command['phase'] for command in commands] == ['REVERSE', 'FINAL_ZERO']
    assert commands[0]['sim_stamp_ns'] == 3_000_000_000
    assert commands[1]['sim_stamp_ns'] == 4_000_000_000
