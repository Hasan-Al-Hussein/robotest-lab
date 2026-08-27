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
import json
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
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


def test_contact_spawn_success_returns_complete_ready_and_result_projection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    app = ContactControlApp(
        manifest=manifest,
        output_path=tmp_path / 'result.json',
        ready_path=tmp_path / 'ready.json',
        arm_path=tmp_path / 'arm.json',
        armed_path=tmp_path / 'armed.json',
        command_progress_path=tmp_path / 'command-progress.json',
        run_id='spawn-projection-test',
        wall_timeout_s=30.0,
        service_timeout_s=2.0,
        raw_ros_args=[],
    )
    app.wall_asset = tmp_path / 'wall.sdf'
    app.wall_asset_sha256 = 'a' * 64
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
    monkeypatch.setattr(app, '_wait_service', lambda *_args, **_kwargs: None)

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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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
    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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

    with pytest.raises(WallTimeoutError, match='30 s steady-wall escape'):
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
        node.delete_response_stamp_ns = 1_000_000_000
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
        assert node.post_delete_entity_latest_sim_stamp_ns == 1_300_000_000
        assert node.post_delete_wall_pose_count == 0

        transform.header.stamp.nanosec = 200_000_000
        with pytest.raises(ProtocolError, match='heartbeat stamp regressed'):
            node._on_entity_poses(TFMessage(transforms=[transform]))

        transform.header.frame_id = 'map'
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
        node.delete_response_stamp_ns = 2_000_000_000
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
        assert node.post_delete_entity_latest_sim_stamp_ns == 2_300_000_000
        assert node.post_delete_pose_count == 0
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
        wall_timeout_s=1.0,
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
        assert app.cleanup['proof']['kind'] == 'successful_delete_response_cleanup_quiet_pending'
        assert app.cleanup['proof']['response_stamp_ns'] == 1_000_000_000
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
        assert app.cleanup['proof']['kind'] == 'successful_delete_response_cleanup_quiet_pending'
        assert app.cleanup['proof']['response_stamp_ns'] == 2_000_000_000
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


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
