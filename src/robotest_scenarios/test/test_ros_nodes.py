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

import uuid
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from robotest_scenarios.contact_control_driver import ContactControlApp, ContactControlNode
from robotest_scenarios.contact_evidence import load_coverage_manifest
from robotest_scenarios.errors import InfrastructureError, ProtocolError, ScenarioFailureError
from robotest_scenarios.models import load_scenario
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


def test_contact_graph_accepts_pinned_jazzy_rmw_endpoint_gid_size() -> None:
    assert ContactControlApp._endpoint_gid_is_valid('ab' * 16)


@pytest.mark.parametrize('value', ['', 'ab' * 15, 'ab' * 17, 'ab' * 24, 'gg' * 16])
def test_contact_graph_rejects_invalid_rmw_endpoint_gids(value: str) -> None:
    assert not ContactControlApp._endpoint_gid_is_valid(value)


def test_scenario_controller_binds_only_one_post_ready_uuid() -> None:
    _init_ros()
    document = load_scenario(str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'))
    node = ScenarioControllerNode(document)
    try:
        node._on_clock(_clock(1_000_000_000))
        node._on_status(GoalStatusArray())
        node.mark_ready()
        first = uuid.UUID('00000000-0000-0000-0000-000000000001').bytes
        second = uuid.UUID('00000000-0000-0000-0000-000000000002').bytes
        node._on_status(_status(first, 1_100_000_000, GoalStatus.STATUS_ACCEPTED))
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
        node._on_status(GoalStatusArray())
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


def test_contact_node_stops_on_exact_manifest_pair() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
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


def test_contact_node_rejects_snapshot_delivery_after_220ms() -> None:
    _init_ros()
    manifest = load_coverage_manifest(str(REPOSITORY / 'config' / 'collision-coverage.yaml'))
    node = ContactControlNode(manifest)
    try:
        node._on_clock(_clock(1_220_000_001))
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
