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

import pytest
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from robotest_scenarios.contact_control_driver import ContactControlApp, ContactControlNode
from robotest_scenarios.contact_evidence import load_coverage_manifest
from robotest_scenarios.errors import ProtocolError
from robotest_scenarios.models import load_scenario
from robotest_scenarios.scenario_controller import ScenarioControllerNode
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
