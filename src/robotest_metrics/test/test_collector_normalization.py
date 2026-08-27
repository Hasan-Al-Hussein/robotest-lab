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

from __future__ import annotations

import inspect

import pytest
import rclpy
import robotest_metrics.collector_node as collector_node
from geometry_msgs.msg import Transform, Vector3
from nav_msgs.msg import Odometry, Path
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from robotest_metrics.constants import COMMAND_CAPACITY, COMMAND_QOS_DEPTH
from ros_gz_interfaces.msg import Contact, Contacts, JointWrench
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def test_collector_source_is_observer_only() -> None:
    source = inspect.getsource(collector_node.MetricsCollectorNode)
    assert 'create_publisher' not in source
    assert 'create_client' not in source
    assert 'ActionClient' not in source
    assert 'create_subscription' in source


def test_collector_waits_for_post_clock_contact_and_destroys_cleanly(
    tmp_path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    rclpy.init()
    node = collector_node.MetricsCollectorNode()
    try:
        assert node.get_name() == 'metrics_collector'
        command_subscription = next(
            item for item in node.subscriptions if item.topic_name == '/cmd_vel'
        )
        command_qos = command_subscription.qos_profile
        assert COMMAND_QOS_DEPTH == COMMAND_CAPACITY
        assert command_qos.history == HistoryPolicy.KEEP_LAST
        assert command_qos.depth == COMMAND_QOS_DEPTH
        assert command_qos.reliability == ReliabilityPolicy.RELIABLE
        assert command_qos.durability == DurabilityPolicy.VOLATILE
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [Contact()]
        zero_clock = Clock()
        clock = Clock()
        clock.clock.sec = 1
        assert not collector_node._wait_for_startup_ready(
            node,
            deadline=collector_node.time.monotonic() - 1.0,
            stop_file=tmp_path / 'no-stop',
        )
        stop_file = tmp_path / 'stop'
        stop_file.touch()
        assert not collector_node._wait_for_startup_ready(
            node,
            deadline=collector_node.time.monotonic() + 1.0,
            stop_file=stop_file,
        )
        stop_file.unlink()

        callback_order = iter(
            (
                lambda: node._on_contacts(message),
                lambda: node._on_clock(zero_clock),
                lambda: node._on_contacts(message),
                lambda: node._on_clock(clock),
                lambda: node._on_contacts(message),
            )
        )
        spin_count = 0

        def spin_once(_node: object, *, timeout_sec: float) -> None:
            nonlocal spin_count
            assert timeout_sec == 0.1
            spin_count += 1
            next(callback_order)()

        monkeypatch.setattr(collector_node.rclpy, 'spin_once', spin_once)
        assert collector_node._wait_for_startup_ready(
            node,
            deadline=collector_node.time.monotonic() + 1.0,
            stop_file=stop_file,
        )
        assert spin_count == 5
        assert node.pre_clock_contact_message_count == 2
        assert node.core.snapshot()['streams']['contacts']['quality']['retained_count'] == 1
        assert node.latest_retained_contact_stamp_ns == 1_000_000_000
        assert node.startup_ready is True
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.try_shutdown()


def test_scan_normalization_retains_only_bounded_metadata_and_payload_hash() -> None:
    message = LaserScan()
    message.header.frame_id = 'laser'
    message.header.stamp.sec = 2
    message.header.stamp.nanosec = 3
    message.ranges = [1.0, 2.0, float('inf')]
    item = collector_node._scan_item(message)
    assert item['stamp_ns'] == 2_000_000_003
    assert item['range_count'] == 3
    assert len(item['payload_sha256']) == 64
    assert 'ranges' not in item


def test_odometry_and_plan_normalization_are_planar() -> None:
    odometry = Odometry()
    odometry.header.frame_id = 'world'
    odometry.pose.pose.position.x = 1.0
    odometry.pose.pose.position.y = 2.0
    odometry.pose.pose.orientation.w = 1.0
    odometry.twist.twist.linear.x = 0.5
    item = collector_node._odometry_item(odometry)
    assert item['x_m'] == 1.0
    assert item['y_m'] == 2.0
    assert item['yaw_rad'] == 0.0
    path = Path()
    path.header.frame_id = 'map'
    path.poses = []
    assert collector_node._plan_item(path) == {'frame_id': 'map', 'poses': [], 'stamp_ns': 0}


def test_transform_normalization_uses_translation_and_quaternion_yaw() -> None:
    transform = Transform()
    transform.translation.x = 1.0
    transform.translation.y = -2.0
    transform.rotation.z = 0.0
    transform.rotation.w = 1.0
    item = collector_node._transform_item(10, 'map', 'odom', transform)
    assert item == {
        'child_frame_id': 'odom',
        'frame_id': 'map',
        'orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
        'stamp_ns': 10,
        'x_m': 1.0,
        'y_m': -2.0,
        'yaw_rad': 0.0,
        'z_m': 0.0,
    }


def test_contact_normalization_extracts_exact_names_depths_and_normal_force() -> None:
    message = Contacts()
    message.header.stamp.sec = 1
    contact = Contact()
    contact.collision1.name = 'robotest::base::collision'
    contact.collision2.name = 'wall::link::collision'
    contact.depths = [0.01]
    normal = Vector3(x=1.0, y=0.0, z=0.0)
    wrench = JointWrench()
    wrench.body_1_wrench.force.x = -5.0
    contact.normals = [normal]
    contact.wrenches = [wrench]
    message.contacts = [contact]
    item = collector_node._contacts_item(message, delivery_clock_stamp_ns=1_010_000_000)
    assert item['stamp_ns'] == 1_000_000_000
    assert item['frame_id'] == ''
    assert item['delivery_clock_stamp_ns'] == 1_010_000_000
    assert item['delivery_clock_offset_ns'] == 10_000_000
    assert item['contacts'] == [
        {
            'collision1': 'robotest::base::collision',
            'collision2': 'wall::link::collision',
            'maximum_penetration_depth_m': 0.01,
            'maximum_normal_force_n': 5.0,
        }
    ]


def test_contact_normalization_rejects_impossible_public_snapshot_shapes() -> None:
    empty = Contacts()
    empty.header.stamp.sec = 1
    with pytest.raises(collector_node.ArtifactError, match='between one and 16'):
        collector_node._contacts_item(empty, delivery_clock_stamp_ns=1_000_000_000)

    oversized = Contacts()
    oversized.header.stamp.sec = 1
    oversized.contacts = [Contact() for _ in range(17)]
    with pytest.raises(collector_node.ArtifactError, match='between one and 16'):
        collector_node._contacts_item(oversized, delivery_clock_stamp_ns=1_000_000_000)

    framed = Contacts()
    framed.header.stamp.sec = 1
    framed.header.frame_id = 'world'
    framed.contacts = [Contact()]
    with pytest.raises(collector_node.ArtifactError, match='frame_id'):
        collector_node._contacts_item(framed, delivery_clock_stamp_ns=1_000_000_000)


def test_runtime_ros_names_are_relative_except_architectural_globals() -> None:
    source = inspect.getsource(collector_node.MetricsCollectorNode)
    assert '/robotest/' not in source
    assert "'/clock'" in source
    assert "'/tf'" in source


@pytest.mark.parametrize(
    ('stale_index', 'label'),
    ((0, 'output'), (1, 'ready'), (2, 'stop'), (3, 'contact progress')),
)
def test_collector_rejects_every_stale_artifact_path(
    tmp_path, stale_index: int, label: str
) -> None:
    paths = [
        tmp_path / 'capture.json',
        tmp_path / 'ready.json',
        tmp_path / 'stop',
        tmp_path / 'contact-progress.json',
    ]
    paths[stale_index].write_text('stale', encoding='utf-8')
    with pytest.raises(SystemExit, match=rf'stale {label} file'):
        collector_node.main(
            [
                '--output',
                str(paths[0]),
                '--ready-file',
                str(paths[1]),
                '--stop-file',
                str(paths[2]),
                '--contact-progress-file',
                str(paths[3]),
            ]
        )
