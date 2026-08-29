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
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import robotest_metrics.collector_node as collector_node
from geometry_msgs.msg import Transform, Twist, Vector3
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rclpy.exceptions import ParameterImmutableException
from rclpy.qos import DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from robotest_metrics.artifacts import canonical_json_bytes
from robotest_metrics.constants import COMMAND_CAPACITY, COMMAND_QOS_DEPTH
from ros_gz_interfaces.msg import Contact, Contacts, JointWrench
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def _collector_cli_arguments(tmp_path: Path) -> list[str]:
    return [
        '--output',
        str(tmp_path / 'capture.json'),
        '--ready-file',
        str(tmp_path / 'ready.json'),
        '--stop-file',
        str(tmp_path / 'stop'),
        '--contact-progress-file',
        str(tmp_path / 'contact-progress.json'),
    ]


def _local_clock_endpoint_gids(node: collector_node.Node) -> tuple[bytes, ...]:
    return tuple(
        bytes(info.endpoint_gid)
        for info in node.get_subscriptions_info_by_topic('/clock')
        if info.node_name == node.get_name() and info.node_namespace == node.get_namespace()
    )


def test_collector_source_is_observer_only() -> None:
    source = inspect.getsource(collector_node.MetricsCollectorNode)
    assert 'create_publisher' not in source
    assert 'create_client' not in source
    assert 'ActionClient' not in source
    assert 'create_subscription' in source
    assert '_subscribe(Clock' not in source


def test_collector_fuses_one_exact_clock_reader_after_time_source_update(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    observed_ros_stamps: list[int] = []
    original_on_clock = collector_node.MetricsCollectorNode._on_clock

    def on_clock_after_time_source(
        node: collector_node.MetricsCollectorNode,
        message: Clock,
    ) -> None:
        observed_ros_stamps.append(int(node.get_clock().now().nanoseconds))
        original_on_clock(node, message)

    monkeypatch.setattr(
        collector_node.MetricsCollectorNode, '_on_clock', on_clock_after_time_source
    )
    rclpy.init()
    node = collector_node.MetricsCollectorNode()
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
        rejected = node.set_parameters([collector_node.Parameter('use_sim_time', value=False)])[0]
        assert rejected.successful is False
        assert 'read-only' in rejected.reason
        assert node.get_parameter('use_sim_time').value is True
        unset = node.set_parameters(
            [
                collector_node.Parameter(
                    'use_sim_time',
                    type_=collector_node.Parameter.Type.NOT_SET,
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

        stamps = (2_000_000_003, 2_000_000_003, 2_050_000_003, 2_040_000_003)
        for stamp_ns in stamps:
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(stamp_ns, 1_000_000_000)
            node.clock_subscription.callback(message)

        assert observed_ros_stamps == list(stamps)
        assert node.get_clock().now().nanoseconds == 2_040_000_003
        assert node.latest_clock_ns == 2_040_000_003
        assert node.core.snapshot()['clock'] == {
            'count': 4,
            'duplicate_count': 1,
            'first_stamp_ns': 2_000_000_003,
            'latest_stamp_ns': 2_040_000_003,
            'maximum_positive_gap_ns': 50_000_000,
            'regression_count': 1,
        }
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_collector_use_sim_time_cannot_be_undeclared() -> None:
    rclpy.init()
    node = collector_node.MetricsCollectorNode()
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


def test_clock_fusion_fails_closed_on_missing_duplicate_or_wrong_qos() -> None:
    exact_qos = collector_node._qos(1, reliable=False)
    exact = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=exact_qos,
        callback=lambda _: None,
    )
    other = SimpleNamespace(
        topic_name='/other',
        msg_type=Clock,
        qos_profile=exact_qos,
        callback=lambda _: None,
    )

    with pytest.raises(RuntimeError, match='exactly one /clock subscription; found 0'):
        collector_node._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[other]), '/clock', lambda _: None
        )
    with pytest.raises(RuntimeError, match='exactly one /clock subscription; found 2'):
        collector_node._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[exact, exact]), '/clock', lambda _: None
        )

    wrong_qos = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=collector_node._qos(2, reliable=False),
        callback=lambda _: None,
    )
    with pytest.raises(RuntimeError, match=r'BEST_EFFORT KEEP_LAST\(1\) VOLATILE'):
        collector_node._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[wrong_qos]), '/clock', lambda _: None
        )

    wrong_type = SimpleNamespace(
        topic_name='/clock',
        msg_type=object,
        qos_profile=exact_qos,
        callback=lambda _: None,
    )
    with pytest.raises(RuntimeError, match='rosgraph_msgs/msg/Clock'):
        collector_node._fuse_time_source_clock_subscription(
            SimpleNamespace(subscriptions=[wrong_type]), '/clock', lambda _: None
        )


def test_clock_fusion_preserves_callback_failure_boundary() -> None:
    events: list[str] = []
    exact_qos = collector_node._qos(1, reliable=False)

    def time_source_callback(_message: Clock) -> None:
        events.append('time_source')

    def evidence_callback(_message: Clock) -> None:
        events.append('evidence')
        raise RuntimeError('evidence failed')

    subscription = SimpleNamespace(
        topic_name='/clock',
        msg_type=Clock,
        qos_profile=exact_qos,
        callback=time_source_callback,
    )
    collector_node._fuse_time_source_clock_subscription(
        SimpleNamespace(subscriptions=[subscription]), '/clock', evidence_callback
    )
    with pytest.raises(RuntimeError, match='evidence failed'):
        subscription.callback(Clock())
    assert events == ['time_source', 'evidence']

    def failed_time_source(_message: Clock) -> None:
        events.append('failed_time_source')
        raise RuntimeError('time source failed')

    subscription.callback = failed_time_source
    collector_node._fuse_time_source_clock_subscription(
        SimpleNamespace(subscriptions=[subscription]), '/clock', evidence_callback
    )
    with pytest.raises(RuntimeError, match='time source failed'):
        subscription.callback(Clock())
    assert events == ['time_source', 'evidence', 'failed_time_source']


def test_collector_constructor_destroys_node_when_clock_fusion_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    destroyed_names: list[str] = []
    original_destroy_node = collector_node.Node.destroy_node

    def fail_fusion(*_args: object) -> None:
        raise RuntimeError('clock fusion failed')

    def record_destroy(node: collector_node.Node) -> None:
        destroyed_names.append(node.get_name())
        original_destroy_node(node)

    monkeypatch.setattr(collector_node, '_fuse_time_source_clock_subscription', fail_fusion)
    monkeypatch.setattr(collector_node.Node, 'destroy_node', record_destroy)
    rclpy.init()
    try:
        with pytest.raises(RuntimeError, match='clock fusion failed'):
            collector_node.MetricsCollectorNode()
        assert destroyed_names == ['metrics_collector']
    finally:
        rclpy.try_shutdown()


def test_collector_waits_for_post_clock_contact_and_destroys_cleanly(
    tmp_path,
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
        idle_executor = SimpleNamespace(
            spin_once=lambda **_: pytest.fail('boundary checks must not spin')
        )
        assert not collector_node._wait_for_startup_ready(
            node,
            executor=idle_executor,
            deadline=collector_node.time.monotonic() - 1.0,
            stop_file=tmp_path / 'no-stop',
        )
        stop_file = tmp_path / 'stop'
        stop_file.touch()
        assert not collector_node._wait_for_startup_ready(
            node,
            executor=idle_executor,
            deadline=collector_node.time.monotonic() + 1.0,
            stop_file=stop_file,
        )
        stop_file.unlink()

        callback_order = iter(
            (
                lambda: node._on_contacts(message),
                lambda: node.clock_subscription.callback(zero_clock),
                lambda: node._on_contacts(message),
                lambda: node.clock_subscription.callback(clock),
                lambda: node._on_contacts(message),
            )
        )
        spin_count = 0

        class FakeExecutor:
            @staticmethod
            def spin_once(*, timeout_sec: float) -> None:
                nonlocal spin_count
                assert timeout_sec == 0.1
                spin_count += 1
                next(callback_order)()

        assert collector_node._wait_for_startup_ready(
            node,
            executor=FakeExecutor(),
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


def test_contact_progress_coalesces_writes_without_dropping_contacts(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    progress_path = tmp_path / 'contact-progress.json'
    writes: list[dict[str, object]] = []

    def capture_write(
        document: dict[str, object],
        path: Path,
        *,
        maximum_bytes: int,
    ) -> None:
        assert path == progress_path
        assert maximum_bytes == collector_node.CONTACT_PROGRESS_MAX_BYTES
        writes.append(document)

    monkeypatch.setattr(collector_node, 'write_json_atomic', capture_write)
    rclpy.init()
    node = collector_node.MetricsCollectorNode(contact_progress_path=progress_path)
    try:
        node.latest_clock_ns = 10_000_000_000

        def retain(stamp: int) -> None:
            message = Contacts()
            message.header.stamp.sec = stamp
            message.contacts = [Contact()]
            node._on_contacts(message)

        retain(1)
        assert writes == []
        assert node.flush_contact_progress(now_ns=0) is True
        assert writes[-1]['retained_message_count'] == 1

        retain(2)
        assert node.flush_contact_progress(now_ns=100_000_000) is False
        retain(3)
        assert node.flush_contact_progress(now_ns=900_000_000) is False
        assert len(writes) == 1
        assert node.core.snapshot()['streams']['contacts']['quality']['retained_count'] == 3

        assert node.flush_contact_progress(now_ns=1_000_000_000) is True
        assert writes[-1]['latest_retained_stamp_ns'] == 3_000_000_000
        assert writes[-1]['retained_message_count'] == 3
        assert node.flush_contact_progress(now_ns=1_100_000_000) is False

        retained = node.core.snapshot()['streams']['contacts']['items']
        assert node.flush_contact_progress(
            force=True,
            retained_contacts=retained,
            now_ns=1_100_000_000,
        )
        assert writes[-1]['latest_retained_stamp_ns'] == retained[-1]['stamp_ns']
        assert writes[-1]['retained_message_count'] == len(retained)
        assert len(writes) == 3
        assert 'write_json_atomic' not in inspect.getsource(node._on_contacts)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_failed_contact_progress_write_remains_dirty(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    progress_path = tmp_path / 'contact-progress.json'
    rclpy.init()
    node = collector_node.MetricsCollectorNode(contact_progress_path=progress_path)
    try:
        node.latest_clock_ns = 2_000_000_000
        message = Contacts()
        message.header.stamp.sec = 1
        message.contacts = [Contact()]
        node._on_contacts(message)

        def fail_write(*_args: object, **_kwargs: object) -> None:
            raise collector_node.ArtifactError('durable write failed')

        monkeypatch.setattr(collector_node, 'write_json_atomic', fail_write)
        with pytest.raises(collector_node.ArtifactError, match='durable write failed'):
            node.flush_contact_progress(now_ns=1)
        assert node.contact_progress_dirty is True
        assert node.contact_progress_last_write_ns is None
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def test_contact_progress_interval_starts_after_durable_write(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    progress_path = tmp_path / 'contact-progress.json'
    writes: list[dict[str, object]] = []
    clock = iter((0, 1_500_000_000, 1_600_000_000))
    monkeypatch.setattr(collector_node.time, 'monotonic_ns', lambda: next(clock))
    monkeypatch.setattr(
        collector_node,
        'write_json_atomic',
        lambda document, *_args, **_kwargs: writes.append(document),
    )
    rclpy.init()
    node = collector_node.MetricsCollectorNode(contact_progress_path=progress_path)
    try:
        node.latest_clock_ns = 3_000_000_000
        first = Contacts()
        first.header.stamp.sec = 1
        first.contacts = [Contact()]
        node._on_contacts(first)
        assert node.flush_contact_progress() is True
        assert node.contact_progress_last_write_ns == 1_500_000_000

        second = Contacts()
        second.header.stamp.sec = 2
        second.contacts = [Contact()]
        node._on_contacts(second)
        assert node.flush_contact_progress() is False
        assert len(writes) == 1
        assert node.contact_progress_dirty is True
    finally:
        node.destroy_node()
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
    path = NavPath()
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


@pytest.mark.parametrize(
    'unpaired_arguments',
    (
        ['--command-progress-file', 'command-progress.json'],
        ['--command-progress-run-id', 'scenario1-trial01'],
    ),
)
def test_command_progress_arguments_must_be_paired(
    tmp_path: Path,
    unpaired_arguments: list[str],
) -> None:
    with pytest.raises(SystemExit, match='must be provided together'):
        collector_node.main(_collector_cli_arguments(tmp_path) + unpaired_arguments)


@pytest.mark.parametrize(
    'conflicting_name',
    ('capture.json', 'ready.json', 'stop', 'contact-progress.json'),
)
def test_command_progress_path_must_be_distinct(
    tmp_path: Path,
    conflicting_name: str,
) -> None:
    with pytest.raises(SystemExit, match='command-progress path must be distinct'):
        collector_node.main(
            [
                *_collector_cli_arguments(tmp_path),
                '--command-progress-file',
                str(tmp_path / conflicting_name),
                '--command-progress-run-id',
                'scenario1-trial01',
            ]
        )


def test_command_progress_path_must_be_fresh(tmp_path: Path) -> None:
    progress_path = tmp_path / 'command-progress.json'
    progress_path.write_text('stale', encoding='utf-8')
    with pytest.raises(SystemExit, match='stale command progress file'):
        collector_node.main(
            [
                *_collector_cli_arguments(tmp_path),
                '--command-progress-file',
                str(progress_path),
                '--command-progress-run-id',
                'scenario1-trial01',
            ]
        )


def test_first_retained_command_writes_exact_canonical_bounded_progress(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    progress_path = tmp_path / 'command-progress.json'
    rclpy.init()
    node = collector_node.MetricsCollectorNode(
        command_progress_path=progress_path,
        command_progress_run_id='scenario1-trial01',
    )
    try:
        callback_stamps = iter((4_000_000_005, 4_000_000_006))
        steady_stamps = iter((8_000_000_009, 8_000_000_010))
        monkeypatch.setattr(node, '_callback_stamp_ns', lambda: next(callback_stamps))
        monkeypatch.setattr(collector_node.time, 'monotonic_ns', lambda: next(steady_stamps))
        first = Twist()
        first.linear.x = 0.25
        first.linear.y = -0.5
        first.angular.z = 0.75
        node._on_cmd_vel(first)
        first_bytes = progress_path.read_bytes()
        document = json.loads(first_bytes)
        assert set(document) == {
            'angular_z_rad_s',
            'linear_x_m_s',
            'linear_y_m_s',
            'observed_steady_ns',
            'producer',
            'public_topic',
            'retained_command_count',
            'run_id',
            'schema_version',
            'stamp_ns',
        }
        assert document == {
            'angular_z_rad_s': 0.75,
            'linear_x_m_s': 0.25,
            'linear_y_m_s': -0.5,
            'observed_steady_ns': 8_000_000_009,
            'producer': 'robotest_metrics/metrics_collector',
            'public_topic': '/robotest/cmd_vel',
            'retained_command_count': 1,
            'run_id': 'scenario1-trial01',
            'schema_version': 1,
            'stamp_ns': 4_000_000_005,
        }
        assert first_bytes == canonical_json_bytes(document)
        assert len(first_bytes) <= collector_node.COMMAND_PROGRESS_MAX_BYTES

        second = Twist()
        second.linear.x = 9.0
        second.linear.y = 8.0
        second.angular.z = 7.0
        node._on_cmd_vel(second)
        assert progress_path.read_bytes() == first_bytes
        assert node.core.snapshot()['streams']['cmd_vel']['quality']['retained_count'] == 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


@pytest.mark.parametrize(
    ('executor_add_succeeds', 'executor_shutdown_outcome'),
    (
        (True, 'success'),
        (True, 'false'),
        (True, 'none'),
        (True, 'raise'),
        (False, 'success'),
        (False, 'false'),
        (False, 'none'),
        (False, 'raise'),
    ),
)
def test_main_uses_one_dedicated_executor_and_cleans_up_add_failures(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    executor_add_succeeds: bool,
    executor_shutdown_outcome: str,
) -> None:
    stop_file = tmp_path / 'stop'
    calls: list[object] = []

    class FakeCore:
        @staticmethod
        def snapshot() -> dict[str, object]:
            return {
                'quality': {'collector_overflow': False},
                'streams': {'contacts': {'items': [{'stamp_ns': 1}]}},
            }

    class FakeNode:
        def __init__(self) -> None:
            self.context = object()
            self.core = FakeCore()
            self.latest_retained_contact_stamp_ns = 1
            self.pre_clock_contact_message_count = 0
            self.startup_ready = False

        @staticmethod
        def get_fully_qualified_name() -> str:
            return '/robotest/metrics_collector'

        @staticmethod
        def destroy_node() -> None:
            calls.append('destroy_node')

        @staticmethod
        def flush_contact_progress(
            *,
            force: bool = False,
            retained_contacts: object = None,
        ) -> bool:
            calls.append(('flush_contact_progress', force, retained_contacts))
            return True

    node = FakeNode()

    class FakeExecutor:
        def __init__(self, *, context: object) -> None:
            assert context is node.context
            self.spin_count = 0
            calls.append('executor_init')

        @staticmethod
        def add_node(added_node: object) -> bool:
            assert added_node is node
            calls.append('add_node')
            return executor_add_succeeds

        def spin_once(self, *, timeout_sec: float) -> None:
            assert timeout_sec == 0.1
            self.spin_count += 1
            calls.append(('spin_once', self.spin_count))
            if self.spin_count == 1:
                node.startup_ready = True
            else:
                stop_file.touch()

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
            if executor_shutdown_outcome == 'success':
                return True
            if executor_shutdown_outcome == 'false':
                return False
            return None

    monkeypatch.setattr(collector_node, 'MetricsCollectorNode', lambda **_: node)
    monkeypatch.setattr(collector_node, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(collector_node.rclpy, 'init', lambda **_: calls.append('rclpy_init'))
    monkeypatch.setattr(collector_node.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(collector_node.rclpy, 'shutdown', lambda: calls.append('rclpy_shutdown'))
    monkeypatch.setattr(
        collector_node.rclpy,
        'spin_once',
        lambda *_args, **_kwargs: pytest.fail('global rclpy.spin_once must not be used'),
    )
    monkeypatch.setattr(collector_node, 'write_json_atomic', lambda *_args, **_kwargs: None)

    if executor_add_succeeds and executor_shutdown_outcome == 'success':
        assert collector_node.main(_collector_cli_arguments(tmp_path)) == 0
    elif executor_add_succeeds:
        failure = (
            'executor shutdown failed: shutdown exploded'
            if executor_shutdown_outcome == 'raise'
            else 'executor shutdown did not complete'
        )
        with pytest.raises(RuntimeError, match=failure):
            collector_node.main(_collector_cli_arguments(tmp_path))
    else:
        with pytest.raises(RuntimeError, match='executor refused the node'):
            collector_node.main(_collector_cli_arguments(tmp_path))

    if executor_add_succeeds:
        assert calls == [
            'rclpy_init',
            'executor_init',
            'add_node',
            ('spin_once', 1),
            ('flush_contact_progress', False, None),
            ('spin_once', 2),
            ('flush_contact_progress', False, None),
            ('flush_contact_progress', True, [{'stamp_ns': 1}]),
            'remove_node',
            'executor_shutdown',
            'destroy_node',
            'rclpy_shutdown',
        ]
    else:
        assert calls == [
            'rclpy_init',
            'executor_init',
            'add_node',
            'executor_shutdown',
            'destroy_node',
            'rclpy_shutdown',
        ]
        if executor_shutdown_outcome != 'success':
            failure = (
                'executor shutdown failed: shutdown exploded'
                if executor_shutdown_outcome == 'raise'
                else 'executor shutdown did not complete'
            )
            assert f'metrics collector teardown warning: {failure}' in capsys.readouterr().err
    assert 'rclpy.spin_once' not in inspect.getsource(collector_node.main)
    assert 'rclpy.spin_once' not in inspect.getsource(collector_node._wait_for_startup_ready)


@pytest.mark.parametrize(
    ('runtime_ok', 'monotonic_values', 'expected_status', 'stop_reason'),
    (
        (True, (0.0, 361.0), 20, 'wall_timeout'),
        (False, (0.0,), 23, 'runtime_shutdown'),
    ),
)
def test_main_preserves_no_contact_timeout_and_shutdown_capture(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    runtime_ok: bool,
    monotonic_values: tuple[float, ...],
    expected_status: int,
    stop_reason: str,
) -> None:
    writes: list[tuple[Path, dict[str, object]]] = []
    clock = iter(monotonic_values)

    class FakeCore:
        @staticmethod
        def snapshot() -> dict[str, object]:
            return {
                'quality': {'collector_overflow': False},
                'streams': {'contacts': {'items': []}},
            }

    class FakeNode:
        context = object()
        core = FakeCore()
        latest_retained_contact_stamp_ns = None
        pre_clock_contact_message_count = 0
        retained_contact_message_count = 0
        startup_ready = False

        @staticmethod
        def flush_contact_progress(**_kwargs: object) -> bool:
            pytest.fail('empty captures must not write contact progress')

        @staticmethod
        def destroy_node() -> None:
            return None

    class FakeExecutor:
        def __init__(self, *, context: object) -> None:
            assert context is FakeNode.context

        @staticmethod
        def add_node(node: object) -> bool:
            assert isinstance(node, FakeNode)
            return True

        @staticmethod
        def spin_once(*, timeout_sec: float) -> None:
            pytest.fail(f'no-contact boundary must not spin: {timeout_sec}')

        @staticmethod
        def remove_node(node: object) -> None:
            assert isinstance(node, FakeNode)

        @staticmethod
        def shutdown(*, timeout_sec: float) -> bool:
            assert timeout_sec == 1.0
            return True

    def capture_write(document: dict[str, object], path: Path, **_kwargs: object) -> None:
        writes.append((path, document))

    monkeypatch.setattr(collector_node, 'MetricsCollectorNode', lambda **_: FakeNode())
    monkeypatch.setattr(collector_node, 'SingleThreadedExecutor', FakeExecutor)
    monkeypatch.setattr(collector_node.rclpy, 'init', lambda **_: None)
    monkeypatch.setattr(collector_node.rclpy, 'ok', lambda: runtime_ok)
    monkeypatch.setattr(collector_node.rclpy, 'shutdown', lambda: None)
    monkeypatch.setattr(collector_node.time, 'monotonic', lambda: next(clock))
    monkeypatch.setattr(collector_node.time, 'monotonic_ns', lambda: 1)
    monkeypatch.setattr(collector_node, 'write_json_atomic', capture_write)

    assert collector_node.main(_collector_cli_arguments(tmp_path)) == expected_status
    assert [path for path, _document in writes] == [tmp_path / 'capture.json']
    assert writes[0][1]['stop_reason'] == stop_reason
    assert not (tmp_path / 'contact-progress.json').exists()


def test_invalid_and_nonretained_commands_do_not_write_progress(
    tmp_path: Path,
) -> None:
    progress_path = tmp_path / 'command-progress.json'
    rclpy.init()
    node = collector_node.MetricsCollectorNode(
        command_progress_path=progress_path,
        command_progress_run_id='scenario1-trial01',
    )
    try:
        invalid = Twist()
        invalid.linear.x = math.nan
        node._on_cmd_vel(invalid)
        assert not progress_path.exists()
        assert node.core.snapshot()['streams']['cmd_vel']['quality']['invalid_count'] == 1

        for stamp_ns in range(COMMAND_CAPACITY):
            assert node.core.record(
                'cmd_vel',
                {
                    'angular_z_rad_s': 0.0,
                    'linear_x_m_s': 0.0,
                    'linear_y_m_s': 0.0,
                    'stamp_ns': stamp_ns,
                },
            )
        node._on_cmd_vel(Twist())
        assert not progress_path.exists()
        assert node.core.snapshot()['streams']['cmd_vel']['quality']['overflow_count'] == 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
