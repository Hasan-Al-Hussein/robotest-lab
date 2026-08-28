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

"""Bounded, observer-only ROS 2 metrics evidence collector."""

from __future__ import annotations

import argparse
import hashlib
import math
import struct
import sys
import time
from collections.abc import Callable, Sequence
from contextlib import suppress
from itertools import islice
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from lifecycle_msgs.msg import TransitionEvent
from nav2_msgs.action import FollowWaypoints
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.subscription import Subscription
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.msg import Contacts, WorldStatistics
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from robotest_interfaces.msg import FaultEvent
from robotest_metrics.artifacts import write_json_atomic
from robotest_metrics.collector import CollectorCore
from robotest_metrics.constants import (
    COMMAND_QOS_DEPTH,
    NAV2_LIFECYCLE_NODES,
    PLAN_POSE_CAPACITY,
)
from robotest_metrics.errors import ArtifactError

FeedbackMessage = FollowWaypoints.Impl.FeedbackMessage
PUBLIC_CONTACT_SNAPSHOT_TOPIC = '/robotest/validation/contacts'
PUBLIC_COMMAND_TOPIC = '/robotest/cmd_vel'
COMMAND_PROGRESS_MAX_BYTES = 4_096


def _qos(depth: int, *, reliable: bool) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
    )


def _fuse_time_source_clock_subscription(
    node: Node,
    topic: str,
    evidence_callback: Callable[[Clock], None],
) -> Subscription:
    clock_subscriptions = [
        subscription for subscription in node.subscriptions if subscription.topic_name == topic
    ]
    if len(clock_subscriptions) != 1:
        raise RuntimeError(
            f'use_sim_time must create exactly one {topic} subscription; '
            f'found {len(clock_subscriptions)}'
        )
    subscription = clock_subscriptions[0]
    if subscription.msg_type is not Clock:
        raise RuntimeError(f'use_sim_time {topic} subscription must use rosgraph_msgs/msg/Clock')
    if subscription.qos_profile != _qos(1, reliable=False):
        raise RuntimeError(
            f'use_sim_time {topic} subscription must use BEST_EFFORT KEEP_LAST(1) VOLATILE QoS'
        )
    time_source_callback = subscription.callback

    def fused_callback(message: Clock) -> None:
        time_source_callback(message)
        evidence_callback(message)

    subscription.callback = fused_callback
    return subscription


def _require_use_sim_time(parameters: list[Parameter]) -> SetParametersResult:
    for parameter in parameters:
        if parameter.name == 'use_sim_time' and (
            parameter.type_ != Parameter.Type.BOOL or parameter.value is not True
        ):
            return SetParametersResult(
                successful=False,
                reason='use_sim_time must remain true while the fused /clock reader is active',
            )
    return SetParametersResult(successful=True)


def _time_ns(value: Any) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def _yaw(quaternion: Any) -> float:
    siny = 2.0 * (
        float(quaternion.w) * float(quaternion.z) + float(quaternion.x) * float(quaternion.y)
    )
    cosy = 1.0 - 2.0 * (float(quaternion.y) ** 2 + float(quaternion.z) ** 2)
    return math.atan2(siny, cosy)


def _uuid(value: Any) -> str:
    raw = bytes(value.uuid)
    if len(raw) != 16:
        return raw.hex()
    hex_value = raw.hex()
    return '-'.join(
        (
            hex_value[0:8],
            hex_value[8:12],
            hex_value[12:16],
            hex_value[16:20],
            hex_value[20:32],
        )
    )


def _pose_item(stamp_ns: int, frame_id: str, pose: Any) -> dict[str, Any]:
    return {
        'frame_id': frame_id,
        'orientation_xyzw': [
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        ],
        'stamp_ns': stamp_ns,
        'x_m': float(pose.position.x),
        'y_m': float(pose.position.y),
        'yaw_rad': _yaw(pose.orientation),
        'z_m': float(pose.position.z),
    }


def _transform_item(
    stamp_ns: int,
    frame_id: str,
    child_frame_id: str,
    transform: Any,
) -> dict[str, Any]:
    return {
        'child_frame_id': child_frame_id,
        'frame_id': frame_id,
        'orientation_xyzw': [
            float(transform.rotation.x),
            float(transform.rotation.y),
            float(transform.rotation.z),
            float(transform.rotation.w),
        ],
        'stamp_ns': stamp_ns,
        'x_m': float(transform.translation.x),
        'y_m': float(transform.translation.y),
        'yaw_rad': _yaw(transform.rotation),
        'z_m': float(transform.translation.z),
    }


def _odometry_item(message: Odometry) -> dict[str, Any]:
    item = _pose_item(_time_ns(message.header.stamp), message.header.frame_id, message.pose.pose)
    item.update(
        {
            'angular_z_rad_s': float(message.twist.twist.angular.z),
            'child_frame_id': message.child_frame_id,
            'linear_x_m_s': float(message.twist.twist.linear.x),
            'linear_y_m_s': float(message.twist.twist.linear.y),
            'nonplanar_integrity': {
                'pose_covariance_sha256': hashlib.sha256(
                    struct.pack('!36d', *[float(value) for value in message.pose.covariance])
                ).hexdigest(),
                'twist': {
                    'angular_x_rad_s': float(message.twist.twist.angular.x),
                    'angular_y_rad_s': float(message.twist.twist.angular.y),
                    'angular_z_rad_s': float(message.twist.twist.angular.z),
                    'linear_x_m_s': float(message.twist.twist.linear.x),
                    'linear_y_m_s': float(message.twist.twist.linear.y),
                    'linear_z_m_s': float(message.twist.twist.linear.z),
                },
                'twist_covariance_sha256': hashlib.sha256(
                    struct.pack('!36d', *[float(value) for value in message.twist.covariance])
                ).hexdigest(),
                'z_m': float(message.pose.pose.position.z),
            },
        }
    )
    return item


def _scan_item(message: LaserScan) -> dict[str, Any]:
    payload = bytes(serialize_message(message))
    return {
        'frame_id': message.header.frame_id,
        'payload_sha256': hashlib.sha256(payload).hexdigest(),
        'range_count': len(message.ranges),
        'stamp_ns': _time_ns(message.header.stamp),
    }


def _plan_item(message: NavPath) -> dict[str, Any]:
    return {
        'frame_id': message.header.frame_id,
        'poses': [
            {
                'x_m': float(pose.pose.position.x),
                'y_m': float(pose.pose.position.y),
            }
            for pose in islice(message.poses, PLAN_POSE_CAPACITY + 1)
        ],
        'stamp_ns': _time_ns(message.header.stamp),
    }


def _maximum_normal_force(contact: Any) -> float | None:
    maximum: float | None = None
    for index, wrench in enumerate(contact.wrenches):
        if index >= len(contact.normals):
            break
        normal = contact.normals[index]
        force = wrench.body_1_wrench.force
        value = abs(
            float(force.x) * float(normal.x)
            + float(force.y) * float(normal.y)
            + float(force.z) * float(normal.z)
        )
        maximum = value if maximum is None else max(maximum, value)
    return maximum


def _maximum_penetration_depth(contact: Any) -> float | None:
    maximum: float | None = None
    for depth in contact.depths:
        value = float(depth)
        maximum = value if maximum is None else max(maximum, value)
    return maximum


def _contacts_item(message: Contacts, *, delivery_clock_stamp_ns: int) -> dict[str, Any]:
    if message.header.frame_id != '':
        raise ArtifactError('authoritative contact snapshot frame_id must be empty')
    if not 1 <= len(message.contacts) <= 16:
        raise ArtifactError(
            'authoritative contact snapshot must contain between one and 16 records'
        )
    stamp_ns = _time_ns(message.header.stamp)
    if stamp_ns <= 0:
        raise ArtifactError('authoritative contact snapshot stamp must be positive')
    if delivery_clock_stamp_ns < 0:
        raise ArtifactError('contact snapshot delivery clock stamp cannot be negative')
    return {
        'contacts': [
            {
                'collision1': contact.collision1.name,
                'collision2': contact.collision2.name,
                'maximum_penetration_depth_m': _maximum_penetration_depth(contact),
                'maximum_normal_force_n': _maximum_normal_force(contact),
            }
            for contact in message.contacts
        ],
        'frame_id': message.header.frame_id,
        'delivery_clock_offset_ns': delivery_clock_stamp_ns - stamp_ns,
        'delivery_clock_stamp_ns': delivery_clock_stamp_ns,
        'stamp_ns': stamp_ns,
    }


def _fault_event_item(message: FaultEvent) -> dict[str, Any]:
    """Normalize the v2 wire event to the mission runner's bounded projection."""
    return {
        'accepted': bool(message.accepted),
        'actual_stamp_ns': _time_ns(message.actual_time),
        'affected_message_count': int(message.affected_message_count),
        'arm_commit_stamp_ns': _time_ns(message.arm_commit_time),
        'arm_margin_ns': int(message.arm_margin_ns),
        'bound_goal_uuid': _uuid(message.bound_goal_uuid),
        'bound_t0_ns': _time_ns(message.bound_t0),
        'committed_fault_count': int(message.committed_fault_count),
        'committed_generation': int(message.committed_generation),
        'committed_schedule_hash': str(message.committed_schedule_hash),
        'configured_activation_stamp_ns': _time_ns(message.configured_activation_time),
        'configured_deactivation_stamp_ns': _time_ns(message.configured_deactivation_time),
        'detail': str(message.detail),
        'event_sequence': int(message.event_sequence),
        'event_type': int(message.event_type),
        'fault_id': str(message.fault_id),
        'header_stamp_ns': _time_ns(message.header.stamp),
        'input_sequence': int(message.input_sequence),
        'mode': int(message.mode),
        'raw_input_count': int(message.raw_input_count),
        'replayed': bool(message.replayed),
        'requested_fault_count': int(message.requested_fault_count),
        'requested_generation': int(message.requested_generation),
        'requested_goal_uuid': _uuid(message.requested_goal_uuid),
        'requested_schedule_hash': str(message.requested_schedule_hash),
        'requested_t0_ns': _time_ns(message.requested_t0),
        'schema_version': int(message.schema_version),
        'seed': int(message.seed),
        'stamp_ns': _time_ns(message.header.stamp),
        'state_after': int(message.state_after),
        'state_before': int(message.state_before),
        'target': int(message.target),
        'validated_output_count': int(message.validated_output_count),
    }


class MetricsCollectorNode(Node):
    """Normalize only observed ROS messages into a bounded ``CollectorCore``."""

    def __init__(
        self,
        *,
        contact_progress_path: Path | None = None,
        command_progress_path: Path | None = None,
        command_progress_run_id: str | None = None,
    ) -> None:
        """Create subscriptions for every Phase 3 observer stream."""
        if (command_progress_path is None) != (command_progress_run_id is None):
            raise ValueError(
                'command_progress_path and command_progress_run_id must be provided together'
            )
        super().__init__(
            'metrics_collector',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        try:
            self.set_descriptor(
                'use_sim_time',
                ParameterDescriptor(
                    type=Parameter.Type.BOOL.value,
                    description='Required by the fused /clock reader',
                    read_only=True,
                ),
            )
            self.clock_subscription = _fuse_time_source_clock_subscription(
                self,
                '/clock',
                self._on_clock,
            )
            self.add_on_set_parameters_callback(_require_use_sim_time)
        except BaseException:
            with suppress(BaseException):
                self.destroy_node()
            raise
        self.core = CollectorCore()
        self.contact_progress_path = contact_progress_path
        self.command_progress_path = command_progress_path
        self.command_progress_run_id = command_progress_run_id
        self.command_progress_written = False
        self.retained_contact_message_count = 0
        self.latest_retained_contact_stamp_ns: int | None = None
        self.pre_clock_contact_message_count = 0
        self.latest_clock_ns: int | None = None
        self._subscribe(
            Odometry,
            'validation/ground_truth',
            lambda message: self.core.record('ground_truth', _odometry_item(message)),
            _qos(10, reliable=True),
        )
        for stream, topic in (
            ('raw_odom', 'raw/odom'),
            ('odom', 'odom'),
        ):
            self._subscribe(
                Odometry,
                topic,
                self._record_callback(stream, _odometry_item),
                _qos(10, reliable=False),
            )
        for stream, topic in (
            ('raw_scan', 'raw/scan'),
            ('scan', 'scan'),
        ):
            self._subscribe(
                LaserScan,
                topic,
                self._record_callback(stream, _scan_item),
                _qos(5, reliable=False),
            )
        self._subscribe(
            NavPath,
            'navigation/plan',
            lambda message: self.core.record_plan(_plan_item(message)),
            _qos(5, reliable=True),
        )
        self._subscribe(
            Twist,
            'cmd_vel',
            self._on_cmd_vel,
            # The evidence reader must retain every admissible command while the
            # actuator-facing bridge remains KEEP_LAST(1).
            _qos(COMMAND_QOS_DEPTH, reliable=True),
        )
        self._subscribe(
            Contacts,
            'validation/contacts',
            self._on_contacts,
            _qos(10, reliable=True),
        )
        self._subscribe(
            WorldStatistics,
            'validation/world_stats',
            self._on_world_stats,
            _qos(10, reliable=True),
        )
        self._subscribe(TFMessage, '/tf', self._on_tf, _qos(100, reliable=True))
        self._subscribe(
            FaultEvent,
            'faults/events',
            self._on_fault_event,
            _qos(100, reliable=True),
        )
        self._subscribe(
            FeedbackMessage,
            'follow_waypoints/_action/feedback',
            self._on_feedback,
            _qos(10, reliable=True),
        )
        self._subscribe(
            GoalStatusArray,
            'follow_waypoints/_action/status',
            self._on_action_status,
            _qos(10, reliable=True),
        )
        self._subscribe(
            CollisionMonitorState,
            'collision_monitor_state',
            self._on_collision_monitor,
            _qos(10, reliable=True),
        )
        for node_name in NAV2_LIFECYCLE_NODES:
            self._subscribe(
                TransitionEvent,
                f'{node_name}/transition_event',
                self._lifecycle_callback(node_name),
                _qos(10, reliable=True),
            )

    def _subscribe(
        self,
        message_type: Any,
        topic: str,
        callback: Callable[[Any], None],
        qos: QoSProfile,
    ) -> None:
        self.create_subscription(message_type, topic, callback, qos)

    def _callback_stamp_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _record_callback(
        self,
        stream: str,
        normalize: Callable[[Any], dict[str, Any]],
    ) -> Callable[[Any], None]:
        def callback(message: Any) -> None:
            self.core.record(stream, normalize(message))

        return callback

    def _on_clock(self, message: Clock) -> None:
        stamp = _time_ns(message.clock)
        self.latest_clock_ns = stamp
        self.core.observe_clock(stamp)

    def _on_cmd_vel(self, message: Twist) -> None:
        stamp_ns = self._callback_stamp_ns()
        item = {
            'angular_z_rad_s': float(message.angular.z),
            'linear_x_m_s': float(message.linear.x),
            'linear_y_m_s': float(message.linear.y),
            'stamp_ns': stamp_ns,
        }
        retained = self.core.record(
            'cmd_vel',
            item,
        )
        if not retained or self.command_progress_path is None or self.command_progress_written:
            return
        assert self.command_progress_run_id is not None
        write_json_atomic(
            {
                'angular_z_rad_s': item['angular_z_rad_s'],
                'linear_x_m_s': item['linear_x_m_s'],
                'linear_y_m_s': item['linear_y_m_s'],
                'observed_steady_ns': time.monotonic_ns(),
                'producer': 'robotest_metrics/metrics_collector',
                'public_topic': PUBLIC_COMMAND_TOPIC,
                'retained_command_count': 1,
                'run_id': self.command_progress_run_id,
                'schema_version': 1,
                'stamp_ns': stamp_ns,
            },
            self.command_progress_path,
            maximum_bytes=COMMAND_PROGRESS_MAX_BYTES,
        )
        self.command_progress_written = True

    def _on_contacts(self, message: Contacts) -> None:
        if self.latest_clock_ns is None or self.latest_clock_ns <= 0:
            self.pre_clock_contact_message_count += 1
            return
        item = _contacts_item(message, delivery_clock_stamp_ns=self.latest_clock_ns)
        if not self.core.record('contacts', item):
            return
        self.retained_contact_message_count += 1
        self.latest_retained_contact_stamp_ns = int(item['stamp_ns'])
        if self.contact_progress_path is not None:
            write_json_atomic(
                {
                    'latest_retained_stamp_ns': int(item['stamp_ns']),
                    'producer': 'robotest_metrics/metrics_collector',
                    'public_topic': PUBLIC_CONTACT_SNAPSHOT_TOPIC,
                    'retained_message_count': self.retained_contact_message_count,
                    'schema_version': 1,
                },
                self.contact_progress_path,
                maximum_bytes=16_384,
            )

    @property
    def startup_ready(self) -> bool:
        """Return whether clock and one authoritative contact seed are retained."""
        clock_seen = self.latest_clock_ns is not None and self.latest_clock_ns > 0
        contact_seeded = self.latest_retained_contact_stamp_ns is not None
        return clock_seen and contact_seeded

    def _on_world_stats(self, message: WorldStatistics) -> None:
        self.core.record(
            'world_stats',
            {
                'paused': bool(message.paused),
                'reported_real_time_factor': float(message.real_time_factor),
                'sim_stamp_ns': _time_ns(message.sim_time),
                'stamp_ns': _time_ns(message.sim_time),
                'steady_wall_ns': time.monotonic_ns(),
            },
        )

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            parent = transform.header.frame_id.strip('/')
            child = transform.child_frame_id.strip('/')
            stream = {
                ('map', 'odom'): 'tf_map_odom',
                ('odom', 'base_footprint'): 'tf_odom_base_footprint',
            }.get((parent, child))
            if stream is None:
                continue
            self.core.record(
                stream,
                _transform_item(
                    _time_ns(transform.header.stamp),
                    parent,
                    child,
                    transform.transform,
                ),
            )

    def _on_fault_event(self, message: FaultEvent) -> None:
        self.core.record('fault_events', _fault_event_item(message))

    def _on_feedback(self, message: Any) -> None:
        self.core.record_state_transition(
            kind='waypoint_feedback',
            subject=_uuid(message.goal_id),
            value=int(message.feedback.current_waypoint),
            stamp_ns=self._callback_stamp_ns(),
            details={'steady_wall_ns': time.monotonic_ns()},
        )

    def _on_action_status(self, message: GoalStatusArray) -> None:
        stamp = self._callback_stamp_ns()
        for status in message.status_list:
            self.core.record_state_transition(
                kind='action_status',
                subject=_uuid(status.goal_info.goal_id),
                value=int(status.status),
                stamp_ns=stamp,
                details={'accepted_stamp_ns': _time_ns(status.goal_info.stamp)},
            )

    def _on_collision_monitor(self, message: CollisionMonitorState) -> None:
        self.core.record_state_transition(
            kind='collision_monitor',
            subject=message.polygon_name or '<none>',
            value=int(message.action_type),
            stamp_ns=self._callback_stamp_ns(),
        )

    def _lifecycle_callback(self, node_name: str) -> Callable[[TransitionEvent], None]:
        def callback(message: TransitionEvent) -> None:
            self.core.record_state_transition(
                kind='lifecycle',
                subject=node_name,
                value=message.goal_state.label,
                stamp_ns=self._callback_stamp_ns(),
                details={
                    'goal_state_id': int(message.goal_state.id),
                    'start_state': message.start_state.label,
                    'transition': message.transition.label,
                },
            )

        return callback


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Collect bounded observer-only RoboTest Phase 3 evidence.',
    )
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ready-file', required=True, type=Path)
    parser.add_argument('--stop-file', required=True, type=Path)
    parser.add_argument('--contact-progress-file', required=True, type=Path)
    parser.add_argument('--command-progress-file', type=Path)
    parser.add_argument('--command-progress-run-id')
    parser.add_argument('--wall-timeout-s', type=float, default=360.0)
    return parser


def _wait_for_startup_ready(
    node: MetricsCollectorNode,
    *,
    executor: SingleThreadedExecutor,
    deadline: float,
    stop_file: Path,
) -> bool:
    """Spin until both clock and a post-clock contact seed are retained."""
    while rclpy.ok() and not stop_file.exists() and not node.startup_ready:
        if time.monotonic() >= deadline:
            return False
        executor.spin_once(timeout_sec=0.1)
    return rclpy.ok() and node.startup_ready


def main(argv: Sequence[str] | None = None) -> int:
    """Run until the stop-file appears or the bounded wall deadline expires."""
    raw_arguments = list(sys.argv) if argv is None else [sys.argv[0], *argv]
    parser = _parser()
    arguments = parser.parse_args(remove_ros_args(args=raw_arguments)[1:])
    if (arguments.command_progress_file is None) != (arguments.command_progress_run_id is None):
        raise SystemExit(
            '--command-progress-file and --command-progress-run-id must be provided together'
        )
    if not math.isfinite(arguments.wall_timeout_s) or arguments.wall_timeout_s <= 0.0:
        raise SystemExit('--wall-timeout-s must be finite and positive')
    for label, path in (
        ('output', arguments.output),
        ('ready', arguments.ready_file),
        ('stop', arguments.stop_file),
        ('contact progress', arguments.contact_progress_file),
    ):
        if path.exists():
            raise SystemExit(f'stale {label} file already exists: {path}')
    if arguments.command_progress_file is not None and (
        arguments.command_progress_file.exists() or arguments.command_progress_file.is_symlink()
    ):
        raise SystemExit(
            f'stale command progress file already exists: {arguments.command_progress_file}'
        )
    artifact_paths = {
        path.expanduser().resolve()
        for path in (
            arguments.output,
            arguments.ready_file,
            arguments.stop_file,
            arguments.contact_progress_file,
        )
    }
    if len(artifact_paths) != 4:
        raise SystemExit('output, ready, stop, and contact-progress paths must be distinct')
    if arguments.command_progress_file is not None:
        resolved_command_progress = arguments.command_progress_file.expanduser().resolve()
        if resolved_command_progress in artifact_paths:
            raise SystemExit(
                'command-progress path must be distinct from output, ready, stop, '
                'and contact-progress paths'
            )
    rclpy.init(args=raw_arguments)
    node: MetricsCollectorNode | None = None
    executor: SingleThreadedExecutor | None = None
    executor_node_added = False
    artifact_failure = False
    timed_out = False
    runtime_interrupted = False
    started_wall_ns = time.monotonic_ns()
    try:
        node = MetricsCollectorNode(
            contact_progress_path=arguments.contact_progress_file,
            command_progress_path=arguments.command_progress_file,
            command_progress_run_id=arguments.command_progress_run_id,
        )
        executor = SingleThreadedExecutor(context=node.context)
        if not executor.add_node(node):
            raise RuntimeError('metrics collector executor refused the node')
        executor_node_added = True
        deadline = time.monotonic() + arguments.wall_timeout_s
        startup_ready = _wait_for_startup_ready(
            node,
            executor=executor,
            deadline=deadline,
            stop_file=arguments.stop_file,
        )
        if startup_ready:
            write_json_atomic(
                {
                    'initial_contact_stamp_ns': node.latest_retained_contact_stamp_ns,
                    'node_name': node.get_fully_qualified_name(),
                    'pre_clock_contact_message_count': node.pre_clock_contact_message_count,
                    'started_steady_wall_ns': started_wall_ns,
                    'status': 'READY',
                },
                arguments.ready_file,
                maximum_bytes=16_384,
            )
            while rclpy.ok() and not arguments.stop_file.exists():
                if time.monotonic() >= deadline:
                    timed_out = True
                    break
                executor.spin_once(timeout_sec=0.1)
        elif rclpy.ok() and not arguments.stop_file.exists():
            timed_out = True
        runtime_interrupted = not rclpy.ok() and not arguments.stop_file.exists()
        capture = node.core.snapshot()
        capture.update(
            {
                'capture_schema_version': 1,
                'finished_steady_wall_ns': time.monotonic_ns(),
                'started_steady_wall_ns': started_wall_ns,
                'stop_reason': (
                    'wall_timeout'
                    if timed_out
                    else 'runtime_shutdown'
                    if runtime_interrupted
                    else 'stop_file'
                ),
            }
        )
        write_json_atomic(capture, arguments.output)
    except ArtifactError as exc:
        artifact_failure = True
        print(f'metrics collector artifact error: {exc}', file=sys.stderr)
        return 22
    finally:
        pending_exception = sys.exc_info()[0] is not None
        teardown_failures: list[str] = []
        if executor is not None:
            if node is not None and executor_node_added:
                try:
                    executor.remove_node(node)
                except Exception as exc:
                    teardown_failures.append(f'executor remove failed: {exc}')
            try:
                if executor.shutdown(timeout_sec=1.0) is not True:
                    teardown_failures.append('executor shutdown did not complete')
            except Exception as exc:
                teardown_failures.append(f'executor shutdown failed: {exc}')
        if node is not None:
            try:
                node.destroy_node()
            except Exception as exc:
                teardown_failures.append(f'node destruction failed: {exc}')
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as exc:
                teardown_failures.append(f'rclpy shutdown failed: {exc}')
        if teardown_failures:
            detail = '; '.join(teardown_failures)
            if pending_exception or artifact_failure:
                print(f'metrics collector teardown warning: {detail}', file=sys.stderr)
            else:
                raise RuntimeError(detail)
    if timed_out:
        return 20
    if runtime_interrupted:
        return 23
    assert node is not None
    return 21 if node.core.snapshot()['quality']['collector_overflow'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
