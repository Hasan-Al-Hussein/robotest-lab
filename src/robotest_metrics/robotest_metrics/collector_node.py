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
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import serialize_message
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.msg import Contacts, WorldStatistics
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

from robotest_interfaces.msg import FaultEvent
from robotest_metrics.artifacts import write_json_atomic
from robotest_metrics.collector import CollectorCore
from robotest_metrics.constants import (
    NAV2_LIFECYCLE_NODES,
    PLAN_POSE_CAPACITY,
)
from robotest_metrics.errors import ArtifactError

FeedbackMessage = FollowWaypoints.Impl.FeedbackMessage
PUBLIC_CONTACT_SNAPSHOT_TOPIC = '/robotest/validation/contacts'


def _qos(depth: int, *, reliable: bool) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
    )


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

    def __init__(self, *, contact_progress_path: Path | None = None) -> None:
        """Create subscriptions for every Phase 3 observer stream."""
        super().__init__(
            'metrics_collector',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.core = CollectorCore()
        self.contact_progress_path = contact_progress_path
        self.retained_contact_message_count = 0
        self.latest_clock_ns: int | None = None
        self._subscribe(Clock, '/clock', self._on_clock, _qos(1, reliable=False))
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
            _qos(1, reliable=True),
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
        self.core.record(
            'cmd_vel',
            {
                'angular_z_rad_s': float(message.angular.z),
                'linear_x_m_s': float(message.linear.x),
                'linear_y_m_s': float(message.linear.y),
                'stamp_ns': self._callback_stamp_ns(),
            },
        )

    def _on_contacts(self, message: Contacts) -> None:
        if self.latest_clock_ns is None:
            raise ArtifactError('cannot retain a public contact snapshot before /clock')
        item = _contacts_item(message, delivery_clock_stamp_ns=self.latest_clock_ns)
        if not self.core.record('contacts', item):
            return
        self.retained_contact_message_count += 1
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
    parser.add_argument('--wall-timeout-s', type=float, default=360.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run until the stop-file appears or the bounded wall deadline expires."""
    raw_arguments = list(sys.argv) if argv is None else [sys.argv[0], *argv]
    arguments = _parser().parse_args(remove_ros_args(args=raw_arguments)[1:])
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
    rclpy.init(args=raw_arguments)
    node: MetricsCollectorNode | None = None
    timed_out = False
    runtime_interrupted = False
    started_wall_ns = time.monotonic_ns()
    try:
        node = MetricsCollectorNode(contact_progress_path=arguments.contact_progress_file)
        write_json_atomic(
            {
                'node_name': node.get_fully_qualified_name(),
                'started_steady_wall_ns': started_wall_ns,
                'status': 'READY',
            },
            arguments.ready_file,
            maximum_bytes=16_384,
        )
        deadline = time.monotonic() + arguments.wall_timeout_s
        while rclpy.ok() and not arguments.stop_file.exists():
            if time.monotonic() >= deadline:
                timed_out = True
                break
            rclpy.spin_once(node, timeout_sec=0.1)
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
        print(f'metrics collector artifact error: {exc}', file=sys.stderr)
        return 22
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if timed_out:
        return 20
    if runtime_interrupted:
        return 23
    assert node is not None
    return 21 if node.core.snapshot()['quality']['collector_overflow'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
