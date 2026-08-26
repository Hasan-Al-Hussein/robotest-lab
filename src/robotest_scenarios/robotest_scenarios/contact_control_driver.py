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

"""Isolated, bounded ADR 0006 collision-positive-control driver."""

from __future__ import annotations

import argparse
import hashlib
import math
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from robotest_scenarios.artifacts import (
    bounded_diagnostic,
    canonical_json_bytes,
    load_schema,
    sha256_sidecar,
    validate_new_output,
    validate_run_id,
    write_canonical_json,
)
from robotest_scenarios.bounded import PrefixBuffer
from robotest_scenarios.constants import (
    ACTOR_CLEANUP_QUIET_NS,
    ACTOR_INITIAL_POSITION_TOLERANCE_M,
    ACTOR_STATE_CAPACITY,
    ACTOR_YAW_TOLERANCE_RAD,
    CLOCK_TOPIC,
    COMMAND_CAPACITY,
    CONTACT_RECORD_CAPACITY,
    CONTACT_RELEASE_GAP_NS,
    CONTACT_SUMMARY_CAPACITY,
    CONTACT_TOPIC,
    CONTROL_COMMAND_PERIOD_NS,
    CONTROL_CONTACT_DEADLINE_NS,
    CONTROL_FORWARD_MPS,
    CONTROL_HOLD_NS,
    CONTROL_REVERSE_MPS,
    CONTROL_REVERSE_NS,
    CONTROL_ROBOT_START,
    CONTROL_STOP_DEADLINE_NS,
    CONTROL_WALL_POSE,
    DDS_DRAIN_GRACE_S,
    DEFAULT_CONTROL_WALL_TIMEOUT_S,
    DEFAULT_SERVICE_TIMEOUT_S,
    DELETE_SERVICE,
    ENTITY_POSE_HEARTBEAT_MODEL,
    ENTITY_POSE_TOPIC,
    FINAL_COMMAND_TOPIC,
    GROUND_TRUTH_ALIGNMENT_NS,
    GROUND_TRUTH_CAPACITY,
    GROUND_TRUTH_TOPIC,
    SPAWN_SERVICE,
    ExitCode,
)
from robotest_scenarios.contact_evidence import (
    ContactEpisodeTracker,
    CoverageManifest,
    load_coverage_manifest,
)
from robotest_scenarios.errors import (
    ArtifactError,
    InfrastructureError,
    ProtocolError,
    RobotestScenarioError,
    ScenarioFailureError,
    ValidationError,
    WallTimeoutError,
)
from robotest_scenarios.geometry import quaternion_yaw, shortest_yaw_error, stamp_to_ns
from robotest_scenarios.models import package_schema_path
from robotest_scenarios.provenance import (
    contact_control_configuration,
    contact_control_configuration_sha256,
    contact_source_binding,
    file_sha256,
)

WALL_NAME = 'phase3_contact_control_wall'
FIXTURE_ID = 'collision_positive_control'
# ROS 2 Jazzy's pinned rmw ABI stores 16 endpoint-GID bytes, rendered as hex here.
ENDPOINT_GID_HEX_LENGTH = 32
FORBIDDEN_NAVIGATION_NODES = frozenset(
    {
        'amcl',
        'behavior_server',
        'bt_navigator',
        'collision_monitor',
        'controller_server',
        'docking_server',
        'lifecycle_manager_navigation',
        'map_server',
        'nav2_container',
        'planner_server',
        'route_server',
        'smoother_server',
        'velocity_smoother',
        'waypoint_follower',
    }
)


@dataclass(frozen=True, slots=True)
class WallPoseEvidence:
    """One independently observed Gazebo wall pose."""

    collector_sequence: int
    stamp_ns: int
    x: float
    y: float
    z: float
    yaw: float

    def as_dict(self) -> dict[str, int | float]:
        """Return stable artifact fields."""
        return {
            'collector_sequence': self.collector_sequence,
            'stamp_ns': self.stamp_ns,
            'x': self.x,
            'y': self.y,
            'yaw': self.yaw,
            'z': self.z,
        }


def _wall_pose_message() -> Pose:
    x_value, y_value, z_value, yaw_value = CONTROL_WALL_POSE
    pose = Pose()
    pose.position.x = x_value
    pose.position.y = y_value
    pose.position.z = z_value
    pose.orientation.z = math.sin(yaw_value / 2.0)
    pose.orientation.w = math.cos(yaw_value / 2.0)
    return pose


class ContactControlNode(Node):
    """ROS graph adapter for the isolated contact fixture."""

    def __init__(self, manifest: CoverageManifest) -> None:
        super().__init__(
            'contact_control_driver',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.manifest = manifest
        self.sequence = 0
        self.current_sim_stamp_ns = 0
        self.clock_seen = False
        self.clock_first_stamp_ns: int | None = None
        self.clock_sample_count = 0
        self.clock_max_gap_ns = 0
        self.clock_regression_count = 0
        self.last_contact_stamp_ns: int | None = None
        self.contact_first_stamp_ns: int | None = None
        self.contact_snapshot_count = 0
        self.contact_max_gap_ns = 0
        self.fatal_error: RobotestScenarioError | None = None
        self.cleanup_mode = False
        self.protocol_error_count = 0
        self.spawn_request_sequence: int | None = None
        self.spawn_request_stamp_ns: int | None = None
        self.wall_poses: list[WallPoseEvidence] = []
        self.last_wall_pose: WallPoseEvidence | None = None
        self.last_wall_stamp_ns: int | None = None
        self.last_ground_truth_stamp_ns: int | None = None
        self.entity_pose_message_count = 0
        self.last_entity_pose_heartbeat_stamp_ns: int | None = None
        self.post_delete_entity_message_count = 0
        self.post_delete_entity_latest_sim_stamp_ns: int | None = None
        self.post_delete_wall_pose_count = 0
        self.delete_response_stamp_ns: int | None = None
        self.control_started_stamp_ns: int | None = None
        self.contact_deadline_ns: int | None = None
        self.first_qualifying_contact: dict[str, Any] | None = None
        self.stop_command_stamp_ns: int | None = None
        self.stop_latency_ns: int | None = None
        self.stop_latency_clock_stamp_ns: int | None = None
        self.pending_stop_contact_stamp_ns: int | None = None
        self.future_snapshot_delivery_count = 0
        self.release_required_through_stamp_ns: int | None = None
        self.qualified_release_snapshot: dict[str, int] | None = None
        self.exact_pair_snapshot_record_count = 0
        self.snapshot_contact_record_count = 0
        self.snapshot_contact_accepted_count = 0
        self.snapshot_contact_invalid_count = 0
        self.snapshot_contact_overflow_count = 0
        self.snapshot_contact_first_overflow_sequence: int | None = None
        self.snapshot_contact_first_overflow_stamp_ns: int | None = None
        self.classified_contact_record_count = 0
        self.latest_ground_truth: dict[str, Any] | None = None
        self.motion_active = False
        self.phase = 'PREPARE'
        self.counterpart_trackers = {WALL_NAME: ContactEpisodeTracker(WALL_NAME)}
        self.tracker = self.counterpart_trackers[WALL_NAME]
        self.contact_snapshots = PrefixBuffer[dict[str, Any]](
            'contact_snapshots', CONTACT_SUMMARY_CAPACITY
        )
        self.contact_snapshot_records = PrefixBuffer[dict[str, Any]](
            'contact_snapshot_records', CONTACT_RECORD_CAPACITY
        )
        self.commands = PrefixBuffer[dict[str, Any]]('commands', COMMAND_CAPACITY)
        self.actor_state = PrefixBuffer[dict[str, Any]]('actor_state', ACTOR_STATE_CAPACITY)
        self.ground_truth = PrefixBuffer[dict[str, Any]]('ground_truth', GROUND_TRUTH_CAPACITY)

        reliable_10 = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_subscription = self.create_subscription(
            Clock,
            CLOCK_TOPIC,
            self._guard(self._on_clock, allow_during_cleanup=True),
            clock_qos,
        )
        self.contact_subscription = self.create_subscription(
            Contacts, CONTACT_TOPIC, self._guard(self._on_contacts), reliable_10
        )
        self.entity_pose_subscription = self.create_subscription(
            TFMessage,
            ENTITY_POSE_TOPIC,
            self._guard(self._on_entity_poses, allow_during_cleanup=True),
            reliable_10,
        )
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            GROUND_TRUTH_TOPIC,
            self._guard(self._on_ground_truth),
            reliable_10,
        )
        self.command_publisher = self.create_publisher(Twist, FINAL_COMMAND_TOPIC, command_qos)
        self.spawn_client = self.create_client(SpawnEntity, SPAWN_SERVICE)
        self.delete_client = self.create_client(DeleteEntity, DELETE_SERVICE)

    def _next_sequence(self) -> int:
        self.sequence += 1
        return self.sequence

    def _guard(
        self,
        callback: Callable[[Any], None],
        *,
        allow_during_cleanup: bool = False,
    ) -> Callable[[Any], None]:
        def guarded(message: Any) -> None:
            if self.cleanup_mode and not allow_during_cleanup:
                return
            if self.fatal_error is not None and not (self.cleanup_mode and allow_during_cleanup):
                return
            try:
                callback(message)
            except RobotestScenarioError as exc:
                self.set_fatal(exc)
            except Exception as exc:
                self.set_fatal(ProtocolError(f'unhandled contact callback error: {exc}'))

        return guarded

    def begin_cleanup(self) -> None:
        """Keep only clock and wall-pose evidence alive after a latched fatal."""
        self.cleanup_mode = True

    def set_fatal(self, error: RobotestScenarioError) -> None:
        """Latch the first callback failure without escaping the executor."""
        if self.fatal_error is None:
            self.fatal_error = error
            if isinstance(error, ProtocolError):
                self.protocol_error_count += 1

    def _on_clock(self, message: Clock) -> None:
        stamp_ns = stamp_to_ns(message.clock)
        if self.clock_seen and stamp_ns < self.current_sim_stamp_ns:
            self.clock_regression_count += 1
            raise ProtocolError('simulation clock regressed during contact control')
        if self.clock_first_stamp_ns is None:
            self.clock_first_stamp_ns = stamp_ns
        elif stamp_ns > self.current_sim_stamp_ns:
            self.clock_max_gap_ns = max(self.clock_max_gap_ns, stamp_ns - self.current_sim_stamp_ns)
        self.clock_sample_count += 1
        self.current_sim_stamp_ns = stamp_ns
        self.clock_seen = True
        if (
            self.pending_stop_contact_stamp_ns is not None
            and stamp_ns >= self.pending_stop_contact_stamp_ns
        ):
            self.stop_latency_clock_stamp_ns = stamp_ns
            self.stop_latency_ns = stamp_ns - self.pending_stop_contact_stamp_ns
            self.pending_stop_contact_stamp_ns = None
            if self.first_qualifying_contact is not None:
                self.first_qualifying_contact['stop_latency_clock_stamp_ns'] = stamp_ns
                self.first_qualifying_contact['stop_latency_upper_bound_ns'] = self.stop_latency_ns
            if self.stop_latency_ns > CONTROL_STOP_DEADLINE_NS:
                raise ScenarioFailureError('contact stop command missed its 0.10 s deadline')

    @staticmethod
    def _collision_name(value: object) -> str:
        if not isinstance(value, str) or not value or len(value.encode('utf-8')) > 4096:
            raise ProtocolError('contact collision name is empty or exceeds 4096 bytes')
        return value

    def _on_contacts(self, message: Contacts) -> None:
        if message.header.frame_id != '':
            self.contact_snapshots.reject_invalid()
            raise ProtocolError('authoritative contact snapshot frame_id must be empty')
        if not 1 <= len(message.contacts) <= 16:
            self.contact_snapshots.reject_invalid()
            raise ProtocolError(
                'authoritative contact snapshot must contain between one and 16 records'
            )
        stamp_ns = stamp_to_ns(message.header.stamp, positive=True)
        delivery_clock_offset_ns = self.current_sim_stamp_ns - stamp_ns
        if abs(delivery_clock_offset_ns) > self.manifest.contact_snapshot_max_clock_lag_ns:
            self.contact_snapshots.reject_invalid()
            raise ProtocolError('authoritative contact snapshot delivery skew exceeded 220 ms')
        if delivery_clock_offset_ns < 0:
            self.future_snapshot_delivery_count += 1
        if self.last_contact_stamp_ns is not None and stamp_ns <= self.last_contact_stamp_ns:
            self.contact_snapshots.reject_invalid()
            raise ProtocolError('authoritative contact snapshot stamp did not strictly advance')
        if self.contact_first_stamp_ns is None:
            self.contact_first_stamp_ns = stamp_ns
        elif self.last_contact_stamp_ns is not None:
            gap_ns = stamp_ns - self.last_contact_stamp_ns
            self.contact_max_gap_ns = max(self.contact_max_gap_ns, gap_ns)
            if gap_ns > self.manifest.contact_snapshot_max_gap_ns:
                self.contact_snapshots.reject_invalid()
                raise ProtocolError('authoritative contact snapshot gap exceeded 220 ms')
        self.last_contact_stamp_ns = stamp_ns
        self.contact_snapshot_count += 1
        summary_sequence = self._next_sequence()
        classified_count = 0
        exact_count = 0
        counted_snapshot_records: list[dict[str, Any]] = []
        snapshot_pairs: dict[str, list[tuple[str, str]]] = {}
        qualifying_candidate: dict[str, Any] | None = None
        for contact in message.contacts:
            self.snapshot_contact_record_count += 1
            try:
                collision_a = self._collision_name(contact.collision1.name)
                collision_b = self._collision_name(contact.collision2.name)
                classification = self.manifest.classify(collision_a, collision_b)
            except ProtocolError:
                self.snapshot_contact_invalid_count += 1
                self.contact_snapshot_records.reject_invalid()
                self.contact_snapshots.reject_invalid()
                raise
            normalized_pair = tuple(sorted((collision_a, collision_b)))
            if classification is not None:
                disposition = 'counted'
            elif (
                collision_a in self.manifest.robot_collisions
                and collision_b in self.manifest.robot_collisions
            ):
                disposition = 'robot_internal_excluded'
            elif frozenset((collision_a, collision_b)) in self.manifest.support_exclusions:
                disposition = 'support_ground_excluded'
            else:
                self.snapshot_contact_invalid_count += 1
                self.contact_snapshot_records.reject_invalid()
                self.contact_snapshots.reject_invalid()
                raise ProtocolError(
                    'manifest-v3 public snapshot contains an impossible non-robot pair'
                )
            record_sequence = self._next_sequence()
            record = {
                'collector_sequence': record_sequence,
                'counterpart_collision': (
                    classification['counterpart_collision'] if classification is not None else None
                ),
                'counterpart_model': (
                    classification['counterpart_model'] if classification is not None else None
                ),
                'disposition': disposition,
                'normalized_pair': list(normalized_pair),
                'robot_collision': (
                    classification['robot_collision'] if classification is not None else None
                ),
                'sim_stamp_ns': stamp_ns,
                'snapshot_sequence': summary_sequence,
            }
            try:
                self.contact_snapshot_records.add(
                    record, sequence=record_sequence, stamp_ns=stamp_ns
                )
            except ProtocolError:
                self.snapshot_contact_overflow_count = self.contact_snapshot_records.overflow_count
                self.snapshot_contact_first_overflow_sequence = (
                    self.contact_snapshot_records.first_overflow_sequence
                )
                self.snapshot_contact_first_overflow_stamp_ns = (
                    self.contact_snapshot_records.first_overflow_stamp_ns
                )
                raise
            self.snapshot_contact_accepted_count += 1
            if classification is None:
                continue
            classified_count += 1
            self.classified_contact_record_count += 1
            counterpart = str(classification['counterpart_model'])
            if counterpart == WALL_NAME and self.control_started_stamp_ns is None:
                raise ProtocolError('wall contact preceded positive-control motion')
            tracker = self.counterpart_trackers.get(counterpart)
            if tracker is None:
                if len(self.counterpart_trackers) >= CONTACT_RECORD_CAPACITY:
                    raise ProtocolError('contact counterpart tracker capacity overflowed')
                tracker = ContactEpisodeTracker(counterpart)
                self.counterpart_trackers[counterpart] = tracker
            snapshot_pairs.setdefault(counterpart, []).append(normalized_pair)
            counted_snapshot_records.append(
                {
                    'counterpart_model': counterpart,
                    'normalized_pair': list(normalized_pair),
                    'record_sequence': record_sequence,
                    'snapshot_sequence': summary_sequence,
                }
            )
            if counterpart != WALL_NAME:
                continue
            if normalized_pair != self.manifest.expected_control_pair:
                continue
            exact_count += 1
            self.exact_pair_snapshot_record_count += 1
            if self.first_qualifying_contact is not None or qualifying_candidate is not None:
                continue
            if self.control_started_stamp_ns is None or not self.motion_active:
                raise ProtocolError('qualifying control contact preceded forward motion')
            if stamp_ns < self.control_started_stamp_ns:
                raise ProtocolError('qualifying contact stamp preceded forward command')
            qualifying_candidate = {
                'callback_clock_offset_ns': self.current_sim_stamp_ns - stamp_ns,
                'callback_clock_stamp_ns': self.current_sim_stamp_ns,
                'collector_sequence': record_sequence,
                'normalized_pair': list(normalized_pair),
                'sim_stamp_ns': stamp_ns,
            }
        for counterpart, tracker in self.counterpart_trackers.items():
            tracker.observe_snapshot(stamp_ns, snapshot_pairs.get(counterpart, []))
        self.contact_snapshots.add(
            {
                'classified_count': classified_count,
                'collector_sequence': summary_sequence,
                'counted_snapshot_records': counted_snapshot_records,
                'exact_pair_count': exact_count,
                'delivery_clock_offset_ns': delivery_clock_offset_ns,
                'delivery_clock_stamp_ns': self.current_sim_stamp_ns,
                'snapshot_record_count': len(message.contacts),
                'sim_stamp_ns': stamp_ns,
            },
            sequence=summary_sequence,
            stamp_ns=stamp_ns,
        )
        if (
            self.release_required_through_stamp_ns is not None
            and self.qualified_release_snapshot is None
            and stamp_ns > self.release_required_through_stamp_ns
            and WALL_NAME not in snapshot_pairs
        ):
            self.qualified_release_snapshot = {
                'collector_sequence': summary_sequence,
                'sim_stamp_ns': stamp_ns,
            }
        if qualifying_candidate is not None:
            if self.contact_deadline_ns is not None and stamp_ns > self.contact_deadline_ns:
                self.publish_command(0.0, phase='LATE_CONTACT_FAIL_SAFE_ZERO')
                self.motion_active = False
                raise ScenarioFailureError('qualifying contact stamp exceeded the 12.0 s deadline')
            self.first_qualifying_contact = qualifying_candidate
            self.publish_command(0.0, phase='CONTACT_STOP')
            self.stop_command_stamp_ns = self.current_sim_stamp_ns
            if self.stop_command_stamp_ns >= stamp_ns:
                self.stop_latency_clock_stamp_ns = self.stop_command_stamp_ns
                self.stop_latency_ns = self.stop_command_stamp_ns - stamp_ns
            else:
                self.pending_stop_contact_stamp_ns = stamp_ns
                self.stop_latency_clock_stamp_ns = None
                self.stop_latency_ns = None
            self.first_qualifying_contact['stop_latency_clock_stamp_ns'] = (
                self.stop_latency_clock_stamp_ns
            )
            self.first_qualifying_contact['stop_latency_upper_bound_ns'] = self.stop_latency_ns
            self.motion_active = False
            self.phase = 'HOLD'
            if self.stop_latency_ns is not None and not (
                0 <= self.stop_latency_ns <= CONTROL_STOP_DEADLINE_NS
            ):
                raise ScenarioFailureError('contact stop command missed its 0.10 s deadline')

    def _on_entity_poses(self, message: TFMessage) -> None:
        self.entity_pose_message_count += 1
        for transform in message.transforms:
            if transform.child_frame_id == ENTITY_POSE_HEARTBEAT_MODEL:
                heartbeat_stamp_ns = stamp_to_ns(transform.header.stamp, positive=True)
                if (
                    self.last_entity_pose_heartbeat_stamp_ns is not None
                    and heartbeat_stamp_ns < self.last_entity_pose_heartbeat_stamp_ns
                ):
                    raise ProtocolError('entity-pose heartbeat stamp regressed')
                if transform.header.frame_id not in {'robotest_lab', 'world'}:
                    raise ProtocolError('entity-pose heartbeat has an invalid frame')
                self.last_entity_pose_heartbeat_stamp_ns = heartbeat_stamp_ns
                if self.delete_response_stamp_ns is not None:
                    self.post_delete_entity_message_count += 1
                    self.post_delete_entity_latest_sim_stamp_ns = heartbeat_stamp_ns
            if transform.child_frame_id != WALL_NAME:
                continue
            if self.spawn_request_sequence is None:
                raise ProtocolError('positive-control wall existed before its spawn request')
            stamp_ns = stamp_to_ns(transform.header.stamp, positive=True)
            if self.last_wall_stamp_ns is not None and stamp_ns < self.last_wall_stamp_ns:
                self.actor_state.reject_invalid()
                raise ProtocolError('positive-control wall pose stamp regressed')
            self.last_wall_stamp_ns = stamp_ns
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (float(translation.x), float(translation.y), float(translation.z))
            if transform.header.frame_id not in {'robotest_lab', 'world'} or not all(
                math.isfinite(value) for value in values
            ):
                self.actor_state.reject_invalid()
                raise ProtocolError('positive-control wall pose has an invalid frame or value')
            try:
                yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
            except ProtocolError:
                self.actor_state.reject_invalid()
                raise
            sequence = self._next_sequence()
            evidence = WallPoseEvidence(sequence, stamp_ns, *values, yaw)
            if (
                self.delete_response_stamp_ns is not None
                and stamp_ns > self.delete_response_stamp_ns
            ):
                self.post_delete_wall_pose_count += 1
            previous = self.last_wall_pose
            changed = previous is None or any(
                abs(current - prior) > 1e-12
                for current, prior in zip(
                    (evidence.x, evidence.y, evidence.z, evidence.yaw),
                    (previous.x, previous.y, previous.z, previous.yaw),
                    strict=True,
                )
            )
            if changed:
                self.actor_state.add(evidence.as_dict(), sequence=sequence, stamp_ns=stamp_ns)
                self.wall_poses.append(evidence)
            else:
                self.actor_state.observe_unretained()
            self.last_wall_pose = evidence

    def _on_ground_truth(self, message: Odometry) -> None:
        stamp_ns = stamp_to_ns(message.header.stamp, positive=True)
        if (
            self.last_ground_truth_stamp_ns is not None
            and stamp_ns < self.last_ground_truth_stamp_ns
        ):
            self.ground_truth.reject_invalid()
            raise ProtocolError('contact-control ground-truth stamp regressed')
        self.last_ground_truth_stamp_ns = stamp_ns
        position = message.pose.pose.position
        rotation = message.pose.pose.orientation
        values = (float(position.x), float(position.y))
        if message.header.frame_id != 'world' or not all(math.isfinite(value) for value in values):
            self.ground_truth.reject_invalid()
            raise ProtocolError('contact-control ground truth has an invalid frame or value')
        try:
            yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
        except ProtocolError:
            self.ground_truth.reject_invalid()
            raise
        sequence = self._next_sequence()
        record = {
            'collector_sequence': sequence,
            'sim_stamp_ns': stamp_ns,
            'x': values[0],
            'y': values[1],
            'yaw': yaw,
        }
        self.ground_truth.add(record, sequence=sequence, stamp_ns=stamp_ns)
        self.latest_ground_truth = record

    def closed_episodes(self) -> list[dict[str, Any]]:
        """Return every closed counterpart episode in deterministic order."""
        episodes = [
            episode
            for tracker in self.counterpart_trackers.values()
            for episode in tracker.episodes
        ]
        return sorted(
            episodes,
            key=lambda item: (item['start_stamp_ns'], item['counterpart_model']),
        )

    def active_counterpart_count(self) -> int:
        """Return the count of counterpart episodes not yet released."""
        return sum(
            tracker.active_start_ns is not None for tracker in self.counterpart_trackers.values()
        )

    def publish_command(self, linear_x: float, *, phase: str) -> dict[str, Any]:
        """Publish and retain one command from the frozen three-value alphabet."""
        if linear_x not in {CONTROL_FORWARD_MPS, 0.0, CONTROL_REVERSE_MPS}:
            raise ProtocolError('contact-control command is outside the frozen value set')
        if not self.clock_seen or self.current_sim_stamp_ns <= 0:
            raise InfrastructureError('cannot publish contact-control command before /clock')
        message = Twist()
        message.linear.x = linear_x
        message.angular.z = 0.0
        self.command_publisher.publish(message)
        sequence = self._next_sequence()
        record = {
            'angular_z': 0.0,
            'collector_sequence': sequence,
            'linear_x': linear_x,
            'phase': phase,
            'sim_stamp_ns': self.current_sim_stamp_ns,
        }
        self.commands.add(record, sequence=sequence, stamp_ns=self.current_sim_stamp_ns)
        return record

    def start_forward(self) -> None:
        """Start the sole controlled motion exactly once."""
        if self.control_started_stamp_ns is not None:
            raise ProtocolError('forward control may start exactly once')
        record = self.publish_command(CONTROL_FORWARD_MPS, phase='FORWARD')
        self.control_started_stamp_ns = int(record['sim_stamp_ns'])
        self.contact_deadline_ns = self.control_started_stamp_ns + CONTROL_CONTACT_DEADLINE_NS
        self.motion_active = True
        self.phase = 'FORWARD'

    def quality(
        self,
        *,
        publisher_count: int,
        forbidden_nodes: Sequence[str],
        contact_graph_topology: Mapping[str, Any],
    ) -> dict[str, Any]:
        """Return bounded collection and graph-isolation evidence."""
        buffers = {
            'actor_state': self.actor_state.quality(),
            'command': self.commands.quality(),
            'contact_snapshot_records': self.contact_snapshot_records.quality(),
            'contact_snapshots': self.contact_snapshots.quality(),
            'ground_truth': self.ground_truth.quality(),
        }
        source_publisher_counts = {
            'contacts': self.count_publishers(CONTACT_TOPIC),
            'entity_pose': self.count_publishers(ENTITY_POSE_TOPIC),
            'ground_truth': self.count_publishers(GROUND_TRUTH_TOPIC),
        }
        return {
            'all_buffers_bounded': True,
            'buffers': buffers,
            'clock': {
                'first_stamp_ns': self.clock_first_stamp_ns,
                'latest_stamp_ns': self.current_sim_stamp_ns if self.clock_seen else None,
                'max_gap_ns': self.clock_max_gap_ns,
                'regression_count': self.clock_regression_count,
                'sample_count': self.clock_sample_count,
            },
            'collision_monitor_absent': 'collision_monitor' not in forbidden_nodes,
            'contact_graph_topology': dict(contact_graph_topology),
            'forbidden_nodes': sorted(forbidden_nodes),
            'nav2_absent': not forbidden_nodes,
            'overflow_free': self.snapshot_contact_overflow_count == 0
            and all(not item['overflow'] for item in buffers.values()),
            'protocol_error_count': self.protocol_error_count,
            'public_contact_snapshot_stream': {
                'accepted_count': self.snapshot_contact_accepted_count,
                'capacity': CONTACT_RECORD_CAPACITY,
                'first_overflow_sequence': self.snapshot_contact_first_overflow_sequence,
                'first_overflow_stamp_ns': self.snapshot_contact_first_overflow_stamp_ns,
                'ingress_count': self.snapshot_contact_record_count,
                'invalid_count': self.snapshot_contact_invalid_count,
                'overflow': self.snapshot_contact_overflow_count > 0,
                'overflow_count': self.snapshot_contact_overflow_count,
                'retained_count': len(self.contact_snapshot_records.items),
            },
            'public_contact_snapshot_heartbeat': {
                'first_stamp_ns': self.contact_first_stamp_ns,
                'future_delivery_count': self.future_snapshot_delivery_count,
                'latest_stamp_ns': self.last_contact_stamp_ns,
                'max_gap_ns': self.contact_max_gap_ns,
                'snapshot_count': self.contact_snapshot_count,
            },
            'relative_project_names': True,
            'sole_cmd_vel_publisher': publisher_count == 1,
            'source_publisher_counts': source_publisher_counts,
            'source_streams_live': all(count >= 1 for count in source_publisher_counts.values()),
            'cmd_vel_publisher_count': publisher_count,
        }


class ContactControlApp:
    """Execute the isolated fixture and finalize component-only evidence."""

    def __init__(
        self,
        *,
        manifest: CoverageManifest,
        output_path: Path,
        ready_path: Path,
        run_id: str,
        wall_timeout_s: float,
        service_timeout_s: float,
        raw_ros_args: Sequence[str],
    ) -> None:
        self.manifest = manifest
        self.output_path = output_path
        self.ready_path = ready_path
        self.run_id = run_id
        self.wall_timeout_s = wall_timeout_s
        self.service_timeout_s = service_timeout_s
        self.raw_ros_args = raw_ros_args
        self.wall_deadline = time.monotonic() + wall_timeout_s
        self.node: ContactControlNode | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.wall_asset: Path | None = None
        self.wall_asset_sha256: str | None = None
        self.wall_spawn_request_sent = False
        self.wall_spawn_committed = False
        self.spawn_attempt_count = 0
        self.delete_attempt_count = 0
        self.last_graph_check_ns = 0
        self.graph_gate_active = False
        self.last_publisher_count = 0
        self.last_source_publisher_counts = {
            'contacts': 0,
            'entity_pose': 0,
            'ground_truth': 0,
        }
        self.contact_graph_audit_count = 0
        self.contact_graph_first_snapshot: dict[str, Any] | None = None
        self.contact_graph_last_snapshot: dict[str, Any] | None = None
        self.contact_graph_first_sha256: str | None = None
        self.contact_graph_last_sha256: str | None = None
        self.forbidden_nodes: list[str] = []
        self.hold_complete_stamp_ns: int | None = None
        self.reverse_start_stamp_ns: int | None = None
        self.final_zero_stamp_ns: int | None = None
        self.release_complete_stamp_ns: int | None = None
        self.release_observed_clock_stamp_ns: int | None = None
        self.contact_clock_bracket: dict[str, int] | None = None
        self.release_contact_snapshot_start_count: int | None = None
        self.observed_start: dict[str, Any] | None = None
        self.setup_evidence: dict[str, Any] = {
            'observed_robot_start': None,
            'observed_wall': None,
            'spawn': {
                'attempt_count': 0,
                'error': None,
                'request_sequence': None,
                'request_stamp_ns': None,
                'response_sequence': None,
                'response_stamp_ns': None,
                'success': None,
            },
        }
        self.cleanup: dict[str, Any] = {
            'actor_absent': True,
            'delete_attempt_count': 0,
            'delete_success': None,
            'proof': {'kind': 'spawn_not_committed'},
            'required': False,
        }

    @property
    def _node(self) -> ContactControlNode:
        if self.node is None:
            raise InfrastructureError('contact-control node is unavailable')
        return self.node

    @property
    def _executor(self) -> SingleThreadedExecutor:
        if self.executor is None:
            raise InfrastructureError('contact-control executor is unavailable')
        return self.executor

    def _graph_state(self) -> tuple[int, list[str]]:
        publisher_count = self._node.count_publishers(FINAL_COMMAND_TOPIC)
        names = {
            name
            for name, _namespace in self._node.get_node_names_and_namespaces()
            if name in FORBIDDEN_NAVIGATION_NODES
        }
        return publisher_count, sorted(names)

    @staticmethod
    def _endpoint_qos_name(value: Any) -> str:
        name = getattr(value, 'name', None)
        return name if isinstance(name, str) else str(value).rsplit('.', 1)[-1]

    @classmethod
    def _endpoint_evidence(
        cls,
        endpoint: Any,
        *,
        expected_depth: int,
    ) -> dict[str, Any]:
        namespace = str(endpoint.node_namespace).rstrip('/')
        node_fqn = f'{namespace}/{endpoint.node_name}' if namespace else f'/{endpoint.node_name}'
        qos = endpoint.qos_profile
        durability = cls._endpoint_qos_name(qos.durability)
        history = cls._endpoint_qos_name(qos.history)
        reliability = cls._endpoint_qos_name(qos.reliability)
        depth = int(qos.depth)
        try:
            endpoint_gid = bytes(endpoint.endpoint_gid).hex()
        except (AttributeError, TypeError, ValueError):
            endpoint_gid = ''
        return {
            'endpoint_gid': endpoint_gid,
            'node_fqn': node_fqn,
            'qos': {
                'depth': depth,
                'durability': durability,
                'history': history,
                'reliability': reliability,
            },
            'qos_status': {
                'depth_matches_or_unknown': depth <= 0 or depth == expected_depth,
                'durability_volatile': durability == 'VOLATILE',
                'history_keep_last_or_unknown': history
                in {
                    'KEEP_LAST',
                    'SYSTEM_DEFAULT',
                    'UNKNOWN',
                },
                'reliability_reliable': reliability == 'RELIABLE',
            },
            'topic_type': str(endpoint.topic_type),
        }

    @staticmethod
    def _endpoint_gid_is_valid(value: Any) -> bool:
        return (
            isinstance(value, str)
            and len(value) == ENDPOINT_GID_HEX_LENGTH
            and all(character in '0123456789abcdef' for character in value)
        )

    def _contact_graph_snapshot(self) -> dict[str, Any]:
        node = self._node
        public_topic = self.manifest.public_contact_snapshot_topic
        private_topic = self.manifest.private_raw_contact_topic

        def normalized(
            endpoints: Sequence[Any],
            *,
            expected_depth: int,
        ) -> list[dict[str, Any]]:
            return sorted(
                (
                    self._endpoint_evidence(endpoint, expected_depth=expected_depth)
                    for endpoint in endpoints
                ),
                key=lambda item: (
                    item['node_fqn'],
                    item['topic_type'],
                    item['endpoint_gid'],
                    canonical_json_bytes(item['qos']),
                ),
            )

        return {
            'private_raw_publishers': normalized(
                node.get_publishers_info_by_topic(private_topic), expected_depth=64
            ),
            'private_raw_subscribers': normalized(
                node.get_subscriptions_info_by_topic(private_topic), expected_depth=64
            ),
            'public_snapshot_publishers': normalized(
                node.get_publishers_info_by_topic(public_topic), expected_depth=10
            ),
            'topics': {
                'private_raw_contact_topic': private_topic,
                'public_contact_snapshot_topic': public_topic,
            },
        }

    def _contact_graph_is_exact(self, snapshot: Mapping[str, Any]) -> bool:
        expected_nodes = {
            'private_raw_publishers': '/robotest/parameter_bridge',
            'private_raw_subscribers': '/robotest/contact_stream_gate',
            'public_snapshot_publishers': '/robotest/contact_stream_gate',
        }
        if snapshot.get('topics') != {
            'private_raw_contact_topic': self.manifest.private_raw_contact_topic,
            'public_contact_snapshot_topic': self.manifest.public_contact_snapshot_topic,
        }:
            return False
        for key, expected_node in expected_nodes.items():
            endpoints = snapshot.get(key)
            if not isinstance(endpoints, list) or len(endpoints) != 1:
                return False
            endpoint = endpoints[0]
            if not isinstance(endpoint, Mapping):
                return False
            endpoint_gid = endpoint.get('endpoint_gid', '')
            if (
                not self._endpoint_gid_is_valid(endpoint_gid)
                or endpoint.get('node_fqn') != expected_node
                or endpoint.get('topic_type') != 'ros_gz_interfaces/msg/Contacts'
                or not isinstance(endpoint.get('qos_status'), Mapping)
                or not all(endpoint['qos_status'].values())
            ):
                return False
        return True

    def _contact_graph_evidence(self) -> dict[str, Any]:
        return {
            'audit_count': self.contact_graph_audit_count,
            'first_sha256': self.contact_graph_first_sha256,
            'first_snapshot': self.contact_graph_first_snapshot,
            'last_sha256': self.contact_graph_last_sha256,
            'last_snapshot': self.contact_graph_last_snapshot,
        }

    def _audit_contact_graph(self) -> None:
        snapshot = self._contact_graph_snapshot()
        if not self._contact_graph_is_exact(snapshot):
            raise InfrastructureError('contact stream graph ownership, type, or QoS changed')
        sha256 = hashlib.sha256(canonical_json_bytes(snapshot)).hexdigest()
        self.contact_graph_audit_count += 1
        if self.contact_graph_first_snapshot is None:
            self.contact_graph_first_snapshot = snapshot
            self.contact_graph_first_sha256 = sha256
        self.contact_graph_last_snapshot = snapshot
        self.contact_graph_last_sha256 = sha256

    def _enforce_graph_isolation(self) -> None:
        publisher_count, forbidden = self._graph_state()
        self.last_publisher_count = publisher_count
        self.forbidden_nodes = forbidden
        self.last_source_publisher_counts = {
            'contacts': self._node.count_publishers(CONTACT_TOPIC),
            'entity_pose': self._node.count_publishers(ENTITY_POSE_TOPIC),
            'ground_truth': self._node.count_publishers(GROUND_TRUTH_TOPIC),
        }
        if publisher_count != 1:
            raise InfrastructureError(
                f'contact-control cmd_vel requires one publisher, observed {publisher_count}'
            )
        if forbidden:
            raise InfrastructureError(
                'Nav2/collision-monitor nodes are present in the isolated fixture: '
                + ', '.join(forbidden)
            )
        missing_sources = [
            name for name, count in self.last_source_publisher_counts.items() if count < 1
        ]
        if missing_sources:
            raise InfrastructureError(
                'positive-control observation sources disappeared: ' + ', '.join(missing_sources)
            )
        self._audit_contact_graph()

    def _spin_once(self, timeout_s: float = 0.02, *, check_fatal: bool = True) -> None:
        if time.monotonic() >= self.wall_deadline:
            raise WallTimeoutError('contact-control 30 s steady-wall escape elapsed')
        remaining_s = max(0.0, self.wall_deadline - time.monotonic())
        self._executor.spin_once(timeout_sec=min(timeout_s, remaining_s))
        if check_fatal and self._node.fatal_error is not None:
            raise self._node.fatal_error
        now_ns = time.monotonic_ns()
        if (
            check_fatal
            and self.graph_gate_active
            and now_ns - self.last_graph_check_ns >= 100_000_000
        ):
            self._enforce_graph_isolation()
            self.last_graph_check_ns = now_ns

    def _wait_for(
        self,
        predicate: Callable[[], bool],
        *,
        reason: str,
        sim_deadline_ns: int | None = None,
        check_fatal: bool = True,
    ) -> None:
        while not predicate():
            self._spin_once(check_fatal=check_fatal)
            if (
                sim_deadline_ns is not None
                and self._node.clock_seen
                and self._node.current_sim_stamp_ns >= sim_deadline_ns
            ):
                raise ScenarioFailureError(reason)

    def _wait_service(self, client: Any, *, name: str, check_fatal: bool = True) -> None:
        deadline = min(self.wall_deadline, time.monotonic() + self.service_timeout_s)
        while not client.service_is_ready():
            if time.monotonic() >= deadline:
                raise InfrastructureError(f'service unavailable before deadline: {name}')
            self._spin_once(check_fatal=check_fatal)

    def _call_service(
        self,
        client: Any,
        request: Any,
        *,
        name: str,
        check_fatal: bool = True,
    ) -> tuple[Any, int, int]:
        self._wait_service(client, name=name, check_fatal=check_fatal)
        request_sequence = self._node._next_sequence()
        request_stamp_ns = self._node.current_sim_stamp_ns
        try:
            future = client.call_async(request)
        except Exception as exc:
            raise InfrastructureError(f'{name} request could not be sent: {exc}') from exc
        deadline = min(self.wall_deadline, time.monotonic() + self.service_timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                raise InfrastructureError(f'{name} response timed out')
            self._spin_once(check_fatal=check_fatal)
        try:
            response = future.result()
        except Exception as exc:
            raise InfrastructureError(f'{name} response failed: {exc}') from exc
        if response is None:
            raise InfrastructureError(f'{name} returned no response')
        return response, request_sequence, request_stamp_ns

    def _resolve_wall_asset(self) -> None:
        try:
            share = Path(get_package_share_directory('robotest_sim'))
        except Exception as exc:
            raise InfrastructureError(f'cannot resolve robotest_sim share: {exc}') from exc
        asset = share / 'models' / 'phase3_contact_control_wall.sdf'
        if not asset.is_file():
            raise InfrastructureError(f'installed contact wall asset is missing: {asset}')
        self.wall_asset = asset.resolve()
        self.wall_asset_sha256 = file_sha256(self.wall_asset)

    def _base_prerequisites(self) -> bool:
        node = self._node
        if not node.clock_seen or node.current_sim_stamp_ns <= 0:
            return False
        publisher_count, forbidden = self._graph_state()
        self.last_publisher_count = publisher_count
        self.forbidden_nodes = forbidden
        graph_snapshot = self._contact_graph_snapshot()
        return (
            publisher_count == 1
            and not forbidden
            and node.count_publishers(CONTACT_TOPIC) >= 1
            and node.count_publishers(ENTITY_POSE_TOPIC) >= 1
            and node.count_publishers(GROUND_TRUTH_TOPIC) >= 1
            and node.latest_ground_truth is not None
            and node.spawn_client.service_is_ready()
            and node.delete_client.service_is_ready()
            and node.resolve_topic_name(CONTACT_TOPIC)
            == self.manifest.public_contact_snapshot_topic
            and node.contact_snapshot_count >= 1
            and node.last_contact_stamp_ns is not None
            and node.current_sim_stamp_ns >= node.last_contact_stamp_ns
            and node.current_sim_stamp_ns - node.last_contact_stamp_ns
            <= self.manifest.contact_snapshot_max_clock_lag_ns
            and self._contact_graph_is_exact(graph_snapshot)
        )

    def _spawn_wall(self) -> dict[str, Any]:
        if self.wall_asset is None:
            raise InfrastructureError('contact-control wall asset is unresolved')
        if self.spawn_attempt_count != 0:
            raise ProtocolError('contact-control wall spawn may be attempted exactly once')
        self.spawn_attempt_count = 1
        spawn_evidence = self.setup_evidence['spawn']
        spawn_evidence['attempt_count'] = 1
        request = SpawnEntity.Request()
        request.entity_factory.name = WALL_NAME
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf_filename = str(self.wall_asset)
        request.entity_factory.pose = _wall_pose_message()
        request.entity_factory.relative_to = 'world'
        self._wait_service(self._node.spawn_client, name=SPAWN_SERVICE)
        self._node.spawn_request_sequence = self._node._next_sequence()
        self._node.spawn_request_stamp_ns = self._node.current_sim_stamp_ns
        spawn_evidence['request_sequence'] = self._node.spawn_request_sequence
        spawn_evidence['request_stamp_ns'] = self._node.spawn_request_stamp_ns
        try:
            future = self._node.spawn_client.call_async(request)
            self.wall_spawn_request_sent = True
            self.cleanup = {
                'actor_absent': False,
                'delete_attempt_count': 0,
                'delete_success': None,
                'proof': {'kind': 'spawn_request_sent_cleanup_pending'},
                'required': True,
            }
        except Exception as exc:
            spawn_evidence['error'] = bounded_diagnostic(exc)
            raise InfrastructureError(f'{SPAWN_SERVICE} request could not be sent: {exc}') from exc
        deadline = min(self.wall_deadline, time.monotonic() + self.service_timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                spawn_evidence['error'] = 'spawn response timed out'
                raise InfrastructureError(f'{SPAWN_SERVICE} response timed out')
            self._spin_once()
        response = future.result()
        if response is None or not bool(response.success):
            spawn_evidence['success'] = False
            spawn_evidence['error'] = 'spawn response rejected the fixture wall'
            raise InfrastructureError(f'{SPAWN_SERVICE} rejected contact-control wall')
        self.wall_spawn_committed = True
        response_sequence = self._node._next_sequence()
        result = {
            'attempt_count': 1,
            'request_sequence': self._node.spawn_request_sequence,
            'request_stamp_ns': self._node.spawn_request_stamp_ns,
            'response_sequence': response_sequence,
            'response_stamp_ns': self._node.current_sim_stamp_ns,
            'success': True,
        }
        spawn_evidence.update(result)
        return result

    def _observe_wall(self, spawn: dict[str, Any]) -> dict[str, Any]:
        def candidate() -> WallPoseEvidence | None:
            for evidence in self._node.wall_poses:
                if evidence.collector_sequence <= int(spawn['request_sequence']):
                    continue
                if evidence.stamp_ns < int(spawn['request_stamp_ns']):
                    continue
                if evidence.stamp_ns > self._node.current_sim_stamp_ns:
                    continue
                return evidence
            return None

        self._wait_for(candidate, reason='spawned contact-control wall pose was not observed')
        evidence = candidate()
        assert evidence is not None
        x_value, y_value, z_value, yaw_value = CONTROL_WALL_POSE
        position_error = math.sqrt(
            (evidence.x - x_value) ** 2 + (evidence.y - y_value) ** 2 + (evidence.z - z_value) ** 2
        )
        yaw_error = shortest_yaw_error(evidence.yaw, yaw_value)
        result = {
            **evidence.as_dict(),
            'position_error_m': position_error,
            'yaw_error_rad': yaw_error,
        }
        self.setup_evidence['observed_wall'] = result
        if (
            position_error > ACTOR_INITIAL_POSITION_TOLERANCE_M
            or yaw_error > ACTOR_YAW_TOLERANCE_RAD
        ):
            raise ScenarioFailureError('contact-control wall pose missed frozen tolerance')
        return result

    def _fixture_document(self) -> dict[str, Any]:
        return {
            'control': contact_control_configuration(),
            'entity': {
                'asset_sha256': self.wall_asset_sha256,
                'name': WALL_NAME,
                'pose': {
                    'x': CONTROL_WALL_POSE[0],
                    'y': CONTROL_WALL_POSE[1],
                    'yaw': CONTROL_WALL_POSE[3],
                    'z': CONTROL_WALL_POSE[2],
                },
            },
            'expected_pair': list(self.manifest.expected_control_pair),
            'fixture_id': FIXTURE_ID,
            'robot_start': {
                'x': CONTROL_ROBOT_START[0],
                'y': CONTROL_ROBOT_START[1],
                'yaw': CONTROL_ROBOT_START[2],
            },
            'schema_version': 1,
        }

    def _fixture_sha256(self) -> str:
        return hashlib.sha256(canonical_json_bytes(self._fixture_document())).hexdigest()

    def _verify_robot_start(self) -> dict[str, Any]:
        record = self._node.latest_ground_truth
        if record is None:
            raise InfrastructureError('contact-control start has no ground-truth sample')
        record_stamp_ns = int(record['sim_stamp_ns'])
        self._wait_for(
            lambda: self._node.current_sim_stamp_ns >= record_stamp_ns,
            reason='simulation clock did not reach the ground-truth start stamp',
        )
        alignment_error_ns = self._node.current_sim_stamp_ns - record_stamp_ns
        target_x, target_y, target_yaw = CONTROL_ROBOT_START
        position_error = math.hypot(
            float(record['x']) - target_x,
            float(record['y']) - target_y,
        )
        yaw_error = shortest_yaw_error(float(record['yaw']), target_yaw)
        result = {
            **record,
            'alignment_error_ns': alignment_error_ns,
            'position_error_m': position_error,
            'yaw_error_rad': yaw_error,
        }
        self.setup_evidence['observed_robot_start'] = result
        if alignment_error_ns > GROUND_TRUTH_ALIGNMENT_NS:
            raise InfrastructureError('contact-control start ground truth is not aligned')
        if (
            position_error > ACTOR_INITIAL_POSITION_TOLERANCE_M
            or yaw_error > ACTOR_YAW_TOLERANCE_RAD
        ):
            raise ScenarioFailureError('robot did not begin at the frozen control start pose')
        return result

    def _write_ready(self, spawn: dict[str, Any], observed_wall: dict[str, Any]) -> None:
        node = self._node
        ready = {
            'expected_pair': list(self.manifest.expected_control_pair),
            'fixture_sha256': self._fixture_sha256(),
            'identity': {'fixture_id': FIXTURE_ID, 'run_id': self.run_id},
            'observed_wall': observed_wall,
            'observed_robot_start': self.observed_start,
            'producer': 'robotest_scenarios/contact_control_driver',
            'resolved_names': {
                'clock': node.resolve_topic_name(CLOCK_TOPIC),
                'cmd_vel': node.resolve_topic_name(FINAL_COMMAND_TOPIC),
                'contacts': node.resolve_topic_name(CONTACT_TOPIC),
                'entity_pose': node.resolve_topic_name(ENTITY_POSE_TOPIC),
                'ground_truth': node.resolve_topic_name(GROUND_TRUTH_TOPIC),
            },
            'schema_version': 1,
            'spawn': spawn,
        }
        write_canonical_json(self.ready_path, ready, write_sidecar=False)

    def _prepare(self) -> None:
        self._resolve_wall_asset()
        self._wait_for(
            self._base_prerequisites,
            reason='contact-control graph did not become ready',
        )
        self._enforce_graph_isolation()
        self.graph_gate_active = True
        spawn = self._spawn_wall()
        observed_wall = self._observe_wall(spawn)
        self.observed_start = self._verify_robot_start()
        self.setup_evidence['observed_wall'] = observed_wall
        self.setup_evidence['observed_robot_start'] = self.observed_start
        self._write_ready(spawn, observed_wall)

    @staticmethod
    def _advance_schedule(next_stamp_ns: int, current_stamp_ns: int) -> int:
        while next_stamp_ns <= current_stamp_ns:
            next_stamp_ns += CONTROL_COMMAND_PERIOD_NS
        return next_stamp_ns

    def _drive_until_contact(self) -> None:
        node = self._node
        node.start_forward()
        assert node.control_started_stamp_ns is not None
        deadline_ns = node.control_started_stamp_ns + CONTROL_CONTACT_DEADLINE_NS
        next_publish_ns = node.control_started_stamp_ns + CONTROL_COMMAND_PERIOD_NS
        drain_deadline: float | None = None
        while node.first_qualifying_contact is None:
            self._spin_once()
            if node.first_qualifying_contact is not None:
                break
            if node.current_sim_stamp_ns >= deadline_ns:
                if drain_deadline is None:
                    drain_deadline = min(self.wall_deadline, time.monotonic() + DDS_DRAIN_GRACE_S)
                elif time.monotonic() >= drain_deadline:
                    raise ScenarioFailureError('no qualifying contact by the 12.0 s deadline')
                continue
            if node.current_sim_stamp_ns >= next_publish_ns:
                node.publish_command(CONTROL_FORWARD_MPS, phase='FORWARD')
                next_publish_ns = self._advance_schedule(next_publish_ns, node.current_sim_stamp_ns)

    def _hold_zero(self) -> None:
        node = self._node
        if node.stop_command_stamp_ns is None or node.stop_latency_clock_stamp_ns is None:
            raise ProtocolError('zero-command anchor is unavailable after contact')
        hold_until_ns = node.stop_latency_clock_stamp_ns + CONTROL_HOLD_NS
        next_publish_ns = node.stop_latency_clock_stamp_ns + CONTROL_COMMAND_PERIOD_NS
        while node.current_sim_stamp_ns < hold_until_ns:
            self._spin_once()
            if node.current_sim_stamp_ns >= hold_until_ns:
                break
            if node.current_sim_stamp_ns >= next_publish_ns:
                node.publish_command(0.0, phase='HOLD')
                next_publish_ns = self._advance_schedule(next_publish_ns, node.current_sim_stamp_ns)
        self.hold_complete_stamp_ns = node.current_sim_stamp_ns

    def _settle_contact_stop_latency(self) -> None:
        node = self._node
        first = node.first_qualifying_contact
        if first is None:
            raise ProtocolError('qualifying contact is unavailable for stop-latency settlement')
        contact_stamp_ns = int(first['sim_stamp_ns'])
        self._wait_for(
            lambda: node.stop_latency_ns is not None,
            reason='contact stop latency did not settle against /clock within 0.10 s',
            sim_deadline_ns=contact_stamp_ns + CONTROL_STOP_DEADLINE_NS,
        )
        assert node.stop_latency_ns is not None
        if not 0 <= node.stop_latency_ns <= CONTROL_STOP_DEADLINE_NS:
            raise ScenarioFailureError('contact stop command missed its 0.10 s deadline')

    def _reverse_and_release(self) -> None:
        node = self._node
        record = node.publish_command(CONTROL_REVERSE_MPS, phase='REVERSE')
        self.reverse_start_stamp_ns = int(record['sim_stamp_ns'])
        node.phase = 'REVERSE'
        reverse_until_ns = self.reverse_start_stamp_ns + CONTROL_REVERSE_NS
        next_publish_ns = self.reverse_start_stamp_ns + CONTROL_COMMAND_PERIOD_NS
        while node.current_sim_stamp_ns < reverse_until_ns:
            self._spin_once()
            if node.current_sim_stamp_ns >= reverse_until_ns:
                break
            if node.current_sim_stamp_ns >= next_publish_ns:
                node.publish_command(CONTROL_REVERSE_MPS, phase='REVERSE')
                next_publish_ns = self._advance_schedule(next_publish_ns, node.current_sim_stamp_ns)
        final = node.publish_command(0.0, phase='FINAL_ZERO')
        self.final_zero_stamp_ns = int(final['sim_stamp_ns'])
        node.release_required_through_stamp_ns = self.final_zero_stamp_ns + CONTACT_RELEASE_GAP_NS
        node.qualified_release_snapshot = None
        self.release_contact_snapshot_start_count = node.contact_snapshot_count
        node.phase = 'RELEASE'

        def released() -> bool:
            minimum = self._release_boundary_ns()
            assert minimum is not None
            release_snapshot = node.qualified_release_snapshot
            if release_snapshot is None or node.tracker.active_start_ns is not None:
                return False
            snapshot_stamp_ns = release_snapshot['sim_stamp_ns']
            if snapshot_stamp_ns <= minimum or node.current_sim_stamp_ns < snapshot_stamp_ns:
                return False
            lag_ns = node.current_sim_stamp_ns - snapshot_stamp_ns
            if lag_ns > self.manifest.contact_snapshot_max_clock_lag_ns:
                raise ProtocolError('qualified release snapshot exceeded the 220 ms /clock bracket')
            return True

        self._wait_for(released, reason='full 0.25 s contact-release gap was not observed')
        assert node.qualified_release_snapshot is not None
        self.release_complete_stamp_ns = node.qualified_release_snapshot['sim_stamp_ns']
        self.release_observed_clock_stamp_ns = node.current_sim_stamp_ns
        self.contact_clock_bracket = {
            'clock_stamp_ns': node.current_sim_stamp_ns,
            'lag_ns': node.current_sim_stamp_ns - self.release_complete_stamp_ns,
            'limit_ns': self.manifest.contact_snapshot_max_clock_lag_ns,
            'snapshot_stamp_ns': self.release_complete_stamp_ns,
        }
        node.phase = 'DONE'

    def _release_boundary_ns(self) -> int | None:
        return self._node.release_required_through_stamp_ns

    def _run_control(self) -> None:
        self._drive_until_contact()
        self._settle_contact_stop_latency()
        self._hold_zero()
        self._reverse_and_release()
        node = self._node
        if node.exact_pair_snapshot_record_count < 1:
            raise ScenarioFailureError('expected rendered chassis-wall pair was not observed')
        episodes = node.closed_episodes()
        if len(episodes) != 1 or node.active_counterpart_count() != 0:
            raise ScenarioFailureError('positive control requires exactly one total episode')
        if episodes[0]['counterpart_model'] != WALL_NAME:
            raise ScenarioFailureError('positive-control episode counterpart is not the wall')
        episode_pairs = {tuple(pair) for pair in episodes[0]['normalized_pairs']}
        if node.manifest.expected_control_pair not in episode_pairs:
            raise ScenarioFailureError('closed wall episode does not contain the expected pair')
        if not node.commands.items or node.commands.items[-1]['linear_x'] != 0.0:
            raise ScenarioFailureError('positive control did not retain a final zero command')

    def _cleanup_wall(self) -> None:
        node = self._node
        if not self.wall_spawn_request_sent:
            return
        if self.delete_attempt_count != 0:
            raise ProtocolError('contact-control wall delete may be attempted exactly once')
        source_publishers_before = node.count_publishers(ENTITY_POSE_TOPIC)
        self.delete_attempt_count = 1
        self.cleanup = {
            'actor_absent': False,
            'delete_attempt_count': 1,
            'delete_success': None,
            'proof': {'kind': 'delete_request_pending'},
            'required': True,
        }
        request = DeleteEntity.Request()
        request.entity.name = WALL_NAME
        request.entity.type = request.entity.MODEL
        response, request_sequence, request_stamp_ns = self._call_service(
            node.delete_client,
            request,
            name=DELETE_SERVICE,
            check_fatal=False,
        )
        success = bool(response.success)
        if not success:
            self.cleanup = {
                'actor_absent': False,
                'delete_attempt_count': 1,
                'delete_success': False,
                'proof': {
                    'kind': 'delete_response_did_not_prove_absence',
                    'request_sequence': request_sequence,
                    'request_stamp_ns': request_stamp_ns,
                },
                'required': True,
            }
            raise InfrastructureError(f'{DELETE_SERVICE} rejected contact wall cleanup')
        response_sequence = node._next_sequence()
        response_stamp_ns = node.current_sim_stamp_ns
        node.delete_response_stamp_ns = response_stamp_ns
        node.post_delete_entity_message_count = 0
        node.post_delete_entity_latest_sim_stamp_ns = None
        node.post_delete_wall_pose_count = 0
        quiet_until_ns = response_stamp_ns + ACTOR_CLEANUP_QUIET_NS
        self.cleanup = {
            'actor_absent': False,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': {
                'kind': 'successful_delete_response_cleanup_quiet_pending',
                'pose_source_publishers_before': source_publishers_before,
                'post_delete_pose_count': 0,
                'post_delete_pose_source_heartbeat_count': 0,
                'post_delete_pose_source_latest_sim_stamp_ns': None,
                'quiet_until_sim_stamp_ns': quiet_until_ns,
                'request_sequence': request_sequence,
                'request_stamp_ns': request_stamp_ns,
                'response_sequence': response_sequence,
                'response_stamp_ns': response_stamp_ns,
            },
            'required': True,
        }
        self._wait_for(
            lambda: (
                node.current_sim_stamp_ns >= quiet_until_ns
                and node.post_delete_entity_latest_sim_stamp_ns is not None
                and node.post_delete_entity_latest_sim_stamp_ns >= quiet_until_ns
            ),
            reason='entity-pose source did not span the wall cleanup quiet interval',
            sim_deadline_ns=quiet_until_ns + ACTOR_CLEANUP_QUIET_NS,
            check_fatal=False,
        )
        source_publishers_after = node.count_publishers(ENTITY_POSE_TOPIC)
        actor_absent = (
            node.post_delete_wall_pose_count == 0
            and node.post_delete_entity_message_count >= 1
            and source_publishers_after >= 1
        )
        self.cleanup = {
            'actor_absent': actor_absent,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': {
                'kind': 'successful_delete_response_and_pose_quiet_interval',
                'pose_source_publishers_after': source_publishers_after,
                'pose_source_publishers_before': source_publishers_before,
                'post_delete_pose_count': node.post_delete_wall_pose_count,
                'post_delete_pose_source_heartbeat_count': (node.post_delete_entity_message_count),
                'post_delete_pose_source_latest_sim_stamp_ns': (
                    node.post_delete_entity_latest_sim_stamp_ns
                ),
                'quiet_until_sim_stamp_ns': quiet_until_ns,
                'request_sequence': request_sequence,
                'request_stamp_ns': request_stamp_ns,
                'response_sequence': response_sequence,
                'response_stamp_ns': response_stamp_ns,
            },
            'required': True,
        }
        if not actor_absent:
            raise ScenarioFailureError('wall pose stream continued after successful delete')

    def _best_effort_zero(self) -> None:
        if self.node is None or not self.node.clock_seen:
            return
        try:
            if not self.node.commands.items or self.node.commands.items[-1]['linear_x'] != 0.0:
                self.node.publish_command(0.0, phase='FAIL_SAFE_ZERO')
        except RobotestScenarioError:
            pass

    def _criteria(self) -> dict[str, bool]:
        node = self._node
        episodes = node.closed_episodes()
        episode_pairs = (
            {tuple(pair) for pair in episodes[0]['normalized_pairs']}
            if len(episodes) == 1
            else set()
        )
        quality = node.quality(
            publisher_count=self.last_publisher_count,
            forbidden_nodes=self.forbidden_nodes,
            contact_graph_topology=self._contact_graph_evidence(),
        )
        release_boundary_ns = self._release_boundary_ns()
        topology = quality['contact_graph_topology']
        topology_complete = (
            topology['audit_count'] >= 2
            and topology['first_snapshot'] is not None
            and topology['last_snapshot'] is not None
            and topology['first_sha256']
            == hashlib.sha256(canonical_json_bytes(topology['first_snapshot'])).hexdigest()
            and topology['last_sha256']
            == hashlib.sha256(canonical_json_bytes(topology['last_snapshot'])).hexdigest()
            and self._contact_graph_is_exact(topology['first_snapshot'])
            and self._contact_graph_is_exact(topology['last_snapshot'])
            and {
                key: topology['first_snapshot'][key][0]['endpoint_gid']
                for key in (
                    'private_raw_publishers',
                    'private_raw_subscribers',
                    'public_snapshot_publishers',
                )
            }
            == {
                key: topology['last_snapshot'][key][0]['endpoint_gid']
                for key in (
                    'private_raw_publishers',
                    'private_raw_subscribers',
                    'public_snapshot_publishers',
                )
            }
        )
        return {
            'contact_before_deadline': (
                node.first_qualifying_contact is not None
                and node.control_started_stamp_ns is not None
                and int(node.first_qualifying_contact['sim_stamp_ns'])
                <= node.control_started_stamp_ns + CONTROL_CONTACT_DEADLINE_NS
            ),
            'episode_reconciled': len(episodes) == 1
            and episodes[0]['counterpart_model'] == WALL_NAME
            and node.manifest.expected_control_pair in episode_pairs,
            'exactly_one_counterpart_episode': len(episodes) == 1
            and node.active_counterpart_count() == 0,
            'final_command_zero': bool(node.commands.items)
            and node.commands.items[-1]['linear_x'] == 0.0
            and node.commands.items[-1]['angular_z'] == 0.0,
            'graph_isolated': quality['nav2_absent']
            and quality['collision_monitor_absent']
            and quality['source_streams_live']
            and topology_complete,
            'hold_completed': (
                self.hold_complete_stamp_ns is not None
                and node.stop_latency_clock_stamp_ns is not None
                and self.hold_complete_stamp_ns - node.stop_latency_clock_stamp_ns
                >= CONTROL_HOLD_NS
            ),
            'overflow_free': bool(quality['overflow_free']),
            'expected_contact_snapshot_observed': node.exact_pair_snapshot_record_count >= 1,
            'release_completed': (
                self.release_complete_stamp_ns is not None
                and node.qualified_release_snapshot is not None
                and self.release_complete_stamp_ns
                == node.qualified_release_snapshot['sim_stamp_ns']
                and self.contact_clock_bracket is not None
                and 0
                <= self.contact_clock_bracket['lag_ns']
                <= self.manifest.contact_snapshot_max_clock_lag_ns
            ),
            'release_source_spanned': (
                self.release_contact_snapshot_start_count is not None
                and node.contact_snapshot_count > self.release_contact_snapshot_start_count
                and release_boundary_ns is not None
                and node.qualified_release_snapshot is not None
                and node.qualified_release_snapshot['sim_stamp_ns'] > release_boundary_ns
            ),
            'robot_start_verified': self.observed_start is not None,
            'reverse_completed': (
                self.reverse_start_stamp_ns is not None
                and self.final_zero_stamp_ns is not None
                and self.final_zero_stamp_ns - self.reverse_start_stamp_ns >= CONTROL_REVERSE_NS
            ),
            'sole_cmd_vel_publisher': bool(quality['sole_cmd_vel_publisher']),
            'stop_within_100ms': (
                node.stop_latency_ns is not None
                and 0 <= node.stop_latency_ns <= CONTROL_STOP_DEADLINE_NS
            ),
            'wall_deleted': bool(self.cleanup['actor_absent'])
            and self.cleanup['delete_success'] is True,
        }

    def _control_evidence(self) -> dict[str, Any]:
        node = self._node
        return {
            'command_trace': list(node.commands.items),
            'observed_robot_start': self.observed_start,
            'setup': self.setup_evidence,
            'contact': {
                'active_counterpart_count': node.active_counterpart_count(),
                'classified_record_count': node.classified_contact_record_count,
                'contact_clock_bracket': self.contact_clock_bracket,
                'counterpart_tracker_count': len(node.counterpart_trackers),
                'episodes': node.closed_episodes(),
                'exact_pair_snapshot_record_count': node.exact_pair_snapshot_record_count,
                'expected_pair': list(node.manifest.expected_control_pair),
                'first_qualifying_contact': node.first_qualifying_contact,
                'snapshot_contact_record_count': node.snapshot_contact_record_count,
                'snapshot_records': list(node.contact_snapshot_records.items),
                'release_snapshot': node.qualified_release_snapshot,
                'snapshots': list(node.contact_snapshots.items),
            },
            'criteria': self._criteria(),
            'metrics_owned': {
                'collector_reconciliation_and_process_group_termination': {
                    'owner': 'robotest_metrics_and_orchestrator',
                    'required': True,
                    'scenario_controller_value': None,
                }
            },
            'timeline': {
                'control_started_stamp_ns': node.control_started_stamp_ns,
                'final_zero_stamp_ns': self.final_zero_stamp_ns,
                'hold_complete_stamp_ns': self.hold_complete_stamp_ns,
                'release_complete_stamp_ns': self.release_complete_stamp_ns,
                'release_observed_clock_stamp_ns': self.release_observed_clock_stamp_ns,
                'release_qualified_snapshot_stamp_ns': self.release_complete_stamp_ns,
                'release_contact_snapshot_start_count': (self.release_contact_snapshot_start_count),
                'release_contact_snapshot_end_count': node.contact_snapshot_count,
                'release_required_through_stamp_ns': self._release_boundary_ns(),
                'reverse_start_stamp_ns': self.reverse_start_stamp_ns,
                'stop_command_stamp_ns': node.stop_command_stamp_ns,
                'stop_latency_clock_stamp_ns': node.stop_latency_clock_stamp_ns,
                'stop_latency_ns': node.stop_latency_ns,
            },
        }

    def _configuration(self) -> dict[str, Any]:
        return {
            'control_configuration': contact_control_configuration(),
            'control_configuration_sha256': contact_control_configuration_sha256(),
            'coverage_manifest_path': str(self.manifest.path),
            'coverage_manifest_provenance': self.manifest.provenance(),
            'coverage_manifest_sha256': self.manifest.sha256,
            'expected_pair': list(self.manifest.expected_control_pair),
            'fixture': self._fixture_document(),
            'fixture_sha256': self._fixture_sha256(),
            'service_timeout_s': self.service_timeout_s,
            'source_binding': contact_source_binding(),
            'wall_asset_sha256': self.wall_asset_sha256,
            'wall_timeout_s': self.wall_timeout_s,
        }

    def _result(self, error: RobotestScenarioError | None) -> tuple[dict[str, Any], int]:
        self._enforce_graph_isolation()
        criteria = self._criteria()
        if error is None and not all(criteria.values()):
            error = ScenarioFailureError('positive-control frozen criteria did not all pass')
        if error is None:
            exit_code = int(ExitCode.COMPLETE)
            status = 'PASS'
            reason = (
                'positive-control mechanics complete; suite qualification remains owned by '
                'the metrics compositor'
            )
        else:
            exit_code = int(error.exit_code)
            status = 'FAIL'
            reason = bounded_diagnostic(error)
        result = {
            'cleanup': self.cleanup,
            'configuration': self._configuration(),
            'control': self._control_evidence(),
            'identity': {
                'fixture_id': FIXTURE_ID,
                'run_id': self.run_id,
                'scenario_sha256': self._fixture_sha256(),
            },
            'producer': 'robotest_scenarios/contact_control_driver',
            'quality': self._node.quality(
                publisher_count=self.last_publisher_count,
                forbidden_nodes=self.forbidden_nodes,
                contact_graph_topology=self._contact_graph_evidence(),
            ),
            'schema_version': 1,
            'status': status,
            'verdict': {
                'authority': 'component_only',
                'benchmark_pass': None,
                'exit_code': exit_code,
                'reason': reason,
            },
        }
        return result, exit_code

    def run(self) -> int:
        """Run, fail-safe, clean up, write evidence, and return a stable exit code."""
        error: RobotestScenarioError | None = None
        initialized = False
        try:
            rclpy.init(args=list(self.raw_ros_args))
            initialized = True
            self.node = ContactControlNode(self.manifest)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self._prepare()
            self._run_control()
        except RobotestScenarioError as exc:
            error = exc
        except Exception as exc:
            error = InfrastructureError(f'unhandled contact-control failure: {exc}')
        finally:
            if self.node is not None and self.executor is not None:
                self.node.begin_cleanup()
                self._best_effort_zero()
                try:
                    self._cleanup_wall()
                except RobotestScenarioError as cleanup_error:
                    if error is None:
                        error = cleanup_error
                try:
                    result, exit_code = self._result(error)
                except RobotestScenarioError as result_error:
                    result, exit_code = self._result_without_graph(result_error)
                try:
                    schema = load_schema(package_schema_path('contact-control-result.schema.json'))
                    write_canonical_json(self.output_path, result, schema=schema)
                except ArtifactError as artifact_error:
                    print(
                        f'contact_control_driver: artifact error: {artifact_error}',
                        file=sys.stderr,
                    )
                    exit_code = int(ExitCode.ARTIFACT_ERROR)
                self.executor.remove_node(self.node)
                self.executor.shutdown(timeout_sec=1.0)
                self.node.destroy_node()
            else:
                exit_code = (
                    int(error.exit_code)
                    if error is not None
                    else int(ExitCode.INFRASTRUCTURE_ERROR)
                )
            if initialized:
                rclpy.try_shutdown()
        return exit_code

    def _result_without_graph(self, error: RobotestScenarioError) -> tuple[dict[str, Any], int]:
        """Finalize a failure artifact when graph discovery itself failed."""
        exit_code = int(error.exit_code)
        return (
            {
                'cleanup': self.cleanup,
                'configuration': self._configuration(),
                'control': self._control_evidence(),
                'identity': {
                    'fixture_id': FIXTURE_ID,
                    'run_id': self.run_id,
                    'scenario_sha256': self._fixture_sha256(),
                },
                'producer': 'robotest_scenarios/contact_control_driver',
                'quality': self._node.quality(
                    publisher_count=self.last_publisher_count,
                    forbidden_nodes=self.forbidden_nodes,
                    contact_graph_topology=self._contact_graph_evidence(),
                ),
                'schema_version': 1,
                'status': 'FAIL',
                'verdict': {
                    'authority': 'component_only',
                    'benchmark_pass': None,
                    'exit_code': exit_code,
                    'reason': bounded_diagnostic(error),
                },
            },
            exit_code,
        )


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ValidationError(message)


def _parser() -> argparse.ArgumentParser:
    parser = _ArgumentParser(
        prog='contact_control_driver',
        description='Run the bounded ADR 0006 collision positive-control fixture.',
    )
    parser.add_argument('--output', required=True, help='new positive-control JSON path')
    parser.add_argument('--ready-file', required=True, help='new atomic readiness JSON path')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--coverage-manifest', required=True)
    parser.add_argument('--wall-timeout-s', type=float, default=DEFAULT_CONTROL_WALL_TIMEOUT_S)
    parser.add_argument('--service-timeout-s', type=float, default=DEFAULT_SERVICE_TIMEOUT_S)
    return parser


def _positive_bounded(value: float, *, label: str, maximum: float) -> float:
    if not math.isfinite(value) or value <= 0.0 or value > maximum:
        raise ValidationError(f'{label} must be finite and in (0, {maximum}]')
    return value


def _inputs(
    raw_args: Sequence[str],
) -> tuple[CoverageManifest, Path, Path, str, float, float]:
    cli = _parser().parse_args(remove_ros_args(args=list(raw_args))[1:])
    manifest = load_coverage_manifest(cli.coverage_manifest)
    run_id = validate_run_id(cli.run_id)
    output = validate_new_output(cli.output, label='contact-control output')
    ready = validate_new_output(cli.ready_file, label='contact-control ready file')
    reserved_paths = {output, sha256_sidecar(output), ready}
    if len(reserved_paths) != 3:
        raise ValidationError(
            'contact-control output, checksum sidecar, and ready file must be distinct'
        )
    wall_timeout = float(cli.wall_timeout_s)
    if wall_timeout != DEFAULT_CONTROL_WALL_TIMEOUT_S:
        raise ValidationError(
            'positive-control wall timeout must remain frozen at '
            f'{DEFAULT_CONTROL_WALL_TIMEOUT_S:.1f} s'
        )
    service_timeout = _positive_bounded(
        cli.service_timeout_s,
        label='service timeout',
        maximum=30.0,
    )
    return manifest, output, ready, run_id, wall_timeout, service_timeout


def main(args: Sequence[str] | None = None) -> int:
    """Run the installed contact-control driver and return its stable exit code."""
    raw_args = list(sys.argv if args is None else ['contact_control_driver', *args])
    try:
        manifest, output, ready, run_id, wall_timeout, service_timeout = _inputs(raw_args)
    except (OSError, RuntimeError, ValidationError) as exc:
        print(f'contact_control_driver: validation error: {exc}', file=sys.stderr)
        return int(ExitCode.VALIDATION_ERROR)
    return ContactControlApp(
        manifest=manifest,
        output_path=output,
        ready_path=ready,
        run_id=run_id,
        wall_timeout_s=wall_timeout,
        service_timeout_s=service_timeout,
        raw_ros_args=raw_args,
    ).run()
