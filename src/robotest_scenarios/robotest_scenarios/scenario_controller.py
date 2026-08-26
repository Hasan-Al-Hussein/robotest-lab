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

"""Bounded Phase 3 scenario actor controller."""

from __future__ import annotations

import argparse
import math
import sys
import time
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMessage
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_action_status_default,
)
from rclpy.utilities import remove_ros_args
from ros_gz_interfaces.srv import DeleteEntity, SetEntityPose, SpawnEntity
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

from robotest_scenarios.artifacts import (
    bounded_diagnostic,
    load_schema,
    sha256_sidecar,
    validate_candidate_id,
    validate_new_output,
    validate_run_id,
    write_canonical_json,
)
from robotest_scenarios.bounded import PrefixBuffer
from robotest_scenarios.constants import (
    ACTION_CANCEL_SERVICE,
    ACTION_FEEDBACK_TOPIC,
    ACTION_STATUS_TOPIC,
    ACTOR_CLEANUP_QUIET_NS,
    ACTOR_INITIAL_POSITION_TOLERANCE_M,
    ACTOR_OBSERVATION_LATENCY_NS,
    ACTOR_POSITION_TOLERANCE_M,
    ACTOR_STATE_CAPACITY,
    ACTOR_YAW_TOLERANCE_RAD,
    CLOCK_TOPIC,
    DDS_DRAIN_GRACE_S,
    DEFAULT_SERVICE_TIMEOUT_S,
    DELETE_SERVICE,
    ENTITY_POSE_HEARTBEAT_MODEL,
    ENTITY_POSE_TOPIC,
    FEEDBACK_CAPACITY,
    GROUND_TRUTH_ALIGNMENT_NS,
    GROUND_TRUTH_CAPACITY,
    GROUND_TRUTH_TOPIC,
    PLAN_CAPACITY,
    PLAN_POSE_CAPACITY,
    PLAN_TOPIC,
    S2_CLEARANCE_M,
    S2_EXCLUSION_RECT,
    S2_OBSERVATION_DEADLINE_OFFSET_NS,
    S2_TRIGGER_OFFSET_NS,
    S3_TARGET_COUNT,
    SET_POSE_SERVICE,
    SPAWN_SERVICE,
    STATUS_CAPACITY,
    ExitCode,
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
from robotest_scenarios.geometry import (
    PlanEvidence,
    plan_avoids_rectangle,
    quaternion_yaw,
    scenario3_target,
    shortest_yaw_error,
    stamp_to_ns,
    trajectory_sha256,
    validate_plan,
)
from robotest_scenarios.models import (
    PoseTarget,
    ScenarioDocument,
    load_scenario,
    package_schema_path,
)
from robotest_scenarios.provenance import (
    configuration_sha256,
    controller_configuration,
    file_sha256,
    source_binding,
)

FeedbackMessage = FollowWaypoints.Impl.FeedbackMessage

_TERMINAL_STATUSES = {
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
}
_ACTIVE_STATUSES = {
    GoalStatus.STATUS_ACCEPTED,
    GoalStatus.STATUS_EXECUTING,
    GoalStatus.STATUS_CANCELING,
}
_LEGAL_STATUS_SUCCESSORS = {
    GoalStatus.STATUS_ACCEPTED: _ACTIVE_STATUSES | _TERMINAL_STATUSES,
    GoalStatus.STATUS_EXECUTING: {
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }
    | _TERMINAL_STATUSES,
    GoalStatus.STATUS_CANCELING: {GoalStatus.STATUS_CANCELING} | _TERMINAL_STATUSES,
    GoalStatus.STATUS_SUCCEEDED: {GoalStatus.STATUS_SUCCEEDED},
    GoalStatus.STATUS_CANCELED: {GoalStatus.STATUS_CANCELED},
    GoalStatus.STATUS_ABORTED: {GoalStatus.STATUS_ABORTED},
}


@dataclass(frozen=True, slots=True)
class ActorPoseEvidence:
    """One observed Gazebo actor pose."""

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


def _uuid_fields(value: object) -> tuple[bytes, str]:
    try:
        raw = bytes(value)
    except (TypeError, ValueError) as exc:
        raise ProtocolError('goal UUID is not a byte sequence') from exc
    if len(raw) != 16 or not any(raw):
        raise ProtocolError('goal UUID must contain 16 nonzero identity bytes')
    return raw, str(uuid.UUID(bytes=raw))


def _pose_message(target: PoseTarget | dict[str, Any]) -> Pose:
    if isinstance(target, PoseTarget):
        x_value, y_value, z_value, yaw_value = target.x, target.y, target.z, target.yaw
    else:
        x_value = float(target['x'])
        y_value = float(target['y'])
        z_value = float(target['z'])
        yaw_value = float(target['yaw'])
    pose = Pose()
    pose.position.x = x_value
    pose.position.y = y_value
    pose.position.z = z_value
    pose.orientation.z = math.sin(yaw_value / 2.0)
    pose.orientation.w = math.cos(yaw_value / 2.0)
    return pose


class ScenarioControllerNode(Node):
    """ROS graph adapter with bounded callback evidence."""

    def __init__(self, document: ScenarioDocument) -> None:
        super().__init__(
            'scenario_controller',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )
        self.document = document
        self.sequence = 0
        self.current_sim_stamp_ns = 0
        self.clock_seen = False
        self.clock_first_stamp_ns: int | None = None
        self.clock_sample_count = 0
        self.clock_max_gap_ns = 0
        self.clock_regression_count = 0
        self.status_message_seen = False
        self.ready = False
        self.ready_sim_stamp_ns: int | None = None
        self.ready_steady_ns: int | None = None
        self.baseline_uuids: set[bytes] = set()
        self.last_status_by_uuid: dict[bytes, tuple[int, int]] = {}
        self.bound_uuid: bytes | None = None
        self.bound_uuid_text: str | None = None
        self.accepted_goal_stamp_ns: int | None = None
        self.terminal_status: int | None = None
        self.terminal_observed_stamp_ns: int | None = None
        self.terminal_observed_sequence: int | None = None
        self.feedback_current_index: int | None = None
        self.last_feedback_stamp_ns: int | None = None
        self.feedback_indices_seen: set[int] = set()
        self.feedback_anchor_stamp_ns: int | None = None
        self.processed_feedback_sequences: set[int] = set()
        self.initial_plan: dict[str, Any] | None = None
        self.replan: dict[str, Any] | None = None
        self.plan_pose_ingress = 0
        self.plan_pose_accepted_count = 0
        self.plan_pose_invalid_count = 0
        self.plan_pose_overflow = False
        self.plan_pose_overflow_count = 0
        self.plan_pose_first_overflow_sequence: int | None = None
        self.plan_pose_first_overflow_stamp_ns: int | None = None
        self.latest_ground_truth: dict[str, Any] | None = None
        self.last_ground_truth_stamp_ns: int | None = None
        self.spawn_request_sequence: int | None = None
        self.spawn_request_stamp_ns: int | None = None
        self.spawn_response_sequence: int | None = None
        self.spawn_response_stamp_ns: int | None = None
        self.actor_poses: list[ActorPoseEvidence] = []
        self.last_actor_pose: ActorPoseEvidence | None = None
        self.last_actor_stamp_ns: int | None = None
        self.trajectory_active = False
        self.s3_observation_requests: dict[int, tuple[int, int]] = {}
        self.matched_s3_actor_poses: dict[int, ActorPoseEvidence] = {}
        self.delete_response_stamp_ns: int | None = None
        self.post_delete_pose_count = 0
        self.entity_pose_message_count = 0
        self.last_entity_pose_heartbeat_stamp_ns: int | None = None
        self.post_delete_entity_message_count = 0
        self.post_delete_entity_latest_sim_stamp_ns: int | None = None
        self.first_spawn_observation: ActorPoseEvidence | None = None
        self.fatal_error: RobotestScenarioError | None = None
        self.cleanup_mode = False
        self.protocol_error_count = 0
        self.status = PrefixBuffer[dict[str, Any]]('status', STATUS_CAPACITY)
        self.feedback = PrefixBuffer[dict[str, Any]]('feedback', FEEDBACK_CAPACITY)
        self.plans = PrefixBuffer[dict[str, Any]]('plans', PLAN_CAPACITY)
        self.ground_truth = PrefixBuffer[dict[str, Any]]('ground_truth', GROUND_TRUTH_CAPACITY)
        self.actor_state = PrefixBuffer[dict[str, Any]]('actor_state', ACTOR_STATE_CAPACITY)

        reliable_10 = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_5 = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        feedback_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
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
        self.status_subscription = self.create_subscription(
            GoalStatusArray,
            ACTION_STATUS_TOPIC,
            self._guard(self._on_status),
            qos_profile_action_status_default,
        )
        self.feedback_subscription = self.create_subscription(
            FeedbackMessage,
            ACTION_FEEDBACK_TOPIC,
            self._guard(self._on_feedback),
            feedback_qos,
        )
        self.plan_subscription = self.create_subscription(
            PathMessage, PLAN_TOPIC, self._guard(self._on_plan), reliable_5
        )
        self.ground_truth_subscription = self.create_subscription(
            Odometry,
            GROUND_TRUTH_TOPIC,
            self._guard(self._on_ground_truth),
            reliable_10,
        )
        self.entity_pose_subscription = self.create_subscription(
            TFMessage,
            ENTITY_POSE_TOPIC,
            self._guard(self._on_entity_poses, allow_during_cleanup=True),
            reliable_10,
        )
        self.spawn_client = self.create_client(SpawnEntity, SPAWN_SERVICE)
        self.set_pose_client = self.create_client(SetEntityPose, SET_POSE_SERVICE)
        self.delete_client = self.create_client(DeleteEntity, DELETE_SERVICE)
        self.cancel_client = self.create_client(CancelGoal, ACTION_CANCEL_SERVICE)

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
                self.set_fatal(ProtocolError(f'unhandled callback error: {exc}'))

        return guarded

    def begin_cleanup(self) -> None:
        """Keep only clock and actor-pose evidence alive after a latched fatal."""
        self.cleanup_mode = True
        self.trajectory_active = False

    def set_fatal(self, error: RobotestScenarioError) -> None:
        """Latch the first failure without throwing through the executor."""
        if self.fatal_error is None:
            self.fatal_error = error
            if isinstance(error, ProtocolError):
                self.protocol_error_count += 1

    def mark_ready(self) -> None:
        """Freeze the readiness boundary after pre-goal checks."""
        if not self.clock_seen or self.current_sim_stamp_ns <= 0:
            raise InfrastructureError('cannot mark ready before a positive /clock sample')
        active = [
            raw_uuid
            for raw_uuid, (_, status_code) in self.last_status_by_uuid.items()
            if status_code in _ACTIVE_STATUSES
        ]
        if active:
            raise InfrastructureError('a FollowWaypoints goal was active before readiness')
        self.ready = True
        self.ready_sim_stamp_ns = self.current_sim_stamp_ns
        self.ready_steady_ns = time.monotonic_ns()

    def _on_clock(self, message: Clock) -> None:
        stamp_ns = stamp_to_ns(message.clock)
        if self.clock_seen and stamp_ns < self.current_sim_stamp_ns:
            self.clock_regression_count += 1
            raise ProtocolError('simulation clock regressed')
        if self.clock_first_stamp_ns is None:
            self.clock_first_stamp_ns = stamp_ns
        elif stamp_ns > self.current_sim_stamp_ns:
            self.clock_max_gap_ns = max(self.clock_max_gap_ns, stamp_ns - self.current_sim_stamp_ns)
        self.clock_sample_count += 1
        self.current_sim_stamp_ns = stamp_ns
        self.clock_seen = True

    def _callback_sim_stamp(self, *, label: str, allow_zero: bool = False) -> int:
        stamp_ns = int(self.get_clock().now().nanoseconds)
        if stamp_ns <= 0 and self.clock_seen and self.current_sim_stamp_ns > 0:
            stamp_ns = self.current_sim_stamp_ns
        if stamp_ns <= 0 and not allow_zero:
            raise ProtocolError(f'{label} callback has no positive ROS simulation time')
        return stamp_ns

    def _on_status(self, message: GoalStatusArray) -> None:
        self.status_message_seen = True
        observed_stamp_ns = self._callback_sim_stamp(label='status', allow_zero=not self.ready)
        for item in message.status_list:
            transition_sequence: int | None = None
            try:
                raw_uuid, uuid_text = _uuid_fields(item.goal_info.goal_id.uuid)
                accepted_stamp_ns = stamp_to_ns(item.goal_info.stamp, positive=True)
                status_code = int(item.status)
                if status_code not in range(1, 7):
                    raise ProtocolError('action status code must be in 1..6')
            except (AttributeError, TypeError, ValueError, ProtocolError):
                self.status.reject_invalid()
                raise
            previous = self.last_status_by_uuid.get(raw_uuid)
            if previous is not None and previous[0] != accepted_stamp_ns:
                self.status.reject_invalid()
                raise ProtocolError('accepted-goal T0 changed for an observed UUID')
            if (
                raw_uuid == self.bound_uuid
                and previous is not None
                and status_code not in _LEGAL_STATUS_SUCCESSORS[previous[1]]
            ):
                self.status.reject_invalid()
                raise ProtocolError('bound FollowWaypoints status transition is impossible')
            self.last_status_by_uuid[raw_uuid] = (accepted_stamp_ns, status_code)
            if previous != (accepted_stamp_ns, status_code):
                sequence = self._next_sequence()
                transition_sequence = sequence
                self.status.add(
                    {
                        'accepted_goal_stamp_ns': accepted_stamp_ns,
                        'collector_sequence': sequence,
                        'observed_sim_stamp_ns': observed_stamp_ns,
                        'status': status_code,
                        'uuid': uuid_text,
                    },
                    sequence=sequence,
                    stamp_ns=observed_stamp_ns,
                )
            else:
                self.status.observe_unretained()
            if not self.ready:
                self.baseline_uuids.add(raw_uuid)
                continue
            assert self.ready_sim_stamp_ns is not None
            if raw_uuid in self.baseline_uuids or accepted_stamp_ns < self.ready_sim_stamp_ns:
                self.baseline_uuids.add(raw_uuid)
                continue
            if self.bound_uuid is None:
                if status_code in _TERMINAL_STATUSES:
                    raise ProtocolError('new goal was first observed only after it became terminal')
                self.bound_uuid = raw_uuid
                self.bound_uuid_text = uuid_text
                self.accepted_goal_stamp_ns = accepted_stamp_ns
                self._process_pending_feedback()
                self._process_pending_plans()
            elif raw_uuid != self.bound_uuid:
                raise ProtocolError(
                    'more than one post-readiness FollowWaypoints goal was observed'
                )
            elif accepted_stamp_ns != self.accepted_goal_stamp_ns:
                raise ProtocolError('bound accepted-goal T0 is not immutable')
            if (
                raw_uuid == self.bound_uuid
                and status_code in _TERMINAL_STATUSES
                and self.terminal_status is None
            ):
                self.terminal_status = status_code
                self.terminal_observed_stamp_ns = observed_stamp_ns
                self.terminal_observed_sequence = transition_sequence

    def _on_feedback(self, message: Any) -> None:
        try:
            observed_stamp_ns = self._callback_sim_stamp(
                label='feedback', allow_zero=not self.ready
            )
            raw_uuid, uuid_text = _uuid_fields(message.goal_id.uuid)
            current_index = int(message.feedback.current_waypoint)
        except (AttributeError, TypeError, ValueError, ProtocolError) as exc:
            self.feedback.reject_invalid()
            raise ProtocolError('FollowWaypoints feedback index is unavailable') from exc
        if not 0 <= current_index < len(self.document.waypoints):
            self.feedback.reject_invalid()
            raise ProtocolError('FollowWaypoints feedback index is out of range')
        if (
            self.last_feedback_stamp_ns is not None
            and observed_stamp_ns < self.last_feedback_stamp_ns
        ):
            self.feedback.reject_invalid()
            raise ProtocolError('FollowWaypoints feedback callback stamp regressed')
        self.last_feedback_stamp_ns = observed_stamp_ns
        if (
            self.bound_uuid is not None
            and raw_uuid != self.bound_uuid
            and raw_uuid not in self.baseline_uuids
        ):
            self.feedback.reject_invalid()
            raise ProtocolError('post-readiness feedback used a wrong goal UUID')
        sequence = self._next_sequence()
        record = {
            'collector_sequence': sequence,
            'current_waypoint': current_index,
            'observed_sim_stamp_ns': observed_stamp_ns,
            'uuid': uuid_text,
            '_uuid_bytes': raw_uuid,
        }
        self.feedback.add(
            record,
            sequence=sequence,
            stamp_ns=observed_stamp_ns,
        )
        if self.bound_uuid == raw_uuid:
            self._process_feedback(record)

    def _process_pending_feedback(self) -> None:
        if self.bound_uuid is None:
            return
        for record in tuple(self.feedback.items):
            if (
                record['_uuid_bytes'] != self.bound_uuid
                and record['_uuid_bytes'] not in self.baseline_uuids
                and self.ready_sim_stamp_ns is not None
                and int(record['observed_sim_stamp_ns']) >= self.ready_sim_stamp_ns
            ):
                self.feedback.invalidate_retained(record)
                raise ProtocolError('post-readiness feedback used a wrong goal UUID')
            if record['_uuid_bytes'] == self.bound_uuid:
                self._process_feedback(record)

    def _process_pending_plans(self) -> None:
        if self.document.scenario_id != 2 or self.accepted_goal_stamp_ns is None:
            return
        for record in self.plans.items:
            self._consider_scenario2_plan(record)

    def _process_feedback(self, record: dict[str, Any]) -> None:
        if (
            self.accepted_goal_stamp_ns is not None
            and int(record['observed_sim_stamp_ns']) < self.accepted_goal_stamp_ns
        ):
            self.feedback.invalidate_retained(record)
            raise ProtocolError('bound feedback was observed before authoritative T0')
        sequence = int(record['collector_sequence'])
        if sequence in self.processed_feedback_sequences:
            return
        self.processed_feedback_sequences.add(sequence)
        index = int(record['current_waypoint'])
        previous = self.feedback_current_index
        if previous is None:
            if index != 0:
                raise ProtocolError('bound feedback must begin at waypoint index 0')
        elif index < previous:
            raise ProtocolError('bound feedback index regressed')
        elif index > previous + 1:
            raise ProtocolError('bound feedback index skipped a waypoint')
        if previous == 1 and index == 2 and self.feedback_anchor_stamp_ns is None:
            stamp_ns = int(record['observed_sim_stamp_ns'])
            if stamp_ns <= 0:
                raise ProtocolError('Scenario 3 feedback anchor is not positive simulation time')
            self.feedback_anchor_stamp_ns = stamp_ns
        self.feedback_current_index = index
        self.feedback_indices_seen.add(index)

    def _on_plan(self, message: PathMessage) -> None:
        if self.document.scenario_id != 2 or self.feedback_current_index not in {None, 0}:
            return
        pose_count = len(message.poses)
        self.plan_pose_ingress += pose_count
        if self.plan_pose_ingress > PLAN_POSE_CAPACITY:
            self.plan_pose_overflow = True
            self.plan_pose_overflow_count += 1
            if self.plan_pose_first_overflow_sequence is None:
                self.plan_pose_first_overflow_sequence = self._next_sequence()
                self.plan_pose_first_overflow_stamp_ns = self.current_sim_stamp_ns
            raise ProtocolError('global plan pose capacity overflowed')
        try:
            stamp_ns = stamp_to_ns(message.header.stamp, positive=True)
            points = tuple(
                (float(pose.pose.position.x), float(pose.pose.position.y)) for pose in message.poses
            )
            waypoint = self.document.waypoints[0]
            evidence = validate_plan(
                message.header.frame_id,
                points,
                (waypoint.x, waypoint.y),
            )
        except ProtocolError:
            self.plans.reject_invalid()
            self.plan_pose_invalid_count += pose_count
            return
        self.plan_pose_accepted_count += pose_count
        sequence = self._next_sequence()
        record = {
            'collector_sequence': sequence,
            'observed_sim_stamp_ns': self.current_sim_stamp_ns,
            'plan': evidence,
            'sim_stamp_ns': stamp_ns,
        }
        self.plans.add(record, sequence=sequence, stamp_ns=stamp_ns)
        self._consider_scenario2_plan(record)

    def _consider_scenario2_plan(self, record: dict[str, Any]) -> None:
        if self.document.scenario_id != 2 or self.accepted_goal_stamp_ns is None:
            return
        stamp_ns = int(record['sim_stamp_ns'])
        observed_ns = int(record['observed_sim_stamp_ns'])
        evidence: PlanEvidence = record['plan']
        trigger_ns = self.accepted_goal_stamp_ns + S2_TRIGGER_OFFSET_NS
        if (
            self.initial_plan is None
            and stamp_ns >= self.accepted_goal_stamp_ns
            and observed_ns >= self.accepted_goal_stamp_ns
            and observed_ns < trigger_ns
            and (self.feedback_current_index is None or self.feedback_current_index == 0)
        ):
            self.initial_plan = self._plan_artifact(record)
            return
        if (
            self.spawn_request_sequence is None
            or int(record['collector_sequence']) <= self.spawn_request_sequence
            or self.spawn_request_stamp_ns is None
            or stamp_ns < self.spawn_request_stamp_ns
            or observed_ns < self.spawn_request_stamp_ns
            or self.initial_plan is None
            or evidence.geometry_sha256 == self.initial_plan['geometry_sha256']
            or self.replan is not None
            or self.feedback_current_index not in {None, 0}
        ):
            return
        artifact = self._plan_artifact(record)
        avoids = plan_avoids_rectangle(evidence.points, S2_EXCLUSION_RECT)
        artifact['all_segments_avoid'] = avoids
        self.replan = artifact
        if not avoids:
            raise ScenarioFailureError(
                'first changed Scenario 2 leg-0 plan intersects exclusion rectangle'
            )

    @staticmethod
    def _plan_artifact(record: dict[str, Any]) -> dict[str, Any]:
        evidence: PlanEvidence = record['plan']
        return {
            'collector_sequence': record['collector_sequence'],
            'endpoint_error_m': evidence.endpoint_error_m,
            'geometry_sha256': evidence.geometry_sha256,
            'length_m': evidence.length_m,
            'observed_sim_stamp_ns': record['observed_sim_stamp_ns'],
            'point_count': len(evidence.points),
            'sim_stamp_ns': record['sim_stamp_ns'],
        }

    def _on_ground_truth(self, message: Odometry) -> None:
        stamp_ns = stamp_to_ns(message.header.stamp, positive=True)
        if (
            self.last_ground_truth_stamp_ns is not None
            and stamp_ns < self.last_ground_truth_stamp_ns
        ):
            self.ground_truth.reject_invalid()
            raise ProtocolError('validation ground-truth stamp regressed')
        self.last_ground_truth_stamp_ns = stamp_ns
        x_value = float(message.pose.pose.position.x)
        y_value = float(message.pose.pose.position.y)
        if message.header.frame_id != 'world' or not all(
            math.isfinite(value) for value in (x_value, y_value)
        ):
            self.ground_truth.reject_invalid()
            return
        sequence = self._next_sequence()
        record = {
            'collector_sequence': sequence,
            'sim_stamp_ns': stamp_ns,
            'x': x_value,
            'y': y_value,
        }
        self.ground_truth.add(record, sequence=sequence, stamp_ns=stamp_ns)
        self.latest_ground_truth = record

    def _on_entity_poses(self, message: TFMessage) -> None:
        actor_name = self.document.actor_name
        if actor_name is None:
            return
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
            if transform.child_frame_id != actor_name:
                continue
            stamp_ns = stamp_to_ns(transform.header.stamp, positive=True)
            if self.last_actor_stamp_ns is not None and stamp_ns < self.last_actor_stamp_ns:
                self.actor_state.reject_invalid()
                raise ProtocolError('scenario actor pose stamp regressed')
            self.last_actor_stamp_ns = stamp_ns
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            values = (float(translation.x), float(translation.y), float(translation.z))
            if transform.header.frame_id not in {'robotest_lab', 'world'} or not all(
                math.isfinite(value) for value in values
            ):
                self.actor_state.reject_invalid()
                raise ProtocolError('actor pose stream contains invalid frame or position')
            try:
                yaw = quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w)
            except ProtocolError:
                self.actor_state.reject_invalid()
                raise
            sequence = self._next_sequence()
            evidence = ActorPoseEvidence(sequence, stamp_ns, *values, yaw)
            matching_index = next(
                (
                    index
                    for index, (request_sequence, target_stamp_ns) in sorted(
                        self.s3_observation_requests.items()
                    )
                    if index not in self.matched_s3_actor_poses
                    and sequence > request_sequence
                    and stamp_ns >= target_stamp_ns
                ),
                None,
            )
            if matching_index is not None:
                self.matched_s3_actor_poses[matching_index] = evidence
            previous = self.last_actor_pose
            changed = previous is None or any(
                abs(current - prior) > 1e-12
                for current, prior in zip(
                    (evidence.x, evidence.y, evidence.z, evidence.yaw),
                    (previous.x, previous.y, previous.z, previous.yaw),
                    strict=True,
                )
            )
            if changed or matching_index is not None:
                self.actor_state.add(evidence.as_dict(), sequence=sequence, stamp_ns=stamp_ns)
                self.actor_poses.append(evidence)
            else:
                self.actor_state.observe_unretained()
            self.last_actor_pose = evidence
            if (
                self.delete_response_stamp_ns is not None
                and stamp_ns > self.delete_response_stamp_ns
            ):
                self.post_delete_pose_count += 1
            if self.spawn_request_sequence is None:
                raise ProtocolError('scenario actor existed before its one-shot spawn request')
            if (
                self.first_spawn_observation is None
                and sequence > self.spawn_request_sequence
                and self.spawn_request_stamp_ns is not None
                and stamp_ns >= self.spawn_request_stamp_ns
            ):
                self.first_spawn_observation = evidence
            if (
                self.document.scenario_id == 3
                and self.ready
                and self.feedback_anchor_stamp_ns is None
            ):
                self._require_pose_near(
                    evidence,
                    self.document.actor_initial_pose,
                    ACTOR_INITIAL_POSITION_TOLERANCE_M,
                )

    @staticmethod
    def _require_pose_near(
        evidence: ActorPoseEvidence,
        target: PoseTarget | None,
        position_tolerance: float,
    ) -> None:
        if target is None:
            raise ProtocolError('actor target pose is unavailable')
        position_error = math.sqrt(
            (evidence.x - target.x) ** 2
            + (evidence.y - target.y) ** 2
            + (evidence.z - target.z) ** 2
        )
        yaw_error = shortest_yaw_error(evidence.yaw, target.yaw)
        if position_error > position_tolerance or yaw_error > ACTOR_YAW_TOLERANCE_RAD:
            raise ScenarioFailureError('Scenario 3 actor moved before its feedback anchor')

    def quality(self) -> dict[str, Any]:
        """Return bounded collector and binding quality fields."""
        buffers = {
            'actor_state': self.actor_state.quality(),
            'feedback': self.feedback.quality(),
            'ground_truth': self.ground_truth.quality(),
            'plans': self.plans.quality(),
            'status': self.status.quality(),
        }
        overflow_free = not self.plan_pose_overflow and all(
            not value['overflow'] for value in buffers.values()
        )
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
            'immutable_t0': self.bound_uuid is not None and self.accepted_goal_stamp_ns is not None,
            'overflow_free': overflow_free,
            'protocol_error_count': self.protocol_error_count,
            'relative_project_names': True,
            'single_goal_binding': self.bound_uuid is not None,
            'plan_pose_stream': {
                'capacity': PLAN_POSE_CAPACITY,
                'accepted_count': self.plan_pose_accepted_count,
                'first_overflow_sequence': self.plan_pose_first_overflow_sequence,
                'first_overflow_stamp_ns': self.plan_pose_first_overflow_stamp_ns,
                'ingress_count': self.plan_pose_ingress,
                'invalid_count': self.plan_pose_invalid_count,
                'overflow': self.plan_pose_overflow,
                'overflow_count': self.plan_pose_overflow_count,
                'retained_count': self.plan_pose_accepted_count,
            },
        }


class ScenarioControllerApp:
    """Run one bounded scenario state machine around the ROS node."""

    def __init__(
        self,
        *,
        document: ScenarioDocument,
        output_path: Path,
        ready_path: Path,
        identity: dict[str, Any],
        wall_timeout_s: float,
        service_timeout_s: float,
        raw_ros_args: Sequence[str],
    ) -> None:
        self.document = document
        self.output_path = output_path
        self.ready_path = ready_path
        self.identity = identity
        self.wall_timeout_s = wall_timeout_s
        self.service_timeout_s = service_timeout_s
        self.raw_ros_args = raw_ros_args
        self.node: ScenarioControllerNode | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.wall_deadline = time.monotonic() + wall_timeout_s
        self.actor_asset: Path | None = None
        self.actor_asset_sha256: str | None = None
        self.actor_spawn_request_sent = False
        self.actor_spawn_committed = False
        self.spawn_attempt_count = 0
        self.spawn_evidence: dict[str, Any] = {
            'attempt_count': 0,
            'error': None,
            'request_sequence': None,
            'request_stamp_ns': None,
            'response_sequence': None,
            'response_stamp_ns': None,
            'success': None,
        }
        self.s3_transactions: list[dict[str, Any]] = []
        self.s3_observation_drain_deadlines: dict[int, float] = {}
        self.set_attempt_count = 0
        self.set_success_count = 0
        self.delete_attempt_count = 0
        self.cancel_attempt_count = 0
        self.cancel_evidence: dict[str, Any] | None = None
        self.cleanup: dict[str, Any] = {
            'actor_absent': not document.has_actor,
            'delete_attempt_count': 0,
            'delete_success': None,
            'proof': {'kind': 'scenario_declares_no_actor'} if not document.has_actor else {},
            'required': False,
        }
        self.interaction = self._empty_interaction()

    def _empty_interaction(self) -> dict[str, Any]:
        if self.document.scenario_id == 2:
            return {'kind': 'static_obstacle', 'criteria': {}}
        if self.document.scenario_id == 3:
            return {
                'kind': 'pose_controlled_obstacle',
                'criteria': {},
                'metrics_owned': {
                    'authoritative_action_result_ordering': {
                        'owner': 'mission_runner_and_robotest_metrics',
                        'required': True,
                        'scenario_controller_value': None,
                    },
                    'collision_monitor_stop_final_command_correlation': {
                        'owner': 'robotest_metrics',
                        'required': True,
                        'scenario_controller_value': None,
                    },
                },
            }
        return {
            'kind': 'none',
            'criteria': {'bound_goal_terminal_succeeded': False},
        }

    @property
    def _node(self) -> ScenarioControllerNode:
        if self.node is None:
            raise InfrastructureError('scenario controller node is unavailable')
        return self.node

    @property
    def _executor(self) -> SingleThreadedExecutor:
        if self.executor is None:
            raise InfrastructureError('scenario controller executor is unavailable')
        return self.executor

    def _spin_once(self, timeout_s: float = 0.02, *, check_fatal: bool = True) -> None:
        if time.monotonic() >= self.wall_deadline:
            raise WallTimeoutError('scenario controller steady-wall escape timeout elapsed')
        remaining_s = max(0.0, self.wall_deadline - time.monotonic())
        self._executor.spin_once(timeout_sec=min(timeout_s, remaining_s))
        if check_fatal and self._node.fatal_error is not None:
            raise self._node.fatal_error

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
                and self._node.current_sim_stamp_ns > sim_deadline_ns
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
        evidence: dict[str, Any] | None = None,
    ) -> tuple[Any, int, int]:
        self._wait_service(client, name=name, check_fatal=check_fatal)
        request_sequence = self._node._next_sequence()
        request_stamp_ns = self._node.current_sim_stamp_ns
        if evidence is not None:
            evidence['request_sequence'] = request_sequence
            evidence['request_stamp_ns'] = request_stamp_ns
        try:
            future = client.call_async(request)
        except Exception as exc:
            if evidence is not None:
                evidence['error'] = bounded_diagnostic(exc)
            raise InfrastructureError(f'{name} request could not be sent: {exc}') from exc
        if name == SET_POSE_SERVICE and evidence is not None and 'index' in evidence:
            index = int(evidence['index'])
            if index in self._node.s3_observation_requests:
                raise ProtocolError(f'duplicate Scenario 3 observation request index {index}')
            self._node.s3_observation_requests[index] = (
                request_sequence,
                int(evidence['target_stamp_ns']),
            )
        deadline = min(self.wall_deadline, time.monotonic() + self.service_timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                if evidence is not None:
                    evidence['error'] = f'{name} response timed out'
                raise InfrastructureError(f'{name} response timed out')
            self._spin_once(check_fatal=check_fatal)
        try:
            response = future.result()
        except Exception as exc:
            if evidence is not None:
                evidence['error'] = bounded_diagnostic(exc)
            raise InfrastructureError(f'{name} response failed: {exc}') from exc
        if response is None:
            if evidence is not None:
                evidence['error'] = f'{name} returned no response'
            raise InfrastructureError(f'{name} returned no response')
        return response, request_sequence, request_stamp_ns

    def _resolve_actor_asset(self) -> None:
        if self.document.actor_asset_name is None:
            return
        try:
            share = Path(get_package_share_directory('robotest_sim'))
        except Exception as exc:
            raise InfrastructureError(f'cannot resolve robotest_sim share: {exc}') from exc
        asset = share / 'models' / self.document.actor_asset_name
        if not asset.is_file():
            raise InfrastructureError(f'installed actor asset is missing: {asset}')
        self.actor_asset = asset.resolve()
        self.actor_asset_sha256 = file_sha256(self.actor_asset)

    def _spawn_actor(self) -> dict[str, Any]:
        if self.actor_asset is None or self.document.actor_initial_pose is None:
            raise InfrastructureError('actor asset and initial pose must resolve before spawn')
        if self.spawn_attempt_count != 0:
            raise ProtocolError('actor spawn may be attempted exactly once')
        self.spawn_attempt_count = 1
        self.spawn_evidence['attempt_count'] = 1
        request = SpawnEntity.Request()
        request.entity_factory.name = self.document.actor_name or ''
        request.entity_factory.allow_renaming = False
        request.entity_factory.sdf_filename = str(self.actor_asset)
        request.entity_factory.pose = _pose_message(self.document.actor_initial_pose)
        request.entity_factory.relative_to = 'world'
        self._wait_service(self._node.spawn_client, name=SPAWN_SERVICE)
        self._node.spawn_request_sequence = self._node._next_sequence()
        self._node.spawn_request_stamp_ns = self._node.current_sim_stamp_ns
        request_sequence = self._node.spawn_request_sequence
        request_stamp_ns = self._node.spawn_request_stamp_ns
        self.spawn_evidence['request_sequence'] = request_sequence
        self.spawn_evidence['request_stamp_ns'] = request_stamp_ns
        try:
            future = self._node.spawn_client.call_async(request)
            self.actor_spawn_request_sent = True
        except Exception as exc:
            self.spawn_evidence['error'] = bounded_diagnostic(exc)
            raise InfrastructureError(f'{SPAWN_SERVICE} request could not be sent: {exc}') from exc
        deadline = min(self.wall_deadline, time.monotonic() + self.service_timeout_s)
        while not future.done():
            if time.monotonic() >= deadline:
                self.spawn_evidence['error'] = 'spawn response timed out'
                raise InfrastructureError(f'{SPAWN_SERVICE} response timed out')
            self._spin_once()
        try:
            response = future.result()
        except Exception as exc:
            raise InfrastructureError(f'{SPAWN_SERVICE} response failed: {exc}') from exc
        if response is None or not bool(response.success):
            self.spawn_evidence['success'] = False
            self.spawn_evidence['error'] = 'spawn response rejected the scenario actor'
            raise InfrastructureError(f'{SPAWN_SERVICE} rejected the one-shot actor spawn')
        self.actor_spawn_committed = True
        self._node.spawn_response_sequence = self._node._next_sequence()
        self._node.spawn_response_stamp_ns = self._node.current_sim_stamp_ns
        self._node._process_pending_plans()
        result = {
            'attempt_count': self.spawn_attempt_count,
            'request_sequence': request_sequence,
            'request_stamp_ns': request_stamp_ns,
            'response_sequence': self._node.spawn_response_sequence,
            'response_stamp_ns': self._node.spawn_response_stamp_ns,
            'success': True,
        }
        self.spawn_evidence.update(result)
        return result

    def _wait_actor_pose(
        self,
        *,
        target: PoseTarget | dict[str, Any],
        position_tolerance_m: float,
        sim_deadline_ns: int | None = None,
        after_sequence: int | None = None,
        target_stamp_ns: int | None = None,
    ) -> tuple[ActorPoseEvidence, float, float]:
        def candidate() -> ActorPoseEvidence | None:
            for evidence in self._node.actor_poses:
                if after_sequence is not None and evidence.collector_sequence <= after_sequence:
                    continue
                if target_stamp_ns is not None and evidence.stamp_ns < target_stamp_ns:
                    continue
                return evidence
            return None

        drain_deadline: float | None = None
        while candidate() is None:
            self._spin_once()
            if (
                sim_deadline_ns is not None
                and self._node.clock_seen
                and self._node.current_sim_stamp_ns > sim_deadline_ns
            ):
                if drain_deadline is None:
                    drain_deadline = min(self.wall_deadline, time.monotonic() + DDS_DRAIN_GRACE_S)
                elif time.monotonic() >= drain_deadline:
                    raise ScenarioFailureError(
                        'actor pose was not independently observed before its simulation deadline'
                    )
        evidence = candidate()
        assert evidence is not None
        if isinstance(target, PoseTarget):
            target_values = (target.x, target.y, target.z, target.yaw)
        else:
            target_values = (
                float(target['x']),
                float(target['y']),
                float(target['z']),
                float(target['yaw']),
            )
        position_error = math.sqrt(
            (evidence.x - target_values[0]) ** 2
            + (evidence.y - target_values[1]) ** 2
            + (evidence.z - target_values[2]) ** 2
        )
        yaw_error = shortest_yaw_error(evidence.yaw, target_values[3])
        if position_error > position_tolerance_m or yaw_error > ACTOR_YAW_TOLERANCE_RAD:
            raise ScenarioFailureError('first independently observed actor pose missed tolerance')
        return evidence, position_error, yaw_error

    def _ready_prerequisites(self) -> bool:
        node = self._node
        if not node.clock_seen or node.current_sim_stamp_ns <= 0:
            return False
        return (
            node.count_publishers(ACTION_STATUS_TOPIC) >= 1
            and node.count_publishers(ACTION_FEEDBACK_TOPIC) >= 1
            and node.count_publishers(PLAN_TOPIC) >= 1
            and node.count_publishers(GROUND_TRUTH_TOPIC) >= 1
            and (not self.document.has_actor or node.count_publishers(ENTITY_POSE_TOPIC) >= 1)
            and node.spawn_client.service_is_ready()
            and node.set_pose_client.service_is_ready()
            and node.delete_client.service_is_ready()
            and node.cancel_client.service_is_ready()
        )

    def _write_ready(self) -> None:
        node = self._node
        node.mark_ready()
        ready = {
            'identity': self.identity,
            'producer': 'robotest_scenarios/scenario_controller',
            'ready_sim_stamp_ns': node.ready_sim_stamp_ns,
            'ready_steady_ns': node.ready_steady_ns,
            'resolved_names': {
                'action_status': node.resolve_topic_name(ACTION_STATUS_TOPIC),
                'action_feedback': node.resolve_topic_name(ACTION_FEEDBACK_TOPIC),
                'clock': node.resolve_topic_name(CLOCK_TOPIC),
                'entity_pose': node.resolve_topic_name(ENTITY_POSE_TOPIC),
                'ground_truth': node.resolve_topic_name(GROUND_TRUTH_TOPIC),
                'plan': node.resolve_topic_name(PLAN_TOPIC),
            },
            'schema_version': 1,
        }
        write_canonical_json(self.ready_path, ready, write_sidecar=False)

    def _prepare(self) -> None:
        self._resolve_actor_asset()
        self._wait_for(
            self._ready_prerequisites,
            reason='controller graph did not become ready',
        )
        if self.document.scenario_id == 3:
            spawn = self._spawn_actor()
            observed, position_error, yaw_error = self._wait_actor_pose(
                target=self.document.actor_initial_pose,
                position_tolerance_m=ACTOR_INITIAL_POSITION_TOLERANCE_M,
                after_sequence=spawn['request_sequence'],
                target_stamp_ns=spawn['request_stamp_ns'],
            )
            self.interaction['actor'] = {
                'asset_sha256': self.actor_asset_sha256,
                'name': self.document.actor_name,
                'parked_pose': self.document.actor_initial_pose.as_dict(),
            }
            self.interaction['preload'] = {
                'observed_parked_before_ready': True,
                'observed_pose': observed.as_dict(),
                'position_error_m': position_error,
                'spawn_attempt_count': spawn['attempt_count'],
                'spawn_success': spawn['success'],
                'yaw_error_rad': yaw_error,
            }
        self._write_ready()

    def _wait_bound_goal(self) -> None:
        self._wait_for(
            lambda: self._node.bound_uuid is not None,
            reason='no post-readiness FollowWaypoints goal was bound',
        )

    def _wait_terminal(self) -> None:
        self._wait_for(
            lambda: self._node.terminal_status is not None,
            reason='bound FollowWaypoints goal did not reach a terminal status',
        )
        if self._node.terminal_status != GoalStatus.STATUS_SUCCEEDED:
            raise ScenarioFailureError(
                'bound FollowWaypoints terminal status was '
                f'{self._node.terminal_status}, not SUCCEEDED'
            )

    def _run_no_actor(self) -> None:
        self._wait_terminal()
        self.interaction['criteria']['bound_goal_terminal_succeeded'] = True

    def _run_scenario2(self) -> None:
        node = self._node
        if node.accepted_goal_stamp_ns is None:
            raise ProtocolError('Scenario 2 cannot calculate trigger without authoritative T0')
        trigger_stamp_ns = node.accepted_goal_stamp_ns + S2_TRIGGER_OFFSET_NS
        observation_deadline_ns = node.accepted_goal_stamp_ns + S2_OBSERVATION_DEADLINE_OFFSET_NS
        self.interaction['trigger'] = {
            'first_observed_stamp_ns': None,
            'request_sequence': None,
            'request_stamp_ns': None,
            'target_stamp_ns': trigger_stamp_ns,
            'within_window': False,
        }
        self._wait_for(
            lambda: node.current_sim_stamp_ns >= trigger_stamp_ns,
            reason='Scenario 2 simulation trigger was not reached',
        )
        if node.feedback_current_index not in {None, 0}:
            raise ScenarioFailureError('Scenario 2 insertion no longer belongs to waypoint leg 0')
        if node.initial_plan is None:
            raise InfrastructureError('Scenario 2 had no valid initial leg-0 plan before insertion')
        self.interaction['initial_plan'] = node.initial_plan
        if node.current_sim_stamp_ns > observation_deadline_ns:
            raise InfrastructureError(
                'Scenario 2 insertion window elapsed before a spawn request could be issued'
            )
        ground_truth = node.latest_ground_truth
        if ground_truth is None:
            raise InfrastructureError('Scenario 2 has no validation ground-truth sample')
        ground_truth_stamp_ns = int(ground_truth['sim_stamp_ns'])
        if (
            ground_truth_stamp_ns > node.current_sim_stamp_ns
            or node.current_sim_stamp_ns - ground_truth_stamp_ns > GROUND_TRUTH_ALIGNMENT_NS
        ):
            raise InfrastructureError('Scenario 2 ground truth is not aligned within 0.25 s')
        clearance = math.hypot(ground_truth['x'] + 1.0, ground_truth['y'] + 3.5)
        self.interaction['clearance'] = {
            'distance_m': clearance,
            'ground_truth_stamp_ns': ground_truth_stamp_ns,
            'minimum_m': S2_CLEARANCE_M,
            'passed': clearance >= S2_CLEARANCE_M,
        }
        if clearance < S2_CLEARANCE_M:
            raise ScenarioFailureError(
                'Scenario 2 insertion clearance gate failed; actor was not spawned'
            )
        spawn = self._spawn_actor()
        self.interaction['spawn'] = dict(self.spawn_evidence)
        self.interaction['trigger'].update(
            {
                'request_sequence': spawn['request_sequence'],
                'request_stamp_ns': spawn['request_stamp_ns'],
            }
        )
        if spawn['request_stamp_ns'] < trigger_stamp_ns:
            raise ProtocolError('Scenario 2 spawn request preceded T0 + 2.0 s')
        observed, position_error, yaw_error = self._wait_actor_pose(
            target=self.document.actor_initial_pose,
            position_tolerance_m=ACTOR_INITIAL_POSITION_TOLERANCE_M,
            after_sequence=spawn['request_sequence'],
            target_stamp_ns=spawn['request_stamp_ns'],
            sim_deadline_ns=observation_deadline_ns,
        )
        within_window = observed.stamp_ns <= observation_deadline_ns
        if not within_window:
            raise ScenarioFailureError('Scenario 2 actor was observed after T0 + 2.25 s')
        self._wait_terminal()
        if node.replan is None:
            raise ScenarioFailureError('Scenario 2 produced no changed post-insertion leg-0 plan')
        initial_plan = node.initial_plan
        replan = node.replan
        plan_before = int(initial_plan['collector_sequence']) < int(
            spawn['request_sequence']
        ) and int(initial_plan['sim_stamp_ns']) <= int(spawn['request_stamp_ns'])
        hash_changed = initial_plan['geometry_sha256'] != replan['geometry_sha256']
        criteria = {
            'all_segments_avoid': bool(replan['all_segments_avoid']),
            'clearance_passed': clearance >= S2_CLEARANCE_M,
            'geometry_hash_changed': hash_changed,
            'observed_by_deadline': within_window,
            'plan_before_request': plan_before,
            'replan_observed_after_request': int(replan['observed_sim_stamp_ns'])
            >= int(spawn['request_stamp_ns']),
            'replan_stamp_after_request': int(replan['sim_stamp_ns'])
            >= int(spawn['request_stamp_ns']),
            'spawn_exactly_once': self.spawn_attempt_count == 1,
        }
        if not all(criteria.values()):
            raise ScenarioFailureError('Scenario 2 frozen controller criteria did not all pass')
        self.interaction = {
            'actor': {
                'asset_sha256': self.actor_asset_sha256,
                'name': self.document.actor_name,
                'target_pose': self.document.actor_initial_pose.as_dict(),
            },
            'clearance': {
                'distance_m': clearance,
                'ground_truth_stamp_ns': ground_truth_stamp_ns,
                'minimum_m': S2_CLEARANCE_M,
                'passed': True,
            },
            'criteria': criteria,
            'initial_plan': initial_plan,
            'kind': 'static_obstacle',
            'observed_pose': {
                **observed.as_dict(),
                'passed': True,
                'position_error_m': position_error,
                'yaw_error_rad': yaw_error,
            },
            'replan': {
                'all_segments_avoid': bool(replan['all_segments_avoid']),
                'exclusion_rectangle': {
                    'x_max': S2_EXCLUSION_RECT[1],
                    'x_min': S2_EXCLUSION_RECT[0],
                    'y_max': S2_EXCLUSION_RECT[3],
                    'y_min': S2_EXCLUSION_RECT[2],
                },
                'found': True,
                'hash_changed': hash_changed,
                'plan': {
                    key: value for key, value in replan.items() if key != 'all_segments_avoid'
                },
            },
            'spawn': spawn,
            'trigger': {
                'first_observed_stamp_ns': observed.stamp_ns,
                'request_sequence': spawn['request_sequence'],
                'request_stamp_ns': spawn['request_stamp_ns'],
                'target_stamp_ns': trigger_stamp_ns,
                'within_window': within_window,
            },
        }

    def _wait_scenario3_anchor(self) -> int:
        node = self._node

        def anchored_or_terminal() -> bool:
            return node.feedback_anchor_stamp_ns is not None or node.terminal_status is not None

        self._wait_for(
            anchored_or_terminal,
            reason='Scenario 3 feedback anchor was not observed',
        )
        if node.feedback_anchor_stamp_ns is None:
            raise ScenarioFailureError(
                'Scenario 3 goal terminated before feedback transition 1 -> 2'
            )
        if node.feedback_indices_seen != {0, 1, 2}:
            raise ProtocolError('Scenario 3 feedback did not cover ordered indices 0, 1, and 2')
        return node.feedback_anchor_stamp_ns

    def _set_actor_target(self, target: dict[str, Any], transaction: dict[str, Any]) -> None:
        node = self._node
        if self.set_attempt_count >= S3_TARGET_COUNT:
            raise ProtocolError('Scenario 3 set-pose transaction count exceeded 121')
        self.set_attempt_count += 1
        transaction['attempt_count'] = 1
        transaction['outcome'] = 'REQUESTING'
        request = SetEntityPose.Request()
        request.entity.name = self.document.actor_name or ''
        request.entity.type = request.entity.MODEL
        request.pose = _pose_message(target)
        try:
            response, _, _ = self._call_service(
                node.set_pose_client,
                request,
                name=SET_POSE_SERVICE,
                evidence=transaction,
            )
        except RobotestScenarioError:
            transaction['outcome'] = 'TRANSACTION_ERROR'
            raise
        transaction['response_sequence'] = node._next_sequence()
        transaction['response_stamp_ns'] = node.current_sim_stamp_ns
        transaction['response_success'] = bool(response.success)
        if not bool(response.success):
            transaction['outcome'] = 'REJECTED'
            transaction['error'] = f'{SET_POSE_SERVICE} rejected the target'
            raise InfrastructureError(f'{SET_POSE_SERVICE} rejected target index {target["index"]}')
        self.set_success_count += 1
        transaction['outcome'] = 'RESPONSE_SUCCEEDED'

    def _resolve_s3_observations(
        self,
        transactions: list[dict[str, Any]],
        used_sequences: set[int],
    ) -> None:
        node = self._node
        for transaction in transactions:
            if transaction['observed_pose'] is not None:
                continue
            request_sequence = transaction['request_sequence']
            if request_sequence is None:
                continue
            candidate = node.matched_s3_actor_poses.get(int(transaction['index']))
            if candidate is None:
                continue
            if candidate.collector_sequence in used_sequences:
                raise ProtocolError('Scenario 3 reused one actor observation')
            used_sequences.add(candidate.collector_sequence)
            target = transaction['target_pose']
            position_error = math.sqrt(
                (candidate.x - float(target['x'])) ** 2
                + (candidate.y - float(target['y'])) ** 2
                + (candidate.z - float(target['z'])) ** 2
            )
            yaw_error = shortest_yaw_error(candidate.yaw, float(target['yaw']))
            latency_ns = candidate.stamp_ns - int(transaction['target_stamp_ns'])
            transaction['observed_pose'] = {
                **candidate.as_dict(),
                'latency_ns': latency_ns,
                'position_error_m': position_error,
                'yaw_error_rad': yaw_error,
            }
            if latency_ns > ACTOR_OBSERVATION_LATENCY_NS:
                transaction['outcome'] = 'OBSERVATION_LATE'
                raise ScenarioFailureError(
                    f'Scenario 3 target {transaction["index"]} observation was late'
                )
            if position_error > ACTOR_POSITION_TOLERANCE_M or yaw_error > ACTOR_YAW_TOLERANCE_RAD:
                transaction['outcome'] = 'OBSERVATION_OUT_OF_TOLERANCE'
                raise ScenarioFailureError(
                    f'Scenario 3 target {transaction["index"]} observation missed tolerance'
                )
            transaction['outcome'] = 'OBSERVED'

    def _reject_overdue_s3_observations(self, transactions: list[dict[str, Any]]) -> None:
        for transaction in transactions:
            if transaction['observed_pose'] is not None:
                continue
            deadline_ns = int(transaction['target_stamp_ns']) + ACTOR_OBSERVATION_LATENCY_NS
            if self._node.current_sim_stamp_ns > deadline_ns:
                index = int(transaction['index'])
                drain_deadline = self.s3_observation_drain_deadlines.setdefault(
                    index,
                    min(self.wall_deadline, time.monotonic() + DDS_DRAIN_GRACE_S),
                )
                if time.monotonic() < drain_deadline:
                    continue
                transaction['outcome'] = 'OBSERVATION_MISSING'
                transaction['error'] = 'no observed pose by the 0.20 s deadline'
                raise ScenarioFailureError(
                    f'Scenario 3 target {transaction["index"]} observation was missing'
                )

    def _run_scenario3(self) -> None:
        node = self._node
        anchor_stamp_ns = self._wait_scenario3_anchor()
        node.trajectory_active = True
        used_sequences: set[int] = set()
        self.interaction['feedback_anchor'] = {
            'from_index': 1,
            'ordered_indices_0_1_2': True,
            'stamp_ns': anchor_stamp_ns,
            'to_index': 2,
        }
        for index in range(S3_TARGET_COUNT):
            target = scenario3_target(index, anchor_stamp_ns)
            target_stamp_ns = int(target['stamp_ns'])
            while node.current_sim_stamp_ns < target_stamp_ns and node.terminal_status is None:
                self._spin_once()
                self._resolve_s3_observations(self.s3_transactions, used_sequences)
                self._reject_overdue_s3_observations(self.s3_transactions)
            self._resolve_s3_observations(self.s3_transactions, used_sequences)
            self._reject_overdue_s3_observations(self.s3_transactions)
            if node.terminal_status is not None:
                raise ScenarioFailureError(
                    f'Scenario 3 goal terminated before target index {index} was applied'
                )
            request_lateness_ns = node.current_sim_stamp_ns - target_stamp_ns
            transaction = {
                'attempt_count': 0,
                'error': None,
                'index': index,
                'observed_pose': None,
                'outcome': 'SCHEDULED',
                'request_lateness_ns': request_lateness_ns,
                'request_sequence': None,
                'request_stamp_ns': None,
                'response_sequence': None,
                'response_stamp_ns': None,
                'response_success': None,
                'target_pose': {
                    'x': target['x'],
                    'y': target['y'],
                    'yaw': target['yaw'],
                    'z': target['z'],
                },
                'target_stamp_ns': target_stamp_ns,
            }
            self.s3_transactions.append(transaction)
            if request_lateness_ns != 0:
                transaction['outcome'] = 'REQUEST_LATE'
                transaction['error'] = 'target was not issued at its exact 10 Hz stamp'
                raise ScenarioFailureError(
                    f'Scenario 3 target {index} missed its exact 10 Hz stamp'
                )
            self._set_actor_target(target, transaction)
            if transaction['request_stamp_ns'] != target_stamp_ns:
                transaction['outcome'] = 'REQUEST_LATE'
                transaction['error'] = 'service request stamp differs from exact target stamp'
                raise ScenarioFailureError(
                    f'Scenario 3 target {index} service request missed its exact stamp'
                )
            self._resolve_s3_observations(self.s3_transactions, used_sequences)
            self._reject_overdue_s3_observations(self.s3_transactions)

        if node.terminal_status is not None:
            raise ScenarioFailureError('Scenario 3 goal terminated before the trajectory finished')
        while any(item['observed_pose'] is None for item in self.s3_transactions):
            self._resolve_s3_observations(self.s3_transactions, used_sequences)
            self._reject_overdue_s3_observations(self.s3_transactions)
            if all(item['observed_pose'] is not None for item in self.s3_transactions):
                break
            if node.terminal_status is not None:
                raise ScenarioFailureError(
                    'Scenario 3 goal terminated before all target poses were observed'
                )
            self._spin_once()
        node.trajectory_active = False
        finished_stamp_ns, finished_sequence = max(
            (
                int(item['observed_pose']['stamp_ns']),
                int(item['observed_pose']['collector_sequence']),
            )
            for item in self.s3_transactions
        )
        self._wait_terminal()
        terminal_stamp_ns = node.terminal_observed_stamp_ns
        finished_before_terminal = (
            terminal_stamp_ns is not None
            and node.terminal_observed_sequence is not None
            and (finished_stamp_ns, finished_sequence)
            < (terminal_stamp_ns, node.terminal_observed_sequence)
        )
        if not finished_before_terminal:
            raise ScenarioFailureError(
                'Scenario 3 trajectory did not finish before terminal action status observation'
            )

        transactions = self.s3_transactions
        move_1 = [float(item['observed_pose']['x']) for item in transactions[:41]]
        dwell = [float(item['observed_pose']['x']) for item in transactions[41:81]]
        move_2 = [float(item['observed_pose']['x']) for item in transactions[81:]]
        monotonic_move_1 = _monotonic_with_tolerance(move_1, 1e-9)
        monotonic_move_2 = _monotonic_with_tolerance(move_2, 1e-9)
        dwell_constant = bool(dwell) and all(
            abs(value) <= ACTOR_POSITION_TOLERANCE_M for value in dwell
        )
        final_observed = transactions[-1]['observed_pose']
        final_endpoint = abs(float(final_observed['x']) - 0.8) <= ACTOR_POSITION_TOLERANCE_M
        target_hash_records = [
            {
                'index': item['index'],
                'target_pose': item['target_pose'],
                'target_stamp_ns': item['target_stamp_ns'],
            }
            for item in transactions
        ]
        observed_hash_records = [
            {
                'index': item['index'],
                'observed_pose': {
                    key: value
                    for key, value in item['observed_pose'].items()
                    if key in {'stamp_ns', 'x', 'y', 'yaw', 'z'}
                },
            }
            for item in transactions
        ]
        criteria = {
            'all_targets_observed': all(item['observed_pose'] is not None for item in transactions),
            'all_transactions_succeeded': self.set_success_count == S3_TARGET_COUNT,
            'exact_single_attempts': self.set_attempt_count == S3_TARGET_COUNT
            and all(item['attempt_count'] == 1 for item in transactions),
            'exact_schedule': all(
                item['request_lateness_ns'] == 0
                and item['request_stamp_ns'] == item['target_stamp_ns']
                for item in transactions
            ),
            'exact_target_count': len(transactions) == S3_TARGET_COUNT,
            'final_endpoint': final_endpoint,
            'finished_before_terminal_status_observation': finished_before_terminal,
            'latency_within_limit': all(
                item['observed_pose']['latency_ns'] <= ACTOR_OBSERVATION_LATENCY_NS
                for item in transactions
            ),
            'monotonic_and_dwell': monotonic_move_1 and dwell_constant and monotonic_move_2,
            'no_early_motion': True,
            'ordered_feedback_anchor': node.feedback_indices_seen == {0, 1, 2},
            'pose_errors_within_limit': all(
                item['observed_pose']['position_error_m'] <= ACTOR_POSITION_TOLERANCE_M
                and item['observed_pose']['yaw_error_rad'] <= ACTOR_YAW_TOLERANCE_RAD
                for item in transactions
            ),
            'preloaded_before_goal': self.actor_spawn_committed,
        }
        if not all(criteria.values()):
            raise ScenarioFailureError('Scenario 3 frozen controller criteria did not all pass')
        self.interaction.update(
            {
                'criteria': criteria,
                'trajectory': {
                    'dwell_constant': dwell_constant,
                    'early_motion_count': 0,
                    'expected_target_count': S3_TARGET_COUNT,
                    'final_endpoint_passed': final_endpoint,
                    'finished_before_terminal_status_observation': finished_before_terminal,
                    'finished_stamp_ns': finished_stamp_ns,
                    'finished_collector_sequence': finished_sequence,
                    'invalid_value_count': node.actor_state.invalid_count,
                    'late_observation_count': 0,
                    'matched_observation_count': len(transactions),
                    'missing_observation_count': 0,
                    'monotonic_move_1': monotonic_move_1,
                    'monotonic_move_2': monotonic_move_2,
                    'observed_trajectory_sha256': trajectory_sha256(observed_hash_records),
                    'set_attempt_count': self.set_attempt_count,
                    'set_success_count': self.set_success_count,
                    'target_trajectory_sha256': trajectory_sha256(target_hash_records),
                    'targets': transactions,
                    'terminal_basis': 'follow_waypoints_status_observation',
                },
            }
        )

    def _cancel_bound_goal_once(self) -> None:
        node = self._node
        if node.bound_uuid is None or self.cancel_attempt_count != 0:
            return
        self.cancel_attempt_count = 1
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = list(node.bound_uuid)
        request.goal_info.stamp.sec = 0
        request.goal_info.stamp.nanosec = 0
        evidence: dict[str, Any] = {
            'attempt_count': 1,
            'goal_uuid': node.bound_uuid_text,
            'request_policy': 'exact_nonzero_uuid_zero_stamp',
        }
        try:
            response, _, request_stamp_ns = self._call_service(
                node.cancel_client,
                request,
                name=ACTION_CANCEL_SERVICE,
                check_fatal=False,
            )
            matched = any(
                bytes(item.goal_id.uuid) == node.bound_uuid for item in response.goals_canceling
            )
            evidence.update(
                {
                    'exact_uuid_acknowledged': matched,
                    'request_stamp_ns': request_stamp_ns,
                    'return_code': int(response.return_code),
                }
            )
        except RobotestScenarioError as exc:
            evidence['error'] = str(exc)
        self.cancel_evidence = evidence

    def _cleanup_actor(self) -> None:
        node = self._node
        if not self.document.has_actor:
            return
        if not self.actor_spawn_request_sent:
            self.cleanup = {
                'actor_absent': True,
                'delete_attempt_count': 0,
                'delete_success': None,
                'proof': {'kind': 'spawn_request_not_sent'},
                'required': False,
            }
            return
        if self.delete_attempt_count != 0:
            raise ProtocolError('actor delete may be attempted exactly once')
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
        request.entity.name = self.document.actor_name or ''
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
                'delete_attempt_count': self.delete_attempt_count,
                'delete_success': False,
                'proof': {
                    'kind': 'delete_response_did_not_prove_absence',
                    'request_sequence': request_sequence,
                    'request_stamp_ns': request_stamp_ns,
                },
                'required': True,
            }
            raise InfrastructureError(f'{DELETE_SERVICE} rejected actor cleanup')
        response_sequence = node._next_sequence()
        response_stamp_ns = node.current_sim_stamp_ns
        node.delete_response_stamp_ns = response_stamp_ns
        node.post_delete_pose_count = 0
        node.post_delete_entity_message_count = 0
        node.post_delete_entity_latest_sim_stamp_ns = None
        quiet_until_ns = response_stamp_ns + ACTOR_CLEANUP_QUIET_NS
        self.cleanup = {
            'actor_absent': False,
            'delete_attempt_count': self.delete_attempt_count,
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
            reason='actor pose source did not span the cleanup quiet interval',
            sim_deadline_ns=quiet_until_ns + ACTOR_CLEANUP_QUIET_NS,
            check_fatal=False,
        )
        source_publishers_after = node.count_publishers(ENTITY_POSE_TOPIC)
        post_delete_pose_count = node.post_delete_pose_count
        pose_source_heartbeat_count = node.post_delete_entity_message_count
        actor_absent = (
            post_delete_pose_count == 0
            and pose_source_heartbeat_count >= 1
            and source_publishers_after >= 1
        )
        self.cleanup = {
            'actor_absent': actor_absent,
            'delete_attempt_count': self.delete_attempt_count,
            'delete_success': success,
            'proof': {
                'kind': 'successful_delete_response_and_pose_quiet_interval',
                'pose_source_publishers_after': source_publishers_after,
                'pose_source_publishers_before': source_publishers_before,
                'post_delete_pose_source_heartbeat_count': pose_source_heartbeat_count,
                'post_delete_pose_source_latest_sim_stamp_ns': (
                    node.post_delete_entity_latest_sim_stamp_ns
                ),
                'post_delete_pose_count': post_delete_pose_count,
                'quiet_until_sim_stamp_ns': quiet_until_ns,
                'request_sequence': request_sequence,
                'request_stamp_ns': request_stamp_ns,
                'response_sequence': response_sequence,
                'response_stamp_ns': response_stamp_ns,
            },
            'required': True,
        }
        if not actor_absent:
            raise ScenarioFailureError(
                'actor pose stream continued after successful delete response'
            )

    def _workflow(self) -> None:
        self._prepare()
        self._wait_bound_goal()
        if self.document.scenario_id == 2:
            self._run_scenario2()
        elif self.document.scenario_id == 3:
            self._run_scenario3()
        else:
            self._run_no_actor()

    def _binding(self) -> dict[str, Any]:
        node = self._node
        status_trace = [dict(item) for item in node.status.items]
        feedback_trace = [
            {key: value for key, value in item.items() if not key.startswith('_')}
            for item in node.feedback.items
            if node.bound_uuid_text is None or item['uuid'] == node.bound_uuid_text
        ]
        return {
            'accepted_goal_stamp_ns': node.accepted_goal_stamp_ns,
            'cancel': self.cancel_evidence,
            'feedback_trace': feedback_trace,
            'goal_uuid': node.bound_uuid_text,
            'ready_sim_stamp_ns': node.ready_sim_stamp_ns,
            'ready_steady_ns': node.ready_steady_ns,
            'status_trace': status_trace,
            'terminal_observed_stamp_ns': node.terminal_observed_stamp_ns,
            'terminal_observed_sequence': node.terminal_observed_sequence,
            'terminal_status': node.terminal_status,
        }

    def _configuration(self) -> dict[str, Any]:
        return {
            'actor_asset_sha256': self.actor_asset_sha256,
            'controller_configuration': controller_configuration(),
            'controller_configuration_sha256': configuration_sha256(),
            'service_timeout_s': self.service_timeout_s,
            'source_binding': source_binding(),
            'wall_timeout_s': self.wall_timeout_s,
        }

    def _project_partial_interaction(self) -> None:
        """Project bounded in-progress mechanics into failure artifacts."""
        node = self._node
        if self.document.has_actor:
            self.interaction.setdefault(
                'actor',
                {
                    'asset_sha256': self.actor_asset_sha256,
                    'name': self.document.actor_name,
                    'target_pose': (
                        self.document.actor_initial_pose.as_dict()
                        if self.document.actor_initial_pose is not None
                        else None
                    ),
                },
            )
            if self.spawn_attempt_count > 0:
                self.interaction['spawn'] = dict(self.spawn_evidence)
            if node.first_spawn_observation is not None:
                self.interaction.setdefault(
                    'first_spawn_observation', node.first_spawn_observation.as_dict()
                )
        if self.document.scenario_id == 2:
            if node.initial_plan is not None:
                self.interaction.setdefault('initial_plan', node.initial_plan)
            if node.replan is not None:
                self.interaction.setdefault('first_changed_replan', node.replan)
        if self.document.scenario_id == 3:
            trajectory = self.interaction.setdefault('trajectory', {})
            trajectory.setdefault('expected_target_count', S3_TARGET_COUNT)
            trajectory.setdefault('set_attempt_count', self.set_attempt_count)
            trajectory.setdefault('set_success_count', self.set_success_count)
            trajectory.setdefault('targets', self.s3_transactions)

    def _result(self, error: RobotestScenarioError | None) -> tuple[dict[str, Any], int]:
        node = self._node
        self._project_partial_interaction()
        if error is None:
            exit_code = int(ExitCode.COMPLETE)
            reason = (
                'scenario controller mechanics complete; benchmark verdict is owned by '
                'the suite compositor'
            )
            status = 'PASS'
        else:
            exit_code = int(error.exit_code)
            reason = bounded_diagnostic(error)
            status = 'FAIL'
        result = {
            'binding': self._binding(),
            'cleanup': self.cleanup,
            'configuration': self._configuration(),
            'identity': self.identity,
            'interaction': self.interaction,
            'producer': 'robotest_scenarios/scenario_controller',
            'quality': node.quality(),
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
        """Execute, clean up, finalize evidence, and return a stable exit code."""
        error: RobotestScenarioError | None = None
        initialized = False
        try:
            rclpy.init(args=list(self.raw_ros_args))
            initialized = True
            self.node = ScenarioControllerNode(self.document)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self._workflow()
        except RobotestScenarioError as exc:
            error = exc
            if self.node is not None:
                self.node.begin_cleanup()
            if self._bound_goal_needs_cancel():
                self._cancel_bound_goal_once()
        except Exception as exc:
            error = InfrastructureError(f'unhandled scenario controller failure: {exc}')
            if self.node is not None:
                self.node.begin_cleanup()
            if self._bound_goal_needs_cancel():
                self._cancel_bound_goal_once()
        finally:
            if self.node is not None and self.executor is not None:
                self.node.begin_cleanup()
                try:
                    self._cleanup_actor()
                except RobotestScenarioError as cleanup_error:
                    if error is None:
                        error = cleanup_error
                result, exit_code = self._result(error)
                try:
                    schema = load_schema(package_schema_path('scenario-result.schema.json'))
                    write_canonical_json(self.output_path, result, schema=schema)
                except ArtifactError as artifact_error:
                    print(f'scenario_controller: artifact error: {artifact_error}', file=sys.stderr)
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

    def _bound_goal_needs_cancel(self) -> bool:
        return (
            self.node is not None
            and self.node.bound_uuid is not None
            and self.node.terminal_status is None
        )


def _monotonic_with_tolerance(values: Sequence[float], tolerance: float) -> bool:
    return bool(values) and all(
        current + tolerance >= previous for previous, current in pairwise(values)
    )


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise ValidationError(message)


def _parser() -> argparse.ArgumentParser:
    parser = _ArgumentParser(
        prog='scenario_controller',
        description='Run one bounded ADR 0006 Phase 3 scenario controller.',
    )
    parser.add_argument('--scenario', required=True, help='strict Phase 3 scenario YAML')
    parser.add_argument('--output', required=True, help='new scenario-result.json path')
    parser.add_argument('--ready-file', required=True, help='new atomic readiness JSON path')
    parser.add_argument('--run-id', required=True)
    parser.add_argument('--candidate-id', required=True)
    parser.add_argument('--repetition-index', required=True, type=int, choices=(0, 1, 2))
    parser.add_argument('--suite-index', required=True, type=int, choices=range(15))
    parser.add_argument('--wall-timeout-s', type=float)
    parser.add_argument('--service-timeout-s', type=float, default=DEFAULT_SERVICE_TIMEOUT_S)
    return parser


def _positive_bounded(value: float, *, label: str, maximum: float) -> float:
    if not math.isfinite(value) or value <= 0.0 or value > maximum:
        raise ValidationError(f'{label} must be finite and in (0, {maximum}]')
    return value


def _inputs(
    raw_args: Sequence[str],
) -> tuple[ScenarioDocument, Path, Path, dict[str, Any], float, float]:
    cli = _parser().parse_args(remove_ros_args(args=list(raw_args))[1:])
    document = load_scenario(cli.scenario)
    run_id = validate_run_id(cli.run_id)
    candidate_id = validate_candidate_id(cli.candidate_id)
    expected_suite_index = (document.scenario_id - 1) * 3 + cli.repetition_index
    if cli.suite_index != expected_suite_index:
        raise ValidationError(
            f'suite index must equal (scenario_id-1)*3+repetition_index ({expected_suite_index})'
        )
    output_path = validate_new_output(cli.output, label='scenario output')
    ready_path = validate_new_output(cli.ready_file, label='scenario ready file')
    reserved_paths = {output_path, sha256_sidecar(output_path), ready_path}
    if len(reserved_paths) != 3:
        raise ValidationError('scenario output, checksum sidecar, and ready file must be distinct')
    wall_timeout_s = document.wall_escape_timeout_s
    if cli.wall_timeout_s is not None and cli.wall_timeout_s != wall_timeout_s:
        raise ValidationError(
            f'benchmark wall timeout must remain frozen at {wall_timeout_s:.1f} s'
        )
    service_timeout_s = _positive_bounded(
        cli.service_timeout_s,
        label='service timeout',
        maximum=30.0,
    )
    identity = {
        'candidate_id': candidate_id,
        'repetition_index': cli.repetition_index,
        'run_id': run_id,
        'scenario_id': document.scenario_id,
        'scenario_name': document.scenario_name,
        'scenario_sha256': document.sha256,
        'suite_index': cli.suite_index,
    }
    return (
        document,
        output_path,
        ready_path,
        identity,
        wall_timeout_s,
        service_timeout_s,
    )


def main(args: Sequence[str] | None = None) -> int:
    """Run the installed scenario controller and return its stable exit code."""
    raw_args = list(sys.argv if args is None else ['scenario_controller', *args])
    try:
        document, output, ready, identity, wall_timeout, service_timeout = _inputs(raw_args)
    except (OSError, RuntimeError, ValidationError) as exc:
        print(f'scenario_controller: validation error: {exc}', file=sys.stderr)
        return int(ExitCode.VALIDATION_ERROR)
    app = ScenarioControllerApp(
        document=document,
        output_path=output,
        ready_path=ready,
        identity=identity,
        wall_timeout_s=wall_timeout,
        service_timeout_s=service_timeout,
        raw_ros_args=raw_args,
    )
    return app.run()
