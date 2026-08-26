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

"""Thin direct rclpy FollowWaypoints adapter."""

from __future__ import annotations

import math
import uuid
from collections.abc import Callable

from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from rclpy.time import Time

from robotest_missions.execution import (
    ActionProtocolError,
    GoalResponse,
    MissedWaypoint,
    TerminalActionResult,
)
from robotest_missions.models import MissionConfig

ACTION_NAME = 'follow_waypoints'
ACTION_STATUS_TOPIC = f'{ACTION_NAME}/_action/status'


def _goal_uuid(value: object) -> str:
    raw = bytes(value)
    if len(raw) != 16:
        raise ActionProtocolError(f'goal UUID must contain 16 bytes, found {len(raw)}')
    return str(uuid.UUID(bytes=raw))


def _stamp_ns(stamp: object) -> int:
    """Project a structurally valid ROS timestamp, including zero."""
    try:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
    except AttributeError as exc:
        raise ActionProtocolError('ROS timestamp fields are unavailable') from exc
    if (
        not isinstance(seconds, int)
        or isinstance(seconds, bool)
        or not isinstance(nanoseconds, int)
        or isinstance(nanoseconds, bool)
        or seconds < 0
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ActionProtocolError('ROS timestamp fields are invalid')
    stamp_ns = seconds * 1_000_000_000 + nanoseconds
    return stamp_ns


def _pose_evidence(message: PoseStamped) -> dict[str, object]:
    stamp_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
    return {
        'frame_id': message.header.frame_id,
        'stamp_ns': stamp_ns,
        'position': {
            'x': message.pose.position.x,
            'y': message.pose.position.y,
            'z': message.pose.position.z,
        },
        'orientation': {
            'x': message.pose.orientation.x,
            'y': message.pose.orientation.y,
            'z': message.pose.orientation.z,
            'w': message.pose.orientation.w,
        },
    }


class RclpyFollowWaypointsDriver:
    """Expose rclpy futures as explicit non-blocking poll operations."""

    def __init__(self, node: Node) -> None:
        self._node = node
        self._client = ActionClient(node, FollowWaypoints, ACTION_NAME)
        self._send_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._goal_uuid_bytes: bytes | None = None
        self._status_stamps: dict[bytes, int] = {}
        self._status_protocol_errors: dict[bytes, str] = {}
        self._status_subscription = node.create_subscription(
            GoalStatusArray,
            ACTION_STATUS_TOPIC,
            self._on_status,
            qos_profile_action_status_default,
        )

    @property
    def action_name(self) -> str:
        return ACTION_NAME

    @property
    def resolved_action_name(self) -> str:
        return self._node.resolve_topic_name(ACTION_NAME)

    def server_is_ready(self) -> bool:
        return self._client.server_is_ready()

    def start_goal(
        self,
        config: MissionConfig,
        submission_stamp_ns: int,
        feedback_callback: Callable[[int], None],
    ) -> None:
        if self._send_future is not None:
            raise ActionProtocolError('only one goal may be active per runner')
        goal = FollowWaypoints.Goal()
        goal.number_of_loops = 0
        goal.goal_index = 0
        stamp = Time(nanoseconds=submission_stamp_ns).to_msg()
        for waypoint in config.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = config.frame_id
            pose.header.stamp = stamp
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.z = math.sin(waypoint.yaw / 2.0)
            pose.pose.orientation.w = math.cos(waypoint.yaw / 2.0)
            goal.poses.append(pose)

        def on_feedback(message: object) -> None:
            try:
                current_waypoint = int(message.feedback.current_waypoint)
            except (AttributeError, TypeError, ValueError):
                # Project asynchronous callback errors into deterministic evidence.
                current_waypoint = -1
            feedback_callback(current_waypoint)

        self._send_future = self._client.send_goal_async(
            goal,
            feedback_callback=on_feedback,
        )

    def poll_goal_response(self) -> GoalResponse | None:
        if self._send_future is None:
            raise ActionProtocolError('goal was not submitted')
        if not self._send_future.done():
            return None
        if self._goal_handle is not None:
            raise ActionProtocolError('goal response was polled more than once')
        try:
            handle = self._send_future.result()
        except Exception as exc:
            raise ActionProtocolError(f'goal response failed: {exc}') from exc
        if handle is None:
            raise ActionProtocolError('goal response future returned no handle')
        if not handle.accepted:
            return GoalResponse(accepted=False)

        raw_uuid = bytes(handle.goal_id.uuid)
        self._goal_uuid_bytes = raw_uuid
        self._goal_handle = handle
        self._result_future = handle.get_result_async()
        goal_uuid = _goal_uuid(raw_uuid)
        response_stamp_ns = _stamp_ns(handle.stamp)
        return GoalResponse(
            accepted=True,
            goal_uuid=goal_uuid,
            response_stamp_ns=response_stamp_ns,
        )

    def accepted_goal_is_active(self) -> bool:
        return self._goal_handle is not None

    def poll_accepted_status_stamp_ns(self) -> int | None:
        if self._goal_uuid_bytes is None:
            raise ActionProtocolError('cannot poll action status before goal acceptance')
        error = self._status_protocol_errors.get(self._goal_uuid_bytes)
        if error is not None:
            raise ActionProtocolError(error)
        stamp_ns = self._status_stamps.get(self._goal_uuid_bytes)
        return stamp_ns if stamp_ns is not None and stamp_ns > 0 else None

    def poll_terminal_result(self) -> TerminalActionResult | None:
        if self._result_future is None or not self._result_future.done():
            return None
        try:
            wrapped = self._result_future.result()
        except Exception as exc:
            raise ActionProtocolError(f'terminal result failed: {exc}') from exc
        if wrapped is None or wrapped.result is None:
            raise ActionProtocolError('terminal result future returned no result')
        result = wrapped.result
        missed = tuple(
            MissedWaypoint(
                index=int(item.index),
                error_code=int(item.error_code),
                goal=_pose_evidence(item.goal),
            )
            for item in result.missed_waypoints
        )
        return TerminalActionResult(
            status_code=int(wrapped.status),
            error_code=int(result.error_code),
            error_message=str(result.error_msg),
            missed_waypoints=missed,
        )

    def request_cancel(self) -> None:
        if self._goal_handle is None:
            raise ActionProtocolError('cannot cancel before goal acceptance')
        if self._cancel_future is not None:
            raise ActionProtocolError('cancellation was already requested')
        self._cancel_future = self._goal_handle.cancel_goal_async()

    def poll_cancel_acknowledgement(self) -> bool | None:
        if self._cancel_future is None:
            raise ActionProtocolError('cancellation was not requested')
        if not self._cancel_future.done():
            return None
        try:
            response = self._cancel_future.result()
        except Exception as exc:
            raise ActionProtocolError(f'cancellation response failed: {exc}') from exc
        if response is None or self._goal_uuid_bytes is None:
            raise ActionProtocolError('cancellation response was incomplete')
        return any(
            bytes(goal.goal_id.uuid) == self._goal_uuid_bytes for goal in response.goals_canceling
        )

    def _on_status(self, message: GoalStatusArray) -> None:
        for status in message.status_list:
            raw_uuid = bytes(status.goal_info.goal_id.uuid)
            try:
                _goal_uuid(raw_uuid)
                stamp_ns = _stamp_ns(status.goal_info.stamp)
                status_code = status.status
                if (
                    not isinstance(status_code, int)
                    or isinstance(status_code, bool)
                    or not 1 <= status_code <= 6
                ):
                    raise ActionProtocolError(
                        'action status code must be an integer in the range 1..6'
                    )
            except ActionProtocolError as exc:
                self._status_protocol_errors[raw_uuid] = (
                    f'invalid action status for goal UUID: {exc}'
                )
                continue
            previous = self._status_stamps.get(raw_uuid)
            if previous is not None and stamp_ns != previous:
                self._status_protocol_errors[raw_uuid] = (
                    'conflicting action-status acceptance stamps for goal UUID'
                )
                continue
            if previous is None:
                self._status_stamps[raw_uuid] = stamp_ns
