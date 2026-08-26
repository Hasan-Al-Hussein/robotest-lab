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

"""Thin relative-name rclpy adapter for deterministic fault orchestration."""

from __future__ import annotations

import json
import uuid
from typing import Any

from builtin_interfaces.msg import Duration, Time
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from robotest_interfaces.msg import FaultEvent
from robotest_interfaces.msg import FaultSpec as FaultSpecMessage
from robotest_interfaces.srv import ArmFaultSchedule, PreloadFaultSchedule
from robotest_missions.execution import (
    ArmResponse,
    FaultProtocolError,
    PreloadResponse,
    ResetResponse,
)
from robotest_missions.models import FaultSchedule

RESET_SERVICE = 'faults/reset'
PRELOAD_SERVICE = 'faults/preload_schedule'
ARM_SERVICE = 'faults/arm_schedule'
EVENT_TOPIC = 'faults/events'
EVENT_SUBSCRIBER_CAPACITY = 512
FAULT_EVENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


def _time_from_ns(value: int) -> Time:
    seconds, nanoseconds = divmod(value, 1_000_000_000)
    return Time(sec=seconds, nanosec=nanoseconds)


def _duration_from_ns(value: int) -> Duration:
    seconds, nanoseconds = divmod(value, 1_000_000_000)
    return Duration(sec=seconds, nanosec=nanoseconds)


def _stamp_ns(stamp: object) -> int:
    try:
        seconds = stamp.sec
        nanoseconds = stamp.nanosec
    except AttributeError as exc:
        raise FaultProtocolError('fault timestamp fields are unavailable') from exc
    if (
        not isinstance(seconds, int)
        or isinstance(seconds, bool)
        or not isinstance(nanoseconds, int)
        or isinstance(nanoseconds, bool)
        or seconds < 0
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise FaultProtocolError('fault timestamp fields are invalid')
    return seconds * 1_000_000_000 + nanoseconds


def _uuid_string(message: object) -> str:
    raw = bytes(message.uuid)
    if len(raw) != 16:
        raise FaultProtocolError('fault-control UUID must contain 16 bytes')
    return str(uuid.UUID(bytes=raw))


def _uuid_message(value: str) -> Any:
    from unique_identifier_msgs.msg import UUID

    message = UUID()
    try:
        raw = uuid.UUID(value).bytes
    except (AttributeError, ValueError) as exc:
        raise FaultProtocolError(f'invalid goal UUID: {value}') from exc
    if not any(raw):
        raise FaultProtocolError('goal UUID must not be all zero')
    message.uuid = list(raw)
    return message


def _future_result(future: Any, operation: str) -> Any | None:
    if future is None or not future.done():
        return None
    try:
        response = future.result()
    except Exception as exc:
        raise FaultProtocolError(f'{operation} service failed: {exc}') from exc
    if response is None:
        raise FaultProtocolError(f'{operation} service returned no response')
    return response


class RclpyFaultControlDriver:
    """Expose fault services and events as bounded non-blocking operations."""

    def __init__(self, node: Node) -> None:
        self._reset_client = node.create_client(Trigger, RESET_SERVICE)
        self._preload_client = node.create_client(PreloadFaultSchedule, PRELOAD_SERVICE)
        self._arm_client = node.create_client(ArmFaultSchedule, ARM_SERVICE)
        self._reset_future = None
        self._preload_future = None
        self._arm_future = None
        self._events: list[dict[str, Any]] = []
        self._event_overflow_count = 0
        self._subscription = node.create_subscription(
            FaultEvent,
            EVENT_TOPIC,
            self._on_event,
            FAULT_EVENT_QOS,
        )

    def services_are_ready(self) -> bool:
        return all(
            client.service_is_ready()
            for client in (self._reset_client, self._preload_client, self._arm_client)
        )

    def start_reset(self) -> None:
        if self._reset_future is not None:
            raise FaultProtocolError('reset request is already pending')
        self._reset_future = self._reset_client.call_async(Trigger.Request())

    def poll_reset_response(self) -> ResetResponse | None:
        response = _future_result(self._reset_future, 'reset')
        if response is None:
            return None
        self._reset_future = None
        return ResetResponse(success=bool(response.success), message=str(response.message))

    def start_preload(self, schedule: FaultSchedule) -> None:
        if self._preload_future is not None:
            raise FaultProtocolError('preload request is already pending')
        request = PreloadFaultSchedule.Request()
        request.schedule_hash = schedule.sha256
        for fault in schedule.faults:
            message = FaultSpecMessage()
            message.schema_version = fault.schema_version
            message.fault_id = fault.fault_id
            message.target = fault.target
            message.mode = fault.mode
            message.start_offset = _duration_from_ns(fault.start_offset_ns)
            message.duration = _duration_from_ns(fault.duration_ns)
            message.seed = fault.seed
            message.parameters_json = json.dumps(
                dict(fault.parameters),
                ensure_ascii=True,
                separators=(',', ':'),
            )
            request.faults.append(message)
        self._preload_future = self._preload_client.call_async(request)

    def poll_preload_response(self) -> PreloadResponse | None:
        response = _future_result(self._preload_future, 'preload')
        if response is None:
            return None
        self._preload_future = None
        return PreloadResponse(
            accepted=bool(response.accepted),
            replayed=bool(response.replayed),
            state=int(response.state),
            schedule_hash=str(response.schedule_hash),
            loaded_count=int(response.loaded_count),
            generation=int(response.generation),
            message=str(response.message),
        )

    def start_arm(
        self,
        schedule_hash: str,
        generation: int,
        goal_uuid: str,
        accepted_goal_stamp_ns: int,
    ) -> None:
        if self._arm_future is not None:
            raise FaultProtocolError('arm request is already pending')
        request = ArmFaultSchedule.Request()
        request.schedule_hash = schedule_hash
        request.generation = generation
        request.goal_uuid = _uuid_message(goal_uuid)
        request.accepted_goal_stamp = _time_from_ns(accepted_goal_stamp_ns)
        self._arm_future = self._arm_client.call_async(request)

    def poll_arm_response(self) -> ArmResponse | None:
        response = _future_result(self._arm_future, 'arm')
        if response is None:
            return None
        self._arm_future = None
        return ArmResponse(
            accepted=bool(response.accepted),
            replayed=bool(response.replayed),
            state=int(response.state),
            schedule_hash=str(response.schedule_hash),
            generation=int(response.generation),
            goal_uuid=_uuid_string(response.goal_uuid),
            accepted_goal_stamp_ns=_stamp_ns(response.accepted_goal_stamp),
            arm_commit_stamp_ns=_stamp_ns(response.arm_commit_stamp),
            arm_margin_ns=int(response.arm_margin_ns),
            message=str(response.message),
        )

    def drain_events(self) -> tuple[dict[str, Any], ...]:
        events = tuple(self._events)
        self._events.clear()
        return events

    @property
    def event_overflow_count(self) -> int:
        return self._event_overflow_count

    def _on_event(self, message: FaultEvent) -> None:
        if len(self._events) >= EVENT_SUBSCRIBER_CAPACITY:
            self._event_overflow_count += 1
            return
        try:
            configured_deactivation_ns = _stamp_ns(message.configured_deactivation_time)
            event = {
                'header_stamp_ns': _stamp_ns(message.header.stamp),
                'schema_version': int(message.schema_version),
                'event_sequence': int(message.event_sequence),
                'event_type': int(message.event_type),
                'state_before': int(message.state_before),
                'state_after': int(message.state_after),
                'accepted': bool(message.accepted),
                'replayed': bool(message.replayed),
                'requested_schedule_hash': str(message.requested_schedule_hash),
                'committed_schedule_hash': str(message.committed_schedule_hash),
                'requested_generation': int(message.requested_generation),
                'committed_generation': int(message.committed_generation),
                'requested_fault_count': int(message.requested_fault_count),
                'committed_fault_count': int(message.committed_fault_count),
                'requested_goal_uuid': _uuid_string(message.requested_goal_uuid),
                'bound_goal_uuid': _uuid_string(message.bound_goal_uuid),
                'requested_t0_ns': _stamp_ns(message.requested_t0),
                'bound_t0_ns': _stamp_ns(message.bound_t0),
                'arm_commit_stamp_ns': _stamp_ns(message.arm_commit_time),
                'arm_margin_ns': int(message.arm_margin_ns),
                'fault_id': str(message.fault_id),
                'target': int(message.target),
                'mode': int(message.mode),
                'configured_activation_stamp_ns': _stamp_ns(message.configured_activation_time),
                'configured_deactivation_stamp_ns': configured_deactivation_ns,
                'actual_stamp_ns': _stamp_ns(message.actual_time),
                'seed': int(message.seed),
                'input_sequence': int(message.input_sequence),
                'raw_input_count': int(message.raw_input_count),
                'validated_output_count': int(message.validated_output_count),
                'affected_message_count': int(message.affected_message_count),
                'detail': str(message.detail),
            }
        except (AttributeError, TypeError, ValueError, FaultProtocolError):
            self._event_overflow_count += 1
            return
        self._events.append(event)
