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

"""Bounded transport-independent mission action state machine."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any, Protocol

from robotest_missions.models import MissionConfig

ACTION_SERVER_WAIT_WALL_S = 30.0
SIM_TIME_READY_WAIT_WALL_S = 30.0
GOAL_RESPONSE_TIMEOUT_WALL_S = 10.0
ACCEPTED_STATUS_TIMEOUT_WALL_S = 10.0
CANCEL_ACK_TIMEOUT_WALL_S = 5.0
CANCEL_RESULT_TIMEOUT_WALL_S = 10.0
FEEDBACK_TRACE_CAPACITY = 4096
SPIN_POLL_WALL_S = 0.05
NAV2_ERROR_NONE = 0
ACCEPTED_GOAL_STAMP_SOURCE = 'uuid_matched_action_status_goal_info'


class ExitCode(IntEnum):
    """Stable process contract shared with the Phase 2 verifier."""

    SUCCESS = 0
    VALIDATION_ERROR = 10
    INFRASTRUCTURE_ERROR = 20
    GOAL_REJECTED = 21
    GOAL_ABORTED = 22
    GOAL_CANCELED = 23
    TIMEOUT = 24
    ARTIFACT_ERROR = 25


class GoalStatusCode(IntEnum):
    """action_msgs/msg/GoalStatus values, kept ROS-independent for tests."""

    UNKNOWN = 0
    ACCEPTED = 1
    EXECUTING = 2
    CANCELING = 3
    SUCCEEDED = 4
    CANCELED = 5
    ABORTED = 6


GOAL_STATUS_NAMES = {status.value: status.name for status in GoalStatusCode}


class ActionProtocolError(RuntimeError):
    """Raised when the action transport violates the bounded client contract."""


class FeedbackTraceOverflowError(ActionProtocolError):
    """Raised after an accepted goal exceeds the fixed feedback evidence bound."""


class MissionClock(Protocol):
    """Two-clock source: ROS simulation time plus steady wall time."""

    def sim_time_ns(self) -> int:
        """Return the current ROS time in nanoseconds."""

    def steady_time_s(self) -> float:
        """Return monotonic steady time in seconds."""


@dataclass(frozen=True, slots=True)
class GoalResponse:
    """Observed action goal response."""

    accepted: bool
    goal_uuid: str | None = None
    response_stamp_ns: int | None = None


@dataclass(frozen=True, slots=True)
class MissedWaypoint:
    """Stable projection of nav2_msgs/msg/MissedWaypoint."""

    index: int
    error_code: int
    goal: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        """Return JSON-safe evidence."""
        return {'index': self.index, 'error_code': self.error_code, 'goal': self.goal}


@dataclass(frozen=True, slots=True)
class TerminalActionResult:
    """Terminal FollowWaypoints result independent of rclpy message objects."""

    status_code: int
    error_code: int
    error_message: str
    missed_waypoints: tuple[MissedWaypoint, ...]


class FollowWaypointsDriver(Protocol):
    """Non-blocking action transport consumed by :class:`MissionExecutor`."""

    @property
    def action_name(self) -> str:
        """Return the relative internal action name."""

    @property
    def resolved_action_name(self) -> str:
        """Return the fully resolved action name for evidence."""

    def server_is_ready(self) -> bool:
        """Return whether the action server is currently ready."""

    def start_goal(
        self,
        config: MissionConfig,
        submission_stamp_ns: int,
        feedback_callback: Callable[[int], None],
    ) -> None:
        """Begin one asynchronous goal request."""

    def poll_goal_response(self) -> GoalResponse | None:
        """Return a completed response or None while pending."""

    def accepted_goal_is_active(self) -> bool:
        """Return whether an accepted goal may still require fail-closed cleanup."""

    def poll_accepted_status_stamp_ns(self) -> int | None:
        """Return the exact goal's positive action-status acceptance stamp."""

    def poll_terminal_result(self) -> TerminalActionResult | None:
        """Return the terminal result or None while pending."""

    def request_cancel(self) -> None:
        """Begin asynchronous cancellation of the accepted goal."""

    def poll_cancel_acknowledgement(self) -> bool | None:
        """Return cancellation acknowledgement, or None while pending."""


@dataclass(slots=True)
class ExecutionRecord:
    """Complete bounded mission observation used to build canonical artifacts."""

    action_name: str
    resolved_action_name: str
    started_sim_stamp_ns: int
    started_steady_s: float
    events: list[dict[str, Any]] = field(default_factory=list)
    feedback_trace: list[dict[str, Any]] = field(default_factory=list)
    feedback_trace_overflow: bool = False
    feedback_trace_overflow_count: int = 0
    invalid_feedback_count: int = 0
    goal_submission_stamp_ns: int | None = None
    goal_response_stamp_ns: int | None = None
    accepted_goal_uuid: str | None = None
    accepted_goal_stamp_ns: int | None = None
    accepted_goal_wall_offset_s: float | None = None
    terminal_action_stamp_ns: int | None = None
    terminal_wall_offset_s: float | None = None
    terminal_result: TerminalActionResult | None = None
    cancellation_requested: bool = False
    cancel_acknowledged: bool | None = None
    deadline_kind: str | None = None
    exit_code: ExitCode = ExitCode.INFRASTRUCTURE_ERROR
    reason: str = 'mission_not_started'

    def event(
        self,
        kind: str,
        clock: MissionClock,
        *,
        sim_stamp_ns: int | None = None,
        **details: Any,
    ) -> None:
        """Append a bounded event with both time domains."""
        self.events.append(
            {
                'kind': kind,
                'sim_stamp_ns': clock.sim_time_ns() if sim_stamp_ns is None else sim_stamp_ns,
                'wall_offset_s': max(0.0, clock.steady_time_s() - self.started_steady_s),
                'details': details,
            }
        )


class MissionExecutor:
    """Drive one FollowWaypoints goal without any unbounded wait."""

    def __init__(
        self,
        config: MissionConfig,
        driver: FollowWaypointsDriver,
        clock: MissionClock,
        spin_once: Callable[[float], None],
        runtime_ok: Callable[[], bool] | None = None,
    ) -> None:
        self._config = config
        self._driver = driver
        self._clock = clock
        self._spin_once = spin_once
        self._runtime_ok = runtime_ok or _always_true
        self._record = ExecutionRecord(
            action_name=driver.action_name,
            resolved_action_name=driver.resolved_action_name,
            started_sim_stamp_ns=clock.sim_time_ns(),
            started_steady_s=clock.steady_time_s(),
        )

    def run(self) -> ExecutionRecord:
        """Run one goal through terminal result, cancellation, or bounded failure."""
        accepted_goal_active = False
        try:
            if not self._wait_for_advancing_positive_sim_time():
                return self._finish(
                    ExitCode.INFRASTRUCTURE_ERROR,
                    'simulation_clock_unavailable',
                )
            if not self._wait_for_server():
                return self._finish(
                    ExitCode.INFRASTRUCTURE_ERROR,
                    'action_server_unavailable',
                )

            self._record.goal_submission_stamp_ns = self._clock.sim_time_ns()
            if not _is_positive_time_ns(self._record.goal_submission_stamp_ns):
                raise ActionProtocolError('goal submission requires positive ROS time')
            self._record.event('goal_submitted', self._clock)
            self._driver.start_goal(
                self._config,
                self._record.goal_submission_stamp_ns,
                self._record_feedback,
            )
            goal_response = self._wait_for_value(
                self._driver.poll_goal_response,
                GOAL_RESPONSE_TIMEOUT_WALL_S,
            )
            if goal_response is None:
                return self._finish(
                    ExitCode.INFRASTRUCTURE_ERROR,
                    'goal_response_timeout',
                )
            if not goal_response.accepted:
                self._record.event('goal_rejected', self._clock)
                return self._finish(ExitCode.GOAL_REJECTED, 'goal_rejected')
            accepted_goal_active = True
            if not goal_response.goal_uuid:
                return self._fail_closed_after_acceptance('accepted_goal_missing_uuid')
            if not _is_nonnegative_time_ns(goal_response.response_stamp_ns):
                raise ActionProtocolError('goal response stamp must be a non-negative integer')

            local_acceptance_observed_ns = self._clock.sim_time_ns()
            if not _is_positive_time_ns(local_acceptance_observed_ns):
                raise ActionProtocolError('local ROS time was not positive at goal acceptance')
            assert self._record.goal_submission_stamp_ns is not None
            if local_acceptance_observed_ns < self._record.goal_submission_stamp_ns:
                raise ActionProtocolError('local ROS time regressed after goal submission')
            local_acceptance_observed_wall_s = self._clock.steady_time_s()

            self._record.accepted_goal_uuid = goal_response.goal_uuid
            self._record.goal_response_stamp_ns = goal_response.response_stamp_ns
            self._record.accepted_goal_wall_offset_s = self._wall_offset()
            self._record.event(
                'goal_response_accepted',
                self._clock,
                goal_uuid=goal_response.goal_uuid,
                response_stamp_ns=goal_response.response_stamp_ns,
            )
            self._raise_if_feedback_trace_overflow()

            accepted_status_stamp_ns = self._wait_for_value(
                self._driver.poll_accepted_status_stamp_ns,
                min(
                    ACCEPTED_STATUS_TIMEOUT_WALL_S,
                    self._config.wall_escape_timeout_s,
                ),
                fail_on_feedback_trace_overflow=True,
            )
            if not _is_positive_time_ns(accepted_status_stamp_ns):
                self._record.event('accepted_status_proof_timeout', self._clock)
                return self._fail_closed_after_acceptance('accepted_goal_status_proof_timeout')

            self._record.accepted_goal_stamp_ns = accepted_status_stamp_ns
            self._record.event(
                'goal_accepted',
                self._clock,
                sim_stamp_ns=accepted_status_stamp_ns,
                goal_uuid=goal_response.goal_uuid,
                evidence_source=ACCEPTED_GOAL_STAMP_SOURCE,
            )
            if (
                goal_response.response_stamp_ns > 0
                and goal_response.response_stamp_ns != accepted_status_stamp_ns
            ):
                raise ActionProtocolError(
                    'nonzero goal response stamp does not match action-status acceptance stamp'
                )
            if not self._wait_for_client_clock_to_reach(
                accepted_status_stamp_ns,
                local_acceptance_observed_ns,
            ):
                self._record.event('accepted_stamp_clock_catchup_timeout', self._clock)
                return self._fail_closed_after_acceptance(
                    'client_clock_did_not_reach_accepted_goal_stamp'
                )
            return self._wait_for_terminal_or_deadline(
                local_acceptance_observed_ns,
                local_acceptance_observed_wall_s,
                accepted_status_stamp_ns,
            )
        except FeedbackTraceOverflowError:
            self._record.event(
                'feedback_trace_overflow_failure',
                self._clock,
                feedback_trace_capacity=FEEDBACK_TRACE_CAPACITY,
                feedback_trace_overflow_count=self._record.feedback_trace_overflow_count,
            )
            if accepted_goal_active or self._driver.accepted_goal_is_active():
                return self._fail_closed_after_acceptance('feedback_trace_overflow')
            return self._finish(ExitCode.INFRASTRUCTURE_ERROR, 'feedback_trace_overflow')
        except ActionProtocolError as exc:
            self._record.event('action_protocol_error', self._clock, message=str(exc))
            reason = f'action_protocol_error: {exc}'
            if accepted_goal_active or self._driver.accepted_goal_is_active():
                return self._fail_closed_after_acceptance(reason)
            return self._finish(ExitCode.INFRASTRUCTURE_ERROR, reason)
        except Exception as exc:
            self._record.event('infrastructure_error', self._clock, message=str(exc))
            reason = f'infrastructure_error: {exc}'
            if accepted_goal_active or self._driver.accepted_goal_is_active():
                return self._fail_closed_after_acceptance(reason)
            return self._finish(ExitCode.INFRASTRUCTURE_ERROR, reason)

    def _wait_for_server(self) -> bool:
        deadline = self._clock.steady_time_s() + min(
            ACTION_SERVER_WAIT_WALL_S,
            self._config.wall_escape_timeout_s,
        )
        while self._runtime_ok() and self._clock.steady_time_s() < deadline:
            if self._driver.server_is_ready():
                self._record.event('action_server_ready', self._clock)
                return True
            self._spin_bounded(deadline)
        return False

    def _wait_for_advancing_positive_sim_time(self) -> bool:
        deadline = self._clock.steady_time_s() + min(
            SIM_TIME_READY_WAIT_WALL_S,
            self._config.wall_escape_timeout_s,
        )
        first_positive_ns: int | None = None
        while self._runtime_ok() and self._clock.steady_time_s() < deadline:
            stamp_ns = self._clock.sim_time_ns()
            if _is_positive_time_ns(stamp_ns):
                if first_positive_ns is None:
                    first_positive_ns = stamp_ns
                elif stamp_ns > first_positive_ns:
                    self._record.event(
                        'simulation_clock_ready',
                        self._clock,
                        sim_stamp_ns=stamp_ns,
                        first_positive_stamp_ns=first_positive_ns,
                        second_positive_stamp_ns=stamp_ns,
                    )
                    return True
            self._spin_bounded(deadline)
        return False

    def _wait_for_value(
        self,
        poll: Callable[[], Any | None],
        timeout_wall_s: float,
        *,
        fail_on_feedback_trace_overflow: bool = False,
    ) -> Any | None:
        deadline = self._clock.steady_time_s() + timeout_wall_s
        while self._runtime_ok() and self._clock.steady_time_s() < deadline:
            if fail_on_feedback_trace_overflow:
                self._raise_if_feedback_trace_overflow()
            value = poll()
            if value is not None:
                return value
            self._spin_bounded(deadline)
        if fail_on_feedback_trace_overflow:
            self._raise_if_feedback_trace_overflow()
        return poll() if self._runtime_ok() else None

    def _wait_for_client_clock_to_reach(
        self,
        accepted_status_stamp_ns: int,
        local_acceptance_observed_ns: int,
    ) -> bool:
        deadline = self._clock.steady_time_s() + min(
            ACCEPTED_STATUS_TIMEOUT_WALL_S,
            self._config.wall_escape_timeout_s,
        )
        while self._runtime_ok() and self._clock.steady_time_s() < deadline:
            self._raise_if_feedback_trace_overflow()
            local_stamp_ns = self._clock.sim_time_ns()
            if not _is_positive_time_ns(local_stamp_ns):
                raise ActionProtocolError('local ROS time became non-positive after acceptance')
            if local_stamp_ns < local_acceptance_observed_ns:
                raise ActionProtocolError('local ROS time regressed after goal acceptance')
            if local_stamp_ns >= accepted_status_stamp_ns:
                self._record.event(
                    'accepted_stamp_clock_correlated',
                    self._clock,
                    accepted_goal_stamp_ns=accepted_status_stamp_ns,
                    client_observation_stamp_ns=local_stamp_ns,
                )
                return True
            self._spin_bounded(deadline)
        return False

    def _wait_for_terminal_or_deadline(
        self,
        local_acceptance_observed_ns: int,
        local_acceptance_observed_wall_s: float,
        accepted_status_stamp_ns: int,
    ) -> ExecutionRecord:
        assert _is_positive_time_ns(local_acceptance_observed_ns)

        while self._runtime_ok():
            self._raise_if_feedback_trace_overflow()
            observed_status_stamp_ns = self._driver.poll_accepted_status_stamp_ns()
            if observed_status_stamp_ns != accepted_status_stamp_ns:
                raise ActionProtocolError(
                    'action-status acceptance stamp became unavailable or changed'
                )
            terminal = self._driver.poll_terminal_result()
            if terminal is not None:
                return self._finish_from_terminal(terminal)

            sim_elapsed_ns = self._clock.sim_time_ns() - local_acceptance_observed_ns
            if sim_elapsed_ns < 0:
                return self._finish(
                    ExitCode.INFRASTRUCTURE_ERROR,
                    'simulation_clock_regressed',
                )
            if sim_elapsed_ns >= int(self._config.mission_timeout_sim_s * 1_000_000_000):
                return self._cancel_after_deadline('simulation_timeout')
            if (
                self._clock.steady_time_s() - local_acceptance_observed_wall_s
                >= self._config.wall_escape_timeout_s
            ):
                return self._cancel_after_deadline('wall_escape_timeout')
            self._spin_once(SPIN_POLL_WALL_S)

        return self._finish(ExitCode.INFRASTRUCTURE_ERROR, 'rclpy_context_stopped')

    def _cancel_after_deadline(self, deadline_kind: str) -> ExecutionRecord:
        self._record.deadline_kind = deadline_kind
        self._record.event('deadline_reached', self._clock, deadline_kind=deadline_kind)
        return self._cancel_and_await_terminal(ExitCode.TIMEOUT, deadline_kind)

    def _fail_closed_after_acceptance(self, reason: str) -> ExecutionRecord:
        try:
            return self._cancel_and_await_terminal(ExitCode.INFRASTRUCTURE_ERROR, reason)
        except Exception as exc:
            self._record.event(
                'fail_closed_cancellation_error',
                self._clock,
                message=str(exc),
            )
            return self._finish(
                ExitCode.INFRASTRUCTURE_ERROR,
                f'{reason}: fail_closed_cancellation_error: {exc}',
            )

    def _cancel_and_await_terminal(
        self,
        completed_exit_code: ExitCode,
        reason: str,
    ) -> ExecutionRecord:
        self._record.cancellation_requested = True
        self._driver.request_cancel()
        self._record.event('cancel_requested', self._clock)

        acknowledged = self._wait_for_value(
            self._driver.poll_cancel_acknowledgement,
            CANCEL_ACK_TIMEOUT_WALL_S,
        )
        self._record.cancel_acknowledged = acknowledged
        if acknowledged is not True:
            self._record.event('cancel_not_acknowledged', self._clock)
            return self._finish(
                ExitCode.INFRASTRUCTURE_ERROR,
                f'{reason}: cancellation_not_acknowledged',
            )
        self._record.event('cancel_acknowledged', self._clock)

        terminal = self._wait_for_value(
            self._driver.poll_terminal_result,
            CANCEL_RESULT_TIMEOUT_WALL_S,
        )
        if terminal is None:
            return self._finish(
                ExitCode.INFRASTRUCTURE_ERROR,
                f'{reason}: cancellation_result_timeout',
            )
        self._capture_terminal(terminal)
        return self._finish(completed_exit_code, reason)

    def _finish_from_terminal(self, terminal: TerminalActionResult) -> ExecutionRecord:
        self._capture_terminal(terminal)
        if (
            self._record.accepted_goal_stamp_ns is not None
            and self._record.terminal_action_stamp_ns is not None
            and self._record.terminal_action_stamp_ns < self._record.accepted_goal_stamp_ns
        ):
            return self._finish(
                ExitCode.INFRASTRUCTURE_ERROR,
                'terminal_stamp_precedes_accepted_goal_stamp',
            )
        status = terminal.status_code
        missed = terminal.missed_waypoints
        if status == GoalStatusCode.SUCCEEDED:
            if terminal.error_code != NAV2_ERROR_NONE:
                return self._finish(ExitCode.GOAL_ABORTED, 'succeeded_with_nav2_error')
            if missed:
                return self._finish(ExitCode.GOAL_ABORTED, 'succeeded_with_missed_waypoints')
            if self._record.invalid_feedback_count:
                return self._finish(ExitCode.INFRASTRUCTURE_ERROR, 'invalid_feedback_observed')
            return self._finish(ExitCode.SUCCESS, 'succeeded')
        if status == GoalStatusCode.ABORTED:
            return self._finish(ExitCode.GOAL_ABORTED, 'goal_aborted')
        if status == GoalStatusCode.CANCELED:
            return self._finish(ExitCode.GOAL_CANCELED, 'goal_canceled')
        return self._finish(
            ExitCode.INFRASTRUCTURE_ERROR,
            f'nonterminal_goal_status_{status}',
        )

    def _capture_terminal(self, terminal: TerminalActionResult) -> None:
        self._record.terminal_result = terminal
        self._record.terminal_action_stamp_ns = self._clock.sim_time_ns()
        self._record.terminal_wall_offset_s = self._wall_offset()
        self._record.event(
            'terminal_result',
            self._clock,
            goal_status_code=terminal.status_code,
            goal_status=GOAL_STATUS_NAMES.get(terminal.status_code, 'INVALID'),
            nav2_error_code=terminal.error_code,
            missed_waypoint_count=len(terminal.missed_waypoints),
        )

    def _record_feedback(self, current_waypoint: int) -> None:
        if len(self._record.feedback_trace) >= FEEDBACK_TRACE_CAPACITY:
            self._record.feedback_trace_overflow = True
            self._record.feedback_trace_overflow_count += 1
            if self._record.feedback_trace_overflow_count == 1:
                self._record.event(
                    'feedback_trace_overflow',
                    self._clock,
                    feedback_trace_capacity=FEEDBACK_TRACE_CAPACITY,
                    retained_feedback_count=len(self._record.feedback_trace),
                )
            return
        valid = 0 <= current_waypoint < len(self._config.waypoints)
        if not valid:
            self._record.invalid_feedback_count += 1
        observation = {
            'current_waypoint': current_waypoint,
            'sim_stamp_ns': self._clock.sim_time_ns(),
            'wall_offset_s': self._wall_offset(),
            'valid_index': valid,
        }
        self._record.feedback_trace.append(observation)

    def _raise_if_feedback_trace_overflow(self) -> None:
        if self._record.feedback_trace_overflow:
            raise FeedbackTraceOverflowError('feedback_trace_overflow')

    def _finish(self, exit_code: ExitCode, reason: str) -> ExecutionRecord:
        self._record.exit_code = exit_code
        self._record.reason = reason
        return self._record

    def _wall_offset(self) -> float:
        return max(0.0, self._clock.steady_time_s() - self._record.started_steady_s)

    def _spin_bounded(self, deadline: float) -> None:
        remaining = max(0.0, deadline - self._clock.steady_time_s())
        self._spin_once(min(SPIN_POLL_WALL_S, remaining))


def _always_true() -> bool:
    """Return the default liveness state for transport-independent callers."""
    return True


def _is_positive_time_ns(value: object) -> bool:
    """Return whether *value* is a strict positive integer ROS timestamp."""
    return isinstance(value, int) and not isinstance(value, bool) and value > 0


def _is_nonnegative_time_ns(value: object) -> bool:
    """Return whether *value* is a non-negative integer ROS timestamp."""
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0
