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

"""Pure bounded-state-machine tests with deterministic fake clocks."""

from dataclasses import replace

import pytest
from robotest_missions.execution import (
    FEEDBACK_TRACE_CAPACITY,
    ActionProtocolError,
    ExitCode,
    GoalResponse,
    GoalStatusCode,
    MissedWaypoint,
    MissionExecutor,
    TerminalActionResult,
)
from robotest_missions.models import MissionConfig, PlanarPose

DEFAULT_ACCEPTED_STAMP_NS = 1_000_000_123
DEFAULT_RESPONSE = GoalResponse(
    True,
    '00000000-0000-0000-0000-000000000001',
    0,
)


class FakeClock:
    def __init__(
        self,
        *,
        frozen_sim: bool = False,
        initial_sim_ns: int = 1_000_000_000,
        ready_after_wall_s: float = 0.0,
        freeze_after_wall_s: float | None = None,
    ) -> None:
        self.wall = 0.0
        self.sim_ns = initial_sim_ns
        self.frozen_sim = frozen_sim
        self.ready_after_wall_s = ready_after_wall_s
        self.freeze_after_wall_s = freeze_after_wall_s

    def sim_time_ns(self) -> int:
        return 0 if self.wall < self.ready_after_wall_s else self.sim_ns

    def steady_time_s(self) -> float:
        return self.wall

    def advance(self, seconds: float) -> None:
        previous_wall = self.wall
        self.wall += seconds
        if self.frozen_sim:
            return
        advancing_from = max(previous_wall, self.ready_after_wall_s)
        advancing_until = self.wall
        if self.freeze_after_wall_s is not None:
            advancing_until = min(advancing_until, self.freeze_after_wall_s)
        if advancing_until > advancing_from:
            self.sim_ns += int((advancing_until - advancing_from) * 1_000_000_000)


class FakeDriver:
    action_name = 'follow_waypoints'
    resolved_action_name = '/robotest/follow_waypoints'

    def __init__(
        self,
        *,
        response: GoalResponse | None = DEFAULT_RESPONSE,
        terminal: TerminalActionResult | None = None,
        terminal_at_spin: int | None = None,
        cancel_ack: bool | None = True,
        cancel_terminal: TerminalActionResult | None = None,
        status_stamp_ns: int | None = DEFAULT_ACCEPTED_STAMP_NS,
        status_error: str | None = None,
        status_error_after_goal_spins: int | None = None,
    ) -> None:
        self.ready = True
        self.response = response
        self.terminal = terminal
        self.terminal_at_spin = terminal_at_spin
        self.cancel_ack = cancel_ack
        self.cancel_terminal = cancel_terminal
        self.status_stamp_ns = status_stamp_ns
        self.status_error = status_error
        self.status_error_after_goal_spins = status_error_after_goal_spins
        self.feedback_callback = None
        self.spins = 0
        self.cancel_requested = False
        self.submission_stamps: list[int] = []
        self.goal_start_spin: int | None = None
        self.feedback_sent = False
        self.goal_accepted = False

    def server_is_ready(self) -> bool:
        return self.ready

    def start_goal(self, config, submission_stamp_ns, feedback_callback) -> None:
        assert config.frame_id == 'map'
        assert submission_stamp_ns > 0
        self.submission_stamps.append(submission_stamp_ns)
        self.goal_start_spin = self.spins
        self.feedback_callback = feedback_callback

    def poll_goal_response(self) -> GoalResponse | None:
        if self.response is not None and self.response.accepted:
            self.goal_accepted = True
        return self.response

    def accepted_goal_is_active(self) -> bool:
        return self.goal_accepted

    def poll_accepted_status_stamp_ns(self) -> int | None:
        goal_spins = 0 if self.goal_start_spin is None else self.spins - self.goal_start_spin
        if self.status_error is not None and (
            self.status_error_after_goal_spins is None
            or goal_spins >= self.status_error_after_goal_spins
        ):
            raise ActionProtocolError(self.status_error)
        return self.status_stamp_ns

    def poll_terminal_result(self) -> TerminalActionResult | None:
        if self.cancel_requested:
            return self.cancel_terminal
        goal_spins = 0 if self.goal_start_spin is None else self.spins - self.goal_start_spin
        if self.terminal_at_spin is not None and goal_spins >= self.terminal_at_spin:
            return self.terminal
        return None

    def request_cancel(self) -> None:
        self.cancel_requested = True

    def poll_cancel_acknowledgement(self) -> bool | None:
        return self.cancel_ack


def mission() -> MissionConfig:
    return MissionConfig(
        schema_version=1,
        mission_name='test_mission',
        mission_seed=42,
        simulator_seed=42,
        frame_id='map',
        start_pose=PlanarPose(0.0, -3.5, 0.0),
        waypoints=(
            PlanarPose(-2.0, -3.5, 0.0),
            PlanarPose(0.0, 0.0, 0.0),
            PlanarPose(0.0, 3.5, 0.0),
        ),
        mission_timeout_sim_s=1.0,
        wall_escape_timeout_s=1.0,
        allowed_collision_count=0,
        fault_schedule=None,
        fault_seed=None,
        expected_outcome='succeeded',
        retries=0,
    )


def result(status: GoalStatusCode, *, error: int = 0) -> TerminalActionResult:
    return TerminalActionResult(
        status_code=int(status),
        error_code=error,
        error_message='' if error == 0 else 'navigation failed',
        missed_waypoints=(),
    )


def execute(config: MissionConfig, driver: FakeDriver, clock: FakeClock):
    def spin(timeout: float) -> None:
        driver.spins += 1
        clock.advance(timeout)
        if driver.feedback_callback is not None and not driver.feedback_sent:
            driver.feedback_sent = True
            driver.feedback_callback(0)

    return MissionExecutor(config, driver, clock, spin).run()


def test_success_records_uuid_feedback_and_terminal_stamps() -> None:
    driver = FakeDriver(
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=2,
    )
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.SUCCESS
    assert record.accepted_goal_uuid == '00000000-0000-0000-0000-000000000001'
    assert record.accepted_goal_stamp_ns == DEFAULT_ACCEPTED_STAMP_NS
    assert record.goal_response_stamp_ns == 0
    assert record.terminal_action_stamp_ns == 1_150_000_000
    accepted_event = next(event for event in record.events if event['kind'] == 'goal_accepted')
    assert accepted_event['sim_stamp_ns'] == DEFAULT_ACCEPTED_STAMP_NS
    assert record.feedback_trace[0]['current_waypoint'] == 0


def test_delayed_simulation_clock_blocks_dispatch_until_positive() -> None:
    driver = FakeDriver(
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=5,
    )
    clock = FakeClock(ready_after_wall_s=0.15)
    record = execute(mission(), driver, clock)
    assert record.exit_code == ExitCode.SUCCESS
    assert len(driver.submission_stamps) == 1
    assert driver.submission_stamps[0] > 0
    assert [event['kind'] for event in record.events[:3]] == [
        'simulation_clock_ready',
        'action_server_ready',
        'goal_submitted',
    ]
    readiness = record.events[0]['details']
    assert readiness['second_positive_stamp_ns'] > readiness['first_positive_stamp_ns'] > 0


def test_never_positive_simulation_clock_fails_bounded_without_dispatch() -> None:
    driver = FakeDriver()
    clock = FakeClock(frozen_sim=True, initial_sim_ns=0)
    record = execute(replace(mission(), wall_escape_timeout_s=0.2), driver, clock)
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'simulation_clock_unavailable'
    assert driver.submission_stamps == []
    assert driver.spins < 10


def test_positive_but_frozen_simulation_clock_does_not_dispatch() -> None:
    driver = FakeDriver()
    clock = FakeClock(frozen_sim=True, initial_sim_ns=1_000_000_000)
    record = execute(replace(mission(), wall_escape_timeout_s=0.2), driver, clock)
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'simulation_clock_unavailable'
    assert driver.submission_stamps == []


@pytest.mark.parametrize('stamp_ns', [None, -1, True])
def test_invalid_raw_goal_response_stamp_is_protocol_failure(stamp_ns) -> None:
    driver = FakeDriver(
        response=GoalResponse(
            True,
            '00000000-0000-0000-0000-000000000001',
            stamp_ns,
        ),
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason.startswith('action_protocol_error:')
    assert record.accepted_goal_stamp_ns is None


def test_small_server_stamp_skew_waits_for_client_clock_without_false_regression() -> None:
    server_stamp_ns = 1_100_000_000
    driver = FakeDriver(
        response=GoalResponse(
            True,
            '00000000-0000-0000-0000-000000000001',
            0,
        ),
        status_stamp_ns=server_stamp_ns,
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=1,
    )
    record = execute(mission(), driver, FakeClock(initial_sim_ns=1_000_000_000))
    assert record.exit_code == ExitCode.SUCCESS
    assert record.accepted_goal_stamp_ns == server_stamp_ns
    assert record.reason == 'succeeded'


def test_terminal_clock_cannot_remain_behind_server_t0_on_success() -> None:
    driver = FakeDriver(
        status_stamp_ns=9_000_000_000,
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(replace(mission(), wall_escape_timeout_s=0.2), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'client_clock_did_not_reach_accepted_goal_stamp'
    assert record.cancellation_requested is True
    assert record.terminal_action_stamp_ns < record.accepted_goal_stamp_ns


def test_positive_response_stamp_mismatch_fails_closed() -> None:
    driver = FakeDriver(
        response=GoalResponse(
            True,
            '00000000-0000-0000-0000-000000000001',
            1_200_000_000,
        ),
        status_stamp_ns=1_100_000_000,
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert 'does not match action-status acceptance stamp' in record.reason
    assert record.cancellation_requested is True


def test_late_status_stamp_conflict_is_polled_before_terminal_success() -> None:
    driver = FakeDriver(
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=2,
        status_error='conflicting action-status acceptance stamps for goal UUID',
        status_error_after_goal_spins=1,
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason.startswith('action_protocol_error:')
    assert record.cancellation_requested is True


def test_missing_status_proof_cancels_goal_before_bounded_failure() -> None:
    driver = FakeDriver(
        status_stamp_ns=None,
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(replace(mission(), wall_escape_timeout_s=0.2), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'accepted_goal_status_proof_timeout'
    assert record.cancellation_requested is True
    assert record.cancel_acknowledged is True
    assert record.terminal_result.status_code == GoalStatusCode.CANCELED
    assert record.accepted_goal_stamp_ns is None


def test_post_acceptance_protocol_error_fails_closed_with_cancellation() -> None:
    driver = FakeDriver(
        status_error='conflicting nonzero action-status acceptance stamps',
        cancel_terminal=result(GoalStatusCode.CANCELED),
    )
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason.startswith('action_protocol_error:')
    assert record.cancellation_requested is True
    assert record.cancel_acknowledged is True
    assert record.terminal_result.status_code == GoalStatusCode.CANCELED


@pytest.mark.parametrize(
    ('driver', 'expected'),
    [
        (FakeDriver(response=GoalResponse(False)), ExitCode.GOAL_REJECTED),
        (
            FakeDriver(terminal=result(GoalStatusCode.ABORTED), terminal_at_spin=1),
            ExitCode.GOAL_ABORTED,
        ),
        (
            FakeDriver(terminal=result(GoalStatusCode.CANCELED), terminal_at_spin=1),
            ExitCode.GOAL_CANCELED,
        ),
        (
            FakeDriver(
                terminal=result(GoalStatusCode.SUCCEEDED, error=5),
                terminal_at_spin=1,
            ),
            ExitCode.GOAL_ABORTED,
        ),
    ],
)
def test_terminal_and_rejection_exit_mapping(driver: FakeDriver, expected: ExitCode) -> None:
    assert execute(mission(), driver, FakeClock()).exit_code == expected


def test_frozen_simulation_clock_uses_wall_escape_and_bounded_cancel() -> None:
    config = replace(mission(), mission_timeout_sim_s=10.0, wall_escape_timeout_s=0.2)
    driver = FakeDriver(cancel_terminal=result(GoalStatusCode.CANCELED))
    record = execute(config, driver, FakeClock(freeze_after_wall_s=0.05))
    assert record.exit_code == ExitCode.TIMEOUT
    assert record.deadline_kind == 'wall_escape_timeout'
    assert record.cancel_acknowledged is True
    assert driver.cancel_requested


def test_simulation_deadline_uses_bounded_cancel() -> None:
    config = replace(mission(), mission_timeout_sim_s=0.1, wall_escape_timeout_s=10.0)
    driver = FakeDriver(cancel_terminal=result(GoalStatusCode.CANCELED))
    record = execute(config, driver, FakeClock())
    assert record.exit_code == ExitCode.TIMEOUT
    assert record.deadline_kind == 'simulation_timeout'


def test_timeout_cancel_not_acknowledged_is_infrastructure_error() -> None:
    config = replace(mission(), wall_escape_timeout_s=0.1)
    driver = FakeDriver(cancel_ack=False)
    record = execute(config, driver, FakeClock(freeze_after_wall_s=0.05))
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.cancel_acknowledged is False
    assert record.reason.endswith('cancellation_not_acknowledged')


def test_timeout_cancel_result_missing_is_infrastructure_error() -> None:
    config = replace(mission(), wall_escape_timeout_s=0.1)
    driver = FakeDriver(cancel_ack=True, cancel_terminal=None)
    record = execute(config, driver, FakeClock(freeze_after_wall_s=0.05))
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason.endswith('cancellation_result_timeout')
    assert driver.spins < 400


def test_invalid_feedback_prevents_false_success() -> None:
    driver = FakeDriver(
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=2,
    )

    def spin_with_invalid_feedback(timeout: float) -> None:
        driver.spins += 1
        clock.advance(timeout)
        if driver.feedback_callback is not None and not driver.feedback_sent:
            driver.feedback_sent = True
            driver.feedback_callback(99)

    clock = FakeClock()
    record = MissionExecutor(mission(), driver, clock, spin_with_invalid_feedback).run()
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.invalid_feedback_count == 1


def test_feedback_trace_at_fixed_capacity_can_complete() -> None:
    driver = FakeDriver(
        terminal=result(GoalStatusCode.SUCCEEDED),
        terminal_at_spin=1,
    )

    waypoint_count = len(mission().waypoints)

    def spin_with_capacity_feedback(timeout: float) -> None:
        driver.spins += 1
        clock.advance(timeout)
        if driver.feedback_callback is not None and not driver.feedback_sent:
            driver.feedback_sent = True
            for index in range(FEEDBACK_TRACE_CAPACITY):
                driver.feedback_callback(index % waypoint_count)

    clock = FakeClock()
    record = MissionExecutor(mission(), driver, clock, spin_with_capacity_feedback).run()
    assert record.exit_code == ExitCode.SUCCESS
    assert len(record.feedback_trace) == FEEDBACK_TRACE_CAPACITY
    assert record.feedback_trace_overflow is False
    assert record.feedback_trace_overflow_count == 0


def test_feedback_trace_overflow_fails_closed_with_bounded_cancel() -> None:
    driver = FakeDriver(cancel_terminal=result(GoalStatusCode.CANCELED))
    waypoint_count = len(mission().waypoints)

    def spin_with_excess_feedback(timeout: float) -> None:
        driver.spins += 1
        clock.advance(timeout)
        if driver.feedback_callback is not None and not driver.feedback_sent:
            driver.feedback_sent = True
            for index in range(FEEDBACK_TRACE_CAPACITY + 1):
                driver.feedback_callback(index % waypoint_count)

    clock = FakeClock()
    record = MissionExecutor(mission(), driver, clock, spin_with_excess_feedback).run()
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'feedback_trace_overflow'
    assert record.cancellation_requested is True
    assert record.cancel_acknowledged is True
    assert record.terminal_result.status_code == GoalStatusCode.CANCELED
    assert len(record.feedback_trace) == FEEDBACK_TRACE_CAPACITY
    assert record.feedback_trace_overflow is True
    assert record.feedback_trace_overflow_count == 1
    overflow_events = [
        event for event in record.events if event['kind'] == 'feedback_trace_overflow'
    ]
    assert len(overflow_events) == 1
    assert overflow_events[0]['details']['feedback_trace_capacity'] == 4096


def test_succeeded_status_with_missed_waypoint_is_logical_failure() -> None:
    terminal = TerminalActionResult(
        status_code=int(GoalStatusCode.SUCCEEDED),
        error_code=0,
        error_message='',
        missed_waypoints=(MissedWaypoint(index=1, error_code=601, goal={'frame_id': 'map'}),),
    )
    driver = FakeDriver(terminal=terminal, terminal_at_spin=1)
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.GOAL_ABORTED
    assert record.terminal_result.missed_waypoints[0].index == 1


def test_action_server_wait_is_bounded_by_wall_escape() -> None:
    driver = FakeDriver()
    driver.ready = False
    record = execute(replace(mission(), wall_escape_timeout_s=0.1), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'action_server_unavailable'
    assert driver.spins < 10


def test_goal_response_wait_is_bounded() -> None:
    driver = FakeDriver(response=None)
    record = execute(mission(), driver, FakeClock())
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.reason == 'goal_response_timeout'
    assert driver.spins < 250
