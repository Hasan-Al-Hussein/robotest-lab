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

"""Pure Reset/Preload/UUID-T0/Arm/cancel/reset orchestration tests."""

import pytest
from robotest_missions.execution import (
    ArmResponse,
    ExitCode,
    GoalResponse,
    GoalStatusCode,
    MissionExecutor,
    PreloadResponse,
    ResetResponse,
    TerminalActionResult,
)
from robotest_missions.fault_schedule import build_fault_schedule
from robotest_missions.models import MissionConfig, PlanarPose

UUID = '00000000-0000-0000-0000-000000000001'
T0 = 1_100_000_000


class Clock:
    def __init__(self) -> None:
        self.wall = 0.0
        self.sim = 1_000_000_000

    def sim_time_ns(self) -> int:
        return self.sim

    def steady_time_s(self) -> float:
        return self.wall

    def advance(self, seconds: float) -> None:
        self.wall += seconds
        self.sim += int(seconds * 1_000_000_000)


class ActionDriver:
    action_name = 'follow_waypoints'
    resolved_action_name = '/robotest/follow_waypoints'

    def __init__(self, *, cancel_ack: bool | None = True) -> None:
        self.started = False
        self.accepted = False
        self.canceled = False
        self.spins = 0
        self.feedback_callback = None
        self.cancel_ack = cancel_ack

    def server_is_ready(self) -> bool:
        return True

    def start_goal(self, config, submission_stamp_ns, feedback_callback) -> None:
        self.started = True
        self.feedback_callback = feedback_callback

    def poll_goal_response(self):
        self.accepted = True
        return GoalResponse(True, UUID, 0)

    def accepted_goal_is_active(self) -> bool:
        return self.accepted

    def poll_accepted_status_stamp_ns(self):
        return T0

    def poll_terminal_result(self):
        if self.canceled:
            return TerminalActionResult(int(GoalStatusCode.CANCELED), 0, '', ())
        if self.spins >= 3:
            return TerminalActionResult(int(GoalStatusCode.SUCCEEDED), 0, '', ())
        return None

    def request_cancel(self) -> None:
        self.canceled = True

    def poll_cancel_acknowledgement(self):
        return self.cancel_ack


def _event(sequence: int, event_type: int, **changes):
    value = {
        'schema_version': 2,
        'header_stamp_ns': 0,
        'event_sequence': sequence,
        'event_type': event_type,
        'state_before': 0,
        'state_after': 0,
        'accepted': True,
        'replayed': False,
        'requested_schedule_hash': '',
        'committed_schedule_hash': '',
        'requested_generation': 0,
        'committed_generation': 0,
        'requested_fault_count': 0,
        'committed_fault_count': 0,
        'requested_goal_uuid': '00000000-0000-0000-0000-000000000000',
        'bound_goal_uuid': '00000000-0000-0000-0000-000000000000',
        'requested_t0_ns': 0,
        'bound_t0_ns': 0,
        'arm_commit_stamp_ns': 0,
        'arm_margin_ns': 0,
        'actual_stamp_ns': 0,
    }
    value.update(changes)
    return value


class FaultDriver:
    def __init__(self) -> None:
        self.ready = True
        self.events = []
        self.event_overflow_count = 0
        self.sequence = 0
        self.reset_calls = 0
        self.reset_success = True
        self.reset_timeout = False
        self.preload_accepted = True
        self.preload_timeout = False
        self.preload_replayed = False
        self.arm_accepted = True
        self.arm_timeout = False
        self.arm_replayed = False
        self.arm_margin_override = None
        self.corrupt_preload_event = False
        self.corrupt_arm_event = False
        self.omit_preload_event = False
        self.omit_arm_event = False
        self.sequence_gap = False
        self.schedule = None
        self.generation = 7
        self.reset_response = None
        self.preload_response = None
        self.arm_response = None

    def services_are_ready(self) -> bool:
        return self.ready

    def _next(self) -> int:
        self.sequence += 2 if self.sequence_gap and self.sequence else 1
        return self.sequence

    def start_reset(self) -> None:
        self.reset_calls += 1
        self.reset_response = ResetResponse(self.reset_success, 'reset')
        self.events.append(_event(self._next(), 3))

    def poll_reset_response(self):
        if self.reset_timeout:
            return None
        value, self.reset_response = self.reset_response, None
        return value

    def start_preload(self, schedule) -> None:
        self.schedule = schedule
        self.preload_response = PreloadResponse(
            self.preload_accepted,
            self.preload_replayed,
            1 if self.preload_accepted else 0,
            schedule.sha256,
            len(schedule.faults),
            self.generation if self.preload_accepted else 0,
            'preload',
        )
        event_hash = '0' * 64 if self.corrupt_preload_event else schedule.sha256
        if not self.omit_preload_event:
            self.events.append(
                _event(
                    self._next(),
                    1 if self.preload_accepted else 2,
                    state_after=1 if self.preload_accepted else 0,
                    accepted=self.preload_accepted,
                    replayed=self.preload_replayed,
                    requested_schedule_hash=schedule.sha256,
                    committed_schedule_hash=event_hash,
                    requested_fault_count=len(schedule.faults),
                    committed_fault_count=(len(schedule.faults) if self.preload_accepted else 0),
                    committed_generation=self.generation if self.preload_accepted else 0,
                )
            )

    def poll_preload_response(self):
        if self.preload_timeout:
            return None
        value, self.preload_response = self.preload_response, None
        return value

    def start_arm(self, schedule_hash, generation, goal_uuid, accepted_goal_stamp_ns) -> None:
        earliest = self.schedule.earliest_start_offset_ns
        commit = accepted_goal_stamp_ns + 100_000_000
        margin = 0 if earliest is None else earliest - 100_000_000
        if self.arm_margin_override is not None:
            margin = self.arm_margin_override
            commit = accepted_goal_stamp_ns + (0 if earliest is None else earliest - margin)
        self.arm_response = ArmResponse(
            self.arm_accepted,
            self.arm_replayed,
            2 if self.arm_accepted else 1,
            schedule_hash,
            generation,
            goal_uuid,
            accepted_goal_stamp_ns,
            commit,
            margin,
            'arm',
        )
        if not self.omit_arm_event:
            self.events.append(
                _event(
                    self._next(),
                    7 if self.arm_accepted else 8,
                    state_before=1,
                    state_after=2 if self.arm_accepted else 1,
                    accepted=self.arm_accepted,
                    replayed=self.arm_replayed,
                    requested_schedule_hash=schedule_hash,
                    committed_schedule_hash=schedule_hash,
                    requested_generation=generation,
                    committed_generation=generation,
                    requested_goal_uuid=goal_uuid,
                    bound_goal_uuid=goal_uuid,
                    requested_t0_ns=accepted_goal_stamp_ns,
                    bound_t0_ns=accepted_goal_stamp_ns,
                    arm_commit_stamp_ns=commit,
                    arm_margin_ns=margin,
                    actual_stamp_ns=commit + (1 if self.corrupt_arm_event else 0),
                    header_stamp_ns=commit,
                )
            )

    def poll_arm_response(self):
        if self.arm_timeout:
            return None
        value, self.arm_response = self.arm_response, None
        return value

    def drain_events(self):
        values, self.events = tuple(self.events), []
        return values


def _fault(active: bool):
    if not active:
        return build_fault_schedule([])
    return build_fault_schedule(
        [
            {
                'schema_version': 1,
                'fault_id': 'scan_drop',
                'target': 1,
                'mode': 1,
                'start_offset_ns': 10_000_000_000,
                'duration_ns': 2_000_000_000,
                'seed': 42,
                'parameters': {},
            }
        ]
    )


def _mission(active: bool = False) -> MissionConfig:
    return MissionConfig(
        schema_version=1,
        mission_name='phase3_test',
        mission_seed=42,
        simulator_seed=42,
        frame_id='map',
        start_pose=PlanarPose(0.0, -3.5, 0.0),
        waypoints=(PlanarPose(-2.0, -3.5, 0.0),),
        mission_timeout_sim_s=10.0,
        wall_escape_timeout_s=1.0,
        allowed_collision_count=0,
        fault_schedule=_fault(active),
        fault_seed=42 if active else None,
        expected_outcome='succeeded',
        retries=0,
        scenario_id=4 if active else 1,
        scenario_controller_seed=42,
    )


def _execute(
    fault_driver: FaultDriver,
    *,
    active: bool = False,
    action: ActionDriver | None = None,
):
    action = action or ActionDriver()
    clock = Clock()

    def spin(timeout: float) -> None:
        action.spins += 1
        clock.advance(timeout)

    record = MissionExecutor(
        _mission(active),
        action,
        clock,
        spin,
        fault_driver=fault_driver,
    ).run()
    return record, action


@pytest.mark.parametrize('active', [False, True])
def test_every_phase3_trial_resets_preloads_arms_and_resets(active: bool) -> None:
    fault_driver = FaultDriver()
    record, action = _execute(fault_driver, active=active)
    assert record.exit_code == ExitCode.SUCCESS
    assert action.started is True
    assert fault_driver.reset_calls == 2
    assert record.fault_reset_before_goal is True
    assert record.fault_reset_after_goal is True
    assert record.fault_protocol_status == 'RESET_CONFIRMED_AFTER_GOAL'
    assert record.fault_arm_margin_ns == (9_900_000_000 if active else 0)
    assert [event['event_type'] for event in record.fault_events] == [3, 1, 7, 3]


def test_exact_preload_and_arm_replays_are_retained() -> None:
    driver = FaultDriver()
    driver.preload_replayed = True
    driver.arm_replayed = True
    record, _ = _execute(driver)
    assert record.exit_code == ExitCode.SUCCESS
    assert record.fault_preload_replayed is True
    assert record.fault_arm_replayed is True


@pytest.mark.parametrize(
    ('configure', 'reason_fragment', 'goal_started'),
    [
        (lambda d: setattr(d, 'ready', False), 'services_unavailable', False),
        (lambda d: setattr(d, 'reset_timeout', True), 'pre_goal_reset_failed', False),
        (lambda d: setattr(d, 'reset_success', False), 'pre_goal_reset_failed', False),
        (lambda d: setattr(d, 'preload_timeout', True), 'preload_response_timeout', False),
        (lambda d: setattr(d, 'preload_accepted', False), 'preload_response_mismatch', False),
        (lambda d: setattr(d, 'generation', 0), 'preload_response_mismatch', False),
        (lambda d: setattr(d, 'omit_preload_event', True), 'event_1_timeout', False),
        (lambda d: setattr(d, 'corrupt_preload_event', True), 'preload_event_mismatch', False),
        (lambda d: setattr(d, 'sequence_gap', True), 'event_sequence_gap', False),
        (lambda d: setattr(d, 'arm_timeout', True), 'arm_response_timeout', True),
        (lambda d: setattr(d, 'arm_accepted', False), 'arm_response_mismatch', True),
        (lambda d: setattr(d, 'omit_arm_event', True), 'event_7_timeout', True),
        (lambda d: setattr(d, 'corrupt_arm_event', True), 'arm_event_mismatch', True),
        (lambda d: setattr(d, 'arm_margin_override', 499_999_999), 'margin_below_500ms', True),
        (
            lambda d: setattr(d, 'arm_margin_override', 10_100_000_000),
            'arm_response_mismatch',
            True,
        ),
    ],
)
def test_fault_protocol_failures_are_bounded_and_fail_closed(
    configure, reason_fragment, goal_started
) -> None:
    driver = FaultDriver()
    configure(driver)
    record, action = _execute(driver, active=True)
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert reason_fragment in record.reason
    assert action.started is goal_started
    if goal_started:
        assert record.cancellation_requested is True
    assert driver.reset_calls >= 1


def test_fault_event_subscriber_overflow_fails_before_goal_dispatch() -> None:
    driver = FaultDriver()
    driver.event_overflow_count = 1
    record, action = _execute(driver)
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert 'fault_event_subscriber_overflow' in record.reason
    assert action.started is False
    assert record.fault_event_overflow is True


def test_fault_driver_is_mandatory_for_phase3() -> None:
    action = ActionDriver()
    clock = Clock()
    record = MissionExecutor(_mission(), action, clock, clock.advance).run()
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert 'fault_control_driver_unavailable' in record.reason
    assert action.started is False


def test_post_acceptance_fault_failure_and_cancel_rejection_remain_bounded() -> None:
    fault_driver = FaultDriver()
    fault_driver.arm_accepted = False
    record, action = _execute(
        fault_driver,
        active=True,
        action=ActionDriver(cancel_ack=False),
    )
    assert record.exit_code == ExitCode.INFRASTRUCTURE_ERROR
    assert record.cancel_acknowledged is False
    assert 'cancellation_not_acknowledged' in record.reason
    assert action.canceled is True
    assert fault_driver.reset_calls == 2
