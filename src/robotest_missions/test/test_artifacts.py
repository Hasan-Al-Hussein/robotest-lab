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

"""Canonical JSON and one-row CSV reconciliation tests."""

import csv
import json
from pathlib import Path

import pytest
from robotest_missions.artifacts import (
    CSV_FIELDS,
    ArtifactError,
    build_result,
    reconcile_artifacts,
    validate_artifact_paths,
    write_result_artifacts,
)
from robotest_missions.execution import (
    ExecutionRecord,
    ExitCode,
    GoalStatusCode,
    TerminalActionResult,
)

from robotest_missions.schema import load_mission

REPOSITORY = Path(__file__).resolve().parents[3]
BASELINE = REPOSITORY / 'scenarios' / 'phase2_baseline.yaml'


def canonical_result(
    *,
    status: GoalStatusCode = GoalStatusCode.SUCCEEDED,
    exit_code: ExitCode = ExitCode.SUCCESS,
) -> dict:
    record = ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=10.0,
        goal_submission_stamp_ns=1_000_000_000,
        goal_response_stamp_ns=0,
        accepted_goal_uuid='00000000-0000-0000-0000-000000000001',
        accepted_goal_stamp_ns=2_000_000_000,
        accepted_goal_wall_offset_s=0.2,
        terminal_action_stamp_ns=5_000_000_000,
        terminal_wall_offset_s=3.4,
        terminal_result=TerminalActionResult(
            status_code=int(status),
            error_code=0,
            error_message='',
            missed_waypoints=(),
        ),
        exit_code=exit_code,
        reason='succeeded' if exit_code == ExitCode.SUCCESS else 'goal_aborted',
    )
    return build_result(
        load_mission(BASELINE),
        record,
        run_id='phase2-mission-test',
        created_utc='2026-08-26T00:00:00.000000Z',
    )


def test_atomic_json_and_csv_round_trip(tmp_path: Path) -> None:
    json_path = tmp_path / 'result.json'
    csv_path = tmp_path / 'result.csv'
    write_result_artifacts(canonical_result(), json_path, csv_path)
    payload = json.loads(json_path.read_text(encoding='utf-8'))
    with csv_path.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert tuple(rows[0]) == CSV_FIELDS
    assert rows[0]['run_id'] == payload['identity']['run_id']
    assert rows[0]['fault_seed'] == ''
    assert rows[0]['goal_submission_stamp_ns'] == '1000000000'
    assert rows[0]['goal_response_stamp_ns'] == '0'
    assert payload['measurements']['accepted_goal_stamp_ns'] == 2_000_000_000
    assert rows[0]['accepted_goal_stamp_source'] == 'uuid_matched_action_status_goal_info'
    assert payload['targets']['feedback_trace_capacity'] == 4096
    assert payload['quality']['feedback_trace_overflow'] is False
    assert payload['quality']['feedback_trace_overflow_count'] == 0
    assert rows[0]['feedback_trace_capacity'] == '4096'
    assert rows[0]['feedback_trace_overflow'] == 'false'
    assert rows[0]['feedback_trace_overflow_count'] == '0'
    assert rows[0]['scenario1_acceptance_status'].startswith('NOT_EVALUATED')


def test_reconciliation_detects_modified_csv(tmp_path: Path) -> None:
    json_path = tmp_path / 'result.json'
    csv_path = tmp_path / 'result.csv'
    write_result_artifacts(canonical_result(), json_path, csv_path)
    text = csv_path.read_text(encoding='utf-8').replace('succeeded', 'tampered', 1)
    csv_path.write_text(text, encoding='utf-8')
    with pytest.raises(ArtifactError, match='does not match'):
        reconcile_artifacts(json_path, csv_path)


@pytest.mark.parametrize(
    ('json_name', 'csv_name'),
    [('same.json', 'same.json'), ('result.txt', 'result.csv'), ('result.json', 'result.txt')],
)
def test_artifact_paths_require_distinct_json_and_csv(tmp_path, json_name, csv_name) -> None:
    with pytest.raises(ArtifactError):
        validate_artifact_paths(tmp_path / json_name, tmp_path / csv_name)


def test_nonfinite_result_is_rejected_before_write(tmp_path: Path) -> None:
    result = canonical_result()
    result['measurements']['completion_time_sim_s'] = float('nan')
    with pytest.raises(ValueError):
        write_result_artifacts(result, tmp_path / 'result.json', tmp_path / 'result.csv')
    assert not (tmp_path / 'result.json').exists()


def test_failed_goal_does_not_invent_completed_waypoint_count() -> None:
    result = canonical_result(status=GoalStatusCode.ABORTED, exit_code=ExitCode.GOAL_ABORTED)
    assert result['measurements']['completed_waypoint_count'] is None


def test_success_artifact_rejects_terminal_stamp_before_authoritative_t0() -> None:
    record = ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=10.0,
        accepted_goal_stamp_ns=9_000_000_000,
        terminal_action_stamp_ns=1_100_000_000,
        terminal_result=TerminalActionResult(
            status_code=int(GoalStatusCode.SUCCEEDED),
            error_code=0,
            error_message='',
            missed_waypoints=(),
        ),
        exit_code=ExitCode.SUCCESS,
        reason='succeeded',
    )
    with pytest.raises(ArtifactError, match='ordered timestamps'):
        build_result(
            load_mission(BASELINE),
            record,
            run_id='phase2-mission-invalid-order',
            created_utc='2026-08-26T00:00:00.000000Z',
        )


def test_success_artifact_rejects_feedback_trace_overflow() -> None:
    record = ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=10.0,
        accepted_goal_stamp_ns=2_000_000_000,
        terminal_action_stamp_ns=5_000_000_000,
        terminal_result=TerminalActionResult(
            status_code=int(GoalStatusCode.SUCCEEDED),
            error_code=0,
            error_message='',
            missed_waypoints=(),
        ),
        feedback_trace_overflow=True,
        feedback_trace_overflow_count=1,
        feedback_trace=[{}] * 4096,
        exit_code=ExitCode.SUCCESS,
        reason='succeeded',
    )
    with pytest.raises(ArtifactError, match='bounded feedback trace'):
        build_result(
            load_mission(BASELINE),
            record,
            run_id='phase2-mission-overflow',
            created_utc='2026-08-26T00:00:00.000000Z',
        )


def test_artifact_rejects_inconsistent_feedback_overflow_evidence() -> None:
    record = ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=10.0,
        feedback_trace_overflow=True,
        feedback_trace_overflow_count=0,
        exit_code=ExitCode.INFRASTRUCTURE_ERROR,
        reason='feedback_trace_overflow',
    )
    with pytest.raises(ArtifactError, match='internally inconsistent'):
        build_result(
            load_mission(BASELINE),
            record,
            run_id='phase2-mission-inconsistent-overflow',
            created_utc='2026-08-26T00:00:00.000000Z',
        )
