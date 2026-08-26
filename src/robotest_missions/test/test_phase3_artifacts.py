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

"""Canonical Phase 3 mission/fault artifact projection tests."""

import csv
import json
from pathlib import Path

import pytest
import robotest_missions.artifacts as artifacts
from robotest_missions.artifacts import ArtifactError, build_result, write_result_artifacts
from robotest_missions.execution import (
    ExecutionRecord,
    ExitCode,
    GoalStatusCode,
    TerminalActionResult,
)

from robotest_missions.schema import load_mission

REPOSITORY = Path(__file__).resolve().parents[3]
SCENARIO = REPOSITORY / 'scenarios' / 'phase3_s4_lidar_dropout.yaml'
IDENTITY = {
    'candidate_id': 'candidate-a',
    'repetition_index': 1,
    'run_id': 'p3-candidate-a-s04-r1',
    'scenario_id': 4,
    'suite_index': 10,
}


def _record() -> ExecutionRecord:
    return ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=1.0,
        goal_submission_stamp_ns=1_100_000_000,
        goal_response_stamp_ns=0,
        accepted_goal_uuid='00000000-0000-0000-0000-000000000001',
        accepted_goal_stamp_ns=1_200_000_000,
        accepted_goal_wall_offset_s=0.2,
        terminal_action_stamp_ns=5_200_000_000,
        terminal_wall_offset_s=4.2,
        terminal_result=TerminalActionResult(int(GoalStatusCode.SUCCEEDED), 0, '', ()),
        fault_events=[{'event_sequence': value} for value in range(1, 5)],
        fault_schedule_hash='5f93838ca7c0be214858ffe8f62fa351b82d260f6223f139dfaf6bd3dcd224c8',
        fault_generation=7,
        fault_preload_replayed=False,
        fault_arm_replayed=False,
        fault_arm_commit_stamp_ns=1_300_000_000,
        fault_arm_margin_ns=9_900_000_000,
        fault_reset_before_goal=True,
        fault_reset_after_goal=True,
        fault_protocol_status='RESET_CONFIRMED_AFTER_GOAL',
        exit_code=ExitCode.SUCCESS,
        reason='succeeded',
    )


def _result() -> dict:
    return build_result(
        load_mission(SCENARIO),
        _record(),
        run_id=IDENTITY['run_id'],
        created_utc='2026-08-26T00:00:00.000000Z',
        trial_identity=IDENTITY,
    )


def test_phase3_json_and_csv_reconcile_fault_and_trial_identity(tmp_path: Path) -> None:
    result = _result()
    json_path = tmp_path / 'mission-result.json'
    csv_path = tmp_path / 'mission-result.csv'
    write_result_artifacts(result, json_path, csv_path)
    stored = json.loads(json_path.read_text(encoding='utf-8'))
    with csv_path.open(encoding='utf-8', newline='') as stream:
        row = next(csv.DictReader(stream))
    assert stored['identity']['suite_index'] == 10
    assert stored['fault']['schedule']['fault_count'] == 1
    assert stored['fault']['control']['arm_margin_ns'] == 9_900_000_000
    assert stored['fault']['control']['event_trace_capacity'] == 512
    assert row['candidate_id'] == 'candidate-a'
    assert row['fault_generation'] == '7'
    assert row['fault_reset_after_goal'] == 'true'
    assert row['fault_event_count'] == '4'


@pytest.mark.parametrize(
    'mutation',
    [
        lambda record: setattr(record, 'fault_reset_before_goal', False),
        lambda record: setattr(record, 'fault_reset_after_goal', False),
        lambda record: setattr(record, 'fault_event_overflow', True),
        lambda record: setattr(record, 'fault_protocol_status', 'ARMED'),
    ],
)
def test_phase3_success_rejects_incomplete_fault_proof(mutation) -> None:
    record = _record()
    mutation(record)
    with pytest.raises(ArtifactError, match='complete bounded fault proof'):
        build_result(
            load_mission(SCENARIO),
            record,
            run_id=IDENTITY['run_id'],
            created_utc='2026-08-26T00:00:00.000000Z',
            trial_identity=IDENTITY,
        )


def test_phase3_requires_trial_identity() -> None:
    with pytest.raises(ArtifactError, match='trial identity'):
        build_result(
            load_mission(SCENARIO),
            _record(),
            run_id=IDENTITY['run_id'],
            created_utc='2026-08-26T00:00:00.000000Z',
        )


def test_artifact_caps_fail_before_any_write(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.setattr(artifacts, 'MAX_JSON_BYTES', 1)
    json_path = tmp_path / 'mission-result.json'
    csv_path = tmp_path / 'mission-result.csv'
    with pytest.raises(ArtifactError, match='JSON exceeds'):
        write_result_artifacts(_result(), json_path, csv_path)
    assert not json_path.exists()
    assert not csv_path.exists()
