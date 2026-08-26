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

"""Canonical mission JSON and matching one-row CSV artifacts."""

from __future__ import annotations

import csv
import json
import os
import tempfile
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from robotest_missions.execution import (
    ACCEPTED_GOAL_STAMP_SOURCE,
    ACCEPTED_STATUS_TIMEOUT_WALL_S,
    ACTION_SERVER_WAIT_WALL_S,
    CANCEL_ACK_TIMEOUT_WALL_S,
    CANCEL_RESULT_TIMEOUT_WALL_S,
    FEEDBACK_TRACE_CAPACITY,
    GOAL_RESPONSE_TIMEOUT_WALL_S,
    GOAL_STATUS_NAMES,
    SIM_TIME_READY_WAIT_WALL_S,
    ExecutionRecord,
    ExitCode,
    GoalStatusCode,
)
from robotest_missions.models import MissionDocument

ACTION_TYPE = 'nav2_msgs/action/FollowWaypoints'
SCENARIO1_NOT_EVALUATED = 'NOT_EVALUATED_PHASE2_MISSION_ACTION_ONLY'
CSV_FIELDS = (
    'run_id',
    'created_utc',
    'mission_name',
    'mission_file',
    'mission_sha256',
    'mission_seed',
    'simulator_seed',
    'fault_schedule_hash',
    'fault_seed',
    'action_name',
    'resolved_action_name',
    'waypoint_count',
    'goal_submission_stamp_ns',
    'accepted_goal_uuid',
    'accepted_goal_stamp_source',
    'goal_response_stamp_ns',
    'accepted_goal_stamp_ns',
    'terminal_action_stamp_ns',
    'completion_time_sim_s',
    'mission_wall_duration_s',
    'feedback_count',
    'feedback_trace_capacity',
    'feedback_trace_overflow',
    'feedback_trace_overflow_count',
    'completed_waypoint_count',
    'missed_waypoint_count',
    'goal_status_code',
    'goal_status',
    'nav2_error_code',
    'nav2_error_message',
    'cancellation_requested',
    'cancel_acknowledged',
    'terminal_result_observed',
    'deadline_kind',
    'expected_outcome',
    'expected_outcome_met',
    'phase2_action_integration_status',
    'scenario1_acceptance_status',
    'exit_code',
    'reason',
)


class ArtifactError(RuntimeError):
    """Raised when canonical artifacts cannot be written or reconciled."""


def _stamp(stamp_ns: int | None) -> dict[str, int] | None:
    if stamp_ns is None:
        return None
    seconds, nanoseconds = divmod(stamp_ns, 1_000_000_000)
    return {'sec': seconds, 'nanosec': nanoseconds, 'nanoseconds': stamp_ns}


def _goal_status(record: ExecutionRecord) -> tuple[int | None, str | None]:
    if record.terminal_result is None:
        return None, None
    code = record.terminal_result.status_code
    return code, GOAL_STATUS_NAMES.get(code, 'INVALID')


def _completed_waypoint_count(document: MissionDocument, record: ExecutionRecord) -> int | None:
    terminal = record.terminal_result
    if terminal is None:
        return None
    if terminal.status_code == GoalStatusCode.SUCCEEDED and not terminal.missed_waypoints:
        return len(document.config.waypoints)
    # FollowWaypoints does not report a normative completed-count field. A
    # terminal failure or missed-waypoint list is insufficient to reconstruct
    # one without overclaiming, so preserve the observation as unknown.
    return None


def build_result(
    document: MissionDocument,
    record: ExecutionRecord,
    *,
    run_id: str,
    created_utc: str,
) -> dict[str, Any]:
    """Build the canonical Phase 2 mission-action result."""
    config = document.config
    terminal = record.terminal_result
    status_code, status_name = _goal_status(record)
    if len(record.feedback_trace) > FEEDBACK_TRACE_CAPACITY:
        raise ArtifactError('feedback trace exceeds its fixed capacity')
    if record.feedback_trace_overflow:
        if (
            len(record.feedback_trace) != FEEDBACK_TRACE_CAPACITY
            or record.feedback_trace_overflow_count < 1
        ):
            raise ArtifactError('feedback overflow evidence is internally inconsistent')
    elif record.feedback_trace_overflow_count != 0:
        raise ArtifactError('feedback overflow count requires an overflow verdict')
    if record.exit_code == ExitCode.SUCCESS and (
        record.accepted_goal_stamp_ns is None
        or record.terminal_action_stamp_ns is None
        or record.terminal_action_stamp_ns < record.accepted_goal_stamp_ns
        or record.feedback_trace_overflow
    ):
        raise ArtifactError(
            'successful execution requires ordered timestamps and a bounded feedback trace'
        )
    completion_time_sim_s = None
    if record.accepted_goal_stamp_ns is not None and record.terminal_action_stamp_ns is not None:
        elapsed_ns = record.terminal_action_stamp_ns - record.accepted_goal_stamp_ns
        if elapsed_ns >= 0:
            completion_time_sim_s = elapsed_ns / 1_000_000_000
    mission_wall_duration_s = None
    accepted_wall = record.accepted_goal_wall_offset_s
    terminal_wall = record.terminal_wall_offset_s
    if accepted_wall is not None and terminal_wall is not None:
        mission_wall_duration_s = max(
            0.0,
            terminal_wall - accepted_wall,
        )

    missed = [] if terminal is None else [item.as_dict() for item in terminal.missed_waypoints]
    succeeded = record.exit_code == ExitCode.SUCCESS
    result = {
        'identity': {
            'action_name': record.action_name,
            'action_type': ACTION_TYPE,
            'created_utc': created_utc,
            'fault_schedule_hash': None,
            'fault_seed': config.fault_seed,
            'mission_file': str(document.source_path),
            'mission_name': config.mission_name,
            'mission_seed': config.mission_seed,
            'mission_sha256': document.source_sha256,
            'resolved_action_name': record.resolved_action_name,
            'run_id': run_id,
            'schema_sha256': document.schema_sha256,
            'simulator_seed': config.simulator_seed,
            'verification_level': 'L3 bounded local simulation action observation',
        },
        'targets': {
            'action_server_wait_wall_s': ACTION_SERVER_WAIT_WALL_S,
            'allowed_collision_count': config.allowed_collision_count,
            'accepted_status_timeout_wall_s': ACCEPTED_STATUS_TIMEOUT_WALL_S,
            'cancel_ack_timeout_wall_s': CANCEL_ACK_TIMEOUT_WALL_S,
            'cancel_result_timeout_wall_s': CANCEL_RESULT_TIMEOUT_WALL_S,
            'expected_outcome': config.expected_outcome,
            'feedback_trace_capacity': FEEDBACK_TRACE_CAPACITY,
            'frame_id': config.frame_id,
            'goal_index': 0,
            'goal_response_timeout_wall_s': GOAL_RESPONSE_TIMEOUT_WALL_S,
            'mission_timeout_sim_s': config.mission_timeout_sim_s,
            'number_of_loops': 0,
            'retries': config.retries,
            'sim_time_ready_wait_wall_s': SIM_TIME_READY_WAIT_WALL_S,
            'start_pose': config.start_pose.as_dict(),
            'wall_escape_timeout_s': config.wall_escape_timeout_s,
            'waypoint_count': len(config.waypoints),
            'waypoints': [waypoint.as_dict() for waypoint in config.waypoints],
        },
        'measurements': {
            'accepted_goal_stamp': _stamp(record.accepted_goal_stamp_ns),
            'accepted_goal_stamp_ns': record.accepted_goal_stamp_ns,
            'accepted_goal_stamp_source': (
                ACCEPTED_GOAL_STAMP_SOURCE if record.accepted_goal_stamp_ns is not None else None
            ),
            'accepted_goal_uuid': record.accepted_goal_uuid,
            'actual_path_length_m': None,
            'collision_count': None,
            'completed_waypoint_count': _completed_waypoint_count(document, record),
            'completion_time_sim_s': completion_time_sim_s,
            'feedback_count': len(record.feedback_trace),
            'feedback_trace': record.feedback_trace,
            'goal_status': status_name,
            'goal_status_code': status_code,
            'goal_response_stamp': _stamp(record.goal_response_stamp_ns),
            'goal_response_stamp_ns': record.goal_response_stamp_ns,
            'goal_submission_stamp': _stamp(record.goal_submission_stamp_ns),
            'goal_submission_stamp_ns': record.goal_submission_stamp_ns,
            'mission_wall_duration_s': mission_wall_duration_s,
            'missed_waypoint_count': len(missed),
            'missed_waypoints': missed,
            'nav2_error_code': None if terminal is None else terminal.error_code,
            'nav2_error_message': None if terminal is None else terminal.error_message,
            'path_efficiency': None,
            'runner_wall_duration_s': record.terminal_wall_offset_s,
            'terminal_action_stamp': _stamp(record.terminal_action_stamp_ns),
            'terminal_action_stamp_ns': record.terminal_action_stamp_ns,
        },
        'events': record.events,
        'quality': {
            'artifact_projection_contract': 'canonical_json_with_matching_one_row_csv',
            'cancel_acknowledged': record.cancel_acknowledged,
            'cancellation_requested': record.cancellation_requested,
            'deadline_kind': record.deadline_kind,
            'fault_schedule_active': False,
            'goal_accepted': record.accepted_goal_uuid is not None,
            'feedback_trace_overflow': record.feedback_trace_overflow,
            'feedback_trace_overflow_count': record.feedback_trace_overflow_count,
            'invalid_feedback_count': record.invalid_feedback_count,
            'metrics_scope': 'mission_action_only',
            'scenario1_metrics_status': SCENARIO1_NOT_EVALUATED,
            'terminal_result_observed': terminal is not None,
        },
        'verdict': {
            'exit_code': int(record.exit_code),
            'expected_outcome_met': succeeded,
            'mission_status': status_name or record.reason.upper(),
            'phase2_action_integration_status': 'PASS' if succeeded else 'FAIL',
            'reason': record.reason,
            'scenario1_acceptance_status': SCENARIO1_NOT_EVALUATED,
        },
    }
    json.dumps(result, allow_nan=False, sort_keys=True)
    return result


def _csv_scalar(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def flatten_result(result: Mapping[str, Any]) -> dict[str, str]:
    """Return the normative one-row scalar projection."""
    identity = result['identity']
    targets = result['targets']
    measurements = result['measurements']
    quality = result['quality']
    verdict = result['verdict']
    values = {
        'run_id': identity['run_id'],
        'created_utc': identity['created_utc'],
        'mission_name': identity['mission_name'],
        'mission_file': identity['mission_file'],
        'mission_sha256': identity['mission_sha256'],
        'mission_seed': identity['mission_seed'],
        'simulator_seed': identity['simulator_seed'],
        'fault_schedule_hash': identity['fault_schedule_hash'],
        'fault_seed': identity['fault_seed'],
        'action_name': identity['action_name'],
        'resolved_action_name': identity['resolved_action_name'],
        'waypoint_count': targets['waypoint_count'],
        'goal_submission_stamp_ns': measurements['goal_submission_stamp_ns'],
        'accepted_goal_uuid': measurements['accepted_goal_uuid'],
        'accepted_goal_stamp_source': measurements['accepted_goal_stamp_source'],
        'goal_response_stamp_ns': measurements['goal_response_stamp_ns'],
        'accepted_goal_stamp_ns': measurements['accepted_goal_stamp_ns'],
        'terminal_action_stamp_ns': measurements['terminal_action_stamp_ns'],
        'completion_time_sim_s': measurements['completion_time_sim_s'],
        'mission_wall_duration_s': measurements['mission_wall_duration_s'],
        'feedback_count': measurements['feedback_count'],
        'feedback_trace_capacity': targets['feedback_trace_capacity'],
        'feedback_trace_overflow': quality['feedback_trace_overflow'],
        'feedback_trace_overflow_count': quality['feedback_trace_overflow_count'],
        'completed_waypoint_count': measurements['completed_waypoint_count'],
        'missed_waypoint_count': measurements['missed_waypoint_count'],
        'goal_status_code': measurements['goal_status_code'],
        'goal_status': measurements['goal_status'],
        'nav2_error_code': measurements['nav2_error_code'],
        'nav2_error_message': measurements['nav2_error_message'],
        'cancellation_requested': quality['cancellation_requested'],
        'cancel_acknowledged': quality['cancel_acknowledged'],
        'terminal_result_observed': quality['terminal_result_observed'],
        'deadline_kind': quality['deadline_kind'],
        'expected_outcome': targets['expected_outcome'],
        'expected_outcome_met': verdict['expected_outcome_met'],
        'phase2_action_integration_status': verdict['phase2_action_integration_status'],
        'scenario1_acceptance_status': verdict['scenario1_acceptance_status'],
        'exit_code': verdict['exit_code'],
        'reason': verdict['reason'],
    }
    row = {name: _csv_scalar(values[name]) for name in CSV_FIELDS}
    if tuple(row) != CSV_FIELDS:
        raise ArtifactError('internal CSV field order mismatch')
    return row


def validate_artifact_paths(json_path: str | Path, csv_path: str | Path) -> tuple[Path, Path]:
    """Resolve distinct result files and reject directory/suffix mistakes."""
    json_target = Path(json_path).expanduser().resolve(strict=False)
    csv_target = Path(csv_path).expanduser().resolve(strict=False)
    if json_target == csv_target:
        raise ArtifactError('JSON and CSV outputs must be different files')
    if json_target.suffix.lower() != '.json':
        raise ArtifactError(f'JSON output must use .json: {json_target}')
    if csv_target.suffix.lower() != '.csv':
        raise ArtifactError(f'CSV output must use .csv: {csv_target}')
    if json_target.exists() and not json_target.is_file():
        raise ArtifactError(f'JSON output is not a regular file: {json_target}')
    if csv_target.exists() and not csv_target.is_file():
        raise ArtifactError(f'CSV output is not a regular file: {csv_target}')
    return json_target, csv_target


def _atomic_text_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            mode='w',
            encoding='utf-8',
            newline='',
            dir=path.parent,
            prefix=f'.{path.name}.',
            suffix='.tmp',
            delete=False,
        ) as stream:
            temporary = Path(stream.name)
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        temporary = None
    except OSError as exc:
        raise ArtifactError(f'atomic write failed for {path}: {exc}') from exc
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def _csv_text(row: Mapping[str, str]) -> str:
    from io import StringIO

    stream = StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS, lineterminator='\n')
    writer.writeheader()
    writer.writerow(row)
    return stream.getvalue()


def reconcile_artifacts(json_path: Path, csv_path: Path) -> None:
    """Re-read both files and require an exact one-row projection match."""
    try:
        result = json.loads(json_path.read_text(encoding='utf-8'))
        with csv_path.open(encoding='utf-8', newline='') as stream:
            rows = list(csv.DictReader(stream))
    except (OSError, json.JSONDecodeError, csv.Error) as exc:
        raise ArtifactError(f'artifact readback failed: {exc}') from exc
    if len(rows) != 1:
        raise ArtifactError(f'expected exactly one CSV row, found {len(rows)}')
    if tuple(rows[0]) != CSV_FIELDS:
        raise ArtifactError('CSV header does not match the canonical field order')
    expected = flatten_result(result)
    if rows[0] != expected:
        raise ArtifactError('CSV row does not match canonical JSON projection')


def write_result_artifacts(
    result: Mapping[str, Any],
    json_path: str | Path,
    csv_path: str | Path,
) -> tuple[Path, Path]:
    """Atomically write canonical JSON, then CSV, then reconcile from disk."""
    json_target, csv_target = validate_artifact_paths(json_path, csv_path)
    row = flatten_result(result)
    json_text = (
        json.dumps(
            result,
            allow_nan=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        + '\n'
    )
    _atomic_text_write(json_target, json_text)
    _atomic_text_write(csv_target, _csv_text(row))
    reconcile_artifacts(json_target, csv_target)
    return json_target, csv_target
