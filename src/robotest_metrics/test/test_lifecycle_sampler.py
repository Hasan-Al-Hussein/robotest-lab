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

from __future__ import annotations

import inspect
import json
from pathlib import Path

import pytest
from jsonschema import Draft202012Validator
from robotest_metrics.artifacts import canonical_json_bytes, write_json_atomic
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.lifecycle_sampler import (
    MAX_LIFECYCLE_SAMPLES,
    MAX_SCHEDULE_BYTES,
    MAX_SCHEDULE_ROUNDS,
    NAV2_LIFECYCLE_NODES,
    LifecycleSamplerCore,
    LifecycleSamplerNode,
    LifecycleSchedule,
    lifecycle_service_name,
    load_schedule,
    main,
    validate_schedule,
)


def _schedule(*stamps: int, run_id: str = 'run-1') -> LifecycleSchedule:
    return validate_schedule(
        {
            'lifecycle_schedule_schema_version': 1,
            'requested_stamps_ns': list(stamps),
            'run_id': run_id,
        }
    )


def _complete_core(*stamps: int) -> LifecycleSamplerCore:
    core = LifecycleSamplerCore(_schedule(*stamps), request_timeout_ns=100)
    core.set_started_wall_ns(1)
    for index, stamp in enumerate(stamps):
        for request in core.observe_clock(stamp, wall_ns=1_000 + index):
            assert core.record_response(
                request.token,
                response_stamp_ns=stamp,
                state_id=3,
                state_label='active',
            )
    core.finalize('stop_file', finished_wall_ns=2_000)
    return core


def _schema(name: str) -> dict[str, object]:
    path = Path(__file__).parents[1] / 'schema' / name
    return json.loads(path.read_text(encoding='utf-8'))


def test_exact_node_allowlist_and_relative_services_are_frozen() -> None:
    assert NAV2_LIFECYCLE_NODES == (
        'map_server',
        'amcl',
        'planner_server',
        'controller_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
        'collision_monitor',
    )
    assert [lifecycle_service_name(node) for node in NAV2_LIFECYCLE_NODES] == [
        f'{node}/get_state' for node in NAV2_LIFECYCLE_NODES
    ]
    assert all(not lifecycle_service_name(node).startswith('/') for node in NAV2_LIFECYCLE_NODES)
    with pytest.raises(ValueError, match='unsupported lifecycle node'):
        lifecycle_service_name('lifecycle_manager_navigation')


def test_runtime_node_source_is_one_participant_and_observer_only() -> None:
    source = inspect.getsource(LifecycleSamplerNode)
    assert source.count('super().__init__(') == 1
    assert "'/clock'" in source
    assert 'create_subscription' in source
    assert 'create_client(GetState' in source
    assert 'all_services_ready' in source
    assert 'cannot arm before all lifecycle services are ready' in source
    assert 'create_publisher' not in source
    assert 'ChangeState' not in source
    assert 'ActionClient' not in source


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (lambda value: value.update(requested_stamps_ns=[]), '1..96'),
        (lambda value: value.update(requested_stamps_ns=list(range(97))), '1..96'),
        (lambda value: value.update(requested_stamps_ns=[1, 1]), 'strictly increasing'),
        (lambda value: value.update(requested_stamps_ns=[2, 1]), 'strictly increasing'),
        (lambda value: value.update(requested_stamps_ns=[-1]), 'must be in'),
        (lambda value: value.update(requested_stamps_ns=[True]), 'must be an integer'),
        (lambda value: value.update(run_id=''), 'non-empty'),
        (lambda value: value.update(run_id='x' * 257), '256-byte'),
        (lambda value: value.update(lifecycle_schedule_schema_version=2), 'must equal 1'),
        (lambda value: value.update(lifecycle_schedule_schema_version=True), 'must equal 1'),
        (lambda value: value.update(extra=True), 'keys mismatch'),
    ],
)
def test_schedule_semantics_reject_every_unsafe_boundary(mutation, message: str) -> None:
    document = {
        'lifecycle_schedule_schema_version': 1,
        'requested_stamps_ns': [0, 1],
        'run_id': 'run-1',
    }
    mutation(document)
    with pytest.raises(MetricUnavailable, match=message):
        validate_schedule(document)


def test_schedule_accepts_exact_maximum_and_hashes_normalized_document() -> None:
    schedule = _schedule(*range(MAX_SCHEDULE_ROUNDS), run_id='r\N{ROBOT FACE}')
    assert schedule.expected_sample_count == MAX_LIFECYCLE_SAMPLES
    assert len(schedule.sha256) == 64
    assert schedule.as_dict()['requested_stamps_ns'] == list(range(MAX_SCHEDULE_ROUNDS))


def test_load_schedule_rejects_duplicate_keys_invalid_utf8_and_file_overflow(tmp_path) -> None:
    duplicate = tmp_path / 'duplicate.json'
    duplicate.write_text(
        '{"lifecycle_schedule_schema_version":1,"run_id":"a","run_id":"b",'
        '"requested_stamps_ns":[1]}',
        encoding='utf-8',
    )
    with pytest.raises(MetricUnavailable, match='duplicate JSON key'):
        load_schedule(duplicate)

    invalid_utf8 = tmp_path / 'invalid.json'
    invalid_utf8.write_bytes(b'\xff')
    with pytest.raises(MetricUnavailable, match='UTF-8'):
        load_schedule(invalid_utf8)

    oversized = tmp_path / 'oversized.json'
    oversized.write_bytes(b' ' * (MAX_SCHEDULE_BYTES + 1))
    with pytest.raises(MetricUnavailable, match='file bound'):
        load_schedule(oversized)


def test_due_rounds_preserve_schedule_and_exact_node_order() -> None:
    core = LifecycleSamplerCore(_schedule(10, 20, 30), request_timeout_ns=5)
    assert core.observe_clock(9, wall_ns=100) == []
    first_two = core.observe_clock(25, wall_ns=110)
    assert len(first_two) == 18
    assert [request.node for request in first_two[:9]] == list(NAV2_LIFECYCLE_NODES)
    assert {request.requested_stamp_ns for request in first_two[:9]} == {10}
    assert {request.requested_stamp_ns for request in first_two[9:]} == {20}
    assert {request.request_stamp_ns for request in first_two} == {25}
    assert {request.deadline_wall_ns for request in first_two} == {115}
    assert len({request.token for request in first_two}) == 18


def test_responses_project_in_callback_order_with_both_sim_stamps() -> None:
    core = LifecycleSamplerCore(_schedule(10), request_timeout_ns=100)
    requests = core.observe_clock(12, wall_ns=100)
    second, first = requests[1], requests[0]
    assert core.record_response(
        second.token,
        response_stamp_ns=14,
        state_id=3,
        state_label='active',
    )
    assert core.record_response(
        first.token,
        response_stamp_ns=15,
        state_id=2,
        state_label='inactive',
    )
    core.finalize('stop_file', finished_wall_ns=200)
    snapshot = core.snapshot()
    assert snapshot['records'][0] == {
        'collector_sequence': 1,
        'error': None,
        'missed': False,
        'node': 'amcl',
        'request_stamp_ns': 12,
        'requested_stamp_ns': 10,
        'response_stamp_ns': 14,
        'round_index': 0,
        'state_id': 3,
        'state_label': 'active',
        'success': True,
    }
    assert snapshot['samples'][:2] == [
        {'collector_sequence': 1, 'node': 'amcl', 'stamp_ns': 14, 'state': 'active'},
        {
            'collector_sequence': 2,
            'node': 'map_server',
            'stamp_ns': 15,
            'state': 'inactive',
        },
    ]
    assert snapshot['quality']['error_count'] == 7
    assert snapshot['quality']['complete'] is False


@pytest.mark.parametrize(
    ('state_id', 'state_label', 'message'),
    [
        (-1, 'active', 'state_id'),
        (256, 'active', 'state_id'),
        (True, 'active', 'state_id'),
        (3, '', 'state_label'),
    ],
)
def test_invalid_service_response_is_retained_as_failure(state_id, state_label, message) -> None:
    core = LifecycleSamplerCore(_schedule(1), request_timeout_ns=10)
    request = core.observe_clock(1, wall_ns=1)[0]
    assert not core.record_response(
        request.token,
        response_stamp_ns=2,
        state_id=state_id,
        state_label=state_label,
    )
    record = core.snapshot()['records'][0]
    assert record['success'] is False
    assert message in record['error']
    assert core.snapshot()['quality']['response_count'] == 1


def test_request_wall_timeout_is_fail_closed_and_late_response_is_ignored() -> None:
    core = LifecycleSamplerCore(_schedule(10), request_timeout_ns=5)
    requests = core.observe_clock(10, wall_ns=100)
    assert core.expire_requests(104) == ()
    expired = core.expire_requests(105)
    assert expired == tuple(request.token for request in requests)
    assert len(core.snapshot()['records']) == 9
    assert not core.record_response(
        requests[0].token,
        response_stamp_ns=11,
        state_id=3,
        state_label='active',
    )
    assert core.snapshot()['quality']['late_response_count'] == 1
    assert len(core.snapshot()['records']) == 9


def test_stop_accounts_for_pending_and_unreached_slots_without_overwrite() -> None:
    core = LifecycleSamplerCore(_schedule(10, 20), request_timeout_ns=100)
    requests = core.observe_clock(10, wall_ns=1)
    canceled = core.finalize('stop_file', finished_wall_ns=2)
    snapshot = core.snapshot()
    assert canceled == tuple(request.token for request in requests)
    assert len(snapshot['records']) == 18
    assert snapshot['quality']['request_count'] == 9
    assert snapshot['quality']['missed_count'] == 9
    assert snapshot['quality']['error_count'] == 18
    assert snapshot['quality']['pending_count'] == 0
    assert snapshot['records'][0]['error'] == 'response_pending_on_stop_file'
    assert snapshot['records'][-1]['error'] == 'requested_stamp_not_reached_before_stop_file'
    assert snapshot['records'][-1]['request_stamp_ns'] is None


def test_maximum_schedule_retains_all_864_successes_without_overflow() -> None:
    core = _complete_core(*range(MAX_SCHEDULE_ROUNDS))
    snapshot = core.snapshot()
    assert len(snapshot['records']) == MAX_LIFECYCLE_SAMPLES
    assert len(snapshot['samples']) == MAX_LIFECYCLE_SAMPLES
    assert snapshot['quality']['retained_count'] == MAX_LIFECYCLE_SAMPLES
    assert snapshot['quality']['collector_overflow'] is False
    assert snapshot['quality']['complete'] is True
    assert [sample['collector_sequence'] for sample in snapshot['samples']] == list(
        range(1, MAX_LIFECYCLE_SAMPLES + 1)
    )


def test_clock_regression_fails_quality_even_when_all_samples_succeed() -> None:
    core = LifecycleSamplerCore(_schedule(1), request_timeout_ns=100)
    requests = core.observe_clock(2, wall_ns=1)
    core.observe_clock(1, wall_ns=2)
    for request in requests:
        core.record_response(
            request.token,
            response_stamp_ns=2,
            state_id=3,
            state_label='active',
        )
    core.finalize('stop_file', finished_wall_ns=3)
    assert core.snapshot()['clock']['regression_count'] == 1
    assert core.snapshot()['quality']['complete'] is False


def test_schedule_and_complete_snapshot_validate_draft_2020_12_contracts() -> None:
    schedule = _schedule(0, 10)
    schedule_schema = _schema('lifecycle-schedule.schema.json')
    snapshot_schema = _schema('lifecycle-snapshot.schema.json')
    Draft202012Validator.check_schema(schedule_schema)
    Draft202012Validator.check_schema(snapshot_schema)
    Draft202012Validator(schedule_schema).validate(schedule.as_dict())
    Draft202012Validator(snapshot_schema).validate(_complete_core(0, 10).snapshot())


def test_canonical_snapshot_output_round_trips_exactly(tmp_path) -> None:
    snapshot = _complete_core(1).snapshot()
    output = tmp_path / 'snapshot.json'
    record = write_json_atomic(snapshot, output)
    assert output.read_bytes() == canonical_json_bytes(snapshot)
    assert record['bytes'] == len(output.read_bytes())
    assert json.loads(output.read_text(encoding='utf-8')) == snapshot


@pytest.mark.parametrize(('stale_index', 'label'), [(0, 'output'), (1, 'ready'), (2, 'stop')])
def test_cli_rejects_every_stale_artifact_before_ros_init(
    tmp_path, stale_index: int, label: str
) -> None:
    schedule = tmp_path / 'schedule.json'
    schedule.write_text(json.dumps(_schedule(1).as_dict()), encoding='utf-8')
    paths = [tmp_path / 'out.json', tmp_path / 'ready.json', tmp_path / 'stop']
    paths[stale_index].write_text('stale', encoding='utf-8')
    with pytest.raises(SystemExit, match=rf'stale {label} file'):
        main(
            [
                '--schedule',
                str(schedule),
                '--output',
                str(paths[0]),
                '--ready-file',
                str(paths[1]),
                '--stop-file',
                str(paths[2]),
            ]
        )


def test_cli_invalid_schedule_returns_bounded_error_without_ros_init(tmp_path) -> None:
    schedule = tmp_path / 'schedule.json'
    schedule.write_text('{}', encoding='utf-8')
    assert (
        main(
            [
                '--schedule',
                str(schedule),
                '--output',
                str(tmp_path / 'out.json'),
                '--ready-file',
                str(tmp_path / 'ready.json'),
                '--stop-file',
                str(tmp_path / 'stop'),
            ]
        )
        == 24
    )


def test_cli_rejects_sub_nanosecond_timeout_before_ros_init(tmp_path) -> None:
    schedule = tmp_path / 'schedule.json'
    schedule.write_text(json.dumps(_schedule(1).as_dict()), encoding='utf-8')
    with pytest.raises(SystemExit, match='at least one nanosecond'):
        main(
            [
                '--schedule',
                str(schedule),
                '--output',
                str(tmp_path / 'out.json'),
                '--ready-file',
                str(tmp_path / 'ready.json'),
                '--stop-file',
                str(tmp_path / 'stop'),
                '--request-timeout-s',
                '1e-20',
            ]
        )
