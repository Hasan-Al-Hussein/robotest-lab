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

import copy
import json
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import pytest
from robotest_metrics.analysis import analyze_run
from robotest_metrics.analyze_cli import main as analyze_main
from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.bundle import verify_result_bundle
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.schema_validation import validate_document


def _set_path(document: dict[str, Any], path: Sequence[str], value: Any) -> None:
    current: dict[str, Any] = document
    for component in path[:-1]:
        current = current[component]
    current[path[-1]] = value


def _trial_context(
    request: dict[str, Any],
    *,
    failure: dict[str, Any] | None = None,
) -> dict[str, Any]:
    return {
        'failure': failure,
        'identity': copy.deepcopy(request['identity']),
        'intended_wall_timeout_s': 300.0,
        'producer': 'robotest_phase3/benchmark_orchestrator',
        'schema_version': 1,
        'targets': copy.deepcopy(request['targets']),
    }


def test_complete_observed_request_produces_sole_pass_verdict(
    analysis_request: dict[str, object],
) -> None:
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'PASS'
    assert result['verdict']['exit_code'] == 0
    assert result['measurements']['actual_path_length_m'] == pytest.approx(1.6)
    assert result['measurements']['initial_planned_path_length_m'] == pytest.approx(1.6)
    assert result['measurements']['path_efficiency'] == pytest.approx(1.0)
    assert result['measurements']['collision_count'] == 0
    assert result['measurements']['localization_position_rmse_m'] == pytest.approx(0.0)
    assert result['measurements']['rtf_median'] == pytest.approx(1.0)
    assert result['measurements']['nav2_recovery_count'] is None
    assert 'measurements.nav2_recovery_count' in result['quality']['metric_unavailable_reasons']
    validate_document(result, 'run-result.schema.json')


def test_required_unavailable_metric_is_null_with_reason_and_fails(
    analysis_request: dict[str, object],
) -> None:
    analysis_request['capture']['streams']['plans']['items'].pop()
    result = analyze_run(analysis_request)
    assert result['measurements']['initial_planned_path_length_m'] is None
    reason = result['quality']['metric_unavailable_reasons'][
        'measurements.initial_planned_path_length_m'
    ]
    assert 'waypoint legs' in reason
    assert result['verdict']['automated_status'] == 'FAIL'


def test_overflow_and_invalid_evidence_fail_integrity_without_using_prefix(
    analysis_request: dict[str, object],
) -> None:
    capture = analysis_request['capture']
    capture['streams']['ground_truth']['quality']['overflowed'] = True
    capture['streams']['ground_truth']['quality']['overflow_count'] = 1
    capture['quality']['collector_overflow'] = True
    result = analyze_run(analysis_request)
    assert result['quality']['collector_overflow'] is True
    assert result['measurements']['actual_path_length_m'] is None
    assert (
        'overflowed'
        in result['quality']['metric_unavailable_reasons']['measurements.actual_path_length_m']
    )
    assert result['verdict']['capture_integrity'] is False
    assert result['verdict']['automated_status'] == 'FAIL'


def test_threshold_and_mission_completion_cannot_be_overridden_by_metrics(
    analysis_request: dict[str, object],
) -> None:
    failed_mission = copy.deepcopy(analysis_request)
    failed_mission['mission']['result']['measurements']['goal_status'] = 'ABORTED'
    failed_mission['mission']['artifact_sha256'] = canonical_sha256(
        failed_mission['mission']['result']
    )
    assert analyze_run(failed_mission)['verdict']['automated_status'] == 'FAIL'
    failed_threshold = copy.deepcopy(analysis_request)
    failed_threshold['targets']['acceptance']['measurements.rtf_p5']['minimum'] = 1.1
    result = analyze_run(failed_threshold)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert result['verdict']['threshold_checks'][1]['passed'] is False


def test_mission_component_hash_mismatch_is_invalid(
    analysis_request: dict[str, object],
) -> None:
    analysis_request['mission']['artifact_sha256'] = '0' * 64
    with pytest.raises(MetricUnavailable, match='mission component artifact hash mismatch'):
        analyze_run(analysis_request)


@pytest.mark.parametrize(
    ('component', 'mutate'),
    [
        ('scenario', lambda request: request['scenario']['result'].__setitem__('status', 'FAIL')),
        (
            'fault_control',
            lambda request: request['mission']['result']['fault']['control'].__setitem__(
                'reset_after_goal', False
            ),
        ),
        (
            'orchestrator',
            lambda request: request['orchestrator']['cleanup'].__setitem__('no_orphans', False),
        ),
    ],
)
def test_component_gate_failures_force_canonical_fail(
    analysis_request: dict[str, object],
    component: str,
    mutate,
) -> None:
    mutate(analysis_request)
    if component == 'scenario':
        analysis_request['scenario']['artifact_sha256'] = canonical_sha256(
            analysis_request['scenario']['result']
        )
    if component == 'fault_control':
        analysis_request['mission']['artifact_sha256'] = canonical_sha256(
            analysis_request['mission']['result']
        )
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert component in result['quality']['component_failures']


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('git', 'dirty'), True),
        (('source_binding', 'source_unchanged'), False),
        (('process', 'cold_stack'), False),
        (('process', 'cpu_affinity'), [0, 1, 2, 3]),
        (('resources', 'peak_rss_sum_bytes'), 7 * 1024**3),
        (('resources', 'missing_sample_count'), 1),
        (('execution', 'wall_timed_out'), True),
        (('cleanup', 'no_orphans'), False),
        (('artifacts', 'prerequisite_checksums_verified'), False),
        (('artifacts', 'pre_mission_graph_sha256'), 'not-a-sha256'),
        (('artifacts', 'prerequisite_maximum_file_bytes'), 33_554_433),
        (('gates', 'graph_contract_pass'), False),
        (('gates', 'qos_contract_pass'), False),
        (('gates', 'namespace_isolation_pass'), False),
    ],
)
def test_every_orchestrator_gate_class_fails_the_sole_verdict(
    analysis_request: dict[str, Any],
    path: tuple[str, str],
    value: Any,
) -> None:
    _set_path(analysis_request['orchestrator'], path, value)
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert 'orchestrator' in result['quality']['component_failures']


def test_orchestrator_graph_artifacts_must_be_distinct(
    analysis_request: dict[str, Any],
) -> None:
    analysis_request['orchestrator']['artifacts']['mission_graph_sha256'] = analysis_request[
        'orchestrator'
    ]['artifacts']['pre_mission_graph_sha256']
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert 'orchestrator' in result['quality']['component_failures']


@pytest.mark.parametrize(
    'field',
    [
        'collector_configuration_sha256',
        'metrics_contract_sha256',
        'source_configuration_sha256',
        'target_set_sha256',
    ],
)
def test_orchestrator_source_binding_must_match_frozen_targets(
    analysis_request: dict[str, Any],
    field: str,
) -> None:
    analysis_request['orchestrator']['source_binding'][field] = '0' * 64
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert field in result['quality']['component_failures']['orchestrator']


@pytest.mark.parametrize(
    'target_field',
    ['collision_coverage_manifest_sha256', 'positive_control_json_sha256'],
)
def test_collision_artifacts_must_match_frozen_targets(
    analysis_request: dict[str, Any],
    target_field: str,
) -> None:
    analysis_request['targets'][target_field] = '0' * 64
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert result['measurements']['collision_count'] is None
    assert (
        'target binding'
        in (result['quality']['metric_unavailable_reasons']['measurements.collision_count'])
    )


def test_scenario_identity_mismatch_fails_even_when_component_claims_pass(
    analysis_request: dict[str, Any],
) -> None:
    scenario = analysis_request['scenario']['result']
    scenario['identity']['candidate_id'] = 'different-candidate'
    analysis_request['scenario']['artifact_sha256'] = canonical_sha256(scenario)
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert 'identity mismatch' in result['quality']['component_failures']['scenario']


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('fault', 'control', 'protocol_status'), 'BROKEN'),
        (('fault', 'control', 'event_trace_overflow'), True),
        (('identity', 'fault_schedule_hash'), '0' * 64),
    ],
)
def test_fault_protocol_or_identity_failure_cannot_be_masked_by_mission_success(
    analysis_request: dict[str, Any],
    path: tuple[str, ...],
    value: Any,
) -> None:
    mission = analysis_request['mission']['result']
    _set_path(mission, path, value)
    analysis_request['mission']['artifact_sha256'] = canonical_sha256(mission)
    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'FAIL'
    assert 'fault_control' in result['quality']['component_failures']


def test_analysis_cli_writes_matching_bundle_and_refuses_stale_outputs(
    tmp_path: Path,
    analysis_request: dict[str, object],
) -> None:
    request_path = tmp_path / 'request.json'
    request_path.write_text(json.dumps(analysis_request), encoding='utf-8')
    context_path = tmp_path / 'trial-context.json'
    context_path.write_text(json.dumps(_trial_context(analysis_request)), encoding='utf-8')
    output = tmp_path / 'result'
    arguments = [
        '--input',
        str(request_path),
        '--output-dir',
        str(output),
        '--trial-context',
        str(context_path),
    ]
    assert analyze_main(arguments) == 0
    result = json.loads((output / 'run-result.json').read_text(encoding='utf-8'))
    assert result['verdict']['automated_status'] == 'PASS'
    assert (output / 'run-result.csv').is_file()
    assert (output / 'report.md').is_file()
    manifest = verify_result_bundle(output)
    assert manifest['identity']['run_result_sha256'] == canonical_sha256(result)
    assert analyze_main(arguments) == 33


@pytest.mark.parametrize('invalid_kind', ['missing', 'malformed', 'missing_terminal'])
def test_analysis_cli_emits_canonical_infrastructure_failure_at_ordered_index(
    tmp_path: Path,
    analysis_request: dict[str, Any],
    invalid_kind: str,
) -> None:
    context_path = tmp_path / 'trial-context.json'
    context_path.write_text(json.dumps(_trial_context(analysis_request)), encoding='utf-8')
    request_path = tmp_path / 'request.json'
    if invalid_kind == 'malformed':
        request_path.write_text('{broken', encoding='utf-8')
    elif invalid_kind == 'missing_terminal':
        del analysis_request['mission']['result']['measurements']['terminal_action_stamp_ns']
        analysis_request['mission']['artifact_sha256'] = canonical_sha256(
            analysis_request['mission']['result']
        )
        request_path.write_text(json.dumps(analysis_request), encoding='utf-8')
    output = tmp_path / 'result'
    assert (
        analyze_main(
            [
                '--input',
                str(request_path),
                '--output-dir',
                str(output),
                '--trial-context',
                str(context_path),
            ]
        )
        == 32
    )
    result = json.loads((output / 'run-result.json').read_text(encoding='utf-8'))
    validate_document(result, 'run-result.schema.json')
    assert result['identity']['suite_index'] == analysis_request['identity']['suite_index']
    assert result['verdict']['automated_status'] == 'FAIL'
    assert result['verdict']['reason'] == 'infrastructure_evidence_invalid'
    assert result['measurements']['terminal_action_stamp_ns'] is None
    expected_stage = 'analysis' if invalid_kind == 'missing_terminal' else 'analysis_request_read'
    if invalid_kind == 'malformed':
        expected_stage = 'analysis_request_read'
    assert result['quality']['infrastructure_failure']['stage'] == expected_stage
    assert not list(output.glob('*.png'))
    verify_result_bundle(output)


def test_analysis_cli_composes_declared_upstream_failure_without_request(
    tmp_path: Path,
    analysis_request: dict[str, Any],
) -> None:
    failure = {
        'evidence_sha256': 'f' * 64,
        'exit_code': 124,
        'kind': 'wall_timeout',
        'reason': 'mission process exceeded the steady-wall deadline',
        'stage': 'mission',
        'wall_timed_out': True,
    }
    context_path = tmp_path / 'trial-context.json'
    context_path.write_text(
        json.dumps(_trial_context(analysis_request, failure=failure)),
        encoding='utf-8',
    )
    output = tmp_path / 'result'
    assert (
        analyze_main(
            [
                '--output-dir',
                str(output),
                '--trial-context',
                str(context_path),
            ]
        )
        == 32
    )
    result = json.loads((output / 'run-result.json').read_text(encoding='utf-8'))
    assert result['quality']['infrastructure_failure']['kind'] == 'wall_timeout'
    assert result['quality']['infrastructure_failure']['exit_code'] == 124
    assert result['quality']['infrastructure_failure']['wall_timed_out'] is True
    verify_result_bundle(output)


def test_analysis_cli_context_mismatch_fails_under_precreated_identity(
    tmp_path: Path,
    analysis_request: dict[str, Any],
) -> None:
    context = _trial_context(analysis_request)
    analysis_request['identity']['run_id'] = 'tampered-run-id'
    request_path = tmp_path / 'request.json'
    request_path.write_text(json.dumps(analysis_request), encoding='utf-8')
    context_path = tmp_path / 'trial-context.json'
    context_path.write_text(json.dumps(context), encoding='utf-8')
    output = tmp_path / 'result'
    assert (
        analyze_main(
            [
                '--input',
                str(request_path),
                '--output-dir',
                str(output),
                '--trial-context',
                str(context_path),
            ]
        )
        == 32
    )
    result = json.loads((output / 'run-result.json').read_text(encoding='utf-8'))
    assert result['identity']['run_id'] == context['identity']['run_id']
    assert result['quality']['infrastructure_failure']['stage'] == 'trial_context_binding'


def test_analysis_cli_rejects_untrusted_context_without_fabricating_result(
    tmp_path: Path,
    analysis_request: dict[str, Any],
) -> None:
    request_path = tmp_path / 'request.json'
    request_path.write_text(json.dumps(analysis_request), encoding='utf-8')
    context = _trial_context(analysis_request)
    context['identity']['suite_index'] = 14
    context_path = tmp_path / 'trial-context.json'
    context_path.write_text(json.dumps(context), encoding='utf-8')
    output = tmp_path / 'result'
    assert (
        analyze_main(
            [
                '--input',
                str(request_path),
                '--output-dir',
                str(output),
                '--trial-context',
                str(context_path),
            ]
        )
        == 34
    )
    assert not output.exists()
