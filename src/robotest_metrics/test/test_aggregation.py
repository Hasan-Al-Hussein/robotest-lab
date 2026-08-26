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

import pytest
from robotest_metrics.aggregation import aggregate_phase3_suite
from robotest_metrics.errors import MetricUnavailable


def _suite() -> list[dict[str, object]]:
    results: list[dict[str, object]] = []
    for index in range(15):
        scenario = index // 3 + 1
        results.append(
            {
                'identity': {
                    'benchmark_suite_index': index,
                    'candidate_id': 'candidate-1',
                    'cold_stack': True,
                    'gazebo_partition': f'partition-{index}',
                    'git_dirty': False,
                    'git_sha': 'a' * 40,
                    'gz_partition': f'partition-{index}',
                    'repetition_index': index % 3,
                    'ros_domain_id': 100 + index,
                    'run_id': f'run-{index}',
                    'scenario_id': scenario,
                    'scenario_index': scenario,
                },
                'measurements': {'value_m': float(index % 3 + 1)},
                'quality': {'metric_unavailable_reasons': {}},
                'targets': {
                    'collector_configuration_sha256': '1' * 64,
                    'collision_coverage_manifest_sha256': '2' * 64,
                    'fault_schedule_sha256': format(scenario + 5, 'x') * 64,
                    'metrics_contract_sha256': '3' * 64,
                    'positive_control_json_sha256': '4' * 64,
                    'scenario_sha256': str(scenario) * 64,
                    'source_configuration_sha256': '5' * 64,
                    'target_set_sha256': '6' * 64,
                },
                'verdict': {'automated_status': 'PASS'},
            }
        )
    return results


def test_exact_clean_ordered_suite_uses_nearest_rank_per_scenario() -> None:
    aggregate = aggregate_phase3_suite(_suite(), metric_paths=['measurements.value_m'])
    assert aggregate['verdict']['automated_status'] == 'PASS'
    first = aggregate['scenarios']['1']
    assert first['denominator'] == 3
    assert first['numerator'] == 3
    assert first['metrics']['measurements.value_m']['median'] == 2.0
    assert first['metrics']['measurements.value_m']['p95'] == 3.0
    assert first['metrics']['measurements.value_m']['p5'] == 1.0
    assert first['metrics']['measurements.value_m']['valid_numeric_count'] == 3
    assert first['run_verdicts'][0] == {'run_id': 'run-0', 'status': 'PASS'}
    assert aggregate['identity']['candidate_id'] == 'candidate-1'
    assert first['metrics']['measurements.value_m']['source_values'][0] == {
        'run_id': 'run-0',
        'value': 1.0,
    }


def test_failures_and_nulls_remain_in_fixed_acceptance_denominator() -> None:
    suite = _suite()
    suite[1]['verdict']['automated_status'] = 'FAIL'
    suite[1]['measurements']['value_m'] = None
    suite[1]['quality']['metric_unavailable_reasons'] = {
        'measurements.value_m': 'collector_overflow'
    }
    aggregate = aggregate_phase3_suite(suite, metric_paths=['measurements.value_m'])
    first = aggregate['scenarios']['1']
    assert first['denominator'] == 3
    assert first['numerator'] == 2
    assert first['success_rate'] == pytest.approx(2 / 3)
    assert first['metrics']['measurements.value_m']['available_count'] == 2
    assert first['metrics']['measurements.value_m']['null_count'] == 1
    assert first['metrics']['measurements.value_m']['nulls'][0]['reason'] == 'collector_overflow'
    assert aggregate['verdict']['automated_status'] == 'FAIL'


@pytest.mark.parametrize(
    'failure',
    [
        'count',
        'dirty',
        'not_cold',
        'order',
        'git',
        'global_hash',
        'scenario_hash',
        'candidate',
        'duplicate',
    ],
)
def test_suite_identity_failures_are_never_aggregated(failure: str) -> None:
    suite = _suite()
    if failure == 'count':
        suite.pop()
    elif failure == 'dirty':
        suite[0]['identity']['git_dirty'] = True
    elif failure == 'not_cold':
        suite[0]['identity']['cold_stack'] = False
    elif failure == 'order':
        suite[0]['identity']['benchmark_suite_index'] = 1
    elif failure == 'git':
        suite[1]['identity']['git_sha'] = 'different'
    elif failure == 'global_hash':
        suite[1]['targets']['metrics_contract_sha256'] = '9' * 64
    elif failure == 'scenario_hash':
        suite[1]['targets']['fault_schedule_sha256'] = '9' * 64
    elif failure == 'candidate':
        suite[1]['identity']['candidate_id'] = 'candidate-2'
    else:
        suite[1]['identity']['run_id'] = suite[0]['identity']['run_id']
    with pytest.raises(MetricUnavailable):
        aggregate_phase3_suite(copy.deepcopy(suite), metric_paths=['measurements.value_m'])


def test_aggregation_rejects_empty_or_nonmeasurement_metric_sets() -> None:
    with pytest.raises(MetricUnavailable, match='aggregate path'):
        aggregate_phase3_suite(_suite())
    with pytest.raises(MetricUnavailable, match='aggregate path'):
        aggregate_phase3_suite(_suite(), metric_paths=['verdict.exit_code'])
