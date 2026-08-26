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

"""Deterministic aggregation for the clean 15-trial Phase 3 suite."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping, Sequence
from typing import Any

from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.statistics import deterministic_median, nearest_rank

_GLOBAL_HASH_FIELDS = (
    'collector_configuration_sha256',
    'collision_coverage_manifest_sha256',
    'metrics_contract_sha256',
    'positive_control_json_sha256',
    'source_configuration_sha256',
    'target_set_sha256',
)
_SCENARIO_HASH_FIELDS = ('fault_schedule_sha256', 'scenario_sha256')
_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_GIT_COMMIT = re.compile(r'^[0-9a-f]{40}$')


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable(f'{name} must be an object')
    return value


def _nested(document: Mapping[str, Any], path: str) -> Any:
    value: Any = document
    for component in path.split('.'):
        if not isinstance(value, Mapping) or component not in value:
            return None
        value = value[component]
    return value


def _metric_aggregate(results: Sequence[Mapping[str, Any]], path: str) -> dict[str, Any]:
    values: list[float] = []
    sources: list[dict[str, Any]] = []
    nulls: list[dict[str, Any]] = []
    for result in results:
        identity = _mapping(result.get('identity'), 'identity')
        value = _nested(result, path)
        if value is None:
            quality = _mapping(result.get('quality'), 'quality')
            reasons = quality.get('metric_unavailable_reasons', {})
            reason = (
                reasons.get(path, 'null_without_reason') if isinstance(reasons, Mapping) else 'null'
            )
            nulls.append({'reason': reason, 'run_id': identity.get('run_id')})
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise MetricUnavailable(f'{path} must be numeric or null')
        converted = float(value)
        if not math.isfinite(converted):
            raise MetricUnavailable(f'{path} must be finite')
        values.append(converted)
        sources.append({'run_id': identity.get('run_id'), 'value': converted})
    summary: dict[str, Any] = {
        'available_count': len(values),
        'null_count': len(nulls),
        'nulls': nulls,
        'source_values': sources,
        'valid_numeric_count': len(values),
    }
    if values:
        summary.update(
            {
                'median': deterministic_median(values),
                'p5': nearest_rank(values, 0.05),
                'p95': nearest_rank(values, 0.95),
            }
        )
    else:
        summary.update({'median': None, 'p5': None, 'p95': None})
    return summary


def aggregate_phase3_suite(
    results: Sequence[Mapping[str, Any]],
    *,
    metric_paths: Sequence[str] = (),
) -> dict[str, Any]:
    """Validate suite identity/order and aggregate only canonical run verdicts."""
    if len(results) != 15:
        raise MetricUnavailable('Phase 3 candidate suite must contain exactly 15 results')
    if (
        not metric_paths
        or len(set(metric_paths)) != len(metric_paths)
        or any(not path.startswith('measurements.') for path in metric_paths)
    ):
        raise MetricUnavailable('at least one unique measurements.* aggregate path is required')
    run_ids: set[str] = set()
    ros_domains: set[int] = set()
    gazebo_partitions: set[str] = set()
    git_sha: str | None = None
    global_hashes: dict[str, str] = {}
    scenario_hashes: dict[int, dict[str, str]] = {}
    candidate_id: str | None = None
    scenario_groups: dict[int, list[Mapping[str, Any]]] = {index: [] for index in range(1, 6)}
    for suite_index, result in enumerate(results):
        identity = _mapping(result.get('identity'), f'results[{suite_index}].identity')
        targets = _mapping(result.get('targets'), f'results[{suite_index}].targets')
        expected_scenario = suite_index // 3 + 1
        expected_repetition = suite_index % 3
        declared_suite_index = identity.get('benchmark_suite_index')
        if declared_suite_index is None:
            declared_suite_index = identity.get('suite_index')
        if declared_suite_index != suite_index:
            raise MetricUnavailable('benchmark suite indices are not ordered 0..14')
        if identity.get('scenario_index') != expected_scenario:
            raise MetricUnavailable('scenario order must be 1..5 with three adjacent trials')
        if identity.get('scenario_id') != expected_scenario:
            raise MetricUnavailable('scenario ID and ordered scenario index differ')
        if identity.get('repetition_index') != expected_repetition:
            raise MetricUnavailable('repetition indices must be 0, 1, 2 per scenario')
        if identity.get('git_dirty') is not False:
            raise MetricUnavailable('every candidate trial must use a clean worktree')
        if identity.get('cold_stack') is not True:
            raise MetricUnavailable('every candidate trial must prove a cold stack')
        current_candidate = identity.get('candidate_id')
        if not isinstance(current_candidate, str) or not current_candidate:
            raise MetricUnavailable('candidate ID is missing')
        if candidate_id is None:
            candidate_id = current_candidate
        elif current_candidate != candidate_id:
            raise MetricUnavailable('candidate trials mix candidate IDs')
        current_sha = identity.get('git_sha')
        if not isinstance(current_sha, str) or _GIT_COMMIT.fullmatch(current_sha) is None:
            raise MetricUnavailable('trial Git SHA is missing')
        if git_sha is None:
            git_sha = current_sha
        elif current_sha != git_sha:
            raise MetricUnavailable('candidate trials do not share one Git commit')
        run_id = identity.get('run_id')
        partition = identity.get('gz_partition')
        domain = identity.get('ros_domain_id')
        if not isinstance(run_id, str) or not run_id or run_id in run_ids:
            raise MetricUnavailable('run IDs must be non-empty and unique')
        if not isinstance(partition, str) or not partition or partition in gazebo_partitions:
            raise MetricUnavailable('Gazebo partitions must be non-empty and unique')
        if isinstance(domain, bool) or not isinstance(domain, int) or domain in ros_domains:
            raise MetricUnavailable('ROS domain IDs must be integers and unique')
        run_ids.add(run_id)
        gazebo_partitions.add(partition)
        ros_domains.add(domain)
        for field in _GLOBAL_HASH_FIELDS:
            value = targets.get(field)
            if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
                raise MetricUnavailable(f'{field} is missing')
            existing = global_hashes.setdefault(field, value)
            if value != existing:
                raise MetricUnavailable(f'{field} differs across candidate trials')
        current_scenario_hashes: dict[str, str] = {}
        for field in _SCENARIO_HASH_FIELDS:
            value = targets.get(field)
            if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
                raise MetricUnavailable(f'{field} is missing')
            current_scenario_hashes[field] = value
        existing_scenario = scenario_hashes.setdefault(
            expected_scenario,
            current_scenario_hashes,
        )
        if current_scenario_hashes != existing_scenario:
            raise MetricUnavailable('scenario-bound hashes differ across repetitions')
        scenario_groups[expected_scenario].append(result)
    scenario_aggregates: dict[str, Any] = {}
    for scenario_index, group in scenario_groups.items():
        passed = sum(
            1
            for result in group
            if _mapping(result.get('verdict'), 'verdict').get('automated_status') == 'PASS'
        )
        scenario_aggregates[str(scenario_index)] = {
            'denominator': 3,
            'metrics': {path: _metric_aggregate(group, path) for path in metric_paths},
            'numerator': passed,
            'run_ids': [_mapping(result.get('identity'), 'identity')['run_id'] for result in group],
            'run_verdicts': [
                {
                    'run_id': _mapping(result.get('identity'), 'identity')['run_id'],
                    'status': _mapping(result.get('verdict'), 'verdict').get('automated_status'),
                }
                for result in group
            ],
            'success_rate': passed / 3.0,
            'verdict': 'PASS' if passed == 3 else 'FAIL',
        }
    suite_passed = all(item['verdict'] == 'PASS' for item in scenario_aggregates.values())
    return {
        'identity': {
            'candidate_id': candidate_id,
            'git_sha': git_sha,
            'ordered_run_ids': [
                _mapping(result.get('identity'), 'identity')['run_id'] for result in results
            ],
            'trial_count': 15,
        },
        'quality': {
            'cold_stack_identity': 'PASS',
            'global_hashes': global_hashes,
            'scenario_hashes': {str(key): value for key, value in scenario_hashes.items()},
        },
        'scenarios': scenario_aggregates,
        'verdict': {'automated_status': 'PASS' if suite_passed else 'FAIL'},
    }
