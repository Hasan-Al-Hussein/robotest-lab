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

"""Canonical fail-closed results for trials without valid analysis evidence."""

from __future__ import annotations

import copy
from collections.abc import Mapping
from itertools import pairwise
from typing import Any

from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.candidate_validation import validate_candidate_identity
from robotest_metrics.constants import (
    CONTRACT_REVISION,
    LOG_MAX_BYTES,
    PER_RUN_CSV_MAX_BYTES,
    PER_RUN_DIRECTORY_MAX_BYTES,
    PER_RUN_JSON_MAX_BYTES,
    PNG_MAX_BYTES,
    PNG_MAX_COUNT,
)
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import require_finite

_CORE_MEASUREMENTS = (
    'accepted_goal_stamp_ns',
    'actual_path_length_m',
    'collision_count',
    'completed_waypoint_count',
    'completion_time_sim_s',
    'initial_planned_path_length_m',
    'path_efficiency',
    'peak_rss_sum_bytes',
    'replan_count',
    'rtf_median',
    'rtf_p5',
    'terminal_action_stamp_ns',
)


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable(f'{name} must be an object')
    return value


def _bounded_reason(value: Any) -> str:
    if not isinstance(value, str) or not value:
        raise MetricUnavailable('infrastructure failure reason must be a non-empty string')
    encoded = value.encode('utf-8')
    if len(encoded) <= 4096:
        return value
    shortened = encoded[:4093]
    while True:
        try:
            return shortened.decode('utf-8') + '...'
        except UnicodeDecodeError:
            shortened = shortened[:-1]


def validate_trial_context(context: Mapping[str, Any]) -> dict[str, Any]:
    """Validate immutable suite identity, hashes, timeout, and optional failure."""
    if context.get('schema_version') != 1 or context.get('producer') != (
        'robotest_phase3/benchmark_orchestrator'
    ):
        raise MetricUnavailable('trial context producer/schema is invalid')
    identity = _mapping(context.get('identity'), 'trial_context.identity')
    targets = _mapping(context.get('targets'), 'trial_context.targets')
    candidate = validate_candidate_identity(identity, targets)
    if (
        require_finite(
            context.get('intended_wall_timeout_s'),
            'trial_context.intended_wall_timeout_s',
        )
        <= 0.0
    ):
        raise MetricUnavailable('trial context wall timeout must be positive')
    required = targets.get('required_metrics')
    if (
        not isinstance(required, list)
        or not required
        or any(
            not isinstance(path, str) or not path.startswith('measurements.') for path in required
        )
    ):
        raise MetricUnavailable('trial context required metrics are invalid')
    ordered_paths = sorted(required)
    if any(later.startswith(earlier + '.') for earlier, later in pairwise(ordered_paths)):
        raise MetricUnavailable('trial context required metric paths conflict')
    acceptance = _mapping(targets.get('acceptance'), 'trial_context.targets.acceptance')
    if any(
        not isinstance(path, str) or not path.startswith('measurements.') for path in acceptance
    ):
        raise MetricUnavailable('trial context acceptance paths are invalid')
    failure = context.get('failure')
    if failure is not None:
        evidence = _mapping(failure, 'trial_context.failure')
        _bounded_reason(evidence.get('reason'))
        exit_code = evidence.get('exit_code')
        if isinstance(exit_code, bool) or not isinstance(exit_code, int) or exit_code == 0:
            raise MetricUnavailable('trial context failure exit code must be nonzero')
    return {
        **candidate,
        'context_sha256': canonical_sha256(context),
        'status': 'CONTEXT_BOUND_ONLY',
    }


def require_request_context_binding(
    request: Mapping[str, Any],
    context: Mapping[str, Any],
) -> None:
    """Reject any request whose complete identity or targets differ from context."""
    if request.get('identity') != context.get('identity'):
        raise MetricUnavailable('analysis request identity differs from immutable trial context')
    if request.get('targets') != context.get('targets'):
        raise MetricUnavailable('analysis request targets differ from immutable trial context')


def automatic_failure(
    *,
    evidence_sha256: str,
    reason: str,
    stage: str,
) -> dict[str, Any]:
    """Create analyzer-observed failure evidence using the frozen exact shape."""
    return {
        'evidence_sha256': evidence_sha256,
        'exit_code': 32,
        'kind': 'invalid_evidence',
        'reason': _bounded_reason(reason),
        'stage': stage,
        'wall_timed_out': False,
    }


def _set_unavailable_measurement(measurements: dict[str, Any], path: str) -> None:
    components = path.split('.')
    if len(components) < 2 or components[0] != 'measurements':
        raise MetricUnavailable('trial context required metric path is invalid')
    current = measurements
    for component in components[1:-1]:
        existing = current.setdefault(component, {})
        if not isinstance(existing, dict):
            raise MetricUnavailable('trial context required metric paths conflict')
        current = existing
    current[components[-1]] = None


def compose_infrastructure_failure(
    context: Mapping[str, Any],
    failure: Mapping[str, Any],
) -> dict[str, Any]:
    """Compose the sole schema-valid FAIL verdict for an infrastructure trial."""
    candidate = validate_trial_context(context)
    failure_evidence = copy.deepcopy(dict(_mapping(failure, 'failure')))
    reason = _bounded_reason(failure_evidence.get('reason'))
    identity = copy.deepcopy(dict(_mapping(context.get('identity'), 'trial_context.identity')))
    targets = copy.deepcopy(dict(_mapping(context.get('targets'), 'trial_context.targets')))
    measurements: dict[str, Any] = {name: None for name in _CORE_MEASUREMENTS}
    measurements['mission_action_status'] = 'INFRASTRUCTURE_ERROR'
    required = targets.get('required_metrics')
    if not isinstance(required, list) or not required:
        raise MetricUnavailable('trial context required metrics are missing')
    unavailable: dict[str, str] = {}
    for path in required:
        if not isinstance(path, str):
            raise MetricUnavailable('trial context required metric path is invalid')
        _set_unavailable_measurement(measurements, path)
        unavailable[path] = reason
    for name in _CORE_MEASUREMENTS:
        unavailable.setdefault(f'measurements.{name}', reason)
    required_checks = [{'passed': False, 'path': path, 'reason': reason} for path in required]
    acceptance = _mapping(targets.get('acceptance'), 'trial_context.targets.acceptance')
    threshold_checks = [
        {
            'actual': None,
            'maximum': _mapping(rule, f'targets.acceptance.{path}').get('maximum'),
            'minimum': _mapping(rule, f'targets.acceptance.{path}').get('minimum'),
            'passed': False,
            'path': path,
        }
        for path, rule in sorted(acceptance.items())
    ]
    failure_evidence['reason'] = reason
    failure_evidence['trial_context_sha256'] = candidate['context_sha256']
    return {
        'events': [{'kind': 'infrastructure_failure', **failure_evidence}],
        'identity': identity,
        'measurements': measurements,
        'quality': {
            'artifact_caps': {
                'canonical_csv_max_bytes': PER_RUN_CSV_MAX_BYTES,
                'canonical_json_max_bytes': PER_RUN_JSON_MAX_BYTES,
                'log_max_bytes': LOG_MAX_BYTES,
                'png_max_bytes': PNG_MAX_BYTES,
                'png_max_count': PNG_MAX_COUNT,
                'run_directory_max_bytes': PER_RUN_DIRECTORY_MAX_BYTES,
            },
            'artifact_projection_preflight': 'PASS',
            'candidate_identity': candidate,
            'capture': {
                'clock': {},
                'failures': [reason],
                'overflowed': None,
                'status': 'UNAVAILABLE',
                'streams': {},
            },
            'collector_overflow': None,
            'component_failures': {'infrastructure': reason},
            'components': {
                'fault_control': None,
                'mission': None,
                'mission_artifact_sha256': None,
                'orchestrator': None,
                'scenario': None,
            },
            'contract_revision': CONTRACT_REVISION,
            'infrastructure_failure': failure_evidence,
            'metric_unavailable_reasons': dict(sorted(unavailable.items())),
        },
        'targets': targets,
        'verdict': {
            'authority': 'robotest_metrics/metrics_analyze',
            'automated_status': 'FAIL',
            'capture_integrity': False,
            'components_complete': False,
            'exit_code': 32,
            'mission_success': False,
            'reason': 'infrastructure_evidence_invalid',
            'required_metric_checks': required_checks,
            'scenario_metric_gate': False,
            'threshold_checks': threshold_checks,
        },
    }
