#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Read-only, exact-path validation for the RoboTest release evidence set."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
import os
import re
import subprocess
import sys
from collections.abc import Mapping
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator
from phase5_ci import EvidenceError, atomic_write_json, canonical_json_bytes, file_sha256
from phase5_release_docs import validate_release_documents

GIT_SHA = re.compile(r'^[0-9a-f]{40}$')
CANDIDATE_ID = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')
PHASE4_RUN_ID = re.compile(r'^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$')
GITHUB_REPOSITORY = re.compile(r'^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$')
SHA256_LINE = re.compile(r'^(?P<sha>[0-9a-f]{64})  (?P<path>[^\r\n]+)$')
MAX_JSON_BYTES = 16 * 1024 * 1024
PHASE4_JSON_MAX_BYTES = 32 * 1024 * 1024
PHASE3_RESULT_JSON_MAX_BYTES = 32 * 1024 * 1024
PHASE3_AGGREGATE_JSON_MAX_BYTES = 64 * 1024 * 1024
PHASE3_AGGREGATE_METRICS = [
    'measurements.completion_time_sim_s',
    'measurements.path_efficiency',
    'measurements.collision_count',
    'measurements.localization_coverage_ratio',
    'measurements.localization_position_rmse_m',
    'measurements.rtf_median',
    'measurements.rtf_p5',
    'measurements.peak_rss_sum_bytes',
    'measurements.sensor_recovery_time_sim_s',
]
PHASE3_RUNTIME_PACKAGES = (
    'robotest_interfaces',
    'robotest_description',
    'robotest_faults',
    'robotest_sim',
    'robotest_navigation',
    'robotest_missions',
    'robotest_metrics',
    'robotest_scenarios',
)
PHASE3_SCENARIO_NAMES = {
    1: 'baseline_navigation',
    2: 'deterministic_static_obstacle_replan',
    3: 'deterministic_dynamic_obstacle',
    4: 'temporary_lidar_dropout',
    5: 'deterministic_odometry_drift',
}
PHASE3_SCENARIO_PATHS = {
    1: 'scenarios/phase3_s1_baseline.yaml',
    2: 'scenarios/phase3_s2_static_obstacle.yaml',
    3: 'scenarios/phase3_s3_dynamic_obstacle.yaml',
    4: 'scenarios/phase3_s4_lidar_dropout.yaml',
    5: 'scenarios/phase3_s5_odom_drift.yaml',
}
PHASE3_REQUIRED_METRICS = [
    'measurements.actual_path_length_m',
    'measurements.collision_count',
    'measurements.completion_time_sim_s',
    'measurements.initial_planned_path_length_m',
    'measurements.localization_coverage_ratio',
    'measurements.localization_position_rmse_m',
    'measurements.path_efficiency',
    'measurements.peak_rss_sum_bytes',
    'measurements.replan_count',
    'measurements.rtf_median',
    'measurements.rtf_p5',
]
PHASE3_GLOBAL_HASH_FIELDS = {
    'collector_configuration_sha256',
    'metrics_contract_sha256',
    'source_configuration_sha256',
    'target_set_sha256',
}
PHASE4_CHECKS = {
    'actual_backoff_is_bounded',
    'cleanup_complete_and_owned',
    'controller_identity_is_exact',
    'event_store_has_no_loss',
    'exact_causal_supervisor_event_chain',
    'exactly_one_failure_event',
    'exactly_one_replacement_child_start',
    'exactly_one_restart_scheduled',
    'exactly_one_successful_followup_completion',
    'followup_json_csv_reconcile',
    'fresh_followup_mission_succeeded',
    'frozen_first_backoff_scheduled',
    'healthz_remained_200',
    'independent_package_reproducibility_passed',
    'installed_package_state_matches_lifecycle',
    'interrupted_completion_matches_outcome',
    'interrupted_mission_never_success',
    'interrupted_mission_pair_is_consistent',
    'interrupted_mission_was_bounded',
    'mission_was_active_before_injection',
    'observed_recovery_is_bounded',
    'only_one_systemd_owned_ros_service',
    'original_process_group_empty_within_5s',
    'package_artifacts_match_lifecycle',
    'package_lifecycle_passed',
    'package_source_binding_passed',
    'published_process_groups_are_bound',
    'readiness_false_event_present',
    'readiness_false_throughout_unavailable_interval',
    'readiness_true_event_present',
    'ready_503_within_3s',
    'ready_restored_within_30s_of_detection',
    'replacement_process_group_is_new',
    'restored_status_is_exactly_ready',
    'run_scoped_supervisor_config_is_exact',
    'runtime_staging_is_bound',
    'source_snapshot_unchanged_and_self_consistent',
    'supervisor_main_pid_stable',
    'systemd_did_not_restart_supervisor',
    'unique_runtime_isolation',
}
PHASE4_TARGETS = {
    'heartbeat_period_wall_s': 0.5,
    'heartbeat_stale_wall_s': 2.0,
    'ready_failure_wall_s_max': 3.0,
    'termination_allowance_wall_s': 5.0,
    'ready_restore_after_detection_wall_s_max': 30.0,
    'restart_backoff_wall_s': [1.0, 2.0, 4.0, 8.0],
    'restart_attempt_limit': 4,
    'restart_window_wall_s': 60.0,
    'stable_reset_wall_s': 60.0,
}
PHASE4_MEASUREMENT_KEYS = {
    'actual_restart_backoff_wall_s',
    'followup_mission_exit_code',
    'interrupted_mission_exit_code',
    'observed_ready_restore_after_503_wall_s',
    'original_child_pgid',
    'original_child_pid',
    'original_group_empty_after_injection_wall_s',
    'ready_503_after_injection_wall_s',
    'replacement_child_pgid',
    'replacement_child_pid',
    'replacement_child_start_count',
    'restart_scheduled_count',
    'supervisor_main_pid',
    'supervisor_recovery_time_wall_s',
}
PHASE4_QUALITY_KEYS = {
    'checks',
    'event_count',
    'event_trace_dropped',
    'raw_evidence_sha256',
    'timeline_count',
}
PHASE4_REQUIRED_RAW = {
    'active-goal.json',
    'cleanup.json',
    'context.json',
    'controller-target.json',
    'followup-mission.json',
    'followup-result.csv',
    'followup-result.json',
    'installed-package-state.json',
    'interrupted-outcome.json',
    'overlay-install-manifest.json',
    'overlay-provenance.json',
    'overlay-source-manifest.json',
    'package-binding.json',
    'package-integrity.json',
    'package-lifecycle.json',
    'ready-restored-status.json',
    'runtime-staging.json',
    'service-final.json',
    'source-snapshot-after.json',
    'source-snapshot-before.json',
    'supervisor-config.json',
    'supervisor-events.jsonl',
    'supervisor-events.meta.json',
    'timeline.jsonl',
}
REMOTE_PROOF_SCOPE = 'Read-only GitHub verification for one exact pushed commit SHA'
EVIDENCE_DOCUMENT_ROOTS = ('docs/results/phase-3/', 'docs/results/phase-4/')
EVIDENCE_DOCUMENT_FILES = {'README.md', 'config/release-claims.json'}
LOCAL_AGGREGATE_SCOPE = (
    'Aggregate static/local routing only; live, campaign, privileged, and remote acceptance '
    'require separate explicitly authorized commands and evidence.'
)
LOCAL_OUTSTANDING_GATES = [
    'Phase 3 authorized 15-run campaign evidence',
    'Phase 4 privileged --apply acceptance evidence',
    'Phase 5 exact pushed-SHA remote CI and evidence-commit verification',
]
ALLOWED_GENERATED_DELTA = [
    'artifacts/evidence/phase0/installed-packages.tsv',
    'artifacts/evidence/phase0/phase0-versions.json',
    'artifacts/evidence/phase0/ros2-doctor.txt',
    'artifacts/evidence/phase0/tool-versions.tsv',
]


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceError(message)


def _passive_release_clock_offset_is_consistent(
    reconciliation: Mapping[str, Any],
) -> bool:
    """Check passive cached-clock arithmetic without imposing a magnitude bound."""
    offset = reconciliation.get('release_delivery_clock_offset_ns')
    delivery = reconciliation.get('release_delivery_clock_stamp_ns')
    release = reconciliation.get('release_qualified_snapshot_stamp_ns')
    if any(
        not isinstance(value, int) or isinstance(value, bool)
        for value in (offset, delivery, release)
    ):
        return False
    return delivery >= 0 and release >= 0 and offset == delivery - release


def _mapping(value: object, label: str) -> dict[str, Any]:
    _require(isinstance(value, dict), f'{label} must be a mapping')
    return value


def _list(value: object, label: str) -> list[Any]:
    _require(isinstance(value, list), f'{label} must be a list')
    return value


def _finite_number(value: object) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _exact_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _exact_json_equal(left: object, right: object) -> bool:
    """Compare JSON values without Python's bool/int equality coercion."""
    try:
        return canonical_json_bytes(left) == canonical_json_bytes(right)
    except (TypeError, ValueError):
        return False


def _resolved_directory(path: Path, label: str) -> Path:
    _require(not path.is_symlink(), f'{label} must not be a symlink')
    resolved = path.resolve(strict=True)
    _require(resolved.is_dir(), f'{label} is not a directory: {path}')
    return resolved


def _repository_directory(repository: Path, relative: str, label: str) -> Path:
    relative_path = Path(relative)
    _require(
        not relative_path.is_absolute() and '..' not in relative_path.parts,
        f'{label} path is unsafe',
    )
    current = repository
    for component in relative_path.parts:
        current = current / component
        _require(not current.is_symlink(), f'{label} contains a symlink: {current}')
        _require(current.is_dir(), f'missing {label}: {current}')
    resolved = current.resolve(strict=True)
    try:
        resolved.relative_to(repository)
    except ValueError as exc:
        raise EvidenceError(f'{label} escapes the repository') from exc
    return resolved


def _regular_file(path: Path, label: str) -> Path:
    _require(path.is_file() and not path.is_symlink(), f'missing regular {label}: {path}')
    return path


def _load_repository_module(repository: Path, relative_path: str, label: str) -> Any:
    path = _regular_file(repository / relative_path, label)
    path_digest = hashlib.sha256(str(path).encode()).hexdigest()[:12]
    source_digest = file_sha256(path)[:12]
    module_name = f'_robotest_release_{path.stem}_{path_digest}_{source_digest}'
    if module_name in sys.modules:
        return sys.modules[module_name]
    tests_root = str(repository / 'tests')
    if tests_root not in sys.path:
        sys.path.insert(0, tests_root)
    spec = importlib.util.spec_from_file_location(module_name, path)
    _require(spec is not None and spec.loader is not None, f'cannot load {label}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


def _activate_repository_packages(repository: Path) -> None:
    package_names = ('robotest_metrics', 'robotest_missions', 'robotest_scenarios')
    for name in tuple(sys.modules):
        if any(name == package or name.startswith(f'{package}.') for package in package_names):
            sys.modules.pop(name, None)
    retained_paths: list[str] = []
    for value in sys.path:
        candidate = Path(value)
        if candidate.name in package_names and candidate.parent.name == 'src':
            continue
        retained_paths.append(value)
    package_roots = [str(repository / f'src/{package}') for package in package_names]
    sys.path[:] = [*package_roots, *retained_paths]
    importlib.invalidate_caches()


def _reject_tree_symlinks(root: Path, label: str) -> None:
    for path in root.rglob('*'):
        _require(not path.is_symlink(), f'{label} contains a symlink: {path}')


def _load_canonical_json(
    path: Path,
    label: str,
    *,
    maximum_bytes: int = MAX_JSON_BYTES,
) -> dict[str, Any]:
    _regular_file(path, label)
    _require(0 < path.stat().st_size <= maximum_bytes, f'{label} exceeds its size bound')
    try:
        payload = path.read_bytes()
        value = _mapping(json.loads(payload), label)
        canonical = canonical_json_bytes(value)
    except (OSError, UnicodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise EvidenceError(f'cannot load canonical {label}: {exc}') from exc
    _require(payload == canonical, f'{label} is not canonical JSON')
    return value


def _canonical_sha256(value: object) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _utc_timestamp(value: object, label: str) -> datetime:
    _require(isinstance(value, str) and value, f'{label} is missing')
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise EvidenceError(f'{label} is invalid') from exc
    _require(
        parsed.tzinfo is not None and parsed.utcoffset() == UTC.utcoffset(parsed),
        f'{label} is not UTC',
    )
    return parsed


def _validate_schema_file(
    repository: Path,
    document: Mapping[str, Any],
    schema_relative_path: str,
    label: str,
) -> None:
    schema_path = repository / schema_relative_path
    _regular_file(schema_path, f'{label} schema')
    try:
        schema = _mapping(json.loads(schema_path.read_bytes()), f'{label} schema')
        Draft202012Validator.check_schema(schema)
        errors = sorted(
            Draft202012Validator(schema).iter_errors(document),
            key=lambda error: tuple(str(component) for component in error.absolute_path),
        )
    except Exception as exc:
        raise EvidenceError(f'cannot apply {label} schema: {exc}') from exc
    if errors:
        error = errors[0]
        location = '.'.join(str(component) for component in error.absolute_path) or '<root>'
        raise EvidenceError(f'{label} schema failed at {location}: {error.message}')


def _validate_schema(
    repository: Path,
    document: Mapping[str, Any],
    schema_name: str,
    label: str,
) -> None:
    _validate_schema_file(
        repository,
        document,
        f'src/robotest_metrics/schema/{schema_name}',
        label,
    )


def _validate_json_sidecar(path: Path, label: str) -> str:
    sidecar = _regular_file(Path(f'{path}.sha256'), f'{label} checksum sidecar')
    _require(sidecar.stat().st_size <= 256, f'{label} checksum sidecar is oversized')
    expected = f'{file_sha256(path)}  {path.name}\n'
    _require(sidecar.read_text(encoding='ascii') == expected, f'{label} checksum sidecar mismatch')
    return expected[:64]


def _csv_scalar(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return canonical_json_bytes(value)[:-1].decode()
    if isinstance(value, str):
        return value
    return canonical_json_bytes(value)[:-1].decode()


def _one_row_csv_bytes(document: Mapping[str, Any]) -> bytes:
    flattened: dict[str, str] = {}

    def visit(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping) and value:
            for key in sorted(value):
                _require(isinstance(key, str) and key, 'CSV projection key is invalid')
                visit(f'{prefix}.{key}' if prefix else key, value[key])
            return
        _require(prefix != '', 'CSV projection root is empty')
        flattened[prefix] = _csv_scalar(value)

    visit('', document)
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=sorted(flattened), lineterminator='\n')
    writer.writeheader()
    writer.writerow(flattened)
    return stream.getvalue().encode()


def _validate_csv_projection(document: Mapping[str, Any], path: Path, label: str) -> None:
    _regular_file(path, label)
    _require(path.read_bytes() == _one_row_csv_bytes(document), f'{label} does not match JSON')


def _phase3_target_contract(scenario_id: int) -> tuple[dict[str, Any], list[str]]:
    acceptance: dict[str, Any] = {
        'measurements.collision_count': {'maximum': 0},
        'measurements.completion_time_sim_s': {'maximum': 180.0},
        'measurements.localization_coverage_ratio': {'minimum': 0.95},
        'measurements.peak_rss_sum_bytes': {'maximum': 6 * 1024**3},
        'measurements.rtf_median': {'minimum': 0.80},
        'measurements.rtf_p5': {'minimum': 0.50},
    }
    required = list(PHASE3_REQUIRED_METRICS)
    if scenario_id == 1:
        acceptance.update(
            {
                'measurements.actual_path_length_m': {'minimum': 0.1},
                'measurements.initial_planned_path_length_m': {'minimum': 0.1},
                'measurements.path_efficiency': {'minimum': 0.75},
            }
        )
    elif scenario_id == 2:
        acceptance.update(
            {
                'measurements.path_efficiency': {'minimum': 0.60},
                'measurements.replan_count': {'minimum': 1},
            }
        )
    elif scenario_id == 3:
        required.append('measurements.scenario3_stop_command')
    elif scenario_id == 4:
        acceptance['measurements.sensor_recovery_time_sim_s'] = {'maximum': 10.0}
        required.extend(
            [
                'measurements.lidar_dropout',
                'measurements.lidar_dropout_command_safety',
                'measurements.sensor_recovery',
                'measurements.sensor_recovery_time_sim_s',
            ]
        )
    elif scenario_id == 5:
        required.append('measurements.odometry_drift')
    else:
        raise EvidenceError('Phase 3 scenario ID is outside the frozen suite')
    return dict(sorted(acceptance.items())), sorted(set(required))


def _metrics_package_root(repository: Path) -> Path:
    package_root = _resolved_directory(
        repository / 'src/robotest_metrics', 'robotest_metrics source package'
    )
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    return package_root


def _validate_tree_manifest(value: object, label: str) -> None:
    manifest = _mapping(value, label)
    _require(
        set(manifest) == {'aggregate_sha256', 'file_count', 'files', 'total_bytes'},
        f'{label} shape changed',
    )
    records = _list(manifest.get('files'), f'{label} files')
    _require(records, f'{label} is empty')
    normalized: list[dict[str, Any]] = []
    paths: list[str] = []
    for value in records:
        record = _mapping(value, f'{label} file')
        path = record.get('path')
        digest = record.get('sha256')
        byte_count = record.get('bytes')
        _require(
            set(record) == {'bytes', 'path', 'sha256'}
            and isinstance(path, str)
            and path
            and path == Path(path).as_posix()
            and not Path(path).is_absolute()
            and '..' not in Path(path).parts
            and path not in paths
            and isinstance(byte_count, int)
            and not isinstance(byte_count, bool)
            and byte_count >= 0
            and isinstance(digest, str)
            and re.fullmatch(r'[0-9a-f]{64}', digest) is not None,
            f'{label} file record is invalid',
        )
        normalized.append({'bytes': byte_count, 'path': path, 'sha256': digest})
        paths.append(path)
    _require(
        normalized == sorted(normalized, key=lambda item: item['path']), f'{label} is unsorted'
    )
    _require(
        _exact_integer(manifest.get('file_count'))
        and manifest.get('file_count') == len(normalized)
        and _exact_integer(manifest.get('total_bytes'))
        and manifest.get('total_bytes') == sum(item['bytes'] for item in normalized)
        and manifest.get('aggregate_sha256') == _canonical_sha256(normalized),
        f'{label} counters or aggregate hash do not reconcile',
    )


def _validate_source_install(value: object) -> None:
    binding = _mapping(value, 'Phase 3 source/install correspondence')
    _require(
        set(binding) == {'aggregate_sha256', 'all_match', 'file_count', 'records'},
        'Phase 3 source/install correspondence shape changed',
    )
    records = _list(binding.get('records'), 'Phase 3 source/install records')
    _require(records, 'Phase 3 source/install correspondence is empty')
    normalized: list[dict[str, Any]] = []
    for value in records:
        record = _mapping(value, 'Phase 3 source/install record')
        _require(
            set(record)
            == {
                'binding_type',
                'installed_path',
                'installed_sha256',
                'matches',
                'package',
                'source_path',
                'source_sha256',
            }
            and record.get('matches') is True
            and isinstance(record.get('binding_type'), str)
            and isinstance(record.get('package'), str)
            and isinstance(record.get('installed_path'), str)
            and isinstance(record.get('source_path'), str)
            and record.get('installed_sha256') == record.get('source_sha256')
            and isinstance(record.get('source_sha256'), str)
            and re.fullmatch(r'[0-9a-f]{64}', record['source_sha256']) is not None,
            'Phase 3 source/install record is invalid',
        )
        normalized.append(dict(record))
    _require(
        normalized == sorted(normalized, key=lambda item: (item['package'], item['source_path']))
        and binding.get('all_match') is True
        and _exact_integer(binding.get('file_count'))
        and binding.get('file_count') == len(normalized)
        and binding.get('aggregate_sha256') == _canonical_sha256(normalized),
        'Phase 3 source/install correspondence does not reconcile',
    )


def _validate_contact_gate_binary_binding(value: object) -> None:
    binding = _mapping(value, 'Phase 3 contact gate binary binding')
    _require(
        set(binding)
        == {
            'build_embedded_source_inventory_match',
            'build_embedded_source_inventory_sha256',
            'build_elf_build_id',
            'build_install_build_id_match',
            'build_install_sha256_match',
            'build_path',
            'build_regular_executable',
            'build_sha256',
            'installed_declared_path',
            'installed_declared_samefile',
            'installed_embedded_source_inventory_match',
            'installed_embedded_source_inventory_sha256',
            'installed_elf_build_id',
            'installed_path',
            'installed_regular_executable',
            'installed_sha256',
            'package',
            'schema_version',
            'source_inventory_sha256',
        }
        and binding.get('package') == 'robotest_sim'
        and binding.get('schema_version') == 1
        and binding.get('build_path') == 'build/robotest_sim/contact_stream_gate'
        and binding.get('installed_declared_path')
        == 'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
        and binding.get('installed_path')
        == 'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
        and binding.get('build_install_build_id_match') is True
        and binding.get('build_install_sha256_match') is True
        and binding.get('build_embedded_source_inventory_match') is True
        and binding.get('build_regular_executable') is True
        and binding.get('installed_declared_samefile') is True
        and binding.get('installed_regular_executable') is True
        and binding.get('installed_embedded_source_inventory_match') is True,
        'Phase 3 contact gate build/install binding changed',
    )
    for field in (
        'build_embedded_source_inventory_sha256',
        'build_sha256',
        'installed_embedded_source_inventory_sha256',
        'installed_sha256',
        'source_inventory_sha256',
    ):
        _require(
            isinstance(binding.get(field), str)
            and re.fullmatch(r'[0-9a-f]{64}', binding[field]) is not None,
            f'Phase 3 contact gate {field} is invalid',
        )
    _require(
        binding.get('build_embedded_source_inventory_sha256')
        == binding.get('source_inventory_sha256')
        and binding.get('installed_embedded_source_inventory_sha256')
        == binding.get('source_inventory_sha256')
        and binding.get('build_sha256') == binding.get('installed_sha256'),
        'Phase 3 contact gate embedded source or build/install hash differs',
    )
    _require(
        isinstance(binding.get('build_elf_build_id'), str)
        and re.fullmatch(r'[0-9a-f]+', binding['build_elf_build_id']) is not None
        and binding.get('installed_elf_build_id') == binding.get('build_elf_build_id'),
        'Phase 3 contact gate ELF build ID binding is invalid',
    )


def _validate_phase3_bundle(
    repository: Path,
    result_directory: Path,
    *,
    git_sha: str,
    expected_identity: Mapping[str, Any],
    expected_plan: Mapping[str, Any],
    build_binding: Mapping[str, Any],
    positive_binding_path: Path,
    orchestration: Any,
) -> tuple[dict[str, Any], str]:
    suite_index = expected_identity['suite_index']
    result_directory = _resolved_directory(result_directory, f'Phase 3 result {suite_index}')
    _metrics_package_root(repository)
    try:
        from robotest_metrics.analysis import _evaluate_thresholds, analyze_run
        from robotest_metrics.bundle import verify_result_bundle
        from robotest_metrics.candidate_validation import validate_candidate_identity
        from robotest_metrics.constants import (
            LOG_MAX_BYTES,
            PER_RUN_CSV_MAX_BYTES,
            PER_RUN_DIRECTORY_MAX_BYTES,
            PER_RUN_JSON_MAX_BYTES,
            PNG_MAX_BYTES,
            PNG_MAX_COUNT,
        )
        from robotest_metrics.failure_results import (
            require_request_context_binding,
            validate_trial_context,
        )

        manifest = verify_result_bundle(result_directory)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 result bundle verification failed: {exc}') from exc
    result_path = result_directory / 'run-result.json'
    result = _load_canonical_json(
        result_path,
        'Phase 3 run result',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    run_root = result_directory.parent
    context_path = run_root / 'trial-context.json'
    request_path = run_root / 'analysis-request.json'
    context = _load_canonical_json(context_path, 'Phase 3 trial context')
    request = _load_canonical_json(
        request_path,
        'Phase 3 analysis request',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    _validate_json_sidecar(context_path, 'Phase 3 trial context')
    _validate_json_sidecar(request_path, 'Phase 3 analysis request')
    _validate_schema_file(
        repository,
        context,
        'src/robotest_metrics/schema/trial-context.schema.json',
        'Phase 3 trial context',
    )
    _validate_schema_file(
        repository,
        request,
        'src/robotest_metrics/schema/analysis-request.schema.json',
        'Phase 3 analysis request',
    )
    mission_path = run_root / 'mission-result.json'
    scenario_path = run_root / 'scenario-result.json'
    capture_path = run_root / 'capture.json'
    contact_drain_path = run_root / 'contact-drain.json'
    contact_progress_path = run_root / 'contact-progress.json'
    initial_contact_gate_path = run_root / 'runtime-gate.json'
    final_contact_gate_path = run_root / 'contact-stream-final-gate.json'
    contact_gate_revalidation_path = run_root / 'contact-gate-revalidation.json'
    orchestrator_path = run_root / 'orchestrator.json'
    lifecycle_path = run_root / 'lifecycle-snapshot.json'
    raw_paths = (
        (mission_path, 'Phase 3 mission result'),
        (scenario_path, 'Phase 3 scenario result'),
        (capture_path, 'Phase 3 capture'),
        (contact_drain_path, 'Phase 3 contact drain'),
        (initial_contact_gate_path, 'Phase 3 initial contact gate'),
        (final_contact_gate_path, 'Phase 3 final contact gate'),
        (contact_gate_revalidation_path, 'Phase 3 contact gate revalidation'),
        (orchestrator_path, 'Phase 3 orchestrator result'),
    )
    for path, label in raw_paths:
        _regular_file(path, label)
        _validate_json_sidecar(path, label)
    mission = _load_canonical_json(
        mission_path,
        'Phase 3 mission result',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    capture_document = _load_canonical_json(
        capture_path,
        'Phase 3 capture',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    _regular_file(contact_progress_path, 'Phase 3 contact progress')
    _load_canonical_json(contact_progress_path, 'Phase 3 contact progress')
    _validate_schema_file(
        repository,
        capture_document,
        'src/robotest_metrics/schema/capture.schema.json',
        'Phase 3 capture',
    )
    scenario_id = expected_identity['scenario_id']
    if scenario_id == 4:
        lifecycle_document = _load_canonical_json(
            lifecycle_path,
            'Phase 3 lifecycle snapshot',
        )
        _validate_json_sidecar(lifecycle_path, 'Phase 3 lifecycle snapshot')
        _validate_schema_file(
            repository,
            lifecycle_document,
            'src/robotest_metrics/schema/lifecycle-snapshot.schema.json',
            'Phase 3 lifecycle snapshot',
        )
    else:
        _require(
            not lifecycle_path.exists(),
            f'Phase 3 run {suite_index} has an unexpected lifecycle snapshot',
        )
    try:
        validate_trial_context(context)
        require_request_context_binding(request, context)
        expected_context = orchestration.make_trial_context(
            expected_plan,
            workspace=repository,
            git_sha=git_sha,
            build=build_binding,
            positive=_load_canonical_json(
                positive_binding_path,
                'Phase 3 positive-control binding',
            ),
        )
        collision = _mapping(request.get('collision'), 'Phase 3 analysis collision input')
        mission_measurements = _mapping(
            mission.get('measurements'),
            'Phase 3 mission measurements',
        )
        terminal_stamp = mission_measurements.get('terminal_action_stamp_ns')
        _require(
            _exact_integer(terminal_stamp) and terminal_stamp > 0,
            'Phase 3 mission terminal stamp is invalid',
        )
        contact_drain = _mapping(collision.get('contact_drain'), 'Phase 3 analysis contact drain')
        drain_stamp = contact_drain.get('qualifying_contact_snapshot_stamp_ns')
        _require(
            _exact_integer(drain_stamp)
            and drain_stamp > terminal_stamp + orchestration.CONTACT_DRAIN_NS
            and collision.get('drain_completed_stamp_ns') == drain_stamp,
            'Phase 3 analysis drain stamp is not a strict authoritative post-terminal snapshot',
        )
        expected_request = orchestration.compose_analysis_request(
            workspace=repository,
            plan=expected_plan,
            mission_path=mission_path,
            scenario_path=scenario_path,
            capture_path=capture_path,
            positive_binding_path=positive_binding_path,
            orchestrator_path=orchestrator_path,
            contact_drain_path=contact_drain_path,
            contact_progress_path=contact_progress_path,
            lifecycle_snapshot_path=(lifecycle_path if scenario_id == 4 else None),
        )
        expected_contact_gate_revalidation = orchestration.reconcile_contact_gate_reobservation(
            initial_contact_gate_path,
            final_contact_gate_path,
            build_binding=build_binding,
            expected_domain_id=expected_plan['ros_domain_id'],
            expected_gz_partition=expected_plan['gz_partition'],
        )
        observed_contact_gate_revalidation = _load_canonical_json(
            contact_gate_revalidation_path,
            'Phase 3 contact gate revalidation',
        )
        _require(
            _exact_json_equal(
                observed_contact_gate_revalidation,
                expected_contact_gate_revalidation,
            ),
            f'Phase 3 run {suite_index} contact gate revalidation changed',
        )
        recomputed_result = analyze_run(request)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 run {suite_index} production replay failed: {exc}') from exc
    _require(
        _exact_json_equal(context, expected_context),
        f'Phase 3 run {suite_index} trial context is not the production recomputation',
    )
    _require(
        _exact_json_equal(request, expected_request),
        f'Phase 3 run {suite_index} analysis request is not the production recomposition',
    )
    identity = _mapping(result.get('identity'), 'Phase 3 run identity')
    targets = _mapping(result.get('targets'), 'Phase 3 run targets')
    verdict = _mapping(result.get('verdict'), 'Phase 3 run verdict')
    required_checks = _list(verdict.get('required_metric_checks'), 'required metric checks')
    threshold_checks = _list(verdict.get('threshold_checks'), 'threshold checks')
    acceptance, required_metrics = _phase3_target_contract(int(expected_identity['scenario_id']))
    expected_target_keys = {
        'acceptance',
        'collector_configuration_sha256',
        'collision_coverage_manifest_sha256',
        'fault_schedule_sha256',
        'metrics_contract_sha256',
        'positive_control_json_sha256',
        'required_metrics',
        'scenario_sha256',
        'source_configuration_sha256',
        'target_set_sha256',
        'world_to_map',
    }
    _require(
        set(targets) == expected_target_keys
        and targets.get('acceptance') == acceptance
        and targets.get('required_metrics') == required_metrics
        and targets.get('world_to_map') == {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        f'Phase 3 run {suite_index} target contract changed',
    )
    for name, expected in expected_identity.items():
        _require(identity.get(name) == expected, f'Phase 3 run identity mismatch for {name}')
    _require(identity.get('git_sha') == git_sha, 'Phase 3 run Git SHA mismatch')
    _require(identity.get('git_dirty') is False, 'Phase 3 run used a dirty worktree')
    _require(identity.get('cold_stack') is True, 'Phase 3 run is not a cold stack')
    try:
        candidate_identity = validate_candidate_identity(identity, targets)
        expected_threshold_checks = _evaluate_thresholds(result, acceptance)
    except Exception as exc:
        raise EvidenceError(
            f'Phase 3 run {suite_index} candidate validation failed: {exc}'
        ) from exc
    expected_required_checks = [
        {'passed': True, 'path': path, 'reason': None} for path in required_metrics
    ]
    _require(
        verdict
        == {
            'automated_status': 'PASS',
            'capture_integrity': True,
            'components_complete': True,
            'exit_code': 0,
            'mission_success': True,
            'required_metric_checks': expected_required_checks,
            'scenario_metric_gate': True,
            'threshold_checks': expected_threshold_checks,
        }
        and required_checks
        and threshold_checks
        and all(item['passed'] is True for item in expected_threshold_checks),
        f'Phase 3 run {suite_index} is not a full canonical PASS',
    )
    quality = _mapping(result.get('quality'), 'Phase 3 run quality')
    components = _mapping(quality.get('components'), 'Phase 3 run components')
    capture = _mapping(quality.get('capture'), 'Phase 3 capture quality')
    artifact_caps = _mapping(quality.get('artifact_caps'), 'Phase 3 artifact caps')
    _require(
        quality.get('artifact_projection_preflight') == 'PASS'
        and quality.get('candidate_identity') == candidate_identity
        and quality.get('collector_overflow') is False
        and quality.get('component_failures') == {}
        and quality.get('contract_revision') == 2
        and quality.get('infrastructure_failure') is None
        and capture.get('status') == 'PASS'
        and capture.get('failures') == []
        and capture.get('overflowed') is False
        and set(components)
        == {'fault_control', 'mission', 'mission_artifact_sha256', 'orchestrator', 'scenario'}
        and all(
            isinstance(components.get(name), dict) and components[name]
            for name in ('fault_control', 'mission', 'orchestrator', 'scenario')
        )
        and isinstance(components.get('mission_artifact_sha256'), str)
        and re.fullmatch(r'[0-9a-f]{64}', components['mission_artifact_sha256']) is not None
        and artifact_caps
        == {
            'canonical_csv_max_bytes': PER_RUN_CSV_MAX_BYTES,
            'canonical_json_max_bytes': PER_RUN_JSON_MAX_BYTES,
            'log_max_bytes': LOG_MAX_BYTES,
            'png_max_bytes': PNG_MAX_BYTES,
            'png_max_count': PNG_MAX_COUNT,
            'run_directory_max_bytes': PER_RUN_DIRECTORY_MAX_BYTES,
        },
        f'Phase 3 run {suite_index} quality is incomplete',
    )
    manifest_identity = _mapping(manifest.get('identity'), 'Phase 3 manifest identity')
    result_sha = file_sha256(result_path)
    _require(
        manifest_identity.get('run_id') == expected_identity['run_id']
        and manifest_identity.get('run_result_sha256') == result_sha,
        'Phase 3 manifest identity mismatch',
    )
    _require(
        _exact_json_equal(result, recomputed_result),
        f'Phase 3 run {suite_index} result is not the production analysis replay',
    )
    return result, result_sha


def _recompute_phase3_aggregate(
    repository: Path,
    results: list[dict[str, Any]],
    result_hashes: list[str],
) -> dict[str, Any]:
    _metrics_package_root(repository)
    try:
        from robotest_metrics.aggregation import aggregate_phase3_suite

        aggregate = aggregate_phase3_suite(results, metric_paths=PHASE3_AGGREGATE_METRICS)
    except Exception as exc:
        raise EvidenceError(f'cannot recompute Phase 3 aggregate: {exc}') from exc
    aggregate['identity']['ordered_source_json_sha256'] = result_hashes
    return aggregate


def _phase3_evidence(
    repository: Path, candidate_root: Path, aggregate_path: Path
) -> dict[str, Any]:
    candidate_root = _resolved_directory(candidate_root, 'Phase 3 candidate root')
    expected_root = _repository_directory(
        repository, 'artifacts/evidence/phase3-benchmarks', 'Phase 3 evidence root'
    )
    _require(
        candidate_root.parent == expected_root,
        'Phase 3 candidate root is outside repository-owned benchmark evidence',
    )
    _reject_tree_symlinks(candidate_root, 'Phase 3 candidate root')
    candidate_id = candidate_root.name
    _require(CANDIDATE_ID.fullmatch(candidate_id) is not None, 'Phase 3 candidate ID is invalid')
    expected_aggregate = candidate_root / 'aggregate/aggregate-result.json'
    _regular_file(aggregate_path, 'Phase 3 aggregate')
    _require(
        aggregate_path.resolve(strict=True) == expected_aggregate.resolve(strict=True),
        'Phase 3 aggregate is not the selected candidate aggregate',
    )
    plan_path = candidate_root / 'suite-plan.json'
    binding_path = candidate_root / 'build-binding.json'
    prepared_path = candidate_root / 'prepared.json'
    plan = _load_canonical_json(plan_path, 'Phase 3 suite plan')
    binding = _load_canonical_json(binding_path, 'Phase 3 build binding')
    prepared = _load_canonical_json(prepared_path, 'Phase 3 prepared marker')
    for path, label in (
        (plan_path, 'Phase 3 suite plan'),
        (binding_path, 'Phase 3 build binding'),
        (prepared_path, 'Phase 3 prepared marker'),
    ):
        _validate_json_sidecar(path, label)
    git = _mapping(binding.get('git'), 'Phase 3 build Git identity')
    git_sha = git.get('sha')
    _require(isinstance(git_sha, str) and GIT_SHA.fullmatch(git_sha), 'Phase 3 Git SHA is invalid')
    _require(
        git.get('dirty') is False and git.get('status_porcelain') == '', 'Phase 3 build is dirty'
    )
    _require(
        set(plan)
        == {
            'aggregate_metrics',
            'candidate_id',
            'cpu_affinity',
            'domain_base',
            'positive_control',
            'producer',
            'schema_version',
            'smoke',
            'trials',
        }
        and plan.get('candidate_id') == candidate_id
        and plan.get('aggregate_metrics') == PHASE3_AGGREGATE_METRICS
        and _exact_json_equal(plan.get('cpu_affinity'), [0, 1, 2, 3, 4, 5])
        and plan.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and _exact_integer(plan.get('schema_version'))
        and plan.get('schema_version') == 1,
        'Phase 3 suite plan contract changed',
    )
    for field in PHASE3_GLOBAL_HASH_FIELDS:
        _require(
            isinstance(binding.get(field), str),
            f'Phase 3 build binding is missing {field}',
        )
        _require(re.fullmatch(r'[0-9a-f]{64}', binding[field]) is not None, f'invalid {field}')
    _require(
        set(binding)
        == {
            'collector_configuration_sha256',
            'contact_gate_binary',
            'created_by',
            'git',
            'install',
            'metrics_contract_sha256',
            'schema_version',
            'source',
            'source_configuration_sha256',
            'source_install',
            'target_set_sha256',
        }
        and set(git) == {'dirty', 'sha', 'status_porcelain'}
        and binding.get('created_by') == 'robotest_phase3/benchmark_orchestrator'
        and _exact_integer(binding.get('schema_version'))
        and binding.get('schema_version') == 1,
        'Phase 3 build binding producer or schema changed',
    )
    _validate_tree_manifest(binding.get('source'), 'Phase 3 source tree manifest')
    _validate_tree_manifest(binding.get('install'), 'Phase 3 install tree manifest')
    _validate_source_install(binding.get('source_install'))
    _validate_contact_gate_binary_binding(binding.get('contact_gate_binary'))
    orchestration = _load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration module',
    )
    try:
        recomputed_binding = orchestration.validate_build_binding(
            repository,
            binding,
            git_sha=git_sha,
            git_status_porcelain='',
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 build binding recomputation failed: {exc}') from exc
    _require(
        _exact_json_equal(binding, recomputed_binding),
        'Phase 3 build binding is not the exact production recomputation',
    )
    _require(
        set(prepared)
        == {
            'build_binding_sha256',
            'candidate_id',
            'git_sha',
            'producer',
            'suite_plan_sha256',
        }
        and prepared.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and prepared.get('candidate_id') == candidate_id
        and prepared.get('git_sha') == git_sha
        and prepared.get('suite_plan_sha256') == file_sha256(plan_path)
        and prepared.get('build_binding_sha256') == file_sha256(binding_path),
        'Phase 3 prepared marker identity mismatch',
    )
    trials = _list(plan.get('trials'), 'Phase 3 suite trials')
    _require(len(trials) == 15, 'Phase 3 suite does not contain exactly 15 trials')
    domain_base = plan.get('domain_base')
    _require(
        isinstance(domain_base, int)
        and not isinstance(domain_base, bool)
        and 0 <= domain_base <= 216,
        'Phase 3 domain base is invalid',
    )

    positive_directory = candidate_root / 'positive-control'
    positive_binding_path = positive_directory / 'positive-binding.json'
    positive_marker_path = positive_directory / 'PASS.json'
    positive_result_path = positive_directory / 'contact-control-result.json'
    positive_capture_path = positive_directory / 'capture.json'
    positive_contact_progress_path = positive_directory / 'contact-progress.json'
    positive_initial_contact_gate_path = positive_directory / 'runtime-gate.json'
    positive_final_contact_gate_path = positive_directory / 'contact-stream-final-gate.json'
    positive_contact_gate_revalidation_path = positive_directory / 'contact-gate-revalidation.json'
    positive_component_manifest_path = positive_directory / 'component-manifest.json'
    positive_binding = _load_canonical_json(
        positive_binding_path, 'Phase 3 positive-control binding'
    )
    positive_marker = _load_canonical_json(positive_marker_path, 'Phase 3 positive-control marker')
    _validate_json_sidecar(positive_binding_path, 'Phase 3 positive-control binding')
    _validate_json_sidecar(positive_marker_path, 'Phase 3 positive-control marker')
    positive_result = _load_canonical_json(
        positive_result_path,
        'Phase 3 positive-control component result',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    positive_capture = _load_canonical_json(
        positive_capture_path,
        'Phase 3 positive-control capture',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    _load_canonical_json(
        positive_contact_progress_path,
        'Phase 3 positive-control contact progress',
    )
    positive_component_manifest = _load_canonical_json(
        positive_component_manifest_path,
        'Phase 3 positive-control component manifest',
    )
    for path, label in (
        (positive_result_path, 'Phase 3 positive-control component result'),
        (positive_capture_path, 'Phase 3 positive-control capture'),
        (positive_initial_contact_gate_path, 'Phase 3 positive initial contact gate'),
        (positive_final_contact_gate_path, 'Phase 3 positive final contact gate'),
        (
            positive_contact_gate_revalidation_path,
            'Phase 3 positive contact gate revalidation',
        ),
        (positive_component_manifest_path, 'Phase 3 positive-control component manifest'),
    ):
        _validate_json_sidecar(path, label)
    _validate_schema_file(
        repository,
        positive_capture,
        'src/robotest_metrics/schema/capture.schema.json',
        'Phase 3 positive-control capture',
    )
    try:
        orchestration.verify_component_manifest(
            positive_component_manifest,
            positive_directory,
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 positive-control component manifest failed: {exc}') from exc
    excluded_component_outputs = {
        'PASS.json',
        'PASS.json.sha256',
        'component-manifest.json',
        'component-manifest.json.sha256',
        'positive-binding.json',
        'positive-binding.json.sha256',
    }
    expected_component_paths = {
        path.relative_to(positive_directory).as_posix()
        for path in positive_directory.rglob('*')
        if path.is_file()
        and path.relative_to(positive_directory).as_posix() not in excluded_component_outputs
    }
    manifest_artifacts = _list(
        positive_component_manifest.get('artifacts'),
        'Phase 3 positive-control component artifacts',
    )
    _require(
        {item.get('path') for item in manifest_artifacts if isinstance(item, dict)}
        == expected_component_paths,
        'Phase 3 positive-control component manifest path set is incomplete',
    )
    positive_resources_path = _regular_file(
        candidate_root / 'positive-control/resources.jsonl',
        'Phase 3 positive-control resource trace',
    )
    try:
        positive_resource_summary = orchestration.summarize_resources(positive_resources_path)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 positive-control resource trace failed: {exc}') from exc
    _require(
        set(positive_marker)
        == {'positive_binding_sha256', 'producer', 'resource_summary', 'status'}
        and positive_marker.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and _exact_json_equal(positive_marker.get('resource_summary'), positive_resource_summary)
        and positive_resource_summary.get('peak_rss_sum_bytes') <= orchestration.RSS_MAX_BYTES
        and positive_resource_summary.get('oom_kill') is False
        and positive_marker.get('status') == 'PASS'
        and positive_marker.get('positive_binding_sha256') == file_sha256(positive_binding_path),
        'Phase 3 positive control is not PASS',
    )
    positive_plan = _mapping(plan.get('positive_control'), 'Phase 3 positive-control plan')
    _require(
        positive_plan
        == {
            'gz_partition': f'robotest_p3_{candidate_id}_positive_control',
            'ros_domain_id': domain_base + 15,
            'run_id': f'{candidate_id}-positive-control',
        },
        'Phase 3 positive-control plan changed',
    )
    benchmark_binding = _mapping(
        positive_binding.get('benchmark_binding'), 'Phase 3 positive benchmark binding'
    )
    coverage_manifest = _mapping(
        positive_binding.get('coverage_manifest'), 'Phase 3 positive coverage manifest'
    )
    positive_json_sha = positive_binding.get('positive_control_json_sha256')
    coverage_sha = coverage_manifest.get('manifest_sha256')
    positive_control = _mapping(
        positive_binding.get('positive_control'), 'Phase 3 positive-control result'
    )
    collector_reconciliation = _mapping(
        positive_binding.get('collector_reconciliation'),
        'Phase 3 positive collector reconciliation',
    )
    external_quality = _mapping(
        benchmark_binding.get('positive_control_external_quality'),
        'Phase 3 positive external quality',
    )
    semantic_coverage = dict(coverage_manifest)
    semantic_coverage.pop('manifest_sha256', None)
    provenance_keys = {
        'bridge_sha256',
        'collector_configuration_sha256',
        'contact_configuration_sha256',
        'coverage_manifest_sha256',
        'rendered_sdf_sha256',
        'robot_description_sha256',
        'world_source_sha256',
    }
    expected_provenance = {
        'bridge_sha256': coverage_manifest.get('bridge_sha256'),
        'collector_configuration_sha256': binding.get('collector_configuration_sha256'),
        'contact_configuration_sha256': coverage_manifest.get('contact_configuration_sha256'),
        'coverage_manifest_sha256': coverage_sha,
        'rendered_sdf_sha256': coverage_manifest.get('rendered_sdf_sha256'),
        'robot_description_sha256': coverage_manifest.get('robot_description_sha256'),
        'world_source_sha256': coverage_manifest.get('world_source_sha256'),
    }
    positive_identity = _mapping(
        positive_control.get('identity'), 'Phase 3 positive-control identity'
    )
    positive_verdict = _mapping(positive_control.get('verdict'), 'Phase 3 positive-control verdict')
    _validate_schema_file(
        repository,
        positive_control,
        'src/robotest_scenarios/schema/contact-control-result.schema.json',
        'Phase 3 positive-control result',
    )
    _require(
        set(positive_binding)
        == {
            'benchmark_binding',
            'capture_sha256',
            'collector_reconciliation',
            'coverage_manifest',
            'positive_control',
            'positive_control_json_sha256',
            'producer',
            'schema_version',
        }
        and set(benchmark_binding)
        == {
            'benchmark_provenance',
            'positive_control_external_quality',
            'positive_control_json_sha256',
            'positive_control_provenance',
            'positive_control_run_id',
            'positive_control_scenario_sha256',
        }
        and positive_binding.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and _exact_integer(positive_binding.get('schema_version'))
        and positive_binding.get('schema_version') == 1
        and _exact_integer(coverage_manifest.get('schema_version'))
        and coverage_manifest.get('schema_version') == 3
        and isinstance(positive_json_sha, str)
        and re.fullmatch(r'[0-9a-f]{64}', positive_json_sha) is not None
        and positive_json_sha == _canonical_sha256(positive_control)
        and positive_control.get('status') == 'PASS'
        and _exact_integer(positive_verdict.get('exit_code'))
        and set(positive_verdict) == {'authority', 'benchmark_pass', 'exit_code', 'reason'}
        and positive_verdict.get('authority') == 'component_only'
        and positive_verdict.get('benchmark_pass') is None
        and positive_verdict.get('exit_code') == 0
        and isinstance(positive_verdict.get('reason'), str)
        and positive_identity.get('run_id') == positive_plan.get('run_id')
        and benchmark_binding.get('positive_control_scenario_sha256')
        == positive_identity.get('scenario_sha256')
        and isinstance(positive_binding.get('capture_sha256'), str)
        and re.fullmatch(r'[0-9a-f]{64}', positive_binding['capture_sha256']) is not None
        and isinstance(coverage_sha, str)
        and re.fullmatch(r'[0-9a-f]{64}', coverage_sha) is not None
        and coverage_sha == _canonical_sha256(semantic_coverage)
        and benchmark_binding.get('positive_control_json_sha256') == positive_json_sha
        and benchmark_binding.get('positive_control_run_id') == positive_plan.get('run_id'),
        'Phase 3 positive-control binding contract changed',
    )
    for name in ('benchmark_provenance', 'positive_control_provenance'):
        provenance = _mapping(benchmark_binding.get(name), f'Phase 3 {name}')
        _require(
            set(provenance) == provenance_keys and provenance == expected_provenance,
            f'Phase 3 {name} does not bind the coverage evidence',
        )
    _require(
        external_quality
        == {
            'checksum_verified': True,
            'collector_capture_sha256': positive_binding.get('capture_sha256'),
            'collector_reconciled': True,
            'owned_process_group_shutdown': True,
        }
        and set(collector_reconciliation)
        == {
            'captured_command_count',
            'captured_exact_pair_count',
            'captured_release_expected_pair_count',
            'captured_release_snapshot_count',
            'contact_projection_episode_count',
            'contact_projection_first_stamp_ns',
            'contact_projection_record_count',
            'contact_projection_sha256',
            'contact_projection_snapshot_count',
            'contact_progress_artifact_sha256',
            'contact_progress_latest_retained_stamp_ns',
            'contact_progress_retained_message_count',
            'component_command_count',
            'component_exact_pair_count',
            'latest_clock_stamp_ns',
            'release_delivery_clock_offset_ns',
            'release_delivery_clock_stamp_ns',
            'release_qualified_snapshot_stamp_ns',
            'release_required_through_stamp_ns',
        }
        and all(
            isinstance(value, int) and not isinstance(value, bool) and value >= 0
            for key, value in collector_reconciliation.items()
            if key
            not in {
                'contact_progress_artifact_sha256',
                'contact_projection_sha256',
                'release_delivery_clock_offset_ns',
            }
        )
        and _passive_release_clock_offset_is_consistent(collector_reconciliation)
        and isinstance(collector_reconciliation['contact_progress_artifact_sha256'], str)
        and isinstance(collector_reconciliation['contact_projection_sha256'], str)
        and re.fullmatch(
            r'[0-9a-f]{64}',
            collector_reconciliation['contact_progress_artifact_sha256'],
        )
        is not None
        and re.fullmatch(r'[0-9a-f]{64}', collector_reconciliation['contact_projection_sha256'])
        is not None
        and collector_reconciliation['contact_progress_artifact_sha256']
        == file_sha256(positive_contact_progress_path)
        and collector_reconciliation['captured_command_count']
        >= collector_reconciliation['component_command_count']
        and collector_reconciliation['captured_exact_pair_count']
        == collector_reconciliation['component_exact_pair_count']
        and collector_reconciliation['contact_projection_episode_count'] == 1
        and collector_reconciliation['contact_projection_snapshot_count'] > 0
        and collector_reconciliation['contact_projection_record_count']
        >= collector_reconciliation['contact_projection_snapshot_count']
        and collector_reconciliation['contact_projection_first_stamp_ns']
        <= collector_reconciliation['release_qualified_snapshot_stamp_ns']
        and collector_reconciliation['captured_release_snapshot_count'] == 1
        and collector_reconciliation['captured_release_expected_pair_count'] == 0
        and collector_reconciliation['release_qualified_snapshot_stamp_ns']
        > collector_reconciliation['release_required_through_stamp_ns']
        and collector_reconciliation['latest_clock_stamp_ns']
        >= collector_reconciliation['release_qualified_snapshot_stamp_ns'],
        'Phase 3 positive-control reconciliation is invalid',
    )
    _metrics_package_root(repository)
    try:
        from robotest_metrics.collision_metrics import validate_collision_qualification
        from robotest_metrics.errors import MetricUnavailable

        qualification = validate_collision_qualification(
            coverage_manifest,
            positive_control,
            benchmark_binding,
            wall_asset_path=(
                repository / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
            ),
        )
    except (MetricUnavailable, KeyError, TypeError, ValueError) as exc:
        raise EvidenceError(f'Phase 3 collision qualification failed: {exc}') from exc
    expected_qualification = {
        'covered_collision_count': len(
            _list(coverage_manifest.get('robot_collisions'), 'Phase 3 covered robot collisions')
        ),
        'coverage_manifest_sha256': coverage_sha,
        'positive_control_json_sha256': positive_json_sha,
        'positive_control_run_id': positive_identity.get('run_id'),
        'positive_control_scenario_sha256': positive_identity.get('scenario_sha256'),
        'status': 'PASS',
    }
    _require(
        _exact_json_equal(qualification, expected_qualification),
        'Phase 3 collision qualification report is not the exact canonical PASS binding',
    )
    try:
        expected_positive_gate_revalidation = orchestration.reconcile_contact_gate_reobservation(
            positive_initial_contact_gate_path,
            positive_final_contact_gate_path,
            build_binding=binding,
            expected_domain_id=positive_plan['ros_domain_id'],
            expected_gz_partition=positive_plan['gz_partition'],
        )
        observed_positive_gate_revalidation = _load_canonical_json(
            positive_contact_gate_revalidation_path,
            'Phase 3 positive contact gate revalidation',
        )
        _require(
            _exact_json_equal(
                observed_positive_gate_revalidation,
                expected_positive_gate_revalidation,
            ),
            'Phase 3 positive contact gate revalidation changed',
        )
        recomputed_positive_binding = orchestration.reconcile_positive_control(
            workspace=repository,
            build_binding=binding,
            result_path=positive_result_path,
            capture_path=positive_capture_path,
            contact_progress_path=positive_contact_progress_path,
            manifest_path=repository / 'config/collision-coverage.yaml',
            collector_configuration_sha256=binding['collector_configuration_sha256'],
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 positive-control recomposition failed: {exc}') from exc
    _require(
        _exact_json_equal(positive_binding, recomputed_positive_binding)
        and _exact_json_equal(positive_control, positive_result),
        'Phase 3 positive-control binding is not the exact production recomposition',
    )

    smoke_marker_path = candidate_root / 'smoke/PASS.json'
    smoke_marker = _load_canonical_json(smoke_marker_path, 'Phase 3 smoke marker')
    _validate_json_sidecar(smoke_marker_path, 'Phase 3 smoke marker')
    smoke_plan = _mapping(plan.get('smoke'), 'Phase 3 smoke plan')
    _require(
        smoke_plan
        == {
            'gz_partition': f'robotest_p3_{candidate_id}_smoke',
            'ros_domain_id': domain_base + 16,
            'run_id': f'{candidate_id}-smoke-s1-r0',
            'scenario_path': PHASE3_SCENARIO_PATHS[1],
        },
        'Phase 3 smoke plan changed',
    )
    first_trial = _mapping(trials[0], 'Phase 3 first trial')
    smoke_identity = {
        'candidate_id': f'{candidate_id}-smoke',
        'gz_partition': f'robotest_p3_{candidate_id}-smoke_00',
        'repetition_index': 0,
        'ros_domain_id': smoke_plan.get('ros_domain_id'),
        'run_id': smoke_plan.get('run_id'),
        'scenario_id': 1,
        'scenario_index': 1,
        'scenario_name': PHASE3_SCENARIO_NAMES[1],
        'scenario_sha256': first_trial.get('scenario_sha256'),
        'suite_index': 0,
    }
    smoke_trial = {
        key: smoke_identity[key]
        for key in (
            'candidate_id',
            'gz_partition',
            'repetition_index',
            'ros_domain_id',
            'run_id',
            'scenario_id',
            'scenario_name',
            'scenario_sha256',
            'suite_index',
        )
    } | {'scenario_path': PHASE3_SCENARIO_PATHS[1]}
    _, smoke_sha = _validate_phase3_bundle(
        repository,
        candidate_root / 'smoke/result',
        git_sha=git_sha,
        expected_identity=smoke_identity,
        expected_plan=smoke_trial,
        build_binding=binding,
        positive_binding_path=positive_binding_path,
        orchestration=orchestration,
    )
    _require(
        set(smoke_marker) == {'producer', 'run_result_sha256', 'status'}
        and smoke_marker.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and smoke_marker.get('status') == 'PASS'
        and smoke_marker.get('run_result_sha256') == smoke_sha,
        'Phase 3 smoke marker does not bind its PASS result',
    )

    runs_root = _resolved_directory(candidate_root / 'runs', 'Phase 3 campaign runs')
    _require(
        {path.name for path in runs_root.iterdir()} == {f'{index:02d}' for index in range(15)},
        'Phase 3 campaign run directory set is not exactly 00..14',
    )
    results: list[dict[str, Any]] = []
    result_hashes: list[str] = []
    run_ids: list[str] = []
    for index, item in enumerate(trials):
        trial = _mapping(item, f'Phase 3 trial {index}')
        expected_scenario = index // 3 + 1
        expected_repetition = index % 3
        _require(
            set(trial)
            == {
                'candidate_id',
                'gz_partition',
                'repetition_index',
                'ros_domain_id',
                'run_id',
                'scenario_id',
                'scenario_name',
                'scenario_path',
                'scenario_sha256',
                'suite_index',
            }
            and trial.get('candidate_id') == candidate_id
            and _exact_integer(trial.get('suite_index'))
            and trial.get('suite_index') == index
            and _exact_integer(trial.get('scenario_id'))
            and trial.get('scenario_id') == expected_scenario
            and _exact_integer(trial.get('repetition_index'))
            and trial.get('repetition_index') == expected_repetition
            and trial.get('scenario_name') == PHASE3_SCENARIO_NAMES[expected_scenario]
            and trial.get('scenario_path') == PHASE3_SCENARIO_PATHS[expected_scenario]
            and _exact_integer(trial.get('ros_domain_id'))
            and trial.get('ros_domain_id') == domain_base + index
            and trial.get('gz_partition') == f'robotest_p3_{candidate_id}_{index:02d}'
            and trial.get('run_id')
            == f'{candidate_id}-s{expected_scenario}-r{expected_repetition}-i{index:02d}'
            and isinstance(trial.get('scenario_sha256'), str)
            and re.fullmatch(r'[0-9a-f]{64}', trial['scenario_sha256']) is not None,
            f'Phase 3 trial order changed: {index}',
        )
        run_id = trial.get('run_id')
        _require(isinstance(run_id, str) and run_id, f'Phase 3 run ID is missing: {index}')
        scenario_path = _regular_file(
            repository / str(trial['scenario_path']), f'Phase 3 scenario {expected_scenario}'
        )
        _require(not scenario_path.is_symlink(), 'Phase 3 scenario must not be a symlink')
        try:
            scenario_document = _mapping(
                yaml.safe_load(scenario_path.read_text(encoding='utf-8')),
                f'Phase 3 scenario {expected_scenario}',
            )
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            raise EvidenceError(f'cannot load Phase 3 scenario {expected_scenario}: {exc}') from exc
        _require(
            file_sha256(scenario_path) == trial.get('scenario_sha256')
            and _exact_integer(scenario_document.get('scenario_id'))
            and scenario_document.get('scenario_id') == expected_scenario
            and scenario_document.get('scenario_name') == PHASE3_SCENARIO_NAMES[expected_scenario]
            and scenario_document.get('simulator_seed') == 42,
            f'Phase 3 scenario source binding changed: {expected_scenario}',
        )
        result, result_sha = _validate_phase3_bundle(
            repository,
            runs_root / f'{index:02d}/result',
            git_sha=git_sha,
            expected_identity={key: value for key, value in trial.items() if key != 'scenario_path'}
            | {'scenario_index': expected_scenario},
            expected_plan=trial,
            build_binding=binding,
            positive_binding_path=positive_binding_path,
            orchestration=orchestration,
        )
        targets = _mapping(result.get('targets'), f'Phase 3 run targets {index}')
        _require(
            all(targets.get(field) == binding.get(field) for field in PHASE3_GLOBAL_HASH_FIELDS)
            and targets.get('positive_control_json_sha256') == positive_json_sha
            and targets.get('collision_coverage_manifest_sha256') == coverage_sha
            and targets.get('scenario_sha256') == trial.get('scenario_sha256')
            and targets.get('fault_schedule_sha256')
            == scenario_document.get('fault_schedule_sha256'),
            f'Phase 3 target binding mismatch: {index}',
        )
        results.append(result)
        result_hashes.append(result_sha)
        run_ids.append(run_id)

    aggregate = _load_canonical_json(
        expected_aggregate,
        'Phase 3 aggregate',
        maximum_bytes=PHASE3_AGGREGATE_JSON_MAX_BYTES,
    )
    _validate_csv_projection(
        aggregate,
        expected_aggregate.with_name('aggregate-result.csv'),
        'Phase 3 aggregate CSV',
    )
    verdict = _mapping(aggregate.get('verdict'), 'Phase 3 aggregate verdict')
    _require(verdict.get('automated_status') == 'PASS', 'Phase 3 aggregate verdict is not PASS')
    recomputed = _recompute_phase3_aggregate(repository, results, result_hashes)
    _require(
        _exact_json_equal(aggregate, recomputed),
        'Phase 3 aggregate does not exactly recompute from runs',
    )
    identity = _mapping(aggregate.get('identity'), 'Phase 3 aggregate identity')
    quality = _mapping(aggregate.get('quality'), 'Phase 3 aggregate quality')
    _require(
        identity.get('candidate_id') == candidate_id
        and identity.get('git_sha') == git_sha
        and identity.get('trial_count') == 15
        and identity.get('ordered_run_ids') == run_ids
        and identity.get('ordered_source_json_sha256') == result_hashes,
        'Phase 3 aggregate identity does not bind the selected 15 runs',
    )
    _require(quality.get('cold_stack_identity') == 'PASS', 'Phase 3 cold-stack proof is absent')
    scenarios = _mapping(aggregate.get('scenarios'), 'Phase 3 scenario aggregates')
    _require(set(scenarios) == {str(index) for index in range(1, 6)}, 'Phase 3 scenarios changed')
    for scenario_index in range(1, 6):
        scenario = _mapping(scenarios[str(scenario_index)], f'Phase 3 scenario {scenario_index}')
        expected_ids = run_ids[(scenario_index - 1) * 3 : scenario_index * 3]
        run_verdicts = _list(scenario.get('run_verdicts'), 'Phase 3 scenario run verdicts')
        _require(
            scenario.get('denominator') == 3
            and scenario.get('numerator') == 3
            and scenario.get('success_rate') == 1.0
            and scenario.get('verdict') == 'PASS'
            and scenario.get('run_ids') == expected_ids
            and [item.get('run_id') for item in run_verdicts] == expected_ids
            and all(item.get('status') == 'PASS' for item in run_verdicts),
            f'Phase 3 scenario {scenario_index} is not 3/3 PASS',
        )
    return {
        'aggregate_path': str(expected_aggregate),
        'aggregate_sha256': file_sha256(expected_aggregate),
        'candidate_id': candidate_id,
        'candidate_root': str(candidate_root),
        'git_sha': git_sha,
        'scenario_count': 5,
        'trial_count': 15,
    }


def _validate_exact_manifest(root: Path) -> dict[str, str]:
    manifest = _regular_file(root / 'SHA256SUMS', 'Phase 4 checksum manifest')
    manifest_payload = manifest.read_bytes()
    expected_files = {
        path.relative_to(root).as_posix(): path
        for path in sorted(root.rglob('*'))
        if path.is_file() and path != manifest
    }
    declared: dict[str, str] = {}
    for line_number, line in enumerate(manifest.read_text(encoding='ascii').splitlines(), 1):
        match = SHA256_LINE.fullmatch(line)
        _require(match is not None, f'Phase 4 checksum line {line_number} is invalid')
        name = match.group('path')
        relative = Path(name)
        _require(
            name == relative.as_posix()
            and not relative.is_absolute()
            and '..' not in relative.parts
            and name not in declared,
            f'Phase 4 checksum path is unsafe or duplicated: {name}',
        )
        declared[name] = match.group('sha')
    _require(set(declared) == set(expected_files), 'Phase 4 checksum coverage is not exact')
    for name, digest in declared.items():
        _require(file_sha256(expected_files[name]) == digest, f'Phase 4 checksum mismatch: {name}')
    expected_payload = ''.join(f'{declared[name]}  {name}\n' for name in sorted(declared)).encode(
        'ascii'
    )
    _require(manifest_payload == expected_payload, 'Phase 4 checksum manifest is not canonical')
    return declared


def _phase4_csv_bytes(result: Mapping[str, Any]) -> bytes:
    identity = _mapping(result.get('identity'), 'Scenario 6 identity')
    measurements = _mapping(result.get('measurements'), 'Scenario 6 measurements')
    verdict = _mapping(result.get('verdict'), 'Scenario 6 verdict')
    row = {
        'run_id': identity.get('run_id'),
        'verdict': verdict.get('status'),
        'failure_count': verdict.get('failure_count'),
        'ready_503_after_injection_wall_s': measurements.get('ready_503_after_injection_wall_s'),
        'original_group_empty_after_injection_wall_s': measurements.get(
            'original_group_empty_after_injection_wall_s'
        ),
        'supervisor_recovery_time_wall_s': measurements.get('supervisor_recovery_time_wall_s'),
        'actual_restart_backoff_wall_s': measurements.get('actual_restart_backoff_wall_s'),
        'restart_scheduled_count': measurements.get('restart_scheduled_count'),
        'interrupted_mission_exit_code': measurements.get('interrupted_mission_exit_code'),
        'followup_mission_exit_code': measurements.get('followup_mission_exit_code'),
        'ros_domain_id': identity.get('ros_domain_id'),
        'gz_partition': identity.get('gz_partition'),
        'source_git_commit': identity.get('source_git_commit'),
    }
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=list(row), lineterminator='\n')
    writer.writeheader()
    writer.writerow({key: '' if value is None else value for key, value in row.items()})
    return stream.getvalue().encode()


def _phase4_evidence(repository: Path, run_directory: Path, scenario6_path: Path) -> dict[str, Any]:
    run_directory = _resolved_directory(run_directory, 'Phase 4 run directory')
    expected_root = _repository_directory(
        repository, 'artifacts/evidence/phase4/runs', 'Phase 4 evidence root'
    )
    _require(
        run_directory.parent == expected_root,
        'Phase 4 run directory is outside repository-owned acceptance evidence',
    )
    _reject_tree_symlinks(run_directory, 'Phase 4 run directory')
    _require(PHASE4_RUN_ID.fullmatch(run_directory.name) is not None, 'Phase 4 run ID is invalid')
    expected_result = run_directory / 'scenario6-result.json'
    _regular_file(scenario6_path, 'Scenario 6 evidence')
    _require(
        scenario6_path.resolve(strict=True) == expected_result.resolve(strict=True),
        'Scenario 6 evidence is not inside the selected Phase 4 run',
    )
    manifest = _validate_exact_manifest(run_directory)
    result = _load_canonical_json(
        expected_result,
        'Scenario 6 result',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    context = _load_canonical_json(
        run_directory / 'context.json',
        'Phase 4 context',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    identity = _mapping(result.get('identity'), 'Scenario 6 identity')
    verdict = _mapping(result.get('verdict'), 'Scenario 6 verdict')
    quality = _mapping(result.get('quality'), 'Scenario 6 quality')
    targets = _mapping(result.get('targets'), 'Scenario 6 targets')
    measurements = _mapping(result.get('measurements'), 'Scenario 6 measurements')
    _require(
        set(result)
        == {
            'identity',
            'measurements',
            'producer',
            'quality',
            'schema_version',
            'targets',
            'verdict',
        }
        and _exact_integer(result.get('schema_version'))
        and result.get('schema_version') == 1
        and result.get('producer') == 'robotest_phase4/acceptance_verifier',
        'Scenario 6 producer schema changed',
    )
    _require(
        set(identity)
        == {
            'completed_utc',
            'gz_partition',
            'managed_child',
            'ros_domain_id',
            'run_id',
            'source_git_commit',
            'source_git_dirty',
            'started_utc',
            'supervisor_unit',
        },
        'Scenario 6 identity schema changed',
    )
    git_sha = identity.get('source_git_commit')
    _require(isinstance(git_sha, str) and GIT_SHA.fullmatch(git_sha), 'Phase 4 Git SHA is invalid')
    _require(
        set(context)
        == {
            'active_overlay_target',
            'baseline_package',
            'cpuset',
            'isolation',
            'lifecycle_evidence',
            'package_directory',
            'run_id',
            'schema_version',
            'source_git_commit',
            'source_git_dirty',
            'started_utc',
            'upgrade_package',
        },
        'Phase 4 context producer schema changed',
    )
    _utc_timestamp(context.get('started_utc'), 'Phase 4 context started_utc')
    for name in ('baseline_package', 'upgrade_package', 'lifecycle_evidence'):
        record = _mapping(context.get(name), f'Phase 4 context {name}')
        _require(
            set(record) == {'path', 'sha256'}
            and isinstance(record.get('path'), str)
            and Path(record['path']).is_absolute()
            and isinstance(record.get('sha256'), str)
            and re.fullmatch(r'[0-9a-f]{64}', record['sha256']) is not None,
            f'Phase 4 context {name} is invalid',
        )
    isolation = _mapping(context.get('isolation'), 'Phase 4 context isolation')
    _require(
        isinstance(context.get('package_directory'), str)
        and Path(context['package_directory']).is_absolute()
        and isinstance(context.get('active_overlay_target'), str)
        and Path(context['active_overlay_target']).is_absolute()
        and context.get('cpuset') == '0-5'
        and set(isolation)
        == {
            'domain_was_unused',
            'gz_partition',
            'inspected_processes',
            'partition_was_unused',
            'ros_domain_id',
            'run_id',
            'schema_version',
            'unreadable_process_environments',
        }
        and _exact_integer(isolation.get('schema_version'))
        and isolation.get('schema_version') == 1
        and isolation.get('run_id') == run_directory.name
        and isolation.get('domain_was_unused') is True
        and isolation.get('partition_was_unused') is True
        and _exact_integer(isolation.get('unreadable_process_environments'))
        and isolation.get('unreadable_process_environments') == 0
        and _exact_integer(isolation.get('inspected_processes'))
        and isolation['inspected_processes'] >= 0
        and _exact_integer(isolation.get('ros_domain_id'))
        and 100 <= isolation['ros_domain_id'] <= 229
        and isinstance(isolation.get('gz_partition'), str)
        and re.fullmatch(r'[A-Za-z0-9_]+', isolation['gz_partition']) is not None,
        'Phase 4 context paths or isolation changed',
    )
    _require(
        identity.get('run_id') == run_directory.name
        and identity.get('ros_domain_id') == isolation.get('ros_domain_id')
        and identity.get('gz_partition') == isolation.get('gz_partition')
        and identity.get('managed_child') == 'robotest-stack'
        and identity.get('supervisor_unit') == 'robotest-supervisor.service'
        and identity.get('source_git_dirty') is False
        and _exact_integer(context.get('schema_version'))
        and context.get('schema_version') == 1
        and context.get('run_id') == run_directory.name
        and context.get('started_utc') == identity.get('started_utc')
        and context.get('source_git_commit') == git_sha
        and context.get('source_git_dirty') is False,
        'Phase 4 run identity is not a clean exact candidate',
    )
    started_utc = _utc_timestamp(identity.get('started_utc'), 'Scenario 6 started_utc')
    completed_utc = _utc_timestamp(identity.get('completed_utc'), 'Scenario 6 completed_utc')
    _require(completed_utc >= started_utc, 'Scenario 6 completion precedes its start')
    _require(_exact_json_equal(targets, PHASE4_TARGETS), 'Scenario 6 frozen targets changed')
    _require(
        set(measurements) == PHASE4_MEASUREMENT_KEYS,
        'Scenario 6 measurement schema changed',
    )
    _require(set(quality) == PHASE4_QUALITY_KEYS, 'Scenario 6 quality schema changed')
    integer_measurements = {
        name: measurements.get(name)
        for name in (
            'original_child_pgid',
            'original_child_pid',
            'replacement_child_pgid',
            'replacement_child_pid',
            'supervisor_main_pid',
        )
    }
    _require(
        all(
            isinstance(value, int) and not isinstance(value, bool) and value > 0
            for value in integer_measurements.values()
        )
        and measurements.get('replacement_child_pid') != measurements.get('original_child_pid')
        and measurements.get('replacement_child_pgid') != measurements.get('original_child_pgid')
        and measurements.get('supervisor_main_pid')
        not in {
            measurements.get('original_child_pid'),
            measurements.get('replacement_child_pid'),
        }
        and _exact_integer(measurements.get('replacement_child_start_count'))
        and measurements.get('replacement_child_start_count') == 1
        and _exact_integer(measurements.get('restart_scheduled_count'))
        and measurements.get('restart_scheduled_count') == 1
        and _exact_integer(measurements.get('interrupted_mission_exit_code'))
        and measurements.get('interrupted_mission_exit_code') != 0
        and _exact_integer(measurements.get('followup_mission_exit_code'))
        and measurements.get('followup_mission_exit_code') == 0
        and _finite_number(measurements.get('actual_restart_backoff_wall_s'))
        and 0.9 <= measurements['actual_restart_backoff_wall_s'] <= 3.0
        and _finite_number(measurements.get('ready_503_after_injection_wall_s'))
        and 0 <= measurements['ready_503_after_injection_wall_s'] <= 3.0
        and _finite_number(measurements.get('original_group_empty_after_injection_wall_s'))
        and 0 <= measurements['original_group_empty_after_injection_wall_s'] <= 5.0
        and _finite_number(measurements.get('supervisor_recovery_time_wall_s'))
        and 0 <= measurements['supervisor_recovery_time_wall_s'] <= 30.0
        and _finite_number(measurements.get('observed_ready_restore_after_503_wall_s'))
        and 0 <= measurements['observed_ready_restore_after_503_wall_s'] <= 30.0,
        'Scenario 6 PASS measurements violate frozen bounds',
    )
    _require(
        _exact_integer(verdict.get('failure_count'))
        and _exact_json_equal(
            verdict,
            {'accepted': True, 'failure_count': 0, 'failures': [], 'status': 'PASS'},
        ),
        'Scenario 6 verdict is not canonical PASS',
    )
    checks = _mapping(quality.get('checks'), 'Scenario 6 checks')
    _require(
        set(checks) == PHASE4_CHECKS and all(value is True for value in checks.values()),
        'Scenario 6 check set is incomplete or failed',
    )
    event_count = quality.get('event_count')
    timeline_count = quality.get('timeline_count')
    _require(
        isinstance(event_count, int)
        and not isinstance(event_count, bool)
        and event_count > 0
        and isinstance(timeline_count, int)
        and not isinstance(timeline_count, bool)
        and timeline_count > 0
        and _exact_integer(quality.get('event_trace_dropped'))
        and quality.get('event_trace_dropped') == 0,
        'Scenario 6 quality counters are invalid',
    )
    event_path = _regular_file(
        run_directory / 'supervisor-events.jsonl', 'Scenario 6 supervisor events'
    )
    timeline_path = _regular_file(run_directory / 'timeline.jsonl', 'Scenario 6 timeline')
    event_meta = _load_canonical_json(
        run_directory / 'supervisor-events.meta.json',
        'Scenario 6 event metadata',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    _require(
        len(event_path.read_bytes().splitlines()) == event_count
        and len(timeline_path.read_bytes().splitlines()) == timeline_count
        and _exact_integer(event_meta.get('dropped_events'))
        and event_meta.get('dropped_events') == 0,
        'Scenario 6 raw evidence counters do not reconcile',
    )
    raw_hashes = _mapping(quality.get('raw_evidence_sha256'), 'Scenario 6 raw evidence hashes')
    expected_raw_names = set(manifest) - {'scenario6-result.json', 'scenario6-result.csv'}
    _require(
        set(raw_hashes) == expected_raw_names and expected_raw_names >= PHASE4_REQUIRED_RAW,
        'Scenario 6 raw evidence hash coverage is not exact',
    )
    for name, digest in raw_hashes.items():
        _require(
            isinstance(name, str) and isinstance(digest, str), 'Scenario 6 raw hash is invalid'
        )
        relative = Path(name)
        _require(
            name == relative.as_posix()
            and not relative.is_absolute()
            and '..' not in relative.parts,
            f'Scenario 6 raw evidence path is unsafe: {name}',
        )
        path = _regular_file(run_directory / relative, f'Scenario 6 raw evidence {name}')
        _require(file_sha256(path) == digest, f'Scenario 6 raw evidence mismatch: {name}')
    csv_path = _regular_file(run_directory / 'scenario6-result.csv', 'Scenario 6 CSV')
    _require(csv_path.read_bytes() == _phase4_csv_bytes(result), 'Scenario 6 CSV differs from JSON')
    relative_result = expected_result.relative_to(run_directory).as_posix()
    _require(
        manifest.get(relative_result) == file_sha256(expected_result),
        'Phase 4 manifest does not bind Scenario 6 result',
    )
    phase4_module = _load_repository_module(
        repository,
        'tests/phase4_acceptance.py',
        'Phase 4 production acceptance module',
    )
    completed_value = identity.get('completed_utc')
    original_utc_now = phase4_module.utc_now
    phase4_module.utc_now = lambda: completed_value
    try:
        recomputed_result = phase4_module.evaluate_run(run_directory)
    except Exception as exc:
        raise EvidenceError(f'Phase 4 production replay failed: {exc}') from exc
    finally:
        phase4_module.utc_now = original_utc_now
    _require(
        _exact_json_equal(result, recomputed_result),
        'Scenario 6 result is not the exact production evaluation replay',
    )
    return {
        'git_sha': git_sha,
        'manifest_file_count': len(manifest),
        'run_directory': str(run_directory),
        'run_id': run_directory.name,
        'scenario6_path': str(expected_result),
        'scenario6_sha256': file_sha256(expected_result),
    }


def _git(repository: Path, arguments: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            'git',
            '-c',
            'core.fsmonitor=false',
            '-c',
            'submodule.recurse=false',
            '-C',
            str(repository),
            *arguments,
        ],
        capture_output=True,
        check=False,
        env=os.environ
        | {
            'GIT_NO_LAZY_FETCH': '1',
            'GIT_OPTIONAL_LOCKS': '0',
            'GIT_TERMINAL_PROMPT': '0',
        },
        text=True,
        timeout=30,
    )


def _git_bytes(repository: Path, arguments: list[str]) -> subprocess.CompletedProcess[bytes]:
    return subprocess.run(
        [
            'git',
            '-c',
            'core.fsmonitor=false',
            '-c',
            'submodule.recurse=false',
            '-C',
            str(repository),
            *arguments,
        ],
        capture_output=True,
        check=False,
        env=os.environ
        | {
            'GIT_NO_LAZY_FETCH': '1',
            'GIT_OPTIONAL_LOCKS': '0',
            'GIT_TERMINAL_PROMPT': '0',
        },
        timeout=30,
    )


def _validate_one_file_checksum(source: Path, label: str) -> tuple[Path, Path]:
    manifest = _regular_file(source.with_suffix('.SHA256SUMS'), f'{label} checksum manifest')
    validation = _regular_file(
        source.with_suffix('.checksum-validation.txt'), f'{label} checksum validation'
    )
    _require(manifest.stat().st_size <= 256, f'{label} checksum manifest is oversized')
    expected_manifest = f'{file_sha256(source)}  {source.name}\n'
    _require(manifest.read_text(encoding='ascii') == expected_manifest, f'{label} hash mismatch')
    _require(
        validation.read_text(encoding='utf-8') == f'{source.name}: OK\n',
        f'{label} checksum validation is not exact',
    )
    return manifest, validation


def _validate_remote_proof_document(
    proof: Mapping[str, Any],
    *,
    repository: Path,
    expected_sha: str,
    expected_mode: str,
    label: str,
) -> dict[str, Any]:
    _require(
        set(proof)
        == {
            'checked_at',
            'provenance',
            'repository',
            'run',
            'schema_version',
            'status',
            'verification_scope',
        }
        and _exact_integer(proof.get('schema_version'))
        and proof.get('schema_version') == 1
        and proof.get('status') == 'PASS'
        and proof.get('verification_scope') == REMOTE_PROOF_SCOPE,
        f'{label} top-level producer contract changed',
    )
    run = _mapping(proof.get('run'), f'{label} run')
    _require(
        set(run)
        == {
            'conclusion',
            'created_at',
            'head_sha',
            'run_id',
            'run_url',
            'status',
            'workflow_name',
        }
        and run.get('head_sha') == expected_sha
        and run.get('status') == 'completed'
        and run.get('conclusion') == 'success'
        and run.get('workflow_name') == 'RoboTest CI',
        f'{label} verdict or identity is invalid',
    )
    created_at = _utc_timestamp(run.get('created_at'), f'{label} run.created_at')
    checked_at = _utc_timestamp(proof.get('checked_at'), f'{label} checked_at')
    _require(checked_at >= created_at, f'{label} was checked before its workflow run')
    run_id = run.get('run_id')
    _require(
        isinstance(run_id, int) and not isinstance(run_id, bool) and 0 < run_id < 2**63,
        f'{label} run ID is invalid',
    )

    repository_record = _mapping(proof.get('repository'), f'{label} repository')
    _require(
        set(repository_record) == {'name_with_owner', 'url', 'visibility'},
        f'{label} repository schema changed',
    )
    repository_name = repository_record.get('name_with_owner')
    repository_url = repository_record.get('url')
    _require(
        isinstance(repository_name, str)
        and GITHUB_REPOSITORY.fullmatch(repository_name) is not None
        and repository_url == f'https://github.com/{repository_name}'
        and repository_record.get('visibility') == 'PUBLIC'
        and run.get('run_url') == f'{repository_url}/actions/runs/{run_id}',
        f'{label} repository or run URL identity is invalid',
    )

    provenance = _mapping(proof.get('provenance'), f'{label} provenance')
    _require(
        set(provenance) == {'command', 'local_resolved_sha', 'platform', 'tools'}
        and provenance.get('local_resolved_sha') == expected_sha,
        f'{label} provenance schema changed',
    )
    command = _mapping(provenance.get('command'), f'{label} command')
    argv = _list(command.get('argv'), f'{label} argv')
    _require(
        set(command) == {'argv', 'cwd'}
        and len(argv) == 3
        and argv[0] == 'scripts/verify_phase5.sh'
        and argv[1:] == [expected_mode, expected_sha]
        and command.get('cwd') == str(repository),
        f'{label} command is invalid',
    )
    platform = _mapping(provenance.get('platform'), f'{label} platform')
    _require(
        set(platform)
        == {'github_runner', 'os_release', 'python', 'ros_distro', 'uname', 'wsl_distro_name'},
        f'{label} platform schema changed',
    )
    github_runner = _mapping(platform.get('github_runner'), f'{label} GitHub runner')
    python_record = _mapping(platform.get('python'), f'{label} Python')
    _require(
        set(github_runner)
        == {'architecture', 'environment', 'image_os', 'image_version', 'operating_system'}
        and set(python_record) == {'executable', 'version'}
        and isinstance(python_record.get('executable'), str)
        and Path(python_record['executable']).is_absolute()
        and isinstance(python_record.get('version'), str)
        and python_record['version']
        and isinstance(platform.get('os_release'), dict)
        and isinstance(platform.get('uname'), list),
        f'{label} platform record is invalid',
    )
    tools = _mapping(provenance.get('tools'), f'{label} tools')
    expected_tools = {
        'gh': ['gh', '--version'],
        'git': ['git', '--version'],
        'sha256sum': ['sha256sum', '--version'],
    }
    _require(set(tools) == set(expected_tools), f'{label} tool set changed')
    for name, expected_argv in expected_tools.items():
        record = _mapping(tools[name], f'{label} tool {name}')
        output = record.get('output')
        executable = record.get('executable')
        _require(
            set(record) == {'argv', 'executable', 'output'}
            and record.get('argv') == expected_argv
            and isinstance(executable, str)
            and Path(executable).is_absolute()
            and isinstance(output, str)
            and output
            and len(output.encode()) <= 16_384,
            f'{label} tool record is invalid: {name}',
        )
    return {
        'checked_at': checked_at,
        'created_at': created_at,
        'repository_name': repository_name,
        'repository_url': repository_url,
        'run_id': run_id,
        'run_url': run.get('run_url'),
    }


def _local_aggregate_evidence(repository: Path, path: Path) -> dict[str, Any]:
    path = _regular_file(path, 'local aggregate summary').resolve(strict=True)
    phase0_root = _repository_directory(
        repository, 'artifacts/evidence/phase0', 'Phase 0 evidence root'
    )
    expected_path = (phase0_root / 'verify-all.json').resolve(strict=True)
    _require(path == expected_path, 'local aggregate is outside its canonical producer path')
    aggregate = _load_canonical_json(path, 'local aggregate summary')
    manifest, validation = _validate_one_file_checksum(path, 'local aggregate summary')
    _require(
        set(aggregate)
        == {
            'checked_at',
            'command',
            'mode',
            'outstanding_gates',
            'release_eligible',
            'release_evidence',
            'results',
            'schema_version',
            'source',
            'status',
            'verification_scope',
        },
        'local aggregate schema changed',
    )
    _require(
        _exact_integer(aggregate.get('schema_version'))
        and aggregate.get('schema_version') == 4
        and aggregate.get('mode') == 'static'
        and aggregate.get('status') == 'incomplete'
        and aggregate.get('release_eligible') is False
        and aggregate.get('release_evidence') is None,
        'local aggregate is not the canonical incomplete result',
    )
    _require(
        aggregate.get('verification_scope') == LOCAL_AGGREGATE_SCOPE
        and aggregate.get('outstanding_gates') == LOCAL_OUTSTANDING_GATES,
        'local aggregate verification scope changed',
    )
    _utc_timestamp(aggregate.get('checked_at'), 'local aggregate checked_at')
    command = _mapping(aggregate.get('command'), 'local aggregate command')
    _require(set(command) == {'argv', 'cwd'}, 'local aggregate command schema changed')
    argv = _list(command.get('argv'), 'local aggregate command argv')
    cwd = command.get('cwd')
    _require(
        len(argv) == 1
        and isinstance(argv[0], str)
        and argv[0].endswith('scripts/verify_all.sh')
        and isinstance(cwd, str)
        and Path(cwd).is_absolute(),
        'local aggregate command is not the bare verifier invocation',
    )
    source = _mapping(aggregate.get('source'), 'local aggregate source')
    _require(
        set(source)
        == {
            'git_dirty_end',
            'git_dirty_start',
            'git_sha_end',
            'git_sha_start',
            'git_status_porcelain_end',
            'git_status_porcelain_start',
            'generated_evidence_delta',
            'source_unchanged',
        },
        'local aggregate source schema changed',
    )
    git_sha = source.get('git_sha_start')
    _require(
        isinstance(git_sha, str)
        and GIT_SHA.fullmatch(git_sha) is not None
        and source.get('git_sha_end') == git_sha
        and source.get('git_dirty_start') is False
        and source.get('git_dirty_end') is True
        and source.get('git_status_porcelain_start') == ''
        and source.get('source_unchanged') is True,
        'local aggregate does not bind one clean unchanged candidate',
    )
    delta = _mapping(source.get('generated_evidence_delta'), 'generated evidence delta')
    _require(
        set(delta) == {'allowed_paths', 'changed_files'}
        and delta.get('allowed_paths') == ALLOWED_GENERATED_DELTA,
        'local aggregate generated-evidence allowlist changed',
    )
    changed_files = _list(delta.get('changed_files'), 'generated evidence changed files')
    changed_paths: list[str] = []
    for item in changed_files:
        record = _mapping(item, 'generated evidence file')
        path_name = record.get('path')
        _require(
            set(record) == {'bytes', 'path', 'sha256'}
            and isinstance(path_name, str)
            and path_name in ALLOWED_GENERATED_DELTA
            and path_name not in changed_paths,
            'generated evidence file record is invalid',
        )
        evidence_path = _regular_file(repository / path_name, 'generated Phase 0 evidence')
        _require(
            _exact_integer(record.get('bytes'))
            and record.get('bytes') >= 0
            and isinstance(record.get('sha256'), str)
            and re.fullmatch(r'[0-9a-f]{64}', record['sha256']) is not None
            and evidence_path.stat().st_size == record.get('bytes')
            and file_sha256(evidence_path) == record.get('sha256'),
            f'generated Phase 0 evidence changed: {path_name}',
        )
        changed_paths.append(path_name)
    _require(
        'artifacts/evidence/phase0/phase0-versions.json' in changed_paths,
        'Phase 0 timestamp evidence delta is absent',
    )
    expected_status = ''.join(f' M {name}\n' for name in sorted(changed_paths)).rstrip('\n')
    _require(
        source.get('git_status_porcelain_end') == expected_status,
        'local aggregate end status is outside the generated-evidence delta',
    )
    results = _list(aggregate.get('results'), 'local aggregate phase results')
    expected_results = [
        {
            'exit_code': '0',
            'phase': str(phase),
            'status': 'passed',
            'verifier': f'scripts/verify_phase{phase}.sh',
        }
        for phase in range(6)
    ]
    _require(
        results == expected_results, 'local aggregate does not contain exact Phase 0-5 PASS rows'
    )
    aggregate_paths = [path, manifest, validation]
    for evidence_path in aggregate_paths:
        relative = evidence_path.relative_to(repository).as_posix()
        _require(
            _git(repository, ['ls-files', '--error-unmatch', '--', relative]).returncode == 0,
            f'local aggregate is not tracked: {relative}',
        )
    return {
        'evidence_commit_paths': [
            *(item.relative_to(repository).as_posix() for item in aggregate_paths),
            *changed_paths,
        ],
        'git_sha': git_sha,
        'manifest_path': str(manifest),
        'path': str(path),
        'sha256': file_sha256(path),
        'validation_path': str(validation),
    }


def _phase5_evidence(
    repository: Path,
    proof_path: Path,
    local_evidence_commit_paths: list[str],
    release_document_paths: list[str],
) -> dict[str, Any]:
    expected_root = _repository_directory(
        repository, 'docs/results/phase-5', 'Phase 5 tracked evidence root'
    )
    proof_path = _regular_file(proof_path, 'Phase 5 remote proof').resolve(strict=True)
    _require(
        proof_path.parent == expected_root,
        'Phase 5 remote proof is outside docs/results/phase-5',
    )
    match = re.fullmatch(r'remote-([0-9a-f]{40})\.json', proof_path.name)
    _require(match is not None, 'Phase 5 remote proof filename is invalid')
    filename_sha = match.group(1)
    manifest_path, validation_path = _validate_one_file_checksum(proof_path, 'remote proof')
    proof = _load_canonical_json(proof_path, 'Phase 5 remote proof')
    proof_identity = _validate_remote_proof_document(
        proof,
        repository=repository,
        expected_sha=filename_sha,
        expected_mode='--remote',
        label='Phase 5 remote proof',
    )
    run_id = proof_identity['run_id']
    repository_name = proof_identity['repository_name']
    repository_url = proof_identity['repository_url']
    relative_paths = [
        path.relative_to(repository).as_posix()
        for path in (proof_path, manifest_path, validation_path)
    ]
    for relative in relative_paths:
        tracked = _git(repository, ['ls-files', '--error-unmatch', '--', relative])
        _require(tracked.returncode == 0, f'Phase 5 remote proof is not tracked: {relative}')
        _require(
            _git(repository, ['diff', '--quiet', '--', relative]).returncode == 0
            and _git(repository, ['diff', '--cached', '--quiet', '--', relative]).returncode == 0,
            f'Phase 5 remote proof has uncommitted content: {relative}',
        )
    index_flags = _git(repository, ['ls-files', '-v'])
    _require(
        index_flags.returncode == 0
        and all(line.startswith('H ') for line in index_flags.stdout.splitlines()),
        'release index contains assume-unchanged, skip-worktree, or non-cached entries',
    )
    status = _git(repository, ['status', '--porcelain=v1', '--untracked-files=all'])
    _require(status.returncode == 0 and status.stdout == '', 'release worktree is not clean')
    origin = _git(repository, ['remote', 'get-url', 'origin'])
    accepted_origins = {
        f'https://github.com/{repository_name}',
        f'https://github.com/{repository_name}.git',
        f'git@github.com:{repository_name}.git',
        f'ssh://git@github.com/{repository_name}.git',
    }
    _require(
        origin.returncode == 0 and origin.stdout.strip() in accepted_origins,
        'Phase 5 proof repository does not match origin',
    )
    parents = _git(repository, ['rev-list', '--parents', '-n', '1', 'HEAD'])
    parent_fields = parents.stdout.split()
    _require(
        parents.returncode == 0 and len(parent_fields) == 2 and parent_fields[1] == filename_sha,
        'HEAD is not a single-parent evidence-only commit directly after the candidate',
    )
    changed = _git(
        repository,
        ['diff-tree', '--no-commit-id', '--name-only', '-r', parent_fields[0]],
    )
    _require(
        changed.returncode == 0
        and set(changed.stdout.splitlines())
        == set(relative_paths) | set(local_evidence_commit_paths) | set(release_document_paths),
        'evidence-only commit changed files outside the remote proof trio',
    )
    for relative in sorted(set(changed.stdout.splitlines())):
        mode = _git(repository, ['ls-tree', parent_fields[0], '--', relative])
        fields = mode.stdout.split()
        _require(
            mode.returncode == 0 and fields and fields[0] == '100644',
            f'evidence-only commit path is not a regular 100644 blob: {relative}',
        )
    return {
        'candidate_git_sha': filename_sha,
        'checked_at': proof_identity['checked_at'].isoformat(),
        'manifest_path': str(manifest_path),
        'proof_path': str(proof_path),
        'proof_sha256': file_sha256(proof_path),
        'evidence_commit_git_sha': parent_fields[0],
        'run_id': run_id,
        'created_at': proof_identity['created_at'].isoformat(),
        'run_url': proof_identity['run_url'],
        'repository_name': repository_name,
        'repository_url': repository_url,
        'validation_path': str(validation_path),
    }


def _phase5_evidence_commit_remote(
    repository: Path,
    proof_path: Path,
    candidate_remote: Mapping[str, Any],
) -> dict[str, Any]:
    expected_root = _repository_directory(
        repository,
        'artifacts/evidence/phase5/remote-evidence-commit',
        'Phase 5 raw remote evidence root',
    )
    proof_path = _regular_file(proof_path, 'evidence-commit remote proof').resolve(strict=True)
    _require(
        proof_path.parent == expected_root,
        'evidence-commit remote proof is outside its ignored raw directory',
    )
    match = re.fullmatch(r'remote-([0-9a-f]{40})\.json', proof_path.name)
    _require(match is not None, 'evidence-commit remote proof filename is invalid')
    evidence_sha = match.group(1)
    manifest, validation = _validate_one_file_checksum(proof_path, 'evidence-commit remote proof')
    proof = _load_canonical_json(proof_path, 'evidence-commit remote proof')
    head = _git(repository, ['rev-parse', '--verify', 'HEAD'])
    _require(
        head.returncode == 0 and head.stdout.strip() == evidence_sha,
        'evidence-commit remote proof verdict or SHA is invalid',
    )
    proof_identity = _validate_remote_proof_document(
        proof,
        repository=repository,
        expected_sha=evidence_sha,
        expected_mode='--remote-evidence-commit',
        label='evidence-commit remote proof',
    )
    run_id = proof_identity['run_id']
    _require(
        proof_identity['repository_name'] == candidate_remote['repository_name']
        and proof_identity['repository_url'] == candidate_remote['repository_url'],
        'candidate and evidence-commit remote repositories differ',
    )
    candidate_checked_at = datetime.fromisoformat(candidate_remote['checked_at'])
    _require(
        run_id != candidate_remote['run_id']
        and proof_identity['created_at'] > candidate_checked_at,
        'evidence-commit remote run is not a distinct later workflow run',
    )
    relative_paths = [
        path.relative_to(repository).as_posix() for path in (proof_path, manifest, validation)
    ]
    for relative in relative_paths:
        _require(
            _git(repository, ['check-ignore', '-q', '--', relative]).returncode == 0
            and _git(repository, ['ls-files', '--error-unmatch', '--', relative]).returncode != 0,
            f'evidence-commit remote proof is not ignored raw evidence: {relative}',
        )
    return {
        'evidence_commit_git_sha': evidence_sha,
        'manifest_path': str(manifest),
        'proof_path': str(proof_path),
        'proof_sha256': file_sha256(proof_path),
        'run_id': run_id,
        'run_url': proof_identity['run_url'],
        'validation_path': str(validation),
    }


def validate_release_evidence(
    repository: Path,
    local_aggregate: Path,
    phase3_candidate_root: Path,
    phase3_aggregate: Path,
    phase4_run_directory: Path,
    phase4_scenario6: Path,
    phase5_remote_proof: Path,
    phase5_evidence_commit_remote_proof: Path,
) -> dict[str, Any]:
    """Validate exact selected evidence without running or discovering acceptance work."""
    repository = _resolved_directory(repository, 'repository')
    _activate_repository_packages(repository)
    local = _local_aggregate_evidence(repository, local_aggregate)
    phase3 = _phase3_evidence(repository, phase3_candidate_root, phase3_aggregate)
    phase4 = _phase4_evidence(repository, phase4_run_directory, phase4_scenario6)
    candidate_sha = local['git_sha']
    candidate_readme = _git_bytes(repository, ['show', f'{candidate_sha}:README.md'])
    candidate_claims = _git_bytes(
        repository, ['show', f'{candidate_sha}:config/release-claims.json']
    )
    _require(
        candidate_readme.returncode == 0 and candidate_claims.returncode == 0,
        'candidate commit lacks README or release claims',
    )
    release_documents = validate_release_documents(
        repository,
        phase3_aggregate,
        phase4_scenario6,
        candidate_readme.stdout,
        candidate_claims.stdout,
    )
    release_document_paths = [
        *release_documents['paths'],
        release_documents['readme_path'],
        release_documents['claims_path'],
    ]
    phase5 = _phase5_evidence(
        repository,
        phase5_remote_proof,
        local['evidence_commit_paths'],
        release_document_paths,
    )
    phase5_evidence_commit = _phase5_evidence_commit_remote(
        repository, phase5_evidence_commit_remote_proof, phase5
    )
    _require(
        candidate_sha == phase3['git_sha'] == phase4['git_sha'] == phase5['candidate_git_sha'],
        'release evidence does not bind one candidate Git SHA',
    )
    return {
        'candidate_git_sha': candidate_sha,
        'local_aggregate': local,
        'release_documents': release_documents,
        'phase3': phase3,
        'phase4': phase4,
        'phase5': phase5,
        'phase5_evidence_commit': phase5_evidence_commit,
        'release_eligible': True,
        'schema_version': 1,
        'status': 'PASS',
        'verification_scope': (
            'Read-only revalidation of caller-selected Phase 3 campaign, Phase 4 Scenario 6, '
            'tracked exact-SHA Phase 5 remote evidence, and a prior local aggregate; no latest '
            'discovery or execution'
        ),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--local-aggregate', type=Path, required=True)
    parser.add_argument('--phase3-candidate-root', type=Path, required=True)
    parser.add_argument('--phase3-aggregate', type=Path, required=True)
    parser.add_argument('--phase4-run-directory', type=Path, required=True)
    parser.add_argument('--phase4-scenario6', type=Path, required=True)
    parser.add_argument('--phase5-remote-proof', type=Path, required=True)
    parser.add_argument('--phase5-evidence-commit-remote-proof', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        report = validate_release_evidence(
            arguments.repository,
            arguments.local_aggregate,
            arguments.phase3_candidate_root,
            arguments.phase3_aggregate,
            arguments.phase4_run_directory,
            arguments.phase4_scenario6,
            arguments.phase5_remote_proof,
            arguments.phase5_evidence_commit_remote_proof,
        )
        atomic_write_json(arguments.output, report)
    except (
        EvidenceError,
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
    ) as error:
        raise SystemExit(str(error)) from error
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
