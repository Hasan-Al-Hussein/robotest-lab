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
import re
import shlex
import subprocess
import sys
from collections.abc import Mapping, Sequence
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
PHASE3_ENDPOINT_GID_PATTERN = re.compile(r'^[0-9a-f]{32}$')
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
PHASE3_GRAPH_PROCESS_TIMEOUTS = {
    'graph_gate': 100.0,
    'mission_graph_gate': 30.0,
}
PHASE3_GRAPH_WATCHED_PROCESS_ROLES = ('full_stack', 'mission_runner')
PHASE3_PREREQUISITE_PROCESS_ROLES = (
    'contact_drain',
    'contact_stream_final_gate',
    'domain_cleanup',
    'domain_preflight',
    'full_stack',
    'goal_observer',
    'graph_gate',
    'lifecycle_gate',
    'metrics_collector',
    'mission_graph_gate',
    'mission_runner',
    'partition_cleanup',
    'partition_preflight',
    'runtime_gate',
    'scenario_controller',
    'startup_gate',
)
PHASE3_GRAPH_BINDING_PATHS = {
    'actions_text_sha256': 'actions.txt',
    'graph_json_sha256': 'graph.json',
    'nodes_text_sha256': 'nodes.txt',
    'services_text_sha256': 'services.txt',
    'topics_text_sha256': 'topics.txt',
}
PHASE3_PREREQUISITE_POST_PATHS = {
    'PASS.json',
    'PASS.json.sha256',
    'analysis-request.json',
    'analysis-request.json.sha256',
    'orchestrator.json',
    'orchestrator.json.sha256',
    'prerequisite-manifest.json',
    'prerequisite-manifest.json.sha256',
}
PHASE3_PREREQUISITE_RAW_PATHS = {
    'capture.json',
    'capture.json.sha256',
    'component-exits.json',
    'contact-drain.json',
    'contact-drain.json.sha256',
    'contact-gate-revalidation.json',
    'contact-gate-revalidation.json.sha256',
    'contact-progress.json',
    'contact-stream-final-gate.json',
    'contact-stream-final-gate.json.sha256',
    'domain-cleanup.json',
    'domain-cleanup.json.sha256',
    'domain-preflight.json',
    'domain-preflight.json.sha256',
    'goal-binding-reconciliation.json',
    'goal-observer.arm',
    'goal-observer.armed.json',
    'goal-observer.json',
    'goal-observer.json.sha256',
    'goal-observer.ready.json',
    'full-stack-combined.log',
    'full-stack-final-log-gate.json',
    'full-stack-final-log-gate.json.sha256',
    'lifecycle-ready.json',
    'lifecycle-startup-result.json',
    'metrics.ready.json',
    'metrics.stop',
    'mission-result.csv',
    'mission-result.json',
    'mission-result.json.sha256',
    'resources.jsonl',
    'runtime-gate.json',
    'runtime-gate.json.sha256',
    'scenario-result.json',
    'scenario-result.json.sha256',
    'scenario.ready.json',
    'startup-gate.json',
    'trial-context.json',
    'trial-context.json.sha256',
}
PHASE3_PREREQUISITE_RAW_PATHS.update(
    {
        f'lifecycle-ready-{node_name}.txt'
        for node_name in (
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
    }
)
PHASE3_RUNTIME_GATE_ENDPOINT_KEYS = {
    'depth',
    'durability',
    'gid',
    'history',
    'node',
    'reliability',
    'topic_type',
}
PHASE3_RUNTIME_GATE_TOPIC_KEYS = {
    'bounded_depth_live_proven',
    'exact_depth_live_proven',
    'expected',
    'publisher_qos_pass',
    'publishers',
    'qos_checks',
    'qos_introspection_complete',
    'subscriber_qos_pass',
    'subscribers',
}
PHASE3_CANDIDATE_RUNTIME_GATE_KEYS = {
    'attempt_count',
    'autonomy_validation_leaks',
    'bounded_depth_live_proven_for_all_endpoints',
    'cmd_vel_owner_pass',
    'command_subscriber_ownership',
    'contact_aggregator_binary_attestation',
    'contact_gate_binary_attestation',
    'contact_subscriber_ownership',
    'elapsed_wall_s',
    'exact_static_qos_depth_contract',
    'fused_clock_subscriber_ownership_pass',
    'legacy_fault_service_absent',
    'mode',
    'namespace_isolation_pass',
    'nodes',
    'producer',
    'publisher_ownership',
    'qos_contract_pass',
    'qos_introspection_complete',
    'required_nodes_missing',
    'required_nodes_outside_namespace',
    'scenario_services_missing',
    'schema_version',
    'topics',
    'validation_autonomy_isolation_pass',
    'verdict',
}
PHASE3_CONTACT_STREAM_RUNTIME_GATE_KEYS = {
    'attempt_count',
    'contact_aggregator_binary_attestation',
    'contact_gate_binary_attestation',
    'contact_stream_gate_present',
    'elapsed_wall_s',
    'mode',
    'nodes',
    'producer',
    'publisher_ownership',
    'qos_contract_pass',
    'raw_subscriber_ownership',
    'schema_version',
    'topics',
    'verdict',
}
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
    'lifecycle_startup_result_is_exact',
    'observed_recovery_is_bounded',
    'only_one_systemd_owned_ros_service',
    'original_process_group_empty_within_5s',
    'package_artifacts_match_lifecycle',
    'package_lifecycle_passed',
    'package_source_binding_passed',
    'package_source_rebuild_is_exact',
    'post_observation_shutdown_is_exact',
    'published_process_groups_are_bound',
    'readiness_false_event_present',
    'readiness_false_throughout_unavailable_interval',
    'readiness_true_event_present',
    'ready_503_within_3s',
    'ready_restored_within_30s_of_detection',
    'replacement_process_group_is_new',
    'restored_status_is_exactly_ready',
    'run_scoped_supervisor_config_is_exact',
    'runtime_cpu_affinity_is_limited',
    'runtime_staging_is_bound',
    'source_snapshot_unchanged_and_self_consistent',
    'supervisor_main_pid_stable',
    'supervisor_status_snapshots_are_exact',
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
    'initial-status.json',
    'installed-package-state.json',
    'interrupted-outcome.json',
    'isolation.json',
    'lifecycle-startup-result.json',
    'original-group-empty.json',
    'overlay-install-manifest.json',
    'overlay-provenance.json',
    'overlay-source-manifest.json',
    'package-binding.json',
    'package-integrity.json',
    'package-lifecycle.json',
    'package-source-rebuild.json',
    'ready-restored-status.json',
    'runtime-affinity-initial.json',
    'runtime-affinity-restored.json',
    'runtime-staging.json',
    'service-final.json',
    'source-snapshot-after.json',
    'source-snapshot-before.json',
    'supervisor-config.json',
    'supervisor-config-check.txt',
    'supervisor-events.jsonl',
    'supervisor-events.meta.json',
    'supervisor-status-final.json',
    'systemd-dropin.conf',
    'systemd-owners-initial.json',
    'systemd-owners-restored.json',
    'timeline.jsonl',
}
PHASE4_AFFINITY_KEYS = {
    'captured_utc',
    'controller_count',
    'controller_pid',
    'expected_cpuset',
    'main_pid',
    'managed_child_pgid',
    'managed_child_pid',
    'phase',
    'process_count',
    'processes',
    'schema_version',
    'snapshot_stable',
    'unit',
    'unit_cgroup',
    'verdict',
}
PHASE4_AFFINITY_PROCESS_KEYS = {
    'cgroup',
    'cpus_allowed',
    'cpus_allowed_list',
    'executable',
    'pgid',
    'pid',
    'ppid',
    'role',
    'start_time_ticks',
}
PHASE4_LIFECYCLE_STARTUP_KEYS = {
    'accepted',
    'command',
    'completed_utc',
    'discovery_grace_sec',
    'elapsed_wall_sec',
    'exit_code',
    'failure_kind',
    'failure_message',
    'response_timeout_sec',
    'schema_version',
    'service_name',
    'service_timeout_sec',
    'started_utc',
    'verdict',
    'watch_pid',
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
CONTACT_GATE_BUILD_PATH = 'build/robotest_sim/contact_stream_gate'
CONTACT_GATE_INSTALL_PATH = 'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
CONTACT_AGGREGATOR_BUILD_PATH = 'build/robotest_sim/librobotest_contact_aggregator_system.so'
CONTACT_AGGREGATOR_INSTALL_PATH = (
    'install/robotest_sim/lib/robotest_sim/librobotest_contact_aggregator_system.so'
)
PHASE3_POSITIVE_PROCESS_ROLES = (
    'domain_preflight',
    'partition_preflight',
    'sim_launch',
    'metrics_collector',
    'contact_control_driver',
    'runtime_gate',
    'contact_stream_final_gate',
    'domain_cleanup',
    'partition_cleanup',
)
PHASE3_POSITIVE_TOP_LEVEL_COMPONENT_PATHS = frozenset(
    {
        'capture.json',
        'command-progress.json',
        'contact-control-result.json',
        'contact-control-result.json.sha256',
        'contact-control.ready.json',
        'contact-control.arm.json',
        'contact-control.armed.json',
        'contact-gate-revalidation.json',
        'contact-gate-revalidation.json.sha256',
        'contact-progress.json',
        'contact-stream-final-gate.json',
        'contact-stream-final-gate.json.sha256',
        'domain-cleanup.json',
        'domain-cleanup.json.sha256',
        'domain-preflight.json',
        'domain-preflight.json.sha256',
        'metrics.ready.json',
        'metrics.stop',
        'resources.jsonl',
        'runtime-gate.json',
        'runtime-gate.json.sha256',
    }
)
PHASE3_POSITIVE_COMPONENT_PATHS = PHASE3_POSITIVE_TOP_LEVEL_COMPONENT_PATHS | frozenset(
    f'processes/{role}.{suffix}'
    for role in PHASE3_POSITIVE_PROCESS_ROLES
    for suffix in ('process.json', 'stdout.log', 'stderr.log')
)
PHASE3_BOUNDED_PROCESS_KEYS = {
    'command',
    'cwd',
    'finished_steady_ns',
    'group_confirmed_empty',
    'pgid',
    'pid',
    'returncode',
    'role',
    'started_steady_ns',
    'stderr',
    'stdout',
    'timed_out',
    'wall_timeout_s',
    'wrapped_command',
}
PHASE3_BOUNDED_STREAM_KEYS = {
    'error',
    'maximum_bytes',
    'observed_bytes',
    'overflow',
    'retained_bytes',
}
PHASE3_FINAL_LAUNCH_LOG_GATE_KEYS = {
    'failure_kind',
    'failure_message',
    'launch_log_path',
    'launch_log_sha256',
    'launch_log_size_bytes',
    'launch_stopped_utc',
    'lifecycle_timeout_recovery',
    'line_count',
    'match_count',
    'matches',
    'scanned_utc',
    'schema_version',
    'signature_definitions',
    'verdict',
}
PHASE3_LIFECYCLE_TIMEOUT_RECOVERY_KEYS = {
    'lifecycle_ready_path',
    'lifecycle_ready_sha256',
    'lifecycle_ready_size_bytes',
    'recovered_line_count',
    'recovered_lines',
}
PHASE3_RECOVERED_LIFECYCLE_TIMEOUT_KEYS = {
    'line_number',
    'service_name',
    'text',
    'timed_out_attempts',
}


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
            'build_install_samefile',
            'build_install_sha256_match',
            'build_path',
            'build_regular_executable',
            'build_sha256',
            'installed_declared_is_symlink',
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
        and _exact_integer(binding.get('schema_version'))
        and binding.get('schema_version') == 1
        and binding.get('build_path') == CONTACT_GATE_BUILD_PATH
        and binding.get('installed_declared_path') == CONTACT_GATE_INSTALL_PATH
        and isinstance(binding.get('installed_declared_is_symlink'), bool)
        and isinstance(binding.get('build_install_samefile'), bool)
        and binding.get('build_install_build_id_match') is True
        and binding.get('build_install_sha256_match') is True
        and binding.get('build_embedded_source_inventory_match') is True
        and binding.get('build_regular_executable') is True
        and binding.get('installed_declared_samefile') is True
        and binding.get('installed_regular_executable') is True
        and binding.get('installed_embedded_source_inventory_match') is True,
        'Phase 3 contact gate build/install binding changed',
    )
    installed_declared_is_symlink = binding['installed_declared_is_symlink']
    expected_installed_path = (
        CONTACT_GATE_BUILD_PATH if installed_declared_is_symlink else CONTACT_GATE_INSTALL_PATH
    )
    _require(
        binding.get('installed_path') == expected_installed_path
        and (not installed_declared_is_symlink or binding.get('build_install_samefile') is True),
        'Phase 3 contact gate declared/resolved install path is invalid',
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


def _validate_contact_aggregator_binary_binding(value: object) -> None:
    binding = _mapping(value, 'Phase 3 contact aggregator binary binding')
    _require(
        set(binding)
        == {
            'build_embedded_source_inventory_match',
            'build_embedded_source_inventory_sha256',
            'build_elf_build_id',
            'build_install_build_id_match',
            'build_install_embedded_source_inventory_match',
            'build_install_samefile',
            'build_install_sha256_match',
            'build_path',
            'build_regular_file',
            'build_sha256',
            'installed_declared_is_symlink',
            'installed_declared_path',
            'installed_embedded_source_inventory_match',
            'installed_embedded_source_inventory_sha256',
            'installed_elf_build_id',
            'installed_path',
            'installed_regular_file',
            'installed_sha256',
            'package',
            'schema_version',
            'source_inventory_sha256',
        }
        and binding.get('package') == 'robotest_sim'
        and _exact_integer(binding.get('schema_version'))
        and binding.get('schema_version') == 1
        and binding.get('build_path') == CONTACT_AGGREGATOR_BUILD_PATH
        and binding.get('installed_declared_path') == CONTACT_AGGREGATOR_INSTALL_PATH
        and isinstance(binding.get('installed_declared_is_symlink'), bool)
        and isinstance(binding.get('build_install_samefile'), bool)
        and binding.get('build_regular_file') is True
        and binding.get('installed_regular_file') is True
        and binding.get('build_install_build_id_match') is True
        and binding.get('build_install_embedded_source_inventory_match') is True
        and binding.get('build_install_sha256_match') is True
        and binding.get('build_embedded_source_inventory_match') is True
        and binding.get('installed_embedded_source_inventory_match') is True,
        'Phase 3 contact aggregator build/install binding changed',
    )
    installed_declared_is_symlink = binding['installed_declared_is_symlink']
    expected_installed_path = (
        CONTACT_AGGREGATOR_BUILD_PATH
        if installed_declared_is_symlink
        else CONTACT_AGGREGATOR_INSTALL_PATH
    )
    _require(
        binding.get('installed_path') == expected_installed_path
        and (not installed_declared_is_symlink or binding.get('build_install_samefile') is True),
        'Phase 3 contact aggregator declared/resolved install path is invalid',
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
            f'Phase 3 contact aggregator {field} is invalid',
        )
    _require(
        binding.get('build_embedded_source_inventory_sha256')
        == binding.get('source_inventory_sha256')
        and binding.get('installed_embedded_source_inventory_sha256')
        == binding.get('source_inventory_sha256')
        and binding.get('build_sha256') == binding.get('installed_sha256'),
        'Phase 3 contact aggregator embedded source or build/install hash differs',
    )
    _require(
        isinstance(binding.get('build_elf_build_id'), str)
        and re.fullmatch(r'[0-9a-f]+', binding['build_elf_build_id']) is not None
        and binding.get('installed_elf_build_id') == binding.get('build_elf_build_id'),
        'Phase 3 contact aggregator ELF build ID binding is invalid',
    )


def _validate_positive_gate_workspace_paths(
    gate: Mapping[str, Any],
    *,
    repository: Path,
    build_binding: Mapping[str, Any],
    label: str,
) -> None:
    """Bind both live binary attestations to this exact release workspace."""
    gate_binding = _mapping(
        build_binding.get('contact_gate_binary'),
        'Phase 3 contact gate build binding',
    )
    gate_path = (repository / gate_binding['installed_path']).resolve(strict=True)
    gate_stat = gate_path.stat()
    gate_attestation = _mapping(
        gate.get('contact_gate_binary_attestation'),
        f'{label} contact gate attestation',
    )
    gate_cmdline_sha256 = gate_attestation.get('live_cmdline_sha256')
    _require(
        gate_attestation.get('live_executable_path') == str(gate_path)
        and gate_attestation.get('live_executable_link') == str(gate_path)
        and gate_attestation.get('live_device') == gate_stat.st_dev
        and gate_attestation.get('live_inode') == gate_stat.st_ino
        and gate_attestation.get('live_size_bytes') == gate_stat.st_size
        and gate_attestation.get('installed_device') == gate_stat.st_dev
        and gate_attestation.get('installed_inode') == gate_stat.st_ino
        and isinstance(gate_cmdline_sha256, str)
        and re.fullmatch(r'[0-9a-f]{64}', gate_cmdline_sha256) is not None,
        f'{label} contact gate attestation is not bound to this workspace',
    )

    aggregator_binding = _mapping(
        build_binding.get('contact_aggregator_binary'),
        'Phase 3 contact aggregator build binding',
    )
    aggregator_path = (repository / aggregator_binding['installed_path']).resolve(strict=True)
    aggregator_stat = aggregator_path.stat()
    aggregator_attestation = _mapping(
        gate.get('contact_aggregator_binary_attestation'),
        f'{label} contact aggregator attestation',
    )
    expected_mapping_paths = [str(aggregator_path)]
    mapping_count = aggregator_attestation.get('live_mapping_count')
    mapping_fingerprint = aggregator_attestation.get('live_mapping_fingerprint_sha256')
    stable_identity = _mapping(
        aggregator_attestation.get('stable_identity'),
        f'{label} contact aggregator stable identity',
    )
    _require(
        aggregator_attestation.get('installed_device') == aggregator_stat.st_dev
        and aggregator_attestation.get('installed_inode') == aggregator_stat.st_ino
        and aggregator_attestation.get('installed_size_bytes') == aggregator_stat.st_size
        and _exact_integer(mapping_count)
        and 1 <= mapping_count <= 64
        and aggregator_attestation.get('live_mapping_device') == aggregator_stat.st_dev
        and aggregator_attestation.get('live_mapping_inode') == aggregator_stat.st_ino
        and aggregator_attestation.get('live_mapping_paths') == expected_mapping_paths
        and isinstance(mapping_fingerprint, str)
        and re.fullmatch(r'[0-9a-f]{64}', mapping_fingerprint) is not None
        and stable_identity.get('installed_device') == aggregator_stat.st_dev
        and stable_identity.get('installed_inode') == aggregator_stat.st_ino
        and stable_identity.get('live_mapping_device') == aggregator_stat.st_dev
        and stable_identity.get('live_mapping_inode') == aggregator_stat.st_ino
        and stable_identity.get('live_mapping_paths') == expected_mapping_paths
        and stable_identity.get('live_mapping_fingerprint_sha256') == mapping_fingerprint,
        f'{label} contact aggregator attestation is not bound to this workspace',
    )


def _validate_positive_component_processes(
    positive_directory: Path,
    *,
    repository: Path,
    positive_plan: Mapping[str, Any],
) -> None:
    """Validate every positive-control process record, retained log, and command."""
    workspace = repository.resolve()
    process_directory = positive_directory / 'processes'
    processes: dict[str, dict[str, Any]] = {}
    pids: set[int] = set()
    for role in PHASE3_POSITIVE_PROCESS_ROLES:
        label = f'Phase 3 positive-control {role} process'
        process = _load_canonical_json(
            process_directory / f'{role}.process.json',
            label,
            maximum_bytes=64 * 1024,
        )
        _require(set(process) == PHASE3_BOUNDED_PROCESS_KEYS, f'{label} fields are invalid')
        pid = process.get('pid')
        started = process.get('started_steady_ns')
        finished = process.get('finished_steady_ns')
        returncode = process.get('returncode')
        _require(
            process.get('role') == role
            and process.get('cwd') == str(workspace)
            and _exact_integer(pid)
            and pid > 0
            and process.get('pgid') == pid
            and pid not in pids
            and _exact_integer(started)
            and started > 0
            and _exact_integer(finished)
            and finished >= started
            and _exact_integer(returncode)
            and (role == 'sim_launch' or returncode == 0)
            and process.get('timed_out') is False
            and process.get('group_confirmed_empty') is True,
            f'{label} identity, lifetime, or exit state is invalid',
        )
        pids.add(pid)
        for stream_name in ('stdout', 'stderr'):
            stream_label = f'{label} {stream_name}'
            stream = _mapping(process.get(stream_name), stream_label)
            maximum_bytes = stream.get('maximum_bytes')
            observed_bytes = stream.get('observed_bytes')
            retained_bytes = stream.get('retained_bytes')
            _require(
                set(stream) == PHASE3_BOUNDED_STREAM_KEYS
                and maximum_bytes == 8 * 1024 * 1024
                and _exact_integer(observed_bytes)
                and _exact_integer(retained_bytes)
                and 0 <= retained_bytes <= maximum_bytes
                and observed_bytes == retained_bytes
                and stream.get('overflow') is False
                and stream.get('error') is None,
                f'{stream_label} fields are invalid',
            )
            log_path = _regular_file(
                process_directory / f'{role}.{stream_name}.log',
                f'{stream_label} retained log',
            )
            _require(
                log_path.stat().st_size == retained_bytes,
                f'{stream_label} retained byte count differs from its log',
            )
        processes[role] = process

    domain_id = positive_plan['ros_domain_id']
    partition = positive_plan['gz_partition']
    run_id = positive_plan['run_id']
    sim_pid = processes['sim_launch']['pid']
    driver_pid = processes['contact_control_driver']['pid']
    script = str(workspace / 'tests/phase3_runtime_gate.py')
    result_path = str((positive_directory / 'contact-control-result.json').resolve())
    ready_path = str((positive_directory / 'contact-control.ready.json').resolve())
    arm_path = str((positive_directory / 'contact-control.arm.json').resolve())
    armed_path = str((positive_directory / 'contact-control.armed.json').resolve())
    command_progress_path = str((positive_directory / 'command-progress.json').resolve())
    expected_commands = {
        'domain_preflight': [
            'python3',
            script,
            '--mode',
            'empty',
            '--output',
            str((positive_directory / 'domain-preflight.json').resolve()),
            '--workspace',
            str(workspace),
            '--wall-timeout-s',
            '10.0',
        ],
        'partition_preflight': ['gz', 'topic', '-l'],
        'sim_launch': [
            'ros2',
            'launch',
            'robotest_sim',
            'sim.launch.py',
            'namespace:=robotest',
            'seed:=42',
            'headless:=true',
            'render_sensors:=true',
            'rviz:=false',
        ],
        'metrics_collector': [
            'ros2',
            'run',
            'robotest_metrics',
            'metrics_collector',
            '--output',
            str((positive_directory / 'capture.json').resolve()),
            '--ready-file',
            str((positive_directory / 'metrics.ready.json').resolve()),
            '--stop-file',
            str((positive_directory / 'metrics.stop').resolve()),
            '--contact-progress-file',
            str((positive_directory / 'contact-progress.json').resolve()),
            '--command-progress-file',
            command_progress_path,
            '--command-progress-run-id',
            run_id,
            '--wall-timeout-s',
            '360',
            '--ros-args',
            '-r',
            '__ns:=/robotest',
        ],
        'contact_control_driver': [
            'ros2',
            'run',
            'robotest_scenarios',
            'contact_control_driver',
            '--output',
            result_path,
            '--ready-file',
            ready_path,
            '--arm-file',
            arm_path,
            '--armed-file',
            armed_path,
            '--command-progress-file',
            command_progress_path,
            '--run-id',
            run_id,
            '--coverage-manifest',
            str(workspace / 'config/collision-coverage.yaml'),
            '--wall-timeout-s',
            '30.0',
            '--ros-args',
            '-r',
            '__ns:=/robotest',
        ],
        'runtime_gate': [
            'python3',
            script,
            '--mode',
            'positive-control',
            '--output',
            str((positive_directory / 'runtime-gate.json').resolve()),
            '--workspace',
            str(workspace),
            '--wall-timeout-s',
            '20.0',
            '--watch-pid',
            str(driver_pid),
            '--launch-pid',
            str(sim_pid),
            '--expected-domain-id',
            str(domain_id),
            '--expected-gz-partition',
            partition,
        ],
        'contact_stream_final_gate': [
            'python3',
            script,
            '--mode',
            'contact-stream',
            '--output',
            str((positive_directory / 'contact-stream-final-gate.json').resolve()),
            '--workspace',
            str(workspace),
            '--wall-timeout-s',
            '10.0',
            '--watch-pid',
            str(sim_pid),
            '--launch-pid',
            str(sim_pid),
            '--expected-domain-id',
            str(domain_id),
            '--expected-gz-partition',
            partition,
        ],
        'domain_cleanup': [
            'python3',
            script,
            '--mode',
            'empty',
            '--output',
            str((positive_directory / 'domain-cleanup.json').resolve()),
            '--workspace',
            str(workspace),
            '--wall-timeout-s',
            '15.0',
        ],
        'partition_cleanup': ['gz', 'topic', '-l'],
    }
    expected_timeouts = {
        'domain_preflight': 15.0,
        'partition_preflight': 15.0,
        'sim_launch': 120.0,
        'metrics_collector': 370.0,
        'contact_control_driver': 45.0,
        'runtime_gate': 25.0,
        'contact_stream_final_gate': 15.0,
        'domain_cleanup': 20.0,
        'partition_cleanup': 15.0,
    }
    for role in PHASE3_POSITIVE_PROCESS_ROLES:
        process = processes[role]
        command = expected_commands[role]
        timeout = expected_timeouts[role]
        _require(
            process.get('command') == command
            and process.get('wall_timeout_s') == timeout
            and process.get('wrapped_command')
            == [
                'timeout',
                '--signal=TERM',
                '--kill-after=10s',
                f'{timeout:.3f}s',
                *command,
            ],
            f'Phase 3 positive-control {role} process command binding is invalid',
        )


def _phase3_graph_probe_command(
    repository: Path,
    run_root: Path,
    *,
    orchestration: Any,
    mission_client: bool,
    watch_pid: int,
) -> list[str]:
    """Rebuild the frozen graph-probe argv for one campaign gate."""
    contracts = orchestration.phase3_graph_contracts(mission_client)
    graph_wall_timeout_s = (
        orchestration.PHASE3_GRAPH_MISSION_WALL_TIMEOUT_S
        if mission_client
        else orchestration.PHASE3_GRAPH_WALL_TIMEOUT_S
    )
    prefix = 'mission-' if mission_client else ''
    command = ['python3', str(repository / 'tests/phase2_graph_probe.py')]
    for name, type_name in contracts['topics'].items():
        command.extend(['--topic', f'{name}={type_name}'])
    for name, type_name in contracts['services'].items():
        command.extend(['--service', f'{name}={type_name}'])
    for name, type_name in contracts['actions'].items():
        command.extend(['--action', f'{name}={type_name}'])
    for name, node_names in contracts['action_servers'].items():
        for node_name in node_names:
            command.extend(['--action-server', f'{name}={node_name}'])
    for name, node_names in contracts['action_clients'].items():
        for node_name in node_names:
            command.extend(['--action-client', f'{name}={node_name}'])
    command.extend(
        [
            '--output',
            str(run_root / f'{prefix}graph.json'),
            '--nodes-output',
            str(run_root / f'{prefix}nodes.txt'),
            '--topics-output',
            str(run_root / f'{prefix}topics.txt'),
            '--services-output',
            str(run_root / f'{prefix}services.txt'),
            '--actions-output',
            str(run_root / f'{prefix}actions.txt'),
            '--watch-pid',
            str(watch_pid),
            '--wall-timeout',
            f'{graph_wall_timeout_s:g}',
        ]
    )
    return command


def _phase3_full_stack_command(run_root: Path) -> list[str]:
    """Rebuild the exact production full-stack launch argv."""
    return [
        'ros2',
        'launch',
        'robotest_navigation',
        'phase2.launch.py',
        'namespace:=robotest',
        'seed:=42',
        'headless:=true',
        'render_sensors:=true',
        'rviz:=false',
        'autostart:=true',
        'navigation_start_delay_sec:=7.0',
        'lifecycle_discovery_grace_sec:=4.0',
        'lifecycle_service_timeout_sec:=20.0',
        'lifecycle_response_timeout_sec:=60.0',
        f'lifecycle_startup_result_path:={run_root / "lifecycle-startup-result.json"}',
    ]


def _phase3_mission_runner_command(
    repository: Path,
    run_root: Path,
    plan: Mapping[str, Any],
) -> list[str]:
    """Rebuild the exact production mission-runner argv from the frozen plan."""
    return [
        'ros2',
        'run',
        'robotest_missions',
        'mission_runner',
        '--mission',
        str(repository / str(plan['scenario_path'])),
        '--json',
        str(run_root / 'mission-result.json'),
        '--csv',
        str(run_root / 'mission-result.csv'),
        '--run-id',
        str(plan['run_id']),
        '--candidate-id',
        str(plan['candidate_id']),
        '--repetition-index',
        str(plan['repetition_index']),
        '--suite-index',
        str(plan['suite_index']),
        '--ros-args',
        '-r',
        '__ns:=/robotest',
    ]


def _phase3_runtime_gate_command(
    repository: Path,
    output: Path,
    *,
    mode: str,
    wall_timeout_s: float,
    watch_pid: int | None = None,
    launch_pid: int | None = None,
    expected_plan: Mapping[str, Any] | None = None,
) -> list[str]:
    """Rebuild one production runtime-gate argv."""
    command = [
        'python3',
        str(repository / 'tests/phase3_runtime_gate.py'),
        '--mode',
        mode,
        '--output',
        str(output),
        '--workspace',
        str(repository),
        '--wall-timeout-s',
        str(wall_timeout_s),
    ]
    if watch_pid is not None:
        command.extend(['--watch-pid', str(watch_pid)])
    if launch_pid is not None:
        command.extend(['--launch-pid', str(launch_pid)])
    if expected_plan is not None:
        command.extend(
            [
                '--expected-domain-id',
                str(expected_plan['ros_domain_id']),
                '--expected-gz-partition',
                str(expected_plan['gz_partition']),
            ]
        )
    return command


def _phase3_process_contracts(
    repository: Path,
    run_root: Path,
    *,
    expected_plan: Mapping[str, Any],
    orchestration: Any,
    process_records: Mapping[str, Mapping[str, Any]],
) -> dict[str, tuple[list[str], float, tuple[int, ...]]]:
    """Rebuild every successful runner process contract from frozen inputs."""
    launch_pid = process_records['full_stack']['pid']
    mission_pid = process_records['mission_runner']['pid']
    lifecycle_arguments: list[str] = []
    for node_name in orchestration.REQUIRED_LIFECYCLE_NODES:
        lifecycle_arguments.extend(['--node', node_name])
    scenario_command = [
        'ros2',
        'run',
        'robotest_scenarios',
        'scenario_controller',
        '--scenario',
        str(repository / str(expected_plan['scenario_path'])),
        '--output',
        str(run_root / 'scenario-result.json'),
        '--ready-file',
        str(run_root / 'scenario.ready.json'),
        '--run-id',
        str(expected_plan['run_id']),
        '--candidate-id',
        str(expected_plan['candidate_id']),
        '--repetition-index',
        str(expected_plan['repetition_index']),
        '--suite-index',
        str(expected_plan['suite_index']),
        '--ros-args',
        '-r',
        '__ns:=/robotest',
    ]
    observer_command = [
        'python3',
        str(repository / 'tests/phase3_runtime_observer.py'),
        'observe-goal',
        '--run-id',
        str(expected_plan['run_id']),
        '--ready-file',
        str(run_root / 'goal-observer.ready.json'),
        '--arm-file',
        str(run_root / 'goal-observer.arm'),
        '--armed-file',
        str(run_root / 'goal-observer.armed.json'),
        '--output',
        str(run_root / 'goal-observer.json'),
        '--watch-pid',
        str(launch_pid),
        '--wall-timeout-s',
        '300',
        '--ros-args',
        '-r',
        '__ns:=/robotest',
    ]
    if expected_plan.get('scenario_id') == 4:
        observer_command.extend(['--schedule-output', str(run_root / 'lifecycle-schedule.json')])
    mission_document = _load_canonical_json(
        run_root / 'mission-result.json',
        'Phase 3 mission result process binding',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    measurements = _mapping(
        mission_document.get('measurements'),
        'Phase 3 mission measurements process binding',
    )
    terminal_stamp = measurements.get('terminal_action_stamp_ns')
    _require(
        _exact_integer(terminal_stamp) and terminal_stamp > 0,
        'Phase 3 mission terminal stamp cannot bind the contact-drain process',
    )
    contracts: dict[str, tuple[list[str], float, tuple[int, ...]]] = {
        'contact_drain': (
            [
                'python3',
                str(repository / 'tests/phase3_runtime_observer.py'),
                'wait-contact-drain',
                '--terminal-action-stamp-ns',
                str(terminal_stamp),
                '--output',
                str(run_root / 'contact-drain.json'),
                '--watch-pid',
                str(launch_pid),
                '--wall-timeout-s',
                '30',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ],
            35.0,
            (0,),
        ),
        'contact_stream_final_gate': (
            _phase3_runtime_gate_command(
                repository,
                run_root / 'contact-stream-final-gate.json',
                mode='contact-stream',
                wall_timeout_s=10.0,
                watch_pid=launch_pid,
                launch_pid=launch_pid,
                expected_plan=expected_plan,
            ),
            15.0,
            (0,),
        ),
        'domain_cleanup': (
            _phase3_runtime_gate_command(
                repository,
                run_root / 'domain-cleanup.json',
                mode='empty',
                wall_timeout_s=15.0,
            ),
            20.0,
            (0,),
        ),
        'domain_preflight': (
            _phase3_runtime_gate_command(
                repository,
                run_root / 'domain-preflight.json',
                mode='empty',
                wall_timeout_s=10.0,
            ),
            15.0,
            (0,),
        ),
        'full_stack': (_phase3_full_stack_command(run_root), 430.0, (-15, 0)),
        'goal_observer': (observer_command, 310.0, (0,)),
        'graph_gate': (
            _phase3_graph_probe_command(
                repository,
                run_root,
                orchestration=orchestration,
                mission_client=False,
                watch_pid=launch_pid,
            ),
            100.0,
            (0,),
        ),
        'lifecycle_gate': (
            [
                'python3',
                str(repository / 'tests/phase2_lifecycle_probe.py'),
                '--namespace',
                '/robotest',
                '--output',
                str(run_root / 'lifecycle-ready.json'),
                '--text-dir',
                str(run_root),
                '--text-prefix',
                'lifecycle-ready-',
                '--wall-timeout',
                '110',
                '--watch-pid',
                str(launch_pid),
                *lifecycle_arguments,
            ],
            120.0,
            (0,),
        ),
        'metrics_collector': (
            [
                'ros2',
                'run',
                'robotest_metrics',
                'metrics_collector',
                '--output',
                str(run_root / 'capture.json'),
                '--ready-file',
                str(run_root / 'metrics.ready.json'),
                '--stop-file',
                str(run_root / 'metrics.stop'),
                '--contact-progress-file',
                str(run_root / 'contact-progress.json'),
                '--wall-timeout-s',
                '360',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ],
            370.0,
            (0,),
        ),
        'mission_graph_gate': (
            _phase3_graph_probe_command(
                repository,
                run_root,
                orchestration=orchestration,
                mission_client=True,
                watch_pid=mission_pid,
            ),
            30.0,
            (0,),
        ),
        'mission_runner': (
            _phase3_mission_runner_command(repository, run_root, expected_plan),
            310.0,
            (0,),
        ),
        'partition_cleanup': (['gz', 'topic', '-l'], 15.0, (0,)),
        'partition_preflight': (['gz', 'topic', '-l'], 15.0, (0,)),
        'runtime_gate': (
            _phase3_runtime_gate_command(
                repository,
                run_root / 'runtime-gate.json',
                mode='candidate',
                wall_timeout_s=60.0,
                watch_pid=launch_pid,
                launch_pid=launch_pid,
                expected_plan=expected_plan,
            ),
            70.0,
            (0,),
        ),
        'scenario_controller': (scenario_command, 310.0, (0,)),
        'startup_gate': (
            [
                'python3',
                str(repository / 'tests/phase2_startup_gate.py'),
                '--result',
                str(run_root / 'lifecycle-startup-result.json'),
                '--output',
                str(run_root / 'startup-gate.json'),
                '--watch-pid',
                str(launch_pid),
                '--wall-timeout',
                '110',
                '--expected-service-name',
                '/robotest/lifecycle_manager_navigation/manage_nodes',
                '--expected-discovery-grace-sec',
                '4.0',
                '--expected-service-timeout-sec',
                '20.0',
                '--expected-response-timeout-sec',
                '60.0',
            ],
            120.0,
            (0,),
        ),
    }
    if expected_plan.get('scenario_id') == 4:
        contracts['lifecycle_sampler'] = (
            [
                'ros2',
                'run',
                'robotest_metrics',
                'metrics_lifecycle_sampler',
                '--schedule',
                str(run_root / 'lifecycle-schedule.json'),
                '--output',
                str(run_root / 'lifecycle-snapshot.json'),
                '--ready-file',
                str(run_root / 'lifecycle-sampler.ready.json'),
                '--stop-file',
                str(run_root / 'lifecycle-sampler.stop'),
                '--wall-timeout-s',
                '360',
                '--request-timeout-s',
                '2',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ],
            370.0,
            (0,),
        )
    return contracts


def _phase3_current_prerequisite_paths(run_root: Path) -> set[str]:
    """Recover the production pre-manifest snapshot from the finalized run tree."""
    paths: set[str] = set()
    for path in run_root.rglob('*'):
        if not path.is_file():
            continue
        relative = path.relative_to(run_root).as_posix()
        if (
            relative in PHASE3_PREREQUISITE_POST_PATHS
            or relative.startswith('analysis-process/')
            or relative.startswith('result/')
        ):
            continue
        paths.add(relative)
    return paths


def _phase3_runtime_gate_endpoint(raw_endpoint: object, label: str) -> dict[str, Any]:
    """Validate the exact JSON projection emitted by ``_endpoint_record``."""
    endpoint = dict(_mapping(raw_endpoint, label))
    _require(
        set(endpoint) == PHASE3_RUNTIME_GATE_ENDPOINT_KEYS
        and _exact_integer(endpoint.get('depth'))
        and 0 <= endpoint['depth'] <= (2**63 - 1)
        and endpoint.get('history') in {'KEEP_ALL', 'KEEP_LAST', 'SYSTEM_DEFAULT', 'UNKNOWN'}
        and isinstance(endpoint.get('reliability'), str)
        and endpoint['reliability']
        and isinstance(endpoint.get('durability'), str)
        and endpoint['durability']
        and isinstance(endpoint.get('gid'), str)
        and PHASE3_ENDPOINT_GID_PATTERN.fullmatch(endpoint['gid']) is not None
        and isinstance(endpoint.get('node'), str)
        and endpoint['node'].startswith('/')
        and isinstance(endpoint.get('topic_type'), str)
        and bool(endpoint['topic_type']),
        f'{label} is not an exact runtime-gate endpoint record',
    )
    return endpoint


def _phase3_runtime_gate_expected_contract(runtime_gate: Any, topic: str) -> dict[str, Any]:
    reliability, durability, depth = runtime_gate.QOS_CONTRACTS[topic]
    return {
        'depth': depth,
        'durability': durability,
        'endpoint_depth_overrides': [
            {'depth': override_depth, 'node': node, 'side': side}
            for (override_topic, side, node), override_depth in sorted(
                runtime_gate.QOS_DEPTH_OVERRIDES.items()
            )
            if override_topic == topic
        ],
        'history': 'KEEP_LAST',
        'reliability': reliability,
    }


def _phase3_validate_runtime_gate_topics(
    raw_topics: object,
    *,
    runtime_gate: Any,
    expected_topics: object,
) -> dict[str, Any]:
    """Replay the producer's exact endpoint/QoS projection from frozen source constants."""
    topics = _mapping(raw_topics, 'Phase 3 runtime-gate topics')
    expected_topic_names = set(expected_topics)
    _require(
        expected_topic_names and set(topics) == expected_topic_names,
        'Phase 3 runtime-gate topic set differs from the producer contract',
    )
    replayed_topics: dict[str, dict[str, Any]] = {}
    exact_contract: dict[str, dict[str, Any]] = {}
    for topic_name in sorted(expected_topic_names):
        _require(
            isinstance(topic_name, str) and topic_name.startswith('/'),
            'Phase 3 runtime-gate topic name is invalid',
        )
        evidence = _mapping(topics[topic_name], f'Phase 3 runtime-gate topic {topic_name}')
        _require(
            set(evidence) == PHASE3_RUNTIME_GATE_TOPIC_KEYS,
            f'Phase 3 runtime-gate topic {topic_name} shape changed',
        )
        endpoints: dict[str, list[dict[str, Any]]] = {}
        for side, plural in (('publisher', 'publishers'), ('subscriber', 'subscribers')):
            raw_endpoints = _list(
                evidence.get(plural),
                f'Phase 3 runtime-gate topic {topic_name} {plural}',
            )
            validated = [
                _phase3_runtime_gate_endpoint(
                    raw_endpoint,
                    f'Phase 3 runtime-gate topic {topic_name} {side} endpoint {index}',
                )
                for index, raw_endpoint in enumerate(raw_endpoints)
            ]
            _require(
                validated == sorted(validated, key=lambda item: (item['node'], item['topic_type'])),
                f'Phase 3 runtime-gate topic {topic_name} {plural} are not canonical',
            )
            endpoints[plural] = validated
        _require(
            len(endpoints['publishers']) + len(endpoints['subscribers'])
            <= runtime_gate.MAX_ENDPOINTS,
            f'Phase 3 runtime-gate topic {topic_name} endpoint graph exceeds the bound',
        )
        checks = []
        for side, plural in (('publisher', 'publishers'), ('subscriber', 'subscribers')):
            for endpoint in endpoints[plural]:
                endpoint_contract = runtime_gate._endpoint_qos_contract(
                    topic_name,
                    side,
                    endpoint['node'],
                )
                checks.append(
                    {
                        **runtime_gate._qos_status(endpoint, endpoint_contract),
                        'node': endpoint['node'],
                        'side': side,
                    }
                )
        checks.sort(key=lambda item: (item['side'], item['node']))
        expected_contract = _phase3_runtime_gate_expected_contract(runtime_gate, topic_name)
        replayed = {
            'bounded_depth_live_proven': bool(checks)
            and all(item['bounded_depth_live_proven'] for item in checks),
            'exact_depth_live_proven': bool(checks)
            and all(item['exact_depth_live_proven'] for item in checks),
            'expected': expected_contract,
            'publisher_qos_pass': bool(endpoints['publishers'])
            and all(item['policy_contract_pass'] for item in checks if item['side'] == 'publisher'),
            'publishers': endpoints['publishers'],
            'qos_checks': checks,
            'qos_introspection_complete': bool(checks)
            and all(item['introspection_complete'] for item in checks),
            'subscriber_qos_pass': all(
                item['policy_contract_pass'] for item in checks if item['side'] == 'subscriber'
            ),
            'subscribers': endpoints['subscribers'],
        }
        _require(
            _exact_json_equal(evidence, replayed),
            f'Phase 3 runtime-gate topic {topic_name} differs from endpoint/QoS replay',
        )
        replayed_topics[topic_name] = replayed
        exact_contract[topic_name] = expected_contract
    return {
        'bounded_depth_live_proven_for_all_endpoints': all(
            evidence['bounded_depth_live_proven'] for evidence in replayed_topics.values()
        ),
        'exact_static_qos_depth_contract': exact_contract,
        'qos_contract_pass': all(
            evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
            for evidence in replayed_topics.values()
        ),
        'qos_introspection_complete': all(
            evidence['qos_introspection_complete'] for evidence in replayed_topics.values()
        ),
        'topics': replayed_topics,
    }


def _phase3_runtime_gate_nodes(value: object, label: str) -> list[str]:
    nodes = _list(value, label)
    _require(
        nodes
        and nodes == sorted(set(nodes))
        and all(isinstance(node, str) and node.startswith('/') for node in nodes),
        f'{label} is not the exact canonical node snapshot',
    )
    return nodes


def _phase3_bounded_process_record(
    repository: Path,
    run_root: Path,
    *,
    role: str,
) -> Mapping[str, Any]:
    """Load one exact, finalized ProcessRegistry record and its retained logs."""
    label = f'Phase 3 {role} process'
    process = _load_canonical_json(
        run_root / f'processes/{role}.process.json',
        label,
        maximum_bytes=64 * 1024,
    )
    _require(set(process) == PHASE3_BOUNDED_PROCESS_KEYS, f'{label} fields are invalid')
    command = _list(process.get('command'), f'{label} command')
    pid = process.get('pid')
    started = process.get('started_steady_ns')
    finished = process.get('finished_steady_ns')
    returncode = process.get('returncode')
    wall_timeout_s = process.get('wall_timeout_s')
    _require(
        process.get('role') == role
        and process.get('cwd') == str(repository.resolve())
        and _exact_integer(pid)
        and pid > 0
        and process.get('pgid') == pid
        and _exact_integer(started)
        and started > 0
        and _exact_integer(finished)
        and finished >= started
        and _exact_integer(returncode)
        and returncode != 124
        and process.get('timed_out') is False
        and process.get('group_confirmed_empty') is True
        and isinstance(wall_timeout_s, (int, float))
        and not isinstance(wall_timeout_s, bool)
        and math.isfinite(wall_timeout_s)
        and wall_timeout_s > 0.0
        and 1 <= len(command) <= 128
        and all(
            isinstance(item, str) and item and len(item.encode('utf-8')) <= 4096 for item in command
        )
        and process.get('wrapped_command')
        == [
            'timeout',
            '--signal=TERM',
            '--kill-after=10s',
            f'{wall_timeout_s:.3f}s',
            *command,
        ],
        f'{label} identity, lifetime, or wrapper binding is invalid',
    )
    for stream_name in ('stdout', 'stderr'):
        stream_label = f'{label} {stream_name}'
        stream = _mapping(process.get(stream_name), stream_label)
        maximum_bytes = stream.get('maximum_bytes')
        observed_bytes = stream.get('observed_bytes')
        retained_bytes = stream.get('retained_bytes')
        _require(
            set(stream) == PHASE3_BOUNDED_STREAM_KEYS
            and maximum_bytes == 8 * 1024 * 1024
            and _exact_integer(observed_bytes)
            and _exact_integer(retained_bytes)
            and 0 <= retained_bytes <= maximum_bytes
            and observed_bytes == retained_bytes
            and stream.get('overflow') is False
            and stream.get('error') is None,
            f'{stream_label} fields are invalid',
        )
        log_path = _regular_file(
            run_root / f'processes/{role}.{stream_name}.log',
            f'{stream_label} retained log',
        )
        _require(
            log_path.stat().st_size == retained_bytes,
            f'{stream_label} retained byte count differs from its log',
        )
    return process


def _validate_phase3_final_launch_log_gate(
    repository: Path,
    run_root: Path,
    *,
    expected_lifecycle_watch_pid: int,
    expected_lifecycle_nodes: tuple[str, ...],
) -> Mapping[str, Any]:
    """Rebuild and replay the manifest-bound final full-stack launch-log gate."""
    source = _load_repository_module(
        repository,
        'tests/phase2_startup_gate.py',
        'Phase 3 final launch-log scanner source contract',
    )
    stdout_path = run_root / 'processes/full_stack.stdout.log'
    stderr_path = run_root / 'processes/full_stack.stderr.log'
    combined_path = _regular_file(
        run_root / 'full-stack-combined.log',
        'Phase 3 final combined full-stack launch log',
    )
    _require(
        0 < combined_path.stat().st_size <= 2 * 8 * 1024 * 1024 + 1,
        'Phase 3 final combined full-stack launch log exceeds its exact size bound',
    )
    try:
        rebuilt = source.combined_launch_log_bytes(
            stdout_path,
            stderr_path,
            maximum_stream_bytes=8 * 1024 * 1024,
        )
        observed_combined = combined_path.read_bytes()
    except Exception as exc:
        raise EvidenceError(
            f'Phase 3 final combined launch-log replay failed: {type(exc).__name__}: {exc}'
        ) from exc
    _require(
        observed_combined == rebuilt,
        'Phase 3 final combined launch log differs from finalized full_stack streams',
    )
    lifecycle_ready_path = _regular_file(
        run_root / 'lifecycle-ready.json',
        'Phase 3 lifecycle-ready evidence',
    )
    lifecycle_ready_bytes = lifecycle_ready_path.read_bytes()
    resolved_lifecycle_ready = lifecycle_ready_path.resolve(strict=True)

    gate_path = run_root / 'full-stack-final-log-gate.json'
    gate = _load_canonical_json(
        gate_path,
        'Phase 3 final launch-log signature gate',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    _validate_json_sidecar(gate_path, 'Phase 3 final launch-log signature gate')
    _require(
        set(gate) == PHASE3_FINAL_LAUNCH_LOG_GATE_KEYS,
        'Phase 3 final launch-log signature gate fields are invalid',
    )
    launch_stopped_utc = gate.get('launch_stopped_utc')
    scanned_utc = gate.get('scanned_utc')
    stopped_at = _utc_timestamp(
        launch_stopped_utc,
        'Phase 3 final launch-log signature gate launch_stopped_utc',
    )
    scanned_at = _utc_timestamp(
        scanned_utc,
        'Phase 3 final launch-log signature gate scanned_utc',
    )
    _require(
        scanned_at >= stopped_at,
        'Phase 3 final launch-log signature gate was scanned before launch stopped',
    )
    resolved_combined = combined_path.resolve(strict=True)
    try:
        replayed = source.scan_final_launch_log(
            resolved_combined,
            launch_stopped_utc,
            scanned_utc=scanned_utc,
            lifecycle_ready_path=resolved_lifecycle_ready,
            expected_lifecycle_watch_pid=expected_lifecycle_watch_pid,
            expected_lifecycle_wall_timeout_s=110.0,
            expected_lifecycle_nodes=expected_lifecycle_nodes,
        )
    except Exception as exc:
        raise EvidenceError(
            f'Phase 3 final launch-log scanner replay failed: {type(exc).__name__}: {exc}'
        ) from exc
    _require(
        set(replayed) == PHASE3_FINAL_LAUNCH_LOG_GATE_KEYS,
        'Phase 3 clone-local final launch-log scanner fields are invalid',
    )
    _require(
        _exact_json_equal(gate, replayed),
        'Phase 3 final launch-log signature gate differs from clone-local scanner replay',
    )
    raw_matches = _list(
        gate.get('matches'),
        'Phase 3 final launch-log raw matches',
    )
    match_count = gate.get('match_count')
    _require(
        _exact_integer(match_count) and match_count >= 0 and match_count == len(raw_matches),
        'Phase 3 final launch-log raw match count is invalid',
    )
    for index, raw_match_value in enumerate(raw_matches):
        raw_match = _mapping(
            raw_match_value,
            f'Phase 3 final launch-log raw match {index}',
        )
        signature_ids = _list(
            raw_match.get('signature_ids'),
            f'Phase 3 final launch-log raw match {index} signatures',
        )
        _require(
            set(raw_match) == {'line_number', 'signature_ids', 'text'}
            and _exact_integer(raw_match.get('line_number'))
            and raw_match['line_number'] > 0
            and signature_ids
            and all(isinstance(value, str) and value for value in signature_ids)
            and isinstance(raw_match.get('text'), str),
            f'Phase 3 final launch-log raw match {index} fields are invalid',
        )
    recovery = _mapping(
        gate.get('lifecycle_timeout_recovery'),
        'Phase 3 lifecycle timeout recovery binding',
    )
    recovered_lines = _list(
        recovery.get('recovered_lines'),
        'Phase 3 recovered lifecycle timeout lines',
    )
    recovered_line_count = recovery.get('recovered_line_count')
    _require(
        set(recovery) == PHASE3_LIFECYCLE_TIMEOUT_RECOVERY_KEYS
        and recovery.get('lifecycle_ready_path') == str(resolved_lifecycle_ready)
        and recovery.get('lifecycle_ready_sha256')
        == hashlib.sha256(lifecycle_ready_bytes).hexdigest()
        and recovery.get('lifecycle_ready_size_bytes') == len(lifecycle_ready_bytes)
        and len(lifecycle_ready_bytes) > 0
        and _exact_integer(recovered_line_count)
        and recovered_line_count >= 0
        and recovered_line_count == len(recovered_lines),
        'Phase 3 lifecycle timeout recovery binding is invalid',
    )
    recovered_pairs: list[tuple[int, str]] = []
    for index, recovered_value in enumerate(recovered_lines):
        recovered = _mapping(
            recovered_value,
            f'Phase 3 recovered lifecycle timeout line {index}',
        )
        line_number = recovered.get('line_number')
        service_name = recovered.get('service_name')
        timed_out_attempts = recovered.get('timed_out_attempts')
        _require(
            set(recovered) == PHASE3_RECOVERED_LIFECYCLE_TIMEOUT_KEYS
            and _exact_integer(line_number)
            and line_number > 0
            and isinstance(service_name, str)
            and re.fullmatch(r'/robotest/[A-Za-z0-9_]+/get_state', service_name) is not None
            and isinstance(recovered.get('text'), str)
            and _exact_integer(timed_out_attempts)
            and timed_out_attempts > 0,
            f'Phase 3 recovered lifecycle timeout line {index} fields are invalid',
        )
        recovered_pairs.append((line_number, recovered['text']))
    raw_pairs = [(value['line_number'], value['text']) for value in raw_matches]
    _require(
        recovered_pairs == raw_pairs
        and len({line_number for line_number, _ in recovered_pairs}) == len(recovered_pairs)
        and (
            not raw_matches
            or all(value['signature_ids'] == ['dds_response_timeout'] for value in raw_matches)
        ),
        'Phase 3 recovered lifecycle timeout lines do not exactly audit raw matches',
    )
    _require(
        gate.get('schema_version') == 2
        and gate.get('verdict') == 'PASS'
        and gate.get('failure_kind') is None
        and gate.get('failure_message') is None
        and gate.get('launch_log_path') == str(resolved_combined)
        and gate.get('launch_log_sha256') == hashlib.sha256(rebuilt).hexdigest()
        and gate.get('launch_log_size_bytes') == len(rebuilt)
        and gate.get('line_count') == len(rebuilt.decode('utf-8').splitlines())
        and match_count == recovered_line_count
        and gate.get('signature_definitions') == replayed['signature_definitions'],
        'Phase 3 final launch-log signature gate is not an exact recovered-or-zero-match PASS',
    )
    return gate


def _phase3_graph_process(
    repository: Path,
    run_root: Path,
    *,
    orchestration: Any,
    role: str,
    mission_client: bool,
    expected_watch_pid: int,
) -> Mapping[str, Any]:
    """Validate one graph gate and tie its argv to an independently owned PID."""
    process = _phase3_bounded_process_record(repository, run_root, role=role)
    label = f'Phase 3 {role} process'
    command = process['command']
    watch_indexes = [index for index, value in enumerate(command) if value == '--watch-pid']
    _require(
        len(watch_indexes) == 1 and watch_indexes[0] + 1 < len(command),
        f'{label} must contain exactly one watched PID',
    )
    watch_pid_text = command[watch_indexes[0] + 1]
    _require(
        isinstance(watch_pid_text, str)
        and re.fullmatch(r'[1-9][0-9]*', watch_pid_text) is not None
        and int(watch_pid_text) == expected_watch_pid,
        f'{label} watched PID does not match its owned component process',
    )
    expected_timeout = PHASE3_GRAPH_PROCESS_TIMEOUTS[role]
    expected_command = _phase3_graph_probe_command(
        repository,
        run_root,
        orchestration=orchestration,
        mission_client=mission_client,
        watch_pid=expected_watch_pid,
    )
    _require(
        process.get('pid') != expected_watch_pid
        and process.get('returncode') == 0
        and process.get('wall_timeout_s') == expected_timeout
        and command == expected_command,
        f'{label} command binding is invalid',
    )
    return process


def _validate_phase3_graph_prerequisites(
    repository: Path,
    run_root: Path,
    *,
    expected_plan: Mapping[str, Any],
    orchestrator_document: Mapping[str, Any],
    orchestration: Any,
) -> dict[str, dict[str, Any]]:
    """Replay the manifest-bound pre-mission and mission graph evidence pair."""
    manifest_path = run_root / 'prerequisite-manifest.json'
    manifest = _load_canonical_json(
        manifest_path,
        'Phase 3 prerequisite manifest',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    _validate_json_sidecar(manifest_path, 'Phase 3 prerequisite manifest')
    _require(
        set(manifest) == {'aggregate_sha256', 'artifact_count', 'artifacts', 'total_bytes'},
        'Phase 3 prerequisite manifest fields are invalid',
    )
    try:
        orchestration.verify_component_manifest(manifest, run_root)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 prerequisite manifest verification failed: {exc}') from exc
    raw_records = _list(manifest.get('artifacts'), 'Phase 3 prerequisite artifact records')
    records: dict[str, Mapping[str, Any]] = {}
    ordered_paths: list[str] = []
    for index, value in enumerate(raw_records):
        record = _mapping(value, f'Phase 3 prerequisite record {index}')
        _require(
            set(record) == {'bytes', 'path', 'sha256'},
            f'Phase 3 prerequisite record {index} fields are invalid',
        )
        path = record.get('path')
        _require(isinstance(path, str) and path not in records, 'duplicate prerequisite path')
        records[path] = record
        ordered_paths.append(path)
    _require(
        ordered_paths == sorted(ordered_paths),
        'Phase 3 prerequisite manifest records are not in canonical path order',
    )
    _require(
        set(records) == _phase3_current_prerequisite_paths(run_root),
        'Phase 3 prerequisite manifest is not the exact production pre-manifest snapshot',
    )

    required_paths = set(PHASE3_PREREQUISITE_RAW_PATHS)
    required_paths.update(
        {
            f'{prefix}{relative}'
            for prefix in ('', 'mission-')
            for relative in PHASE3_GRAPH_BINDING_PATHS.values()
        }
    )
    expected_process_roles = set(PHASE3_PREREQUISITE_PROCESS_ROLES)
    if expected_plan.get('scenario_id') == 4:
        expected_process_roles.add('lifecycle_sampler')
        required_paths.update(
            {
                'lifecycle-sampler.ready.json',
                'lifecycle-sampler.stop',
                'lifecycle-schedule.json',
                'lifecycle-schedule.json.sha256',
                'lifecycle-snapshot.json',
                'lifecycle-snapshot.json.sha256',
            }
        )
    expected_process_paths = {
        f'processes/{role}.{suffix}'
        for role in expected_process_roles
        for suffix in ('process.json', 'stderr.log', 'stdout.log')
    }
    observed_process_paths = {path for path in records if path.startswith('processes/')}
    _require(
        observed_process_paths == expected_process_paths,
        'Phase 3 prerequisite process triplet set is not the production role set',
    )
    for role in expected_process_roles:
        required_paths.update(
            {
                f'processes/{role}.process.json',
                f'processes/{role}.stderr.log',
                f'processes/{role}.stdout.log',
            }
        )
    _require(
        set(records) == required_paths,
        'Phase 3 prerequisite manifest path set is not the exact successful-run snapshot',
    )

    process_records = {
        role: _phase3_bounded_process_record(repository, run_root, role=role)
        for role in expected_process_roles
    }
    process_contracts = _phase3_process_contracts(
        repository,
        run_root,
        expected_plan=expected_plan,
        orchestration=orchestration,
        process_records=process_records,
    )
    _require(
        set(process_contracts) == expected_process_roles,
        'Phase 3 process contract role set is inconsistent',
    )
    for role, process in process_records.items():
        expected_command, expected_timeout, allowed_returncodes = process_contracts[role]
        _require(
            process.get('command') == expected_command
            and process.get('wall_timeout_s') == expected_timeout
            and process.get('returncode') in allowed_returncodes,
            f'Phase 3 {role} process command, timeout, or outcome binding is invalid',
        )
    _require(
        len({process['pid'] for process in process_records.values()}) == len(process_records),
        'Phase 3 process PIDs are not all distinct',
    )
    for role in ('partition_preflight', 'partition_cleanup'):
        stdout_path = run_root / f'processes/{role}.stdout.log'
        try:
            stdout_empty = not stdout_path.read_text(encoding='utf-8').strip()
        except (OSError, UnicodeError) as exc:
            raise EvidenceError(f'cannot read Phase 3 {role} stdout: {exc}') from exc
        _require(stdout_empty, f'Phase 3 {role} did not prove an unused Gazebo partition')

    full_stack_process = process_records['full_stack']
    mission_process = process_records['mission_runner']
    _validate_phase3_final_launch_log_gate(
        repository,
        run_root,
        expected_lifecycle_watch_pid=full_stack_process['pid'],
        expected_lifecycle_nodes=tuple(orchestration.REQUIRED_LIFECYCLE_NODES),
    )
    expected_mission_command = process_contracts['mission_runner'][0]
    _require(
        full_stack_process.get('returncode') in (-15, 0)
        and mission_process.get('returncode') == 0
        and full_stack_process.get('pid') != mission_process.get('pid'),
        'Phase 3 watched component process outcomes are invalid',
    )
    _phase3_graph_process(
        repository,
        run_root,
        orchestration=orchestration,
        role='graph_gate',
        mission_client=False,
        expected_watch_pid=full_stack_process['pid'],
    )
    _phase3_graph_process(
        repository,
        run_root,
        orchestration=orchestration,
        role='mission_graph_gate',
        mission_client=True,
        expected_watch_pid=mission_process['pid'],
    )
    process_started = {
        role: process['started_steady_ns'] for role, process in process_records.items()
    }
    process_finished = {
        role: process['finished_steady_ns'] for role, process in process_records.items()
    }
    online_roles = expected_process_roles - {
        'domain_cleanup',
        'domain_preflight',
        'full_stack',
        'partition_cleanup',
        'partition_preflight',
    }
    timeline_valid = (
        process_started['domain_preflight']
        <= process_finished['domain_preflight']
        <= process_started['partition_preflight']
        <= process_finished['partition_preflight']
        <= process_started['full_stack']
        <= process_started['startup_gate']
        <= process_finished['startup_gate']
        <= process_started['lifecycle_gate']
        <= process_finished['lifecycle_gate']
        <= process_started['metrics_collector']
        <= process_started['scenario_controller']
        <= process_started['goal_observer']
        <= process_started['runtime_gate']
        <= process_finished['runtime_gate']
        <= process_started['graph_gate']
        <= process_finished['graph_gate']
        <= process_started['mission_runner']
        <= process_finished['goal_observer']
        <= process_started['mission_graph_gate']
        <= process_finished['mission_graph_gate']
        <= process_finished['mission_runner']
        <= process_started['contact_drain']
        and process_finished['scenario_controller'] <= process_started['contact_drain']
        and process_finished['contact_drain']
        <= process_started['contact_stream_final_gate']
        <= process_finished['contact_stream_final_gate']
        <= process_finished['metrics_collector']
        and all(
            process_started['full_stack'] <= process_started[role]
            and process_finished[role] <= process_finished['full_stack']
            for role in online_roles
        )
        and process_finished['full_stack']
        <= process_started['domain_cleanup']
        <= process_finished['domain_cleanup']
        <= process_started['partition_cleanup']
        <= process_finished['partition_cleanup']
    )
    if expected_plan.get('scenario_id') == 4:
        timeline_valid = (
            timeline_valid
            and process_finished['goal_observer']
            <= process_started['lifecycle_sampler']
            <= process_started['mission_graph_gate']
            and process_finished['metrics_collector'] <= process_finished['lifecycle_sampler']
        )
    _require(timeline_valid, 'Phase 3 successful process timeline is invalid')
    execution = _mapping(
        orchestrator_document.get('execution'),
        'Phase 3 orchestrator execution evidence',
    )
    expected_execution = {
        'command': shlex.join(expected_mission_command),
        'exit_code': mission_process['returncode'],
        'wall_duration_s': (
            mission_process['finished_steady_ns'] - mission_process['started_steady_ns']
        )
        / 1_000_000_000,
        'wall_timed_out': False,
        'wall_timeout_s': 300.0,
        'working_directory': str(repository.resolve()),
    }
    _require(
        execution == expected_execution,
        'Phase 3 orchestrator execution does not exactly bind the mission process',
    )
    for gate_name, expected_timeout in (
        ('domain-preflight.json', 10.0),
        ('domain-cleanup.json', 15.0),
    ):
        gate_path = run_root / gate_name
        gate = _load_canonical_json(
            gate_path,
            f'Phase 3 {gate_name} empty-domain evidence',
        )
        _validate_json_sidecar(gate_path, f'Phase 3 {gate_name} empty-domain evidence')
        attempt_count = gate.get('attempt_count')
        elapsed_wall_s = gate.get('elapsed_wall_s')
        _require(
            set(gate)
            == {
                'attempt_count',
                'elapsed_wall_s',
                'mode',
                'nodes',
                'producer',
                'remaining_nodes',
                'schema_version',
                'verdict',
            }
            and _exact_integer(attempt_count)
            and 1 <= attempt_count <= 4096
            and isinstance(elapsed_wall_s, (int, float))
            and not isinstance(elapsed_wall_s, bool)
            and math.isfinite(elapsed_wall_s)
            and 0.0 <= elapsed_wall_s <= expected_timeout
            and gate.get('mode') == 'empty'
            and gate.get('nodes') == ['/robotest/evidence/phase3_runtime_gate']
            and gate.get('producer') == 'robotest_phase3/runtime_gate'
            and gate.get('remaining_nodes') == []
            and gate.get('schema_version') == 1
            and gate.get('verdict') == 'PASS',
            f'Phase 3 {gate_name} is not an exact empty-domain PASS',
        )
    resource_summary = orchestration.summarize_resources(run_root / 'resources.jsonl')
    expected_orchestrator_resources = {
        field: resource_summary[field] for field in orchestration.RESOURCE_METRIC_FIELDS
    }
    observed_orchestrator_resources = _mapping(
        orchestrator_document.get('resources'),
        'Phase 3 orchestrator resource evidence',
    )
    _require(
        observed_orchestrator_resources == expected_orchestrator_resources,
        'Phase 3 orchestrator resources do not exactly match the raw resource trace',
    )
    runtime_gate_source = _load_repository_module(
        repository,
        'tests/phase3_runtime_gate.py',
        'Phase 3 runtime-gate source contract',
    )
    runtime_gates: dict[str, Mapping[str, Any]] = {}
    for gate_name in ('runtime-gate.json', 'contact-stream-final-gate.json'):
        runtime_gate = _load_canonical_json(
            run_root / gate_name,
            f'Phase 3 {gate_name} launch binding',
            maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
        )
        runtime_gates[gate_name] = runtime_gate
        for attestation_name in (
            'contact_aggregator_binary_attestation',
            'contact_gate_binary_attestation',
        ):
            attestation = _mapping(
                runtime_gate.get(attestation_name),
                f'Phase 3 {gate_name} {attestation_name}',
            )
            _require(
                attestation.get('launch_root_pid') == full_stack_process['pid'],
                f'Phase 3 {gate_name} does not independently bind the full-stack launch PID',
            )
    initial_gate = runtime_gates['runtime-gate.json']
    initial_attempts = initial_gate.get('attempt_count')
    initial_elapsed = initial_gate.get('elapsed_wall_s')
    _require(
        set(initial_gate) == PHASE3_CANDIDATE_RUNTIME_GATE_KEYS
        and _exact_integer(initial_attempts)
        and 1 <= initial_attempts <= 4096
        and isinstance(initial_elapsed, (int, float))
        and not isinstance(initial_elapsed, bool)
        and math.isfinite(initial_elapsed)
        and 0.0 <= initial_elapsed <= 60.0
        and initial_gate.get('mode') == 'candidate'
        and initial_gate.get('producer') == 'robotest_phase3/runtime_gate'
        and initial_gate.get('schema_version') == 1
        and initial_gate.get('verdict') == 'PASS'
        and type(initial_gate.get('bounded_depth_live_proven_for_all_endpoints')) is bool
        and initial_gate.get('cmd_vel_owner_pass') is True
        and initial_gate.get('fused_clock_subscriber_ownership_pass') is True
        and initial_gate.get('legacy_fault_service_absent') is True
        and initial_gate.get('namespace_isolation_pass') is True
        and initial_gate.get('qos_contract_pass') is True
        and type(initial_gate.get('qos_introspection_complete')) is bool
        and initial_gate.get('validation_autonomy_isolation_pass') is True
        and initial_gate.get('autonomy_validation_leaks') == []
        and initial_gate.get('required_nodes_missing') == []
        and initial_gate.get('required_nodes_outside_namespace') == []
        and initial_gate.get('scenario_services_missing') == [],
        'Phase 3 candidate runtime gate is not a complete claimed PASS',
    )
    initial_nodes = _phase3_runtime_gate_nodes(
        initial_gate.get('nodes'),
        'Phase 3 candidate runtime-gate nodes',
    )
    initial_topic_replay = _phase3_validate_runtime_gate_topics(
        initial_gate.get('topics'),
        runtime_gate=runtime_gate_source,
        expected_topics=runtime_gate_source.QOS_CONTRACTS,
    )
    initial_topics = initial_topic_replay['topics']
    expected_fused_clock_subscriber_ownership = (
        runtime_gate_source._fused_clock_subscriber_ownership(initial_topics['/clock'])
    )
    _require(
        expected_fused_clock_subscriber_ownership
        and initial_gate.get('fused_clock_subscriber_ownership_pass')
        is expected_fused_clock_subscriber_ownership,
        'Phase 3 candidate fused /clock subscriber ownership failed',
    )
    authoritative_publisher_ownership = runtime_gate_source._authoritative_publisher_ownership(
        initial_topics
    )
    _require(
        set(authoritative_publisher_ownership)
        == set(runtime_gate_source.AUTHORITATIVE_PUBLISHER_CONTRACTS)
        and all(authoritative_publisher_ownership.values()),
        'Phase 3 candidate runtime-gate authoritative publisher ownership failed',
    )
    expected_publisher_ownership = {
        topic: (
            authoritative_publisher_ownership[topic]
            if topic in authoritative_publisher_ownership
            else runtime_gate_source._exact_endpoint_owners(
                initial_topics[topic]['publishers'],
                expected_nodes,
                expected_type=(
                    runtime_gate_source.CONTACT_MESSAGE_TYPE
                    if topic in runtime_gate_source.CONTACT_TOPICS
                    else None
                ),
            )
        )
        for topic, expected_nodes in runtime_gate_source.CANDIDATE_EXPECTED_PUBLISHERS.items()
    }
    expected_command_subscriber_ownership = {
        topic: runtime_gate_source._exact_endpoint_owners(
            initial_topics[topic]['subscribers'],
            expected_nodes,
        )
        for topic, expected_nodes in (
            runtime_gate_source.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS.items()
        )
    }
    expected_contact_subscriber_ownership = {
        topic: runtime_gate_source._exact_endpoint_owners(
            initial_topics[topic]['subscribers'],
            expected_nodes,
            expected_type=runtime_gate_source.CONTACT_MESSAGE_TYPE,
        )
        for topic, expected_nodes in (
            runtime_gate_source.CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS.items()
        )
    }
    for field, expected_ownership in (
        ('publisher_ownership', expected_publisher_ownership),
        ('command_subscriber_ownership', expected_command_subscriber_ownership),
        ('contact_subscriber_ownership', expected_contact_subscriber_ownership),
    ):
        observed_ownership = _mapping(
            initial_gate.get(field),
            f'Phase 3 candidate runtime-gate {field}',
        )
        _require(
            _exact_json_equal(observed_ownership, expected_ownership)
            and expected_ownership
            and all(expected_ownership.values()),
            f'Phase 3 candidate runtime-gate {field} differs from endpoint ownership replay',
        )
    short_node_names = {node.rsplit('/', 1)[-1] for node in initial_nodes}
    expected_required_nodes_missing = sorted(
        runtime_gate_source.CANDIDATE_REQUIRED_NODES - short_node_names
    )
    expected_required_nodes_outside_namespace = sorted(
        node
        for node in initial_nodes
        if node.rsplit('/', 1)[-1] in runtime_gate_source.CANDIDATE_REQUIRED_NODES
        and not node.startswith('/robotest/')
    )
    project_endpoints = [
        endpoint
        for evidence in initial_topics.values()
        for plural in ('publishers', 'subscribers')
        for endpoint in evidence[plural]
    ]
    endpoint_namespace_pass = all(
        endpoint['node'].startswith('/robotest/') for endpoint in project_endpoints
    )
    expected_namespace_pass = (
        endpoint_namespace_pass and not expected_required_nodes_outside_namespace
    )
    expected_autonomy_leaks = [
        {'node': subscriber['node'], 'topic': topic}
        for topic in runtime_gate_source.VALIDATION_TOPICS
        for subscriber in initial_topics[topic]['subscribers']
        if subscriber['node'].rsplit('/', 1)[-1] in runtime_gate_source.AUTONOMY_NODE_NAMES
    ]
    _require(
        initial_gate.get('bounded_depth_live_proven_for_all_endpoints')
        == initial_topic_replay['bounded_depth_live_proven_for_all_endpoints']
        and initial_gate.get('qos_introspection_complete')
        == initial_topic_replay['qos_introspection_complete']
        and initial_gate.get('qos_contract_pass') == initial_topic_replay['qos_contract_pass']
        and _exact_json_equal(
            initial_gate.get('exact_static_qos_depth_contract'),
            initial_topic_replay['exact_static_qos_depth_contract'],
        )
        and initial_gate.get('cmd_vel_owner_pass')
        is expected_publisher_ownership['/robotest/cmd_vel']
        and _exact_json_equal(
            initial_gate.get('autonomy_validation_leaks'),
            expected_autonomy_leaks,
        )
        and initial_gate.get('validation_autonomy_isolation_pass') is (not expected_autonomy_leaks)
        and _exact_json_equal(
            initial_gate.get('required_nodes_missing'),
            expected_required_nodes_missing,
        )
        and _exact_json_equal(
            initial_gate.get('required_nodes_outside_namespace'),
            expected_required_nodes_outside_namespace,
        )
        and initial_gate.get('namespace_isolation_pass') is expected_namespace_pass,
        'Phase 3 candidate runtime-gate claims differ from source-bound replay',
    )

    final_gate = runtime_gates['contact-stream-final-gate.json']
    final_attempts = final_gate.get('attempt_count')
    final_elapsed = final_gate.get('elapsed_wall_s')
    _require(
        set(final_gate) == PHASE3_CONTACT_STREAM_RUNTIME_GATE_KEYS
        and _exact_integer(final_attempts)
        and 1 <= final_attempts <= 4096
        and isinstance(final_elapsed, (int, float))
        and not isinstance(final_elapsed, bool)
        and math.isfinite(final_elapsed)
        and 0.0 <= final_elapsed <= 10.0
        and final_gate.get('mode') == 'contact_stream'
        and final_gate.get('producer') == 'robotest_phase3/runtime_gate'
        and final_gate.get('schema_version') == 1
        and final_gate.get('verdict') == 'PASS'
        and final_gate.get('contact_stream_gate_present') is True
        and final_gate.get('qos_contract_pass') is True
        and final_gate.get('raw_subscriber_ownership') is True,
        'Phase 3 final contact-stream runtime gate is not a complete claimed PASS',
    )
    final_nodes = _phase3_runtime_gate_nodes(
        final_gate.get('nodes'),
        'Phase 3 final runtime-gate nodes',
    )
    final_topic_replay = _phase3_validate_runtime_gate_topics(
        final_gate.get('topics'),
        runtime_gate=runtime_gate_source,
        expected_topics=runtime_gate_source.CONTACT_TOPICS,
    )
    final_topics = final_topic_replay['topics']
    expected_final_publisher_ownership = {
        topic: runtime_gate_source._exact_endpoint_owners(
            final_topics[topic]['publishers'],
            runtime_gate_source.CANDIDATE_EXPECTED_PUBLISHERS[topic],
            expected_type=runtime_gate_source.CONTACT_MESSAGE_TYPE,
        )
        for topic in runtime_gate_source.CONTACT_TOPICS
    }
    expected_raw_subscriber_ownership = runtime_gate_source._exact_endpoint_owners(
        final_topics['/robotest/internal/raw_contacts']['subscribers'],
        {'/robotest/contact_stream_gate'},
        expected_type=runtime_gate_source.CONTACT_MESSAGE_TYPE,
    )
    _require(
        _exact_json_equal(
            final_gate.get('publisher_ownership'),
            expected_final_publisher_ownership,
        )
        and all(expected_final_publisher_ownership.values())
        and final_gate.get('raw_subscriber_ownership') is expected_raw_subscriber_ownership
        and expected_raw_subscriber_ownership
        and final_gate.get('contact_stream_gate_present')
        is ('/robotest/contact_stream_gate' in final_nodes)
        and final_gate.get('qos_contract_pass') is final_topic_replay['qos_contract_pass'],
        'Phase 3 final runtime-gate claims differ from source-bound replay',
    )
    try:
        pre_binding = orchestration.validate_phase3_graph_artifacts(
            run_root,
            mission_client=False,
            expected_watch_pid=full_stack_process['pid'],
        )
        mission_binding = orchestration.validate_phase3_graph_artifacts(
            run_root,
            mission_client=True,
            expected_watch_pid=mission_process['pid'],
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 graph artifact replay failed: {exc}') from exc

    pre_graph_document = orchestration._load_phase3_graph_json(run_root / 'graph.json')
    mission_graph_document = orchestration._load_phase3_graph_json(run_root / 'mission-graph.json')
    for role, graph_document in (
        ('graph_gate', pre_graph_document),
        ('mission_graph_gate', mission_graph_document),
    ):
        elapsed_wall_seconds = graph_document.get('elapsed_wall_seconds')
        process = process_records[role]
        process_duration_seconds = (
            process['finished_steady_ns'] - process['started_steady_ns']
        ) / 1_000_000_000
        _require(
            _finite_number(elapsed_wall_seconds)
            and 0.0 <= elapsed_wall_seconds <= process_duration_seconds,
            f'Phase 3 {role} graph elapsed time exceeds its owned process duration',
        )
    pre_graph_observed = _mapping(
        pre_graph_document.get('observed'),
        'Phase 3 validated pre-mission graph observations',
    )
    pre_graph_services = _mapping(
        pre_graph_observed.get('services'),
        'Phase 3 validated pre-mission services',
    )
    pre_graph_topics = _mapping(
        pre_graph_observed.get('topics'),
        'Phase 3 validated pre-mission topics',
    )
    pre_graph_nodes = set(
        _list(
            pre_graph_observed.get('node_names'),
            'Phase 3 validated pre-mission node names',
        )
    )
    expected_scenario_services_missing = sorted(
        set(runtime_gate_source.SCENARIO_SERVICES) - set(pre_graph_services)
    )
    _require(
        _exact_json_equal(
            initial_gate.get('scenario_services_missing'),
            expected_scenario_services_missing,
        )
        and initial_gate.get('legacy_fault_service_absent')
        is ('/robotest/faults/load_schedule' not in pre_graph_services),
        'Phase 3 candidate runtime-gate service claims differ from the validated graph',
    )
    try:
        orchestration.validate_phase3_runtime_graph_node_join(initial_gate, pre_binding)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 runtime-gate/graph node replay failed: {exc}') from exc
    for gate_label, gate_topics in (
        ('candidate', initial_topics),
        ('final contact-stream', final_topics),
    ):
        for topic_name, topic_evidence in gate_topics.items():
            graph_types = _list(
                pre_graph_topics.get(topic_name),
                f'Phase 3 validated pre-mission topic {topic_name} types',
            )
            _require(
                len(graph_types) == 1,
                f'Phase 3 validated pre-mission topic {topic_name} is not an exact singleton type',
            )
            endpoints = [
                endpoint
                for plural in ('publishers', 'subscribers')
                for endpoint in topic_evidence[plural]
            ]
            _require(
                all(endpoint['node'] in pre_graph_nodes for endpoint in endpoints),
                f'Phase 3 {gate_label} runtime-gate {topic_name} endpoint nodes differ from '
                'the validated graph',
            )
            _require(
                all(endpoint['topic_type'] == graph_types[0] for endpoint in endpoints),
                f'Phase 3 {gate_label} runtime-gate {topic_name} endpoint types differ from '
                'the validated graph',
            )

    for prefix, binding in (('', pre_binding), ('mission-', mission_binding)):
        for binding_key, relative in PHASE3_GRAPH_BINDING_PATHS.items():
            manifest_record = records[f'{prefix}{relative}']
            _require(
                manifest_record.get('sha256') == binding[binding_key],
                f'Phase 3 {prefix}{relative} is not bound by the prerequisite manifest',
            )
    try:
        orchestration.validate_phase3_graph_pair(
            pre_binding,
            mission_binding,
            expected_mission_auxiliary_nodes=(
                ['/robotest/lifecycle_sampler'] if expected_plan.get('scenario_id') == 4 else []
            ),
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 graph pair replay failed: {exc}') from exc

    artifacts = _mapping(
        orchestrator_document.get('artifacts'),
        'Phase 3 orchestrator artifact evidence',
    )
    maximum_file_bytes = max(record['bytes'] for record in raw_records)
    runtime_stderr_bytes = sum(
        record['bytes'] for record in raw_records if record['path'].endswith('.stderr.log')
    )
    runtime_stdout_bytes = sum(
        record['bytes'] for record in raw_records if record['path'].endswith('.stdout.log')
    )
    _require(
        artifacts.get('pre_mission_graph_sha256') == pre_binding['graph_json_sha256']
        and artifacts.get('mission_graph_sha256') == mission_binding['graph_json_sha256']
        and artifacts.get('prerequisite_artifact_count') == len(raw_records)
        and artifacts.get('prerequisite_checksums_verified') is True
        and artifacts.get('prerequisite_manifest_sha256') == file_sha256(manifest_path)
        and artifacts.get('prerequisite_maximum_file_bytes') == maximum_file_bytes
        and artifacts.get('prerequisite_total_bytes') == manifest.get('total_bytes')
        and artifacts.get('prerequisites_finalized') is True
        and artifacts.get('prerequisites_within_caps') is True
        and artifacts.get('runtime_stderr_bytes') == runtime_stderr_bytes
        and artifacts.get('runtime_stdout_bytes') == runtime_stdout_bytes,
        'Phase 3 orchestrator artifact evidence does not match its prerequisite manifest',
    )
    return {'mission': mission_binding, 'pre_mission': pre_binding}


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
        from robotest_missions.artifacts import reconcile_artifacts as reconcile_mission_artifacts

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
    mission_csv_path = run_root / 'mission-result.csv'
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
        (mission_path, 'Phase 3 mission result', True),
        (scenario_path, 'Phase 3 scenario result', True),
        (capture_path, 'Phase 3 capture', True),
        (contact_drain_path, 'Phase 3 contact drain', True),
        (initial_contact_gate_path, 'Phase 3 initial contact gate', True),
        (final_contact_gate_path, 'Phase 3 final contact gate', True),
        (contact_gate_revalidation_path, 'Phase 3 contact gate revalidation', True),
        (orchestrator_path, 'Phase 3 orchestrator result', True),
    )
    for path, label, has_sidecar in raw_paths:
        _regular_file(path, label)
        if has_sidecar:
            _validate_json_sidecar(path, label)
    mission = _load_canonical_json(
        mission_path,
        'Phase 3 mission result',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    try:
        reconcile_mission_artifacts(mission_path, mission_csv_path)
    except Exception as exc:
        raise EvidenceError(f'Phase 3 mission JSON/CSV reconciliation failed: {exc}') from exc
    scenario_document = _load_canonical_json(
        scenario_path,
        'Phase 3 scenario result',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    capture_document = _load_canonical_json(
        capture_path,
        'Phase 3 capture',
        maximum_bytes=PHASE3_RESULT_JSON_MAX_BYTES,
    )
    orchestrator_document = _load_canonical_json(
        orchestrator_path,
        'Phase 3 orchestrator result',
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
        observer_path = run_root / 'goal-observer.json'
        observer = _load_canonical_json(observer_path, 'Phase 3 goal observer')
        _validate_json_sidecar(observer_path, 'Phase 3 goal observer')
        expected_goal_binding = orchestration.reconcile_goal_binding(
            observer,
            mission,
            scenario_document,
        )
        observed_goal_binding = _load_canonical_json(
            run_root / 'goal-binding-reconciliation.json',
            'Phase 3 goal-binding reconciliation',
        )
        _require(
            _exact_json_equal(observed_goal_binding, expected_goal_binding),
            'Phase 3 goal-binding reconciliation is not the exact production recomputation',
        )
        _validate_phase3_graph_prerequisites(
            repository,
            run_root,
            expected_plan=expected_plan,
            orchestrator_document=orchestrator_document,
            orchestration=orchestration,
        )
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
    profiler = _load_repository_module(
        repository,
        'tests/phase3_smoke_host_profiler.py',
        'Phase 3 smoke host profiler',
    )
    smoke_profile_path = (
        repository
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{candidate_id}-smoke-profile.json'
    )
    try:
        smoke_profile_binding = profiler.validate_campaign_smoke_profile(
            repository, candidate_root, candidate_id
        )
    except Exception as exc:
        raise EvidenceError(f'Phase 3 smoke host profile failed validation: {exc}') from exc
    _require(
        isinstance(smoke_profile_binding, dict)
        and smoke_profile_binding.get('profile_relative_path')
        == smoke_profile_path.relative_to(repository).as_posix(),
        'Phase 3 smoke host profile returned a foreign path binding',
    )
    smoke_profile_sha256 = file_sha256(
        _regular_file(smoke_profile_path, 'Phase 3 smoke host profile')
    )
    _require(
        smoke_profile_binding.get('profile_sha256') == smoke_profile_sha256,
        'Phase 3 smoke host profile changed after validation',
    )
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
            'contact_aggregator_binary',
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
    contact_aggregator_binary = _mapping(
        binding.get('contact_aggregator_binary'),
        'Phase 3 contact aggregator binary binding',
    )
    contact_gate_binary = _mapping(
        binding.get('contact_gate_binary'), 'Phase 3 contact gate binary binding'
    )
    _validate_contact_aggregator_binary_binding(contact_aggregator_binary)
    _validate_contact_gate_binary_binding(contact_gate_binary)
    _require(
        contact_aggregator_binary.get('source_inventory_sha256')
        == contact_gate_binary.get('source_inventory_sha256'),
        'Phase 3 contact binaries do not share one source inventory',
    )
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

    positive_directory = candidate_root / 'positive-control'
    positive_binding_path = positive_directory / 'positive-binding.json'
    positive_marker_path = positive_directory / 'PASS.json'
    positive_result_path = positive_directory / 'contact-control-result.json'
    positive_capture_path = positive_directory / 'capture.json'
    positive_command_progress_path = positive_directory / 'command-progress.json'
    positive_contact_progress_path = positive_directory / 'contact-progress.json'
    positive_driver_ready_path = positive_directory / 'contact-control.ready.json'
    positive_arm_request_path = positive_directory / 'contact-control.arm.json'
    positive_armed_ack_path = positive_directory / 'contact-control.armed.json'
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
        positive_command_progress_path,
        'Phase 3 positive-control command progress',
        maximum_bytes=4_096,
    )
    _load_canonical_json(
        positive_contact_progress_path,
        'Phase 3 positive-control contact progress',
    )
    _load_canonical_json(
        positive_driver_ready_path,
        'Phase 3 positive-control driver readiness',
        maximum_bytes=64 * 1024,
    )
    _load_canonical_json(
        positive_arm_request_path,
        'Phase 3 positive-control arm request',
        maximum_bytes=4_096,
    )
    _load_canonical_json(
        positive_armed_ack_path,
        'Phase 3 positive-control armed acknowledgment',
        maximum_bytes=4_096,
    )
    positive_initial_contact_gate = _load_canonical_json(
        positive_initial_contact_gate_path,
        'Phase 3 positive-control runtime gate',
    )
    positive_final_contact_gate = _load_canonical_json(
        positive_final_contact_gate_path,
        'Phase 3 positive-control final contact gate',
    )
    positive_component_manifest = _load_canonical_json(
        positive_component_manifest_path,
        'Phase 3 positive-control component manifest',
    )
    for path, label in (
        (positive_result_path, 'Phase 3 positive-control component result'),
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
    actual_component_paths = {
        path.relative_to(positive_directory).as_posix()
        for path in positive_directory.rglob('*')
        if path.is_file()
        and path.relative_to(positive_directory).as_posix() not in excluded_component_outputs
    }
    _require(
        actual_component_paths == PHASE3_POSITIVE_COMPONENT_PATHS,
        'Phase 3 positive-control component artifact path set is not exact',
    )
    for relative_path in PHASE3_POSITIVE_COMPONENT_PATHS:
        _regular_file(
            positive_directory / relative_path,
            f'Phase 3 positive-control component artifact {relative_path}',
        )
    manifest_artifacts = _list(
        positive_component_manifest.get('artifacts'),
        'Phase 3 positive-control component artifacts',
    )
    _require(
        {item.get('path') for item in manifest_artifacts if isinstance(item, dict)}
        == PHASE3_POSITIVE_COMPONENT_PATHS,
        'Phase 3 positive-control component manifest path set is not exact',
    )
    _validate_positive_component_processes(
        positive_directory,
        repository=repository,
        positive_plan=positive_plan,
    )
    for gate, label in (
        (positive_initial_contact_gate, 'Phase 3 positive initial contact gate'),
        (positive_final_contact_gate, 'Phase 3 positive final contact gate'),
    ):
        _validate_positive_gate_workspace_paths(
            gate,
            repository=repository,
            build_binding=binding,
            label=label,
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
    positive_control_trace = _list(
        _mapping(
            positive_control.get('control'),
            'Phase 3 positive-control control',
        ).get('command_trace'),
        'Phase 3 positive-control command trace',
    )
    _require(positive_control_trace, 'Phase 3 positive-control command trace is empty')
    first_component_command = _mapping(
        positive_control_trace[0],
        'Phase 3 positive-control first component command',
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
            'collector_command_progress_sha256': file_sha256(positive_command_progress_path),
            'collector_reconciled': True,
            'owned_process_group_shutdown': True,
        }
        and set(collector_reconciliation)
        == {
            'captured_command_count',
            'captured_exact_pair_count',
            'captured_release_expected_pair_count',
            'captured_release_snapshot_count',
            'command_progress_artifact_sha256',
            'command_progress_observed_steady_ns',
            'command_progress_stamp_ns',
            'contact_delivery_offset_strict_from_collector_sequence',
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
                'command_progress_artifact_sha256',
                'contact_progress_artifact_sha256',
                'contact_projection_sha256',
                'release_delivery_clock_offset_ns',
            }
        )
        and _passive_release_clock_offset_is_consistent(collector_reconciliation)
        and isinstance(collector_reconciliation['contact_progress_artifact_sha256'], str)
        and isinstance(collector_reconciliation['contact_projection_sha256'], str)
        and isinstance(collector_reconciliation['command_progress_artifact_sha256'], str)
        and re.fullmatch(
            r'[0-9a-f]{64}',
            collector_reconciliation['command_progress_artifact_sha256'],
        )
        is not None
        and re.fullmatch(
            r'[0-9a-f]{64}',
            collector_reconciliation['contact_progress_artifact_sha256'],
        )
        is not None
        and re.fullmatch(r'[0-9a-f]{64}', collector_reconciliation['contact_projection_sha256'])
        is not None
        and collector_reconciliation['contact_progress_artifact_sha256']
        == file_sha256(positive_contact_progress_path)
        and collector_reconciliation['command_progress_artifact_sha256']
        == file_sha256(positive_command_progress_path)
        and collector_reconciliation['captured_command_count']
        == collector_reconciliation['component_command_count'] + 1
        and collector_reconciliation['captured_exact_pair_count']
        == collector_reconciliation['component_exact_pair_count']
        and collector_reconciliation['contact_delivery_offset_strict_from_collector_sequence']
        == first_component_command.get('collector_sequence')
        and first_component_command.get('phase') == 'FORWARD'
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
            command_progress_path=positive_command_progress_path,
            driver_ready_path=positive_driver_ready_path,
            arm_request_path=positive_arm_request_path,
            armed_ack_path=positive_armed_ack_path,
            runtime_gate_path=positive_initial_contact_gate_path,
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
            'gz_partition': f'robotest_p3_{candidate_id}-smoke_00',
            'ros_domain_id': domain_base + 16,
            'run_id': f'{candidate_id}-smoke-s1-r0',
            'scenario_path': PHASE3_SCENARIO_PATHS[1],
        },
        'Phase 3 smoke plan changed',
    )
    first_trial = _mapping(trials[0], 'Phase 3 first trial')
    smoke_identity = {
        'candidate_id': f'{candidate_id}-smoke',
        'gz_partition': smoke_plan.get('gz_partition'),
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
        'smoke_profile_path': str(smoke_profile_path),
        'smoke_profile_sha256': smoke_profile_sha256,
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


def _canonical_cpu_list(cpu_ids: Sequence[int]) -> str:
    _require(cpu_ids, 'CPU ID list is empty')
    ranges: list[str] = []
    start = previous = cpu_ids[0]
    for cpu_id in cpu_ids[1:]:
        if cpu_id == previous + 1:
            previous = cpu_id
            continue
        ranges.append(str(start) if start == previous else f'{start}-{previous}')
        start = previous = cpu_id
    ranges.append(str(start) if start == previous else f'{start}-{previous}')
    return ','.join(ranges)


def _phase4_cpu_ids(value: object, label: str) -> list[int]:
    _require(
        isinstance(value, str)
        and value
        and len(value) <= 1024
        and re.fullmatch(r'\d+(?:-\d+)?(?:,\d+(?:-\d+)?)*', value) is not None,
        f'{label} is invalid',
    )
    cpu_ids: list[int] = []
    for component in value.split(','):
        bounds = component.split('-', 1)
        start = int(bounds[0])
        end = int(bounds[-1])
        _require(0 <= start <= end <= 4095, f'{label} contains an invalid range')
        cpu_ids.extend(range(start, end + 1))
    _require(
        cpu_ids == sorted(set(cpu_ids)) and value == _canonical_cpu_list(cpu_ids),
        f'{label} is not canonical',
    )
    return cpu_ids


def _phase4_cpu_mask_ids(value: object, label: str) -> list[int]:
    _require(
        isinstance(value, str)
        and len(value) <= 1024
        and re.fullmatch(r'[0-9a-f]{8}(?:,[0-9a-f]{8})*', value) is not None,
        f'{label} is invalid',
    )
    mask = int(value.replace(',', ''), 16)
    _require(mask.bit_length() <= 4096, f'{label} exceeds the CPU ID bound')
    cpu_ids = [cpu_id for cpu_id in range(mask.bit_length()) if mask & (1 << cpu_id)]
    _require(cpu_ids, f'{label} is empty')
    return cpu_ids


def _phase4_affinity_evidence(
    path: Path,
    label: str,
    *,
    phase: str,
    main_pid: int,
    managed_child_pid: int,
    managed_child_pgid: int,
    controller_target: Mapping[str, Any] | None = None,
) -> tuple[dict[str, Any], datetime]:
    value = _load_canonical_json(path, label, maximum_bytes=PHASE4_JSON_MAX_BYTES)
    _require(set(value) == PHASE4_AFFINITY_KEYS, f'{label} schema changed')
    expected_cpu_ids = list(range(6))
    _require(
        _exact_integer(value.get('schema_version'))
        and value.get('schema_version') == 1
        and value.get('phase') == phase
        and value.get('unit') == 'robotest-supervisor.service'
        and value.get('unit_cgroup') == '/system.slice/robotest-supervisor.service'
        and value.get('expected_cpuset') == '0-5'
        and value.get('main_pid') == main_pid
        and value.get('managed_child_pid') == managed_child_pid
        and value.get('managed_child_pgid') == managed_child_pgid
        and _exact_integer(value.get('controller_pid'))
        and value.get('controller_pid') > 1
        and _exact_integer(value.get('controller_count'))
        and value.get('controller_count') == 1
        and value.get('snapshot_stable') is True
        and value.get('verdict') == 'PASS',
        f'{label} identity or unit affinity changed',
    )
    process_values = _list(value.get('processes'), f'{label} processes')
    _require(process_values, f'{label} process set is empty')
    processes: list[dict[str, Any]] = []
    for index, process_value in enumerate(process_values):
        process = _mapping(process_value, f'{label} process {index}')
        _require(
            set(process) == PHASE4_AFFINITY_PROCESS_KEYS,
            f'{label} process {index} schema changed',
        )
        pid = process.get('pid')
        ppid = process.get('ppid')
        pgid = process.get('pgid')
        start_ticks = process.get('start_time_ticks')
        executable = process.get('executable')
        role = process.get('role')
        cgroup = _list(process.get('cgroup'), f'{label} process {pid} cgroup')
        raw_cpu_mask = process.get('cpus_allowed')
        raw_cpu_list = process.get('cpus_allowed_list')
        allowed_cpu_ids = _phase4_cpu_ids(raw_cpu_list, f'{label} process {pid} CPU list')
        mask_cpu_ids = _phase4_cpu_mask_ids(raw_cpu_mask, f'{label} process {pid} CPU mask')
        cgroup_parts = [line.split(':', 2) for line in cgroup if isinstance(line, str)]
        cgroup_paths = [parts[-1] for parts in cgroup_parts]
        _require(
            _exact_integer(pid)
            and pid > 1
            and _exact_integer(ppid)
            and ppid >= 0
            and _exact_integer(pgid)
            and pgid > 1
            and _exact_integer(start_ticks)
            and start_ticks > 0
            and isinstance(executable, str)
            and Path(executable).is_absolute()
            and role in {'controller', 'managed_child', 'supervisor_main', 'unit_descendant'}
            and len(cgroup_parts) == len(cgroup)
            and all(len(parts) == 3 for parts in cgroup_parts)
            and value['unit_cgroup'] in cgroup_paths
            and allowed_cpu_ids == mask_cpu_ids
            and set(allowed_cpu_ids) <= set(expected_cpu_ids),
            f'{label} process {pid} identity, cgroup, or CPU mask is invalid',
        )
        processes.append(process)
    process_ids = [process['pid'] for process in processes]
    _require(
        process_ids == sorted(set(process_ids))
        and _exact_integer(value.get('process_count'))
        and value.get('process_count') == len(processes),
        f'{label} process count or ordering is invalid',
    )
    by_pid = {process['pid']: process for process in processes}
    _require(
        main_pid in by_pid
        and managed_child_pid in by_pid
        and by_pid[managed_child_pid]['pgid'] == managed_child_pgid,
        f'{label} omits the supervisor or managed child identity',
    )
    for pid in process_ids:
        if pid == main_pid:
            continue
        current = pid
        visited: set[int] = set()
        while current != main_pid:
            _require(
                current in by_pid and current not in visited,
                f'{label} process {pid} does not descend from MainPID',
            )
            visited.add(current)
            current = by_pid[current]['ppid']
    _require(
        value['controller_pid'] in by_pid
        and by_pid[value['controller_pid']].get('role') == 'controller'
        and Path(by_pid[value['controller_pid']]['executable']).name == 'controller_server'
        and sum(process.get('role') == 'supervisor_main' for process in processes) == 1
        and sum(process.get('role') == 'managed_child' for process in processes) == 1
        and sum(process.get('role') == 'controller' for process in processes) == 1
        and by_pid[main_pid].get('role') == 'supervisor_main'
        and by_pid[managed_child_pid].get('role') == 'managed_child',
        f'{label} controller identity is not unique and exact',
    )
    if controller_target is not None:
        controller = by_pid[value['controller_pid']]
        _require(
            controller.get('pid') == controller_target.get('pid')
            and controller.get('ppid') == controller_target.get('ppid')
            and controller.get('pgid') == controller_target.get('pgid')
            and controller.get('start_time_ticks') == controller_target.get('start_time_ticks')
            and controller.get('executable') == controller_target.get('executable')
            and _exact_json_equal(controller.get('cgroup'), controller_target.get('cgroup')),
            f'{label} controller differs from the injected target',
        )
    captured_utc = _utc_timestamp(value.get('captured_utc'), f'{label} captured_utc')
    return value, captured_utc


def _phase4_lifecycle_startup(
    path: Path,
    *,
    context_started_utc: datetime,
    initial_affinity_utc: datetime,
) -> dict[str, Any]:
    value = _load_canonical_json(
        path,
        'Phase 4 lifecycle startup result',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    _require(
        set(value) == PHASE4_LIFECYCLE_STARTUP_KEYS,
        'Phase 4 lifecycle startup result schema changed',
    )
    _require(
        _exact_integer(value.get('schema_version'))
        and value.get('schema_version') == 1
        and value.get('verdict') == 'PASS'
        and value.get('accepted') is True
        and _exact_integer(value.get('exit_code'))
        and value.get('exit_code') == 0
        and value.get('service_name') == '/robotest/lifecycle_manager_navigation/manage_nodes'
        and _exact_integer(value.get('command'))
        and value.get('command') == 0
        and value.get('discovery_grace_sec') == 4.0
        and value.get('service_timeout_sec') == 20.0
        and value.get('response_timeout_sec') == 60.0
        and _finite_number(value.get('elapsed_wall_sec'))
        and 0 <= value['elapsed_wall_sec'] <= 110.0
        and value.get('watch_pid') is None
        and value.get('failure_kind') is None
        and value.get('failure_message') is None,
        'Phase 4 lifecycle startup result is not canonical PASS',
    )
    started_utc = _utc_timestamp(value.get('started_utc'), 'Phase 4 lifecycle started_utc')
    completed_utc = _utc_timestamp(value.get('completed_utc'), 'Phase 4 lifecycle completed_utc')
    _require(
        context_started_utc <= started_utc <= completed_utc <= initial_affinity_utc,
        'Phase 4 lifecycle timestamps are outside the initial-ready bracket',
    )
    return value


def _phase4_supervisor_config(
    run_directory: Path,
    *,
    run_id: str,
    isolation: Mapping[str, Any],
) -> dict[str, Any]:
    config = _load_canonical_json(
        run_directory / 'supervisor-config.json',
        'Phase 4 supervisor config',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    state_directory = f'/var/lib/robotest-supervisor/{run_id}'
    expected = {
        'schema_version': 1,
        'listen_address': '127.0.0.1:9080',
        'state_directory': state_directory,
        'heartbeat_poll_ms': 500,
        'heartbeat_stale_ms': 2000,
        'heartbeat_startup_timeout_ms': 110000,
        'termination_grace_ms': 5000,
        'shutdown_timeout_ms': 15000,
        'maximum_event_entries': 4096,
        'maximum_event_bytes': 8388608,
        'restart': {
            'initial_backoff_ms': 1000,
            'maximum_backoff_ms': 8000,
            'maximum_attempts': 4,
            'window_ms': 60000,
            'stable_reset_ms': 60000,
        },
        'children': [
            {
                'name': 'robotest-stack',
                'argv': ['/usr/libexec/robotest-supervisor/start-robotest-stack'],
                'working_directory': '/opt/robotest-lab',
                'environment': {
                    'GZ_PARTITION': isolation.get('gz_partition'),
                    'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
                    'ROBOTEST_CPUSET': '0-5',
                    'ROBOTEST_NAMESPACE': 'robotest',
                    'ROBOTEST_RUNTIME_STATE_DIRECTORY': state_directory,
                    'ROS_DOMAIN_ID': str(isolation.get('ros_domain_id')),
                },
                'required': True,
                'heartbeat_file': f'{state_directory}/robotest-stack.heartbeat',
            }
        ],
    }
    _require(
        _exact_json_equal(config, expected),
        'Phase 4 run-scoped supervisor config is not exact',
    )
    check_path = _regular_file(
        run_directory / 'supervisor-config-check.txt',
        'Phase 4 supervisor config check',
    )
    _require(
        check_path.read_bytes() == b'configuration valid\n',
        'Phase 4 supervisor config check did not pass exactly',
    )
    return config


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
    _require(
        set(manifest) >= PHASE4_REQUIRED_RAW,
        'Scenario 6 raw evidence hash coverage is not exact',
    )
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
    controller_target = _load_canonical_json(
        run_directory / 'controller-target.json',
        'Phase 4 controller target',
        maximum_bytes=PHASE4_JSON_MAX_BYTES,
    )
    _initial_affinity, initial_affinity_utc = _phase4_affinity_evidence(
        run_directory / 'runtime-affinity-initial.json',
        'Phase 4 initial runtime affinity',
        phase='initial',
        main_pid=measurements['supervisor_main_pid'],
        managed_child_pid=measurements['original_child_pid'],
        managed_child_pgid=measurements['original_child_pgid'],
        controller_target=controller_target,
    )
    _restored_affinity, restored_affinity_utc = _phase4_affinity_evidence(
        run_directory / 'runtime-affinity-restored.json',
        'Phase 4 restored runtime affinity',
        phase='restored',
        main_pid=measurements['supervisor_main_pid'],
        managed_child_pid=measurements['replacement_child_pid'],
        managed_child_pgid=measurements['replacement_child_pgid'],
    )
    _require(
        started_utc <= initial_affinity_utc <= restored_affinity_utc <= completed_utc,
        'Phase 4 runtime affinity captures are outside the run bracket',
    )
    _phase4_lifecycle_startup(
        run_directory / 'lifecycle-startup-result.json',
        context_started_utc=started_utc,
        initial_affinity_utc=initial_affinity_utc,
    )
    _phase4_supervisor_config(
        run_directory,
        run_id=run_directory.name,
        isolation=isolation,
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


def _git_invocation(repository: Path, arguments: list[str]) -> tuple[list[str], dict[str, str]]:
    git_directory = repository / '.git'
    _require(
        git_directory.is_dir() and not git_directory.is_symlink(),
        'repository Git directory is missing or linked',
    )
    git_info = git_directory / 'info'
    grafts_path = git_info / 'grafts'
    _require(
        not grafts_path.exists() and not grafts_path.is_symlink(),
        'repository Git graft metadata is present',
    )
    attributes_path = git_info / 'attributes'
    _require(
        not attributes_path.exists() and not attributes_path.is_symlink(),
        'repository Git local attributes are present',
    )
    exclude_path = git_info / 'exclude'
    if exclude_path.exists() or exclude_path.is_symlink():
        _require(
            exclude_path.is_file() and not exclude_path.is_symlink(),
            'repository Git local excludes are invalid',
        )
        _require(
            exclude_path.stat().st_size <= 16_384,
            'repository Git local excludes are oversized',
        )
        exclude_payload = exclude_path.read_bytes()
        _require(
            b'\r' not in exclude_payload
            and (not exclude_payload or exclude_payload.endswith(b'\n')),
            'repository Git local excludes are noncanonical',
        )
        try:
            exclude_lines = exclude_payload.decode('utf-8').splitlines()
        except UnicodeDecodeError as exc:
            raise EvidenceError('repository Git local excludes are not UTF-8') from exc
        _require(
            all(not line or line.startswith('#') for line in exclude_lines),
            'repository Git local excludes contain active rules',
        )
    command = [
        'git',
        '-C',
        str(repository),
        f'--git-dir={git_directory}',
        f'--work-tree={repository}',
        '-c',
        f'safe.directory={repository}',
        '-c',
        'core.fsmonitor=false',
        '-c',
        'core.hooksPath=/dev/null',
        '-c',
        'core.attributesFile=/dev/null',
        '-c',
        'core.bare=false',
        '-c',
        'core.excludesFile=/dev/null',
        '-c',
        'core.fileMode=true',
        '-c',
        'submodule.recurse=false',
        *arguments,
    ]
    environment = {
        'PATH': '/usr/bin:/bin',
        'LANG': 'C',
        'HOME': '/nonexistent',
        'GIT_CONFIG_NOSYSTEM': '1',
        'GIT_CONFIG_GLOBAL': '/dev/null',
        'GIT_ATTR_NOSYSTEM': '1',
        'GIT_NO_LAZY_FETCH': '1',
        'GIT_OPTIONAL_LOCKS': '0',
        'GIT_NO_REPLACE_OBJECTS': '1',
        'GIT_TERMINAL_PROMPT': '0',
    }
    return command, environment


def _git(repository: Path, arguments: list[str]) -> subprocess.CompletedProcess[str]:
    command, environment = _git_invocation(repository, arguments)
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        env=environment,
        text=True,
        timeout=30,
    )


def _git_bytes(repository: Path, arguments: list[str]) -> subprocess.CompletedProcess[bytes]:
    command, environment = _git_invocation(repository, arguments)
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        env=environment,
        timeout=30,
    )


def _git_ignore_is_from_tracked_root(repository: Path, relative: str) -> bool:
    ignored = _git(repository, ['check-ignore', '-v', '--', relative])
    if ignored.returncode != 0 or not ignored.stdout.endswith('\n'):
        return False
    lines = ignored.stdout.splitlines()
    if len(lines) != 1 or '\t' not in lines[0]:
        return False
    provenance, ignored_path = lines[0].split('\t', 1)
    fields = provenance.split(':', 2)
    if (
        len(fields) != 3
        or fields[0] != '.gitignore'
        or not fields[1].isdigit()
        or int(fields[1]) < 1
        or not fields[2]
        or ignored_path != relative
    ):
        return False
    ignore_path = repository / '.gitignore'
    if ignore_path.is_symlink() or not ignore_path.is_file():
        return False
    tracked = _git(repository, ['ls-files', '--error-unmatch', '--', '.gitignore'])
    worktree_diff = _git(repository, ['diff', '--quiet', '--', '.gitignore'])
    index_diff = _git(repository, ['diff', '--cached', '--quiet', '--', '.gitignore'])
    return tracked.returncode == 0 and worktree_diff.returncode == 0 and index_diff.returncode == 0


def _ignored_source_path_is_allowed(relative: str) -> bool:
    parts = Path(relative).parts
    if relative == 'config/release-claims.json':
        return True
    if parts and parts[0] in {'config', 'scenarios', 'src'}:
        return any(part in {'.pytest_cache', '__pycache__'} for part in parts[1:])
    if parts[:2] in {('packaging', 'debian'), ('supervisor', 'cmd')} or parts[:2] == (
        'supervisor',
        'internal',
    ):
        return False
    if parts[:2] == ('docs', 'results'):
        return True
    return any(
        part
        in {
            '.mypy_cache',
            '.pytest_cache',
            '__pycache__',
            'artifacts',
            'build',
            'install',
            'log',
        }
        for part in parts[1:]
    )


def _validate_ignored_source_paths(repository: Path) -> None:
    roots = ('config', 'docs', 'packaging', 'scenarios', 'scripts', 'src', 'supervisor', 'tests')
    ignored = _git(
        repository,
        [
            'ls-files',
            '-z',
            '--others',
            '--ignored',
            '--exclude-standard',
            '--',
            *roots,
        ],
    )
    _require(ignored.returncode == 0, 'cannot inspect ignored candidate source paths')
    _require(
        not ignored.stdout or ignored.stdout.endswith('\0'),
        'ignored candidate source path output is incomplete',
    )
    paths = ignored.stdout.split('\0')[:-1] if ignored.stdout else []
    _require(
        paths == sorted(set(paths)),
        'ignored candidate source paths are not sorted and unique',
    )
    for relative in paths:
        _require(
            _ignored_source_path_is_allowed(relative),
            f'ignored untracked source input is forbidden: {relative}',
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
            _git_ignore_is_from_tracked_root(repository, relative)
            and _git(repository, ['ls-files', '--error-unmatch', '--', relative]).returncode != 0,
            'evidence-commit remote proof is not ignored by the tracked root '
            f'.gitignore: {relative}',
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
    replace_refs = _git(
        repository,
        ['for-each-ref', '--format=%(refname)', 'refs/replace/'],
    )
    _require(
        replace_refs.returncode == 0 and replace_refs.stdout == '',
        'repository contains Git replacement refs',
    )
    _validate_ignored_source_paths(repository)
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
            'Read-only revalidation of the clone-local Phase 3 smoke host profile and '
            'caller-selected Phase 3 campaign, Phase 4 Scenario 6, tracked exact-SHA Phase 5 '
            'remote evidence, and a prior local aggregate; no latest discovery or execution'
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
