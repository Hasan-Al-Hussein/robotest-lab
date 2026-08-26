#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Pure, bounded evidence utilities for the Phase 3 benchmark orchestrator.

This module deliberately has no ROS imports.  The shell verifier and benchmark
runner use it for immutable suite planning, provenance manifests, trial
contexts, lifecycle schedules, component reconciliation, and canonical
analysis-request composition.  Runtime ROS observation lives in
``phase3_runtime_observer.py`` so every function here can be unit tested without
starting a graph or simulator.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tempfile
from typing import Any

import yaml

PRODUCER = 'robotest_phase3/benchmark_orchestrator'
SCHEMA_VERSION = 1
SUITE_SIZE = 15
REPETITIONS = 3
CPU_AFFINITY = [0, 1, 2, 3, 4, 5]
MAX_DOMAIN_ID = 232
MAX_CANDIDATE_DOMAIN_BASE = MAX_DOMAIN_ID - 16
JSON_MAX_BYTES = 32 * 1024 * 1024
CSV_MAX_BYTES = 1 * 1024 * 1024
LOG_MAX_BYTES = 8 * 1024 * 1024
PNG_MAX_BYTES = 4 * 1024 * 1024
PNG_MAX_COUNT = 8
RUN_DIRECTORY_MAX_BYTES = 256 * 1024 * 1024
AGGREGATE_DIRECTORY_MAX_BYTES = 64 * 1024 * 1024
RSS_MAX_BYTES = 6 * 1024**3
STRING_MAX_BYTES = 4096
COLLECTOR_WALL_TIMEOUT_S = 360.0
TRIAL_WALL_TIMEOUT_S = 300.0
CONTACT_CONTROL_WALL_TIMEOUT_S = 30.0
CONTACT_DRAIN_NS = 250_000_000
CONTACT_HEARTBEAT_NS = 200_000_000
CONTACT_MAX_PUBLIC_GAP_NS = 220_000_000
CONTACT_MAX_CLOCK_LAG_NS = 220_000_000
CONTACT_PUBLIC_TOPIC = '/robotest/validation/contacts'
CONTACT_GATE_NODE = '/robotest/contact_stream_gate'
CONTACT_PRIVATE_RAW_TOPIC = '/robotest/internal/raw_contacts'
CONTACT_INGRESS_MEMORY_BOUND_SCOPE_PARTS = (
    'post_dds_deserialization_of',
    'trusted_sole_private_bridge_input',
)
CONTACT_INGRESS_MEMORY_BOUND_SCOPE = '_'.join(CONTACT_INGRESS_MEMORY_BOUND_SCOPE_PARTS)
EXPECTED_CONTACT_STREAM_POLICY = {
    'active_pair_expiry_ns': 250_000_000,
    'active_pair_scope': 'support_robot_internal_and_countable_robot_external',
    'accepted_run_scope': 'bounded_pending_raw_and_public_source_liveness_required',
    'capacity_claim_scope': 'unchanged_pair_set_heartbeat_only',
    'completed_stamp_batching': 'finalize_on_strictly_greater_raw_stamp',
    'delivery_semantics': 'authoritative_delivered_active_pair_snapshot',
    'emission_policy': ('immediate_active_pair_set_transition_else_heartbeat_at_or_after_200ms'),
    'heartbeat_period_ns': 200_000_000,
    'ingress_memory_bound_scope': CONTACT_INGRESS_MEMORY_BOUND_SCOPE,
    'initial_finalized_stamp_suppressed': True,
    'max_pending_batch_clock_lag_ns': 220_000_000,
    'max_public_snapshot_gap_ns': 220_000_000,
    'max_public_snapshot_clock_lag_ns': 220_000_000,
    'max_raw_clock_lag_ns': 220_000_000,
    'passive_callback_clock_offset_semantics': 'diagnostic_noncausal',
    'limits': {
        'max_active_contact_pairs': 16,
        'max_active_contact_records': 16,
        'max_active_string_bytes': 65_536,
        'max_body_name_bytes': 4_096,
        'max_collision_name_bytes': 4_096,
        'max_contact_points_per_record': 64,
        'max_contact_records_per_pair': 4,
        'max_contact_string_bytes': 8_192,
        'max_frame_id_bytes': 256,
        'max_raw_contact_records': 16,
        'max_raw_messages_per_completed_stamp': 7,
        'max_raw_string_bytes_per_completed_stamp': 65_536,
    },
    'release_comparison': ('completed_absent_stamp_strictly_greater_than_last_seen_plus_gap'),
    'raw_contact_positions': 'required_nonempty_1_to_64',
    'raw_stamp_gap_semantics': (
        'consecutive_nonempty_raw_gaps_are_not_absence_evidence_and_have_no_independent_bound'
    ),
    'raw_messages_require_nonempty_contacts': True,
    'public_snapshot_cardinality': 'required_nonempty_1_to_16',
    'public_snapshot_clock_lag_scope': (
        'active_positive_control_and_explicit_caught_up_brackets_only'
    ),
    'public_snapshots_per_finalized_stamp': 'at_most_one',
    'public_snapshots_require_nonempty_contacts': True,
    'required_raw_frame_id': '',
    'retained_contact_payload': 'exact_nested_contact_record_copy',
    'synthesized_envelope': [
        'container',
        'current_completed_header',
        'normalized_pair_order',
    ],
    'synchronization': 'second_finalized_stamp_seeds_public_stream',
    'semantic_fatal_delivery': 'best_effort_diagnostic_snapshot_before_process_failure',
    'string_budget_accounting': ('payload_strings_plus_normalized_pair_key_once_per_stored_pair'),
}
LIFECYCLE_SAMPLE_PERIOD_NS = 200_000_000
LIFECYCLE_FIRST_OFFSET_NS = 11_600_000_000
LIFECYCLE_SAMPLE_COUNT = 96
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')
GIT_SHA_PATTERN = re.compile(r'^[0-9a-f]{40}$')
IDENTIFIER_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$')

SCENARIOS: tuple[tuple[int, str, str], ...] = (
    (1, 'baseline_navigation', 'scenarios/phase3_s1_baseline.yaml'),
    (2, 'deterministic_static_obstacle_replan', 'scenarios/phase3_s2_static_obstacle.yaml'),
    (3, 'deterministic_dynamic_obstacle', 'scenarios/phase3_s3_dynamic_obstacle.yaml'),
    (4, 'temporary_lidar_dropout', 'scenarios/phase3_s4_lidar_dropout.yaml'),
    (5, 'deterministic_odometry_drift', 'scenarios/phase3_s5_odom_drift.yaml'),
)

REQUIRED_LIFECYCLE_NODES = [
    'map_server',
    'amcl',
    'planner_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
    'collision_monitor',
]

AGGREGATE_METRICS = [
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

COLLECTOR_CONFIGURATION_FILES = (
    'src/robotest_metrics/robotest_metrics/buffers.py',
    'src/robotest_metrics/robotest_metrics/collector.py',
    'src/robotest_metrics/robotest_metrics/collector_node.py',
    'src/robotest_metrics/robotest_metrics/constants.py',
    'src/robotest_metrics/schema/capture.schema.json',
)

SOURCE_CONFIGURATION_FILES = (
    'config/collision-coverage.yaml',
    'docs/architecture/metrics-contract.md',
    'docs/architecture/topic-and-tf-contract.md',
    'docs/decisions/0005-phase3-deterministic-fault-protocol.md',
    'docs/decisions/0006-phase3-scenario-mechanics.md',
    'docs/testing/acceptance-criteria.md',
    'docs/testing/verification-matrix.md',
    *(item[2] for item in SCENARIOS),
)

SOURCE_TREE_ROOTS = (
    'pyproject.toml',
    'config/collision-coverage.yaml',
    'docs/architecture/metrics-contract.md',
    'docs/architecture/topic-and-tf-contract.md',
    'docs/decisions/0005-phase3-deterministic-fault-protocol.md',
    'docs/decisions/0006-phase3-scenario-mechanics.md',
    'docs/testing/acceptance-criteria.md',
    'docs/testing/verification-matrix.md',
    'scenarios',
    'scripts/run_benchmarks.sh',
    'scripts/verify_phase3.sh',
    'tests/phase2_graph_probe.py',
    'tests/phase2_lifecycle_probe.py',
    'tests/phase2_parameter_probe.py',
    'tests/phase2_startup_gate.py',
    'tests/phase3_orchestration.py',
    'tests/phase3_benchmark_runner.py',
    'tests/phase3_benchmark_runner_test.py',
    'tests/phase3_orchestration_test.py',
    'tests/phase3_runtime_gate.py',
    'tests/phase3_runtime_observer.py',
    'src/robotest_interfaces',
    'src/robotest_description',
    'src/robotest_faults',
    'src/robotest_sim',
    'src/robotest_navigation',
    'src/robotest_missions',
    'src/robotest_metrics',
    'src/robotest_scenarios',
)

RUNTIME_PACKAGES = (
    'robotest_interfaces',
    'robotest_description',
    'robotest_faults',
    'robotest_sim',
    'robotest_navigation',
    'robotest_missions',
    'robotest_metrics',
    'robotest_scenarios',
)

RESOURCE_METRIC_FIELDS = (
    'cpu_percent_mean',
    'cpu_percent_p95',
    'cpu_percent_peak',
    'missing_sample_count',
    'oom_kill',
    'overflow_free',
    'peak_rss_sum_bytes',
    'pid_reuse_detected',
    'sample_count',
    'sampler_started_before_launch',
    'sampler_stopped_after_shutdown',
    'wsl_peak_memory_bytes',
    'wsl_peak_swap_bytes',
)

PACKAGE_SHARE_SOURCE_DIRECTORIES = (
    'behavior_trees',
    'config',
    'launch',
    'maps',
    'meshes',
    'msg',
    'rviz',
    'schema',
    'srv',
    'urdf',
    'worlds',
)

IGNORED_PARTS = {
    '.git',
    '.pytest_cache',
    '__pycache__',
    'build',
    'install',
    'log',
}
IGNORED_SUFFIXES = {'.pyc', '.pyo', '.swp', '.tmp'}


class EvidenceError(RuntimeError):
    """Raised when evidence is missing, malformed, ambiguous, or unbounded."""


@dataclass(frozen=True)
class TrialPlan:
    """Immutable identity and isolation allocation for one suite index."""

    candidate_id: str
    gz_partition: str
    repetition_index: int
    ros_domain_id: int
    run_id: str
    scenario_id: int
    scenario_name: str
    scenario_path: str
    scenario_sha256: str
    suite_index: int


def _require_mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError(f'{name} must be an object')
    return value


def _require_bool(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise EvidenceError(f'{name} must be a boolean')
    return value


def _require_int(value: Any, name: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f'{name} must be an integer')
    if minimum is not None and value < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return value


def _require_number(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f'{name} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise EvidenceError(f'{name} must be finite')
    if minimum is not None and result < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return result


def require_bounded_string(value: Any, name: str) -> str:
    """Require a non-empty UTF-8 string under the revision-2 byte cap."""
    if not isinstance(value, str) or not value:
        raise EvidenceError(f'{name} must be a non-empty string')
    try:
        encoded = value.encode('utf-8')
    except UnicodeEncodeError as exc:
        raise EvidenceError(f'{name} is not valid UTF-8') from exc
    if len(encoded) > STRING_MAX_BYTES:
        raise EvidenceError(f'{name} exceeds {STRING_MAX_BYTES} UTF-8 bytes')
    return value


def require_sha256(value: Any, name: str) -> str:
    """Require a lowercase SHA-256 string."""
    if not isinstance(value, str) or SHA256_PATTERN.fullmatch(value) is None:
        raise EvidenceError(f'{name} must be a lowercase SHA-256')
    return value


def canonical_json_bytes(document: Any) -> bytes:
    """Return strict canonical JSON bytes with one trailing newline."""
    try:
        payload = json.dumps(
            document,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise EvidenceError(f'document is not canonical-JSON serializable: {exc}') from exc
    return (payload + '\n').encode('utf-8')


def canonical_sha256(document: Any) -> str:
    """Hash the exact canonical JSON bytes used by Phase 3 artifacts."""
    return hashlib.sha256(canonical_json_bytes(document)).hexdigest()


def file_sha256(path: Path) -> str:
    """Stream a regular file into SHA-256 without an unbounded read."""
    if not path.is_file():
        raise EvidenceError(f'missing regular file: {path}')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def atomic_write_bytes(path: Path, payload: bytes, maximum_bytes: int) -> None:
    """Atomically write one bounded artifact and fsync the containing directory."""
    if len(payload) > maximum_bytes:
        raise EvidenceError(
            f'{path} is {len(payload)} bytes and exceeds its {maximum_bytes}-byte cap'
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'wb') as target:
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, path)
        parent_fd = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_json(
    path: Path,
    document: Any,
    *,
    maximum_bytes: int = JSON_MAX_BYTES,
    sidecar: bool = False,
) -> str:
    """Write canonical JSON and optionally an exact GNU-style SHA sidecar."""
    payload = canonical_json_bytes(document)
    atomic_write_bytes(path, payload, maximum_bytes)
    digest = hashlib.sha256(payload).hexdigest()
    if sidecar:
        atomic_write_bytes(
            Path(f'{path}.sha256'),
            f'{digest}  {path.name}\n'.encode('ascii'),
            256,
        )
    return digest


def load_json(path: Path, *, maximum_bytes: int = JSON_MAX_BYTES) -> Any:
    """Load one bounded UTF-8 JSON document."""
    if not path.is_file():
        raise EvidenceError(f'missing JSON artifact: {path}')
    size = path.stat().st_size
    if size > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot load JSON artifact {path}: {exc}') from exc


def load_yaml(path: Path, *, maximum_bytes: int = 1024 * 1024) -> Mapping[str, Any]:
    """Load a bounded YAML mapping with safe construction."""
    if not path.is_file() or path.stat().st_size > maximum_bytes:
        raise EvidenceError(f'YAML input is missing or too large: {path}')
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise EvidenceError(f'cannot load YAML input {path}: {exc}') from exc
    return _require_mapping(document, str(path))


def verify_json_sidecar(path: Path) -> str:
    """Verify the exact single-line sidecar convention used by components."""
    sidecar = Path(f'{path}.sha256')
    if not sidecar.is_file() or sidecar.stat().st_size > 256:
        raise EvidenceError(f'missing or oversized SHA sidecar: {sidecar}')
    try:
        line = sidecar.read_text(encoding='ascii')
    except (OSError, UnicodeError) as exc:
        raise EvidenceError(f'cannot read SHA sidecar {sidecar}: {exc}') from exc
    expected = f'{file_sha256(path)}  {path.name}\n'
    if line != expected:
        raise EvidenceError(f'SHA sidecar does not match canonical artifact: {sidecar}')
    return expected[:64]


def safe_candidate_id(value: str) -> str:
    """Validate the path-safe immutable candidate identifier."""
    value = require_bounded_string(value, 'candidate_id')
    if IDENTIFIER_PATTERN.fullmatch(value) is None:
        raise EvidenceError('candidate_id must match [A-Za-z0-9][A-Za-z0-9_.-]{0,127}')
    return value


def plan_suite(workspace: Path, candidate_id: str, domain_base: int) -> list[TrialPlan]:
    """Freeze the exact ordered 15-run candidate ledger."""
    candidate_id = safe_candidate_id(candidate_id)
    if not 0 <= domain_base <= MAX_CANDIDATE_DOMAIN_BASE:
        raise EvidenceError(
            f'domain_base must be in [0, {MAX_CANDIDATE_DOMAIN_BASE}] so the '
            '15 trials, positive control, and smoke all receive unique domains'
        )
    plans: list[TrialPlan] = []
    for scenario_id, scenario_name, relative_path in SCENARIOS:
        path = workspace / relative_path
        scenario = load_yaml(path)
        if scenario.get('scenario_id') != scenario_id or scenario.get('scenario_name') != (
            scenario_name
        ):
            raise EvidenceError(f'scenario identity mismatch in {relative_path}')
        if scenario.get('simulator_seed') != 42 or scenario.get('retries') != 0:
            raise EvidenceError(f'scenario seed/retry policy is not frozen in {relative_path}')
        if (
            float(scenario.get('mission_timeout_sim_s', -1.0)) != 180.0
            or float(scenario.get('wall_escape_timeout_s', -1.0)) != TRIAL_WALL_TIMEOUT_S
        ):
            raise EvidenceError(f'scenario timeout policy is not frozen in {relative_path}')
        scenario_hash = file_sha256(path)
        for repetition in range(REPETITIONS):
            suite_index = (scenario_id - 1) * REPETITIONS + repetition
            plans.append(
                TrialPlan(
                    candidate_id=candidate_id,
                    gz_partition=f'robotest_p3_{candidate_id}_{suite_index:02d}',
                    repetition_index=repetition,
                    ros_domain_id=domain_base + suite_index,
                    run_id=f'{candidate_id}-s{scenario_id}-r{repetition}-i{suite_index:02d}',
                    scenario_id=scenario_id,
                    scenario_name=scenario_name,
                    scenario_path=relative_path,
                    scenario_sha256=scenario_hash,
                    suite_index=suite_index,
                )
            )
    if len(plans) != SUITE_SIZE or [plan.suite_index for plan in plans] != list(range(SUITE_SIZE)):
        raise EvidenceError('internal suite-plan ordering failure')
    if (
        len({plan.ros_domain_id for plan in plans}) != SUITE_SIZE
        or len({plan.gz_partition for plan in plans}) != SUITE_SIZE
    ):
        raise EvidenceError('suite isolation identifiers are not unique')
    return plans


def suite_document(workspace: Path, candidate_id: str, domain_base: int) -> dict[str, Any]:
    """Return the canonical candidate plan plus reserved preflight identities."""
    plans = plan_suite(workspace, candidate_id, domain_base)
    return {
        'aggregate_metrics': AGGREGATE_METRICS,
        'candidate_id': safe_candidate_id(candidate_id),
        'cpu_affinity': CPU_AFFINITY,
        'domain_base': domain_base,
        'positive_control': {
            'gz_partition': f'robotest_p3_{candidate_id}_positive_control',
            'ros_domain_id': domain_base + 15,
            'run_id': f'{candidate_id}-positive-control',
        },
        'producer': PRODUCER,
        'schema_version': SCHEMA_VERSION,
        'smoke': {
            'gz_partition': f'robotest_p3_{candidate_id}_smoke',
            'ros_domain_id': domain_base + 16,
            'run_id': f'{candidate_id}-smoke-s1-r0',
            'scenario_path': SCENARIOS[0][2],
        },
        'trials': [asdict(plan) for plan in plans],
    }


def _iter_files(root: Path, entries: Iterable[str]) -> Iterable[tuple[str, Path]]:
    seen: set[str] = set()
    for relative in entries:
        path = root / relative
        if path.is_symlink() and not path.exists():
            raise EvidenceError(f'broken provenance symlink: {path}')
        candidates = [path] if path.is_file() else sorted(path.rglob('*')) if path.is_dir() else []
        if not candidates:
            raise EvidenceError(f'missing provenance input: {path}')
        for candidate in candidates:
            if not candidate.is_file():
                continue
            relative_path = candidate.relative_to(root).as_posix()
            parts = set(Path(relative_path).parts)
            if parts & IGNORED_PARTS or candidate.suffix in IGNORED_SUFFIXES:
                continue
            if relative_path in seen:
                continue
            seen.add(relative_path)
            yield relative_path, candidate


def tree_manifest(root: Path, entries: Iterable[str]) -> dict[str, Any]:
    """Hash a fixed, normalized set of source or installed files."""
    records = [
        {
            'bytes': path.stat().st_size,
            'path': relative,
            'sha256': file_sha256(path),
        }
        for relative, path in _iter_files(root, entries)
    ]
    if not records:
        raise EvidenceError('provenance manifest cannot be empty')
    records.sort(key=lambda item: item['path'])
    return {
        'aggregate_sha256': canonical_sha256(records),
        'file_count': len(records),
        'files': records,
        'total_bytes': sum(item['bytes'] for item in records),
    }


def install_manifest(workspace: Path) -> dict[str, Any]:
    """Hash the exact installed runtime prefixes used by Phase 3."""
    install_root = workspace / 'install'
    return tree_manifest(install_root, RUNTIME_PACKAGES)


def _elf_build_id(path: Path) -> str:
    result = subprocess.run(
        ['readelf', '-n', str(path)],
        capture_output=True,
        check=False,
        text=True,
        timeout=10.0,
    )
    for line in result.stdout.splitlines():
        if 'Build ID:' in line:
            value = line.split('Build ID:', 1)[1].strip()
            if value:
                return value
    raise EvidenceError(f'ELF build ID is unavailable: {path}')


CONTACT_GATE_SOURCE_TAG = b'ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
CONTACT_GATE_SOURCE_PATHS = (
    'src/robotest_sim/CMakeLists.txt',
    'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
    'src/robotest_sim/src/contact_stream_gate.cpp',
    'src/robotest_sim/src/contact_stream_gate_node.cpp',
)


def _elf_embedded_source_inventory_sha256(path: Path) -> str:
    pattern = re.compile(re.escape(CONTACT_GATE_SOURCE_TAG) + rb'([0-9a-f]{64})')
    overlap = b''
    matches: set[str] = set()
    retained = len(CONTACT_GATE_SOURCE_TAG) + 63
    with path.open('rb') as stream:
        while chunk := stream.read(1024 * 1024):
            combined = overlap + chunk
            for match in pattern.finditer(combined):
                matches.add(match.group(1).decode('ascii'))
                if len(matches) > 1:
                    raise EvidenceError(
                        f'contact gate ELF has conflicting tagged source inventory values: {path}'
                    )
            overlap = combined[-retained:]
    if len(matches) != 1:
        raise EvidenceError(f'contact gate ELF lacks tagged source inventory: {path}')
    return next(iter(matches))


def contact_gate_source_inventory(workspace: Path) -> dict[str, Any]:
    return {
        'schema_version': 1,
        'sources': [
            {'path': path, 'sha256': file_sha256(workspace / path)}
            for path in CONTACT_GATE_SOURCE_PATHS
        ],
    }


def contact_gate_build_install_binding(workspace: Path) -> dict[str, Any]:
    """Bind the compiled contact gate build artifact to its installed ELF."""
    build_path = (workspace / 'build/robotest_sim/contact_stream_gate').resolve(strict=True)
    installed_declared_path = workspace / (
        'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
    )
    installed_path = installed_declared_path.resolve(strict=True)
    if not build_path.is_file() or not installed_path.is_file():
        raise EvidenceError('contact stream gate build/install artifact is not regular')
    if not os.access(build_path, os.X_OK) or not os.access(installed_path, os.X_OK):
        raise EvidenceError('contact stream gate build/install artifact is not executable')
    build_sha256 = file_sha256(build_path)
    installed_sha256 = file_sha256(installed_path)
    build_install_sha256_match = build_sha256 == installed_sha256
    if not build_install_sha256_match:
        raise EvidenceError('contact stream gate build/install ELF hashes differ')
    build_id = _elf_build_id(build_path)
    installed_build_id = _elf_build_id(installed_path)
    if build_id != installed_build_id:
        raise EvidenceError('contact stream gate build/install ELF build IDs differ')
    source_inventory_sha256 = canonical_sha256(contact_gate_source_inventory(workspace))
    build_embedded_source = _elf_embedded_source_inventory_sha256(build_path)
    installed_embedded_source = _elf_embedded_source_inventory_sha256(installed_path)
    build_embedded_source_match = build_embedded_source == source_inventory_sha256
    installed_embedded_source_match = installed_embedded_source == source_inventory_sha256
    if not build_embedded_source_match or not installed_embedded_source_match:
        raise EvidenceError(
            'contact stream gate ELF does not embed the current source inventory hash'
        )
    return {
        'build_embedded_source_inventory_match': build_embedded_source_match,
        'build_embedded_source_inventory_sha256': build_embedded_source,
        'build_elf_build_id': build_id,
        'build_regular_executable': True,
        'build_path': build_path.relative_to(workspace).as_posix(),
        'build_sha256': build_sha256,
        'build_install_build_id_match': True,
        'build_install_sha256_match': build_install_sha256_match,
        'installed_declared_path': installed_declared_path.relative_to(workspace).as_posix(),
        'installed_declared_samefile': installed_declared_path.samefile(installed_path),
        'installed_embedded_source_inventory_match': installed_embedded_source_match,
        'installed_embedded_source_inventory_sha256': installed_embedded_source,
        'installed_elf_build_id': installed_build_id,
        'installed_path': installed_path.relative_to(workspace).as_posix(),
        'installed_regular_executable': True,
        'installed_sha256': installed_sha256,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': source_inventory_sha256,
    }


def source_install_correspondence(workspace: Path) -> dict[str, Any]:
    """Prove installed Python/share bytes equal their clean source inputs."""
    records: list[dict[str, Any]] = []
    mismatches: list[str] = []
    for package in RUNTIME_PACKAGES:
        source_root = workspace / 'src' / package
        install_root = workspace / 'install' / package
        candidates: list[tuple[str, Path, Path]] = []
        package_xml = source_root / 'package.xml'
        if package_xml.is_file():
            candidates.append(
                (
                    'package_share',
                    package_xml,
                    install_root / 'share' / package / 'package.xml',
                )
            )
        python_root = source_root / package
        if python_root.is_dir():
            direct_roots = list((install_root / 'lib').glob(f'python*/site-packages/{package}'))
            binding_type = 'python_module'
            if len(direct_roots) == 1:
                installed_python_root = direct_roots[0]
            else:
                egg_links = list(
                    (install_root / 'lib').glob(
                        f'python*/site-packages/{package.replace("_", "-")}*.egg-link'
                    )
                )
                if len(egg_links) != 1:
                    mismatches.append(
                        f'{package}:python_runtime_roots='
                        f'{len(direct_roots)}:egg_links={len(egg_links)}'
                    )
                    installed_python_root = None
                else:
                    try:
                        egg_lines = egg_links[0].read_text(encoding='utf-8').splitlines()
                        egg_build_root = Path(egg_lines[0]).resolve()
                        egg_build_root.relative_to((workspace / 'build' / package).resolve())
                        installed_python_root = egg_build_root / package
                        if installed_python_root.resolve() != python_root.resolve():
                            raise ValueError('egg-link module root does not resolve to source')
                        binding_type = 'python_egg_link_module'
                    except (IndexError, OSError, UnicodeError, ValueError) as exc:
                        mismatches.append(f'{package}:invalid_egg_link:{exc}')
                        installed_python_root = None
            for relative, source in _iter_files(source_root, (package,)):
                if source.suffix != '.py':
                    continue
                inside_package = Path(relative).relative_to(package)
                if installed_python_root is None:
                    continue
                candidates.append((binding_type, source, installed_python_root / inside_package))
        for directory in PACKAGE_SHARE_SOURCE_DIRECTORIES:
            source_directory = source_root / directory
            if not source_directory.is_dir():
                continue
            for relative, source in _iter_files(source_root, (directory,)):
                candidates.append(
                    (
                        'package_share',
                        source,
                        install_root / 'share' / package / relative,
                    )
                )
        for binding_type, source, installed in candidates:
            source_hash = file_sha256(source)
            installed_hash = file_sha256(installed) if installed.is_file() else None
            matches = installed_hash == source_hash
            record = {
                'binding_type': binding_type,
                'installed_path': installed.relative_to(workspace).as_posix(),
                'installed_sha256': installed_hash,
                'matches': matches,
                'package': package,
                'source_path': source.relative_to(workspace).as_posix(),
                'source_sha256': source_hash,
            }
            records.append(record)
            if not matches:
                mismatches.append(f'{package}:{record["source_path"]}')
    if not records:
        raise EvidenceError('source/install correspondence cannot be empty')
    if mismatches:
        raise EvidenceError(
            'source/install correspondence failed: ' + ', '.join(sorted(mismatches)[:16])
        )
    records.sort(key=lambda item: (item['package'], item['source_path']))
    return {
        'aggregate_sha256': canonical_sha256(records),
        'all_match': True,
        'file_count': len(records),
        'records': records,
    }


def configuration_hash(workspace: Path, entries: Iterable[str]) -> str:
    """Hash canonical path/content identities for a fixed configuration set."""
    return tree_manifest(workspace, entries)['aggregate_sha256']


def build_binding(
    workspace: Path,
    *,
    git_sha: str,
    git_status_porcelain: str,
) -> dict[str, Any]:
    """Create the immutable source/install binding consumed by every trial."""
    if GIT_SHA_PATTERN.fullmatch(git_sha) is None:
        raise EvidenceError('git_sha must be a lowercase 40-character commit')
    source = tree_manifest(workspace, SOURCE_TREE_ROOTS)
    installed = install_manifest(workspace)
    source_install = source_install_correspondence(workspace)
    contact_gate_binary = contact_gate_build_install_binding(workspace)
    metrics_contract_sha = file_sha256(workspace / 'docs/architecture/metrics-contract.md')
    target_set_sha = file_sha256(workspace / 'docs/testing/acceptance-criteria.md')
    return {
        'collector_configuration_sha256': configuration_hash(
            workspace, COLLECTOR_CONFIGURATION_FILES
        ),
        'created_by': PRODUCER,
        'contact_gate_binary': contact_gate_binary,
        'git': {
            'dirty': bool(git_status_porcelain),
            'sha': git_sha,
            'status_porcelain': git_status_porcelain,
        },
        'install': installed,
        'metrics_contract_sha256': metrics_contract_sha,
        'schema_version': SCHEMA_VERSION,
        'source': source,
        'source_install': source_install,
        'source_configuration_sha256': configuration_hash(workspace, SOURCE_CONFIGURATION_FILES),
        'target_set_sha256': target_set_sha,
    }


def validate_build_binding(
    workspace: Path,
    binding: Mapping[str, Any],
    *,
    git_sha: str,
    git_status_porcelain: str,
) -> dict[str, Any]:
    """Recompute a build binding and return exact start/end comparison facts."""
    expected = build_binding(
        workspace,
        git_sha=git_sha,
        git_status_porcelain=git_status_porcelain,
    )
    for field in (
        'collector_configuration_sha256',
        'metrics_contract_sha256',
        'source_configuration_sha256',
        'target_set_sha256',
    ):
        if binding.get(field) != expected[field]:
            raise EvidenceError(f'build binding differs for {field}')
    source = _require_mapping(binding.get('source'), 'build_binding.source')
    installed = _require_mapping(binding.get('install'), 'build_binding.install')
    source_install = _require_mapping(
        binding.get('source_install'),
        'build_binding.source_install',
    )
    contact_gate_binary = _require_mapping(
        binding.get('contact_gate_binary'), 'build_binding.contact_gate_binary'
    )
    if source.get('aggregate_sha256') != expected['source']['aggregate_sha256']:
        raise EvidenceError('source tree differs from verified build binding')
    if installed.get('aggregate_sha256') != expected['install']['aggregate_sha256']:
        raise EvidenceError('installed overlay differs from verified build binding')
    if (
        source_install.get('all_match') is not True
        or source_install.get('aggregate_sha256') != expected['source_install']['aggregate_sha256']
    ):
        raise EvidenceError('source/install correspondence differs from verified binding')
    if binding.get('git') != expected['git']:
        raise EvidenceError('Git state differs from verified build binding')
    if dict(contact_gate_binary) != expected['contact_gate_binary']:
        raise EvidenceError('contact gate binary differs from verified build binding')
    return expected


def lifecycle_schedule(run_id: str, accepted_goal_stamp_ns: int) -> dict[str, Any]:
    """Generate 96 absolute snapshots spanning the complete S4 recovery horizon."""
    run_id = require_bounded_string(run_id, 'run_id')
    t0 = _require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns', minimum=1)
    stamps = [
        t0 + LIFECYCLE_FIRST_OFFSET_NS + index * LIFECYCLE_SAMPLE_PERIOD_NS
        for index in range(LIFECYCLE_SAMPLE_COUNT)
    ]
    return {
        'lifecycle_schedule_schema_version': 1,
        'requested_stamps_ns': stamps,
        'run_id': run_id,
    }


def acceptance_for_scenario(scenario_id: int) -> tuple[dict[str, Any], list[str]]:
    """Return only thresholds frozen in the revision-2 target set."""
    if scenario_id not in range(1, 6):
        raise EvidenceError('scenario_id must be in [1, 5]')
    acceptance: dict[str, Any] = {
        'measurements.collision_count': {'maximum': 0},
        'measurements.completion_time_sim_s': {'maximum': 180.0},
        'measurements.localization_coverage_ratio': {'minimum': 0.95},
        'measurements.peak_rss_sum_bytes': {'maximum': RSS_MAX_BYTES},
        'measurements.rtf_median': {'minimum': 0.80},
        'measurements.rtf_p5': {'minimum': 0.50},
    }
    required = [
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
    else:
        required.append('measurements.odometry_drift')
    return dict(sorted(acceptance.items())), sorted(set(required))


def _collision_provenance(
    manifest: Mapping[str, Any],
    collector_configuration_sha256: str,
) -> dict[str, str]:
    result = {
        'bridge_sha256': require_sha256(manifest.get('bridge_sha256'), 'bridge_sha256'),
        'collector_configuration_sha256': require_sha256(
            collector_configuration_sha256, 'collector_configuration_sha256'
        ),
        'contact_configuration_sha256': require_sha256(
            manifest.get('contact_configuration_sha256'), 'contact_configuration_sha256'
        ),
        'coverage_manifest_sha256': require_sha256(
            manifest.get('manifest_sha256'), 'manifest_sha256'
        ),
        'rendered_sdf_sha256': require_sha256(
            manifest.get('rendered_sdf_sha256'), 'rendered_sdf_sha256'
        ),
        'robot_description_sha256': require_sha256(
            manifest.get('robot_description_sha256'), 'robot_description_sha256'
        ),
        'world_source_sha256': require_sha256(
            manifest.get('world_source_sha256'), 'world_source_sha256'
        ),
    }
    return result


def _contact_stream_manifest_v3(manifest: Mapping[str, Any], workspace: Path) -> Mapping[str, Any]:
    """Reject legacy manifests and return the hash-bound contact-stream contract."""
    if manifest.get('schema_version') != 3:
        raise EvidenceError('collision coverage manifest must be schema_version 3')
    contact_stream = _require_mapping(manifest.get('contact_stream'), 'contact_stream')
    if set(contact_stream) != {
        'gate',
        'policy',
        'policy_sha256',
        'qos',
        'schema_version',
        'topics',
    }:
        raise EvidenceError('contact stream manifest fields differ from the frozen contract')
    if contact_stream.get('schema_version') != 1:
        raise EvidenceError('contact stream manifest must be schema_version 1')
    topics = _require_mapping(contact_stream.get('topics'), 'contact_stream.topics')
    if dict(topics) != {
        'gazebo_raw': CONTACT_PUBLIC_TOPIC,
        'private_raw_ros': CONTACT_PRIVATE_RAW_TOPIC,
        'public_ros': CONTACT_PUBLIC_TOPIC,
    }:
        raise EvidenceError('contact stream topics differ from the frozen contract')
    qos = _require_mapping(contact_stream.get('qos'), 'contact_stream.qos')
    if dict(qos) != {
        'private_raw_ros': {
            'depth': 64,
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST',
            'reliability': 'RELIABLE',
        },
        'public_ros': {
            'depth': 10,
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST',
            'reliability': 'RELIABLE',
        },
    }:
        raise EvidenceError('contact stream QoS differs from the frozen contract')
    gate = _require_mapping(contact_stream.get('gate'), 'contact_stream.gate')
    if set(gate) != {
        'executable',
        'launch_sha256',
        'package',
        'source_inventory',
        'source_inventory_sha256',
    }:
        raise EvidenceError('contact stream gate binding fields are invalid')
    if gate.get('package') != 'robotest_sim' or gate.get('executable') != 'contact_stream_gate':
        raise EvidenceError('contact stream gate owner differs from the frozen contract')
    launch_sha256 = require_sha256(gate.get('launch_sha256'), 'contact_stream.gate.launch_sha256')
    if launch_sha256 != file_sha256(workspace / 'src/robotest_sim/launch/sim.launch.py'):
        raise EvidenceError('contact stream launch hash differs from workspace source')
    source_inventory = _require_mapping(
        gate.get('source_inventory'), 'contact_stream.gate.source_inventory'
    )
    if (
        set(source_inventory) != {'schema_version', 'sources'}
        or source_inventory.get('schema_version') != 1
    ):
        raise EvidenceError('contact stream gate source inventory schema is invalid')
    sources = source_inventory.get('sources')
    if not isinstance(sources, list) or len(sources) != len(CONTACT_GATE_SOURCE_PATHS):
        raise EvidenceError('contact stream gate source inventory length is invalid')
    for index, (entry_value, expected_path) in enumerate(
        zip(sources, CONTACT_GATE_SOURCE_PATHS, strict=True)
    ):
        entry = _require_mapping(
            entry_value, f'contact_stream.gate.source_inventory.sources[{index}]'
        )
        if set(entry) != {'path', 'sha256'} or entry.get('path') != expected_path:
            raise EvidenceError('contact stream gate source inventory ordering is invalid')
        declared_sha256 = require_sha256(
            entry.get('sha256'), f'contact stream gate source {expected_path}'
        )
        if declared_sha256 != file_sha256(workspace / expected_path):
            raise EvidenceError('contact stream gate source hash differs from workspace')
    declared_inventory_hash = require_sha256(
        gate.get('source_inventory_sha256'),
        'contact_stream.gate.source_inventory_sha256',
    )
    if canonical_sha256(source_inventory) != declared_inventory_hash:
        raise EvidenceError('contact stream gate source inventory hash mismatch')
    policy = _require_mapping(contact_stream.get('policy'), 'contact_stream.policy')
    if dict(policy) != EXPECTED_CONTACT_STREAM_POLICY:
        raise EvidenceError('contact stream policy differs from the frozen v3 contract')
    declared_policy_hash = require_sha256(
        contact_stream.get('policy_sha256'), 'contact_stream.policy_sha256'
    )
    if canonical_sha256(policy) != declared_policy_hash:
        raise EvidenceError('contact stream policy hash mismatch')
    return contact_stream


def positive_control_qualified_snapshot_stamp(result: Mapping[str, Any]) -> int:
    """Return the positive-control release snapshot after validating its binding."""
    if result.get('schema_version') != 1 or result.get('producer') != (
        'robotest_scenarios/contact_control_driver'
    ):
        raise EvidenceError('positive-control producer/schema is invalid')
    verdict = _require_mapping(result.get('verdict'), 'positive_control.verdict')
    if (
        result.get('status') != 'PASS'
        or verdict.get('authority') != 'component_only'
        or verdict.get('benchmark_pass') is not None
        or verdict.get('exit_code') != 0
    ):
        raise EvidenceError('positive-control component did not PASS')
    control = _require_mapping(result.get('control'), 'positive_control.control')
    timeline = _require_mapping(control.get('timeline'), 'positive_control.control.timeline')
    boundary = _require_int(
        timeline.get('release_required_through_stamp_ns'),
        'positive_control.release_required_through_stamp_ns',
        minimum=1,
    )
    qualifying = _require_int(
        timeline.get('release_qualified_snapshot_stamp_ns'),
        'positive_control.release_qualified_snapshot_stamp_ns',
        minimum=1,
    )
    if qualifying <= boundary:
        raise EvidenceError(
            'positive-control release snapshot is not strictly beyond its release boundary'
        )
    return qualifying


_COMPONENT_CONTACT_SNAPSHOT_FIELDS = {
    'classified_count',
    'collector_sequence',
    'counted_snapshot_records',
    'delivery_clock_offset_ns',
    'delivery_clock_stamp_ns',
    'exact_pair_count',
    'sim_stamp_ns',
    'snapshot_record_count',
}
_COMPONENT_CONTACT_RECORD_FIELDS = {
    'collector_sequence',
    'counterpart_collision',
    'counterpart_model',
    'disposition',
    'normalized_pair',
    'robot_collision',
    'sim_stamp_ns',
    'snapshot_sequence',
}


def _scoped_contact_name(value: Any, label: str) -> str:
    name = require_bounded_string(value, label)
    segments = name.split('::')
    if len(segments) < 3 or any(not segment for segment in segments):
        raise EvidenceError(f'{label} is not model::link::collision scoped')
    return name


def _normalized_pair(value: Any, label: str) -> tuple[str, str]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise EvidenceError(f'{label} is not a collision pair')
    pair = tuple(
        sorted(_scoped_contact_name(item, f'{label}[{index}]') for index, item in enumerate(value))
    )
    return pair


def _component_contact_projection(
    contact: Mapping[str, Any],
    *,
    expected_pair: tuple[str, str],
    manifest: Mapping[str, Any],
    qualifying_stamp_ns: int,
) -> list[tuple[int, tuple[tuple[str, str], ...]]]:
    summaries = contact.get('snapshots')
    records = contact.get('snapshot_records')
    if not isinstance(summaries, list) or not summaries:
        raise EvidenceError('positive-control authoritative contact snapshots are missing')
    if not isinstance(records, list) or not records:
        raise EvidenceError('positive-control authoritative contact records are missing')

    records_by_summary: dict[int, list[tuple[str, str]]] = {}
    record_stamps_by_summary: dict[int, set[int]] = {}
    previous_record_sequence = 0
    for index, raw_record in enumerate(records):
        record = _require_mapping(raw_record, f'positive_control.snapshot_records[{index}]')
        if set(record) != _COMPONENT_CONTACT_RECORD_FIELDS:
            raise EvidenceError('positive-control snapshot record shape changed')
        record_sequence = _require_int(
            record.get('collector_sequence'),
            f'positive_control.snapshot_records[{index}].collector_sequence',
            minimum=1,
        )
        if record_sequence <= previous_record_sequence:
            raise EvidenceError('positive-control snapshot record sequence is not strict')
        previous_record_sequence = record_sequence
        summary_sequence = _require_int(
            record.get('snapshot_sequence'),
            f'positive_control.snapshot_records[{index}].snapshot_sequence',
            minimum=1,
        )
        record_stamp = _require_int(
            record.get('sim_stamp_ns'),
            f'positive_control.snapshot_records[{index}].sim_stamp_ns',
            minimum=1,
        )
        pair = _normalized_pair(
            record.get('normalized_pair'),
            f'positive_control.snapshot_records[{index}].normalized_pair',
        )
        if list(pair) != record.get('normalized_pair'):
            raise EvidenceError('positive-control snapshot record pair is not normalized')
        records_by_summary.setdefault(summary_sequence, []).append(pair)
        record_stamps_by_summary.setdefault(summary_sequence, set()).add(record_stamp)

    projection: list[tuple[int, tuple[tuple[str, str], ...]]] = []
    seen_summaries: set[int] = set()
    previous_summary_sequence = 0
    previous_stamp: int | None = None
    for index, raw_summary in enumerate(summaries):
        summary = _require_mapping(raw_summary, f'positive_control.snapshots[{index}]')
        if set(summary) != _COMPONENT_CONTACT_SNAPSHOT_FIELDS:
            raise EvidenceError('positive-control snapshot summary shape changed')
        summary_sequence = _require_int(
            summary.get('collector_sequence'),
            f'positive_control.snapshots[{index}].collector_sequence',
            minimum=1,
        )
        stamp = _require_int(
            summary.get('sim_stamp_ns'),
            f'positive_control.snapshots[{index}].sim_stamp_ns',
            minimum=1,
        )
        delivery_clock = _require_int(
            summary.get('delivery_clock_stamp_ns'),
            f'positive_control.snapshots[{index}].delivery_clock_stamp_ns',
            minimum=0,
        )
        delivery_offset = _require_int(
            summary.get('delivery_clock_offset_ns'),
            f'positive_control.snapshots[{index}].delivery_clock_offset_ns',
        )
        if (
            summary_sequence <= previous_summary_sequence
            or (previous_stamp is not None and stamp <= previous_stamp)
            or (previous_stamp is not None and stamp - previous_stamp > CONTACT_MAX_CLOCK_LAG_NS)
            or delivery_clock - stamp != delivery_offset
            or abs(delivery_offset) > CONTACT_MAX_CLOCK_LAG_NS
        ):
            raise EvidenceError('positive-control snapshot ordering/liveness is invalid')
        snapshot_pairs = records_by_summary.get(summary_sequence, [])
        if not 1 <= len(snapshot_pairs) <= 16 or _require_int(
            summary.get('snapshot_record_count'),
            f'positive_control.snapshots[{index}].snapshot_record_count',
            minimum=1,
        ) != len(snapshot_pairs):
            raise EvidenceError('positive-control snapshot record count does not reconcile')
        if record_stamps_by_summary.get(summary_sequence) != {stamp}:
            raise EvidenceError('positive-control snapshot record stamp linkage changed')
        exact_count = sum(pair == expected_pair for pair in snapshot_pairs)
        if (
            _require_int(
                summary.get('exact_pair_count'),
                f'positive_control.snapshots[{index}].exact_pair_count',
                minimum=0,
            )
            != exact_count
        ):
            raise EvidenceError('positive-control snapshot exact-pair count does not reconcile')
        seen_summaries.add(summary_sequence)
        previous_summary_sequence = summary_sequence
        previous_stamp = stamp
        projection.append((stamp, tuple(sorted(snapshot_pairs))))

    if set(records_by_summary) != seen_summaries:
        raise EvidenceError('positive-control snapshot record references a missing summary')
    qualifying_indexes = [
        index for index, (stamp, _pairs) in enumerate(projection) if stamp == qualifying_stamp_ns
    ]
    if len(qualifying_indexes) != 1:
        raise EvidenceError(
            'positive-control qualifying release snapshot is not present exactly once'
        )
    qualifying_index = qualifying_indexes[0]
    suffix_start = qualifying_index + 1
    suffix = projection[suffix_start:]
    if _captured_contact_episodes(suffix, manifest=manifest):
        message = 'positive-control component observed countable recontact after release'
        raise EvidenceError(message)
    return projection[: qualifying_index + 1]


def _captured_contact_projection(
    captured_contacts: Sequence[Any],
    *,
    first_stamp_ns: int,
    qualifying_stamp_ns: int,
) -> list[tuple[int, tuple[tuple[str, str], ...]]]:
    projection: list[tuple[int, tuple[tuple[str, str], ...]]] = []
    previous_stamp: int | None = None
    for message_index, raw_message in enumerate(captured_contacts):
        message = _require_mapping(raw_message, f'captured contacts[{message_index}]')
        stamp = _require_int(
            message.get('stamp_ns'), f'captured contacts[{message_index}].stamp_ns', minimum=1
        )
        if previous_stamp is not None and stamp <= previous_stamp:
            raise EvidenceError('captured contact snapshot stamps are not strict')
        previous_stamp = stamp
        if message.get('frame_id') != '':
            raise EvidenceError('captured contact snapshot frame_id must be empty')
        delivery_clock = _require_int(
            message.get('delivery_clock_stamp_ns'),
            f'captured contacts[{message_index}].delivery_clock_stamp_ns',
            minimum=0,
        )
        delivery_offset = _require_int(
            message.get('delivery_clock_offset_ns'),
            f'captured contacts[{message_index}].delivery_clock_offset_ns',
        )
        if delivery_clock - stamp != delivery_offset:
            raise EvidenceError('captured contact snapshot delivery /clock offset is inconsistent')
        records = message.get('contacts')
        if not isinstance(records, list) or not 1 <= len(records) <= 16:
            raise EvidenceError('captured contact snapshot must contain 1 to 16 records')
        pairs = tuple(
            sorted(
                _normalized_pair(
                    (
                        _require_mapping(record, 'captured contact record').get('collision1'),
                        _require_mapping(record, 'captured contact record').get('collision2'),
                    ),
                    f'captured contacts[{message_index}].contacts[{record_index}]',
                )
                for record_index, record in enumerate(records)
            )
        )
        if first_stamp_ns <= stamp <= qualifying_stamp_ns:
            projection.append((stamp, pairs))
    return projection


def _captured_contact_episodes(
    projection: Sequence[tuple[int, tuple[tuple[str, str], ...]]],
    *,
    manifest: Mapping[str, Any],
) -> list[dict[str, Any]]:
    robot_model = require_bounded_string(manifest.get('robot_model'), 'coverage robot model')
    robot_collisions = {
        _scoped_contact_name(
            _require_mapping(entry, 'coverage robot collision').get('name'),
            'coverage robot collision name',
        )
        for entry in manifest.get('robot_collisions', [])
    }
    support_pairs = {
        _normalized_pair(
            (
                _require_mapping(entry, 'coverage support pair').get('robot_collision'),
                _require_mapping(entry, 'coverage support pair').get('environment_collision'),
            ),
            'coverage support pair',
        )
        for entry in manifest.get('support_pairs', [])
    }
    episodes: list[dict[str, Any]] = []
    active: dict[str, dict[str, Any]] = {}
    for stamp, pairs in projection:
        present: dict[str, list[tuple[str, str]]] = {}
        for pair in pairs:
            first_robot = pair[0] in robot_collisions
            second_robot = pair[1] in robot_collisions
            for name in pair:
                if name.split('::', 1)[0] == robot_model and name not in robot_collisions:
                    raise EvidenceError(
                        'captured contact references an unknown rendered robot collision'
                    )
            if first_robot and second_robot:
                continue
            if not first_robot and not second_robot:
                raise EvidenceError('captured contact snapshot contains a non-robot pair')
            if pair in support_pairs:
                continue
            counterpart = pair[1] if first_robot else pair[0]
            counterpart_model = counterpart.split('::', 1)[0]
            if counterpart_model == robot_model:
                raise EvidenceError('captured contact counterpart cannot be resolved')
            present.setdefault(counterpart_model, []).append(pair)
        for counterpart in list(active):
            if counterpart in present:
                continue
            episode = active.pop(counterpart)
            episode['end_stamp_ns'] = stamp
            episode['normalized_pairs'] = [
                list(pair) for pair in sorted(episode['normalized_pairs'])
            ]
            episodes.append(episode)
        for counterpart, counterpart_pairs in present.items():
            episode = active.get(counterpart)
            if episode is None:
                episode = {
                    'counterpart_model': counterpart,
                    'normalized_pairs': set(),
                    'snapshot_record_count': 0,
                    'start_stamp_ns': stamp,
                }
                active[counterpart] = episode
            episode['normalized_pairs'].update(counterpart_pairs)
            episode['snapshot_record_count'] += len(counterpart_pairs)
    if active:
        raise EvidenceError('positive-control countable contact lacks an absence snapshot')
    return sorted(
        episodes,
        key=lambda episode: (
            episode['counterpart_model'],
            episode['start_stamp_ns'],
            episode['end_stamp_ns'],
        ),
    )


def reconcile_positive_control(
    *,
    workspace: Path,
    build_binding: Mapping[str, Any],
    result_path: Path,
    capture_path: Path,
    contact_progress_path: Path,
    manifest_path: Path,
    collector_configuration_sha256: str,
    owned_process_group_shutdown: bool,
    checksum_verified: bool,
) -> dict[str, Any]:
    """Fail closed on the external positive-control proof and freeze its binding."""
    result_hash = verify_json_sidecar(result_path)
    result = _require_mapping(load_json(result_path), 'positive_control')
    capture = _require_mapping(load_json(capture_path), 'positive_control_capture')
    manifest = load_yaml(manifest_path)
    contact_stream = _contact_stream_manifest_v3(
        _require_mapping(manifest, 'coverage_manifest'), workspace
    )
    gate = _require_mapping(contact_stream.get('gate'), 'contact_stream.gate')
    frozen_gate_binary = _require_mapping(
        build_binding.get('contact_gate_binary'), 'build_binding.contact_gate_binary'
    )
    if gate.get('source_inventory_sha256') != frozen_gate_binary.get('source_inventory_sha256'):
        raise EvidenceError('contact stream source inventory differs from build binding')
    semantic_without_hash = dict(manifest)
    declared_manifest_hash = require_sha256(
        semantic_without_hash.pop('manifest_sha256', None), 'manifest_sha256'
    )
    if canonical_sha256(semantic_without_hash) != declared_manifest_hash:
        raise EvidenceError('collision coverage manifest self-hash mismatch')
    qualifying_contact_snapshot_stamp_ns = positive_control_qualified_snapshot_stamp(result)
    identity = _require_mapping(result.get('identity'), 'positive_control.identity')
    run_id = require_bounded_string(identity.get('run_id'), 'positive_control.run_id')
    scenario_hash = require_sha256(
        identity.get('scenario_sha256'), 'positive_control.scenario_sha256'
    )
    quality = _require_mapping(result.get('quality'), 'positive_control.quality')
    if quality.get('overflow_free') is not True:
        raise EvidenceError('positive-control component overflowed')
    capture_quality = _require_mapping(capture.get('quality'), 'positive_capture.quality')
    clock = _require_mapping(capture.get('clock'), 'positive_capture.clock')
    if (
        capture.get('stop_reason') != 'stop_file'
        or capture_quality.get('collector_overflow') is not False
        or clock.get('regression_count') != 0
    ):
        raise EvidenceError('positive-control metrics capture is incomplete')
    capture_hash = file_sha256(capture_path)
    provenance = _collision_provenance(manifest, collector_configuration_sha256)
    result_configuration = _require_mapping(
        result.get('configuration'), 'positive_control.configuration'
    )
    coverage_provenance = _require_mapping(
        result_configuration.get('coverage_manifest_provenance'),
        'positive_control.configuration.coverage_manifest_provenance',
    )
    for key in (
        'bridge_sha256',
        'contact_configuration_sha256',
        'rendered_sdf_sha256',
        'robot_description_sha256',
        'world_source_sha256',
    ):
        if coverage_provenance.get(key) != provenance[key]:
            raise EvidenceError(f'positive-control provenance mismatch for {key}')
    if result_configuration.get('coverage_manifest_sha256') != declared_manifest_hash:
        raise EvidenceError('positive-control coverage manifest binding mismatch')
    streams = _require_mapping(capture.get('streams'), 'positive_capture.streams')
    contact_stream = _require_mapping(streams.get('contacts'), 'positive_capture.contacts')
    command_stream = _require_mapping(streams.get('cmd_vel'), 'positive_capture.cmd_vel')
    captured_contacts = contact_stream.get('items')
    captured_commands = command_stream.get('items')
    if not isinstance(captured_contacts, list) or not isinstance(captured_commands, list):
        raise EvidenceError('positive-control capture streams are malformed')
    control = _require_mapping(result.get('control'), 'positive_control.control')
    contact = _require_mapping(control.get('contact'), 'positive_control.control.contact')
    expected_pair = contact.get('expected_pair')
    if (
        not isinstance(expected_pair, list)
        or len(expected_pair) != 2
        or any(not isinstance(value, str) or not value for value in expected_pair)
    ):
        raise EvidenceError('positive-control expected pair is malformed')
    normalized_expected = _normalized_pair(expected_pair, 'positive-control expected pair')
    component_projection = _component_contact_projection(
        contact,
        expected_pair=normalized_expected,
        manifest=manifest,
        qualifying_stamp_ns=qualifying_contact_snapshot_stamp_ns,
    )
    captured_projection = _captured_contact_projection(
        captured_contacts,
        first_stamp_ns=component_projection[0][0],
        qualifying_stamp_ns=qualifying_contact_snapshot_stamp_ns,
    )
    if captured_projection != component_projection:
        raise EvidenceError(
            'collector contact snapshot stamp/pair multiset is not bijective with the driver'
        )
    captured_exact_count = sum(
        pairs.count(normalized_expected) for _stamp, pairs in captured_projection
    )
    captured_exact_stamps = [
        stamp
        for stamp, pairs in captured_projection
        for _occurrence in range(pairs.count(normalized_expected))
    ]
    component_exact_count = _require_int(
        contact.get('exact_pair_snapshot_record_count'),
        'positive_control.exact_pair_snapshot_record_count',
        minimum=1,
    )
    first_contact = _require_mapping(
        contact.get('first_qualifying_contact'), 'positive_control.first_qualifying_contact'
    )
    first_contact_stamp = _require_int(
        first_contact.get('sim_stamp_ns'), 'positive_control.first_contact_stamp', minimum=1
    )
    if (
        captured_exact_count != component_exact_count
        or not captured_exact_stamps
        or first_contact_stamp != captured_exact_stamps[0]
    ):
        raise EvidenceError('collector contact evidence does not reconcile with the driver')
    if first_contact_stamp >= qualifying_contact_snapshot_stamp_ns:
        raise EvidenceError('positive-control first wall contact is not before release')
    _capture_contains_qualified_contact_snapshot(capture, qualifying_contact_snapshot_stamp_ns)
    contact_progress = validate_final_contact_progress(
        _require_mapping(load_json(contact_progress_path), 'positive_contact_progress'),
        capture=capture,
        minimum_retained_stamp_ns=qualifying_contact_snapshot_stamp_ns,
    )
    contact_progress_sha256 = file_sha256(contact_progress_path)
    release_messages = [
        _require_mapping(item, 'captured release snapshot')
        for item in captured_contacts
        if _require_mapping(item, 'captured contact message').get('stamp_ns')
        == qualifying_contact_snapshot_stamp_ns
    ]
    if len(release_messages) != 1:
        raise EvidenceError('collector did not retain exactly one release snapshot')
    release_records = release_messages[0].get('contacts')
    if not isinstance(release_records, list) or not 1 <= len(release_records) <= 16:
        raise EvidenceError('captured release snapshot must contain 1 to 16 records')
    release_delivery_clock_stamp_ns = _require_int(
        release_messages[0].get('delivery_clock_stamp_ns'),
        'captured release delivery clock stamp',
        minimum=0,
    )
    release_delivery_clock_offset_ns = _require_int(
        release_messages[0].get('delivery_clock_offset_ns'),
        'captured release delivery clock offset',
    )
    if (
        release_delivery_clock_offset_ns
        != release_delivery_clock_stamp_ns - qualifying_contact_snapshot_stamp_ns
    ):
        raise EvidenceError('positive-control release delivery /clock offset is inconsistent')
    robot_collisions = {
        require_bounded_string(
            _require_mapping(entry, 'coverage robot collision').get('name'),
            'coverage robot collision name',
        )
        for entry in manifest.get('robot_collisions', [])
    }
    support_pairs = {
        tuple(
            sorted(
                (
                    require_bounded_string(
                        _require_mapping(entry, 'coverage support pair').get('robot_collision'),
                        'coverage support robot collision',
                    ),
                    require_bounded_string(
                        _require_mapping(entry, 'coverage support pair').get(
                            'environment_collision'
                        ),
                        'coverage support environment collision',
                    ),
                )
            )
        )
        for entry in manifest.get('support_pairs', [])
    }
    release_expected_pair_count = 0
    for record_value in release_records:
        record = _require_mapping(record_value, 'captured release contact record')
        collision1 = record.get('collision1')
        collision2 = record.get('collision2')
        pair = tuple(
            sorted(
                (
                    require_bounded_string(collision1, 'captured release collision1'),
                    require_bounded_string(collision2, 'captured release collision2'),
                )
            )
        )
        if pair == normalized_expected:
            release_expected_pair_count += 1
            continue
        if pair not in support_pairs and not all(name in robot_collisions for name in pair):
            raise EvidenceError('captured release snapshot contains an unexpected contact pair')
    if release_expected_pair_count:
        raise EvidenceError('expected positive-control wall pair remains in release snapshot')
    captured_episodes = _captured_contact_episodes(
        captured_projection,
        manifest=manifest,
    )
    component_episode_values = contact.get('episodes')
    if not isinstance(component_episode_values, list):
        raise EvidenceError('positive-control component episodes are malformed')
    component_episodes: list[dict[str, Any]] = []
    for index, raw_episode in enumerate(component_episode_values):
        episode = _require_mapping(raw_episode, f'positive_control.episodes[{index}]')
        if set(episode) != {
            'counterpart_model',
            'end_stamp_ns',
            'normalized_pairs',
            'snapshot_record_count',
            'start_stamp_ns',
        }:
            raise EvidenceError('positive-control component episode shape changed')
        normalized_pairs_value = episode.get('normalized_pairs')
        if not isinstance(normalized_pairs_value, list) or not normalized_pairs_value:
            raise EvidenceError('positive-control component episode pairs are missing')
        normalized_pairs = sorted(
            _normalized_pair(
                pair,
                f'positive_control.episodes[{index}].normalized_pairs[{pair_index}]',
            )
            for pair_index, pair in enumerate(normalized_pairs_value)
        )
        normalized_pair_lists = [list(pair) for pair in normalized_pairs]
        if normalized_pair_lists != normalized_pairs_value or len(set(normalized_pairs)) != len(
            normalized_pairs
        ):
            raise EvidenceError('positive-control component episode pairs are not canonical')
        component_episodes.append(
            {
                'counterpart_model': require_bounded_string(
                    episode.get('counterpart_model'),
                    f'positive_control.episodes[{index}].counterpart_model',
                ),
                'end_stamp_ns': _require_int(
                    episode.get('end_stamp_ns'),
                    f'positive_control.episodes[{index}].end_stamp_ns',
                    minimum=1,
                ),
                'normalized_pairs': normalized_pair_lists,
                'snapshot_record_count': _require_int(
                    episode.get('snapshot_record_count'),
                    f'positive_control.episodes[{index}].snapshot_record_count',
                    minimum=1,
                ),
                'start_stamp_ns': _require_int(
                    episode.get('start_stamp_ns'),
                    f'positive_control.episodes[{index}].start_stamp_ns',
                    minimum=1,
                ),
            }
        )
    component_episodes.sort(
        key=lambda episode: (
            episode['counterpart_model'],
            episode['start_stamp_ns'],
            episode['end_stamp_ns'],
        )
    )
    if len(component_episodes) != 1 or component_episodes != captured_episodes:
        raise EvidenceError(
            'collector counterpart episodes do not exactly reconcile with the driver'
        )
    component_commands = control.get('command_trace')
    if not isinstance(component_commands, list) or not component_commands:
        raise EvidenceError('positive-control component command trace is missing')
    captured_command_projection = [
        (
            _require_int(item.get('stamp_ns'), 'captured command stamp', minimum=1),
            _require_number(item.get('linear_x_m_s'), 'captured command linear_x'),
            _require_number(item.get('angular_z_rad_s'), 'captured command angular_z'),
        )
        for item_value in captured_commands
        for item in [_require_mapping(item_value, 'captured command')]
    ]
    cursor = 0
    for command_value in component_commands:
        command = _require_mapping(command_value, 'component command')
        expected_stamp = _require_int(
            command.get('sim_stamp_ns'), 'component command stamp', minimum=1
        )
        expected_linear = _require_number(command.get('linear_x'), 'component command linear_x')
        expected_angular = _require_number(command.get('angular_z'), 'component command angular_z')
        latest_expected_stamp = expected_stamp + 100_000_000
        while cursor < len(captured_command_projection):
            captured_command = captured_command_projection[cursor]
            captured_stamp = captured_command[0]
            captured_linear = captured_command[1]
            captured_angular = captured_command[2]
            stamp_matches = expected_stamp <= captured_stamp <= latest_expected_stamp
            if (
                stamp_matches
                and captured_linear == expected_linear
                and captured_angular == expected_angular
            ):
                break
            cursor += 1
        if cursor >= len(captured_command_projection):
            raise EvidenceError('collector command stream is not a complete component subsequence')
        cursor += 1
    timeline = _require_mapping(control.get('timeline'), 'positive_control.control.timeline')
    release_boundary = _require_int(
        timeline.get('release_required_through_stamp_ns'),
        'positive_control.release_required_through_stamp_ns',
        minimum=1,
    )
    latest_clock = _require_int(
        clock.get('latest_stamp_ns'), 'positive_capture.clock.latest_stamp_ns', minimum=1
    )
    if latest_clock < qualifying_contact_snapshot_stamp_ns:
        raise EvidenceError('positive-control final /clock did not reach the release snapshot')
    if not owned_process_group_shutdown or not checksum_verified:
        raise EvidenceError('positive-control external process/checksum gate failed')
    contact_projection_document = [
        {
            'normalized_pairs': [list(pair) for pair in pairs],
            'stamp_ns': stamp,
        }
        for stamp, pairs in component_projection
    ]
    return {
        'benchmark_binding': {
            'benchmark_provenance': provenance,
            'positive_control_external_quality': {
                'checksum_verified': True,
                'collector_capture_sha256': capture_hash,
                'collector_reconciled': True,
                'owned_process_group_shutdown': True,
            },
            'positive_control_json_sha256': result_hash,
            'positive_control_provenance': provenance,
            'positive_control_run_id': run_id,
            'positive_control_scenario_sha256': scenario_hash,
        },
        'capture_sha256': capture_hash,
        'collector_reconciliation': {
            'captured_command_count': len(captured_command_projection),
            'captured_exact_pair_count': captured_exact_count,
            'captured_release_expected_pair_count': release_expected_pair_count,
            'captured_release_snapshot_count': len(release_messages),
            'contact_projection_episode_count': len(captured_episodes),
            'contact_projection_first_stamp_ns': component_projection[0][0],
            'contact_projection_record_count': sum(
                len(pairs) for _stamp, pairs in component_projection
            ),
            'contact_projection_sha256': canonical_sha256(contact_projection_document),
            'contact_projection_snapshot_count': len(component_projection),
            'contact_progress_artifact_sha256': contact_progress_sha256,
            'contact_progress_latest_retained_stamp_ns': contact_progress[
                'latest_retained_stamp_ns'
            ],
            'contact_progress_retained_message_count': contact_progress['retained_message_count'],
            'component_command_count': len(component_commands),
            'component_exact_pair_count': component_exact_count,
            'latest_clock_stamp_ns': latest_clock,
            'release_delivery_clock_offset_ns': release_delivery_clock_offset_ns,
            'release_delivery_clock_stamp_ns': release_delivery_clock_stamp_ns,
            'release_qualified_snapshot_stamp_ns': (qualifying_contact_snapshot_stamp_ns),
            'release_required_through_stamp_ns': release_boundary,
        },
        'coverage_manifest': dict(manifest),
        'positive_control': dict(result),
        'positive_control_json_sha256': result_hash,
        'producer': PRODUCER,
        'schema_version': SCHEMA_VERSION,
    }


def _fault_kind(scenario_id: int) -> str:
    return {4: 'lidar_dropout', 5: 'odometry_drift'}.get(scenario_id, 'none')


def make_trial_context(
    plan: Mapping[str, Any],
    *,
    workspace: Path,
    git_sha: str,
    build: Mapping[str, Any],
    positive: Mapping[str, Any],
    failure: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Create the immutable context metrics needs even for upstream failure."""
    if GIT_SHA_PATTERN.fullmatch(git_sha) is None:
        raise EvidenceError('trial context Git SHA is invalid')
    scenario_id = _require_int(plan.get('scenario_id'), 'scenario_id')
    identity = {
        'candidate_id': plan['candidate_id'],
        'cold_stack': True,
        'git_dirty': False,
        'git_sha': git_sha,
        'gz_partition': plan['gz_partition'],
        'repetition_index': plan['repetition_index'],
        'ros_domain_id': plan['ros_domain_id'],
        'run_id': plan['run_id'],
        'scenario_id': scenario_id,
        'scenario_index': scenario_id,
        'scenario_name': plan['scenario_name'],
        'scenario_sha256': plan['scenario_sha256'],
        'suite_index': plan['suite_index'],
    }
    acceptance, required_metrics = acceptance_for_scenario(scenario_id)
    scenario = load_yaml(workspace / plan['scenario_path'])
    positive_manifest = _require_mapping(
        positive.get('coverage_manifest'), 'positive.coverage_manifest'
    )
    targets = {
        'acceptance': acceptance,
        'collector_configuration_sha256': require_sha256(
            build.get('collector_configuration_sha256'),
            'build.collector_configuration_sha256',
        ),
        'collision_coverage_manifest_sha256': require_sha256(
            positive_manifest.get('manifest_sha256'),
            'positive.coverage_manifest.manifest_sha256',
        ),
        'fault_schedule_sha256': require_sha256(
            scenario.get('fault_schedule_sha256'), 'scenario.fault_schedule_sha256'
        ),
        'metrics_contract_sha256': require_sha256(
            build.get('metrics_contract_sha256'), 'build.metrics_contract_sha256'
        ),
        'positive_control_json_sha256': require_sha256(
            positive.get('positive_control_json_sha256'),
            'positive.positive_control_json_sha256',
        ),
        'scenario_sha256': require_sha256(plan.get('scenario_sha256'), 'scenario_sha256'),
        'source_configuration_sha256': require_sha256(
            build.get('source_configuration_sha256'),
            'build.source_configuration_sha256',
        ),
        'target_set_sha256': require_sha256(
            build.get('target_set_sha256'), 'build.target_set_sha256'
        ),
        'required_metrics': required_metrics,
        'world_to_map': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
    }
    normalized_failure = None
    if failure is not None:
        failure = _require_mapping(failure, 'failure')
        normalized_failure = {
            'evidence_sha256': require_sha256(
                failure.get('evidence_sha256'), 'failure.evidence_sha256'
            ),
            'exit_code': _require_int(failure.get('exit_code'), 'failure.exit_code'),
            'kind': require_bounded_string(failure.get('kind'), 'failure.kind'),
            'reason': require_bounded_string(failure.get('reason'), 'failure.reason'),
            'stage': require_bounded_string(failure.get('stage'), 'failure.stage'),
            'wall_timed_out': _require_bool(
                failure.get('wall_timed_out'), 'failure.wall_timed_out'
            ),
        }
        if normalized_failure['exit_code'] == 0:
            raise EvidenceError('failure.exit_code must be nonzero')
        for field in ('stage', 'kind'):
            if re.fullmatch(r'[a-z][a-z0-9_]{0,63}', normalized_failure[field]) is None:
                raise EvidenceError(f'failure.{field} is not a lowercase token')
    return {
        'failure': normalized_failure,
        'identity': identity,
        'intended_wall_timeout_s': TRIAL_WALL_TIMEOUT_S,
        'producer': PRODUCER,
        'schema_version': SCHEMA_VERSION,
        'targets': targets,
    }


def component_manifest(paths: Iterable[Path], base: Path) -> dict[str, Any]:
    """Hash a unique bounded component set in canonical relative-path order."""
    records: list[dict[str, Any]] = []
    seen: set[str] = set()
    resolved_base = base.resolve()
    for path in paths:
        resolved = path.resolve()
        try:
            relative = resolved.relative_to(resolved_base).as_posix()
        except ValueError as exc:
            raise EvidenceError(f'component artifact escapes run directory: {path}') from exc
        if relative in seen:
            raise EvidenceError(f'duplicate component artifact: {relative}')
        seen.add(relative)
        if not resolved.is_file():
            raise EvidenceError(f'missing component artifact: {resolved}')
        records.append(
            {
                'bytes': resolved.stat().st_size,
                'path': relative,
                'sha256': file_sha256(resolved),
            }
        )
    records.sort(key=lambda item: item['path'])
    if not 1 <= len(records) <= 256:
        raise EvidenceError('component manifest artifact count is outside [1, 256]')
    return {
        'aggregate_sha256': canonical_sha256(records),
        'artifact_count': len(records),
        'artifacts': records,
        'total_bytes': sum(item['bytes'] for item in records),
    }


def verify_component_manifest(document: Mapping[str, Any], base: Path) -> bool:
    """Re-read and verify every exact record in a component manifest."""
    records = document.get('artifacts')
    if not isinstance(records, list) or not 1 <= len(records) <= 256:
        raise EvidenceError('component manifest artifact count is outside [1, 256]')
    if document.get('artifact_count') != len(records):
        raise EvidenceError('component manifest artifact_count does not reconcile')
    resolved_base = base.resolve()
    observed: list[dict[str, Any]] = []
    seen: set[str] = set()
    for index, value in enumerate(records):
        record = _require_mapping(value, f'component manifest record {index}')
        relative = require_bounded_string(record.get('path'), f'component[{index}].path')
        relative_path = Path(relative)
        if (
            relative_path.is_absolute()
            or relative_path.as_posix() != relative
            or any(part in ('', '.', '..') for part in relative_path.parts)
            or relative in seen
        ):
            raise EvidenceError('component manifest path is non-canonical or duplicated')
        seen.add(relative)
        path = (base / relative).resolve()
        try:
            path.relative_to(resolved_base)
        except ValueError as exc:
            raise EvidenceError('component manifest path escapes its base') from exc
        size = _require_int(record.get('bytes'), f'component[{index}].bytes', minimum=0)
        digest = require_sha256(record.get('sha256'), f'component[{index}].sha256')
        if not path.is_file() or path.stat().st_size != size or file_sha256(path) != digest:
            raise EvidenceError(f'component manifest mismatch: {relative}')
        observed.append({'bytes': size, 'path': relative, 'sha256': digest})
    observed.sort(key=lambda item: item['path'])
    if canonical_sha256(observed) != document.get('aggregate_sha256'):
        raise EvidenceError('component manifest aggregate hash mismatch')
    if sum(item['bytes'] for item in observed) != document.get('total_bytes'):
        raise EvidenceError('component manifest total bytes do not reconcile')
    return True


def summarize_resources(path: Path) -> dict[str, Any]:
    """Summarize a bounded JSON-lines resource trace into contract fields."""
    if not path.is_file() or path.stat().st_size > LOG_MAX_BYTES:
        raise EvidenceError('resource trace is missing or exceeds 8 MiB')
    samples: list[Mapping[str, Any]] = []
    for line_number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
        try:
            sample = json.loads(line)
        except json.JSONDecodeError as exc:
            raise EvidenceError(f'invalid resource sample line {line_number}') from exc
        samples.append(_require_mapping(sample, f'resource sample {line_number}'))
        if len(samples) > 4096:
            raise EvidenceError('resource trace exceeds 4,096 samples')
    if len(samples) < 2:
        raise EvidenceError('resource trace requires at least two samples')
    cpu = [
        _require_number(
            item.get('cpu_percent'),
            'cpu_percent',
            minimum=0.0,
        )
        for item in samples
    ]
    rss = [_require_int(item.get('rss_sum_bytes'), 'rss_sum_bytes', minimum=0) for item in samples]
    memory = [
        _require_int(item.get('wsl_memory_bytes'), 'wsl_memory_bytes', minimum=0)
        for item in samples
    ]
    swap = [
        _require_int(item.get('wsl_swap_bytes'), 'wsl_swap_bytes', minimum=0) for item in samples
    ]
    ordered = sorted(cpu)
    p95_index = max(0, math.ceil(0.95 * len(ordered)) - 1)
    missing = sum(
        _require_int(item.get('missing_count', 0), 'missing_count', minimum=0) for item in samples
    )
    affinity_checked_pid_count = sum(
        _require_int(
            item.get('affinity_checked_pid_count'),
            'affinity_checked_pid_count',
            minimum=0,
        )
        for item in samples
    )
    affinity_escape_count = sum(
        _require_int(item.get('affinity_escape_count'), 'affinity_escape_count', minimum=0)
        for item in samples
    )
    affinity_unreadable_count = sum(
        _require_int(item.get('affinity_unreadable_count'), 'affinity_unreadable_count', minimum=0)
        for item in samples
    )
    affinity_union: set[int] = set()
    for item in samples:
        observed = item.get('affinity_observed_cpu_union')
        if not isinstance(observed, list) or len(observed) > 1024:
            raise EvidenceError('invalid affinity_observed_cpu_union')
        for cpu_id in observed:
            affinity_union.add(_require_int(cpu_id, 'affinity CPU', minimum=0))
        for field in ('affinity_escape_prefix', 'affinity_unreadable_pid_prefix'):
            prefix = item.get(field)
            if not isinstance(prefix, list) or len(prefix) > 64:
                raise EvidenceError(f'invalid bounded {field}')
    if affinity_checked_pid_count < 1:
        raise EvidenceError('resource trace contains no live child-affinity observation')
    if affinity_escape_count or affinity_unreadable_count:
        raise EvidenceError('resource trace contains an affinity escape or unreadable member')
    if not affinity_union.issubset(set(CPU_AFFINITY)):
        raise EvidenceError('resource trace CPU union escapes the frozen CPU set')
    return {
        'affinity_checked_pid_count': affinity_checked_pid_count,
        'affinity_escape_count': affinity_escape_count,
        'affinity_observed_cpu_union': sorted(affinity_union),
        'affinity_unreadable_count': affinity_unreadable_count,
        'cpu_percent_mean': sum(cpu) / len(cpu),
        'cpu_percent_p95': ordered[p95_index],
        'cpu_percent_peak': max(cpu),
        'missing_sample_count': missing,
        'oom_kill': any(item.get('oom_kill') is True for item in samples),
        'overflow_free': len(samples) <= 4096 and path.stat().st_size <= LOG_MAX_BYTES,
        'peak_rss_sum_bytes': max(rss),
        'pid_reuse_detected': any(item.get('pid_reuse_detected') is True for item in samples),
        'sample_count': len(samples),
        'sampler_started_before_launch': samples[0].get('phase') == 'before_launch',
        'sampler_stopped_after_shutdown': samples[-1].get('phase') == 'after_shutdown',
        'wsl_peak_memory_bytes': max(memory),
        'wsl_peak_swap_bytes': max(swap),
    }


def _artifact_sizes(
    run_dir: Path,
    *,
    component_manifest_sha256: str,
) -> dict[str, Any]:
    files = [item for item in run_dir.rglob('*') if item.is_file()]
    stdout = sum(item.stat().st_size for item in files if item.name.endswith('.stdout.log'))
    stderr = sum(item.stat().st_size for item in files if item.name.endswith('.stderr.log'))
    manifest = _require_mapping(
        load_json(run_dir / 'prerequisite-manifest.json'), 'prerequisite_manifest'
    )
    if file_sha256(run_dir / 'prerequisite-manifest.json') != require_sha256(
        component_manifest_sha256, 'prerequisite_manifest_sha256'
    ):
        raise EvidenceError('prerequisite manifest file hash changed before composition')
    verify_component_manifest(manifest, run_dir)
    records = manifest.get('artifacts')
    if not isinstance(records, list) or not records:
        raise EvidenceError('prerequisite manifest has no artifact records')
    verified_sizes: list[int] = []
    for index, record_value in enumerate(records):
        record = _require_mapping(record_value, f'prerequisite record {index}')
        relative = require_bounded_string(record.get('path'), f'prerequisite[{index}].path')
        path = (run_dir / relative).resolve()
        try:
            path.relative_to(run_dir.resolve())
        except ValueError as exc:
            raise EvidenceError('prerequisite manifest path escapes the run directory') from exc
        expected_bytes = _require_int(record.get('bytes'), 'prerequisite bytes', minimum=0)
        expected_sha = require_sha256(record.get('sha256'), 'prerequisite sha256')
        if (
            not path.is_file()
            or path.stat().st_size != expected_bytes
            or file_sha256(path) != (expected_sha)
        ):
            raise EvidenceError(f'prerequisite manifest record does not reconcile: {relative}')
        verified_sizes.append(expected_bytes)
    total = sum(verified_sizes)
    return {
        'prerequisite_artifact_count': len(records),
        'prerequisite_checksums_verified': True,
        'prerequisite_manifest_sha256': require_sha256(
            component_manifest_sha256, 'prerequisite_manifest_sha256'
        ),
        'prerequisite_maximum_file_bytes': max(verified_sizes),
        'prerequisite_total_bytes': total,
        'prerequisites_finalized': True,
        'prerequisites_within_caps': (
            1 <= len(records) <= 256
            and max(verified_sizes) <= JSON_MAX_BYTES
            and total <= RUN_DIRECTORY_MAX_BYTES
            and stdout <= LOG_MAX_BYTES
            and stderr <= LOG_MAX_BYTES
        ),
        'runtime_stderr_bytes': stderr,
        'runtime_stdout_bytes': stdout,
    }


def make_orchestrator_evidence(
    *,
    plan: Mapping[str, Any],
    build_start: Mapping[str, Any],
    build_end: Mapping[str, Any],
    git_sha: str,
    git_status_porcelain: str,
    resource_summary: Mapping[str, Any],
    execution: Mapping[str, Any],
    process: Mapping[str, Any],
    cleanup: Mapping[str, Any],
    gates: Mapping[str, Any],
    run_dir: Path,
    component_manifest_sha256: str,
) -> dict[str, Any]:
    """Compose the strict metrics orchestrator evidence object."""
    source_start = _require_mapping(build_start.get('source'), 'build_start.source')
    source_end = _require_mapping(build_end.get('source'), 'build_end.source')
    install_start = _require_mapping(build_start.get('install'), 'build_start.install')
    install_end = _require_mapping(build_end.get('install'), 'build_end.install')
    correspondence_start = _require_mapping(
        build_start.get('source_install'), 'build_start.source_install'
    )
    correspondence_end = _require_mapping(
        build_end.get('source_install'), 'build_end.source_install'
    )
    contact_gate_binary_start = _require_mapping(
        build_start.get('contact_gate_binary'), 'build_start.contact_gate_binary'
    )
    contact_gate_binary_end = _require_mapping(
        build_end.get('contact_gate_binary'), 'build_end.contact_gate_binary'
    )
    if dict(contact_gate_binary_start) != dict(contact_gate_binary_end):
        raise EvidenceError('contact gate build/install binding changed during the run')
    identity = {
        key: plan[key]
        for key in (
            'candidate_id',
            'gz_partition',
            'repetition_index',
            'ros_domain_id',
            'run_id',
            'scenario_id',
            'scenario_sha256',
            'suite_index',
        )
    }
    process_required = (
        'cold_stack',
        'fresh_fault_generation',
        'fresh_localization',
        'new_process_group',
        'partition_unused_before_start',
        'previous_trial_gone',
        'ros_domain_unused_before_start',
    )
    process_value = {field: _require_bool(process.get(field), field) for field in process_required}
    observed_affinity = resource_summary.get('affinity_observed_cpu_union')
    if (
        not isinstance(observed_affinity, list)
        or not observed_affinity
        or any(cpu not in CPU_AFFINITY for cpu in observed_affinity)
        or resource_summary.get('affinity_escape_count') != 0
        or resource_summary.get('affinity_unreadable_count') != 0
    ):
        raise EvidenceError('owned child-process affinity proof is absent or invalid')
    process_value['cpu_affinity'] = CPU_AFFINITY
    cleanup_required = (
        'all_owned_processes_exited',
        'discovery_endpoints_gone',
        'no_orphans',
    )
    gate_required = (
        'graph_contract_pass',
        'namespace_isolation_pass',
        'qos_contract_pass',
        'source_install_binding_pass',
        'validation_autonomy_isolation_pass',
    )
    return {
        'artifacts': _artifact_sizes(
            run_dir,
            component_manifest_sha256=component_manifest_sha256,
        ),
        'cleanup': {field: _require_bool(cleanup.get(field), field) for field in cleanup_required},
        'execution': {
            'command': require_bounded_string(execution.get('command'), 'execution.command'),
            'exit_code': _require_int(execution.get('exit_code'), 'execution.exit_code'),
            'wall_duration_s': _require_number(
                execution.get('wall_duration_s'), 'execution.wall_duration_s', minimum=0.0
            ),
            'wall_timed_out': _require_bool(
                execution.get('wall_timed_out'), 'execution.wall_timed_out'
            ),
            'wall_timeout_s': _require_number(
                execution.get('wall_timeout_s'), 'execution.wall_timeout_s', minimum=0.000001
            ),
            'working_directory': require_bounded_string(
                execution.get('working_directory'), 'execution.working_directory'
            ),
        },
        'gates': {field: _require_bool(gates.get(field), field) for field in gate_required},
        'git': {
            'dirty': bool(git_status_porcelain),
            'end_head': git_sha,
            'start_head': git_sha,
            'status_porcelain': git_status_porcelain,
        },
        'identity': identity,
        'process': process_value,
        'resources': {field: resource_summary[field] for field in RESOURCE_METRIC_FIELDS},
        'schema_version': SCHEMA_VERSION,
        'source_binding': {
            'collector_configuration_sha256': build_start['collector_configuration_sha256'],
            'contact_gate_binary': dict(contact_gate_binary_start),
            'install_end_sha256': install_end['aggregate_sha256'],
            'install_start_sha256': install_start['aggregate_sha256'],
            'install_unchanged': install_start['aggregate_sha256']
            == install_end['aggregate_sha256'],
            'metrics_contract_sha256': build_start['metrics_contract_sha256'],
            'source_configuration_sha256': build_start['source_configuration_sha256'],
            'source_end_sha256': source_end['aggregate_sha256'],
            'source_install_match': (
                correspondence_start.get('all_match') is True
                and correspondence_end.get('all_match') is True
                and correspondence_start.get('aggregate_sha256')
                == correspondence_end.get('aggregate_sha256')
            ),
            'source_start_sha256': source_start['aggregate_sha256'],
            'source_unchanged': source_start['aggregate_sha256'] == source_end['aggregate_sha256'],
            'target_set_sha256': build_start['target_set_sha256'],
        },
    }


CONTACT_DRAIN_FIELDS = {
    'clock_first_stamp_ns',
    'clock_latest_stamp_ns',
    'clock_message_count',
    'clock_minus_qualifying_contact_ns',
    'clock_regression_count',
    'contact_message_count',
    'contact_record_count_violation_count',
    'contact_stamp_duplicate_count',
    'contact_stamp_regression_count',
    'first_contact_stamp_ns',
    'gate_node_present',
    'latest_contact_stamp_ns',
    'limits',
    'maximum_contact_record_count',
    'maximum_contact_source_gap_ns',
    'minimum_contact_record_count',
    'minimum_same_pair_set_interval_ns',
    'producer',
    'public_publisher_nodes',
    'public_topic',
    'qualifying_contact_snapshot_stamp_ns',
    'same_pair_set_interval_violation_count',
    'schema_version',
    'target_stamp_ns',
    'terminal_action_stamp_ns',
}
CONTACT_PROGRESS_FIELDS = {
    'latest_retained_stamp_ns',
    'producer',
    'public_topic',
    'retained_message_count',
    'schema_version',
}

CONTACT_GATE_ATTESTATION_FIELDS = {
    'build_embedded_source_inventory_match',
    'build_embedded_source_inventory_sha256',
    'build_elf_build_id',
    'build_install_build_id_match',
    'build_install_sha256_match',
    'build_path',
    'build_sha256',
    'exact_live_process_count',
    'installed_declared_path',
    'installed_declared_samefile',
    'installed_device',
    'installed_embedded_source_inventory_match',
    'installed_embedded_source_inventory_sha256',
    'installed_elf_build_id',
    'installed_inode',
    'installed_path',
    'installed_regular_executable',
    'installed_sha256',
    'identity_revalidated_after_hashing',
    'launch_root_pid',
    'live_cmdline_sha256',
    'live_device',
    'live_embedded_source_inventory_match',
    'live_embedded_source_inventory_sha256',
    'live_elf_build_id',
    'live_executable_link',
    'live_executable_path',
    'live_executable_sha256',
    'live_inode',
    'live_installed_build_id_match',
    'live_installed_inode_match',
    'live_installed_sha256_match',
    'live_pgid',
    'live_pid',
    'live_ppid',
    'live_sid',
    'live_size_bytes',
    'live_start_ticks',
    'observed_gz_partition',
    'observed_ros_domain_id',
    'package',
    'process_identity_match',
    'schema_version',
    'source_inventory_sha256',
    'verdict',
}


def reconcile_contact_gate_reobservation(
    initial_gate_path: Path,
    final_gate_path: Path,
    *,
    build_binding: Mapping[str, Any],
    expected_domain_id: int,
    expected_gz_partition: str,
) -> dict[str, Any]:
    """Require one unchanged installed contact-gate process at ready and drain."""
    initial_sha256 = verify_json_sidecar(initial_gate_path)
    final_sha256 = verify_json_sidecar(final_gate_path)
    frozen_binary = _require_mapping(
        build_binding.get('contact_gate_binary'), 'build_binding.contact_gate_binary'
    )
    observations: list[Mapping[str, Any]] = []
    for label, path in (('initial', initial_gate_path), ('final', final_gate_path)):
        document = _require_mapping(load_json(path), f'{label}_runtime_gate')
        if document.get('verdict') != 'PASS':
            raise EvidenceError(f'{label} runtime gate did not PASS')
        attestation = _require_mapping(
            document.get('contact_gate_binary_attestation'),
            f'{label}_runtime_gate.contact_gate_binary_attestation',
        )
        if set(attestation) != CONTACT_GATE_ATTESTATION_FIELDS:
            raise EvidenceError(f'{label} contact gate attestation fields are invalid')
        if (
            attestation.get('schema_version') != 1
            or attestation.get('package') != 'robotest_sim'
            or attestation.get('verdict') != 'PASS'
            or attestation.get('exact_live_process_count') != 1
            or attestation.get('observed_ros_domain_id') != str(expected_domain_id)
            or attestation.get('observed_gz_partition') != expected_gz_partition
            or any(
                attestation.get(field) is not True
                for field in (
                    'build_embedded_source_inventory_match',
                    'build_install_build_id_match',
                    'build_install_sha256_match',
                    'installed_declared_samefile',
                    'installed_embedded_source_inventory_match',
                    'installed_regular_executable',
                    'identity_revalidated_after_hashing',
                    'live_installed_build_id_match',
                    'live_embedded_source_inventory_match',
                    'live_installed_inode_match',
                    'live_installed_sha256_match',
                    'process_identity_match',
                )
            )
        ):
            raise EvidenceError(f'{label} contact gate binary attestation did not PASS')
        for field in (
            'launch_root_pid',
            'live_pid',
            'live_ppid',
            'live_pgid',
            'live_sid',
            'live_start_ticks',
            'live_inode',
            'installed_inode',
            'live_size_bytes',
        ):
            _require_int(attestation.get(field), f'{label}.{field}', minimum=1)
        for field in ('live_device', 'installed_device'):
            _require_int(attestation.get(field), f'{label}.{field}', minimum=0)
        if not (
            attestation.get('live_pgid') == attestation.get('launch_root_pid')
            and attestation.get('live_sid') == attestation.get('launch_root_pid')
            and attestation.get('live_device') == attestation.get('installed_device')
            and attestation.get('live_inode') == attestation.get('installed_inode')
            and attestation.get('live_executable_sha256') == attestation.get('installed_sha256')
            and attestation.get('live_elf_build_id') == attestation.get('installed_elf_build_id')
            and attestation.get('live_executable_link') == attestation.get('live_executable_path')
        ):
            raise EvidenceError(f'{label} contact gate process identity fields conflict')
        for field in (
            'build_embedded_source_inventory_sha256',
            'build_sha256',
            'installed_embedded_source_inventory_sha256',
            'installed_sha256',
            'live_embedded_source_inventory_sha256',
            'live_executable_sha256',
            'source_inventory_sha256',
        ):
            require_sha256(attestation.get(field), f'{label}.{field}')
        if (
            attestation.get('build_sha256') != frozen_binary.get('build_sha256')
            or attestation.get('installed_sha256') != frozen_binary.get('installed_sha256')
            or attestation.get('build_elf_build_id') != frozen_binary.get('build_elf_build_id')
            or attestation.get('installed_elf_build_id')
            != frozen_binary.get('installed_elf_build_id')
            or attestation.get('source_inventory_sha256')
            != frozen_binary.get('source_inventory_sha256')
            or attestation.get('build_embedded_source_inventory_sha256')
            != frozen_binary.get('build_embedded_source_inventory_sha256')
            or attestation.get('installed_embedded_source_inventory_sha256')
            != frozen_binary.get('installed_embedded_source_inventory_sha256')
            or attestation.get('live_embedded_source_inventory_sha256')
            != frozen_binary.get('installed_embedded_source_inventory_sha256')
            or attestation.get('build_path') != frozen_binary.get('build_path')
            or attestation.get('installed_declared_path')
            != frozen_binary.get('installed_declared_path')
            or attestation.get('installed_path') != frozen_binary.get('installed_path')
            or frozen_binary.get('build_regular_executable') is not True
            or frozen_binary.get('installed_regular_executable') is not True
            or frozen_binary.get('build_install_build_id_match') is not True
            or frozen_binary.get('build_install_sha256_match') is not True
            or frozen_binary.get('build_embedded_source_inventory_match') is not True
            or frozen_binary.get('installed_embedded_source_inventory_match') is not True
            or frozen_binary.get('installed_declared_samefile') is not True
        ):
            raise EvidenceError(
                f'{label} contact gate binary differs from the prelaunch build binding'
            )
        observations.append(attestation)
    stable_fields = (
        'build_elf_build_id',
        'build_embedded_source_inventory_sha256',
        'build_path',
        'build_sha256',
        'installed_device',
        'installed_elf_build_id',
        'installed_embedded_source_inventory_sha256',
        'installed_inode',
        'installed_path',
        'installed_sha256',
        'launch_root_pid',
        'live_cmdline_sha256',
        'live_device',
        'live_elf_build_id',
        'live_embedded_source_inventory_sha256',
        'live_executable_link',
        'live_executable_path',
        'live_executable_sha256',
        'live_inode',
        'live_pgid',
        'live_pid',
        'live_ppid',
        'live_sid',
        'live_size_bytes',
        'live_start_ticks',
        'observed_gz_partition',
        'observed_ros_domain_id',
        'source_inventory_sha256',
    )
    initial, final = observations
    if any(initial.get(field) != final.get(field) for field in stable_fields):
        raise EvidenceError('contact gate process/binary identity changed before final drain')
    return {
        'final_gate_artifact_sha256': final_sha256,
        'frozen_contact_gate_binary_sha256': canonical_sha256(frozen_binary),
        'initial_gate_artifact_sha256': initial_sha256,
        'live_executable_sha256': initial['live_executable_sha256'],
        'live_inode': initial['live_inode'],
        'live_pid': initial['live_pid'],
        'live_start_ticks': initial['live_start_ticks'],
        'producer': PRODUCER,
        'schema_version': 1,
        'stable_identity': True,
    }


def validate_contact_progress(
    evidence: Mapping[str, Any], *, minimum_retained_stamp_ns: int
) -> dict[str, Any]:
    """Validate the collector's atomic retained-contact progress marker."""
    if set(evidence) != CONTACT_PROGRESS_FIELDS:
        raise EvidenceError('contact progress fields do not match schema version 1')
    if evidence.get('schema_version') != 1 or evidence.get('producer') != (
        'robotest_metrics/metrics_collector'
    ):
        raise EvidenceError('contact progress producer/schema is invalid')
    if evidence.get('public_topic') != CONTACT_PUBLIC_TOPIC:
        raise EvidenceError('contact progress public topic differs from the frozen contract')
    latest = _require_int(
        evidence.get('latest_retained_stamp_ns'),
        'contact_progress.latest_retained_stamp_ns',
        minimum=1,
    )
    if latest < minimum_retained_stamp_ns:
        raise EvidenceError('collector has not retained the qualifying contact snapshot yet')
    if (
        _require_int(
            evidence.get('retained_message_count'),
            'contact_progress.retained_message_count',
            minimum=1,
        )
        < 1
    ):
        raise EvidenceError('collector contact progress count is empty')
    return dict(evidence)


def validate_final_contact_progress(
    evidence: Mapping[str, Any],
    *,
    capture: Mapping[str, Any],
    minimum_retained_stamp_ns: int,
) -> dict[str, Any]:
    """Bind the collector's final ACK exactly to its retained contact capture."""
    validated = validate_contact_progress(
        evidence, minimum_retained_stamp_ns=minimum_retained_stamp_ns
    )
    streams = _require_mapping(capture.get('streams'), 'capture.streams')
    contacts = _require_mapping(streams.get('contacts'), 'capture.streams.contacts')
    items = contacts.get('items')
    if not isinstance(items, list) or not items:
        raise EvidenceError('capture contact stream is empty')
    final_item = _require_mapping(items[-1], 'capture final contact snapshot')
    final_stamp_ns = _require_int(
        final_item.get('stamp_ns'), 'capture final contact snapshot stamp', minimum=1
    )
    if validated['latest_retained_stamp_ns'] != final_stamp_ns:
        raise EvidenceError('contact progress latest stamp differs from the final capture')
    if validated['retained_message_count'] != len(items):
        raise EvidenceError('contact progress count differs from the retained capture length')
    return validated


def _capture_contains_qualified_contact_snapshot(
    capture: Mapping[str, Any], qualifying_stamp_ns: int
) -> None:
    """Fail closed unless the collector retained the exact authoritative snapshot."""
    streams = _require_mapping(capture.get('streams'), 'capture.streams')
    contact_stream = _require_mapping(streams.get('contacts'), 'capture.streams.contacts')
    items = contact_stream.get('items')
    if not isinstance(items, list) or not items:
        raise EvidenceError('capture contact stream is empty')
    previous_stamp: int | None = None
    previous_pair_set: frozenset[tuple[str, str]] | None = None
    qualifying_count = 0
    for index, raw_item in enumerate(items):
        item = _require_mapping(raw_item, f'capture.contacts.items[{index}]')
        stamp = _require_int(item.get('stamp_ns'), f'capture.contacts.items[{index}].stamp_ns')
        if stamp <= 0:
            raise EvidenceError('capture contact snapshot stamp must be positive')
        delivery_clock_stamp_ns = _require_int(
            item.get('delivery_clock_stamp_ns'),
            f'capture.contacts.items[{index}].delivery_clock_stamp_ns',
            minimum=0,
        )
        delivery_clock_offset_ns = _require_int(
            item.get('delivery_clock_offset_ns'),
            f'capture.contacts.items[{index}].delivery_clock_offset_ns',
        )
        if delivery_clock_offset_ns != delivery_clock_stamp_ns - stamp:
            raise EvidenceError('capture contact snapshot delivery /clock offset is inconsistent')
        if item.get('frame_id') != '':
            raise EvidenceError('capture public contact snapshot frame_id must be empty')
        contacts = item.get('contacts')
        if not isinstance(contacts, list) or not 1 <= len(contacts) <= 16:
            raise EvidenceError('capture contact snapshot must contain 1 to 16 records')
        pairs: set[tuple[str, str]] = set()
        for contact_index, raw_contact in enumerate(contacts):
            contact = _require_mapping(
                raw_contact,
                f'capture.contacts.items[{index}].contacts[{contact_index}]',
            )
            first = require_bounded_string(
                contact.get('collision1'),
                f'capture.contacts.items[{index}].contacts[{contact_index}].collision1',
            )
            second = require_bounded_string(
                contact.get('collision2'),
                f'capture.contacts.items[{index}].contacts[{contact_index}].collision2',
            )
            if any(
                len(name.split('::')) < 3 or any(not segment for segment in name.split('::'))
                for name in (first, second)
            ):
                raise EvidenceError('capture contact snapshot contains a malformed scoped name')
            pairs.add(tuple(sorted((first, second))))
        pair_set = frozenset(pairs)
        if previous_stamp is not None:
            delta_ns = stamp - previous_stamp
            if delta_ns <= 0:
                raise EvidenceError('capture contact snapshot stamps are not strictly increasing')
            if delta_ns > CONTACT_MAX_PUBLIC_GAP_NS:
                raise EvidenceError('capture contact snapshot source gap exceeded 220 ms')
            if pair_set == previous_pair_set and delta_ns < CONTACT_HEARTBEAT_NS:
                raise EvidenceError(
                    'capture unchanged contact pair set repeated before the 200 ms heartbeat'
                )
        if stamp == qualifying_stamp_ns:
            qualifying_count += 1
        previous_stamp = stamp
        previous_pair_set = pair_set
    if qualifying_count != 1:
        raise EvidenceError(
            'collector did not retain exactly one qualifying terminal contact snapshot'
        )


def validate_contact_drain_evidence(
    evidence: Mapping[str, Any],
    *,
    terminal_action_stamp_ns: int,
    capture: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate the sidecar-protected drain bracket and collector retention."""
    if set(evidence) != CONTACT_DRAIN_FIELDS:
        raise EvidenceError('contact drain evidence fields do not match schema version 3')
    if evidence.get('schema_version') != 3 or evidence.get('producer') != (
        'robotest_phase3/contact_drain_observer'
    ):
        raise EvidenceError('contact drain producer/schema is invalid')
    if evidence.get('public_topic') != CONTACT_PUBLIC_TOPIC:
        raise EvidenceError('contact drain public topic differs from the frozen contract')
    if evidence.get('gate_node_present') is not True or evidence.get('public_publisher_nodes') != [
        CONTACT_GATE_NODE
    ]:
        raise EvidenceError('contact stream gate survival evidence is invalid')
    terminal = _require_int(
        evidence.get('terminal_action_stamp_ns'),
        'contact_drain.terminal_action_stamp_ns',
        minimum=1,
    )
    if terminal != terminal_action_stamp_ns:
        raise EvidenceError('contact drain terminal stamp does not match the mission')
    target = _require_int(evidence.get('target_stamp_ns'), 'contact_drain.target_stamp_ns')
    if target != terminal + CONTACT_DRAIN_NS:
        raise EvidenceError('contact drain target is not terminal + 250 ms')
    qualifying = _require_int(
        evidence.get('qualifying_contact_snapshot_stamp_ns'),
        'contact_drain.qualifying_contact_snapshot_stamp_ns',
        minimum=1,
    )
    if qualifying <= target:
        raise EvidenceError('qualifying contact snapshot is not strictly beyond the drain target')
    first_contact = _require_int(
        evidence.get('first_contact_stamp_ns'), 'contact_drain.first_contact_stamp_ns', minimum=1
    )
    latest_contact = _require_int(
        evidence.get('latest_contact_stamp_ns'),
        'contact_drain.latest_contact_stamp_ns',
        minimum=1,
    )
    if not first_contact <= qualifying <= latest_contact:
        raise EvidenceError('qualifying contact snapshot is outside the observed contact span')
    if (
        _require_int(
            evidence.get('contact_message_count'),
            'contact_drain.contact_message_count',
        )
        < 1
    ):
        raise EvidenceError('contact drain observed no public snapshots')
    minimum_record_count = _require_int(
        evidence.get('minimum_contact_record_count'),
        'contact_drain.minimum_contact_record_count',
        minimum=1,
    )
    maximum_record_count = _require_int(
        evidence.get('maximum_contact_record_count'),
        'contact_drain.maximum_contact_record_count',
        minimum=1,
    )
    if maximum_record_count > 16 or minimum_record_count > maximum_record_count:
        raise EvidenceError('contact drain record-count bounds are invalid')
    if (
        _require_int(
            evidence.get('contact_record_count_violation_count'),
            'contact_drain.contact_record_count_violation_count',
        )
        != 0
    ):
        raise EvidenceError('contact drain observed a snapshot outside the 1 to 16 record bound')
    if (
        _require_int(
            evidence.get('contact_stamp_regression_count'),
            'contact_drain.contact_stamp_regression_count',
        )
        != 0
        or _require_int(
            evidence.get('contact_stamp_duplicate_count'),
            'contact_drain.contact_stamp_duplicate_count',
        )
        != 0
    ):
        raise EvidenceError('contact drain public snapshot stamps were not strictly increasing')
    maximum_gap = _require_int(
        evidence.get('maximum_contact_source_gap_ns'),
        'contact_drain.maximum_contact_source_gap_ns',
        minimum=0,
    )
    if maximum_gap > CONTACT_MAX_PUBLIC_GAP_NS:
        raise EvidenceError('contact drain public source gap exceeded 220 ms')
    minimum_same_pair_interval = evidence.get('minimum_same_pair_set_interval_ns')
    if (
        minimum_same_pair_interval is not None
        and _require_int(
            minimum_same_pair_interval,
            'contact_drain.minimum_same_pair_set_interval_ns',
            minimum=CONTACT_HEARTBEAT_NS,
        )
        < CONTACT_HEARTBEAT_NS
    ):
        raise EvidenceError('contact drain heartbeat interval is below 200 ms')
    if (
        _require_int(
            evidence.get('same_pair_set_interval_violation_count'),
            'contact_drain.same_pair_set_interval_violation_count',
        )
        != 0
    ):
        raise EvidenceError('contact drain observed an early unchanged-pair heartbeat')
    limits = _require_mapping(evidence.get('limits'), 'contact_drain.limits')
    if dict(limits) != {
        'heartbeat_period_ns': CONTACT_HEARTBEAT_NS,
        'max_clock_lag_ns': CONTACT_MAX_CLOCK_LAG_NS,
        'max_public_gap_ns': CONTACT_MAX_PUBLIC_GAP_NS,
        'release_gap_ns': CONTACT_DRAIN_NS,
    }:
        raise EvidenceError('contact drain limits differ from the frozen contract')
    latest_clock = _require_int(
        evidence.get('clock_latest_stamp_ns'), 'contact_drain.clock_latest_stamp_ns'
    )
    _require_int(evidence.get('clock_first_stamp_ns'), 'contact_drain.clock_first_stamp_ns')
    if _require_int(evidence.get('clock_message_count'), 'contact_drain.clock_message_count') < 1:
        raise EvidenceError('contact drain observed no clock samples')
    clock_regression_count = _require_int(
        evidence.get('clock_regression_count'),
        'contact_drain.clock_regression_count',
    )
    if clock_regression_count != 0:
        raise EvidenceError('contact drain clock regressed')
    lag = _require_int(
        evidence.get('clock_minus_qualifying_contact_ns'),
        'contact_drain.clock_minus_qualifying_contact_ns',
    )
    if (
        latest_clock < qualifying
        or lag != latest_clock - qualifying
        or not (0 <= lag <= CONTACT_MAX_CLOCK_LAG_NS)
    ):
        raise EvidenceError('contact drain clock bracket is invalid')
    _capture_contains_qualified_contact_snapshot(capture, qualifying)
    return dict(evidence)


def compose_analysis_request(
    *,
    workspace: Path,
    plan: Mapping[str, Any],
    mission_path: Path,
    scenario_path: Path,
    capture_path: Path,
    positive_binding_path: Path,
    orchestrator_path: Path,
    contact_drain_path: Path,
    contact_progress_path: Path,
    lifecycle_snapshot_path: Path | None,
) -> dict[str, Any]:
    """Compose one exact Phase 3 metrics-analysis request."""
    mission = _require_mapping(load_json(mission_path), 'mission_result')
    scenario = _require_mapping(load_json(scenario_path), 'scenario_result')
    capture = _require_mapping(load_json(capture_path), 'capture')
    positive = _require_mapping(load_json(positive_binding_path), 'positive_binding')
    orchestrator = _require_mapping(load_json(orchestrator_path), 'orchestrator')
    build = _require_mapping(orchestrator.get('source_binding'), 'orchestrator.source_binding')
    manifest = _require_mapping(positive.get('coverage_manifest'), 'coverage_manifest')
    contact_stream = _contact_stream_manifest_v3(manifest, workspace)
    benchmark_binding = _require_mapping(positive.get('benchmark_binding'), 'benchmark_binding')
    positive_result = _require_mapping(positive.get('positive_control'), 'positive_control')
    scenario_id = _require_int(plan.get('scenario_id'), 'scenario_id')
    mission_measurements = _require_mapping(
        mission.get('measurements'), 'mission_result.measurements'
    )
    terminal_action_stamp_ns = _require_int(
        mission_measurements.get('terminal_action_stamp_ns'),
        'mission_result.measurements.terminal_action_stamp_ns',
        minimum=1,
    )
    contact_gate_binary = _require_mapping(
        build.get('contact_gate_binary'), 'orchestrator.source_binding.contact_gate_binary'
    )
    contact_gate = _require_mapping(contact_stream.get('gate'), 'contact_stream.gate')
    if contact_gate.get('source_inventory_sha256') != contact_gate_binary.get(
        'source_inventory_sha256'
    ):
        raise EvidenceError('contact stream source inventory differs from trial build binding')
    verify_json_sidecar(contact_drain_path)
    contact_drain = validate_contact_drain_evidence(
        _require_mapping(load_json(contact_drain_path), 'contact_drain'),
        terminal_action_stamp_ns=terminal_action_stamp_ns,
        capture=capture,
    )
    contact_progress = validate_final_contact_progress(
        _require_mapping(load_json(contact_progress_path), 'contact_progress'),
        capture=capture,
        minimum_retained_stamp_ns=contact_drain['qualifying_contact_snapshot_stamp_ns'],
    )
    contact_drain_ack = {
        **contact_progress,
        'artifact_sha256': file_sha256(contact_progress_path),
    }
    topics = _require_mapping(contact_stream.get('topics'), 'contact_stream.topics')
    if contact_drain['public_topic'] != topics.get('public_ros'):
        raise EvidenceError('contact drain topic does not match the coverage manifest')
    identity = {
        'candidate_id': plan['candidate_id'],
        'cold_stack': True,
        'git_dirty': False,
        'git_sha': orchestrator['git']['start_head'],
        'gz_partition': plan['gz_partition'],
        'repetition_index': plan['repetition_index'],
        'ros_domain_id': plan['ros_domain_id'],
        'run_id': plan['run_id'],
        'scenario_id': scenario_id,
        'scenario_index': scenario_id,
        'scenario_name': plan['scenario_name'],
        'scenario_sha256': plan['scenario_sha256'],
        'suite_index': plan['suite_index'],
    }
    acceptance, required_metrics = acceptance_for_scenario(scenario_id)
    scenario_document = load_yaml(workspace / plan['scenario_path'])
    fault: dict[str, Any] = {'kind': _fault_kind(scenario_id)}
    if scenario_id == 4:
        if lifecycle_snapshot_path is None:
            raise EvidenceError('Scenario 4 requires a lifecycle snapshot artifact')
        fault.update(
            {
                'collision_monitor_source_timeout_s': 0.60,
                'lifecycle_snapshot': load_json(lifecycle_snapshot_path),
                'required_lifecycle_nodes': REQUIRED_LIFECYCLE_NODES,
            }
        )
    request = {
        'capture': dict(capture),
        'collision': {
            'benchmark_binding': dict(benchmark_binding),
            'coverage_manifest': dict(manifest),
            'contact_drain': contact_drain,
            'contact_drain_ack': contact_drain_ack,
            'drain_completed_stamp_ns': contact_drain['qualifying_contact_snapshot_stamp_ns'],
            'positive_control': dict(positive_result),
        },
        'fault': fault,
        'identity': identity,
        'mission': {
            'artifact_sha256': canonical_sha256(mission),
            'result': dict(mission),
        },
        'orchestrator': dict(orchestrator),
        'scenario': {
            'artifact_sha256': canonical_sha256(scenario),
            'result': dict(scenario),
        },
        'targets': {
            'acceptance': acceptance,
            'collector_configuration_sha256': build['collector_configuration_sha256'],
            'collision_coverage_manifest_sha256': manifest['manifest_sha256'],
            'fault_schedule_sha256': scenario_document['fault_schedule_sha256'],
            'metrics_contract_sha256': build['metrics_contract_sha256'],
            'positive_control_json_sha256': positive['positive_control_json_sha256'],
            'required_metrics': required_metrics,
            'scenario_sha256': plan['scenario_sha256'],
            'source_configuration_sha256': build['source_configuration_sha256'],
            'target_set_sha256': build['target_set_sha256'],
            'world_to_map': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        },
    }
    return request


def failure_evidence(
    *,
    stage: str,
    kind: str,
    exit_code: int,
    wall_timed_out: bool,
    reason: str,
    evidence: Any,
) -> dict[str, Any]:
    """Normalize a bounded observed upstream failure for metrics composition."""
    return {
        'evidence_sha256': canonical_sha256(evidence),
        'exit_code': int(exit_code),
        'kind': require_bounded_string(kind, 'failure.kind'),
        'reason': require_bounded_string(reason, 'failure.reason'),
        'stage': require_bounded_string(stage, 'failure.stage'),
        'wall_timed_out': bool(wall_timed_out),
    }


def reconcile_goal_binding(
    observer: Mapping[str, Any],
    mission: Mapping[str, Any],
    scenario: Mapping[str, Any],
) -> dict[str, Any]:
    """Require exact UUID/T0 equality across observer, mission, and controller."""
    measurements = _require_mapping(mission.get('measurements'), 'mission.measurements')
    binding = _require_mapping(scenario.get('binding'), 'scenario.binding')
    observed_uuid = require_bounded_string(
        observer.get('accepted_goal_uuid'), 'observer.accepted_goal_uuid'
    )
    observed_t0 = _require_int(
        observer.get('accepted_goal_stamp_ns'), 'observer.accepted_goal_stamp_ns', minimum=1
    )
    values = {
        'mission_goal_uuid': measurements.get('accepted_goal_uuid'),
        'mission_t0_ns': measurements.get('accepted_goal_stamp_ns'),
        'observer_goal_uuid': observed_uuid,
        'observer_t0_ns': observed_t0,
        'scenario_goal_uuid': binding.get('goal_uuid'),
        'scenario_t0_ns': binding.get('accepted_goal_stamp_ns'),
    }
    if not (
        values['mission_goal_uuid'] == observed_uuid == values['scenario_goal_uuid']
        and values['mission_t0_ns'] == observed_t0 == values['scenario_t0_ns']
    ):
        raise EvidenceError('goal UUID/T0 evidence differs across runtime components')
    return {**values, 'immutable_exact_match': True}


def drain_bounded_log(source: Any, output: Path, maximum_bytes: int) -> dict[str, Any]:
    """Drain an entire byte stream while retaining only its bounded prefix."""
    if maximum_bytes <= 0:
        raise EvidenceError('log maximum_bytes must be positive')
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{output.name}.', dir=output.parent)
    temporary = Path(temporary_name)
    retained = 0
    observed = 0
    try:
        with os.fdopen(descriptor, 'wb') as target:
            while block := source.read(64 * 1024):
                observed += len(block)
                remaining = maximum_bytes - retained
                if remaining > 0:
                    prefix = block[:remaining]
                    target.write(prefix)
                    retained += len(prefix)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, output)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return {
        'maximum_bytes': maximum_bytes,
        'observed_bytes': observed,
        'overflow': observed > maximum_bytes,
        'overflow_bytes': max(0, observed - maximum_bytes),
        'retained_bytes': retained,
        'sha256': file_sha256(output),
    }


def _command_plan(arguments: argparse.Namespace) -> int:
    document = suite_document(arguments.workspace, arguments.candidate_id, arguments.domain_base)
    atomic_write_json(arguments.output, document, sidecar=True)
    return 0


def _command_build_binding(arguments: argparse.Namespace) -> int:
    document = build_binding(
        arguments.workspace,
        git_sha=arguments.git_sha,
        git_status_porcelain=arguments.git_status_porcelain,
    )
    atomic_write_json(arguments.output, document, sidecar=True)
    return 0


def _command_lifecycle_schedule(arguments: argparse.Namespace) -> int:
    document = lifecycle_schedule(arguments.run_id, arguments.accepted_goal_stamp_ns)
    atomic_write_json(arguments.output, document, sidecar=True)
    return 0


def _command_bounded_log(arguments: argparse.Namespace) -> int:
    metadata = drain_bounded_log(os.sys.stdin.buffer, arguments.output, arguments.maximum_bytes)
    atomic_write_json(arguments.metadata, metadata, maximum_bytes=4096)
    return 1 if metadata['overflow'] else 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    plan = subparsers.add_parser('plan-suite', help='write the exact 15-trial ledger')
    plan.add_argument('--workspace', required=True, type=Path)
    plan.add_argument('--candidate-id', required=True)
    plan.add_argument('--domain-base', required=True, type=int)
    plan.add_argument('--output', required=True, type=Path)
    plan.set_defaults(function=_command_plan)
    binding = subparsers.add_parser('build-binding', help='write source/install/config provenance')
    binding.add_argument('--workspace', required=True, type=Path)
    binding.add_argument('--git-sha', required=True)
    binding.add_argument('--git-status-porcelain', default='')
    binding.add_argument('--output', required=True, type=Path)
    binding.set_defaults(function=_command_build_binding)
    schedule = subparsers.add_parser(
        'lifecycle-schedule', help='write the exact S4 absolute sample schedule'
    )
    schedule.add_argument('--run-id', required=True)
    schedule.add_argument('--accepted-goal-stamp-ns', required=True, type=int)
    schedule.add_argument('--output', required=True, type=Path)
    schedule.set_defaults(function=_command_lifecycle_schedule)
    log = subparsers.add_parser('bounded-log', help='drain stdin into a bounded prefix log')
    log.add_argument('--output', required=True, type=Path)
    log.add_argument('--metadata', required=True, type=Path)
    log.add_argument('--maximum-bytes', type=int, default=LOG_MAX_BYTES)
    log.set_defaults(function=_command_bounded_log)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run a pure orchestration utility subcommand."""
    arguments = _parser().parse_args(argv)
    try:
        return int(arguments.function(arguments))
    except EvidenceError as exc:
        print(f'phase3 orchestration evidence error: {exc}', file=os.sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
