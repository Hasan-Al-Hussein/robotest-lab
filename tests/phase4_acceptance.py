#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Fail-closed Phase 4 process, package, and Scenario 6 evidence utilities.

The module intentionally has no ROS imports.  Runtime ROS observation lives in
``phase4_active_goal_probe.py``; everything here is deterministic and unit
testable without starting Gazebo, systemd, or a supervisor process.
"""

from __future__ import annotations

import argparse
import contextlib
import csv
import ctypes
import errno
import grp
import hashlib
import json
import math
import os
import pwd
import re
import secrets
import shlex
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from collections.abc import Callable, Mapping, Sequence
from datetime import UTC, datetime
from itertools import pairwise
from pathlib import Path
from typing import Any

SCHEMA_VERSION = 1
ACTIVE_GOAL_SCHEMA_VERSION = 2
PRODUCER = 'robotest_phase4/acceptance_verifier'
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_TEXT_BYTES = 8 * 1024 * 1024
MAX_TIMELINE_ENTRIES = 4096
MAX_PROCESS_COUNT = 65536
MAX_PROC_FILE_BYTES = 1024 * 1024
MAX_EVIDENCE_FILES = 512
MAX_SOURCE_FILES = 20000
MAX_PACKAGE_BYTES = 512 * 1024 * 1024
MAX_GRAPH_ENDPOINTS = 4096
MAX_GRAPH_TYPES_PER_ENDPOINT = 16
MAX_PUBLICATION_FILE_BYTES = 64 * 1024 * 1024
MAX_PUBLICATION_TOTAL_BYTES = 512 * 1024 * 1024
MAX_PUBLICATION_DEPTH = 8
READY_FAILURE_TARGET_S = 3.0
PROCESS_GROUP_EXIT_TARGET_S = 5.0
RECOVERY_TARGET_S = 30.0
EXPECTED_BACKOFF_MS = 1000
EXPECTED_CHILD = 'robotest-stack'
EXPECTED_UNIT = 'robotest-supervisor.service'
FOLLOW_WAYPOINTS_ACTION = '/robotest/follow_waypoints'
FOLLOW_WAYPOINTS_ACTION_TYPE = 'nav2_msgs/action/FollowWaypoints'
FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE = f'{FOLLOW_WAYPOINTS_ACTION}/_action/send_goal'
FOLLOW_WAYPOINTS_SEND_GOAL_TYPE = f'{FOLLOW_WAYPOINTS_ACTION_TYPE}_SendGoal'
MISSION_RUNNER_NODE = '/robotest/mission_runner'
SHA256_RE = re.compile(r'^[0-9a-f]{64}$')
RUN_ID_RE = re.compile(r'^phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+$')
PARTITION_RE = re.compile(r'^[A-Za-z][A-Za-z0-9_]{0,127}$')
UUID_RE = re.compile(r'^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$')
CPU_MASK_RE = re.compile(r'^[0-9a-f]{8}(?:,[0-9a-f]{8})*$')
CPU_LIST_RE = re.compile(r'^[0-9]+(?:-[0-9]+)?(?:,[0-9]+(?:-[0-9]+)?)*$')
_DEFAULT_API = object()

EXPECTED_CPUSET = '0-5'
EXPECTED_UNIT_CGROUP = f'/system.slice/{EXPECTED_UNIT}'
EXPECTED_RUNTIME_STATE_ROOT = '/var/lib/robotest-supervisor'
RUNTIME_STATE_ENVIRONMENT = 'ROBOTEST_RUNTIME_STATE_DIRECTORY'
HEARTBEAT_NAME = 'robotest-stack.heartbeat'
STARTUP_RESULT_NAME = 'lifecycle-startup-result.json'
SUPERVISOR_CONFIG_CHECK = b'configuration valid\n'
LIVE_EVIDENCE_ROOT = Path('/run/robotest-phase4-verifier')
PUBLIC_EVIDENCE_RELATIVE_ROOT = Path('artifacts/evidence/phase4/runs')
LIVE_EVIDENCE_MARKER = '.phase4-live-owned'
LIFECYCLE_STARTUP_RESULT_KEYS = frozenset(
    {
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
)
RUNTIME_AFFINITY_KEYS = frozenset(
    {
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
)
RUNTIME_AFFINITY_PROCESS_KEYS = frozenset(
    {
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
)
SUPERVISOR_STATUS_KEYS = frozenset(
    {
        'children',
        'dropped_events',
        'event_count',
        'healthy',
        'persistence_healthy',
        'ready',
        'schema_version',
        'shutting_down',
        'supervisor_utc',
    }
)
SUPERVISOR_CHILD_REQUIRED_KEYS = frozenset(
    {
        'circuit_open',
        'heartbeat_age_ms',
        'heartbeat_fresh',
        'name',
        'pgid',
        'pid',
        'required',
        'restart_count',
        'running',
    }
)
SUPERVISOR_CHILD_OPTIONAL_KEYS = frozenset(
    {
        'last_exit_code',
        'last_failure_kind',
        'last_failure_utc',
        'last_ready_utc',
        'started_utc',
    }
)
SYSTEMD_OWNERS_KEYS = frozenset(
    {
        'captured_utc',
        'gz_partition',
        'ros_domain_id',
        'schema_version',
        'unit_count',
        'units',
        'verdict',
    }
)
PROCESS_GROUP_EVIDENCE_KEYS = frozenset(
    {'captured_utc', 'member_count', 'members', 'pgid', 'schema_version'}
)
SERVICE_FINAL_KEYS = frozenset(
    {
        'active_before_stop',
        'captured_utc',
        'enabled_before_stop',
        'main_pid_before_stop',
        'nrestarts_before_stop',
        'schema_version',
    }
)
CONTEXT_KEYS = frozenset(
    {
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
    }
)
CONTEXT_FILE_DESCRIPTOR_KEYS = frozenset({'path', 'sha256'})
ISOLATION_KEYS = frozenset(
    {
        'domain_was_unused',
        'gz_partition',
        'inspected_processes',
        'partition_was_unused',
        'ros_domain_id',
        'run_id',
        'schema_version',
        'unreadable_process_environments',
    }
)
EVENT_META_KEYS = frozenset(
    {'dropped_events', 'last_attempted_sequence', 'saturated', 'schema_version'}
)
TIMELINE_KEYS = frozenset(
    {'details', 'kind', 'monotonic_ns', 'schema_version', 'sequence', 'timestamp_utc'}
)
HTTP_TIMELINE_DETAIL_KEYS = frozenset({'body', 'body_sha256', 'http_status', 'url'})
TIMELINE_DETAIL_KEYS = {
    'active_goal_probe_started': frozenset({'pgid', 'pid'}),
    'cleanup_complete': frozenset(),
    'failure_injected': frozenset(
        {'event_sequence_before', 'mission_pid', 'original_pgid', 'target_pid'}
    ),
    'followup_finished': frozenset({'exit_code'}),
    'followup_started': frozenset({'pgid', 'pid'}),
    'health_probe': HTTP_TIMELINE_DETAIL_KEYS,
    'initial_ready': frozenset(
        {
            'child_pgid',
            'child_pid',
            'main_pid',
            'systemd_nrestarts',
            'systemd_owned_ros_service_count',
        }
    ),
    'interrupted_mission_finished': frozenset(
        {'exit_code', 'process_group_empty', 'terminated_by_harness'}
    ),
    'mission_started': frozenset({'pgid', 'pid'}),
    'original_group_empty': frozenset({'member_count'}),
    'ready_probe': HTTP_TIMELINE_DETAIL_KEYS,
    'ready_restored': frozenset(
        {
            'child_pgid',
            'child_pid',
            'event_sequence_after_recovery',
            'http_status',
            'main_pid',
            'systemd_nrestarts',
            'systemd_owned_ros_service_count',
        }
    ),
    'ready_unavailable': frozenset({'http_status'}),
    'service_stopped': frozenset({'event_sequence_before_stop'}),
    'startup_health_probe': HTTP_TIMELINE_DETAIL_KEYS,
    'startup_ready_probe': HTTP_TIMELINE_DETAIL_KEYS,
}
CONTROLLER_TARGET_KEYS = frozenset(
    {
        'cgroup',
        'cmdline',
        'cmdline_sha256',
        'comm',
        'executable',
        'gz_partition',
        'lineage',
        'pgid',
        'pid',
        'ppid',
        'root_pid',
        'ros_domain_id',
        'start_time_ticks',
        'state',
        'unit',
    }
)
CONTROLLER_LINEAGE_KEYS = frozenset({'pid', 'ppid', 'start_time_ticks'})

ACTIVE_GOAL_CLIENT_EVIDENCE_KEYS = frozenset(
    {
        'goal_capable_action_clients',
        'mission_runner_is_goal_capable_action_client',
        'projected_action_client_participants',
        'projected_action_clients',
        'send_goal_service_name',
        'send_goal_service_type',
        'service_clients',
    }
)
ACTIVE_GOAL_KEYS = frozenset(
    {
        'accepted_goal_stamp_ns',
        'action_name',
        'action_type',
        'attempt_count',
        'captured_utc',
        'elapsed_wall_s',
        'goal_capable_action_clients',
        'goal_status',
        'goal_status_code',
        'goal_uuid',
        'gz_partition',
        'mission_node',
        'mission_pid',
        'mission_process_alive',
        'mission_runner_is_goal_capable_action_client',
        'observed_monotonic_ns',
        'projected_action_client_participants',
        'projected_action_clients',
        'ros_domain_id',
        'schema_version',
        'send_goal_service_name',
        'send_goal_service_type',
        'service_clients',
        'status_entry_count',
        'verdict',
    }
)

LIFECYCLE_KEYS = frozenset(
    {
        'baseline_package',
        'completed_utc',
        'conffile',
        'final_state',
        'quality',
        'schema_version',
        'state',
        'upgrade_package',
        'verdict',
    }
)
LIFECYCLE_PACKAGE_KEYS = frozenset(
    {
        'architecture',
        'name',
        'sha256',
        'vendor_config_sha256',
        'version',
    }
)
LIFECYCLE_CONFFILE_KEYS = frozenset(
    {
        'modified_sha256',
        'modified_value',
        'preserved_during_remove',
        'preserved_during_upgrade',
        'removed_during_purge',
        'upgrade_vendor_default_restored_on_final_install',
    }
)
LIFECYCLE_STATE_KEYS = frozenset(
    {
        'log_directory_preserved',
        'sentinel_preserved_during_purge',
        'sentinel_preserved_during_remove',
        'state_directory_preserved',
    }
)
LIFECYCLE_FINAL_KEYS = frozenset(
    {
        'configuration_mode_owner',
        'dpkg_verify_clean',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_smoke_contract_passed',
        'installed_unit_sha256',
        'log_directory_mode_owner',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'stale_dpkg_conffile_artifacts',
        'state_directory_mode_owner',
        'systemd_unit_verified',
    }
)
LIFECYCLE_QUALITY_KEYS = frozenset(
    {
        'baseline_provenance_verified',
        'both_package_manifests_exact',
        'genuine_versioned_upgrade',
        'legacy_noble_lintian_static_built_using_warning',
        'lintian_unexpected_diagnostics',
    }
)
INSTALLED_STATE_KEYS = frozenset(
    {
        'architecture',
        'captured_utc',
        'configuration_mode_owner',
        'configuration_sha256',
        'dpkg_verify_clean',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_smoke_contract_passed',
        'installed_unit_sha256',
        'log_directory_mode_owner',
        'package_installed',
        'package_name',
        'schema_version',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'stale_dpkg_conffile_artifacts',
        'state_directory_mode_owner',
        'systemd_unit_verified',
        'verdict',
        'version',
    }
)
REPRODUCIBILITY_KEYS = frozenset(
    {
        'binary_packages_byte_identical',
        'builds',
        'completed_utc',
        'files',
        'generated_metadata_policy',
        'schema_version',
        'source_manifest_byte_identical',
        'verdict',
    }
)
REPRODUCIBILITY_FILE_KEYS = frozenset(
    {
        'build_a_sha256',
        'build_b_sha256',
        'comparison',
        'name',
        'normalized_sha256',
        'size_bytes',
    }
)
BASELINE_SIDECAR_KEYS = frozenset(
    {'created_utc', 'final_package', 'fixture_kind', 'fixture_package', 'schema_version'}
)
BASELINE_FINAL_PACKAGE_KEYS = frozenset({'name', 'sha256', 'version'})
BASELINE_FIXTURE_PACKAGE_KEYS = frozenset({'listen_address', 'name', 'sha256', 'version'})
SOURCE_MANIFEST_KEYS = frozenset({'files', 'schema_version'})
SOURCE_MANIFEST_FILE_KEYS = frozenset({'mode', 'path', 'sha256', 'size_bytes'})
PACKAGE_BINDING_KEYS = frozenset(
    {
        'candidate_manifest',
        'candidate_manifest_sha256',
        'extra_paths',
        'file_count',
        'mismatched_paths',
        'missing_paths',
        'repository_manifest_sha256',
        'schema_version',
        'verdict',
    }
)
PACKAGE_INTEGRITY_KEYS = frozenset(
    {
        'baseline_package_sha256',
        'baseline_sidecar_sha256',
        'build_a_binary_sha256',
        'build_a_debug_sha256',
        'build_b_binary_sha256',
        'build_b_debug_sha256',
        'package_directory',
        'reproducibility_evidence_sha256',
        'schema_version',
        'source_manifest_sha256',
        'upgrade_package_sha256',
        'verdict',
    }
)
PACKAGE_SOURCE_REBUILD_KEYS = frozenset(
    {
        'build_script_sha256',
        'byte_equal',
        'completed_utc',
        'rebuilt_artifacts',
        'schema_version',
        'selected_artifacts',
        'source_version',
        'verdict',
    }
)
PACKAGE_SOURCE_REBUILD_ARTIFACT_KEYS = frozenset({'kind', 'name', 'sha256', 'size_bytes'})
PACKAGE_SOURCE_REBUILD_EQUALITY_KEYS = frozenset(
    {'binary_package', 'debug_package', 'source_manifest'}
)
OVERLAY_PROVENANCE_KEYS = frozenset(
    {
        'active_path',
        'build_command',
        'created_utc',
        'git_commit',
        'git_dirty',
        'install_manifest_sha256',
        'packages',
        'release_id',
        'schema_version',
        'source_manifest_sha256',
        'source_workspace',
    }
)
OVERLAY_PACKAGES = [
    'robotest_description',
    'robotest_faults',
    'robotest_interfaces',
    'robotest_metrics',
    'robotest_missions',
    'robotest_navigation',
    'robotest_scenarios',
    'robotest_sim',
]
CLEANUP_KEYS = frozenset(
    {
        'captured_utc',
        'daemon_reloaded',
        'dropin_absent',
        'followup_last_pgid',
        'followup_process_group_empty',
        'heartbeat_absent',
        'interrupted_last_pgid',
        'interrupted_process_group_empty',
        'mission_process_group_empty',
        'original_child_pgid',
        'original_process_group_empty',
        'owned_paths_only',
        'probe_last_pgid',
        'probe_process_group_empty',
        'replacement_child_pgid',
        'replacement_process_group_empty',
        'run_state_directory_absent',
        'schema_version',
        'service_disabled',
        'service_inactive',
        'source_unchanged',
        'startup_result_absent',
    }
)
INTERRUPTED_OUTCOME_KEYS = frozenset(
    {
        'bounded_wait',
        'captured_utc',
        'csv_present',
        'json_present',
        'process_exit_code',
        'process_group_empty',
        'schema_version',
        'terminated_by_harness',
    }
)


class EvidenceError(RuntimeError):
    """Raised for missing, ambiguous, malformed, or unbounded evidence."""


def utc_now() -> str:
    """Return an RFC 3339 UTC timestamp."""
    return datetime.now(UTC).isoformat(timespec='microseconds').replace('+00:00', 'Z')


def canonical_json_bytes(value: Any) -> bytes:
    """Serialize strict canonical JSON with one trailing newline."""
    try:
        payload = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise EvidenceError(f'value is not strict JSON: {exc}') from exc
    return (payload + '\n').encode('utf-8')


def file_sha256(path: Path) -> str:
    """Hash one regular non-symlink file without an unbounded read."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'not a regular non-symlink file: {path}')
    digest = hashlib.sha256()
    with path.open('rb') as source:
        while block := source.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _read_bounded_bytes(path: Path, maximum_bytes: int) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'not a regular non-symlink file: {path}')
    if path.stat().st_size > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    payload = path.read_bytes()
    if len(payload) > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    return payload


def _atomic_write(path: Path, payload: bytes, *, mode: int = 0o644) -> None:
    if len(payload) > MAX_JSON_BYTES:
        raise EvidenceError(f'{path} exceeds the {MAX_JSON_BYTES}-byte artifact cap')
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, 'wb') as target:
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
        os.replace(temporary, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise


def atomic_write_json(path: Path, value: Any, *, mode: int = 0o644) -> None:
    """Atomically write strict canonical JSON."""
    _atomic_write(path, canonical_json_bytes(value), mode=mode)


def load_json(path: Path, *, maximum_bytes: int = MAX_JSON_BYTES) -> Any:
    """Load one bounded strict JSON value and reject trailing data."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'missing regular JSON file: {path}')
    size = path.stat().st_size
    if size > maximum_bytes:
        raise EvidenceError(f'{path} exceeds its {maximum_bytes}-byte cap')
    try:
        text = path.read_text(encoding='utf-8')
        decoder = json.JSONDecoder()
        value, end = decoder.raw_decode(text)
        if text[end:].strip():
            raise EvidenceError(f'{path} contains trailing JSON data')
        return value
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot decode {path}: {exc}') from exc


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    """Load a bounded JSON-lines file with object-only records."""
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'missing regular JSONL file: {path}')
    if path.stat().st_size > MAX_JSON_BYTES:
        raise EvidenceError(f'{path} exceeds the JSONL byte cap')
    records: list[dict[str, Any]] = []
    try:
        with path.open(encoding='utf-8') as source:
            for line_number, line in enumerate(source, 1):
                if line_number > MAX_TIMELINE_ENTRIES:
                    raise EvidenceError(f'{path} exceeds the record cap')
                if not line.endswith('\n'):
                    raise EvidenceError(f'{path} has an incomplete final record')
                value = json.loads(line)
                if not isinstance(value, dict):
                    raise EvidenceError(f'{path}:{line_number} is not an object')
                records.append(value)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot decode {path}: {exc}') from exc
    return records


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise EvidenceError(f'{name} must be an object')
    return value


def _integer(value: Any, name: str, *, minimum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise EvidenceError(f'{name} must be an integer')
    if minimum is not None and value < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return value


def _number(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise EvidenceError(f'{name} must be numeric')
    result = float(value)
    if not math.isfinite(result):
        raise EvidenceError(f'{name} must be finite')
    if minimum is not None and result < minimum:
        raise EvidenceError(f'{name} must be >= {minimum}')
    return result


def _string(value: Any, name: str, *, maximum: int = 4096) -> str:
    if not isinstance(value, str) or not value:
        raise EvidenceError(f'{name} must be a non-empty string')
    if len(value.encode('utf-8')) > maximum:
        raise EvidenceError(f'{name} exceeds the UTF-8 byte cap')
    return value


def _sha256(value: Any, name: str) -> str:
    result = _string(value, name, maximum=64)
    if SHA256_RE.fullmatch(result) is None:
        raise EvidenceError(f'{name} must be a lowercase SHA-256')
    return result


def _boolean(value: Any, name: str) -> bool:
    if not isinstance(value, bool):
        raise EvidenceError(f'{name} must be a boolean')
    return value


def _exact_keys(value: Mapping[str, Any], expected: frozenset[str], name: str) -> None:
    actual = set(value)
    if actual != expected:
        missing = sorted(expected - actual)
        extra = sorted(actual - expected)
        raise EvidenceError(f'{name} schema mismatch; missing={missing}, extra={extra}')


def _type_exact_json_equal(actual: Any, expected: Any) -> bool:
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _type_exact_json_equal(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _type_exact_json_equal(left, right)
            for left, right in zip(actual, expected, strict=True)
        )
    return bool(actual == expected)


def normalize_graph_endpoint_entries(value: Any, name: str) -> list[list[Any]]:
    """Return a bounded, deterministic graph endpoint snapshot."""
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise EvidenceError(f'{name} must be a sequence')
    if len(value) > MAX_GRAPH_ENDPOINTS:
        raise EvidenceError(f'{name} exceeds the {MAX_GRAPH_ENDPOINTS}-endpoint cap')
    endpoints: dict[str, set[str]] = {}
    for index, raw_entry in enumerate(value):
        entry_name = f'{name}[{index}]'
        if isinstance(raw_entry, (str, bytes)) or not isinstance(raw_entry, Sequence):
            raise EvidenceError(f'{entry_name} must be a two-item sequence')
        if len(raw_entry) != 2:
            raise EvidenceError(f'{entry_name} must contain exactly a name and type list')
        endpoint = _string(raw_entry[0], f'{entry_name}.name')
        raw_types = raw_entry[1]
        if isinstance(raw_types, (str, bytes)) or not isinstance(raw_types, Sequence):
            raise EvidenceError(f'{entry_name}.types must be a sequence')
        if len(raw_types) > MAX_GRAPH_TYPES_PER_ENDPOINT:
            raise EvidenceError(
                f'{entry_name}.types exceeds the {MAX_GRAPH_TYPES_PER_ENDPOINT}-type cap'
            )
        endpoint_types = endpoints.setdefault(endpoint, set())
        for type_index, raw_type in enumerate(raw_types):
            endpoint_types.add(_string(raw_type, f'{entry_name}.types[{type_index}]'))
        if len(endpoint_types) > MAX_GRAPH_TYPES_PER_ENDPOINT:
            raise EvidenceError(
                f'{endpoint} exceeds the {MAX_GRAPH_TYPES_PER_ENDPOINT}-type cap after merging'
            )
    return [[endpoint, sorted(types)] for endpoint, types in sorted(endpoints.items())]


def action_client_evidence_from_graph_entries(
    service_clients: Any,
    projected_action_clients: Any,
) -> dict[str, Any]:
    """Derive exact Phase 4 goal capability from the hidden SendGoal client.

    Jazzy can project passive action endpoint consumers as action clients.  The
    projected entries remain diagnostic, while only the exact SendGoal service
    client establishes that the mission runner can submit a goal.
    """
    normalized_services = normalize_graph_endpoint_entries(
        service_clients, 'mission runner service clients'
    )
    normalized_projected = normalize_graph_endpoint_entries(
        projected_action_clients, 'mission runner projected action clients'
    )
    send_goal_types: list[str] | None = None
    for service_name, service_types in normalized_services:
        if service_name == FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE:
            send_goal_types = service_types
            break

    goal_capable: dict[str, dict[str, list[str]]] = {}
    if send_goal_types is not None:
        action_types: set[str] = set()
        for service_type in send_goal_types:
            if service_type == FOLLOW_WAYPOINTS_SEND_GOAL_TYPE:
                action_types.add(FOLLOW_WAYPOINTS_ACTION_TYPE)
            elif service_type.endswith('_SendGoal'):
                action_types.add(service_type[: -len('_SendGoal')])
            else:
                action_types.add(service_type)
        goal_capable = {FOLLOW_WAYPOINTS_ACTION: {MISSION_RUNNER_NODE: sorted(action_types)}}

    projected_participants = {
        action_name: {MISSION_RUNNER_NODE: action_types}
        for action_name, action_types in normalized_projected
    }
    expected_goal_capable = {
        FOLLOW_WAYPOINTS_ACTION: {MISSION_RUNNER_NODE: [FOLLOW_WAYPOINTS_ACTION_TYPE]}
    }
    return {
        'goal_capable_action_clients': goal_capable,
        'mission_runner_is_goal_capable_action_client': goal_capable == expected_goal_capable,
        'projected_action_client_participants': projected_participants,
        'projected_action_clients': normalized_projected,
        'send_goal_service_name': FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE,
        'send_goal_service_type': FOLLOW_WAYPOINTS_SEND_GOAL_TYPE,
        'service_clients': normalized_services,
    }


def _utc_timestamp(value: Any, name: str) -> str:
    result = _string(value, name, maximum=64)
    try:
        parsed = datetime.fromisoformat(result.replace('Z', '+00:00'))
    except ValueError as exc:
        raise EvidenceError(f'{name} must be an ISO-8601 timestamp') from exc
    if parsed.tzinfo is None or parsed.utcoffset() != UTC.utcoffset(parsed):
        raise EvidenceError(f'{name} must have an explicit UTC offset')
    return result


def _run_command(
    command: Sequence[str],
    *,
    accepted_returncodes: frozenset[int] = frozenset({0}),
    timeout_s: float = 60.0,
) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            list(command),
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout_s,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise EvidenceError(f'command failed to execute: {command[0]}: {exc}') from exc
    if result.returncode not in accepted_returncodes:
        stderr = result.stderr.strip()[:4096]
        raise EvidenceError(f'command returned {result.returncode}: {command!r}: {stderr}')
    if len(result.stdout.encode('utf-8')) > MAX_TEXT_BYTES:
        raise EvidenceError(f'command output exceeded byte cap: {command!r}')
    return result


def _package_field(package: Path, field: str) -> str:
    output = _run_command(['dpkg-deb', '--field', str(package), field]).stdout.strip()
    return _string(output, f'{package.name} {field}', maximum=256)


def repository_changelog_version(repository: Path) -> str:
    """Return the exact current robotest-supervisor Debian source version."""
    changelog = repository.resolve(strict=True) / 'packaging/debian/changelog'
    payload = _read_bounded_bytes(changelog, MAX_TEXT_BYTES)
    if not payload.endswith(b'\n'):
        raise EvidenceError('Debian changelog has an incomplete final line')
    try:
        first_line = payload.decode('utf-8').splitlines()[0]
    except (UnicodeDecodeError, IndexError) as exc:
        raise EvidenceError('Debian changelog has no UTF-8 header') from exc
    match = re.fullmatch(
        r'robotest-supervisor \(([^()\s]+)\) '
        r'[a-z0-9][a-z0-9+.-]*; urgency=(?:low|medium|high|critical|emergency)',
        first_line,
    )
    if match is None:
        raise EvidenceError('Debian changelog has a noncanonical first header')
    version = match.group(1)
    _run_command(['dpkg', '--validate-version', version])
    return version


def _deb822_fields(path: Path) -> dict[str, str]:
    payload = _read_bounded_bytes(path, MAX_TEXT_BYTES)
    if not payload.endswith(b'\n'):
        raise EvidenceError(f'{path.name} has an incomplete final line')
    try:
        lines = payload.decode('utf-8').splitlines()
    except UnicodeDecodeError as exc:
        raise EvidenceError(f'{path.name} is not UTF-8') from exc
    fields: dict[str, str] = {}
    current = ''
    for line_number, line in enumerate(lines, 1):
        if not line:
            continue
        if line.startswith((' ', '\t')):
            if not current:
                raise EvidenceError(f'{path.name}:{line_number} has an orphan continuation')
            fields[current] += f'\n{line[1:]}'
            continue
        match = re.fullmatch(r'([A-Za-z0-9][A-Za-z0-9-]*):(?: (.*))?', line)
        if match is None:
            raise EvidenceError(f'{path.name}:{line_number} is not canonical deb822')
        current = match.group(1)
        if current in fields:
            raise EvidenceError(f'{path.name} repeats the {current} field')
        fields[current] = match.group(2) or ''
    return fields


def _validate_generated_package_metadata(
    path: Path,
    *,
    version: str,
    architecture: str,
) -> None:
    fields = _deb822_fields(path)
    for name in ('Architecture', 'Binary', 'Format', 'Source', 'Version'):
        if name not in fields:
            raise EvidenceError(f'{path.name} is missing the {name} field')
    if path.name.endswith('.buildinfo'):
        expected_format = '1.0'
        expected_binaries = ['robotest-supervisor', 'robotest-supervisor-dbgsym']
    elif path.name.endswith('.changes'):
        expected_format = '1.8'
        expected_binaries = ['robotest-supervisor']
    else:
        raise EvidenceError(f'{path.name} is not generated package metadata')
    binary_field = fields['Binary']
    if (
        re.fullmatch(
            r'[a-z0-9][a-z0-9+.-]*(?:[ \t\n]+[a-z0-9][a-z0-9+.-]*)*',
            binary_field,
        )
        is None
    ):
        raise EvidenceError(f'{path.name} has the wrong Binary field')
    binaries = sorted(re.split(r'[ \t\n]+', binary_field))
    if fields['Format'] != expected_format:
        raise EvidenceError(f'{path.name} has the wrong Format field')
    if fields['Source'] != 'robotest-supervisor':
        raise EvidenceError(f'{path.name} has the wrong Source field')
    if fields['Version'] != version:
        raise EvidenceError(f'{path.name} has the wrong Version field')
    if fields['Architecture'] != architecture:
        raise EvidenceError(f'{path.name} has the wrong Architecture field')
    if binaries != expected_binaries:
        raise EvidenceError(f'{path.name} has the wrong Binary field')


def package_artifact_descriptor(package: Path) -> dict[str, str]:
    """Bind package metadata and required installed payload bytes to one .deb."""
    if package.is_symlink():
        raise EvidenceError(f'package must not be a symbolic link: {package}')
    package = package.resolve(strict=True)
    if not package.is_file():
        raise EvidenceError(f'package is not a regular non-symlink file: {package}')
    if package.stat().st_size > MAX_PACKAGE_BYTES:
        raise EvidenceError(f'package exceeds the {MAX_PACKAGE_BYTES}-byte cap: {package}')
    with tempfile.TemporaryDirectory(prefix='robotest-phase4-package.') as temporary:
        extraction = Path(temporary) / 'root'
        extraction.mkdir()
        _run_command(['dpkg-deb', '--extract', str(package), str(extraction)])
        config = extraction / 'etc/robotest-supervisor/config.json'
        binary = extraction / 'usr/bin/robotest-supervisor'
        helper = extraction / 'usr/libexec/robotest-supervisor/start-robotest-stack'
        unit = extraction / 'usr/lib/systemd/system/robotest-supervisor.service'
        config_value = _mapping(
            load_json(config, maximum_bytes=MAX_TEXT_BYTES),
            f'{package.name} vendor config',
        )
        listen_address = _string(
            config_value.get('listen_address'),
            f'{package.name} vendor listen_address',
            maximum=256,
        )
        modified_value = dict(config_value)
        modified_value['listen_address'] = '127.0.0.1:9081'
        modified_payload = (
            json.dumps(modified_value, indent=2, sort_keys=True).encode('utf-8') + b'\n'
        )
        return {
            'name': _package_field(package, 'Package'),
            'version': _package_field(package, 'Version'),
            'architecture': _package_field(package, 'Architecture'),
            'sha256': file_sha256(package),
            'vendor_config_sha256': file_sha256(config),
            'vendor_listen_address': listen_address,
            'modified_config_sha256': hashlib.sha256(modified_payload).hexdigest(),
            'installed_binary_sha256': file_sha256(binary),
            'installed_helper_sha256': file_sha256(helper),
            'installed_unit_sha256': file_sha256(unit),
        }


def _normalize_generated_metadata(name: str, payload: bytes) -> bytes:
    try:
        text = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise EvidenceError(f'generated package metadata is not UTF-8: {name}') from exc
    if name.endswith('.buildinfo'):
        text, count = re.subn(
            r'^Build-Date: .+$',
            'Build-Date: <dpkg-wall-clock-build-date>',
            text,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise EvidenceError(f'{name} has no unique Build-Date field')
        return text.encode()
    if name.endswith('.changes'):
        text, count = re.subn(
            r'^(\s+)\S+(\s+.*\.buildinfo)$',
            r'\1<buildinfo-checksum>\2',
            text,
            flags=re.MULTILINE,
        )
        if count != 3:
            raise EvidenceError(f'{name} has {count} buildinfo checksums instead of 3')
        return text.encode()
    if name == 'SHA256SUMS':
        text, buildinfo_count = re.subn(
            r'^[0-9a-f]{64}(  \S+\.buildinfo)$',
            r'<buildinfo-sha256>\1',
            text,
            flags=re.MULTILINE,
        )
        text, changes_count = re.subn(
            r'^[0-9a-f]{64}(  \S+\.changes)$',
            r'<changes-sha256>\1',
            text,
            flags=re.MULTILINE,
        )
        if buildinfo_count != 1 or changes_count != 1:
            raise EvidenceError('SHA256SUMS has an unexpected generated-metadata shape')
        return text.encode()
    return payload


def _package_build_files(directory: Path) -> dict[str, Path]:
    if directory.is_symlink() or not directory.is_dir():
        raise EvidenceError(f'package build directory is invalid: {directory}')
    files: dict[str, Path] = {}
    for path in directory.iterdir():
        if path.is_symlink() or not path.is_file():
            raise EvidenceError(f'package build contains a non-regular entry: {path}')
        if path.name in files:
            raise EvidenceError(f'duplicate package build filename: {path.name}')
        files[path.name] = path
    return files


def _validate_local_sha256sums(directory: Path, files: Mapping[str, Path]) -> None:
    manifest = files.get('SHA256SUMS')
    if manifest is None:
        raise EvidenceError(f'{directory} is missing SHA256SUMS')
    payload = _read_bounded_bytes(manifest, MAX_TEXT_BYTES)
    if payload and not payload.endswith(b'\n'):
        raise EvidenceError(f'{manifest} has an incomplete final line')
    expected_names = set(files) - {'SHA256SUMS'}
    observed: dict[str, str] = {}
    for line_number, raw_line in enumerate(payload.decode('utf-8').splitlines(), 1):
        match = re.fullmatch(r'([0-9a-f]{64})  ([^/\s]+)', raw_line)
        if match is None:
            raise EvidenceError(f'{manifest}:{line_number} is not canonical')
        digest, name = match.groups()
        if name in observed:
            raise EvidenceError(f'{manifest} repeats {name}')
        observed[name] = digest
    if set(observed) != expected_names:
        raise EvidenceError(f'{manifest} does not cover the exact build file set')
    for name, digest in observed.items():
        if file_sha256(files[name]) != digest:
            raise EvidenceError(f'{manifest} does not match {name}')


def verify_package_candidate(
    package_directory: Path,
    upgrade_package: Path,
    baseline_package: Path,
    repository: Path,
) -> dict[str, Any]:
    """Independently reconcile both builds, repro evidence, and baseline provenance."""
    if package_directory.is_symlink():
        raise EvidenceError(f'package directory must not be a symlink: {package_directory}')
    package_directory = package_directory.resolve(strict=True)
    if not package_directory.is_dir():
        raise EvidenceError(f'package directory is invalid: {package_directory}')
    upgrade_package = upgrade_package.resolve(strict=True)
    baseline_package = baseline_package.resolve(strict=True)
    source_version = repository_changelog_version(repository)
    upgrade = package_artifact_descriptor(upgrade_package)
    if upgrade['version'] != source_version:
        raise EvidenceError(
            'selected upgrade version does not match the repository Debian changelog'
        )
    architecture = upgrade['architecture']
    binary_name = f'robotest-supervisor_{source_version}_{architecture}.deb'
    debug_name = f'robotest-supervisor-dbgsym_{source_version}_{architecture}.ddeb'
    buildinfo_name = f'robotest-supervisor_{source_version}_{architecture}.buildinfo'
    changes_name = f'robotest-supervisor_{source_version}_{architecture}.changes'
    expected_build_names = {
        'SHA256SUMS',
        'SOURCE-MANIFEST.json',
        binary_name,
        buildinfo_name,
        changes_name,
        debug_name,
    }
    build_a = _package_build_files(package_directory / 'build-a')
    build_b = _package_build_files(package_directory / 'build-b')
    if set(build_a) != set(build_b):
        raise EvidenceError('build-a and build-b file sets differ')
    if set(build_a) != expected_build_names:
        raise EvidenceError('package build filenames do not match source version and architecture')
    if upgrade_package != build_a[binary_name].resolve(strict=True):
        raise EvidenceError('selected upgrade package is not the exact build-a binary')
    _validate_local_sha256sums(package_directory / 'build-a', build_a)
    _validate_local_sha256sums(package_directory / 'build-b', build_b)
    for build in (build_a, build_b):
        _validate_generated_package_metadata(
            build[buildinfo_name],
            version=source_version,
            architecture=architecture,
        )
        _validate_generated_package_metadata(
            build[changes_name],
            version=source_version,
            architecture=architecture,
        )

    reproducibility_path = package_directory / 'reproducibility.json'
    reproducibility = _mapping(load_json(reproducibility_path), 'package reproducibility evidence')
    _exact_keys(
        reproducibility,
        REPRODUCIBILITY_KEYS,
        'package reproducibility evidence',
    )
    if _integer(reproducibility.get('schema_version'), 'repro schema_version') != 1:
        raise EvidenceError('package reproducibility schema_version must be 1')
    if reproducibility.get('verdict') != 'PASS':
        raise EvidenceError('package reproducibility verdict must be PASS')
    _utc_timestamp(reproducibility.get('completed_utc'), 'repro completed_utc')
    if reproducibility.get('builds') != ['build-a', 'build-b']:
        raise EvidenceError('package reproducibility builds are not exact')
    if (
        _boolean(
            reproducibility.get('binary_packages_byte_identical'),
            'binary_packages_byte_identical',
        )
        is not True
    ):
        raise EvidenceError('binary package reproducibility is not proven')
    if (
        _boolean(
            reproducibility.get('source_manifest_byte_identical'),
            'source_manifest_byte_identical',
        )
        is not True
    ):
        raise EvidenceError('source manifest reproducibility is not proven')
    policy = (
        'dpkg .buildinfo Build-Date is wall-clock metadata; .changes and SHA256SUMS '
        'may differ only through the cascading .buildinfo/.changes checksums'
    )
    if reproducibility.get('generated_metadata_policy') != policy:
        raise EvidenceError('package reproducibility metadata policy is not exact')

    records = _mapping_list(reproducibility.get('files'), 'reproducibility files')
    names = [_string(record.get('name'), 'repro file name', maximum=255) for record in records]
    if names != sorted(names) or len(names) != len(set(names)) or set(names) != set(build_a):
        raise EvidenceError('reproducibility file list is not the exact sorted build set')
    if set(names) != expected_build_names:
        raise EvidenceError('package build artifact set is not exact')

    for record, name in zip(records, names, strict=True):
        _exact_keys(record, REPRODUCIBILITY_FILE_KEYS, f'reproducibility file {name}')
        first_payload = _read_bounded_bytes(build_a[name], MAX_PACKAGE_BYTES)
        second_payload = _read_bounded_bytes(build_b[name], MAX_PACKAGE_BYTES)
        comparison = 'byte_identical'
        normalized = first_payload
        if first_payload != second_payload:
            if not (
                name == 'SHA256SUMS' or name.endswith('.buildinfo') or name.endswith('.changes')
            ):
                raise EvidenceError(f'cross-build byte mismatch: {name}')
            normalized = _normalize_generated_metadata(name, first_payload)
            if normalized != _normalize_generated_metadata(name, second_payload):
                raise EvidenceError(f'cross-build normalized mismatch: {name}')
            comparison = 'normalized_dpkg_build_date_only'
        expected_record = {
            'build_a_sha256': hashlib.sha256(first_payload).hexdigest(),
            'build_b_sha256': hashlib.sha256(second_payload).hexdigest(),
            'comparison': comparison,
            'name': name,
            'normalized_sha256': hashlib.sha256(normalized).hexdigest(),
            'size_bytes': len(first_payload),
        }
        if len(first_payload) != len(second_payload):
            raise EvidenceError(f'cross-build size mismatch: {name}')
        for field in ('build_a_sha256', 'build_b_sha256', 'normalized_sha256'):
            _sha256(record.get(field), f'{name}.{field}')
        _integer(record.get('size_bytes'), f'{name}.size_bytes', minimum=0)
        if dict(record) != expected_record:
            raise EvidenceError(f'reproducibility evidence does not reconcile {name}')

    source_name = 'SOURCE-MANIFEST.json'
    for name in (binary_name, debug_name, source_name):
        if file_sha256(build_a[name]) != file_sha256(build_b[name]):
            raise EvidenceError(f'required byte-identical artifact differs: {name}')

    baseline = package_artifact_descriptor(baseline_package)
    sidecar_path = Path(f'{baseline_package}.fixture.json')
    if sidecar_path.is_symlink():
        raise EvidenceError('baseline provenance sidecar must not be a symlink')
    sidecar = _mapping(load_json(sidecar_path), 'baseline provenance sidecar')
    _exact_keys(sidecar, BASELINE_SIDECAR_KEYS, 'baseline provenance sidecar')
    if _integer(sidecar.get('schema_version'), 'baseline sidecar schema_version') != 1:
        raise EvidenceError('baseline provenance schema_version must be 1')
    _utc_timestamp(sidecar.get('created_utc'), 'baseline sidecar created_utc')
    if sidecar.get('fixture_kind') != 'genuine_lower_version_different_default_conffile':
        raise EvidenceError('baseline provenance fixture_kind is not exact')
    final_package = _mapping(sidecar.get('final_package'), 'sidecar final_package')
    fixture_package = _mapping(sidecar.get('fixture_package'), 'sidecar fixture_package')
    _exact_keys(final_package, BASELINE_FINAL_PACKAGE_KEYS, 'sidecar final_package')
    _exact_keys(fixture_package, BASELINE_FIXTURE_PACKAGE_KEYS, 'sidecar fixture_package')
    expected_final = {
        'name': upgrade_package.name,
        'sha256': upgrade['sha256'],
        'version': upgrade['version'],
    }
    expected_fixture = {
        'listen_address': baseline['vendor_listen_address'],
        'name': baseline_package.name,
        'sha256': baseline['sha256'],
        'version': baseline['version'],
    }
    if dict(final_package) != expected_final or dict(fixture_package) != expected_fixture:
        raise EvidenceError('baseline provenance does not bind both selected packages')
    if baseline['vendor_config_sha256'] == upgrade['vendor_config_sha256']:
        raise EvidenceError('baseline and upgrade vendor configurations are identical')

    report = {
        'baseline_package_sha256': baseline['sha256'],
        'baseline_sidecar_sha256': file_sha256(sidecar_path),
        'build_a_binary_sha256': file_sha256(build_a[binary_name]),
        'build_a_debug_sha256': file_sha256(build_a[debug_name]),
        'build_b_binary_sha256': file_sha256(build_b[binary_name]),
        'build_b_debug_sha256': file_sha256(build_b[debug_name]),
        'package_directory': str(package_directory),
        'reproducibility_evidence_sha256': file_sha256(reproducibility_path),
        'schema_version': SCHEMA_VERSION,
        'source_manifest_sha256': file_sha256(build_a[source_name]),
        'upgrade_package_sha256': upgrade['sha256'],
        'verdict': 'PASS',
    }
    _exact_keys(report, PACKAGE_INTEGRITY_KEYS, 'package integrity report')
    return report


def _rebuild_artifact_record(kind: str, path: Path) -> dict[str, Any]:
    if path.is_symlink() or not path.is_file():
        raise EvidenceError(f'rebuild artifact is not a regular file: {path}')
    return {
        'kind': kind,
        'name': path.name,
        'sha256': file_sha256(path),
        'size_bytes': path.stat().st_size,
    }


def package_source_rebuild_attestation(
    repository: Path,
    package_directory: Path,
    upgrade_package: Path,
    rebuilt_directory: Path,
) -> dict[str, Any]:
    """Compare a fresh controlled source rebuild to the selected build-a bytes."""
    repository = repository.resolve(strict=True)
    package_directory = package_directory.resolve(strict=True)
    rebuilt_directory = rebuilt_directory.resolve(strict=True)
    upgrade = package_artifact_descriptor(upgrade_package)
    source_version = repository_changelog_version(repository)
    if upgrade['version'] != source_version:
        raise EvidenceError('rebuild source version does not match the selected upgrade')
    architecture = upgrade['architecture']
    names = {
        'binary_package': f'robotest-supervisor_{source_version}_{architecture}.deb',
        'debug_package': f'robotest-supervisor-dbgsym_{source_version}_{architecture}.ddeb',
        'source_manifest': 'SOURCE-MANIFEST.json',
    }
    selected_root = package_directory / 'build-a'
    selected = [
        _rebuild_artifact_record(kind, selected_root / name) for kind, name in names.items()
    ]
    rebuilt = [
        _rebuild_artifact_record(kind, rebuilt_directory / name) for kind, name in names.items()
    ]
    byte_equal = {
        kind: _read_bounded_bytes(selected_root / name, MAX_PACKAGE_BYTES)
        == _read_bounded_bytes(rebuilt_directory / name, MAX_PACKAGE_BYTES)
        for kind, name in names.items()
    }
    if not all(byte_equal.values()):
        raise EvidenceError('fresh package source rebuild differs from the selected artifacts')
    report = {
        'build_script_sha256': file_sha256(repository / 'scripts/build_debian_package.sh'),
        'byte_equal': byte_equal,
        'completed_utc': utc_now(),
        'rebuilt_artifacts': rebuilt,
        'schema_version': 1,
        'selected_artifacts': selected,
        'source_version': source_version,
        'verdict': 'PASS',
    }
    _exact_keys(report, PACKAGE_SOURCE_REBUILD_KEYS, 'package source rebuild')
    return report


def import_package_source_rebuild_attestation(
    repository: Path,
    package_directory: Path,
    upgrade_package: Path,
    rebuilt_directory: Path,
    worker_attestation_path: Path,
) -> dict[str, Any]:
    """Replay an unprivileged rebuild and preserve its validated completion time."""
    worker_payload = _read_bounded_bytes(worker_attestation_path, MAX_JSON_BYTES)
    worker = _mapping(load_json(worker_attestation_path), 'worker rebuild attestation')
    if worker_payload != canonical_json_bytes(worker):
        raise EvidenceError('worker rebuild attestation is not canonical JSON')
    completed_utc = _utc_timestamp(
        worker.get('completed_utc'), 'worker rebuild attestation.completed_utc'
    )
    replay = package_source_rebuild_attestation(
        repository,
        package_directory,
        upgrade_package,
        rebuilt_directory,
    )
    replay['completed_utc'] = completed_utc
    if not _type_exact_json_equal(worker, replay):
        raise EvidenceError('worker rebuild attestation differs from the root replay')
    return replay


def package_source_rebuild_is_exact(
    value: Mapping[str, Any],
    repository: Path,
    package_directory: Path,
    upgrade_package: Path,
) -> bool:
    """Replay every retained source-rebuild attestation join."""
    try:
        _exact_keys(value, PACKAGE_SOURCE_REBUILD_KEYS, 'package source rebuild')
        if _integer(value.get('schema_version'), 'package source rebuild schema_version') != 1:
            return False
        if value.get('verdict') != 'PASS':
            return False
        _utc_timestamp(value.get('completed_utc'), 'package source rebuild completed_utc')
        repository = repository.resolve(strict=True)
        source_version = repository_changelog_version(repository)
        upgrade = package_artifact_descriptor(upgrade_package)
        if value.get('source_version') != source_version or upgrade['version'] != source_version:
            return False
        if value.get('build_script_sha256') != file_sha256(
            repository / 'scripts/build_debian_package.sh'
        ):
            return False
        architecture = upgrade['architecture']
        selected_root = package_directory.resolve(strict=True) / 'build-a'
        names = {
            'binary_package': f'robotest-supervisor_{source_version}_{architecture}.deb',
            'debug_package': f'robotest-supervisor-dbgsym_{source_version}_{architecture}.ddeb',
            'source_manifest': 'SOURCE-MANIFEST.json',
        }
        expected_selected = [
            _rebuild_artifact_record(kind, selected_root / name) for kind, name in names.items()
        ]
        selected = _mapping_list(
            value.get('selected_artifacts'), 'package source rebuild selected_artifacts'
        )
        rebuilt = _mapping_list(
            value.get('rebuilt_artifacts'), 'package source rebuild rebuilt_artifacts'
        )
        for label, records in (('selected', selected), ('rebuilt', rebuilt)):
            for index, record in enumerate(records):
                _exact_keys(
                    record,
                    PACKAGE_SOURCE_REBUILD_ARTIFACT_KEYS,
                    f'package source rebuild {label}[{index}]',
                )
        equality = _mapping(value.get('byte_equal'), 'package source rebuild byte_equal')
        _exact_keys(equality, PACKAGE_SOURCE_REBUILD_EQUALITY_KEYS, 'rebuild byte_equal')
        return (
            _type_exact_json_equal(selected, expected_selected)
            and _type_exact_json_equal(rebuilt, expected_selected)
            and _type_exact_json_equal(
                equality,
                {
                    'binary_package': True,
                    'debug_package': True,
                    'source_manifest': True,
                },
            )
        )
    except (EvidenceError, OSError, RuntimeError):
        return False


def _validate_lifecycle_package(
    value: Any,
    expected: Mapping[str, Any],
    name: str,
) -> Mapping[str, Any]:
    package = _mapping(value, name)
    _exact_keys(package, LIFECYCLE_PACKAGE_KEYS, name)
    expected_fields = {key: expected.get(key) for key in LIFECYCLE_PACKAGE_KEYS}
    for field in ('name', 'version', 'architecture'):
        _string(package.get(field), f'{name}.{field}', maximum=256)
        _string(expected_fields.get(field), f'expected {name}.{field}', maximum=256)
    for field in ('sha256', 'vendor_config_sha256'):
        _sha256(package.get(field), f'{name}.{field}')
        _sha256(expected_fields.get(field), f'expected {name}.{field}')
    if dict(package) != expected_fields:
        raise EvidenceError(f'{name} does not match its selected package artifact')
    return package


def validate_lifecycle_evidence(
    value: Any,
    upgrade: Mapping[str, Any],
    baseline: Mapping[str, Any],
) -> None:
    """Validate the exact lifecycle schema and every frozen proof fact."""
    lifecycle = _mapping(value, 'package lifecycle')
    _exact_keys(lifecycle, LIFECYCLE_KEYS, 'package lifecycle')
    if _integer(lifecycle.get('schema_version'), 'package lifecycle schema_version') != (
        SCHEMA_VERSION
    ):
        raise EvidenceError('package lifecycle schema_version must be 1')
    if lifecycle.get('verdict') != 'PASS':
        raise EvidenceError('package lifecycle verdict must be PASS')
    _utc_timestamp(lifecycle.get('completed_utc'), 'package lifecycle completed_utc')

    baseline_value = _validate_lifecycle_package(
        lifecycle.get('baseline_package'), baseline, 'baseline_package'
    )
    upgrade_value = _validate_lifecycle_package(
        lifecycle.get('upgrade_package'), upgrade, 'upgrade_package'
    )
    if baseline_value['name'] != 'robotest-supervisor' or upgrade_value['name'] != (
        'robotest-supervisor'
    ):
        raise EvidenceError('lifecycle packages must both be robotest-supervisor')
    if baseline_value['architecture'] != upgrade_value['architecture']:
        raise EvidenceError('lifecycle package architectures differ')
    version_check = _run_command(
        [
            'dpkg',
            '--compare-versions',
            str(baseline_value['version']),
            'lt',
            str(upgrade_value['version']),
        ],
        accepted_returncodes=frozenset({0, 1}),
    )
    if version_check.returncode != 0:
        raise EvidenceError('lifecycle baseline version is not lower than the upgrade')
    if baseline_value['vendor_config_sha256'] == upgrade_value['vendor_config_sha256']:
        raise EvidenceError('lifecycle packages do not have distinct vendor configurations')

    conffile = _mapping(lifecycle.get('conffile'), 'package lifecycle conffile')
    _exact_keys(conffile, LIFECYCLE_CONFFILE_KEYS, 'package lifecycle conffile')
    modified_sha = _sha256(conffile.get('modified_sha256'), 'conffile.modified_sha256')
    expected_modified_sha = _sha256(
        baseline.get('modified_config_sha256'), 'expected modified config SHA'
    )
    if modified_sha != expected_modified_sha or modified_sha in {
        baseline_value['vendor_config_sha256'],
        upgrade_value['vendor_config_sha256'],
    }:
        raise EvidenceError('modified conffile SHA is not the exact lifecycle mutation')
    modified_value = _mapping(conffile.get('modified_value'), 'conffile.modified_value')
    _exact_keys(modified_value, frozenset({'listen_address'}), 'conffile.modified_value')
    if dict(modified_value) != {'listen_address': '127.0.0.1:9081'}:
        raise EvidenceError('conffile.modified_value is not the frozen lifecycle mutation')
    for field in (
        'preserved_during_remove',
        'preserved_during_upgrade',
        'removed_during_purge',
        'upgrade_vendor_default_restored_on_final_install',
    ):
        if _boolean(conffile.get(field), f'conffile.{field}') is not True:
            raise EvidenceError(f'conffile.{field} must be true')

    state_value = _mapping(lifecycle.get('state'), 'package lifecycle state')
    _exact_keys(state_value, LIFECYCLE_STATE_KEYS, 'package lifecycle state')
    for field in sorted(LIFECYCLE_STATE_KEYS):
        if _boolean(state_value.get(field), f'state.{field}') is not True:
            raise EvidenceError(f'state.{field} must be true')

    final = _mapping(lifecycle.get('final_state'), 'package lifecycle final_state')
    _exact_keys(final, LIFECYCLE_FINAL_KEYS, 'package lifecycle final_state')
    expected_final: dict[str, Any] = {
        'configuration_mode_owner': '644:root:root',
        'dpkg_verify_clean': True,
        'installed_binary_sha256': upgrade.get('installed_binary_sha256'),
        'installed_helper_sha256': upgrade.get('installed_helper_sha256'),
        'installed_smoke_contract_passed': True,
        'installed_unit_sha256': upgrade.get('installed_unit_sha256'),
        'log_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
        'package_installed': True,
        'service_account_render_group': True,
        'service_account_video_group': True,
        'service_active': False,
        'service_enabled': False,
        'stale_dpkg_conffile_artifacts': 0,
        'state_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
        'systemd_unit_verified': True,
    }
    for field in (
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_unit_sha256',
    ):
        _sha256(final.get(field), f'final_state.{field}')
        _sha256(expected_final.get(field), f'expected final_state.{field}')
    for field in (
        'dpkg_verify_clean',
        'installed_smoke_contract_passed',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'systemd_unit_verified',
    ):
        _boolean(final.get(field), f'final_state.{field}')
    _integer(
        final.get('stale_dpkg_conffile_artifacts'),
        'final_state.stale_dpkg_conffile_artifacts',
        minimum=0,
    )
    for field in (
        'configuration_mode_owner',
        'log_directory_mode_owner',
        'state_directory_mode_owner',
    ):
        _string(final.get(field), f'final_state.{field}', maximum=128)
    if dict(final) != expected_final:
        raise EvidenceError('package lifecycle final_state is incomplete or tampered')

    quality = _mapping(lifecycle.get('quality'), 'package lifecycle quality')
    _exact_keys(quality, LIFECYCLE_QUALITY_KEYS, 'package lifecycle quality')
    for field in (
        'baseline_provenance_verified',
        'both_package_manifests_exact',
        'genuine_versioned_upgrade',
    ):
        if _boolean(quality.get(field), f'quality.{field}') is not True:
            raise EvidenceError(f'quality.{field} must be true')
    _boolean(
        quality.get('legacy_noble_lintian_static_built_using_warning'),
        'quality.legacy_noble_lintian_static_built_using_warning',
    )
    if (
        _integer(
            quality.get('lintian_unexpected_diagnostics'),
            'quality.lintian_unexpected_diagnostics',
            minimum=0,
        )
        != 0
    ):
        raise EvidenceError('quality.lintian_unexpected_diagnostics must be zero')


def _mode_owner(path: Path) -> str:
    if path.is_symlink() or not path.exists():
        raise EvidenceError(f'installed path is missing or a symlink: {path}')
    metadata = path.stat()
    return (
        f'{stat.S_IMODE(metadata.st_mode):o}:'
        f'{pwd.getpwuid(metadata.st_uid).pw_name}:'
        f'{grp.getgrgid(metadata.st_gid).gr_name}'
    )


def validate_installed_package_state(
    value: Any,
    lifecycle: Mapping[str, Any],
    upgrade: Mapping[str, Any],
) -> None:
    """Validate the exact direct re-observation of the installed candidate."""
    report = _mapping(value, 'installed package state')
    _exact_keys(report, INSTALLED_STATE_KEYS, 'installed package state')
    final = _mapping(lifecycle.get('final_state'), 'package lifecycle final_state')
    expected: dict[str, Any] = {
        'architecture': upgrade.get('architecture'),
        'captured_utc': report.get('captured_utc'),
        'configuration_mode_owner': final.get('configuration_mode_owner'),
        'configuration_sha256': upgrade.get('vendor_config_sha256'),
        'dpkg_verify_clean': final.get('dpkg_verify_clean'),
        'installed_binary_sha256': final.get('installed_binary_sha256'),
        'installed_helper_sha256': final.get('installed_helper_sha256'),
        'installed_smoke_contract_passed': final.get('installed_smoke_contract_passed'),
        'installed_unit_sha256': final.get('installed_unit_sha256'),
        'log_directory_mode_owner': final.get('log_directory_mode_owner'),
        'package_installed': final.get('package_installed'),
        'package_name': upgrade.get('name'),
        'schema_version': SCHEMA_VERSION,
        'service_account_render_group': final.get('service_account_render_group'),
        'service_account_video_group': final.get('service_account_video_group'),
        'service_active': final.get('service_active'),
        'service_enabled': final.get('service_enabled'),
        'stale_dpkg_conffile_artifacts': final.get('stale_dpkg_conffile_artifacts'),
        'state_directory_mode_owner': final.get('state_directory_mode_owner'),
        'systemd_unit_verified': final.get('systemd_unit_verified'),
        'verdict': 'PASS',
        'version': upgrade.get('version'),
    }
    if _integer(report.get('schema_version'), 'installed package schema_version') != (
        SCHEMA_VERSION
    ):
        raise EvidenceError('installed package schema_version must be 1')
    _utc_timestamp(report.get('captured_utc'), 'installed package captured_utc')
    for field in (
        'architecture',
        'configuration_mode_owner',
        'log_directory_mode_owner',
        'package_name',
        'state_directory_mode_owner',
        'verdict',
        'version',
    ):
        _string(report.get(field), f'installed package {field}', maximum=256)
    for field in (
        'configuration_sha256',
        'installed_binary_sha256',
        'installed_helper_sha256',
        'installed_unit_sha256',
    ):
        _sha256(report.get(field), f'installed package {field}')
    for field in (
        'dpkg_verify_clean',
        'installed_smoke_contract_passed',
        'package_installed',
        'service_account_render_group',
        'service_account_video_group',
        'service_active',
        'service_enabled',
        'systemd_unit_verified',
    ):
        _boolean(report.get(field), f'installed package {field}')
    _integer(
        report.get('stale_dpkg_conffile_artifacts'),
        'installed package stale_dpkg_conffile_artifacts',
        minimum=0,
    )
    if dict(report) != expected:
        raise EvidenceError('installed package state does not match lifecycle evidence')


def validate_cleanup_evidence(
    value: Any,
    *,
    interrupted_pgid: int,
    followup_pgid: int,
    probe_pgid: int,
    original_pgid: int,
    replacement_pgid: int,
) -> None:
    """Require exact, write-once cleanup evidence bound to every published PGID."""
    cleanup = _mapping(value, 'cleanup evidence')
    _exact_keys(cleanup, CLEANUP_KEYS, 'cleanup evidence')
    if _integer(cleanup.get('schema_version'), 'cleanup schema_version') != 1:
        raise EvidenceError('cleanup schema_version must be 1')
    _utc_timestamp(cleanup.get('captured_utc'), 'cleanup captured_utc')
    boolean_fields = CLEANUP_KEYS - {
        'captured_utc',
        'followup_last_pgid',
        'interrupted_last_pgid',
        'original_child_pgid',
        'probe_last_pgid',
        'replacement_child_pgid',
        'schema_version',
    }
    for field in sorted(boolean_fields):
        if _boolean(cleanup.get(field), f'cleanup.{field}') is not True:
            raise EvidenceError(f'cleanup.{field} must be true')
    expected_pgids = {
        'interrupted_last_pgid': interrupted_pgid,
        'followup_last_pgid': followup_pgid,
        'probe_last_pgid': probe_pgid,
        'original_child_pgid': original_pgid,
        'replacement_child_pgid': replacement_pgid,
    }
    for field, expected in expected_pgids.items():
        actual = _integer(cleanup.get(field), f'cleanup.{field}', minimum=2)
        if actual != expected:
            raise EvidenceError(f'cleanup.{field} does not match its published PGID')


def capture_installed_package_state(
    lifecycle: Mapping[str, Any],
    upgrade: Mapping[str, Any],
    smoke_script: Path,
) -> dict[str, Any]:
    """Directly re-observe the installed candidate without changing package state."""
    if smoke_script.is_symlink():
        raise EvidenceError(f'installed smoke script must not be a symlink: {smoke_script}')
    smoke_script = smoke_script.resolve(strict=True)
    if not smoke_script.is_file():
        raise EvidenceError(f'invalid installed smoke script: {smoke_script}')
    query = (
        _run_command(
            [
                'dpkg-query',
                '--show',
                '--showformat=${Package}\t${Version}\t${Architecture}',
                'robotest-supervisor',
            ]
        )
        .stdout.strip()
        .split('\t')
    )
    if len(query) != 3:
        raise EvidenceError('installed package metadata is malformed')
    verify = _run_command(['dpkg', '--verify', 'robotest-supervisor'])
    enabled = _run_command(
        ['systemctl', 'is-enabled', EXPECTED_UNIT],
        accepted_returncodes=frozenset({0, 1, 3, 4}),
    ).stdout.strip()
    active = _run_command(
        ['systemctl', 'is-active', EXPECTED_UNIT],
        accepted_returncodes=frozenset({0, 1, 2, 3, 4}),
    ).stdout.strip()
    if enabled != 'disabled':
        raise EvidenceError(f'{EXPECTED_UNIT} is not exactly disabled: {enabled!r}')
    if active != 'inactive':
        raise EvidenceError(f'{EXPECTED_UNIT} is not exactly inactive: {active!r}')
    groups = set(_run_command(['id', '-nG', 'robotest-supervisor']).stdout.split())
    _run_command([str(smoke_script)])
    config = Path('/etc/robotest-supervisor/config.json')
    state_directory = Path('/var/lib/robotest-supervisor')
    log_directory = Path('/var/log/robotest-supervisor')
    binary = Path('/usr/bin/robotest-supervisor')
    helper = Path('/usr/libexec/robotest-supervisor/start-robotest-stack')
    unit = Path('/usr/lib/systemd/system/robotest-supervisor.service')
    report = {
        'architecture': query[2],
        'captured_utc': utc_now(),
        'configuration_mode_owner': _mode_owner(config),
        'configuration_sha256': file_sha256(config),
        'dpkg_verify_clean': not verify.stdout.strip() and not verify.stderr.strip(),
        'installed_binary_sha256': file_sha256(binary),
        'installed_helper_sha256': file_sha256(helper),
        'installed_smoke_contract_passed': True,
        'installed_unit_sha256': file_sha256(unit),
        'log_directory_mode_owner': _mode_owner(log_directory),
        'package_installed': True,
        'package_name': query[0],
        'schema_version': SCHEMA_VERSION,
        'service_account_render_group': 'render' in groups,
        'service_account_video_group': 'video' in groups,
        'service_active': active == 'active',
        'service_enabled': enabled == 'enabled',
        'stale_dpkg_conffile_artifacts': len(list(config.parent.glob('config.json.dpkg-*'))),
        'state_directory_mode_owner': _mode_owner(state_directory),
        'systemd_unit_verified': True,
        'verdict': 'PASS',
        'version': query[1],
    }
    validate_installed_package_state(report, lifecycle, upgrade)
    return report


def append_timeline(path: Path, kind: str, details: Mapping[str, Any]) -> dict[str, Any]:
    """Append one fsynced, sequence-checked timeline observation."""
    if not re.fullmatch(r'[a-z][a-z0-9_]{1,63}', kind):
        raise EvidenceError(f'invalid timeline kind: {kind!r}')
    previous = load_jsonl(path) if path.exists() else []
    sequence = len(previous) + 1
    if sequence > MAX_TIMELINE_ENTRIES:
        raise EvidenceError('timeline is saturated')
    for index, record in enumerate(previous, 1):
        if record.get('sequence') != index:
            raise EvidenceError('existing timeline sequence is not contiguous')
    record = {
        'schema_version': SCHEMA_VERSION,
        'sequence': sequence,
        'timestamp_utc': utc_now(),
        'monotonic_ns': time.monotonic_ns(),
        'kind': kind,
        'details': dict(details),
    }
    payload = canonical_json_bytes(record)
    if path.exists() and path.stat().st_size + len(payload) > MAX_JSON_BYTES:
        raise EvidenceError('timeline byte cap would be exceeded')
    path.parent.mkdir(parents=True, exist_ok=True)
    flags = os.O_WRONLY | os.O_CREAT | os.O_APPEND
    descriptor = os.open(path, flags, 0o640)
    try:
        written = os.write(descriptor, payload)
        if written != len(payload):
            raise EvidenceError('short timeline write')
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    return record


def probe_http(url: str, timeout_s: float = 1.0) -> tuple[int, str]:
    """Return a bounded HTTP status/body, including HTTP error responses."""
    request = urllib.request.Request(url, method='GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            body = response.read(65537)
            if len(body) > 65536:
                raise EvidenceError(f'HTTP body exceeded 65536 bytes: {url}')
            return int(response.status), body.decode('utf-8', errors='replace')
    except urllib.error.HTTPError as exc:
        body = exc.read(65537)
        if len(body) > 65536:
            raise EvidenceError(f'HTTP error body exceeded 65536 bytes: {url}') from exc
        return int(exc.code), body.decode('utf-8', errors='replace')
    except (OSError, urllib.error.URLError):
        return 0, ''


def fetch_http_json(url: str, output: Path, expected_status: int = 200) -> None:
    """Fetch one bounded HTTP JSON document and atomically retain it."""
    status, body = probe_http(url)
    if status != expected_status:
        raise EvidenceError(f'{url} returned HTTP {status}, expected {expected_status}')
    try:
        value = json.loads(body)
    except json.JSONDecodeError as exc:
        raise EvidenceError(f'{url} returned invalid JSON: {exc}') from exc
    if not isinstance(value, dict):
        raise EvidenceError(f'{url} did not return a JSON object')
    atomic_write_json(output, value)


def _read_proc_file(path: Path, *, binary: bool = False) -> bytes | str:
    if not path.is_file() or path.is_symlink():
        raise EvidenceError(f'process file is unavailable: {path}')
    if path.stat().st_size > MAX_PROC_FILE_BYTES:
        raise EvidenceError(f'process file exceeds cap: {path}')
    payload = path.read_bytes()
    if len(payload) > MAX_PROC_FILE_BYTES:
        raise EvidenceError(f'process file exceeds cap: {path}')
    if binary:
        return payload
    return payload.decode('utf-8', errors='strict')


def _parse_proc_stat(payload: str) -> dict[str, Any]:
    right = payload.rfind(')')
    left = payload.find('(')
    if left <= 0 or right <= left or not payload.endswith('\n'):
        raise EvidenceError('malformed /proc stat record')
    pid = int(payload[:left].strip())
    rest = payload[right + 2 :].split()
    if len(rest) < 20:
        raise EvidenceError('truncated /proc stat record')
    return {
        'pid': pid,
        'comm': payload[left + 1 : right],
        'state': rest[0],
        'ppid': int(rest[1]),
        'pgid': int(rest[2]),
        'start_time_ticks': int(rest[19]),
    }


def process_table(proc_root: Path = Path('/proc')) -> dict[int, dict[str, Any]]:
    """Read a bounded Linux process table, tolerating concurrent exits."""
    table: dict[int, dict[str, Any]] = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        if len(table) >= MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the process-count cap')
        try:
            stat_value = _parse_proc_stat(_read_proc_file(entry / 'stat'))
        except (EvidenceError, FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        table[stat_value['pid']] = stat_value
    return table


def descendant_pids(table: Mapping[int, Mapping[str, Any]], root_pid: int) -> set[int]:
    """Return the exact transitive descendant PID set including ``root_pid``."""
    if root_pid not in table:
        raise EvidenceError(f'root PID {root_pid} is absent')
    result = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, record in table.items():
            if pid not in result and record.get('ppid') in result:
                result.add(pid)
                changed = True
    return result


def process_lineage(
    table: Mapping[int, Mapping[str, Any]], target_pid: int, root_pid: int
) -> list[dict[str, int]]:
    """Return the exact root-to-target PID/start-time lineage."""
    if root_pid not in table or target_pid not in table:
        raise EvidenceError('lineage endpoint is absent from the process table')
    reverse: list[dict[str, int]] = []
    current = target_pid
    visited: set[int] = set()
    while True:
        if current in visited:
            raise EvidenceError('process lineage contains a cycle')
        visited.add(current)
        record = table.get(current)
        if record is None:
            raise EvidenceError('process lineage is incomplete')
        reverse.append(
            {
                'pid': _integer(record.get('pid'), 'lineage pid', minimum=1),
                'ppid': _integer(record.get('ppid'), 'lineage ppid', minimum=0),
                'start_time_ticks': _integer(
                    record.get('start_time_ticks'), 'lineage start time', minimum=0
                ),
            }
        )
        if current == root_pid:
            break
        current = int(record.get('ppid', -1))
    reverse.reverse()
    return reverse


def _process_environment(path: Path) -> dict[str, str]:
    payload = _read_proc_file(path / 'environ', binary=True)
    assert isinstance(payload, bytes)
    environment: dict[str, str] = {}
    for item in payload.split(b'\0'):
        if not item or b'=' not in item:
            continue
        raw_key, raw_value = item.split(b'=', 1)
        try:
            key = raw_key.decode('utf-8')
            value = raw_value.decode('utf-8')
        except UnicodeDecodeError:
            continue
        environment[key] = value
    return environment


def _process_details(pid: int, proc_root: Path = Path('/proc')) -> dict[str, Any]:
    root = proc_root / str(pid)
    stat_value = _parse_proc_stat(_read_proc_file(root / 'stat'))
    raw_cmdline = _read_proc_file(root / 'cmdline', binary=True)
    assert isinstance(raw_cmdline, bytes)
    cmdline = [
        item.decode('utf-8', errors='replace')
        for item in raw_cmdline.rstrip(b'\0').split(b'\0')
        if item
    ]
    cgroup = str(_read_proc_file(root / 'cgroup')).splitlines()
    try:
        executable = os.readlink(root / 'exe')
    except OSError:
        executable = ''
    environment = _process_environment(root)
    return {
        **stat_value,
        'cmdline': cmdline,
        'cmdline_sha256': hashlib.sha256(raw_cmdline).hexdigest(),
        'cgroup': cgroup,
        'executable': executable,
        'ros_domain_id': environment.get('ROS_DOMAIN_ID'),
        'gz_partition': environment.get('GZ_PARTITION'),
    }


def validate_controller_target_document(
    target: Mapping[str, Any],
) -> list[Mapping[str, Any]]:
    """Validate the exact retained controller identity and root-to-target lineage."""
    _exact_keys(target, CONTROLLER_TARGET_KEYS, 'controller target')
    for field in ('pid', 'pgid', 'root_pid', 'start_time_ticks'):
        _integer(target.get(field), f'controller target.{field}', minimum=1)
    _integer(target.get('ppid'), 'controller target.ppid', minimum=0)
    for field in ('comm', 'executable', 'gz_partition', 'ros_domain_id', 'state', 'unit'):
        _string(target.get(field), f'controller target.{field}')
    _sha256(target.get('cmdline_sha256'), 'controller target.cmdline_sha256')
    cmdline = target.get('cmdline')
    cgroup = target.get('cgroup')
    if not isinstance(cmdline, list) or not cmdline or len(cmdline) > 4096:
        raise EvidenceError('controller target.cmdline must be a nonempty bounded list')
    if not isinstance(cgroup, list) or not cgroup or len(cgroup) > 1024:
        raise EvidenceError('controller target.cgroup must be a nonempty bounded list')
    for index, value in enumerate(cmdline):
        _string(value, f'controller target.cmdline[{index}]')
    for index, value in enumerate(cgroup):
        _string(value, f'controller target.cgroup[{index}]')
    lineage = _mapping_list(target.get('lineage'), 'controller target.lineage')
    if len(lineage) < 2 or len(lineage) > MAX_PROCESS_COUNT:
        raise EvidenceError('controller target.lineage must be a bounded root-to-target path')
    lineage_pids: list[int] = []
    for index, row in enumerate(lineage):
        name = f'controller target.lineage[{index}]'
        _exact_keys(row, CONTROLLER_LINEAGE_KEYS, name)
        lineage_pids.append(_integer(row.get('pid'), f'{name}.pid', minimum=1))
        _integer(row.get('ppid'), f'{name}.ppid', minimum=0)
        _integer(row.get('start_time_ticks'), f'{name}.start_time_ticks', minimum=1)
    if len(lineage_pids) != len(set(lineage_pids)):
        raise EvidenceError('controller target.lineage repeats a PID')
    if any(
        lineage[index].get('ppid') != lineage[index - 1].get('pid')
        for index in range(1, len(lineage))
    ):
        raise EvidenceError('controller target.lineage is not parent-contiguous')
    if (
        lineage[0].get('pid') != target.get('root_pid')
        or lineage[-1].get('pid') != target.get('pid')
        or lineage[-1].get('ppid') != target.get('ppid')
        or lineage[-1].get('start_time_ticks') != target.get('start_time_ticks')
    ):
        raise EvidenceError('controller target.lineage endpoints do not match the identity')
    return lineage


def _in_unit_cgroup(lines: Sequence[str], unit: str) -> bool:
    suffix = f'/system.slice/{unit}'
    return any(line.rpartition(':')[2] == suffix for line in lines)


def _parse_cpu_list(value: Any, name: str) -> frozenset[int]:
    text = _string(value, name, maximum=4096)
    if CPU_LIST_RE.fullmatch(text) is None:
        raise EvidenceError(f'{name} is not a canonical Linux CPU list')
    cpus: set[int] = set()
    previous = -1
    for item in text.split(','):
        bounds = item.split('-', 1)
        start = int(bounds[0])
        end = int(bounds[-1])
        if end < start or start <= previous or end > 4095:
            raise EvidenceError(f'{name} is not strictly ordered and bounded')
        cpus.update(range(start, end + 1))
        previous = end
    if not cpus:
        raise EvidenceError(f'{name} is empty')
    canonical: list[str] = []
    ordered = sorted(cpus)
    start = previous = ordered[0]
    for cpu in ordered[1:]:
        if cpu == previous + 1:
            previous = cpu
            continue
        canonical.append(str(start) if start == previous else f'{start}-{previous}')
        start = previous = cpu
    canonical.append(str(start) if start == previous else f'{start}-{previous}')
    if text != ','.join(canonical):
        raise EvidenceError(f'{name} is not canonical')
    return frozenset(cpus)


def _parse_cpu_mask(value: Any, name: str) -> frozenset[int]:
    text = _string(value, name, maximum=4096)
    if CPU_MASK_RE.fullmatch(text) is None:
        raise EvidenceError(f'{name} is not a Linux hexadecimal CPU mask')
    mask = int(text.replace(',', ''), 16)
    if mask.bit_length() > 4096:
        raise EvidenceError(f'{name} exceeds the CPU ID bound')
    if mask <= 0:
        raise EvidenceError(f'{name} must enable at least one CPU')
    return frozenset(index for index in range(mask.bit_length()) if mask & (1 << index))


def _proc_affinity(path: Path) -> tuple[str, str, frozenset[int]]:
    status = str(_read_proc_file(path / 'status'))
    fields: dict[str, str] = {}
    for line in status.splitlines():
        key, separator, value = line.partition(':')
        if separator and key in {'Pid', 'PPid', 'Cpus_allowed', 'Cpus_allowed_list'}:
            fields[key] = value.strip()
    expected = {'Pid', 'PPid', 'Cpus_allowed', 'Cpus_allowed_list'}
    if set(fields) != expected:
        raise EvidenceError(f'process status affinity schema is incomplete: {path}')
    mask = fields['Cpus_allowed'].lower()
    cpu_list = fields['Cpus_allowed_list']
    mask_cpus = _parse_cpu_mask(mask, f'{path}.Cpus_allowed')
    list_cpus = _parse_cpu_list(cpu_list, f'{path}.Cpus_allowed_list')
    if mask_cpus != list_cpus:
        raise EvidenceError(f'CPU mask/list disagree for process {path.parent.name}')
    return mask, cpu_list, list_cpus


def _unit_process_pids(
    table: Mapping[int, Mapping[str, Any]],
    unit: str,
    proc_root: Path,
) -> set[int]:
    result: set[int] = set()
    for pid in sorted(table):
        try:
            lines = str(_read_proc_file(proc_root / str(pid) / 'cgroup')).splitlines()
        except (EvidenceError, FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if _in_unit_cgroup(lines, unit):
            result.add(pid)
    return result


def _runtime_affinity_process(
    pid: int,
    role: str,
    expected_cpus: frozenset[int],
    *,
    unit: str,
    proc_root: Path,
) -> dict[str, Any]:
    first = _process_details(pid, proc_root)
    first_mask, first_list, first_cpus = _proc_affinity(proc_root / str(pid))
    second = _process_details(pid, proc_root)
    second_mask, second_list, second_cpus = _proc_affinity(proc_root / str(pid))
    stable_fields = ('pid', 'ppid', 'pgid', 'start_time_ticks', 'executable', 'cgroup')
    if any(first.get(field) != second.get(field) for field in stable_fields):
        raise EvidenceError(f'process identity changed during affinity capture: PID {pid}')
    if (first_mask, first_list, first_cpus) != (second_mask, second_list, second_cpus):
        raise EvidenceError(f'CPU affinity changed during capture: PID {pid}')
    if first.get('state') == 'Z':
        raise EvidenceError(f'zombie process is present in the unit cgroup: PID {pid}')
    if not _in_unit_cgroup(first['cgroup'], unit):
        raise EvidenceError(f'process left the exact unit cgroup during capture: PID {pid}')
    if not first_cpus.issubset(expected_cpus):
        raise EvidenceError(f'process affinity exceeds the expected cpuset: PID {pid}')
    return {
        'role': role,
        'pid': first['pid'],
        'ppid': first['ppid'],
        'pgid': first['pgid'],
        'start_time_ticks': first['start_time_ticks'],
        'executable': first['executable'],
        'cgroup': first['cgroup'],
        'cpus_allowed': first_mask,
        'cpus_allowed_list': first_list,
    }


def capture_runtime_affinity(
    main_pid: int,
    managed_child_pid: int,
    managed_child_pgid: int,
    ros_domain_id: int,
    gz_partition: str,
    phase: str,
    *,
    unit: str = EXPECTED_UNIT,
    expected_cpuset: str = EXPECTED_CPUSET,
    proc_root: Path = Path('/proc'),
) -> dict[str, Any]:
    """Capture one stable, exact service-cgroup CPU-affinity snapshot."""
    if phase not in {'initial', 'restored'}:
        raise EvidenceError('runtime affinity phase must be initial or restored')
    for value, name in (
        (main_pid, 'main PID'),
        (managed_child_pid, 'managed child PID'),
        (managed_child_pgid, 'managed child PGID'),
    ):
        _integer(value, name, minimum=2)
    if managed_child_pid != managed_child_pgid:
        raise EvidenceError('managed child PID and PGID must be identical')
    expected_cpus = _parse_cpu_list(expected_cpuset, 'expected cpuset')
    table_before = process_table(proc_root)
    descendants_before = descendant_pids(table_before, main_pid)
    unit_pids_before = _unit_process_pids(table_before, unit, proc_root)
    if descendants_before != unit_pids_before:
        raise EvidenceError('MainPID descendants do not equal the exact unit-cgroup process set')
    if managed_child_pid not in descendants_before:
        raise EvidenceError('managed child is not a MainPID descendant in the unit cgroup')
    managed = table_before[managed_child_pid]
    if managed.get('pgid') != managed_child_pgid or managed.get('state') == 'Z':
        raise EvidenceError('managed child process-group identity is invalid')

    controller = find_exact_controller(
        managed_child_pid,
        managed_child_pgid,
        ros_domain_id,
        gz_partition,
        unit=unit,
        proc_root=proc_root,
    )
    controller_pid = _integer(controller.get('pid'), 'controller PID', minimum=2)
    records: list[dict[str, Any]] = []
    for pid in sorted(unit_pids_before):
        if pid == main_pid:
            role = 'supervisor_main'
        elif pid == managed_child_pid:
            role = 'managed_child'
        elif pid == controller_pid:
            role = 'controller'
        else:
            role = 'unit_descendant'
        records.append(
            _runtime_affinity_process(
                pid,
                role,
                expected_cpus,
                unit=unit,
                proc_root=proc_root,
            )
        )

    table_after = process_table(proc_root)
    descendants_after = descendant_pids(table_after, main_pid)
    unit_pids_after = _unit_process_pids(table_after, unit, proc_root)
    if descendants_after != descendants_before or unit_pids_after != unit_pids_before:
        raise EvidenceError('unit-cgroup process membership changed during affinity capture')
    for record in records:
        current = table_after.get(record['pid'])
        if current is None or any(
            current.get(field) != record[field]
            for field in ('pid', 'ppid', 'pgid', 'start_time_ticks')
        ):
            raise EvidenceError('process identity changed after affinity capture')

    return {
        'schema_version': SCHEMA_VERSION,
        'captured_utc': utc_now(),
        'phase': phase,
        'unit': unit,
        'unit_cgroup': f'/system.slice/{unit}',
        'expected_cpuset': expected_cpuset,
        'main_pid': main_pid,
        'managed_child_pid': managed_child_pid,
        'managed_child_pgid': managed_child_pgid,
        'controller_pid': controller_pid,
        'controller_count': 1,
        'process_count': len(records),
        'snapshot_stable': True,
        'processes': records,
        'verdict': 'PASS',
    }


def validate_runtime_affinity_evidence(
    value: Mapping[str, Any],
    *,
    phase: str,
    main_pid: int,
    managed_child_pid: int,
    managed_child_pgid: int,
    expected_cpuset: str = EXPECTED_CPUSET,
    unit: str = EXPECTED_UNIT,
) -> dict[str, Any]:
    """Validate one retained affinity snapshot and return its exact role join."""
    _exact_keys(value, RUNTIME_AFFINITY_KEYS, f'{phase} runtime affinity')
    captured_utc = _utc_timestamp(value.get('captured_utc'), f'{phase} affinity captured_utc')
    if value.get('phase') != phase:
        raise EvidenceError(f'{phase} affinity phase does not match its evidence role')
    if value.get('unit') != unit or value.get('unit_cgroup') != f'/system.slice/{unit}':
        raise EvidenceError(f'{phase} affinity unit cgroup is not exact')
    if value.get('expected_cpuset') != expected_cpuset:
        raise EvidenceError(f'{phase} affinity expected cpuset changed')
    expected_cpus = _parse_cpu_list(expected_cpuset, f'{phase} expected cpuset')
    if value.get('snapshot_stable') is not True or value.get('verdict') != 'PASS':
        raise EvidenceError(f'{phase} affinity snapshot was not captured stable and PASS')
    if value.get('main_pid') != main_pid:
        raise EvidenceError(f'{phase} affinity MainPID does not reconcile')
    if value.get('managed_child_pid') != managed_child_pid:
        raise EvidenceError(f'{phase} affinity managed child PID does not reconcile')
    if value.get('managed_child_pgid') != managed_child_pgid:
        raise EvidenceError(f'{phase} affinity managed child PGID does not reconcile')
    if managed_child_pid != managed_child_pgid:
        raise EvidenceError(f'{phase} managed child PID/PGID is not an owned group')

    raw_processes = value.get('processes')
    if isinstance(raw_processes, (str, bytes)) or not isinstance(raw_processes, Sequence):
        raise EvidenceError(f'{phase} affinity processes must be a sequence')
    if not 3 <= len(raw_processes) <= MAX_PROCESS_COUNT:
        raise EvidenceError(f'{phase} affinity process count is outside the bound')
    if value.get('process_count') != len(raw_processes):
        raise EvidenceError(f'{phase} affinity process_count does not reconcile')
    if value.get('controller_count') != 1:
        raise EvidenceError(f'{phase} affinity controller_count must be exactly one')

    roles: dict[str, list[Mapping[str, Any]]] = {
        'supervisor_main': [],
        'managed_child': [],
        'controller': [],
        'unit_descendant': [],
    }
    processes: list[Mapping[str, Any]] = []
    pids: list[int] = []
    affinity_limited = True
    for index, raw_process in enumerate(raw_processes):
        process = _mapping(raw_process, f'{phase} affinity processes[{index}]')
        _exact_keys(
            process,
            RUNTIME_AFFINITY_PROCESS_KEYS,
            f'{phase} affinity processes[{index}]',
        )
        role = _string(process.get('role'), f'{phase} affinity role')
        if role not in roles:
            raise EvidenceError(f'{phase} affinity process has an unsupported role: {role}')
        pid = _integer(process.get('pid'), f'{phase} affinity PID', minimum=2)
        _integer(process.get('ppid'), f'{phase} affinity PPID', minimum=0)
        _integer(process.get('pgid'), f'{phase} affinity PGID', minimum=1)
        _integer(
            process.get('start_time_ticks'),
            f'{phase} affinity start_time_ticks',
            minimum=1,
        )
        executable = _string(process.get('executable'), f'{phase} affinity executable')
        if not Path(executable).is_absolute():
            raise EvidenceError(f'{phase} affinity executable is not absolute')
        raw_cgroup = process.get('cgroup')
        if isinstance(raw_cgroup, (str, bytes)) or not isinstance(raw_cgroup, Sequence):
            raise EvidenceError(f'{phase} affinity cgroup must be a raw line sequence')
        cgroup = [_string(item, f'{phase} affinity cgroup line') for item in raw_cgroup]
        if not cgroup or not _in_unit_cgroup(cgroup, unit):
            raise EvidenceError(f'{phase} affinity process is outside the exact unit cgroup')
        mask_cpus = _parse_cpu_mask(process.get('cpus_allowed'), f'{phase} affinity Cpus_allowed')
        list_cpus = _parse_cpu_list(
            process.get('cpus_allowed_list'), f'{phase} affinity Cpus_allowed_list'
        )
        if mask_cpus != list_cpus:
            raise EvidenceError(f'{phase} affinity CPU mask/list do not reconcile')
        affinity_limited = affinity_limited and list_cpus.issubset(expected_cpus)
        roles[role].append(process)
        processes.append(process)
        pids.append(pid)

    if pids != sorted(pids) or len(set(pids)) != len(pids):
        raise EvidenceError(f'{phase} affinity process PIDs are not sorted and unique')
    for role in ('supervisor_main', 'managed_child', 'controller'):
        if len(roles[role]) != 1:
            raise EvidenceError(f'{phase} affinity must contain exactly one {role}')
    main = roles['supervisor_main'][0]
    child = roles['managed_child'][0]
    controller = roles['controller'][0]
    if main.get('pid') != main_pid:
        raise EvidenceError(f'{phase} affinity main role does not match MainPID')
    if child.get('pid') != managed_child_pid or child.get('pgid') != managed_child_pgid:
        raise EvidenceError(f'{phase} affinity child role does not match managed identity')
    if child.get('ppid') != main_pid:
        raise EvidenceError(f'{phase} managed child is not a direct MainPID child')
    if controller.get('pid') != value.get('controller_pid'):
        raise EvidenceError(f'{phase} affinity controller PID does not reconcile')
    if controller.get('pgid') != managed_child_pgid:
        raise EvidenceError(f'{phase} affinity controller left the managed group')
    if Path(str(controller.get('executable'))).name != 'controller_server':
        raise EvidenceError(f'{phase} affinity controller executable is not exact')
    by_pid = {process['pid']: process for process in processes}
    for process in processes:
        pid = process['pid']
        if pid == main_pid:
            continue
        if process.get('pgid') != managed_child_pgid:
            raise EvidenceError(f'{phase} affinity process left the managed group')
        current = pid
        visited: set[int] = set()
        while current != main_pid:
            if current in visited or current not in by_pid:
                raise EvidenceError(f'{phase} affinity process {pid} does not descend from MainPID')
            visited.add(current)
            current = by_pid[current]['ppid']
    return {
        'limited': affinity_limited,
        'captured_utc': datetime.fromisoformat(captured_utc.replace('Z', '+00:00')),
        'main': main,
        'child': child,
        'controller': controller,
        'processes': processes,
    }


def validate_supervisor_status(value: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    """Validate one exact supervisor snapshot and return its only child."""
    _exact_keys(value, SUPERVISOR_STATUS_KEYS, name)
    if _integer(value.get('schema_version'), f'{name}.schema_version') != 1:
        raise EvidenceError(f'{name}.schema_version must be 1')
    _utc_timestamp(value.get('supervisor_utc'), f'{name}.supervisor_utc')
    for field in ('healthy', 'persistence_healthy', 'ready', 'shutting_down'):
        _boolean(value.get(field), f'{name}.{field}')
    _integer(value.get('event_count'), f'{name}.event_count', minimum=0)
    _integer(value.get('dropped_events'), f'{name}.dropped_events', minimum=0)
    children = _mapping_list(value.get('children'), f'{name}.children')
    if len(children) != 1:
        raise EvidenceError(f'{name} must contain exactly one child')
    child = children[0]
    keys = set(child)
    if not keys >= SUPERVISOR_CHILD_REQUIRED_KEYS or not keys <= (
        SUPERVISOR_CHILD_REQUIRED_KEYS | SUPERVISOR_CHILD_OPTIONAL_KEYS
    ):
        raise EvidenceError(f'{name}.children[0] has an invalid schema')
    if child.get('name') != EXPECTED_CHILD:
        raise EvidenceError(f'{name} child name is not exact')
    for field in ('required', 'running', 'heartbeat_fresh', 'circuit_open'):
        _boolean(child.get(field), f'{name}.child.{field}')
    for field in ('pid', 'pgid', 'restart_count'):
        _integer(child.get(field), f'{name}.child.{field}', minimum=0)
    heartbeat_age = child.get('heartbeat_age_ms')
    if heartbeat_age is not None:
        _integer(heartbeat_age, f'{name}.child.heartbeat_age_ms', minimum=0)
    if 'last_exit_code' in child:
        _integer(child.get('last_exit_code'), f'{name}.child.last_exit_code')
    for field in ('last_failure_utc', 'last_ready_utc', 'started_utc'):
        if field in child:
            _utc_timestamp(child.get(field), f'{name}.child.{field}')
    if 'last_failure_kind' in child:
        _string(child.get('last_failure_kind'), f'{name}.child.last_failure_kind')
    return child


def validate_systemd_owners(
    value: Mapping[str, Any],
    *,
    ros_domain_id: int,
    gz_partition: str,
    expected_pids: list[int],
    name: str,
) -> None:
    _exact_keys(value, SYSTEMD_OWNERS_KEYS, name)
    if _integer(value.get('schema_version'), f'{name}.schema_version') != 1:
        raise EvidenceError(f'{name}.schema_version must be 1')
    _utc_timestamp(value.get('captured_utc'), f'{name}.captured_utc')
    units = _mapping(value.get('units'), f'{name}.units')
    expected_units = {EXPECTED_UNIT: sorted(expected_pids)}
    expected = {
        'captured_utc': value.get('captured_utc'),
        'gz_partition': gz_partition,
        'ros_domain_id': ros_domain_id,
        'schema_version': 1,
        'unit_count': 1,
        'units': expected_units,
        'verdict': 'PASS',
    }
    if not _type_exact_json_equal(value, expected) or not units[EXPECTED_UNIT]:
        raise EvidenceError(f'{name} does not match the exact isolated unit PID set')


def validate_stable_runtime_ownership_capture(
    before: Mapping[str, Any],
    after: Mapping[str, Any],
    affinity: Mapping[str, Any],
) -> None:
    """Require an unchanged owner scan bracketing one affinity snapshot."""
    for name, value in (('owners before', before), ('owners after', after)):
        _exact_keys(value, SYSTEMD_OWNERS_KEYS, name)
        if _integer(value.get('schema_version'), f'{name}.schema_version') != 1:
            raise EvidenceError(f'{name}.schema_version must be 1')
        _utc_timestamp(value.get('captured_utc'), f'{name}.captured_utc')
    before_projection = {key: value for key, value in before.items() if key != 'captured_utc'}
    after_projection = {key: value for key, value in after.items() if key != 'captured_utc'}
    if not _type_exact_json_equal(before_projection, after_projection):
        raise EvidenceError('systemd owner set changed across affinity capture')
    units = _mapping(after.get('units'), 'stable systemd owner units')
    expected_pids = sorted(
        _integer(process.get('pid'), 'stable affinity PID', minimum=1)
        for process in _mapping_list(affinity.get('processes'), 'stable affinity processes')
        if process.get('role') != 'supervisor_main'
    )
    if (
        after.get('verdict') != 'PASS'
        or after.get('unit_count') != 1
        or not _type_exact_json_equal(units, {EXPECTED_UNIT: expected_pids})
    ):
        raise EvidenceError('stable systemd owner set does not match affinity descendants')


def lifecycle_startup_result_is_exact(value: Mapping[str, Any]) -> bool:
    """Return whether the retained launch-owned lifecycle STARTUP result is exact."""
    _exact_keys(value, LIFECYCLE_STARTUP_RESULT_KEYS, 'lifecycle startup result')
    schema_version = _integer(value.get('schema_version'), 'startup schema_version')
    verdict = _string(value.get('verdict'), 'startup verdict')
    accepted = _boolean(value.get('accepted'), 'startup accepted')
    exit_code = _integer(value.get('exit_code'), 'startup exit_code')
    service_name = _string(value.get('service_name'), 'startup service_name')
    command = _integer(value.get('command'), 'startup command')
    discovery_grace = _number(
        value.get('discovery_grace_sec'), 'startup discovery_grace_sec', minimum=0.0
    )
    service_timeout = _number(
        value.get('service_timeout_sec'), 'startup service_timeout_sec', minimum=0.0
    )
    response_timeout = _number(
        value.get('response_timeout_sec'), 'startup response_timeout_sec', minimum=0.0
    )
    started = _utc_timestamp(value.get('started_utc'), 'startup result started_utc')
    completed = _utc_timestamp(value.get('completed_utc'), 'startup result completed_utc')
    started_at = datetime.fromisoformat(started.replace('Z', '+00:00'))
    completed_at = datetime.fromisoformat(completed.replace('Z', '+00:00'))
    elapsed = _number(value.get('elapsed_wall_sec'), 'startup elapsed_wall_sec', minimum=0.0)
    return (
        schema_version == 1
        and verdict == 'PASS'
        and accepted is True
        and exit_code == 0
        and service_name == '/robotest/lifecycle_manager_navigation/manage_nodes'
        and command == 0
        and 0.0 <= elapsed <= 110.0
        and discovery_grace == 4.0
        and service_timeout == 20.0
        and response_timeout == 60.0
        and value.get('watch_pid') is None
        and value.get('failure_kind') is None
        and value.get('failure_message') is None
        and completed_at >= started_at
    )


def find_exact_controller(
    root_pid: int,
    pgid: int,
    ros_domain_id: int,
    gz_partition: str,
    *,
    unit: str = EXPECTED_UNIT,
    proc_root: Path = Path('/proc'),
) -> dict[str, Any]:
    """Resolve one nested Nav2 controller using lineage, PGID, cgroup, and env."""
    table = process_table(proc_root)
    descendants = descendant_pids(table, root_pid)
    candidates: list[dict[str, Any]] = []
    for pid in sorted(descendants - {root_pid}):
        if table[pid].get('pgid') != pgid or table[pid].get('state') == 'Z':
            continue
        try:
            details = _process_details(pid, proc_root)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not (proc_root / str(pid)).exists():
                continue
            raise EvidenceError(f'cannot inspect descendant PID {pid}: {exc}') from exc
        if details.get('state') == 'Z' or details.get('pgid') != pgid:
            continue
        executable_name = Path(details['executable']).name
        if executable_name != 'controller_server':
            continue
        if details['ros_domain_id'] != str(ros_domain_id):
            continue
        if details['gz_partition'] != gz_partition:
            continue
        if not _in_unit_cgroup(details['cgroup'], unit):
            continue
        candidates.append(details)
    if len(candidates) != 1:
        raise EvidenceError(f'expected one exact nested controller_server, found {len(candidates)}')
    candidate = candidates[0]
    candidate['root_pid'] = root_pid
    candidate['unit'] = unit
    candidate['lineage'] = process_lineage(table, int(candidate['pid']), root_pid)
    if table[root_pid].get('state') == 'Z' or table[root_pid].get('pgid') != pgid:
        raise EvidenceError('managed root is zombie or left its captured process group')
    return candidate


def assert_process_identity(
    identity: Mapping[str, Any], *, proc_root: Path = Path('/proc')
) -> dict[str, Any]:
    """Re-resolve a captured PID and reject PID reuse or identity drift."""
    pid = _integer(identity.get('pid'), 'identity.pid', minimum=1)
    root_pid = _integer(identity.get('root_pid'), 'identity.root_pid', minimum=1)
    expected_pgid = _integer(identity.get('pgid'), 'identity.pgid', minimum=1)
    if Path(_string(identity.get('executable'), 'identity.executable')).name != (
        'controller_server'
    ):
        raise EvidenceError('captured executable is not exactly controller_server')
    table = process_table(proc_root)
    current_lineage = process_lineage(table, pid, root_pid)
    expected_lineage = [
        dict(item) for item in _mapping_list(identity.get('lineage'), 'identity.lineage')
    ]
    current = _process_details(pid, proc_root)
    fields = (
        'start_time_ticks',
        'pgid',
        'executable',
        'cmdline_sha256',
        'ros_domain_id',
        'gz_partition',
    )
    mismatches = [name for name in fields if current.get(name) != identity.get(name)]
    if current.get('cgroup') != identity.get('cgroup'):
        mismatches.append('cgroup')
    if current_lineage != expected_lineage:
        mismatches.append('lineage')
    if current.get('state') == 'Z':
        mismatches.append('zombie')
    if Path(str(current.get('executable', ''))).name != 'controller_server':
        mismatches.append('executable_basename')
    root = table.get(root_pid)
    if root is None or root.get('state') == 'Z':
        mismatches.append('root_state')
    elif root.get('pgid') != expected_pgid:
        mismatches.append('root_pgid')
    unit = _string(identity.get('unit'), 'identity.unit')
    if not _in_unit_cgroup(current['cgroup'], unit):
        mismatches.append('unit_cgroup')
    if mismatches:
        raise EvidenceError(f'captured process identity changed: {sorted(mismatches)}')
    return current


def signal_process_identity(
    identity: Mapping[str, Any],
    signum: int,
    *,
    proc_root: Path = Path('/proc'),
    pidfd_opener: Any = _DEFAULT_API,
    pidfd_signaler: Any = _DEFAULT_API,
    fd_closer: Any = _DEFAULT_API,
) -> int:
    """Pin, revalidate, and signal the exact task through one pidfd."""
    if signum not in {signal.SIGTERM, signal.SIGKILL}:
        raise EvidenceError(f'unsupported identity signal: {signum}')
    if pidfd_opener is _DEFAULT_API:
        pidfd_opener = getattr(os, 'pidfd_open', None)
    if pidfd_signaler is _DEFAULT_API:
        pidfd_signaler = getattr(signal, 'pidfd_send_signal', None)
    if fd_closer is _DEFAULT_API:
        fd_closer = os.close
    if not callable(pidfd_opener) or not callable(pidfd_signaler) or not callable(fd_closer):
        raise EvidenceError('pidfd signaling APIs are unavailable; numeric signaling is forbidden')
    pid = _integer(identity.get('pid'), 'identity.pid', minimum=1)
    try:
        descriptor = pidfd_opener(pid, 0)
    except (OSError, TypeError, ValueError) as exc:
        raise EvidenceError(f'could not open pidfd for exact PID {pid}: {exc}') from exc
    descriptor = _integer(descriptor, 'pidfd descriptor', minimum=0)
    try:
        current = assert_process_identity(identity, proc_root=proc_root)
        if _integer(current.get('pid'), 'current identity PID', minimum=1) != pid:
            raise EvidenceError('pidfd-pinned task does not match the captured PID')
        try:
            pidfd_signaler(descriptor, signum, None, 0)
        except (OSError, TypeError, ValueError) as exc:
            raise EvidenceError(f'could not signal pidfd for exact PID {pid}: {exc}') from exc
    finally:
        try:
            fd_closer(descriptor)
        except (OSError, TypeError, ValueError) as exc:
            raise EvidenceError(f'could not close pidfd for exact PID {pid}: {exc}') from exc
    return pid


def process_group_members(pgid: int, proc_root: Path = Path('/proc')) -> list[dict[str, Any]]:
    """Return all members of an exact Linux process group, including zombies."""
    if pgid <= 0:
        raise EvidenceError('PGID must be positive')
    return [
        dict(record)
        for _pid, record in sorted(process_table(proc_root).items())
        if record.get('pgid') == pgid
    ]


def systemd_owned_units(
    ros_domain_id: int,
    gz_partition: str,
    proc_root: Path = Path('/proc'),
) -> dict[str, list[int]]:
    """Map matching isolated processes to exact system-slice service units."""
    units: dict[str, list[int]] = {}
    unit_pattern = re.compile(r'/system\.slice/([^/]+\.service)$')
    inspected = 0
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        inspected += 1
        if inspected > MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the systemd-owner scan cap')
        try:
            environment = _process_environment(entry)
            if (
                environment.get('ROS_DOMAIN_ID') != str(ros_domain_id)
                or environment.get('GZ_PARTITION') != gz_partition
            ):
                continue
            cgroup = str(_read_proc_file(entry / 'cgroup')).splitlines()
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not entry.exists():
                continue
            raise EvidenceError(
                f'cannot inspect PID {entry.name} for systemd ownership: {exc}'
            ) from exc
        for line in cgroup:
            match = unit_pattern.search(line)
            if match is not None:
                units.setdefault(match.group(1), []).append(int(entry.name))
    return {unit: sorted(set(pids)) for unit, pids in sorted(units.items())}


def allocated_isolation(run_id: str, proc_root: Path = Path('/proc')) -> dict[str, Any]:
    """Select an unused bounded ROS domain and unique Gazebo partition."""
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise EvidenceError('run ID does not match the Phase 4 format')
    used_domains: set[int] = set()
    used_partitions: set[str] = set()
    inspected = 0
    unreadable = 0
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        inspected += 1
        if inspected > MAX_PROCESS_COUNT:
            raise EvidenceError('process table exceeds the isolation-scan cap')
        try:
            environment = _process_environment(entry)
        except (FileNotFoundError, ProcessLookupError):
            continue
        except (EvidenceError, PermissionError) as exc:
            if not entry.exists():
                continue
            unreadable += 1
            raise EvidenceError(
                f'cannot prove isolation because PID {entry.name} is unreadable: {exc}'
            ) from exc
        raw_domain = environment.get('ROS_DOMAIN_ID')
        if raw_domain is not None and raw_domain.isdigit():
            used_domains.add(int(raw_domain))
        partition = environment.get('GZ_PARTITION')
        if partition:
            used_partitions.add(partition)
    seed = int(hashlib.sha256(run_id.encode()).hexdigest()[:8], 16)
    candidates = list(range(100, 230))
    candidates = candidates[seed % len(candidates) :] + candidates[: seed % len(candidates)]
    try:
        domain = next(value for value in candidates if value not in used_domains)
    except StopIteration as exc:
        raise EvidenceError('no unused ROS domain is available in 100..229') from exc
    partition = f'robotest_phase4_{run_id.replace("-", "_")}'
    if PARTITION_RE.fullmatch(partition) is None or partition in used_partitions:
        raise EvidenceError('generated Gazebo partition is invalid or already active')
    return {
        'schema_version': SCHEMA_VERSION,
        'run_id': run_id,
        'ros_domain_id': domain,
        'gz_partition': partition,
        'inspected_processes': inspected,
        'unreadable_process_environments': unreadable,
        'domain_was_unused': True,
        'partition_was_unused': True,
    }


def expected_supervisor_config(
    state_directory: str,
    ros_domain_id: int,
    gz_partition: str,
) -> dict[str, Any]:
    """Return the one complete type-exact packaged supervisor configuration."""
    return {
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
                'name': EXPECTED_CHILD,
                'argv': ['/usr/libexec/robotest-supervisor/start-robotest-stack'],
                'working_directory': '/opt/robotest-lab',
                'environment': {
                    'GZ_PARTITION': gz_partition,
                    'RCUTILS_LOGGING_BUFFERED_STREAM': '1',
                    'ROBOTEST_CPUSET': EXPECTED_CPUSET,
                    'ROBOTEST_NAMESPACE': 'robotest',
                    RUNTIME_STATE_ENVIRONMENT: state_directory,
                    'ROS_DOMAIN_ID': str(ros_domain_id),
                },
                'required': True,
                'heartbeat_file': f'{state_directory}/{HEARTBEAT_NAME}',
            }
        ],
    }


def render_overlay_stage_script(
    template_path: Path,
    output_path: Path,
    project_root: Path,
    evidence_directory: Path,
) -> None:
    """Render the overlay helper with its only writable evidence path under /run."""
    for path, label in (
        (project_root, 'overlay project root'),
        (evidence_directory, 'overlay evidence directory'),
        (output_path, 'overlay helper output'),
    ):
        if not path.is_absolute() or '..' in path.parts:
            raise EvidenceError(f'{label} must be a canonical absolute path')
    if project_root.resolve(strict=True) != project_root:
        raise EvidenceError('overlay project root must not traverse symbolic links')
    payload = _read_bounded_bytes(template_path, MAX_TEXT_BYTES)
    try:
        text = payload.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise EvidenceError('overlay helper template is not UTF-8') from exc
    replacements = {
        'SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"': (
            f'SCRIPT_DIR={shlex.quote(str(project_root / "scripts"))}'
        ),
        'PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd -P)"': (
            f'PROJECT_ROOT={shlex.quote(str(project_root))}'
        ),
        'readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase4"': (
            f'readonly EVIDENCE_DIR={shlex.quote(str(evidence_directory))}'
        ),
    }
    for original, replacement in replacements.items():
        if text.count(original) != 1:
            raise EvidenceError(f'overlay helper assignment changed: {original}')
        text = text.replace(original, replacement, 1)
    _atomic_write(output_path, text.encode('utf-8'), mode=0o700)


def render_supervisor_config(
    template_path: Path,
    output_path: Path,
    state_directory: str,
    ros_domain_id: int,
    gz_partition: str,
) -> dict[str, Any]:
    """Render only run-scoped isolation/state values into the frozen config."""
    value = _mapping(load_json(template_path), 'supervisor template')
    expected_template = expected_supervisor_config(
        EXPECTED_RUNTIME_STATE_ROOT,
        42,
        'robotest_supervised',
    )
    if not _type_exact_json_equal(value, expected_template):
        raise EvidenceError('package config changed the complete frozen contract')
    state_path = Path(state_directory)
    if (
        state_path.as_posix() != state_directory
        or state_path.parent.as_posix() != EXPECTED_RUNTIME_STATE_ROOT
        or RUN_ID_RE.fullmatch(state_path.name) is None
    ):
        raise EvidenceError('run state directory is outside the Phase 4-owned prefix')
    if not 1 <= ros_domain_id <= 232:
        raise EvidenceError('ROS domain ID is outside the DDS range')
    if PARTITION_RE.fullmatch(gz_partition) is None:
        raise EvidenceError('Gazebo partition is invalid')
    rendered = expected_supervisor_config(state_directory, ros_domain_id, gz_partition)
    atomic_write_json(output_path, rendered, mode=0o640)
    return rendered


def expected_followup_mission() -> dict[str, Any]:
    """Return the one exact fresh one-waypoint recovery mission."""
    return {
        'schema_version': 1,
        'mission_name': 'phase4_recovery_followup',
        'mission_seed': 42,
        'simulator_seed': 42,
        'frame_id': 'map',
        'start_pose': {'x': 0.0, 'y': -3.5, 'yaw': 0.0},
        'waypoints': [{'x': -0.5, 'y': -3.5, 'yaw': 0.0}],
        'mission_timeout_sim_s': 60.0,
        'wall_escape_timeout_s': 120.0,
        'allowed_collision_count': 0,
        'fault_schedule': None,
        'fault_seed': None,
        'expected_outcome': 'succeeded',
        'retries': 0,
    }


def render_followup_mission(output_path: Path) -> dict[str, Any]:
    """Write the one-goal fresh recovery mission as immutable canonical JSON."""
    mission = expected_followup_mission()
    atomic_write_json(output_path, mission)
    return mission


def _manifest_source_mappings(repository: Path) -> tuple[tuple[Path, Path], ...]:
    return (
        (repository / 'supervisor/cmd', Path('cmd')),
        (repository / 'supervisor/internal', Path('internal')),
        (repository / 'supervisor/go.mod', Path('go.mod')),
        (repository / 'supervisor/README.md', Path('README.md')),
        (repository / 'supervisor/config.example.json', Path('config.example.json')),
        (repository / 'LICENSE', Path('LICENSE')),
        (repository / 'packaging/debian', Path('debian')),
    )


def repository_package_manifest(repository: Path) -> dict[str, Any]:
    """Rebuild the package builder's exact source manifest."""
    entries: list[dict[str, Any]] = []
    for source, destination in _manifest_source_mappings(repository):
        if source.is_dir() and not source.is_symlink():
            paths = sorted(path for path in source.rglob('*') if path.is_file())
            if any(path.is_symlink() for path in source.rglob('*')):
                raise EvidenceError(f'package source contains a symbolic link: {source}')
            for path in paths:
                metadata = path.stat()
                entries.append(
                    {
                        'mode': format(stat.S_IMODE(metadata.st_mode), '04o'),
                        'path': (destination / path.relative_to(source)).as_posix(),
                        'sha256': file_sha256(path),
                        'size_bytes': metadata.st_size,
                    }
                )
        else:
            if source.is_symlink() or not source.is_file():
                raise EvidenceError(f'missing package source: {source}')
            metadata = source.stat()
            entries.append(
                {
                    'mode': format(stat.S_IMODE(metadata.st_mode), '04o'),
                    'path': destination.as_posix(),
                    'sha256': file_sha256(source),
                    'size_bytes': metadata.st_size,
                }
            )
    return {'schema_version': 1, 'files': sorted(entries, key=lambda item: item['path'])}


def runtime_source_manifest(repository: Path) -> dict[str, Any]:
    """Replay the runtime overlay source-manifest algorithm exactly."""
    repository = repository.resolve(strict=True)
    entries: list[dict[str, Any]] = []
    for name in ('config', 'scenarios', 'src'):
        runtime_root = repository / name
        if runtime_root.is_symlink() or not runtime_root.is_dir():
            raise EvidenceError(f'missing regular runtime source directory: {runtime_root}')
        for directory, directory_names, file_names in os.walk(runtime_root, followlinks=False):
            directory_path = Path(directory)
            for directory_name in directory_names:
                candidate = directory_path / directory_name
                if candidate.is_symlink():
                    raise EvidenceError(
                        f'runtime source contains a symbolic-link directory: {candidate}'
                    )
            directory_names[:] = sorted(
                directory_name
                for directory_name in directory_names
                if directory_name not in {'__pycache__', '.pytest_cache', '.ruff_cache'}
            )
            for file_name in sorted(file_names):
                if file_name.endswith(('.pyc', '.pyo')):
                    continue
                path = directory_path / file_name
                if path.is_symlink():
                    raise EvidenceError(f'runtime source contains a symbolic link: {path}')
                relative = path.relative_to(repository).as_posix()
                if relative == 'config/release-claims.json':
                    continue
                resolved = path.resolve(strict=True)
                if repository not in resolved.parents or not resolved.is_file():
                    raise EvidenceError(f'invalid runtime source entry: {path}')
                metadata = resolved.stat()
                entries.append(
                    {
                        'mode': format(stat.S_IMODE(metadata.st_mode), '04o'),
                        'path': relative,
                        'sha256': file_sha256(resolved),
                        'size_bytes': metadata.st_size,
                    }
                )
                if len(entries) > MAX_SOURCE_FILES:
                    raise EvidenceError('runtime source manifest exceeds its file-count cap')
    return {'schema_version': 1, 'files': sorted(entries, key=lambda item: item['path'])}


def _canonical_manifest_document(path: Path, name: str) -> Mapping[str, Any]:
    payload = _read_bounded_bytes(path, MAX_JSON_BYTES)
    document = _mapping(load_json(path), name)
    _exact_keys(document, SOURCE_MANIFEST_KEYS, name)
    if _integer(document.get('schema_version'), f'{name}.schema_version') != 1:
        raise EvidenceError(f'{name}.schema_version must be 1')
    rows = _mapping_list(document.get('files'), f'{name}.files')
    if not rows or len(rows) > MAX_SOURCE_FILES:
        raise EvidenceError(f'{name}.files must be nonempty and bounded')
    paths: list[str] = []
    for index, row in enumerate(rows):
        row_name = f'{name}.files[{index}]'
        _exact_keys(row, SOURCE_MANIFEST_FILE_KEYS, row_name)
        mode = _string(row.get('mode'), f'{row_name}.mode', maximum=4)
        relative = _string(row.get('path'), f'{row_name}.path')
        if re.fullmatch(r'[0-7]{4}', mode) is None:
            raise EvidenceError(f'{row_name}.mode is not canonical')
        if (
            relative.startswith('/')
            or '\\' in relative
            or Path(relative).as_posix() != relative
            or any(part in ('', '.', '..') for part in Path(relative).parts)
        ):
            raise EvidenceError(f'{row_name}.path is not a canonical relative path')
        _sha256(row.get('sha256'), f'{row_name}.sha256')
        _integer(row.get('size_bytes'), f'{row_name}.size_bytes', minimum=0)
        paths.append(relative)
    if paths != sorted(paths) or len(paths) != len(set(paths)):
        raise EvidenceError(f'{name}.files paths are not sorted and unique')
    if payload != canonical_json_bytes(document):
        raise EvidenceError(f'{name} bytes are not canonical')
    return document


def runtime_staging_evidence_is_exact(
    repository: Path,
    context: Mapping[str, Any],
    staging: Mapping[str, Any],
    overlay_provenance: Mapping[str, Any],
    source_manifest_path: Path,
    install_manifest_path: Path,
) -> bool:
    """Replay the exact source, install, provenance, and active-target joins."""
    try:
        repository = repository.resolve(strict=True)
        _exact_keys(staging, OVERLAY_PROVENANCE_KEYS, 'runtime staging provenance')
        if not _type_exact_json_equal(staging, overlay_provenance):
            return False
        if _integer(staging.get('schema_version'), 'runtime staging schema_version') != 1:
            return False
        _utc_timestamp(staging.get('created_utc'), 'runtime staging created_utc')
        git_commit = _string(staging.get('git_commit'), 'runtime staging git_commit')
        git_dirty = _boolean(staging.get('git_dirty'), 'runtime staging git_dirty')
        if re.fullmatch(r'[0-9a-f]{40}', git_commit) is None:
            return False
        expected_source = runtime_source_manifest(repository)
        if _read_bounded_bytes(source_manifest_path, MAX_JSON_BYTES) != canonical_json_bytes(
            expected_source
        ):
            return False
        _canonical_manifest_document(install_manifest_path, 'overlay install manifest')
        source_sha = file_sha256(source_manifest_path)
        install_sha = file_sha256(install_manifest_path)
        release_id = f'{git_commit[:12]}-{source_sha[:16]}-{str(git_dirty).lower()}'
        release_path = f'/opt/robotest-lab-releases/{release_id}'
        expected_build_command = [
            'colcon',
            '--log-base',
            f'{release_path}/.log',
            'build',
            '--base-paths',
            f'{repository}/src',
            '--build-base',
            f'{release_path}/.build',
            '--install-base',
            f'{release_path}/install',
            '--executor',
            'parallel',
            '--parallel-workers',
            '4',
            '--event-handlers',
            'console_direct+',
            '--cmake-args',
            '-DBUILD_TESTING=OFF',
        ]
        expected_provenance = {
            'active_path': '/opt/robotest-lab',
            'build_command': expected_build_command,
            'created_utc': staging.get('created_utc'),
            'git_commit': git_commit,
            'git_dirty': git_dirty,
            'install_manifest_sha256': install_sha,
            'packages': OVERLAY_PACKAGES,
            'release_id': release_id,
            'schema_version': 1,
            'source_manifest_sha256': source_sha,
            'source_workspace': str(repository),
        }
        return (
            _type_exact_json_equal(staging, expected_provenance)
            and git_dirty is False
            and context.get('source_git_commit') == git_commit
            and context.get('source_git_dirty') is False
            and context.get('active_overlay_target') == release_path
        )
    except (EvidenceError, OSError, RuntimeError):
        return False


def source_snapshot(repository: Path) -> dict[str, Any]:
    """Hash the bounded implementation/contract source set used by Phase 4."""
    roots = ('config', 'docs', 'packaging', 'scenarios', 'scripts', 'src', 'supervisor', 'tests')
    ignored_parts = {
        '.git',
        '.mypy_cache',
        '.pytest_cache',
        '__pycache__',
        'artifacts',
        'build',
        'install',
        'log',
    }
    entries: list[dict[str, Any]] = []
    for name in roots:
        root = repository / name
        if not root.exists():
            raise EvidenceError(f'source snapshot root is missing: {root}')
        candidates = [root] if root.is_file() else sorted(root.rglob('*'))
        for path in candidates:
            relative = path.relative_to(repository)
            if any(part in ignored_parts for part in relative.parts):
                continue
            if relative.parts[:2] == ('docs', 'results'):
                continue
            if relative.as_posix() == 'config/release-claims.json':
                continue
            if path.is_symlink():
                raise EvidenceError(f'source snapshot contains a symlink: {relative}')
            if not path.is_file():
                continue
            entries.append(
                {
                    'path': relative.as_posix(),
                    'sha256': file_sha256(path),
                    'size_bytes': path.stat().st_size,
                }
            )
            if len(entries) > MAX_SOURCE_FILES:
                raise EvidenceError('source snapshot exceeds its file-count cap')
    snapshot_sha = hashlib.sha256(canonical_json_bytes(entries)).hexdigest()
    return {
        'schema_version': 1,
        'repository': str(repository.resolve()),
        'file_count': len(entries),
        'snapshot_sha256': snapshot_sha,
        'files': entries,
    }


def package_source_binding(repository: Path, manifest_path: Path) -> dict[str, Any]:
    """Compare a package candidate's source manifest to the live source bytes."""
    candidate_payload = _read_bounded_bytes(manifest_path, MAX_JSON_BYTES)
    expected = load_json(manifest_path)
    actual = repository_package_manifest(repository)
    expected_document = _mapping(expected, 'candidate source manifest')
    expected_rows = _mapping_list(expected_document.get('files'), 'candidate source manifest files')
    expected_files = {_string(item.get('path'), 'manifest path'): item for item in expected_rows}
    actual_files = {item['path']: item for item in actual['files']}
    missing = sorted(set(expected_files) - set(actual_files))
    extra = sorted(set(actual_files) - set(expected_files))
    mismatched = sorted(
        path
        for path in set(expected_files) & set(actual_files)
        if expected_files[path] != actual_files[path]
    )
    passed = (
        not missing
        and not extra
        and not mismatched
        and len(expected_files) == len(expected_rows)
        and _type_exact_json_equal(expected_document, actual)
        and candidate_payload == canonical_json_bytes(actual)
    )
    report = {
        'schema_version': SCHEMA_VERSION,
        'verdict': 'PASS' if passed else 'FAIL',
        'candidate_manifest': str(manifest_path.resolve()),
        'candidate_manifest_sha256': file_sha256(manifest_path),
        'repository_manifest_sha256': hashlib.sha256(canonical_json_bytes(actual)).hexdigest(),
        'missing_paths': missing,
        'extra_paths': extra,
        'mismatched_paths': mismatched,
        'file_count': len(actual_files),
    }
    _exact_keys(report, PACKAGE_BINDING_KEYS, 'package source binding report')
    return report


def _mapping_list(value: Any, name: str) -> list[Mapping[str, Any]]:
    if not isinstance(value, list) or not all(isinstance(item, Mapping) for item in value):
        raise EvidenceError(f'{name} must be a list of objects')
    return value


def validate_mission_terminal_verdict(result: Mapping[str, Any], name: str) -> int:
    """Validate the canonical terminal/status/verdict relationship."""
    measurements = _mapping(result.get('measurements'), f'{name}.measurements')
    quality = _mapping(result.get('quality'), f'{name}.quality')
    verdict = _mapping(result.get('verdict'), f'{name}.verdict')
    exit_code = _integer(verdict.get('exit_code'), f'{name}.verdict.exit_code', minimum=0)
    expected_outcome_met = _boolean(
        verdict.get('expected_outcome_met'), f'{name}.verdict.expected_outcome_met'
    )
    phase_status = _string(
        verdict.get('phase2_action_integration_status'),
        f'{name}.verdict.phase2_action_integration_status',
        maximum=16,
    )
    if phase_status not in {'PASS', 'FAIL'}:
        raise EvidenceError(f'{name} has an invalid Phase 2 integration status')
    reason = _string(verdict.get('reason'), f'{name}.verdict.reason')
    mission_status = _string(verdict.get('mission_status'), f'{name}.verdict.mission_status')
    terminal_observed = _boolean(
        quality.get('terminal_result_observed'), f'{name}.quality.terminal_result_observed'
    )
    goal_accepted = _boolean(quality.get('goal_accepted'), f'{name}.quality.goal_accepted')
    missed_count = _integer(
        measurements.get('missed_waypoint_count'),
        f'{name}.measurements.missed_waypoint_count',
        minimum=0,
    )

    goal_status_code = measurements.get('goal_status_code')
    goal_status = measurements.get('goal_status')
    terminal_stamp = measurements.get('terminal_action_stamp_ns')
    nav2_error_code = measurements.get('nav2_error_code')
    nav2_error_message = measurements.get('nav2_error_message')
    status_names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
    if terminal_observed:
        status_code = _integer(
            goal_status_code,
            f'{name}.measurements.goal_status_code',
            minimum=0,
        )
        if status_code not in status_names or goal_status != status_names[status_code]:
            raise EvidenceError(f'{name} terminal goal status is inconsistent')
        _integer(
            terminal_stamp,
            f'{name}.measurements.terminal_action_stamp_ns',
            minimum=0,
        )
        _integer(nav2_error_code, f'{name}.measurements.nav2_error_code')
        if not isinstance(nav2_error_message, str):
            raise EvidenceError(f'{name}.measurements.nav2_error_message must be a string')
        if not goal_accepted:
            raise EvidenceError(f'{name} observed a terminal result without an accepted goal')
        expected_mission_status = status_names[status_code]
    else:
        if any(
            value is not None
            for value in (
                goal_status_code,
                goal_status,
                terminal_stamp,
                measurements.get('terminal_action_stamp'),
                nav2_error_code,
                nav2_error_message,
            )
        ):
            raise EvidenceError(f'{name} has terminal fields without a terminal result')
        expected_mission_status = reason.upper()

    succeeded = exit_code == 0
    if expected_outcome_met is not succeeded:
        raise EvidenceError(f'{name} expected-outcome verdict disagrees with exit code')
    if phase_status != ('PASS' if succeeded else 'FAIL'):
        raise EvidenceError(f'{name} Phase 2 verdict disagrees with exit code')
    if mission_status != expected_mission_status:
        raise EvidenceError(f'{name} mission status disagrees with terminal status')
    if succeeded and not (
        terminal_observed and goal_status_code == 4 and nav2_error_code == 0 and missed_count == 0
    ):
        raise EvidenceError(f'{name} successful verdict lacks a successful terminal result')
    if terminal_observed and goal_status_code == 4 and not succeeded:
        raise EvidenceError(f'{name} reports SUCCEEDED with a nonzero exit code')
    if exit_code == 22 and goal_status_code != 6:
        raise EvidenceError(f'{name} GOAL_ABORTED exit lacks ABORTED terminal status')
    if exit_code == 23 and goal_status_code != 5:
        raise EvidenceError(f'{name} GOAL_CANCELED exit lacks CANCELED terminal status')
    return exit_code


def mission_pair(path_json: Path, path_csv: Path) -> tuple[Mapping[str, Any], bool]:
    """Validate canonical mission JSON against its normative one-row CSV."""
    result = _mapping(load_json(path_json), 'mission result')
    validate_mission_terminal_verdict(result, 'mission result')
    repository = Path(__file__).resolve().parents[1]
    mission_source = repository / 'src/robotest_missions'
    sys.path.insert(0, str(mission_source))
    try:
        from robotest_missions.artifacts import flatten_result
    except ImportError as exc:
        raise EvidenceError(f'cannot import canonical mission projection: {exc}') from exc
    expected = flatten_result(result)
    if path_csv.is_symlink() or not path_csv.is_file():
        raise EvidenceError(f'missing regular mission CSV: {path_csv}')
    if path_csv.stat().st_size > 1024 * 1024:
        raise EvidenceError('mission CSV exceeds the byte cap')
    try:
        with path_csv.open(newline='', encoding='utf-8') as source:
            reader = csv.DictReader(source)
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as exc:
        raise EvidenceError(f'cannot parse mission CSV: {exc}') from exc
    if len(rows) != 1 or tuple(reader.fieldnames or ()) != tuple(expected):
        raise EvidenceError('mission CSV shape does not match the canonical projection')
    return result, rows[0] == expected


def _validate_timeline_details(kind: str, details: Mapping[str, Any]) -> None:
    expected_keys = TIMELINE_DETAIL_KEYS.get(kind)
    if expected_keys is None:
        raise EvidenceError(f'timeline kind is not produced by Phase 4: {kind}')
    _exact_keys(details, expected_keys, f'timeline.{kind}.details')
    if kind in {
        'active_goal_probe_started',
        'followup_started',
        'mission_started',
    }:
        _integer(details.get('pid'), f'timeline.{kind}.pid', minimum=2)
        _integer(details.get('pgid'), f'timeline.{kind}.pgid', minimum=2)
    elif kind == 'initial_ready':
        for field in ('child_pgid', 'child_pid', 'main_pid'):
            _integer(details.get(field), f'timeline.{kind}.{field}', minimum=2)
        for field in ('systemd_nrestarts', 'systemd_owned_ros_service_count'):
            _integer(details.get(field), f'timeline.{kind}.{field}', minimum=0)
    elif kind == 'failure_injected':
        for field in ('mission_pid', 'original_pgid', 'target_pid'):
            _integer(details.get(field), f'timeline.{kind}.{field}', minimum=2)
        _integer(
            details.get('event_sequence_before'),
            f'timeline.{kind}.event_sequence_before',
            minimum=1,
        )
    elif kind in {'ready_unavailable', 'original_group_empty'}:
        field = 'http_status' if kind == 'ready_unavailable' else 'member_count'
        _integer(details.get(field), f'timeline.{kind}.{field}', minimum=0)
    elif kind == 'ready_restored':
        for field in ('child_pgid', 'child_pid', 'main_pid'):
            _integer(details.get(field), f'timeline.{kind}.{field}', minimum=2)
        for field in (
            'event_sequence_after_recovery',
            'http_status',
            'systemd_nrestarts',
            'systemd_owned_ros_service_count',
        ):
            _integer(details.get(field), f'timeline.{kind}.{field}', minimum=0)
    elif kind == 'interrupted_mission_finished':
        _integer(details.get('exit_code'), f'timeline.{kind}.exit_code')
        _boolean(
            details.get('process_group_empty'),
            f'timeline.{kind}.process_group_empty',
        )
        _boolean(
            details.get('terminated_by_harness'),
            f'timeline.{kind}.terminated_by_harness',
        )
    elif kind in {'followup_finished', 'service_stopped'}:
        field = 'exit_code' if kind == 'followup_finished' else 'event_sequence_before_stop'
        _integer(details.get(field), f'timeline.{kind}.{field}', minimum=0)


def _timeline_index(records: Sequence[Mapping[str, Any]]) -> dict[str, list[Mapping[str, Any]]]:
    index: dict[str, list[Mapping[str, Any]]] = {}
    previous_monotonic = -1
    for sequence, record in enumerate(records, 1):
        _exact_keys(record, TIMELINE_KEYS, f'timeline[{sequence - 1}]')
        schema_version = _integer(
            record.get('schema_version'), f'timeline[{sequence - 1}].schema_version'
        )
        record_sequence = _integer(
            record.get('sequence'), f'timeline[{sequence - 1}].sequence', minimum=1
        )
        if schema_version != 1 or record_sequence != sequence:
            raise EvidenceError('timeline schema or sequence is invalid')
        _utc_timestamp(record.get('timestamp_utc'), f'timeline[{sequence - 1}].timestamp_utc')
        monotonic = _integer(record.get('monotonic_ns'), 'timeline.monotonic_ns', minimum=0)
        if monotonic < previous_monotonic:
            raise EvidenceError('timeline monotonic time regressed')
        previous_monotonic = monotonic
        kind = _string(record.get('kind'), 'timeline.kind')
        details = _mapping(record.get('details'), 'timeline.details')
        _validate_timeline_details(kind, details)
        index.setdefault(kind, []).append(record)
    return index


def _validate_http_probe_record(
    record: Mapping[str, Any],
    *,
    endpoint: str,
    name: str,
) -> int:
    details = _mapping(record.get('details'), f'{name}.details')
    _exact_keys(
        details,
        frozenset({'body', 'body_sha256', 'http_status', 'url'}),
        f'{name}.details',
    )
    status = _integer(details.get('http_status'), f'{name}.http_status', minimum=0)
    body = details.get('body')
    if not isinstance(body, str) or len(body.encode()) > 65536:
        raise EvidenceError(f'{name}.body must be a bounded string')
    expected_bodies = (
        {200: 'ok\n', 0: ''}
        if endpoint == '/healthz'
        else {200: 'ready\n', 503: 'not ready\n', 0: ''}
    )
    if details.get('url') != f'http://127.0.0.1:9080{endpoint}':
        raise EvidenceError(f'{name} URL is not the exact supervisor endpoint')
    if status not in expected_bodies or body != expected_bodies[status]:
        raise EvidenceError(f'{name} status/body semantics are invalid')
    if details.get('body_sha256') != hashlib.sha256(body.encode()).hexdigest():
        raise EvidenceError(f'{name} body hash does not reconcile')
    return status


def _validate_http_probe_pairs(
    health_records: Sequence[Mapping[str, Any]],
    ready_records: Sequence[Mapping[str, Any]],
    *,
    lower_sequence: int,
    upper_sequence: int,
    name: str,
    require_health_200: bool = True,
) -> list[tuple[int, int]]:
    if not health_records or len(health_records) != len(ready_records):
        raise EvidenceError(f'{name} HTTP probe pairs are missing or unbalanced')
    statuses: list[tuple[int, int]] = []
    for index, (health, ready) in enumerate(zip(health_records, ready_records, strict=True)):
        health_sequence = _integer(health.get('sequence'), f'{name}[{index}] health sequence')
        ready_sequence = _integer(ready.get('sequence'), f'{name}[{index}] ready sequence')
        if not (
            lower_sequence < health_sequence
            and ready_sequence == health_sequence + 1
            and ready_sequence < upper_sequence
        ):
            raise EvidenceError(f'{name}[{index}] HTTP pair ordering is invalid')
        health_status = _validate_http_probe_record(
            health, endpoint='/healthz', name=f'{name}[{index}].health'
        )
        if require_health_200 and health_status != 200:
            raise EvidenceError(f'{name}[{index}] health probe was not 200')
        ready_status = _validate_http_probe_record(
            ready, endpoint='/readyz', name=f'{name}[{index}].ready'
        )
        statuses.append((health_status, ready_status))
    return statuses


def _one(index: Mapping[str, list[Mapping[str, Any]]], kind: str) -> Mapping[str, Any]:
    values = index.get(kind, [])
    if len(values) != 1:
        raise EvidenceError(f'timeline requires exactly one {kind}, found {len(values)}')
    return values[0]


def _duration_seconds(start: Mapping[str, Any], end: Mapping[str, Any]) -> float:
    start_ns = _integer(start.get('monotonic_ns'), 'start monotonic', minimum=0)
    end_ns = _integer(end.get('monotonic_ns'), 'end monotonic', minimum=0)
    if end_ns < start_ns:
        raise EvidenceError('duration endpoint precedes its start')
    return (end_ns - start_ns) / 1_000_000_000


def _event_sequence(events: Sequence[Mapping[str, Any]]) -> None:
    previous_steady = -1
    for index, event in enumerate(events, 1):
        if event.get('schema_version') != 1 or event.get('sequence') != index:
            raise EvidenceError('supervisor events are not contiguous from sequence one')
        steady = _integer(event.get('steady_wall_ns'), 'event steady_wall_ns', minimum=0)
        if steady < previous_steady:
            raise EvidenceError('supervisor event steady time regressed')
        previous_steady = steady
        _string(event.get('kind'), 'event kind')


def _validate_supervisor_event_schema(event: Mapping[str, Any]) -> None:
    kind = _string(event.get('kind'), 'supervisor event kind')
    base = {'kind', 'schema_version', 'sequence', 'steady_wall_ns', 'timestamp_utc'}
    suffixes = {
        'supervisor_started': set(),
        'shutdown_requested': set(),
        'supervisor_stopped': set(),
        'child_started': {'child', 'pgid', 'pid'},
        'heartbeat_fresh': {'child'},
        'heartbeat_stale': {'child'},
        'readiness_changed': {'ready'},
        'restart_scheduled': {'backoff_ms', 'child', 'restart_attempt'},
        'restart_exhausted': {'child', 'restart_attempt'},
        'child_stop_requested': {'child', 'pgid'},
        'child_kill_escalated': {'child', 'pgid'},
        'child_stopped': {'child', 'exit_code'},
        'residual_process_group_detected': {'child', 'pgid'},
        'residual_process_group_kill_escalated': {'child', 'pgid'},
        'residual_process_group_stopped': {'child', 'pgid'},
        'child_group_cleanup_incomplete': {
            'child',
            'details',
            'failure_kind',
            'pgid',
            'pid',
        },
    }
    if kind == 'failure_detected':
        failure_kind = _string(event.get('failure_kind'), 'failure_detected.failure_kind')
        failure_suffixes = {
            'heartbeat_stale': {'child', 'failure_kind', 'pgid', 'pid'},
            'heartbeat_startup_timeout': {'child', 'failure_kind', 'pgid', 'pid'},
            'start_failed': {'child', 'details', 'failure_kind'},
            'unexpected_exit': {
                'child',
                'details',
                'exit_code',
                'failure_kind',
                'pgid',
                'pid',
            },
        }
        expected_suffix = failure_suffixes.get(failure_kind)
    else:
        expected_suffix = suffixes.get(kind)
    if expected_suffix is None or set(event) != base | expected_suffix:
        raise EvidenceError(f'supervisor event {kind!r} has a noncanonical schema')
    if _integer(event.get('schema_version'), f'{kind}.schema_version') != 1:
        raise EvidenceError(f'supervisor event {kind!r} has the wrong schema version')
    _integer(event.get('sequence'), f'{kind}.sequence', minimum=1)
    _integer(event.get('steady_wall_ns'), f'{kind}.steady_wall_ns', minimum=0)
    _utc_timestamp(event.get('timestamp_utc'), f'{kind}.timestamp_utc')
    if 'child' in event and _string(event.get('child'), f'{kind}.child') != EXPECTED_CHILD:
        raise EvidenceError(f'supervisor event {kind!r} names an unexpected child')
    for field in ('pid', 'pgid', 'restart_attempt', 'backoff_ms'):
        if field in event:
            _integer(event.get(field), f'{kind}.{field}', minimum=1)
    if 'exit_code' in event:
        _integer(event.get('exit_code'), f'{kind}.exit_code')
    if 'ready' in event:
        _boolean(event.get('ready'), f'{kind}.ready')
    if 'failure_kind' in event:
        _string(event.get('failure_kind'), f'{kind}.failure_kind')
    if kind == 'failure_detected' and 'details' in event:
        details = _mapping(event.get('details'), f'{kind}.details')
        _exact_keys(details, frozenset({'error'}), f'{kind}.details')
        _string(details.get('error'), f'{kind}.details.error')
    if kind == 'child_group_cleanup_incomplete':
        details = _mapping(event.get('details'), f'{kind}.details')
        _exact_keys(
            details,
            frozenset({'captured_pgid', 'error', 'group_empty', 'leader_reaped'}),
            f'{kind}.details',
        )
        _integer(details.get('captured_pgid'), f'{kind}.captured_pgid', minimum=1)
        _string(details.get('error'), f'{kind}.error')
        _boolean(details.get('group_empty'), f'{kind}.group_empty')
        _boolean(details.get('leader_reaped'), f'{kind}.leader_reaped')


def _add_check(checks: dict[str, bool], failures: list[str], name: str, condition: bool) -> None:
    checks[name] = bool(condition)
    if not condition:
        failures.append(name)


def _relative_evidence_hashes(run_directory: Path, excluded: set[str]) -> dict[str, str]:
    files = [path for path in run_directory.rglob('*') if path.is_file()]
    if len(files) > MAX_EVIDENCE_FILES:
        raise EvidenceError('run evidence exceeds the file-count cap')
    if any(path.is_symlink() for path in run_directory.rglob('*')):
        raise EvidenceError('run evidence must not contain symbolic links')
    return {
        path.relative_to(run_directory).as_posix(): file_sha256(path)
        for path in sorted(files)
        if path.relative_to(run_directory).as_posix() not in excluded
    }


def evaluate_run(run_directory: Path) -> dict[str, Any]:
    """Evaluate all frozen Scenario 6 gates from retained raw evidence."""
    context = _mapping(load_json(run_directory / 'context.json'), 'context')
    _exact_keys(context, CONTEXT_KEYS, 'context')
    if _integer(context.get('schema_version'), 'context.schema_version') != 1:
        raise EvidenceError('context schema is unsupported')
    run_id = _string(context.get('run_id'), 'context.run_id')
    if RUN_ID_RE.fullmatch(run_id) is None:
        raise EvidenceError('context.run_id is not canonical')
    _utc_timestamp(context.get('started_utc'), 'context.started_utc')
    source_git_commit = _string(
        context.get('source_git_commit'), 'context.source_git_commit', maximum=64
    )
    if re.fullmatch(r'[0-9a-f]{40}', source_git_commit) is None:
        raise EvidenceError('context.source_git_commit is not a canonical Git object ID')
    _boolean(context.get('source_git_dirty'), 'context.source_git_dirty')
    _string(context.get('package_directory'), 'context.package_directory')
    _string(context.get('active_overlay_target'), 'context.active_overlay_target')
    _string(context.get('cpuset'), 'context.cpuset')
    for descriptor_name in ('upgrade_package', 'baseline_package', 'lifecycle_evidence'):
        descriptor = _mapping(context.get(descriptor_name), f'context.{descriptor_name}')
        _exact_keys(descriptor, CONTEXT_FILE_DESCRIPTOR_KEYS, f'context.{descriptor_name}')
        _string(descriptor.get('path'), f'context.{descriptor_name}.path')
        _sha256(descriptor.get('sha256'), f'context.{descriptor_name}.sha256')
    context_isolation = _mapping(context.get('isolation'), 'context.isolation')
    _exact_keys(context_isolation, ISOLATION_KEYS, 'context.isolation')
    if _integer(context_isolation.get('schema_version'), 'context.isolation.schema_version') != 1:
        raise EvidenceError('context.isolation schema is unsupported')
    if _string(context_isolation.get('run_id'), 'context.isolation.run_id') != run_id:
        raise EvidenceError('context and isolation run IDs differ')
    _integer(context_isolation.get('ros_domain_id'), 'context.isolation.ros_domain_id')
    _string(context_isolation.get('gz_partition'), 'context.isolation.gz_partition')
    _integer(
        context_isolation.get('inspected_processes'),
        'context.isolation.inspected_processes',
        minimum=0,
    )
    _integer(
        context_isolation.get('unreadable_process_environments'),
        'context.isolation.unreadable_process_environments',
        minimum=0,
    )
    _boolean(context_isolation.get('domain_was_unused'), 'context.isolation.domain_was_unused')
    _boolean(
        context_isolation.get('partition_was_unused'),
        'context.isolation.partition_was_unused',
    )
    retained_isolation = _mapping(load_json(run_directory / 'isolation.json'), 'retained isolation')
    if not _type_exact_json_equal(retained_isolation, context_isolation):
        raise EvidenceError('retained isolation does not equal the context projection')
    timeline = load_jsonl(run_directory / 'timeline.jsonl')
    index = _timeline_index(timeline)
    initial = _one(index, 'initial_ready')
    mission_started = _one(index, 'mission_started')
    probe_started = _one(index, 'active_goal_probe_started')
    injection = _one(index, 'failure_injected')
    unavailable = _one(index, 'ready_unavailable')
    group_empty = _one(index, 'original_group_empty')
    restored = _one(index, 'ready_restored')
    interrupted_finished = _one(index, 'interrupted_mission_finished')
    followup_started = _one(index, 'followup_started')
    followup_finished = _one(index, 'followup_finished')
    service_stopped = _one(index, 'service_stopped')
    cleanup_event = _one(index, 'cleanup_complete')
    startup_probe_statuses = _validate_http_probe_pairs(
        index.get('startup_health_probe', []),
        index.get('startup_ready_probe', []),
        lower_sequence=0,
        upper_sequence=_integer(initial.get('sequence'), 'initial_ready sequence'),
        name='startup',
        require_health_200=False,
    )
    recovery_probe_statuses = _validate_http_probe_pairs(
        index.get('health_probe', []),
        index.get('ready_probe', []),
        lower_sequence=_integer(injection.get('sequence'), 'failure_injected sequence'),
        upper_sequence=_integer(restored.get('sequence'), 'ready_restored sequence'),
        name='recovery',
    )
    if startup_probe_statuses[-1] != (200, 200):
        raise EvidenceError('startup HTTP probes do not end ready')
    if 503 not in [ready for _health, ready in recovery_probe_statuses] or recovery_probe_statuses[
        -1
    ] != (200, 200):
        raise EvidenceError('recovery HTTP probes do not span unavailable through ready')
    if not (
        int(initial['monotonic_ns']) < int(mission_started['monotonic_ns'])
        and int(mission_started['monotonic_ns']) < int(probe_started['monotonic_ns'])
        and int(probe_started['monotonic_ns']) < int(injection['monotonic_ns'])
        and int(injection['monotonic_ns']) < int(unavailable['monotonic_ns'])
        and int(injection['monotonic_ns']) < int(group_empty['monotonic_ns'])
        and int(unavailable['monotonic_ns']) < int(restored['monotonic_ns'])
        and int(group_empty['monotonic_ns']) < int(restored['monotonic_ns'])
        and int(restored['monotonic_ns']) < int(interrupted_finished['monotonic_ns'])
        and int(interrupted_finished['monotonic_ns']) < int(followup_started['monotonic_ns'])
        and int(followup_started['monotonic_ns']) < int(followup_finished['monotonic_ns'])
        and int(followup_finished['monotonic_ns']) < int(service_stopped['monotonic_ns'])
        and int(service_stopped['monotonic_ns']) < int(cleanup_event['monotonic_ns'])
    ):
        raise EvidenceError('Scenario 6 timeline causal order is invalid')

    initial_details = _mapping(initial['details'], 'initial_ready.details')
    injection_details = _mapping(injection['details'], 'failure_injected.details')
    restored_details = _mapping(restored['details'], 'ready_restored.details')
    mission_started_details = _mapping(mission_started['details'], 'mission_started.details')
    probe_started_details = _mapping(probe_started['details'], 'active_goal_probe_started.details')
    followup_started_details = _mapping(followup_started['details'], 'followup_started.details')
    service_stopped_details = _mapping(service_stopped['details'], 'service_stopped.details')
    original_pgid = _integer(initial_details.get('child_pgid'), 'initial child PGID', minimum=1)
    original_child_pid = _integer(initial_details.get('child_pid'), 'initial child PID', minimum=1)
    main_pid = _integer(initial_details.get('main_pid'), 'initial supervisor PID', minimum=1)
    baseline_sequence = _integer(
        injection_details.get('event_sequence_before'),
        'event sequence before injection',
        minimum=1,
    )
    recovery_end_sequence = _integer(
        restored_details.get('event_sequence_after_recovery'),
        'event sequence after recovery',
        minimum=baseline_sequence + 1,
    )
    observation_end_sequence = _integer(
        service_stopped_details.get('event_sequence_before_stop'),
        'event sequence before service stop',
        minimum=recovery_end_sequence,
    )
    if injection_details.get('original_pgid') != original_pgid:
        raise EvidenceError('injection PGID does not match initial status')

    target = _mapping(load_json(run_directory / 'controller-target.json'), 'controller target')
    target_lineage = validate_controller_target_document(target)
    active_goal = _mapping(load_json(run_directory / 'active-goal.json'), 'active goal')
    _exact_keys(active_goal, ACTIVE_GOAL_KEYS, 'active goal')
    active_goal_schema = _integer(active_goal.get('schema_version'), 'active goal schema_version')
    active_goal_verdict = _string(active_goal.get('verdict'), 'active goal verdict')
    _utc_timestamp(active_goal.get('captured_utc'), 'active goal captured_utc')
    active_goal_observed_ns = _integer(
        active_goal.get('observed_monotonic_ns'),
        'active goal observed_monotonic_ns',
        minimum=1,
    )
    active_goal_elapsed_s = _number(
        active_goal.get('elapsed_wall_s'), 'active goal elapsed_wall_s', minimum=0.0
    )
    active_goal_pid = _integer(active_goal.get('mission_pid'), 'active goal mission_pid', minimum=2)
    active_goal_process_alive = _boolean(
        active_goal.get('mission_process_alive'), 'active goal mission_process_alive'
    )
    active_goal_node = _string(active_goal.get('mission_node'), 'active goal mission_node')
    active_goal_name = _string(active_goal.get('action_name'), 'active goal action_name')
    active_goal_type = _string(active_goal.get('action_type'), 'active goal action_type')
    active_goal_uuid = _string(active_goal.get('goal_uuid'), 'active goal goal_uuid', maximum=36)
    active_goal_accepted_stamp_ns = _integer(
        active_goal.get('accepted_goal_stamp_ns'),
        'active goal accepted_goal_stamp_ns',
        minimum=1,
    )
    active_goal_status_code = _integer(
        active_goal.get('goal_status_code'), 'active goal goal_status_code'
    )
    active_goal_status = _string(active_goal.get('goal_status'), 'active goal goal_status')
    active_goal_status_count = _integer(
        active_goal.get('status_entry_count'), 'active goal status_entry_count', minimum=1
    )
    active_goal_attempt_count = _integer(
        active_goal.get('attempt_count'), 'active goal attempt_count', minimum=1
    )
    active_goal_domain = _string(active_goal.get('ros_domain_id'), 'active goal ros_domain_id')
    active_goal_partition = _string(active_goal.get('gz_partition'), 'active goal gz_partition')
    mission_runner_is_goal_capable = _boolean(
        active_goal.get('mission_runner_is_goal_capable_action_client'),
        'active goal mission_runner_is_goal_capable_action_client',
    )
    _mapping(active_goal.get('goal_capable_action_clients'), 'active goal goal clients')
    _mapping(
        active_goal.get('projected_action_client_participants'),
        'active goal projected participants',
    )
    rebound_active_client_evidence = action_client_evidence_from_graph_entries(
        active_goal.get('service_clients'),
        active_goal.get('projected_action_clients'),
    )
    active_client_evidence_exact = frozenset(
        rebound_active_client_evidence
    ) == ACTIVE_GOAL_CLIENT_EVIDENCE_KEYS and all(
        active_goal.get(key) == value for key, value in rebound_active_client_evidence.items()
    )
    events = load_jsonl(run_directory / 'supervisor-events.jsonl')
    _event_sequence(events)
    for event in events:
        _validate_supervisor_event_schema(event)
    event_meta = _mapping(
        load_json(run_directory / 'supervisor-events.meta.json'), 'event metadata'
    )
    _exact_keys(event_meta, EVENT_META_KEYS, 'event metadata')
    event_meta_schema = _integer(event_meta.get('schema_version'), 'event metadata schema_version')
    event_meta_saturated = _boolean(event_meta.get('saturated'), 'event metadata saturated')
    event_meta_dropped = _integer(
        event_meta.get('dropped_events'), 'event metadata dropped_events', minimum=0
    )
    event_meta_last_sequence = _integer(
        event_meta.get('last_attempted_sequence'),
        'event metadata last_attempted_sequence',
        minimum=0,
    )
    if observation_end_sequence > len(events):
        raise EvidenceError('pre-stop event sequence exceeds retained supervisor events')
    startup_prefix = events[:baseline_sequence]
    startup_prefix_valid = (
        baseline_sequence == 4
        and [event.get('kind') for event in startup_prefix]
        == [
            'supervisor_started',
            'child_started',
            'heartbeat_fresh',
            'readiness_changed',
        ]
        and startup_prefix[1].get('child') == EXPECTED_CHILD
        and startup_prefix[1].get('pid') == original_child_pid
        and startup_prefix[1].get('pgid') == original_pgid
        and startup_prefix[2].get('child') == EXPECTED_CHILD
        and startup_prefix[3].get('ready') is True
        and all(
            left < right
            for left, right in pairwise(
                [
                    _integer(event.get('steady_wall_ns'), 'startup event time')
                    for event in startup_prefix
                ]
            )
        )
    )
    shutdown_suffix = events[observation_end_sequence:]
    shutdown_kinds = [event.get('kind') for event in shutdown_suffix]
    expected_shutdown_kinds = [
        'shutdown_requested',
        'child_stop_requested',
        'child_stopped',
        'readiness_changed',
        'supervisor_stopped',
    ]
    if 'child_kill_escalated' in shutdown_kinds:
        expected_shutdown_kinds.insert(2, 'child_kill_escalated')
    post_observation_shutdown_valid = (
        shutdown_kinds == expected_shutdown_kinds
        and shutdown_suffix[1].get('child') == EXPECTED_CHILD
        and shutdown_suffix[1].get('pgid') == restored_details.get('child_pgid')
        and shutdown_suffix[-2].get('ready') is False
        and shutdown_suffix[-3].get('child') == EXPECTED_CHILD
        and shutdown_suffix[-3].get('exit_code') == 143
    )
    if 'child_kill_escalated' in shutdown_kinds:
        post_observation_shutdown_valid = (
            post_observation_shutdown_valid
            and shutdown_suffix[2].get('child') == EXPECTED_CHILD
            and shutdown_suffix[2].get('pgid') == restored_details.get('child_pgid')
        )
    after = [
        event
        for event in events
        if baseline_sequence < int(event['sequence']) <= observation_end_sequence
    ]
    failures_detected = [
        event
        for event in after
        if event.get('kind') == 'failure_detected' and event.get('child') == EXPECTED_CHILD
    ]
    restart_events = [
        event
        for event in after
        if event.get('kind') == 'restart_scheduled' and event.get('child') == EXPECTED_CHILD
    ]
    child_starts = [
        event
        for event in after
        if event.get('kind') == 'child_started' and event.get('child') == EXPECTED_CHILD
    ]
    ready_false = [
        event
        for event in after
        if event.get('kind') == 'readiness_changed' and event.get('ready') is False
    ]
    ready_true = [
        event
        for event in after
        if event.get('kind') == 'readiness_changed' and event.get('ready') is True
    ]
    if len(ready_true) == 1 and int(ready_true[0]['sequence']) > recovery_end_sequence:
        raise EvidenceError('ready transition occurs after the recorded recovery bound')
    transition_kinds = {
        'failure_detected',
        'heartbeat_fresh',
        'restart_scheduled',
        'child_started',
        'readiness_changed',
    }
    transition_chain = [event for event in after if event.get('kind') in transition_kinds]
    causal_chain_valid = False
    if [event.get('kind') for event in transition_chain] == [
        'failure_detected',
        'readiness_changed',
        'restart_scheduled',
        'child_started',
        'heartbeat_fresh',
        'readiness_changed',
    ]:
        (
            failure_event,
            false_event,
            restart_event,
            start_event,
            heartbeat_event,
            true_event,
        ) = transition_chain
        chain_times = [int(event['steady_wall_ns']) for event in transition_chain]
        failure_exit = failure_event.get('exit_code')
        replacement_pid = restored_details.get('child_pid')
        replacement_pgid = restored_details.get('child_pgid')
        causal_chain_valid = (
            failure_event.get('child') == EXPECTED_CHILD
            and failure_event.get('pid') == original_child_pid
            and failure_event.get('pgid') == original_pgid
            and failure_event.get('failure_kind') == 'unexpected_exit'
            and isinstance(failure_exit, int)
            and not isinstance(failure_exit, bool)
            and failure_exit != 0
            and false_event.get('ready') is False
            and false_event.get('child') is None
            and restart_event.get('child') == EXPECTED_CHILD
            and restart_event.get('restart_attempt') == 1
            and restart_event.get('backoff_ms') == EXPECTED_BACKOFF_MS
            and restart_event.get('pid') is None
            and restart_event.get('pgid') is None
            and start_event.get('child') == EXPECTED_CHILD
            and start_event.get('pid') == replacement_pid
            and start_event.get('pgid') == replacement_pgid
            and heartbeat_event.get('child') == EXPECTED_CHILD
            and isinstance(replacement_pid, int)
            and not isinstance(replacement_pid, bool)
            and replacement_pid > 1
            and replacement_pid == replacement_pgid
            and replacement_pgid != original_pgid
            and true_event.get('ready') is True
            and true_event.get('child') is None
            and all(left < right for left, right in pairwise(chain_times))
        )

    ready_samples = index.get('ready_probe', [])
    first_ready_503 = next(
        sample
        for sample in ready_samples
        if _mapping(sample.get('details'), 'ready probe details').get('http_status') == 503
    )
    final_ready_200 = ready_samples[-1]
    unavailable_sequence = _integer(unavailable.get('sequence'), 'ready unavailable sequence')
    first_ready_503_sequence = _integer(first_ready_503.get('sequence'), 'first 503 sequence')
    following_probe = min(
        (
            sample
            for sample in [*index.get('health_probe', []), *ready_samples]
            if _integer(sample.get('sequence'), 'following probe sequence') > unavailable_sequence
        ),
        key=lambda sample: _integer(sample.get('sequence'), 'following probe sequence'),
    )
    first_ready_503_ns = _integer(first_ready_503.get('monotonic_ns'), 'first 503 time')
    unavailable_ns = _integer(unavailable.get('monotonic_ns'), 'ready unavailable time')
    ready_unavailable_marker_exact = (
        unavailable_sequence == first_ready_503_sequence + 1
        and first_ready_503_ns <= unavailable_ns
        and unavailable_ns <= _integer(following_probe.get('monotonic_ns'), 'following probe time')
        and _mapping(unavailable.get('details'), 'ready unavailable details')
        == {'http_status': 503}
    )
    detection_s = _duration_seconds(injection, first_ready_503)
    group_cleanup_s = _duration_seconds(injection, group_empty)
    observed_recovery_s = _duration_seconds(first_ready_503, final_ready_200)
    event_recovery_s: float | None = None
    actual_backoff_s: float | None = None
    if len(failures_detected) == len(restart_events) == len(child_starts) == 1 and ready_true:
        event_recovery_s = (
            int(ready_true[0]['steady_wall_ns']) - int(failures_detected[0]['steady_wall_ns'])
        ) / 1_000_000_000
        actual_backoff_s = (
            int(child_starts[0]['steady_wall_ns']) - int(restart_events[0]['steady_wall_ns'])
        ) / 1_000_000_000

    health_samples = index.get('health_probe', [])
    first_ready_503_index = ready_samples.index(first_ready_503)
    recovery_ready_statuses = [
        _mapping(sample['details'], 'recovery ready sample').get('http_status')
        for sample in ready_samples[first_ready_503_index:]
    ]
    readiness_interval_valid = (
        len(recovery_ready_statuses) >= 2
        and recovery_ready_statuses[-1] == 200
        and all(status == 503 for status in recovery_ready_statuses[:-1])
    )

    interrupted = _mapping(
        load_json(run_directory / 'interrupted-outcome.json'), 'interrupted outcome'
    )
    _exact_keys(interrupted, INTERRUPTED_OUTCOME_KEYS, 'interrupted outcome')
    if _integer(interrupted.get('schema_version'), 'interrupted schema_version') != 1:
        raise EvidenceError('interrupted outcome schema_version must be 1')
    _utc_timestamp(interrupted.get('captured_utc'), 'interrupted captured_utc')
    interrupted_exit = _integer(interrupted.get('process_exit_code'), 'interrupted exit')
    interrupted_json_present = _boolean(interrupted.get('json_present'), 'interrupted json_present')
    interrupted_csv_present = _boolean(interrupted.get('csv_present'), 'interrupted csv_present')
    interrupted_terminated = _boolean(
        interrupted.get('terminated_by_harness'), 'interrupted terminated_by_harness'
    )
    interrupted_group_empty = _boolean(
        interrupted.get('process_group_empty'), 'interrupted process_group_empty'
    )
    interrupted_bounded = _boolean(interrupted.get('bounded_wait'), 'interrupted bounded_wait')
    interrupted_event_details = _mapping(
        interrupted_finished.get('details'), 'interrupted_mission_finished.details'
    )
    _exact_keys(
        interrupted_event_details,
        frozenset({'exit_code', 'process_group_empty', 'terminated_by_harness'}),
        'interrupted_mission_finished.details',
    )
    interrupted_event_exit = _integer(
        interrupted_event_details.get('exit_code'), 'interrupted completion exit_code'
    )
    interrupted_event_terminated = _boolean(
        interrupted_event_details.get('terminated_by_harness'),
        'interrupted completion terminated_by_harness',
    )
    interrupted_event_group_empty = _boolean(
        interrupted_event_details.get('process_group_empty'),
        'interrupted completion process_group_empty',
    )
    interrupted_json_path = run_directory / 'interrupted-result.json'
    interrupted_csv_path = run_directory / 'interrupted-result.csv'
    actual_interrupted_json = (
        interrupted_json_path.is_file() and not interrupted_json_path.is_symlink()
    )
    actual_interrupted_csv = (
        interrupted_csv_path.is_file() and not interrupted_csv_path.is_symlink()
    )
    interrupted_success = False
    interrupted_pair_valid = (
        interrupted_json_present is actual_interrupted_json
        and interrupted_csv_present is actual_interrupted_csv
        and not interrupted_json_present
        and not interrupted_csv_present
    )
    if interrupted_json_present or interrupted_csv_present:
        if interrupted_json_present and interrupted_csv_present:
            interrupted_result, interrupted_pair_valid = mission_pair(
                interrupted_json_path,
                interrupted_csv_path,
            )
            interrupted_verdict = _mapping(
                interrupted_result.get('verdict'), 'interrupted mission verdict'
            )
            interrupted_artifact_exit = _integer(
                interrupted_verdict.get('exit_code'),
                'interrupted artifact verdict.exit_code',
                minimum=0,
            )
            interrupted_pair_valid = interrupted_pair_valid and (
                interrupted_artifact_exit == interrupted_exit == interrupted_event_exit
            )
            interrupted_success = (
                interrupted_artifact_exit == 0
                or interrupted_verdict.get('phase2_action_integration_status') == 'PASS'
                or interrupted_verdict.get('expected_outcome_met') is True
            )
        else:
            interrupted_pair_valid = False

    followup, followup_csv_valid = mission_pair(
        run_directory / 'followup-result.json', run_directory / 'followup-result.csv'
    )
    followup_identity = _mapping(followup.get('identity'), 'followup identity')
    followup_verdict = _mapping(followup.get('verdict'), 'followup verdict')
    followup_measurements = _mapping(followup.get('measurements'), 'followup measurements')
    followup_mission_path = run_directory / 'followup-mission.json'
    followup_mission_sha = file_sha256(followup_mission_path)
    expected_followup = expected_followup_mission()
    followup_mission = _mapping(load_json(followup_mission_path), 'followup mission')
    followup_mission_is_exact = _read_bounded_bytes(
        followup_mission_path, MAX_JSON_BYTES
    ) == canonical_json_bytes(expected_followup) and _type_exact_json_equal(
        followup_mission, expected_followup
    )
    followup_event_details = _mapping(followup_finished.get('details'), 'followup_finished.details')
    _exact_keys(
        followup_event_details,
        frozenset({'exit_code'}),
        'followup_finished.details',
    )
    followup_event_exit = _integer(
        followup_event_details.get('exit_code'), 'followup completion exit_code'
    )

    initial_status = _mapping(load_json(run_directory / 'initial-status.json'), 'initial status')
    restored_status = _mapping(
        load_json(run_directory / 'ready-restored-status.json'), 'restored status'
    )
    supervisor_status_final = _mapping(
        load_json(run_directory / 'supervisor-status-final.json'),
        'final supervisor status',
    )
    initial_owners = _mapping(
        load_json(run_directory / 'systemd-owners-initial.json'), 'initial systemd owners'
    )
    restored_owners = _mapping(
        load_json(run_directory / 'systemd-owners-restored.json'), 'restored systemd owners'
    )
    original_group_evidence = _mapping(
        load_json(run_directory / 'original-group-empty.json'), 'original group evidence'
    )
    service_final = _mapping(load_json(run_directory / 'service-final.json'), 'service final')
    _exact_keys(service_final, SERVICE_FINAL_KEYS, 'service final')
    if _integer(service_final.get('schema_version'), 'service final schema_version') != 1:
        raise EvidenceError('service final schema_version must be 1')
    _utc_timestamp(service_final.get('captured_utc'), 'service final captured_utc')
    for field in ('active_before_stop', 'enabled_before_stop'):
        _boolean(service_final.get(field), f'service final {field}')
    _integer(service_final.get('main_pid_before_stop'), 'service final main PID', minimum=0)
    _integer(service_final.get('nrestarts_before_stop'), 'service final NRestarts', minimum=0)
    cleanup = _mapping(load_json(run_directory / 'cleanup.json'), 'cleanup')
    interrupted_pgid = _integer(
        mission_started_details.get('pgid'), 'interrupted mission PGID', minimum=2
    )
    probe_pgid = _integer(probe_started_details.get('pgid'), 'active goal probe PGID', minimum=2)
    followup_pgid = _integer(
        followup_started_details.get('pgid'), 'followup mission PGID', minimum=2
    )
    validate_cleanup_evidence(
        cleanup,
        interrupted_pgid=interrupted_pgid,
        followup_pgid=followup_pgid,
        probe_pgid=probe_pgid,
        original_pgid=original_pgid,
        replacement_pgid=_integer(
            restored_details.get('child_pgid'), 'replacement child PGID', minimum=2
        ),
    )
    package_binding = _mapping(load_json(run_directory / 'package-binding.json'), 'package binding')
    staging = _mapping(load_json(run_directory / 'runtime-staging.json'), 'runtime staging')
    overlay_provenance = _mapping(
        load_json(run_directory / 'overlay-provenance.json'), 'overlay provenance'
    )
    overlay_source_manifest_path = run_directory / 'overlay-source-manifest.json'
    overlay_install_manifest_path = run_directory / 'overlay-install-manifest.json'
    load_json(overlay_source_manifest_path)
    load_json(overlay_install_manifest_path)
    lifecycle = _mapping(load_json(run_directory / 'package-lifecycle.json'), 'package lifecycle')
    isolation = _mapping(context.get('isolation'), 'context.isolation')
    supervisor_config = _mapping(
        load_json(run_directory / 'supervisor-config.json'), 'supervisor config used'
    )
    supervisor_config_check_exact = (
        _read_bounded_bytes(run_directory / 'supervisor-config-check.txt', 4096)
        == SUPERVISOR_CONFIG_CHECK
    )
    expected_systemd_dropin = (
        '[Service]\n'
        'ExecStart=\n'
        'ExecStart=/usr/bin/robotest-supervisor --config '
        f'/var/lib/robotest-supervisor/{run_id}/config.json\n'
    ).encode()
    systemd_dropin_exact = (
        _read_bounded_bytes(run_directory / 'systemd-dropin.conf', 4096) == expected_systemd_dropin
    )
    lifecycle_startup_result = _mapping(
        load_json(run_directory / STARTUP_RESULT_NAME), 'lifecycle startup result'
    )
    initial_affinity = _mapping(
        load_json(run_directory / 'runtime-affinity-initial.json'),
        'initial runtime affinity',
    )
    restored_affinity = _mapping(
        load_json(run_directory / 'runtime-affinity-restored.json'),
        'restored runtime affinity',
    )
    source_before = _mapping(
        load_json(run_directory / 'source-snapshot-before.json'), 'source snapshot before'
    )
    source_after = _mapping(
        load_json(run_directory / 'source-snapshot-after.json'), 'source snapshot after'
    )
    before_files = source_before.get('files')
    after_files = source_after.get('files')
    before_snapshot_hash = hashlib.sha256(canonical_json_bytes(before_files)).hexdigest()
    after_snapshot_hash = hashlib.sha256(canonical_json_bytes(after_files)).hexdigest()
    repository = Path(__file__).resolve().parents[1]
    expected_source_snapshot = source_snapshot(repository)
    upgrade_context = _mapping(context.get('upgrade_package'), 'context upgrade package')
    baseline_context = _mapping(context.get('baseline_package'), 'context baseline package')
    lifecycle_context = _mapping(context.get('lifecycle_evidence'), 'context lifecycle evidence')
    upgrade_path = Path(_string(upgrade_context.get('path'), 'upgrade path'))
    baseline_path = Path(_string(baseline_context.get('path'), 'baseline path'))
    package_directory = Path(
        _string(context.get('package_directory'), 'context package_directory')
    ).resolve(strict=True)
    candidate_manifest_path = Path(
        _string(package_binding.get('candidate_manifest'), 'candidate manifest')
    )
    canonical_candidate_manifest = (package_directory / 'build-a/SOURCE-MANIFEST.json').resolve(
        strict=True
    )
    candidate_manifest_is_canonical = (
        not candidate_manifest_path.is_symlink()
        and candidate_manifest_path.resolve(strict=True) == canonical_candidate_manifest
    )
    rebound_package = package_source_binding(repository, candidate_manifest_path)
    package_integrity = _mapping(
        load_json(run_directory / 'package-integrity.json'), 'package integrity'
    )
    package_source_rebuild = _mapping(
        load_json(run_directory / 'package-source-rebuild.json'),
        'package source rebuild',
    )
    _exact_keys(package_integrity, PACKAGE_INTEGRITY_KEYS, 'package integrity')
    rebound_integrity = verify_package_candidate(
        package_directory,
        upgrade_path,
        baseline_path,
        repository,
    )
    canonical_candidate_manifest_sha256 = file_sha256(canonical_candidate_manifest)
    upgrade_descriptor = package_artifact_descriptor(upgrade_path)
    baseline_descriptor = package_artifact_descriptor(baseline_path)
    validate_lifecycle_evidence(lifecycle, upgrade_descriptor, baseline_descriptor)
    lifecycle_upgrade = _mapping(lifecycle.get('upgrade_package'), 'lifecycle upgrade package')
    lifecycle_baseline = _mapping(lifecycle.get('baseline_package'), 'lifecycle baseline package')
    installed_package_state = _mapping(
        load_json(run_directory / 'installed-package-state.json'),
        'installed package state',
    )
    validate_installed_package_state(
        installed_package_state,
        lifecycle,
        upgrade_descriptor,
    )
    context_cpuset = _string(context.get('cpuset'), 'context cpuset')
    initial_affinity_join = validate_runtime_affinity_evidence(
        initial_affinity,
        phase='initial',
        main_pid=main_pid,
        managed_child_pid=original_child_pid,
        managed_child_pgid=original_pgid,
        expected_cpuset=context_cpuset,
    )
    restored_child_pid = _integer(
        restored_details.get('child_pid'), 'restored child PID', minimum=2
    )
    restored_child_pgid = _integer(
        restored_details.get('child_pgid'), 'restored child PGID', minimum=2
    )
    restored_affinity_join = validate_runtime_affinity_evidence(
        restored_affinity,
        phase='restored',
        main_pid=main_pid,
        managed_child_pid=restored_child_pid,
        managed_child_pgid=restored_child_pgid,
        expected_cpuset=context_cpuset,
    )
    initial_status_child = validate_supervisor_status(initial_status, 'initial status')
    restored_status_child = validate_supervisor_status(restored_status, 'restored status')
    final_status_child = validate_supervisor_status(
        supervisor_status_final, 'final supervisor status'
    )
    isolation_domain = _integer(isolation.get('ros_domain_id'), 'isolation ROS domain')
    isolation_partition = _string(isolation.get('gz_partition'), 'isolation Gazebo partition')
    validate_systemd_owners(
        initial_owners,
        ros_domain_id=isolation_domain,
        gz_partition=isolation_partition,
        expected_pids=[
            process['pid']
            for process in initial_affinity_join['processes']
            if process['role'] != 'supervisor_main'
        ],
        name='initial systemd owners',
    )
    validate_systemd_owners(
        restored_owners,
        ros_domain_id=isolation_domain,
        gz_partition=isolation_partition,
        expected_pids=[
            process['pid']
            for process in restored_affinity_join['processes']
            if process['role'] != 'supervisor_main'
        ],
        name='restored systemd owners',
    )
    _exact_keys(
        original_group_evidence,
        PROCESS_GROUP_EVIDENCE_KEYS,
        'original group evidence',
    )
    if (
        _integer(
            original_group_evidence.get('schema_version'),
            'original group evidence schema_version',
        )
        != 1
    ):
        raise EvidenceError('original group evidence schema_version must be 1')
    _utc_timestamp(
        original_group_evidence.get('captured_utc'),
        'original group evidence captured_utc',
    )
    original_group_members = _mapping_list(
        original_group_evidence.get('members'), 'original group evidence members'
    )
    status_projection_valid = (
        initial_status.get('healthy') is True
        and initial_status.get('ready') is True
        and initial_status.get('persistence_healthy') is True
        and initial_status.get('shutting_down') is False
        and initial_status.get('event_count') == baseline_sequence
        and initial_status.get('dropped_events') == 0
        and initial_status_child.get('required') is True
        and initial_status_child.get('running') is True
        and initial_status_child.get('heartbeat_fresh') is True
        and initial_status_child.get('circuit_open') is False
        and initial_status_child.get('pid') == original_child_pid
        and initial_status_child.get('pgid') == original_pgid
        and initial_status_child.get('restart_count') == 0
        and restored_status.get('healthy') is True
        and restored_status.get('ready') is True
        and restored_status.get('persistence_healthy') is True
        and restored_status.get('shutting_down') is False
        and restored_status.get('event_count') == recovery_end_sequence
        and restored_status.get('dropped_events') == 0
        and restored_status_child.get('required') is True
        and restored_status_child.get('running') is True
        and restored_status_child.get('heartbeat_fresh') is True
        and restored_status_child.get('circuit_open') is False
        and restored_status_child.get('pid') == restored_child_pid
        and restored_status_child.get('pgid') == restored_child_pgid
        and restored_status_child.get('restart_count') == 1
        and supervisor_status_final.get('healthy') is True
        and supervisor_status_final.get('ready') is False
        and supervisor_status_final.get('persistence_healthy') is True
        and supervisor_status_final.get('shutting_down') is True
        and supervisor_status_final.get('event_count') == len(events)
        and supervisor_status_final.get('dropped_events') == 0
        and final_status_child.get('running') is False
        and final_status_child.get('heartbeat_fresh') is False
        and final_status_child.get('pid') == 0
        and final_status_child.get('pgid') == 0
        and final_status_child.get('restart_count') == 1
        and final_status_child.get('last_exit_code') == 143
    )
    context_started_utc = datetime.fromisoformat(
        _utc_timestamp(context.get('started_utc'), 'context started_utc').replace('Z', '+00:00')
    )
    startup_started_utc = datetime.fromisoformat(
        _utc_timestamp(
            lifecycle_startup_result.get('started_utc'), 'startup result started_utc'
        ).replace('Z', '+00:00')
    )
    startup_completed_utc = datetime.fromisoformat(
        _utc_timestamp(
            lifecycle_startup_result.get('completed_utc'), 'startup result completed_utc'
        ).replace('Z', '+00:00')
    )

    checks: dict[str, bool] = {}
    failures: list[str] = []
    _add_check(
        checks,
        failures,
        'healthz_remained_200',
        bool(health_samples)
        and all(
            _mapping(sample['details'], 'health sample').get('http_status') == 200
            for sample in health_samples
        ),
    )
    _add_check(
        checks,
        failures,
        'ready_503_within_3s',
        ready_unavailable_marker_exact and detection_s <= READY_FAILURE_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'readiness_false_throughout_unavailable_interval',
        readiness_interval_valid,
    )
    _add_check(
        checks,
        failures,
        'original_process_group_empty_within_5s',
        group_cleanup_s <= PROCESS_GROUP_EXIT_TARGET_S
        and _mapping(group_empty['details'], 'group empty').get('member_count') == 0
        and original_group_evidence.get('pgid') == original_pgid
        and original_group_evidence.get('member_count') == 0
        and original_group_members == [],
    )
    _add_check(checks, failures, 'exactly_one_failure_event', len(failures_detected) == 1)
    _add_check(checks, failures, 'exactly_one_restart_scheduled', len(restart_events) == 1)
    _add_check(checks, failures, 'exactly_one_replacement_child_start', len(child_starts) == 1)
    _add_check(checks, failures, 'readiness_false_event_present', len(ready_false) == 1)
    _add_check(checks, failures, 'readiness_true_event_present', len(ready_true) == 1)
    _add_check(
        checks,
        failures,
        'exact_causal_supervisor_event_chain',
        startup_prefix_valid and causal_chain_valid,
    )
    _add_check(
        checks,
        failures,
        'post_observation_shutdown_is_exact',
        post_observation_shutdown_valid,
    )
    _add_check(
        checks,
        failures,
        'frozen_first_backoff_scheduled',
        len(restart_events) == 1
        and restart_events[0].get('backoff_ms') == EXPECTED_BACKOFF_MS
        and restart_events[0].get('restart_attempt') == 1,
    )
    _add_check(
        checks,
        failures,
        'actual_backoff_is_bounded',
        actual_backoff_s is not None and 0.9 <= actual_backoff_s <= 3.0,
    )
    _add_check(
        checks,
        failures,
        'ready_restored_within_30s_of_detection',
        event_recovery_s is not None and 0.0 <= event_recovery_s <= RECOVERY_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'observed_recovery_is_bounded',
        0.0 <= observed_recovery_s <= RECOVERY_TARGET_S,
    )
    _add_check(
        checks,
        failures,
        'supervisor_main_pid_stable',
        restored_details.get('main_pid') == main_pid
        and service_final.get('main_pid_before_stop') == main_pid,
    )
    _add_check(
        checks,
        failures,
        'supervisor_status_snapshots_are_exact',
        status_projection_valid,
    )
    _add_check(
        checks,
        failures,
        'systemd_did_not_restart_supervisor',
        initial_details.get('systemd_nrestarts') == 0
        and restored_details.get('systemd_nrestarts') == 0
        and service_final.get('nrestarts_before_stop') == 0
        and service_final.get('active_before_stop') is True
        and service_final.get('enabled_before_stop') is False,
    )
    _add_check(
        checks,
        failures,
        'only_one_systemd_owned_ros_service',
        initial_details.get('systemd_owned_ros_service_count') == 1
        and restored_details.get('systemd_owned_ros_service_count') == 1
        and initial_owners.get('unit_count') == 1
        and restored_owners.get('unit_count') == 1,
    )
    _add_check(
        checks,
        failures,
        'replacement_process_group_is_new',
        _integer(restored_details.get('child_pgid'), 'restored PGID', minimum=1) != original_pgid,
    )
    _add_check(
        checks,
        failures,
        'restored_status_is_exactly_ready',
        restored_status.get('schema_version') == 1
        and restored_status.get('ready') is True
        and restored_status.get('healthy') is True
        and restored_status_child.get('name') == EXPECTED_CHILD
        and restored_status_child.get('running') is True
        and restored_status_child.get('heartbeat_fresh') is True
        and restored_status_child.get('restart_count') == 1,
    )
    _add_check(
        checks,
        failures,
        'controller_identity_is_exact',
        target.get('pid') == injection_details.get('target_pid')
        and target.get('pgid') == original_pgid
        and target.get('root_pid') == original_child_pid
        and target.get('unit') == EXPECTED_UNIT
        and target.get('ros_domain_id') == str(isolation.get('ros_domain_id'))
        and target.get('gz_partition') == isolation.get('gz_partition')
        and Path(str(target.get('executable', ''))).name == 'controller_server'
        and len(target_lineage) >= 2
        and target_lineage[0].get('pid') == original_child_pid
        and target_lineage[-1].get('pid') == target.get('pid'),
    )
    _add_check(
        checks,
        failures,
        'mission_was_active_before_injection',
        active_goal_schema == ACTIVE_GOAL_SCHEMA_VERSION
        and active_goal_verdict == 'PASS'
        and active_goal_observed_ns > 0
        and active_goal_elapsed_s <= 60.0
        and active_goal_pid == injection_details.get('mission_pid')
        and active_goal_process_alive is True
        and active_goal_node == MISSION_RUNNER_NODE
        and active_goal_name == FOLLOW_WAYPOINTS_ACTION
        and active_goal_type == FOLLOW_WAYPOINTS_ACTION_TYPE
        and UUID_RE.fullmatch(active_goal_uuid) is not None
        and active_goal_accepted_stamp_ns > 0
        and active_goal_status_code == 2
        and active_goal_status == 'EXECUTING'
        and active_goal_status_count <= 1024
        and active_goal_attempt_count <= 2048
        and active_goal_domain == str(isolation.get('ros_domain_id'))
        and active_goal_partition == isolation.get('gz_partition')
        and mission_runner_is_goal_capable is True
        and active_client_evidence_exact,
    )
    _add_check(
        checks,
        failures,
        'published_process_groups_are_bound',
        mission_started_details.get('pid') == interrupted_pgid
        and injection_details.get('mission_pid') == interrupted_pgid
        and active_goal_pid == interrupted_pgid
        and probe_started_details.get('pid') == probe_pgid
        and followup_started_details.get('pid') == followup_pgid,
    )
    _add_check(checks, failures, 'interrupted_mission_pair_is_consistent', interrupted_pair_valid)
    _add_check(
        checks,
        failures,
        'interrupted_completion_matches_outcome',
        interrupted_event_exit == interrupted_exit
        and interrupted_event_terminated is interrupted_terminated
        and interrupted_event_group_empty is interrupted_group_empty,
    )
    _add_check(
        checks,
        failures,
        'interrupted_mission_never_success',
        interrupted_exit != 0 and not interrupted_success,
    )
    _add_check(
        checks,
        failures,
        'interrupted_mission_was_bounded',
        interrupted_bounded and interrupted_group_empty,
    )
    _add_check(checks, failures, 'followup_json_csv_reconcile', followup_csv_valid)
    _add_check(
        checks,
        failures,
        'exactly_one_successful_followup_completion',
        followup_event_exit == 0 and followup_event_exit == followup_verdict.get('exit_code'),
    )
    _add_check(
        checks,
        failures,
        'fresh_followup_mission_succeeded',
        followup_mission_is_exact
        and followup_verdict.get('exit_code') == 0
        and followup_verdict.get('phase2_action_integration_status') == 'PASS'
        and followup_verdict.get('expected_outcome_met') is True
        and followup_measurements.get('goal_status') == 'SUCCEEDED'
        and followup_identity.get('mission_sha256') == followup_mission_sha,
    )
    _add_check(
        checks,
        failures,
        'event_store_has_no_loss',
        event_meta_schema == 1
        and event_meta_saturated is False
        and event_meta_dropped == 0
        and event_meta_last_sequence == len(events),
    )
    _add_check(
        checks,
        failures,
        'package_source_binding_passed',
        package_binding.get('verdict') == 'PASS'
        and rebound_package.get('verdict') == 'PASS'
        and candidate_manifest_is_canonical
        and _type_exact_json_equal(package_binding, rebound_package)
        and rebound_package.get('candidate_manifest_sha256')
        == package_binding.get('candidate_manifest_sha256')
        == rebound_integrity.get('source_manifest_sha256')
        == canonical_candidate_manifest_sha256,
    )
    _add_check(
        checks,
        failures,
        'independent_package_reproducibility_passed',
        package_integrity.get('verdict') == 'PASS'
        and _type_exact_json_equal(package_integrity, rebound_integrity),
    )
    _add_check(
        checks,
        failures,
        'package_source_rebuild_is_exact',
        package_source_rebuild_is_exact(
            package_source_rebuild,
            repository,
            package_directory,
            upgrade_path,
        ),
    )
    _add_check(
        checks,
        failures,
        'package_artifacts_match_lifecycle',
        file_sha256(upgrade_path)
        == _sha256(upgrade_context.get('sha256'), 'upgrade context SHA')
        == lifecycle_upgrade.get('sha256')
        and file_sha256(baseline_path)
        == _sha256(baseline_context.get('sha256'), 'baseline context SHA')
        == lifecycle_baseline.get('sha256'),
    )
    _add_check(
        checks,
        failures,
        'package_lifecycle_passed',
        file_sha256(Path(_string(lifecycle_context.get('path'), 'lifecycle evidence path')))
        == _sha256(lifecycle_context.get('sha256'), 'lifecycle evidence context SHA')
        == file_sha256(run_directory / 'package-lifecycle.json'),
    )
    _add_check(
        checks,
        failures,
        'installed_package_state_matches_lifecycle',
        installed_package_state.get('verdict') == 'PASS',
    )
    _add_check(
        checks,
        failures,
        'lifecycle_startup_result_is_exact',
        lifecycle_startup_result_is_exact(lifecycle_startup_result)
        and context_started_utc
        <= startup_started_utc
        <= startup_completed_utc
        <= initial_affinity_join['captured_utc'],
    )
    affinity_identity_fields = (
        'pid',
        'ppid',
        'pgid',
        'start_time_ticks',
        'executable',
        'cgroup',
    )
    initial_affinity_main = initial_affinity_join['main']
    restored_affinity_main = restored_affinity_join['main']
    initial_affinity_controller = initial_affinity_join['controller']
    _add_check(
        checks,
        failures,
        'runtime_cpu_affinity_is_limited',
        context_cpuset == EXPECTED_CPUSET
        and initial_affinity_join['limited'] is True
        and restored_affinity_join['limited'] is True
        and context_started_utc
        <= initial_affinity_join['captured_utc']
        <= restored_affinity_join['captured_utc']
        and all(
            initial_affinity_main.get(field) == restored_affinity_main.get(field)
            for field in affinity_identity_fields
        )
        and all(
            initial_affinity_controller.get(field) == target.get(field)
            for field in affinity_identity_fields
        ),
    )
    _add_check(
        checks,
        failures,
        'runtime_staging_is_bound',
        runtime_staging_evidence_is_exact(
            repository,
            context,
            staging,
            overlay_provenance,
            overlay_source_manifest_path,
            overlay_install_manifest_path,
        ),
    )
    exact_runtime_config = expected_supervisor_config(
        f'/var/lib/robotest-supervisor/{run_id}',
        isolation.get('ros_domain_id'),
        isolation.get('gz_partition'),
    )
    _add_check(
        checks,
        failures,
        'run_scoped_supervisor_config_is_exact',
        _type_exact_json_equal(supervisor_config, exact_runtime_config)
        and supervisor_config_check_exact
        and systemd_dropin_exact,
    )
    _add_check(
        checks,
        failures,
        'source_snapshot_unchanged_and_self_consistent',
        bool(expected_source_snapshot.get('files'))
        and source_before.get('snapshot_sha256') == before_snapshot_hash
        and source_after.get('snapshot_sha256') == after_snapshot_hash
        and before_snapshot_hash == after_snapshot_hash
        and _type_exact_json_equal(source_before, source_after)
        and _type_exact_json_equal(source_before, expected_source_snapshot),
    )
    _add_check(
        checks,
        failures,
        'unique_runtime_isolation',
        isinstance(isolation.get('ros_domain_id'), int)
        and 100 <= isolation['ros_domain_id'] <= 229
        and isolation.get('domain_was_unused') is True
        and isolation.get('partition_was_unused') is True
        and PARTITION_RE.fullmatch(str(isolation.get('gz_partition', ''))) is not None,
    )
    _add_check(
        checks,
        failures,
        'cleanup_complete_and_owned',
        cleanup.get('owned_paths_only') is True,
    )

    evidence_hashes = _relative_evidence_hashes(
        run_directory,
        {'scenario6-result.json', 'scenario6-result.csv', 'SHA256SUMS'},
    )
    return {
        'schema_version': SCHEMA_VERSION,
        'producer': PRODUCER,
        'identity': {
            'run_id': run_id,
            'started_utc': context.get('started_utc'),
            'completed_utc': utc_now(),
            'source_git_commit': context.get('source_git_commit'),
            'source_git_dirty': context.get('source_git_dirty'),
            'ros_domain_id': isolation.get('ros_domain_id'),
            'gz_partition': isolation.get('gz_partition'),
            'supervisor_unit': EXPECTED_UNIT,
            'managed_child': EXPECTED_CHILD,
        },
        'targets': {
            'heartbeat_period_wall_s': 0.5,
            'heartbeat_stale_wall_s': 2.0,
            'ready_failure_wall_s_max': READY_FAILURE_TARGET_S,
            'termination_allowance_wall_s': PROCESS_GROUP_EXIT_TARGET_S,
            'ready_restore_after_detection_wall_s_max': RECOVERY_TARGET_S,
            'restart_backoff_wall_s': [1.0, 2.0, 4.0, 8.0],
            'restart_attempt_limit': 4,
            'restart_window_wall_s': 60.0,
            'stable_reset_wall_s': 60.0,
        },
        'measurements': {
            'ready_503_after_injection_wall_s': detection_s,
            'original_group_empty_after_injection_wall_s': group_cleanup_s,
            'observed_ready_restore_after_503_wall_s': observed_recovery_s,
            'supervisor_recovery_time_wall_s': event_recovery_s,
            'actual_restart_backoff_wall_s': actual_backoff_s,
            'restart_scheduled_count': len(restart_events),
            'replacement_child_start_count': len(child_starts),
            'original_child_pid': original_child_pid,
            'original_child_pgid': original_pgid,
            'replacement_child_pid': restored_details.get('child_pid'),
            'replacement_child_pgid': restored_details.get('child_pgid'),
            'supervisor_main_pid': main_pid,
            'interrupted_mission_exit_code': interrupted_exit,
            'followup_mission_exit_code': followup_verdict.get('exit_code'),
        },
        'quality': {
            'checks': checks,
            'event_count': len(events),
            'event_trace_dropped': event_meta.get('dropped_events'),
            'timeline_count': len(timeline),
            'raw_evidence_sha256': evidence_hashes,
        },
        'verdict': {
            'status': 'PASS' if not failures else 'FAIL',
            'accepted': not failures,
            'failure_count': len(failures),
            'failures': failures,
        },
    }


def write_result(run_directory: Path) -> dict[str, Any]:
    """Write canonical Scenario 6 JSON and one-row CSV, including FAIL evidence."""
    try:
        result = evaluate_run(run_directory)
    except Exception as exc:
        context_path = run_directory / 'context.json'
        run_id = 'unknown'
        if context_path.is_file():
            with contextlib.suppress(EvidenceError):
                run_id = str(_mapping(load_json(context_path), 'context').get('run_id', 'unknown'))
        result = {
            'schema_version': SCHEMA_VERSION,
            'producer': PRODUCER,
            'identity': {'run_id': run_id, 'completed_utc': utc_now()},
            'targets': {},
            'measurements': {},
            'quality': {'checks': {}, 'raw_evidence_sha256': {}},
            'verdict': {
                'status': 'FAIL',
                'accepted': False,
                'failure_count': 1,
                'failures': [f'evidence_error: {type(exc).__name__}: {exc}'],
            },
        }
    json_path = run_directory / 'scenario6-result.json'
    csv_path = run_directory / 'scenario6-result.csv'
    atomic_write_json(json_path, result)
    identity = _mapping(result.get('identity'), 'result identity')
    measurements = _mapping(result.get('measurements'), 'result measurements')
    verdict = _mapping(result.get('verdict'), 'result verdict')
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
    descriptor, temporary_name = tempfile.mkstemp(prefix='.scenario6-result.', dir=run_directory)
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='') as target:
            writer = csv.DictWriter(target, fieldnames=list(row), lineterminator='\n')
            writer.writeheader()
            writer.writerow({key: '' if value is None else value for key, value in row.items()})
            target.flush()
            os.fsync(target.fileno())
        os.chmod(temporary, 0o644)
        os.replace(temporary, csv_path)
    except BaseException:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
        raise
    return result


def write_checksums(run_directory: Path) -> None:
    """Write and immediately verify a sorted checksum manifest."""
    hashes = _relative_evidence_hashes(run_directory, {'SHA256SUMS'})
    payload = ''.join(f'{digest}  {name}\n' for name, digest in sorted(hashes.items())).encode()
    _atomic_write(run_directory / 'SHA256SUMS', payload)
    for name, expected in hashes.items():
        if file_sha256(run_directory / name) != expected:
            raise EvidenceError(f'checksum changed during manifest creation: {name}')


def _publication_directory(
    path: Path,
    label: str,
    *,
    owner_uid: int,
    mode: int | None = None,
) -> os.stat_result:
    try:
        metadata = os.lstat(path)
    except OSError as exc:
        raise EvidenceError(f'{label} is unavailable: {path}') from exc
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise EvidenceError(f'{label} must be a non-symlink directory: {path}')
    if metadata.st_uid != owner_uid:
        raise EvidenceError(f'{label} has the wrong owner: {path}')
    if mode is not None and stat.S_IMODE(metadata.st_mode) != mode:
        raise EvidenceError(f'{label} mode is not {mode:04o}: {path}')
    return metadata


def _publication_file_digest(path: Path, metadata: os.stat_result, label: str) -> str:
    flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise EvidenceError(f'cannot open {label} without following links: {path}') from exc
    digest = hashlib.sha256()
    observed = 0
    try:
        opened = os.fstat(descriptor)
        if (
            not stat.S_ISREG(opened.st_mode)
            or (opened.st_dev, opened.st_ino) != (metadata.st_dev, metadata.st_ino)
            or opened.st_nlink != 1
        ):
            raise EvidenceError(f'{label} changed identity while opening: {path}')
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            observed += len(chunk)
            if observed > MAX_PUBLICATION_FILE_BYTES:
                raise EvidenceError(f'{label} exceeds the per-file publication bound: {path}')
            digest.update(chunk)
        closed = os.fstat(descriptor)
        if (
            observed != metadata.st_size
            or closed.st_size != metadata.st_size
            or closed.st_mtime_ns != metadata.st_mtime_ns
        ):
            raise EvidenceError(f'{label} changed while hashing: {path}')
    finally:
        os.close(descriptor)
    return digest.hexdigest()


def _publication_tree_snapshot(
    root: Path,
    label: str,
    *,
    owner_uid: int,
    owner_gid: int,
    directory_mode: int,
    file_mode: int,
) -> tuple[dict[str, tuple[str, int]], tuple[str, ...], int]:
    _publication_directory(root, label, owner_uid=owner_uid, mode=directory_mode)
    files: dict[str, tuple[str, int]] = {}
    directories: list[str] = []
    total_bytes = 0
    entry_count = 0
    for directory, raw_directory_names, raw_file_names in os.walk(root, followlinks=False):
        current = Path(directory)
        relative_directory = current.relative_to(root)
        if len(relative_directory.parts) > MAX_PUBLICATION_DEPTH:
            raise EvidenceError(f'{label} exceeds the publication depth bound')
        current_metadata = os.lstat(current)
        if (
            not stat.S_ISDIR(current_metadata.st_mode)
            or stat.S_ISLNK(current_metadata.st_mode)
            or current_metadata.st_uid != owner_uid
            or current_metadata.st_gid != owner_gid
            or stat.S_IMODE(current_metadata.st_mode) != directory_mode
        ):
            raise EvidenceError(f'{label} contains an unsafe directory: {current}')
        raw_directory_names.sort()
        raw_file_names.sort()
        for name in raw_directory_names:
            path = current / name
            metadata = os.lstat(path)
            entry_count += 1
            if (
                stat.S_ISLNK(metadata.st_mode)
                or not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != owner_uid
                or metadata.st_gid != owner_gid
                or stat.S_IMODE(metadata.st_mode) != directory_mode
            ):
                raise EvidenceError(f'{label} contains an unsafe directory entry: {path}')
            relative = path.relative_to(root).as_posix()
            if (
                not relative
                or len(relative.encode('utf-8')) > 4096
                or any(character in relative for character in '\r\n')
            ):
                raise EvidenceError(f'{label} contains an unsafe relative directory path')
            directories.append(relative)
        for name in raw_file_names:
            path = current / name
            metadata = os.lstat(path)
            entry_count += 1
            if (
                stat.S_ISLNK(metadata.st_mode)
                or not stat.S_ISREG(metadata.st_mode)
                or metadata.st_nlink != 1
                or metadata.st_uid != owner_uid
                or metadata.st_gid != owner_gid
                or stat.S_IMODE(metadata.st_mode) != file_mode
            ):
                raise EvidenceError(f'{label} contains a linked or special file: {path}')
            if not 0 <= metadata.st_size <= MAX_PUBLICATION_FILE_BYTES:
                raise EvidenceError(f'{label} file exceeds the publication bound: {path}')
            relative = path.relative_to(root).as_posix()
            if (
                not relative
                or len(relative.encode('utf-8')) > 4096
                or any(character in relative for character in '\r\n')
            ):
                raise EvidenceError(f'{label} contains an unsafe relative file path')
            digest = _publication_file_digest(path, metadata, label)
            files[relative] = (digest, metadata.st_size)
            total_bytes += metadata.st_size
            if total_bytes > MAX_PUBLICATION_TOTAL_BYTES:
                raise EvidenceError(f'{label} exceeds the total publication byte bound')
        if entry_count > MAX_EVIDENCE_FILES:
            raise EvidenceError(f'{label} exceeds the publication entry-count bound')
    return files, tuple(sorted(directories)), total_bytes


def _validate_publication_manifest(
    root: Path,
    files: Mapping[str, tuple[str, int]],
    label: str,
) -> None:
    manifest = root / 'SHA256SUMS'
    if 'SHA256SUMS' not in files:
        raise EvidenceError(f'{label} is missing SHA256SUMS')
    payload = _read_bounded_bytes(manifest, MAX_TEXT_BYTES)
    try:
        text = payload.decode('ascii')
    except UnicodeDecodeError as exc:
        raise EvidenceError(f'{label} SHA256SUMS is not ASCII') from exc
    if not text or not text.endswith('\n'):
        raise EvidenceError(f'{label} SHA256SUMS has an incomplete final line')
    declared: dict[str, str] = {}
    for line_number, line in enumerate(text.splitlines(), 1):
        match = re.fullmatch(r'([0-9a-f]{64})  ([^\r\n]+)', line)
        if match is None:
            raise EvidenceError(f'{label} SHA256SUMS line {line_number} is invalid')
        digest, relative = match.groups()
        path = Path(relative)
        if (
            relative != path.as_posix()
            or path.is_absolute()
            or '..' in path.parts
            or relative in declared
            or relative == 'SHA256SUMS'
        ):
            raise EvidenceError(f'{label} SHA256SUMS path is unsafe: {relative}')
        declared[relative] = digest
    expected = set(files) - {'SHA256SUMS'}
    if set(declared) != expected:
        raise EvidenceError(f'{label} SHA256SUMS coverage is not exact')
    for relative, digest in declared.items():
        if files[relative][0] != digest:
            raise EvidenceError(f'{label} SHA256SUMS mismatch: {relative}')
    canonical = ''.join(
        f'{declared[relative]}  {relative}\n' for relative in sorted(declared)
    ).encode('ascii')
    if payload != canonical:
        raise EvidenceError(f'{label} SHA256SUMS is not canonical')


def _copy_publication_tree(
    source: Path,
    destination: Path,
    source_files: Mapping[str, tuple[str, int]],
    source_directories: Sequence[str],
) -> None:
    for relative in sorted(source_directories, key=lambda value: (value.count('/'), value)):
        path = destination / relative
        try:
            path.mkdir(mode=0o750)
        except OSError as exc:
            raise EvidenceError(f'cannot create publication directory: {relative}') from exc
        os.chmod(path, 0o750)
    for relative in sorted(source_files):
        source_path = source / relative
        destination_path = destination / relative
        source_flags = os.O_RDONLY | getattr(os, 'O_NOFOLLOW', 0)
        destination_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, 'O_NOFOLLOW', 0)
        try:
            source_descriptor = os.open(source_path, source_flags)
            destination_descriptor = os.open(destination_path, destination_flags, 0o640)
        except OSError as exc:
            with contextlib.suppress(UnboundLocalError, OSError):
                os.close(source_descriptor)
            raise EvidenceError(f'cannot create publication file: {relative}') from exc
        digest = hashlib.sha256()
        copied = 0
        try:
            source_metadata = os.fstat(source_descriptor)
            if not stat.S_ISREG(source_metadata.st_mode) or source_metadata.st_nlink != 1:
                raise EvidenceError(f'publication source changed identity: {relative}')
            while True:
                chunk = os.read(source_descriptor, 1024 * 1024)
                if not chunk:
                    break
                copied += len(chunk)
                if copied > MAX_PUBLICATION_FILE_BYTES:
                    raise EvidenceError(f'publication source grew beyond its bound: {relative}')
                digest.update(chunk)
                view = memoryview(chunk)
                while view:
                    written = os.write(destination_descriptor, view)
                    if written <= 0:
                        raise EvidenceError(f'short publication write: {relative}')
                    view = view[written:]
            expected_digest, expected_size = source_files[relative]
            if copied != expected_size or digest.hexdigest() != expected_digest:
                raise EvidenceError(f'publication source changed while copying: {relative}')
            os.fchmod(destination_descriptor, 0o640)
            os.fsync(destination_descriptor)
        finally:
            os.close(source_descriptor)
            os.close(destination_descriptor)
    for relative in sorted(source_directories, key=lambda value: value.count('/'), reverse=True):
        descriptor = os.open(
            destination / relative,
            os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
        )
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    os.chmod(destination, 0o750)
    descriptor = os.open(
        destination,
        os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def _rename_directory_noreplace(
    parent_descriptor: int,
    source_name: str,
    destination_name: str,
) -> None:
    libc = ctypes.CDLL(None, use_errno=True)
    renameat2 = getattr(libc, 'renameat2', None)
    if renameat2 is None:
        raise EvidenceError('renameat2(RENAME_NOREPLACE) is unavailable')
    renameat2.argtypes = (
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_int,
        ctypes.c_char_p,
        ctypes.c_uint,
    )
    renameat2.restype = ctypes.c_int
    result = renameat2(
        parent_descriptor,
        os.fsencode(source_name),
        parent_descriptor,
        os.fsencode(destination_name),
        1,
    )
    if result != 0:
        error_number = ctypes.get_errno()
        if error_number == errno.EEXIST:
            raise EvidenceError('public evidence destination appeared before no-replace rename')
        raise EvidenceError(f'no-replace evidence rename failed: {os.strerror(error_number)}')
    os.fsync(parent_descriptor)


def publish_run_evidence(
    source: Path,
    destination: Path,
    repository: Path,
    *,
    live_root: Path = LIVE_EVIDENCE_ROOT,
    expected_source_uid: int = 0,
    require_unprivileged: bool = True,
    temporary_name_factory: Callable[[], str] | None = None,
    before_rename: Callable[[Path], None] | None = None,
) -> dict[str, Any]:
    """Copy one frozen root evidence tree and publish it with no-replace rename."""
    publisher_uid = os.geteuid()
    if require_unprivileged and publisher_uid == 0:
        raise EvidenceError('evidence publication must run unprivileged')
    for path, label in (
        (source, 'live evidence source'),
        (destination, 'public evidence destination'),
        (repository, 'repository'),
        (live_root, 'live evidence root'),
    ):
        if not path.is_absolute() or '..' in path.parts:
            raise EvidenceError(f'{label} must be a canonical absolute path')
    try:
        resolved_repository = repository.resolve(strict=True)
        resolved_live_root = live_root.resolve(strict=True)
    except OSError as exc:
        raise EvidenceError('publication root is unavailable') from exc
    if resolved_repository != repository or resolved_live_root != live_root:
        raise EvidenceError('publication roots must not traverse symbolic links')
    run_id = source.name
    if RUN_ID_RE.fullmatch(run_id) is None or source.parent != live_root:
        raise EvidenceError('live evidence source is outside the exact run root')
    expected_destination = repository / PUBLIC_EVIDENCE_RELATIVE_ROOT / run_id
    if destination != expected_destination:
        raise EvidenceError('public evidence destination is not canonical')

    live_metadata = _publication_directory(
        live_root,
        'live evidence root',
        owner_uid=expected_source_uid,
        mode=0o710,
    )
    source_metadata = _publication_directory(
        source,
        'live evidence source',
        owner_uid=expected_source_uid,
        mode=0o550,
    )
    if source_metadata.st_gid != live_metadata.st_gid:
        raise EvidenceError('live evidence source group differs from its root')
    marker = source / LIVE_EVIDENCE_MARKER
    marker_metadata = os.lstat(marker)
    if (
        not stat.S_ISREG(marker_metadata.st_mode)
        or stat.S_ISLNK(marker_metadata.st_mode)
        or marker_metadata.st_nlink != 1
        or marker_metadata.st_uid != expected_source_uid
        or marker_metadata.st_gid != source_metadata.st_gid
        or stat.S_IMODE(marker_metadata.st_mode) != 0o440
        or _read_bounded_bytes(marker, 4096) != f'{run_id}\n'.encode()
    ):
        raise EvidenceError('live evidence ownership marker is invalid')
    source_files, source_directories, total_bytes = _publication_tree_snapshot(
        source,
        'live evidence source',
        owner_uid=expected_source_uid,
        owner_gid=source_metadata.st_gid,
        directory_mode=0o550,
        file_mode=0o440,
    )
    _validate_publication_manifest(source, source_files, 'live evidence source')

    _publication_directory(repository, 'repository', owner_uid=publisher_uid)
    current = repository
    for component in PUBLIC_EVIDENCE_RELATIVE_ROOT.parts[:-1]:
        current = current / component
        metadata = _publication_directory(
            current,
            'public evidence ancestor',
            owner_uid=publisher_uid,
        )
        if stat.S_IMODE(metadata.st_mode) & 0o022:
            raise EvidenceError(f'public evidence ancestor is group/other writable: {current}')
    public_parent = repository / PUBLIC_EVIDENCE_RELATIVE_ROOT
    with contextlib.suppress(FileExistsError):
        os.mkdir(public_parent, 0o750)
    parent_metadata = _publication_directory(
        public_parent,
        'public evidence parent',
        owner_uid=publisher_uid,
    )
    if stat.S_IMODE(parent_metadata.st_mode) & 0o022:
        raise EvidenceError('public evidence parent is group/other writable')
    parent_descriptor = os.open(
        public_parent,
        os.O_RDONLY | os.O_DIRECTORY | getattr(os, 'O_NOFOLLOW', 0),
    )
    opened_parent = os.fstat(parent_descriptor)
    if (opened_parent.st_dev, opened_parent.st_ino) != (
        parent_metadata.st_dev,
        parent_metadata.st_ino,
    ):
        os.close(parent_descriptor)
        raise EvidenceError('public evidence parent changed while opening')
    temporary_name = ''
    temporary_created = False
    published = False
    try:
        try:
            os.stat(run_id, dir_fd=parent_descriptor, follow_symlinks=False)
        except FileNotFoundError:
            pass
        else:
            raise EvidenceError('public evidence destination already exists')
        token = (
            temporary_name_factory()
            if temporary_name_factory is not None
            else secrets.token_hex(12)
        )
        if re.fullmatch(r'[A-Za-z0-9_-]{1,64}', token) is None:
            raise EvidenceError('publication temporary token is invalid')
        temporary_name = f'.{run_id}.publish-{token}'
        try:
            os.mkdir(temporary_name, 0o700, dir_fd=parent_descriptor)
        except FileExistsError as exc:
            raise EvidenceError('publication temporary directory already exists') from exc
        temporary_created = True
        descriptor_root = Path(f'/proc/self/fd/{parent_descriptor}')
        temporary = descriptor_root / temporary_name
        _copy_publication_tree(source, temporary, source_files, source_directories)
        copied_gid = os.lstat(temporary).st_gid
        copied_files, copied_directories, copied_total = _publication_tree_snapshot(
            temporary,
            'copied evidence',
            owner_uid=publisher_uid,
            owner_gid=copied_gid,
            directory_mode=0o750,
            file_mode=0o640,
        )
        _validate_publication_manifest(temporary, copied_files, 'copied evidence')
        if (
            copied_files != source_files
            or copied_directories != source_directories
            or copied_total != total_bytes
        ):
            raise EvidenceError('copied evidence differs from the frozen source')
        if before_rename is not None:
            before_rename(destination)
        _rename_directory_noreplace(parent_descriptor, temporary_name, run_id)
        published = True
        final = descriptor_root / run_id
        final_files, final_directories, final_total = _publication_tree_snapshot(
            final,
            'published evidence',
            owner_uid=publisher_uid,
            owner_gid=copied_gid,
            directory_mode=0o750,
            file_mode=0o640,
        )
        _validate_publication_manifest(final, final_files, 'published evidence')
        if (
            final_files != source_files
            or final_directories != source_directories
            or final_total != total_bytes
        ):
            raise EvidenceError('published evidence differs from the frozen source')
        current_parent = os.lstat(public_parent)
        if (current_parent.st_dev, current_parent.st_ino) != (
            parent_metadata.st_dev,
            parent_metadata.st_ino,
        ):
            raise EvidenceError('public evidence parent changed during publication')
    finally:
        if temporary_created and not published:
            with contextlib.suppress(OSError):
                shutil.rmtree(temporary_name, dir_fd=parent_descriptor)
        os.close(parent_descriptor)
    return {
        'destination': str(destination),
        'file_count': len(source_files),
        'total_bytes': total_bytes,
    }


def _parse_details(values: Sequence[str]) -> dict[str, Any]:
    details: dict[str, Any] = {}
    for value in values:
        if '=' not in value:
            raise EvidenceError(f'--detail must be KEY=VALUE: {value!r}')
        key, raw = value.split('=', 1)
        if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', key) or key in details:
            raise EvidenceError(f'invalid or duplicate detail key: {key!r}')
        try:
            details[key] = json.loads(raw)
        except json.JSONDecodeError:
            details[key] = raw
    return details


def build_parser() -> argparse.ArgumentParser:
    """Build the bounded helper CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    allocate = subparsers.add_parser('allocate-isolation')
    allocate.add_argument('--run-id', required=True)
    allocate.add_argument('--output', type=Path, required=True)

    render = subparsers.add_parser('render-config')
    render.add_argument('--template', type=Path, required=True)
    render.add_argument('--output', type=Path, required=True)
    render.add_argument('--state-directory', required=True)
    render.add_argument('--ros-domain-id', type=int, required=True)
    render.add_argument('--gz-partition', required=True)

    overlay_script = subparsers.add_parser('render-overlay-stage-script')
    overlay_script.add_argument('--template', type=Path, required=True)
    overlay_script.add_argument('--output', type=Path, required=True)
    overlay_script.add_argument('--project-root', type=Path, required=True)
    overlay_script.add_argument('--evidence-directory', type=Path, required=True)

    followup = subparsers.add_parser('render-followup')
    followup.add_argument('--output', type=Path, required=True)

    binding = subparsers.add_parser('package-binding')
    binding.add_argument('--repository', type=Path, required=True)
    binding.add_argument('--manifest', type=Path, required=True)
    binding.add_argument('--output', type=Path, required=True)

    integrity = subparsers.add_parser('verify-package-candidate')
    integrity.add_argument('--package-directory', type=Path, required=True)
    integrity.add_argument('--upgrade', type=Path, required=True)
    integrity.add_argument('--baseline', type=Path, required=True)
    integrity.add_argument('--repository', type=Path, required=True)
    integrity.add_argument('--output', type=Path, required=True)

    rebuild = subparsers.add_parser('package-source-rebuild')
    rebuild.add_argument('--repository', type=Path, required=True)
    rebuild.add_argument('--package-directory', type=Path, required=True)
    rebuild.add_argument('--upgrade', type=Path, required=True)
    rebuild.add_argument('--rebuilt-directory', type=Path, required=True)
    rebuild.add_argument('--output', type=Path, required=True)

    rebuild_import = subparsers.add_parser('import-package-source-rebuild')
    rebuild_import.add_argument('--repository', type=Path, required=True)
    rebuild_import.add_argument('--package-directory', type=Path, required=True)
    rebuild_import.add_argument('--upgrade', type=Path, required=True)
    rebuild_import.add_argument('--rebuilt-directory', type=Path, required=True)
    rebuild_import.add_argument('--attestation', type=Path, required=True)
    rebuild_import.add_argument('--output', type=Path, required=True)

    lifecycle = subparsers.add_parser('validate-lifecycle')
    lifecycle.add_argument('--evidence', type=Path, required=True)
    lifecycle.add_argument('--upgrade', type=Path, required=True)
    lifecycle.add_argument('--baseline', type=Path, required=True)

    installed = subparsers.add_parser('capture-installed-state')
    installed.add_argument('--evidence', type=Path, required=True)
    installed.add_argument('--upgrade', type=Path, required=True)
    installed.add_argument('--baseline', type=Path, required=True)
    installed.add_argument('--smoke-script', type=Path, required=True)
    installed.add_argument('--output', type=Path, required=True)

    snapshot = subparsers.add_parser('source-snapshot')
    snapshot.add_argument('--repository', type=Path, required=True)
    snapshot.add_argument('--output', type=Path, required=True)

    record = subparsers.add_parser('record')
    record.add_argument('--timeline', type=Path, required=True)
    record.add_argument('--kind', required=True)
    record.add_argument('--detail', action='append', default=[])

    http = subparsers.add_parser('probe-http')
    http.add_argument('--timeline', type=Path, required=True)
    http.add_argument('--kind', required=True)
    http.add_argument('--url', required=True)
    http.add_argument('--timeout', type=float, default=1.0)

    fetch = subparsers.add_parser('fetch-json')
    fetch.add_argument('--url', required=True)
    fetch.add_argument('--output', type=Path, required=True)
    fetch.add_argument('--expected-status', type=int, default=200)

    controller = subparsers.add_parser('find-controller')
    controller.add_argument('--root-pid', type=int, required=True)
    controller.add_argument('--pgid', type=int, required=True)
    controller.add_argument('--ros-domain-id', type=int, required=True)
    controller.add_argument('--gz-partition', required=True)
    controller.add_argument('--unit', default=EXPECTED_UNIT)
    controller.add_argument('--output', type=Path, required=True)

    identity = subparsers.add_parser('assert-identity')
    identity.add_argument('--identity', type=Path, required=True)

    signal_identity = subparsers.add_parser('signal-identity')
    signal_identity.add_argument('--identity', type=Path, required=True)
    signal_identity.add_argument('--signal', choices=('TERM', 'KILL'), required=True)

    group = subparsers.add_parser('process-group')
    group.add_argument('--pgid', type=int, required=True)
    group.add_argument('--output', type=Path)
    group.add_argument('--require-empty', action='store_true')

    owners = subparsers.add_parser('systemd-owners')
    owners.add_argument('--ros-domain-id', type=int, required=True)
    owners.add_argument('--gz-partition', required=True)
    owners.add_argument('--expected-unit', default=EXPECTED_UNIT)
    owners.add_argument('--output', type=Path, required=True)

    affinity = subparsers.add_parser('capture-runtime-affinity')
    affinity.add_argument('--main-pid', type=int, required=True)
    affinity.add_argument('--managed-child-pid', type=int, required=True)
    affinity.add_argument('--managed-child-pgid', type=int, required=True)
    affinity.add_argument('--ros-domain-id', type=int, required=True)
    affinity.add_argument('--gz-partition', required=True)
    affinity.add_argument('--phase', choices=('initial', 'restored'), required=True)
    affinity.add_argument('--unit', default=EXPECTED_UNIT)
    affinity.add_argument('--expected-cpuset', default=EXPECTED_CPUSET)
    affinity.add_argument('--output', type=Path, required=True)

    ownership = subparsers.add_parser('validate-runtime-ownership')
    ownership.add_argument('--before', type=Path, required=True)
    ownership.add_argument('--after', type=Path, required=True)
    ownership.add_argument('--affinity', type=Path, required=True)

    compose = subparsers.add_parser('compose')
    compose.add_argument('--run-directory', type=Path, required=True)

    checksums = subparsers.add_parser('checksums')
    checksums.add_argument('--run-directory', type=Path, required=True)

    publish = subparsers.add_parser('publish-evidence')
    publish.add_argument('--source', type=Path, required=True)
    publish.add_argument('--destination', type=Path, required=True)
    publish.add_argument('--repository', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run one helper operation with stable nonzero failure behavior."""
    args = build_parser().parse_args(argv)
    try:
        if args.command == 'allocate-isolation':
            value = allocated_isolation(args.run_id)
            atomic_write_json(args.output, value)
        elif args.command == 'render-config':
            render_supervisor_config(
                args.template,
                args.output,
                args.state_directory,
                args.ros_domain_id,
                args.gz_partition,
            )
        elif args.command == 'render-overlay-stage-script':
            render_overlay_stage_script(
                args.template,
                args.output,
                args.project_root,
                args.evidence_directory,
            )
        elif args.command == 'render-followup':
            render_followup_mission(args.output)
        elif args.command == 'package-binding':
            value = package_source_binding(args.repository.resolve(), args.manifest)
            atomic_write_json(args.output, value)
            if value['verdict'] != 'PASS':
                return 1
        elif args.command == 'verify-package-candidate':
            atomic_write_json(
                args.output,
                verify_package_candidate(
                    args.package_directory,
                    args.upgrade,
                    args.baseline,
                    args.repository,
                ),
            )
        elif args.command == 'package-source-rebuild':
            atomic_write_json(
                args.output,
                package_source_rebuild_attestation(
                    args.repository,
                    args.package_directory,
                    args.upgrade,
                    args.rebuilt_directory,
                ),
            )
        elif args.command == 'import-package-source-rebuild':
            atomic_write_json(
                args.output,
                import_package_source_rebuild_attestation(
                    args.repository,
                    args.package_directory,
                    args.upgrade,
                    args.rebuilt_directory,
                    args.attestation,
                ),
            )
        elif args.command == 'validate-lifecycle':
            lifecycle = _mapping(load_json(args.evidence), 'package lifecycle')
            validate_lifecycle_evidence(
                lifecycle,
                package_artifact_descriptor(args.upgrade),
                package_artifact_descriptor(args.baseline),
            )
        elif args.command == 'capture-installed-state':
            lifecycle = _mapping(load_json(args.evidence), 'package lifecycle')
            upgrade = package_artifact_descriptor(args.upgrade)
            validate_lifecycle_evidence(
                lifecycle,
                upgrade,
                package_artifact_descriptor(args.baseline),
            )
            atomic_write_json(
                args.output,
                capture_installed_package_state(
                    lifecycle,
                    upgrade,
                    args.smoke_script,
                ),
            )
        elif args.command == 'source-snapshot':
            atomic_write_json(args.output, source_snapshot(args.repository.resolve()))
        elif args.command == 'record':
            value = append_timeline(args.timeline, args.kind, _parse_details(args.detail))
            print(value['monotonic_ns'])
        elif args.command == 'probe-http':
            status, body = probe_http(args.url, args.timeout)
            append_timeline(
                args.timeline,
                args.kind,
                {
                    'url': args.url,
                    'http_status': status,
                    'body': body,
                    'body_sha256': hashlib.sha256(body.encode()).hexdigest(),
                },
            )
            print(status)
        elif args.command == 'fetch-json':
            fetch_http_json(args.url, args.output, args.expected_status)
        elif args.command == 'find-controller':
            value = find_exact_controller(
                args.root_pid,
                args.pgid,
                args.ros_domain_id,
                args.gz_partition,
                unit=args.unit,
            )
            atomic_write_json(args.output, value)
            print(value['pid'])
        elif args.command == 'assert-identity':
            assert_process_identity(_mapping(load_json(args.identity), 'identity'))
        elif args.command == 'signal-identity':
            signum = signal.SIGTERM if args.signal == 'TERM' else signal.SIGKILL
            print(signal_process_identity(_mapping(load_json(args.identity), 'identity'), signum))
        elif args.command == 'process-group':
            members = process_group_members(args.pgid)
            value = {
                'schema_version': 1,
                'captured_utc': utc_now(),
                'pgid': args.pgid,
                'member_count': len(members),
                'members': members,
            }
            if args.output is not None:
                atomic_write_json(args.output, value)
            else:
                print(json.dumps(value, sort_keys=True))
            if args.require_empty and members:
                return 1
        elif args.command == 'systemd-owners':
            units = systemd_owned_units(args.ros_domain_id, args.gz_partition)
            value = {
                'schema_version': 1,
                'captured_utc': utc_now(),
                'ros_domain_id': args.ros_domain_id,
                'gz_partition': args.gz_partition,
                'units': units,
                'unit_count': len(units),
                'verdict': (
                    'PASS'
                    if set(units) == {args.expected_unit} and bool(units[args.expected_unit])
                    else 'FAIL'
                ),
            }
            atomic_write_json(args.output, value)
            print(len(units))
            if value['verdict'] != 'PASS':
                return 1
        elif args.command == 'capture-runtime-affinity':
            value = capture_runtime_affinity(
                args.main_pid,
                args.managed_child_pid,
                args.managed_child_pgid,
                args.ros_domain_id,
                args.gz_partition,
                args.phase,
                unit=args.unit,
                expected_cpuset=args.expected_cpuset,
            )
            atomic_write_json(args.output, value)
            print(value['process_count'])
        elif args.command == 'validate-runtime-ownership':
            validate_stable_runtime_ownership_capture(
                _mapping(load_json(args.before), 'owners before'),
                _mapping(load_json(args.after), 'owners after'),
                _mapping(load_json(args.affinity), 'runtime affinity'),
            )
        elif args.command == 'compose':
            result = write_result(args.run_directory.resolve())
            if result['verdict']['status'] != 'PASS':
                return 1
        elif args.command == 'checksums':
            write_checksums(args.run_directory.resolve())
        elif args.command == 'publish-evidence':
            result = publish_run_evidence(
                args.source,
                args.destination,
                args.repository,
            )
            print(json.dumps(result, sort_keys=True))
        else:
            raise EvidenceError(f'unsupported command: {args.command}')
    except (EvidenceError, OSError, ValueError) as exc:
        print(f'phase4_acceptance: {exc}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
