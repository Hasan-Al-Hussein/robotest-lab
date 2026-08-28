#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""
Bounded passive host profiler for one prepared Phase 3 smoke.

Start this command before the benchmark runner's ``smoke`` command. It waits
for the one exact-isolation process mapping the prepared contact-aggregator
DSO. It never starts, signals, or otherwise controls ROS or Gazebo processes.
``ROBOTEST_CONTACT_PROFILE=1`` must be exported for this profiler and the
runner. Output is frozen below ``artifacts/evidence/phase3/performance-profiles``.
"""

from __future__ import annotations

import argparse
import ast
from collections.abc import Callable, Mapping, Sequence
import copy
import csv
from dataclasses import dataclass, field
import datetime
import errno
import hashlib
import io
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile
import time
from typing import Any

from jsonschema import Draft202012Validator
from jsonschema.exceptions import SchemaError

PRODUCER = 'robotest_phase3/smoke_host_profiler'
BENCHMARK_PRODUCER = 'robotest_phase3/benchmark_orchestrator'
SCHEMA_VERSION = 1
CANONICAL_STARTUP_TIMEOUT_S = 180.0
CANONICAL_SAMPLE_PERIOD_S = 0.5
CANONICAL_MAX_DURATION_S = 480.0
FULL_STACK_CLOSE_TIMEOUT_S = 15.0
CONTACT_PROFILE_PREFIX = b'ROBOTEST_CONTACT_PROFILE '
CSV_PROJECTION_CONTRACT = 'bounded_scalar_summary_v1'
CSV_PROJECTION_CONTRACT_COLUMN = '__robotest_projection_contract'
CSV_PROJECTION_JSON_SHA256_COLUMN = '__robotest_canonical_json_sha256'
PROCFS_DISAPPEARANCE_ERRNOS = frozenset({errno.ENOENT, errno.ESRCH})
CONTACT_PROFILE_KEYS = frozenset(
    {
        'cached_event_state_check_ns',
        'clock_id',
        'contact_policy_protobuf_ns',
        'exhaustive_event_rescan_ns',
        'locked_binding_validation_ns',
        'measured_total_ns',
        'observation_count',
        'profile_epoch_start_sim_stamp_ns',
        'publish_count',
        'publish_ns',
        'rescan_count',
        'saturated',
        'schema_version',
        'sim_stamp_ns',
        'status',
        'thread_contributions',
    }
)
CONTACT_PROFILE_FAILURE_KEYS = frozenset(
    {'failure_kind', 'schema_version', 'sim_stamp_ns', 'status'}
)
CONTACT_PROFILE_FAILURE_KINDS = frozenset(
    {
        'clock_regressed',
        'clock_unavailable',
        'counter_overflow',
        'invalid_category',
        'invalid_thread_identity',
        'output_unavailable',
        'thread_contribution_overflow',
    }
)
CONTACT_THREAD_CONTRIBUTION_KEYS = frozenset(
    {
        'cached_event_state_check_ns',
        'contact_policy_protobuf_ns',
        'exhaustive_event_rescan_ns',
        'linux_tid',
        'locked_binding_validation_ns',
        'measured_total_ns',
        'observation_count',
        'publish_count',
        'publish_ns',
        'rescan_count',
    }
)
CONTACT_BUCKET_KEYS = (
    'cached_event_state_check_ns',
    'contact_policy_protobuf_ns',
    'exhaustive_event_rescan_ns',
    'locked_binding_validation_ns',
    'publish_ns',
)
CONTACT_MONOTONIC_KEYS = (
    *CONTACT_BUCKET_KEYS,
    'measured_total_ns',
    'observation_count',
    'publish_count',
    'rescan_count',
    'sim_stamp_ns',
)
CONTACT_CONTRIBUTION_MONOTONIC_KEYS = (
    *CONTACT_BUCKET_KEYS,
    'measured_total_ns',
    'observation_count',
    'publish_count',
    'rescan_count',
)
UINT64_MAX = 2**64 - 1
INT64_MAX = 2**63 - 1

JSON_MAX_BYTES = 16 * 1024 * 1024
LOG_MAX_BYTES = 8 * 1024 * 1024
MAPS_MAX_BYTES = 16 * 1024 * 1024
ENVIRON_MAX_BYTES = 1024 * 1024
CMDLINE_MAX_BYTES = 1024 * 1024
RENDERER_MAX_BYTES = 16 * 1024 * 1024
BINARY_MAX_BYTES = 256 * 1024 * 1024
OUTPUT_MAX_BYTES = 256 * 1024 * 1024
MAX_PROC_ENTRIES = 32768
MAX_PROCESSES = 1024
MAX_THREADS_PER_PROCESS = 4096
MAX_THREAD_IDENTITIES = 16384
MAX_LIVE_THREADS_PER_SAMPLE = 512
MAX_THREAD_RECORDS = 524288
MAX_SAMPLES = 2048
MAX_LINE_BYTES = 65536
MAX_RENDERER_LINES = 256
MAX_CONTACT_RECORDS = 128
MAX_CONTACT_THREAD_CONTRIBUTIONS = 128
MAX_CMDLINE_TOTAL_BYTES = 4 * 1024 * 1024
MAX_ANCESTRY_DEPTH = 256
MAX_FOREIGN_DIAGNOSTIC_IDENTITIES = 8
MINIMUM_SAMPLES = 4

CANDIDATE_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,127}')
PARTITION_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,255}')
GIT_SHA_RE = re.compile(r'[0-9a-f]{40}')
SHA256_RE = re.compile(r'[0-9a-f]{64}')
RENDERER_RE = re.compile(
    r'(?:GL_RENDERER|Device Name|OpenGL|RenderSystem Name|Vulkan)', re.IGNORECASE
)
VMSTAT_KEYS = ('oom_kill', 'pgfault', 'pgmajfault', 'pgpgin', 'pgpgout', 'pswpin', 'pswpout')
PROC_STAT_KEYS = ('ctxt', 'processes', 'procs_blocked', 'procs_running')
TERMINAL_PROCESS_STATES = frozenset({'X', 'Z'})

PROFILE_TOP_LEVEL_KEYS = frozenset(
    {
        'anchor',
        'candidate',
        'contact_profile',
        'finished_utc',
        'full_stack_process',
        'host_clock_ticks_per_second',
        'limits',
        'process_lifecycles',
        'producer',
        'profile_started_boot_ticks',
        'profile_started_monotonic_ns',
        'profile_started_utc',
        'renderer',
        'sampling',
        'schema_version',
        'status',
    }
)
FILE_IDENTITY_KEYS = frozenset({'device', 'inode', 'mtime_ns', 'path', 'sha256', 'size_bytes'})
PROFILE_CANDIDATE_KEYS = frozenset(
    {
        'build_binding',
        'candidate_id',
        'candidate_root',
        'git',
        'plugin',
        'prepared_marker',
        'profiler_producer',
        'smoke',
        'suite_plan',
    }
)
PROFILE_ANCHOR_KEYS = frozenset(
    {
        'cmdline',
        'cmdline_sha256',
        'cmdline_size_bytes',
        'comm',
        'executable',
        'executable_link',
        'expected_plugin',
        'mapping_count',
        'mapping_fingerprint_sha256',
        'mappings',
        'pid',
        'ppid',
        'process_group',
        'session',
        'start_ticks',
    }
)
PROFILE_LIMIT_KEYS = frozenset(
    {
        'maximum_duration_s',
        'maximum_output_bytes',
        'maximum_process_identities',
        'maximum_retained_cmdline_bytes',
        'maximum_samples',
        'maximum_live_threads_per_sample',
        'maximum_thread_identities',
        'maximum_thread_records',
        'sample_period_s',
        'startup_timeout_s',
    }
)
PROFILE_SAMPLING_KEYS = frozenset(
    {
        'anchor_alive_sample_count',
        'cadence_overrun_count',
        'exact_processes_seen_before_anchor',
        'retained_cmdline_bytes',
        'sample_count',
        'samples',
        'target_alive_sample_count',
        'thread_record_count',
        'totals',
    }
)


class ProfileError(RuntimeError):
    """One fail-closed profiler or evidence error."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


class ProcessDisappearedError(ProfileError):
    """A structured procfs disappearance preserving the affected PID."""

    def __init__(self, pid: int, label: str) -> None:
        super().__init__('missing_identity', f'{label} disappeared')
        self.pid = pid


class ProcessParentChangedError(ProfileError):
    """A structured PPID-only transition preserving the affected PID."""

    def __init__(self, pid: int, label: str) -> None:
        super().__init__('identity_changed', f'{label} ancestry changed')
        self.pid = pid


@dataclass(frozen=True)
class ProcStat:
    """Fields consumed from one Linux ``/proc/*/stat`` record."""

    pid: int
    comm: str
    state: str
    ppid: int
    process_group: int
    session: int
    utime_ticks: int
    stime_ticks: int
    start_ticks: int
    processor: int | None

    @property
    def cpu_ticks(self) -> int:
        return self.utime_ticks + self.stime_ticks


ThreadSampleUpdate = tuple[
    tuple[int, int, int, int],
    tuple[int, int, int],
    ProcStat,
    int | None,
]


@dataclass(frozen=True)
class ProfileConfig:
    """Frozen inputs and paths for one smoke profile."""

    workspace: Path
    candidate_root: Path
    candidate_id: str
    build_binding: Path
    ros_domain_id: int
    gz_partition: str
    startup_timeout_s: float
    sample_period_s: float
    max_duration_s: float
    renderer_log: Path
    output_path: Path


@dataclass
class LatchedProcess:
    """Stable identity and observed lifecycle of one matching process."""

    pid: int
    start_ticks: int
    process_group: int
    session: int
    comm: str
    cmdline: list[str]
    cmdline_sha256: str
    cmdline_size_bytes: int
    executable_link: str
    first_sample: int
    ended_sample: int | None = None


@dataclass(frozen=True)
class SmokeRunnerOwner:
    """Stable benchmark-runner identity that owns one smoke process forest."""

    pid: int
    start_ticks: int
    ppid: int
    process_group: int
    session: int
    comm: str
    cmdline_sha256: str
    executable_link: str


@dataclass
class ProfileState:
    """Bounded mutable state retained by the profiler."""

    processes: dict[int, LatchedProcess] = field(default_factory=dict)
    samples: list[dict[str, Any]] = field(default_factory=list)
    last_thread_ticks: dict[tuple[int, int, int, int], int] = field(default_factory=dict)
    thread_start_by_tid: dict[tuple[int, int, int], int] = field(default_factory=dict)
    thread_names: dict[tuple[int, int, int, int], str] = field(default_factory=dict)
    thread_delta_ticks: dict[tuple[int, int, int, int], int] = field(default_factory=dict)
    process_delta_ticks: dict[tuple[int, int], int] = field(default_factory=dict)
    cmdline_bytes: int = 0
    thread_records: int = 0
    cadence_overruns: int = 0
    smoke_runner_owner: SmokeRunnerOwner | None = None


def canonical_json_bytes(document: Any) -> bytes:
    """Return strict canonical JSON with the repository newline convention."""
    try:
        text = json.dumps(
            document,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise ProfileError('noncanonical_output', f'cannot serialize JSON: {exc}') from exc
    return (text + '\n').encode()


def _procfs_entry_disappeared(error: OSError) -> bool:
    return error.errno in PROCFS_DISAPPEARANCE_ERRNOS


def _bounded_proc_read(path: Path, maximum: int, label: str) -> bytes:
    """Read one procfs pseudo-file under an explicit byte cap."""
    try:
        with path.open('rb') as source:
            payload = source.read(maximum + 1)
    except OSError as exc:
        if _procfs_entry_disappeared(exc):
            raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), path) from exc
        raise ProfileError('incomplete_profile', f'cannot read {label}: {path}: {exc}') from exc
    if len(payload) > maximum:
        raise ProfileError('overflow', f'{label} exceeds {maximum} bytes: {path}')
    return payload


def _bounded_regular_read(path: Path, maximum: int, label: str) -> bytes:
    """Read one regular non-symlink file under a size cap."""
    try:
        before_path = path.lstat()
    except OSError as exc:
        raise ProfileError('missing_identity', f'{label} is unavailable: {path}: {exc}') from exc
    if stat.S_ISLNK(before_path.st_mode) or not stat.S_ISREG(before_path.st_mode):
        raise ProfileError('missing_identity', f'{label} is not a regular file: {path}')
    if before_path.st_size > maximum:
        raise ProfileError('overflow', f'{label} exceeds {maximum} bytes: {path}')
    try:
        with path.open('rb') as source:
            before_handle = os.fstat(source.fileno())
            if (
                before_handle.st_dev != before_path.st_dev
                or before_handle.st_ino != before_path.st_ino
            ):
                raise ProfileError('identity_changed', f'{label} changed before reading: {path}')
            payload = source.read(maximum + 1)
            after_handle = os.fstat(source.fileno())
        after_path = path.lstat()
    except OSError as exc:
        raise ProfileError('incomplete_profile', f'cannot read {label}: {path}: {exc}') from exc
    if len(payload) > maximum:
        raise ProfileError('overflow', f'{label} grew beyond {maximum} bytes: {path}')
    stable = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns')
    if (
        len(payload) != after_handle.st_size
        or any(getattr(before_handle, key) != getattr(after_handle, key) for key in stable)
        or any(getattr(after_handle, key) != getattr(after_path, key) for key in stable)
    ):
        raise ProfileError('identity_changed', f'{label} changed while reading: {path}')
    return payload


def _file_identity(path: Path, maximum: int, label: str) -> dict[str, Any]:
    """Hash a stable regular non-symlink file and bind its stat identity."""
    try:
        before = path.lstat()
    except OSError as exc:
        raise ProfileError('missing_identity', f'{label} is unavailable: {path}: {exc}') from exc
    if path.is_symlink() or not path.is_file():
        raise ProfileError('missing_identity', f'{label} is not a regular file: {path}')
    if before.st_size > maximum:
        raise ProfileError('overflow', f'{label} exceeds {maximum} bytes: {path}')
    digest = hashlib.sha256()
    observed = 0
    try:
        with path.open('rb') as source:
            while block := source.read(1024 * 1024):
                observed += len(block)
                if observed > maximum:
                    raise ProfileError('overflow', f'{label} exceeds {maximum} bytes: {path}')
                digest.update(block)
        after = path.lstat()
    except OSError as exc:
        raise ProfileError('incomplete_profile', f'cannot hash {label}: {path}: {exc}') from exc
    stable = ('st_dev', 'st_ino', 'st_size', 'st_mtime_ns')
    identity_changed = any(getattr(before, key) != getattr(after, key) for key in stable)
    if observed != after.st_size or identity_changed:
        raise ProfileError('identity_changed', f'{label} changed while hashing: {path}')
    return {
        'device': after.st_dev,
        'inode': after.st_ino,
        'mtime_ns': after.st_mtime_ns,
        'path': str(path),
        'sha256': digest.hexdigest(),
        'size_bytes': after.st_size,
    }


def _load_canonical(
    path: Path,
    label: str,
    maximum_bytes: int = JSON_MAX_BYTES,
) -> tuple[Mapping[str, Any], str]:
    payload = _bounded_regular_read(path, maximum_bytes, label)
    try:
        document = json.loads(payload.decode())
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ProfileError('missing_identity', f'{label} is not valid UTF-8 JSON: {path}') from exc
    if not isinstance(document, Mapping) or payload != canonical_json_bytes(document):
        raise ProfileError('missing_identity', f'{label} is not canonical JSON: {path}')
    digest = hashlib.sha256(payload).hexdigest()
    sidecar = Path(f'{path}.sha256')
    expected = f'{digest}  {path.name}\n'.encode('ascii')
    if _bounded_regular_read(sidecar, 256, f'{label} sidecar') != expected:
        raise ProfileError('missing_identity', f'{label} sidecar mismatch: {sidecar}')
    return document, digest


def _load_canonical_without_sidecar(
    path: Path,
    label: str,
    maximum_bytes: int = JSON_MAX_BYTES,
) -> tuple[Mapping[str, Any], str]:
    payload = _bounded_regular_read(path, maximum_bytes, label)
    try:
        document = json.loads(payload.decode())
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ProfileError('missing_identity', f'{label} is not valid UTF-8 JSON: {path}') from exc
    if not isinstance(document, Mapping) or payload != canonical_json_bytes(document):
        raise ProfileError('missing_identity', f'{label} is not canonical JSON: {path}')
    return document, hashlib.sha256(payload).hexdigest()


def _atomic_create(path: Path, payload: bytes, maximum: int) -> None:
    if len(payload) > maximum:
        raise ProfileError('overflow', f'output exceeds {maximum} bytes: {path}')
    if path.exists() or path.is_symlink():
        raise ProfileError('output_exists', f'refusing to replace evidence: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, 'wb') as target:
            os.fchmod(target.fileno(), 0o644)
            target.write(payload)
            target.flush()
            os.fsync(target.fileno())
        if path.exists() or path.is_symlink():
            raise ProfileError('output_exists', f'refusing to replace evidence: {path}')
        os.link(temporary, path)
        temporary.unlink()
        parent = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(parent)
        finally:
            os.close(parent)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise


def write_profile(path: Path, document: Mapping[str, Any]) -> str:
    """Write canonical JSON and one GNU-style checksum sidecar without replacement."""
    payload = canonical_json_bytes(document)
    digest = hashlib.sha256(payload).hexdigest()
    sidecar = Path(f'{path}.sha256')
    if sidecar.exists() or sidecar.is_symlink():
        raise ProfileError('output_exists', f'refusing to replace evidence: {sidecar}')
    _atomic_create(path, payload, OUTPUT_MAX_BYTES)
    try:
        _atomic_create(sidecar, f'{digest}  {path.name}\n'.encode('ascii'), 256)
    except BaseException:
        path.unlink(missing_ok=True)
        raise
    return digest


def parse_proc_stat(text: str) -> ProcStat:
    """Parse proc stat by its final comm delimiter, including odd comm strings."""
    opening = text.find('(')
    closing = text.rfind(')')
    if opening <= 0 or closing <= opening:
        raise ProfileError('incomplete_profile', 'malformed proc stat')
    try:
        pid = int(text[:opening].strip())
        fields = text[closing + 1 :].split()  # noqa: E203, RUF100
        if len(fields) < 20:
            raise ValueError('too few fields')
        result = ProcStat(
            pid=pid,
            comm=text[opening + 1 : closing],  # noqa: E203, RUF100
            state=fields[0],
            ppid=int(fields[1]),
            process_group=int(fields[2]),
            session=int(fields[3]),
            utime_ticks=int(fields[11]),
            stime_ticks=int(fields[12]),
            start_ticks=int(fields[19]),
            processor=int(fields[36]) if len(fields) > 36 else None,
        )
    except (IndexError, ValueError) as exc:
        raise ProfileError('incomplete_profile', 'malformed proc stat fields') from exc
    if result.pid <= 0 or result.start_ticks < 0 or result.cpu_ticks < 0:
        raise ProfileError('incomplete_profile', 'invalid proc stat numeric field')
    return result


def _read_stat(path: Path) -> ProcStat:
    payload = _bounded_proc_read(path, 64 * 1024, 'proc stat')
    try:
        return parse_proc_stat(payload.decode('ascii'))
    except UnicodeError as exc:
        raise ProfileError('incomplete_profile', f'non-ASCII proc stat: {path}') from exc


def _same_process(left: ProcStat, right: ProcStat) -> bool:
    return left.pid == right.pid and left.start_ticks == right.start_ticks


def parse_environ(payload: bytes) -> dict[bytes, list[bytes]]:
    """Preserve duplicates so exact environment matching can reject ambiguity."""
    result: dict[bytes, list[bytes]] = {}
    for item in payload.split(b'\0'):
        if not item:
            continue
        if b'=' not in item:
            raise ProfileError('incomplete_profile', 'malformed proc environ')
        key, value = item.split(b'=', 1)
        result.setdefault(key, []).append(value)
    return result


def has_exact_environment(payload: bytes, domain_id: int, partition: str) -> bool:
    environment = parse_environ(payload)
    domain_matches = environment.get(b'ROS_DOMAIN_ID') == [str(domain_id).encode('ascii')]
    partition_matches = environment.get(b'GZ_PARTITION') == [partition.encode()]
    return domain_matches and partition_matches


def parse_maps(text: str, expected_device: int, expected_inode: int) -> list[dict[str, Any]]:
    """Select only mappings whose exact device/inode identify the prepared DSO."""
    expected = (os.major(expected_device), os.minor(expected_device), expected_inode)
    matches: list[dict[str, Any]] = []
    for line in text.splitlines():
        fields = line.split(maxsplit=5)
        if len(fields) < 5:
            raise ProfileError('incomplete_profile', 'malformed proc maps line')
        try:
            major, minor = (int(item, 16) for item in fields[3].split(':', 1))
            identity = (major, minor, int(fields[4]))
            offset = int(fields[2], 16)
        except (TypeError, ValueError) as exc:
            raise ProfileError('incomplete_profile', 'malformed proc maps identity') from exc
        if identity != expected:
            continue
        mapped_path = fields[5] if len(fields) == 6 else ''
        if mapped_path.endswith(' (deleted)'):
            raise ProfileError('missing_identity', 'contact DSO mapping is deleted')
        matches.append(
            {
                'address_range': fields[0],
                'offset': offset,
                'path': mapped_path,
                'permissions': fields[1],
            }
        )
    return matches


def parse_loadavg(text: str) -> dict[str, Any]:
    fields = text.split()
    if len(fields) != 5 or '/' not in fields[3]:
        raise ProfileError('incomplete_profile', 'malformed loadavg')
    runnable, entities = fields[3].split('/', 1)
    try:
        result = {
            'load_1m': float(fields[0]),
            'load_5m': float(fields[1]),
            'load_15m': float(fields[2]),
            'runnable_entities': int(runnable),
            'total_entities': int(entities),
            'last_pid': int(fields[4]),
        }
    except ValueError as exc:
        raise ProfileError('incomplete_profile', 'malformed loadavg value') from exc
    if not all(math.isfinite(result[key]) for key in ('load_1m', 'load_5m', 'load_15m')):
        raise ProfileError('incomplete_profile', 'non-finite loadavg value')
    if (
        any(result[key] < 0 for key in ('load_1m', 'load_5m', 'load_15m'))
        or result['runnable_entities'] < 0
        or result['total_entities'] <= 0
        or result['runnable_entities'] > result['total_entities']
        or result['last_pid'] <= 0
    ):
        raise ProfileError('incomplete_profile', 'invalid loadavg range')
    return result


def _parse_named_ints(text: str, required: Sequence[str], label: str) -> dict[str, int]:
    values: dict[str, int] = {}
    for line in text.splitlines():
        fields = line.split()
        if len(fields) == 2 and fields[0] in required:
            try:
                values[fields[0]] = int(fields[1])
            except ValueError as exc:
                raise ProfileError('incomplete_profile', f'malformed {label}') from exc
            if values[fields[0]] < 0:
                raise ProfileError('incomplete_profile', f'negative {label} value')
    missing = sorted(set(required) - values.keys())
    if missing:
        raise ProfileError('incomplete_profile', f'{label} lacks {", ".join(missing)}')
    return {key: values[key] for key in required}


def parse_psi(text: str, resource: str) -> dict[str, dict[str, float | int]]:
    """Parse PSI rows, allowing CPU's normal some-only file."""
    result: dict[str, dict[str, float | int]] = {}
    for line in text.splitlines():
        fields = line.split()
        if not fields or fields[0] not in {'some', 'full'} or fields[0] in result:
            raise ProfileError('incomplete_profile', f'malformed {resource} PSI')
        values: dict[str, float | int] = {}
        for item in fields[1:]:
            if '=' not in item:
                raise ProfileError('incomplete_profile', f'malformed {resource} PSI')
            key, value = item.split('=', 1)
            try:
                values[key] = int(value) if key == 'total' else float(value)
            except ValueError as exc:
                raise ProfileError('incomplete_profile', f'malformed {resource} PSI') from exc
        if set(values) != {'avg10', 'avg60', 'avg300', 'total'} or not all(
            math.isfinite(float(value)) and float(value) >= 0 for value in values.values()
        ):
            raise ProfileError('incomplete_profile', f'incomplete {resource} PSI')
        result[fields[0]] = values
    if 'some' not in result or (resource != 'cpu' and 'full' not in result):
        raise ProfileError('incomplete_profile', f'incomplete {resource} PSI')
    return result


def read_host_sample(proc_root: Path) -> dict[str, Any]:
    """Read load, run queue, paging, OOM, and PSI evidence."""

    def read(relative: str, maximum: int = 1024 * 1024) -> str:
        payload = _bounded_proc_read(proc_root / relative, maximum, relative)
        try:
            return payload.decode('ascii')
        except UnicodeError as exc:
            raise ProfileError('incomplete_profile', f'non-ASCII {relative}') from exc

    return {
        'loadavg': parse_loadavg(read('loadavg', 4096)),
        'pressure': {
            item: parse_psi(read(f'pressure/{item}', 64 * 1024), item)
            for item in ('cpu', 'io', 'memory')
        },
        'proc_stat': _parse_named_ints(read('stat'), PROC_STAT_KEYS, 'proc stat'),
        'vmstat': _parse_named_ints(read('vmstat'), VMSTAT_KEYS, 'vmstat'),
    }


def _clock_ticks_per_second() -> int:
    ticks = int(os.sysconf('SC_CLK_TCK'))
    if ticks <= 0:
        raise ProfileError('incomplete_profile', 'SC_CLK_TCK is invalid')
    return ticks


def read_boot_ticks(proc_root: Path, clock_ticks_per_second: int) -> int:
    """Return a conservative boot-relative tick latch from ``/proc/uptime``."""
    payload = _bounded_proc_read(proc_root / 'uptime', 4096, 'proc uptime')
    try:
        fields = payload.decode('ascii').split()
        if len(fields) != 2:
            raise ValueError('wrong field count')
        uptime_seconds = float(fields[0])
        idle_seconds = float(fields[1])
    except (UnicodeError, ValueError) as exc:
        raise ProfileError('incomplete_profile', 'malformed proc uptime') from exc
    if (
        not math.isfinite(uptime_seconds)
        or not math.isfinite(idle_seconds)
        or uptime_seconds < 0
        or idle_seconds < 0
    ):
        raise ProfileError('incomplete_profile', 'invalid proc uptime')
    return math.floor(uptime_seconds * clock_ticks_per_second)


def _git_state(workspace: Path) -> dict[str, str]:
    def run(arguments: Sequence[str]) -> str:
        try:
            result = subprocess.run(
                ['git', '-C', str(workspace), *arguments],
                check=True,
                capture_output=True,
                text=True,
                timeout=10,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise ProfileError('missing_identity', f'cannot read Git identity: {exc}') from exc
        if len(result.stdout.encode()) > 4 * 1024 * 1024:
            raise ProfileError('overflow', 'Git identity exceeds 4 MiB')
        return result.stdout

    return {
        'sha': run(('rev-parse', 'HEAD')).strip(),
        'status_porcelain': run(('status', '--porcelain=v1', '--untracked-files=all')),
    }


def _validate_output_parent(workspace: Path, parent: Path) -> None:
    """Reject output-parent aliases while permitting not-yet-created directories."""
    try:
        relative = parent.relative_to(workspace)
    except ValueError as exc:
        raise ProfileError('invalid_argument', 'profile output parent escapes workspace') from exc
    current = workspace
    for component in relative.parts:
        current /= component
        try:
            status = current.lstat()
        except FileNotFoundError:
            continue
        except OSError as exc:
            message = f'cannot inspect profile output parent: {current}: {exc}'
            raise ProfileError('invalid_argument', message) from exc
        if stat.S_ISLNK(status.st_mode):
            message = f'profile output parent is a symlink: {current}'
            raise ProfileError('invalid_argument', message)
        if not stat.S_ISDIR(status.st_mode):
            raise ProfileError(
                'invalid_argument', f'profile output parent is not a directory: {current}'
            )
    if parent.resolve(strict=False) != parent:
        raise ProfileError('invalid_argument', 'profile output parent is not canonical')


def _require_real_directory_tree(base: Path, relative: Path, label: str) -> Path:
    """Require every existing component below ``base`` to be a real directory."""
    current = base
    for component in relative.parts:
        current /= component
        try:
            status = current.lstat()
        except OSError as exc:
            raise ProfileError('missing_identity', f'{label} is unavailable: {current}') from exc
        if stat.S_ISLNK(status.st_mode) or not stat.S_ISDIR(status.st_mode):
            raise ProfileError('missing_identity', f'{label} is not a real directory: {current}')
    return current


def _reject_existing_output_leaf(path: Path) -> None:
    try:
        path.lstat()
    except FileNotFoundError:
        return
    except OSError as exc:
        raise ProfileError(
            'output_exists', f'cannot inspect profile output: {path}: {exc}'
        ) from exc
    raise ProfileError('output_exists', f'refusing existing profile output: {path}')


def validated_config(arguments: argparse.Namespace) -> ProfileConfig:
    """Resolve and freeze every CLI value before any monitoring begins."""
    if CANDIDATE_RE.fullmatch(arguments.candidate_id) is None:
        raise ProfileError('invalid_argument', 'candidate ID is not path-safe')
    if PARTITION_RE.fullmatch(arguments.gz_partition) is None:
        raise ProfileError('invalid_argument', 'Gazebo partition is not path-safe')
    if not 0 <= arguments.ros_domain_id <= 232:
        raise ProfileError('invalid_argument', 'ROS domain ID must be in [0, 232]')
    for label, value, expected in (
        ('startup timeout', arguments.startup_timeout_s, CANONICAL_STARTUP_TIMEOUT_S),
        ('sample period', arguments.sample_period_s, CANONICAL_SAMPLE_PERIOD_S),
        ('maximum duration', arguments.max_duration_s, CANONICAL_MAX_DURATION_S),
    ):
        if not math.isfinite(value) or value != expected:
            raise ProfileError(
                'invalid_argument', f'{label} must be the canonical value {expected:g}'
            )
    try:
        workspace = arguments.workspace.resolve(strict=True)
        candidate_root = arguments.candidate_root.resolve(strict=True)
        build_binding = arguments.build_binding.resolve(strict=True)
    except OSError as exc:
        raise ProfileError('missing_identity', f'cannot resolve required path: {exc}') from exc
    benchmark_root = workspace / 'artifacts/evidence/phase3-benchmarks'
    declared_candidate = benchmark_root / arguments.candidate_id
    if declared_candidate.is_symlink():
        raise ProfileError('invalid_argument', 'candidate root must not be a symlink')
    expected_candidate = declared_candidate.resolve()
    if candidate_root != expected_candidate or not candidate_root.is_relative_to(workspace):
        raise ProfileError('invalid_argument', f'candidate root must be {expected_candidate}')
    declared_binding = candidate_root / 'build-binding.json'
    if declared_binding.is_symlink():
        raise ProfileError('invalid_argument', 'build binding must not be a symlink')
    expected_binding = declared_binding.resolve()
    if build_binding != expected_binding:
        raise ProfileError('invalid_argument', f'build binding must be {expected_binding}')
    output_parent = workspace / 'artifacts/evidence/phase3/performance-profiles'
    _validate_output_parent(workspace, output_parent)
    output = output_parent / f'{arguments.candidate_id}-smoke-profile.json'
    sidecar = Path(f'{output}.sha256')
    if not output.is_relative_to(workspace) or output.is_relative_to(candidate_root):
        raise ProfileError('invalid_argument', 'profile output path is unsafe')
    _reject_existing_output_leaf(output)
    _reject_existing_output_leaf(sidecar)
    return ProfileConfig(
        workspace=workspace,
        candidate_root=candidate_root,
        candidate_id=arguments.candidate_id,
        build_binding=build_binding,
        ros_domain_id=arguments.ros_domain_id,
        gz_partition=arguments.gz_partition,
        startup_timeout_s=arguments.startup_timeout_s,
        sample_period_s=arguments.sample_period_s,
        max_duration_s=arguments.max_duration_s,
        renderer_log=Path(os.path.expanduser('~/.gz/rendering/ogre2.log')).resolve(),
        output_path=output,
    )


def collect_static_identity(
    config: ProfileConfig,
    git_reader: Callable[[Path], Mapping[str, str]] = _git_state,
    environment: Mapping[str, str] = os.environ,
    producer_path: Path | None = None,
) -> dict[str, Any]:
    """Bind prepared candidate, clean Git, plan, build binding, and DSO bytes."""
    binding, binding_hash = _load_canonical(config.build_binding, 'candidate build binding')
    plan_path = config.candidate_root / 'suite-plan.json'
    plan, plan_hash = _load_canonical(plan_path, 'candidate suite plan')
    prepared_path = config.candidate_root / 'prepared.json'
    prepared, prepared_hash = _load_canonical(prepared_path, 'candidate prepared marker')
    if plan.get('candidate_id') != config.candidate_id:
        raise ProfileError('missing_identity', 'suite plan candidate differs from CLI')
    smoke = plan.get('smoke')
    if not isinstance(smoke, Mapping):
        raise ProfileError('missing_identity', 'suite plan lacks smoke identity')
    if (
        smoke.get('ros_domain_id') != config.ros_domain_id
        or smoke.get('gz_partition') != config.gz_partition
    ):
        raise ProfileError('missing_identity', 'CLI isolation differs from prepared smoke')
    if smoke.get('run_id') != f'{config.candidate_id}-smoke-s1-r0':
        raise ProfileError('missing_identity', 'suite plan smoke run ID is not frozen')
    if environment.get('ROBOTEST_CONTACT_PROFILE') != '1':
        raise ProfileError('missing_identity', 'ROBOTEST_CONTACT_PROFILE must be exactly 1')
    git_binding = binding.get('git')
    if not isinstance(git_binding, Mapping):
        raise ProfileError('missing_identity', 'build binding lacks Git identity')
    git_sha = git_binding.get('sha')
    if (
        git_binding.get('dirty') is not False
        or git_binding.get('status_porcelain') != ''
        or not isinstance(git_sha, str)
        or GIT_SHA_RE.fullmatch(git_sha) is None
    ):
        raise ProfileError('dirty_git', 'build binding is not clean Git evidence')
    current_git = dict(git_reader(config.workspace))
    if current_git != {'sha': git_sha, 'status_porcelain': ''}:
        raise ProfileError('dirty_git', 'current Git state differs from clean binding')
    if (
        prepared.get('candidate_id') != config.candidate_id
        or prepared.get('git_sha') != git_sha
        or prepared.get('build_binding_sha256') != binding_hash
        or prepared.get('suite_plan_sha256') != plan_hash
    ):
        raise ProfileError('missing_identity', 'prepared marker does not bind candidate inputs')
    binary = binding.get('contact_aggregator_binary')
    if not isinstance(binary, Mapping):
        raise ProfileError('missing_identity', 'binding lacks contact aggregator identity')
    verified_fields = (
        'build_embedded_source_inventory_match',
        'build_install_build_id_match',
        'build_install_embedded_source_inventory_match',
        'build_install_sha256_match',
        'installed_embedded_source_inventory_match',
        'installed_regular_file',
    )
    if any(binary.get(name) is not True for name in verified_fields):
        raise ProfileError('missing_identity', 'contact aggregator binding is not verified')
    installed_path = binary.get('installed_path')
    installed_hash = binary.get('installed_sha256')
    if (
        not isinstance(installed_path, str)
        or Path(installed_path).is_absolute()
        or not isinstance(installed_hash, str)
        or SHA256_RE.fullmatch(installed_hash) is None
    ):
        raise ProfileError('missing_identity', 'installed DSO identity is malformed')
    try:
        plugin_path = (config.workspace / installed_path).resolve(strict=True)
        plugin_path.relative_to(config.workspace)
    except (OSError, ValueError) as exc:
        raise ProfileError('missing_identity', 'installed DSO escapes workspace') from exc
    plugin = _file_identity(plugin_path, BINARY_MAX_BYTES, 'contact aggregator DSO')
    if plugin['sha256'] != installed_hash:
        raise ProfileError('missing_identity', 'contact DSO hash differs from build binding')
    for field_name in (
        'installed_elf_build_id',
        'installed_declared_path',
        'source_inventory_sha256',
    ):
        if not isinstance(binary.get(field_name), str) or not binary[field_name]:
            raise ProfileError('missing_identity', f'contact binding lacks {field_name}')
    if SHA256_RE.fullmatch(binary['source_inventory_sha256']) is None:
        raise ProfileError('missing_identity', 'contact source inventory hash is malformed')
    plugin.update(
        {
            'build_id': binary.get('installed_elf_build_id'),
            'declared_path': binary.get('installed_declared_path'),
            'source_inventory_sha256': binary.get('source_inventory_sha256'),
        }
    )
    expected_producer = (config.workspace / 'tests/phase3_smoke_host_profiler.py').resolve()
    resolved_producer = (producer_path or Path(__file__)).resolve(strict=True)
    if producer_path is None and resolved_producer != expected_producer:
        raise ProfileError('missing_identity', f'profiler producer must be {expected_producer}')
    producer = _file_identity(resolved_producer, 2 * 1024 * 1024, 'profiler producer')
    return {
        'candidate_id': config.candidate_id,
        'candidate_root': str(config.candidate_root),
        'build_binding': {'path': str(config.build_binding), 'sha256': binding_hash},
        'suite_plan': {'path': str(plan_path), 'sha256': plan_hash},
        'prepared_marker': {'path': str(prepared_path), 'sha256': prepared_hash},
        'git': current_git,
        'smoke': {
            'ros_domain_id': config.ros_domain_id,
            'gz_partition': config.gz_partition,
            'run_id': smoke.get('run_id'),
            'contact_profile_enabled': True,
        },
        'plugin': plugin,
        'profiler_producer': producer,
    }


def _iter_pids(proc_root: Path) -> list[int]:
    try:
        entries = list(proc_root.iterdir())
    except OSError as exc:
        raise ProfileError('incomplete_profile', f'cannot enumerate procfs: {exc}') from exc
    if len(entries) > MAX_PROC_ENTRIES:
        raise ProfileError('overflow', f'procfs exceeds {MAX_PROC_ENTRIES} entries')
    return sorted(int(path.name) for path in entries if path.name.isdigit())


def discover_matching(
    proc_root: Path, domain_id: int, partition: str
) -> list[tuple[int, ProcStat]]:
    """Return stable PID/start pairs with the exact smoke isolation values."""
    matches: list[tuple[int, ProcStat]] = []
    for pid in _iter_pids(proc_root):
        root = proc_root / str(pid)
        try:
            before = _read_stat(root / 'stat')
            environment = _bounded_proc_read(root / 'environ', ENVIRON_MAX_BYTES, 'proc environ')
            after = _read_stat(root / 'stat')
        except FileNotFoundError:
            continue
        except ProfileError as exc:
            if exc.kind == 'incomplete_profile':
                continue
            raise
        if before.pid != pid or not _same_process(before, after):
            raise ProfileError('pid_reuse', f'PID {pid} changed during discovery')
        if after.state in TERMINAL_PROCESS_STATES:
            continue
        if not has_exact_environment(environment, domain_id, partition):
            continue
        matches.append((pid, after))
        if len(matches) > MAX_PROCESSES:
            raise ProfileError('overflow', 'matching process set exceeds 1,024')
    return matches


def _plugin_maps(proc_root: Path, pid: int, plugin: Mapping[str, Any]) -> list[dict[str, Any]]:
    payload = _bounded_proc_read(proc_root / str(pid) / 'maps', MAPS_MAX_BYTES, 'proc maps')
    try:
        text = payload.decode()
    except UnicodeError as exc:
        raise ProfileError('incomplete_profile', f'non-UTF-8 maps for PID {pid}') from exc
    return parse_maps(text, int(plugin['device']), int(plugin['inode']))


def _command_identity(proc_root: Path, pid: int) -> dict[str, Any]:
    payload = _bounded_proc_read(proc_root / str(pid) / 'cmdline', CMDLINE_MAX_BYTES, 'cmdline')
    executable_path = proc_root / str(pid) / 'exe'
    try:
        executable_link = os.readlink(executable_path)
    except OSError as exc:
        if _procfs_entry_disappeared(exc):
            disappeared = FileNotFoundError(
                errno.ENOENT, os.strerror(errno.ENOENT), executable_path
            )
            raise ProfileError('missing_identity', f'cannot read PID {pid} executable') from (
                disappeared
            )
        raise ProfileError('missing_identity', f'cannot read PID {pid} executable') from exc
    return {
        'cmdline': [item.decode(errors='replace') for item in payload.split(b'\0') if item],
        'cmdline_sha256': hashlib.sha256(payload).hexdigest(),
        'cmdline_size_bytes': len(payload),
        'executable_link': executable_link,
    }


def _stat_context(process: ProcStat) -> tuple[int, int, int, int, int]:
    return (
        process.pid,
        process.start_ticks,
        process.ppid,
        process.process_group,
        process.session,
    )


def _only_parent_changed(before: ProcStat, after: ProcStat) -> bool:
    return (
        before.pid == after.pid
        and before.start_ticks == after.start_ticks
        and before.process_group == after.process_group
        and before.session == after.session
        and before.comm == after.comm
        and before.ppid != after.ppid
    )


def _read_stable_process_stat(proc_root: Path, pid: int, label: str) -> ProcStat:
    """Bracket one ancestry read so reparenting and PID reuse fail closed."""
    try:
        before = _read_stat(proc_root / str(pid) / 'stat')
        after = _read_stat(proc_root / str(pid) / 'stat')
    except FileNotFoundError as exc:
        raise ProcessDisappearedError(pid, label) from exc
    if not _same_process(before, after):
        raise ProfileError('pid_reuse', f'{label} PID {pid} was reused')
    if _stat_context(before) != _stat_context(after):
        if _only_parent_changed(before, after):
            raise ProcessParentChangedError(pid, label)
        raise ProfileError('identity_changed', f'{label} ancestry changed')
    return after


def _single_command_option(command: Sequence[str], name: str) -> str:
    values: list[str] = []
    for index, item in enumerate(command):
        if item == name:
            if index + 1 >= len(command):
                raise ProfileError('missing_identity', f'smoke runner {name} is incomplete')
            values.append(command[index + 1])
        elif item.startswith(f'{name}='):
            values.append(item.split('=', 1)[1])
    if len(values) != 1:
        raise ProfileError('missing_identity', f'smoke runner {name} is not unique')
    return values[0]


def _runner_command_path(workspace: Path, value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else workspace / path


def _reject_symlinked_command_path(workspace: Path, path: Path, label: str) -> None:
    try:
        relative = path.relative_to(workspace)
    except ValueError as exc:
        raise ProfileError('missing_identity', f'{label} escapes workspace') from exc
    if '..' in relative.parts:
        raise ProfileError('missing_identity', f'{label} is not canonical')
    current = workspace
    for part in relative.parts:
        current /= part
        if current.is_symlink():
            raise ProfileError('missing_identity', f'{label} contains a symlink')


def _validate_smoke_runner_command(config: ProfileConfig, command: Sequence[str]) -> None:
    expected_script = (config.workspace / 'tests/phase3_benchmark_runner.py').resolve()
    if len(command) < 2:
        raise ProfileError('missing_identity', 'smoke runner command line is incomplete')
    try:
        observed_script = Path(command[1]).resolve()
        observed_workspace = Path(_single_command_option(command, '--workspace')).resolve()
        domain_base = int(_single_command_option(command, '--domain-base'))
        output_root = _runner_command_path(
            config.workspace, _single_command_option(command, '--output-root')
        ).resolve(strict=True)
        build_binding = _runner_command_path(
            config.workspace, _single_command_option(command, '--build-binding')
        )
        resolved_binding = build_binding.resolve(strict=True)
    except (OSError, ValueError) as exc:
        raise ProfileError('missing_identity', 'smoke runner command identity is invalid') from exc
    if (
        observed_script != expected_script
        or observed_workspace != config.workspace
        or _single_command_option(command, '--mode') != 'smoke'
        or _single_command_option(command, '--candidate-id') != config.candidate_id
        or domain_base + 16 != config.ros_domain_id
        or output_root != config.candidate_root.parent
    ):
        raise ProfileError('missing_identity', 'anchor owner is not the exact smoke runner')
    evidence_root = (config.workspace / 'artifacts/evidence').resolve()
    if not resolved_binding.is_relative_to(evidence_root):
        raise ProfileError('missing_identity', 'smoke runner build binding escapes evidence root')
    _reject_symlinked_command_path(config.workspace, build_binding, 'smoke runner build binding')
    _observed_binding, observed_hash = _load_canonical(build_binding, 'smoke runner build binding')
    _expected_binding, expected_hash = _load_canonical(
        config.build_binding, 'candidate build binding'
    )
    if observed_hash != expected_hash:
        raise ProfileError('missing_identity', 'smoke runner build binding differs from candidate')


def discover_smoke_runner_owner(
    proc_root: Path, config: ProfileConfig, anchor: Mapping[str, Any]
) -> SmokeRunnerOwner:
    """Latch the runner above the anchor's independently owned session."""
    anchor_pid = anchor.get('pid')
    anchor_start = anchor.get('start_ticks')
    anchor_group = anchor.get('process_group')
    anchor_session = anchor.get('session')
    if (
        type(anchor_pid) is not int
        or type(anchor_start) is not int
        or type(anchor_group) is not int
        or type(anchor_session) is not int
        or anchor_pid <= 0
        or anchor_start < 0
        or anchor_group <= 1
        or anchor_session <= 1
    ):
        raise ProfileError('missing_identity', 'anchor process ancestry is invalid')
    current = _read_stable_process_stat(proc_root, anchor_pid, 'anchor process')
    if (
        current.start_ticks != anchor_start
        or current.process_group != anchor_group
        or current.session != anchor_session
    ):
        raise ProfileError('identity_changed', 'anchor process ancestry differs')
    seen: set[tuple[int, int]] = set()
    for _depth in range(MAX_ANCESTRY_DEPTH):
        identity = (current.pid, current.start_ticks)
        if identity in seen:
            raise ProfileError('incomplete_profile', 'anchor ancestry contains a cycle')
        seen.add(identity)
        if current.pid == anchor_session:
            break
        if current.session != anchor_session or current.ppid <= 1:
            raise ProfileError('missing_identity', 'anchor does not reach its session leader')
        child = current
        current = _read_stable_process_stat(proc_root, child.ppid, 'anchor ancestor process')
        child_after = _read_stable_process_stat(proc_root, child.pid, 'anchor process')
        if _stat_context(child_after) != _stat_context(child):
            raise ProfileError('identity_changed', 'anchor ancestry changed during discovery')
    else:
        raise ProfileError('overflow', 'anchor ancestry exceeds 256 processes')
    session_leader = current
    if (
        session_leader.pid != anchor_session
        or session_leader.process_group != anchor_session
        or session_leader.session != anchor_session
        or session_leader.ppid <= 1
    ):
        raise ProfileError('missing_identity', 'anchor session leader identity is invalid')
    owner_stat = _read_stable_process_stat(proc_root, session_leader.ppid, 'smoke runner owner')
    session_leader_pid = session_leader.pid
    leader_label = 'anchor session leader'
    leader_after = _read_stable_process_stat(proc_root, session_leader_pid, leader_label)
    if _stat_context(leader_after) != _stat_context(session_leader):
        raise ProfileError('identity_changed', 'anchor session ownership changed')
    command = _command_identity(proc_root, owner_stat.pid)
    owner_after = _read_stable_process_stat(proc_root, owner_stat.pid, 'smoke runner owner')
    if _stat_context(owner_after) != _stat_context(owner_stat):
        raise ProfileError('identity_changed', 'smoke runner identity changed while latching')
    _validate_smoke_runner_command(config, command['cmdline'])
    owner = SmokeRunnerOwner(
        pid=owner_after.pid,
        start_ticks=owner_after.start_ticks,
        ppid=owner_after.ppid,
        process_group=owner_after.process_group,
        session=owner_after.session,
        comm=owner_after.comm,
        cmdline_sha256=command['cmdline_sha256'],
        executable_link=command['executable_link'],
    )
    _revalidate_smoke_runner_owner(proc_root, owner)
    return owner


def _revalidate_smoke_runner_owner(proc_root: Path, owner: SmokeRunnerOwner) -> None:
    current = _read_stable_process_stat(proc_root, owner.pid, 'smoke runner owner')
    if current.start_ticks != owner.start_ticks:
        raise ProfileError('pid_reuse', f'smoke runner owner PID {owner.pid} was reused')
    if (
        _stat_context(current)
        != (
            owner.pid,
            owner.start_ticks,
            owner.ppid,
            owner.process_group,
            owner.session,
        )
        or current.comm != owner.comm
    ):
        raise ProfileError('identity_changed', 'smoke runner owner identity changed')
    command = _command_identity(proc_root, owner.pid)
    after = _read_stable_process_stat(proc_root, owner.pid, 'smoke runner owner')
    if _stat_context(after) != _stat_context(current):
        raise ProfileError('identity_changed', 'smoke runner owner changed during validation')
    if (
        command['cmdline_sha256'] != owner.cmdline_sha256
        or command['executable_link'] != owner.executable_link
    ):
        raise ProfileError('identity_changed', 'smoke runner command identity changed')


def _process_descends_from_smoke_runner(
    proc_root: Path, observed: ProcStat, owner: SmokeRunnerOwner
) -> bool:
    """Return true only for a stable ancestry chain reaching the latched runner."""
    current = observed
    seen: set[tuple[int, int]] = set()
    for _depth in range(MAX_ANCESTRY_DEPTH):
        stable = _read_stable_process_stat(proc_root, current.pid, 'matching process')
        if stable.start_ticks != current.start_ticks:
            raise ProfileError('pid_reuse', f'matching PID {current.pid} was reused')
        if _stat_context(stable) != _stat_context(current):
            if _only_parent_changed(current, stable):
                raise ProcessParentChangedError(current.pid, 'matching process')
            raise ProfileError('identity_changed', 'matching process ancestry changed')
        current = stable
        identity = (current.pid, current.start_ticks)
        if identity == (owner.pid, owner.start_ticks):
            return True
        if identity in seen:
            raise ProfileError('incomplete_profile', 'matching process ancestry contains a cycle')
        seen.add(identity)
        if current.ppid <= 1:
            return False
        child = current
        parent = _read_stable_process_stat(proc_root, current.ppid, 'matching process ancestor')
        child_after = _read_stable_process_stat(proc_root, child.pid, 'matching process')
        if _stat_context(child_after) != _stat_context(child):
            if _only_parent_changed(child, child_after):
                raise ProcessParentChangedError(child.pid, 'matching process')
            raise ProfileError('identity_changed', 'matching process ancestry changed')
        current = parent
    raise ProfileError('overflow', 'matching process ancestry exceeds 256 processes')


def _discovered_process_ended_after_ancestry_failure(
    proc_root: Path,
    observed: ProcStat,
    error: ProcessDisappearedError,
) -> bool:
    """Confirm only an observed process's own terminal race as benign."""
    if error.pid != observed.pid:
        return False
    try:
        current = _read_stat(proc_root / str(observed.pid) / 'stat')
    except FileNotFoundError:
        return True
    if not _same_process(current, observed):
        raise ProfileError('pid_reuse', f'matching PID {observed.pid} was reused')
    return current.state in TERMINAL_PROCESS_STATES


def _reparented_command_identity(proc_root: Path, pid: int) -> dict[str, Any]:
    """Preserve command-read disappearance so the caller can prove a terminal race."""
    try:
        return _command_identity(proc_root, pid)
    except FileNotFoundError as exc:
        if not _procfs_entry_disappeared(exc):
            raise
        raise ProcessDisappearedError(pid, 'reparented matching process') from exc
    except ProfileError as exc:
        cause = exc.__cause__
        if (
            exc.kind != 'missing_identity'
            or not isinstance(cause, OSError)
            or not _procfs_entry_disappeared(cause)
        ):
            raise
        raise ProcessDisappearedError(pid, 'reparented matching process') from exc


def _validate_latched_process_stat_identity(observed: ProcStat, latched: LatchedProcess) -> None:
    if observed.start_ticks != latched.start_ticks:
        raise ProfileError('pid_reuse', f'latched PID {observed.pid} was reused')
    if observed.process_group != latched.process_group or observed.session != latched.session:
        raise ProfileError(
            'identity_changed',
            f'latched PID {observed.pid} process group or session changed after reparenting',
        )
    if observed.comm != latched.comm:
        raise ProfileError('identity_changed', f'latched PID {observed.pid} command name changed')


def _validate_latched_process_identity(
    proc_root: Path,
    config: ProfileConfig,
    observed: ProcStat,
    latched: LatchedProcess,
) -> None:
    """Revalidate every immutable identity except the intentionally mutable PPID."""
    if latched.ended_sample is not None:
        raise ProfileError('identity_changed', f'ended PID {observed.pid} became live')
    _validate_latched_process_stat_identity(observed, latched)
    command = _reparented_command_identity(proc_root, observed.pid)
    try:
        environment = _bounded_proc_read(
            proc_root / str(observed.pid) / 'environ',
            ENVIRON_MAX_BYTES,
            'reparented proc environ',
        )
    except FileNotFoundError as exc:
        raise ProcessDisappearedError(observed.pid, 'reparented matching process') from exc
    after = _read_stable_process_stat(proc_root, observed.pid, 'reparented matching process')
    _validate_latched_process_stat_identity(after, latched)
    if after.state in TERMINAL_PROCESS_STATES:
        raise ProcessDisappearedError(observed.pid, 'reparented matching process')
    if (
        command['cmdline_sha256'] != latched.cmdline_sha256
        or command['cmdline_size_bytes'] != latched.cmdline_size_bytes
        or command['executable_link'] != latched.executable_link
    ):
        raise ProfileError(
            'identity_changed',
            f'latched PID {observed.pid} command identity changed after reparenting',
        )
    if not has_exact_environment(environment, config.ros_domain_id, config.gz_partition):
        raise ProfileError(
            'identity_changed', f'latched PID {observed.pid} changed isolation identity'
        )


def _latched_process_survived_reparenting(
    proc_root: Path,
    config: ProfileConfig,
    observed: ProcStat,
    latched: LatchedProcess | None,
) -> bool:
    """Keep tracking one proven target after only its parent ancestry changes."""
    if latched is None:
        return False
    _validate_latched_process_identity(proc_root, config, observed, latched)
    return True


def _reclassify_latched_process_once(
    proc_root: Path,
    config: ProfileConfig,
    observed: ProcStat,
    latched: LatchedProcess,
    owner: SmokeRunnerOwner,
) -> bool | None:
    """Resolve one ancestry race once; return ``None`` only for a proven end."""
    try:
        fresh = _read_stable_process_stat(proc_root, observed.pid, 'revalidated matching process')
    except ProcessDisappearedError as exc:
        if _discovered_process_ended_after_ancestry_failure(proc_root, observed, exc):
            return None
        raise
    _validate_latched_process_stat_identity(fresh, latched)
    if fresh.state in TERMINAL_PROCESS_STATES:
        return None

    descends_from_owner = _process_descends_from_smoke_runner(proc_root, fresh, owner)
    try:
        if descends_from_owner:
            _validate_latched_process_identity(proc_root, config, fresh, latched)
        else:
            _latched_process_survived_reparenting(proc_root, config, fresh, latched)
    except ProcessDisappearedError as exc:
        if _discovered_process_ended_after_ancestry_failure(proc_root, fresh, exc):
            return None
        raise
    return True


def _classify_matching_process(
    proc_root: Path,
    config: ProfileConfig,
    observed: ProcStat,
    latched: LatchedProcess | None,
    owner: SmokeRunnerOwner,
) -> bool | None:
    """Classify one exact-isolation process with at most one ancestry retry."""
    try:
        descends_from_owner = _process_descends_from_smoke_runner(proc_root, observed, owner)
    except (ProcessDisappearedError, ProcessParentChangedError) as exc:
        if latched is None or latched.ended_sample is not None:
            raise
        _validate_latched_process_stat_identity(observed, latched)
        if isinstance(exc, ProcessDisappearedError) and exc.pid == observed.pid:
            if _discovered_process_ended_after_ancestry_failure(proc_root, observed, exc):
                return None
            raise
        retryable = isinstance(exc, ProcessDisappearedError) or (
            isinstance(exc, ProcessParentChangedError) and exc.pid != owner.pid
        )
        if not retryable:
            raise
        return _reclassify_latched_process_once(proc_root, config, observed, latched, owner)
    if descends_from_owner:
        return True
    try:
        return _latched_process_survived_reparenting(proc_root, config, observed, latched)
    except ProcessDisappearedError as exc:
        if (
            latched is not None
            and latched.ended_sample is None
            and latched.start_ticks == observed.start_ticks
            and _discovered_process_ended_after_ancestry_failure(proc_root, observed, exc)
        ):
            return None
        raise


def _foreign_matching_message(processes: Sequence[ProcStat]) -> str:
    selected = processes[:MAX_FOREIGN_DIAGNOSTIC_IDENTITIES]
    identities = '; '.join(
        f'pid={item.pid},start={item.start_ticks},ppid={item.ppid},'
        f'pgid={item.process_group},sid={item.session}'
        for item in selected
    )
    omitted = len(processes) - len(selected)
    suffix = '' if omitted == 0 else f'; omitted={omitted}'
    return (
        'exact candidate isolation is shared outside the smoke runner process tree: '
        f'{identities}{suffix}'
    )


def collect_anchor(
    proc_root: Path, pid: int, observed: ProcStat, plugin: Mapping[str, Any]
) -> dict[str, Any]:
    """Bind stable process, executable bytes, and exact DSO map segments."""
    root = proc_root / str(pid)
    before = _read_stat(root / 'stat')
    if not _same_process(before, observed):
        raise ProfileError('pid_reuse', f'anchor PID {pid} changed before capture')
    profile_environment = parse_environ(
        _bounded_proc_read(root / 'environ', ENVIRON_MAX_BYTES, 'anchor proc environ')
    )
    if profile_environment.get(b'ROBOTEST_CONTACT_PROFILE') != [b'1']:
        raise ProfileError(
            'missing_identity',
            'anchor ROBOTEST_CONTACT_PROFILE must be exactly 1',
        )
    mappings = _plugin_maps(proc_root, pid, plugin)
    if (
        not mappings
        or not any(item['offset'] == 0 for item in mappings)
        or not any('x' in item['permissions'] for item in mappings)
    ):
        raise ProfileError('missing_identity', 'anchor lacks complete contact DSO mappings')
    command = _command_identity(proc_root, pid)
    executable_path = (root / 'exe').resolve(strict=True)
    executable = _file_identity(executable_path, BINARY_MAX_BYTES, 'anchor executable')
    after_mappings = _plugin_maps(proc_root, pid, plugin)
    after = _read_stat(root / 'stat')
    if not _same_process(before, after):
        raise ProfileError('pid_reuse', f'anchor PID {pid} changed during capture')
    if mappings != after_mappings:
        raise ProfileError('identity_changed', 'anchor DSO mappings changed during capture')
    return {
        **command,
        'pid': pid,
        'start_ticks': after.start_ticks,
        'ppid': after.ppid,
        'process_group': after.process_group,
        'session': after.session,
        'comm': after.comm,
        'executable': executable,
        'mappings': mappings,
        'mapping_count': len(mappings),
        'mapping_fingerprint_sha256': hashlib.sha256(canonical_json_bytes(mappings)).hexdigest(),
        'expected_plugin': {key: plugin[key] for key in ('device', 'inode', 'path', 'sha256')},
    }


def discover_anchor(
    proc_root: Path, domain_id: int, partition: str, plugin: Mapping[str, Any]
) -> tuple[dict[str, Any] | None, int]:
    """Find exactly one prepared contact DSO host in the exact isolation."""
    matching = discover_matching(proc_root, domain_id, partition)
    candidates: list[tuple[int, ProcStat]] = []
    for pid, process_stat in matching:
        try:
            mappings = _plugin_maps(proc_root, pid, plugin)
        except FileNotFoundError:
            continue
        if mappings:
            candidates.append((pid, process_stat))
    if len(candidates) > 1:
        raise ProfileError('missing_identity', 'multiple exact-isolation DSO hosts found')
    if not candidates:
        return None, len(matching)
    pid, process_stat = candidates[0]
    return collect_anchor(proc_root, pid, process_stat, plugin), len(matching)


def _latch(proc_root: Path, pid: int, process_stat: ProcStat, index: int) -> LatchedProcess:
    command = _command_identity(proc_root, pid)
    return LatchedProcess(
        pid=pid,
        start_ticks=process_stat.start_ticks,
        process_group=process_stat.process_group,
        session=process_stat.session,
        comm=process_stat.comm,
        cmdline=command['cmdline'],
        cmdline_sha256=command['cmdline_sha256'],
        cmdline_size_bytes=command['cmdline_size_bytes'],
        executable_link=command['executable_link'],
        first_sample=index,
    )


def _sample_threads(
    proc_root: Path, process: LatchedProcess, index: int, state: ProfileState
) -> dict[str, Any] | None:
    root = proc_root / str(process.pid)
    try:
        before = _read_stat(root / 'stat')
    except FileNotFoundError:
        process.ended_sample = index
        return None
    if before.start_ticks != process.start_ticks:
        raise ProfileError('pid_reuse', f'PID {process.pid} was reused')
    if before.state in TERMINAL_PROCESS_STATES:
        process.ended_sample = index
        return None
    try:
        tids = sorted(int(path.name) for path in (root / 'task').iterdir() if path.name.isdigit())
    except OSError as exc:
        if _procfs_entry_disappeared(exc):
            process.ended_sample = index
            return None
        message = f'cannot enumerate PID {process.pid} threads'
        raise ProfileError('incomplete_profile', message) from exc
    if len(tids) > MAX_THREADS_PER_PROCESS:
        raise ProfileError('overflow', f'PID {process.pid} exceeds 4,096 threads')
    records: list[dict[str, Any]] = []
    updates: list[ThreadSampleUpdate] = []
    new_identities: set[tuple[int, int, int, int]] = set()
    vanished = 0
    for tid in tids:
        try:
            thread = _read_stat(root / 'task' / str(tid) / 'stat')
        except FileNotFoundError:
            vanished += 1
            continue
        identity = (process.pid, process.start_ticks, tid, thread.start_ticks)
        tid_identity = (process.pid, process.start_ticks, tid)
        prior_start = state.thread_start_by_tid.get(tid_identity)
        if prior_start is not None and prior_start != thread.start_ticks:
            raise ProfileError('pid_reuse', f'thread {process.pid}/{tid} was reused')
        if thread.pid != tid:
            message = f'thread stat PID differs for {process.pid}/{tid}'
            raise ProfileError('incomplete_profile', message)
        prior = state.last_thread_ticks.get(identity)
        if prior is not None and thread.cpu_ticks < prior:
            raise ProfileError('pid_reuse', f'thread {process.pid}/{tid} CPU ticks regressed')
        delta = None if prior is None else thread.cpu_ticks - prior
        if identity not in state.last_thread_ticks:
            new_identities.add(identity)
        if len(state.last_thread_ticks) + len(new_identities) > MAX_THREAD_IDENTITIES:
            raise ProfileError('overflow', 'profile exceeds 16,384 thread identities')
        updates.append((identity, tid_identity, thread, delta))
        records.append(
            {
                'tid': tid,
                'start_ticks': thread.start_ticks,
                'comm': thread.comm,
                'state': thread.state,
                'processor': thread.processor,
                'utime_ticks': thread.utime_ticks,
                'stime_ticks': thread.stime_ticks,
                'cpu_ticks': thread.cpu_ticks,
                'cpu_delta_ticks': delta,
            }
        )
    if len(records) > MAX_THREAD_RECORDS - state.thread_records:
        raise ProfileError(
            'overflow', f'profile exceeds {MAX_THREAD_RECORDS:,} thread sample records'
        )
    try:
        after = _read_stat(root / 'stat')
    except FileNotFoundError:
        process.ended_sample = index
        return None
    if after.start_ticks != process.start_ticks:
        raise ProfileError('pid_reuse', f'PID {process.pid} changed while sampling')
    if after.state in TERMINAL_PROCESS_STATES:
        process.ended_sample = index
        return None
    process_identity = (process.pid, process.start_ticks)
    process_delta = 0
    has_process_delta = False
    for identity, tid_identity, thread, delta in updates:
        state.thread_start_by_tid[tid_identity] = thread.start_ticks
        state.last_thread_ticks[identity] = thread.cpu_ticks
        state.thread_names[identity] = thread.comm
        if delta is not None:
            state.thread_delta_ticks[identity] = state.thread_delta_ticks.get(identity, 0) + delta
            process_delta += delta
            has_process_delta = True
    if has_process_delta:
        state.process_delta_ticks[process_identity] = (
            state.process_delta_ticks.get(process_identity, 0) + process_delta
        )
    state.thread_records += len(records)
    return {
        'pid': process.pid,
        'start_ticks': process.start_ticks,
        'comm': process.comm,
        'cpu_ticks': after.cpu_ticks,
        'thread_count': len(records),
        'threads_vanished_during_sample': vanished,
        'threads': records,
    }


def _capture_sample(
    proc_root: Path,
    config: ProfileConfig,
    anchor: Mapping[str, Any],
    state: ProfileState,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    wall_time_ns: Callable[[], int] = time.time_ns,
) -> bool:
    """Capture one host and per-thread sample; return whether any target remains."""
    if len(state.samples) >= MAX_SAMPLES:
        raise ProfileError('overflow', 'profile exceeds 2,048 samples')
    index = len(state.samples)
    matching = discover_matching(proc_root, config.ros_domain_id, config.gz_partition)
    if matching:
        if state.smoke_runner_owner is None:
            state.smoke_runner_owner = discover_smoke_runner_owner(proc_root, config, anchor)
        else:
            _revalidate_smoke_runner_owner(proc_root, state.smoke_runner_owner)
    target_matching: list[tuple[int, ProcStat]] = []
    foreign_matching: list[ProcStat] = []
    ancestry_vanished = 0
    owner = state.smoke_runner_owner
    if matching and owner is None:
        raise ProfileError('missing_identity', 'smoke runner owner was not latched')
    for pid, process_stat in matching:
        assert owner is not None
        classification = _classify_matching_process(
            proc_root,
            config,
            process_stat,
            state.processes.get(pid),
            owner,
        )
        if classification is None:
            ancestry_vanished += 1
            continue
        if classification:
            target_matching.append((pid, process_stat))
        else:
            foreign_matching.append(process_stat)
    if foreign_matching:
        raise ProfileError('missing_identity', _foreign_matching_message(foreign_matching))
    matching_by_pid = dict(target_matching)
    for pid, process in state.processes.items():
        try:
            current = _read_stat(proc_root / str(pid) / 'stat')
        except FileNotFoundError:
            if process.ended_sample is None:
                process.ended_sample = index
            continue
        if current.start_ticks != process.start_ticks:
            raise ProfileError('pid_reuse', f'latched PID {pid} was reused')
        if current.state in TERMINAL_PROCESS_STATES:
            if process.ended_sample is None:
                process.ended_sample = index
            continue
        if process.ended_sample is not None:
            raise ProfileError('identity_changed', f'ended PID {pid} became live')
        if pid not in matching_by_pid:
            raise ProfileError('identity_changed', f'PID {pid} changed isolation identity')
    for pid, process_stat in target_matching:
        if pid in state.processes:
            continue
        if len(state.processes) >= MAX_PROCESSES:
            raise ProfileError('overflow', 'profile exceeds 1,024 process identities')
        latched = _latch(proc_root, pid, process_stat, index)
        if state.cmdline_bytes + latched.cmdline_size_bytes > MAX_CMDLINE_TOTAL_BYTES:
            raise ProfileError('overflow', 'retained process command lines exceed 4 MiB')
        state.cmdline_bytes += latched.cmdline_size_bytes
        state.processes[pid] = latched
    anchor_pid = int(anchor['pid'])
    if anchor_pid not in state.processes:
        raise ProfileError('missing_identity', 'anchor is absent from matching process set')
    anchor_process = state.processes[anchor_pid]
    if anchor_process.ended_sample is None:
        try:
            mappings = _plugin_maps(proc_root, anchor_pid, anchor['expected_plugin'])
        except FileNotFoundError:
            anchor_process.ended_sample = index
        else:
            if (
                not mappings
                or not any(item['offset'] == 0 for item in mappings)
                or not any('x' in item['permissions'] for item in mappings)
            ):
                raise ProfileError('identity_changed', 'anchor DSO mapping became incomplete')
    process_samples = []
    sample_thread_records = 0
    vanished = ancestry_vanished
    for pid in sorted(state.processes):
        process = state.processes[pid]
        if process.ended_sample is not None:
            continue
        sampled = _sample_threads(proc_root, process, index, state)
        if sampled is None:
            vanished += 1
            if vanished > MAX_PROCESSES:
                raise ProfileError('overflow', 'sample vanished process count exceeds bound')
        else:
            sample_thread_records += sampled['thread_count']
            if sample_thread_records > MAX_LIVE_THREADS_PER_SAMPLE:
                raise ProfileError(
                    'overflow',
                    f'profile sample exceeds {MAX_LIVE_THREADS_PER_SAMPLE:,} live threads',
                )
            process_samples.append(sampled)
    anchor_alive = state.processes[anchor_pid].ended_sample is None
    target_alive = any(process.ended_sample is None for process in state.processes.values())
    state.samples.append(
        {
            'index': index,
            'monotonic_ns': monotonic_ns(),
            'wall_epoch_ns': wall_time_ns(),
            'anchor_alive': anchor_alive,
            'target_process_set_alive': target_alive,
            'host': read_host_sample(proc_root),
            'process_count': len(process_samples),
            'processes_vanished_during_sample': vanished,
            'processes': process_samples,
        }
    )
    return target_alive


def capture_sample(
    proc_root: Path,
    config: ProfileConfig,
    anchor: Mapping[str, Any],
    state: ProfileState,
    monotonic_ns: Callable[[], int] = time.monotonic_ns,
    wall_time_ns: Callable[[], int] = time.time_ns,
) -> bool:
    """Atomically capture one sample without exposing partially updated state."""
    working = ProfileState(
        processes=copy.deepcopy(state.processes),
        samples=list(state.samples),
        last_thread_ticks=dict(state.last_thread_ticks),
        thread_start_by_tid=dict(state.thread_start_by_tid),
        thread_names=dict(state.thread_names),
        thread_delta_ticks=dict(state.thread_delta_ticks),
        process_delta_ticks=dict(state.process_delta_ticks),
        cmdline_bytes=state.cmdline_bytes,
        thread_records=state.thread_records,
        cadence_overruns=state.cadence_overruns,
        smoke_runner_owner=state.smoke_runner_owner,
    )
    target_alive = _capture_sample(
        proc_root,
        config,
        anchor,
        working,
        monotonic_ns,
        wall_time_ns,
    )
    state.processes = working.processes
    state.samples = working.samples
    state.last_thread_ticks = working.last_thread_ticks
    state.thread_start_by_tid = working.thread_start_by_tid
    state.thread_names = working.thread_names
    state.thread_delta_ticks = working.thread_delta_ticks
    state.process_delta_ticks = working.process_delta_ticks
    state.cmdline_bytes = working.cmdline_bytes
    state.thread_records = working.thread_records
    state.cadence_overruns = working.cadence_overruns
    state.smoke_runner_owner = working.smoke_runner_owner
    return target_alive


def _renderer_baseline(path: Path) -> dict[str, Any]:
    if not path.exists() and not path.is_symlink():
        return {'present': False, 'path': str(path)}
    payload = _bounded_regular_read(path, RENDERER_MAX_BYTES, 'pre-smoke renderer log')
    result = _file_identity(path, RENDERER_MAX_BYTES, 'pre-smoke renderer log')
    if hashlib.sha256(payload).hexdigest() != result['sha256']:
        raise ProfileError('identity_changed', 'renderer baseline changed during capture')
    result['_payload'] = payload
    result['present'] = True
    return result


def collect_renderer(
    path: Path, baseline: Mapping[str, Any], profile_started_epoch_ns: int
) -> dict[str, Any]:
    """Require fresh renderer/device identity attributable to this smoke."""
    payload = _bounded_regular_read(path, RENDERER_MAX_BYTES, 'smoke renderer log')
    identity = _file_identity(path, RENDERER_MAX_BYTES, 'smoke renderer log')
    if hashlib.sha256(payload).hexdigest() != identity['sha256']:
        raise ProfileError('identity_changed', 'renderer log changed during capture')
    before_payload = baseline.get('_payload', b'')
    if not isinstance(before_payload, bytes):
        raise ProfileError('missing_identity', 'renderer baseline payload is invalid')
    if baseline.get('present') is not True:
        segment = payload
        segment_offset = 0
        segment_mode = 'created'
    elif (
        baseline.get('device') == identity['device']
        and baseline.get('inode') == identity['inode']
        and payload.startswith(before_payload)
    ):
        segment = payload[len(before_payload) :]  # noqa: E203, RUF100
        segment_offset = len(before_payload)
        segment_mode = 'appended'
    else:
        segment = payload
        segment_offset = 0
        segment_mode = 'rewritten_or_rotated'
    if not segment:
        raise ProfileError('missing_identity', 'renderer log has no post-start byte segment')
    try:
        lines = segment.decode().splitlines()
    except UnicodeError as exc:
        raise ProfileError('missing_identity', 'renderer log is not UTF-8') from exc
    evidence = []
    for line in lines:
        if len(line.encode()) > MAX_LINE_BYTES:
            raise ProfileError('overflow', 'renderer log line exceeds 64 KiB')
        if RENDERER_RE.search(line):
            evidence.append(line)
            if len(evidence) > MAX_RENDERER_LINES:
                raise ProfileError('overflow', 'renderer identity exceeds 256 lines')
    if not evidence:
        raise ProfileError('missing_identity', 'renderer log lacks device identity')
    before_hash = baseline.get('sha256') if baseline.get('present') is True else None
    changed = before_hash is None or before_hash != identity['sha256']
    fresh = identity['mtime_ns'] >= profile_started_epoch_ns
    if not changed or not fresh:
        raise ProfileError('missing_identity', 'renderer log is not fresh smoke evidence')
    identity.update(
        {
            'baseline_sha256': before_hash,
            'content_delta_from_pre_smoke': changed,
            'fresh_after_profile_start': fresh,
            'identity_lines': evidence,
            'post_start_segment_mode': segment_mode,
            'post_start_segment_offset_bytes': segment_offset,
            'post_start_segment_sha256': hashlib.sha256(segment).hexdigest(),
            'post_start_segment_size_bytes': len(segment),
            'pre_smoke_identity': {
                key: value for key, value in baseline.items() if key != '_payload'
            },
        }
    )
    return identity


def validate_contact_profile_records(records: Sequence[Mapping[str, Any]]) -> None:
    """Validate cumulative migration-safe CLOCK_THREAD_CPUTIME_ID records."""
    if not records:
        raise ProfileError('missing_identity', 'contact profile record sequence is empty')
    previous: Mapping[str, Any] | None = None
    previous_contributions: dict[int, Mapping[str, Any]] = {}
    for index, record in enumerate(records):
        if record.get('status') == 'FAIL':
            if (
                set(record) != CONTACT_PROFILE_FAILURE_KEYS
                or type(record.get('schema_version')) is not int
                or record.get('schema_version') != 2
                or type(record.get('sim_stamp_ns')) is not int
                or not 0 <= record['sim_stamp_ns'] <= INT64_MAX
                or record.get('failure_kind') not in CONTACT_PROFILE_FAILURE_KINDS
            ):
                raise ProfileError('missing_identity', 'contact profile failure record is invalid')
            raise ProfileError(
                'incomplete_profile',
                f'contact profile producer reported {record["failure_kind"]}',
            )
        if set(record) != CONTACT_PROFILE_KEYS:
            raise ProfileError('missing_identity', f'contact profile record {index} keys differ')
        if (
            record.get('status') != 'PASS'
            or type(record['schema_version']) is not int
            or record['schema_version'] != 2
        ):
            raise ProfileError('missing_identity', 'contact profile schema version differs')
        if record['clock_id'] != 'CLOCK_THREAD_CPUTIME_ID':
            raise ProfileError('missing_identity', 'contact profile clock ID differs')
        if record['saturated'] is not False:
            raise ProfileError('overflow', 'contact profile counter saturation was reported')
        for field_name in CONTACT_MONOTONIC_KEYS:
            value = record[field_name]
            if type(value) is not int or not 0 <= value <= UINT64_MAX:
                raise ProfileError('missing_identity', f'contact profile {field_name} is invalid')
        for field_name in ('profile_epoch_start_sim_stamp_ns', 'sim_stamp_ns'):
            value = record[field_name]
            if type(value) is not int or not 0 <= value <= INT64_MAX:
                raise ProfileError('missing_identity', f'contact profile {field_name} is invalid')
        measured = sum(record[field_name] for field_name in CONTACT_BUCKET_KEYS)
        if measured > UINT64_MAX or record['measured_total_ns'] != measured:
            raise ProfileError('missing_identity', 'contact measured total does not reconcile')
        contributions = record.get('thread_contributions')
        if (
            not isinstance(contributions, list)
            or not 1 <= len(contributions) <= MAX_CONTACT_THREAD_CONTRIBUTIONS
        ):
            raise ProfileError('missing_identity', 'contact thread contributions are invalid')
        current_contributions: dict[int, Mapping[str, Any]] = {}
        aggregate = {field_name: 0 for field_name in CONTACT_CONTRIBUTION_MONOTONIC_KEYS}
        order: list[int] = []
        for contribution_index, contribution in enumerate(contributions):
            if (
                not isinstance(contribution, Mapping)
                or set(contribution) != CONTACT_THREAD_CONTRIBUTION_KEYS
            ):
                raise ProfileError(
                    'missing_identity',
                    f'contact thread contribution {contribution_index} keys differ',
                )
            linux_tid = contribution.get('linux_tid')
            if type(linux_tid) is not int or not 1 <= linux_tid <= INT64_MAX:
                raise ProfileError('missing_identity', 'contact profile Linux TID is invalid')
            order.append(linux_tid)
            for field_name in CONTACT_CONTRIBUTION_MONOTONIC_KEYS:
                value = contribution[field_name]
                if type(value) is not int or not 0 <= value <= UINT64_MAX:
                    raise ProfileError(
                        'missing_identity',
                        f'contact thread contribution {field_name} is invalid',
                    )
                aggregate[field_name] += value
                if aggregate[field_name] > UINT64_MAX:
                    raise ProfileError('overflow', 'contact contribution aggregate overflowed')
            contribution_measured = sum(
                contribution[field_name] for field_name in CONTACT_BUCKET_KEYS
            )
            if (
                contribution_measured > UINT64_MAX
                or contribution['measured_total_ns'] != contribution_measured
            ):
                raise ProfileError(
                    'missing_identity', 'contact thread measured total does not reconcile'
                )
            prior_contribution = previous_contributions.get(linux_tid)
            if prior_contribution is not None and any(
                contribution[field_name] < prior_contribution[field_name]
                for field_name in CONTACT_CONTRIBUTION_MONOTONIC_KEYS
            ):
                raise ProfileError('missing_identity', 'contact thread contribution regressed')
            current_contributions[linux_tid] = contribution
        if order != sorted(order) or len(order) != len(set(order)):
            raise ProfileError('missing_identity', 'contact thread contributions are not sorted')
        if not previous_contributions.keys() <= current_contributions.keys():
            raise ProfileError('identity_changed', 'contact thread contribution disappeared')
        if any(record[field_name] != aggregate[field_name] for field_name in aggregate):
            raise ProfileError('missing_identity', 'contact contribution totals do not reconcile')
        epoch = record['profile_epoch_start_sim_stamp_ns']
        if record['sim_stamp_ns'] < epoch + 5_000_000_000:
            message = 'contact profile record precedes first due stamp'
            raise ProfileError('missing_identity', message)
        if previous is not None:
            if epoch != previous['profile_epoch_start_sim_stamp_ns']:
                raise ProfileError('identity_changed', 'contact profile epoch changed')
            if record['sim_stamp_ns'] - previous['sim_stamp_ns'] < 5_000_000_000:
                raise ProfileError('missing_identity', 'contact profile cadence is too short')
            for field_name in CONTACT_MONOTONIC_KEYS:
                if record[field_name] < previous[field_name]:
                    raise ProfileError(
                        'missing_identity', f'contact profile {field_name} regressed'
                    )
        previous = record
        previous_contributions = current_contributions


def parse_contact_profile_logs(run_dir: Path) -> dict[str, Any]:
    """Parse the last valid canonical cumulative record, when one is present."""
    sources = []
    selected: list[tuple[Path, Mapping[str, Any], int]] = []
    for path in (
        run_dir / 'processes/full_stack.stdout.log',
        run_dir / 'processes/full_stack.stderr.log',
    ):
        if not path.exists() and not path.is_symlink():
            continue
        payload = _bounded_regular_read(path, LOG_MAX_BYTES, 'full-stack profile log')
        valid: list[Mapping[str, Any]] = []
        invalid = 0
        for line in payload.splitlines():
            if len(line) > MAX_LINE_BYTES:
                raise ProfileError('overflow', f'full-stack line exceeds 64 KiB: {path}')
            offset = line.find(CONTACT_PROFILE_PREFIX)
            if offset < 0:
                continue
            encoded = line[offset + len(CONTACT_PROFILE_PREFIX) :]  # noqa: E203, RUF100
            try:
                record = json.loads(encoded.decode())
                if (
                    not isinstance(record, Mapping)
                    or canonical_json_bytes(record).rstrip(b'\n') != encoded
                ):
                    raise ValueError('not canonical object')
            except (UnicodeError, json.JSONDecodeError, ProfileError, ValueError):
                invalid += 1
                continue
            valid.append(record)
            if len(valid) > MAX_CONTACT_RECORDS:
                raise ProfileError('overflow', 'contact profile exceeds 128 records')
        sources.append(
            {
                'path': str(path),
                'sha256': hashlib.sha256(payload).hexdigest(),
                'size_bytes': len(payload),
                'valid_record_count': len(valid),
                'invalid_record_count': invalid,
            }
        )
        if invalid:
            raise ProfileError('missing_identity', f'invalid contact profile line in {path}')
        if valid:
            validate_contact_profile_records(valid)
            selected.append((path, valid[-1], len(valid)))
    if len(selected) > 1:
        raise ProfileError('incomplete_profile', 'contact records span both output streams')
    if not selected:
        return {
            'present': False,
            'selected_source': None,
            'selected_source_valid_record_count': 0,
            'last_valid_cumulative_record': None,
            'sources': sources,
        }
    path, record, count = selected[0]
    return {
        'present': True,
        'selected_source': str(path),
        'selected_source_valid_record_count': count,
        'last_valid_cumulative_record': record,
        'sources': sources,
    }


def wait_for_full_stack_close(
    run_dir: Path,
    anchor: Mapping[str, Any],
    monotonic: Callable[[], float],
    sleep: Callable[[float], None],
    *,
    profile_started_monotonic_ns: int,
    timeout_s: float = FULL_STACK_CLOSE_TIMEOUT_S,
) -> dict[str, Any]:
    """Wait for the runner's bounded-drain metadata after observed targets exit."""
    if type(profile_started_monotonic_ns) is not int or profile_started_monotonic_ns < 0:
        raise ProfileError('incomplete_profile', 'profile start steady time is invalid')
    path = run_dir / 'processes/full_stack.process.json'
    deadline = monotonic() + timeout_s
    while monotonic() < deadline and not path.exists():
        sleep(min(0.05, max(0.0, deadline - monotonic())))
    if not path.exists():
        raise ProfileError('incomplete_profile', 'full-stack process metadata did not appear')
    payload = _bounded_regular_read(path, 256 * 1024, 'full-stack process metadata')
    try:
        document = json.loads(payload.decode())
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ProfileError('incomplete_profile', 'full-stack metadata is invalid JSON') from exc
    if not isinstance(document, Mapping):
        raise ProfileError('incomplete_profile', 'full-stack metadata is not an object')
    started_steady_ns = document.get('started_steady_ns')
    finished_steady_ns = document.get('finished_steady_ns')
    if (
        payload != canonical_json_bytes(document)
        or document.get('role') != 'full_stack'
        or document.get('group_confirmed_empty') is not True
        or document.get('timed_out') is not False
        or type(document.get('returncode')) is not int
        or type(started_steady_ns) is not int
        or type(finished_steady_ns) is not int
        or started_steady_ns <= 0
        or finished_steady_ns <= 0
        or started_steady_ns < profile_started_monotonic_ns
        or finished_steady_ns < started_steady_ns
    ):
        raise ProfileError('incomplete_profile', 'full-stack process did not close cleanly')
    pid = document.get('pid')
    pgid = document.get('pgid')
    if (
        type(pid) is not int
        or type(pgid) is not int
        or pid <= 0
        or pid != pgid
        or anchor.get('session') != pgid
    ):
        raise ProfileError('missing_identity', 'anchor is not in the full-stack session')
    stream_evidence: dict[str, dict[str, Any]] = {}
    expected_stream_keys = {
        'error',
        'maximum_bytes',
        'observed_bytes',
        'overflow',
        'retained_bytes',
    }
    for stream_name in ('stdout', 'stderr'):
        stream = document.get(stream_name)
        log_path = run_dir / 'processes' / f'full_stack.{stream_name}.log'
        log_payload = _bounded_regular_read(log_path, LOG_MAX_BYTES, f'full-stack {stream_name}')
        if (
            not isinstance(stream, Mapping)
            or set(stream) != expected_stream_keys
            or stream.get('error') is not None
            or stream.get('maximum_bytes') != LOG_MAX_BYTES
            or stream.get('overflow') is not False
            or stream.get('observed_bytes') != len(log_payload)
            or stream.get('retained_bytes') != len(log_payload)
        ):
            raise ProfileError(
                'incomplete_profile', f'full-stack {stream_name} was not fully retained'
            )
        stream_evidence[stream_name] = {
            'path': str(log_path),
            'sha256': hashlib.sha256(log_payload).hexdigest(),
            'size_bytes': len(log_payload),
        }
    return {
        'path': str(path),
        'sha256': hashlib.sha256(payload).hexdigest(),
        'pid': pid,
        'pgid': pgid,
        'started_steady_ns': started_steady_ns,
        'finished_steady_ns': finished_steady_ns,
        'returncode': document.get('returncode'),
        'group_confirmed_empty': True,
        'streams': stream_evidence,
    }


def reconcile_contact_log_sources(
    contact: Mapping[str, Any], full_stack: Mapping[str, Any]
) -> None:
    """Join the parser's second read to the already closed stream identities."""
    sources = contact.get('sources')
    streams = full_stack.get('streams')
    if not isinstance(sources, list) or not isinstance(streams, Mapping):
        raise ProfileError('incomplete_profile', 'contact/full-stack log evidence is incomplete')
    parsed_by_path = {item.get('path'): item for item in sources if isinstance(item, Mapping)}
    if len(parsed_by_path) != len(sources) or len(parsed_by_path) != len(streams):
        raise ProfileError('identity_changed', 'contact parser stream set differs from full stack')
    for stream_name in ('stdout', 'stderr'):
        expected = streams.get(stream_name)
        if not isinstance(expected, Mapping):
            message = f'full-stack {stream_name} identity is absent'
            raise ProfileError('incomplete_profile', message)
        parsed = parsed_by_path.get(expected.get('path'))
        if not isinstance(parsed, Mapping) or any(
            parsed.get(key) != expected.get(key) for key in ('path', 'sha256', 'size_bytes')
        ):
            raise ProfileError('identity_changed', f'full-stack {stream_name} changed after close')


def bind_contact_profile_threads(
    contact: Mapping[str, Any], anchor: Mapping[str, Any], state: ProfileState
) -> list[dict[str, Any]]:
    """Bind every in-plugin Linux TID to one sampled thread in the DSO host."""
    record = contact.get('last_valid_cumulative_record')
    anchor_pid = anchor.get('pid')
    anchor_start = anchor.get('start_ticks')
    if (
        not isinstance(record, Mapping)
        or type(anchor_pid) is not int
        or type(anchor_start) is not int
    ):
        raise ProfileError('missing_identity', 'contact/anchor thread identity is incomplete')
    contributions = record.get('thread_contributions')
    if not isinstance(contributions, list):
        raise ProfileError('missing_identity', 'contact profile thread contributions are absent')
    bindings = []
    for contribution in contributions:
        if not isinstance(contribution, Mapping) or type(contribution.get('linux_tid')) is not int:
            raise ProfileError('missing_identity', 'contact profile Linux TID is invalid')
        linux_tid = contribution['linux_tid']
        tid_identity = (anchor_pid, anchor_start, linux_tid)
        thread_start = state.thread_start_by_tid.get(tid_identity)
        if thread_start is None:
            raise ProfileError(
                'missing_identity',
                'contact profile Linux TID was not sampled in the DSO host',
            )
        identity = (*tid_identity, thread_start)
        comm = state.thread_names.get(identity)
        if not isinstance(comm, str):
            raise ProfileError('missing_identity', 'contact profile host thread name is absent')
        sample_indexes = [
            sample['index']
            for sample in state.samples
            for process in sample['processes']
            if process['pid'] == anchor_pid and process['start_ticks'] == anchor_start
            for thread in process['threads']
            if thread['tid'] == linux_tid and thread['start_ticks'] == thread_start
        ]
        if not sample_indexes:
            raise ProfileError('missing_identity', 'contact profile host thread has no samples')
        bindings.append(
            {
                'anchor_pid': anchor_pid,
                'anchor_start_ticks': anchor_start,
                'linux_tid': linux_tid,
                'thread_start_ticks': thread_start,
                'comm': comm,
                'first_sample_index': min(sample_indexes),
                'last_sample_index': max(sample_indexes),
                'sample_count': len(sample_indexes),
                'sampled_cpu_delta_ticks': state.thread_delta_ticks.get(identity, 0),
            }
        )
    if len(bindings) > MAX_CONTACT_THREAD_CONTRIBUTIONS:
        raise ProfileError('overflow', 'contact host-thread bindings exceed the bound')
    return bindings


def _process_lifecycles(state: ProfileState) -> list[dict[str, Any]]:
    return [
        {
            'pid': item.pid,
            'start_ticks': item.start_ticks,
            'comm': item.comm,
            'cmdline': item.cmdline,
            'cmdline_sha256': item.cmdline_sha256,
            'cmdline_size_bytes': item.cmdline_size_bytes,
            'executable_link': item.executable_link,
            'first_sample_index': item.first_sample,
            'ended_sample_index': item.ended_sample,
        }
        for item in sorted(state.processes.values(), key=lambda value: value.pid)
    ]


def _cpu_totals(state: ProfileState, clock_ticks: int) -> dict[str, Any]:
    threads = [
        {
            'pid': identity[0],
            'process_start_ticks': identity[1],
            'tid': identity[2],
            'thread_start_ticks': identity[3],
            'comm': state.thread_names[identity],
            'cpu_ticks': ticks,
            'cpu_seconds': ticks / clock_ticks,
        }
        for identity, ticks in state.thread_delta_ticks.items()
    ]
    threads.sort(key=lambda item: (-item['cpu_ticks'], item['pid'], item['tid']))
    processes = [
        {
            'pid': identity[0],
            'start_ticks': identity[1],
            'cpu_ticks': ticks,
            'cpu_seconds': ticks / clock_ticks,
        }
        for identity, ticks in state.process_delta_ticks.items()
    ]
    processes.sort(key=lambda item: (-item['cpu_ticks'], item['pid']))
    return {'processes': processes, 'threads': threads}


def _utc(epoch_ns: int) -> str:
    return (
        datetime.datetime.fromtimestamp(epoch_ns / 1_000_000_000, datetime.UTC)
        .isoformat(timespec='microseconds')
        .replace('+00:00', 'Z')
    )


def _profile_mapping(
    value: object,
    label: str,
    keys: frozenset[str] | set[str] | None = None,
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ProfileError('invalid_profile', f'{label} is not an object')
    if keys is not None and set(value) != set(keys):
        raise ProfileError('invalid_profile', f'{label} keys differ from the frozen schema')
    return value


def _profile_list(value: object, label: str, maximum: int) -> list[Any]:
    if not isinstance(value, list) or len(value) > maximum:
        raise ProfileError('invalid_profile', f'{label} is not a bounded array')
    return value


def _profile_int(
    value: object,
    label: str,
    minimum: int = 0,
    maximum: int = INT64_MAX,
) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ProfileError('invalid_profile', f'{label} is outside its integer bounds')
    return value


def _profile_number(
    value: object,
    label: str,
    minimum: float = 0.0,
    maximum: float = float(INT64_MAX),
) -> float:
    if type(value) not in (int, float):
        raise ProfileError('invalid_profile', f'{label} is not a number')
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ProfileError('invalid_profile', f'{label} is outside its numeric bounds')
    return result


def _profile_string(value: object, label: str, maximum: int = 4096) -> str:
    if not isinstance(value, str) or not value or len(value.encode()) > maximum:
        raise ProfileError('invalid_profile', f'{label} is not a bounded string')
    return value


def _profile_sha256(value: object, label: str) -> str:
    text = _profile_string(value, label, 64)
    if SHA256_RE.fullmatch(text) is None:
        raise ProfileError('invalid_profile', f'{label} is not a SHA-256 digest')
    return text


def _profile_utc(value: object, label: str) -> int:
    text = _profile_string(value, label, 27)
    try:
        parsed = datetime.datetime.strptime(text, '%Y-%m-%dT%H:%M:%S.%fZ').replace(
            tzinfo=datetime.UTC
        )
    except ValueError as exc:
        raise ProfileError('invalid_profile', f'{label} is not canonical UTC') from exc
    if parsed.strftime('%Y-%m-%dT%H:%M:%S.%fZ') != text:
        raise ProfileError('invalid_profile', f'{label} is not canonical UTC')
    return int(parsed.timestamp() * 1_000_000_000)


def _recorded_workspace_root(candidate_root: object, candidate_id: str) -> tuple[Path, Path]:
    recorded_candidate = Path(_profile_string(candidate_root, 'candidate root', 16 * 1024))
    tail = Path('artifacts/evidence/phase3-benchmarks') / candidate_id
    if not recorded_candidate.is_absolute() or '..' in recorded_candidate.parts:
        raise ProfileError('invalid_profile', 'recorded candidate root is not canonical')
    if len(recorded_candidate.parts) <= len(tail.parts):
        raise ProfileError('invalid_profile', 'recorded candidate root has the wrong suffix')
    recorded_workspace = recorded_candidate.parents[len(tail.parts) - 1]
    if recorded_workspace / tail != recorded_candidate:
        raise ProfileError('invalid_profile', 'recorded candidate root has the wrong suffix')
    if not recorded_workspace.is_absolute():
        raise ProfileError('invalid_profile', 'recorded workspace root is malformed')
    return recorded_workspace, recorded_candidate


def _validate_recorded_file_identity(
    value: object,
    *,
    label: str,
    recorded_path: Path,
    current_path: Path,
    maximum_bytes: int,
) -> Mapping[str, Any]:
    identity = _profile_mapping(value, label, FILE_IDENTITY_KEYS)
    if identity.get('path') != str(recorded_path):
        raise ProfileError('identity_changed', f'{label} recorded path differs')
    for field_name in ('device', 'inode', 'mtime_ns', 'size_bytes'):
        _profile_int(identity.get(field_name), f'{label} {field_name}', 0, UINT64_MAX)
    recorded_hash = _profile_sha256(identity.get('sha256'), f'{label} SHA-256')
    current = _file_identity(current_path, maximum_bytes, label)
    if recorded_hash != current['sha256'] or identity.get('size_bytes') != current['size_bytes']:
        raise ProfileError('identity_changed', f'{label} bytes differ from recorded evidence')
    return identity


def _validate_profile_candidate(
    profile: Mapping[str, Any],
    *,
    workspace: Path,
    candidate_root: Path,
    candidate_id: str,
) -> dict[str, Any]:
    candidate = _profile_mapping(
        profile.get('candidate'), 'profile candidate', PROFILE_CANDIDATE_KEYS
    )
    if candidate.get('candidate_id') != candidate_id:
        raise ProfileError('identity_changed', 'profile candidate ID differs')
    recorded_workspace, recorded_candidate = _recorded_workspace_root(
        candidate.get('candidate_root'), candidate_id
    )

    binding_path = candidate_root / 'build-binding.json'
    plan_path = candidate_root / 'suite-plan.json'
    prepared_path = candidate_root / 'prepared.json'
    binding, binding_hash = _load_canonical(binding_path, 'candidate build binding')
    plan, plan_hash = _load_canonical(plan_path, 'candidate suite plan')
    prepared, prepared_hash = _load_canonical(prepared_path, 'candidate prepared marker')
    for key, path, digest in (
        ('build_binding', recorded_candidate / 'build-binding.json', binding_hash),
        ('suite_plan', recorded_candidate / 'suite-plan.json', plan_hash),
        ('prepared_marker', recorded_candidate / 'prepared.json', prepared_hash),
    ):
        reference = _profile_mapping(candidate.get(key), f'profile candidate {key}')
        if set(reference) != {'path', 'sha256'}:
            raise ProfileError('invalid_profile', f'profile candidate {key} keys differ')
        if reference.get('path') != str(path) or reference.get('sha256') != digest:
            raise ProfileError('identity_changed', f'profile candidate {key} differs')

    git = _profile_mapping(
        candidate.get('git'),
        'profile candidate Git',
        {'sha', 'status_porcelain'},
    )
    binding_git = _profile_mapping(binding.get('git'), 'build binding Git')
    git_sha = _profile_string(git.get('sha'), 'profile Git SHA', 40)
    if (
        GIT_SHA_RE.fullmatch(git_sha) is None
        or git.get('status_porcelain') != ''
        or binding_git.get('sha') != git_sha
        or binding_git.get('dirty') is not False
        or binding_git.get('status_porcelain') != ''
    ):
        raise ProfileError('identity_changed', 'profile Git identity differs from clean build')

    if set(prepared) != {
        'build_binding_sha256',
        'candidate_id',
        'git_sha',
        'producer',
        'suite_plan_sha256',
    }:
        raise ProfileError('invalid_profile', 'prepared marker keys differ')
    if prepared != {
        'build_binding_sha256': binding_hash,
        'candidate_id': candidate_id,
        'git_sha': git_sha,
        'producer': BENCHMARK_PRODUCER,
        'suite_plan_sha256': plan_hash,
    }:
        raise ProfileError('identity_changed', 'prepared marker does not bind profile inputs')

    smoke = _profile_mapping(
        candidate.get('smoke'),
        'profile smoke identity',
        {'contact_profile_enabled', 'gz_partition', 'ros_domain_id', 'run_id'},
    )
    plan_smoke = _profile_mapping(plan.get('smoke'), 'suite smoke identity')
    if (
        smoke.get('contact_profile_enabled') is not True
        or smoke.get('gz_partition') != plan_smoke.get('gz_partition')
        or smoke.get('ros_domain_id') != plan_smoke.get('ros_domain_id')
        or smoke.get('run_id') != plan_smoke.get('run_id')
        or plan.get('candidate_id') != candidate_id
    ):
        raise ProfileError('identity_changed', 'profile smoke isolation differs from suite plan')
    _profile_int(smoke.get('ros_domain_id'), 'profile smoke ROS domain', 0, 232)
    if smoke.get('run_id') != f'{candidate_id}-smoke-s1-r0':
        raise ProfileError('identity_changed', 'profile smoke run ID is not frozen')

    binary = _profile_mapping(
        binding.get('contact_aggregator_binary'), 'contact aggregator build binding'
    )
    for field_name in (
        'build_embedded_source_inventory_match',
        'build_install_build_id_match',
        'build_install_embedded_source_inventory_match',
        'build_install_sha256_match',
        'installed_embedded_source_inventory_match',
        'installed_regular_file',
    ):
        if binary.get(field_name) is not True:
            raise ProfileError('identity_changed', f'contact build binding lacks {field_name}')
    installed = _profile_string(binary.get('installed_path'), 'installed plugin path', 4096)
    if Path(installed).is_absolute() or '..' in Path(installed).parts:
        raise ProfileError('invalid_profile', 'installed plugin path is not workspace-relative')
    try:
        current_plugin = (workspace / installed).resolve(strict=True)
        plugin_relative = current_plugin.relative_to(workspace)
    except (OSError, ValueError) as exc:
        raise ProfileError(
            'identity_changed', 'installed plugin escapes current workspace'
        ) from exc
    recorded_plugin = recorded_workspace / plugin_relative
    plugin = _profile_mapping(
        candidate.get('plugin'),
        'profile plugin identity',
        FILE_IDENTITY_KEYS | {'build_id', 'declared_path', 'source_inventory_sha256'},
    )
    _validate_recorded_file_identity(
        {key: plugin[key] for key in FILE_IDENTITY_KEYS},
        label='contact aggregator DSO',
        recorded_path=recorded_plugin,
        current_path=current_plugin,
        maximum_bytes=BINARY_MAX_BYTES,
    )
    for profile_key, binding_key in (
        ('sha256', 'installed_sha256'),
        ('build_id', 'installed_elf_build_id'),
        ('declared_path', 'installed_declared_path'),
        ('source_inventory_sha256', 'source_inventory_sha256'),
    ):
        if plugin.get(profile_key) != binary.get(binding_key):
            raise ProfileError('identity_changed', f'profile plugin {profile_key} differs')
    _profile_sha256(plugin.get('source_inventory_sha256'), 'plugin source inventory SHA-256')

    current_producer = workspace / 'tests/phase3_smoke_host_profiler.py'
    recorded_producer = recorded_workspace / 'tests/phase3_smoke_host_profiler.py'
    producer = _validate_recorded_file_identity(
        candidate.get('profiler_producer'),
        label='profiler producer',
        recorded_path=recorded_producer,
        current_path=current_producer,
        maximum_bytes=2 * 1024 * 1024,
    )
    return {
        'binding': binding,
        'binding_sha256': binding_hash,
        'git_sha': git_sha,
        'plan': plan,
        'plan_sha256': plan_hash,
        'plugin_sha256': plugin['sha256'],
        'prepared_sha256': prepared_hash,
        'producer_sha256': producer['sha256'],
        'recorded_candidate': recorded_candidate,
        'recorded_workspace': recorded_workspace,
        'smoke': smoke,
    }


def _profile_limits(config: ProfileConfig | None = None) -> dict[str, int | float]:
    return {
        'startup_timeout_s': (
            CANONICAL_STARTUP_TIMEOUT_S if config is None else config.startup_timeout_s
        ),
        'sample_period_s': CANONICAL_SAMPLE_PERIOD_S if config is None else config.sample_period_s,
        'maximum_duration_s': (
            CANONICAL_MAX_DURATION_S if config is None else config.max_duration_s
        ),
        'maximum_output_bytes': OUTPUT_MAX_BYTES,
        'maximum_samples': MAX_SAMPLES,
        'maximum_process_identities': MAX_PROCESSES,
        'maximum_live_threads_per_sample': MAX_LIVE_THREADS_PER_SAMPLE,
        'maximum_thread_identities': MAX_THREAD_IDENTITIES,
        'maximum_thread_records': MAX_THREAD_RECORDS,
        'maximum_retained_cmdline_bytes': MAX_CMDLINE_TOTAL_BYTES,
    }


def _validate_profile_limits(value: object) -> None:
    limits = _profile_mapping(value, 'profile limits', PROFILE_LIMIT_KEYS)
    expected = _profile_limits()
    if limits != expected:
        raise ProfileError('invalid_profile', 'profile limits differ from canonical bounds')


def _validate_profile_anchor(
    value: object,
    candidate: Mapping[str, Any],
) -> Mapping[str, Any]:
    anchor = _profile_mapping(value, 'profile anchor', PROFILE_ANCHOR_KEYS)
    pid = _profile_int(anchor.get('pid'), 'anchor PID', 1)
    start = _profile_int(anchor.get('start_ticks'), 'anchor start ticks')
    _profile_int(anchor.get('ppid'), 'anchor parent PID')
    _profile_int(anchor.get('process_group'), 'anchor process group', 1)
    _profile_int(anchor.get('session'), 'anchor session', 1)
    _profile_string(anchor.get('comm'), 'anchor command name', 4096)
    _profile_string(anchor.get('executable_link'), 'anchor executable link', 16 * 1024)
    cmdline = _profile_list(anchor.get('cmdline'), 'anchor command line', 4096)
    if any(not isinstance(item, str) or '\x00' in item for item in cmdline):
        raise ProfileError('invalid_profile', 'anchor command line contains invalid entries')
    _profile_sha256(anchor.get('cmdline_sha256'), 'anchor command-line SHA-256')
    _profile_int(
        anchor.get('cmdline_size_bytes'),
        'anchor command-line size',
        0,
        CMDLINE_MAX_BYTES,
    )

    anchor_executable = anchor.get('executable')
    executable = _profile_mapping(anchor_executable, 'anchor executable', FILE_IDENTITY_KEYS)
    for field_name in ('device', 'inode', 'mtime_ns', 'size_bytes'):
        _profile_int(executable.get(field_name), f'anchor executable {field_name}', 0, UINT64_MAX)
    _profile_string(executable.get('path'), 'anchor executable path', 16 * 1024)
    _profile_sha256(executable.get('sha256'), 'anchor executable SHA-256')

    expected_plugin = _profile_mapping(
        anchor.get('expected_plugin'),
        'anchor expected plugin',
        {'device', 'inode', 'path', 'sha256'},
    )
    plugin = _profile_mapping(candidate.get('plugin'), 'profile plugin')
    for field_name in ('device', 'inode'):
        _profile_int(expected_plugin.get(field_name), f'anchor plugin {field_name}', 0, UINT64_MAX)
    if expected_plugin != {key: plugin[key] for key in expected_plugin}:
        raise ProfileError('identity_changed', 'anchor expected plugin differs from candidate')

    mappings = _profile_list(anchor.get('mappings'), 'anchor plugin mappings', 65536)
    mapping_keys = {'address_range', 'offset', 'path', 'permissions'}
    for index, item in enumerate(mappings):
        mapping = _profile_mapping(item, f'anchor mapping {index}', mapping_keys)
        _profile_string(mapping.get('address_range'), f'anchor mapping {index} range')
        _profile_int(mapping.get('offset'), f'anchor mapping {index} offset', 0, UINT64_MAX)
        if mapping.get('path') != plugin.get('path'):
            raise ProfileError('identity_changed', 'anchor mapping path differs from plugin')
        permissions = _profile_string(
            mapping.get('permissions'), f'anchor mapping {index} permissions', 16
        )
        if not set(permissions) <= set('rwxps-'):
            raise ProfileError('invalid_profile', 'anchor mapping permissions are invalid')
    if (
        not mappings
        or not any(mapping['offset'] == 0 for mapping in mappings)
        or not any('x' in mapping['permissions'] for mapping in mappings)
        or anchor.get('mapping_count') != len(mappings)
        or anchor.get('mapping_fingerprint_sha256')
        != hashlib.sha256(canonical_json_bytes(mappings)).hexdigest()
    ):
        raise ProfileError('identity_changed', 'anchor plugin mappings do not reconcile')
    _profile_int(anchor.get('mapping_count'), 'anchor mapping count', 1, 65536)
    if pid != anchor.get('pid') or start != anchor.get('start_ticks'):
        raise ProfileError('invalid_profile', 'anchor process identity is invalid')
    return anchor


def _validate_renderer(
    value: object,
    profile_started_epoch_ns: int,
    profile_finished_epoch_ns: int,
) -> None:
    keys = FILE_IDENTITY_KEYS | {
        'baseline_sha256',
        'content_delta_from_pre_smoke',
        'fresh_after_profile_start',
        'identity_lines',
        'post_start_segment_mode',
        'post_start_segment_offset_bytes',
        'post_start_segment_sha256',
        'post_start_segment_size_bytes',
        'pre_smoke_identity',
    }
    renderer = _profile_mapping(value, 'renderer evidence', keys)
    for field_name in ('device', 'inode', 'mtime_ns', 'size_bytes'):
        _profile_int(renderer.get(field_name), f'renderer {field_name}', 0, UINT64_MAX)
    renderer_path = Path(_profile_string(renderer.get('path'), 'renderer path', 16 * 1024))
    if not renderer_path.is_absolute() or '..' in renderer_path.parts:
        raise ProfileError('invalid_profile', 'renderer path is not canonical')
    _profile_sha256(renderer.get('sha256'), 'renderer SHA-256')
    baseline_hash = renderer.get('baseline_sha256')
    if baseline_hash is not None:
        _profile_sha256(baseline_hash, 'renderer baseline SHA-256')
    if (
        renderer.get('content_delta_from_pre_smoke') is not True
        or renderer.get('fresh_after_profile_start') is not True
        or renderer.get('mtime_ns') < profile_started_epoch_ns
        or renderer.get('mtime_ns') > profile_finished_epoch_ns + 1_000
    ):
        raise ProfileError('identity_changed', 'renderer evidence is not fresh')
    lines = _profile_list(
        renderer.get('identity_lines'), 'renderer identity lines', MAX_RENDERER_LINES
    )
    if not lines or any(
        not isinstance(line, str)
        or not line
        or len(line.encode()) > MAX_LINE_BYTES
        or RENDERER_RE.search(line) is None
        for line in lines
    ):
        raise ProfileError('invalid_profile', 'renderer identity lines are invalid')
    if renderer.get('post_start_segment_mode') not in {
        'appended',
        'created',
        'rewritten_or_rotated',
    }:
        raise ProfileError('invalid_profile', 'renderer segment mode is invalid')
    offset = _profile_int(
        renderer.get('post_start_segment_offset_bytes'),
        'renderer segment offset',
        0,
        RENDERER_MAX_BYTES,
    )
    size = _profile_int(
        renderer.get('post_start_segment_size_bytes'),
        'renderer segment size',
        1,
        RENDERER_MAX_BYTES,
    )
    _profile_sha256(renderer.get('post_start_segment_sha256'), 'renderer segment SHA-256')
    if offset + size > renderer.get('size_bytes'):
        raise ProfileError('invalid_profile', 'renderer segment exceeds recorded file size')
    pre = _profile_mapping(renderer.get('pre_smoke_identity'), 'pre-smoke renderer identity')
    if pre.get('present') is False:
        if set(pre) != {'path', 'present'} or pre.get('path') != str(renderer_path):
            raise ProfileError('invalid_profile', 'absent renderer baseline is malformed')
        if (
            baseline_hash is not None
            or offset != 0
            or renderer.get('post_start_segment_mode') != 'created'
        ):
            raise ProfileError('identity_changed', 'created renderer baseline does not reconcile')
    elif pre.get('present') is True:
        if set(pre) != FILE_IDENTITY_KEYS | {'present'} or pre.get('path') != str(renderer_path):
            raise ProfileError('invalid_profile', 'present renderer baseline is malformed')
        for field_name in ('device', 'inode', 'mtime_ns', 'size_bytes'):
            _profile_int(pre.get(field_name), f'pre-smoke renderer {field_name}', 0, UINT64_MAX)
        if pre.get('sha256') != baseline_hash:
            raise ProfileError('identity_changed', 'renderer baseline hash does not reconcile')
        _profile_sha256(pre.get('sha256'), 'pre-smoke renderer SHA-256')
    else:
        raise ProfileError('invalid_profile', 'renderer baseline presence is not boolean')


def _validate_full_stack_profile(
    value: object,
    *,
    candidate_root: Path,
    recorded_candidate: Path,
    recorded_workspace: Path,
    anchor: Mapping[str, Any],
    profile_started_monotonic_ns: int,
) -> Mapping[str, Any]:
    keys = {
        'finished_steady_ns',
        'group_confirmed_empty',
        'path',
        'pgid',
        'pid',
        'pre_smoke_outputs_absent',
        'returncode',
        'sha256',
        'started_steady_ns',
        'streams',
    }
    evidence = _profile_mapping(value, 'full-stack profile evidence', keys)
    recorded_process = recorded_candidate / 'smoke/processes/full_stack.process.json'
    current_process = candidate_root / 'smoke/processes/full_stack.process.json'
    if evidence.get('path') != str(recorded_process):
        raise ProfileError('identity_changed', 'full-stack metadata path differs')
    for field_name in ('pid', 'pgid', 'started_steady_ns', 'finished_steady_ns'):
        _profile_int(evidence.get(field_name), f'full-stack projection {field_name}', 1)
    if type(evidence.get('returncode')) is not int:
        raise ProfileError('invalid_profile', 'full-stack projected return code is invalid')
    payload = _bounded_regular_read(current_process, 256 * 1024, 'full-stack process metadata')
    try:
        metadata = json.loads(payload.decode())
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise ProfileError('invalid_profile', 'full-stack process metadata is invalid') from exc
    metadata_keys = {
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
    metadata = _profile_mapping(metadata, 'full-stack process metadata', metadata_keys)
    if payload != canonical_json_bytes(metadata):
        raise ProfileError('invalid_profile', 'full-stack process metadata is not canonical')
    started = _profile_int(metadata.get('started_steady_ns'), 'full-stack start time', 1)
    finished = _profile_int(metadata.get('finished_steady_ns'), 'full-stack finish time', started)
    pid = _profile_int(metadata.get('pid'), 'full-stack PID', 1)
    pgid = _profile_int(metadata.get('pgid'), 'full-stack process group', 1)
    returncode = metadata.get('returncode')
    if type(returncode) is not int:
        raise ProfileError('invalid_profile', 'full-stack return code is not an integer')
    command = _profile_list(metadata.get('command'), 'full-stack command', 4096)
    wrapped = _profile_list(metadata.get('wrapped_command'), 'wrapped full-stack command', 4100)
    if any(not isinstance(item, str) or not item for item in (*command, *wrapped)):
        raise ProfileError('invalid_profile', 'full-stack command entries are invalid')
    wall_timeout = _profile_number(
        metadata.get('wall_timeout_s'), 'full-stack wall timeout', 1, 600
    )
    expected_wrapped = [
        'timeout',
        '--signal=TERM',
        f'--kill-after={10:.0f}s',
        f'{wall_timeout:.3f}s',
        *command,
    ]
    if (
        not command
        or wrapped != expected_wrapped
        or metadata.get('cwd') != str(recorded_workspace)
        or metadata.get('role') != 'full_stack'
        or metadata.get('group_confirmed_empty') is not True
        or metadata.get('timed_out') is not False
        or pid != pgid
        or started < profile_started_monotonic_ns
        or anchor.get('session') != pgid
    ):
        raise ProfileError('identity_changed', 'full-stack process identity does not reconcile')
    if (
        evidence.get('sha256') != hashlib.sha256(payload).hexdigest()
        or evidence.get('pid') != pid
        or evidence.get('pgid') != pgid
        or evidence.get('started_steady_ns') != started
        or evidence.get('finished_steady_ns') != finished
        or evidence.get('returncode') != returncode
        or evidence.get('group_confirmed_empty') is not True
        or evidence.get('pre_smoke_outputs_absent') is not True
    ):
        raise ProfileError('identity_changed', 'full-stack recorded projection differs')
    _profile_sha256(evidence.get('sha256'), 'full-stack metadata SHA-256')

    streams = _profile_mapping(
        evidence.get('streams'), 'full-stack stream evidence', {'stderr', 'stdout'}
    )
    stream_metadata_keys = {
        'error',
        'maximum_bytes',
        'observed_bytes',
        'overflow',
        'retained_bytes',
    }
    for stream_name in ('stdout', 'stderr'):
        state = _profile_mapping(
            metadata.get(stream_name), f'full-stack {stream_name} state', stream_metadata_keys
        )
        current_log = candidate_root / f'smoke/processes/full_stack.{stream_name}.log'
        recorded_log = recorded_candidate / f'smoke/processes/full_stack.{stream_name}.log'
        log_label = f'full-stack {stream_name}'
        log_payload = _bounded_regular_read(current_log, LOG_MAX_BYTES, log_label)
        size = len(log_payload)
        if (
            state.get('error') is not None
            or state.get('maximum_bytes') != LOG_MAX_BYTES
            or state.get('observed_bytes') != size
            or state.get('overflow') is not False
            or state.get('retained_bytes') != size
        ):
            raise ProfileError('invalid_profile', f'full-stack {stream_name} was not retained')
        for field_name in ('maximum_bytes', 'observed_bytes', 'retained_bytes'):
            _profile_int(
                state.get(field_name), f'full-stack {stream_name} {field_name}', 0, UINT64_MAX
            )
        projected = _profile_mapping(
            streams.get(stream_name),
            f'full-stack {stream_name} projection',
            {'path', 'sha256', 'size_bytes'},
        )
        _profile_int(
            projected.get('size_bytes'),
            f'full-stack {stream_name} projected size',
            0,
            LOG_MAX_BYTES,
        )
        if projected != {
            'path': str(recorded_log),
            'sha256': hashlib.sha256(log_payload).hexdigest(),
            'size_bytes': size,
        }:
            raise ProfileError('identity_changed', f'full-stack {stream_name} projection differs')
    return evidence


def _relocate_contact_paths(
    contact: Mapping[str, Any],
    recorded_candidate: Path,
    current_candidate: Path,
) -> dict[str, Any]:
    relocated = dict(contact)
    selected = relocated.get('selected_source')
    if selected is not None:
        for stream_name in ('stdout', 'stderr'):
            recorded = recorded_candidate / f'smoke/processes/full_stack.{stream_name}.log'
            if selected == str(recorded):
                relocated['selected_source'] = str(
                    current_candidate / f'smoke/processes/full_stack.{stream_name}.log'
                )
                break
        else:
            raise ProfileError('identity_changed', 'contact selected source path differs')
    sources = _profile_list(relocated.get('sources'), 'contact source list', 2)
    current_sources = []
    for source in sources:
        item = dict(
            _profile_mapping(
                source,
                'contact source',
                {
                    'invalid_record_count',
                    'path',
                    'sha256',
                    'size_bytes',
                    'valid_record_count',
                },
            )
        )
        _profile_sha256(item.get('sha256'), 'contact source SHA-256')
        _profile_int(item.get('size_bytes'), 'contact source size', 0, LOG_MAX_BYTES)
        _profile_int(
            item.get('valid_record_count'),
            'contact source valid record count',
            0,
            MAX_CONTACT_RECORDS,
        )
        _profile_int(
            item.get('invalid_record_count'),
            'contact source invalid record count',
            0,
            MAX_CONTACT_RECORDS,
        )
        matched = False
        for stream_name in ('stdout', 'stderr'):
            recorded = recorded_candidate / f'smoke/processes/full_stack.{stream_name}.log'
            if item.get('path') == str(recorded):
                item['path'] = str(
                    current_candidate / f'smoke/processes/full_stack.{stream_name}.log'
                )
                matched = True
                break
        if not matched:
            raise ProfileError('identity_changed', 'contact source path differs')
        current_sources.append(item)
    relocated['sources'] = current_sources
    relocated.pop('host_thread_bindings', None)
    return relocated


def _validate_contact_profile(
    value: object,
    *,
    candidate_root: Path,
    recorded_candidate: Path,
    full_stack: Mapping[str, Any],
) -> tuple[Mapping[str, Any], Sequence[Mapping[str, Any]]]:
    keys = {
        'host_thread_bindings',
        'last_valid_cumulative_record',
        'present',
        'selected_source',
        'selected_source_valid_record_count',
        'sources',
    }
    contact = _profile_mapping(value, 'contact profile evidence', keys)
    if contact.get('present') is not True:
        raise ProfileError('invalid_profile', 'contact profile is not present')
    count = _profile_int(
        contact.get('selected_source_valid_record_count'),
        'contact profile record count',
        1,
        MAX_CONTACT_RECORDS,
    )
    record = _profile_mapping(
        contact.get('last_valid_cumulative_record'), 'last contact profile record'
    )
    validate_contact_profile_records((record,))
    parsed = parse_contact_profile_logs(candidate_root / 'smoke')
    recorded_without_binding = _relocate_contact_paths(contact, recorded_candidate, candidate_root)
    if parsed != recorded_without_binding or count != parsed['selected_source_valid_record_count']:
        raise ProfileError('identity_changed', 'contact profile differs from closed smoke logs')
    current_streams = {
        name: {
            **dict(_profile_mapping(item, f'full-stack {name} stream')),
            'path': str(candidate_root / f'smoke/processes/full_stack.{name}.log'),
        }
        for name, item in _profile_mapping(
            full_stack.get('streams'), 'full-stack stream evidence'
        ).items()
    }
    reconcile_contact_log_sources(parsed, {'streams': current_streams})
    binding_keys = {
        'anchor_pid',
        'anchor_start_ticks',
        'comm',
        'first_sample_index',
        'last_sample_index',
        'linux_tid',
        'sample_count',
        'sampled_cpu_delta_ticks',
        'thread_start_ticks',
    }
    bindings = _profile_list(
        contact.get('host_thread_bindings'),
        'contact host-thread bindings',
        MAX_CONTACT_THREAD_CONTRIBUTIONS,
    )
    if not bindings:
        raise ProfileError('invalid_profile', 'contact host-thread bindings are empty')
    parsed_bindings = [
        _profile_mapping(item, f'contact host-thread binding {index}', binding_keys)
        for index, item in enumerate(bindings)
    ]
    return contact, parsed_bindings


def _validate_host_sample(value: object, label: str) -> None:
    host = _profile_mapping(value, label, {'loadavg', 'pressure', 'proc_stat', 'vmstat'})
    loadavg = _profile_mapping(
        host.get('loadavg'),
        f'{label} loadavg',
        {
            'last_pid',
            'load_15m',
            'load_1m',
            'load_5m',
            'runnable_entities',
            'total_entities',
        },
    )
    for field_name in ('load_1m', 'load_5m', 'load_15m'):
        _profile_number(loadavg.get(field_name), f'{label} {field_name}')
    runnable = _profile_int(loadavg.get('runnable_entities'), f'{label} runnable entities')
    total = _profile_int(loadavg.get('total_entities'), f'{label} total entities', 1)
    _profile_int(loadavg.get('last_pid'), f'{label} last PID', 1)
    if runnable > total:
        raise ProfileError('invalid_profile', f'{label} run queue exceeds entity count')
    for group_name, required in (('proc_stat', PROC_STAT_KEYS), ('vmstat', VMSTAT_KEYS)):
        group = _profile_mapping(host.get(group_name), f'{label} {group_name}', set(required))
        for field_name in required:
            _profile_int(
                group.get(field_name),
                f'{label} {group_name} {field_name}',
                0,
                UINT64_MAX,
            )
    pressure = _profile_mapping(host.get('pressure'), f'{label} PSI', {'cpu', 'io', 'memory'})
    for resource in ('cpu', 'io', 'memory'):
        rows = _profile_mapping(pressure.get(resource), f'{label} {resource} PSI')
        expected_rows = {'some'} if resource == 'cpu' else {'full', 'some'}
        if set(rows) != expected_rows:
            raise ProfileError('invalid_profile', f'{label} {resource} PSI rows differ')
        for row_name, row_value in rows.items():
            row = _profile_mapping(
                row_value,
                f'{label} {resource} {row_name} PSI',
                {'avg10', 'avg60', 'avg300', 'total'},
            )
            for field_name in ('avg10', 'avg60', 'avg300'):
                _profile_number(row.get(field_name), f'{label} PSI {field_name}')
            _profile_int(row.get('total'), f'{label} PSI total', 0, UINT64_MAX)


def _validate_sampling(
    value: object,
    *,
    anchor: Mapping[str, Any],
    clock_ticks: int,
    profile_started_monotonic_ns: int,
) -> dict[str, Any]:
    sampling = _profile_mapping(value, 'profile sampling', PROFILE_SAMPLING_KEYS)
    samples = _profile_list(sampling.get('samples'), 'profile samples', MAX_SAMPLES)
    sample_count = _profile_int(
        sampling.get('sample_count'), 'sample count', MINIMUM_SAMPLES, MAX_SAMPLES
    )
    if sample_count != len(samples):
        raise ProfileError('invalid_profile', 'sample count does not match samples')
    anchor_alive_count = _profile_int(
        sampling.get('anchor_alive_sample_count'), 'anchor-alive sample count', 2, sample_count
    )
    target_alive_count = _profile_int(
        sampling.get('target_alive_sample_count'), 'target-alive sample count', 1, sample_count - 1
    )
    _profile_int(
        sampling.get('cadence_overrun_count'), 'cadence overrun count', 0, 2 * sample_count
    )
    _profile_int(
        sampling.get('exact_processes_seen_before_anchor'),
        'pre-anchor exact process count',
        1,
        MAX_PROCESSES,
    )

    thread_last_ticks: dict[tuple[int, int, int, int], int] = {}
    thread_names: dict[tuple[int, int, int, int], str] = {}
    thread_delta_totals: dict[tuple[int, int, int, int], int] = {}
    process_delta_totals: dict[tuple[int, int], int] = {}
    process_samples: dict[tuple[int, int], list[int]] = {}
    thread_samples: dict[tuple[int, int, int, int], list[int]] = {}
    thread_records = 0
    previous_monotonic: int | None = None
    first_monotonic: int | None = None
    previous_wall: int | None = None
    first_wall: int | None = None
    observed_anchor_alive = 0
    observed_target_alive = 0
    anchor_has_ended = False
    sample_keys = {
        'anchor_alive',
        'host',
        'index',
        'monotonic_ns',
        'process_count',
        'processes',
        'processes_vanished_during_sample',
        'target_process_set_alive',
        'wall_epoch_ns',
    }
    process_keys = {
        'comm',
        'cpu_ticks',
        'pid',
        'start_ticks',
        'thread_count',
        'threads',
        'threads_vanished_during_sample',
    }
    thread_keys = {
        'comm',
        'cpu_delta_ticks',
        'cpu_ticks',
        'processor',
        'start_ticks',
        'state',
        'stime_ticks',
        'tid',
        'utime_ticks',
    }
    for index, sample_value in enumerate(samples):
        sample = _profile_mapping(sample_value, f'profile sample {index}', sample_keys)
        _profile_int(sample.get('index'), f'profile sample {index} index', 0, MAX_SAMPLES - 1)
        if sample.get('index') != index:
            raise ProfileError('invalid_profile', 'profile sample indexes are not contiguous')
        monotonic_ns = _profile_int(sample.get('monotonic_ns'), 'sample monotonic time')
        wall_ns = _profile_int(sample.get('wall_epoch_ns'), 'sample wall time')
        if first_monotonic is None:
            first_monotonic = monotonic_ns
        if first_wall is None:
            first_wall = wall_ns
        if (
            monotonic_ns < profile_started_monotonic_ns
            or first_monotonic
            > profile_started_monotonic_ns + int(CANONICAL_STARTUP_TIMEOUT_S * 1_000_000_000)
            or monotonic_ns > first_monotonic + int(CANONICAL_MAX_DURATION_S * 1_000_000_000)
            or (previous_monotonic is not None and monotonic_ns <= previous_monotonic)
            or (
                previous_monotonic is not None
                and monotonic_ns - previous_monotonic
                < int(CANONICAL_SAMPLE_PERIOD_S * 1_000_000_000)
            )
            or (previous_wall is not None and wall_ns <= previous_wall)
        ):
            raise ProfileError('invalid_profile', 'sample clocks are not strictly increasing')
        previous_monotonic = monotonic_ns
        previous_wall = wall_ns
        if (
            type(sample.get('anchor_alive')) is not bool
            or type(sample.get('target_process_set_alive')) is not bool
        ):
            raise ProfileError('invalid_profile', 'sample lifecycle flags are not booleans')
        observed_anchor_alive += int(sample['anchor_alive'])
        observed_target_alive += int(sample['target_process_set_alive'])
        if sample['anchor_alive'] and not sample['target_process_set_alive']:
            raise ProfileError('invalid_profile', 'live anchor has an empty target set')
        if anchor_has_ended and sample['anchor_alive']:
            raise ProfileError('invalid_profile', 'anchor-alive history is not monotonic')
        anchor_has_ended |= not sample['anchor_alive']
        _validate_host_sample(sample.get('host'), f'profile sample {index} host')
        processes = _profile_list(
            sample.get('processes'), f'profile sample {index} processes', MAX_PROCESSES
        )
        sample_thread_records = 0
        _profile_int(
            sample.get('process_count'),
            f'profile sample {index} process count',
            0,
            MAX_PROCESSES,
        )
        if sample.get('process_count') != len(processes):
            raise ProfileError('invalid_profile', 'sample process count does not reconcile')
        _profile_int(
            sample.get('processes_vanished_during_sample'),
            'sample vanished process count',
            0,
            MAX_PROCESSES,
        )
        process_order: list[int] = []
        anchor_present = False
        for process_index, process_value in enumerate(processes):
            process = _profile_mapping(
                process_value,
                f'profile sample {index} process {process_index}',
                process_keys,
            )
            pid = _profile_int(process.get('pid'), 'sample process PID', 1)
            start = _profile_int(process.get('start_ticks'), 'sample process start ticks')
            process_order.append(pid)
            identity = (pid, start)
            process_samples.setdefault(identity, []).append(index)
            anchor_present |= identity == (anchor['pid'], anchor['start_ticks'])
            _profile_string(process.get('comm'), 'sample process command name')
            _profile_int(process.get('cpu_ticks'), 'sample process CPU ticks', 0, UINT64_MAX)
            threads = _profile_list(
                process.get('threads'), 'sample process threads', MAX_THREADS_PER_PROCESS
            )
            _profile_int(
                process.get('thread_count'),
                'sample process thread count',
                0,
                MAX_THREADS_PER_PROCESS,
            )
            if process.get('thread_count') != len(threads):
                raise ProfileError('invalid_profile', 'sample thread count does not reconcile')
            sample_thread_records += len(threads)
            if sample_thread_records > MAX_LIVE_THREADS_PER_SAMPLE:
                raise ProfileError('invalid_profile', 'sample exceeds the live-thread bound')
            _profile_int(
                process.get('threads_vanished_during_sample'),
                'sample vanished thread count',
                0,
                MAX_THREADS_PER_PROCESS,
            )
            thread_order: list[int] = []
            for thread_index, thread_value in enumerate(threads):
                thread = _profile_mapping(
                    thread_value,
                    f'profile sample {index} thread {thread_index}',
                    thread_keys,
                )
                tid = _profile_int(thread.get('tid'), 'sample thread TID', 1)
                thread_start = _profile_int(thread.get('start_ticks'), 'sample thread start ticks')
                thread_order.append(tid)
                thread_identity = (pid, start, tid, thread_start)
                comm = _profile_string(thread.get('comm'), 'sample thread command name')
                state = _profile_string(thread.get('state'), 'sample thread state', 1)
                if len(state) != 1:
                    raise ProfileError('invalid_profile', 'sample thread state is malformed')
                _profile_int(thread.get('processor'), 'sample thread processor', 0, 1_048_575)
                utime = _profile_int(
                    thread.get('utime_ticks'), 'sample thread user ticks', 0, UINT64_MAX
                )
                stime = _profile_int(
                    thread.get('stime_ticks'), 'sample thread system ticks', 0, UINT64_MAX
                )
                cpu = _profile_int(
                    thread.get('cpu_ticks'), 'sample thread CPU ticks', 0, UINT64_MAX
                )
                if cpu != utime + stime:
                    raise ProfileError(
                        'invalid_profile', 'sample thread CPU ticks do not reconcile'
                    )
                prior = thread_last_ticks.get(thread_identity)
                expected_delta = None if prior is None else cpu - prior
                if prior is not None and cpu < prior:
                    raise ProfileError('invalid_profile', 'sample thread CPU ticks regressed')
                delta = thread.get('cpu_delta_ticks')
                if delta != expected_delta or (delta is not None and type(delta) is not int):
                    raise ProfileError('invalid_profile', 'sample thread CPU delta differs')
                thread_last_ticks[thread_identity] = cpu
                thread_names.setdefault(thread_identity, comm)
                if thread_names[thread_identity] != comm:
                    raise ProfileError('identity_changed', 'sample thread name changed')
                thread_samples.setdefault(thread_identity, []).append(index)
                if delta is not None:
                    thread_delta_totals[thread_identity] = (
                        thread_delta_totals.get(thread_identity, 0) + delta
                    )
                    process_delta_totals[identity] = process_delta_totals.get(identity, 0) + delta
            if thread_order != sorted(thread_order) or len(thread_order) != len(set(thread_order)):
                raise ProfileError('invalid_profile', 'sample threads are not uniquely sorted')
            thread_records += len(threads)
        if process_order != sorted(process_order) or len(process_order) != len(set(process_order)):
            raise ProfileError('invalid_profile', 'sample processes are not uniquely sorted')
        if sample['anchor_alive'] != anchor_present:
            raise ProfileError('identity_changed', 'anchor-alive flag differs from sampled anchor')
        expected_target_alive = index < sample_count - 1
        if (
            sample['target_process_set_alive'] != bool(processes)
            or sample['target_process_set_alive'] != expected_target_alive
        ):
            raise ProfileError('invalid_profile', 'target lifecycle flags do not reconcile')
    if (
        samples[0].get('anchor_alive') is not True
        or samples[-1].get('target_process_set_alive') is not False
        or observed_anchor_alive != anchor_alive_count
        or observed_target_alive != target_alive_count
        or sampling.get('thread_record_count') != thread_records
        or thread_records > MAX_THREAD_RECORDS
    ):
        raise ProfileError('invalid_profile', 'sampling summary does not reconcile')
    _profile_int(sampling.get('thread_record_count'), 'thread record count', 1, MAX_THREAD_RECORDS)

    expected_threads = [
        {
            'pid': identity[0],
            'process_start_ticks': identity[1],
            'tid': identity[2],
            'thread_start_ticks': identity[3],
            'comm': thread_names[identity],
            'cpu_ticks': ticks,
            'cpu_seconds': ticks / clock_ticks,
        }
        for identity, ticks in thread_delta_totals.items()
    ]
    expected_threads.sort(key=lambda item: (-item['cpu_ticks'], item['pid'], item['tid']))
    expected_processes = [
        {
            'pid': identity[0],
            'start_ticks': identity[1],
            'cpu_ticks': ticks,
            'cpu_seconds': ticks / clock_ticks,
        }
        for identity, ticks in process_delta_totals.items()
    ]
    expected_processes.sort(key=lambda item: (-item['cpu_ticks'], item['pid']))
    totals = _profile_mapping(sampling.get('totals'), 'sampling totals', {'processes', 'threads'})
    total_processes = _profile_list(
        totals.get('processes'), 'sampling total processes', MAX_PROCESSES
    )
    total_threads = _profile_list(
        totals.get('threads'), 'sampling total threads', MAX_THREAD_IDENTITIES
    )
    for index, item in enumerate(total_processes):
        record = _profile_mapping(
            item,
            f'sampling total process {index}',
            {'cpu_seconds', 'cpu_ticks', 'pid', 'start_ticks'},
        )
        _profile_int(record.get('pid'), 'total process PID', 1)
        _profile_int(record.get('start_ticks'), 'total process start ticks')
        _profile_int(record.get('cpu_ticks'), 'total process CPU ticks', 0, UINT64_MAX)
        _profile_number(record.get('cpu_seconds'), 'total process CPU seconds')
    for index, item in enumerate(total_threads):
        record = _profile_mapping(
            item,
            f'sampling total thread {index}',
            {
                'comm',
                'cpu_seconds',
                'cpu_ticks',
                'pid',
                'process_start_ticks',
                'thread_start_ticks',
                'tid',
            },
        )
        _profile_string(record.get('comm'), 'total thread command name')
        for field_name in ('pid', 'tid'):
            _profile_int(record.get(field_name), f'total thread {field_name}', 1)
        for field_name in ('process_start_ticks', 'thread_start_ticks', 'cpu_ticks'):
            _profile_int(record.get(field_name), f'total thread {field_name}', 0, UINT64_MAX)
        _profile_number(record.get('cpu_seconds'), 'total thread CPU seconds')
    if totals != {'processes': expected_processes, 'threads': expected_threads}:
        raise ProfileError('invalid_profile', 'sampling CPU totals do not reconcile')
    return {
        'process_samples': process_samples,
        'thread_delta_totals': thread_delta_totals,
        'thread_names': thread_names,
        'thread_samples': thread_samples,
        'last_sample_index': sample_count - 1,
        'sample_monotonic_ns': [sample['monotonic_ns'] for sample in samples],
        'first_sample_monotonic_ns': first_monotonic,
        'first_sample_wall_epoch_ns': first_wall,
        'last_sample_monotonic_ns': previous_monotonic,
        'last_sample_wall_epoch_ns': previous_wall,
    }


def _validate_process_lifecycles(
    value: object,
    *,
    sampling: Mapping[str, Any],
    anchor: Mapping[str, Any],
    retained_cmdline_bytes: object,
) -> None:
    lifecycles = _profile_list(value, 'process lifecycles', MAX_PROCESSES)
    if not lifecycles:
        raise ProfileError('invalid_profile', 'process lifecycle list is empty')
    keys = {
        'cmdline',
        'cmdline_sha256',
        'cmdline_size_bytes',
        'comm',
        'ended_sample_index',
        'executable_link',
        'first_sample_index',
        'pid',
        'start_ticks',
    }
    process_samples = sampling['process_samples']
    last_sample = sampling['last_sample_index']
    observed: set[tuple[int, int]] = set()
    total_cmdline = 0
    order: list[int] = []
    anchor_lifecycle: Mapping[str, Any] | None = None
    for index, lifecycle_value in enumerate(lifecycles):
        lifecycle = _profile_mapping(lifecycle_value, f'process lifecycle {index}', keys)
        pid = _profile_int(lifecycle.get('pid'), 'lifecycle PID', 1)
        start = _profile_int(lifecycle.get('start_ticks'), 'lifecycle start ticks')
        identity = (pid, start)
        if identity in observed:
            raise ProfileError('invalid_profile', 'process lifecycle identity is duplicated')
        observed.add(identity)
        order.append(pid)
        _profile_string(lifecycle.get('comm'), 'lifecycle command name')
        _profile_string(lifecycle.get('executable_link'), 'lifecycle executable link', 16 * 1024)
        first = _profile_int(
            lifecycle.get('first_sample_index'), 'lifecycle first sample', 0, last_sample
        )
        ended = _profile_int(
            lifecycle.get('ended_sample_index'), 'lifecycle end sample', first, last_sample
        )
        cmdline = _profile_list(lifecycle.get('cmdline'), 'lifecycle command line', 4096)
        if any(not isinstance(item, str) or '\x00' in item for item in cmdline):
            raise ProfileError('invalid_profile', 'lifecycle command line is invalid')
        encoded = b'\0'.join(item.encode() for item in cmdline)
        if cmdline:
            encoded += b'\0'
        size = _profile_int(
            lifecycle.get('cmdline_size_bytes'),
            'lifecycle command-line size',
            0,
            CMDLINE_MAX_BYTES,
        )
        if len(encoded) != size or hashlib.sha256(encoded).hexdigest() != lifecycle.get(
            'cmdline_sha256'
        ):
            raise ProfileError('invalid_profile', 'lifecycle command-line hash differs')
        _profile_sha256(lifecycle.get('cmdline_sha256'), 'lifecycle command-line SHA-256')
        total_cmdline += size
        appearances = process_samples.get(identity, [])
        expected_end = appearances[-1] + 1 if appearances else first
        if (
            (appearances and appearances[0] != first)
            or ended != expected_end
            or ended > last_sample
        ):
            raise ProfileError('identity_changed', 'process lifecycle does not match samples')
        if identity == (anchor['pid'], anchor['start_ticks']):
            anchor_lifecycle = lifecycle
    if order != sorted(order) or len(order) != len(set(order)):
        raise ProfileError('invalid_profile', 'process lifecycles are not uniquely sorted')
    if set(process_samples) - observed:
        raise ProfileError('identity_changed', 'sampled process lacks a lifecycle')
    if (
        anchor_lifecycle is None
        or anchor_lifecycle.get('cmdline') != anchor.get('cmdline')
        or anchor_lifecycle.get('cmdline_sha256') != anchor.get('cmdline_sha256')
        or anchor_lifecycle.get('cmdline_size_bytes') != anchor.get('cmdline_size_bytes')
        or anchor_lifecycle.get('comm') != anchor.get('comm')
        or anchor_lifecycle.get('executable_link') != anchor.get('executable_link')
    ):
        raise ProfileError('identity_changed', 'anchor lifecycle identity differs')
    retained = _profile_int(
        retained_cmdline_bytes,
        'retained command-line bytes',
        0,
        MAX_CMDLINE_TOTAL_BYTES,
    )
    if total_cmdline != retained:
        raise ProfileError('invalid_profile', 'retained command-line bytes do not reconcile')


def _validate_contact_thread_bindings(
    bindings: Sequence[Mapping[str, Any]],
    contact: Mapping[str, Any],
    anchor: Mapping[str, Any],
    sampling: Mapping[str, Any],
) -> None:
    record = _profile_mapping(
        contact.get('last_valid_cumulative_record'), 'last contact profile record'
    )
    contributions = _profile_list(
        record.get('thread_contributions'),
        'contact profile thread contributions',
        MAX_CONTACT_THREAD_CONTRIBUTIONS,
    )
    linux_tids = [
        _profile_int(
            _profile_mapping(item, f'contact thread contribution {index}').get('linux_tid'),
            'contact profile Linux TID',
            1,
        )
        for index, item in enumerate(contributions)
    ]
    if len(bindings) != len(linux_tids):
        raise ProfileError('identity_changed', 'contact host-thread binding count differs')
    anchor_id = (anchor['pid'], anchor['start_ticks'])
    expected_bindings = []
    for linux_tid in linux_tids:
        candidates = [
            identity
            for identity in sampling['thread_samples']
            if identity[:2] == anchor_id and identity[2] == linux_tid
        ]
        if len(candidates) != 1:
            raise ProfileError('identity_changed', 'contact thread does not bind uniquely')
        identity = candidates[0]
        indexes = sampling['thread_samples'][identity]
        expected_bindings.append(
            {
                'anchor_pid': anchor['pid'],
                'anchor_start_ticks': anchor['start_ticks'],
                'linux_tid': linux_tid,
                'thread_start_ticks': identity[3],
                'comm': sampling['thread_names'][identity],
                'first_sample_index': min(indexes),
                'last_sample_index': max(indexes),
                'sample_count': len(indexes),
                'sampled_cpu_delta_ticks': sampling['thread_delta_totals'].get(identity, 0),
            }
        )
    if list(bindings) != expected_bindings:
        raise ProfileError('identity_changed', 'contact host-thread bindings do not reconcile')


def _metrics_result_caps(workspace: Path) -> dict[str, int]:
    constants_path = workspace / 'src/robotest_metrics/robotest_metrics/constants.py'
    _require_real_directory_tree(
        workspace, Path('src/robotest_metrics/robotest_metrics'), 'metrics source tree'
    )
    payload = _bounded_regular_read(constants_path, 1024 * 1024, 'metrics constants')
    try:
        tree = ast.parse(payload.decode(), filename=str(constants_path))
    except (UnicodeError, SyntaxError) as exc:
        raise ProfileError('missing_identity', 'metrics constants source is invalid') from exc
    required = {
        'LOG_MAX_BYTES',
        'PER_RUN_CSV_MAX_BYTES',
        'PER_RUN_DIRECTORY_MAX_BYTES',
        'PER_RUN_JSON_MAX_BYTES',
        'PNG_MAX_BYTES',
        'PNG_MAX_COUNT',
    }
    values: dict[str, int] = {}
    for statement in tree.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if not isinstance(target, ast.Name) or target.id not in required:
            continue
        try:
            value = ast.literal_eval(statement.value)
        except (ValueError, TypeError) as exc:
            raise ProfileError('missing_identity', 'metrics cap is not a literal') from exc
        values[target.id] = _profile_int(value, f'metrics cap {target.id}', 1, UINT64_MAX)
    if set(values) != required:
        raise ProfileError('missing_identity', 'metrics constants lack result bundle caps')
    return values


def _csv_scalar(value: Any) -> str:
    if value is None:
        return ''
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, (int, float)):
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    if isinstance(value, str):
        return value
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    )


def _csv_sequence_descriptor(value: list[Any] | tuple[Any, ...]) -> str:
    descriptor = {
        'element_count': len(value),
        'kind': 'sequence',
        'sha256': hashlib.sha256(canonical_json_bytes(value)).hexdigest(),
    }
    return _csv_scalar(descriptor)


def _one_row_csv_bytes(document: Mapping[str, Any]) -> bytes:
    if not document:
        raise ProfileError('invalid_profile', 'CSV projection root is empty')
    reserved = {
        CSV_PROJECTION_CONTRACT_COLUMN,
        CSV_PROJECTION_JSON_SHA256_COLUMN,
    }
    if any(key in document for key in reserved):
        raise ProfileError('invalid_profile', 'CSV projection root uses a reserved column')
    flattened: dict[str, str] = {
        CSV_PROJECTION_CONTRACT_COLUMN: CSV_PROJECTION_CONTRACT,
        CSV_PROJECTION_JSON_SHA256_COLUMN: hashlib.sha256(
            canonical_json_bytes(document)
        ).hexdigest(),
    }

    def visit(prefix: str, value: Any) -> None:
        if isinstance(value, Mapping) and value:
            for key in sorted(value):
                if not isinstance(key, str) or not key:
                    raise ProfileError('invalid_profile', 'CSV projection key is invalid')
                visit(f'{prefix}.{key}' if prefix else key, value[key])
            return
        if not prefix:
            raise ProfileError('invalid_profile', 'CSV projection root is empty')
        if prefix in flattened:
            raise ProfileError('invalid_profile', f'CSV projection column collision at {prefix!r}')
        flattened[prefix] = (
            _csv_sequence_descriptor(value)
            if isinstance(value, (list, tuple))
            else _csv_scalar(value)
        )

    visit('', document)
    output = io.StringIO(newline='')
    writer = csv.DictWriter(output, fieldnames=sorted(flattened), lineterminator='\n')
    writer.writeheader()
    writer.writerow(flattened)
    return output.getvalue().encode()


def _validate_clone_local_run_result_schema(
    workspace: Path,
    result: Mapping[str, Any],
) -> None:
    _require_real_directory_tree(
        workspace, Path('src/robotest_metrics/schema'), 'metrics schema tree'
    )
    schema_path = workspace / 'src/robotest_metrics/schema/run-result.schema.json'
    payload = _bounded_regular_read(schema_path, 1024 * 1024, 'run-result schema')
    try:
        schema = json.loads(payload.decode())
        Draft202012Validator.check_schema(schema)
        errors = sorted(
            Draft202012Validator(schema).iter_errors(result),
            key=lambda error: tuple(str(item) for item in error.absolute_path),
        )
    except (
        UnicodeError,
        json.JSONDecodeError,
        SchemaError,
        TypeError,
        ValueError,
    ) as exc:
        raise ProfileError('missing_identity', 'run-result schema is invalid') from exc
    if errors:
        path = '.'.join(str(item) for item in errors[0].absolute_path) or '<root>'
        raise ProfileError('invalid_profile', f'run-result schema failed at {path}')


def _validate_smoke_result_bundle(
    workspace: Path,
    result_dir: Path,
) -> tuple[Mapping[str, Any], str, str]:
    caps = _metrics_result_caps(workspace)
    manifest_path = result_dir / 'run-artifacts.manifest.json'
    manifest, manifest_hash = _load_canonical(
        manifest_path,
        'smoke result manifest',
        caps['PER_RUN_JSON_MAX_BYTES'],
    )
    manifest = _profile_mapping(
        manifest,
        'smoke result manifest',
        {'artifacts', 'identity', 'producer', 'quality', 'schema_version'},
    )
    if (
        type(manifest.get('schema_version')) is not int
        or manifest.get('schema_version') != 1
        or manifest.get('producer') != 'robotest_metrics/metrics_analyze'
    ):
        raise ProfileError('invalid_profile', 'smoke result manifest identity differs')
    allowed = {
        'localization-error.png',
        'measurement-summary.png',
        'real-time-factor.png',
        'report.html',
        'report.md',
        'run-result.csv',
        'run-result.json',
        'trajectory.png',
    }
    required = {'report.html', 'report.md', 'run-result.csv', 'run-result.json'}
    try:
        entries = list(result_dir.iterdir())
    except OSError as exc:
        raise ProfileError('missing_identity', 'cannot inspect smoke result bundle') from exc
    if len(entries) > len(allowed) + 2:
        raise ProfileError('invalid_profile', 'smoke result bundle has too many entries')
    actual: dict[str, Path] = {}
    for entry in entries:
        if entry.is_symlink() or not entry.is_file():
            raise ProfileError('missing_identity', 'smoke result bundle entry is not regular')
        actual[entry.name] = entry
    expected_names = set(actual) - {
        'run-artifacts.manifest.json',
        'run-artifacts.manifest.json.sha256',
    }
    if not expected_names >= required or not expected_names <= allowed:
        raise ProfileError('invalid_profile', 'smoke result bundle path set differs')
    records = _profile_list(
        manifest.get('artifacts'), 'smoke result artifact records', len(allowed)
    )
    for index, item in enumerate(records):
        record = _profile_mapping(
            item,
            f'smoke result artifact record {index}',
            {'bytes', 'path', 'sha256'},
        )
        _profile_int(
            record.get('bytes'),
            'smoke result artifact byte count',
            1,
            UINT64_MAX,
        )
        _profile_string(record.get('path'), 'smoke result artifact path', 64)
        _profile_sha256(record.get('sha256'), 'smoke result artifact SHA-256')
    expected_records = []
    total_bytes = 0
    for name in sorted(expected_names):
        path = actual[name]
        if name == 'run-result.json':
            maximum = caps['PER_RUN_JSON_MAX_BYTES']
        elif name == 'run-result.csv':
            maximum = caps['PER_RUN_CSV_MAX_BYTES']
        elif name in {'report.html', 'report.md'}:
            maximum = caps['LOG_MAX_BYTES']
        else:
            maximum = caps['PNG_MAX_BYTES']
        payload = _bounded_regular_read(path, maximum, f'smoke result artifact {name}')
        if not payload:
            raise ProfileError('invalid_profile', f'smoke result artifact {name} is empty')
        total_bytes += len(payload)
        expected_records.append(
            {
                'bytes': len(payload),
                'path': name,
                'sha256': hashlib.sha256(payload).hexdigest(),
            }
        )
    if records != expected_records:
        raise ProfileError('identity_changed', 'smoke result artifact records differ')
    result_path = result_dir / 'run-result.json'
    result, result_hash = _load_canonical_without_sidecar(
        result_path, 'smoke run result', caps['PER_RUN_JSON_MAX_BYTES']
    )
    _validate_clone_local_run_result_schema(workspace, result)
    expected_csv = _one_row_csv_bytes(result)
    actual_csv = _bounded_regular_read(
        result_dir / 'run-result.csv',
        caps['PER_RUN_CSV_MAX_BYTES'],
        'smoke result CSV',
    )
    if actual_csv != expected_csv:
        raise ProfileError('identity_changed', 'smoke result CSV projection differs')
    identity = _profile_mapping(
        manifest.get('identity'),
        'smoke result manifest identity',
        {'run_id', 'run_result_sha256'},
    )
    result_identity = _profile_mapping(result.get('identity'), 'smoke result identity')
    if (
        identity.get('run_id') != result_identity.get('run_id')
        or identity.get('run_result_sha256') != result_hash
    ):
        raise ProfileError('identity_changed', 'smoke result manifest join differs')
    quality = _profile_mapping(
        manifest.get('quality'),
        'smoke result manifest quality',
        {
            'artifact_bytes_excluding_manifest',
            'artifact_count',
            'caps_within_limits',
            'hashes_verified',
            'path_set_complete',
        },
    )
    _profile_int(
        quality.get('artifact_bytes_excluding_manifest'),
        'smoke result artifact byte total',
        1,
        caps['PER_RUN_DIRECTORY_MAX_BYTES'],
    )
    _profile_int(
        quality.get('artifact_count'), 'smoke result artifact count', len(required), len(allowed)
    )
    if quality != {
        'artifact_bytes_excluding_manifest': total_bytes,
        'artifact_count': len(expected_records),
        'caps_within_limits': True,
        'hashes_verified': True,
        'path_set_complete': True,
    }:
        raise ProfileError('invalid_profile', 'smoke result manifest quality differs')
    directory_bytes = sum(path.stat().st_size for path in actual.values())
    if directory_bytes > caps['PER_RUN_DIRECTORY_MAX_BYTES']:
        raise ProfileError('overflow', 'smoke result bundle exceeds its directory cap')
    if len(expected_names - required) > caps['PNG_MAX_COUNT']:
        raise ProfileError('overflow', 'smoke result bundle exceeds its chart cap')
    return result, result_hash, manifest_hash


def _validate_smoke_success(
    *,
    workspace: Path,
    candidate_root: Path,
    candidate_id: str,
    candidate: Mapping[str, Any],
) -> dict[str, str]:
    marker_path = candidate_root / 'smoke/PASS.json'
    marker, marker_hash = _load_canonical(marker_path, 'smoke PASS marker')
    result, result_hash, manifest_hash = _validate_smoke_result_bundle(
        workspace, candidate_root / 'smoke/result'
    )
    if set(marker) != {'producer', 'run_result_sha256', 'status'} or marker != {
        'producer': BENCHMARK_PRODUCER,
        'run_result_sha256': result_hash,
        'status': 'PASS',
    }:
        raise ProfileError('invalid_profile', 'smoke PASS marker does not bind the result')
    verdict = _profile_mapping(result.get('verdict'), 'smoke result verdict')
    quality = _profile_mapping(result.get('quality'), 'smoke result quality')
    _profile_int(verdict.get('exit_code'), 'smoke result exit code', 0, INT64_MAX)
    if (
        verdict.get('automated_status') != 'PASS'
        or verdict.get('exit_code') != 0
        or quality.get('infrastructure_failure') is not None
    ):
        raise ProfileError('invalid_profile', 'smoke run result is not a complete PASS')
    identity = _profile_mapping(
        result.get('identity'),
        'smoke result identity',
        {
            'candidate_id',
            'cold_stack',
            'git_dirty',
            'git_sha',
            'gz_partition',
            'repetition_index',
            'ros_domain_id',
            'run_id',
            'scenario_id',
            'scenario_index',
            'scenario_name',
            'scenario_sha256',
            'suite_index',
        },
    )
    for field_name, minimum, maximum in (
        ('repetition_index', 0, 2),
        ('ros_domain_id', 0, 232),
        ('scenario_id', 1, 5),
        ('scenario_index', 1, 5),
        ('suite_index', 0, 14),
    ):
        _profile_int(identity.get(field_name), f'smoke result {field_name}', minimum, maximum)
    if type(identity.get('cold_stack')) is not bool or type(identity.get('git_dirty')) is not bool:
        raise ProfileError('invalid_profile', 'smoke result boolean identity is malformed')
    trials = _profile_list(candidate['plan'].get('trials'), 'suite trials', 15)
    if not trials:
        raise ProfileError('invalid_profile', 'suite plan has no smoke source trial')
    trial = _profile_mapping(trials[0], 'first suite trial')
    expected = {
        'candidate_id': f'{candidate_id}-smoke',
        'cold_stack': True,
        'git_dirty': False,
        'git_sha': candidate['git_sha'],
        'gz_partition': candidate['smoke']['gz_partition'],
        'repetition_index': trial.get('repetition_index'),
        'ros_domain_id': candidate['smoke']['ros_domain_id'],
        'run_id': candidate['smoke']['run_id'],
        'scenario_id': trial.get('scenario_id'),
        'scenario_index': trial.get('scenario_id'),
        'scenario_name': trial.get('scenario_name'),
        'scenario_sha256': trial.get('scenario_sha256'),
        'suite_index': trial.get('suite_index'),
    }
    if identity != expected:
        raise ProfileError('identity_changed', 'smoke result identity differs from suite plan')
    return {
        'manifest_sha256': manifest_hash,
        'marker_sha256': marker_hash,
        'result_sha256': result_hash,
    }


def validate_campaign_smoke_profile(
    workspace: Path,
    candidate_root: Path,
    candidate_id: str,
) -> dict[str, Any]:
    """
    Validate the canonical profiled smoke before any campaign mutation.

    Recorded absolute paths are checked as one internally consistent historical
    workspace. File reads are derived from the supplied clone-local workspace,
    and the returned binding contains portable identities only.
    """
    if CANDIDATE_RE.fullmatch(candidate_id) is None:
        raise ProfileError('invalid_argument', 'candidate ID is not path-safe')
    try:
        supplied_workspace = Path(workspace)
        current_workspace = supplied_workspace.resolve(strict=True)
        supplied_candidate = Path(candidate_root)
        current_candidate = supplied_candidate.resolve(strict=True)
    except OSError as exc:
        raise ProfileError('missing_identity', f'cannot resolve campaign input: {exc}') from exc
    expected_candidate = current_workspace / 'artifacts/evidence/phase3-benchmarks' / candidate_id
    if (
        supplied_workspace != current_workspace
        or supplied_candidate != current_candidate
        or current_candidate != expected_candidate
        or supplied_workspace.is_symlink()
        or supplied_candidate.is_symlink()
        or not current_workspace.is_dir()
        or not current_candidate.is_dir()
    ):
        message = 'campaign workspace/candidate path is not canonical'
        raise ProfileError('invalid_argument', message)
    _require_real_directory_tree(
        current_workspace,
        Path('artifacts/evidence/phase3-benchmarks') / candidate_id,
        'campaign candidate tree',
    )
    for relative in (Path('smoke'), Path('smoke/processes'), Path('smoke/result')):
        _require_real_directory_tree(current_candidate, relative, 'campaign smoke tree')

    output_parent = current_workspace / 'artifacts/evidence/phase3/performance-profiles'
    _validate_output_parent(current_workspace, output_parent)
    if not output_parent.is_dir() or output_parent.is_symlink():
        message = 'profile output parent is not a canonical directory'
        raise ProfileError('missing_identity', message)
    profile_path = output_parent / f'{candidate_id}-smoke-profile.json'
    profile, profile_hash = _load_canonical(
        profile_path,
        'campaign smoke host profile',
        OUTPUT_MAX_BYTES,
    )
    _profile_mapping(profile, 'campaign smoke host profile', PROFILE_TOP_LEVEL_KEYS)
    if (
        type(profile.get('schema_version')) is not int
        or profile.get('schema_version') != SCHEMA_VERSION
        or profile.get('producer') != PRODUCER
        or profile.get('status') != 'PASS'
    ):
        raise ProfileError(
            'invalid_profile', 'profile schema/producer/status is not canonical PASS'
        )
    started_epoch_ns = _profile_utc(profile.get('profile_started_utc'), 'profile start UTC')
    finished_epoch_ns = _profile_utc(profile.get('finished_utc'), 'profile finish UTC')
    if finished_epoch_ns < started_epoch_ns:
        raise ProfileError('invalid_profile', 'profile finish precedes profile start')
    started_monotonic_ns = _profile_int(
        profile.get('profile_started_monotonic_ns'), 'profile start monotonic time'
    )
    started_boot_ticks = _profile_int(
        profile.get('profile_started_boot_ticks'), 'profile start boot ticks'
    )
    clock_ticks = _profile_int(
        profile.get('host_clock_ticks_per_second'), 'host clock ticks per second', 1, 1_000_000_000
    )

    candidate = _validate_profile_candidate(
        profile,
        workspace=current_workspace,
        candidate_root=current_candidate,
        candidate_id=candidate_id,
    )
    _validate_profile_limits(profile.get('limits'))
    anchor = _validate_profile_anchor(profile.get('anchor'), profile['candidate'])
    if anchor['start_ticks'] + 1 < started_boot_ticks:
        raise ProfileError('identity_changed', 'anchor predates the profiled smoke')
    _validate_renderer(profile.get('renderer'), started_epoch_ns, finished_epoch_ns)
    full_stack = _validate_full_stack_profile(
        profile.get('full_stack_process'),
        candidate_root=current_candidate,
        recorded_candidate=candidate['recorded_candidate'],
        recorded_workspace=candidate['recorded_workspace'],
        anchor=anchor,
        profile_started_monotonic_ns=started_monotonic_ns,
    )
    contact, contact_bindings = _validate_contact_profile(
        profile.get('contact_profile'),
        candidate_root=current_candidate,
        recorded_candidate=candidate['recorded_candidate'],
        full_stack=full_stack,
    )
    sampling = _validate_sampling(
        profile.get('sampling'),
        anchor=anchor,
        clock_ticks=clock_ticks,
        profile_started_monotonic_ns=started_monotonic_ns,
    )
    first_wall_ns = sampling['first_sample_wall_epoch_ns']
    last_wall_ns = sampling['last_sample_wall_epoch_ns']
    if (
        first_wall_ns is None
        or last_wall_ns is None
        or profile.get('finished_utc') < _utc(last_wall_ns)
        or profile.get('profile_started_utc') > _utc(first_wall_ns)
    ):
        raise ProfileError('invalid_profile', 'profile wall-clock bounds do not cover samples')
    first_sample_monotonic_ns = sampling['first_sample_monotonic_ns']
    if (
        first_sample_monotonic_ns is None
        or full_stack.get('started_steady_ns')
        > first_sample_monotonic_ns + int(CANONICAL_SAMPLE_PERIOD_S * 1_000_000_000)
        or full_stack.get('finished_steady_ns')
        > first_sample_monotonic_ns
        + int((CANONICAL_MAX_DURATION_S + FULL_STACK_CLOSE_TIMEOUT_S) * 1_000_000_000)
        or full_stack.get('finished_steady_ns')
        > sampling['last_sample_monotonic_ns'] + int(FULL_STACK_CLOSE_TIMEOUT_S * 1_000_000_000)
    ):
        raise ProfileError('invalid_profile', 'full-stack lifecycle exceeds profile bounds')
    _validate_process_lifecycles(
        profile.get('process_lifecycles'),
        sampling=sampling,
        anchor=anchor,
        retained_cmdline_bytes=profile['sampling'].get('retained_cmdline_bytes'),
    )
    _validate_contact_thread_bindings(contact_bindings, contact, anchor, sampling)
    group_leader_identities = [
        identity
        for identity, indexes in sampling['process_samples'].items()
        if identity[0] == full_stack.get('pid') and indexes and indexes[0] == 0
    ]
    if len(group_leader_identities) != 1:
        raise ProfileError(
            'identity_changed', 'full-stack group leader lacks an exact sampled lifecycle'
        )
    group_leader_last_sample = sampling['process_samples'][group_leader_identities[0]][-1]
    if (
        full_stack.get('finished_steady_ns')
        < sampling['sample_monotonic_ns'][group_leader_last_sample]
    ):
        raise ProfileError(
            'identity_changed', 'full-stack finish precedes its last sampled lifecycle'
        )
    smoke = _validate_smoke_success(
        workspace=current_workspace,
        candidate_root=current_candidate,
        candidate_id=candidate_id,
        candidate=candidate,
    )
    return {
        'build_binding_sha256': candidate['binding_sha256'],
        'candidate_id': candidate_id,
        'git_sha': candidate['git_sha'],
        'plugin_sha256': candidate['plugin_sha256'],
        'prepared_marker_sha256': candidate['prepared_sha256'],
        'profile_relative_path': profile_path.relative_to(current_workspace).as_posix(),
        'profile_sha256': profile_hash,
        'profiler_producer_sha256': candidate['producer_sha256'],
        'schema_version': SCHEMA_VERSION,
        'smoke_marker_sha256': smoke['marker_sha256'],
        'smoke_result_manifest_sha256': smoke['manifest_sha256'],
        'smoke_run_id': candidate['smoke']['run_id'],
        'smoke_run_result_sha256': smoke['result_sha256'],
        'suite_plan_sha256': candidate['plan_sha256'],
    }


class SmokeHostProfiler:
    """Bounded monitor with injectable procfs and clocks for non-live tests."""

    def __init__(
        self,
        config: ProfileConfig,
        static_identity: Mapping[str, Any],
        proc_root: Path = Path('/proc'),
        monotonic: Callable[[], float] = time.monotonic,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        wall_time_ns: Callable[[], int] = time.time_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.config = config
        self.static_identity = dict(static_identity)
        self.proc_root = proc_root
        self.monotonic = monotonic
        self.monotonic_ns = monotonic_ns
        self.wall_time_ns = wall_time_ns
        self.sleep = sleep
        self.state = ProfileState()

    def run(self) -> dict[str, Any]:
        """Wait for, sample, and finalize one exact smoke process set."""
        started_epoch_ns = self.wall_time_ns()
        started_monotonic_ns = self.monotonic_ns()
        ticks_per_second = _clock_ticks_per_second()
        started_boot_ticks = read_boot_ticks(self.proc_root, ticks_per_second)
        run_dir = self.config.candidate_root / 'smoke'
        process_dir = run_dir / 'processes'
        pre_smoke_outputs = (
            process_dir / 'full_stack.process.json',
            process_dir / 'full_stack.stdout.log',
            process_dir / 'full_stack.stderr.log',
        )
        if any(path.exists() or path.is_symlink() for path in pre_smoke_outputs):
            raise ProfileError('missing_identity', 'full-stack output predates profiler start')
        renderer_before = _renderer_baseline(self.config.renderer_log)
        wait_deadline = self.monotonic() + self.config.startup_timeout_s
        anchor = None
        exact_before_anchor = 0
        while self.monotonic() < wait_deadline:
            anchor, exact_count = discover_anchor(
                self.proc_root,
                self.config.ros_domain_id,
                self.config.gz_partition,
                self.static_identity['plugin'],
            )
            exact_before_anchor = max(exact_before_anchor, exact_count)
            if self.monotonic() >= wait_deadline:
                raise ProfileError('no_target', 'anchor discovery exceeded startup timeout')
            if anchor is not None:
                break
            remaining = wait_deadline - self.monotonic()
            self.sleep(min(0.1, max(0.0, remaining)))
        if anchor is None:
            raise ProfileError('no_target', 'no exact candidate smoke DSO host appeared')
        anchor_start_ticks = anchor.get('start_ticks')
        if type(anchor_start_ticks) is not int or anchor_start_ticks + 1 < started_boot_ticks:
            raise ProfileError('missing_identity', 'anchor predates profiler start')
        sample_started = self.monotonic()
        duration_deadline = sample_started + self.config.max_duration_s
        next_sample = sample_started
        anchor_alive_samples = 0
        target_alive_samples = 0
        while True:
            now = self.monotonic()
            if now >= duration_deadline:
                raise ProfileError('incomplete_profile', 'target remained alive past timeout')
            if now < next_sample:
                self.sleep(min(next_sample, duration_deadline) - now)
                now = self.monotonic()
                if now >= duration_deadline:
                    raise ProfileError('incomplete_profile', 'target remained alive past timeout')
            if now - next_sample > self.config.sample_period_s:
                self.state.cadence_overruns += 1
            capture_started = now
            target_alive = capture_sample(
                self.proc_root,
                self.config,
                anchor,
                self.state,
                self.monotonic_ns,
                self.wall_time_ns,
            )
            capture_finished = self.monotonic()
            if capture_finished >= duration_deadline:
                raise ProfileError('incomplete_profile', 'target sampling exceeded timeout')
            if capture_finished - capture_started >= self.config.sample_period_s:
                self.state.cadence_overruns += 1
            if self.state.samples[-1]['anchor_alive']:
                anchor_alive_samples += 1
            if not target_alive:
                break
            target_alive_samples += 1
            next_sample = capture_finished + self.config.sample_period_s
        if len(self.state.samples) < MINIMUM_SAMPLES or anchor_alive_samples < 2:
            raise ProfileError('incomplete_profile', 'target ended before minimum sample coverage')
        if any(process.ended_sample is None for process in self.state.processes.values()):
            raise ProfileError('incomplete_profile', 'latched target process did not end')
        full_stack = wait_for_full_stack_close(
            run_dir,
            anchor,
            self.monotonic,
            self.sleep,
            profile_started_monotonic_ns=started_monotonic_ns,
        )
        full_stack['pre_smoke_outputs_absent'] = True
        renderer = collect_renderer(self.config.renderer_log, renderer_before, started_epoch_ns)
        contact = parse_contact_profile_logs(run_dir)
        if contact['present'] is not True:
            raise ProfileError('missing_identity', 'canonical contact profile record is absent')
        reconcile_contact_log_sources(contact, full_stack)
        contact['host_thread_bindings'] = bind_contact_profile_threads(contact, anchor, self.state)
        return {
            'schema_version': SCHEMA_VERSION,
            'producer': PRODUCER,
            'status': 'PASS',
            'profile_started_utc': _utc(started_epoch_ns),
            'profile_started_monotonic_ns': started_monotonic_ns,
            'profile_started_boot_ticks': started_boot_ticks,
            'finished_utc': _utc(self.wall_time_ns()),
            'candidate': self.static_identity,
            'anchor': anchor,
            'renderer': renderer,
            'contact_profile': contact,
            'full_stack_process': full_stack,
            'host_clock_ticks_per_second': ticks_per_second,
            'limits': _profile_limits(self.config),
            'process_lifecycles': _process_lifecycles(self.state),
            'sampling': {
                'sample_count': len(self.state.samples),
                'anchor_alive_sample_count': anchor_alive_samples,
                'target_alive_sample_count': target_alive_samples,
                'thread_record_count': self.state.thread_records,
                'retained_cmdline_bytes': self.state.cmdline_bytes,
                'cadence_overrun_count': self.state.cadence_overruns,
                'exact_processes_seen_before_anchor': exact_before_anchor,
                'samples': self.state.samples,
                'totals': _cpu_totals(self.state, ticks_per_second),
            },
        }


def _failure_document(
    config: ProfileConfig,
    error: ProfileError,
    identity: Mapping[str, Any] | None,
    profiler: SmokeHostProfiler | None,
) -> dict[str, Any]:
    document: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'producer': PRODUCER,
        'status': 'FAIL',
        'finished_utc': _utc(time.time_ns()),
        'candidate_id': config.candidate_id,
        'candidate': identity,
        'failure': {'kind': error.kind, 'message': str(error).replace('\x00', '?')[:1024]},
        'limits': _profile_limits(config),
    }
    if profiler is not None:
        document['partial_profile'] = {
            'sample_count': len(profiler.state.samples),
            'thread_record_count': profiler.state.thread_records,
            'retained_cmdline_bytes': profiler.state.cmdline_bytes,
            'process_lifecycles': _process_lifecycles(profiler.state),
            'samples': profiler.state.samples,
        }
    return document


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--candidate-root', required=True, type=Path)
    parser.add_argument('--candidate-id', required=True)
    parser.add_argument('--build-binding', required=True, type=Path)
    parser.add_argument('--ros-domain-id', required=True, type=int)
    parser.add_argument('--gz-partition', required=True)
    parser.add_argument('--startup-timeout-s', type=float, default=CANONICAL_STARTUP_TIMEOUT_S)
    parser.add_argument('--sample-period-s', type=float, default=CANONICAL_SAMPLE_PERIOD_S)
    parser.add_argument('--max-duration-s', type=float, default=CANONICAL_MAX_DURATION_S)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    try:
        config = validated_config(_parser().parse_args(argv))
    except ProfileError as exc:
        print(f'Phase 3 smoke host profiler: {exc.kind}: {exc}', file=sys.stderr)
        return 2
    sidecar = Path(f'{config.output_path}.sha256')
    if config.output_path.exists() or config.output_path.is_symlink() or sidecar.exists():
        print(f'Phase 3 smoke host profiler: output_exists: {config.output_path}', file=sys.stderr)
        return 2
    identity: Mapping[str, Any] | None = None
    profiler: SmokeHostProfiler | None = None
    try:
        identity = collect_static_identity(config)
        profiler = SmokeHostProfiler(config, identity)
        document = profiler.run()
        if collect_static_identity(config) != identity:
            raise ProfileError('identity_changed', 'candidate identity changed during profile')
    except ProfileError as exc:
        document = _failure_document(config, exc, identity, profiler)
        status = 1
    except Exception as exc:  # fail closed with bounded unexpected-error evidence
        failure = ProfileError('internal_error', f'{type(exc).__name__}: {str(exc)[:900]}')
        document = _failure_document(config, failure, identity, profiler)
        status = 1
    else:
        status = 0
    try:
        digest = write_profile(config.output_path, document)
    except ProfileError as exc:
        print(f'Phase 3 smoke host profiler: {exc.kind}: {exc}', file=sys.stderr)
        return 2
    print(f'Phase 3 smoke host profile {document["status"]}: {config.output_path}')
    print(f'Phase 3 smoke host profile SHA-256: {digest}')
    return status


if __name__ == '__main__':
    raise SystemExit(main())
