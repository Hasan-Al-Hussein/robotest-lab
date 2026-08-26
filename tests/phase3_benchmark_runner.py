#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Owned-process Phase 3 positive-control, smoke, and candidate runner.

The CLI is intentionally staged.  ``prepare`` freezes a clean candidate
ledger, ``positive-control`` qualifies the collision path, ``smoke`` executes
one non-candidate integration run, and ``campaign`` is the only mode allowed to
start the exact ordered 15 cold-stack trials.  Campaign mode requires an
explicit literal authorization and never retries or replaces an index.
"""

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
from contextlib import suppress
from dataclasses import dataclass
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import threading
import time
from typing import Any, BinaryIO

from phase3_orchestration import (
    AGGREGATE_METRICS,
    atomic_write_bytes,
    atomic_write_json,
    build_binding,
    canonical_json_bytes,
    component_manifest,
    compose_analysis_request,
    CONTACT_CONTROL_WALL_TIMEOUT_S,
    CONTACT_DRAIN_NS,
    CPU_AFFINITY,
    EvidenceError,
    failure_evidence,
    file_sha256,
    load_json,
    LOG_MAX_BYTES,
    make_orchestrator_evidence,
    make_trial_context,
    positive_control_qualified_snapshot_stamp,
    PRODUCER,
    reconcile_contact_gate_reobservation,
    reconcile_goal_binding,
    reconcile_positive_control,
    safe_candidate_id,
    suite_document,
    summarize_resources,
    validate_build_binding,
    validate_contact_drain_evidence,
    validate_contact_progress,
    verify_component_manifest,
    verify_json_sidecar,
)

CAMPAIGN_AUTHORIZATION = 'I_AUTHORIZE_EXACTLY_15_COLD_STACK_TRIALS_NO_RETRIES'
MAX_PROCESSES = 64
MAX_RESOURCE_SAMPLES = 4096
MAX_RESOURCE_PID_IDENTITIES = 4096
MAX_AFFINITY_EVIDENCE_PREFIX = 64
RESOURCE_SAMPLE_INTERVAL_S = 1.0
GROUP_TERM_GRACE_S = 5.0
PROCESS_KILL_GRACE_S = 10.0
READY_POLL_S = 0.05
TOKEN_PATTERN = re.compile(r'^[a-z][a-z0-9_]{0,63}$')


def _bounded_reason(value: object) -> str:
    """Return one non-empty diagnostic bounded to 4,096 UTF-8 bytes."""
    text = str(value) or 'unspecified orchestration failure'
    encoded = text.encode('utf-8', errors='replace')
    if len(encoded) <= 4096:
        return encoded.decode('utf-8')
    prefix = encoded[:4093]
    while True:
        try:
            return prefix.decode('utf-8') + '...'
        except UnicodeDecodeError:
            prefix = prefix[:-1]


class StageFailure(EvidenceError):  # noqa: N818
    """Observed, bounded orchestration failure eligible for canonical composition."""

    def __init__(
        self,
        stage: str,
        kind: str,
        reason: str,
        *,
        exit_code: int = 1,
        wall_timed_out: bool = False,
        evidence: Any | None = None,
    ) -> None:
        super().__init__(_bounded_reason(reason))
        if TOKEN_PATTERN.fullmatch(stage) is None or TOKEN_PATTERN.fullmatch(kind) is None:
            raise ValueError('StageFailure stage/kind must be lowercase bounded tokens')
        self.stage = stage
        self.kind = kind
        self.exit_code = exit_code if exit_code != 0 else 1
        self.wall_timed_out = wall_timed_out
        self.evidence = {'reason': reason} if evidence is None else evidence

    def document(self) -> dict[str, Any]:
        """Return the strict trial-context failure object."""
        return failure_evidence(
            stage=self.stage,
            kind=self.kind,
            exit_code=self.exit_code,
            wall_timed_out=self.wall_timed_out,
            reason=str(self),
            evidence=self.evidence,
        )


@dataclass
class StreamState:
    """Bounded drain state for one child output stream."""

    maximum_bytes: int
    observed_bytes: int = 0
    retained_bytes: int = 0
    overflow: bool = False
    error: str | None = None


class BoundedProcess:
    """One direct child whose complete tree is owned by a new POSIX process group."""

    def __init__(
        self,
        *,
        role: str,
        command: Sequence[str],
        cwd: Path,
        env: Mapping[str, str],
        evidence_dir: Path,
        wall_timeout_s: float,
    ) -> None:
        if TOKEN_PATTERN.fullmatch(role) is None:
            raise EvidenceError(f'invalid process role token: {role}')
        if not 0.0 < wall_timeout_s <= 600.0:
            raise EvidenceError(f'invalid process wall timeout for {role}')
        self.role = role
        self.command = [str(item) for item in command]
        self.cwd = cwd
        self.wall_timeout_s = wall_timeout_s
        self.started_steady_ns = time.monotonic_ns()
        self.finished_steady_ns: int | None = None
        self.returncode: int | None = None
        self.timed_out = False
        self._metadata_written = False
        self._group_confirmed_empty = False
        wrapped = [
            'timeout',
            '--signal=TERM',
            f'--kill-after={PROCESS_KILL_GRACE_S:.0f}s',
            f'{wall_timeout_s:.3f}s',
            *self.command,
        ]
        self.wrapped_command = wrapped
        self.stdout_path = evidence_dir / f'{role}.stdout.log'
        self.stderr_path = evidence_dir / f'{role}.stderr.log'
        self.metadata_path = evidence_dir / f'{role}.process.json'
        for path in (self.stdout_path, self.stderr_path, self.metadata_path):
            if path.exists() or path.is_symlink():
                raise EvidenceError(f'process evidence path already exists: {path}')
        evidence_dir.mkdir(parents=True, exist_ok=True)
        self.process = subprocess.Popen(
            wrapped,
            cwd=cwd,
            env=dict(env),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        self.pid = self.process.pid
        self.pgid = os.getpgid(self.pid)
        if self.pid != self.pgid:
            self.process.kill()
            self.process.wait(timeout=5)
            raise EvidenceError(f'{role} did not become its process-group leader')
        self.start_ticks = _process_start_ticks(self.pid)
        self.stdout_state = StreamState(LOG_MAX_BYTES)
        self.stderr_state = StreamState(LOG_MAX_BYTES)
        assert self.process.stdout is not None and self.process.stderr is not None
        self._threads = [
            threading.Thread(
                target=self._drain,
                args=(self.process.stdout, self.stdout_path, self.stdout_state),
                name=f'{role}-stdout-drain',
                daemon=True,
            ),
            threading.Thread(
                target=self._drain,
                args=(self.process.stderr, self.stderr_path, self.stderr_state),
                name=f'{role}-stderr-drain',
                daemon=True,
            ),
        ]
        for thread in self._threads:
            thread.start()

    @staticmethod
    def _drain(source: BinaryIO, path: Path, state: StreamState) -> None:
        try:
            with path.open('wb') as target:
                while block := source.read(64 * 1024):
                    state.observed_bytes += len(block)
                    remaining = state.maximum_bytes - state.retained_bytes
                    if remaining > 0:
                        prefix = block[:remaining]
                        target.write(prefix)
                        state.retained_bytes += len(prefix)
                    if state.observed_bytes > state.maximum_bytes:
                        state.overflow = True
                target.flush()
                os.fsync(target.fileno())
        except (OSError, ValueError) as exc:
            state.error = _bounded_reason(f'{type(exc).__name__}: {exc}')

    def poll(self) -> int | None:
        """Return the child exit status without changing ownership."""
        return self.process.poll()

    def wait(self, timeout_s: float | None = None) -> int:
        """Wait under a caller bound, then close bounded stream evidence."""
        try:
            self.returncode = self.process.wait(timeout=timeout_s)
        except subprocess.TimeoutExpired:
            self.timed_out = True
            self.stop()
            self.returncode = self.process.returncode
        self._finish()
        assert self.returncode is not None
        return self.returncode

    def stop(self) -> bool:
        """TERM then KILL only this exact owned process group."""
        if self._group_confirmed_empty:
            return True
        if self.process.poll() is None:
            try:
                current_pgid = os.getpgid(self.pid)
                current_start_ticks = _process_start_ticks(self.pid)
            except (FileNotFoundError, ProcessLookupError):
                current_pgid = -1
                current_start_ticks = -1
            if self.process.poll() is not None:
                current_pgid = self.pgid
                current_start_ticks = self.start_ticks
            if current_pgid != self.pgid or current_start_ticks != self.start_ticks:
                raise EvidenceError(f'refusing to signal changed process identity for {self.role}')
        else:
            members = _group_members(self.pgid)
            if not members:
                self._group_confirmed_empty = True
                self._finish()
                return True
            if any(pid == self.pgid for pid, _start_ticks in members) or any(
                start_ticks < self.start_ticks for _pid, start_ticks in members
            ):
                raise EvidenceError(
                    f'refusing to signal an unowned/reused process group for {self.role}'
                )
        alive = _group_alive(self.pgid)
        if alive:
            try:
                os.killpg(self.pgid, signal.SIGTERM)
            except ProcessLookupError:
                alive = False
            deadline = time.monotonic() + GROUP_TERM_GRACE_S
            while alive and time.monotonic() < deadline:
                time.sleep(0.1)
                alive = _group_alive(self.pgid)
            if alive:
                with suppress(ProcessLookupError):
                    os.killpg(self.pgid, signal.SIGKILL)
        try:
            self.returncode = self.process.wait(timeout=PROCESS_KILL_GRACE_S)
        except subprocess.TimeoutExpired:
            self.returncode = None
        self._group_confirmed_empty = not _group_alive(self.pgid)
        self._finish()
        return self._group_confirmed_empty

    def _finish(self) -> None:
        if self.finished_steady_ns is None and self.process.poll() is not None:
            self.finished_steady_ns = time.monotonic_ns()
            if not _group_alive(self.pgid):
                self._group_confirmed_empty = True
        for thread in self._threads:
            thread.join(timeout=5.0)
        if any(thread.is_alive() for thread in self._threads):
            raise EvidenceError(f'{self.role} output drain thread did not finish')
        if not self._metadata_written:
            atomic_write_json(
                self.metadata_path,
                {
                    'command': self.command,
                    'cwd': str(self.cwd),
                    'finished_steady_ns': self.finished_steady_ns,
                    'group_confirmed_empty': self._group_confirmed_empty,
                    'pgid': self.pgid,
                    'pid': self.pid,
                    'returncode': self.returncode,
                    'role': self.role,
                    'started_steady_ns': self.started_steady_ns,
                    'stderr': vars(self.stderr_state),
                    'stdout': vars(self.stdout_state),
                    'timed_out': self.timed_out or self.returncode == 124,
                    'wall_timeout_s': self.wall_timeout_s,
                    'wrapped_command': self.wrapped_command,
                },
                maximum_bytes=64 * 1024,
            )
            self._metadata_written = True
        if self.stdout_state.error is not None or self.stderr_state.error is not None:
            raise EvidenceError(f'{self.role} output drain failed')

    @property
    def logs_within_cap(self) -> bool:
        return (
            not self.stdout_state.overflow
            and not self.stderr_state.overflow
            and self.stdout_state.error is None
            and self.stderr_state.error is None
        )


class ProcessRegistry:
    """Bounded owner of all process groups created in one stage or trial."""

    def __init__(self, evidence_dir: Path, cwd: Path, env: Mapping[str, str]) -> None:
        self.evidence_dir = evidence_dir
        self.cwd = cwd
        self.env = dict(env)
        self.processes: list[BoundedProcess] = []
        self._roles: set[str] = set()
        self._lock = threading.Lock()

    def start(
        self,
        role: str,
        command: Sequence[str],
        *,
        wall_timeout_s: float,
    ) -> BoundedProcess:
        with self._lock:
            if len(self.processes) >= MAX_PROCESSES:
                raise EvidenceError('process registry exceeded 64 owned groups')
            if role in self._roles:
                raise EvidenceError(f'duplicate process role: {role}')
            self._roles.add(role)
        process = BoundedProcess(
            role=role,
            command=command,
            cwd=self.cwd,
            env=self.env,
            evidence_dir=self.evidence_dir,
            wall_timeout_s=wall_timeout_s,
        )
        with self._lock:
            self.processes.append(process)
        return process

    def run_checked(
        self,
        role: str,
        command: Sequence[str],
        *,
        wall_timeout_s: float,
        stage: str,
    ) -> BoundedProcess:
        process = self.start(role, command, wall_timeout_s=wall_timeout_s)
        status = process.wait(wall_timeout_s + PROCESS_KILL_GRACE_S + 2.0)
        if status != 0:
            raise StageFailure(
                stage,
                'process_exit',
                f'{role} exited with status {status}',
                exit_code=status,
                wall_timed_out=process.timed_out or status == 124,
                evidence=load_json(process.metadata_path),
            )
        if not process.logs_within_cap:
            raise StageFailure(
                stage,
                'log_overflow',
                f'{role} exceeded the 8 MiB per-stream log cap',
                evidence=load_json(process.metadata_path),
            )
        return process

    def snapshot(self) -> list[BoundedProcess]:
        with self._lock:
            return list(self.processes)

    def stop_all(self) -> bool:
        result = True
        for process in reversed(self.snapshot()):
            try:
                result = process.stop() and result
            except (EvidenceError, OSError):
                result = False
        return result and all(not _group_alive(item.pgid) for item in self.snapshot())

    def logs_within_cap(self) -> bool:
        return all(item.logs_within_cap for item in self.snapshot())


class ResourceSampler:
    """Bounded one-Hz sampler for every PID in every owned process group."""

    def __init__(self, registry: ProcessRegistry, output: Path) -> None:
        self.registry = registry
        self.output = output
        self.samples: list[dict[str, Any]] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pid_start_ticks: dict[int, int] = {}
        self._cpu_ticks_by_identity: dict[tuple[int, int], int] = {}
        self._oom_start = _oom_kill_count()
        self._last_cpu_total: float | None = None
        self._last_wall: float | None = None
        self.failure: str | None = None

    def start(self) -> None:
        if self.output.exists() or self.output.is_symlink():
            raise EvidenceError(f'resource trace already exists: {self.output}')
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.sample('before_launch')
        self._thread = threading.Thread(target=self._loop, name='phase3-resource-sampler')
        self._thread.start()

    def _loop(self) -> None:
        while not self._stop.wait(RESOURCE_SAMPLE_INTERVAL_S):
            try:
                self.sample('run')
            except Exception as exc:  # retained as bounded evidence; caller fails closed
                self.failure = str(exc)[:512]
                self._stop.set()
                return

    def sample(self, phase: str) -> None:
        if len(self.samples) >= MAX_RESOURCE_SAMPLES:
            raise EvidenceError('resource sampler exceeded 4,096 samples')
        processes = self.registry.snapshot()
        pgids = {item.pgid for item in processes}
        rss_sum = 0
        observed_pids: set[int] = set()
        observed_pgids: set[int] = set()
        affinity_checked_pid_count = 0
        affinity_observed_cpu_union: set[int] = set()
        affinity_escape_count = 0
        affinity_escape_prefix: list[dict[str, Any]] = []
        affinity_unreadable_count = 0
        affinity_unreadable_pid_prefix: list[int] = []
        pid_reuse = False
        expected_affinity = set(CPU_AFFINITY)
        for path in Path('/proc').iterdir():
            if not path.name.isdigit():
                continue
            pid = int(path.name)
            try:
                observed_pgid = os.getpgid(pid)
                if observed_pgid not in pgids:
                    continue
                start_ticks, process_cpu_ticks = _process_stat(pid)
                rss_sum += _process_rss_bytes(pid)
            except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
                continue
            try:
                allowed_cpus = sorted(os.sched_getaffinity(pid))
            except ProcessLookupError:
                continue
            except (OSError, PermissionError):
                affinity_unreadable_count += 1
                if len(affinity_unreadable_pid_prefix) < MAX_AFFINITY_EVIDENCE_PREFIX:
                    affinity_unreadable_pid_prefix.append(pid)
                continue
            affinity_checked_pid_count += 1
            affinity_observed_cpu_union.update(allowed_cpus)
            escaped = sorted(set(allowed_cpus) - expected_affinity)
            if escaped:
                affinity_escape_count += 1
                if len(affinity_escape_prefix) < MAX_AFFINITY_EVIDENCE_PREFIX:
                    affinity_escape_prefix.append(
                        {'allowed_cpus': allowed_cpus, 'escaped_cpus': escaped, 'pid': pid}
                    )
            prior = self._pid_start_ticks.setdefault(pid, start_ticks)
            pid_reuse = pid_reuse or prior != start_ticks
            self._cpu_ticks_by_identity[(pid, start_ticks)] = process_cpu_ticks
            if len(self._cpu_ticks_by_identity) > MAX_RESOURCE_PID_IDENTITIES:
                raise EvidenceError('resource sampler exceeded 4,096 PID identities')
            observed_pids.add(pid)
            observed_pgids.add(observed_pgid)
        cpu_ticks = sum(self._cpu_ticks_by_identity.values())
        now = time.monotonic()
        cpu_percent = 0.0
        if self._last_cpu_total is not None and self._last_wall is not None:
            tick_delta = max(0.0, cpu_ticks - self._last_cpu_total)
            wall_delta = max(1e-9, now - self._last_wall)
            cpu_percent = tick_delta / os.sysconf('SC_CLK_TCK') / wall_delta * 100.0
        self._last_cpu_total = float(cpu_ticks)
        self._last_wall = now
        expected_live = sum(1 for item in processes if item.poll() is None)
        memory, swap = _wsl_memory_usage()
        sample = {
            'affinity_checked_pid_count': affinity_checked_pid_count,
            'affinity_escape_count': affinity_escape_count,
            'affinity_escape_prefix': affinity_escape_prefix,
            'affinity_observed_cpu_union': sorted(affinity_observed_cpu_union),
            'affinity_unreadable_count': affinity_unreadable_count,
            'affinity_unreadable_pid_prefix': affinity_unreadable_pid_prefix,
            'cpu_percent': cpu_percent,
            'missing_count': max(0, expected_live - len(observed_pgids)),
            'oom_kill': _oom_kill_count() > self._oom_start,
            'owned_group_count': len(pgids),
            'phase': phase,
            'pid_count': len(observed_pids),
            'pid_reuse_detected': pid_reuse,
            'rss_sum_bytes': rss_sum,
            'steady_wall_ns': time.monotonic_ns(),
            'wsl_memory_bytes': memory,
            'wsl_swap_bytes': swap,
        }
        self.samples.append(sample)
        with self.output.open('ab') as stream:
            payload = canonical_json_bytes(sample)
            if self.output.stat().st_size + len(payload) > LOG_MAX_BYTES:
                raise EvidenceError('resource trace exceeded 8 MiB')
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        if affinity_escape_count:
            raise EvidenceError('owned process escaped the frozen CPU affinity')
        if affinity_unreadable_count:
            raise EvidenceError('owned process CPU affinity was unreadable')

    def stop_after_shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            if self._thread.is_alive():
                raise EvidenceError('resource sampler thread did not stop')
        self.sample('after_shutdown')
        if self.failure is not None:
            raise EvidenceError(f'resource sampler failed: {self.failure}')


def _process_stat(pid: int) -> tuple[int, int]:
    text = Path(f'/proc/{pid}/stat').read_text(encoding='ascii')
    closing = text.rfind(')')
    if closing < 0:
        raise ValueError('malformed /proc stat')
    fields = text[closing + 2 :].split()  # noqa: E203, RUF100
    return int(fields[19]), int(fields[11]) + int(fields[12])


def _process_start_ticks(pid: int) -> int:
    return _process_stat(pid)[0]


def _process_rss_bytes(pid: int) -> int:
    for line in Path(f'/proc/{pid}/status').read_text(encoding='ascii').splitlines():
        if line.startswith('VmRSS:'):
            return int(line.split()[1]) * 1024
    return 0


def _group_alive(pgid: int) -> bool:
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _group_members(pgid: int) -> list[tuple[int, int]]:
    """Return live PID/start-time pairs for one exact POSIX process group."""
    members: list[tuple[int, int]] = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        pid = int(path.name)
        try:
            if os.getpgid(pid) == pgid:
                members.append((pid, _process_start_ticks(pid)))
                if len(members) > MAX_RESOURCE_PID_IDENTITIES:
                    raise EvidenceError('owned process group exceeded 4,096 PID identities')
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
            continue
    return members


def _oom_kill_count() -> int:
    try:
        for line in Path('/proc/vmstat').read_text(encoding='ascii').splitlines():
            if line.startswith('oom_kill '):
                return int(line.split()[1])
    except (OSError, ValueError, IndexError):
        pass
    return 0


def _wsl_memory_usage() -> tuple[int, int]:
    values: dict[str, int] = {}
    for line in Path('/proc/meminfo').read_text(encoding='ascii').splitlines():
        parts = line.split()
        if len(parts) >= 2:
            values[parts[0].rstrip(':')] = int(parts[1]) * 1024
    memory = max(0, values.get('MemTotal', 0) - values.get('MemAvailable', 0))
    swap = max(0, values.get('SwapTotal', 0) - values.get('SwapFree', 0))
    return memory, swap


def _git_state(workspace: Path) -> tuple[str, str]:
    head = subprocess.run(
        ['git', '-C', str(workspace), 'rev-parse', 'HEAD'],
        capture_output=True,
        check=True,
        text=True,
        timeout=10,
        start_new_session=True,
    ).stdout.strip()
    status = subprocess.run(
        ['git', '-C', str(workspace), 'status', '--porcelain=v1', '--untracked-files=all'],
        capture_output=True,
        check=True,
        text=True,
        timeout=15,
        start_new_session=True,
    ).stdout
    return head, status


def _require_exact_affinity() -> None:
    observed = sorted(os.sched_getaffinity(0))
    if observed != CPU_AFFINITY:
        raise EvidenceError(
            f'Phase 3 runtime affinity must be exactly {CPU_AFFINITY}, got {observed}'
        )


def _new_directory(path: Path) -> None:
    if path.exists() and (not path.is_dir() or any(path.iterdir())):
        raise EvidenceError(f'Phase 3 refuses stale/non-directory output: {path}')
    path.mkdir(parents=True, exist_ok=True)


def _write_marker(path: Path, document: Mapping[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise EvidenceError(f'marker already exists: {path}')
    atomic_write_json(path, dict(document), maximum_bytes=64 * 1024, sidecar=True)


def _wait_for_file(
    path: Path,
    *,
    timeout_s: float,
    watched: Sequence[BoundedProcess],
    stage: str,
) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if path.is_file():
            return
        for process in watched:
            status = process.poll()
            if status is not None:
                process.wait(1.0)
                raise StageFailure(
                    stage,
                    'readiness_process_exit',
                    f'{process.role} exited with status {status} before {path.name}',
                    exit_code=status,
                    evidence=load_json(process.metadata_path),
                )
        time.sleep(READY_POLL_S)
    raise StageFailure(
        stage,
        'readiness_timeout',
        f'timed out waiting {timeout_s:.1f}s for {path.name}',
        exit_code=124,
        wall_timed_out=True,
        evidence={'path': str(path), 'timeout_s': timeout_s},
    )


def _validate_goal_observer_armed(
    document: object,
    ready_document: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate the observer-owned arm acknowledgment before mission launch."""
    if not isinstance(document, Mapping):
        raise EvidenceError('goal observer arm acknowledgment must be an object')
    expected_keys = {
        'armed_steady_ns',
        'prearm_uuid_set_sha256',
        'producer',
        'schema_version',
    }
    if set(document) != expected_keys:
        raise EvidenceError('goal observer arm acknowledgment keys are invalid')
    armed_steady_ns = document.get('armed_steady_ns')
    if (
        isinstance(armed_steady_ns, bool)
        or not isinstance(armed_steady_ns, int)
        or armed_steady_ns <= 0
    ):
        raise EvidenceError('goal observer armed_steady_ns must be positive')
    if document.get('producer') != 'robotest_phase3/goal_observer':
        raise EvidenceError('goal observer arm acknowledgment producer is invalid')
    schema_version = document.get('schema_version')
    if isinstance(schema_version, bool) or schema_version != 1:
        raise EvidenceError('goal observer arm acknowledgment schema is invalid')
    prearm_hash = document.get('prearm_uuid_set_sha256')
    if not isinstance(prearm_hash, str) or re.fullmatch(r'[0-9a-f]{64}', prearm_hash) is None:
        raise EvidenceError('goal observer arm acknowledgment prearm hash is invalid')
    if prearm_hash != ready_document.get('prearm_uuid_set_sha256'):
        raise EvidenceError('goal observer prearm UUID set changed after readiness')
    return dict(document)


def _wait_for_contact_progress(
    path: Path,
    *,
    qualifying_stamp_ns: int,
    timeout_s: float,
    watched: Sequence[BoundedProcess],
    stage: str = 'contact_drain',
) -> dict[str, Any]:
    """Wait until the collector acknowledges retaining at least the drain stamp."""
    deadline = time.monotonic() + timeout_s
    last_error: str | None = None
    while time.monotonic() < deadline:
        if path.is_file():
            try:
                return validate_contact_progress(
                    load_json(path), minimum_retained_stamp_ns=qualifying_stamp_ns
                )
            except (EvidenceError, OSError) as exc:
                last_error = str(exc)
        for process in watched:
            status = process.poll()
            if status is not None:
                process.wait(1.0)
                raise StageFailure(
                    stage,
                    'progress_process_exit',
                    f'{process.role} exited with status {status} before contact retention ack',
                    exit_code=status,
                    evidence=load_json(process.metadata_path),
                )
        time.sleep(READY_POLL_S)
    raise StageFailure(
        stage,
        'progress_timeout',
        f'collector did not acknowledge contact stamp {qualifying_stamp_ns} within '
        f'{timeout_s:.1f}s',
        exit_code=124,
        wall_timed_out=True,
        evidence={'last_error': last_error, 'path': str(path)},
    )


def _metrics_collector_command(
    *,
    capture_path: Path,
    ready_path: Path,
    stop_path: Path,
    contact_progress_path: Path,
) -> list[str]:
    """Build the collector command around its one producer/consumer ACK path."""
    return [
        'ros2',
        'run',
        'robotest_metrics',
        'metrics_collector',
        '--output',
        str(capture_path),
        '--ready-file',
        str(ready_path),
        '--stop-file',
        str(stop_path),
        '--contact-progress-file',
        str(contact_progress_path),
        '--wall-timeout-s',
        '360',
        '--ros-args',
        '-r',
        '__ns:=/robotest',
    ]


def _environment(domain_id: int, partition: str) -> dict[str, str]:
    env = dict(os.environ)
    env.update(
        {
            'GZ_PARTITION': partition,
            'RCUTILS_COLORIZED_OUTPUT': '0',
            'ROS2CLI_NO_DAEMON': '1',
            'ROS_DOMAIN_ID': str(domain_id),
            'ROS_LOCALHOST_ONLY': '1',
        }
    )
    return env


def _require_empty_stdout(process: BoundedProcess, *, stage: str, subject: str) -> None:
    if process.stdout_path.read_text(encoding='utf-8').strip():
        raise StageFailure(
            stage,
            'isolation_in_use',
            f'{subject} was not unused/empty',
            evidence=load_json(process.metadata_path),
        )


def _require_campaign_aggregate_pass(
    status: int, aggregate_path: Path, *, cleanup_ok: bool
) -> None:
    """Preserve a canonical aggregate FAIL but never report campaign success for it."""
    if not aggregate_path.is_file():
        raise EvidenceError(f'metrics aggregate exited {status} without a canonical result')
    aggregate = load_json(aggregate_path)
    verdict = aggregate.get('verdict')
    automated_status = verdict.get('automated_status') if isinstance(verdict, Mapping) else None
    if status != 0 or automated_status != 'PASS':
        raise EvidenceError(
            f'candidate aggregate verdict is {automated_status!r} with exit status {status}'
        )
    if not cleanup_ok:
        raise EvidenceError('metrics aggregate owned process group did not shut down cleanly')


def _runtime_gate_command(
    workspace: Path,
    mode: str,
    output: Path,
    *,
    watch_pid: int | None,
    launch_pid: int | None,
    expected_domain_id: int | None,
    expected_gz_partition: str | None,
    wall_timeout_s: float,
) -> list[str]:
    command = [
        'python3',
        str(workspace / 'tests/phase3_runtime_gate.py'),
        '--mode',
        mode,
        '--output',
        str(output),
        '--workspace',
        str(workspace),
        '--wall-timeout-s',
        str(wall_timeout_s),
    ]
    if watch_pid is not None:
        command.extend(['--watch-pid', str(watch_pid)])
    if launch_pid is not None:
        command.extend(['--launch-pid', str(launch_pid)])
    if expected_domain_id is not None:
        command.extend(['--expected-domain-id', str(expected_domain_id)])
    if expected_gz_partition is not None:
        command.extend(['--expected-gz-partition', expected_gz_partition])
    return command


def _graph_probe_command(
    workspace: Path,
    run_dir: Path,
    *,
    watch_pid: int,
    mission_client: bool,
) -> list[str]:
    topics = {
        '/clock': 'rosgraph_msgs/msg/Clock',
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
        '/robotest/cmd_vel': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_behavior_unused': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_nav': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_smoothed': 'geometry_msgs/msg/Twist',
        '/robotest/collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState',
        '/robotest/faults/events': 'robotest_interfaces/msg/FaultEvent',
        '/robotest/imu': 'sensor_msgs/msg/Imu',
        '/robotest/map': 'nav_msgs/msg/OccupancyGrid',
        '/robotest/navigation/plan': 'nav_msgs/msg/Path',
        '/robotest/odom': 'nav_msgs/msg/Odometry',
        '/robotest/raw/imu': 'sensor_msgs/msg/Imu',
        '/robotest/raw/odom': 'nav_msgs/msg/Odometry',
        '/robotest/raw/scan': 'sensor_msgs/msg/LaserScan',
        '/robotest/scan': 'sensor_msgs/msg/LaserScan',
        '/robotest/validation/contacts': 'ros_gz_interfaces/msg/Contacts',
        '/robotest/validation/ground_truth': 'nav_msgs/msg/Odometry',
        '/robotest/validation/scenario_entity_poses': 'tf2_msgs/msg/TFMessage',
        '/robotest/validation/world_stats': 'ros_gz_interfaces/msg/WorldStatistics',
    }
    services = {
        '/robotest/faults/arm_schedule': 'robotest_interfaces/srv/ArmFaultSchedule',
        '/robotest/faults/preload_schedule': 'robotest_interfaces/srv/PreloadFaultSchedule',
        '/robotest/faults/reset': 'std_srvs/srv/Trigger',
        '/robotest/scenario/delete_entity': 'ros_gz_interfaces/srv/DeleteEntity',
        '/robotest/scenario/set_entity_pose': 'ros_gz_interfaces/srv/SetEntityPose',
        '/robotest/scenario/spawn_entity': 'ros_gz_interfaces/srv/SpawnEntity',
    }
    prefix = 'mission-' if mission_client else ''
    command = ['python3', str(workspace / 'tests/phase2_graph_probe.py')]
    for name, type_name in topics.items():
        command.extend(['--topic', f'{name}={type_name}'])
    for name, type_name in services.items():
        command.extend(['--service', f'{name}={type_name}'])
    command.extend(
        [
            '--action',
            '/robotest/follow_waypoints=nav2_msgs/action/FollowWaypoints',
            '--action-server',
            '/robotest/follow_waypoints=/robotest/waypoint_follower',
        ]
    )
    if mission_client:
        command.extend(
            [
                '--action-client',
                '/robotest/follow_waypoints=/robotest/mission_runner',
            ]
        )
    command.extend(
        [
            '--output',
            str(run_dir / f'{prefix}graph.json'),
            '--nodes-output',
            str(run_dir / f'{prefix}nodes.txt'),
            '--topics-output',
            str(run_dir / f'{prefix}topics.txt'),
            '--services-output',
            str(run_dir / f'{prefix}services.txt'),
            '--actions-output',
            str(run_dir / f'{prefix}actions.txt'),
            '--watch-pid',
            str(watch_pid),
            '--wall-timeout',
            '90',
        ]
    )
    return command


class BenchmarkRunner:
    """Staged Phase 3 runner with no replacement retry path."""

    def __init__(self, arguments: argparse.Namespace) -> None:
        self.workspace = arguments.workspace.resolve()
        self.candidate_id = safe_candidate_id(arguments.candidate_id)
        self.domain_base = arguments.domain_base
        self.output_root = arguments.output_root.resolve()
        expected_output_root = (self.workspace / 'artifacts/evidence/phase3-benchmarks').resolve()
        if self.output_root != expected_output_root:
            raise EvidenceError(f'output root must be the frozen path {expected_output_root}')
        self.candidate_root = self.output_root / self.candidate_id
        self.build_binding_source = arguments.build_binding.resolve()
        self.authorization = arguments.authorization
        self.plan_path = self.candidate_root / 'suite-plan.json'
        self.binding_path = self.candidate_root / 'build-binding.json'
        self._active_registry: ProcessRegistry | None = None

    def prepare(self) -> None:
        _require_exact_affinity()
        _new_directory(self.candidate_root)
        head, status = _git_state(self.workspace)
        if status:
            raise EvidenceError('Phase 3 candidate preparation requires a clean Git worktree')
        source_binding = load_json(self.build_binding_source)
        validate_build_binding(
            self.workspace,
            source_binding,
            git_sha=head,
            git_status_porcelain=status,
        )
        plan = suite_document(self.workspace, self.candidate_id, self.domain_base)
        atomic_write_json(self.plan_path, plan, sidecar=True)
        atomic_write_json(self.binding_path, source_binding, sidecar=True)
        _write_marker(
            self.candidate_root / 'prepared.json',
            {
                'build_binding_sha256': file_sha256(self.binding_path),
                'candidate_id': self.candidate_id,
                'git_sha': head,
                'producer': PRODUCER,
                'suite_plan_sha256': file_sha256(self.plan_path),
            },
        )

    def _load_state(self) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
        verify_json_sidecar(self.plan_path)
        verify_json_sidecar(self.binding_path)
        plan = load_json(self.plan_path)
        binding = load_json(self.binding_path)
        head, status = _git_state(self.workspace)
        if status:
            raise EvidenceError('Phase 3 runtime requires a clean Git worktree')
        validate_build_binding(self.workspace, binding, git_sha=head, git_status_porcelain=status)
        if plan != suite_document(self.workspace, self.candidate_id, self.domain_base):
            raise EvidenceError('suite plan differs from the frozen workspace inputs')
        return plan, binding

    def positive_control(self) -> None:
        _require_exact_affinity()
        plan, binding = self._load_state()
        stage = plan['positive_control']
        run_dir = self.candidate_root / 'positive-control'
        _new_directory(run_dir)
        env = _environment(stage['ros_domain_id'], stage['gz_partition'])
        registry = ProcessRegistry(run_dir / 'processes', self.workspace, env)
        self._active_registry = registry
        sampler = ResourceSampler(registry, run_dir / 'resources.jsonl')
        sampler.start()
        cleanup_ok = False
        try:
            registry.run_checked(
                'domain_preflight',
                _runtime_gate_command(
                    self.workspace,
                    'empty',
                    run_dir / 'domain-preflight.json',
                    watch_pid=None,
                    launch_pid=None,
                    expected_domain_id=None,
                    expected_gz_partition=None,
                    wall_timeout_s=10.0,
                ),
                wall_timeout_s=15.0,
                stage='positive_preflight',
            )
            gz_preflight = registry.run_checked(
                'partition_preflight',
                ['gz', 'topic', '-l'],
                wall_timeout_s=15.0,
                stage='positive_preflight',
            )
            _require_empty_stdout(
                gz_preflight,
                stage='positive_preflight',
                subject='positive-control Gazebo partition',
            )
            launch = registry.start(
                'sim_launch',
                [
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
                wall_timeout_s=120.0,
            )
            collector_ready = run_dir / 'metrics.ready.json'
            collector_stop = run_dir / 'metrics.stop'
            contact_progress_path = run_dir / 'contact-progress.json'
            capture_path = run_dir / 'capture.json'
            collector = registry.start(
                'metrics_collector',
                _metrics_collector_command(
                    capture_path=capture_path,
                    ready_path=collector_ready,
                    stop_path=collector_stop,
                    contact_progress_path=contact_progress_path,
                ),
                wall_timeout_s=370.0,
            )
            _wait_for_file(
                collector_ready,
                timeout_s=30.0,
                watched=(launch, collector),
                stage='positive_collector_ready',
            )
            result_path = run_dir / 'contact-control-result.json'
            driver_ready = run_dir / 'contact-control.ready.json'
            driver = registry.start(
                'contact_control_driver',
                [
                    'ros2',
                    'run',
                    'robotest_scenarios',
                    'contact_control_driver',
                    '--output',
                    str(result_path),
                    '--ready-file',
                    str(driver_ready),
                    '--run-id',
                    stage['run_id'],
                    '--coverage-manifest',
                    str(self.workspace / 'config/collision-coverage.yaml'),
                    '--wall-timeout-s',
                    str(CONTACT_CONTROL_WALL_TIMEOUT_S),
                    '--ros-args',
                    '-r',
                    '__ns:=/robotest',
                ],
                wall_timeout_s=45.0,
            )
            _wait_for_file(
                driver_ready,
                timeout_s=20.0,
                watched=(launch, collector, driver),
                stage='positive_driver_ready',
            )
            registry.run_checked(
                'runtime_gate',
                _runtime_gate_command(
                    self.workspace,
                    'positive-control',
                    run_dir / 'runtime-gate.json',
                    watch_pid=driver.pid,
                    launch_pid=launch.pid,
                    expected_domain_id=stage['ros_domain_id'],
                    expected_gz_partition=stage['gz_partition'],
                    wall_timeout_s=20.0,
                ),
                wall_timeout_s=25.0,
                stage='positive_runtime_gate',
            )
            driver_status = driver.wait(50.0)
            if driver_status != 0:
                raise StageFailure(
                    'positive_control',
                    'component_exit',
                    f'contact control driver exited {driver_status}',
                    exit_code=driver_status,
                    evidence=load_json(driver.metadata_path),
                )
            verify_json_sidecar(result_path)
            positive_qualifying_stamp_ns = positive_control_qualified_snapshot_stamp(
                load_json(result_path)
            )
            _wait_for_contact_progress(
                contact_progress_path,
                qualifying_stamp_ns=positive_qualifying_stamp_ns,
                timeout_s=5.0,
                watched=(launch, collector),
                stage='positive_contact_retention',
            )
            registry.run_checked(
                'contact_stream_final_gate',
                _runtime_gate_command(
                    self.workspace,
                    'contact-stream',
                    run_dir / 'contact-stream-final-gate.json',
                    watch_pid=launch.pid,
                    launch_pid=launch.pid,
                    expected_domain_id=stage['ros_domain_id'],
                    expected_gz_partition=stage['gz_partition'],
                    wall_timeout_s=10.0,
                ),
                wall_timeout_s=15.0,
                stage='positive_contact_stream_final_gate',
            )
            atomic_write_json(
                run_dir / 'contact-gate-revalidation.json',
                reconcile_contact_gate_reobservation(
                    run_dir / 'runtime-gate.json',
                    run_dir / 'contact-stream-final-gate.json',
                    build_binding=binding,
                    expected_domain_id=stage['ros_domain_id'],
                    expected_gz_partition=stage['gz_partition'],
                ),
                sidecar=True,
            )
            atomic_write_bytes(collector_stop, b'positive-control-complete\n', 256)
            collector_status = collector.wait(30.0)
            if collector_status != 0:
                raise StageFailure(
                    'positive_collector',
                    'component_exit',
                    f'metrics collector exited {collector_status}',
                    exit_code=collector_status,
                    evidence=load_json(collector.metadata_path),
                )
            cleanup_ok = registry.stop_all()
            registry.run_checked(
                'domain_cleanup',
                _runtime_gate_command(
                    self.workspace,
                    'empty',
                    run_dir / 'domain-cleanup.json',
                    watch_pid=None,
                    launch_pid=None,
                    expected_domain_id=None,
                    expected_gz_partition=None,
                    wall_timeout_s=15.0,
                ),
                wall_timeout_s=20.0,
                stage='positive_cleanup',
            )
            gz_cleanup = registry.run_checked(
                'partition_cleanup',
                ['gz', 'topic', '-l'],
                wall_timeout_s=15.0,
                stage='positive_cleanup',
            )
            _require_empty_stdout(
                gz_cleanup,
                stage='positive_cleanup',
                subject='positive-control Gazebo partition after shutdown',
            )
            cleanup_ok = registry.stop_all() and cleanup_ok
            sampler.stop_after_shutdown()
            resources = summarize_resources(run_dir / 'resources.jsonl')
            if resources['peak_rss_sum_bytes'] > 6 * 1024**3 or resources['oom_kill']:
                raise StageFailure(
                    'positive_resources',
                    'resource_gate',
                    'positive-control resource gate failed',
                    evidence=resources,
                )
            end_head, end_status = _git_state(self.workspace)
            end = build_binding(
                self.workspace,
                git_sha=end_head,
                git_status_porcelain=end_status,
            )
            if end != binding:
                raise StageFailure(
                    'positive_provenance',
                    'source_mutation',
                    'source/install binding changed during positive control',
                    evidence={'start': binding, 'end': end},
                )
            checksum_document = component_manifest(
                sorted(item for item in run_dir.rglob('*') if item.is_file()),
                run_dir,
            )
            atomic_write_json(run_dir / 'component-manifest.json', checksum_document, sidecar=True)
            checksum_verified = verify_component_manifest(checksum_document, run_dir)
            positive = reconcile_positive_control(
                workspace=self.workspace,
                build_binding=binding,
                result_path=result_path,
                capture_path=capture_path,
                contact_progress_path=contact_progress_path,
                manifest_path=self.workspace / 'config/collision-coverage.yaml',
                collector_configuration_sha256=binding['collector_configuration_sha256'],
                owned_process_group_shutdown=cleanup_ok,
                checksum_verified=checksum_verified,
            )
            atomic_write_json(run_dir / 'positive-binding.json', positive, sidecar=True)
            _write_marker(
                run_dir / 'PASS.json',
                {
                    'positive_binding_sha256': file_sha256(run_dir / 'positive-binding.json'),
                    'producer': PRODUCER,
                    'resource_summary': resources,
                    'status': 'PASS',
                },
            )
        finally:
            cleanup_ok = registry.stop_all() and cleanup_ok
            if sampler._thread is not None and sampler._thread.is_alive():
                sampler.stop_after_shutdown()
            self._active_registry = None

    def smoke(self) -> None:
        plan, binding = self._load_state()
        positive = self._load_positive()
        smoke = dict(plan['trials'][0])
        smoke_candidate = f'{self.candidate_id}-smoke'
        smoke.update(
            {
                'candidate_id': smoke_candidate,
                'gz_partition': f'robotest_p3_{smoke_candidate}_00',
                'ros_domain_id': plan['smoke']['ros_domain_id'],
                'run_id': plan['smoke']['run_id'],
            }
        )
        result_path = self._run_trial(smoke, binding, positive, self.candidate_root / 'smoke')
        result = load_json(result_path)
        if result.get('verdict', {}).get('automated_status') != 'PASS':
            raise EvidenceError('single-scenario smoke did not PASS; campaign remains blocked')
        _write_marker(
            self.candidate_root / 'smoke' / 'PASS.json',
            {
                'producer': PRODUCER,
                'run_result_sha256': file_sha256(result_path),
                'status': 'PASS',
            },
        )

    def campaign(self) -> None:
        if self.authorization != CAMPAIGN_AUTHORIZATION:
            raise EvidenceError(
                'campaign mode requires the exact explicit --authorization literal shown in --help'
            )
        plan, binding = self._load_state()
        positive = self._load_positive()
        smoke_marker = self.candidate_root / 'smoke/PASS.json'
        verify_json_sidecar(smoke_marker)
        if load_json(smoke_marker).get('status') != 'PASS':
            raise EvidenceError('campaign requires a passing single-scenario smoke')
        run_results: list[Path] = []
        abort_failure: StageFailure | None = None
        for item in plan['trials']:
            run_dir = self.candidate_root / 'runs' / f'{item["suite_index"]:02d}'
            if abort_failure is None:
                try:
                    result_path = self._run_trial(item, binding, positive, run_dir)
                except StageFailure as exc:
                    abort_failure = exc
                    result_path = self._emit_unstarted_failure(
                        item, binding, positive, run_dir, exc, allow_existing=True
                    )
            else:
                result_path = self._emit_unstarted_failure(
                    item,
                    binding,
                    positive,
                    run_dir,
                    StageFailure(
                        'campaign_aborted',
                        'unsafe_prior_cleanup',
                        'trial not started after an unsafe prior cleanup condition',
                        evidence=abort_failure.document(),
                    ),
                )
            run_results.append(result_path)
        if len(run_results) != 15:
            raise EvidenceError('campaign did not retain exactly 15 ordered run results')
        aggregate_dir = self.candidate_root / 'aggregate'
        _new_directory(aggregate_dir)
        command = ['ros2', 'run', 'robotest_metrics', 'metrics_aggregate']
        for path in run_results:
            command.extend(['--input', str(path)])
        for metric in AGGREGATE_METRICS:
            command.extend(['--metric', metric])
        command.extend(['--output-dir', str(aggregate_dir)])
        registry = ProcessRegistry(
            aggregate_dir / 'processes',
            self.workspace,
            _environment(self.domain_base, f'robotest_p3_{self.candidate_id}_aggregate'),
        )
        self._active_registry = registry
        aggregate_cleanup_ok = False
        try:
            process = registry.start('metrics_aggregate', command, wall_timeout_s=120.0)
            status = process.wait(130.0)
        finally:
            aggregate_cleanup_ok = registry.stop_all()
            self._active_registry = None
        _require_campaign_aggregate_pass(
            status,
            aggregate_dir / 'aggregate-result.json',
            cleanup_ok=aggregate_cleanup_ok,
        )

    def _load_positive(self) -> Mapping[str, Any]:
        marker = self.candidate_root / 'positive-control/PASS.json'
        binding_path = self.candidate_root / 'positive-control/positive-binding.json'
        verify_json_sidecar(marker)
        verify_json_sidecar(binding_path)
        if load_json(marker).get('status') != 'PASS':
            raise EvidenceError('positive-control marker is not PASS')
        return load_json(binding_path)

    def _run_trial(
        self,
        plan: Mapping[str, Any],
        build_start: Mapping[str, Any],
        positive: Mapping[str, Any],
        run_dir: Path,
    ) -> Path:
        _require_exact_affinity()
        _new_directory(run_dir)
        env = _environment(plan['ros_domain_id'], plan['gz_partition'])
        registry = ProcessRegistry(run_dir / 'processes', self.workspace, env)
        self._active_registry = registry
        sampler = ResourceSampler(registry, run_dir / 'resources.jsonl')
        context_path = run_dir / 'trial-context.json'
        context = make_trial_context(
            plan,
            workspace=self.workspace,
            git_sha=build_start['git']['sha'],
            build=build_start,
            positive=positive,
            failure=None,
        )
        atomic_write_json(context_path, context, sidecar=True)
        context_start_hash = file_sha256(context_path)
        sampler.start()
        cleanup_ok = False
        stage_failure: StageFailure | None = None
        lifecycle_process: BoundedProcess | None = None
        mission: BoundedProcess | None = None
        scenario: BoundedProcess | None = None
        collector: BoundedProcess | None = None
        launch: BoundedProcess | None = None
        try:
            registry.run_checked(
                'domain_preflight',
                _runtime_gate_command(
                    self.workspace,
                    'empty',
                    run_dir / 'domain-preflight.json',
                    watch_pid=None,
                    launch_pid=None,
                    expected_domain_id=None,
                    expected_gz_partition=None,
                    wall_timeout_s=10.0,
                ),
                wall_timeout_s=15.0,
                stage='domain_preflight',
            )
            gz_preflight = registry.run_checked(
                'partition_preflight',
                ['gz', 'topic', '-l'],
                wall_timeout_s=15.0,
                stage='domain_preflight',
            )
            _require_empty_stdout(
                gz_preflight,
                stage='domain_preflight',
                subject='Gazebo partition',
            )
            startup_result = run_dir / 'lifecycle-startup-result.json'
            launch = registry.start(
                'full_stack',
                [
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
                    f'lifecycle_startup_result_path:={startup_result}',
                ],
                wall_timeout_s=430.0,
            )
            registry.run_checked(
                'startup_gate',
                [
                    'python3',
                    str(self.workspace / 'tests/phase2_startup_gate.py'),
                    '--result',
                    str(startup_result),
                    '--output',
                    str(run_dir / 'startup-gate.json'),
                    '--watch-pid',
                    str(launch.pid),
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
                wall_timeout_s=120.0,
                stage='startup_gate',
            )
            lifecycle_args: list[str] = []
            for name in (
                'map_server',
                'amcl',
                'planner_server',
                'controller_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
                'collision_monitor',
            ):
                lifecycle_args.extend(['--node', name])
            registry.run_checked(
                'lifecycle_gate',
                [
                    'python3',
                    str(self.workspace / 'tests/phase2_lifecycle_probe.py'),
                    '--namespace',
                    '/robotest',
                    '--output',
                    str(run_dir / 'lifecycle-ready.json'),
                    '--text-dir',
                    str(run_dir),
                    '--text-prefix',
                    'lifecycle-ready-',
                    '--wall-timeout',
                    '110',
                    '--watch-pid',
                    str(launch.pid),
                    *lifecycle_args,
                ],
                wall_timeout_s=120.0,
                stage='lifecycle_gate',
            )
            collector_ready = run_dir / 'metrics.ready.json'
            collector_stop = run_dir / 'metrics.stop'
            contact_progress_path = run_dir / 'contact-progress.json'
            capture_path = run_dir / 'capture.json'
            collector = registry.start(
                'metrics_collector',
                _metrics_collector_command(
                    capture_path=capture_path,
                    ready_path=collector_ready,
                    stop_path=collector_stop,
                    contact_progress_path=contact_progress_path,
                ),
                wall_timeout_s=370.0,
            )
            _wait_for_file(
                collector_ready,
                timeout_s=30.0,
                watched=(launch, collector),
                stage='collector_ready',
            )
            scenario_result = run_dir / 'scenario-result.json'
            scenario_ready = run_dir / 'scenario.ready.json'
            scenario = registry.start(
                'scenario_controller',
                [
                    'ros2',
                    'run',
                    'robotest_scenarios',
                    'scenario_controller',
                    '--scenario',
                    str(self.workspace / plan['scenario_path']),
                    '--output',
                    str(scenario_result),
                    '--ready-file',
                    str(scenario_ready),
                    '--run-id',
                    plan['run_id'],
                    '--candidate-id',
                    plan['candidate_id'],
                    '--repetition-index',
                    str(plan['repetition_index']),
                    '--suite-index',
                    str(plan['suite_index']),
                    '--ros-args',
                    '-r',
                    '__ns:=/robotest',
                ],
                wall_timeout_s=310.0,
            )
            _wait_for_file(
                scenario_ready,
                timeout_s=60.0,
                watched=(launch, collector, scenario),
                stage='scenario_ready',
            )
            observer_ready = run_dir / 'goal-observer.ready.json'
            observer_arm = run_dir / 'goal-observer.arm'
            observer_armed = run_dir / 'goal-observer.armed.json'
            observer_result = run_dir / 'goal-observer.json'
            schedule_path = run_dir / 'lifecycle-schedule.json'
            observer_command = [
                'python3',
                str(self.workspace / 'tests/phase3_runtime_observer.py'),
                'observe-goal',
                '--run-id',
                plan['run_id'],
                '--ready-file',
                str(observer_ready),
                '--arm-file',
                str(observer_arm),
                '--armed-file',
                str(observer_armed),
                '--output',
                str(observer_result),
                '--watch-pid',
                str(launch.pid),
                '--wall-timeout-s',
                '300',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ]
            if plan['scenario_id'] == 4:
                observer_command.extend(['--schedule-output', str(schedule_path)])
            observer = registry.start('goal_observer', observer_command, wall_timeout_s=310.0)
            _wait_for_file(
                observer_ready,
                timeout_s=20.0,
                watched=(launch, collector, scenario, observer),
                stage='goal_observer_ready',
            )
            registry.run_checked(
                'runtime_gate',
                _runtime_gate_command(
                    self.workspace,
                    'candidate',
                    run_dir / 'runtime-gate.json',
                    watch_pid=launch.pid,
                    launch_pid=launch.pid,
                    expected_domain_id=plan['ros_domain_id'],
                    expected_gz_partition=plan['gz_partition'],
                    wall_timeout_s=60.0,
                ),
                wall_timeout_s=70.0,
                stage='runtime_gate',
            )
            registry.run_checked(
                'graph_gate',
                _graph_probe_command(
                    self.workspace, run_dir, watch_pid=launch.pid, mission_client=False
                ),
                wall_timeout_s=100.0,
                stage='graph_gate',
            )
            atomic_write_bytes(observer_arm, b'arm-next-new-goal\n', 256)
            _wait_for_file(
                observer_armed,
                timeout_s=20.0,
                watched=(launch, collector, scenario, observer),
                stage='goal_observer_armed',
            )
            try:
                _validate_goal_observer_armed(
                    load_json(observer_armed),
                    load_json(observer_ready),
                )
            except EvidenceError as exc:
                raise StageFailure(
                    'goal_observer_armed',
                    'invalid_evidence',
                    str(exc),
                    evidence=load_json(observer_armed),
                ) from exc
            mission_result = run_dir / 'mission-result.json'
            mission_csv = run_dir / 'mission-result.csv'
            mission = registry.start(
                'mission_runner',
                [
                    'ros2',
                    'run',
                    'robotest_missions',
                    'mission_runner',
                    '--mission',
                    str(self.workspace / plan['scenario_path']),
                    '--json',
                    str(mission_result),
                    '--csv',
                    str(mission_csv),
                    '--run-id',
                    plan['run_id'],
                    '--candidate-id',
                    plan['candidate_id'],
                    '--repetition-index',
                    str(plan['repetition_index']),
                    '--suite-index',
                    str(plan['suite_index']),
                    '--ros-args',
                    '-r',
                    '__ns:=/robotest',
                ],
                wall_timeout_s=310.0,
            )
            _wait_for_file(
                observer_result,
                timeout_s=30.0,
                watched=(launch, collector, scenario, observer, mission),
                stage='goal_binding',
            )
            observer_status = observer.wait(5.0)
            if observer_status != 0:
                raise StageFailure(
                    'goal_binding',
                    'observer_exit',
                    f'goal observer exited {observer_status}',
                    exit_code=observer_status,
                    evidence=load_json(observer.metadata_path),
                )
            if plan['scenario_id'] == 4:
                lifecycle_ready = run_dir / 'lifecycle-sampler.ready.json'
                lifecycle_stop = run_dir / 'lifecycle-sampler.stop'
                lifecycle_process = registry.start(
                    'lifecycle_sampler',
                    [
                        'ros2',
                        'run',
                        'robotest_metrics',
                        'metrics_lifecycle_sampler',
                        '--schedule',
                        str(schedule_path),
                        '--output',
                        str(run_dir / 'lifecycle-snapshot.json'),
                        '--ready-file',
                        str(lifecycle_ready),
                        '--stop-file',
                        str(lifecycle_stop),
                        '--wall-timeout-s',
                        '360',
                        '--request-timeout-s',
                        '2',
                        '--ros-args',
                        '-r',
                        '__ns:=/robotest',
                    ],
                    wall_timeout_s=370.0,
                )
                _wait_for_file(
                    lifecycle_ready,
                    timeout_s=20.0,
                    watched=(launch, collector, scenario, mission, lifecycle_process),
                    stage='lifecycle_sampler_ready',
                )
            registry.run_checked(
                'mission_graph_gate',
                _graph_probe_command(
                    self.workspace, run_dir, watch_pid=mission.pid, mission_client=True
                ),
                wall_timeout_s=30.0,
                stage='mission_graph_gate',
            )
            mission_status = mission.wait(320.0)
            scenario_status = scenario.wait(30.0)
            atomic_write_json(
                run_dir / 'component-exits.json',
                {
                    'mission_exit_code': mission_status,
                    'scenario_exit_code': scenario_status,
                },
            )
            if not mission_result.is_file() or not scenario_result.is_file():
                raise StageFailure(
                    'goal_binding',
                    'missing_component_artifact',
                    'mission/scenario artifact is missing after component exit',
                    evidence={
                        'mission_result_exists': mission_result.is_file(),
                        'scenario_result_exists': scenario_result.is_file(),
                    },
                )
            verify_json_sidecar(observer_result)
            binding_evidence = reconcile_goal_binding(
                load_json(observer_result),
                load_json(mission_result),
                load_json(scenario_result),
            )
            atomic_write_json(run_dir / 'goal-binding-reconciliation.json', binding_evidence)
            contact_drain_path = run_dir / 'contact-drain.json'
            terminal: int
            if mission_result.is_file():
                mission_document = load_json(mission_result)
                terminal = mission_document.get('measurements', {}).get('terminal_action_stamp_ns')
                if isinstance(terminal, int) and terminal > 0:
                    registry.run_checked(
                        'contact_drain',
                        [
                            'python3',
                            str(self.workspace / 'tests/phase3_runtime_observer.py'),
                            'wait-contact-drain',
                            '--terminal-action-stamp-ns',
                            str(terminal),
                            '--output',
                            str(contact_drain_path),
                            '--watch-pid',
                            str(launch.pid),
                            '--wall-timeout-s',
                            '30',
                            '--ros-args',
                            '-r',
                            '__ns:=/robotest',
                        ],
                        wall_timeout_s=35.0,
                        stage='contact_drain',
                    )
                    verify_json_sidecar(contact_drain_path)
                    drain_document = load_json(contact_drain_path)
                    qualifying_key = 'qualifying_contact_snapshot_stamp_ns'
                    qualifying_stamp_ns = drain_document.get(qualifying_key)
                    if (
                        isinstance(qualifying_stamp_ns, bool)
                        or not isinstance(qualifying_stamp_ns, int)
                        or qualifying_stamp_ns <= terminal + CONTACT_DRAIN_NS
                    ):
                        raise StageFailure(
                            'contact_drain',
                            'invalid_qualifying_stamp',
                            'contact drain artifact lacks a strict qualifying snapshot stamp',
                            evidence=drain_document,
                        )
                    _wait_for_contact_progress(
                        contact_progress_path,
                        qualifying_stamp_ns=qualifying_stamp_ns,
                        timeout_s=5.0,
                        watched=(launch, collector),
                    )
                    registry.run_checked(
                        'contact_stream_final_gate',
                        _runtime_gate_command(
                            self.workspace,
                            'contact-stream',
                            run_dir / 'contact-stream-final-gate.json',
                            watch_pid=launch.pid,
                            launch_pid=launch.pid,
                            expected_domain_id=plan['ros_domain_id'],
                            expected_gz_partition=plan['gz_partition'],
                            wall_timeout_s=10.0,
                        ),
                        wall_timeout_s=15.0,
                        stage='contact_stream_final_gate',
                    )
                    atomic_write_json(
                        run_dir / 'contact-gate-revalidation.json',
                        reconcile_contact_gate_reobservation(
                            run_dir / 'runtime-gate.json',
                            run_dir / 'contact-stream-final-gate.json',
                            build_binding=build_start,
                            expected_domain_id=plan['ros_domain_id'],
                            expected_gz_partition=plan['gz_partition'],
                        ),
                        sidecar=True,
                    )
                else:
                    raise StageFailure(
                        'contact_drain',
                        'missing_terminal_stamp',
                        'mission artifact lacks a positive terminal action stamp',
                        evidence=mission_document,
                    )
            else:
                raise StageFailure(
                    'contact_drain',
                    'missing_mission_artifact',
                    'mission result is absent, so terminal drain cannot be proven',
                )
            atomic_write_bytes(collector_stop, b'terminal-contact-drain-complete\n', 256)
            collector_status = collector.wait(30.0)
            if lifecycle_process is not None:
                atomic_write_bytes(run_dir / 'lifecycle-sampler.stop', b'mission-complete\n', 256)
                lifecycle_status = lifecycle_process.wait(30.0)
                if lifecycle_status != 0:
                    raise StageFailure(
                        'lifecycle_sampler',
                        'component_exit',
                        f'lifecycle sampler exited {lifecycle_status}',
                        exit_code=lifecycle_status,
                        evidence=load_json(lifecycle_process.metadata_path),
                    )
            if collector_status != 0:
                raise StageFailure(
                    'metrics_collector',
                    'component_exit',
                    f'metrics collector exited {collector_status}',
                    exit_code=collector_status,
                    evidence=load_json(collector.metadata_path),
                )
            try:
                validate_contact_drain_evidence(
                    load_json(contact_drain_path),
                    terminal_action_stamp_ns=terminal,
                    capture=load_json(capture_path),
                )
            except EvidenceError as exc:
                raise StageFailure(
                    'contact_drain',
                    'collector_retention',
                    str(exc),
                    evidence=load_json(contact_drain_path),
                ) from exc
        except StageFailure as exc:
            stage_failure = exc
        except (
            EvidenceError,
            KeyError,
            OSError,
            subprocess.SubprocessError,
            TypeError,
            ValueError,
        ) as exc:
            stage_failure = StageFailure(
                'orchestration',
                'evidence_failure',
                f'{type(exc).__name__}: {exc}',
                exit_code=2,
                evidence={'exception_type': type(exc).__name__},
            )
        finally:
            cleanup_ok = registry.stop_all()
            try:
                registry.run_checked(
                    'domain_cleanup',
                    _runtime_gate_command(
                        self.workspace,
                        'empty',
                        run_dir / 'domain-cleanup.json',
                        watch_pid=None,
                        launch_pid=None,
                        expected_domain_id=None,
                        expected_gz_partition=None,
                        wall_timeout_s=15.0,
                    ),
                    wall_timeout_s=20.0,
                    stage='domain_cleanup',
                )
                gz_cleanup = registry.run_checked(
                    'partition_cleanup',
                    ['gz', 'topic', '-l'],
                    wall_timeout_s=15.0,
                    stage='domain_cleanup',
                )
                _require_empty_stdout(
                    gz_cleanup,
                    stage='domain_cleanup',
                    subject='Gazebo partition after shutdown',
                )
                cleanup_ok = registry.stop_all() and cleanup_ok
            except (
                EvidenceError,
                OSError,
                subprocess.SubprocessError,
            ) as cleanup_error:
                cleanup_ok = False
                if stage_failure is None:
                    stage_failure = (
                        cleanup_error
                        if isinstance(cleanup_error, StageFailure)
                        else StageFailure(
                            'domain_cleanup',
                            'cleanup_error',
                            f'{type(cleanup_error).__name__}: {cleanup_error}',
                            evidence={'exception_type': type(cleanup_error).__name__},
                        )
                    )
            try:
                sampler.stop_after_shutdown()
            except EvidenceError as exc:
                if stage_failure is None:
                    stage_failure = StageFailure(
                        'resource_sampler',
                        'sampler_failure',
                        str(exc),
                        evidence={'failure': sampler.failure},
                    )
            self._active_registry = None
        if not cleanup_ok:
            raise StageFailure(
                'domain_cleanup',
                'unsafe_cleanup',
                'one or more owned process groups/endpoints survived bounded cleanup',
                evidence=[load_json(item.metadata_path) for item in registry.snapshot()],
            )
        if not registry.logs_within_cap():
            stage_failure = stage_failure or StageFailure(
                'artifact_gate',
                'log_overflow',
                'one or more runtime logs exceeded 8 MiB',
            )
        try:
            end_head, end_status = _git_state(self.workspace)
            build_end = build_binding(
                self.workspace,
                git_sha=end_head,
                git_status_porcelain=end_status,
            )
        except (EvidenceError, OSError, subprocess.SubprocessError) as exc:
            raise StageFailure(
                'source_binding',
                'binding_error',
                f'cannot recompute the source/install binding: {exc}',
                evidence={'exception_type': type(exc).__name__},
            ) from exc
        if build_end != build_start:
            raise StageFailure(
                'source_binding',
                'source_mutation',
                'source or installed overlay changed during the trial',
                evidence={'start': build_start, 'end': build_end},
            )
        if stage_failure is not None:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                stage_failure,
                initial_context_sha256=context_start_hash,
            )
        try:
            resources = summarize_resources(run_dir / 'resources.jsonl')
        except EvidenceError as exc:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                StageFailure(
                    'artifact_gate',
                    'resource_evidence_invalid',
                    str(exc),
                    evidence={'resource_trace': str(run_dir / 'resources.jsonl')},
                ),
                initial_context_sha256=context_start_hash,
            )
        core_paths = sorted(item for item in run_dir.rglob('*') if item.is_file())
        manifest_path = run_dir / 'prerequisite-manifest.json'
        try:
            manifest = component_manifest(core_paths, run_dir)
            atomic_write_json(manifest_path, manifest, sidecar=True)
            verify_component_manifest(manifest, run_dir)
        except (EvidenceError, OSError) as exc:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                StageFailure(
                    'artifact_gate',
                    'prerequisite_finalization',
                    str(exc),
                    evidence={'exception_type': type(exc).__name__},
                ),
                initial_context_sha256=context_start_hash,
            )
        assert mission is not None
        if mission.finished_steady_ns is None or mission.returncode is None:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                StageFailure(
                    'artifact_gate',
                    'mission_process_evidence',
                    'mission process metadata was not finalized',
                    evidence={'mission_metadata': str(mission.metadata_path)},
                ),
                initial_context_sha256=context_start_hash,
            )
        execution_duration = (
            mission.finished_steady_ns - mission.started_steady_ns
        ) / 1_000_000_000
        try:
            orchestrator = make_orchestrator_evidence(
                plan=plan,
                build_start=build_start,
                build_end=build_end,
                git_sha=build_start['git']['sha'],
                git_status_porcelain='',
                resource_summary=resources,
                execution={
                    'command': shlex.join(mission.command),
                    'exit_code': mission.returncode,
                    'wall_duration_s': execution_duration,
                    'wall_timed_out': mission.timed_out or mission.returncode == 124,
                    'wall_timeout_s': 300.0,
                    'working_directory': str(self.workspace),
                },
                process={
                    'cold_stack': True,
                    'fresh_fault_generation': True,
                    'fresh_localization': True,
                    'new_process_group': True,
                    'partition_unused_before_start': True,
                    'previous_trial_gone': True,
                    'ros_domain_unused_before_start': True,
                },
                cleanup={
                    'all_owned_processes_exited': True,
                    'discovery_endpoints_gone': True,
                    'no_orphans': True,
                },
                gates={
                    'graph_contract_pass': True,
                    'namespace_isolation_pass': True,
                    'qos_contract_pass': True,
                    'source_install_binding_pass': True,
                    'validation_autonomy_isolation_pass': True,
                },
                run_dir=run_dir,
                component_manifest_sha256=file_sha256(manifest_path),
            )
        except (EvidenceError, OSError) as exc:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                StageFailure(
                    'artifact_gate',
                    'orchestrator_finalization',
                    str(exc),
                    evidence={'exception_type': type(exc).__name__},
                ),
                initial_context_sha256=context_start_hash,
            )
        orchestrator_path = run_dir / 'orchestrator.json'
        atomic_write_json(orchestrator_path, orchestrator, sidecar=True)
        request_path = run_dir / 'analysis-request.json'
        try:
            mission_document = load_json(run_dir / 'mission-result.json')
            request = compose_analysis_request(
                workspace=self.workspace,
                plan=plan,
                mission_path=run_dir / 'mission-result.json',
                scenario_path=run_dir / 'scenario-result.json',
                capture_path=run_dir / 'capture.json',
                positive_binding_path=self.candidate_root
                / 'positive-control/positive-binding.json',
                orchestrator_path=orchestrator_path,
                contact_drain_path=run_dir / 'contact-drain.json',
                contact_progress_path=run_dir / 'contact-progress.json',
                lifecycle_snapshot_path=(
                    run_dir / 'lifecycle-snapshot.json' if plan['scenario_id'] == 4 else None
                ),
            )
            atomic_write_json(request_path, request, sidecar=True)
        except (EvidenceError, KeyError, OSError, TypeError, ValueError) as exc:
            return self._analyze_failure(
                plan,
                build_start,
                positive,
                run_dir,
                context_path,
                StageFailure(
                    'analysis_request',
                    'composition_failure',
                    f'{type(exc).__name__}: {exc}',
                    evidence={'exception_type': type(exc).__name__},
                ),
                initial_context_sha256=context_start_hash,
            )
        return self._analyze(context_path, request_path, run_dir)

    def _analyze_failure(
        self,
        plan: Mapping[str, Any],
        build: Mapping[str, Any],
        positive: Mapping[str, Any],
        run_dir: Path,
        context_path: Path,
        failure: StageFailure,
        *,
        initial_context_sha256: str | None = None,
    ) -> Path:
        """Finalize one trusted direct-failure context through metrics."""
        context = make_trial_context(
            plan,
            workspace=self.workspace,
            git_sha=build['git']['sha'],
            build=build,
            positive=positive,
            failure=failure.document(),
        )
        atomic_write_json(context_path, context, sidecar=True)
        if initial_context_sha256 is not None:
            atomic_write_json(
                run_dir / 'context-transition.json',
                {
                    'initial_null_failure_context_sha256': initial_context_sha256,
                    'terminal_failure_context_sha256': file_sha256(context_path),
                },
            )
        return self._analyze(context_path, None, run_dir)

    def _analyze(self, context_path: Path, request_path: Path | None, run_dir: Path) -> Path:
        result_dir = run_dir / 'result'
        _new_directory(result_dir)
        command = [
            'ros2',
            'run',
            'robotest_metrics',
            'metrics_analyze',
            '--trial-context',
            str(context_path),
        ]
        if request_path is not None:
            command.extend(['--input', str(request_path)])
        command.extend(['--output-dir', str(result_dir)])
        registry = ProcessRegistry(
            run_dir / 'analysis-process',
            self.workspace,
            _environment(0, 'robotest_phase3_offline_analysis'),
        )
        self._active_registry = registry
        try:
            process = registry.start('metrics_analyze', command, wall_timeout_s=120.0)
            status = process.wait(130.0)
        finally:
            cleanup = registry.stop_all()
            self._active_registry = None
        result_path = result_dir / 'run-result.json'
        if status not in (0, 30, 31, 32) or not cleanup or not result_path.is_file():
            raise StageFailure(
                'metrics_analyze',
                'canonical_result_missing',
                f'metrics analyzer exited {status} without a trusted canonical result',
                exit_code=status,
                evidence=load_json(process.metadata_path),
            )
        manifest_path = result_dir / 'run-artifacts.manifest.json'
        verify_json_sidecar(manifest_path)
        return result_path

    def _emit_unstarted_failure(
        self,
        plan: Mapping[str, Any],
        build: Mapping[str, Any],
        positive: Mapping[str, Any],
        run_dir: Path,
        failure: StageFailure,
        *,
        allow_existing: bool = False,
    ) -> Path:
        if allow_existing:
            run_dir.mkdir(parents=True, exist_ok=True)
            if (run_dir / 'result').exists() and any((run_dir / 'result').iterdir()):
                raise EvidenceError('cannot replace an existing canonical trial result')
        else:
            _new_directory(run_dir)
        context = make_trial_context(
            plan,
            workspace=self.workspace,
            git_sha=build['git']['sha'],
            build=build,
            positive=positive,
            failure=failure.document(),
        )
        context_path = run_dir / 'trial-context.json'
        atomic_write_json(context_path, context, sidecar=True)
        return self._analyze(context_path, None, run_dir)

    def stop_active(self) -> None:
        if self._active_registry is not None:
            self._active_registry.stop_all()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--mode',
        required=True,
        choices=('prepare', 'positive-control', 'smoke', 'campaign'),
    )
    parser.add_argument('--workspace', required=True, type=Path)
    parser.add_argument('--candidate-id', required=True)
    parser.add_argument('--domain-base', required=True, type=int)
    parser.add_argument('--output-root', required=True, type=Path)
    parser.add_argument('--build-binding', required=True, type=Path)
    parser.add_argument(
        '--authorization',
        default='',
        help=f'required only for campaign; exact value: {CAMPAIGN_AUTHORIZATION}',
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run exactly one explicit orchestration stage."""
    arguments = _parser().parse_args(argv)
    runner = BenchmarkRunner(arguments)

    def terminate(_signum: int, _frame: Any) -> None:
        runner.stop_active()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, terminate)
    signal.signal(signal.SIGTERM, terminate)
    try:
        if arguments.mode == 'prepare':
            runner.prepare()
        elif arguments.mode == 'positive-control':
            runner.positive_control()
        elif arguments.mode == 'smoke':
            runner.smoke()
        else:
            runner.campaign()
        return 0
    except KeyboardInterrupt:
        runner.stop_active()
        return 130
    except (EvidenceError, OSError, subprocess.SubprocessError) as exc:
        runner.stop_active()
        print(f'phase3 benchmark runner error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
