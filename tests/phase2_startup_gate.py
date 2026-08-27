#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Fail closed on Phase 2 lifecycle startup and final launch-log evidence."""

from __future__ import annotations

import argparse
from collections.abc import Callable, Collection
from dataclasses import dataclass
from datetime import datetime, UTC
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import tempfile
import time
from typing import Any

SCHEMA_VERSION = 1
FINAL_LAUNCH_LOG_SCHEMA_VERSION = 2
EXPECTED_SERVICE_NAME = '/robotest/lifecycle_manager_navigation/manage_nodes'
EXPECTED_STARTUP_COMMAND = 0
DEFAULT_DISCOVERY_GRACE_SEC = 4.0
DEFAULT_SERVICE_TIMEOUT_SEC = 20.0
DEFAULT_RESPONSE_TIMEOUT_SEC = 60.0
DEFAULT_WALL_TIMEOUT_SEC = 110.0
POLL_PERIOD_SEC = 0.05
MAXIMUM_RESULT_BYTES = 64 * 1024
MAXIMUM_LIFECYCLE_READY_BYTES = 64 * 1024
MAXIMUM_LAUNCH_LOG_BYTES = 128 * 1024 * 1024
DEFAULT_MAXIMUM_LAUNCH_STREAM_BYTES = 8 * 1024 * 1024

EXIT_PASS = 0
EXIT_GATE_FAILURE = 1
EXIT_INTERNAL_ERROR = 3

RESULT_KEYS = frozenset(
    {
        'schema_version',
        'verdict',
        'accepted',
        'exit_code',
        'service_name',
        'command',
        'elapsed_wall_sec',
        'discovery_grace_sec',
        'service_timeout_sec',
        'response_timeout_sec',
        'watch_pid',
        'failure_kind',
        'failure_message',
        'started_utc',
        'completed_utc',
    }
)
_UTC_PATTERN = re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d{1,9})?Z$')
_LIFECYCLE_NODE_PATTERN = re.compile(r'^[A-Za-z0-9_][A-Za-z0-9_]*$')
_DDS_RESPONSE_TIMEOUT_PATTERN = re.compile(
    r'failed to send response[^\r\n]*?\(timeout\)', re.IGNORECASE
)
_RECOVERABLE_DDS_RESPONSE_TIMEOUT_PATTERN = re.compile(
    r'failed to send response to '
    r'(?P<service>/robotest/[A-Za-z0-9_][A-Za-z0-9_]*/get_state) '
    r'\(timeout\)'
)
_LIFECYCLE_READY_KEYS = frozenset(
    {
        'failure',
        'namespace',
        'required_state',
        'states',
        'verdict',
        'wall_timeout_s',
        'watch_pid',
    }
)
_LIFECYCLE_STATE_KEYS = frozenset(
    {
        'attempts',
        'label',
        'service',
        'service_seen',
        'state_id',
        'timed_out_attempts',
    }
)
LAUNCH_LOG_SIGNATURES = (
    (
        'lifecycle_startup_rejection',
        'Lifecycle STARTUP rejection',
        re.compile(
            r'(?:\bSTARTUP\b[^\r\n]*\breject(?:ed|ion)\b|'
            r'\breject(?:ed|ion)\b[^\r\n]*\bSTARTUP\b)',
            re.IGNORECASE,
        ),
    ),
    (
        'nav2_bringup_failure',
        'Nav2 failed to bring up all requested nodes',
        re.compile(r'Failed to bring up all requested nodes', re.IGNORECASE),
    ),
    (
        'lifecycle_async_send_request_failure',
        'Lifecycle service async request failure',
        re.compile(r'service client:\s*async_send_request failed', re.IGNORECASE),
    ),
    (
        'dds_response_timeout',
        'DDS failed to send a response before timeout',
        _DDS_RESPONSE_TIMEOUT_PATTERN,
    ),
    (
        'fatal_process_signature',
        'Fatal, segmentation-fault, or out-of-memory process signature',
        re.compile(
            r'(^|[^a-z])(fatal|segmentation fault|out of memory|oom-kill)([^a-z]|$)',
            re.IGNORECASE,
        ),
    ),
)


class GateError(RuntimeError):
    """Expected acceptance failure with a stable machine-readable kind."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


@dataclass(frozen=True)
class StartupContract:
    """Immutable values expected from the launched startup helper."""

    service_name: str = EXPECTED_SERVICE_NAME
    command: int = EXPECTED_STARTUP_COMMAND
    discovery_grace_sec: float = DEFAULT_DISCOVERY_GRACE_SEC
    service_timeout_sec: float = DEFAULT_SERVICE_TIMEOUT_SEC
    response_timeout_sec: float = DEFAULT_RESPONSE_TIMEOUT_SEC


def utc_now() -> str:
    """Return a stable second-resolution UTC timestamp."""
    return datetime.now(UTC).strftime('%Y-%m-%dT%H:%M:%SZ')


def serialize_result(result: dict[str, Any]) -> str:
    """Serialize canonical strict JSON without non-finite numbers."""
    return json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + '\n'


def atomic_write_text(path: Path, content: str) -> None:
    """Atomically replace one UTF-8/LF evidence file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, pending_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.pending',
        text=True,
    )
    pending = Path(pending_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
            descriptor = -1
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        pending.replace(path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        pending.unlink(missing_ok=True)


def _stable_regular_bytes(
    path: Path,
    *,
    maximum_bytes: int,
    allow_empty: bool,
    label: str,
    activity: str,
) -> bytes:
    """Read one bounded regular file while rejecting replacement races."""
    try:
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise GateError(
                'launch_log_invalid',
                f'{label} must be a regular non-symlink file',
            )
        minimum_bytes = 0 if allow_empty else 1
        if metadata.st_size < minimum_bytes or metadata.st_size > maximum_bytes:
            raise GateError(
                'launch_log_invalid',
                f'{label} size {metadata.st_size} is outside {minimum_bytes}..{maximum_bytes}',
            )
        payload = path.read_bytes()
        after = path.lstat()
        before_identity = (
            metadata.st_dev,
            metadata.st_ino,
            metadata.st_size,
            metadata.st_mtime_ns,
        )
        after_identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
        if (
            len(payload) != metadata.st_size
            or before_identity != after_identity
            or stat.S_ISLNK(after.st_mode)
        ):
            raise GateError('launch_log_invalid', f'{label} changed while {activity}')
        return payload
    except GateError:
        raise
    except OSError as error:
        raise GateError(
            'launch_log_invalid',
            f'cannot read {label}: {type(error).__name__}: {error}',
        ) from error


def combined_launch_log_bytes(
    stdout_path: Path,
    stderr_path: Path,
    *,
    maximum_stream_bytes: int = DEFAULT_MAXIMUM_LAUNCH_STREAM_BYTES,
) -> bytes:
    """Return deterministic bytes for finalized, separately drained launch streams."""
    if (
        isinstance(maximum_stream_bytes, bool)
        or not isinstance(maximum_stream_bytes, int)
        or maximum_stream_bytes <= 0
        or maximum_stream_bytes > MAXIMUM_LAUNCH_LOG_BYTES
    ):
        raise GateError(
            'launch_log_invalid',
            'maximum launch-stream bytes must be a positive bounded integer',
        )
    stdout = _stable_regular_bytes(
        stdout_path,
        maximum_bytes=maximum_stream_bytes,
        allow_empty=True,
        label='full_stack stdout log',
        activity='being combined',
    )
    stderr = _stable_regular_bytes(
        stderr_path,
        maximum_bytes=maximum_stream_bytes,
        allow_empty=True,
        label='full_stack stderr log',
        activity='being combined',
    )
    separator = b'\n' if stdout and stderr and not stdout.endswith(b'\n') else b''
    payload = stdout + separator + stderr
    if len(payload) > MAXIMUM_LAUNCH_LOG_BYTES:
        raise GateError(
            'launch_log_invalid',
            f'combined launch log exceeds {MAXIMUM_LAUNCH_LOG_BYTES} bytes',
        )
    return payload


def watched_process_alive(pid: int) -> bool:
    """Return whether a positive PID still identifies a live process."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _strict_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON key: {key}')
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ValueError(f'non-standard JSON numeric constant: {value}')


def _is_number(value: Any) -> bool:
    return type(value) in (int, float) and math.isfinite(float(value))


def _parse_utc(field: str, value: Any, failures: list[str]) -> datetime | None:
    if not isinstance(value, str) or _UTC_PATTERN.fullmatch(value) is None:
        failures.append(f'{field} must be an ISO-8601 UTC string ending in Z')
        return None
    try:
        return datetime.fromisoformat(value[:-1] + '+00:00')
    except ValueError:
        failures.append(f'{field} is not a valid calendar timestamp')
        return None


def validate_startup_result(result: Any, contract: StartupContract) -> list[str]:
    """Return every strict schema and acceptance-contract violation."""
    if not isinstance(result, dict):
        return ['startup result must be a JSON object']

    failures: list[str] = []
    observed_keys = set(result)
    missing = sorted(RESULT_KEYS - observed_keys)
    unexpected = sorted(observed_keys - RESULT_KEYS)
    if missing:
        failures.append(f'missing keys: {missing}')
    if unexpected:
        failures.append(f'unexpected keys: {unexpected}')
    if missing:
        return failures

    if type(result['schema_version']) is not int or result['schema_version'] != SCHEMA_VERSION:
        failures.append(f'schema_version must be integer {SCHEMA_VERSION}')
    if not isinstance(result['verdict'], str) or result['verdict'] not in {'PASS', 'FAIL'}:
        failures.append("verdict must be exactly 'PASS' or 'FAIL'")
    if type(result['accepted']) is not bool:
        failures.append('accepted must be a JSON boolean')
    if type(result['exit_code']) is not int or not 0 <= result['exit_code'] <= 255:
        failures.append('exit_code must be an integer from 0 through 255')
    if result['service_name'] != contract.service_name:
        failures.append(
            f'service_name must be {contract.service_name!r}; observed {result["service_name"]!r}'
        )
    if type(result['command']) is not int or result['command'] != contract.command:
        failures.append(f'command must be integer STARTUP value {contract.command}')

    elapsed = result['elapsed_wall_sec']
    if not _is_number(elapsed) or float(elapsed) < 0.0:
        failures.append('elapsed_wall_sec must be finite and nonnegative')
    for key, expected in (
        ('discovery_grace_sec', contract.discovery_grace_sec),
        ('service_timeout_sec', contract.service_timeout_sec),
        ('response_timeout_sec', contract.response_timeout_sec),
    ):
        value = result[key]
        if not _is_number(value) or float(value) <= 0.0:
            failures.append(f'{key} must be finite and positive')
        elif float(value) != expected:
            failures.append(f'{key} must equal configured value {expected}; observed {value}')

    watch_pid = result['watch_pid']
    if watch_pid is not None and (type(watch_pid) is not int or watch_pid <= 0):
        failures.append('watch_pid must be null or a positive integer')

    started = _parse_utc('started_utc', result['started_utc'], failures)
    completed = _parse_utc('completed_utc', result['completed_utc'], failures)
    if started is not None and completed is not None and completed < started:
        failures.append('completed_utc precedes started_utc')

    verdict = result['verdict']
    accepted = result['accepted']
    exit_code = result['exit_code']
    failure_kind = result['failure_kind']
    failure_message = result['failure_message']
    if verdict == 'PASS':
        if accepted is not True:
            failures.append('PASS requires accepted=true')
        if exit_code != 0:
            failures.append('PASS requires exit_code=0')
        if failure_kind is not None or failure_message is not None:
            failures.append('PASS requires null failure_kind and failure_message')
    elif verdict == 'FAIL':
        if accepted is not False:
            failures.append('FAIL requires accepted=false')
        if type(exit_code) is int and exit_code == 0:
            failures.append('FAIL requires a nonzero exit_code')
        if not isinstance(failure_kind, str) or not failure_kind.strip():
            failures.append('FAIL requires a nonempty failure_kind')
        if not isinstance(failure_message, str) or not failure_message.strip():
            failures.append('FAIL requires a nonempty failure_message')
    return failures


def read_startup_result(path: Path) -> tuple[dict[str, Any], str, int]:
    """Read one bounded, regular, non-symlink strict JSON artifact."""
    try:
        metadata = path.lstat()
    except FileNotFoundError as error:
        raise GateError('result_disappeared', f'{path} disappeared while being read') from error
    if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
        raise GateError('malformed_result', 'startup result must be a regular non-symlink file')
    if metadata.st_size <= 0 or metadata.st_size > MAXIMUM_RESULT_BYTES:
        raise GateError(
            'malformed_result',
            f'startup result size {metadata.st_size} is outside 1..{MAXIMUM_RESULT_BYTES}',
        )
    payload = path.read_bytes()
    if len(payload) != metadata.st_size:
        raise GateError('malformed_result', 'startup result changed while being read')
    try:
        after = path.lstat()
    except FileNotFoundError as error:
        raise GateError('result_disappeared', f'{path} disappeared while being read') from error
    before_identity = (
        metadata.st_dev,
        metadata.st_ino,
        metadata.st_size,
        metadata.st_mtime_ns,
    )
    after_identity = (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
    if before_identity != after_identity or stat.S_ISLNK(after.st_mode):
        raise GateError('malformed_result', 'startup result changed while being read')
    try:
        document = json.loads(
            payload.decode('utf-8'),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise GateError(
            'malformed_result',
            f'startup result is not strict UTF-8 JSON: {error}',
        ) from error
    if not isinstance(document, dict):
        raise GateError('malformed_result', 'startup result must be a JSON object')
    return document, hashlib.sha256(payload).hexdigest(), len(payload)


def _read_lifecycle_ready(
    path: Path,
    *,
    expected_watch_pid: int,
    expected_wall_timeout_s: float,
    expected_nodes: Collection[str],
) -> tuple[dict[str, dict[str, Any]], Path, str, int]:
    """Read and validate the canonical lifecycle proof used for timeout recovery."""
    if type(expected_watch_pid) is not int or expected_watch_pid <= 0:
        raise GateError(
            'launch_log_recovery_invalid',
            'expected lifecycle watch PID must be a positive integer',
        )
    if (
        type(expected_wall_timeout_s) is not float
        or not math.isfinite(expected_wall_timeout_s)
        or expected_wall_timeout_s <= 0.0
    ):
        raise GateError(
            'launch_log_recovery_invalid',
            'expected lifecycle wall timeout must be a finite positive float',
        )
    if isinstance(expected_nodes, (str, bytes, dict)):
        raise GateError(
            'launch_log_recovery_invalid',
            'expected lifecycle nodes must be a nonempty unique collection',
        )
    expected_node_names = tuple(expected_nodes)
    expected_node_set = set(expected_node_names)
    if (
        not expected_node_names
        or len(expected_node_set) != len(expected_node_names)
        or any(
            not isinstance(name, str) or _LIFECYCLE_NODE_PATTERN.fullmatch(name) is None
            for name in expected_node_names
        )
    ):
        node_collection_error = (
            'expected lifecycle nodes must be a nonempty unique collection of canonical node names'
        )
        raise GateError(
            'launch_log_recovery_invalid',
            node_collection_error,
        )
    try:
        resolved_path = path.resolve(strict=True)
        payload = _stable_regular_bytes(
            path,
            maximum_bytes=MAXIMUM_LIFECYCLE_READY_BYTES,
            allow_empty=False,
            label='lifecycle-ready artifact',
            activity='being correlated with the final launch log',
        )
        if path.resolve(strict=True) != resolved_path:
            raise GateError(
                'launch_log_recovery_invalid',
                'lifecycle-ready artifact resolved path changed while being read',
            )
        document = json.loads(
            payload.decode('utf-8'),
            object_pairs_hook=_strict_object,
            parse_constant=_reject_constant,
        )
    except GateError as error:
        raise GateError('launch_log_recovery_invalid', str(error)) from error
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, ValueError) as error:
        raise GateError(
            'launch_log_recovery_invalid',
            f'lifecycle-ready artifact is not canonical strict UTF-8 JSON: {error}',
        ) from error

    failures: list[str] = []
    if not isinstance(document, dict):
        failures.append('lifecycle-ready artifact must be a JSON object')
    else:
        observed_keys = set(document)
        missing = sorted(_LIFECYCLE_READY_KEYS - observed_keys)
        unexpected = sorted(observed_keys - _LIFECYCLE_READY_KEYS)
        if missing:
            failures.append(f'lifecycle-ready artifact missing keys: {missing}')
        if unexpected:
            failures.append(f'lifecycle-ready artifact has unexpected keys: {unexpected}')

    if failures:
        raise GateError('launch_log_recovery_invalid', '; '.join(failures))
    assert isinstance(document, dict)

    if serialize_result(document).encode('utf-8') != payload:
        failures.append('lifecycle-ready artifact must use canonical serialization')
    if document['verdict'] != 'PASS':
        failures.append("lifecycle-ready verdict must be exactly 'PASS'")
    if document['failure'] is not None:
        failures.append('lifecycle-ready PASS requires failure=null')
    if document['namespace'] != '/robotest':
        failures.append("lifecycle-ready namespace must be exactly '/robotest'")
    required_state = document['required_state']
    if (
        not isinstance(required_state, dict)
        or set(required_state) != {'id', 'label'}
        or type(required_state.get('id')) is not int
        or required_state.get('id') != 3
        or required_state.get('label') != 'active'
    ):
        failures.append(
            "lifecycle-ready required_state must be exactly {'id': 3, 'label': 'active'}"
        )
    wall_timeout = document['wall_timeout_s']
    if type(wall_timeout) is not float or not math.isfinite(wall_timeout):
        failures.append('lifecycle-ready wall_timeout_s must be a finite float')
    elif wall_timeout != expected_wall_timeout_s:
        failures.append(
            'lifecycle-ready wall_timeout_s must equal the correlated launch '
            f'value {expected_wall_timeout_s}; observed {wall_timeout}'
        )
    watch_pid = document['watch_pid']
    if type(watch_pid) is not int or watch_pid != expected_watch_pid:
        failures.append(
            'lifecycle-ready watch_pid must equal the correlated launch PID '
            f'{expected_watch_pid}; observed {watch_pid!r}'
        )

    states = document['states']
    service_states: dict[str, dict[str, Any]] = {}
    if not isinstance(states, dict) or not states:
        failures.append('lifecycle-ready states must be a nonempty JSON object')
    else:
        if set(states) != expected_node_set:
            missing_nodes = sorted(expected_node_set - set(states))
            unexpected_nodes = sorted(set(states) - expected_node_set)
            failures.append(
                'lifecycle-ready states must equal the correlated node set; '
                f'missing={missing_nodes}, unexpected={unexpected_nodes}'
            )
        for node_name, state in states.items():
            if (
                not isinstance(node_name, str)
                or _LIFECYCLE_NODE_PATTERN.fullmatch(node_name) is None
            ):
                failures.append(
                    f'lifecycle-ready state name {node_name!r} is not a canonical node name'
                )
                continue
            if not isinstance(state, dict):
                failures.append(f'lifecycle-ready state {node_name!r} must be an object')
                continue
            observed_state_keys = set(state)
            missing_state = sorted(_LIFECYCLE_STATE_KEYS - observed_state_keys)
            unexpected_state = sorted(observed_state_keys - _LIFECYCLE_STATE_KEYS)
            if missing_state:
                failures.append(
                    f'lifecycle-ready state {node_name!r} missing keys: {missing_state}'
                )
            if unexpected_state:
                failures.append(
                    f'lifecycle-ready state {node_name!r} has unexpected keys: {unexpected_state}'
                )
            if missing_state:
                continue

            expected_service = f'/robotest/{node_name}/get_state'
            if state['service'] != expected_service:
                failures.append(
                    f'lifecycle-ready state {node_name!r} service must be {expected_service!r}'
                )
            if state['service_seen'] is not True:
                failures.append(f'lifecycle-ready state {node_name!r} requires service_seen=true')
            if state['state_id'] != 3 or type(state['state_id']) is not int:
                failures.append(f'lifecycle-ready state {node_name!r} state_id must be integer 3')
            if state['label'] != 'active':
                label_error = f'lifecycle-ready state {node_name!r} label must be exactly active'
                failures.append(label_error)
            attempts = state['attempts']
            timed_out_attempts = state['timed_out_attempts']
            if type(attempts) is not int or attempts <= 0:
                failures.append(f'lifecycle-ready state {node_name!r} attempts must be positive')
            if type(timed_out_attempts) is not int or timed_out_attempts < 0:
                failures.append(
                    f'lifecycle-ready state {node_name!r} timed_out_attempts must be nonnegative'
                )
            elif type(attempts) is int and timed_out_attempts >= attempts:
                failures.append(
                    f'lifecycle-ready state {node_name!r} requires attempts > timed_out_attempts'
                )
            service_states[expected_service] = state

    if failures:
        raise GateError('launch_log_recovery_invalid', '; '.join(failures))
    return (
        service_states,
        resolved_path,
        hashlib.sha256(payload).hexdigest(),
        len(payload),
    )


def _recovered_timeout_lines(
    matches: list[dict[str, Any]],
    service_states: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    """Return the exact raw timeout lines covered by lifecycle timeout counters."""
    recovered: list[dict[str, Any]] = []
    recovered_per_service: dict[str, int] = {}
    for match in matches:
        if match['signature_ids'] != ['dds_response_timeout']:
            continue
        signature_matches = list(_DDS_RESPONSE_TIMEOUT_PATTERN.finditer(match['text']))
        phrase_matches = list(_RECOVERABLE_DDS_RESPONSE_TIMEOUT_PATTERN.finditer(match['text']))
        if len(signature_matches) != 1 or len(phrase_matches) != 1:
            continue
        service_name = phrase_matches[0].group('service')
        state = service_states.get(service_name)
        if state is None or state['timed_out_attempts'] < 1:
            continue
        recovered_count = recovered_per_service.get(service_name, 0)
        if recovered_count >= state['timed_out_attempts']:
            continue
        recovered_per_service[service_name] = recovered_count + 1
        recovered.append(
            {
                'line_number': match['line_number'],
                'service_name': service_name,
                'text': match['text'],
                'timed_out_attempts': state['timed_out_attempts'],
            }
        )
    return recovered


def scan_final_launch_log(
    path: Path,
    launch_stopped_utc: str,
    *,
    scanned_utc: str | None = None,
    lifecycle_ready_path: Path | None = None,
    expected_lifecycle_watch_pid: int | None = None,
    expected_lifecycle_wall_timeout_s: float | None = None,
    expected_lifecycle_nodes: Collection[str] | None = None,
) -> dict[str, Any]:
    """Return strict, hash-bound evidence for every prohibited launch-log signature."""
    timestamp_failures: list[str] = []
    stopped_at = _parse_utc('launch_stopped_utc', launch_stopped_utc, timestamp_failures)
    scan_timestamp = utc_now() if scanned_utc is None else scanned_utc
    scanned_at = _parse_utc('scanned_utc', scan_timestamp, timestamp_failures)
    if stopped_at is not None and scanned_at is not None and scanned_at < stopped_at:
        timestamp_failures.append('scanned_utc must not precede launch_stopped_utc')
    evidence: dict[str, Any] = {
        'schema_version': FINAL_LAUNCH_LOG_SCHEMA_VERSION,
        'verdict': 'FAIL',
        'failure_kind': None,
        'failure_message': None,
        'scanned_utc': scan_timestamp,
        'launch_stopped_utc': launch_stopped_utc,
        'launch_log_path': str(path),
        'launch_log_sha256': None,
        'launch_log_size_bytes': None,
        'line_count': None,
        'match_count': 0,
        'matches': [],
        'lifecycle_timeout_recovery': {
            'lifecycle_ready_path': None,
            'lifecycle_ready_sha256': None,
            'lifecycle_ready_size_bytes': None,
            'recovered_line_count': 0,
            'recovered_lines': [],
        },
        'signature_definitions': [
            {
                'signature_id': signature_id,
                'description': description,
                'pattern': pattern.pattern,
                'ignore_case': bool(pattern.flags & re.IGNORECASE),
            }
            for signature_id, description, pattern in LAUNCH_LOG_SIGNATURES
        ],
    }
    if timestamp_failures:
        evidence['failure_kind'] = 'launch_log_invalid'
        evidence['failure_message'] = '; '.join(timestamp_failures)
        return evidence

    try:
        payload = _stable_regular_bytes(
            path,
            maximum_bytes=MAXIMUM_LAUNCH_LOG_BYTES,
            allow_empty=False,
            label='launch log',
            activity='being scanned',
        )
        text = payload.decode('utf-8')
    except (OSError, UnicodeDecodeError, GateError) as error:
        evidence['failure_kind'] = (
            error.kind if isinstance(error, GateError) else 'launch_log_invalid'
        )
        evidence['failure_message'] = f'{type(error).__name__}: {error}'
        return evidence

    lines = text.splitlines()
    matches = []
    for line_number, line in enumerate(lines, start=1):
        signature_ids = [
            signature_id
            for signature_id, _, pattern in LAUNCH_LOG_SIGNATURES
            if pattern.search(line) is not None
        ]
        if signature_ids:
            matches.append(
                {
                    'line_number': line_number,
                    'signature_ids': signature_ids,
                    'text': line,
                }
            )
    evidence.update(
        {
            'launch_log_sha256': hashlib.sha256(payload).hexdigest(),
            'launch_log_size_bytes': len(payload),
            'line_count': len(lines),
            'match_count': len(matches),
            'matches': matches,
        }
    )
    recovered_lines: list[dict[str, Any]] = []
    if lifecycle_ready_path is None and any(
        value is not None
        for value in (
            expected_lifecycle_watch_pid,
            expected_lifecycle_wall_timeout_s,
            expected_lifecycle_nodes,
        )
    ):
        evidence['failure_kind'] = 'launch_log_recovery_invalid'
        evidence['failure_message'] = (
            'expected lifecycle recovery values require lifecycle_ready_path'
        )
        return evidence
    if lifecycle_ready_path is not None:
        if (
            expected_lifecycle_watch_pid is None
            or expected_lifecycle_wall_timeout_s is None
            or expected_lifecycle_nodes is None
        ):
            evidence['failure_kind'] = 'launch_log_recovery_invalid'
            evidence['failure_message'] = (
                'lifecycle timeout recovery requires expected watch PID, wall timeout, '
                'and exact node set'
            )
            return evidence
        try:
            service_states, resolved_path, digest, size = _read_lifecycle_ready(
                lifecycle_ready_path,
                expected_watch_pid=expected_lifecycle_watch_pid,
                expected_wall_timeout_s=expected_lifecycle_wall_timeout_s,
                expected_nodes=expected_lifecycle_nodes,
            )
        except GateError as error:
            evidence['failure_kind'] = error.kind
            evidence['failure_message'] = str(error)
            return evidence
        recovered_lines = _recovered_timeout_lines(matches, service_states)
        evidence['lifecycle_timeout_recovery'] = {
            'lifecycle_ready_path': str(resolved_path),
            'lifecycle_ready_sha256': digest,
            'lifecycle_ready_size_bytes': size,
            'recovered_line_count': len(recovered_lines),
            'recovered_lines': recovered_lines,
        }

    if matches and len(recovered_lines) != len(matches):
        categories = sorted(
            {signature_id for match in matches for signature_id in match['signature_ids']}
        )
        evidence['failure_kind'] = 'launch_log_signature_detected'
        evidence['failure_message'] = (
            f'{len(matches)} prohibited launch-log line(s) matched, '
            f'{len(recovered_lines)} recovered: {categories}'
        )
    else:
        evidence['verdict'] = 'PASS'
    return evidence


def initial_gate_evidence(
    result_path: Path,
    watch_pid: int,
    wall_timeout_sec: float,
    contract: StartupContract,
) -> dict[str, Any]:
    """Build a complete fail-closed gate record before waiting starts."""
    return {
        'schema_version': SCHEMA_VERSION,
        'verdict': 'FAIL',
        'failure_kind': None,
        'failure_message': None,
        'started_utc': utc_now(),
        'completed_utc': None,
        'elapsed_wall_sec': 0.0,
        'watch_pid': watch_pid,
        'launch_alive_at_completion': None,
        'wall_timeout_sec': wall_timeout_sec,
        'result_path': str(result_path),
        'result_observed': False,
        'result_sha256': None,
        'result_size_bytes': None,
        'startup_result': None,
        'expected': {
            'schema_version': SCHEMA_VERSION,
            'service_name': contract.service_name,
            'command': contract.command,
            'discovery_grace_sec': contract.discovery_grace_sec,
            'service_timeout_sec': contract.service_timeout_sec,
            'response_timeout_sec': contract.response_timeout_sec,
        },
    }


def wait_for_startup_result(
    result_path: Path,
    watch_pid: int,
    wall_timeout_sec: float,
    contract: StartupContract,
    *,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
    alive: Callable[[int], bool] = watched_process_alive,
) -> dict[str, Any]:
    """Wait for one atomic result while enforcing launch liveness and wall bounds."""
    if not math.isfinite(wall_timeout_sec) or wall_timeout_sec <= 0.0:
        raise ValueError('wall_timeout_sec must be finite and positive')
    if watch_pid <= 0:
        raise ValueError('watch_pid must be positive')

    evidence = initial_gate_evidence(result_path, watch_pid, wall_timeout_sec, contract)
    started = clock()
    deadline = started + wall_timeout_sec
    failure: GateError | None = None
    while True:
        now = clock()
        if not alive(watch_pid):
            failure = GateError(
                'launch_process_exited',
                f'watched launch process {watch_pid} exited before startup acceptance',
            )
            break
        if result_path.exists() or result_path.is_symlink():
            evidence['result_observed'] = True
            try:
                startup_result, digest, size = read_startup_result(result_path)
                evidence['startup_result'] = startup_result
                evidence['result_sha256'] = digest
                evidence['result_size_bytes'] = size
                validation_failures = validate_startup_result(startup_result, contract)
                result_watch_pid = startup_result.get('watch_pid')
                if result_watch_pid not in (None, watch_pid):
                    validation_failures.append(
                        f'watch_pid must be null or equal gate watch PID {watch_pid}; '
                        f'observed {result_watch_pid!r}'
                    )
                result_elapsed = startup_result.get('elapsed_wall_sec')
                if _is_number(result_elapsed) and float(result_elapsed) > wall_timeout_sec:
                    validation_failures.append(
                        'elapsed_wall_sec exceeds the independent gate wall timeout '
                        f'{wall_timeout_sec}'
                    )
                if validation_failures:
                    raise GateError('malformed_result', '; '.join(validation_failures))
                if startup_result['verdict'] == 'FAIL':
                    kind = (
                        'startup_rejected'
                        if startup_result['failure_kind'] == 'startup_rejected'
                        else 'startup_result_failure'
                    )
                    raise GateError(kind, startup_result['failure_message'])
                evidence['verdict'] = 'PASS'
            except GateError as error:
                failure = error
            break
        if now >= deadline:
            failure = GateError(
                'wall_timeout',
                f'startup result was absent after {wall_timeout_sec:.3f}s',
            )
            break
        sleeper(min(POLL_PERIOD_SEC, max(0.0, deadline - now)))

    evidence['elapsed_wall_sec'] = round(max(0.0, clock() - started), 9)
    evidence['completed_utc'] = utc_now()
    evidence['launch_alive_at_completion'] = alive(watch_pid)
    if failure is None and not evidence['launch_alive_at_completion']:
        evidence['verdict'] = 'FAIL'
        failure = GateError(
            'launch_process_exited',
            f'watched launch process {watch_pid} exited during startup acceptance',
        )
    if failure is not None:
        evidence['failure_kind'] = failure.kind
        evidence['failure_message'] = str(failure)
    return evidence


class _FakeClock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def sleep(self, duration: float) -> None:
        self.value += duration


def _valid_result(verdict: str = 'PASS') -> dict[str, Any]:
    failed = verdict == 'FAIL'
    return {
        'schema_version': 1,
        'verdict': verdict,
        'accepted': not failed,
        'exit_code': 6 if failed else 0,
        'service_name': EXPECTED_SERVICE_NAME,
        'command': EXPECTED_STARTUP_COMMAND,
        'elapsed_wall_sec': 0.25,
        'discovery_grace_sec': DEFAULT_DISCOVERY_GRACE_SEC,
        'service_timeout_sec': DEFAULT_SERVICE_TIMEOUT_SEC,
        'response_timeout_sec': DEFAULT_RESPONSE_TIMEOUT_SEC,
        'watch_pid': None,
        'failure_kind': 'startup_rejected' if failed else None,
        'failure_message': 'manager rejected STARTUP' if failed else None,
        'started_utc': '2026-08-25T22:38:51Z',
        'completed_utc': '2026-08-25T22:38:59Z',
    }


def run_self_test() -> int:
    """Exercise success, rejection, dead-owner, timeout, and malformed paths."""
    contract = StartupContract()
    with tempfile.TemporaryDirectory(prefix='phase2-startup-gate-') as directory:
        root = Path(directory)
        success_path = root / 'success.json'
        success_path.write_text(serialize_result(_valid_result()), encoding='utf-8')
        success = wait_for_startup_result(success_path, 101, 1.0, contract, alive=lambda _: True)
        assert success['verdict'] == 'PASS'
        assert success['result_sha256'] == hashlib.sha256(success_path.read_bytes()).hexdigest()

        failure_path = root / 'failure.json'
        failure_path.write_text(serialize_result(_valid_result('FAIL')), encoding='utf-8')
        failure = wait_for_startup_result(failure_path, 102, 1.0, contract, alive=lambda _: True)
        assert failure['failure_kind'] == 'startup_rejected'

        dead = wait_for_startup_result(
            root / 'dead.json', 103, 1.0, contract, alive=lambda _: False
        )
        assert dead['failure_kind'] == 'launch_process_exited'

        fake_clock = _FakeClock()
        timed_out = wait_for_startup_result(
            root / 'timeout.json',
            104,
            0.2,
            contract,
            clock=fake_clock,
            sleeper=fake_clock.sleep,
            alive=lambda _: True,
        )
        assert timed_out['failure_kind'] == 'wall_timeout'
        assert timed_out['elapsed_wall_sec'] == 0.2

        malformed_path = root / 'malformed.json'
        malformed = _valid_result()
        malformed['unexpected'] = True
        malformed_path.write_text(serialize_result(malformed), encoding='utf-8')
        malformed_evidence = wait_for_startup_result(
            malformed_path, 105, 1.0, contract, alive=lambda _: True
        )
        assert malformed_evidence['failure_kind'] == 'malformed_result'

        duplicate_path = root / 'duplicate.json'
        duplicate_path.write_text('{"schema_version":1,"schema_version":1}\n', encoding='utf-8')
        duplicate = wait_for_startup_result(
            duplicate_path, 106, 1.0, contract, alive=lambda _: True
        )
        assert duplicate['failure_kind'] == 'malformed_result'

        invalid_bool = _valid_result()
        invalid_bool['exit_code'] = False
        assert any('exit_code' in item for item in validate_startup_result(invalid_bool, contract))

        invalid_verdict = _valid_result()
        invalid_verdict['verdict'] = []
        verdict_errors = validate_startup_result(invalid_verdict, contract)
        assert any('verdict' in item for item in verdict_errors)

        evidence_path = root / 'evidence.json'
        atomic_write_text(evidence_path, serialize_result(success))
        assert json.loads(evidence_path.read_text(encoding='utf-8'))['verdict'] == 'PASS'

        clean_log = root / 'clean-launch.log'
        clean_log.write_text('Nav2 active\nmission completed\n', encoding='utf-8')
        clean_scan = scan_final_launch_log(clean_log, '2026-08-25T22:40:00Z')
        assert clean_scan['verdict'] == 'PASS'
        assert clean_scan['match_count'] == 0
        clean_sha256 = hashlib.sha256(clean_log.read_bytes()).hexdigest()
        assert clean_scan['launch_log_sha256'] == clean_sha256

        rejected_log = root / 'rejected-launch.log'
        rejected_log.write_text(
            '\n'.join(
                (
                    'lifecycle STARTUP rejected by manager',
                    'Failed to bring up all requested nodes',
                    'service client: async_send_request failed',
                    'rmw: failed to send response to /robotest/amcl/get_state (timeout)',
                    'fatal process condition',
                )
            )
            + '\n',
            encoding='utf-8',
        )
        rejected_scan = scan_final_launch_log(rejected_log, '2026-08-25T22:40:00Z')
        assert rejected_scan['verdict'] == 'FAIL'
        assert rejected_scan['failure_kind'] == 'launch_log_signature_detected'
        assert {
            signature_id
            for match in rejected_scan['matches']
            for signature_id in match['signature_ids']
        } == {signature_id for signature_id, _, _ in LAUNCH_LOG_SIGNATURES}

        invalid_log = root / 'invalid-launch.log'
        invalid_log.write_bytes(b'\xff\xfe')
        invalid_scan = scan_final_launch_log(invalid_log, '2026-08-25T22:40:00Z')
        assert invalid_scan['verdict'] == 'FAIL'
        assert invalid_scan['failure_kind'] == 'launch_log_invalid'

        invalid_stop = scan_final_launch_log(clean_log, 'not-a-utc-timestamp')
        assert invalid_stop['verdict'] == 'FAIL'
        assert invalid_stop['failure_kind'] == 'launch_log_invalid'
    print('phase2_startup_gate self-test PASS')
    return EXIT_PASS


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return parsed


def _positive_pid(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be a positive integer')
    return parsed


def create_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--watch-pid', type=_positive_pid)
    parser.add_argument('--scan-launch-log', type=Path)
    parser.add_argument('--launch-stopped-utc')
    parser.add_argument('--wall-timeout', type=_positive_float, default=DEFAULT_WALL_TIMEOUT_SEC)
    parser.add_argument('--expected-service-name', default=EXPECTED_SERVICE_NAME)
    parser.add_argument(
        '--expected-discovery-grace-sec',
        type=_positive_float,
        default=DEFAULT_DISCOVERY_GRACE_SEC,
    )
    parser.add_argument(
        '--expected-service-timeout-sec',
        type=_positive_float,
        default=DEFAULT_SERVICE_TIMEOUT_SEC,
    )
    parser.add_argument(
        '--expected-response-timeout-sec',
        type=_positive_float,
        default=DEFAULT_RESPONSE_TIMEOUT_SEC,
    )
    parser.add_argument('--self-test', action='store_true')
    return parser


def main() -> int:
    parser = create_argument_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.output is not None and args.output.is_symlink():
        parser.error('--output must not be a symlink')
    if args.scan_launch_log is not None:
        if args.output is None or args.launch_stopped_utc is None:
            parser.error('--output and --launch-stopped-utc are required with --scan-launch-log')
        if args.result is not None or args.watch_pid is not None:
            parser.error('--scan-launch-log cannot be combined with --result or --watch-pid')
        if args.scan_launch_log.absolute() == args.output.absolute():
            parser.error('--scan-launch-log and --output must be distinct paths')
        evidence = scan_final_launch_log(args.scan_launch_log, args.launch_stopped_utc)
        atomic_write_text(args.output, serialize_result(evidence))
        print(serialize_result(evidence), end='')
        return EXIT_PASS if evidence['verdict'] == 'PASS' else EXIT_GATE_FAILURE
    if args.result is None or args.output is None or args.watch_pid is None:
        parser.error('--result, --output, and --watch-pid are required outside --self-test')
    if args.launch_stopped_utc is not None:
        parser.error('--launch-stopped-utc requires --scan-launch-log')
    if args.result.absolute() == args.output.absolute():
        parser.error('--result and --output must be distinct paths')

    contract = StartupContract(
        service_name=args.expected_service_name,
        discovery_grace_sec=args.expected_discovery_grace_sec,
        service_timeout_sec=args.expected_service_timeout_sec,
        response_timeout_sec=args.expected_response_timeout_sec,
    )
    try:
        evidence = wait_for_startup_result(
            args.result,
            args.watch_pid,
            args.wall_timeout,
            contract,
        )
        exit_code = EXIT_PASS if evidence['verdict'] == 'PASS' else EXIT_GATE_FAILURE
    except Exception as error:  # pragma: no cover - defensive outer boundary
        evidence = initial_gate_evidence(args.result, args.watch_pid, args.wall_timeout, contract)
        evidence.update(
            {
                'failure_kind': 'internal_error',
                'failure_message': f'{type(error).__name__}: {error}',
                'completed_utc': utc_now(),
                'launch_alive_at_completion': watched_process_alive(args.watch_pid),
            }
        )
        exit_code = EXIT_INTERNAL_ERROR

    atomic_write_text(args.output, serialize_result(evidence))
    print(serialize_result(evidence), end='')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
