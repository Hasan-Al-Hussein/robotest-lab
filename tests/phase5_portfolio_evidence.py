#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Produce and revalidate the Phase 5 documentation and media portfolio.

The producer deliberately has two stages.  ``prepare_portfolio`` replays each
documented command in its own detached candidate worktree and writes immutable
render inputs.  A human renders and reviews those exact inputs.
``finalize_portfolio`` then binds the reviewed SVG bytes, the deterministic
Scenario 5 chart, and the replay transcript into one raw attempt.  Projection
and final validation never discover a "latest" attempt.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import selectors
import shlex
import signal
import stat
import struct
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
import zlib
from collections.abc import Mapping, Sequence
from contextlib import suppress
from datetime import UTC, datetime
from pathlib import Path
from typing import Any, NamedTuple

from phase5_ci import (
    EvidenceError,
    canonical_json_bytes,
    file_sha256,
)

GIT_SHA = re.compile(r'^[0-9a-f]{40}$')
SHA256 = re.compile(r'^[0-9a-f]{64}$')
SAFE_ID = re.compile(r'^[a-z0-9][a-z0-9-]{0,63}$')
ATTEMPT_ID = re.compile(r'^\d{8}T\d{6}Z-[1-9]\d{0,9}$')
README_PATH = 'README.md'
REPLAY_MATRIX_PATH = 'config/phase5-readme-replay.json'
DOCUMENTED_REPOSITORY = '/home/hasan/robotest-lab'
PORTFOLIO_RAW_PREFIX = 'artifacts/evidence/phase5/portfolio'
PORTFOLIO_PROJECTION_ROOT = 'docs/results/phase-5'
SCENARIO5_RESULT_RELATIVE = 'runs/12/result/run-result.json'
SCENARIO5_NAME = 'deterministic_odometry_drift'
DIAGRAM_IDS = ('architecture', 'release-flow')
DIAGRAM_MARKERS = {
    'architecture': '<!-- ROBOTEST_PORTFOLIO_DIAGRAM: architecture -->',
    'release-flow': '<!-- ROBOTEST_PORTFOLIO_DIAGRAM: release-flow -->',
}
EXPECTED_RESULT_ARTIFACTS = {
    'localization-error.png',
    'measurement-summary.png',
    'real-time-factor.png',
    'report.html',
    'report.md',
    'run-result.csv',
    'run-result.json',
    'trajectory.png',
}
FORBIDDEN_REPLAY_ENVIRONMENT = (
    'AMENT_PREFIX_PATH',
    'CMAKE_PREFIX_PATH',
    'COLCON_PREFIX_PATH',
    'GAZEBO_MODEL_PATH',
    'GZ_PARTITION',
    'IGN_GAZEBO_RESOURCE_PATH',
    'LD_LIBRARY_PATH',
    'PYTHONPATH',
    'RMW_IMPLEMENTATION',
    'ROS_DISTRO',
    'ROS_DOMAIN_ID',
    'ROS_LOCALHOST_ONLY',
    'ROS_VERSION',
)
MAX_README_BYTES = 2 * 1024 * 1024
MAX_MATRIX_BYTES = 2 * 1024 * 1024
MAX_JSON_BYTES = 32 * 1024 * 1024
MAX_RESULT_DIRECTORY_BYTES = 256 * 1024 * 1024
MAX_RESULT_ARTIFACT_BYTES = 32 * 1024 * 1024
MAX_PNG_BYTES = 4 * 1024 * 1024
MAX_SVG_BYTES = 2 * 1024 * 1024
MAX_REPLAY_LOG_BYTES = 8 * 1024 * 1024
MAX_PROCESS_TABLE_ENTRIES = 65_536
PROCESS_GROUP_TERM_GRACE_S = 2.0
PROCESS_GROUP_KILL_GRACE_S = 5.0
PROCESS_GROUP_POLL_INTERVAL_S = 0.01
QUIESCENT_PROCESS_STATES = frozenset({'X', 'Z', 'x'})
REPLAY_MAPPING_POLICY = (
    'Validate the frozen documented-root assertion, then execute only the second '
    'line with cwd set to a fresh detached candidate worktree.'
)
PHASE0_REFRESH_DELTA = (
    'artifacts/evidence/phase0/installed-packages.tsv',
    'artifacts/evidence/phase0/phase0-versions.json',
    'artifacts/evidence/phase0/ros2-doctor.txt',
    'artifacts/evidence/phase0/tool-versions.tsv',
)
REPLAY_ALLOWED_CHANGES = {
    'phase0-source-check': ('artifacts/evidence/phase0/ros-apt-source.json',),
    'phase0-source-apply': ('artifacts/evidence/phase0/ros-apt-source.json',),
    'phase0-preinstall': (
        'artifacts/evidence/phase0/apt-planned-packages.txt',
        'artifacts/evidence/phase0/apt-resolution.tsv',
        'artifacts/evidence/phase0/apt-simulation.txt',
        'artifacts/evidence/phase0/phase0-preinstall.json',
    ),
    'phase0-install': tuple(
        sorted(
            {
                'artifacts/evidence/phase0/apt-planned-packages.txt',
                'artifacts/evidence/phase0/apt-resolution.tsv',
                'artifacts/evidence/phase0/apt-simulation.txt',
                'artifacts/evidence/phase0/install-transcript.txt',
                'artifacts/evidence/phase0/phase0-preinstall.json',
                'artifacts/evidence/phase0/ros-apt-source.json',
                *PHASE0_REFRESH_DELTA,
            }
        )
    ),
    'phase0-verify': PHASE0_REFRESH_DELTA,
    'aggregate-local': PHASE0_REFRESH_DELTA,
    'phase1-verify': (),
    'phase2-verify': (),
    'phase3-static': (),
    'phase4-static': (),
    'phase5-local': (),
}
RUFF_REQUIRED_REPLAY_IDS = {'aggregate-local', 'phase0-verify', 'phase5-local'}
RUFF_VERSION = '0.16.4'
RUFF_BYTES = 23_911_904
RUFF_SHA256 = 'b17e9a2ddd4932620a6349d6a475d61a49c8aa699c4e318528a98e4ad5773913'
MAX_RUFF_BYTES = 64 * 1024 * 1024
MAX_RAW_FILE_COUNT = 128
MAX_RAW_TOTAL_BYTES = 384 * 1024 * 1024
TRACKED_IGNORE_RULES = {
    'portfolio_raw_root': (
        '/artifacts/evidence/phase5/',
        f'{PORTFOLIO_RAW_PREFIX}/ignore-boundary-probe',
    ),
    'ruff_tooling_seed': ('/.venv/', '.venv/bin/ruff'),
}
REPLAY_ASSIGNMENTS = {
    identifier: (
        {'ROBOTEST_CPUSET': '0-5', 'ROBOTEST_SIM_SEED': '42'}
        if identifier == 'phase2-verify'
        else {}
    )
    for identifier in REPLAY_ALLOWED_CHANGES
}
REPLAY_EXECUTABLES = {
    'phase0-source-check': 'scripts/setup_ros2_repository.sh --check',
    'phase0-source-apply': 'scripts/setup_ros2_repository.sh --apply',
    'phase0-preinstall': 'scripts/verify_phase0_preinstall.sh',
    'phase0-install': 'scripts/install_dependencies.sh --apply',
    'phase0-verify': 'scripts/verify_phase0.sh',
    'aggregate-local': 'scripts/verify_all.sh',
    'phase1-verify': 'taskset -c 0-5 scripts/verify_phase1.sh',
    'phase2-verify': 'ROBOTEST_SIM_SEED=42 ROBOTEST_CPUSET=0-5 scripts/verify_phase2.sh',
    'phase3-static': 'taskset -c 0-5 scripts/verify_phase3.sh',
    'phase4-static': 'taskset -c 0-5 scripts/verify_phase4.sh',
    'phase5-local': 'scripts/verify_phase5.sh --local',
}
REPLAY_MUTATING_IDS = {'phase0-install', 'phase0-source-apply'}
REPLAY_TIMEOUTS = {
    'phase0-source-check': 300,
    'phase0-source-apply': 1800,
    'phase0-preinstall': 1800,
    'phase0-install': 7200,
    'phase0-verify': 900,
    'aggregate-local': 14_400,
    'phase1-verify': 7200,
    'phase2-verify': 7200,
    'phase3-static': 14_400,
    'phase4-static': 14_400,
    'phase5-local': 14_400,
}


class _ProcessIdentity(NamedTuple):
    pid: int
    command: str
    state: str
    parent_pid: int
    process_group_id: int
    session_id: int
    start_ticks: int


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceError(message)


def _mapping(value: object, label: str) -> dict[str, Any]:
    _require(isinstance(value, dict), f'{label} must be a mapping')
    return value


def _list(value: object, label: str) -> list[Any]:
    _require(isinstance(value, list), f'{label} must be a list')
    return value


def _exact_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _reject_symlink_components(path: Path, label: str) -> None:
    absolute = path.absolute()
    current = Path(absolute.anchor)
    for component in absolute.parts[1:]:
        current /= component
        if current.exists() or current.is_symlink():
            _require(not current.is_symlink(), f'{label} contains a symlink: {current}')


def _resolved_directory(path: Path, label: str) -> Path:
    _reject_symlink_components(path, label)
    resolved = path.resolve(strict=True)
    _require(resolved.is_dir(), f'{label} is not a directory: {path}')
    _require(resolved == path.absolute(), f'{label} resolves through an alias: {path}')
    return resolved


def _regular_file(
    path: Path,
    label: str,
    *,
    maximum_bytes: int | None = None,
    allow_empty: bool = False,
) -> Path:
    _reject_symlink_components(path, label)
    try:
        metadata = path.lstat()
    except FileNotFoundError as exc:
        raise EvidenceError(f'missing regular {label}: {path}') from exc
    _require(stat.S_ISREG(metadata.st_mode), f'{label} is not a regular file: {path}')
    _require(metadata.st_nlink == 1, f'{label} is hard-linked: {path}')
    if not allow_empty:
        _require(metadata.st_size > 0, f'{label} is empty: {path}')
    if maximum_bytes is not None:
        _require(metadata.st_size <= maximum_bytes, f'{label} exceeds its size bound')
    return path


def _stable_read(
    path: Path,
    label: str,
    *,
    maximum_bytes: int,
    allow_empty: bool = False,
) -> bytes:
    _reject_symlink_components(path, label)
    try:
        initial = path.lstat()
    except FileNotFoundError as exc:
        raise EvidenceError(f'missing regular {label}: {path}') from exc
    _require(stat.S_ISREG(initial.st_mode), f'{label} is not a regular file')
    _require(initial.st_nlink == 1, f'{label} is hard-linked')
    _require(
        (allow_empty or initial.st_size > 0) and initial.st_size <= maximum_bytes,
        f'{label} exceeds its size bound or is empty',
    )
    flags = (
        os.O_RDONLY
        | getattr(os, 'O_CLOEXEC', 0)
        | getattr(os, 'O_NOFOLLOW', 0)
        | getattr(os, 'O_NONBLOCK', 0)
    )
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise EvidenceError(f'cannot open stable regular {label}: {path}') from exc
    try:
        before = os.fstat(descriptor)
        _require(stat.S_ISREG(before.st_mode), f'{label} is not a regular file')
        _require(before.st_nlink == 1, f'{label} is hard-linked')
        _require(
            (allow_empty or before.st_size > 0) and before.st_size <= maximum_bytes,
            f'{label} exceeds its size bound or is empty',
        )
        chunks: list[bytes] = []
        observed = 0
        while True:
            chunk = os.read(descriptor, min(1024 * 1024, maximum_bytes + 1 - observed))
            if not chunk:
                break
            chunks.append(chunk)
            observed += len(chunk)
            _require(observed <= maximum_bytes, f'{label} exceeds its size bound')
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    stable_fields = (
        'st_dev',
        'st_ino',
        'st_mode',
        'st_nlink',
        'st_size',
        'st_mtime_ns',
        'st_ctime_ns',
    )
    _require(
        all(getattr(initial, field) == getattr(before, field) for field in stable_fields)
        and all(getattr(before, field) == getattr(after, field) for field in stable_fields)
        and observed == before.st_size,
        f'{label} changed while it was read',
    )
    payload = b''.join(chunks)
    _require(allow_empty or payload, f'{label} is empty')
    return payload


def _safe_relative(value: object, label: str) -> str:
    _require(isinstance(value, str) and value, f'{label} must be a non-empty string')
    _require(
        '\\' not in value
        and '\x00' not in value
        and '\n' not in value
        and '\r' not in value
        and '\t' not in value,
        f'{label} contains unsafe characters',
    )
    relative = Path(value)
    _require(
        not relative.is_absolute()
        and value == relative.as_posix()
        and all(part not in {'', '.', '..'} for part in relative.parts),
        f'{label} is not a safe canonical relative path: {value}',
    )
    return value


def _contained_path(root: Path, relative: object, label: str) -> Path:
    safe = _safe_relative(relative, label)
    candidate = root / safe
    resolved = candidate.resolve(strict=True)
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise EvidenceError(f'{label} escapes its root') from exc
    _require(resolved == candidate.absolute(), f'{label} resolves through an alias')
    return resolved


def _load_canonical_json(
    path: Path,
    label: str,
    *,
    maximum_bytes: int = MAX_JSON_BYTES,
) -> dict[str, Any]:
    payload = _stable_read(path, label, maximum_bytes=maximum_bytes)
    try:
        value = _mapping(json.loads(payload), label)
        canonical = canonical_json_bytes(value)
    except (UnicodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
        raise EvidenceError(f'cannot load canonical {label}: {exc}') from exc
    _require(payload == canonical, f'{label} is not canonical JSON')
    return value


def _validate_sha_sidecar(path: Path, label: str) -> str:
    payload = _stable_read(path, label, maximum_bytes=MAX_JSON_BYTES)
    sidecar_payload = _stable_read(
        Path(f'{path}.sha256'),
        f'{label} checksum sidecar',
        maximum_bytes=256,
    )
    digest = _sha256_bytes(payload)
    _require(
        sidecar_payload == f'{digest}  {path.name}\n'.encode(),
        f'{label} checksum sidecar mismatch',
    )
    return digest


def _git_environment() -> dict[str, str]:
    return {
        'GIT_ATTR_NOSYSTEM': '1',
        'GIT_CONFIG_GLOBAL': '/dev/null',
        'GIT_CONFIG_NOSYSTEM': '1',
        'GIT_NO_LAZY_FETCH': '1',
        'GIT_NO_REPLACE_OBJECTS': '1',
        'GIT_OPTIONAL_LOCKS': '0',
        'GIT_TERMINAL_PROMPT': '0',
        'HOME': '/nonexistent',
        'LANG': 'C',
        'PATH': '/usr/bin:/bin',
    }


def _git(
    repository: Path,
    arguments: Sequence[str],
    *,
    binary: bool = False,
    timeout: int = 30,
) -> subprocess.CompletedProcess[Any]:
    command = [
        '/usr/bin/git',
        '-C',
        str(repository),
        '-c',
        f'safe.directory={repository}',
        '-c',
        'core.fsmonitor=false',
        '-c',
        'core.hooksPath=/dev/null',
        '-c',
        'core.attributesFile=/dev/null',
        '-c',
        'core.excludesFile=/dev/null',
        '-c',
        'submodule.recurse=false',
        *arguments,
    ]
    return subprocess.run(
        command,
        capture_output=True,
        check=False,
        env=_git_environment(),
        text=not binary,
        timeout=timeout,
    )


def _candidate_blob(repository: Path, candidate_sha: str, relative: str) -> bytes:
    _safe_relative(relative, 'candidate blob path')
    result = _git(repository, ['show', f'{candidate_sha}:{relative}'], binary=True)
    _require(result.returncode == 0, f'candidate lacks tracked {relative}')
    return result.stdout


def _candidate_regular_blob(repository: Path, candidate_sha: str, relative: str) -> bytes:
    listing = _git(repository, ['ls-tree', candidate_sha, '--', relative])
    fields = listing.stdout.rstrip('\n').split(maxsplit=3) if listing.returncode == 0 else []
    _require(
        len(fields) == 4
        and fields[0] == '100644'
        and fields[1] == 'blob'
        and fields[3] == relative,
        f'candidate path is not one exact regular 100644 blob: {relative}',
    )
    return _candidate_blob(repository, candidate_sha, relative)


def _validate_candidate_sha(repository: Path, candidate_sha: str) -> str:
    _require(
        GIT_SHA.fullmatch(candidate_sha) is not None, 'candidate Git SHA must be full lowercase'
    )
    result = _git(repository, ['rev-parse', '--verify', f'{candidate_sha}^{{commit}}'])
    _require(
        result.returncode == 0 and result.stdout.strip() == candidate_sha,
        'candidate Git SHA does not resolve to the exact commit',
    )
    return candidate_sha


def _parse_readme_fences(readme: bytes) -> dict[str, list[dict[str, str]]]:
    _require(0 < len(readme) <= MAX_README_BYTES, 'candidate README exceeds its size bound')
    _require(b'\r' not in readme and readme.endswith(b'\n'), 'candidate README is not LF canonical')
    try:
        text = readme.decode('utf-8')
    except UnicodeError as exc:
        raise EvidenceError('candidate README is not UTF-8') from exc
    lines = text.splitlines(keepends=True)
    result: dict[str, list[dict[str, str]]] = {'bash': [], 'mermaid': []}
    index = 0
    while index < len(lines):
        opening = re.fullmatch(r'```(?P<language>[A-Za-z0-9_-]+)\n', lines[index])
        if opening is None:
            index += 1
            continue
        language = opening.group('language')
        start = index
        index += 1
        body: list[str] = []
        while index < len(lines) and lines[index] != '```\n':
            body.append(lines[index])
            index += 1
        _require(index < len(lines), f'unclosed {language} fence at README line {start + 1}')
        if language in result:
            payload = ''.join(body)
            _require(payload and payload.endswith('\n'), f'empty or noncanonical {language} fence')
            marker = lines[start - 1].strip() if start else ''
            result[language].append(
                {
                    'body': payload,
                    'marker': marker,
                    'source_line': str(start + 1),
                }
            )
        index += 1
    return result


def _documented_command(command: str) -> str:
    lines = command.splitlines()
    _require(
        len(lines) == 2 and lines[0] == f'cd {DOCUMENTED_REPOSITORY}',
        'README replay unit must be exactly documented-root assertion plus one command line',
    )
    executable = lines[1]
    _require(executable and not executable.isspace(), 'README replay command is empty')
    _require('\x00' not in executable and '\r' not in executable, 'README replay command is unsafe')
    _require(
        re.search(r'<[^>\n]+>', executable) is None,
        'README executable fence contains an unresolved placeholder',
    )
    _require(
        not any(token in executable for token in ('#', ';', '|', '&', '<', '>', '`', '$(', '\\')),
        'README replay unit must contain one command without shell control syntax',
    )
    try:
        words = shlex.split(executable, posix=True)
    except ValueError as exc:
        raise EvidenceError(f'README replay command is not parseable: {exc}') from exc
    _require(words, 'README replay command has no argv')
    executable_index = 0
    assignment = re.compile(r'^[A-Z_][A-Z0-9_]*=[^\x00\r\n]*$')
    while executable_index < len(words) and assignment.fullmatch(words[executable_index]):
        executable_index += 1
    _require(executable_index < len(words), 'README replay command contains only assignments')
    invoked = words[executable_index]
    _require(
        invoked.startswith('scripts/') or invoked == 'taskset',
        'README replay command must invoke one reviewed repository script',
    )
    if invoked == 'taskset':
        _require(
            any(word.startswith('scripts/') for word in words[executable_index + 1 :]),
            'README taskset command does not invoke a repository script',
        )
    return executable


def _validate_matrix(
    matrix: dict[str, Any],
    fences: dict[str, list[dict[str, str]]],
) -> dict[str, Any]:
    _require(
        set(matrix) == {'commands', 'diagrams', 'readme_path', 'schema_version'},
        'README replay matrix top-level contract changed',
    )
    _require(matrix.get('schema_version') == 1, 'README replay matrix schema changed')
    _require(matrix.get('readme_path') == README_PATH, 'README replay matrix path changed')
    commands = _list(matrix.get('commands'), 'README replay commands')
    _require(len(commands) == len(fences['bash']) and commands, 'README command count drifted')
    normalized_commands: list[dict[str, Any]] = []
    seen_ids: set[str] = set()
    for ordinal, (raw, fence) in enumerate(zip(commands, fences['bash'], strict=True), 1):
        command = _mapping(raw, f'README replay command {ordinal}')
        _require(
            set(command)
            == {
                'allowed_tracked_changes',
                'authorization',
                'command',
                'expected_exit_code',
                'id',
                'maximum_stderr_bytes',
                'maximum_stdout_bytes',
                'timeout_seconds',
            },
            f'README replay command {ordinal} contract changed',
        )
        identifier = command.get('id')
        _require(
            isinstance(identifier, str)
            and SAFE_ID.fullmatch(identifier) is not None
            and identifier not in seen_ids,
            f'README replay command {ordinal} ID is invalid or duplicated',
        )
        seen_ids.add(identifier)
        source = command.get('command')
        _require(source == fence['body'], f'README replay command drifted at ordinal {ordinal}')
        executable = _documented_command(source)
        authorization = command.get('authorization')
        _require(
            authorization in {'read_only', 'mutating_requires_explicit_authorization'},
            f'README replay command {identifier} authorization is invalid',
        )
        if '--apply' in executable:
            _require(
                authorization == 'mutating_requires_explicit_authorization',
                f'README replay mutating command {identifier} lacks explicit authorization',
            )
        exit_code = command.get('expected_exit_code')
        timeout_seconds = command.get('timeout_seconds')
        stdout_limit = command.get('maximum_stdout_bytes')
        stderr_limit = command.get('maximum_stderr_bytes')
        _require(
            _exact_integer(exit_code) and 0 <= exit_code <= 125,
            f'README replay command {identifier} exit code is invalid',
        )
        _require(
            _exact_integer(timeout_seconds) and 1 <= timeout_seconds <= 14_400,
            f'README replay command {identifier} timeout is invalid',
        )
        for value, stream in ((stdout_limit, 'stdout'), (stderr_limit, 'stderr')):
            _require(
                _exact_integer(value) and 1024 <= value <= MAX_REPLAY_LOG_BYTES,
                f'README replay command {identifier} {stream} bound is invalid',
            )
        allowed = _list(
            command.get('allowed_tracked_changes'),
            f'README replay command {identifier} allowed tracked changes',
        )
        allowed_paths = [_safe_relative(item, 'allowed tracked change') for item in allowed]
        _require(
            allowed_paths == sorted(set(allowed_paths)),
            f'README replay command {identifier} change allowlist is not sorted and unique',
        )
        _require(
            identifier in REPLAY_ALLOWED_CHANGES, f'unknown README replay command ID: {identifier}'
        )
        _require(
            tuple(allowed_paths) == REPLAY_ALLOWED_CHANGES[identifier],
            f'README replay command {identifier} generated-evidence allowlist changed',
        )
        _parse_replay_argv(identifier, executable)
        _require(
            executable == REPLAY_EXECUTABLES[identifier],
            f'README replay executable changed: {identifier}',
        )
        expected_authorization = (
            'mutating_requires_explicit_authorization'
            if identifier in REPLAY_MUTATING_IDS
            else 'read_only'
        )
        _require(
            authorization == expected_authorization,
            f'README replay authorization changed: {identifier}',
        )
        _require(
            exit_code == (3 if identifier == 'aggregate-local' else 0),
            f'README replay exit contract changed: {identifier}',
        )
        _require(
            timeout_seconds == REPLAY_TIMEOUTS[identifier]
            and stdout_limit == stderr_limit == MAX_REPLAY_LOG_BYTES,
            f'README replay resource bounds changed: {identifier}',
        )
        normalized_commands.append(
            {
                **command,
                'command_sha256': _sha256_bytes(source.encode()),
                'documented_root': DOCUMENTED_REPOSITORY,
                'executable': executable,
                'ordinal': ordinal,
                'source_line': int(fence['source_line']),
            }
        )
    _require(
        tuple(command['id'] for command in normalized_commands) == tuple(REPLAY_ALLOWED_CHANGES),
        'README replay command ID set or order changed',
    )
    diagrams = _list(matrix.get('diagrams'), 'README portfolio diagrams')
    _require(len(diagrams) == len(DIAGRAM_IDS), 'README portfolio diagram count changed')
    _require(len(fences['mermaid']) == len(DIAGRAM_IDS), 'README Mermaid fence count changed')
    normalized_diagrams: list[dict[str, Any]] = []
    for ordinal, (identifier, raw, fence) in enumerate(
        zip(DIAGRAM_IDS, diagrams, fences['mermaid'], strict=True), 1
    ):
        diagram = _mapping(raw, f'portfolio diagram {identifier}')
        _require(
            set(diagram) == {'id', 'marker', 'source_path'},
            f'portfolio diagram {identifier} contract changed',
        )
        _require(diagram.get('id') == identifier, 'portfolio diagram order changed')
        _require(
            diagram.get('marker') == DIAGRAM_MARKERS[identifier]
            and fence['marker'] == DIAGRAM_MARKERS[identifier],
            f'portfolio diagram marker changed: {identifier}',
        )
        _require(diagram.get('source_path') == README_PATH, 'portfolio diagram source changed')
        normalized_diagrams.append(
            {
                **diagram,
                'ordinal': ordinal,
                'source': fence['body'],
                'source_line': int(fence['source_line']),
                'source_sha256': _sha256_bytes(fence['body'].encode()),
            }
        )
    return {'commands': normalized_commands, 'diagrams': normalized_diagrams}


def validate_candidate_contract(repository: Path, candidate_sha: str) -> dict[str, Any]:
    """Validate the exact README command and required-Mermaid inventory at C."""
    repository = _resolved_directory(repository, 'repository')
    _require((repository / '.git').is_dir(), 'repository Git directory is missing')
    candidate_sha = _validate_candidate_sha(repository, candidate_sha)
    readme = _candidate_regular_blob(repository, candidate_sha, README_PATH)
    matrix_bytes = _candidate_regular_blob(repository, candidate_sha, REPLAY_MATRIX_PATH)
    tree_entries, tree_listing_sha256 = _candidate_tree_entries(repository, candidate_sha)
    _require(
        0 < len(matrix_bytes) <= MAX_MATRIX_BYTES,
        'candidate README replay matrix exceeds its size bound',
    )
    try:
        matrix = _mapping(json.loads(matrix_bytes), 'README replay matrix')
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceError(f'cannot load README replay matrix: {exc}') from exc
    _require(matrix_bytes == canonical_json_bytes(matrix), 'README replay matrix is not canonical')
    normalized = _validate_matrix(matrix, _parse_readme_fences(readme))
    return {
        'candidate_git_sha': candidate_sha,
        'candidate_tree_entry_count': len(tree_entries),
        'candidate_tree_listing_sha256': tree_listing_sha256,
        'commands': normalized['commands'],
        'diagrams': normalized['diagrams'],
        'matrix_sha256': _sha256_bytes(matrix_bytes),
        'readme_sha256': _sha256_bytes(readme),
        'schema_version': 1,
        'status': 'PASS',
    }


def portfolio_projection_paths(candidate_sha: str) -> tuple[str, ...]:
    """Return the exact six evidence-commit paths for candidate C."""
    _require(
        GIT_SHA.fullmatch(candidate_sha) is not None, 'candidate Git SHA must be full lowercase'
    )
    prefix = PORTFOLIO_PROJECTION_ROOT
    return (
        f'{prefix}/portfolio-{candidate_sha}.json',
        f'{prefix}/portfolio-{candidate_sha}.SHA256SUMS',
        f'{prefix}/portfolio-{candidate_sha}.validation.txt',
        f'{prefix}/architecture-{candidate_sha}.svg',
        f'{prefix}/release-flow-{candidate_sha}.svg',
        f'{prefix}/scenario5-localization-error-{candidate_sha}.png',
    )


def _png_dimensions(path: Path, label: str) -> tuple[int, int]:
    payload = _stable_read(path, label, maximum_bytes=MAX_PNG_BYTES)
    _require(payload.startswith(b'\x89PNG\r\n\x1a\n'), f'{label} PNG signature is invalid')
    offset = 8
    chunks: list[bytes] = []
    width = height = 0
    saw_iend = False
    while offset < len(payload):
        _require(offset + 12 <= len(payload), f'{label} PNG chunk header is truncated')
        length = struct.unpack('>I', payload[offset : offset + 4])[0]
        chunk_type = payload[offset + 4 : offset + 8]
        end = offset + 12 + length
        _require(end <= len(payload), f'{label} PNG chunk is truncated')
        data = payload[offset + 8 : offset + 8 + length]
        expected_crc = struct.unpack('>I', payload[offset + 8 + length : end])[0]
        actual_crc = zlib.crc32(chunk_type + data) & 0xFFFFFFFF
        _require(actual_crc == expected_crc, f'{label} PNG chunk CRC is invalid')
        _require(not saw_iend, f'{label} PNG has trailing chunks')
        if not chunks:
            _require(chunk_type == b'IHDR' and length == 13, f'{label} PNG IHDR is invalid')
            width, height = struct.unpack('>II', data[:8])
        if chunk_type == b'IEND':
            _require(length == 0, f'{label} PNG IEND is invalid')
            saw_iend = True
        chunks.append(chunk_type)
        offset = end
    _require(
        saw_iend and offset == len(payload), f'{label} PNG is incomplete or has trailing bytes'
    )
    _require(chunks.count(b'IHDR') == 1 and b'IDAT' in chunks, f'{label} PNG chunks are incomplete')
    _require((width, height) == (960, 540), f'{label} PNG dimensions are not 960x540')
    return width, height


def _svg_dimensions(path: Path, label: str) -> tuple[float, float]:
    payload = _stable_read(path, label, maximum_bytes=MAX_SVG_BYTES)
    _require(b'\x00' not in payload and b'\r' not in payload, f'{label} SVG is not canonical text')
    try:
        text = payload.decode('utf-8')
    except UnicodeError as exc:
        raise EvidenceError(f'{label} SVG is not UTF-8') from exc
    _require('\\' not in text, f'{label} SVG contains a CSS escape')
    lowered = text.casefold()
    _require(
        '<!doctype' not in lowered and '<!entity' not in lowered,
        f'{label} SVG contains a document type or entity',
    )
    xml_declaration = re.compile(
        r"\A<\?xml\s+version\s*=\s*(['\"])1\.0\1"
        r"(?:\s+encoding\s*=\s*(['\"])UTF-8\2)?"
        r"(?:\s+standalone\s*=\s*(['\"])(?:yes|no)\3)?\s*\?>",
        re.IGNORECASE,
    )
    text_without_declaration = xml_declaration.sub('', text, count=1)
    _require(
        '<?' not in text_without_declaration,
        f'{label} SVG contains a processing instruction',
    )
    try:
        parser = ET.XMLParser(target=ET.TreeBuilder(insert_pis=True))
        root = ET.fromstring(text, parser=parser)
    except ET.ParseError as exc:
        raise EvidenceError(f'{label} SVG is malformed: {exc}') from exc
    local_root = root.tag.rsplit('}', 1)[-1]
    _require(local_root == 'svg', f'{label} SVG root is invalid')
    view_box = root.attrib.get('viewBox')
    _require(isinstance(view_box, str), f'{label} SVG lacks a viewBox')
    try:
        values = [float(value) for value in re.split(r'[ ,]+', view_box.strip())]
    except ValueError as exc:
        raise EvidenceError(f'{label} SVG viewBox is invalid') from exc
    _require(
        len(values) == 4
        and all(value == value and abs(value) != float('inf') for value in values)
        and values[2] >= 320
        and values[3] >= 180
        and values[2] <= 8192
        and values[3] <= 8192,
        f'{label} SVG viewBox is outside the accepted bounds',
    )
    forbidden_elements = {
        'animate',
        'animatecolor',
        'animatemotion',
        'animatetransform',
        'animation',
        'audio',
        'discard',
        'embed',
        'foreignobject',
        'handler',
        'iframe',
        'image',
        'listener',
        'object',
        'prefetch',
        'script',
        'set',
        'video',
    }
    unsafe_text = re.compile(
        r'(?:@import|expression\s*\(|javascript\s*:|data\s*:|file\s*:|https?\s*:'
        r'|/\*|(?:-[a-z][a-z0-9]*-)?'
        r'(?:cross-fade|image|image-set|image-rect|paint)\s*\()',
        re.IGNORECASE,
    )
    unsafe_url = re.compile(r'url\s*\(\s*(?!#[A-Za-z_][A-Za-z0-9_.:-]*\s*\))', re.IGNORECASE)
    for element in root.iter():
        _require(
            isinstance(element.tag, str),
            f'{label} SVG contains a processing instruction',
        )
        local_name = element.tag.rsplit('}', 1)[-1]
        _require(
            local_name.casefold() not in forbidden_elements,
            f'{label} SVG contains {local_name}',
        )
        if local_name.casefold() == 'style':
            _require(
                len(element) == 0,
                f'{label} SVG contains unsafe CSS',
            )
            style_text = ''.join(element.itertext())
            _require(
                '\\' not in style_text
                and unsafe_text.search(style_text) is None
                and unsafe_url.search(style_text) is None,
                f'{label} SVG contains unsafe CSS',
            )
        for text_fragment in (element.text, element.tail):
            if not text_fragment:
                continue
            _require(
                '\\' not in text_fragment
                and unsafe_text.search(text_fragment) is None
                and unsafe_url.search(text_fragment) is None,
                f'{label} SVG contains unsafe CSS',
            )
        for raw_name, value in element.attrib.items():
            name = raw_name.rsplit('}', 1)[-1]
            _require(not name.casefold().startswith('on'), f'{label} SVG contains an event handler')
            _require(name.casefold() != 'base', f'{label} SVG contains an external base')
            _require(
                '\\' not in value
                and unsafe_text.search(value) is None
                and unsafe_url.search(value) is None,
                f'{label} SVG contains an unsafe URI',
            )
            if name in {'href', 'src'}:
                _require(value.startswith('#'), f'{label} SVG contains an external reference')
    return values[2], values[3]


def _artifact_records(
    result_directory: Path,
    manifest: Mapping[str, Any],
) -> dict[str, dict[str, Any]]:
    artifacts = _list(manifest.get('artifacts'), 'Scenario 5 artifact manifest records')
    records: dict[str, dict[str, Any]] = {}
    total = 0
    for raw in artifacts:
        record = _mapping(raw, 'Scenario 5 artifact record')
        _require(set(record) == {'bytes', 'path', 'sha256'}, 'Scenario 5 artifact record changed')
        relative = _safe_relative(record.get('path'), 'Scenario 5 artifact path')
        _require(relative not in records, f'duplicate Scenario 5 artifact path: {relative}')
        size = record.get('bytes')
        digest = record.get('sha256')
        _require(
            _exact_integer(size) and 0 < size <= MAX_RESULT_ARTIFACT_BYTES,
            f'Scenario 5 artifact size is invalid: {relative}',
        )
        _require(
            isinstance(digest, str) and SHA256.fullmatch(digest) is not None,
            f'Scenario 5 artifact digest is invalid: {relative}',
        )
        artifact = _contained_path(result_directory, relative, 'Scenario 5 artifact')
        payload = _stable_read(
            artifact,
            f'Scenario 5 artifact {relative}',
            maximum_bytes=MAX_RESULT_ARTIFACT_BYTES,
        )
        _require(
            len(payload) == size and _sha256_bytes(payload) == digest,
            f'Scenario 5 artifact does not match its manifest: {relative}',
        )
        records[relative] = dict(record)
        total += size
    _require(set(records) == EXPECTED_RESULT_ARTIFACTS, 'Scenario 5 artifact path set changed')
    _require(total <= MAX_RESULT_DIRECTORY_BYTES, 'Scenario 5 result bundle exceeds its size bound')
    quality = _mapping(manifest.get('quality'), 'Scenario 5 artifact quality')
    _require(
        set(quality)
        == {
            'artifact_bytes_excluding_manifest',
            'artifact_count',
            'caps_within_limits',
            'hashes_verified',
            'path_set_complete',
        }
        and quality.get('artifact_bytes_excluding_manifest') == total
        and quality.get('artifact_count') == len(records)
        and quality.get('caps_within_limits') is True
        and quality.get('hashes_verified') is True
        and quality.get('path_set_complete') is True,
        'Scenario 5 artifact quality is not canonical PASS',
    )
    return records


def _all_checks_pass(value: object, label: str) -> bool:
    checks = _list(value, label)
    return bool(checks) and all(
        isinstance(item, dict) and item.get('passed') is True for item in checks
    )


def validate_scenario5_media(
    phase3_candidate_root: Path,
    candidate_sha: str,
) -> dict[str, Any]:
    """Independently join deterministic run 12 to suite, aggregate, manifest, and PNG."""
    _require(
        GIT_SHA.fullmatch(candidate_sha) is not None, 'candidate Git SHA must be full lowercase'
    )
    root = _resolved_directory(phase3_candidate_root, 'Phase 3 candidate root')
    result_path = _regular_file(
        root / SCENARIO5_RESULT_RELATIVE,
        'Scenario 5 run result',
        maximum_bytes=MAX_JSON_BYTES,
    )
    result_directory = result_path.parent
    suite_path = root / 'suite-plan.json'
    aggregate_path = root / 'aggregate/aggregate-result.json'
    suite = _load_canonical_json(suite_path, 'Phase 3 suite plan')
    suite_sha = _validate_sha_sidecar(suite_path, 'Phase 3 suite plan')
    aggregate = _load_canonical_json(aggregate_path, 'Phase 3 aggregate')
    result = _load_canonical_json(result_path, 'Scenario 5 run result')
    result_sha = file_sha256(result_path)
    manifest_path = result_directory / 'run-artifacts.manifest.json'
    manifest = _load_canonical_json(manifest_path, 'Scenario 5 artifact manifest')
    manifest_sha = _validate_sha_sidecar(manifest_path, 'Scenario 5 artifact manifest')
    _require(
        set(manifest) == {'artifacts', 'identity', 'producer', 'quality', 'schema_version'}
        and manifest.get('schema_version') == 1
        and manifest.get('producer') == 'robotest_metrics/metrics_analyze',
        'Scenario 5 artifact manifest contract changed',
    )
    records = _artifact_records(result_directory, manifest)
    trials = _list(suite.get('trials'), 'Phase 3 suite trials')
    _require(
        suite.get('schema_version') == 1
        and suite.get('producer') == 'robotest_phase3/benchmark_orchestrator'
        and len(trials) == 15,
        'Phase 3 suite plan is not the exact 15-run contract',
    )
    selected = _mapping(trials[12], 'Phase 3 suite trial 12')
    identity = _mapping(result.get('identity'), 'Scenario 5 result identity')
    verdict = _mapping(result.get('verdict'), 'Scenario 5 result verdict')
    _require(
        selected.get('suite_index') == 12
        and selected.get('scenario_id') == 5
        and selected.get('repetition_index') == 0
        and selected.get('scenario_name') == SCENARIO5_NAME,
        'Phase 3 suite trial 12 is not deterministic Scenario 5 repetition 0',
    )
    candidate_id = suite.get('candidate_id')
    _require(
        isinstance(candidate_id, str)
        and selected.get('candidate_id') == candidate_id
        and identity.get('candidate_id') == candidate_id
        and identity.get('git_sha') == candidate_sha
        and identity.get('git_dirty') is False
        and identity.get('cold_stack') is True
        and identity.get('suite_index') == 12
        and identity.get('scenario_id') == 5
        and identity.get('scenario_index') == 5
        and identity.get('repetition_index') == 0
        and identity.get('scenario_name') == SCENARIO5_NAME
        and identity.get('run_id') == selected.get('run_id')
        and identity.get('scenario_sha256') == selected.get('scenario_sha256'),
        'Scenario 5 result identity does not match suite trial 12 and candidate C',
    )
    _require(
        verdict.get('automated_status') == 'PASS'
        and verdict.get('capture_integrity') is True
        and verdict.get('components_complete') is True
        and verdict.get('exit_code') == 0
        and verdict.get('mission_success') is True
        and verdict.get('scenario_metric_gate') is True
        and _all_checks_pass(verdict.get('required_metric_checks'), 'required metric checks')
        and _all_checks_pass(verdict.get('threshold_checks'), 'threshold checks'),
        'Scenario 5 result is not a full canonical PASS',
    )
    manifest_identity = _mapping(manifest.get('identity'), 'Scenario 5 manifest identity')
    _require(
        set(manifest_identity) == {'run_id', 'run_result_sha256'}
        and manifest_identity.get('run_id') == identity.get('run_id')
        and manifest_identity.get('run_result_sha256') == result_sha
        and records['run-result.json']['sha256'] == result_sha,
        'Scenario 5 manifest does not bind its run result',
    )
    aggregate_identity = _mapping(aggregate.get('identity'), 'Phase 3 aggregate identity')
    aggregate_verdict = _mapping(aggregate.get('verdict'), 'Phase 3 aggregate verdict')
    ordered_ids = _list(aggregate_identity.get('ordered_run_ids'), 'aggregate ordered run IDs')
    ordered_hashes = _list(
        aggregate_identity.get('ordered_source_json_sha256'),
        'aggregate ordered source result hashes',
    )
    _require(
        aggregate_identity.get('candidate_id') == candidate_id
        and aggregate_identity.get('git_sha') == candidate_sha
        and aggregate_identity.get('trial_count') == 15
        and len(ordered_ids) == len(ordered_hashes) == 15
        and ordered_ids[12] == identity.get('run_id')
        and ordered_hashes[12] == result_sha
        and aggregate_verdict.get('automated_status') == 'PASS',
        'Phase 3 aggregate does not bind selected Scenario 5 run 12 and candidate C',
    )
    scenario5 = _mapping(
        _mapping(aggregate.get('scenarios'), 'Phase 3 aggregate scenarios').get('5'),
        'Phase 3 Scenario 5 aggregate',
    )
    scenario_run_ids = _list(scenario5.get('run_ids'), 'Phase 3 Scenario 5 run IDs')
    scenario_verdicts = _list(scenario5.get('run_verdicts'), 'Phase 3 Scenario 5 verdicts')
    _require(
        scenario5.get('denominator') == 3
        and scenario5.get('numerator') == 3
        and scenario5.get('success_rate') == 1.0
        and scenario5.get('verdict') == 'PASS'
        and scenario_run_ids == ordered_ids[12:15]
        and [item.get('run_id') for item in scenario_verdicts if isinstance(item, dict)]
        == scenario_run_ids
        and all(
            isinstance(item, dict) and item.get('status') == 'PASS' for item in scenario_verdicts
        ),
        'Phase 3 Scenario 5 aggregate is not exact 3/3 PASS',
    )
    chart_path = result_directory / 'localization-error.png'
    width, height = _png_dimensions(chart_path, 'Scenario 5 localization-error chart')
    chart = records['localization-error.png']
    _require(
        chart.get('bytes') == chart_path.stat().st_size
        and chart.get('sha256') == file_sha256(chart_path),
        'Scenario 5 chart does not match the artifact manifest',
    )
    return {
        'aggregate_relative_path': 'aggregate/aggregate-result.json',
        'aggregate_sha256': file_sha256(aggregate_path),
        'candidate_id': candidate_id,
        'chart_bytes': chart['bytes'],
        'chart_dimensions': {'height': height, 'width': width},
        'chart_relative_path': 'runs/12/result/localization-error.png',
        'chart_sha256': chart['sha256'],
        'manifest_relative_path': 'runs/12/result/run-artifacts.manifest.json',
        'manifest_sha256': manifest_sha,
        'result_relative_path': SCENARIO5_RESULT_RELATIVE,
        'result_sha256': result_sha,
        'run_id': identity['run_id'],
        'suite_plan_relative_path': 'suite-plan.json',
        'suite_plan_sha256': suite_sha,
        'suite_index': 12,
    }


def _write_once(path: Path, payload: bytes, *, mode: int = 0o600) -> None:
    _require(
        _exact_integer(mode) and mode in {0o600, 0o644, 0o755},
        f'evidence mode is invalid: {mode!r}',
    )
    _reject_symlink_components(path.parent, 'evidence parent')
    _require(not path.exists() and not path.is_symlink(), f'refusing to overwrite evidence: {path}')
    missing_parents: list[Path] = []
    cursor = path.parent
    while not cursor.exists():
        missing_parents.append(cursor)
        cursor = cursor.parent
    path.parent.mkdir(parents=True, exist_ok=True)
    _reject_symlink_components(path.parent, 'evidence parent')
    if mode != 0o644:
        for created_parent in reversed(missing_parents):
            created_parent.chmod(0o700)
    parent_metadata = path.parent.lstat()
    _require(
        stat.S_ISDIR(parent_metadata.st_mode),
        f'evidence parent is not a directory: {path.parent}',
    )
    if mode != 0o644:
        _require(
            stat.S_IMODE(parent_metadata.st_mode) == 0o700,
            f'evidence parent mode differs from the requested mode: {path.parent}',
        )
    descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, mode)
    try:
        os.fchmod(descriptor, mode)
        _require(
            stat.S_IMODE(os.fstat(descriptor).st_mode) == mode,
            f'evidence mode differs from the requested mode: {path}',
        )
        view = memoryview(payload)
        while view:
            written = os.write(descriptor, view)
            _require(written > 0, f'cannot complete evidence write: {path}')
            view = view[written:]
        os.fsync(descriptor)
        metadata = os.fstat(descriptor)
        _require(
            stat.S_ISREG(metadata.st_mode)
            and metadata.st_nlink == 1
            and stat.S_IMODE(metadata.st_mode) == mode,
            f'evidence mode or identity changed during write: {path}',
        )
    except BaseException:
        os.close(descriptor)
        path.unlink(missing_ok=True)
        raise
    os.close(descriptor)


def _write_json_once(path: Path, value: object) -> None:
    _write_once(path, canonical_json_bytes(value))


def _copy_once(source: Path, destination: Path, label: str, *, maximum_bytes: int) -> None:
    payload = _stable_read(source, label, maximum_bytes=maximum_bytes)
    _write_once(destination, payload)


def _attempt_identifier(value: str | None = None) -> str:
    if value is None:
        value = datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ') + f'-{os.getpid()}'
    _require(ATTEMPT_ID.fullmatch(value) is not None, 'portfolio attempt ID is invalid')
    return value


def _portfolio_root(repository: Path, candidate_sha: str, *, create: bool) -> Path:
    expected = repository / PORTFOLIO_RAW_PREFIX / candidate_sha
    if create:
        _reject_symlink_components(expected, 'Phase 5 portfolio root')
        expected.mkdir(parents=True, exist_ok=True)
    root = _resolved_directory(expected, 'Phase 5 portfolio root')
    for child in root.iterdir():
        _require(
            child.name == 'attempts' and child.is_dir() and not child.is_symlink(),
            f'unexpected Phase 5 portfolio root entry: {child.name}',
        )
        for attempt in child.iterdir():
            _require(
                ATTEMPT_ID.fullmatch(attempt.name) is not None
                and attempt.is_dir()
                and not attempt.is_symlink(),
                f'invalid Phase 5 portfolio attempt entry: {attempt.name}',
            )
            for path in attempt.rglob('*'):
                _require(not path.is_symlink(), f'portfolio attempt contains a symlink: {path}')
                metadata = path.lstat()
                _require(
                    stat.S_ISDIR(metadata.st_mode)
                    or (stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1),
                    f'portfolio attempt contains a special or hard-linked file: {path}',
                )
    return root


def _attempt_directory(portfolio_root: Path, attempt_id: str, *, create: bool) -> Path:
    attempt_id = _attempt_identifier(attempt_id)
    attempts = portfolio_root / 'attempts'
    if create:
        _reject_symlink_components(attempts, 'portfolio attempts root')
        attempts.mkdir(parents=True, exist_ok=True)
        _reject_symlink_components(attempts, 'portfolio attempts root')
        attempt = attempts / attempt_id
        _reject_symlink_components(attempt, 'portfolio attempt')
        _require(
            not attempt.exists() and not attempt.is_symlink(), 'portfolio attempt already exists'
        )
        attempt.mkdir(mode=0o700)
    attempt = attempts / attempt_id
    return _resolved_directory(attempt, 'portfolio attempt')


def _parse_replay_argv(command_id: str, executable: str) -> tuple[dict[str, str], list[str]]:
    words = shlex.split(executable, posix=True)
    assignments: dict[str, str] = {}
    assignment = re.compile(r'^(?P<name>[A-Z_][A-Z0-9_]*)=(?P<value>[^\x00\r\n]*)$')
    index = 0
    while index < len(words):
        match = assignment.fullmatch(words[index])
        if match is None:
            break
        name = match.group('name')
        _require(name not in FORBIDDEN_REPLAY_ENVIRONMENT, f'forbidden replay assignment: {name}')
        _require(name not in assignments, f'duplicate replay assignment: {name}')
        assignments[name] = match.group('value')
        index += 1
    argv = words[index:]
    _require(argv, 'README replay command has no executable')
    _require(
        assignments == REPLAY_ASSIGNMENTS[command_id],
        f'README replay assignments changed: {command_id}',
    )
    return assignments, argv


def _read_process_identity(pid: int, proc_root: Path = Path('/proc')) -> _ProcessIdentity:
    _require(_exact_integer(pid) and pid > 0, 'replay process PID is invalid')
    path = proc_root / str(pid) / 'stat'
    try:
        with path.open('rb', buffering=0) as stream:
            payload = stream.read(16_385)
    except (FileNotFoundError, ProcessLookupError):
        raise
    except OSError as exc:
        raise EvidenceError(f'cannot read replay process identity for PID {pid}') from exc
    _require(len(payload) <= 16_384, f'replay process identity exceeds byte cap: PID {pid}')
    opening = payload.find(b'(')
    closing = payload.rfind(b') ')
    _require(opening > 0 and closing > opening, f'replay process identity is malformed: PID {pid}')
    try:
        pid_text = payload[:opening].strip().decode('ascii')
        fields = payload[closing + 2 :].decode('ascii').split()
    except UnicodeError as exc:
        raise EvidenceError(f'replay process identity fields are not ASCII: PID {pid}') from exc
    _require(len(fields) >= 20, f'replay process identity is truncated: PID {pid}')
    try:
        observed_pid = int(pid_text)
        state = fields[0]
        identity = _ProcessIdentity(
            pid=observed_pid,
            command=payload[opening + 1 : closing].decode('utf-8', errors='replace'),
            state=state,
            parent_pid=int(fields[1]),
            process_group_id=int(fields[2]),
            session_id=int(fields[3]),
            start_ticks=int(fields[19]),
        )
    except ValueError as exc:
        raise EvidenceError(f'replay process identity has a non-integer field: PID {pid}') from exc
    _require(
        identity.pid == pid and len(identity.state) == 1 and identity.start_ticks > 0,
        f'replay process identity fields are invalid: PID {pid}',
    )
    return identity


def _pin_process_group(
    process: subprocess.Popen[bytes], proc_root: Path = Path('/proc')
) -> _ProcessIdentity:
    identity = _read_process_identity(process.pid, proc_root)
    _require(
        identity.pid == identity.process_group_id == identity.session_id,
        'replay process is not its exact process-group and session leader',
    )
    return identity


def _process_group_members(
    leader: _ProcessIdentity, proc_root: Path = Path('/proc')
) -> list[_ProcessIdentity]:
    members: list[_ProcessIdentity] = []
    inspected = 0
    try:
        entries = proc_root.iterdir()
        for entry in entries:
            if not entry.name.isdigit():
                continue
            inspected += 1
            _require(
                inspected <= MAX_PROCESS_TABLE_ENTRIES,
                'process table exceeds replay cleanup scan cap',
            )
            try:
                identity = _read_process_identity(int(entry.name), proc_root)
            except (FileNotFoundError, ProcessLookupError):
                continue
            if identity.process_group_id != leader.process_group_id:
                continue
            _require(
                identity.session_id == leader.session_id,
                'replay process group crossed its pinned session identity',
            )
            members.append(identity)
    except OSError as exc:
        raise EvidenceError('cannot scan replay process group membership') from exc
    members.sort(key=lambda item: item.pid)
    pinned = [item for item in members if item.pid == leader.pid]
    _require(
        len(pinned) == 1
        and pinned[0].process_group_id == leader.process_group_id
        and pinned[0].session_id == leader.session_id
        and pinned[0].start_ticks == leader.start_ticks,
        'replay process-group leader identity changed before cleanup',
    )
    return members


def _live_process_group_members(
    leader: _ProcessIdentity, proc_root: Path = Path('/proc')
) -> list[_ProcessIdentity]:
    return [
        identity
        for identity in _process_group_members(leader, proc_root)
        if identity.state not in QUIESCENT_PROCESS_STATES
    ]


def _stable_live_process_group_members(leader: _ProcessIdentity) -> list[_ProcessIdentity]:
    live = _live_process_group_members(leader)
    if live:
        return live
    time.sleep(PROCESS_GROUP_POLL_INTERVAL_S)
    return _live_process_group_members(leader)


def _leader_exited_without_reaping(leader: _ProcessIdentity) -> bool:
    try:
        status = os.waitid(
            os.P_PID,
            leader.pid,
            os.WEXITED | os.WNOHANG | os.WNOWAIT,
        )
    except ChildProcessError as exc:
        raise EvidenceError('replay process leader was reaped before group cleanup') from exc
    return status is not None


def _wait_for_leader_exit(leader: _ProcessIdentity, deadline: float) -> bool:
    while not _leader_exited_without_reaping(leader):
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return False
        time.sleep(min(PROCESS_GROUP_POLL_INTERVAL_S, remaining))
    return True


def _signal_process_group(leader: _ProcessIdentity, signum: int) -> None:
    current = _read_process_identity(leader.pid)
    _require(
        current.pid == leader.pid
        and current.process_group_id == leader.process_group_id
        and current.session_id == leader.session_id
        and current.start_ticks == leader.start_ticks,
        'replay process-group leader identity changed before signal',
    )
    try:
        os.killpg(leader.process_group_id, signum)
    except ProcessLookupError:
        return
    except PermissionError as exc:
        raise EvidenceError('cannot signal pinned replay process group') from exc


def _wait_for_process_group_quiescence(
    leader: _ProcessIdentity, deadline: float
) -> list[_ProcessIdentity]:
    empty_scan_seen = False
    while True:
        live = _live_process_group_members(leader)
        if not live:
            if empty_scan_seen:
                return []
            empty_scan_seen = True
        else:
            empty_scan_seen = False
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return live if live else _live_process_group_members(leader)
        time.sleep(min(PROCESS_GROUP_POLL_INTERVAL_S, remaining))


def _cleanup_unpinned_process_group(process: subprocess.Popen[bytes]) -> None:
    signal_error: PermissionError | None = None
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except PermissionError as exc:
        signal_error = exc
    try:
        process.wait(timeout=PROCESS_GROUP_KILL_GRACE_S)
    except subprocess.TimeoutExpired as exc:
        raise EvidenceError('unvalidated replay process group did not terminate') from exc
    if signal_error is not None:
        raise EvidenceError('cannot clean unvalidated replay process group') from signal_error


def _terminate_process_group(
    process: subprocess.Popen[bytes],
    leader: _ProcessIdentity,
    *,
    forced_cleanup: bool,
) -> None:
    normal_survivor = False
    if not forced_cleanup:
        normal_survivor = not _wait_for_leader_exit(
            leader, time.monotonic() + PROCESS_GROUP_TERM_GRACE_S
        )

    live = _stable_live_process_group_members(leader)
    term_sent = bool(live)
    if live:
        normal_survivor = normal_survivor or not forced_cleanup
        _signal_process_group(leader, signal.SIGTERM)
        live = _wait_for_process_group_quiescence(
            leader, time.monotonic() + PROCESS_GROUP_TERM_GRACE_S
        )
    if term_sent:
        _signal_process_group(leader, signal.SIGKILL)
        live = _wait_for_process_group_quiescence(
            leader, time.monotonic() + PROCESS_GROUP_KILL_GRACE_S
        )
    if live:
        with suppress(subprocess.TimeoutExpired):
            process.wait(timeout=0)
        raise EvidenceError('replay process group survived bounded SIGKILL cleanup')

    try:
        process.wait(timeout=PROCESS_GROUP_KILL_GRACE_S)
    except subprocess.TimeoutExpired as exc:
        raise EvidenceError('replay process leader survived quiescent group cleanup') from exc
    if normal_survivor:
        raise EvidenceError('replay process group survived normal command completion')


def _run_bounded(
    argv: Sequence[str],
    *,
    cwd: Path,
    environment: Mapping[str, str],
    timeout_seconds: int,
    stdout_limit: int,
    stderr_limit: int,
) -> dict[str, Any]:
    command = [
        '/usr/bin/env',
        '-i',
        *(f'{name}={value}' for name, value in sorted(environment.items())),
        *argv,
    ]
    started = time.monotonic_ns()
    process = subprocess.Popen(
        command,
        cwd=cwd,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    retained = {'stdout': bytearray(), 'stderr': bytearray()}
    observed = {'stdout': 0, 'stderr': 0}
    limits = {'stdout': stdout_limit, 'stderr': stderr_limit}
    overflow = {'stdout': False, 'stderr': False}
    timed_out = False
    deadline = time.monotonic() + timeout_seconds
    leader: _ProcessIdentity | None = None
    selector: selectors.BaseSelector | None = None
    interrupted = False
    try:
        leader = _pin_process_group(process)
        _require(
            process.stdout is not None and process.stderr is not None,
            'replay pipes unavailable',
        )
        selector = selectors.DefaultSelector()
        selector.register(process.stdout, selectors.EVENT_READ, 'stdout')
        selector.register(process.stderr, selectors.EVENT_READ, 'stderr')
        while selector.get_map():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                timed_out = True
                break
            events = selector.select(min(remaining, 0.25))
            if not events and _leader_exited_without_reaping(leader):
                events = selector.select(0)
                if not events:
                    break
            for key, _ in events:
                chunk = os.read(key.fileobj.fileno(), 65_536)
                if not chunk:
                    selector.unregister(key.fileobj)
                    continue
                stream = key.data
                observed[stream] += len(chunk)
                remaining_capacity = max(0, limits[stream] - len(retained[stream]))
                retained[stream].extend(chunk[:remaining_capacity])
                if observed[stream] > limits[stream]:
                    overflow[stream] = True
            if any(overflow.values()):
                break
    except BaseException:
        interrupted = True
        raise
    finally:
        if selector is not None:
            selector.close()
        if leader is None:
            _cleanup_unpinned_process_group(process)
        else:
            _terminate_process_group(
                process,
                leader,
                forced_cleanup=interrupted or timed_out or any(overflow.values()),
            )
    return {
        'argv': list(argv),
        'elapsed_wall_ns': time.monotonic_ns() - started,
        'returncode': process.returncode,
        'stderr': bytes(retained['stderr']),
        'stderr_observed_bytes': observed['stderr'],
        'stderr_overflow': overflow['stderr'],
        'stdout': bytes(retained['stdout']),
        'stdout_observed_bytes': observed['stdout'],
        'stdout_overflow': overflow['stdout'],
        'timed_out': timed_out,
        'wrapped_argv': command,
    }


def _candidate_tree_entries(
    repository: Path,
    candidate_sha: str,
) -> tuple[list[dict[str, str]], str]:
    listing = _git(repository, ['ls-tree', '-rz', candidate_sha], binary=True)
    _require(listing.returncode == 0 and listing.stderr == b'', 'cannot inspect candidate tree')
    entries: list[dict[str, str]] = []
    for raw in listing.stdout.split(b'\0'):
        if not raw:
            continue
        try:
            header, raw_path = raw.split(b'\t', 1)
            mode, kind, object_id = header.decode('ascii').split(' ')
            relative = raw_path.decode('utf-8')
        except (UnicodeError, ValueError) as exc:
            raise EvidenceError('candidate tree entry is malformed') from exc
        _safe_relative(relative, 'candidate tree path')
        _require(
            kind == 'blob'
            and mode in {'100644', '100755'}
            and GIT_SHA.fullmatch(object_id) is not None,
            f'candidate tree contains an unsupported mode or object: {relative}',
        )
        entries.append(
            {
                'mode': mode,
                'object_id': object_id,
                'path': relative,
            }
        )
    _require(entries, 'candidate tree is empty')
    return entries, _sha256_bytes(listing.stdout)


def _git_blob_sha1(payload: bytes) -> str:
    digest = hashlib.sha1(usedforsecurity=False)
    digest.update(f'blob {len(payload)}\0'.encode('ascii'))
    digest.update(payload)
    return digest.hexdigest()


def _expected_tracked_tree_state(
    repository: Path,
    candidate_sha: str,
    allowed_modified_paths: Sequence[str],
) -> dict[str, Any]:
    entries, listing_sha256 = _candidate_tree_entries(repository, candidate_sha)
    allowed = list(allowed_modified_paths)
    _require(
        len(allowed) == len(set(allowed)) and set(allowed) <= {entry['path'] for entry in entries},
        'tracked-tree allowed modified paths are invalid',
    )
    return {
        'allowed_modified_paths': allowed,
        'candidate_tree_entry_count': len(entries),
        'candidate_tree_listing_sha256': listing_sha256,
        'checked_immutable_path_count': len(entries) - len(allowed),
        'schema_version': 1,
        'status': 'PASS',
    }


def _tracked_tree_state(
    worktree: Path,
    candidate_sha: str,
    allowed_modified_paths: Sequence[str],
) -> dict[str, Any]:
    entries, _listing_sha256 = _candidate_tree_entries(worktree, candidate_sha)
    allowed = list(allowed_modified_paths)
    _require(
        len(allowed) == len(set(allowed)) and set(allowed) <= {entry['path'] for entry in entries},
        'tracked-tree allowed modified paths are invalid',
    )
    checked = 0
    for entry in entries:
        relative = entry['path']
        path = worktree / relative
        _reject_symlink_components(path, f'tracked candidate path {relative}')
        try:
            metadata = path.lstat()
        except FileNotFoundError as exc:
            raise EvidenceError(f'missing tracked candidate path: {relative}') from exc
        _require(
            stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1,
            f'tracked candidate path is not one regular file: {relative}',
        )
        expected_executable = entry['mode'] == '100755'
        _require(
            bool(metadata.st_mode & 0o111) == expected_executable,
            f'tracked candidate executable mode changed: {relative}',
        )
        if relative in allowed:
            continue
        payload = _stable_read(
            path,
            f'tracked candidate path {relative}',
            maximum_bytes=MAX_RESULT_DIRECTORY_BYTES,
            allow_empty=True,
        )
        _require(
            _git_blob_sha1(payload) == entry['object_id'],
            f'tracked candidate bytes differ from C without filters: {relative}',
        )
        checked += 1
    expected = _expected_tracked_tree_state(worktree, candidate_sha, allowed)
    _require(
        checked == expected['checked_immutable_path_count'],
        'tracked candidate immutable path count changed',
    )
    return expected


def _worktree_state(
    worktree: Path,
    candidate_sha: str,
    label: str,
    *,
    allowed_tracked_changes: Sequence[str] = (),
) -> dict[str, Any]:
    head = _git(worktree, ['rev-parse', '--verify', 'HEAD'])
    detached = _git(worktree, ['symbolic-ref', '-q', 'HEAD'])
    status_result = _git(
        worktree, ['status', '--porcelain=v1', '-z', '--untracked-files=all'], binary=True
    )
    changed_result = _git(worktree, ['diff', '--name-only', '-z', 'HEAD', '--'], binary=True)
    staged_result = _git(worktree, ['diff', '--cached', '--name-only', '-z', '--'], binary=True)
    untracked_result = _git(
        worktree,
        ['ls-files', '--others', '--exclude-standard', '-z'],
        binary=True,
    )
    _require(
        head.returncode
        == status_result.returncode
        == changed_result.returncode
        == staged_result.returncode
        == untracked_result.returncode
        == 0,
        f'cannot inspect {label}',
    )
    _require(detached.returncode == 1 and detached.stdout == '', f'{label} is not detached')
    changed = sorted(
        {
            value.decode('utf-8')
            for payload in (changed_result.stdout, staged_result.stdout)
            for value in payload.split(b'\0')
            if value
        }
    )
    untracked = [value.decode('utf-8') for value in untracked_result.stdout.split(b'\0') if value]
    for path in [*changed, *untracked]:
        _safe_relative(path, f'{label} changed path')
    tracked_tree = _tracked_tree_state(worktree, candidate_sha, allowed_tracked_changes)
    return {
        'candidate_git_sha': candidate_sha,
        'changed_tracked_paths': changed,
        'detached_head': True,
        'git_head': head.stdout.strip(),
        'status_bytes': len(status_result.stdout),
        'status_sha256': _sha256_bytes(status_result.stdout),
        'tracked_tree': tracked_tree,
        'untracked_paths': sorted(untracked),
    }


def _candidate_ignore_contract(repository: Path, candidate_sha: str) -> dict[str, Any]:
    payload = _candidate_regular_blob(repository, candidate_sha, '.gitignore')
    _require(
        payload
        == _stable_read(
            repository / '.gitignore',
            'working candidate .gitignore',
            maximum_bytes=MAX_MATRIX_BYTES,
        ),
        'working .gitignore differs from candidate C',
    )
    try:
        lines = payload.decode('utf-8').splitlines()
    except UnicodeError as exc:
        raise EvidenceError('candidate .gitignore is not UTF-8') from exc
    local_metadata: dict[str, int] = {}
    for git_relative in ('info/attributes', 'info/exclude'):
        location = _git(repository, ['rev-parse', '--git-path', git_relative])
        _require(
            location.returncode == 0
            and location.stderr == ''
            and location.stdout.endswith('\n')
            and '\x00' not in location.stdout,
            f'cannot resolve repository-local Git {git_relative}',
        )
        raw_path = Path(location.stdout.rstrip('\n'))
        metadata_path = raw_path if raw_path.is_absolute() else repository / raw_path
        active: list[str] = []
        if git_relative == 'info/attributes':
            _require(
                not metadata_path.exists() and not metadata_path.is_symlink(),
                'repository-local Git info/attributes cannot participate in candidate checkout',
            )
        if metadata_path.exists() or metadata_path.is_symlink():
            metadata_payload = _stable_read(
                metadata_path,
                f'repository-local Git {git_relative}',
                maximum_bytes=1024 * 1024,
                allow_empty=True,
            )
            try:
                metadata_lines = metadata_payload.decode('utf-8').splitlines()
            except UnicodeError as exc:
                raise EvidenceError(f'repository-local Git {git_relative} is not UTF-8') from exc
            active = [
                line.strip()
                for line in metadata_lines
                if line.strip() and not line.lstrip().startswith('#')
            ]
        _require(
            not active,
            f'repository-local Git {git_relative} cannot replace a tracked root '
            '.gitignore ignore boundary',
        )
        local_metadata[git_relative] = 0
    records: list[dict[str, Any]] = []
    for purpose, (pattern, probe) in TRACKED_IGNORE_RULES.items():
        matching_lines = [index for index, line in enumerate(lines, start=1) if line == pattern]
        _require(
            len(matching_lines) == 1,
            f'candidate .gitignore lacks exact tracked {purpose} rule',
        )
        line_number = matching_lines[0]
        result = _git(
            repository,
            ['check-ignore', '-v', '--no-index', '--', probe],
        )
        expected = f'.gitignore:{line_number}:{pattern}\t{probe}\n'
        _require(
            result.returncode == 0 and result.stdout == expected and result.stderr == '',
            f'{purpose} is not ignored by the exact tracked root .gitignore rule',
        )
        records.append(
            {
                'line': line_number,
                'path': probe,
                'pattern': pattern,
                'purpose': purpose,
                'source': '.gitignore',
            }
        )
    return {
        'candidate_gitignore_mode': '100644',
        'candidate_gitignore_sha256': _sha256_bytes(payload),
        'repository_local_git_metadata_active_rules': local_metadata,
        'rules': records,
        'sanitized_exclude_and_attribute_configuration': True,
        'schema_version': 1,
    }


def _expected_index_contract(repository: Path, candidate_sha: str) -> dict[str, Any]:
    entries, _listing_sha256 = _candidate_tree_entries(repository, candidate_sha)
    expected_listing = b''.join(f'H {entry["path"]}\0'.encode() for entry in entries)
    return {
        'all_index_entries_normal': True,
        'index_entry_count': len(entries),
        'index_listing_sha256': _sha256_bytes(expected_listing),
        'no_replace_objects_environment': True,
        'replace_refs_absent': True,
        'schema_version': 1,
        'submodules_absent': True,
    }


def _repository_index_contract(repository: Path, candidate_sha: str) -> dict[str, Any]:
    entries, _listing_sha256 = _candidate_tree_entries(repository, candidate_sha)
    expected_listing = b''.join(f'H {entry["path"]}\0'.encode() for entry in entries)
    index = _git(repository, ['ls-files', '-v', '-z'], binary=True)
    replacements = _git(repository, ['replace', '-l'])
    submodules = _git(repository, ['submodule', 'status', '--recursive'])
    _require(
        index.returncode == 0 and index.stderr == b'' and index.stdout == expected_listing,
        'primary repository index has hidden, non-candidate, or non-normal entries',
    )
    _require(
        replacements.returncode == 0 and replacements.stdout == replacements.stderr == '',
        'primary repository has replacement refs',
    )
    _require(
        submodules.returncode == 0 and submodules.stdout == submodules.stderr == '',
        'primary repository has submodule state outside candidate C',
    )
    return _expected_index_contract(repository, candidate_sha)


def _primary_candidate_state(repository: Path, candidate_sha: str) -> dict[str, Any]:
    head = _git(repository, ['rev-parse', '--verify', 'HEAD'])
    status = _git(
        repository,
        ['status', '--porcelain=v1', '-z', '--untracked-files=all'],
        binary=True,
    )
    _require(head.returncode == status.returncode == 0, 'cannot inspect primary candidate state')
    _require(head.stdout.strip() == candidate_sha, 'primary repository HEAD is not candidate C')
    _require(status.stdout == b'', 'primary repository is not clean at portfolio capture')
    ignore_contract = _candidate_ignore_contract(repository, candidate_sha)
    index_contract = _repository_index_contract(repository, candidate_sha)
    tracked_tree = _tracked_tree_state(repository, candidate_sha, ())
    return {
        'candidate_git_sha': candidate_sha,
        'git_head': head.stdout.strip(),
        'ignore_contract': ignore_contract,
        'ignored_boundaries_excluded_by_tracked_gitignore': True,
        'index_contract': index_contract,
        'schema_version': 1,
        'status_bytes': 0,
        'status_sha256': _sha256_bytes(b''),
        'tracked_tree': tracked_tree,
        'worktree_clean': True,
    }


def _replay_environment(state_root: Path) -> tuple[dict[str, str], dict[str, Any]]:
    paths = {
        'HOME': state_root / 'home',
        'ROS_HOME': state_root / 'ros',
        'ROS_LOG_DIR': state_root / 'ros/log',
        'XDG_CACHE_HOME': state_root / 'xdg/cache',
        'XDG_CONFIG_HOME': state_root / 'xdg/config',
        'XDG_DATA_HOME': state_root / 'xdg/data',
        'XDG_STATE_HOME': state_root / 'xdg/state',
    }
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    environment = {
        **{name: str(path) for name, path in paths.items()},
        'LANG': 'C.UTF-8',
        'LC_ALL': 'C.UTF-8',
        'PATH': '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin',
        'TZ': 'UTC',
    }
    return environment, {
        'forbidden_overlay_keys': list(FORBIDDEN_REPLAY_ENVIRONMENT),
        'inherited_environment': False,
        'policy': 'env -i with an explicit non-secret allowlist',
        'schema_version': 1,
        'set_keys': sorted(environment),
    }


def _ruff_version(path: Path) -> str:
    result = subprocess.run(
        [str(path), '--version'],
        capture_output=True,
        check=False,
        env={'HOME': '/nonexistent', 'LANG': 'C', 'PATH': '/usr/bin:/bin'},
        text=True,
        timeout=10,
    )
    _require(
        result.returncode == 0
        and result.stdout == f'ruff {RUFF_VERSION}\n'
        and result.stderr == '',
        f'Ruff tooling seed is not exact version {RUFF_VERSION}',
    )
    return result.stdout.rstrip('\n')


def _seed_replay_tooling(repository: Path, worktree: Path, command_id: str) -> dict[str, Any]:
    if command_id not in RUFF_REQUIRED_REPLAY_IDS:
        return {'required': False, 'schema_version': 1}
    source = _regular_file(
        repository / '.venv/bin/ruff',
        'primary repository Ruff tooling seed',
        maximum_bytes=MAX_RUFF_BYTES,
    )
    _require(os.access(source, os.X_OK), 'primary repository Ruff tooling seed is not executable')
    payload = _stable_read(
        source,
        'primary repository Ruff tooling seed',
        maximum_bytes=MAX_RUFF_BYTES,
    )
    _require(
        len(payload) == RUFF_BYTES and _sha256_bytes(payload) == RUFF_SHA256,
        'Ruff tooling seed does not match the trusted candidate-bound digest',
    )
    destination = worktree / '.venv/bin/ruff'
    _write_once(destination, payload, mode=0o755)
    version = _ruff_version(destination)
    _require(
        _sha256_bytes(
            _stable_read(destination, 'copied Ruff tooling seed', maximum_bytes=MAX_RUFF_BYTES)
        )
        == _sha256_bytes(payload),
        'Ruff tooling seed copy changed',
    )
    return {
        'bytes': len(payload),
        'destination': '.venv/bin/ruff',
        'required': True,
        'schema_version': 1,
        'sha256': _sha256_bytes(payload),
        'source': '.venv/bin/ruff',
        'version': version,
    }


def _generated_evidence_postconditions(
    worktree: Path,
    allowed_paths: Sequence[str],
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for relative in allowed_paths:
        path = worktree / relative
        payload = _stable_read(
            path,
            f'generated replay postcondition {relative}',
            maximum_bytes=64 * 1024 * 1024,
        )
        if path.suffix == '.json':
            try:
                document = _mapping(
                    json.loads(payload), f'generated replay postcondition {relative}'
                )
            except (UnicodeError, json.JSONDecodeError) as exc:
                raise EvidenceError(f'generated replay JSON is invalid: {relative}') from exc
            _require(
                document.get('status') in {'PASS', 'passed'} or relative == 'phase0-versions.json',
                f'generated replay JSON lacks a passing status: {relative}',
            )
        records.append(
            {
                'bytes': len(payload),
                'path': relative,
                'sha256': _sha256_bytes(payload),
            }
        )
    return records


def _replay_repository_preflight(repository: Path) -> dict[str, Any]:
    filters = _git(repository, ['config', '--show-origin', '--get-regexp', r'^filter\.'])
    _require(
        filters.returncode == 1 and filters.stdout == filters.stderr == '',
        'effective Git filter configuration is forbidden before detached replay checkout',
    )
    location = _git(repository, ['rev-parse', '--git-path', 'info/attributes'])
    _require(
        location.returncode == 0
        and location.stderr == ''
        and location.stdout.endswith('\n')
        and '\x00' not in location.stdout,
        'cannot resolve repository-local Git info/attributes',
    )
    raw_path = Path(location.stdout.rstrip('\n'))
    info_attributes = raw_path if raw_path.is_absolute() else repository / raw_path
    _require(
        not info_attributes.exists() and not info_attributes.is_symlink(),
        'repository-local Git info/attributes is forbidden before detached replay checkout',
    )
    return {
        'effective_filter_configuration_absent': True,
        'info_attributes_absent': True,
        'sanitized_git_environment': True,
        'schema_version': 1,
    }


def _replay_one_command(
    repository: Path,
    candidate_sha: str,
    command: Mapping[str, Any],
    output: Path,
) -> dict[str, Any]:
    identifier = str(command['id'])
    repository_preflight = _replay_repository_preflight(repository)
    with tempfile.TemporaryDirectory(prefix=f'robotest-p5-{identifier}-') as temporary:
        temporary_root = Path(temporary).resolve(strict=True)
        worktree = temporary_root / 'checkout'
        added = _git(
            repository, ['worktree', 'add', '--detach', str(worktree), candidate_sha], timeout=60
        )
        _require(added.returncode == 0, f'cannot create detached replay worktree: {identifier}')
        try:
            before = _worktree_state(worktree, candidate_sha, f'{identifier} pre-state')
            _require(
                before['git_head'] == candidate_sha
                and before['status_bytes'] == 0
                and before['changed_tracked_paths'] == []
                and before['untracked_paths'] == [],
                f'README replay command {identifier} did not start from clean C',
            )
            tooling_seed = _seed_replay_tooling(repository, worktree, identifier)
            base_environment, environment_report = _replay_environment(temporary_root / 'state')
            assignments, argv = _parse_replay_argv(identifier, str(command['executable']))
            allowed = list(command['allowed_tracked_changes'])
            run = _run_bounded(
                argv,
                cwd=worktree,
                environment={**base_environment, **assignments},
                timeout_seconds=int(command['timeout_seconds']),
                stdout_limit=int(command['maximum_stdout_bytes']),
                stderr_limit=int(command['maximum_stderr_bytes']),
            )
            after = _worktree_state(
                worktree,
                candidate_sha,
                f'{identifier} post-state',
                allowed_tracked_changes=allowed,
            )
            postcondition_files = _generated_evidence_postconditions(worktree, allowed)
            stdout_path = output / 'stdout.log'
            stderr_path = output / 'stderr.log'
            _write_once(stdout_path, run.pop('stdout'), mode=0o600)
            _write_once(stderr_path, run.pop('stderr'), mode=0o600)
            _write_json_once(output / 'environment.json', environment_report)
            _write_json_once(output / 'tooling-seed.json', tooling_seed)
            _write_json_once(output / 'source-state-before.json', before)
            _write_json_once(output / 'source-state-after.json', after)
            failures: list[str] = []
            if after['git_head'] != candidate_sha:
                failures.append('replay HEAD changed')
            if after['untracked_paths']:
                failures.append('untracked source paths were created')
            if not set(after['changed_tracked_paths']) <= set(allowed):
                failures.append(
                    'tracked paths changed outside the frozen generated-evidence allowlist'
                )
            if run['timed_out']:
                failures.append('command timed out')
            if run['stdout_overflow'] or run['stderr_overflow']:
                failures.append('a replay log was truncated')
            if run['returncode'] != command['expected_exit_code']:
                failures.append(
                    f'return code {run["returncode"]} != {command["expected_exit_code"]}'
                )
            result = {
                'allowed_tracked_changes': allowed,
                'authorization': command['authorization'],
                'argv': run['argv'],
                'command_sha256': command['command_sha256'],
                'documented_root': DOCUMENTED_REPOSITORY,
                'effective_checkout_policy': REPLAY_MAPPING_POLICY,
                'execution_role': 'fresh_detached_candidate_worktree',
                'environment_assignments': assignments,
                'expected_exit_code': command['expected_exit_code'],
                'id': identifier,
                'failure_reasons': failures,
                'observed_tracked_changes': after['changed_tracked_paths'],
                'ordinal': command['ordinal'],
                'returncode': run['returncode'],
                'repository_preflight': repository_preflight,
                'schema_version': 1,
                'postcondition_files': postcondition_files,
                'stderr': {
                    'bytes': (output / 'stderr.log').stat().st_size,
                    'maximum_bytes': command['maximum_stderr_bytes'],
                    'observed_bytes': run['stderr_observed_bytes'],
                    'overflow': run['stderr_overflow'],
                    'path': f'documentation/{output.name}/stderr.log',
                    'sha256': file_sha256(output / 'stderr.log'),
                },
                'stdout': {
                    'bytes': (output / 'stdout.log').stat().st_size,
                    'maximum_bytes': command['maximum_stdout_bytes'],
                    'observed_bytes': run['stdout_observed_bytes'],
                    'overflow': run['stdout_overflow'],
                    'path': f'documentation/{output.name}/stdout.log',
                    'sha256': file_sha256(output / 'stdout.log'),
                },
                'timed_out': run['timed_out'],
                'wall_duration_ns': run['elapsed_wall_ns'],
                'worktree_head': after['git_head'],
                'status': 'PASS' if not failures else 'FAIL',
            }
            _write_json_once(output / 'result.json', result)
            _require(not failures, f'{identifier} replay failed: {"; ".join(failures)}')
            return result
        finally:
            removed = _git(repository, ['worktree', 'remove', '--force', str(worktree)], timeout=60)
            _require(removed.returncode == 0, f'cannot remove replay worktree: {identifier}')


def _copy_phase3_lineage(
    phase3_candidate_root: Path,
    attempt: Path,
    media: Mapping[str, Any],
) -> None:
    source_root = _resolved_directory(phase3_candidate_root, 'Phase 3 candidate root')
    destination = attempt / 'phase3'
    copies = {
        'aggregate-result.json': media['aggregate_relative_path'],
        'localization-error.png': media['chart_relative_path'],
        'run-artifacts.manifest.json': media['manifest_relative_path'],
        'run-artifacts.manifest.json.sha256': f'{media["manifest_relative_path"]}.sha256',
        'run-result.json': media['result_relative_path'],
        'suite-plan.json': media['suite_plan_relative_path'],
        'suite-plan.json.sha256': f'{media["suite_plan_relative_path"]}.sha256',
    }
    for name, relative in copies.items():
        source = _contained_path(source_root, relative, 'Phase 3 lineage input')
        maximum = MAX_PNG_BYTES if name.endswith('.png') else MAX_JSON_BYTES
        _copy_once(source, destination / name, f'Phase 3 lineage {name}', maximum_bytes=maximum)
    _write_json_once(destination / 'media-report.json', dict(media))


def _render_request(
    attempt_id: str,
    candidate_sha: str,
    diagrams: Sequence[Mapping[str, Any]],
) -> dict[str, Any]:
    return {
        'attempt_id': attempt_id,
        'candidate_git_sha': candidate_sha,
        'diagrams': [
            {
                'diagram_id': diagram['id'],
                'render_input_path': f'diagrams/{diagram["id"]}.mmd',
                'source_path': diagram['source_path'],
                'source_sha256': diagram['source_sha256'],
            }
            for diagram in diagrams
        ],
        'required_output': 'UTF-8 SVG with safe text-only content and a bounded viewBox',
        'schema_version': 1,
        'status': 'REVIEW_REQUIRED',
    }


def _write_attempt_failure(
    attempt: Path,
    candidate_sha: str,
    error: BaseException,
    *,
    stage: str,
) -> None:
    _require(
        stage in {'portfolio_prepare', 'portfolio_finalize'}, 'portfolio failure stage invalid'
    )
    path = attempt / 'failure.json'
    if path.exists() or path.is_symlink():
        return
    try:
        _write_json_once(
            path,
            {
                'attempt_id': attempt.name,
                'candidate_git_sha': candidate_sha,
                'error': str(error),
                'failed_utc': datetime.now(UTC).isoformat().replace('+00:00', 'Z'),
                'producer': f'robotest_phase5/{stage}',
                'schema_version': 1,
                'stage': stage,
                'status': 'FAIL',
            },
        )
    except OSError:
        return


def prepare_portfolio(
    repository: Path,
    candidate_sha: str,
    phase3_candidate_root: Path,
    *,
    authorize_mutating_commands: bool = False,
    attempt_id: str | None = None,
) -> dict[str, Any]:
    """Replay C in isolated per-unit worktrees and create a review-required attempt."""
    repository = _resolved_directory(repository, 'repository')
    candidate_sha = _validate_candidate_sha(repository, candidate_sha)
    portfolio_root = _portfolio_root(repository, candidate_sha, create=True)
    attempt_id = _attempt_identifier(attempt_id)
    attempt = _attempt_directory(portfolio_root, attempt_id, create=True)
    try:
        primary_state = _primary_candidate_state(repository, candidate_sha)
        _write_json_once(attempt / 'primary-candidate-state.json', primary_state)
        candidate = validate_candidate_contract(repository, candidate_sha)
        mutating = [
            command['id']
            for command in candidate['commands']
            if command['authorization'] == 'mutating_requires_explicit_authorization'
        ]
        _require(
            authorize_mutating_commands or not mutating,
            'documentation replay includes mutating commands and requires explicit authorization: '
            + ', '.join(mutating),
        )
        media = validate_scenario5_media(phase3_candidate_root, candidate_sha)
        _write_json_once(attempt / 'candidate-contract.json', candidate)
        for diagram in candidate['diagrams']:
            _write_once(
                attempt / f'diagrams/{diagram["id"]}.mmd',
                str(diagram['source']).encode(),
            )
        render_request = _render_request(attempt_id, candidate_sha, candidate['diagrams'])
        _write_json_once(attempt / 'render-request.json', render_request)
        _copy_phase3_lineage(phase3_candidate_root, attempt, media)
        results: list[dict[str, Any]] = []
        for command in candidate['commands']:
            unit_name = f'{command["ordinal"]:02d}-{command["id"]}'
            results.append(
                _replay_one_command(
                    repository,
                    candidate_sha,
                    command,
                    attempt / 'documentation' / unit_name,
                )
            )
        replay = {
            'authorized_mutating_command_ids': mutating,
            'candidate_git_sha': candidate_sha,
            'command_count': len(results),
            'commands': results,
            'cwd_mapping_policy': REPLAY_MAPPING_POLICY,
            'fresh_detached_worktree_per_command': True,
            'inherited_overlay': False,
            'matrix_sha256': candidate['matrix_sha256'],
            'readme_sha256': candidate['readme_sha256'],
            'schema_version': 1,
            'status': 'PASS',
            'mutating_commands_authorized': bool(authorize_mutating_commands),
        }
        _write_json_once(attempt / 'documentation/replay-report.json', replay)
        report = {
            'attempt_id': attempt_id,
            'attempt_path': attempt.relative_to(repository).as_posix(),
            'candidate_contract_sha256': file_sha256(attempt / 'candidate-contract.json'),
            'candidate_git_sha': candidate_sha,
            'documentation_replay_sha256': file_sha256(
                attempt / 'documentation/replay-report.json'
            ),
            'phase3_media_sha256': file_sha256(attempt / 'phase3/media-report.json'),
            'primary_candidate_state_sha256': file_sha256(attempt / 'primary-candidate-state.json'),
            'prepared_utc': datetime.now(UTC).isoformat().replace('+00:00', 'Z'),
            'producer': 'robotest_phase5/portfolio_prepare',
            'render_request_sha256': file_sha256(attempt / 'render-request.json'),
            'schema_version': 1,
            'status': 'REVIEW_REQUIRED',
            'authorized_mutating_command_ids': mutating,
            'mutating_commands_authorized': bool(authorize_mutating_commands),
            'verification_scope': (
                'Exact README inventory replay in one fresh detached C worktree per unit; '
                'candidate-bound Mermaid sources and deterministic Scenario 5 run 12 lineage; '
                'external rendering and human review remain required'
            ),
        }
        _write_json_once(attempt / 'prepare-report.json', report)
        return report
    except BaseException as error:
        _write_attempt_failure(
            attempt,
            candidate_sha,
            error,
            stage='portfolio_prepare',
        )
        raise


def _utc_timestamp(value: object, label: str) -> datetime:
    _require(isinstance(value, str) and value.endswith('Z'), f'{label} is not canonical UTC')
    try:
        parsed = datetime.fromisoformat(value[:-1] + '+00:00')
    except ValueError as exc:
        raise EvidenceError(f'{label} is invalid') from exc
    _require(
        parsed.tzinfo is not None and parsed.utcoffset() == UTC.utcoffset(parsed),
        f'{label} is invalid',
    )
    return parsed


def _clean_review_text(value: object, label: str, *, maximum: int) -> str:
    _require(
        isinstance(value, str)
        and 1 <= len(value) <= maximum
        and value == value.strip()
        and all(character.isprintable() for character in value),
        f'{label} is invalid',
    )
    return value


def _validate_replay_logs(
    attempt: Path,
    candidate: Mapping[str, Any],
) -> dict[str, Any]:
    report = _load_canonical_json(
        attempt / 'documentation/replay-report.json',
        'documentation replay report',
    )
    commands = _list(report.get('commands'), 'documentation replay report commands')
    _require(
        set(report)
        == {
            'authorized_mutating_command_ids',
            'candidate_git_sha',
            'command_count',
            'commands',
            'cwd_mapping_policy',
            'fresh_detached_worktree_per_command',
            'inherited_overlay',
            'matrix_sha256',
            'mutating_commands_authorized',
            'readme_sha256',
            'schema_version',
            'status',
        }
        and report.get('candidate_git_sha') == candidate['candidate_git_sha']
        and report.get('command_count') == len(candidate['commands']) == 11
        and report.get('cwd_mapping_policy') == REPLAY_MAPPING_POLICY
        and report.get('fresh_detached_worktree_per_command') is True
        and report.get('inherited_overlay') is False
        and report.get('authorized_mutating_command_ids')
        == ['phase0-source-apply', 'phase0-install']
        and report.get('mutating_commands_authorized') is True
        and report.get('matrix_sha256') == candidate['matrix_sha256']
        and report.get('readme_sha256') == candidate['readme_sha256']
        and report.get('schema_version') == 1
        and report.get('status') == 'PASS'
        and len(commands) == 11,
        'documentation replay report is not canonical PASS',
    )
    for expected, raw in zip(candidate['commands'], commands, strict=True):
        record = _mapping(raw, f'documentation replay command {expected["id"]}')
        unit = attempt / 'documentation' / f'{expected["ordinal"]:02d}-{expected["id"]}'
        stored = _load_canonical_json(unit / 'result.json', 'documentation replay result')
        _require(stored == record, f'documentation replay result drifted: {expected["id"]}')
        _require(
            set(record)
            == {
                'allowed_tracked_changes',
                'authorization',
                'argv',
                'command_sha256',
                'documented_root',
                'effective_checkout_policy',
                'environment_assignments',
                'execution_role',
                'expected_exit_code',
                'failure_reasons',
                'id',
                'observed_tracked_changes',
                'ordinal',
                'postcondition_files',
                'returncode',
                'repository_preflight',
                'schema_version',
                'status',
                'stderr',
                'stdout',
                'timed_out',
                'wall_duration_ns',
                'worktree_head',
            }
            and record.get('id') == expected['id']
            and record.get('ordinal') == expected['ordinal']
            and record.get('authorization') == expected['authorization']
            and record.get('argv') == _parse_replay_argv(expected['id'], expected['executable'])[1]
            and record.get('allowed_tracked_changes') == expected['allowed_tracked_changes']
            and record.get('command_sha256') == expected['command_sha256']
            and record.get('documented_root') == DOCUMENTED_REPOSITORY
            and record.get('effective_checkout_policy') == REPLAY_MAPPING_POLICY
            and record.get('execution_role') == 'fresh_detached_candidate_worktree'
            and record.get('environment_assignments') == REPLAY_ASSIGNMENTS[expected['id']]
            and record.get('expected_exit_code') == expected['expected_exit_code']
            and record.get('returncode') == expected['expected_exit_code']
            and _exact_integer(record.get('returncode'))
            and record.get('failure_reasons') == []
            and record.get('schema_version') == 1
            and record.get('status') == 'PASS'
            and record.get('timed_out') is False
            and _exact_integer(record.get('wall_duration_ns'))
            and record['wall_duration_ns'] > 0
            and record.get('worktree_head') == candidate['candidate_git_sha']
            and set(record.get('observed_tracked_changes', []))
            <= set(expected['allowed_tracked_changes']),
            f'documentation replay command is not exact PASS: {expected["id"]}',
        )
        _require(
            record.get('repository_preflight')
            == {
                'effective_filter_configuration_absent': True,
                'info_attributes_absent': True,
                'sanitized_git_environment': True,
                'schema_version': 1,
            },
            f'documentation replay repository preflight changed: {expected["id"]}',
        )
        postconditions = _list(record.get('postcondition_files'), 'replay postcondition files')
        _require(
            [item.get('path') for item in postconditions if isinstance(item, dict)]
            == expected['allowed_tracked_changes'],
            f'documentation replay postconditions changed: {expected["id"]}',
        )
        for expected_path, raw_postcondition in zip(
            expected['allowed_tracked_changes'], postconditions, strict=True
        ):
            postcondition = _mapping(
                raw_postcondition,
                f'{expected["id"]} replay postcondition {expected_path}',
            )
            _require(
                set(postcondition) == {'bytes', 'path', 'sha256'}
                and postcondition.get('path') == expected_path
                and _exact_integer(postcondition.get('bytes'))
                and postcondition['bytes'] > 0
                and SHA256.fullmatch(str(postcondition.get('sha256'))) is not None,
                f'documentation replay postcondition is invalid: {expected_path}',
            )
        for stream in ('stdout', 'stderr'):
            stream_record = _mapping(record.get(stream), f'{expected["id"]} {stream}')
            log = unit / f'{stream}.log'
            payload = _stable_read(
                log,
                f'{expected["id"]} {stream} log',
                maximum_bytes=int(expected[f'maximum_{stream}_bytes']),
                allow_empty=True,
            )
            _require(
                set(stream_record)
                == {
                    'bytes',
                    'maximum_bytes',
                    'observed_bytes',
                    'overflow',
                    'path',
                    'sha256',
                }
                and stream_record.get('path') == f'documentation/{unit.name}/{stream}.log'
                and stream_record.get('bytes') == len(payload)
                and _exact_integer(stream_record.get('bytes'))
                and stream_record.get('observed_bytes') == len(payload)
                and _exact_integer(stream_record.get('observed_bytes'))
                and stream_record.get('maximum_bytes') == expected[f'maximum_{stream}_bytes']
                and _exact_integer(stream_record.get('maximum_bytes'))
                and stream_record.get('overflow') is False
                and stream_record.get('sha256') == _sha256_bytes(payload),
                f'documentation replay {stream} log is incomplete or changed: {expected["id"]}',
            )
        environment = _load_canonical_json(unit / 'environment.json', 'replay environment')
        _require(
            set(environment)
            == {
                'forbidden_overlay_keys',
                'inherited_environment',
                'policy',
                'schema_version',
                'set_keys',
            }
            and environment.get('inherited_environment') is False
            and environment.get('forbidden_overlay_keys') == list(FORBIDDEN_REPLAY_ENVIRONMENT)
            and environment.get('policy') == 'env -i with an explicit non-secret allowlist'
            and environment.get('schema_version') == 1
            and environment.get('set_keys')
            == [
                'HOME',
                'LANG',
                'LC_ALL',
                'PATH',
                'ROS_HOME',
                'ROS_LOG_DIR',
                'TZ',
                'XDG_CACHE_HOME',
                'XDG_CONFIG_HOME',
                'XDG_DATA_HOME',
                'XDG_STATE_HOME',
            ],
            f'documentation replay environment changed: {expected["id"]}',
        )
        tooling = _load_canonical_json(unit / 'tooling-seed.json', 'replay tooling seed')
        if expected['id'] in RUFF_REQUIRED_REPLAY_IDS:
            _require(
                set(tooling)
                == {
                    'bytes',
                    'destination',
                    'required',
                    'schema_version',
                    'sha256',
                    'source',
                    'version',
                }
                and tooling.get('required') is True
                and tooling.get('version') == f'ruff {RUFF_VERSION}'
                and tooling.get('bytes') == RUFF_BYTES
                and _exact_integer(tooling.get('bytes'))
                and tooling.get('sha256') == RUFF_SHA256
                and tooling.get('source') == '.venv/bin/ruff'
                and tooling.get('destination') == '.venv/bin/ruff'
                and tooling.get('schema_version') == 1,
                f'documentation replay Ruff seed changed: {expected["id"]}',
            )
        else:
            _require(
                tooling == {'required': False, 'schema_version': 1},
                f'unexpected documentation replay tooling seed: {expected["id"]}',
            )
        before = _load_canonical_json(unit / 'source-state-before.json', 'replay pre-state')
        after = _load_canonical_json(unit / 'source-state-after.json', 'replay post-state')
        state_keys = {
            'candidate_git_sha',
            'changed_tracked_paths',
            'detached_head',
            'git_head',
            'status_bytes',
            'status_sha256',
            'tracked_tree',
            'untracked_paths',
        }
        expected_before_tree = {
            'allowed_modified_paths': [],
            'candidate_tree_entry_count': candidate['candidate_tree_entry_count'],
            'candidate_tree_listing_sha256': candidate['candidate_tree_listing_sha256'],
            'checked_immutable_path_count': candidate['candidate_tree_entry_count'],
            'schema_version': 1,
            'status': 'PASS',
        }
        expected_after_tree = {
            **expected_before_tree,
            'allowed_modified_paths': expected['allowed_tracked_changes'],
            'checked_immutable_path_count': candidate['candidate_tree_entry_count']
            - len(expected['allowed_tracked_changes']),
        }
        _require(
            set(before) == set(after) == state_keys
            and before.get('candidate_git_sha')
            == after.get('candidate_git_sha')
            == candidate['candidate_git_sha']
            and before.get('detached_head') is True
            and after.get('detached_head') is True
            and before.get('git_head') == after.get('git_head') == candidate['candidate_git_sha']
            and before.get('status_bytes') == 0
            and _exact_integer(before.get('status_bytes'))
            and before.get('status_sha256') == _sha256_bytes(b'')
            and before.get('changed_tracked_paths') == []
            and before.get('untracked_paths') == []
            and before.get('tracked_tree') == expected_before_tree
            and after.get('untracked_paths') == []
            and after.get('changed_tracked_paths') == record.get('observed_tracked_changes')
            and _exact_integer(after.get('status_bytes'))
            and after['status_bytes'] >= 0
            and SHA256.fullmatch(str(after.get('status_sha256'))) is not None
            and after.get('tracked_tree') == expected_after_tree,
            f'documentation replay source-state proof changed: {expected["id"]}',
        )
    return report


def _validate_phase3_lineage_copy(
    attempt: Path,
    phase3_candidate_root: Path,
    candidate_sha: str,
) -> dict[str, Any]:
    expected = validate_scenario5_media(phase3_candidate_root, candidate_sha)
    observed = _load_canonical_json(attempt / 'phase3/media-report.json', 'portfolio media report')
    _require(observed == expected, 'portfolio media report differs from Scenario 5 source evidence')
    source_root = _resolved_directory(phase3_candidate_root, 'Phase 3 candidate root')
    copies = {
        'aggregate-result.json': expected['aggregate_relative_path'],
        'localization-error.png': expected['chart_relative_path'],
        'run-artifacts.manifest.json': expected['manifest_relative_path'],
        'run-artifacts.manifest.json.sha256': f'{expected["manifest_relative_path"]}.sha256',
        'run-result.json': expected['result_relative_path'],
        'suite-plan.json': expected['suite_plan_relative_path'],
        'suite-plan.json.sha256': f'{expected["suite_plan_relative_path"]}.sha256',
    }
    for name, relative in copies.items():
        maximum = MAX_PNG_BYTES if name.endswith('.png') else MAX_JSON_BYTES
        source = _stable_read(
            _contained_path(source_root, relative, 'Phase 3 lineage source'),
            f'Phase 3 lineage source {name}',
            maximum_bytes=maximum,
        )
        copied = _stable_read(
            attempt / 'phase3' / name,
            f'portfolio Phase 3 lineage copy {name}',
            maximum_bytes=maximum,
        )
        _require(copied == source, f'portfolio Phase 3 lineage copy changed: {name}')
    return expected


def _validate_prepared_attempt(
    repository: Path,
    attempt: Path,
    candidate_sha: str,
    phase3_candidate_root: Path,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    _require(not (attempt / 'failure.json').exists(), 'portfolio attempt is a preserved failure')
    candidate = validate_candidate_contract(repository, candidate_sha)
    stored_candidate = _load_canonical_json(
        attempt / 'candidate-contract.json',
        'stored candidate contract',
    )
    _require(stored_candidate == candidate, 'stored portfolio candidate contract drifted')
    primary_state = _load_canonical_json(
        attempt / 'primary-candidate-state.json',
        'stored primary candidate state',
    )
    ignore_contract = _candidate_ignore_contract(repository, candidate_sha)
    index_contract = _expected_index_contract(repository, candidate_sha)
    tracked_tree = _expected_tracked_tree_state(repository, candidate_sha, ())
    _require(
        primary_state
        == {
            'candidate_git_sha': candidate_sha,
            'git_head': candidate_sha,
            'ignore_contract': ignore_contract,
            'ignored_boundaries_excluded_by_tracked_gitignore': True,
            'index_contract': index_contract,
            'schema_version': 1,
            'status_bytes': 0,
            'status_sha256': _sha256_bytes(b''),
            'tracked_tree': tracked_tree,
            'worktree_clean': True,
        },
        'stored primary candidate state changed',
    )
    for diagram in candidate['diagrams']:
        payload = _stable_read(
            attempt / f'diagrams/{diagram["id"]}.mmd',
            f'{diagram["id"]} Mermaid source',
            maximum_bytes=MAX_README_BYTES,
        )
        _require(
            payload == str(diagram['source']).encode()
            and _sha256_bytes(payload) == diagram['source_sha256'],
            f'{diagram["id"]} Mermaid source drifted from candidate C',
        )
    render_request = _load_canonical_json(attempt / 'render-request.json', 'render request')
    expected_request = _render_request(attempt.name, candidate_sha, candidate['diagrams'])
    _require(render_request == expected_request, 'portfolio render request drifted')
    replay = _validate_replay_logs(attempt, candidate)
    media = _validate_phase3_lineage_copy(attempt, phase3_candidate_root, candidate_sha)
    prepare = _load_canonical_json(attempt / 'prepare-report.json', 'portfolio prepare report')
    _require(
        prepare.get('attempt_id') == attempt.name
        and prepare.get('attempt_path') == attempt.relative_to(repository).as_posix()
        and prepare.get('candidate_git_sha') == candidate_sha
        and prepare.get('candidate_contract_sha256')
        == file_sha256(attempt / 'candidate-contract.json')
        and prepare.get('documentation_replay_sha256')
        == file_sha256(attempt / 'documentation/replay-report.json')
        and prepare.get('phase3_media_sha256') == file_sha256(attempt / 'phase3/media-report.json')
        and prepare.get('render_request_sha256') == file_sha256(attempt / 'render-request.json')
        and prepare.get('primary_candidate_state_sha256')
        == file_sha256(attempt / 'primary-candidate-state.json')
        and prepare.get('authorized_mutating_command_ids')
        == ['phase0-source-apply', 'phase0-install']
        and prepare.get('mutating_commands_authorized') is True
        and prepare.get('producer') == 'robotest_phase5/portfolio_prepare'
        and prepare.get('schema_version') == 1
        and prepare.get('status') == 'REVIEW_REQUIRED',
        'portfolio prepare report changed',
    )
    _utc_timestamp(prepare.get('prepared_utc'), 'portfolio prepared timestamp')
    return candidate, replay, media


def _validate_visual_review(
    review: Mapping[str, Any],
    *,
    attempt_id: str,
    candidate_sha: str,
    candidate: Mapping[str, Any],
    render_records: Sequence[Mapping[str, Any]],
    prepared_utc: datetime,
) -> dict[str, Any]:
    _require(
        set(review)
        == {
            'attempt_id',
            'candidate_git_sha',
            'diagrams',
            'reviewed_utc',
            'reviewer',
            'schema_version',
        }
        and review.get('attempt_id') == attempt_id
        and review.get('candidate_git_sha') == candidate_sha
        and review.get('schema_version') == 1,
        'portfolio visual review identity changed',
    )
    reviewer = _clean_review_text(review.get('reviewer'), 'portfolio visual reviewer', maximum=128)
    _require(
        reviewer.casefold() not in {'automated', 'ci', 'robot', 'unknown'},
        'portfolio visual reviewer is invalid',
    )
    reviewed_utc = _utc_timestamp(review.get('reviewed_utc'), 'portfolio visual review timestamp')
    _require(reviewed_utc >= prepared_utc, 'portfolio visual review predates render request')
    reviews = _list(review.get('diagrams'), 'portfolio visual diagram reviews')
    _require(len(reviews) == len(DIAGRAM_IDS), 'portfolio visual review count changed')
    by_render = {record['diagram_id']: record for record in render_records}
    by_source = {diagram['id']: diagram for diagram in candidate['diagrams']}
    for identifier, raw in zip(DIAGRAM_IDS, reviews, strict=True):
        item = _mapping(raw, f'portfolio visual review {identifier}')
        rendered_utc = _utc_timestamp(
            item.get('rendered_utc'),
            f'portfolio {identifier} rendered timestamp',
        )
        renderer = _mapping(item.get('renderer'), f'portfolio {identifier} renderer')
        _require(
            set(renderer) == {'identity', 'mode', 'reference', 'version'},
            f'portfolio {identifier} renderer contract changed',
        )
        identity = _clean_review_text(
            renderer.get('identity'), f'portfolio {identifier} renderer identity', maximum=128
        )
        version = _clean_review_text(
            renderer.get('version'),
            f'portfolio {identifier} renderer identity/version',
            maximum=128,
        )
        mode = renderer.get('mode')
        reference = renderer.get('reference')
        _require(
            identity == renderer.get('identity')
            and version == renderer.get('version')
            and mode in {'argv', 'url'},
            f'portfolio {identifier} renderer identity is invalid',
        )
        if mode == 'argv':
            _require(
                isinstance(reference, list)
                and 1 <= len(reference) <= 32
                and all(
                    isinstance(argument, str)
                    and 1 <= len(argument) <= 4096
                    and argument == argument.strip()
                    and all(character.isprintable() for character in argument)
                    for argument in reference
                ),
                f'portfolio {identifier} renderer argv is invalid',
            )
        else:
            _require(
                isinstance(reference, str)
                and 1 <= len(reference) <= 2048
                and re.fullmatch(
                    rf'https://github\.com/'
                    rf'[A-Za-z0-9][A-Za-z0-9.-]{{0,99}}/'
                    rf'[A-Za-z0-9_.-]{{1,100}}/blob/{candidate_sha}/README\.md',
                    reference,
                )
                is not None,
                f'portfolio {identifier} renderer URL is invalid',
            )
        _require(
            set(item)
            == {
                'diagram_id',
                'render_sha256',
                'rendered_utc',
                'renderer',
                'source_sha256',
                'verdict',
            }
            and item.get('diagram_id') == identifier
            and item.get('render_sha256') == by_render[identifier]['render_sha256']
            and item.get('source_sha256') == by_source[identifier]['source_sha256']
            and item.get('verdict') == 'PASS',
            f'portfolio visual review does not bind exact PASS bytes: {identifier}',
        )
        _require(
            prepared_utc <= rendered_utc <= reviewed_utc,
            f'portfolio {identifier} render/review chronology is invalid',
        )
        by_render[identifier]['rendered_utc'] = item['rendered_utc']
        by_render[identifier]['renderer'] = dict(renderer)
    return dict(review)


def _projection_file_map(candidate_sha: str) -> dict[str, str]:
    paths = portfolio_projection_paths(candidate_sha)
    return {
        'proof': paths[0],
        'manifest': paths[1],
        'validation': paths[2],
        'architecture': paths[3],
        'release-flow': paths[4],
        'chart': paths[5],
    }


def _projection_checksum_payload(
    projection: Path,
    candidate_sha: str,
) -> tuple[bytes, bytes]:
    names = _projection_file_map(candidate_sha)
    covered = sorted([names['proof'], names['architecture'], names['release-flow'], names['chart']])
    digest_by_relative = {
        names['proof']: file_sha256(projection / Path(names['proof']).name),
        names['architecture']: file_sha256(projection / Path(names['architecture']).name),
        names['release-flow']: file_sha256(projection / Path(names['release-flow']).name),
        names['chart']: file_sha256(projection / Path(names['chart']).name),
    }
    manifest = ''.join(
        f'{digest_by_relative[relative]}  {relative}\n' for relative in covered
    ).encode()
    validation = ''.join(f'{relative}: OK\n' for relative in covered).encode()
    return manifest, validation


def _write_raw_checksum_manifest(attempt: Path) -> dict[str, str]:
    excluded = {attempt / 'SHA256SUMS', attempt / 'checksum-validation.txt'}
    records: dict[str, str] = {}
    total_bytes = 0
    for path in sorted(attempt.rglob('*')):
        _require(not path.is_symlink(), f'raw portfolio attempt contains a symlink: {path}')
        if path.is_dir():
            continue
        _require(path not in excluded, 'raw checksum outputs already exist')
        relative = path.relative_to(attempt).as_posix()
        _safe_relative(relative, 'raw portfolio checksum path')
        payload = _stable_read(
            path,
            f'raw portfolio file {relative}',
            maximum_bytes=MAX_RESULT_DIRECTORY_BYTES,
            allow_empty=True,
        )
        records[relative] = _sha256_bytes(payload)
        total_bytes += len(payload)
    _require(len(records) <= MAX_RAW_FILE_COUNT, 'raw portfolio file-count cap exceeded')
    _require(total_bytes <= MAX_RAW_TOTAL_BYTES, 'raw portfolio aggregate byte cap exceeded')
    _require(records, 'raw portfolio attempt is empty')
    manifest = ''.join(f'{records[path]}  {path}\n' for path in sorted(records)).encode()
    validation = ''.join(f'{path}: OK\n' for path in sorted(records)).encode()
    _write_once(attempt / 'SHA256SUMS', manifest)
    _write_once(attempt / 'checksum-validation.txt', validation)
    return records


def _raw_payload_stats(attempt: Path) -> dict[str, Any]:
    excluded = {'SHA256SUMS', 'checksum-validation.txt', 'raw-manifest-metadata.json'}
    count = 0
    total = 0
    for path in sorted(attempt.rglob('*')):
        if path.is_dir():
            continue
        relative = path.relative_to(attempt).as_posix()
        if relative in excluded:
            continue
        payload = _stable_read(
            path,
            f'raw portfolio payload {relative}',
            maximum_bytes=MAX_RESULT_DIRECTORY_BYTES,
            allow_empty=True,
        )
        count += 1
        total += len(payload)
    _require(count <= MAX_RAW_FILE_COUNT, 'raw portfolio file-count cap exceeded')
    _require(total <= MAX_RAW_TOTAL_BYTES, 'raw portfolio aggregate byte cap exceeded')
    return {
        'maximum_file_count': MAX_RAW_FILE_COUNT,
        'maximum_total_bytes': MAX_RAW_TOTAL_BYTES,
        'payload_file_count': count,
        'payload_total_bytes': total,
        'schema_version': 1,
    }


def finalize_portfolio(
    repository: Path,
    candidate_sha: str,
    phase3_candidate_root: Path,
    attempt_id: str,
    architecture_svg: Path,
    release_flow_svg: Path,
    visual_review: Path,
) -> dict[str, Any]:
    """Bind externally rendered, human-reviewed diagrams and finalize one attempt."""
    repository = _resolved_directory(repository, 'repository')
    candidate_sha = _validate_candidate_sha(repository, candidate_sha)
    portfolio_root = _portfolio_root(repository, candidate_sha, create=False)
    attempt = _attempt_directory(portfolio_root, attempt_id, create=False)
    _require(not (attempt / 'SHA256SUMS').exists(), 'portfolio attempt is already finalized')
    _require(not (attempt / 'finalize-report.json').exists(), 'portfolio attempt was already used')
    try:
        candidate, replay, media = _validate_prepared_attempt(
            repository,
            attempt,
            candidate_sha,
            phase3_candidate_root,
        )
        source_svgs = {
            'architecture': architecture_svg,
            'release-flow': release_flow_svg,
        }
        render_records: list[dict[str, Any]] = []
        for identifier in DIAGRAM_IDS:
            destination = attempt / f'diagrams/{identifier}.svg'
            _copy_once(
                source_svgs[identifier],
                destination,
                f'external {identifier} SVG',
                maximum_bytes=MAX_SVG_BYTES,
            )
            width, height = _svg_dimensions(destination, f'{identifier} rendered diagram')
            payload = _stable_read(
                destination,
                f'{identifier} rendered diagram',
                maximum_bytes=MAX_SVG_BYTES,
            )
            source_record = next(
                diagram for diagram in candidate['diagrams'] if diagram['id'] == identifier
            )
            render_records.append(
                {
                    'bytes': len(payload),
                    'diagram_id': identifier,
                    'dimensions': {'height': height, 'width': width},
                    'render_path': f'diagrams/{identifier}.svg',
                    'render_sha256': _sha256_bytes(payload),
                    'source_path': source_record['source_path'],
                    'source_sha256': source_record['source_sha256'],
                    'structural_verdict': 'PASS',
                }
            )
        review = _load_canonical_json(visual_review, 'portfolio visual review')
        prepare = _load_canonical_json(attempt / 'prepare-report.json', 'portfolio prepare report')
        review = _validate_visual_review(
            review,
            attempt_id=attempt_id,
            candidate_sha=candidate_sha,
            candidate=candidate,
            render_records=render_records,
            prepared_utc=_utc_timestamp(prepare['prepared_utc'], 'portfolio prepared timestamp'),
        )
        finalized_utc = datetime.now(UTC).isoformat().replace('+00:00', 'Z')
        _require(
            _utc_timestamp(finalized_utc, 'portfolio finalized timestamp')
            >= _utc_timestamp(review['reviewed_utc'], 'portfolio visual review timestamp'),
            'portfolio finalization predates human visual review',
        )
        _copy_once(
            visual_review,
            attempt / 'visual-review.json',
            'portfolio visual review',
            maximum_bytes=MAX_JSON_BYTES,
        )
        projection = attempt / 'projection'
        projection.mkdir(mode=0o700)
        names = _projection_file_map(candidate_sha)
        for identifier in DIAGRAM_IDS:
            _copy_once(
                attempt / f'diagrams/{identifier}.svg',
                projection / Path(names[identifier]).name,
                f'{identifier} projection source',
                maximum_bytes=MAX_SVG_BYTES,
            )
        _copy_once(
            attempt / 'phase3/localization-error.png',
            projection / Path(names['chart']).name,
            'Scenario 5 chart projection source',
            maximum_bytes=MAX_PNG_BYTES,
        )
        proof = {
            'candidate': {
                'git_sha': candidate_sha,
                'readme_sha256': candidate['readme_sha256'],
                'replay_matrix_sha256': candidate['matrix_sha256'],
                'tree_entry_count': candidate['candidate_tree_entry_count'],
                'tree_listing_sha256': candidate['candidate_tree_listing_sha256'],
            },
            'diagrams': render_records,
            'documentation_replay': {
                'authorized_mutating_command_ids': replay['authorized_mutating_command_ids'],
                'attempt_id': attempt_id,
                'command_count': replay['command_count'],
                'command_ids': [command['id'] for command in replay['commands']],
                'cwd_mapping_policy': REPLAY_MAPPING_POLICY,
                'fresh_detached_worktree_per_command': True,
                'mutating_commands_authorized': replay['mutating_commands_authorized'],
                'report_sha256': file_sha256(attempt / 'documentation/replay-report.json'),
            },
            'phase3_chart': media,
            'prepared_utc': prepare['prepared_utc'],
            'producer': 'robotest_phase5/portfolio_finalize',
            'projection': {
                'checksum_covered_paths': sorted(
                    [names['proof'], names['architecture'], names['release-flow'], names['chart']]
                ),
                'paths': list(portfolio_projection_paths(candidate_sha)),
                'required_git_mode': '100644',
            },
            'quality': {
                'candidate_contract_passed': True,
                'documentation_replay_passed': True,
                'human_visual_review_passed': True,
                'phase3_chart_lineage_passed': True,
                'safe_svg_passed': True,
            },
            'schema_version': 1,
            'status': 'PASS',
            'finalized_utc': finalized_utc,
        }
        proof_path = projection / Path(names['proof']).name
        _write_json_once(proof_path, proof)
        manifest_payload, validation_payload = _projection_checksum_payload(
            projection,
            candidate_sha,
        )
        _write_once(projection / Path(names['manifest']).name, manifest_payload)
        _write_once(projection / Path(names['validation']).name, validation_payload)
        final_report = {
            'attempt_id': attempt_id,
            'candidate_git_sha': candidate_sha,
            'finalized_utc': finalized_utc,
            'portfolio_proof_path': proof_path.relative_to(repository).as_posix(),
            'portfolio_proof_sha256': file_sha256(proof_path),
            'producer': 'robotest_phase5/portfolio_finalize',
            'prepared_utc': prepare['prepared_utc'],
            'projection_paths': list(portfolio_projection_paths(candidate_sha)),
            'schema_version': 1,
            'status': 'PASS',
            'visual_review_sha256': file_sha256(attempt / 'visual-review.json'),
        }
        _write_json_once(attempt / 'finalize-report.json', final_report)
        _write_json_once(attempt / 'raw-manifest-metadata.json', _raw_payload_stats(attempt))
        raw_records = _write_raw_checksum_manifest(attempt)
        validated_raw_records = _validate_raw_checksum_manifest(attempt, candidate_sha)
        _require(raw_records == validated_raw_records, 'finalized raw portfolio manifest changed')
        return {**final_report, 'raw_manifest_file_count': len(validated_raw_records)}
    except BaseException as error:
        _write_attempt_failure(
            attempt,
            candidate_sha,
            error,
            stage='portfolio_finalize',
        )
        raise


def _expected_finalized_raw_paths(candidate_sha: str) -> set[str]:
    _require(
        GIT_SHA.fullmatch(candidate_sha) is not None, 'candidate Git SHA must be full lowercase'
    )
    paths = {
        'SHA256SUMS',
        'candidate-contract.json',
        'checksum-validation.txt',
        'diagrams/architecture.mmd',
        'diagrams/architecture.svg',
        'diagrams/release-flow.mmd',
        'diagrams/release-flow.svg',
        'documentation/replay-report.json',
        'finalize-report.json',
        'phase3/aggregate-result.json',
        'phase3/localization-error.png',
        'phase3/media-report.json',
        'phase3/run-artifacts.manifest.json',
        'phase3/run-artifacts.manifest.json.sha256',
        'phase3/run-result.json',
        'phase3/suite-plan.json',
        'phase3/suite-plan.json.sha256',
        'prepare-report.json',
        'primary-candidate-state.json',
        'raw-manifest-metadata.json',
        'render-request.json',
        'visual-review.json',
    }
    unit_files = {
        'environment.json',
        'result.json',
        'source-state-after.json',
        'source-state-before.json',
        'stderr.log',
        'stdout.log',
        'tooling-seed.json',
    }
    for ordinal, identifier in enumerate(REPLAY_ALLOWED_CHANGES, start=1):
        root = f'documentation/{ordinal:02d}-{identifier}'
        paths.update(f'{root}/{name}' for name in unit_files)
    paths.update(
        f'projection/{Path(relative).name}'
        for relative in portfolio_projection_paths(candidate_sha)
    )
    return paths


def _validate_raw_checksum_manifest(attempt: Path, candidate_sha: str) -> dict[str, str]:
    attempt_metadata = attempt.lstat()
    _require(
        stat.S_ISDIR(attempt_metadata.st_mode) and stat.S_IMODE(attempt_metadata.st_mode) == 0o700,
        'raw portfolio attempt directory mode is not 0700',
    )
    expected_files = _expected_finalized_raw_paths(candidate_sha)
    expected_directories: set[str] = set()
    for relative in expected_files:
        parent = Path(relative).parent
        while parent != Path('.'):
            expected_directories.add(parent.as_posix())
            parent = parent.parent
    actual_files: set[str] = set()
    actual_directories: set[str] = set()
    for path in sorted(attempt.rglob('*')):
        _require(not path.is_symlink(), f'raw portfolio attempt contains a symlink: {path}')
        relative = path.relative_to(attempt).as_posix()
        metadata = path.lstat()
        if stat.S_ISDIR(metadata.st_mode):
            actual_directories.add(relative)
        else:
            _require(
                stat.S_ISREG(metadata.st_mode) and metadata.st_nlink == 1,
                f'raw portfolio file type or link count changed: {relative}',
            )
            actual_files.add(relative)
    _require(
        actual_files == expected_files and actual_directories == expected_directories,
        'raw portfolio attempt file set differs from the exact allowlist',
    )
    for relative in sorted(expected_directories):
        _require(
            stat.S_IMODE((attempt / relative).lstat().st_mode) == 0o700,
            f'raw portfolio directory mode is not 0700: {relative}',
        )
    for relative in sorted(expected_files):
        _require(
            stat.S_IMODE((attempt / relative).lstat().st_mode) == 0o600,
            f'raw portfolio file mode is not 0600: {relative}',
        )
    manifest_path = attempt / 'SHA256SUMS'
    validation_path = attempt / 'checksum-validation.txt'
    manifest = _stable_read(
        manifest_path,
        'raw portfolio checksum manifest',
        maximum_bytes=4 * 1024 * 1024,
    )
    validation = _stable_read(
        validation_path,
        'raw portfolio checksum validation',
        maximum_bytes=4 * 1024 * 1024,
    )
    try:
        lines = manifest.decode('ascii').splitlines()
    except UnicodeError as exc:
        raise EvidenceError('raw portfolio checksum manifest is not ASCII') from exc
    declared: dict[str, str] = {}
    for line in lines:
        match = re.fullmatch(r'(?P<sha>[0-9a-f]{64})  (?P<path>[^\r\n]+)', line)
        _require(match is not None, 'raw portfolio checksum line is invalid')
        relative = _safe_relative(match.group('path'), 'raw portfolio checksum path')
        _require(relative not in declared, 'raw portfolio checksum path is duplicated')
        declared[relative] = match.group('sha')
    actual: dict[str, bytes] = {}
    total_bytes = 0
    for relative in sorted(expected_files):
        if relative in {'SHA256SUMS', 'checksum-validation.txt'}:
            continue
        path = attempt / relative
        _safe_relative(relative, 'raw portfolio file path')
        payload = _stable_read(
            path,
            f'raw portfolio file {relative}',
            maximum_bytes=MAX_RESULT_DIRECTORY_BYTES,
            allow_empty=True,
        )
        actual[relative] = payload
        total_bytes += len(payload)
    _require(len(actual) <= MAX_RAW_FILE_COUNT, 'raw portfolio file-count cap exceeded')
    _require(total_bytes <= MAX_RAW_TOTAL_BYTES, 'raw portfolio aggregate byte cap exceeded')
    _require(set(actual) == set(declared), 'raw portfolio checksum coverage is not exact')
    _require(
        manifest == ''.join(f'{declared[path]}  {path}\n' for path in sorted(declared)).encode(),
        'raw portfolio checksum manifest is not canonical',
    )
    for relative, payload in actual.items():
        _require(
            _sha256_bytes(payload) == declared[relative],
            f'raw portfolio checksum mismatch: {relative}',
        )
    expected_validation = ''.join(f'{path}: OK\n' for path in sorted(declared)).encode()
    _require(validation == expected_validation, 'raw portfolio checksum validation changed')
    metadata = _load_canonical_json(
        attempt / 'raw-manifest-metadata.json',
        'raw portfolio manifest metadata',
    )
    _require(metadata == _raw_payload_stats(attempt), 'raw portfolio aggregate metadata changed')
    return declared


def _validate_projection_checksum(projection: Path, candidate_sha: str) -> None:
    names = _projection_file_map(candidate_sha)
    expected_manifest, expected_validation = _projection_checksum_payload(
        projection,
        candidate_sha,
    )
    observed_manifest = _stable_read(
        projection / Path(names['manifest']).name,
        'portfolio projection checksum manifest',
        maximum_bytes=4096,
    )
    observed_validation = _stable_read(
        projection / Path(names['validation']).name,
        'portfolio projection checksum validation',
        maximum_bytes=4096,
    )
    _require(observed_manifest == expected_manifest, 'portfolio projection checksums changed')
    _require(observed_validation == expected_validation, 'portfolio projection validation changed')


def _validate_finalized_attempt(
    repository: Path,
    portfolio_root: Path,
    candidate_sha: str,
    phase3_candidate_root: Path,
    attempt_id: str,
) -> dict[str, Any]:
    attempt = _attempt_directory(portfolio_root, attempt_id, create=False)
    first_manifest = _validate_raw_checksum_manifest(attempt, candidate_sha)
    candidate, _replay, media = _validate_prepared_attempt(
        repository,
        attempt,
        candidate_sha,
        phase3_candidate_root,
    )
    prepare = _load_canonical_json(attempt / 'prepare-report.json', 'portfolio prepare report')
    render_records: list[dict[str, Any]] = []
    for identifier in DIAGRAM_IDS:
        source_record = next(
            diagram for diagram in candidate['diagrams'] if diagram['id'] == identifier
        )
        svg = attempt / f'diagrams/{identifier}.svg'
        width, height = _svg_dimensions(svg, f'{identifier} rendered diagram')
        payload = _stable_read(svg, f'{identifier} rendered diagram', maximum_bytes=MAX_SVG_BYTES)
        render_records.append(
            {
                'bytes': len(payload),
                'diagram_id': identifier,
                'dimensions': {'height': height, 'width': width},
                'render_path': f'diagrams/{identifier}.svg',
                'render_sha256': _sha256_bytes(payload),
                'source_path': source_record['source_path'],
                'source_sha256': source_record['source_sha256'],
                'structural_verdict': 'PASS',
            }
        )
    review = _load_canonical_json(attempt / 'visual-review.json', 'stored portfolio visual review')
    review = _validate_visual_review(
        review,
        attempt_id=attempt_id,
        candidate_sha=candidate_sha,
        candidate=candidate,
        render_records=render_records,
        prepared_utc=_utc_timestamp(prepare['prepared_utc'], 'portfolio prepared timestamp'),
    )
    projection = attempt / 'projection'
    names = _projection_file_map(candidate_sha)
    expected_paths = list(portfolio_projection_paths(candidate_sha))
    expected_files = {Path(path).name for path in expected_paths}
    _require(
        {path.name for path in projection.iterdir()} == expected_files,
        'raw portfolio projection file set changed',
    )
    for identifier in DIAGRAM_IDS:
        source = _stable_read(
            attempt / f'diagrams/{identifier}.svg',
            f'{identifier} finalized SVG',
            maximum_bytes=MAX_SVG_BYTES,
        )
        projected = _stable_read(
            projection / Path(names[identifier]).name,
            f'{identifier} raw projection',
            maximum_bytes=MAX_SVG_BYTES,
        )
        _require(projected == source, f'{identifier} raw projection differs from reviewed SVG')
    chart_source = _stable_read(
        attempt / 'phase3/localization-error.png',
        'raw Scenario 5 chart',
        maximum_bytes=MAX_PNG_BYTES,
    )
    chart_projected = _stable_read(
        projection / Path(names['chart']).name,
        'raw Scenario 5 chart projection',
        maximum_bytes=MAX_PNG_BYTES,
    )
    _require(chart_projected == chart_source, 'raw Scenario 5 chart projection changed')
    proof_path = projection / Path(names['proof']).name
    proof = _load_canonical_json(proof_path, 'raw portfolio proof')
    _require(
        set(proof)
        == {
            'candidate',
            'diagrams',
            'documentation_replay',
            'finalized_utc',
            'phase3_chart',
            'prepared_utc',
            'producer',
            'projection',
            'quality',
            'schema_version',
            'status',
        }
        and proof.get('candidate')
        == {
            'git_sha': candidate_sha,
            'readme_sha256': candidate['readme_sha256'],
            'replay_matrix_sha256': candidate['matrix_sha256'],
            'tree_entry_count': candidate['candidate_tree_entry_count'],
            'tree_listing_sha256': candidate['candidate_tree_listing_sha256'],
        }
        and proof.get('diagrams') == render_records
        and proof.get('phase3_chart') == media
        and proof.get('prepared_utc') == prepare.get('prepared_utc')
        and proof.get('producer') == 'robotest_phase5/portfolio_finalize'
        and proof.get('schema_version') == 1
        and proof.get('status') == 'PASS',
        'raw portfolio proof identity or evidence changed',
    )
    prepared_utc = _utc_timestamp(proof.get('prepared_utc'), 'portfolio prepared timestamp')
    finalized_utc = _utc_timestamp(proof.get('finalized_utc'), 'portfolio finalized timestamp')
    reviewed_utc = _utc_timestamp(review.get('reviewed_utc'), 'portfolio visual review timestamp')
    _require(
        prepared_utc <= reviewed_utc <= finalized_utc,
        'portfolio prepared/reviewed/finalized chronology changed',
    )
    documentation = _mapping(proof.get('documentation_replay'), 'portfolio documentation proof')
    _require(
        documentation.get('attempt_id') == attempt_id
        and documentation.get('command_count') == 11
        and documentation.get('command_ids') == list(REPLAY_ALLOWED_CHANGES)
        and documentation.get('cwd_mapping_policy') == REPLAY_MAPPING_POLICY
        and documentation.get('fresh_detached_worktree_per_command') is True
        and documentation.get('authorized_mutating_command_ids')
        == ['phase0-source-apply', 'phase0-install']
        and documentation.get('mutating_commands_authorized') is True
        and documentation.get('report_sha256')
        == file_sha256(attempt / 'documentation/replay-report.json'),
        'raw portfolio documentation replay binding changed',
    )
    projection_record = _mapping(proof.get('projection'), 'portfolio projection record')
    _require(
        projection_record.get('paths') == expected_paths
        and projection_record.get('required_git_mode') == '100644'
        and projection_record.get('checksum_covered_paths')
        == sorted([names['proof'], names['architecture'], names['release-flow'], names['chart']]),
        'raw portfolio projection contract changed',
    )
    _require(
        proof.get('quality')
        == {
            'candidate_contract_passed': True,
            'documentation_replay_passed': True,
            'human_visual_review_passed': True,
            'phase3_chart_lineage_passed': True,
            'safe_svg_passed': True,
        },
        'raw portfolio quality is not exact PASS',
    )
    _validate_projection_checksum(projection, candidate_sha)
    final_report = _load_canonical_json(
        attempt / 'finalize-report.json', 'portfolio finalize report'
    )
    _require(
        set(final_report)
        == {
            'attempt_id',
            'candidate_git_sha',
            'finalized_utc',
            'portfolio_proof_path',
            'portfolio_proof_sha256',
            'prepared_utc',
            'producer',
            'projection_paths',
            'schema_version',
            'status',
            'visual_review_sha256',
        }
        and final_report.get('attempt_id') == attempt_id
        and final_report.get('candidate_git_sha') == candidate_sha
        and final_report.get('finalized_utc') == proof.get('finalized_utc')
        and final_report.get('portfolio_proof_path')
        == proof_path.relative_to(repository).as_posix()
        and final_report.get('portfolio_proof_sha256') == file_sha256(proof_path)
        and final_report.get('projection_paths') == expected_paths
        and final_report.get('prepared_utc') == proof.get('prepared_utc')
        and final_report.get('producer') == 'robotest_phase5/portfolio_finalize'
        and final_report.get('schema_version') == 1
        and final_report.get('status') == 'PASS'
        and final_report.get('visual_review_sha256') == file_sha256(attempt / 'visual-review.json'),
        'portfolio finalize report changed',
    )
    _require(
        _validate_raw_checksum_manifest(attempt, candidate_sha) == first_manifest,
        'raw portfolio attempt changed during validation',
    )
    return {
        'attempt': attempt,
        'media': media,
        'portfolio_proof': proof,
        'portfolio_proof_path': proof_path,
        'projection': projection,
        'review': review,
        'prepared_utc': proof['prepared_utc'],
        'finalized_utc': proof['finalized_utc'],
    }


def portfolio_attempt_id(
    portfolio_root: Path,
    raw_proof: Path,
    candidate_sha: str,
) -> str:
    """Return the safe attempt ID only for its exact finalized raw-proof path."""
    _require(
        GIT_SHA.fullmatch(candidate_sha) is not None, 'candidate Git SHA must be full lowercase'
    )
    root = _resolved_directory(portfolio_root, 'Phase 5 portfolio root')
    proof = _regular_file(raw_proof, 'raw portfolio proof').resolve(strict=True)
    expected_name = f'portfolio-{candidate_sha}.json'
    _require(
        proof.name == expected_name
        and proof.parent.name == 'projection'
        and proof.parents[2].name == 'attempts'
        and proof.parents[3] == root,
        'raw portfolio proof is outside the exact attempt projection path',
    )
    attempt_id = _attempt_identifier(proof.parent.parent.name)
    document = _load_canonical_json(proof, 'raw portfolio proof')
    documentation = _mapping(document.get('documentation_replay'), 'raw portfolio replay proof')
    _require(
        documentation.get('attempt_id') == attempt_id
        and _mapping(document.get('candidate'), 'raw portfolio candidate').get('git_sha')
        == candidate_sha,
        'raw portfolio proof does not bind its path identity',
    )
    return attempt_id


def project_portfolio(
    repository: Path,
    portfolio_root: Path,
    candidate_sha: str,
    phase3_candidate_root: Path,
    attempt_id: str,
) -> dict[str, Any]:
    """Copy exactly six validated raw projection files into the evidence commit."""
    repository = _resolved_directory(repository, 'repository')
    candidate_sha = _validate_candidate_sha(repository, candidate_sha)
    expected_root = _portfolio_root(repository, candidate_sha, create=False)
    supplied_root = _resolved_directory(portfolio_root, 'Phase 5 portfolio root')
    _require(supplied_root == expected_root, 'Phase 5 portfolio root is not canonical')
    validated = _validate_finalized_attempt(
        repository,
        supplied_root,
        candidate_sha,
        phase3_candidate_root,
        attempt_id,
    )
    projection = validated['projection']
    paths = portfolio_projection_paths(candidate_sha)
    payloads: dict[str, bytes] = {}
    projection_root = repository / PORTFOLIO_PROJECTION_ROOT
    _reject_symlink_components(projection_root, 'tracked portfolio projection root')
    projection_root.mkdir(parents=True, exist_ok=True)
    _resolved_directory(projection_root, 'tracked portfolio projection root')
    for relative in paths:
        source = projection / Path(relative).name
        payloads[relative] = _stable_read(
            source,
            f'raw projection {relative}',
            maximum_bytes=MAX_JSON_BYTES,
        )
        destination = repository / relative
        _require(
            not destination.exists() and not destination.is_symlink(),
            f'refusing to overwrite tracked portfolio projection: {relative}',
        )
    created: dict[Path, tuple[int, int] | None] = {}
    try:
        for relative in paths:
            destination = repository / relative
            created[destination] = None
            _write_once(destination, payloads[relative], mode=0o644)
            metadata = destination.lstat()
            _require(
                stat.S_ISREG(metadata.st_mode)
                and metadata.st_nlink == 1
                and stat.S_IMODE(metadata.st_mode) == 0o644,
                f'tracked portfolio projection mode is not 0644: {relative}',
            )
            created[destination] = (metadata.st_dev, metadata.st_ino)
        proof_path = repository / paths[0]
        report = {
            'attempt_id': attempt_id,
            'candidate_git_sha': candidate_sha,
            'finalized_utc': validated['finalized_utc'],
            'portfolio_proof_path': paths[0],
            'portfolio_proof_sha256': file_sha256(proof_path),
            'prepared_utc': validated['prepared_utc'],
            'projection_paths': list(paths),
            'scenario5_run_id': validated['media']['run_id'],
            'status': 'PASS',
        }
        return report
    except BaseException:
        cleanup_failed: list[str] = []
        for destination, identity in reversed(created.items()):
            try:
                metadata = destination.lstat()
                if (
                    stat.S_ISREG(metadata.st_mode)
                    and metadata.st_nlink == 1
                    and (identity is None or (metadata.st_dev, metadata.st_ino) == identity)
                    and _stable_read(
                        destination,
                        'partial tracked portfolio projection',
                        maximum_bytes=MAX_JSON_BYTES,
                    )
                    == payloads[destination.relative_to(repository).as_posix()]
                ):
                    destination.unlink()
                else:
                    cleanup_failed.append(str(destination))
            except FileNotFoundError:
                continue
            except (EvidenceError, OSError):
                cleanup_failed.append(str(destination))
        _require(
            not cleanup_failed,
            'cannot safely remove partial tracked portfolio projection: '
            + ', '.join(cleanup_failed),
        )
        raise


def validate_portfolio_evidence(
    repository: Path,
    portfolio_root: Path,
    portfolio_proof: Path,
    phase3_candidate_root: Path,
    candidate_sha: str,
) -> dict[str, Any]:
    """Read-only validation of one explicit raw attempt and its six tracked copies."""
    repository = _resolved_directory(repository, 'repository')
    candidate_sha = _validate_candidate_sha(repository, candidate_sha)
    expected_root = _portfolio_root(repository, candidate_sha, create=False)
    supplied_root = _resolved_directory(portfolio_root, 'Phase 5 portfolio root')
    _require(supplied_root == expected_root, 'Phase 5 portfolio root is not canonical')
    paths = portfolio_projection_paths(candidate_sha)
    expected_proof = repository / paths[0]
    supplied_proof = _regular_file(portfolio_proof, 'tracked portfolio proof').resolve(strict=True)
    _require(supplied_proof == expected_proof, 'tracked portfolio proof path is not exact')
    tracked_document = _load_canonical_json(supplied_proof, 'tracked portfolio proof')
    documentation = _mapping(
        tracked_document.get('documentation_replay'),
        'tracked portfolio documentation replay',
    )
    attempt_id = _attempt_identifier(documentation.get('attempt_id'))
    validated = _validate_finalized_attempt(
        repository,
        supplied_root,
        candidate_sha,
        phase3_candidate_root,
        attempt_id,
    )
    projection = validated['projection']
    for relative in paths:
        tracked_path = repository / relative
        tracked_metadata = tracked_path.lstat()
        _require(
            stat.S_ISREG(tracked_metadata.st_mode)
            and tracked_metadata.st_nlink == 1
            and stat.S_IMODE(tracked_metadata.st_mode) == 0o644,
            f'tracked portfolio projection type, link count, or mode changed: {relative}',
        )
        raw = _stable_read(
            projection / Path(relative).name,
            f'raw portfolio projection {relative}',
            maximum_bytes=MAX_JSON_BYTES,
        )
        tracked = _stable_read(
            tracked_path,
            f'tracked portfolio projection {relative}',
            maximum_bytes=MAX_JSON_BYTES,
        )
        _require(tracked == raw, f'tracked portfolio projection differs from raw: {relative}')
    _require(
        tracked_document == validated['portfolio_proof'],
        'tracked portfolio proof differs from finalized raw proof',
    )
    return {
        'attempt_id': attempt_id,
        'candidate_git_sha': candidate_sha,
        'finalized_utc': validated['finalized_utc'],
        'portfolio_proof_path': paths[0],
        'portfolio_proof_sha256': file_sha256(supplied_proof),
        'prepared_utc': validated['prepared_utc'],
        'projection_paths': list(paths),
        'scenario5_run_id': validated['media']['run_id'],
        'status': 'PASS',
        'verification_scope': (
            'Read-only exact candidate README replay, reviewed safe SVG, deterministic Phase 3 '
            'Scenario 5 run 12 lineage, raw checksums, and six byte-identical tracked projections'
        ),
    }


def _common_candidate_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--repository', type=Path, required=True)
    parser.add_argument('--candidate-sha', required=True)


def _common_portfolio_arguments(parser: argparse.ArgumentParser) -> None:
    _common_candidate_arguments(parser)
    parser.add_argument('--phase3-candidate-root', type=Path, required=True)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='operation', required=True)
    contract = subparsers.add_parser('contract')
    _common_candidate_arguments(contract)
    prepare = subparsers.add_parser('prepare')
    _common_portfolio_arguments(prepare)
    prepare.add_argument('--attempt-id')
    prepare.add_argument('--authorize-mutating-commands', action='store_true')
    finalize = subparsers.add_parser('finalize')
    _common_portfolio_arguments(finalize)
    finalize.add_argument('--attempt-id', required=True)
    finalize.add_argument('--architecture-svg', type=Path, required=True)
    finalize.add_argument('--release-flow-svg', type=Path, required=True)
    finalize.add_argument('--visual-review', type=Path, required=True)
    project = subparsers.add_parser('project')
    _common_portfolio_arguments(project)
    project.add_argument('--portfolio-root', type=Path, required=True)
    project.add_argument('--attempt-id', required=True)
    validate = subparsers.add_parser('validate')
    _common_portfolio_arguments(validate)
    validate.add_argument('--portfolio-root', type=Path, required=True)
    validate.add_argument('--portfolio-proof', type=Path, required=True)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        if arguments.operation == 'contract':
            report = validate_candidate_contract(arguments.repository, arguments.candidate_sha)
            exit_code = 0
        elif arguments.operation == 'prepare':
            report = prepare_portfolio(
                arguments.repository,
                arguments.candidate_sha,
                arguments.phase3_candidate_root,
                authorize_mutating_commands=arguments.authorize_mutating_commands,
                attempt_id=arguments.attempt_id,
            )
            exit_code = 3
        elif arguments.operation == 'finalize':
            report = finalize_portfolio(
                arguments.repository,
                arguments.candidate_sha,
                arguments.phase3_candidate_root,
                arguments.attempt_id,
                arguments.architecture_svg,
                arguments.release_flow_svg,
                arguments.visual_review,
            )
            exit_code = 0
        elif arguments.operation == 'project':
            report = project_portfolio(
                arguments.repository,
                arguments.portfolio_root,
                arguments.candidate_sha,
                arguments.phase3_candidate_root,
                arguments.attempt_id,
            )
            exit_code = 0
        else:
            report = validate_portfolio_evidence(
                arguments.repository,
                arguments.portfolio_root,
                arguments.portfolio_proof,
                arguments.phase3_candidate_root,
                arguments.candidate_sha,
            )
            exit_code = 0
    except (
        EvidenceError,
        OSError,
        UnicodeError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
    ) as error:
        raise SystemExit(str(error)) from error
    print(canonical_json_bytes(report).decode(), end='')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
