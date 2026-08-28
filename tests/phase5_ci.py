#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Fail-closed helpers for the Phase 5 local and remote CI gates."""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.metadata
import io
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import tempfile
import tomllib
import xml.etree.ElementTree as ET
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import yaml

ACTION_PINS = {
    'actions/checkout': 'de0fac2e4500dabe0009e67214ff5f5447ce83dd',  # v6.0.2
    'actions/upload-artifact': '043fb46d1a93c77aae656e7c1c64a875d1fc6a0a',  # v7.0.1
    'ros-tooling/setup-ros': '649ef6bcd696da05bc27ceb3fab69d810c0daeab',  # 0.7.19
}
ACTION_REFERENCE = re.compile(r'^(?P<owner>[^/@\s]+/[^/@\s]+)@(?P<sha>[0-9a-f]{40})$')
GIT_SHA = re.compile(r'^[0-9a-f]{40}$')
GITHUB_REPOSITORY = re.compile(r'^[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+$')
LOCAL_RUN_NAME = re.compile(r'^\d{8}T\d{6}Z-\d+$')
CHECKSUM_LINE = re.compile(r'^(?P<sha>[0-9a-f]{64})  (?P<path>[^\r\n]+)$')
CHECKSUM_FILES = {'SHA256SUMS', 'checksum-validation.txt'}
SOURCE_SNAPSHOT_INPUTS = (
    '.editorconfig',
    '.gitattributes',
    '.github',
    '.gitignore',
    '.pre-commit-config.yaml',
    'CONTRIBUTING.md',
    'LICENSE',
    'NOTICE.md',
    'README.md',
    'SECURITY.md',
    'benchmarks',
    'config',
    'docs',
    'packaging',
    'pyproject.toml',
    'scenarios',
    'scripts',
    'src',
    'supervisor',
    'tests',
)
GENERATED_SOURCE_PARTS = {'__pycache__', '.pytest_cache', '.ruff_cache'}
NON_LIVE_TEST_PATTERNS = {
    'Gazebo command': re.compile(r'\bgz\s+sim\b', re.IGNORECASE),
    'Phase 1 live verifier': re.compile(r'\bverify_phase1\.sh\b'),
    'Phase 2 live verifier': re.compile(r'\bverify_phase2\.sh\b'),
    'Phase 3 benchmark runner': re.compile(r'\brun_benchmarks\.sh\b'),
    'Phase 4 apply': re.compile(r'\bverify_phase4\.sh\s+--apply\b'),
    'ROS launch command': re.compile(r'\bros2\s+launch\b', re.IGNORECASE),
    'launch test registration': re.compile(r'\badd_launch_test\b|\blaunch_testing\b'),
    'systemd command': re.compile(r'\bsystemctl\b'),
}
PACKAGE_DEPENDENCY_TAGS = {
    'build_depend',
    'build_export_depend',
    'buildtool_depend',
    'buildtool_export_depend',
    'depend',
    'exec_depend',
    'test_depend',
}
SAFE_COMPONENT = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_.+-]*$')
FORBIDDEN_INVENTORY_TEXT = re.compile(r'\b(?:future|placeholder|tbd|todo|unknown)\b', re.I)


class EvidenceError(ValueError):
    """Raised when Phase 5 evidence or policy is incomplete."""


def canonical_json_bytes(value: object) -> bytes:
    """Return strict, deterministic JSON with a final newline."""
    return (
        json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
        + '\n'
    ).encode()


def atomic_write_json(path: Path, value: object) -> None:
    """Atomically replace a JSON evidence file on the same filesystem."""
    atomic_write_bytes(path, canonical_json_bytes(value))


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    """Atomically replace one evidence file and fsync its complete payload."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary: Path | None = None
    try:
        with tempfile.NamedTemporaryFile(
            dir=path.parent, prefix=f'.{path.name}.', delete=False
        ) as out:
            temporary = Path(out.name)
            out.write(payload)
            out.flush()
            os.fsync(out.fileno())
        os.replace(temporary, path)
    except BaseException:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
        raise


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def source_snapshot(repository: Path) -> dict[str, object]:
    """Hash the complete first-party surface exercised by the Phase 5 gate."""
    root = repository.resolve(strict=True)
    _require(root.is_dir() and not repository.is_symlink(), f'invalid repository: {repository}')
    files: set[Path] = set()
    for entry in SOURCE_SNAPSHOT_INPUTS:
        candidate = root / entry
        _require(
            candidate.exists() and not candidate.is_symlink(), f'missing source input: {entry}'
        )
        if candidate.is_file():
            files.add(candidate)
            continue
        _require(candidate.is_dir(), f'source input is not a file or directory: {entry}')
        for path in candidate.rglob('*'):
            _require(not path.is_symlink(), f'source input is a symlink: {path}')
            relative = path.relative_to(root)
            generated_benchmark = relative.parts[:2] == ('benchmarks', 'raw')
            if (
                generated_benchmark
                or GENERATED_SOURCE_PARTS & set(relative.parts)
                or path.suffix in {'.pyc', '.pyo'}
            ):
                continue
            if path.is_file():
                files.add(path)
    records = [
        {
            'bytes': path.stat().st_size,
            'path': path.relative_to(root).as_posix(),
            'sha256': file_sha256(path),
        }
        for path in sorted(files)
    ]
    _require(records, 'source snapshot is empty')
    return {
        'aggregate_sha256': hashlib.sha256(canonical_json_bytes(records)).hexdigest(),
        'file_count': len(records),
        'inputs': list(SOURCE_SNAPSHOT_INPUTS),
        'records': records,
        'schema_version': 1,
    }


def prune_local_runs(
    evidence_root: Path,
    current_run: Path,
    *,
    maximum_prior: int,
) -> dict[str, object]:
    """Retain the current resolved direct child and only the newest prior runs."""
    _require(0 <= maximum_prior <= 32, 'maximum prior runs must be in [0, 32]')
    root = evidence_root.resolve(strict=True)
    _require(root.is_dir() and not evidence_root.is_symlink(), 'invalid evidence root')
    _require(not current_run.is_symlink(), 'current run must not be a symlink')
    current = current_run.resolve(strict=True)
    _require(current.is_dir(), 'current run must be a directory')
    _require(current.parent == root, 'current run must be a direct child of the evidence root')
    _require(LOCAL_RUN_NAME.fullmatch(current.name) is not None, 'current run name is invalid')

    candidates: list[Path] = []
    for child in root.iterdir():
        if LOCAL_RUN_NAME.fullmatch(child.name) is None:
            continue
        _require(not child.is_symlink(), f'local evidence run is a symlink: {child.name}')
        resolved = child.resolve(strict=True)
        _require(
            resolved == child.absolute() and resolved.parent == root and resolved.is_dir(),
            f'local evidence run is not a resolved direct child: {child.name}',
        )
        candidates.append(resolved)

    prior = sorted((path for path in candidates if path != current), key=lambda path: path.name)
    retained_prior = prior[-maximum_prior:] if maximum_prior else []
    removed = [path for path in prior if path not in retained_prior]
    for path in removed:
        _require(not path.is_symlink(), f'retention target became a symlink: {path.name}')
        resolved = path.resolve(strict=True)
        _require(
            resolved == path and resolved.parent == root and resolved != current,
            f'unsafe retention target: {path}',
        )
        shutil.rmtree(resolved)
    return {
        'current_run': current.name,
        'maximum_prior_runs': maximum_prior,
        'policy': 'resolved direct-child local runs only',
        'removed_runs': [path.name for path in removed],
        'retained_prior_runs': [path.name for path in retained_prior],
        'schema_version': 1,
    }


def _evidence_hashes(run_directory: Path) -> dict[str, str]:
    root = run_directory.resolve(strict=True)
    _require(root.is_dir() and not run_directory.is_symlink(), 'invalid evidence run directory')
    hashes: dict[str, str] = {}
    for path in sorted(root.rglob('*')):
        _require(not path.is_symlink(), f'evidence path is a symlink: {path}')
        if not path.is_file():
            continue
        relative = path.relative_to(root).as_posix()
        if relative in CHECKSUM_FILES:
            continue
        _require('\n' not in relative and '\r' not in relative, 'evidence path contains a newline')
        hashes[relative] = file_sha256(path)
    _require(hashes, 'evidence run contains no files to hash')
    return hashes


def write_checksum_manifest(run_directory: Path) -> dict[str, object]:
    """Write an exact sorted manifest, excluding the manifest and validation sidecar."""
    hashes = _evidence_hashes(run_directory)
    payload = ''.join(f'{digest}  {name}\n' for name, digest in sorted(hashes.items())).encode(
        'utf-8'
    )
    atomic_write_bytes(run_directory / 'SHA256SUMS', payload)
    return {'file_count': len(hashes), 'manifest_sha256': hashlib.sha256(payload).hexdigest()}


def validate_checksum_manifest(run_directory: Path) -> dict[str, object]:
    """Validate hashes and prove the manifest names every eligible evidence file."""
    root = run_directory.resolve(strict=True)
    manifest = root / 'SHA256SUMS'
    _require(manifest.is_file() and not manifest.is_symlink(), 'checksum manifest is missing')
    declared: dict[str, str] = {}
    for line_number, line in enumerate(manifest.read_text(encoding='utf-8').splitlines(), start=1):
        match = CHECKSUM_LINE.fullmatch(line)
        _require(match is not None, f'invalid checksum line {line_number}')
        relative = match.group('path')
        relative_path = Path(relative)
        _require(
            not relative_path.is_absolute()
            and '..' not in relative_path.parts
            and relative not in CHECKSUM_FILES,
            f'unsafe checksum path: {relative}',
        )
        _require(relative not in declared, f'duplicate checksum path: {relative}')
        declared[relative] = match.group('sha')
    actual = _evidence_hashes(root)
    _require(set(declared) == set(actual), 'checksum manifest file set is not exact')
    for relative, expected in declared.items():
        _require(actual[relative] == expected, f'checksum mismatch: {relative}')
    return {'file_count': len(declared), 'status': 'PASS'}


def write_file_checksum_manifest(source: Path, manifest: Path) -> dict[str, object]:
    """Write a one-file checksum manifest for remote evidence."""
    _require(source.is_file() and not source.is_symlink(), f'missing remote evidence: {source}')
    _require(
        source.parent.resolve(strict=True) == manifest.parent.resolve(strict=True), 'sidecar moved'
    )
    _require(source.name != manifest.name, 'remote manifest cannot replace its source')
    _require('\n' not in source.name and '\r' not in source.name, 'unsafe remote evidence name')
    digest = file_sha256(source)
    payload = f'{digest}  {source.name}\n'.encode()
    atomic_write_bytes(manifest, payload)
    return {'file': source.name, 'sha256': digest}


def validate_file_checksum_manifest(source: Path, manifest: Path) -> dict[str, object]:
    """Validate a one-file remote evidence checksum manifest."""
    _require(
        manifest.is_file() and not manifest.is_symlink(), 'remote checksum manifest is missing'
    )
    lines = manifest.read_text(encoding='utf-8').splitlines()
    _require(len(lines) == 1, 'remote checksum manifest must contain exactly one record')
    match = CHECKSUM_LINE.fullmatch(lines[0])
    _require(
        match is not None and match.group('path') == source.name, 'remote checksum path changed'
    )
    _require(file_sha256(source) == match.group('sha'), 'remote evidence checksum mismatch')
    return {'file': source.name, 'status': 'PASS'}


def _command_version(command: list[str], cwd: Path) -> dict[str, object]:
    executable = shutil.which(command[0]) if not Path(command[0]).is_absolute() else command[0]
    _require(executable is not None and Path(executable).is_file(), f'missing tool: {command[0]}')
    completed = subprocess.run(
        [str(executable), *command[1:]],
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=15,
        check=False,
    )
    output = completed.stdout.strip()
    _require(completed.returncode == 0 and output, f'cannot identify tool: {command[0]}')
    _require(len(output.encode()) <= 16_384, f'tool version output is unbounded: {command[0]}')
    return {
        'argv': command,
        'executable': str(Path(executable).resolve()),
        'output': output,
    }


def _os_release() -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in Path('/etc/os-release').read_text(encoding='utf-8').splitlines():
        if '=' not in line:
            continue
        key, value = line.split('=', 1)
        fields[key] = value.strip().strip('"')
    return {key: fields[key] for key in ('ID', 'VERSION_ID', 'PRETTY_NAME') if key in fields}


def _platform_provenance() -> dict[str, object]:
    return {
        'github_runner': {
            'architecture': os.environ.get('RUNNER_ARCH'),
            'environment': os.environ.get('RUNNER_ENVIRONMENT'),
            # GitHub-hosted runners expose these two names with exact mixed casing.
            'image_os': os.environ.get('ImageOS'),  # noqa: SIM112
            'image_version': os.environ.get('ImageVersion'),  # noqa: SIM112
            'operating_system': os.environ.get('RUNNER_OS'),
        },
        'os_release': _os_release(),
        'python': {
            'executable': str(Path(sys.executable).resolve()),
            'version': platform.python_version(),
        },
        'ros_distro': os.environ.get('ROS_DISTRO'),
        'uname': list(platform.uname()),
        'wsl_distro_name': os.environ.get('WSL_DISTRO_NAME'),
    }


def write_source_snapshot(repository: Path, output: Path) -> None:
    atomic_write_json(output, source_snapshot(repository))


def write_provenance(args: argparse.Namespace) -> None:
    repository = args.repository.resolve(strict=True)
    start = json.loads(args.source_snapshot.read_text(encoding='utf-8'))
    end = source_snapshot(repository)
    _require(start == end, 'source tree changed during Phase 5 verification')
    command = list(args.command_argument)
    _require(command, 'exact verifier command is missing')
    git_status = args.git_status.read_text(encoding='utf-8')
    tools = {
        'bash': _command_version(['bash', '--version'], repository),
        'cmake': _command_version(['cmake', '--version'], repository),
        'gcc': _command_version(['gcc', '--version'], repository),
        'git': _command_version(['git', '--version'], repository),
        'go': _command_version(['go', 'version'], repository),
        'rosdep': _command_version(['rosdep', '--version'], repository),
        'ruff': _command_version([str(args.ruff), '--version'], repository),
        'sha256sum': _command_version(['sha256sum', '--version'], repository),
        'shellcheck': _command_version(['shellcheck', '--version'], repository),
    }
    distributions = {}
    for distribution in ('colcon-core', 'pytest', 'PyYAML'):
        distributions[distribution] = importlib.metadata.version(distribution)
    colcon = shutil.which('colcon')
    _require(colcon is not None, 'missing tool: colcon')
    tools['colcon'] = {
        'distribution': 'colcon-core',
        'executable': str(Path(colcon).resolve()),
        'version': distributions['colcon-core'],
    }
    pytest_executable = shutil.which('pytest')
    _require(pytest_executable is not None, 'missing tool: pytest')
    tools['pytest'] = {
        'distribution': 'pytest',
        'executable': str(Path(pytest_executable).resolve()),
        'version': distributions['pytest'],
    }
    atomic_write_json(
        args.output,
        {
            'command': {'argv': command, 'cwd': str(args.cwd.resolve(strict=True))},
            'git': {
                'dirty': args.git_dirty == 'true',
                'sha': args.git_sha,
                'status_porcelain': git_status,
                'status_porcelain_sha256': file_sha256(args.git_status),
            },
            'mode': args.mode,
            'platform': _platform_provenance(),
            'repository': str(repository),
            'schema_version': 1,
            'source': {
                'aggregate_sha256': start['aggregate_sha256'],
                'file_count': start['file_count'],
                'snapshot': args.source_snapshot.name,
            },
            'tool_distributions': distributions,
            'tools': tools,
            'worker_limit': args.maximum_workers,
        },
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceError(message)


def _mapping(value: object, label: str) -> dict[str, Any]:
    _require(isinstance(value, dict), f'{label} must be a mapping')
    return value


def validate_workflow(path: Path) -> dict[str, object]:
    """Validate the standard-runner workflow without executing it."""
    _require(path.is_file() and not path.is_symlink(), f'missing regular workflow: {path}')
    raw = path.read_text(encoding='utf-8')
    _require('\t' not in raw, 'workflow contains tab indentation')
    document = _mapping(yaml.safe_load(raw), 'workflow')
    normalized_keys = {'on' if key is True else key for key in document}
    _require(
        normalized_keys == {'name', 'on', 'permissions', 'concurrency', 'env', 'jobs'},
        'workflow top-level contract changed',
    )
    _require(document.get('name') == 'RoboTest CI', 'workflow name changed')

    triggers = document.get('on', document.get(True))
    triggers = _mapping(triggers, 'workflow triggers')
    _require('pull_request_target' not in triggers, 'pull_request_target is forbidden')
    _require(
        set(triggers) == {'push', 'pull_request', 'workflow_dispatch'},
        'workflow triggers changed',
    )

    concurrency = _mapping(document.get('concurrency'), 'workflow concurrency')
    _require(
        concurrency
        == {
            'group': 'robotest-ci-${{ github.workflow }}-${{ github.ref }}',
            'cancel-in-progress': True,
        },
        'workflow concurrency contract changed',
    )

    permissions = _mapping(document.get('permissions'), 'workflow permissions')
    _require(
        permissions == {'contents': 'read'},
        'workflow permissions must be contents: read only',
    )

    jobs = _mapping(document.get('jobs'), 'workflow jobs')
    _require(set(jobs) == {'quality'}, 'workflow must expose exactly one quality job')
    job = _mapping(jobs['quality'], 'quality job')
    _require(
        set(job) == {'name', 'runs-on', 'timeout-minutes', 'steps'},
        'quality job contract changed',
    )
    _require(job.get('runs-on') == 'ubuntu-24.04', 'quality job must use ubuntu-24.04')
    _require('container' not in job, 'quality job must use the standard hosted runner directly')
    timeout = job.get('timeout-minutes')
    _require(isinstance(timeout, int) and 1 <= timeout <= 60, 'job timeout must be 1..60 minutes')

    environment = _mapping(document.get('env'), 'workflow environment')
    _require(
        set(environment)
        == {'CMAKE_BUILD_PARALLEL_LEVEL', 'MAKEFLAGS', 'ROBOTEST_CI_MAX_WORKERS', 'ROS_DISTRO'},
        'workflow environment contract changed',
    )
    _require(str(environment.get('ROS_DISTRO')) == 'jazzy', 'ROS_DISTRO must be jazzy')
    _require(
        str(environment.get('ROBOTEST_CI_MAX_WORKERS')) == '4',
        'ROBOTEST_CI_MAX_WORKERS must be exactly 4',
    )
    _require(
        str(environment.get('CMAKE_BUILD_PARALLEL_LEVEL')) == '4',
        'CMAKE_BUILD_PARALLEL_LEVEL must be exactly 4',
    )
    _require(str(environment.get('MAKEFLAGS')) == '-j4', 'MAKEFLAGS must be exactly -j4')

    steps = job.get('steps')
    _require(isinstance(steps, list) and steps, 'quality job must contain steps')
    expected_step_names = [
        'Check out the exact workflow revision',
        'Prepare ROS 2 Jazzy',
        'Resolve declared ROS dependencies',
        'Prepare pinned repository tooling',
        'Run the non-live Phase 5 CI gate',
        'Preserve bounded Phase 5 evidence',
    ]
    expected_step_keys = [
        {'name', 'uses', 'with'},
        {'name', 'uses', 'with'},
        {'name', 'shell', 'run'},
        {'name', 'shell', 'run'},
        {'name', 'shell', 'run'},
        {'name', 'if', 'uses', 'with'},
    ]
    _require(len(steps) == len(expected_step_names), 'workflow step count changed')
    action_uses: list[str] = []
    run_commands: list[str] = []
    checkout_step: dict[str, Any] | None = None
    setup_step: dict[str, Any] | None = None
    upload_step: dict[str, Any] | None = None
    for index, item in enumerate(steps):
        step = _mapping(item, f'quality step {index}')
        _require(step.get('name') == expected_step_names[index], f'quality step {index} changed')
        _require(set(step) == expected_step_keys[index], f'quality step {index} keys changed')
        if 'uses' in step:
            reference = step['uses']
            _require(isinstance(reference, str), f'quality step {index} uses must be text')
            match = ACTION_REFERENCE.fullmatch(reference)
            _require(match is not None, f'action is not pinned to a full SHA: {reference}')
            owner = match.group('owner')
            _require(owner in ACTION_PINS, f'unreviewed external action: {owner}')
            _require(match.group('sha') == ACTION_PINS[owner], f'wrong reviewed pin for {owner}')
            action_uses.append(owner)
            if owner == 'actions/checkout':
                checkout_step = step
            elif owner == 'ros-tooling/setup-ros':
                setup_step = step
            elif owner == 'actions/upload-artifact':
                upload_step = step
        if 'run' in step:
            command = step['run']
            _require(isinstance(command, str), f'quality step {index} run must be text')
            _require(step.get('shell') == 'bash', f'quality step {index} must use Bash')
            run_commands.append(command)

    _require(
        action_uses == ['actions/checkout', 'ros-tooling/setup-ros', 'actions/upload-artifact'],
        'workflow action order or allowlist changed',
    )
    _require(checkout_step is not None, 'checkout action is missing')
    checkout_inputs = _mapping(checkout_step.get('with'), 'checkout inputs')
    _require(
        checkout_inputs.get('persist-credentials') is False,
        'checkout must disable persisted credentials',
    )
    _require(set(checkout_inputs) == {'persist-credentials'}, 'checkout inputs changed')

    _require(setup_step is not None, 'setup-ros action is missing')
    setup_inputs = _mapping(setup_step.get('with'), 'setup-ros inputs')
    _require(
        setup_inputs == {'required-ros-distributions': 'jazzy'},
        'setup-ros must install Jazzy',
    )

    _require(upload_step is not None, 'upload-artifact action is missing')
    _require(upload_step.get('if') == 'always()', 'evidence upload must run after failures')
    upload_inputs = _mapping(upload_step.get('with'), 'upload-artifact inputs')
    _require(
        upload_inputs
        == {
            'name': 'phase5-ci-${{ github.sha }}',
            'path': 'artifacts/evidence/phase5/',
            'if-no-files-found': 'error',
            'retention-days': 14,
        },
        'evidence upload contract changed',
    )

    combined_commands = '\n'.join(run_commands)
    _require(
        combined_commands.count('scripts/verify_phase5.sh --ci') == 1,
        'workflow must invoke the Phase 5 CI gate exactly once',
    )
    _require(
        combined_commands.count('rosdep update --rosdistro jazzy') == 1,
        'workflow must refresh the Jazzy rosdep cache exactly once',
    )
    _require(
        'rosdep install --from-paths src --ignore-src' in combined_commands,
        'workflow must resolve declared repository dependencies',
    )
    forbidden = {
        'Phase 1 live verifier': r'\bverify_phase1\.sh\b',
        'Phase 2 live verifier': r'\bverify_phase2\.sh\b',
        'Phase 3 campaign': r'\brun_benchmarks\.sh\s+campaign\b',
        'Phase 3 campaign authorization': r'I_AUTHORIZE_EXACTLY_15_COLD_STACK_TRIALS_NO_RETRIES',
        'Phase 4 apply': r'\bverify_phase4\.sh\s+--apply\b',
        'aggregate verifier': r'\bverify_all\.sh\b',
        'Gazebo launch': r'\bgz\s+sim\b|\bros2\s+launch\b',
        'systemd mutation': r'\bsystemctl\b',
        'package lifecycle mutation': r'\bdpkg\b|\bapt-get\b',
        'repository publication': r'\bgit\s+push\b|\bgh\s+repo\s+create\b',
    }
    for label, pattern in forbidden.items():
        _require(re.search(pattern, combined_commands) is None, f'workflow contains {label}')

    expected_run_lines = [
        [
            'set -Eeuo pipefail',
            'rosdep update --rosdistro jazzy',
            'rosdep install --from-paths src --ignore-src --rosdistro jazzy -y',
        ],
        [
            'set -Eeuo pipefail',
            'python3 -m venv --system-site-packages .venv',
            '.venv/bin/python -m pip install \\',
            '  --disable-pip-version-check \\',
            '  --no-deps \\',
            '  ruff==0.16.4',
        ],
        ['scripts/verify_phase5.sh --ci'],
    ]
    _require(
        [command.splitlines() for command in run_commands] == expected_run_lines,
        'workflow run-command contract changed',
    )

    return {
        'action_pins': {name: ACTION_PINS[name] for name in sorted(ACTION_PINS)},
        'job_timeout_minutes': timeout,
        'maximum_workers': 4,
        'path': path.as_posix(),
        'permissions': permissions,
        'runner': job['runs-on'],
        'sha256': file_sha256(path),
        'verification_scope': 'L0 workflow syntax and policy; no hosted job was executed',
    }


def _repository_regular_file(repository: Path, relative: str, label: str) -> Path:
    root = repository.resolve(strict=True)
    relative_path = Path(relative)
    _require(not relative_path.is_absolute() and '..' not in relative_path.parts, f'unsafe {label}')
    candidate = root / relative_path
    _require(
        candidate.is_file() and not candidate.is_symlink(), f'missing regular {label}: {relative}'
    )
    resolved = candidate.resolve(strict=True)
    _require(resolved.is_relative_to(root), f'{label} escapes repository: {relative}')
    return candidate


def _nonempty_strings(value: object, label: str) -> list[str]:
    _require(isinstance(value, list) and value, f'{label} must be a nonempty list')
    _require(
        all(isinstance(item, str) and item.strip() == item and item for item in value),
        f'{label} contains an invalid value',
    )
    return value


def _package_dependency_surface(package_xmls: list[Path], first_party: set[str]) -> set[str]:
    return {
        element.text.strip()
        for package_xml in package_xmls
        for element in ET.parse(package_xml).getroot()
        if element.tag in PACKAGE_DEPENDENCY_TAGS
        and element.text
        and element.text.strip()
        and element.text.strip() not in first_party
    }


def validate_license_declarations(repository: Path) -> dict[str, object]:
    """Validate first-party licensing and the complete direct-dependency inventory."""
    repository = repository.resolve(strict=True)
    license_path = _repository_regular_file(repository, 'LICENSE', 'root license')
    notice_path = _repository_regular_file(repository, 'NOTICE.md', 'notice')
    pyproject_path = _repository_regular_file(repository, 'pyproject.toml', 'pyproject')
    inventory_path = _repository_regular_file(
        repository,
        'config/dependency-license-inventory.json',
        'dependency license inventory',
    )
    root_license = license_path.read_text(encoding='utf-8')
    _require('Apache License' in root_license, 'LICENSE is not Apache')
    notice = notice_path.read_text(encoding='utf-8')
    _require('Third-party projects' in notice, 'NOTICE is incomplete')
    _require(
        '`config/dependency-license-inventory.json`' in notice,
        'NOTICE does not identify the dependency inventory',
    )
    _require('will be generated' not in notice.lower(), 'NOTICE still promises a future inventory')

    pyproject = tomllib.loads(pyproject_path.read_text(encoding='utf-8'))
    _require(
        pyproject.get('project', {}).get('license', {}).get('text') == 'Apache-2.0',
        'pyproject license must be Apache-2.0',
    )
    inventory = _mapping(
        json.loads(inventory_path.read_text(encoding='utf-8')),
        'dependency license inventory',
    )
    expected_inventory_keys = {
        'apt_packages',
        'audit',
        'first_party_packages',
        'github_actions',
        'go_modules',
        'python_distributions',
        'ros_dependencies',
        'rosdep_system_dependencies',
        'schema_version',
    }
    _require(set(inventory) == expected_inventory_keys, 'dependency inventory keys changed')
    _require(inventory['schema_version'] == 1, 'dependency inventory schema changed')
    inventory_text = canonical_json_bytes(inventory).decode()
    _require(
        FORBIDDEN_INVENTORY_TEXT.search(inventory_text) is None,
        'dependency inventory contains a placeholder',
    )
    audit = _mapping(inventory['audit'], 'dependency inventory audit')
    _require(
        set(audit) == {'captured_at_utc', 'platform', 'scope'},
        'dependency inventory audit keys changed',
    )
    for key in ('captured_at_utc', 'platform', 'scope'):
        _require(isinstance(audit[key], str) and audit[key], f'inventory audit {key} is missing')
    _require(
        'Direct repository declarations only' in audit['scope']
        and 'Resolver-added transitive' in audit['scope'],
        'dependency inventory scope is ambiguous',
    )

    package_xmls = sorted((repository / 'src').glob('*/package.xml'))
    packages: list[str] = []
    for package_xml in package_xmls:
        _require(not package_xml.is_symlink(), f'package manifest is a symlink: {package_xml}')
        package = ET.parse(package_xml).getroot()
        package_name = package.findtext('name')
        licenses = [item.text for item in package.findall('license')]
        _require(package_name is not None, f'package name missing: {package_xml}')
        _require(licenses == ['Apache-2.0'], f'{package_name} license must be Apache-2.0')
        package_license = package_xml.parent / 'LICENSE'
        _require(
            package_license.is_file() and not package_license.is_symlink(),
            f'{package_name} is missing a regular package-local LICENSE',
        )
        _require(
            package_license.read_text(encoding='utf-8').rstrip() == root_license.rstrip(),
            f'{package_name} package-local LICENSE differs from the root license',
        )
        packages.append(package_name)
    _require(packages, 'no ROS package manifests found')
    _require(packages == sorted(packages), 'first-party packages are not deterministically ordered')
    _require(
        inventory['first_party_packages'] == packages,
        'first-party dependency inventory is incomplete',
    )

    manifest_path = _repository_regular_file(
        repository, 'config/apt-packages.txt', 'apt package manifest'
    )
    apt_names = manifest_path.read_text(encoding='utf-8').splitlines()
    _require(
        apt_names
        and len(apt_names) == len(set(apt_names))
        and all(SAFE_COMPONENT.fullmatch(name) is not None for name in apt_names),
        'apt package manifest is invalid',
    )
    apt_inventory = _mapping(inventory['apt_packages'], 'apt package inventory')
    _require(set(apt_inventory) == set(apt_names), 'apt dependency inventory is incomplete')
    installed_observed = 0
    locally_revalidated = 0
    for name in apt_names:
        record = _mapping(apt_inventory[name], f'apt inventory record {name}')
        allowed_keys = {
            'audited_version',
            'copyright_package',
            'copyright_sha256',
            'declared_licenses',
        }
        _require(
            set(record) <= allowed_keys
            and {'audited_version', 'copyright_sha256', 'declared_licenses'} <= set(record),
            f'apt inventory record keys changed: {name}',
        )
        _require(
            isinstance(record['audited_version'], str) and record['audited_version'],
            f'apt inventory version is missing: {name}',
        )
        _require(
            isinstance(record['copyright_sha256'], str)
            and re.fullmatch(r'[0-9a-f]{64}', record['copyright_sha256']) is not None,
            f'apt copyright hash is invalid: {name}',
        )
        _nonempty_strings(record['declared_licenses'], f'apt licenses for {name}')
        copyright_package = record.get('copyright_package', name)
        _require(
            isinstance(copyright_package, str)
            and SAFE_COMPONENT.fullmatch(copyright_package) is not None,
            f'apt copyright package is invalid: {name}',
        )
        completed = subprocess.run(
            ['dpkg-query', '-W', '-f=${Version}', name],
            capture_output=True,
            check=False,
            text=True,
            timeout=10,
        )
        if completed.returncode != 0:
            continue
        installed_observed += 1
        if completed.stdout != record['audited_version']:
            continue
        copyright_path = Path('/usr/share/doc') / copyright_package / 'copyright'
        _require(copyright_path.is_file(), f'audited apt copyright file is missing: {name}')
        _require(
            file_sha256(copyright_path) == record['copyright_sha256'],
            f'audited apt copyright file changed: {name}',
        )
        locally_revalidated += 1

    first_party = set(packages)
    direct_dependencies = _package_dependency_surface(package_xmls, first_party)
    ros_inventory = _mapping(inventory['ros_dependencies'], 'ROS dependency inventory')
    system_inventory = _mapping(
        inventory['rosdep_system_dependencies'], 'rosdep system dependency inventory'
    )
    _require(
        set(ros_inventory).isdisjoint(system_inventory),
        'ROS and system dependency inventories overlap',
    )
    _require(
        set(ros_inventory) | set(system_inventory) == direct_dependencies,
        'package.xml dependency inventory is incomplete',
    )
    _require(
        all(value in apt_inventory for value in system_inventory.values()),
        'rosdep system dependency is absent from the apt inventory',
    )
    ros_locally_revalidated = 0
    for name in sorted(ros_inventory):
        record = _mapping(ros_inventory[name], f'ROS dependency record {name}')
        _require(
            set(record) == {'audited_version', 'declared_licenses'},
            f'ROS dependency record keys changed: {name}',
        )
        _require(
            isinstance(record['audited_version'], str) and record['audited_version'],
            f'ROS dependency version is missing: {name}',
        )
        expected_licenses = _nonempty_strings(
            record['declared_licenses'], f'ROS dependency licenses for {name}'
        )
        installed_manifest = Path('/opt/ros/jazzy/share') / name / 'package.xml'
        if not installed_manifest.is_file():
            continue
        installed_package = ET.parse(installed_manifest).getroot()
        installed_licenses = sorted(
            item.text.strip()
            for item in installed_package.findall('license')
            if item.text and item.text.strip()
        )
        _require(
            installed_licenses == expected_licenses,
            f'installed ROS license declarations changed: {name}',
        )
        ros_locally_revalidated += 1

    python_inventory = _mapping(inventory['python_distributions'], 'Python dependency inventory')
    _require(set(python_inventory) == {'ruff'}, 'Python dependency inventory changed')
    ruff_record = _mapping(python_inventory['ruff'], 'Ruff inventory record')
    _require(
        set(ruff_record) == {'declared_license', 'source', 'version'}
        and ruff_record['declared_license'] == 'MIT'
        and ruff_record['version'] == '0.16.4',
        'Ruff dependency inventory changed',
    )
    _require(
        pyproject.get('project', {}).get('optional-dependencies', {}).get('dev')
        == [f'ruff=={ruff_record["version"]}'],
        'Python dependency declarations do not match the inventory',
    )

    action_inventory = _mapping(inventory['github_actions'], 'GitHub Action inventory')
    _require(set(action_inventory) == set(ACTION_PINS), 'GitHub Action inventory changed')
    for name, expected_revision in ACTION_PINS.items():
        record = _mapping(action_inventory[name], f'GitHub Action record {name}')
        _require(
            set(record) == {'declared_license', 'revision', 'source'},
            f'GitHub Action inventory record changed: {name}',
        )
        _require(record['revision'] == expected_revision, f'GitHub Action revision changed: {name}')
        _require(
            isinstance(record['declared_license'], str) and record['declared_license'],
            f'GitHub Action license is missing: {name}',
        )
        _require(
            record['source'] == f'https://github.com/{name}',
            f'GitHub Action source is invalid: {name}',
        )

    _require(inventory['go_modules'] == [], 'direct Go dependency inventory changed')
    go_mod = _repository_regular_file(repository, 'supervisor/go.mod', 'Go module manifest')
    _require(
        re.search(r'^\s*require(?:\s|\()', go_mod.read_text(encoding='utf-8'), re.MULTILINE)
        is None,
        'go.mod has an unrecorded direct dependency',
    )
    return {
        'dependency_inventory': {
            'apt_package_count': len(apt_inventory),
            'github_action_count': len(action_inventory),
            'go_module_count': 0,
            'installed_apt_observed_count': installed_observed,
            'locally_revalidated_apt_count': locally_revalidated,
            'locally_revalidated_ros_count': ros_locally_revalidated,
            'path': inventory_path.relative_to(repository).as_posix(),
            'python_distribution_count': len(python_inventory),
            'ros_dependency_count': len(ros_inventory),
            'rosdep_system_dependency_count': len(system_inventory),
            'sha256': file_sha256(inventory_path),
        },
        'license': 'Apache-2.0',
        'notice_sha256': file_sha256(notice_path),
        'package_count': len(packages),
        'packages': packages,
        'verification_scope': audit['scope'],
    }


def validate_release_claims(repository: Path) -> dict[str, object]:
    """Bind published README and portfolio claims to checked-in results."""
    repository = repository.resolve(strict=True)
    audit_path = _repository_regular_file(
        repository, 'config/release-claims.json', 'release claim audit'
    )
    audit = _mapping(json.loads(audit_path.read_text(encoding='utf-8')), 'release claim audit')
    _require(set(audit) == {'claims', 'schema_version', 'scope'}, 'claim audit keys changed')
    _require(audit['schema_version'] == 1, 'claim audit schema changed')
    _require(
        isinstance(audit['scope'], str)
        and 'README.md' in audit['scope']
        and 'docs/portfolio.md' in audit['scope'],
        'claim audit scope is missing',
    )
    claims = audit['claims']
    _require(isinstance(claims, list) and claims, 'claim audit is empty')
    ids: list[str] = []
    evidence_files: set[Path] = set()
    for index, item in enumerate(claims):
        claim = _mapping(item, f'claim audit record {index}')
        _require(
            set(claim)
            == {'claim_document', 'claim_text', 'evidence_document', 'evidence_text', 'id'},
            f'claim audit record keys changed: {index}',
        )
        for key in claim:
            _require(
                isinstance(claim[key], str) and claim[key],
                f'claim audit record {index} has an empty {key}',
            )
        _require(SAFE_COMPONENT.fullmatch(claim['id']) is not None, 'claim ID is invalid')
        claim_document = _repository_regular_file(
            repository, claim['claim_document'], f'claim document {claim["id"]}'
        )
        evidence_document = _repository_regular_file(
            repository, claim['evidence_document'], f'claim evidence {claim["id"]}'
        )
        published_text = ' '.join(claim_document.read_text(encoding='utf-8').split())
        evidence_text = ' '.join(evidence_document.read_text(encoding='utf-8').split())
        _require(
            ' '.join(claim['claim_text'].split()) in published_text,
            f'published claim text is absent: {claim["id"]}',
        )
        _require(
            ' '.join(claim['evidence_text'].split()) in evidence_text,
            f'claim evidence text is absent: {claim["id"]}',
        )
        ids.append(claim['id'])
        evidence_files.add(evidence_document)
    _require(len(ids) == len(set(ids)), 'claim audit IDs are not unique')
    base_ids = {
        'phase1-checksum-count',
        'phase1-displacement',
        'phase1-peak-rss',
        'phase1-rtf-median',
        'phase1-rtf-p5',
        'phase1-test-count',
        'phase2-checksum-count',
        'phase2-mission-result',
        'phase2-peak-rss',
        'phase2-rtf-median',
        'phase2-rtf-p5',
        'phase2-test-count',
    }
    portfolio_contracts = {
        'portfolio-phase1-checksum-count': {
            'claim_text': 'all 45 Phase 1 manifest entries',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': 'all 45 manifest entries validated',
        },
        'portfolio-phase1-displacement': {
            'claim_text': '`0.2962 m` of bounded displacement',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': '`0.29619887895167835 m`',
        },
        'portfolio-phase1-peak-rss': {
            'claim_text': 'peak launch-group RSS to `795,476 KiB`',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': '`795,476 KiB` (`776.83 MiB`)',
        },
        'portfolio-phase1-rtf-median': {
            'claim_text': 'calculated real-time factor median `0.9999`',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': '`0.9999416661302953`',
        },
        'portfolio-phase1-rtf-p5': {
            'claim_text': 'p5 `0.9162`',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': '`0.9162105536383762`',
        },
        'portfolio-phase1-test-count': {
            'claim_text': 'Phase 1 development run completed 103 tests with 0 errors or failures',
            'evidence_document': 'docs/results/phase-1/20260825T200725Z-1333.md',
            'evidence_text': '103 tests, 0 errors, 0 failures',
        },
        'portfolio-phase2-checksum-count': {
            'claim_text': 'all 111 Phase 2 manifest entries',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': 'all 111 checksum entries validated',
        },
        'portfolio-phase2-mission-result': {
            'claim_text': 'returned `SUCCEEDED` for 3/3 ordered waypoints',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': 'Nav2 `SUCCEEDED`; 3/3 completed',
        },
        'portfolio-phase2-peak-rss': {
            'claim_text': 'peak owned-process-group RSS to `1,372,244 KiB`',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': '`1,372,244 KiB`',
        },
        'portfolio-phase2-rtf-median': {
            'claim_text': 'calculated real-time factor median `0.9863`',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': '`0.9863029171557808`',
        },
        'portfolio-phase2-rtf-p5': {
            'claim_text': 'p5 `0.9302`',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': '`0.9302367113894893`',
        },
        'portfolio-phase2-test-count': {
            'claim_text': 'completed 213 tests with 0 errors or failures',
            'evidence_document': 'docs/results/phase-2/20260826T010218Z-466.md',
            'evidence_text': '213 tests, 0 errors, 0 failures',
        },
    }
    final_contracts = {
        'phase3-final-acceptance': {
            'claim_document': 'README.md',
            'claim_text': 'Canonical Phase 3 campaign evidence',
            'evidence_pattern': r'docs/results/phase-3/[A-Za-z0-9][A-Za-z0-9_.-]{0,127}\.md',
            'evidence_text': 'Automated verdict: **PASS**',
        },
        'phase4-final-acceptance': {
            'claim_document': 'README.md',
            'claim_text': 'canonical Phase 4 Scenario 6 evidence',
            'evidence_pattern': r'docs/results/phase-4/phase4-[0-9]{8}T[0-9]{6}Z-[0-9]+\.md',
            'evidence_text': 'Scenario 6 verdict: **PASS**',
        },
        'phase5-candidate-public-ci': {
            'claim_document': 'README.md',
            'claim_text': 'the exact candidate public-CI proof',
            'evidence_pattern': r'docs/results/phase-5/remote-[0-9a-f]{40}\.json',
            'evidence_text': '"status":"PASS"',
        },
    }
    observed_ids = set(ids)
    base_ids |= set(portfolio_contracts)
    _require(
        observed_ids in (base_ids, base_ids | set(final_contracts)),
        'claim audit must contain the exact base set or exact final extension',
    )
    claims_by_id = {claim['id']: claim for claim in claims}
    for claim_id, contract in portfolio_contracts.items():
        claim = claims_by_id[claim_id]
        _require(
            claim['claim_document'] == 'docs/portfolio.md'
            and all(claim[key] == value for key, value in contract.items()),
            f'portfolio claim contract changed: {claim_id}',
        )
    if observed_ids != base_ids:
        for claim_id, contract in final_contracts.items():
            claim = claims_by_id[claim_id]
            _require(
                claim['claim_document'] == contract['claim_document']
                and claim['claim_text'] == contract['claim_text']
                and claim['evidence_text'] == contract['evidence_text']
                and re.fullmatch(contract['evidence_pattern'], claim['evidence_document'])
                is not None,
                f'final claim contract changed: {claim_id}',
            )
    return {
        'audit_path': audit_path.relative_to(repository).as_posix(),
        'audit_sha256': file_sha256(audit_path),
        'claim_count': len(claims),
        'evidence_file_count': len(evidence_files),
        'evidence_files': {
            path.relative_to(repository).as_posix(): file_sha256(path)
            for path in sorted(evidence_files)
        },
        'status': 'PASS',
        'verification_scope': audit['scope'],
    }


def validate_non_live_test_surface(repository: Path) -> dict[str, object]:
    """Reject package-test registrations that could start live acceptance paths."""
    source = repository / 'src'
    _require(source.is_dir() and not source.is_symlink(), f'missing regular source tree: {source}')
    candidates: list[Path] = []
    for path in sorted(source.rglob('*')):
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        registration_file = path.name in {'CMakeLists.txt', 'package.xml', 'setup.py'}
        test_source = bool({'test', 'tests'} & set(relative.parts)) and path.suffix in {
            '.cpp',
            '.hpp',
            '.py',
            '.sh',
        }
        if registration_file or test_source:
            candidates.append(path)
    _require(candidates, 'no ROS package test registration files were found')
    for path in candidates:
        _require(not path.is_symlink(), f'test registration input is a symlink: {path}')
        value = path.read_text(encoding='utf-8')
        for label, pattern in NON_LIVE_TEST_PATTERNS.items():
            _require(
                pattern.search(value) is None,
                f'{label} is forbidden in the CI package-test surface: {path}',
            )
    return {
        'forbidden_pattern_count': len(NON_LIVE_TEST_PATTERNS),
        'live_test_registration_found': False,
        'scanned_file_count': len(candidates),
        'verification_scope': (
            'L0 registration scan before colcon test; node-level unit tests may create an isolated '
            'ROS context, but no launch, Gazebo, systemd, campaign, or privileged path is '
            'registered'
        ),
    }


def select_successful_run(records: object, sha: str) -> dict[str, object]:
    """Select the newest completed successful canonical workflow for one exact SHA."""
    _require(GIT_SHA.fullmatch(sha) is not None, 'remote SHA must be 40 lowercase hex characters')
    _require(isinstance(records, list), 'GitHub run response must be a list')
    candidates: list[tuple[dict[str, Any], datetime]] = []
    for index, item in enumerate(records):
        record = _mapping(item, f'GitHub run {index}')
        if record.get('headSha') != sha:
            continue
        if record.get('workflowName') != 'RoboTest CI':
            continue
        if record.get('status') != 'completed' or record.get('conclusion') != 'success':
            continue
        url = record.get('url')
        _require(
            isinstance(url, str) and url.startswith('https://github.com/'),
            'successful run URL is not an HTTPS GitHub URL',
        )
        database_id = record.get('databaseId')
        _require(isinstance(database_id, int) and database_id > 0, 'successful run ID is invalid')
        created_at = record.get('createdAt')
        completed_at = record.get('updatedAt')
        _require(
            isinstance(created_at, str)
            and created_at.endswith('Z')
            and isinstance(completed_at, str)
            and completed_at.endswith('Z'),
            'successful run timestamps are missing or noncanonical',
        )
        try:
            created_timestamp = datetime.fromisoformat(created_at[:-1] + '+00:00')
            completed_timestamp = datetime.fromisoformat(completed_at[:-1] + '+00:00')
        except ValueError as exc:
            raise EvidenceError('successful run timestamp is invalid') from exc
        _require(
            created_timestamp.tzinfo is not None
            and created_timestamp.utcoffset() == UTC.utcoffset(created_timestamp)
            and completed_timestamp.tzinfo is not None
            and completed_timestamp.utcoffset() == UTC.utcoffset(completed_timestamp)
            and completed_timestamp >= created_timestamp,
            'successful run completion timestamp is invalid',
        )
        candidates.append((record, created_timestamp))
    _require(candidates, f'no successful completed RoboTest CI run exists for {sha}')
    selected = max(candidates, key=lambda item: (item[1], item[0]['databaseId']))[0]
    return {
        'conclusion': 'success',
        'completed_at': selected['updatedAt'],
        'created_at': selected['createdAt'],
        'head_sha': sha,
        'run_id': selected['databaseId'],
        'run_url': selected['url'],
        'status': 'completed',
        'workflow_name': 'RoboTest CI',
    }


def bounded_log(output: Path, metadata: Path, maximum_bytes: int) -> None:
    """Drain stdin while retaining a deterministic bounded prefix."""
    _require(maximum_bytes > 0, 'maximum log bytes must be positive')
    output.parent.mkdir(parents=True, exist_ok=True)
    total = 0
    retained = 0
    with output.open('wb') as destination:
        while chunk := os.read(0, 65536):
            total += len(chunk)
            remaining = maximum_bytes - retained
            if remaining > 0:
                kept = chunk[:remaining]
                destination.write(kept)
                retained += len(kept)
        destination.flush()
        os.fsync(destination.fileno())
    atomic_write_json(
        metadata,
        {
            'maximum_bytes': maximum_bytes,
            'retained_bytes': retained,
            'schema_version': 1,
            'total_bytes': total,
            'truncated': total > retained,
        },
    )


def write_local_summary(args: argparse.Namespace) -> None:
    with args.checks.open(encoding='utf-8', newline='') as source:
        checks = [
            {
                'exit_code': int(row['exit_code']),
                'log': row['log'],
                'name': row['name'],
                'status': row['status'],
            }
            for row in csv.DictReader(source, delimiter='\t')
        ]
    _require(args.status in {'PASS', 'FAIL'}, 'summary status is invalid')
    _require(GIT_SHA.fullmatch(args.git_sha) is not None, 'summary Git SHA is invalid')
    for check in checks:
        _require(check['status'] in {'passed', 'failed'}, 'check status is invalid')
        _require(
            (check['status'] == 'passed' and check['exit_code'] == 0)
            or (check['status'] == 'failed' and check['exit_code'] != 0),
            f'check status/exit mismatch: {check["name"]}',
        )
    if args.status == 'PASS':
        _require(
            checks and all(check['status'] == 'passed' for check in checks),
            'PASS has failed checks',
        )
    reports: dict[str, object] = {}
    for name, path in (
        ('workflow_contract', args.workflow_report),
        ('license_inventory', args.license_report),
        ('release_claims', args.claims_report),
        ('test_surface', args.test_surface_report),
        ('retention', args.retention_report),
        ('provenance', args.provenance_report),
        ('source_snapshot', args.source_snapshot),
    ):
        if path.is_file():
            reports[name] = json.loads(path.read_text(encoding='utf-8'))
        elif args.status == 'PASS':
            raise EvidenceError(f'PASS summary is missing {name}')
        else:
            reports[name] = {'status': 'unavailable'}
    checked_at = datetime.now(UTC).isoformat()
    passed_checks = sum(check['status'] == 'passed' for check in checks)
    failed_checks = sum(check['status'] == 'failed' for check in checks)
    verification_level = 'L2' if args.status == 'PASS' else 'L0-L2 attempted'
    runtime_scope = {
        'gazebo_started': False,
        'hardware_accessed': False,
        'phase1_or_phase2_live_verifier_started': False,
        'phase3_campaign_started': False,
        'phase4_apply_started': False,
        'systemd_mutated': False,
    }
    provenance = _mapping(reports['provenance'], 'provenance report')
    source = _mapping(reports['source_snapshot'], 'source snapshot')
    summary = {
        'checked_at': checked_at,
        'check_counts': {
            'failed': failed_checks,
            'passed': passed_checks,
            'total': len(checks),
        },
        'checks': checks,
        'ci_context': {
            'github_run_id': args.github_run_id or None,
            'github_sha': args.github_sha or None,
            'present': bool(args.github_run_id and args.github_sha),
        },
        'evidence': {
            'checksum_manifest': 'SHA256SUMS',
            'checksum_validation': 'checksum-validation.txt',
            'matching_csv': args.csv_output.name,
            'provenance': args.provenance_report.name,
            'source_snapshot': args.source_snapshot.name,
        },
        'license_inventory': reports['license_inventory'],
        'maximum_workers': args.maximum_workers,
        'mode': args.mode,
        'provenance': {
            'command': provenance.get('command'),
            'path': args.provenance_report.name,
            'platform': provenance.get('platform'),
            'sha256': (
                file_sha256(args.provenance_report) if args.provenance_report.is_file() else None
            ),
            'source': provenance.get('source'),
            'tool_distributions': provenance.get('tool_distributions'),
            'tools': provenance.get('tools'),
        },
        'retention': reports['retention'],
        'release_claims': reports['release_claims'],
        'runtime_scope': runtime_scope,
        'schema_version': 1,
        'source': {'git_dirty': args.git_dirty == 'true', 'git_sha': args.git_sha},
        'status': args.status,
        'test_surface': reports['test_surface'],
        'verification_level': verification_level,
        'verification_scope': (
            'Static analysis, pure unit tests, fresh ROS build/package tests, and CLI help smoke; '
            'remote CI, Gazebo, hardware, and privileged acceptance are excluded'
        ),
        'workflow_contract': reports['workflow_contract'],
    }
    atomic_write_json(args.output, summary)
    row = {
        'checked_at': checked_at,
        'checks_failed': failed_checks,
        'checks_passed': passed_checks,
        'checks_total': len(checks),
        'ci_context_present': str(bool(args.github_run_id and args.github_sha)).lower(),
        'git_dirty': str(args.git_dirty == 'true').lower(),
        'git_sha': args.git_sha,
        'github_run_id': args.github_run_id or '',
        'github_sha': args.github_sha or '',
        'hardware_accessed': str(runtime_scope['hardware_accessed']).lower(),
        'maximum_workers': args.maximum_workers,
        'mode': args.mode,
        'phase3_campaign_started': str(runtime_scope['phase3_campaign_started']).lower(),
        'phase4_apply_started': str(runtime_scope['phase4_apply_started']).lower(),
        'provenance_sha256': summary['provenance']['sha256'],
        'remote_ci_verified': 'false',
        'source_aggregate_sha256': source.get('aggregate_sha256'),
        'status': args.status,
        'systemd_mutated': str(runtime_scope['systemd_mutated']).lower(),
        'verification_level': verification_level,
    }
    stream = io.StringIO(newline='')
    writer = csv.DictWriter(stream, fieldnames=list(row), lineterminator='\n')
    writer.writeheader()
    writer.writerow(row)
    atomic_write_bytes(args.csv_output, stream.getvalue().encode())
    with args.csv_output.open(encoding='utf-8', newline='') as source_stream:
        persisted_rows = list(csv.DictReader(source_stream))
    _require(len(persisted_rows) == 1, 'summary CSV must contain exactly one data row')
    persisted = persisted_rows[0]
    _require(persisted.get('status') == summary['status'], 'summary CSV status mismatch')
    _require(persisted.get('git_sha') == summary['source']['git_sha'], 'summary CSV SHA mismatch')
    _require(
        persisted.get('source_aggregate_sha256') == source.get('aggregate_sha256'),
        'summary CSV source provenance mismatch',
    )


def write_remote_summary(args: argparse.Namespace) -> None:
    records = json.loads(args.runs_json.read_text(encoding='utf-8'))
    run = select_successful_run(records, args.sha)
    _require(args.remote_sha == args.sha, 'GitHub commit resolution does not match requested SHA')
    _require(
        args.local_resolved_sha == args.sha, 'local commit resolution does not match requested SHA'
    )
    _require(args.visibility == 'PUBLIC', 'remote repository is not public')
    _require(
        GITHUB_REPOSITORY.fullmatch(args.repository) is not None,
        'GitHub repository name is invalid',
    )
    expected_repository_url = f'https://github.com/{args.repository}'
    _require(
        args.repository_url == expected_repository_url,
        'repository URL does not match the GitHub repository name',
    )
    _require(
        run['run_url'] == f'{expected_repository_url}/actions/runs/{run["run_id"]}',
        'workflow run URL does not match the repository and run ID',
    )
    command = list(args.command_argument)
    _require(command, 'exact remote verifier command is missing')
    repository_path = args.repository_path.resolve(strict=True)
    atomic_write_json(
        args.output,
        {
            'checked_at': datetime.now(UTC).isoformat(),
            'repository': {
                'name_with_owner': args.repository,
                'url': args.repository_url,
                'visibility': args.visibility,
            },
            'run': run,
            'schema_version': 1,
            'status': 'PASS',
            'provenance': {
                'command': {'argv': command, 'cwd': str(args.cwd.resolve(strict=True))},
                'local_resolved_sha': args.local_resolved_sha,
                'platform': _platform_provenance(),
                'tools': {
                    'gh': _command_version(['gh', '--version'], repository_path),
                    'git': _command_version(['git', '--version'], repository_path),
                    'sha256sum': _command_version(['sha256sum', '--version'], repository_path),
                },
            },
            'verification_scope': 'Read-only GitHub verification for one exact pushed commit SHA',
        },
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)

    workflow = subparsers.add_parser('workflow')
    workflow.add_argument('--workflow', type=Path, required=True)
    workflow.add_argument('--output', type=Path, required=True)

    licenses = subparsers.add_parser('licenses')
    licenses.add_argument('--repository', type=Path, required=True)
    licenses.add_argument('--output', type=Path, required=True)

    claims = subparsers.add_parser('claims')
    claims.add_argument('--repository', type=Path, required=True)
    claims.add_argument('--output', type=Path, required=True)

    test_surface = subparsers.add_parser('test-surface')
    test_surface.add_argument('--repository', type=Path, required=True)
    test_surface.add_argument('--output', type=Path, required=True)

    snapshot = subparsers.add_parser('source-snapshot')
    snapshot.add_argument('--repository', type=Path, required=True)
    snapshot.add_argument('--output', type=Path, required=True)

    retention = subparsers.add_parser('retention')
    retention.add_argument('--evidence-root', type=Path, required=True)
    retention.add_argument('--current-run', type=Path, required=True)
    retention.add_argument('--maximum-prior', type=int, required=True)
    retention.add_argument('--output', type=Path, required=True)

    checksums = subparsers.add_parser('checksums')
    checksums.add_argument('--run-directory', type=Path, required=True)

    checksum_validate = subparsers.add_parser('checksum-validate')
    checksum_validate.add_argument('--run-directory', type=Path, required=True)

    file_checksum = subparsers.add_parser('file-checksum')
    file_checksum.add_argument('--file', type=Path, required=True)
    file_checksum.add_argument('--manifest', type=Path, required=True)

    file_checksum_validate = subparsers.add_parser('file-checksum-validate')
    file_checksum_validate.add_argument('--file', type=Path, required=True)
    file_checksum_validate.add_argument('--manifest', type=Path, required=True)

    log = subparsers.add_parser('bounded-log')
    log.add_argument('--output', type=Path, required=True)
    log.add_argument('--metadata', type=Path, required=True)
    log.add_argument('--maximum-bytes', type=int, required=True)

    summary = subparsers.add_parser('summary')
    summary.add_argument('--checks', type=Path, required=True)
    summary.add_argument('--claims-report', type=Path, required=True)
    summary.add_argument('--csv-output', type=Path, required=True)
    summary.add_argument('--git-dirty', choices=('true', 'false'), required=True)
    summary.add_argument('--git-sha', required=True)
    summary.add_argument('--github-run-id', default='')
    summary.add_argument('--github-sha', default='')
    summary.add_argument('--license-report', type=Path, required=True)
    summary.add_argument('--mode', choices=('local', 'ci'), required=True)
    summary.add_argument('--output', type=Path, required=True)
    summary.add_argument('--provenance-report', type=Path, required=True)
    summary.add_argument('--retention-report', type=Path, required=True)
    summary.add_argument('--source-snapshot', type=Path, required=True)
    summary.add_argument('--status', choices=('PASS', 'FAIL'), required=True)
    summary.add_argument('--test-surface-report', type=Path, required=True)
    summary.add_argument('--workflow-report', type=Path, required=True)
    summary.add_argument('--maximum-workers', type=int, required=True)

    provenance = subparsers.add_parser('provenance')
    provenance.add_argument('--command-argument', action='append', default=[])
    provenance.add_argument('--cwd', type=Path, required=True)
    provenance.add_argument('--git-dirty', choices=('true', 'false'), required=True)
    provenance.add_argument('--git-sha', required=True)
    provenance.add_argument('--git-status', type=Path, required=True)
    provenance.add_argument('--maximum-workers', type=int, required=True)
    provenance.add_argument('--mode', choices=('local', 'ci'), required=True)
    provenance.add_argument('--output', type=Path, required=True)
    provenance.add_argument('--repository', type=Path, required=True)
    provenance.add_argument('--ruff', type=Path, required=True)
    provenance.add_argument('--source-snapshot', type=Path, required=True)

    remote = subparsers.add_parser('remote')
    remote.add_argument('--command-argument', action='append', default=[])
    remote.add_argument('--cwd', type=Path, required=True)
    remote.add_argument('--local-resolved-sha', required=True)
    remote.add_argument('--output', type=Path, required=True)
    remote.add_argument('--remote-sha', required=True)
    remote.add_argument('--repository', required=True)
    remote.add_argument('--repository-path', type=Path, required=True)
    remote.add_argument('--repository-url', required=True)
    remote.add_argument('--runs-json', type=Path, required=True)
    remote.add_argument('--sha', required=True)
    remote.add_argument('--visibility', required=True)
    return parser


def main() -> int:
    args = _parser().parse_args()
    try:
        if args.command == 'workflow':
            atomic_write_json(args.output, validate_workflow(args.workflow))
        elif args.command == 'licenses':
            atomic_write_json(args.output, validate_license_declarations(args.repository))
        elif args.command == 'claims':
            atomic_write_json(args.output, validate_release_claims(args.repository))
        elif args.command == 'test-surface':
            atomic_write_json(args.output, validate_non_live_test_surface(args.repository))
        elif args.command == 'source-snapshot':
            write_source_snapshot(args.repository, args.output)
        elif args.command == 'retention':
            atomic_write_json(
                args.output,
                prune_local_runs(
                    args.evidence_root,
                    args.current_run,
                    maximum_prior=args.maximum_prior,
                ),
            )
        elif args.command == 'checksums':
            write_checksum_manifest(args.run_directory)
        elif args.command == 'checksum-validate':
            validate_checksum_manifest(args.run_directory)
        elif args.command == 'file-checksum':
            write_file_checksum_manifest(args.file, args.manifest)
        elif args.command == 'file-checksum-validate':
            validate_file_checksum_manifest(args.file, args.manifest)
        elif args.command == 'bounded-log':
            bounded_log(args.output, args.metadata, args.maximum_bytes)
        elif args.command == 'summary':
            write_local_summary(args)
        elif args.command == 'provenance':
            write_provenance(args)
        elif args.command == 'remote':
            write_remote_summary(args)
        else:  # pragma: no cover - argparse constrains this branch
            raise EvidenceError(f'unknown command: {args.command}')
    except (
        EvidenceError,
        KeyError,
        OSError,
        UnicodeError,
        importlib.metadata.PackageNotFoundError,
        json.JSONDecodeError,
        subprocess.SubprocessError,
        ET.ParseError,
    ) as error:
        raise SystemExit(str(error)) from error
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
