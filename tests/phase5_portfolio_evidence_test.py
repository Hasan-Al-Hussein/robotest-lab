#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Pure adversarial tests for the Phase 5 portfolio evidence contract."""

from __future__ import annotations

import ast
import json
import os
import shutil
import signal
import socket
import stat
import struct
import subprocess
import sys
import textwrap
import zlib
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

import pytest

TESTS = Path(__file__).resolve().parent
REPOSITORY = TESTS.parent
WRAPPER = REPOSITORY / 'scripts/capture_phase5_portfolio.sh'
sys.path.insert(0, str(TESTS))

import phase5_portfolio_evidence as portfolio_module  # noqa: E402
from phase5_ci import EvidenceError, canonical_json_bytes  # noqa: E402

CANDIDATE_SHA = 'a' * 40
ATTEMPT_ID = 'portfolio-a-001'


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding='utf-8')
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def _run_wrapper(
    tmp_path: Path,
    arguments: Sequence[str],
    *,
    fake_python_exit: int = 0,
) -> subprocess.CompletedProcess[str]:
    fake_bin = tmp_path / 'fake-bin'
    fake_bin.mkdir(exist_ok=True)
    _write_executable(
        fake_bin / 'python3',
        '#!/bin/sh\nprintf \'%s\\n\' "$@"\nexit "${ROBOTEST_FAKE_PYTHON_EXIT:-0}"\n',
    )
    environment = {
        **os.environ,
        'PATH': f'{fake_bin}{os.pathsep}{os.environ["PATH"]}',
        'ROBOTEST_FAKE_PYTHON_EXIT': str(fake_python_exit),
    }
    return subprocess.run(
        [str(WRAPPER), *arguments],
        check=False,
        capture_output=True,
        env=environment,
        text=True,
        timeout=10,
    )


def _common_wrapper_arguments(tmp_path: Path) -> list[str]:
    return [
        '--repository',
        str(tmp_path / 'repository with spaces'),
        '--candidate-sha',
        CANDIDATE_SHA,
        '--phase3-candidate-root',
        str(tmp_path / 'phase3 candidate with spaces'),
    ]


def _forwarded_arguments(result: subprocess.CompletedProcess[str]) -> list[str]:
    assert result.stderr == ''
    lines = result.stdout.splitlines()
    assert Path(lines[0]).name == 'phase5_portfolio_evidence.py'
    return lines[1:]


def test_projection_paths_are_exact_and_deterministically_ordered() -> None:
    assert portfolio_module.portfolio_projection_paths(CANDIDATE_SHA) == (
        f'docs/results/phase-5/portfolio-{CANDIDATE_SHA}.json',
        f'docs/results/phase-5/portfolio-{CANDIDATE_SHA}.SHA256SUMS',
        f'docs/results/phase-5/portfolio-{CANDIDATE_SHA}.validation.txt',
        f'docs/results/phase-5/architecture-{CANDIDATE_SHA}.svg',
        f'docs/results/phase-5/release-flow-{CANDIDATE_SHA}.svg',
        f'docs/results/phase-5/scenario5-localization-error-{CANDIDATE_SHA}.png',
    )


@pytest.mark.parametrize('candidate_sha', ['', 'A' * 40, 'a' * 39, '../' + 'a' * 37])
def test_projection_paths_reject_unsafe_or_noncanonical_candidate_sha(candidate_sha: str) -> None:
    with pytest.raises(EvidenceError, match='candidate'):
        portfolio_module.portfolio_projection_paths(candidate_sha)


def test_wrapper_maps_documentation_replay_and_preserves_literal_paths(tmp_path: Path) -> None:
    arguments = [
        '--documentation-replay',
        *_common_wrapper_arguments(tmp_path),
        '--attempt-id',
        ATTEMPT_ID,
        '--authorize-mutating-commands',
    ]
    result = _run_wrapper(tmp_path, arguments)
    assert result.returncode == 0
    assert _forwarded_arguments(result) == [
        'prepare',
        *_common_wrapper_arguments(tmp_path),
        '--attempt-id',
        ATTEMPT_ID,
        '--authorize-mutating-commands',
    ]


def test_wrapper_preserves_review_required_exit_three(tmp_path: Path) -> None:
    result = _run_wrapper(
        tmp_path,
        ['--documentation-replay', *_common_wrapper_arguments(tmp_path)],
        fake_python_exit=3,
    )
    assert result.returncode == 3
    assert _forwarded_arguments(result)[0] == 'prepare'


@pytest.mark.parametrize(
    ('mode_arguments', 'subcommand', 'forwarded'),
    [
        (
            [
                '--finalize',
                '--attempt-id',
                ATTEMPT_ID,
                '--architecture-svg',
                '/tmp/architecture.svg',
                '--release-flow-svg',
                '/tmp/release-flow.svg',
                '--visual-review',
                '/tmp/review.json',
            ],
            'finalize',
            [
                '--attempt-id',
                ATTEMPT_ID,
                '--architecture-svg',
                '/tmp/architecture.svg',
                '--release-flow-svg',
                '/tmp/release-flow.svg',
                '--visual-review',
                '/tmp/review.json',
            ],
        ),
        (
            [
                '--project',
                '--portfolio-root',
                '/tmp/portfolio',
                '--attempt-id',
                ATTEMPT_ID,
            ],
            'project',
            ['--portfolio-root', '/tmp/portfolio', '--attempt-id', ATTEMPT_ID],
        ),
        (
            [
                '--validate',
                '--portfolio-root',
                '/tmp/portfolio',
                '--portfolio-proof',
                '/tmp/portfolio-proof.json',
            ],
            'validate',
            [
                '--portfolio-root',
                '/tmp/portfolio',
                '--portfolio-proof',
                '/tmp/portfolio-proof.json',
            ],
        ),
    ],
)
def test_wrapper_maps_each_later_stage_exactly(
    tmp_path: Path,
    mode_arguments: list[str],
    subcommand: str,
    forwarded: list[str],
) -> None:
    result = _run_wrapper(tmp_path, [*mode_arguments, *_common_wrapper_arguments(tmp_path)])
    assert result.returncode == 0
    assert _forwarded_arguments(result) == [
        subcommand,
        *_common_wrapper_arguments(tmp_path),
        *forwarded,
    ]


@pytest.mark.parametrize(
    ('arguments', 'message'),
    [
        ([], 'Exactly one mode is required'),
        (['--documentation-replay', '--validate'], 'Exactly one mode is required'),
        (['--documentation-replay', '--repository'], '--repository requires a value'),
        (
            ['--documentation-replay', '--repository', '/tmp/a', '--repository', '/tmp/b'],
            'Duplicate --repository',
        ),
        (
            [
                '--documentation-replay',
                '--portfolio-root',
                '/tmp/portfolio',
                '--repository',
                '/tmp/repository',
                '--candidate-sha',
                CANDIDATE_SHA,
                '--phase3-candidate-root',
                '/tmp/phase3',
            ],
            'reserved for a later stage',
        ),
        (
            [
                '--validate',
                '--authorize-mutating-commands',
                '--repository',
                '/tmp/repository',
                '--candidate-sha',
                CANDIDATE_SHA,
                '--phase3-candidate-root',
                '/tmp/phase3',
                '--portfolio-root',
                '/tmp/portfolio',
                '--portfolio-proof',
                '/tmp/proof',
            ],
            'valid only with --documentation-replay',
        ),
    ],
)
def test_wrapper_rejects_ambiguous_or_cross_stage_arguments(
    tmp_path: Path, arguments: list[str], message: str
) -> None:
    result = _run_wrapper(tmp_path, arguments)
    assert result.returncode == 1
    assert message in result.stderr


def test_wrapper_help_is_side_effect_free() -> None:
    result = subprocess.run(
        [str(WRAPPER), '--help'],
        check=False,
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0
    assert '--documentation-replay' in result.stdout
    assert result.stdout.count('<YYYYMMDDTHHMMSSZ-serial>') == 3
    assert 'serial is 1-10 digits and its first digit is 1-9' in result.stdout
    assert 'exits 3' in result.stdout
    assert result.stderr == ''


def test_canonical_fixture_writer_rejects_nan() -> None:
    with pytest.raises(ValueError):
        canonical_json_bytes({'invalid': float('nan')})


def _write_canonical_json(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(canonical_json_bytes(document))


def _read_json(path: Path) -> dict[str, object]:
    document = json.loads(path.read_bytes())
    assert isinstance(document, dict)
    return document


def _git(repository: Path, *arguments: str) -> str:
    result = subprocess.run(
        ['git', '-C', str(repository), *arguments],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return result.stdout.strip()


def _commit_all(repository: Path, message: str) -> str:
    _git(repository, 'add', '--all')
    _git(
        repository,
        '-c',
        'user.name=RoboTest Fixture',
        '-c',
        'user.email=fixture@example.invalid',
        'commit',
        '-m',
        message,
    )
    return _git(repository, 'rev-parse', 'HEAD')


def _candidate_contract_fixture(tmp_path: Path) -> tuple[Path, str]:
    repository = tmp_path / 'candidate-repository'
    repository.mkdir()
    _git(repository, 'init', '--quiet')
    commands: list[dict[str, object]] = []
    readme_lines = ['# Synthetic candidate', '']
    script_paths = {
        word
        for executable in portfolio_module.REPLAY_EXECUTABLES.values()
        for word in executable.split()
        if word.startswith('scripts/')
    }
    (repository / 'scripts').mkdir()
    for relative in sorted(script_paths):
        exit_code = 3 if relative == 'scripts/verify_all.sh' else 0
        _write_executable(
            repository / relative,
            f"#!/bin/sh\nset -eu\nprintf 'synthetic {relative}\\n'\nexit {exit_code}\n",
        )
    (repository / '.gitignore').write_text(
        '/.venv/\n/artifacts/evidence/phase5/\n',
        encoding='utf-8',
    )
    tooling = repository / '.venv/bin/ruff'
    tooling.parent.mkdir(parents=True)
    shutil.copy2(REPOSITORY / '.venv/bin/ruff', tooling)
    for identifier in portfolio_module.REPLAY_ALLOWED_CHANGES:
        mutating = identifier in portfolio_module.REPLAY_MUTATING_IDS
        executable = portfolio_module.REPLAY_EXECUTABLES[identifier]
        body = f'cd {portfolio_module.DOCUMENTED_REPOSITORY}\n{executable}\n'
        readme_lines.extend(['```bash', *body.rstrip('\n').splitlines(), '```', ''])
        commands.append(
            {
                'allowed_tracked_changes': list(
                    portfolio_module.REPLAY_ALLOWED_CHANGES[identifier]
                ),
                'authorization': (
                    'mutating_requires_explicit_authorization' if mutating else 'read_only'
                ),
                'command': body,
                'expected_exit_code': 3 if identifier == 'aggregate-local' else 0,
                'id': identifier,
                'maximum_stderr_bytes': portfolio_module.MAX_REPLAY_LOG_BYTES,
                'maximum_stdout_bytes': portfolio_module.MAX_REPLAY_LOG_BYTES,
                'timeout_seconds': portfolio_module.REPLAY_TIMEOUTS[identifier],
            }
        )
    diagrams = []
    for identifier in portfolio_module.DIAGRAM_IDS:
        marker = portfolio_module.DIAGRAM_MARKERS[identifier]
        readme_lines.extend(
            [
                marker,
                '```mermaid',
                f'flowchart LR\n  source[{identifier}] --> proof[Evidence]',
                '```',
                '',
            ]
        )
        diagrams.append({'id': identifier, 'marker': marker, 'source_path': 'README.md'})
    (repository / 'README.md').write_text('\n'.join(readme_lines), encoding='utf-8')
    generated_paths = {
        relative
        for allowed in portfolio_module.REPLAY_ALLOWED_CHANGES.values()
        for relative in allowed
    }
    for relative in sorted(generated_paths):
        path = repository / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix == '.json':
            _write_canonical_json(path, {'schema_version': 1, 'status': 'PASS'})
        else:
            path.write_text(f'synthetic generated evidence: {relative}\n', encoding='utf-8')
    _write_canonical_json(
        repository / 'config/phase5-readme-replay.json',
        {
            'commands': commands,
            'diagrams': diagrams,
            'readme_path': 'README.md',
            'schema_version': 1,
        },
    )
    return repository, _commit_all(repository, 'candidate fixture')


def test_candidate_contract_accepts_exact_eleven_units_and_two_diagrams(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    report = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    assert report['candidate_git_sha'] == candidate_sha
    assert report['status'] == 'PASS'
    assert [item['ordinal'] for item in report['commands']] == list(range(1, 12))
    assert [item['id'] for item in report['diagrams']] == list(portfolio_module.DIAGRAM_IDS)


def test_workspace_readme_replay_matrix_is_canonical_and_exact() -> None:
    matrix = portfolio_module._load_canonical_json(
        REPOSITORY / 'config/phase5-readme-replay.json',
        'workspace README replay matrix',
    )
    report = portfolio_module._validate_matrix(
        matrix,
        portfolio_module._parse_readme_fences((REPOSITORY / 'README.md').read_bytes()),
    )
    assert [item['id'] for item in report['commands']] == list(
        portfolio_module.REPLAY_ALLOWED_CHANGES
    )
    assert [item['id'] for item in report['diagrams']] == list(portfolio_module.DIAGRAM_IDS)


def test_primary_candidate_state_rejects_info_exclude_only_ignore_boundaries(
    tmp_path: Path,
) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    exclude = repository / '.git/info/exclude'
    exclude.write_text(
        f'{exclude.read_text(encoding="utf-8")}\n/.venv/\n/artifacts/evidence/phase5/\n',
        encoding='utf-8',
    )
    (repository / '.gitignore').write_text(
        '# required boundaries moved outside candidate C\n',
        encoding='utf-8',
    )
    _git(repository, 'add', '.gitignore')
    _git(
        repository,
        '-c',
        'user.name=RoboTest Fixture',
        '-c',
        'user.email=fixture@example.invalid',
        'commit',
        '-m',
        'move ignore boundaries out of candidate',
    )
    candidate_sha = _git(repository, 'rev-parse', 'HEAD')
    with pytest.raises(EvidenceError, match=r'tracked root|\.gitignore|ignore boundary'):
        portfolio_module._primary_candidate_state(repository, candidate_sha)


def test_primary_candidate_state_rejects_assume_unchanged_concealment(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    _git(repository, 'update-index', '--assume-unchanged', 'README.md')
    readme = repository / 'README.md'
    readme.write_bytes(readme.read_bytes() + b'concealed mutation\n')
    assert _git(repository, 'status', '--porcelain=v1', '--untracked-files=all') == ''
    with pytest.raises(
        EvidenceError,
        match=r'index hidden|non-normal|tracked bytes differ|assume-unchanged',
    ):
        portfolio_module._primary_candidate_state(repository, candidate_sha)


def test_portfolio_root_rejects_parent_symlink_without_outside_creation(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    outside = tmp_path / 'outside-portfolio'
    outside.mkdir()
    link = repository / portfolio_module.PORTFOLIO_RAW_PREFIX
    link.parent.mkdir(parents=True, exist_ok=True)
    link.symlink_to(outside, target_is_directory=True)
    with pytest.raises(EvidenceError, match='symlink'):
        portfolio_module._portfolio_root(repository, candidate_sha, create=True)
    assert list(outside.iterdir()) == []


def test_write_once_rejects_parent_symlink_without_outside_creation(tmp_path: Path) -> None:
    outside = tmp_path / 'outside-write'
    outside.mkdir()
    parent = tmp_path / 'inside'
    parent.mkdir()
    (parent / 'alias').symlink_to(outside, target_is_directory=True)
    with pytest.raises(EvidenceError, match='symlink'):
        portfolio_module._write_once(parent / 'alias/escaped.txt', b'escape\n')
    assert list(outside.iterdir()) == []


def test_candidate_contract_reads_candidate_objects_not_dirty_worktree(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    (repository / 'README.md').write_text('dirty and invalid\n', encoding='utf-8')
    report = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    assert report['candidate_git_sha'] == candidate_sha
    assert len(report['commands']) == 11


def test_candidate_contract_rejects_ten_units_even_when_matrix_matches(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    matrix_path = repository / 'config/phase5-readme-replay.json'
    matrix = _read_json(matrix_path)
    assert isinstance(matrix['commands'], list)
    removed = matrix['commands'].pop()
    assert isinstance(removed, dict)
    command = removed['command']
    assert isinstance(command, str)
    readme = (repository / 'README.md').read_text(encoding='utf-8')
    (repository / 'README.md').write_text(
        readme.replace(f'```bash\n{command}```\n\n', '', 1),
        encoding='utf-8',
    )
    _write_canonical_json(matrix_path, matrix)
    candidate_sha = _commit_all(repository, 'remove replay unit')
    with pytest.raises(EvidenceError, match=r'ID set|order|11|count'):
        portfolio_module.validate_candidate_contract(repository, candidate_sha)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        ('command_drift', 'command drifted'),
        ('root_escape', 'documented-root'),
        ('command_reorder', 'command drifted'),
        ('diagram_reorder', 'diagram order'),
        ('diagram_marker', 'diagram marker'),
        ('extra_bash', 'command count'),
        ('extra_mermaid', 'Mermaid fence count'),
    ],
)
def test_candidate_contract_rejects_fence_matrix_or_marker_drift(
    tmp_path: Path, mutation: str, message: str
) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    readme_path = repository / 'README.md'
    matrix_path = repository / 'config/phase5-readme-replay.json'
    readme = readme_path.read_text(encoding='utf-8')
    matrix = _read_json(matrix_path)
    commands = matrix['commands']
    diagrams = matrix['diagrams']
    assert isinstance(commands, list) and isinstance(diagrams, list)
    if mutation == 'command_drift':
        readme = readme.replace(
            'scripts/setup_ros2_repository.sh --check',
            'scripts/setup_ros2_repository.sh --altered',
            1,
        )
    elif mutation == 'root_escape':
        readme = readme.replace(
            f'cd {portfolio_module.DOCUMENTED_REPOSITORY}',
            'cd /tmp/escaped',
            1,
        )
        assert isinstance(commands[0], dict)
        commands[0]['command'] = str(commands[0]['command']).replace(
            portfolio_module.DOCUMENTED_REPOSITORY, '/tmp/escaped', 1
        )
    elif mutation == 'command_reorder':
        commands[0], commands[1] = commands[1], commands[0]
    elif mutation == 'diagram_reorder':
        diagrams[0], diagrams[1] = diagrams[1], diagrams[0]
    elif mutation == 'diagram_marker':
        readme = readme.replace(
            portfolio_module.DIAGRAM_MARKERS['architecture'],
            '<!-- ROBOTEST_PORTFOLIO_DIAGRAM: altered -->',
            1,
        )
    elif mutation == 'extra_bash':
        readme += "```bash\ncd /home/hasan/robotest-lab\nprintf 'extra\\n'\n```\n"
    elif mutation == 'extra_mermaid':
        readme += '```mermaid\nflowchart LR\n  extra --> fence\n```\n'
    else:  # pragma: no cover - parametrization is exhaustive
        raise AssertionError(mutation)
    readme_path.write_text(readme, encoding='utf-8')
    _write_canonical_json(matrix_path, matrix)
    candidate_sha = _commit_all(repository, f'apply {mutation}')
    with pytest.raises(EvidenceError, match=message):
        portfolio_module.validate_candidate_contract(repository, candidate_sha)


def test_candidate_contract_rejects_noncanonical_matrix_json(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    path = repository / 'config/phase5-readme-replay.json'
    path.write_text(json.dumps(_read_json(path), indent=2) + '\n', encoding='utf-8')
    candidate_sha = _commit_all(repository, 'make matrix noncanonical')
    with pytest.raises(EvidenceError, match='not canonical'):
        portfolio_module.validate_candidate_contract(repository, candidate_sha)


def test_candidate_contract_rejects_tracked_readme_symlink(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    readme = repository / 'README.md'
    target = repository / 'README-target.md'
    target.write_bytes(readme.read_bytes())
    readme.unlink()
    readme.symlink_to(target.name)
    candidate_sha = _commit_all(repository, 'symlink candidate README')
    with pytest.raises(EvidenceError, match='regular 100644 blob'):
        portfolio_module.validate_candidate_contract(repository, candidate_sha)


def _png_bytes(width: int = 960, height: int = 540) -> bytes:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack('>I', len(payload)) + kind + payload + struct.pack('>I', checksum)

    scanline = b'\x00' + b'\x00\x00\x00' * width
    pixels = scanline * height
    return (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(pixels, level=9))
        + chunk(b'IEND', b'')
    )


def _write_sidecar(path: Path) -> None:
    digest = portfolio_module.file_sha256(path)
    Path(f'{path}.sha256').write_text(f'{digest}  {path.name}\n', encoding='ascii')


def _phase3_media_fixture(tmp_path: Path, *, candidate_sha: str = CANDIDATE_SHA) -> Path:
    root = tmp_path / 'phase3-candidate'
    result_directory = root / 'runs/12/result'
    result_directory.mkdir(parents=True)
    candidate_id = 'phase3-aaaaaaaa-001'
    run_ids = [f'{candidate_id}-s{index // 3 + 1}-r{index % 3}-i{index:02d}' for index in range(15)]
    trials = []
    for index, run_id in enumerate(run_ids):
        scenario_id = index // 3 + 1
        trials.append(
            {
                'candidate_id': candidate_id,
                'repetition_index': index % 3,
                'run_id': run_id,
                'scenario_id': scenario_id,
                'scenario_name': (
                    portfolio_module.SCENARIO5_NAME
                    if scenario_id == 5
                    else f'scenario-{scenario_id}'
                ),
                'scenario_sha256': f'{scenario_id}' * 64,
                'suite_index': index,
            }
        )
    suite = {
        'candidate_id': candidate_id,
        'producer': 'robotest_phase3/benchmark_orchestrator',
        'schema_version': 1,
        'trials': trials,
    }
    suite_path = root / 'suite-plan.json'
    _write_canonical_json(suite_path, suite)
    _write_sidecar(suite_path)
    result = {
        'identity': {
            'candidate_id': candidate_id,
            'cold_stack': True,
            'git_dirty': False,
            'git_sha': candidate_sha,
            'repetition_index': 0,
            'run_id': run_ids[12],
            'scenario_id': 5,
            'scenario_index': 5,
            'scenario_name': portfolio_module.SCENARIO5_NAME,
            'scenario_sha256': '5' * 64,
            'suite_index': 12,
        },
        'verdict': {
            'automated_status': 'PASS',
            'capture_integrity': True,
            'components_complete': True,
            'exit_code': 0,
            'mission_success': True,
            'required_metric_checks': [{'passed': True}],
            'scenario_metric_gate': True,
            'threshold_checks': [{'passed': True}],
        },
    }
    result_path = result_directory / 'run-result.json'
    _write_canonical_json(result_path, result)
    result_sha = portfolio_module.file_sha256(result_path)
    for filename in sorted(portfolio_module.EXPECTED_RESULT_ARTIFACTS - {'run-result.json'}):
        path = result_directory / filename
        if path.suffix == '.png':
            path.write_bytes(_png_bytes())
        else:
            path.write_text(f'synthetic {filename}\n', encoding='utf-8')
    artifacts = []
    for filename in sorted(portfolio_module.EXPECTED_RESULT_ARTIFACTS):
        path = result_directory / filename
        artifacts.append(
            {
                'bytes': path.stat().st_size,
                'path': filename,
                'sha256': portfolio_module.file_sha256(path),
            }
        )
    total = sum(int(item['bytes']) for item in artifacts)
    manifest = {
        'artifacts': artifacts,
        'identity': {'run_id': run_ids[12], 'run_result_sha256': result_sha},
        'producer': 'robotest_metrics/metrics_analyze',
        'quality': {
            'artifact_bytes_excluding_manifest': total,
            'artifact_count': len(artifacts),
            'caps_within_limits': True,
            'hashes_verified': True,
            'path_set_complete': True,
        },
        'schema_version': 1,
    }
    manifest_path = result_directory / 'run-artifacts.manifest.json'
    _write_canonical_json(manifest_path, manifest)
    _write_sidecar(manifest_path)
    ordered_hashes = ['0' * 64 for _ in run_ids]
    ordered_hashes[12] = result_sha
    aggregate = {
        'identity': {
            'candidate_id': candidate_id,
            'git_sha': candidate_sha,
            'ordered_run_ids': run_ids,
            'ordered_source_json_sha256': ordered_hashes,
            'trial_count': 15,
        },
        'scenarios': {
            '5': {
                'denominator': 3,
                'numerator': 3,
                'run_ids': run_ids[12:15],
                'run_verdicts': [{'run_id': run_id, 'status': 'PASS'} for run_id in run_ids[12:15]],
                'success_rate': 1.0,
                'verdict': 'PASS',
            }
        },
        'verdict': {'automated_status': 'PASS'},
    }
    _write_canonical_json(root / 'aggregate/aggregate-result.json', aggregate)
    return root


def _refresh_manifest(root: Path) -> None:
    result_directory = root / 'runs/12/result'
    manifest_path = result_directory / 'run-artifacts.manifest.json'
    manifest = _read_json(manifest_path)
    artifacts = manifest['artifacts']
    assert isinstance(artifacts, list)
    total = 0
    for record in artifacts:
        assert isinstance(record, dict) and isinstance(record['path'], str)
        path = result_directory / record['path']
        record['bytes'] = path.stat().st_size
        record['sha256'] = portfolio_module.file_sha256(path)
        total += path.stat().st_size
    quality = manifest['quality']
    assert isinstance(quality, dict)
    quality['artifact_bytes_excluding_manifest'] = total
    _write_canonical_json(manifest_path, manifest)
    _write_sidecar(manifest_path)


def test_scenario5_media_accepts_exact_run_twelve_and_three_of_three(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    report = portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)
    assert report['suite_index'] == 12
    assert report['chart_dimensions'] == {'height': 540, 'width': 960}
    assert report['chart_relative_path'] == 'runs/12/result/localization-error.png'
    assert report['run_id'].endswith('-s5-r0-i12')


@pytest.mark.parametrize(
    ('document_name', 'mutation', 'message'),
    [
        ('suite-plan.json', ('trials', 12, 'scenario_id', 4), 'trial 12'),
        ('aggregate/aggregate-result.json', ('identity', 'trial_count', 14), 'aggregate'),
        (
            'aggregate/aggregate-result.json',
            ('scenarios', '5', 'numerator', 2),
            '3/3 PASS',
        ),
        ('runs/12/result/run-result.json', ('identity', 'suite_index', 13), 'result identity'),
        (
            'runs/12/result/run-result.json',
            ('verdict', 'automated_status', 'FAIL'),
            'canonical PASS',
        ),
    ],
)
def test_scenario5_media_rejects_suite_aggregate_or_result_mismatch(
    tmp_path: Path,
    document_name: str,
    mutation: tuple[object, ...],
    message: str,
) -> None:
    root = _phase3_media_fixture(tmp_path)
    path = root / document_name
    document = _read_json(path)
    target: object = document
    for key in mutation[:-2]:
        if isinstance(key, int):
            assert isinstance(target, list)
            target = target[key]
        else:
            assert isinstance(target, dict)
            target = target[key]
    field, value = mutation[-2:]
    assert isinstance(target, dict) and isinstance(field, str)
    target[field] = value
    _write_canonical_json(path, document)
    if document_name == 'suite-plan.json':
        _write_sidecar(path)
    elif document_name.endswith('run-result.json'):
        result_sha = portfolio_module.file_sha256(path)
        manifest_path = path.parent / 'run-artifacts.manifest.json'
        manifest = _read_json(manifest_path)
        assert isinstance(manifest['identity'], dict)
        manifest['identity']['run_result_sha256'] = result_sha
        assert isinstance(manifest['artifacts'], list)
        for record in manifest['artifacts']:
            if isinstance(record, dict) and record.get('path') == 'run-result.json':
                record['bytes'] = path.stat().st_size
                record['sha256'] = result_sha
        assert isinstance(manifest['quality'], dict)
        manifest['quality']['artifact_bytes_excluding_manifest'] = sum(
            int(record['bytes']) for record in manifest['artifacts'] if isinstance(record, dict)
        )
        _write_canonical_json(manifest_path, manifest)
        _write_sidecar(manifest_path)
        aggregate_path = root / 'aggregate/aggregate-result.json'
        aggregate = _read_json(aggregate_path)
        assert isinstance(aggregate['identity'], dict)
        hashes = aggregate['identity']['ordered_source_json_sha256']
        assert isinstance(hashes, list)
        hashes[12] = result_sha
        _write_canonical_json(aggregate_path, aggregate)
    with pytest.raises(EvidenceError, match=message):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


@pytest.mark.parametrize(
    ('payload', 'message'),
    [
        (_png_bytes() + b'trailing', r'trailing|truncated'),
        (_png_bytes(width=959), 'dimensions'),
        (b'not-a-png', 'signature'),
    ],
)
def test_scenario5_media_rejects_malformed_or_wrong_dimension_chart(
    tmp_path: Path, payload: bytes, message: str
) -> None:
    root = _phase3_media_fixture(tmp_path)
    chart = root / 'runs/12/result/localization-error.png'
    chart.write_bytes(payload)
    _refresh_manifest(root)
    with pytest.raises(EvidenceError, match=message):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def test_scenario5_media_rejects_png_crc_tamper(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    chart = root / 'runs/12/result/localization-error.png'
    payload = bytearray(chart.read_bytes())
    idat_data = payload.index(b'IDAT') + 4
    payload[idat_data] ^= 0x01
    chart.write_bytes(payload)
    _refresh_manifest(root)
    with pytest.raises(EvidenceError, match='CRC'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


@pytest.mark.parametrize('unsafe_path', ['../escape', '/tmp/escape', 'bad\npath'])
def test_scenario5_media_rejects_unsafe_manifest_paths(tmp_path: Path, unsafe_path: str) -> None:
    root = _phase3_media_fixture(tmp_path)
    manifest_path = root / 'runs/12/result/run-artifacts.manifest.json'
    manifest = _read_json(manifest_path)
    assert isinstance(manifest['artifacts'], list)
    assert isinstance(manifest['artifacts'][0], dict)
    manifest['artifacts'][0]['path'] = unsafe_path
    _write_canonical_json(manifest_path, manifest)
    _write_sidecar(manifest_path)
    with pytest.raises(EvidenceError, match=r'safe canonical|unsafe'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def test_scenario5_media_rejects_chart_hardlink(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    chart = root / 'runs/12/result/localization-error.png'
    second_name = tmp_path / 'second-chart-link.png'
    os.link(chart, second_name)
    with pytest.raises(EvidenceError, match='hard-linked'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def test_scenario5_media_rejects_chart_symlink(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    chart = root / 'runs/12/result/localization-error.png'
    target = tmp_path / 'external-chart.png'
    target.write_bytes(chart.read_bytes())
    chart.unlink()
    chart.symlink_to(target)
    with pytest.raises(EvidenceError, match=r'symlink|alias|escapes'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def test_scenario5_media_rejects_noncanonical_result_json(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    path = root / 'runs/12/result/run-result.json'
    path.write_text(json.dumps(_read_json(path), indent=2) + '\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='not canonical'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def test_scenario5_media_rejects_suite_sidecar_tamper(tmp_path: Path) -> None:
    root = _phase3_media_fixture(tmp_path)
    Path(f'{root / "suite-plan.json"}.sha256').write_text(
        f'{"0" * 64}  suite-plan.json\n', encoding='ascii'
    )
    with pytest.raises(EvidenceError, match='checksum sidecar mismatch'):
        portfolio_module.validate_scenario5_media(root, CANDIDATE_SHA)


def _write_svg(path: Path, body: str) -> None:
    path.write_text(
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">{body}</svg>\n',
        encoding='utf-8',
    )


def test_svg_accepts_safe_local_marker_url_reference(tmp_path: Path) -> None:
    path = tmp_path / 'safe.svg'
    _write_svg(
        path,
        '<defs><marker id="arrowhead" markerWidth="10" markerHeight="7">'
        '<path d="M 0 0 L 10 3.5 L 0 7 z"/></marker></defs>'
        '<path d="M 20 20 L 900 500" marker-end="url(#arrowhead)"/>',
    )
    assert portfolio_module._svg_dimensions(path, 'safe diagram') == (960.0, 540.0)


@pytest.mark.parametrize(
    'unsafe_body',
    [
        '<use href="http://example.invalid/image.svg#shape"/>',
        '<path style="fill:url(http://example.invalid/fill.svg)"/>',
        '<path style="fill:url(data:image/svg+xml;base64,AAAA)"/>',
        '<path style="fill:url(file:///tmp/fill.svg)"/>',
        '<path style="fill:url(non-fragment-reference)"/>',
    ],
)
def test_svg_rejects_external_data_file_http_or_nonfragment_url(
    tmp_path: Path, unsafe_body: str
) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(path, unsafe_body)
    with pytest.raises(EvidenceError, match=r'unsafe|external'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


@pytest.mark.parametrize(
    ('payload', 'message'),
    [
        (
            '<!DOCTYPE svg [<!ENTITY injected "unsafe">]>\n'
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
            '<text>&injected;</text></svg>\n',
            'document type or entity',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
            '<script>alert(1)</script></svg>\n',
            'script',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
            '<g onload="alert(1)"/></svg>\n',
            'event handler',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
            '<foreignObject><p>HTML</p></foreignObject></svg>\n',
            'foreignObject',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
            '<style>@import "https://example.invalid/evil.css";</style></svg>\n',
            'unsafe CSS',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 319 180">'
            '<path d="M0 0"/></svg>\n',
            'viewBox is outside',
        ),
        (
            '<svg xmlns="http://www.w3.org/2000/svg"><path d="M0 0"/></svg>\n',
            'lacks a viewBox',
        ),
    ],
)
def test_svg_rejects_active_content_unsafe_css_or_invalid_viewbox(
    tmp_path: Path,
    payload: str,
    message: str,
) -> None:
    path = tmp_path / 'unsafe.svg'
    path.write_text(payload, encoding='utf-8')
    with pytest.raises(EvidenceError, match=message):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


def test_svg_rejects_external_stylesheet_processing_instruction(tmp_path: Path) -> None:
    path = tmp_path / 'unsafe.svg'
    path.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<?xml-stylesheet type="text/css" href="https://example.invalid/hostile.css"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
        '<path d="M0 0"/></svg>\n',
        encoding='utf-8',
    )
    with pytest.raises(EvidenceError, match='processing instruction'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


def test_svg_rejects_unsafe_css_in_element_tail(tmp_path: Path) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(
        path,
        '<style><title/>@import url(https://example.invalid/hostile.css)</style>'
        '<rect width="960" height="540"/>',
    )
    with pytest.raises(EvidenceError, match='unsafe CSS'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


def test_svg_rejects_unsafe_css_split_across_style_children(tmp_path: Path) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(
        path,
        '<style>@im<title/>port "//example.invalid/hostile.css";</style>'
        '<rect width="960" height="540"/>',
    )
    with pytest.raises(EvidenceError, match='unsafe CSS'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


@pytest.mark.parametrize(
    'unsafe_body',
    [
        '<style>@im\\70ort "\\68ttps://example.invalid/hostile.css";</style>',
        '<path style="fill:u\\72l(\\68ttps://example.invalid/hostile.svg)"/>',
        '<style>@im&#x5c;70ort "&#92;68ttps://example.invalid/hostile.css";</style>',
        '<path style="fill:u&#92;72l(&#x5c;68ttps://example.invalid/hostile.svg)"/>',
    ],
)
def test_svg_rejects_obfuscated_css_escapes(tmp_path: Path, unsafe_body: str) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(path, unsafe_body)
    with pytest.raises(EvidenceError, match=r'CSS escape|unsafe CSS|unsafe URI'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


@pytest.mark.parametrize(
    ('element_name', 'unsafe_body'),
    [
        ('handler', '<handler type="application/ecmascript">alert(1)</handler>'),
        ('listener', '<listener event="load" handler="#safe"/>'),
        ('set', '<set attributeName="opacity" to="0"/>'),
        ('animate', '<animate attributeName="opacity" values="0;1"/>'),
        ('animateColor', '<animateColor attributeName="fill" from="#000" to="#fff"/>'),
        ('animateMotion', '<animateMotion path="M 0 0 L 10 10"/>'),
        (
            'animateTransform',
            '<animateTransform attributeName="transform" type="rotate" from="0" to="90"/>',
        ),
        ('discard', '<discard begin="1s"/>'),
        ('animation', '<animation href="#safe"/>'),
        ('prefetch', '<prefetch href="#safe"/>'),
    ],
)
def test_svg_rejects_event_and_smil_active_elements(
    tmp_path: Path, element_name: str, unsafe_body: str
) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(path, unsafe_body)
    with pytest.raises(EvidenceError, match=element_name):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


@pytest.mark.parametrize(
    'unsafe_body',
    [
        '<style>.node{mask-image:image("relative.svg")}</style>',
        '<path style="mask-image:image(\'relative.svg\')"/>',
        '<path style="mask-image:image-set(\'relative.svg\' 1x)"/>',
        '<path style="mask-image:-webkit-image-set(\'relative.svg\' 1x)"/>',
        '<path style="mask-image:-acme-image-set(\'relative.svg\' 1x)"/>',
        "<path style=\"mask-image:cross-fade('a.svg', 'b.svg')\"/>",
        '<path style="mask-image:-acme-image-rect(\'relative.svg\', 0, 0, 1, 1)"/>',
        '<path style="mask-image:im/**/age(\'relative.svg\')"/>',
    ],
)
def test_svg_rejects_relative_css_image_functions(tmp_path: Path, unsafe_body: str) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(path, unsafe_body)
    with pytest.raises(EvidenceError, match=r'unsafe CSS|unsafe URI'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


def test_svg_rejects_relative_xml_base(tmp_path: Path) -> None:
    path = tmp_path / 'unsafe.svg'
    _write_svg(path, '<g xml:base="../external/"><path d="M0 0"/></g>')
    with pytest.raises(EvidenceError, match='external base'):
        portfolio_module._svg_dimensions(path, 'unsafe diagram')


def test_bounded_runner_uses_only_explicit_environment(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setenv('ROS_DISTRO', 'must-not-leak')
    result = portfolio_module._run_bounded(
        ['/usr/bin/env'],
        cwd=tmp_path,
        environment={'PATH': '/usr/bin:/bin', 'ROBOTEST_SAFE': 'present'},
        timeout_seconds=5,
        stdout_limit=8192,
        stderr_limit=1024,
    )
    output = result['stdout'].decode('utf-8').splitlines()
    assert result['returncode'] == 0
    assert 'ROBOTEST_SAFE=present' in output
    assert not any(line.startswith('ROS_DISTRO=') for line in output)


def test_bounded_runner_fails_closed_on_stdout_overflow(tmp_path: Path) -> None:
    result = portfolio_module._run_bounded(
        ['/usr/bin/python3', '-c', 'import sys; sys.stdout.write("x" * 4096)'],
        cwd=tmp_path,
        environment={'PATH': '/usr/bin:/bin'},
        timeout_seconds=5,
        stdout_limit=1024,
        stderr_limit=1024,
    )
    assert result['stdout_overflow'] is True
    assert result['stdout_observed_bytes'] > 1024
    assert len(result['stdout']) == 1024


def test_bounded_runner_terminates_timed_out_process_group(tmp_path: Path) -> None:
    result = portfolio_module._run_bounded(
        ['/bin/sh', '-c', 'sleep 30 & wait'],
        cwd=tmp_path,
        environment={'PATH': '/usr/bin:/bin'},
        timeout_seconds=1,
        stdout_limit=1024,
        stderr_limit=1024,
    )
    assert result['timed_out'] is True
    assert result['returncode'] is not None


def _write_process_stat(
    proc_root: Path,
    pid: int,
    *,
    command: bytes = b'replay',
    state: str = 'S',
    parent_pid: int = 1,
    process_group_id: int | None = None,
    session_id: int | None = None,
    start_ticks: int = 100,
) -> None:
    process_group_id = pid if process_group_id is None else process_group_id
    session_id = pid if session_id is None else session_id
    fields = [
        state,
        str(parent_pid),
        str(process_group_id),
        str(session_id),
        *(['0'] * 15),
        str(start_ticks),
    ]
    process_root = proc_root / str(pid)
    process_root.mkdir(parents=True, exist_ok=True)
    process_root.joinpath('stat').write_bytes(
        str(pid).encode('ascii')
        + b' ('
        + command
        + b') '
        + ' '.join(fields).encode('ascii')
        + b'\n'
    )


def test_process_identity_parses_non_ascii_parenthesized_command(tmp_path: Path) -> None:
    _write_process_stat(tmp_path, 101, command=b'odd-\xff) name', start_ticks=999)
    identity = portfolio_module._read_process_identity(101, tmp_path)
    assert identity == portfolio_module._ProcessIdentity(
        pid=101,
        command='odd-\ufffd) name',
        state='S',
        parent_pid=1,
        process_group_id=101,
        session_id=101,
        start_ticks=999,
    )
    assert portfolio_module._pin_process_group(SimpleNamespace(pid=101), tmp_path) == identity


@pytest.mark.parametrize('state', sorted(portfolio_module.QUIESCENT_PROCESS_STATES))
def test_process_group_scan_treats_kernel_terminal_states_as_quiescent(
    tmp_path: Path, state: str
) -> None:
    _write_process_stat(tmp_path, 101, state='Z', start_ticks=999)
    _write_process_stat(
        tmp_path,
        102,
        state=state,
        parent_pid=101,
        process_group_id=101,
        session_id=101,
        start_ticks=1_000,
    )
    leader = portfolio_module._pin_process_group(SimpleNamespace(pid=101), tmp_path)
    assert portfolio_module._live_process_group_members(leader, tmp_path) == []

    _write_process_stat(
        tmp_path,
        102,
        state='S',
        parent_pid=101,
        process_group_id=101,
        session_id=101,
        start_ticks=1_000,
    )
    live_pids = [
        item.pid for item in portfolio_module._live_process_group_members(leader, tmp_path)
    ]
    assert live_pids == [102]


def test_process_group_scan_rejects_reused_leader_identity(tmp_path: Path) -> None:
    _write_process_stat(tmp_path, 101, state='Z', start_ticks=999)
    leader = portfolio_module._pin_process_group(SimpleNamespace(pid=101), tmp_path)
    _write_process_stat(tmp_path, 101, state='Z', start_ticks=1_000)
    with pytest.raises(EvidenceError, match='leader identity changed'):
        portfolio_module._process_group_members(leader, tmp_path)


def test_process_group_signal_revalidates_identity_before_numeric_pgid(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = portfolio_module._ProcessIdentity(101, 'leader', 'S', 1, 101, 101, 999)
    reused = leader._replace(start_ticks=1_000)
    signals: list[tuple[int, int]] = []
    monkeypatch.setattr(portfolio_module, '_read_process_identity', lambda _pid: reused)
    monkeypatch.setattr(
        portfolio_module.os,
        'killpg',
        lambda process_group_id, signum: signals.append((process_group_id, signum)),
    )
    with pytest.raises(EvidenceError, match='identity changed before signal'):
        portfolio_module._signal_process_group(leader, signal.SIGTERM)
    assert signals == []


def test_initial_empty_group_scan_rechecks_and_cleans_late_member(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = portfolio_module._ProcessIdentity(101, 'leader', 'Z', 1, 101, 101, 999)
    child = portfolio_module._ProcessIdentity(102, 'child', 'S', 1, 101, 101, 1_000)
    initial_scans = iter(([], [child]))
    events: list[str] = []

    def live_members(_leader: object) -> list[portfolio_module._ProcessIdentity]:
        events.append('initial-scan')
        return next(initial_scans)

    monkeypatch.setattr(portfolio_module, '_wait_for_leader_exit', lambda *_args: True)
    monkeypatch.setattr(portfolio_module, '_live_process_group_members', live_members)
    monkeypatch.setattr(portfolio_module.time, 'sleep', lambda _seconds: None)
    monkeypatch.setattr(
        portfolio_module,
        '_signal_process_group',
        lambda _leader, signum: events.append(f'signal-{signum}'),
    )
    monkeypatch.setattr(
        portfolio_module,
        '_wait_for_process_group_quiescence',
        lambda _leader, _deadline: events.append('post-signal-scan') or [],
    )

    def wait(timeout: float) -> int:
        events.append(f'reap-{int(timeout)}')
        return 0

    with pytest.raises(EvidenceError, match='survived normal command completion'):
        portfolio_module._terminate_process_group(
            SimpleNamespace(pid=101, wait=wait), leader, forced_cleanup=False
        )
    assert events == [
        'initial-scan',
        'initial-scan',
        f'signal-{signal.SIGTERM}',
        'post-signal-scan',
        f'signal-{signal.SIGKILL}',
        'post-signal-scan',
        'reap-5',
    ]


def test_forced_process_group_cleanup_orders_term_kill_then_reap(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = portfolio_module._ProcessIdentity(101, 'leader', 'S', 1, 101, 101, 999)
    child = portfolio_module._ProcessIdentity(102, 'child', 'S', 101, 101, 101, 1_000)
    events: list[tuple[str, int | None]] = []
    scans = iter(([child], []))

    def wait(timeout: float) -> int:
        events.append(('reap', int(timeout)))
        return -signal.SIGKILL

    monkeypatch.setattr(portfolio_module, '_live_process_group_members', lambda _leader: [child])
    monkeypatch.setattr(
        portfolio_module,
        '_signal_process_group',
        lambda _leader, signum: events.append(('signal', signum)),
    )

    def scan(_leader: object, _deadline: float) -> list[portfolio_module._ProcessIdentity]:
        events.append(('scan', None))
        return next(scans)

    monkeypatch.setattr(portfolio_module, '_wait_for_process_group_quiescence', scan)
    portfolio_module._terminate_process_group(
        SimpleNamespace(pid=101, wait=wait), leader, forced_cleanup=True
    )
    assert events == [
        ('signal', signal.SIGTERM),
        ('scan', None),
        ('signal', signal.SIGKILL),
        ('scan', None),
        ('reap', int(portfolio_module.PROCESS_GROUP_KILL_GRACE_S)),
    ]


def test_persistent_live_process_group_fails_after_kill_grace_before_best_effort_reap(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = portfolio_module._ProcessIdentity(101, 'leader', 'Z', 1, 101, 101, 999)
    child = portfolio_module._ProcessIdentity(102, 'child', 'D', 101, 101, 101, 1_000)
    events: list[tuple[str, int | None]] = []

    monkeypatch.setattr(portfolio_module, '_live_process_group_members', lambda _leader: [child])
    monkeypatch.setattr(
        portfolio_module,
        '_signal_process_group',
        lambda _leader, signum: events.append(('signal', signum)),
    )

    def scan(_leader: object, _deadline: float) -> list[portfolio_module._ProcessIdentity]:
        events.append(('scan', None))
        return [child]

    def wait(timeout: float) -> int:
        events.append(('reap', int(timeout)))
        return -signal.SIGKILL

    monkeypatch.setattr(portfolio_module, '_wait_for_process_group_quiescence', scan)
    with pytest.raises(EvidenceError, match='survived bounded SIGKILL cleanup'):
        portfolio_module._terminate_process_group(
            SimpleNamespace(pid=101, wait=wait), leader, forced_cleanup=True
        )
    assert events == [
        ('signal', signal.SIGTERM),
        ('scan', None),
        ('signal', signal.SIGKILL),
        ('scan', None),
        ('reap', 0),
    ]


def test_normal_exit_with_live_group_member_cleans_then_fails(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    leader = portfolio_module._ProcessIdentity(101, 'leader', 'Z', 1, 101, 101, 999)
    child = portfolio_module._ProcessIdentity(102, 'child', 'S', 1, 101, 101, 1_000)
    events: list[str] = []

    monkeypatch.setattr(
        portfolio_module,
        '_wait_for_leader_exit',
        lambda _leader, _deadline: events.append('leader-exit') or True,
    )
    monkeypatch.setattr(portfolio_module, '_live_process_group_members', lambda _leader: [child])
    monkeypatch.setattr(
        portfolio_module,
        '_signal_process_group',
        lambda _leader, signum: events.append(f'signal-{signum}'),
    )
    monkeypatch.setattr(
        portfolio_module,
        '_wait_for_process_group_quiescence',
        lambda _leader, _deadline: events.append('scan') or [],
    )

    def wait(timeout: float) -> int:
        events.append(f'reap-{int(timeout)}')
        return 0

    with pytest.raises(EvidenceError, match='survived normal command completion'):
        portfolio_module._terminate_process_group(
            SimpleNamespace(pid=101, wait=wait), leader, forced_cleanup=False
        )
    assert events == [
        'leader-exit',
        f'signal-{signal.SIGTERM}',
        'scan',
        f'signal-{signal.SIGKILL}',
        'scan',
        'reap-5',
    ]


def test_runner_cleans_process_when_selector_setup_fails(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    created: list[subprocess.Popen[bytes]] = []
    real_popen = subprocess.Popen

    def capture_popen(*args: object, **kwargs: object) -> subprocess.Popen[bytes]:
        process = real_popen(*args, **kwargs)
        created.append(process)
        return process

    class FailingSelector:
        def register(self, *_args: object) -> None:
            raise OSError('selector setup failed')

        def close(self) -> None:
            return None

    monkeypatch.setattr(portfolio_module.subprocess, 'Popen', capture_popen)
    monkeypatch.setattr(portfolio_module.selectors, 'DefaultSelector', FailingSelector)
    with pytest.raises(OSError, match='selector setup failed'):
        portfolio_module._run_bounded(
            ['/bin/sh', '-c', 'sleep 30'],
            cwd=tmp_path,
            environment={'PATH': '/usr/bin:/bin'},
            timeout_seconds=5,
            stdout_limit=1024,
            stderr_limit=1024,
        )
    assert len(created) == 1
    assert created[0].returncode is not None
    assert created[0].stdout is not None and created[0].stderr is not None
    created[0].stdout.close()
    created[0].stderr.close()


def test_runner_handles_adopted_zombies_and_term_ignoring_descendants(tmp_path: Path) -> None:
    probe_source = textwrap.dedent(
        r"""
        import ctypes
        import os
        import signal
        import sys
        import textwrap
        import time
        from pathlib import Path

        sys.path.insert(0, sys.argv[1])
        import phase5_portfolio_evidence as module

        libc = ctypes.CDLL(None, use_errno=True)
        assert libc.prctl(36, 1, 0, 0, 0) == 0, ctypes.get_errno()
        work = Path(sys.argv[2])

        def run(argv, timeout_seconds=1):
            return module._run_bounded(
                argv,
                cwd=work,
                environment={'PATH': '/usr/bin:/bin'},
                timeout_seconds=timeout_seconds,
                stdout_limit=1024,
                stderr_limit=1024,
            )

        def reap_adopted():
            deadline = time.monotonic() + 3
            while True:
                try:
                    pid, _ = os.waitpid(-1, os.WNOHANG)
                except ChildProcessError:
                    return
                if pid > 0:
                    continue
                assert time.monotonic() < deadline, 'adopted child was not quiescent'
                time.sleep(0.01)

        exact = run(['/bin/sh', '-c', 'sleep 30 & wait'])
        assert exact['timed_out'] is True
        assert exact['returncode'] is not None
        reap_adopted()

        ignore_term_source = textwrap.dedent(
            '''
            import os
            import signal
            import time

            read_fd, write_fd = os.pipe()
            child = os.fork()
            if child == 0:
                os.close(read_fd)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                os.write(write_fd, b'1')
                os.close(write_fd)
                time.sleep(30)
            else:
                os.close(write_fd)
                assert os.read(read_fd, 1) == b'1'
                os.close(read_fd)
                signal.signal(signal.SIGTERM, lambda *_args: os._exit(0))
                time.sleep(30)
            '''
        )
        sent = []
        original_signal_group = module._signal_process_group

        def record_signal_group(leader, signum):
            sent.append(signum)
            original_signal_group(leader, signum)

        module._signal_process_group = record_signal_group
        try:
            ignored = run(['/usr/bin/python3', '-c', ignore_term_source])
        finally:
            module._signal_process_group = original_signal_group
        assert ignored['timed_out'] is True
        assert sent == [signal.SIGTERM, signal.SIGKILL], sent
        reap_adopted()

        leaked_child_source = textwrap.dedent(
            '''
            import os
            import signal
            import time

            read_fd, write_fd = os.pipe()
            child = os.fork()
            if child == 0:
                os.close(read_fd)
                devnull = os.open('/dev/null', os.O_RDWR)
                os.dup2(devnull, 1)
                os.dup2(devnull, 2)
                if devnull > 2:
                    os.close(devnull)
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                os.write(write_fd, b'1')
                os.close(write_fd)
                time.sleep(30)
            else:
                os.close(write_fd)
                assert os.read(read_fd, 1) == b'1'
                os.close(read_fd)
            '''
        )
        try:
            run(['/usr/bin/python3', '-c', leaked_child_source], timeout_seconds=5)
        except module.EvidenceError as exc:
            assert 'survived normal command completion' in str(exc), str(exc)
        else:
            raise AssertionError('normal command leaked a live group member without failure')
        reap_adopted()
        """
    )
    probe = subprocess.run(
        ['/usr/bin/python3', '-c', probe_source, str(TESTS), str(tmp_path)],
        check=False,
        capture_output=True,
        env={'PATH': '/usr/bin:/bin', 'PYTHONDONTWRITEBYTECODE': '1'},
        text=True,
        timeout=20,
    )
    assert probe.returncode == 0, f'stdout:\n{probe.stdout}\nstderr:\n{probe.stderr}'


def test_replay_rejects_staged_tracked_mutation_outside_allowlist(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    script = repository / 'scripts/setup_ros2_repository.sh'
    _write_executable(
        script,
        "#!/bin/sh\nset -eu\nprintf 'staged mutation\\n' >>README.md\ngit add README.md\n",
    )
    candidate_sha = _commit_all(repository, 'staged mutation fixture')
    contract = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    command = contract['commands'][0]
    assert command['allowed_tracked_changes'] == list(
        portfolio_module.REPLAY_ALLOWED_CHANGES[command['id']]
    )
    with pytest.raises(
        EvidenceError,
        match=r'outside.*allowlist|delta did not match exactly|differ from C without filters',
    ):
        portfolio_module._replay_one_command(
            repository,
            candidate_sha,
            command,
            tmp_path / 'replay-output',
        )


def test_replay_rejects_clean_smudge_filter_attack_before_execution(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    target = repository / 'scripts/setup_ros2_repository.sh'
    candidate_payload = target.read_bytes()
    execution_marker = tmp_path / 'altered-command-executed'
    filter_marker = tmp_path / 'smudge-filter-executed'
    clean_filter = tmp_path / 'clean-filter.py'
    smudge_filter = tmp_path / 'smudge-filter.py'
    _write_executable(
        clean_filter,
        '#!/usr/bin/python3\n'
        'import sys\n'
        'sys.stdin.buffer.read()\n'
        f'sys.stdout.buffer.write({candidate_payload!r})\n',
    )
    altered_payload = (
        f"#!/bin/sh\nset -eu\nprintf 'executed\\n' >{execution_marker}\n"
        "printf 'smudged checkout\\n'\n"
    ).encode()
    _write_executable(
        smudge_filter,
        '#!/usr/bin/python3\n'
        'from pathlib import Path\n'
        'import sys\n'
        'sys.stdin.buffer.read()\n'
        f'Path({str(filter_marker)!r}).write_text("executed\\n", encoding="utf-8")\n'
        f'sys.stdout.buffer.write({altered_payload!r})\n',
    )
    _git(repository, 'config', 'filter.robotest-attack.clean', str(clean_filter))
    _git(repository, 'config', 'filter.robotest-attack.smudge', str(smudge_filter))
    _git(repository, 'config', 'filter.robotest-attack.required', 'true')
    (repository / '.gitattributes').write_text(
        'scripts/setup_ros2_repository.sh filter=robotest-attack\n',
        encoding='utf-8',
    )
    candidate_sha = _commit_all(repository, 'clean smudge attack fixture')
    contract = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    output = tmp_path / 'filter-attack-output'
    output.mkdir()
    with pytest.raises(EvidenceError, match=r'filter|byte|blob|candidate|worktree'):
        portfolio_module._replay_one_command(
            repository,
            candidate_sha,
            contract['commands'][0],
            output,
        )
    assert not filter_marker.exists()
    assert not execution_marker.exists()


def test_producer_has_one_reachable_replay_implementation() -> None:
    syntax = ast.parse(
        (TESTS / 'phase5_portfolio_evidence.py').read_text(encoding='utf-8'),
        filename='phase5_portfolio_evidence.py',
    )
    definitions = [
        node
        for node in syntax.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == '_replay_one_command'
    ]
    assert len(definitions) == 1
    function = definitions[0]
    assert function.body
    assert not isinstance(function.body[0], ast.Raise)


def _attempt_path(repository: Path, candidate_sha: str, attempt_id: str) -> Path:
    return (
        repository / portfolio_module.PORTFOLIO_RAW_PREFIX / candidate_sha / 'attempts' / attempt_id
    )


def _tree_hashes(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): portfolio_module.file_sha256(path)
        for path in sorted(root.rglob('*'))
        if path.is_file() and not path.is_symlink()
    }


def test_prepare_requires_explicit_mutating_command_authorization(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    phase3_root = _phase3_media_fixture(tmp_path, candidate_sha=candidate_sha)
    with pytest.raises(EvidenceError, match='requires explicit authorization'):
        portfolio_module.prepare_portfolio(
            repository,
            candidate_sha,
            phase3_root,
            attempt_id='20260828T120000Z-1',
        )
    attempt = _attempt_path(repository, candidate_sha, '20260828T120000Z-1')
    failure = _read_json(attempt / 'failure.json')
    assert failure['status'] == 'FAIL'
    assert failure['producer'] == 'robotest_phase5/portfolio_prepare'
    assert (attempt / 'primary-candidate-state.json').is_file()


def test_prepare_replays_all_units_in_fresh_candidate_worktrees(tmp_path: Path) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    phase3_root = _phase3_media_fixture(tmp_path, candidate_sha=candidate_sha)
    attempt_id = '20260828T120001Z-1'
    report = portfolio_module.prepare_portfolio(
        repository,
        candidate_sha,
        phase3_root,
        authorize_mutating_commands=True,
        attempt_id=attempt_id,
    )
    assert report['status'] == 'REVIEW_REQUIRED'
    assert report['candidate_git_sha'] == candidate_sha
    attempt = _attempt_path(repository, candidate_sha, attempt_id)
    replay = _read_json(attempt / 'documentation/replay-report.json')
    assert replay['status'] == 'PASS'
    assert replay['command_count'] == 11
    assert replay['fresh_detached_worktree_per_command'] is True
    assert replay['inherited_overlay'] is False
    commands = replay['commands']
    assert isinstance(commands, list)
    assert [item['id'] for item in commands] == list(portfolio_module.REPLAY_ALLOWED_CHANGES)
    assert all(
        item['status'] == 'PASS'
        and item['worktree_head'] == candidate_sha
        and item['failure_reasons'] == []
        for item in commands
        if isinstance(item, dict)
    )
    assert (attempt / 'render-request.json').is_file()
    assert not (attempt / 'raw-SHA256SUMS').exists()
    assert _git(repository, 'worktree', 'list', '--porcelain').count('worktree ') == 1


def test_replay_validation_rejects_checksum_rebound_boolean_wall_duration(
    tmp_path: Path,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    attempt = Path(fixture['attempt'])
    report_path = attempt / 'documentation/replay-report.json'
    report = _read_json(report_path)
    commands = report['commands']
    assert isinstance(commands, list) and isinstance(commands[0], dict)
    commands[0]['wall_duration_ns'] = False
    result_path = attempt / 'documentation/01-phase0-source-check/result.json'
    _write_canonical_json(result_path, commands[0])
    _write_canonical_json(report_path, report)
    prepare_path = attempt / 'prepare-report.json'
    prepare = _read_json(prepare_path)
    prepare['documentation_replay_sha256'] = portfolio_module.file_sha256(report_path)
    _write_canonical_json(prepare_path, prepare)
    with pytest.raises(EvidenceError, match=r'wall duration|exact|contract'):
        portfolio_module._validate_replay_logs(
            attempt,
            _read_json(attempt / 'candidate-contract.json'),
        )


def test_prepare_preserves_failed_attempt_and_refuses_overwrite(tmp_path: Path) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    _write_executable(
        repository / 'scripts/setup_ros2_repository.sh',
        "#!/bin/sh\nset -eu\nprintf 'synthetic failure\\n' >&2\nexit 9\n",
    )
    candidate_sha = _commit_all(repository, 'failing replay fixture')
    phase3_root = _phase3_media_fixture(tmp_path, candidate_sha=candidate_sha)
    attempt_id = '20260828T120002Z-1'
    arguments = (
        repository,
        candidate_sha,
        phase3_root,
    )
    with pytest.raises(EvidenceError, match='return code 9'):
        portfolio_module.prepare_portfolio(
            *arguments,
            authorize_mutating_commands=True,
            attempt_id=attempt_id,
        )
    attempt = _attempt_path(repository, candidate_sha, attempt_id)
    failure = _read_json(attempt / 'failure.json')
    assert failure['status'] == 'FAIL'
    first_result = _read_json(attempt / 'documentation/01-phase0-source-check/result.json')
    assert first_result['status'] == 'FAIL'
    assert first_result['returncode'] == 9
    before = _tree_hashes(attempt)
    with pytest.raises(EvidenceError, match='already exists'):
        portfolio_module.prepare_portfolio(
            *arguments,
            authorize_mutating_commands=True,
            attempt_id=attempt_id,
        )
    assert _tree_hashes(attempt) == before
    assert _git(repository, 'worktree', 'list', '--porcelain').count('worktree ') == 1


def _review_contract_fixture() -> tuple[
    dict[str, object], list[dict[str, object]], dict[str, object], datetime
]:
    source_hashes = {'architecture': '1' * 64, 'release-flow': '2' * 64}
    render_hashes = {'architecture': '3' * 64, 'release-flow': '4' * 64}
    candidate = {
        'diagrams': [
            {'id': identifier, 'source_sha256': source_hashes[identifier]}
            for identifier in portfolio_module.DIAGRAM_IDS
        ]
    }
    render_records = [
        {'diagram_id': identifier, 'render_sha256': render_hashes[identifier]}
        for identifier in portfolio_module.DIAGRAM_IDS
    ]
    review = {
        'attempt_id': '20260828T120006Z-1',
        'candidate_git_sha': CANDIDATE_SHA,
        'diagrams': [
            {
                'diagram_id': 'architecture',
                'render_sha256': render_hashes['architecture'],
                'rendered_utc': '2026-08-28T12:01:00Z',
                'renderer': {
                    'identity': 'Mermaid CLI',
                    'mode': 'argv',
                    'reference': ['mmdc', '--input', 'architecture.mmd'],
                    'version': '11.12.0',
                },
                'source_sha256': source_hashes['architecture'],
                'verdict': 'PASS',
            },
            {
                'diagram_id': 'release-flow',
                'render_sha256': render_hashes['release-flow'],
                'rendered_utc': '2026-08-28T12:01:30Z',
                'renderer': {
                    'identity': 'GitHub Mermaid renderer',
                    'mode': 'url',
                    'reference': (
                        f'https://github.com/example/robotest/blob/{CANDIDATE_SHA}/README.md'
                    ),
                    'version': 'github-markdown-2026-08-28',
                },
                'source_sha256': source_hashes['release-flow'],
                'verdict': 'PASS',
            },
        ],
        'reviewed_utc': '2026-08-28T12:02:00Z',
        'reviewer': 'Human Reviewer',
        'schema_version': 1,
    }
    prepared = datetime(2026, 8, 28, 12, 0, tzinfo=UTC)
    return candidate, render_records, review, prepared


def test_visual_review_accepts_hash_bound_argv_and_https_renderer_provenance() -> None:
    candidate, render_records, review, prepared = _review_contract_fixture()
    validated = portfolio_module._validate_visual_review(
        review,
        attempt_id='20260828T120006Z-1',
        candidate_sha=CANDIDATE_SHA,
        candidate=candidate,
        render_records=render_records,
        prepared_utc=prepared,
    )
    assert validated == review
    assert render_records[0]['renderer'] == review['diagrams'][0]['renderer']
    assert render_records[1]['rendered_utc'] == '2026-08-28T12:01:30Z'


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        ('stale_review', 'predates render request'),
        ('pre_rendered', 'chronology'),
        ('render_after_review', 'chronology'),
        ('render_hash', 'exact PASS bytes'),
        ('source_hash', 'exact PASS bytes'),
        ('argv_string', 'renderer argv'),
        ('http_url', 'renderer URL'),
        ('url_userinfo', 'renderer URL'),
        ('unrelated_https', 'renderer URL'),
        ('renderer_extra_key', 'renderer contract'),
        ('version_newline', 'renderer identity'),
        ('reviewer_bidi_control', 'reviewer is invalid'),
        ('identity_zero_width_control', 'renderer identity'),
        ('version_bidi_isolate', 'renderer identity'),
        ('argv_bidi_control', 'renderer argv'),
        ('automated_reviewer', 'reviewer is invalid'),
    ],
)
def test_visual_review_rejects_chronology_hash_or_renderer_provenance_drift(
    mutation: str, message: str
) -> None:
    candidate, render_records, review, prepared = _review_contract_fixture()
    diagrams = review['diagrams']
    assert isinstance(diagrams, list)
    first = diagrams[0]
    second = diagrams[1]
    assert isinstance(first, dict) and isinstance(second, dict)
    first_renderer = first['renderer']
    second_renderer = second['renderer']
    assert isinstance(first_renderer, dict) and isinstance(second_renderer, dict)
    if mutation == 'stale_review':
        review['reviewed_utc'] = '2026-08-28T11:59:59Z'
    elif mutation == 'pre_rendered':
        first['rendered_utc'] = '2026-08-28T11:59:59Z'
    elif mutation == 'render_after_review':
        first['rendered_utc'] = '2026-08-28T12:02:01Z'
    elif mutation == 'render_hash':
        first['render_sha256'] = '0' * 64
    elif mutation == 'source_hash':
        first['source_sha256'] = '0' * 64
    elif mutation == 'argv_string':
        first_renderer['reference'] = 'mmdc --input architecture.mmd'
    elif mutation == 'http_url':
        second_renderer['reference'] = 'http://example.invalid/render'
    elif mutation == 'url_userinfo':
        second_renderer['reference'] = 'https://user@example.invalid/render'
    elif mutation == 'unrelated_https':
        second_renderer['reference'] = 'https://example.invalid/unrelated-render'
    elif mutation == 'renderer_extra_key':
        first_renderer['unreviewed'] = True
    elif mutation == 'version_newline':
        first_renderer['version'] = '11.12.0\nforged'
    elif mutation == 'reviewer_bidi_control':
        review['reviewer'] = 'Human\u202eReviewer'
    elif mutation == 'identity_zero_width_control':
        first_renderer['identity'] = 'Mermaid\u200bCLI'
    elif mutation == 'version_bidi_isolate':
        first_renderer['version'] = '11.12\u2066.0'
    elif mutation == 'argv_bidi_control':
        first_renderer['reference'] = ['mmdc', '--input', 'architecture\u202e.mmd']
    elif mutation == 'automated_reviewer':
        review['reviewer'] = 'ci'
    else:  # pragma: no cover - parametrization is exhaustive
        raise AssertionError(mutation)
    with pytest.raises(EvidenceError, match=message):
        portfolio_module._validate_visual_review(
            review,
            attempt_id='20260828T120006Z-1',
            candidate_sha=CANDIDATE_SHA,
            candidate=candidate,
            render_records=render_records,
            prepared_utc=prepared,
        )


def _prepared_finalize_fixture(tmp_path: Path) -> dict[str, object]:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    phase3_root = _phase3_media_fixture(tmp_path, candidate_sha=candidate_sha)
    attempt_id = '20260828T120007Z-1'
    prepare = portfolio_module.prepare_portfolio(
        repository,
        candidate_sha,
        phase3_root,
        authorize_mutating_commands=True,
        attempt_id=attempt_id,
    )
    attempt = _attempt_path(repository, candidate_sha, attempt_id)
    architecture_svg = tmp_path / 'architecture.svg'
    release_flow_svg = tmp_path / 'release-flow.svg'
    _write_svg(
        architecture_svg,
        '<defs><marker id="arrowhead"><path d="M0 0 L10 5 L0 10 z"/></marker></defs>'
        '<path d="M20 20 L900 500" marker-end="url(#arrowhead)"/>',
    )
    _write_svg(release_flow_svg, '<path d="M20 500 L900 20"/><text>Release flow</text>')
    candidate = _read_json(attempt / 'candidate-contract.json')
    diagrams = candidate['diagrams']
    assert isinstance(diagrams, list)
    sources = {item['id']: item['source_sha256'] for item in diagrams if isinstance(item, dict)}
    review_path = tmp_path / 'visual-review.json'
    review = {
        'attempt_id': attempt_id,
        'candidate_git_sha': candidate_sha,
        'diagrams': [
            {
                'diagram_id': 'architecture',
                'render_sha256': portfolio_module.file_sha256(architecture_svg),
                'rendered_utc': prepare['prepared_utc'],
                'renderer': {
                    'identity': 'Mermaid CLI',
                    'mode': 'argv',
                    'reference': ['mmdc', '--input', 'architecture.mmd'],
                    'version': '11.12.0',
                },
                'source_sha256': sources['architecture'],
                'verdict': 'PASS',
            },
            {
                'diagram_id': 'release-flow',
                'render_sha256': portfolio_module.file_sha256(release_flow_svg),
                'rendered_utc': prepare['prepared_utc'],
                'renderer': {
                    'identity': 'GitHub Mermaid renderer',
                    'mode': 'url',
                    'reference': (
                        f'https://github.com/example/robotest/blob/{candidate_sha}/README.md'
                    ),
                    'version': 'github-markdown-2026-08-28',
                },
                'source_sha256': sources['release-flow'],
                'verdict': 'PASS',
            },
        ],
        'reviewed_utc': prepare['prepared_utc'],
        'reviewer': 'Human Reviewer',
        'schema_version': 1,
    }
    _write_canonical_json(review_path, review)
    return {
        'architecture_svg': architecture_svg,
        'attempt': attempt,
        'attempt_id': attempt_id,
        'candidate_sha': candidate_sha,
        'phase3_root': phase3_root,
        'release_flow_svg': release_flow_svg,
        'repository': repository,
        'review_path': review_path,
    }


def _finalize_fixture(fixture: dict[str, object]) -> dict[str, object]:
    return portfolio_module.finalize_portfolio(
        Path(fixture['repository']),
        str(fixture['candidate_sha']),
        Path(fixture['phase3_root']),
        str(fixture['attempt_id']),
        Path(fixture['architecture_svg']),
        Path(fixture['release_flow_svg']),
        Path(fixture['review_path']),
    )


def test_finalize_project_and_validate_exact_six_byte_identical_projections(
    tmp_path: Path,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    finalized = _finalize_fixture(fixture)
    assert finalized['status'] == 'PASS'
    assert finalized['prepared_utc'] <= finalized['finalized_utc']
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    attempt = Path(fixture['attempt'])
    portfolio_root = attempt.parents[1]
    projected = portfolio_module.project_portfolio(
        repository,
        portfolio_root,
        candidate_sha,
        Path(fixture['phase3_root']),
        str(fixture['attempt_id']),
    )
    expected_paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    assert set(projected) == {
        'attempt_id',
        'candidate_git_sha',
        'finalized_utc',
        'portfolio_proof_path',
        'portfolio_proof_sha256',
        'prepared_utc',
        'projection_paths',
        'scenario5_run_id',
        'status',
    }
    assert tuple(projected['projection_paths']) == expected_paths
    assert all((repository / relative).is_file() for relative in expected_paths)
    assert all(
        stat.S_IMODE((repository / relative).stat().st_mode) == 0o644 for relative in expected_paths
    )
    before_raw = _tree_hashes(attempt)
    before_tracked = {
        relative: portfolio_module.file_sha256(repository / relative) for relative in expected_paths
    }
    proof = repository / expected_paths[0]
    first = portfolio_module.validate_portfolio_evidence(
        repository,
        portfolio_root,
        proof,
        Path(fixture['phase3_root']),
        candidate_sha,
    )
    second = portfolio_module.validate_portfolio_evidence(
        repository,
        portfolio_root,
        proof,
        Path(fixture['phase3_root']),
        candidate_sha,
    )
    assert first == second
    assert set(first) == {
        'attempt_id',
        'candidate_git_sha',
        'finalized_utc',
        'portfolio_proof_path',
        'portfolio_proof_sha256',
        'prepared_utc',
        'projection_paths',
        'scenario5_run_id',
        'status',
        'verification_scope',
    }
    assert first['status'] == 'PASS'
    assert first['prepared_utc'] == projected['prepared_utc'] == finalized['prepared_utc']
    assert first['finalized_utc'] == projected['finalized_utc'] == finalized['finalized_utc']
    proof = _read_json(attempt / 'projection' / Path(expected_paths[0]).name)
    assert set(proof) == {
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
    assert proof['prepared_utc'] == first['prepared_utc']
    assert proof['finalized_utc'] == first['finalized_utc']
    prepared = portfolio_module._utc_timestamp(proof['prepared_utc'], 'prepared timestamp')
    finalized_at = portfolio_module._utc_timestamp(proof['finalized_utc'], 'finalized timestamp')
    review = _read_json(attempt / 'visual-review.json')
    reviewed = portfolio_module._utc_timestamp(review['reviewed_utc'], 'reviewed timestamp')
    rendered = [
        portfolio_module._utc_timestamp(item['rendered_utc'], 'rendered timestamp')
        for item in review['diagrams']
    ]
    assert all(prepared <= value <= reviewed <= finalized_at for value in rendered)
    assert _tree_hashes(attempt) == before_raw
    assert {
        relative: portfolio_module.file_sha256(repository / relative) for relative in expected_paths
    } == before_tracked


def test_finalize_refuses_review_hash_drift_and_preserves_stage_failure(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    review_path = Path(fixture['review_path'])
    review = _read_json(review_path)
    diagrams = review['diagrams']
    assert isinstance(diagrams, list) and isinstance(diagrams[0], dict)
    diagrams[0]['render_sha256'] = '0' * 64
    _write_canonical_json(review_path, review)
    with pytest.raises(EvidenceError, match='exact PASS bytes'):
        _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    failure = _read_json(attempt / 'failure.json')
    assert failure['status'] == 'FAIL'
    assert failure['producer'] == 'robotest_phase5/portfolio_finalize'
    before = _tree_hashes(attempt)
    with pytest.raises(EvidenceError, match=r'preserved failure|already used'):
        _finalize_fixture(fixture)
    assert _tree_hashes(attempt) == before


def test_finalize_preserves_selected_prepared_attempt_validation_failure(
    tmp_path: Path,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    attempt = Path(fixture['attempt'])
    request_path = attempt / 'render-request.json'
    request = _read_json(request_path)
    request['status'] = 'TAMPERED'
    _write_canonical_json(request_path, request)
    with pytest.raises(EvidenceError, match='render request drifted'):
        _finalize_fixture(fixture)
    failure = _read_json(attempt / 'failure.json')
    assert set(failure) == {
        'attempt_id',
        'candidate_git_sha',
        'error',
        'failed_utc',
        'producer',
        'schema_version',
        'stage',
        'status',
    }
    assert failure['attempt_id'] == fixture['attempt_id']
    assert failure['candidate_git_sha'] == fixture['candidate_sha']
    assert failure['producer'] == 'robotest_phase5/portfolio_finalize'
    assert failure['stage'] == 'portfolio_finalize'
    assert failure['status'] == 'FAIL'
    portfolio_module._utc_timestamp(failure['failed_utc'], 'failure timestamp')


def test_finalize_rejects_extra_preexisting_raw_file_even_when_it_can_be_checksummed(
    tmp_path: Path,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    attempt = Path(fixture['attempt'])
    (attempt / 'forged-preexisting.txt').write_text(
        'adversary-controlled but checksum-compatible\n',
        encoding='utf-8',
    )
    with pytest.raises(EvidenceError, match=r'file set|unexpected|allowlist'):
        _finalize_fixture(fixture)
    failure = _read_json(attempt / 'failure.json')
    assert failure['status'] == 'FAIL'
    assert failure['producer'] == 'robotest_phase5/portfolio_finalize'
    _rebind_raw_attempt(attempt)
    with pytest.raises(EvidenceError, match=r'file set|unexpected|allowlist'):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            str(fixture['candidate_sha']),
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


def test_validation_rejects_tracked_projection_tamper(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    attempt = Path(fixture['attempt'])
    portfolio_root = attempt.parents[1]
    portfolio_module.project_portfolio(
        repository,
        portfolio_root,
        candidate_sha,
        Path(fixture['phase3_root']),
        str(fixture['attempt_id']),
    )
    paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    tracked_svg = repository / paths[3]
    tracked_svg.write_bytes(tracked_svg.read_bytes().replace(b'</svg>', b'<g/></svg>'))
    with pytest.raises(EvidenceError, match='differs from raw'):
        portfolio_module.validate_portfolio_evidence(
            repository,
            portfolio_root,
            repository / paths[0],
            Path(fixture['phase3_root']),
            candidate_sha,
        )


def test_validation_rejects_tracked_projection_mode_drift(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    attempt = Path(fixture['attempt'])
    portfolio_root = attempt.parents[1]
    portfolio_module.project_portfolio(
        repository,
        portfolio_root,
        candidate_sha,
        Path(fixture['phase3_root']),
        str(fixture['attempt_id']),
    )
    tracked_svg = repository / portfolio_module.portfolio_projection_paths(candidate_sha)[3]
    tracked_svg.chmod(0o755)
    with pytest.raises(EvidenceError, match='mode changed'):
        portfolio_module.validate_portfolio_evidence(
            repository,
            portfolio_root,
            repository / portfolio_module.portfolio_projection_paths(candidate_sha)[0],
            Path(fixture['phase3_root']),
            candidate_sha,
        )


def test_worktree_state_rejects_failed_untracked_query(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    repository, candidate_sha = _candidate_contract_fixture(tmp_path)
    _git(repository, 'checkout', '--detach', '--quiet', candidate_sha)
    original_git = portfolio_module._git

    def failed_untracked_git(
        repository_path: Path,
        arguments: Sequence[str],
        **kwargs: object,
    ) -> subprocess.CompletedProcess[object]:
        if list(arguments[:2]) == ['ls-files', '--others']:
            binary = bool(kwargs.get('binary'))
            return subprocess.CompletedProcess(
                list(arguments),
                1,
                stdout=b'' if binary else '',
                stderr=b'injected failure' if binary else 'injected failure',
            )
        return original_git(repository_path, arguments, **kwargs)

    monkeypatch.setattr(portfolio_module, '_git', failed_untracked_git)
    with pytest.raises(EvidenceError, match='cannot inspect'):
        portfolio_module._worktree_state(repository, candidate_sha, 'injected worktree')


def _rebind_raw_attempt(attempt: Path) -> None:
    _write_canonical_json(
        attempt / 'raw-manifest-metadata.json',
        portfolio_module._raw_payload_stats(attempt),
    )
    excluded = {'SHA256SUMS', 'checksum-validation.txt'}
    records = {
        path.relative_to(attempt).as_posix(): portfolio_module.file_sha256(path)
        for path in sorted(attempt.rglob('*'))
        if path.is_file()
        and not path.is_symlink()
        and path.relative_to(attempt).as_posix() not in excluded
    }
    (attempt / 'SHA256SUMS').write_text(
        ''.join(f'{records[relative]}  {relative}\n' for relative in sorted(records)),
        encoding='ascii',
    )
    (attempt / 'checksum-validation.txt').write_text(
        ''.join(f'{relative}: OK\n' for relative in sorted(records)),
        encoding='ascii',
    )


@pytest.mark.parametrize(
    'mutation',
    ['prepared_drift', 'finalized_before_review', 'final_report_drift'],
)
def test_finalized_attempt_rejects_rebound_chronology_tamper(
    tmp_path: Path,
    mutation: str,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    candidate_sha = str(fixture['candidate_sha'])
    proof_path = attempt / 'projection' / f'portfolio-{candidate_sha}.json'
    proof = _read_json(proof_path)
    final_report_path = attempt / 'finalize-report.json'
    final_report = _read_json(final_report_path)
    if mutation == 'prepared_drift':
        proof['prepared_utc'] = '2000-01-01T00:00:00Z'
        final_report['prepared_utc'] = proof['prepared_utc']
    elif mutation == 'finalized_before_review':
        proof['finalized_utc'] = '2000-01-01T00:00:00Z'
        final_report['finalized_utc'] = proof['finalized_utc']
    else:
        final_report['finalized_utc'] = '2099-01-01T00:00:00Z'
    _write_canonical_json(proof_path, proof)
    final_report['portfolio_proof_sha256'] = portfolio_module.file_sha256(proof_path)
    _write_canonical_json(final_report_path, final_report)
    _rebind_raw_attempt(attempt)
    with pytest.raises(
        EvidenceError,
        match=r'timestamp|chronology|prepared|finalized|proof|report',
    ):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


def test_finalized_raw_attempt_rejects_extra_rebound_file(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    (attempt / 'forged-extra.txt').write_text('forged but checksummed\n', encoding='utf-8')
    _rebind_raw_attempt(attempt)
    with pytest.raises(EvidenceError, match=r'file set|unexpected|allowlist'):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            str(fixture['candidate_sha']),
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


@pytest.mark.parametrize('mutation', ['missing_raw_file', 'extra_projection_file'])
def test_finalized_raw_attempt_rejects_rebound_file_set_drift(
    tmp_path: Path,
    mutation: str,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    if mutation == 'missing_raw_file':
        (attempt / 'diagrams/architecture.mmd').unlink()
    else:
        (attempt / 'projection/unexpected.txt').write_text('unexpected\n', encoding='utf-8')
    _rebind_raw_attempt(attempt)
    with pytest.raises(EvidenceError, match=r'file set|missing|unexpected|projection'):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            str(fixture['candidate_sha']),
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


def test_finalized_raw_attempt_rejects_checksum_tamper(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    path = attempt / 'diagrams/architecture.mmd'
    path.write_bytes(path.read_bytes() + b'tamper\n')
    with pytest.raises(EvidenceError, match='checksum mismatch'):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            str(fixture['candidate_sha']),
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


@pytest.mark.parametrize('target', ['attempt', 'directory', 'file'])
def test_finalized_raw_attempt_rejects_mode_drift(tmp_path: Path, target: str) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    attempt = Path(fixture['attempt'])
    if target == 'attempt':
        changed = attempt
    elif target == 'directory':
        changed = attempt / 'diagrams'
    else:
        changed = attempt / 'candidate-contract.json'
    changed.chmod(0o755)
    with pytest.raises(EvidenceError, match='mode'):
        portfolio_module._validate_finalized_attempt(
            Path(fixture['repository']),
            attempt.parents[1],
            str(fixture['candidate_sha']),
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )


def test_project_rolls_back_all_six_paths_after_late_sixth_write_failure(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    attempt = Path(fixture['attempt'])
    paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    destinations = {repository / relative for relative in paths}
    projection_root = next(iter(destinations)).parent
    projection_root.mkdir(parents=True, exist_ok=True)
    projection_root.chmod(0o700)
    projection_root_mode = stat.S_IMODE(projection_root.stat().st_mode)
    original_write_once = portfolio_module._write_once
    writes = 0

    def fail_after_sixth_write(path: Path, payload: bytes, *, mode: int = 0o600) -> None:
        nonlocal writes
        original_write_once(path, payload, mode=mode)
        if path in destinations:
            writes += 1
            if writes == 6:
                raise OSError('injected post-sixth-write failure')

    monkeypatch.setattr(portfolio_module, '_write_once', fail_after_sixth_write)
    with pytest.raises(OSError, match='post-sixth-write'):
        portfolio_module.project_portfolio(
            repository,
            attempt.parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )
    assert writes == 6
    assert all(
        not destination.exists() and not destination.is_symlink() for destination in destinations
    )
    assert stat.S_IMODE(projection_root.stat().st_mode) == projection_root_mode


def test_project_enforces_mode_0644_under_restrictive_umask(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    previous_umask = os.umask(0o077)
    try:
        portfolio_module.project_portfolio(
            repository,
            Path(fixture['attempt']).parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )
    finally:
        os.umask(previous_umask)
    assert all(stat.S_IMODE((repository / relative).stat().st_mode) == 0o644 for relative in paths)


def test_project_rolls_back_if_written_mode_is_not_0644(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    destinations = {repository / relative for relative in paths}
    original_write_once = portfolio_module._write_once

    def force_wrong_mode(path: Path, payload: bytes, *, mode: int = 0o600) -> None:
        original_write_once(path, payload, mode=mode)
        if path in destinations:
            path.chmod(0o600)

    previous_umask = os.umask(0o077)
    try:
        monkeypatch.setattr(portfolio_module, '_write_once', force_wrong_mode)
        with pytest.raises(EvidenceError, match='mode is not 0644'):
            portfolio_module.project_portfolio(
                repository,
                Path(fixture['attempt']).parents[1],
                candidate_sha,
                Path(fixture['phase3_root']),
                str(fixture['attempt_id']),
            )
        assert all(
            not destination.exists() and not destination.is_symlink()
            for destination in destinations
        )
        monkeypatch.setattr(portfolio_module, '_write_once', original_write_once)
        portfolio_module.project_portfolio(
            repository,
            Path(fixture['attempt']).parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )
    finally:
        os.umask(previous_umask)
    assert all(stat.S_IMODE(destination.stat().st_mode) == 0o644 for destination in destinations)


def test_project_refuses_overwrite_before_creating_any_projection(tmp_path: Path) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    paths = portfolio_module.portfolio_projection_paths(candidate_sha)
    sentinel = repository / paths[-1]
    sentinel.parent.mkdir(parents=True, exist_ok=True)
    sentinel.write_bytes(b'preserve existing projection\n')
    with pytest.raises(EvidenceError, match='refusing to overwrite'):
        portfolio_module.project_portfolio(
            repository,
            Path(fixture['attempt']).parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )
    assert sentinel.read_bytes() == b'preserve existing projection\n'
    assert all(not (repository / relative).exists() for relative in paths[:-1])


def test_project_rejects_projection_parent_symlink_without_outside_creation(
    tmp_path: Path,
) -> None:
    fixture = _prepared_finalize_fixture(tmp_path)
    _finalize_fixture(fixture)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    outside = tmp_path / 'outside-projection'
    outside.mkdir()
    projection_root = repository / portfolio_module.PORTFOLIO_PROJECTION_ROOT
    projection_root.parent.mkdir(parents=True, exist_ok=True)
    projection_root.symlink_to(outside, target_is_directory=True)
    with pytest.raises(EvidenceError, match='symlink'):
        portfolio_module.project_portfolio(
            repository,
            Path(fixture['attempt']).parents[1],
            candidate_sha,
            Path(fixture['phase3_root']),
            str(fixture['attempt_id']),
        )
    assert list(outside.iterdir()) == []


def test_fake_ignored_ruff_with_matching_version_string_is_not_authentic(
    tmp_path: Path,
) -> None:
    repository, _ = _candidate_contract_fixture(tmp_path)
    fake = repository / '.venv/bin/ruff'
    _write_executable(fake, "#!/bin/sh\nprintf 'ruff 0.16.4\\n'\n")
    with pytest.raises(EvidenceError, match=r'authentic|digest|SHA|trusted'):
        portfolio_module._seed_replay_tooling(
            repository,
            tmp_path / 'detached-worktree',
            'phase5-local',
        )


def test_raw_manifest_rejects_symlink(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    target = tmp_path / 'target.txt'
    target.write_text('target\n', encoding='utf-8')
    (attempt / 'link.txt').symlink_to(target)
    with pytest.raises(EvidenceError, match='symlink'):
        portfolio_module._write_raw_checksum_manifest(attempt)


def test_raw_manifest_rejects_hardlink(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    path = attempt / 'linked.txt'
    path.write_text('linked\n', encoding='utf-8')
    os.link(path, tmp_path / 'second-link.txt')
    with pytest.raises(EvidenceError, match='hard-linked'):
        portfolio_module._write_raw_checksum_manifest(attempt)


def test_raw_manifest_rejects_unsafe_filename(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    (attempt / 'unsafe\nname.txt').write_text('unsafe\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='unsafe characters'):
        portfolio_module._write_raw_checksum_manifest(attempt)


def test_raw_manifest_rejects_oversize_sparse_file(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    path = attempt / 'oversize.bin'
    with path.open('wb') as stream:
        stream.truncate(portfolio_module.MAX_RESULT_DIRECTORY_BYTES + 1)
    with pytest.raises(EvidenceError, match='size bound'):
        portfolio_module._write_raw_checksum_manifest(attempt)


def test_stable_read_rejects_identity_change_during_read(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    path = tmp_path / 'unstable.txt'
    path.write_text('unstable identity\n', encoding='utf-8')
    real_fstat = os.fstat
    calls = 0

    def changed_second_fstat(descriptor: int) -> os.stat_result | SimpleNamespace:
        nonlocal calls
        observed = real_fstat(descriptor)
        calls += 1
        if calls == 1:
            return observed
        return SimpleNamespace(
            st_ctime_ns=observed.st_ctime_ns,
            st_dev=observed.st_dev,
            st_ino=observed.st_ino,
            st_mode=observed.st_mode,
            st_mtime_ns=observed.st_mtime_ns + 1,
            st_nlink=observed.st_nlink,
            st_size=observed.st_size,
        )

    monkeypatch.setattr(portfolio_module.os, 'fstat', changed_second_fstat)
    with pytest.raises(EvidenceError, match='changed while it was read'):
        portfolio_module._stable_read(path, 'unstable fixture', maximum_bytes=1024)


def test_raw_manifest_rejects_unix_socket(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    path = attempt / 'special.sock'
    with socket.socket(socket.AF_UNIX) as listener:
        listener.bind(str(path))
        with pytest.raises(EvidenceError, match=r'cannot open|not a regular file'):
            portfolio_module._write_raw_checksum_manifest(attempt)


def test_raw_manifest_rejects_fifo_without_blocking(tmp_path: Path) -> None:
    attempt = tmp_path / 'attempt'
    attempt.mkdir()
    os.mkfifo(attempt / 'special.fifo')
    with pytest.raises(EvidenceError, match='not a regular file'):
        portfolio_module._write_raw_checksum_manifest(attempt)


def _write_raw_proof_identity(path: Path, attempt_id: str) -> None:
    _write_canonical_json(
        path,
        {
            'candidate': {'git_sha': CANDIDATE_SHA},
            'documentation_replay': {'attempt_id': attempt_id},
        },
    )


def test_portfolio_attempt_id_accepts_exact_attempt_projection_path(tmp_path: Path) -> None:
    root = tmp_path / 'portfolio-root'
    attempt_id = '20260828T120003Z-1'
    proof = root / 'attempts' / attempt_id / 'projection' / f'portfolio-{CANDIDATE_SHA}.json'
    _write_raw_proof_identity(proof, attempt_id)
    assert portfolio_module.portfolio_attempt_id(root, proof, CANDIDATE_SHA) == attempt_id


@pytest.mark.parametrize('shape', ['sibling', 'extra-depth', 'outside-root'])
def test_portfolio_attempt_id_rejects_nonexact_path_shapes(tmp_path: Path, shape: str) -> None:
    root = tmp_path / 'portfolio-root'
    root.mkdir()
    attempt_id = '20260828T120004Z-1'
    filename = f'portfolio-{CANDIDATE_SHA}.json'
    if shape == 'sibling':
        proof = root / 'attempts' / attempt_id / 'not-projection' / filename
    elif shape == 'extra-depth':
        proof = root / 'attempts' / attempt_id / 'extra' / 'projection' / filename
    else:
        proof = tmp_path / 'other-root' / 'attempts' / attempt_id / 'projection' / filename
    _write_raw_proof_identity(proof, attempt_id)
    with pytest.raises(EvidenceError, match='outside the exact'):
        portfolio_module.portfolio_attempt_id(root, proof, CANDIDATE_SHA)


def test_portfolio_attempt_id_rejects_proof_alias(tmp_path: Path) -> None:
    root = tmp_path / 'portfolio-root'
    root.mkdir()
    attempt_id = '20260828T120005Z-1'
    target = tmp_path / 'proof-target.json'
    _write_raw_proof_identity(target, attempt_id)
    proof = root / 'attempts' / attempt_id / 'projection' / f'portfolio-{CANDIDATE_SHA}.json'
    proof.parent.mkdir(parents=True)
    proof.symlink_to(target)
    with pytest.raises(EvidenceError, match=r'symlink|alias'):
        portfolio_module.portfolio_attempt_id(root, proof, CANDIDATE_SHA)


def test_portfolio_attempt_id_requires_full_lowercase_candidate_sha(tmp_path: Path) -> None:
    uppercase_sha = 'A' * 40
    root = tmp_path / 'portfolio-root'
    attempt_id = '20260828T120008Z-1'
    proof = root / 'attempts' / attempt_id / 'projection' / f'portfolio-{uppercase_sha}.json'
    _write_canonical_json(
        proof,
        {
            'candidate': {'git_sha': uppercase_sha},
            'documentation_replay': {'attempt_id': attempt_id},
        },
    )
    with pytest.raises(EvidenceError, match=r'full lowercase|Git SHA'):
        portfolio_module.portfolio_attempt_id(root, proof, uppercase_sha)
