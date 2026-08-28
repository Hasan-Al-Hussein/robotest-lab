#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Pure adversarial tests for the Phase 5 CI and remote-evidence contract."""

from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import importlib.util
import json
import math
import os
import shlex
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib
from datetime import UTC, datetime, timedelta
from pathlib import Path

import pytest
import yaml

TESTS = Path(__file__).resolve().parent
REPOSITORY = TESTS.parent
sys.path.insert(0, str(TESTS))

import phase5_ci as phase5_module  # noqa: E402
import phase5_portfolio_evidence as portfolio_module  # noqa: E402
import phase5_release_docs as release_docs_module  # noqa: E402
import phase5_release_evidence as release_module  # noqa: E402
from phase5_ci import (  # noqa: E402
    ACTION_PINS,
    SOURCE_SNAPSHOT_INPUTS,
    EvidenceError,
    prune_local_runs,
    select_successful_run,
    source_snapshot,
    validate_checksum_manifest,
    validate_file_checksum_manifest,
    validate_license_declarations,
    validate_non_live_test_surface,
    validate_release_claims,
    validate_workflow,
    write_checksum_manifest,
    write_file_checksum_manifest,
    write_remote_summary,
)
from phase5_release_evidence import validate_release_evidence  # noqa: E402

WORKFLOW = REPOSITORY / '.github/workflows/robotest-ci.yml'
PHASE5_FIXTURE_INSTALL_ROOT_ENV = 'ROBOTEST_PHASE5_FIXTURE_INSTALL_ROOT'

_RELEASE_FIXTURE_TEMPLATE_DIRECTORY: tempfile.TemporaryDirectory | None = None
_RELEASE_FIXTURE_TEMPLATE: dict[str, Path | str] | None = None
_RELEASE_FIXTURE_LAST_REPOSITORY: Path | None = None


def test_phase5_docs_freeze_clean_candidate_release_order() -> None:
    document = (REPOSITORY / 'docs/testing/phase5-ci.md').read_text(encoding='utf-8')
    ordered_steps = (
        'Create and push the clean candidate commit C.',
        "wait\n   for C's hosted `RoboTest CI` workflow to complete successfully.",
        'run the separately authorized Phase 3 campaign and\n   Phase 4 acceptance workflow',
        'Capture P5-04/P5-05 from exact C',
        'run bare `scripts/verify_all.sh`',
        'Capture its candidate remote proof',
        'create the exact evidence-only child commit E',
        "Capture E's successful workflow",
        'Run `scripts/verify_all.sh --release-evidence`',
    )
    offsets = [document.index(step) for step in ordered_steps]
    assert offsets == sorted(offsets)
    assert 'Do not rerun a\n   clean-start live or bare gate after this point.' in document
    for option in (
        '--phase3-candidate-root',
        '--phase5-portfolio-root',
        '--phase5-portfolio-raw-proof',
        '--phase5-portfolio-proof',
    ):
        assert option in document


def test_repository_workflow_is_pinned_non_live_and_standard_runner() -> None:
    report = validate_workflow(WORKFLOW)
    assert report['runner'] == 'ubuntu-24.04'
    assert report['maximum_workers'] == 4
    assert report['permissions'] == {'contents': 'read'}
    assert report['action_pins'] == {name: ACTION_PINS[name] for name in sorted(ACTION_PINS)}
    assert len(report['sha256']) == 64


def _changed_workflow(tmp_path: Path, old: str, new: str) -> Path:
    source = WORKFLOW.read_text(encoding='utf-8')
    assert old in source
    path = tmp_path / 'workflow.yml'
    path.write_text(source.replace(old, new, 1), encoding='utf-8')
    return path


def test_workflow_rejects_movable_action_tag(tmp_path: Path) -> None:
    checkout_pin = ACTION_PINS['actions/checkout']
    path = _changed_workflow(tmp_path, f'actions/checkout@{checkout_pin}', 'actions/checkout@v6')
    with pytest.raises(EvidenceError, match='full SHA'):
        validate_workflow(path)


def test_workflow_requires_explicit_rosdep_cache_refresh(tmp_path: Path) -> None:
    path = _changed_workflow(tmp_path, 'rosdep update --rosdistro jazzy\n', '')
    with pytest.raises(EvidenceError, match='refresh the Jazzy rosdep cache'):
        validate_workflow(path)


def test_workflow_rejects_job_permission_override(tmp_path: Path) -> None:
    path = _changed_workflow(
        tmp_path,
        '    timeout-minutes: 60\n',
        '    timeout-minutes: 60\n    permissions:\n      contents: write\n',
    )
    with pytest.raises(EvidenceError, match='quality job contract changed'):
        validate_workflow(path)


def test_workflow_rejects_unreviewed_run_command(tmp_path: Path) -> None:
    path = _changed_workflow(
        tmp_path,
        'scripts/verify_phase5.sh --ci',
        'echo unreviewed-command\n          scripts/verify_phase5.sh --ci',
    )
    with pytest.raises(EvidenceError, match='run-command contract changed'):
        validate_workflow(path)


def test_workflow_requires_failure_evidence_upload(tmp_path: Path) -> None:
    path = _changed_workflow(tmp_path, 'if: always()', 'if: success()')
    with pytest.raises(EvidenceError, match='evidence upload must run after failures'):
        validate_workflow(path)


@pytest.mark.parametrize(
    'forbidden_command',
    [
        'scripts/run_benchmarks.sh campaign',
        'scripts/verify_phase4.sh --apply',
        'scripts/verify_all.sh',
        'ros2 launch robotest_navigation phase2.launch.py',
        'systemctl start robotest-supervisor.service',
        'git push origin HEAD',
    ],
)
def test_workflow_rejects_live_privileged_or_publication_commands(
    tmp_path: Path, forbidden_command: str
) -> None:
    path = _changed_workflow(
        tmp_path,
        'scripts/verify_phase5.sh --ci',
        f'scripts/verify_phase5.sh --ci\n          {forbidden_command}',
    )
    with pytest.raises(EvidenceError, match='workflow contains'):
        validate_workflow(path)


def test_workflow_yaml_parses_and_has_only_one_job() -> None:
    document = yaml.safe_load(WORKFLOW.read_text(encoding='utf-8'))
    assert set(document['jobs']) == {'quality'}


def test_dependency_license_inventory_and_first_party_licenses_are_complete() -> None:
    report = validate_license_declarations(REPOSITORY)
    assert report['license'] == 'Apache-2.0'
    assert report['package_count'] == 8
    assert report['packages'] == sorted(report['packages'])
    dependency_inventory = report['dependency_inventory']
    assert dependency_inventory['apt_package_count'] == 43
    assert dependency_inventory['github_action_count'] == 3
    assert dependency_inventory['python_distribution_count'] == 1
    assert dependency_inventory['ros_dependency_count'] == 62
    assert dependency_inventory['rosdep_system_dependency_count'] == 6
    assert 'Direct repository declarations only' in report['verification_scope']


def test_stage_runtime_overlay_requires_regular_contact_aggregator_dso() -> None:
    script = (REPOSITORY / 'scripts/stage_runtime_overlay.sh').read_text(encoding='utf-8')
    dso = (
        '${release_target}/install/robotest_sim/lib/robotest_sim/'
        'librobotest_contact_aggregator_system.so'
    )
    assert f'[[ -f "{dso}" &&\n    ! -L "{dso}" ]] ||' in script
    assert "die 'Release contact aggregator DSO is missing or not a regular file.'" in script


def _copy_license_fixture(destination: Path) -> None:
    for relative in ('LICENSE', 'NOTICE.md', 'pyproject.toml'):
        shutil.copy2(REPOSITORY / relative, destination / relative)
    for directory in ('config', 'src', 'supervisor'):
        shutil.copytree(
            REPOSITORY / directory,
            destination / directory,
            ignore=shutil.ignore_patterns('__pycache__', '.pytest_cache', '.ruff_cache', '*.pyc'),
        )


def test_license_inventory_rejects_an_unrecorded_apt_dependency(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    repository.mkdir()
    _copy_license_fixture(repository)
    path = repository / 'config/dependency-license-inventory.json'
    inventory = json.loads(path.read_text(encoding='utf-8'))
    del inventory['apt_packages']['shellcheck']
    path.write_text(json.dumps(inventory) + '\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='apt dependency inventory is incomplete'):
        validate_license_declarations(repository)


def test_license_inventory_requires_every_package_local_license(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    repository.mkdir()
    _copy_license_fixture(repository)
    (repository / 'src/robotest_sim/LICENSE').unlink()
    with pytest.raises(EvidenceError, match='robotest_sim is missing'):
        validate_license_declarations(repository)


def test_release_claims_resolve_to_checked_in_evidence() -> None:
    report = validate_release_claims(REPOSITORY)
    assert report['status'] == 'PASS'
    assert report['claim_count'] == 24
    assert report['evidence_file_count'] == 2
    assert 'README.md' in report['verification_scope']
    assert 'docs/portfolio.md' in report['verification_scope']


def test_release_claim_audit_rejects_missing_evidence_text(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    for relative in (
        'README.md',
        'config/release-claims.json',
        'docs/portfolio.md',
        'docs/results/phase-1/20260825T200725Z-1333.md',
        'docs/results/phase-2/20260826T010218Z-466.md',
    ):
        source = REPOSITORY / relative
        destination = repository / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    phase1 = repository / 'docs/results/phase-1/20260825T200725Z-1333.md'
    phase1.write_text(
        phase1.read_text(encoding='utf-8').replace('103 tests, 0 errors, 0 failures', 'removed'),
        encoding='utf-8',
    )
    with pytest.raises(EvidenceError, match='claim evidence text is absent'):
        validate_release_claims(repository)


@pytest.mark.parametrize(
    ('field', 'replacement', 'message'),
    [
        (
            'claim_document',
            'docs/portfolio-copy.md',
            'portfolio claim contract changed',
        ),
        ('claim_text', 'calculated real-time factor median `9.9999`', 'published claim text'),
        ('evidence_text', '`9.9999999999999999`', 'claim evidence text'),
    ],
)
def test_release_claim_audit_rejects_portfolio_contract_drift(
    tmp_path: Path,
    field: str,
    replacement: str,
    message: str,
) -> None:
    repository = tmp_path / 'repository'
    for relative in (
        'README.md',
        'config/release-claims.json',
        'docs/portfolio.md',
        'docs/results/phase-1/20260825T200725Z-1333.md',
        'docs/results/phase-2/20260826T010218Z-466.md',
    ):
        source = REPOSITORY / relative
        destination = repository / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    audit_path = repository / 'config/release-claims.json'
    audit = json.loads(audit_path.read_text(encoding='utf-8'))
    claim = next(item for item in audit['claims'] if item['id'] == 'portfolio-phase1-rtf-median')
    if field == 'claim_document':
        shutil.copy2(repository / 'docs/portfolio.md', repository / replacement)
    claim[field] = replacement
    _canonical_file(audit_path, audit)
    with pytest.raises(EvidenceError, match=message):
        validate_release_claims(repository)


def test_release_claim_audit_requires_portfolio_scope(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    for relative in (
        'README.md',
        'config/release-claims.json',
        'docs/portfolio.md',
        'docs/results/phase-1/20260825T200725Z-1333.md',
        'docs/results/phase-2/20260826T010218Z-466.md',
    ):
        source = REPOSITORY / relative
        destination = repository / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
    audit_path = repository / 'config/release-claims.json'
    audit = json.loads(audit_path.read_text(encoding='utf-8'))
    audit['scope'] = 'Quantitative and outcome claims published in README.md.'
    _canonical_file(audit_path, audit)
    with pytest.raises(EvidenceError, match='claim audit scope is missing'):
        validate_release_claims(repository)


def _source_snapshot_fixture(root: Path) -> dict[str, Path]:
    directory_inputs = {
        '.github',
        'benchmarks',
        'config',
        'docs',
        'packaging',
        'scenarios',
        'scripts',
        'src',
        'supervisor',
        'tests',
    }
    representatives: dict[str, Path] = {}
    for entry in SOURCE_SNAPSHOT_INPUTS:
        path = root / entry
        if entry in directory_inputs:
            path.mkdir(parents=True, exist_ok=True)
            representative = path / 'representative.txt'
            representative.write_text(f'{entry}\n', encoding='utf-8')
            representatives[entry] = representative
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(f'{entry}\n', encoding='utf-8')
            representatives[entry] = path
    return representatives


def test_source_snapshot_binds_every_release_surface_input(tmp_path: Path) -> None:
    representatives = _source_snapshot_fixture(tmp_path)
    baseline = source_snapshot(tmp_path)
    assert baseline['inputs'] == list(SOURCE_SNAPSHOT_INPUTS)
    for entry, representative in representatives.items():
        original = representative.read_text(encoding='utf-8')
        representative.write_text(f'{original}changed\n', encoding='utf-8')
        changed = source_snapshot(tmp_path)
        assert changed['aggregate_sha256'] != baseline['aggregate_sha256'], entry
        representative.write_text(original, encoding='utf-8')


def test_source_snapshot_excludes_generated_caches_and_benchmark_raw(tmp_path: Path) -> None:
    _source_snapshot_fixture(tmp_path)
    baseline = source_snapshot(tmp_path)
    generated = (
        tmp_path / 'tests/__pycache__/generated.pyc',
        tmp_path / 'tests/.pytest_cache/state',
        tmp_path / 'src/.ruff_cache/state',
        tmp_path / 'benchmarks/raw/runtime.json',
    )
    for path in generated:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text('generated\n', encoding='utf-8')
    assert source_snapshot(tmp_path) == baseline


def test_repository_package_test_surface_is_non_live() -> None:
    report = validate_non_live_test_surface(REPOSITORY)
    assert report['live_test_registration_found'] is False
    assert report['scanned_file_count'] > 0


def test_package_test_surface_rejects_launch_registration(tmp_path: Path) -> None:
    package = tmp_path / 'src/example'
    package.mkdir(parents=True)
    (package / 'package.xml').write_text('<package/>\n', encoding='utf-8')
    (package / 'CMakeLists.txt').write_text(
        'add_launch_test(test/live.launch.py)\n', encoding='utf-8'
    )
    with pytest.raises(EvidenceError, match='launch test registration'):
        validate_non_live_test_surface(tmp_path)


def _run_record(
    sha: str,
    *,
    run_id: int = 10,
    status: str = 'completed',
    conclusion: str = 'success',
    workflow: str = 'RoboTest CI',
    updated_at: str | None = None,
) -> dict[str, object]:
    created_at = f'2026-08-26T00:00:{run_id:02d}Z'
    return {
        'conclusion': conclusion,
        'createdAt': created_at,
        'databaseId': run_id,
        'headSha': sha,
        'status': status,
        'updatedAt': updated_at or created_at,
        'url': f'https://github.com/example/robotest/actions/runs/{run_id}',
        'workflowName': workflow,
    }


def test_remote_run_selection_is_exact_sha_and_newest_success() -> None:
    sha = '1' * 40
    records = [
        _run_record('2' * 40, run_id=30),
        _run_record(sha, run_id=10),
        _run_record(sha, run_id=20, conclusion='failure'),
        _run_record(sha, run_id=15),
    ]
    selected = select_successful_run(records, sha)
    assert selected['run_id'] == 15
    assert selected['head_sha'] == sha
    assert selected['conclusion'] == 'success'
    assert selected['completed_at'] == '2026-08-26T00:00:15Z'


@pytest.mark.parametrize('updated_at', [None, 'not-a-timestamp', '2026-08-25T23:59:59Z'])
def test_remote_run_selection_rejects_invalid_completion_timestamp(
    updated_at: str | None,
) -> None:
    record = _run_record('1' * 40)
    if updated_at is None:
        del record['updatedAt']
    else:
        record['updatedAt'] = updated_at
    with pytest.raises(EvidenceError, match='timestamp'):
        select_successful_run([record], '1' * 40)


@pytest.mark.parametrize(
    ('records', 'sha', 'message'),
    [
        ([], '1' * 40, 'no successful'),
        ([_run_record('1' * 40, status='in_progress', conclusion='')], '1' * 40, 'no successful'),
        ([_run_record('1' * 40, workflow='Another Workflow')], '1' * 40, 'no successful'),
        ([_run_record('1' * 40)], 'ABC', '40 lowercase'),
    ],
)
def test_remote_run_selection_fails_closed(
    records: list[dict[str, object]], sha: str, message: str
) -> None:
    with pytest.raises(EvidenceError, match=message):
        select_successful_run(records, sha)


def test_remote_summary_binds_command_repository_run_and_local_resolution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    sha = '1' * 40
    runs = tmp_path / 'runs.json'
    runs.write_text(json.dumps([_run_record(sha)]) + '\n', encoding='utf-8')
    output = tmp_path / 'remote.json'
    monkeypatch.setattr(
        phase5_module,
        '_command_version',
        lambda command, _cwd: {'argv': command, 'executable': command[0], 'output': 'test'},
    )
    arguments = argparse.Namespace(
        command_argument=['scripts/verify_phase5.sh', '--remote', sha],
        cwd=tmp_path,
        local_resolved_sha=sha,
        output=output,
        remote_sha=sha,
        repository='example/robotest',
        repository_path=tmp_path,
        repository_url='https://github.com/example/robotest',
        runs_json=runs,
        sha=sha,
        visibility='PUBLIC',
    )
    write_remote_summary(arguments)
    result = json.loads(output.read_text(encoding='utf-8'))
    assert result['run']['run_url'] == 'https://github.com/example/robotest/actions/runs/10'
    assert result['run']['completed_at'] == '2026-08-26T00:00:10Z'
    assert result['provenance']['command']['argv'] == [
        'scripts/verify_phase5.sh',
        '--remote',
        sha,
    ]
    assert result['provenance']['local_resolved_sha'] == sha


def test_bounded_log_drains_input_and_records_truncation(tmp_path: Path) -> None:
    output = tmp_path / 'output.log'
    metadata = tmp_path / 'metadata.json'
    payload = b'0123456789abcdef'
    result = subprocess.run(
        [
            sys.executable,
            str(TESTS / 'phase5_ci.py'),
            'bounded-log',
            '--output',
            str(output),
            '--metadata',
            str(metadata),
            '--maximum-bytes',
            '8',
        ],
        input=payload,
        check=False,
    )
    assert result.returncode == 0
    assert output.read_bytes() == payload[:8]
    assert json.loads(metadata.read_text(encoding='utf-8')) == {
        'maximum_bytes': 8,
        'retained_bytes': 8,
        'schema_version': 1,
        'total_bytes': 16,
        'truncated': True,
    }


def test_checksum_manifest_is_exact_and_detects_tampering(tmp_path: Path) -> None:
    run = tmp_path / 'run'
    nested = run / 'nested'
    nested.mkdir(parents=True)
    (run / 'summary.json').write_text('{"status":"PASS"}\n', encoding='utf-8')
    (nested / 'test.log').write_text('passed\n', encoding='utf-8')
    report = write_checksum_manifest(run)
    assert report['file_count'] == 2
    assert validate_checksum_manifest(run) == {'file_count': 2, 'status': 'PASS'}

    (nested / 'test.log').write_text('tampered\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='checksum mismatch'):
        validate_checksum_manifest(run)


def _contact_gate_binary_fixture(*, symlink_install: bool) -> dict[str, object]:
    return {
        'build_embedded_source_inventory_match': True,
        'build_embedded_source_inventory_sha256': '3' * 64,
        'build_elf_build_id': 'a1',
        'build_install_build_id_match': True,
        'build_install_samefile': symlink_install,
        'build_install_sha256_match': True,
        'build_path': release_module.CONTACT_GATE_BUILD_PATH,
        'build_regular_executable': True,
        'build_sha256': '4' * 64,
        'installed_declared_is_symlink': symlink_install,
        'installed_declared_path': release_module.CONTACT_GATE_INSTALL_PATH,
        'installed_declared_samefile': True,
        'installed_elf_build_id': 'a1',
        'installed_embedded_source_inventory_match': True,
        'installed_embedded_source_inventory_sha256': '3' * 64,
        'installed_path': (
            release_module.CONTACT_GATE_BUILD_PATH
            if symlink_install
            else release_module.CONTACT_GATE_INSTALL_PATH
        ),
        'installed_regular_executable': True,
        'installed_sha256': '4' * 64,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': '3' * 64,
    }


def test_phase5_source_install_and_contact_gate_binding_validators_fail_closed() -> None:
    source_record = {
        'binding_type': 'python_module',
        'installed_path': 'install/robotest_metrics/module.py',
        'installed_sha256': '1' * 64,
        'matches': True,
        'package': 'robotest_metrics',
        'source_path': 'src/robotest_metrics/module.py',
        'source_sha256': '1' * 64,
    }
    source_install = {
        'aggregate_sha256': release_module._canonical_sha256([source_record]),
        'all_match': True,
        'file_count': 1,
        'records': [source_record],
    }
    release_module._validate_source_install(source_install)
    forged_source_install = copy.deepcopy(source_install)
    forged_source_install['records'][0]['installed_sha256'] = '2' * 64
    with pytest.raises(EvidenceError, match='source/install record'):
        release_module._validate_source_install(forged_source_install)

    gate_binding = _contact_gate_binary_fixture(symlink_install=True)
    release_module._validate_contact_gate_binary_binding(gate_binding)
    forged_gate_binding = copy.deepcopy(gate_binding)
    forged_gate_binding['installed_embedded_source_inventory_sha256'] = '6' * 64
    with pytest.raises(EvidenceError, match='embedded source or build/install hash'):
        release_module._validate_contact_gate_binary_binding(forged_gate_binding)


@pytest.mark.parametrize('symlink_install', [False, True])
def test_phase5_contact_gate_binding_accepts_copy_and_symlink_installs(
    symlink_install: bool,
) -> None:
    release_module._validate_contact_gate_binary_binding(
        _contact_gate_binary_fixture(symlink_install=symlink_install)
    )


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('installed_declared_is_symlink', 1, 'binding changed'),
        (
            'installed_path',
            release_module.CONTACT_GATE_INSTALL_PATH,
            'declared/resolved install path',
        ),
        ('build_install_samefile', False, 'declared/resolved install path'),
    ],
)
def test_phase5_contact_gate_binding_rejects_path_tampering(
    field: str, value: object, message: str
) -> None:
    binding = _contact_gate_binary_fixture(symlink_install=True)
    binding[field] = value
    with pytest.raises(EvidenceError, match=message):
        release_module._validate_contact_gate_binary_binding(binding)


def test_remote_checksum_sidecar_detects_tampering(tmp_path: Path) -> None:
    evidence = tmp_path / 'remote.json'
    manifest = tmp_path / 'remote.SHA256SUMS'
    evidence.write_text('{"status":"PASS"}\n', encoding='utf-8')
    write_file_checksum_manifest(evidence, manifest)
    assert validate_file_checksum_manifest(evidence, manifest)['status'] == 'PASS'
    strict = subprocess.run(
        ['sha256sum', '-c', '--strict', manifest.name],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
    )
    assert strict.returncode == 0, strict.stdout + strict.stderr
    evidence.write_text('{"status":"FAIL"}\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='checksum mismatch'):
        validate_file_checksum_manifest(evidence, manifest)


def test_retention_keeps_current_and_four_newest_prior_direct_children(tmp_path: Path) -> None:
    root = tmp_path / 'phase5'
    root.mkdir()
    runs = [root / f'20260826T00000{index}Z-{index}' for index in range(1, 8)]
    for run in runs:
        run.mkdir()
        (run / 'sentinel').write_text(run.name, encoding='utf-8')
    unrelated = root / 'manual-notes'
    unrelated.mkdir()

    report = prune_local_runs(root, runs[-1], maximum_prior=4)
    assert report['current_run'] == runs[-1].name
    assert report['retained_prior_runs'] == [run.name for run in runs[2:6]]
    assert report['removed_runs'] == [run.name for run in runs[:2]]
    assert [run.exists() for run in runs] == [False, False, True, True, True, True, True]
    assert unrelated.is_dir()


def test_retention_rejects_matching_symlink_without_deleting_target(tmp_path: Path) -> None:
    root = tmp_path / 'phase5'
    root.mkdir()
    current = root / '20260826T000002Z-2'
    current.mkdir()
    external = tmp_path / 'external'
    external.mkdir()
    (external / 'sentinel').write_text('keep', encoding='utf-8')
    (root / '20260826T000001Z-1').symlink_to(external, target_is_directory=True)

    with pytest.raises(EvidenceError, match='symlink'):
        prune_local_runs(root, current, maximum_prior=4)
    assert (external / 'sentinel').read_text(encoding='utf-8') == 'keep'


def test_local_summary_keeps_live_and_remote_claims_out_of_scope(tmp_path: Path) -> None:
    checks = tmp_path / 'checks.tsv'
    checks.write_text('name\tstatus\texit_code\tlog\nunit\tpassed\t0\tunit.log\n', encoding='utf-8')
    reports = {}
    for name in ('workflow', 'licenses', 'claims', 'test-surface', 'retention'):
        path = tmp_path / f'{name}.json'
        path.write_text(json.dumps({'status': 'checked'}) + '\n', encoding='utf-8')
        reports[name] = path
    reports['source'] = tmp_path / 'source.json'
    reports['source'].write_text(
        json.dumps({'aggregate_sha256': 'a' * 64, 'file_count': 10}) + '\n',
        encoding='utf-8',
    )
    reports['provenance'] = tmp_path / 'provenance.json'
    reports['provenance'].write_text(
        json.dumps(
            {
                'command': {'argv': ['scripts/verify_phase5.sh', '--local'], 'cwd': str(tmp_path)},
                'platform': {'ros_distro': 'jazzy'},
                'source': {'aggregate_sha256': 'a' * 64, 'file_count': 10},
                'tool_distributions': {'pytest': 'test'},
                'tools': {'python': {'output': 'test'}},
            }
        )
        + '\n',
        encoding='utf-8',
    )
    output = tmp_path / 'summary.json'
    csv_output = tmp_path / 'summary.csv'
    result = subprocess.run(
        [
            sys.executable,
            str(TESTS / 'phase5_ci.py'),
            'summary',
            '--checks',
            str(checks),
            '--claims-report',
            str(reports['claims']),
            '--csv-output',
            str(csv_output),
            '--git-dirty',
            'true',
            '--git-sha',
            '1' * 40,
            '--license-report',
            str(reports['licenses']),
            '--maximum-workers',
            '3',
            '--mode',
            'local',
            '--output',
            str(output),
            '--provenance-report',
            str(reports['provenance']),
            '--retention-report',
            str(reports['retention']),
            '--status',
            'PASS',
            '--source-snapshot',
            str(reports['source']),
            '--test-surface-report',
            str(reports['test-surface']),
            '--workflow-report',
            str(reports['workflow']),
        ],
        check=False,
    )
    assert result.returncode == 0
    summary = json.loads(output.read_text(encoding='utf-8'))
    assert summary['status'] == 'PASS'
    assert summary['verification_level'] == 'L2'
    assert summary['source']['git_dirty'] is True
    assert summary['ci_context']['present'] is False
    assert not any(summary['runtime_scope'].values())
    assert summary['check_counts'] == {'failed': 0, 'passed': 1, 'total': 1}
    assert summary['maximum_workers'] == 3
    with csv_output.open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1
    assert rows[0]['status'] == summary['status']
    assert rows[0]['git_sha'] == summary['source']['git_sha']
    assert rows[0]['source_aggregate_sha256'] == 'a' * 64


def test_phase5_script_has_no_implicit_mode() -> None:
    script = REPOSITORY / 'scripts/verify_phase5.sh'
    script_text = script.read_text(encoding='utf-8')
    fixture_build = script_text.index('run_check colcon-build 1200s')
    overlay_source = script_text.index('source "${WORK_ROOT}/install/setup.bash"')
    pure_tests = script_text.index('run_check pure-python-tests 2700s')
    colcon_tests = script_text.index('run_check colcon-test 1500s')
    assert fixture_build < pure_tests < overlay_source < colcon_tests
    assert 'env ROBOTEST_PHASE5_FIXTURE_INSTALL_ROOT="${WORK_ROOT}/install"' in script_text
    assert 'run_check portfolio-contract 30s' in script_text
    assert 'createdAt,updatedAt' in script_text
    result = subprocess.run(['bash', str(script)], capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'There is deliberately no implicit mode.' in result.stderr
    help_result = subprocess.run(
        ['bash', str(script), '--help'], capture_output=True, text=True, check=False
    )
    assert help_result.returncode == 0
    assert 'never start Gazebo' in help_result.stdout


def test_release_fixture_install_root_uses_explicit_absolute_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    install_root = tmp_path / 'install'
    install_root.mkdir()
    monkeypatch.setenv(PHASE5_FIXTURE_INSTALL_ROOT_ENV, str(install_root))
    assert _release_fixture_install_root() == install_root.resolve(strict=True)


def test_release_fixture_install_root_rejects_relative_directory(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(PHASE5_FIXTURE_INSTALL_ROOT_ENV, 'install')
    with pytest.raises(RuntimeError, match='must be absolute'):
        _release_fixture_install_root()


def test_release_fixture_install_root_rejects_missing_directory(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setenv(PHASE5_FIXTURE_INSTALL_ROOT_ENV, str(tmp_path / 'missing'))
    with pytest.raises(RuntimeError, match='is unavailable'):
        _release_fixture_install_root()


def test_release_fixture_install_root_rejects_symlink(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    install_root = tmp_path / 'install'
    install_root.mkdir()
    alias = tmp_path / 'install-alias'
    alias.symlink_to(install_root, target_is_directory=True)
    monkeypatch.setenv(PHASE5_FIXTURE_INSTALL_ROOT_ENV, str(alias))
    with pytest.raises(RuntimeError, match='must not be a symlink'):
        _release_fixture_install_root()


def test_phase5_ci_mode_cannot_be_claimed_outside_github_actions() -> None:
    script = REPOSITORY / 'scripts/verify_phase5.sh'
    environment = os.environ.copy()
    environment.pop('GITHUB_ACTIONS', None)
    environment.pop('GITHUB_RUN_ID', None)
    environment.pop('GITHUB_SHA', None)
    result = subprocess.run(
        ['bash', str(script), '--ci'],
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert '--ci is restricted to GitHub Actions' in result.stderr


def test_phase5_remote_rejects_noncanonical_sha_before_network_access() -> None:
    script = REPOSITORY / 'scripts/verify_phase5.sh'
    result = subprocess.run(
        ['bash', str(script), '--remote', 'ABC'],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    assert '40 lowercase hexadecimal' in result.stderr


def test_local_evidence_is_ignored_but_compact_remote_evidence_is_trackable() -> None:
    sha = '1' * 40
    ignored = subprocess.run(
        ['git', 'check-ignore', '--no-index', 'artifacts/evidence/phase5/local.log'],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        check=False,
    )
    assert ignored.returncode == 0, ignored.stderr
    for suffix in ('json', 'SHA256SUMS', 'checksum-validation.txt'):
        candidate = f'docs/results/phase-5/remote-{sha}.{suffix}'
        trackable = subprocess.run(
            ['git', 'check-ignore', '--no-index', candidate],
            cwd=REPOSITORY,
            capture_output=True,
            text=True,
            check=False,
        )
        assert trackable.returncode == 1, f'{candidate}: {trackable.stdout}{trackable.stderr}'
    script = (REPOSITORY / 'scripts/verify_phase5.sh').read_text(encoding='utf-8')
    assert 'readonly REMOTE_EVIDENCE_ROOT="${PROJECT_ROOT}/docs/results/phase-5"' in script
    assert (
        'readonly EVIDENCE_COMMIT_REMOTE_ROOT="${EVIDENCE_ROOT}/remote-evidence-commit"' in script
    )
    assert 'output="${output_root}/remote-${REMOTE_SHA}.json"' in script
    raw_second = subprocess.run(
        [
            'git',
            'check-ignore',
            '--no-index',
            f'artifacts/evidence/phase5/remote-evidence-commit/remote-{sha}.json',
        ],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        check=False,
    )
    assert raw_second.returncode == 0, raw_second.stderr


def test_failed_gate_still_finalizes_checksums_csv_and_provenance(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    for directory in (
        '.github/workflows',
        'benchmarks',
        'config',
        'docs/testing',
        'packaging',
        'scenarios',
        'scripts',
        'src',
        'supervisor',
        'tests',
        '.venv/bin',
    ):
        (repository / directory).mkdir(parents=True, exist_ok=True)
    shutil.copy2(REPOSITORY / 'scripts/verify_phase5.sh', repository / 'scripts/verify_phase5.sh')
    shutil.copy2(TESTS / 'phase5_ci.py', repository / 'tests/phase5_ci.py')
    shutil.copy2(
        TESTS / 'phase5_portfolio_evidence.py',
        repository / 'tests/phase5_portfolio_evidence.py',
    )
    shutil.copy2(TESTS / 'phase5_release_docs.py', repository / 'tests/phase5_release_docs.py')
    shutil.copy2(REPOSITORY / 'docs/testing/phase5-ci.md', repository / 'docs/testing/phase5-ci.md')
    workflow = WORKFLOW.read_text(encoding='utf-8').replace(
        'permissions:\n  contents: read',
        'permissions:\n  contents: write',
        1,
    )
    (repository / '.github/workflows/robotest-ci.yml').write_text(workflow, encoding='utf-8')
    for relative in (
        '.editorconfig',
        '.gitattributes',
        '.pre-commit-config.yaml',
        'CONTRIBUTING.md',
        'LICENSE',
        'NOTICE.md',
        'README.md',
        'SECURITY.md',
        'config/input.yaml',
        'pyproject.toml',
        'src/input.txt',
        'supervisor/input.txt',
        'tests/smoke_test.py',
    ):
        (repository / relative).write_text('test input\n', encoding='utf-8')
    (repository / '.gitignore').write_text(
        '/.venv/\n/artifacts/evidence/phase5/\n', encoding='utf-8'
    )
    (repository / '.venv/bin/ruff').symlink_to(REPOSITORY / '.venv/bin/ruff')
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    subprocess.run(['git', 'add', '.'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit',
            '-q',
            '--no-gpg-sign',
            '-m',
            'fixture',
        ],
        cwd=repository,
        check=True,
    )

    script = repository / 'scripts/verify_phase5.sh'
    local_environment = os.environ.copy()
    for variable in ('GITHUB_ACTIONS', 'GITHUB_RUN_ID', 'GITHUB_SHA'):
        local_environment.pop(variable, None)
    result = subprocess.run(
        ['bash', str(script), '--local'],
        cwd=repository,
        env=local_environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode != 0
    runs = list((repository / 'artifacts/evidence/phase5').glob('20*T*-*'))
    assert len(runs) == 1
    run = runs[0]
    summary = json.loads((run / 'verification-summary.json').read_text(encoding='utf-8'))
    assert summary['status'] == 'FAIL'
    assert summary['check_counts']['failed'] == 1
    with (run / 'verification-summary.csv').open(encoding='utf-8', newline='') as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 1 and rows[0]['status'] == 'FAIL'
    provenance = json.loads((run / 'provenance.json').read_text(encoding='utf-8'))
    assert provenance['command'] == {
        'argv': [str(script), '--local'],
        'cwd': str(repository),
    }
    checksum = subprocess.run(
        ['sha256sum', '-c', '--strict', 'SHA256SUMS'],
        cwd=run,
        capture_output=True,
        text=True,
        check=False,
    )
    assert checksum.returncode == 0, checksum.stdout + checksum.stderr
    validation = (run / 'checksum-validation.txt').read_text(encoding='utf-8')
    assert 'verification-summary.json: OK' in validation
    assert validate_checksum_manifest(run)['status'] == 'PASS'


def test_verify_all_routes_only_phase5_local(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    scripts = repository / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(REPOSITORY / '.gitignore', repository / '.gitignore')
    orchestrator = scripts / 'verify_all.sh'
    shutil.copy2(REPOSITORY / 'scripts/verify_all.sh', orchestrator)
    orchestrator.chmod(0o755)
    call_log = tmp_path / 'calls.jsonl'
    fake = """#!/usr/bin/env bash
python3 - "$0" "$@" <<'PY'
import json
import os
import pathlib
import sys

with pathlib.Path(os.environ['CALL_LOG']).open('a', encoding='utf-8') as output:
    value = {'script': pathlib.Path(sys.argv[1]).name, 'args': sys.argv[2:]}
    output.write(json.dumps(value) + '\\n')
if pathlib.Path(sys.argv[1]).name == 'verify_phase0.sh':
    evidence = pathlib.Path.cwd() / 'artifacts/evidence/phase0/phase0-versions.json'
    evidence.write_text(
        json.dumps(
            {'checked_at': 'after', 'schema_version': 1},
            separators=(',', ':'),
            sort_keys=True,
        )
        + '\\n',
        encoding='utf-8',
    )
PY
"""
    for phase in range(6):
        verifier = scripts / f'verify_phase{phase}.sh'
        verifier.write_text(fake, encoding='utf-8')
        verifier.chmod(0o755)
    phase0_version = repository / 'artifacts/evidence/phase0/phase0-versions.json'
    _canonical_file(phase0_version, {'checked_at': 'before', 'schema_version': 1})
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    _commit_all(repository, 'candidate')

    environment = os.environ | {'CALL_LOG': str(call_log)}
    result = subprocess.run(
        ['bash', str(orchestrator)],
        cwd=repository,
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 3, result.stderr
    calls = [json.loads(line) for line in call_log.read_text(encoding='utf-8').splitlines()]
    assert calls == [
        {'script': 'verify_phase0.sh', 'args': []},
        {'script': 'verify_phase1.sh', 'args': []},
        {'script': 'verify_phase2.sh', 'args': []},
        {'script': 'verify_phase3.sh', 'args': []},
        {'script': 'verify_phase4.sh', 'args': []},
        {'script': 'verify_phase5.sh', 'args': ['--local']},
    ]
    all_arguments = [argument for call in calls for argument in call['args']]
    assert '--apply' not in all_arguments
    assert 'I_AUTHORIZE_EXACTLY_15_COLD_STACK_TRIALS_NO_RETRIES' not in all_arguments
    aggregate = json.loads(
        (repository / 'artifacts/evidence/phase0/verify-all.json').read_text(encoding='utf-8')
    )
    assert aggregate['status'] == 'incomplete'
    assert aggregate['schema_version'] == 4
    assert aggregate['source']['git_dirty_start'] is False
    assert aggregate['source']['git_dirty_end'] is True
    assert aggregate['source']['source_unchanged'] is True
    assert aggregate['source']['generated_evidence_delta']['changed_files'][0]['path'] == (
        'artifacts/evidence/phase0/phase0-versions.json'
    )
    checksum = subprocess.run(
        ['sha256sum', '-c', '--strict', 'verify-all.SHA256SUMS'],
        cwd=repository / 'artifacts/evidence/phase0',
        capture_output=True,
        text=True,
        check=False,
    )
    assert checksum.returncode == 0, checksum.stdout + checksum.stderr
    assert aggregate['release_eligible'] is False
    assert len(aggregate['outstanding_gates']) == 3
    assert 'INCOMPLETE' in result.stderr


def test_verify_all_refuses_dirty_worktree_before_routing(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    scripts = repository / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(REPOSITORY / '.gitignore', repository / '.gitignore')
    orchestrator = scripts / 'verify_all.sh'
    shutil.copy2(REPOSITORY / 'scripts/verify_all.sh', orchestrator)
    orchestrator.chmod(0o755)
    call_log = tmp_path / 'calls.log'
    fake = """#!/usr/bin/env bash
printf '%s\n' "$(basename "$0")" >>"${CALL_LOG}"
"""
    for phase in range(6):
        verifier = scripts / f'verify_phase{phase}.sh'
        verifier.write_text(fake, encoding='utf-8')
        verifier.chmod(0o755)
    phase0_version = repository / 'artifacts/evidence/phase0/phase0-versions.json'
    _canonical_file(phase0_version, {'checked_at': 'before', 'schema_version': 1})
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    _commit_all(repository, 'candidate')
    _canonical_file(phase0_version, {'checked_at': 'already-dirty', 'schema_version': 1})

    result = subprocess.run(
        ['bash', str(orchestrator)],
        cwd=repository,
        env=os.environ | {'CALL_LOG': str(call_log)},
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 2
    assert 'dirty worktree' in result.stderr
    assert not call_log.exists()
    assert not (repository / 'artifacts/evidence/phase0/verify-all.json').exists()


def test_verify_all_rejects_source_delta_outside_phase0_allowlist(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    scripts = repository / 'scripts'
    scripts.mkdir(parents=True)
    shutil.copy2(REPOSITORY / '.gitignore', repository / '.gitignore')
    shutil.copy2(REPOSITORY / 'scripts/verify_all.sh', scripts / 'verify_all.sh')
    fake = """#!/usr/bin/env bash
set -Eeuo pipefail
case "$(basename "$0")" in
  verify_phase0.sh)
    printf '{"checked_at":"after","schema_version":1}\n' \
      > artifacts/evidence/phase0/phase0-versions.json
    ;;
  verify_phase1.sh)
    printf 'changed\n' > config/source.txt
    ;;
esac
"""
    for phase in range(6):
        verifier = scripts / f'verify_phase{phase}.sh'
        verifier.write_text(fake, encoding='utf-8')
        verifier.chmod(0o755)
    _canonical_file(
        repository / 'artifacts/evidence/phase0/phase0-versions.json',
        {'checked_at': 'before', 'schema_version': 1},
    )
    source = repository / 'config/source.txt'
    source.parent.mkdir()
    source.write_text('before\n', encoding='utf-8')
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    _commit_all(repository, 'candidate')
    result = subprocess.run(
        ['bash', str(scripts / 'verify_all.sh')],
        cwd=repository,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 1
    aggregate = json.loads(
        (repository / 'artifacts/evidence/phase0/verify-all.json').read_text(encoding='utf-8')
    )
    assert aggregate['status'] == 'failed'
    assert aggregate['source']['source_unchanged'] is False
    assert 'config/source.txt' in aggregate['source']['git_status_porcelain_end']


def _release_evidence_placeholder_arguments() -> list[str]:
    return [
        '--release-evidence',
        '--local-aggregate',
        '/tmp/local.json',
        '--phase3-candidate-root',
        '/tmp/phase3',
        '--phase3-aggregate',
        '/tmp/phase3/aggregate.json',
        '--phase4-run-directory',
        '/tmp/phase4',
        '--phase4-scenario6',
        '/tmp/phase4/scenario6.json',
        '--phase5-portfolio-root',
        '/tmp/portfolio',
        '--phase5-portfolio-proof',
        '/tmp/portfolio/proof.json',
        '--phase5-remote-proof',
        '/tmp/remote.json',
        '--phase5-evidence-commit-remote-proof',
        '/tmp/evidence-remote.json',
    ]


def test_verify_all_release_help_names_every_acceptance_lane() -> None:
    result = subprocess.run(
        ['bash', str(REPOSITORY / 'scripts/verify_all.sh'), '--help'],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0
    assert 'Phase 3, Phase 4, portfolio, and public-CI' in result.stdout
    assert '--phase5-portfolio-root <exact-path>' in result.stdout
    assert '--phase5-portfolio-proof <exact-path>' in result.stdout


@pytest.mark.parametrize(
    'missing_option',
    ['--phase5-portfolio-root', '--phase5-portfolio-proof'],
)
def test_verify_all_release_mode_requires_both_portfolio_arguments(
    missing_option: str,
) -> None:
    arguments = _release_evidence_placeholder_arguments()
    index = arguments.index(missing_option)
    del arguments[index : index + 2]
    result = subprocess.run(
        ['bash', str(REPOSITORY / 'scripts/verify_all.sh'), *arguments],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2
    assert 'Usage:' in result.stderr


@pytest.mark.parametrize(
    'arguments',
    [
        ['--release-evidence', '--phase5-portfolio-root'],
        [
            '--release-evidence',
            '--phase5-portfolio-root',
            '/tmp/one',
            '--phase5-portfolio-root',
            '/tmp/two',
        ],
    ],
)
def test_verify_all_release_mode_rejects_malformed_portfolio_arguments(
    arguments: list[str],
) -> None:
    result = subprocess.run(
        ['bash', str(REPOSITORY / 'scripts/verify_all.sh'), *arguments],
        cwd=REPOSITORY,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 2


def _canonical_file(path: Path, value: object, *, sidecar: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(phase5_module.canonical_json_bytes(value))
    if sidecar:
        Path(f'{path}.sha256').write_text(
            f'{phase5_module.file_sha256(path)}  {path.name}\n',
            encoding='ascii',
        )


def _phase3_contact_profile_record(*, linux_tid: int, step: int) -> dict[str, object]:
    buckets = {
        'cached_event_state_check_ns': 20 + step,
        'contact_policy_protobuf_ns': 40 + step,
        'exhaustive_event_rescan_ns': 30 + step,
        'locked_binding_validation_ns': 10 + step,
        'publish_ns': 50 + step,
    }
    contribution = {
        **buckets,
        'linux_tid': linux_tid,
        'measured_total_ns': sum(buckets.values()),
        'observation_count': 2 + step,
        'publish_count': 1 + step,
        'rescan_count': 1 + step,
    }
    return {
        **buckets,
        'clock_id': 'CLOCK_THREAD_CPUTIME_ID',
        'measured_total_ns': sum(buckets.values()),
        'observation_count': 2 + step,
        'profile_epoch_start_sim_stamp_ns': 1_000,
        'publish_count': 1 + step,
        'rescan_count': 1 + step,
        'saturated': False,
        'schema_version': 2,
        'sim_stamp_ns': 5_000_001_000 + step * 5_000_000_000,
        'status': 'PASS',
        'thread_contributions': [contribution],
    }


def _phase3_contact_profile_stdout(*, linux_tid: int) -> bytes:
    prefix = b'[gazebo-1] ROBOTEST_CONTACT_PROFILE '
    return b''.join(
        prefix
        + phase5_module.canonical_json_bytes(
            _phase3_contact_profile_record(linux_tid=linux_tid, step=step)
        ).rstrip(b'\n')
        + b'\n'
        for step in range(2)
    )


def _phase3_profile_proc_stat(
    pid: int,
    *,
    process_group: int,
    start_ticks: int,
    user_ticks: int,
    comm: str = 'gz sim server',
    parent_pid: int = 1,
    session_id: int | None = None,
) -> str:
    fields = ['0'] * 50
    fields[0] = 'S'
    fields[1] = str(parent_pid)
    fields[2] = str(process_group)
    fields[3] = str(process_group if session_id is None else session_id)
    fields[11] = str(user_ticks)
    fields[12] = '5'
    fields[17] = '1'
    fields[19] = str(start_ticks)
    fields[36] = '2'
    return f'{pid} ({comm}) {" ".join(fields)}\n'


def _write_phase3_profile_proc_host(
    proc_root: Path,
    *,
    repository: Path,
    candidate_root: Path,
    domain_base: int,
) -> None:
    (proc_root / 'pressure').mkdir(parents=True)
    (proc_root / 'uptime').write_text('0.50 0.25\n', encoding='ascii')
    (proc_root / 'loadavg').write_text('1.00 0.50 0.25 2/100 999\n', encoding='ascii')
    (proc_root / 'stat').write_text(
        'ctxt 100\nprocesses 20\nprocs_running 2\nprocs_blocked 0\n',
        encoding='ascii',
    )
    profiler_vmstat_keys = (
        'oom_kill',
        'pgfault',
        'pgmajfault',
        'pgpgin',
        'pgpgout',
        'pswpin',
        'pswpout',
    )
    (proc_root / 'vmstat').write_text(
        ''.join(f'{key} {index}\n' for index, key in enumerate(profiler_vmstat_keys, 1)),
        encoding='ascii',
    )
    some = 'some avg10=0.10 avg60=0.20 avg300=0.30 total=10\n'
    full = 'full avg10=0.00 avg60=0.00 avg300=0.00 total=0\n'
    (proc_root / 'pressure/cpu').write_text(some, encoding='ascii')
    (proc_root / 'pressure/io').write_text(some + full, encoding='ascii')
    (proc_root / 'pressure/memory').write_text(some + full, encoding='ascii')
    owner = proc_root / '40'
    owner.mkdir()
    owner.joinpath('stat').write_text(
        _phase3_profile_proc_stat(
            40,
            process_group=30,
            session_id=30,
            start_ticks=40,
            user_ticks=10,
            comm='python3',
        ),
        encoding='ascii',
    )
    owner.joinpath('cmdline').write_bytes(
        b'/usr/bin/python3\0'
        + str(repository / 'tests/phase3_benchmark_runner.py').encode()
        + b'\0--workspace\0'
        + str(repository).encode()
        + b'\0--mode\0smoke\0--candidate-id\0'
        + candidate_root.name.encode()
        + b'\0--domain-base\0'
        + str(domain_base).encode()
        + b'\0--output-root\0artifacts/evidence/phase3-benchmarks\0'
        + b'--build-binding\0'
        + str(candidate_root / 'build-binding.json').encode()
        + b'\0'
    )
    owner_executable = proc_root / 'smoke-runner-python'
    owner_executable.write_bytes(b'python')
    owner.joinpath('exe').symlink_to(owner_executable)


def _write_phase3_profile_proc_process(
    proc_root: Path,
    *,
    pid: int,
    start_ticks: int,
    user_ticks: int,
    domain_id: int,
    gz_partition: str,
    plugin_path: Path,
) -> Path:
    process_root = proc_root / str(pid)
    thread_root = process_root / 'task' / str(pid)
    thread_root.mkdir(parents=True, exist_ok=True)
    process_root.joinpath('environ').write_bytes(
        (
            f'ROS_DOMAIN_ID={domain_id}\0GZ_PARTITION={gz_partition}\0ROBOTEST_CONTACT_PROFILE=1\0'
        ).encode()
    )
    process_root.joinpath('cmdline').write_bytes(b'/usr/bin/gz\0sim\0-s\0')
    stat_text = _phase3_profile_proc_stat(
        pid,
        process_group=pid,
        start_ticks=start_ticks,
        user_ticks=user_ticks,
        parent_pid=40,
    )
    process_root.joinpath('stat').write_text(stat_text, encoding='ascii')
    thread_root.joinpath('stat').write_text(stat_text, encoding='ascii')
    process_root.joinpath('exe').symlink_to(Path(sys.executable).resolve(strict=True))
    plugin_stat = plugin_path.stat()
    device = f'{os.major(plugin_stat.st_dev):x}:{os.minor(plugin_stat.st_dev):x}'
    process_root.joinpath('maps').write_text(
        f'1000-2000 r--p 00000000 {device} {plugin_stat.st_ino} {plugin_path}\n'
        f'2000-3000 r-xp 00001000 {device} {plugin_stat.st_ino} {plugin_path}\n',
        encoding='utf-8',
    )
    return process_root


def _write_phase3_smoke_profile_fixture(
    repository: Path,
    candidate_root: Path,
    *,
    candidate_sha: str,
) -> tuple[Path, str]:
    """Write one canonical PASS profile joined to the production-shaped smoke."""
    profiler = release_module._load_repository_module(
        repository,
        'tests/phase3_smoke_host_profiler.py',
        'Phase 3 smoke host profiler fixture',
    )
    candidate_id = candidate_root.name
    plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))
    smoke = plan['smoke']
    output = (
        repository
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{candidate_id}-smoke-profile.json'
    )
    renderer_path = output.with_name(f'{candidate_id}-renderer.log')
    config = profiler.ProfileConfig(
        workspace=repository,
        candidate_root=candidate_root,
        candidate_id=candidate_id,
        build_binding=candidate_root / 'build-binding.json',
        ros_domain_id=smoke['ros_domain_id'],
        gz_partition=smoke['gz_partition'],
        startup_timeout_s=profiler.CANONICAL_STARTUP_TIMEOUT_S,
        sample_period_s=profiler.CANONICAL_SAMPLE_PERIOD_S,
        max_duration_s=profiler.CANONICAL_MAX_DURATION_S,
        renderer_log=renderer_path,
        output_path=output,
    )
    identity = profiler.collect_static_identity(
        config,
        git_reader=lambda _workspace: {'sha': candidate_sha, 'status_porcelain': ''},
        environment={'ROBOTEST_CONTACT_PROFILE': '1'},
        producer_path=repository / 'tests/phase3_smoke_host_profiler.py',
    )
    full_stack_metadata = json.loads(
        (candidate_root / 'smoke/processes/full_stack.process.json').read_text(encoding='utf-8')
    )
    pid = full_stack_metadata['pid']
    start_ticks = 100
    profile_started_epoch_ns = 1_800_000_000_000_000_000
    profile_started_monotonic_ns = 0
    sample_elapsed_ns = (
        500_000_000,
        1_000_000_000,
        1_500_000_000,
        18_000_000_000,
        18_500_000_000,
    )
    state = profiler.ProfileState()
    with tempfile.TemporaryDirectory(
        prefix='robotest-profile-proc-', dir=repository.parent
    ) as name:
        proc_root = Path(name)
        _write_phase3_profile_proc_host(
            proc_root,
            repository=repository,
            candidate_root=candidate_root,
            domain_base=config.ros_domain_id - 16,
        )
        plugin_path = Path(identity['plugin']['path'])
        process_root = _write_phase3_profile_proc_process(
            proc_root,
            pid=pid,
            start_ticks=start_ticks,
            user_ticks=10,
            domain_id=config.ros_domain_id,
            gz_partition=config.gz_partition,
            plugin_path=plugin_path,
        )
        anchor, exact_count = profiler.discover_anchor(
            proc_root,
            config.ros_domain_id,
            config.gz_partition,
            identity['plugin'],
        )
        assert anchor is not None and exact_count == 1
        for step in range(4):
            stat_text = _phase3_profile_proc_stat(
                pid,
                process_group=pid,
                start_ticks=start_ticks,
                user_ticks=10 + step * 5,
                parent_pid=40,
            )
            process_root.joinpath('stat').write_text(stat_text, encoding='ascii')
            process_root.joinpath('task', str(pid), 'stat').write_text(stat_text, encoding='ascii')
            assert profiler.capture_sample(
                proc_root,
                config,
                anchor,
                state,
                monotonic_ns=lambda: sample_elapsed_ns[len(state.samples)],
                wall_time_ns=lambda: (
                    profile_started_epoch_ns + sample_elapsed_ns[len(state.samples)]
                ),
            )
        shutil.rmtree(process_root)
        assert not profiler.capture_sample(
            proc_root,
            config,
            anchor,
            state,
            monotonic_ns=lambda: sample_elapsed_ns[len(state.samples)],
            wall_time_ns=lambda: profile_started_epoch_ns + sample_elapsed_ns[len(state.samples)],
        )

    full_stack = profiler.wait_for_full_stack_close(
        candidate_root / 'smoke',
        anchor,
        monotonic=lambda: 0.0,
        sleep=lambda _duration: None,
        profile_started_monotonic_ns=profile_started_monotonic_ns,
    )
    full_stack['pre_smoke_outputs_absent'] = True
    profile_finished_epoch_ns = profile_started_epoch_ns + (
        full_stack['finished_steady_ns'] - profile_started_monotonic_ns
    )
    contact = profiler.parse_contact_profile_logs(candidate_root / 'smoke')
    profiler.reconcile_contact_log_sources(contact, full_stack)
    contact['host_thread_bindings'] = profiler.bind_contact_profile_threads(contact, anchor, state)
    renderer_path.parent.mkdir(parents=True, exist_ok=True)
    renderer_baseline = profiler._renderer_baseline(renderer_path)
    renderer_path.write_bytes(b'Device Name: llvmpipe deterministic fixture\n')
    os.utime(
        renderer_path,
        ns=(profile_started_epoch_ns + 1, profile_started_epoch_ns + 1),
    )
    renderer = profiler.collect_renderer(renderer_path, renderer_baseline, profile_started_epoch_ns)
    renderer_path.unlink()
    document = {
        'anchor': anchor,
        'candidate': identity,
        'contact_profile': contact,
        'finished_utc': profiler._utc(profile_finished_epoch_ns),
        'full_stack_process': full_stack,
        'host_clock_ticks_per_second': 100,
        'limits': profiler._profile_limits(config),
        'process_lifecycles': profiler._process_lifecycles(state),
        'producer': profiler.PRODUCER,
        'profile_started_boot_ticks': 0,
        'profile_started_monotonic_ns': profile_started_monotonic_ns,
        'profile_started_utc': profiler._utc(profile_started_epoch_ns),
        'renderer': renderer,
        'sampling': {
            'anchor_alive_sample_count': 4,
            'cadence_overrun_count': 0,
            'exact_processes_seen_before_anchor': exact_count,
            'retained_cmdline_bytes': state.cmdline_bytes,
            'sample_count': len(state.samples),
            'samples': state.samples,
            'target_alive_sample_count': 4,
            'thread_record_count': state.thread_records,
            'totals': profiler._cpu_totals(state, 100),
        },
        'schema_version': profiler.SCHEMA_VERSION,
        'status': 'PASS',
    }
    digest = profiler.write_profile(output, document)
    binding = profiler.validate_campaign_smoke_profile(repository, candidate_root, candidate_id)
    assert binding['profile_sha256'] == digest
    return output, digest


def _phase3_fault_event(
    metrics_fixture: object,
    *,
    sequence: int,
    event_type: int,
    stamp_ns: int,
    goal_uuid: str,
    accepted_goal_stamp_ns: int,
    schedule_sha256: str,
    fault: dict | None,
    affected_message_count: int = 0,
) -> dict:
    event = metrics_fixture._fault_event(sequence, event_type, stamp_ns, goal_uuid=goal_uuid)
    event['bound_t0_ns'] = accepted_goal_stamp_ns if event_type == 7 else 0
    event['requested_t0_ns'] = accepted_goal_stamp_ns if event_type == 7 else 0
    if fault is None:
        return event
    activation = accepted_goal_stamp_ns + int(fault['start_offset_ns'])
    deactivation = activation + int(fault['duration_ns'])
    if event_type in {1, 4, 5, 7, 8, 9}:
        event.update(
            {
                'committed_fault_count': 1,
                'committed_generation': 1,
                'committed_schedule_hash': schedule_sha256,
                'configured_activation_stamp_ns': activation,
                'configured_deactivation_stamp_ns': deactivation,
                'fault_id': fault['fault_id'],
                'mode': fault['mode'],
                'seed': fault['seed'],
                'target': fault['target'],
            }
        )
    if event_type in {1, 7}:
        event.update(
            {
                'requested_fault_count': 1,
                'requested_generation': 1,
                'requested_schedule_hash': schedule_sha256,
            }
        )
    if event_type == 7:
        event['arm_margin_ns'] = 500_000_000
    if event_type in {5, 9}:
        event['actual_stamp_ns'] = stamp_ns
        event['affected_message_count'] = affected_message_count
    return event


def _positive_runtime_gate_graph_fixture() -> tuple[dict[str, dict], dict[str, dict]]:
    """Return a compact, production-shaped positive-control topic/QoS graph."""
    topic_contracts = {
        '/clock': ('BEST_EFFORT', 'VOLATILE', 1, []),
        '/robotest/cmd_vel': (
            'RELIABLE',
            'VOLATILE',
            1,
            [{'depth': 4_096, 'node': '/robotest/metrics_collector', 'side': 'subscriber'}],
        ),
        '/robotest/internal/raw_contacts': ('RELIABLE', 'VOLATILE', 64, []),
        '/robotest/validation/contacts': ('RELIABLE', 'VOLATILE', 10, []),
        '/robotest/validation/ground_truth': ('RELIABLE', 'VOLATILE', 10, []),
        '/robotest/validation/scenario_entity_poses': ('RELIABLE', 'VOLATILE', 10, []),
        '/robotest/validation/world_stats': ('RELIABLE', 'VOLATILE', 10, []),
    }
    endpoint_layout = {
        '/clock': (
            [('/robotest/parameter_bridge', 'rosgraph_msgs/msg/Clock', 1)],
            [('/robotest/contact_control_driver', 'rosgraph_msgs/msg/Clock', 1)],
        ),
        '/robotest/cmd_vel': (
            [('/robotest/contact_control_driver', 'geometry_msgs/msg/Twist', 1)],
            [
                ('/robotest/metrics_collector', 'geometry_msgs/msg/Twist', 4_096),
                ('/robotest/parameter_bridge', 'geometry_msgs/msg/Twist', 1),
            ],
        ),
        '/robotest/internal/raw_contacts': (
            [('/robotest/parameter_bridge', 'ros_gz_interfaces/msg/Contacts', 64)],
            [('/robotest/contact_stream_gate', 'ros_gz_interfaces/msg/Contacts', 64)],
        ),
        '/robotest/validation/contacts': (
            [('/robotest/contact_stream_gate', 'ros_gz_interfaces/msg/Contacts', 10)],
            [
                ('/robotest/contact_control_driver', 'ros_gz_interfaces/msg/Contacts', 10),
                ('/robotest/metrics_collector', 'ros_gz_interfaces/msg/Contacts', 10),
            ],
        ),
        '/robotest/validation/ground_truth': (
            [('/robotest/parameter_bridge', 'nav_msgs/msg/Odometry', 10)],
            [
                ('/robotest/contact_control_driver', 'nav_msgs/msg/Odometry', 10),
                ('/robotest/metrics_collector', 'nav_msgs/msg/Odometry', 10),
            ],
        ),
        '/robotest/validation/scenario_entity_poses': (
            [
                ('/robotest/parameter_bridge', 'tf2_msgs/msg/TFMessage', 10),
                ('/robotest/parameter_bridge', 'tf2_msgs/msg/TFMessage', 10),
                ('/robotest/parameter_bridge', 'tf2_msgs/msg/TFMessage', 10),
                ('/robotest/parameter_bridge', 'tf2_msgs/msg/TFMessage', 10),
            ],
            [('/robotest/contact_control_driver', 'tf2_msgs/msg/TFMessage', 10)],
        ),
        '/robotest/validation/world_stats': (
            [
                (
                    '/robotest/parameter_bridge',
                    'ros_gz_interfaces/msg/WorldStatistics',
                    10,
                )
            ],
            [
                (
                    '/robotest/metrics_collector',
                    'ros_gz_interfaces/msg/WorldStatistics',
                    10,
                )
            ],
        ),
    }
    gid_counter = 1
    exact_contract: dict[str, dict] = {}
    topics: dict[str, dict] = {}
    for topic, (reliability, durability, depth, overrides) in topic_contracts.items():
        expected = {
            'depth': depth,
            'durability': durability,
            'endpoint_depth_overrides': copy.deepcopy(overrides),
            'history': 'KEEP_LAST',
            'reliability': reliability,
        }
        exact_contract[topic] = copy.deepcopy(expected)
        publishers = []
        subscribers = []
        qos_checks = []
        for side, layout, target in (
            ('publisher', endpoint_layout[topic][0], publishers),
            ('subscriber', endpoint_layout[topic][1], subscribers),
        ):
            for node, topic_type, endpoint_depth in layout:
                target.append(
                    {
                        'depth': endpoint_depth,
                        'durability': durability,
                        'gid': f'{gid_counter:032x}',
                        'history': 'KEEP_LAST',
                        'node': node,
                        'reliability': reliability,
                        'topic_type': topic_type,
                    }
                )
                gid_counter += 1
                qos_checks.append(
                    {
                        'bounded_depth_live_proven': True,
                        'exact_depth_live_proven': True,
                        'expected_depth': endpoint_depth,
                        'explicit_keep_all': False,
                        'introspection_complete': True,
                        'node': node,
                        'policy_contract_pass': True,
                        'side': side,
                    }
                )
        topics[topic] = {
            'bounded_depth_live_proven': True,
            'exact_depth_live_proven': True,
            'expected': copy.deepcopy(expected),
            'publisher_qos_pass': True,
            'publishers': sorted(publishers, key=lambda item: (item['node'], item['topic_type'])),
            'qos_checks': sorted(qos_checks, key=lambda item: (item['side'], item['node'])),
            'qos_introspection_complete': True,
            'subscriber_qos_pass': True,
            'subscribers': sorted(
                subscribers,
                key=lambda item: (item['node'], item['topic_type']),
            ),
        }
    return exact_contract, topics


def _candidate_runtime_gate_graph_fixture(runtime_gate: object) -> dict[str, dict]:
    """Build all producer-contract topics with factual Fast DDS UNKNOWN/0 QoS."""
    topic_types = {
        '/clock': 'rosgraph_msgs/msg/Clock',
        '/robotest/cmd_vel': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_behavior_unused': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_nav': 'geometry_msgs/msg/Twist',
        '/robotest/cmd_vel_smoothed': 'geometry_msgs/msg/Twist',
        '/robotest/collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState',
        '/robotest/faults/events': 'robotest_interfaces/msg/FaultEvent',
        '/robotest/imu': 'sensor_msgs/msg/Imu',
        '/robotest/internal/raw_contacts': runtime_gate.CONTACT_MESSAGE_TYPE,
        '/robotest/map': 'nav_msgs/msg/OccupancyGrid',
        '/robotest/navigation/plan': 'nav_msgs/msg/Path',
        '/robotest/odom': 'nav_msgs/msg/Odometry',
        '/robotest/raw/imu': 'sensor_msgs/msg/Imu',
        '/robotest/raw/odom': 'nav_msgs/msg/Odometry',
        '/robotest/raw/scan': 'sensor_msgs/msg/LaserScan',
        '/robotest/scan': 'sensor_msgs/msg/LaserScan',
        '/robotest/validation/contacts': runtime_gate.CONTACT_MESSAGE_TYPE,
        '/robotest/validation/ground_truth': 'nav_msgs/msg/Odometry',
        '/robotest/validation/scenario_entity_poses': 'tf2_msgs/msg/TFMessage',
        '/robotest/validation/world_stats': 'ros_gz_interfaces/msg/WorldStatistics',
        '/tf': 'tf2_msgs/msg/TFMessage',
        '/tf_static': 'tf2_msgs/msg/TFMessage',
    }
    assert set(topic_types) == set(runtime_gate.QOS_CONTRACTS)
    expected_subscribers = {
        '/clock': set(runtime_gate.CANDIDATE_FUSED_CLOCK_SUBSCRIBERS),
        '/robotest/internal/raw_contacts': {'/robotest/contact_stream_gate'},
        '/robotest/validation/contacts': {'/robotest/metrics_collector'},
        '/robotest/validation/ground_truth': {'/robotest/metrics_collector'},
        '/robotest/validation/scenario_entity_poses': {'/robotest/metrics_collector'},
        '/robotest/validation/world_stats': {'/robotest/metrics_collector'},
        **runtime_gate.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS,
        **runtime_gate.CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS,
    }
    gid_counter = 1

    def endpoint(topic: str, side: str, node: str) -> dict:
        nonlocal gid_counter
        reliability, durability, _depth = runtime_gate._endpoint_qos_contract(
            topic,
            side,
            node,
        )
        result = {
            'depth': 0,
            'durability': durability,
            'gid': f'{gid_counter:032x}',
            'history': 'UNKNOWN',
            'node': node,
            'reliability': reliability,
            'topic_type': topic_types[topic],
        }
        gid_counter += 1
        return result

    topics: dict[str, dict] = {}
    for topic in runtime_gate.QOS_CONTRACTS:
        authoritative = runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS.get(topic)
        if authoritative is None:
            publisher_nodes = sorted(runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS[topic])
        else:
            publisher_nodes = sorted(authoritative[0]) * authoritative[2]
        publishers = sorted(
            [endpoint(topic, 'publisher', node) for node in publisher_nodes],
            key=lambda item: (item['node'], item['topic_type']),
        )
        subscribers = sorted(
            [
                endpoint(topic, 'subscriber', node)
                for node in sorted(expected_subscribers.get(topic, set()))
            ],
            key=lambda item: (item['node'], item['topic_type']),
        )
        checks = []
        for side, endpoint_records in (
            ('publisher', publishers),
            ('subscriber', subscribers),
        ):
            checks.extend(
                {
                    **runtime_gate._qos_status(
                        record,
                        runtime_gate._endpoint_qos_contract(topic, side, record['node']),
                    ),
                    'node': record['node'],
                    'side': side,
                }
                for record in endpoint_records
            )
        checks.sort(key=lambda item: (item['side'], item['node']))
        reliability, durability, depth = runtime_gate.QOS_CONTRACTS[topic]
        expected = {
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
        topics[topic] = {
            'bounded_depth_live_proven': bool(checks)
            and all(item['bounded_depth_live_proven'] for item in checks),
            'exact_depth_live_proven': bool(checks)
            and all(item['exact_depth_live_proven'] for item in checks),
            'expected': expected,
            'publisher_qos_pass': bool(publishers)
            and all(item['policy_contract_pass'] for item in checks if item['side'] == 'publisher'),
            'publishers': publishers,
            'qos_checks': checks,
            'qos_introspection_complete': bool(checks)
            and all(item['introspection_complete'] for item in checks),
            'subscriber_qos_pass': all(
                item['policy_contract_pass'] for item in checks if item['side'] == 'subscriber'
            ),
            'subscribers': subscribers,
        }
    authoritative_ownership = runtime_gate._authoritative_publisher_ownership(topics)
    publisher_ownership = {
        topic: (
            authoritative_ownership[topic]
            if topic in authoritative_ownership
            else runtime_gate._exact_endpoint_owners(
                topics[topic]['publishers'],
                expected_nodes,
                expected_type=(
                    runtime_gate.CONTACT_MESSAGE_TYPE
                    if topic in runtime_gate.CONTACT_TOPICS
                    else None
                ),
            )
        )
        for topic, expected_nodes in runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS.items()
    }
    command_subscriber_ownership = {
        topic: runtime_gate._exact_endpoint_owners(
            topics[topic]['subscribers'],
            expected_nodes,
        )
        for topic, expected_nodes in runtime_gate.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS.items()
    }
    contact_subscriber_ownership = {
        topic: runtime_gate._exact_endpoint_owners(
            topics[topic]['subscribers'],
            expected_nodes,
            expected_type=runtime_gate.CONTACT_MESSAGE_TYPE,
        )
        for topic, expected_nodes in runtime_gate.CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS.items()
    }
    assert all(publisher_ownership.values())
    assert all(command_subscriber_ownership.values())
    assert all(contact_subscriber_ownership.values())
    fused_clock_subscriber_ownership_pass = runtime_gate._fused_clock_subscriber_ownership(
        topics['/clock']
    )
    assert fused_clock_subscriber_ownership_pass
    return {
        'command_subscriber_ownership': command_subscriber_ownership,
        'contact_subscriber_ownership': contact_subscriber_ownership,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in topics.items()
        },
        'fused_clock_subscriber_ownership_pass': fused_clock_subscriber_ownership_pass,
        'publisher_ownership': publisher_ownership,
        'topics': topics,
    }


def _bounded_process_fixture(
    *,
    role: str,
    command: list[str],
    cwd: Path,
    pid: int,
    started_steady_ns: int,
    finished_steady_ns: int,
    wall_timeout_s: float,
    returncode: int = 0,
) -> dict:
    timeout_text = f'{wall_timeout_s:.3f}s'
    stream = {
        'error': None,
        'maximum_bytes': 8 * 1024 * 1024,
        'observed_bytes': 0,
        'overflow': False,
        'retained_bytes': 0,
    }
    return {
        'command': list(command),
        'cwd': str(cwd),
        'finished_steady_ns': finished_steady_ns,
        'group_confirmed_empty': True,
        'pgid': pid,
        'pid': pid,
        'returncode': returncode,
        'role': role,
        'started_steady_ns': started_steady_ns,
        'stderr': copy.deepcopy(stream),
        'stdout': copy.deepcopy(stream),
        'timed_out': False,
        'wall_timeout_s': wall_timeout_s,
        'wrapped_command': [
            'timeout',
            '--signal=TERM',
            '--kill-after=10s',
            timeout_text,
            *command,
        ],
    }


def _write_bounded_process_artifacts(
    directory: Path,
    process: dict,
    *,
    stdout_bytes: bytes = b'',
    stderr_bytes: bytes = b'',
) -> None:
    """Write one production-shaped process record and its retained log files."""
    role = process['role']
    process_directory = directory / 'processes'
    process_directory.mkdir(parents=True, exist_ok=True)
    payloads = {'stderr': stderr_bytes, 'stdout': stdout_bytes}
    for stream_name, payload in payloads.items():
        stream = process[stream_name]
        assert isinstance(payload, bytes)
        assert len(payload) <= stream['maximum_bytes']
        stream['observed_bytes'] = len(payload)
        stream['retained_bytes'] = len(payload)
        (process_directory / f'{role}.{stream_name}.log').write_bytes(payload)
    _canonical_file(process_directory / f'{role}.process.json', process)


def _phase3_lifecycle_replay_contract(repository: Path, run_root: Path) -> dict[str, object]:
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 lifecycle replay fixture contract',
    )
    full_stack = json.loads(
        (run_root / 'processes/full_stack.process.json').read_text(encoding='utf-8')
    )
    return {
        'expected_lifecycle_watch_pid': full_stack['pid'],
        'expected_lifecycle_wall_timeout_s': 110.0,
        'expected_lifecycle_nodes': tuple(orchestration.REQUIRED_LIFECYCLE_NODES),
    }


def _write_phase3_lifecycle_ready(
    repository: Path,
    run_root: Path,
    document: dict,
) -> None:
    source = release_module._load_repository_module(
        repository,
        'tests/phase2_startup_gate.py',
        'Phase 3 lifecycle-ready fixture serializer',
    )
    (run_root / 'lifecycle-ready.json').write_text(
        source.serialize_result(document),
        encoding='utf-8',
    )


def _write_phase3_final_launch_log_gate(
    repository: Path,
    run_root: Path,
    *,
    launch_stopped_utc: str = '2026-08-26T00:00:00Z',
    scanned_utc: str = '2026-08-26T00:00:01Z',
    expected_verdict: str = 'PASS',
) -> dict:
    """Write production-shaped combined launch-log bytes and canonical scan evidence."""
    source = release_module._load_repository_module(
        repository,
        'tests/phase2_startup_gate.py',
        'Phase 3 final launch-log fixture scanner',
    )
    combined_path = run_root / 'full-stack-combined.log'
    combined_path.write_bytes(
        source.combined_launch_log_bytes(
            run_root / 'processes/full_stack.stdout.log',
            run_root / 'processes/full_stack.stderr.log',
            maximum_stream_bytes=8 * 1024 * 1024,
        )
    )
    gate = source.scan_final_launch_log(
        combined_path.resolve(strict=True),
        launch_stopped_utc,
        scanned_utc=scanned_utc,
        lifecycle_ready_path=(run_root / 'lifecycle-ready.json').resolve(strict=True),
        **_phase3_lifecycle_replay_contract(repository, run_root),
    )
    assert gate['verdict'] == expected_verdict, gate
    _canonical_file(run_root / 'full-stack-final-log-gate.json', gate, sidecar=True)
    return gate


def _append_phase3_full_stack_log(
    run_root: Path,
    payload: bytes,
    *,
    stream_name: str = 'stdout',
) -> None:
    """Append retained full-stack bytes and keep its process counters exact."""
    stream_path = run_root / f'processes/full_stack.{stream_name}.log'
    retained = stream_path.read_bytes() + payload
    stream_path.write_bytes(retained)
    process_path = run_root / 'processes/full_stack.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    process[stream_name]['observed_bytes'] = len(retained)
    process[stream_name]['retained_bytes'] = len(retained)
    _canonical_file(process_path, process)


def _phase3_recovered_lifecycle_timeout_line(node_name: str) -> bytes:
    return (
        f'\n[{node_name}-15] [WARN] [1.000000000] [robotest.{node_name}.rclcpp]: '
        f'failed to send response to /robotest/{node_name}/get_state (timeout): '
        'client will not receive response\n'
    ).encode()


def _write_phase3_recovered_lifecycle_timeout_gate(
    repository: Path,
    run_root: Path,
    *,
    node_name: str = 'collision_monitor',
) -> dict:
    """Write one source-valid recovered lifecycle timeout and its v2 gate."""
    lifecycle_path = run_root / 'lifecycle-ready.json'
    lifecycle = json.loads(lifecycle_path.read_text(encoding='utf-8'))
    lifecycle['states'][node_name].update({'attempts': 2, 'timed_out_attempts': 1})
    _write_phase3_lifecycle_ready(repository, run_root, lifecycle)
    _append_phase3_full_stack_log(
        run_root,
        _phase3_recovered_lifecycle_timeout_line(node_name),
    )
    return _write_phase3_final_launch_log_gate(repository, run_root)


def _fixture_contact_gate_cmdline_sha256(installed_path: Path) -> str:
    """Model launch_ros argv without claiming its ephemeral params path is replayable."""
    argv = (
        str(installed_path),
        '--ros-args',
        '-r',
        '__node:=contact_stream_gate',
        '-r',
        '__ns:=/robotest',
        '--params-file',
        str(installed_path.parent / 'phase5-fixture-launch-params.yaml'),
    )
    return hashlib.sha256(('\0'.join(argv) + '\0').encode()).hexdigest()


def _fixture_contact_aggregator_mapping_identity(
    installed_path: Path,
    *,
    device: int,
    inode: int,
    orchestration: object,
) -> tuple[int, str]:
    """Model and hash a realistic ELF mapping using the production record shape."""
    base_address = 0x7F00_0000_0000
    segment_layout = (
        ('r--p', 0x0000),
        ('r-xp', 0x1000),
        ('r--p', 0x2000),
        ('r--p', 0x3000),
        ('rw-p', 0x4000),
    )
    records = [
        {
            'address_end': base_address + ((index + 1) * 0x1000),
            'address_start': base_address + (index * 0x1000),
            'device': device,
            'inode': inode,
            'offset': offset,
            'path': str(installed_path),
            'permissions': permissions,
        }
        for index, (permissions, offset) in enumerate(segment_layout)
    ]
    return len(records), orchestration.canonical_sha256(records)


def _write_phase3_contact_gate_reobservation(
    repository: Path,
    directory: Path,
    *,
    orchestration: object,
    build_binding: dict,
    ros_domain_id: int,
    gz_partition: str,
    mode: str = 'candidate',
) -> None:
    frozen = build_binding['contact_gate_binary']
    frozen_aggregator = build_binding['contact_aggregator_binary']
    runtime_gate_source = release_module._load_repository_module(
        repository,
        'tests/phase3_runtime_gate.py',
        'Phase 3 runtime-gate fixture source contract',
    )
    installed_path = (repository / frozen['installed_path']).resolve(strict=True)
    installed_stat = installed_path.stat()
    aggregator_installed_path = (repository / frozen_aggregator['installed_path']).resolve(
        strict=True
    )
    aggregator_installed_stat = aggregator_installed_path.stat()
    launch_pid = 40_000 + ros_domain_id
    attestation = {
        'build_embedded_source_inventory_match': True,
        'build_embedded_source_inventory_sha256': frozen['build_embedded_source_inventory_sha256'],
        'build_elf_build_id': frozen['build_elf_build_id'],
        'build_install_build_id_match': True,
        'build_install_samefile': frozen['build_install_samefile'],
        'build_install_sha256_match': True,
        'build_path': frozen['build_path'],
        'build_regular_executable': True,
        'build_sha256': frozen['build_sha256'],
        'exact_live_process_count': 1,
        'identity_revalidated_after_hashing': True,
        'installed_declared_path': frozen['installed_declared_path'],
        'installed_declared_is_symlink': frozen['installed_declared_is_symlink'],
        'installed_declared_samefile': True,
        'installed_device': installed_stat.st_dev,
        'installed_embedded_source_inventory_match': True,
        'installed_embedded_source_inventory_sha256': frozen[
            'installed_embedded_source_inventory_sha256'
        ],
        'installed_elf_build_id': frozen['installed_elf_build_id'],
        'installed_inode': installed_stat.st_ino,
        'installed_path': frozen['installed_path'],
        'installed_regular_executable': True,
        'installed_sha256': frozen['installed_sha256'],
        'launch_root_pid': launch_pid,
        'live_cmdline_sha256': _fixture_contact_gate_cmdline_sha256(installed_path),
        'live_device': installed_stat.st_dev,
        'live_elf_build_id': frozen['installed_elf_build_id'],
        'live_embedded_source_inventory_match': True,
        'live_embedded_source_inventory_sha256': frozen[
            'installed_embedded_source_inventory_sha256'
        ],
        'live_executable_link': str(installed_path),
        'live_executable_path': str(installed_path),
        'live_executable_sha256': frozen['installed_sha256'],
        'live_inode': installed_stat.st_ino,
        'live_installed_build_id_match': True,
        'live_installed_inode_match': True,
        'live_installed_sha256_match': True,
        'live_pgid': launch_pid,
        'live_pid': launch_pid + 1,
        'live_ppid': launch_pid,
        'live_sid': launch_pid,
        'live_size_bytes': installed_stat.st_size,
        'live_start_ticks': 1_000_000 + ros_domain_id,
        'observed_gz_partition': gz_partition,
        'observed_ros_domain_id': str(ros_domain_id),
        'package': 'robotest_sim',
        'process_identity_match': True,
        'schema_version': 1,
        'source_inventory_sha256': frozen['source_inventory_sha256'],
        'verdict': 'PASS',
    }
    live_executable_path = str(Path(sys.executable).resolve(strict=True))
    live_mapping_paths = [str(aggregator_installed_path)]
    live_mapping_count, live_mapping_fingerprint_sha256 = (
        _fixture_contact_aggregator_mapping_identity(
            aggregator_installed_path,
            device=aggregator_installed_stat.st_dev,
            inode=aggregator_installed_stat.st_ino,
            orchestration=orchestration,
        )
    )
    aggregator_attestation = {
        **frozen_aggregator,
        'attestation_method': 'proc_maps_exact_device_inode',
        'exact_live_process_count': 1,
        'identity_revalidated_after_hashing': True,
        'installed_device': aggregator_installed_stat.st_dev,
        'installed_identity_revalidated_after_hashing': True,
        'installed_inode': aggregator_installed_stat.st_ino,
        'installed_size_bytes': aggregator_installed_stat.st_size,
        'launch_root_pid': launch_pid,
        'live_cmdline_sha256': hashlib.sha256(
            f'{live_executable_path}\0-r\0robotest_lab.sdf\0'.encode()
        ).hexdigest(),
        'live_elf_build_id': frozen_aggregator['installed_elf_build_id'],
        'live_embedded_source_inventory_match': True,
        'live_embedded_source_inventory_sha256': frozen_aggregator[
            'installed_embedded_source_inventory_sha256'
        ],
        'live_executable_link': live_executable_path,
        'live_executable_path': live_executable_path,
        'live_installed_build_id_match': True,
        'live_installed_inode_match': True,
        'live_installed_sha256_match': True,
        'live_mapping_count': live_mapping_count,
        'live_mapping_device': aggregator_installed_stat.st_dev,
        'live_mapping_fingerprint_sha256': live_mapping_fingerprint_sha256,
        'live_mapping_has_executable': True,
        'live_mapping_has_offset_zero': True,
        'live_mapping_inode': aggregator_installed_stat.st_ino,
        'live_mapping_paths': live_mapping_paths,
        'live_pgid': launch_pid,
        'live_pid': launch_pid + 2,
        'live_ppid': launch_pid,
        'live_sid': launch_pid,
        'live_start_ticks': 2_000_000 + ros_domain_id,
        'maps_revalidated_after_hashing': True,
        'observed_gz_partition': gz_partition,
        'observed_ros_domain_id': str(ros_domain_id),
        'process_identity_match': True,
        'verdict': 'PASS',
    }
    stable_identity = {
        field: aggregator_attestation[field]
        for field in orchestration.CONTACT_AGGREGATOR_STABLE_IDENTITY_FIELDS
    }
    aggregator_attestation['stable_identity'] = stable_identity
    aggregator_attestation['stable_identity_sha256'] = orchestration.canonical_sha256(
        stable_identity
    )
    gate_document = {
        'contact_aggregator_binary_attestation': aggregator_attestation,
        'contact_gate_binary_attestation': attestation,
        'verdict': 'PASS',
    }
    if mode == 'positive_control':
        exact_static_qos, topics = _positive_runtime_gate_graph_fixture()
        gate_document.update(
            {
                'attempt_count': 1,
                'authoritative_publisher_ownership': {
                    '/clock': True,
                    '/robotest/validation/ground_truth': True,
                    '/robotest/validation/scenario_entity_poses': True,
                    '/robotest/validation/world_stats': True,
                },
                'bounded_depth_live_proven_for_all_endpoints': True,
                'cmd_vel_owner_pass': True,
                'cmd_vel_subscriber_ownership_pass': True,
                'contact_publisher_ownership': {
                    '/robotest/internal/raw_contacts': True,
                    '/robotest/validation/contacts': True,
                },
                'contact_subscriber_ownership': {
                    '/robotest/internal/raw_contacts': True,
                },
                'elapsed_wall_s': 0.5,
                'exact_static_qos_depth_contract': exact_static_qos,
                'forbidden_nodes_present': [],
                'mode': 'positive_control',
                'namespace_isolation_pass': True,
                'nodes': [
                    '/robotest/contact_control_driver',
                    '/robotest/contact_stream_gate',
                    '/robotest/evidence/phase3_runtime_gate',
                    '/robotest/fault_proxy',
                    '/robotest/metrics_collector',
                    '/robotest/parameter_bridge',
                    '/robotest/robot_state_publisher',
                    '/robotest/scenario_bridge',
                ],
                'producer': 'robotest_phase3/runtime_gate',
                'qos_contract_pass': True,
                'qos_introspection_complete': True,
                'required_nodes_missing': [],
                'scenario_services_missing': [],
                'schema_version': 1,
                'topics': topics,
                'validation_autonomy_isolation_pass': True,
            }
        )
    else:
        candidate_graph = _candidate_runtime_gate_graph_fixture(runtime_gate_source)
        topics = candidate_graph['topics']
        candidate_nodes = sorted(
            {
                '/robotest/evidence/phase3_runtime_gate',
                *(f'/robotest/{name}' for name in runtime_gate_source.CANDIDATE_REQUIRED_NODES),
            }
        )
        gate_document.update(
            {
                'attempt_count': 1,
                'autonomy_validation_leaks': [],
                'bounded_depth_live_proven_for_all_endpoints': all(
                    evidence['bounded_depth_live_proven'] for evidence in topics.values()
                ),
                'cmd_vel_owner_pass': True,
                'command_subscriber_ownership': candidate_graph['command_subscriber_ownership'],
                'contact_subscriber_ownership': candidate_graph['contact_subscriber_ownership'],
                'elapsed_wall_s': 0.5,
                'exact_static_qos_depth_contract': candidate_graph[
                    'exact_static_qos_depth_contract'
                ],
                'fused_clock_subscriber_ownership_pass': candidate_graph[
                    'fused_clock_subscriber_ownership_pass'
                ],
                'legacy_fault_service_absent': True,
                'mode': 'candidate',
                'namespace_isolation_pass': True,
                'nodes': candidate_nodes,
                'producer': 'robotest_phase3/runtime_gate',
                'publisher_ownership': candidate_graph['publisher_ownership'],
                'qos_contract_pass': True,
                'qos_introspection_complete': all(
                    evidence['qos_introspection_complete'] for evidence in topics.values()
                ),
                'required_nodes_missing': [],
                'required_nodes_outside_namespace': [],
                'scenario_services_missing': [],
                'schema_version': 1,
                'topics': topics,
                'validation_autonomy_isolation_pass': True,
            }
        )
    initial_path = directory / 'runtime-gate.json'
    final_path = directory / 'contact-stream-final-gate.json'
    _canonical_file(initial_path, gate_document, sidecar=True)
    final_document = gate_document
    if mode in {'positive_control', 'candidate'}:
        final_topics = {
            topic: copy.deepcopy(gate_document['topics'][topic])
            for topic in (
                '/robotest/internal/raw_contacts',
                '/robotest/validation/contacts',
            )
        }
        public_contacts = final_topics['/robotest/validation/contacts']
        public_contacts['subscribers'] = [
            endpoint
            for endpoint in public_contacts['subscribers']
            if endpoint['node'] == '/robotest/metrics_collector'
        ]
        public_contacts['qos_checks'] = [
            check
            for check in public_contacts['qos_checks']
            if check['side'] == 'publisher' or check['node'] == '/robotest/metrics_collector'
        ]
        final_document = {
            'attempt_count': 1,
            'contact_aggregator_binary_attestation': copy.deepcopy(aggregator_attestation),
            'contact_gate_binary_attestation': copy.deepcopy(attestation),
            'contact_stream_gate_present': True,
            'elapsed_wall_s': 0.5,
            'mode': 'contact_stream',
            'nodes': sorted(
                [
                    '/robotest/contact_stream_gate',
                    '/robotest/evidence/phase3_runtime_gate',
                    '/robotest/fault_proxy',
                    '/robotest/metrics_collector',
                    '/robotest/parameter_bridge',
                    '/robotest/robot_state_publisher',
                ]
            ),
            'producer': 'robotest_phase3/runtime_gate',
            'publisher_ownership': {
                '/robotest/internal/raw_contacts': True,
                '/robotest/validation/contacts': True,
            },
            'qos_contract_pass': True,
            'raw_subscriber_ownership': True,
            'schema_version': 1,
            'topics': final_topics,
            'verdict': 'PASS',
        }
    _canonical_file(final_path, final_document, sidecar=True)
    _canonical_file(
        directory / 'contact-gate-revalidation.json',
        orchestration.reconcile_contact_gate_reobservation(
            initial_path,
            final_path,
            build_binding=build_binding,
            expected_domain_id=ros_domain_id,
            expected_gz_partition=gz_partition,
        ),
        sidecar=True,
    )


def _write_phase3_positive_handshake(
    repository: Path,
    directory: Path,
    *,
    orchestration: object,
    build_binding: dict,
    positive_control: dict,
    ros_domain_id: int,
    gz_partition: str,
) -> None:
    """Create READY, gate/process, ARM, and ACK evidence in causal order."""
    control_configuration = positive_control['configuration']['control_configuration']
    protocol = control_configuration['arm_protocol']
    ready_steady_ns = 1_000_000
    ready_document = {
        'arm_protocol': copy.deepcopy(protocol),
        'arm_protocol_sha256': release_module._canonical_sha256(protocol),
        'control_configuration_sha256': positive_control['configuration'][
            'control_configuration_sha256'
        ],
        'expected_pair': copy.deepcopy(positive_control['configuration']['expected_pair']),
        'fixture_sha256': positive_control['configuration']['fixture_sha256'],
        'identity': {
            'fixture_id': positive_control['identity']['fixture_id'],
            'run_id': positive_control['identity']['run_id'],
        },
        'observed_robot_start': copy.deepcopy(positive_control['control']['observed_robot_start']),
        'observed_wall': copy.deepcopy(positive_control['control']['setup']['observed_wall']),
        'producer': 'robotest_scenarios/contact_control_driver',
        'ready_steady_ns': ready_steady_ns,
        'resolved_names': {
            'clock': '/clock',
            'cmd_vel': '/robotest/cmd_vel',
            'contacts': '/robotest/validation/contacts',
            'entity_pose': '/robotest/validation/scenario_entity_poses',
            'ground_truth': '/robotest/validation/ground_truth',
        },
        'schema_version': 1,
        'spawn': copy.deepcopy(positive_control['control']['setup']['spawn']),
    }
    ready_path = directory / 'contact-control.ready.json'
    _canonical_file(ready_path, ready_document)

    _write_phase3_contact_gate_reobservation(
        repository,
        directory,
        orchestration=orchestration,
        build_binding=build_binding,
        ros_domain_id=ros_domain_id,
        gz_partition=gz_partition,
        mode='positive_control',
    )
    runtime_gate_path = directory / 'runtime-gate.json'
    runtime_gate_started_ns = 2_000_000
    runtime_gate_finished_ns = 3_000_000
    sim_pid = 40_000 + ros_domain_id
    driver_pid = 41_000 + ros_domain_id
    runtime_gate_pid = 51_000 + ros_domain_id
    runtime_gate_command = [
        'python3',
        str(repository / 'tests/phase3_runtime_gate.py'),
        '--mode',
        'positive-control',
        '--output',
        str(runtime_gate_path),
        '--workspace',
        str(repository),
        '--wall-timeout-s',
        '20.0',
        '--watch-pid',
        str(driver_pid),
        '--launch-pid',
        str(sim_pid),
        '--expected-domain-id',
        str(ros_domain_id),
        '--expected-gz-partition',
        gz_partition,
    ]
    _write_bounded_process_artifacts(
        directory,
        _bounded_process_fixture(
            role='runtime_gate',
            command=runtime_gate_command,
            cwd=repository,
            pid=runtime_gate_pid,
            started_steady_ns=runtime_gate_started_ns,
            finished_steady_ns=runtime_gate_finished_ns,
            wall_timeout_s=25.0,
        ),
    )

    arm_request = orchestration.build_contact_control_arm_request(
        ready_document,
        ready_sha256=phase5_module.file_sha256(ready_path),
        runtime_gate_sha256=phase5_module.file_sha256(runtime_gate_path),
        arm_requested_steady_ns=4_000_000,
    )
    arm_request_path = directory / 'contact-control.arm.json'
    _canonical_file(arm_request_path, arm_request)
    arm_request_sha256 = phase5_module.file_sha256(arm_request_path)
    command_progress = {
        'angular_z_rad_s': 0.0,
        'linear_x_m_s': 0.0,
        'linear_y_m_s': 0.0,
        'observed_steady_ns': 5_300_000,
        'producer': 'robotest_metrics/metrics_collector',
        'public_topic': '/robotest/cmd_vel',
        'retained_command_count': 1,
        'run_id': arm_request['run_id'],
        'schema_version': 1,
        'stamp_ns': 840_000_000,
    }
    command_progress_path = directory / 'command-progress.json'
    _canonical_file(command_progress_path, command_progress)
    acknowledgment = {
        'arm_observed_clock_sample_count': 10,
        'arm_observed_sim_stamp_ns': 800_000_000,
        'arm_observed_steady_ns': 5_000_000,
        'arm_protocol_sha256': ready_document['arm_protocol_sha256'],
        'arm_request_sha256': arm_request_sha256,
        'arm_requested_steady_ns': arm_request['arm_requested_steady_ns'],
        'armed_clock_sample_count': 11,
        'armed_sim_stamp_ns': 900_000_000,
        'armed_steady_ns': 6_000_000,
        'command_delivery_probe': {
            'collector_progress_observed_steady_ns': command_progress['observed_steady_ns'],
            'collector_progress_sha256': phase5_module.file_sha256(command_progress_path),
            'collector_progress_stamp_ns': command_progress['stamp_ns'],
            'matched_subscription_count': 2,
            'match_observed_steady_ns': 5_100_000,
            'probe_publish_returned_steady_ns': 5_400_000,
            'probe_publish_started_steady_ns': 5_200_000,
            'probe_sim_stamp_ns': 850_000_000,
            'required_subscription_count': 2,
        },
        'producer': protocol['ack_producer'],
        'ready_sha256': arm_request['ready_sha256'],
        'run_id': arm_request['run_id'],
        'runtime_gate_sha256': arm_request['runtime_gate_sha256'],
        'schema_version': protocol['schema_version'],
    }
    armed_path = directory / 'contact-control.armed.json'
    _canonical_file(armed_path, acknowledgment)

    positive_control['control']['arm'] = {
        'acknowledgment': copy.deepcopy(acknowledgment),
        'acknowledgment_sha256': phase5_module.file_sha256(armed_path),
        'first_nonzero_publish_returned_steady_ns': 8_000_000,
        'first_nonzero_publish_started_steady_ns': 7_000_000,
        'request': copy.deepcopy(arm_request),
        'request_sha256': arm_request_sha256,
    }
    positive_control['control']['timeline']['control_started_steady_ns'] = 7_000_000
    _write_bounded_process_artifacts(
        directory,
        _bounded_process_fixture(
            role='sim_launch',
            command=[
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
            cwd=repository,
            pid=sim_pid,
            started_steady_ns=100_000,
            finished_steady_ns=10_000_000,
            wall_timeout_s=120.0,
            returncode=-15,
        ),
    )
    _write_bounded_process_artifacts(
        directory,
        _bounded_process_fixture(
            role='contact_control_driver',
            command=[
                'ros2',
                'run',
                'robotest_scenarios',
                'contact_control_driver',
                '--output',
                str(directory / 'contact-control-result.json'),
                '--ready-file',
                str(ready_path),
                '--arm-file',
                str(arm_request_path),
                '--armed-file',
                str(armed_path),
                '--command-progress-file',
                str(command_progress_path),
                '--run-id',
                positive_control['identity']['run_id'],
                '--coverage-manifest',
                str(repository / 'config/collision-coverage.yaml'),
                '--wall-timeout-s',
                '30.0',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ],
            cwd=repository,
            pid=driver_pid,
            started_steady_ns=500_000,
            finished_steady_ns=9_000_000,
            wall_timeout_s=45.0,
        ),
    )
    runtime_gate_script = str(repository / 'tests/phase3_runtime_gate.py')
    process_fixtures = (
        _bounded_process_fixture(
            role='domain_preflight',
            command=[
                'python3',
                runtime_gate_script,
                '--mode',
                'empty',
                '--output',
                str(directory / 'domain-preflight.json'),
                '--workspace',
                str(repository),
                '--wall-timeout-s',
                '10.0',
            ],
            cwd=repository,
            pid=42_000 + ros_domain_id,
            started_steady_ns=10_000,
            finished_steady_ns=50_000,
            wall_timeout_s=15.0,
        ),
        _bounded_process_fixture(
            role='partition_preflight',
            command=['gz', 'topic', '-l'],
            cwd=repository,
            pid=43_000 + ros_domain_id,
            started_steady_ns=60_000,
            finished_steady_ns=90_000,
            wall_timeout_s=15.0,
        ),
        _bounded_process_fixture(
            role='metrics_collector',
            command=[
                'ros2',
                'run',
                'robotest_metrics',
                'metrics_collector',
                '--output',
                str(directory / 'capture.json'),
                '--ready-file',
                str(directory / 'metrics.ready.json'),
                '--stop-file',
                str(directory / 'metrics.stop'),
                '--contact-progress-file',
                str(directory / 'contact-progress.json'),
                '--command-progress-file',
                str(command_progress_path),
                '--command-progress-run-id',
                positive_control['identity']['run_id'],
                '--wall-timeout-s',
                '360',
                '--ros-args',
                '-r',
                '__ns:=/robotest',
            ],
            cwd=repository,
            pid=44_000 + ros_domain_id,
            started_steady_ns=200_000,
            finished_steady_ns=9_500_000,
            wall_timeout_s=370.0,
        ),
        _bounded_process_fixture(
            role='contact_stream_final_gate',
            command=[
                'python3',
                runtime_gate_script,
                '--mode',
                'contact-stream',
                '--output',
                str(directory / 'contact-stream-final-gate.json'),
                '--workspace',
                str(repository),
                '--wall-timeout-s',
                '10.0',
                '--watch-pid',
                str(sim_pid),
                '--launch-pid',
                str(sim_pid),
                '--expected-domain-id',
                str(ros_domain_id),
                '--expected-gz-partition',
                gz_partition,
            ],
            cwd=repository,
            pid=52_000 + ros_domain_id,
            started_steady_ns=8_200_000,
            finished_steady_ns=8_400_000,
            wall_timeout_s=15.0,
        ),
        _bounded_process_fixture(
            role='domain_cleanup',
            command=[
                'python3',
                runtime_gate_script,
                '--mode',
                'empty',
                '--output',
                str(directory / 'domain-cleanup.json'),
                '--workspace',
                str(repository),
                '--wall-timeout-s',
                '15.0',
            ],
            cwd=repository,
            pid=53_000 + ros_domain_id,
            started_steady_ns=10_100_000,
            finished_steady_ns=10_200_000,
            wall_timeout_s=20.0,
        ),
        _bounded_process_fixture(
            role='partition_cleanup',
            command=['gz', 'topic', '-l'],
            cwd=repository,
            pid=54_000 + ros_domain_id,
            started_steady_ns=10_300_000,
            finished_steady_ns=10_400_000,
            wall_timeout_s=15.0,
        ),
    )
    for process in process_fixtures:
        _write_bounded_process_artifacts(directory, process)

    empty_gate = {
        'attempt_count': 1,
        'elapsed_wall_s': 0.05,
        'mode': 'empty',
        'nodes': ['/robotest/evidence/phase3_runtime_gate'],
        'producer': 'robotest_phase3/runtime_gate',
        'remaining_nodes': [],
        'schema_version': 1,
        'verdict': 'PASS',
    }
    _canonical_file(directory / 'domain-preflight.json', empty_gate, sidecar=True)
    _canonical_file(directory / 'domain-cleanup.json', empty_gate, sidecar=True)
    _canonical_file(
        directory / 'metrics.ready.json',
        {
            'initial_contact_stamp_ns': 1,
            'node_name': '/robotest/metrics_collector',
            'pre_clock_contact_message_count': 0,
            'started_steady_wall_ns': 200_000,
            'status': 'READY',
        },
    )
    (directory / 'metrics.stop').write_text('positive-control-complete\n', encoding='utf-8')


def _phase3_capture_fixture(
    metrics_fixture: object,
    *,
    scenario_id: int,
    goal_uuid: str,
    accepted_goal_stamp_ns: int,
    terminal_action_stamp_ns: int,
    fault_events: list[dict],
    fault: dict | None,
    support_pair: tuple[str, str],
) -> dict:
    core = metrics_fixture.CollectorCore()
    activation = (
        accepted_goal_stamp_ns + int(fault['start_offset_ns']) if fault is not None else None
    )
    deactivation = activation + int(fault['duration_ns']) if activation is not None else None
    x_rate = 0.0 if scenario_id != 5 else int(fault['parameters']['x_rate_nm_per_s']) * 1e-9
    yaw_rate = 0.0 if scenario_id != 5 else int(fault['parameters']['yaw_rate_nrad_per_s']) * 1e-9
    odometry_integrity = {
        'pose_covariance_sha256': 'a' * 64,
        'twist': {'angular': [0.0, 0.0, 0.1], 'linear': [0.2, 0.0, 0.0]},
        'twist_covariance_sha256': 'b' * 64,
        'z_m': 0.2,
    }
    for stamp_ns in range(0, terminal_action_stamp_ns + 1, 200_000_000):
        raw_x = stamp_ns / 10_000_000_000
        raw_y = 0.0
        raw_yaw = 0.0
        active_drift = (
            scenario_id == 5
            and activation is not None
            and deactivation is not None
            and activation <= stamp_ns < deactivation
        )
        elapsed_s = 0.0 if not active_drift else (stamp_ns - activation) / 1_000_000_000
        dx = x_rate * elapsed_s
        dyaw = yaw_rate * elapsed_s
        validated_x = math.cos(dyaw) * raw_x - math.sin(dyaw) * raw_y + dx
        validated_y = math.sin(dyaw) * raw_x + math.cos(dyaw) * raw_y
        validated_yaw = raw_yaw + dyaw
        core.record(
            'ground_truth',
            {
                'frame_id': 'world',
                'stamp_ns': stamp_ns,
                'x_m': raw_x,
                'y_m': raw_y,
                'yaw_rad': raw_yaw,
            },
        )
        core.record(
            'tf_map_odom',
            {
                'frame_id': 'map',
                'stamp_ns': stamp_ns,
                'x_m': 0.0,
                'y_m': 0.0,
                'yaw_rad': 0.0,
            },
        )
        core.record(
            'tf_odom_base_footprint',
            {
                'child_frame_id': 'base_footprint',
                'frame_id': 'odom',
                'orientation_xyzw': [
                    0.0,
                    0.0,
                    math.sin(validated_yaw / 2.0),
                    math.cos(validated_yaw / 2.0),
                ],
                'stamp_ns': stamp_ns,
                'x_m': validated_x,
                'y_m': validated_y,
                'yaw_rad': validated_yaw,
            },
        )
        if (
            scenario_id == 5
            and activation is not None
            and deactivation is not None
            and (activation <= stamp_ns <= deactivation)
        ):
            raw_odometry = {
                'child_frame_id': 'base_footprint',
                'frame_id': 'odom',
                'nonplanar_integrity': copy.deepcopy(odometry_integrity),
                'orientation_xyzw': [0.0, 0.0, 0.0, 1.0],
                'stamp_ns': stamp_ns,
                'x_m': raw_x,
                'y_m': raw_y,
                'yaw_rad': raw_yaw,
            }
            validated_odometry = copy.deepcopy(raw_odometry)
            validated_odometry.update(
                {
                    'orientation_xyzw': [
                        0.0,
                        0.0,
                        math.sin(validated_yaw / 2.0),
                        math.cos(validated_yaw / 2.0),
                    ],
                    'x_m': validated_x,
                    'y_m': validated_y,
                    'yaw_rad': validated_yaw,
                }
            )
            core.record('raw_odom', raw_odometry)
            core.record('odom', validated_odometry)

    core.record_state_transition(
        kind='waypoint_feedback',
        subject=goal_uuid,
        value=0,
        stamp_ns=accepted_goal_stamp_ns,
    )
    core.record_state_transition(
        kind='waypoint_feedback',
        subject=goal_uuid,
        value=1,
        stamp_ns=1_500_000_000,
    )
    core.record_state_transition(
        kind='waypoint_feedback',
        subject=goal_uuid,
        value=2,
        stamp_ns=2_000_000_000,
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [{'x_m': 0.1, 'y_m': 0.0}, {'x_m': 1.0, 'y_m': 0.0}],
            'stamp_ns': 1_200_000_000,
        }
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [{'x_m': 1.0, 'y_m': 0.0}, {'x_m': 2.0, 'y_m': 0.0}],
            'stamp_ns': 1_700_000_000,
        }
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [{'x_m': 2.0, 'y_m': 0.0}, {'x_m': 3.0, 'y_m': 0.0}],
            'stamp_ns': 10_000_000_000,
        }
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [
                {'x_m': 2.0, 'y_m': 0.0},
                {'x_m': 2.5, 'y_m': 0.1},
                {'x_m': 3.0, 'y_m': 0.0},
            ],
            'stamp_ns': 11_000_000_000,
        }
    )
    support_contact = {
        'collision1': support_pair[0],
        'collision2': support_pair[1],
        'maximum_normal_force_n': 5.0,
        'maximum_penetration_depth_m': 0.01,
    }
    contact_drain_stamp_ns = terminal_action_stamp_ns + 400_000_000
    for stamp_ns in range(
        accepted_goal_stamp_ns,
        contact_drain_stamp_ns + 1,
        200_000_000,
    ):
        core.record(
            'contacts',
            {
                'contacts': [support_contact],
                'delivery_clock_offset_ns': 0,
                'delivery_clock_stamp_ns': stamp_ns,
                'frame_id': '',
                'stamp_ns': stamp_ns,
            },
        )
    for index, stamp_ns in enumerate(
        (accepted_goal_stamp_ns, 18_000_000_000, terminal_action_stamp_ns)
    ):
        core.record(
            'world_stats',
            {
                'paused': False,
                'reported_real_time_factor': 1.0,
                'sim_stamp_ns': stamp_ns,
                'stamp_ns': stamp_ns,
                'steady_wall_ns': index * 17_000_000_000,
            },
        )
    if scenario_id == 3:
        core.record_state_transition(
            kind='collision_monitor', subject='safety', value=1, stamp_ns=3_900_000_000
        )
        core.record_state_transition(
            kind='collision_monitor', subject='safety', value=0, stamp_ns=12_200_000_000
        )
        for stamp_ns, linear_x in (
            (3_800_000_000, 0.2),
            (3_900_000_000, 0.0),
            (8_000_000_000, 0.0),
            (12_100_000_000, 0.0),
            (12_200_000_000, 0.2),
        ):
            core.record(
                'cmd_vel',
                {
                    'angular_z_rad_s': 0.0,
                    'linear_x_m_s': linear_x,
                    'linear_y_m_s': 0.0,
                    'stamp_ns': stamp_ns,
                },
            )
    if scenario_id == 4 and activation is not None and deactivation is not None:
        for stamp_ns in range(activation - 400_000_000, deactivation + 1_200_000_001, 200_000_000):
            scan = {'payload_sha256': 'c' * 64, 'stamp_ns': stamp_ns}
            core.record('raw_scan', scan)
            if stamp_ns < activation or stamp_ns >= deactivation:
                core.record('scan', scan)
        for stamp_ns, linear_x in (
            (activation - 200_000_000, 0.2),
            (activation + 400_000_000, 0.0),
            (deactivation - 200_000_000, 0.0),
            (deactivation + 200_000_000, 0.2),
        ):
            core.record(
                'cmd_vel',
                {
                    'angular_z_rad_s': 0.0,
                    'linear_x_m_s': linear_x,
                    'linear_y_m_s': 0.0,
                    'stamp_ns': stamp_ns,
                },
            )
    for event in fault_events:
        core.record('fault_events', {**event, 'stamp_ns': event['header_stamp_ns']})
    for stamp_ns in (0, 18_000_000_000, terminal_action_stamp_ns, contact_drain_stamp_ns):
        core.observe_clock(stamp_ns)
    capture = core.snapshot()
    capture.update(
        {
            'capture_schema_version': 1,
            'finished_steady_wall_ns': terminal_action_stamp_ns,
            'started_steady_wall_ns': 0,
            'stop_reason': 'stop_file',
        }
    )
    return capture


def _phase3_graph_document(
    orchestration: object,
    *,
    mission_client: bool,
    mission_auxiliary_nodes: set[str],
    persistent_nodes: set[str],
    watch_pid: int,
) -> dict:
    """Build one production-shaped schema-2 graph-probe PASS document."""
    contracts = orchestration.phase3_graph_contracts(mission_client)
    action_name = orchestration.PHASE3_GRAPH_ACTION_NAME
    action_type = orchestration.PHASE3_GRAPH_ACTION_TYPE
    passive_clients = ['/robotest/metrics_collector', '/robotest/scenario_controller']
    goal_clients = (
        {action_name: {orchestration.PHASE3_GRAPH_MISSION_CLIENT_NODE: [action_type]}}
        if mission_client
        else {}
    )
    participant_nodes = [
        *passive_clients,
        *([orchestration.PHASE3_GRAPH_MISSION_CLIENT_NODE] if mission_client else []),
    ]
    action_client_participants = {
        action_name: {node: [action_type] for node in sorted(participant_nodes)}
    }
    action_servers = {action_name: {orchestration.PHASE3_GRAPH_ACTION_SERVER_NODE: [action_type]}}
    topics = {name: [type_name] for name, type_name in contracts['topics'].items()}
    topics['/robotest/internal/raw_contacts'] = [
        contracts['topics']['/robotest/validation/contacts']
    ]
    services = {name: [type_name] for name, type_name in contracts['services'].items()}
    transitioned_nodes = (
        {orchestration.PHASE3_GRAPH_MISSION_CLIENT_NODE, *mission_auxiliary_nodes}
        if mission_client
        else {orchestration.PHASE3_GRAPH_GOAL_OBSERVER_NODE}
    )
    for node in transitioned_nodes:
        services.update(orchestration._phase3_standard_rclpy_services(node))
    actions = {name: [type_name] for name, type_name in contracts['actions'].items()}
    stable_nodes = persistent_nodes - {orchestration.PHASE3_GRAPH_GOAL_OBSERVER_NODE}
    node_names = sorted(
        {
            *stable_nodes,
            *passive_clients,
            orchestration.PHASE3_GRAPH_ACTION_SERVER_NODE,
            *transitioned_nodes,
            orchestration.PHASE3_GRAPH_PARTICIPANT,
        }
    )
    identities = []
    for fully_qualified_name in node_names:
        namespace, name = fully_qualified_name.rsplit('/', 1)
        identities.append(
            {
                'fully_qualified_name': fully_qualified_name,
                'hidden': False,
                'is_probe_participant': (
                    fully_qualified_name == orchestration.PHASE3_GRAPH_PARTICIPANT
                ),
                'name': name,
                'namespace': namespace,
            }
        )
    identities.sort(key=lambda item: (item['name'], item['namespace']))
    public_node_names = sorted(
        item['fully_qualified_name'] for item in identities if not item['is_probe_participant']
    )
    node_name_counts = {name: 1 for name in public_node_names}
    topic_results, missing_topics, topic_mismatches = orchestration._phase3_graph_contract_results(
        contracts['topics'], topics
    )
    service_results, missing_services, service_mismatches = (
        orchestration._phase3_graph_contract_results(contracts['services'], services)
    )
    action_results, missing_actions, action_mismatches = (
        orchestration._phase3_graph_contract_results(contracts['actions'], actions)
    )
    ownership_results, ownership_mismatches = orchestration._phase3_graph_action_results(
        contracts,
        goal_clients,
        action_servers,
    )
    assert not any(
        (
            missing_topics,
            missing_services,
            missing_actions,
            topic_mismatches,
            service_mismatches,
            action_mismatches,
            ownership_mismatches,
        )
    )
    return {
        'action_ownership_mismatches': [],
        'attempt_count': orchestration.PHASE3_GRAPH_MINIMUM_OBSERVATIONS,
        'contracts': contracts,
        'duplicate_node_names': [],
        'elapsed_wall_seconds': orchestration.PHASE3_GRAPH_QUIET_WINDOW_S,
        'failure': None,
        'failure_kind': None,
        'limits': {
            'maximum_graph_names': orchestration.PHASE3_GRAPH_MAXIMUM_GRAPH_NAMES,
            'maximum_graph_nodes': orchestration.PHASE3_GRAPH_MAXIMUM_GRAPH_NODES,
            'maximum_types_per_name': orchestration.PHASE3_GRAPH_MAXIMUM_TYPES_PER_NAME,
            'wall_timeout_seconds': (
                orchestration.PHASE3_GRAPH_MISSION_WALL_TIMEOUT_S
                if mission_client
                else orchestration.PHASE3_GRAPH_WALL_TIMEOUT_S
            ),
        },
        'missing_actions': [],
        'missing_services': [],
        'missing_topics': [],
        'node_name_counts': node_name_counts,
        'observed': {
            'action_client_participants': action_client_participants,
            'action_clients': goal_clients,
            'action_servers': action_servers,
            'actions': actions,
            'node_identities': identities,
            'node_names': public_node_names,
            'services': services,
            'topics': topics,
        },
        'participant': orchestration.PHASE3_GRAPH_PARTICIPANT,
        'query_errors': [],
        'results': {
            'action_ownership': ownership_results,
            'actions': action_results,
            'services': service_results,
            'topics': topic_results,
        },
        'schema_version': orchestration.PHASE3_GRAPH_SCHEMA_VERSION,
        'type_mismatches': {'actions': [], 'services': [], 'topics': []},
        'verdict': 'PASS',
        'watch_pid': watch_pid,
    }


def _write_phase3_graph_evidence(
    run_root: Path,
    *,
    mission_auxiliary_nodes: set[str],
    orchestration: object,
    mission_client: bool,
    persistent_nodes: set[str],
    watch_pid: int,
) -> dict:
    """Write one graph JSON and its four exact text projections."""
    prefix = 'mission-' if mission_client else ''
    document = _phase3_graph_document(
        orchestration,
        mission_client=mission_client,
        mission_auxiliary_nodes=mission_auxiliary_nodes,
        persistent_nodes=persistent_nodes,
        watch_pid=watch_pid,
    )
    (run_root / f'{prefix}graph.json').write_bytes(orchestration._phase3_graph_json_bytes(document))
    observed = document['observed']
    (run_root / f'{prefix}nodes.txt').write_text(
        ''.join(f'{name}\n' for name in observed['node_names']),
        encoding='utf-8',
    )
    for name in ('topics', 'services', 'actions'):
        (run_root / f'{prefix}{name}.txt').write_text(
            orchestration._phase3_graph_text(observed[name]),
            encoding='utf-8',
        )
    return orchestration.validate_phase3_graph_artifacts(
        run_root,
        mission_client=mission_client,
        expected_watch_pid=watch_pid,
    )


def _rewrite_phase3_graph_fixture(
    run_root: Path,
    orchestration: object,
    graph: dict,
    *,
    mission_client: bool,
    projections: tuple[str, ...] = (),
) -> None:
    """Rewrite a graph document and selected exact text projections for tamper tests."""
    prefix = 'mission-' if mission_client else ''
    (run_root / f'{prefix}graph.json').write_bytes(orchestration._phase3_graph_json_bytes(graph))
    for projection in projections:
        if projection == 'nodes':
            payload = ''.join(f'{name}\n' for name in graph['observed']['node_names'])
        else:
            payload = orchestration._phase3_graph_text(graph['observed'][projection])
        (run_root / f'{prefix}{projection}.txt').write_text(payload, encoding='utf-8')


def _add_phase3_graph_node(
    graph: dict,
    fully_qualified_name: str,
    *,
    hidden: bool = False,
) -> None:
    """Add one internally reconciled node to a graph tamper document."""
    namespace, name = fully_qualified_name.rsplit('/', 1)
    graph['observed']['node_identities'].append(
        {
            'fully_qualified_name': fully_qualified_name,
            'hidden': hidden,
            'is_probe_participant': False,
            'name': name,
            'namespace': namespace,
        }
    )
    graph['observed']['node_identities'].sort(key=lambda item: (item['name'], item['namespace']))
    if not hidden:
        graph['observed']['node_names'].append(fully_qualified_name)
        graph['observed']['node_names'].sort()
    graph['node_name_counts'][fully_qualified_name] = 1


def _write_phase3_graph_prerequisites(
    repository: Path,
    run_root: Path,
    *,
    orchestration: object,
    plan: dict,
    full_stack_stdout_bytes: bytes | None = None,
) -> tuple[dict, dict]:
    """Write the graph pair and the complete successful-run process registry."""
    runtime_gate_source = release_module._load_repository_module(
        repository,
        'tests/phase3_runtime_gate.py',
        'Phase 3 runtime-gate graph fixture source contract',
    )
    persistent_nodes = {
        f'/robotest/{name}' for name in runtime_gate_source.CANDIDATE_REQUIRED_NODES
    }
    mission_auxiliary_nodes = {'/robotest/lifecycle_sampler'} if plan['scenario_id'] == 4 else set()
    ros_domain_id = plan['ros_domain_id']
    pre_watch_pid = 40_000 + ros_domain_id
    mission_watch_pid = 41_000 + ros_domain_id
    role_pids = {
        role: 70_000 + ros_domain_id * 100 + index
        for index, role in enumerate(release_module.PHASE3_PREREQUISITE_PROCESS_ROLES)
    }
    role_pids.update(
        {
            'full_stack': pre_watch_pid,
            'graph_gate': 55_000 + ros_domain_id,
            'mission_graph_gate': 56_000 + ros_domain_id,
            'mission_runner': mission_watch_pid,
        }
    )
    if plan['scenario_id'] == 4:
        role_pids['lifecycle_sampler'] = 90_000 + ros_domain_id
    timelines = {
        'domain_preflight': (100_000_000, 200_000_000),
        'partition_preflight': (300_000_000, 400_000_000),
        'full_stack': (500_000_000, 33_000_000_000),
        'startup_gate': (600_000_000, 700_000_000),
        'lifecycle_gate': (800_000_000, 900_000_000),
        'metrics_collector': (1_000_000_000, 31_000_000_000),
        'scenario_controller': (1_100_000_000, 26_000_000_000),
        'goal_observer': (1_200_000_000, 8_500_000_000),
        'runtime_gate': (1_300_000_000, 1_400_000_000),
        'graph_gate': (1_500_000_000, 7_000_000_000),
        'mission_runner': (8_000_000_000, 25_000_000_000),
        'lifecycle_sampler': (8_600_000_000, 32_000_000_000),
        'mission_graph_gate': (9_000_000_000, 14_500_000_000),
        'contact_drain': (27_000_000_000, 28_000_000_000),
        'contact_stream_final_gate': (29_000_000_000, 30_000_000_000),
        'domain_cleanup': (34_000_000_000, 34_500_000_000),
        'partition_cleanup': (35_000_000_000, 35_500_000_000),
    }
    process_stubs = {role: {'pid': pid} for role, pid in role_pids.items()}
    contracts = release_module._phase3_process_contracts(
        repository,
        run_root,
        expected_plan=plan,
        orchestration=orchestration,
        process_records=process_stubs,
    )
    assert set(contracts) == set(role_pids)
    for role, (command, wall_timeout_s, allowed_returncodes) in contracts.items():
        started_steady_ns, finished_steady_ns = timelines[role]
        _write_bounded_process_artifacts(
            run_root,
            _bounded_process_fixture(
                role=role,
                command=command,
                cwd=repository,
                pid=role_pids[role],
                started_steady_ns=started_steady_ns,
                finished_steady_ns=finished_steady_ns,
                wall_timeout_s=wall_timeout_s,
                returncode=(-15 if role == 'full_stack' else allowed_returncodes[-1]),
            ),
            stdout_bytes=(
                (
                    full_stack_stdout_bytes
                    if full_stack_stdout_bytes is not None
                    else b'[robotest] full stack stopped cleanly'
                )
                if role == 'full_stack'
                else b''
            ),
            stderr_bytes=(b'[robotest] shutdown complete\n' if role == 'full_stack' else b''),
        )
    pre_binding = _write_phase3_graph_evidence(
        run_root,
        mission_auxiliary_nodes=mission_auxiliary_nodes,
        orchestration=orchestration,
        mission_client=False,
        persistent_nodes=persistent_nodes,
        watch_pid=pre_watch_pid,
    )
    mission_binding = _write_phase3_graph_evidence(
        run_root,
        mission_auxiliary_nodes=mission_auxiliary_nodes,
        orchestration=orchestration,
        mission_client=True,
        persistent_nodes=persistent_nodes,
        watch_pid=mission_watch_pid,
    )
    return pre_binding, mission_binding


def _phase3_bundle(
    repository: Path,
    directory: Path,
    *,
    orchestration: object,
    plan: dict,
    git_sha: str,
    build_binding: dict,
    positive_binding_path: Path,
    full_stack_stdout_bytes: bytes | None = None,
) -> str:
    directory.mkdir(parents=True)
    metrics_fixture = release_module._load_repository_module(
        repository,
        'src/robotest_metrics/test/conftest.py',
        'Phase 3 metrics producer fixture',
    )
    request_fixture = metrics_fixture.complete_request()
    scenario_id = int(plan['scenario_id'])
    scenario_document = yaml.safe_load(
        (repository / plan['scenario_path']).read_text(encoding='utf-8')
    )
    fault = copy.deepcopy(scenario_document.get('fault'))
    faults = [] if fault is None else [fault]
    schedule_document = {'schema_version': 1, 'faults': faults}
    schedule_canonical = json.dumps(schedule_document, separators=(',', ':'))
    schedule_sha256 = hashlib.sha256(schedule_canonical.encode()).hexdigest()
    assert schedule_sha256 == scenario_document['fault_schedule_sha256']
    accepted_goal_stamp_ns = 1_000_000_000
    terminal_action_stamp_ns = 35_000_000_000
    goal_uuid = f'goal-{plan["run_id"]}'
    event_types = [3, 1, 7]
    event_stamps = [500_000_000, 700_000_000, accepted_goal_stamp_ns]
    affected_count = 0
    if fault is not None:
        activation = accepted_goal_stamp_ns + int(fault['start_offset_ns'])
        deactivation = activation + int(fault['duration_ns'])
        event_types.extend([5, 9])
        event_stamps.extend([activation, deactivation])
        if scenario_id == 4:
            affected_count = int(fault['duration_ns']) // 200_000_000
    event_types.append(3)
    event_stamps.append(terminal_action_stamp_ns + 100_000_000)
    fault_events = [
        _phase3_fault_event(
            metrics_fixture,
            sequence=index,
            event_type=event_type,
            stamp_ns=stamp_ns,
            goal_uuid=goal_uuid,
            accepted_goal_stamp_ns=accepted_goal_stamp_ns,
            schedule_sha256=schedule_sha256,
            fault=fault,
            affected_message_count=(1 if event_type == 5 else affected_count),
        )
        for index, (event_type, stamp_ns) in enumerate(
            zip(event_types, event_stamps, strict=True), start=1
        )
    ]
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
    mission_measurements = {
        'accepted_goal_stamp_ns': accepted_goal_stamp_ns,
        'accepted_goal_uuid': goal_uuid,
        'completed_waypoint_count': 3,
        'goal_status': 'SUCCEEDED',
        'goal_status_code': 4,
        'missed_waypoint_count': 0,
        'terminal_action_stamp_ns': terminal_action_stamp_ns,
    }
    mission = copy.deepcopy(request_fixture['mission']['result'])
    mission['targets'] = {
        'waypoint_count': 3,
        'waypoints': [
            {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 2.0, 'y': 0.0, 'yaw': 0.0},
            {'x': 3.0, 'y': 0.0, 'yaw': 0.0},
        ],
    }
    mission['identity'].update(
        {
            'candidate_id': plan['candidate_id'],
            'fault_schedule_hash': schedule_sha256,
            'mission_sha256': plan['scenario_sha256'],
            'repetition_index': plan['repetition_index'],
            'run_id': plan['run_id'],
            'scenario_id': scenario_id,
            'suite_index': plan['suite_index'],
        }
    )
    mission['measurements'] = mission_measurements
    mission['fault'] = {
        'control': {
            'arm_commit_stamp_ns': accepted_goal_stamp_ns,
            'arm_margin_ns': 0 if fault is None else 500_000_000,
            'arm_replayed': False,
            'event_count': len(fault_events),
            'event_trace_capacity': 512,
            'event_trace_overflow': False,
            'event_trace_overflow_count': 0,
            'generation': 1,
            'preload_replayed': False,
            'protocol_status': 'RESET_CONFIRMED_AFTER_GOAL',
            'reset_after_goal': True,
            'reset_before_goal': True,
        },
        'events': fault_events,
        'schedule': {
            'canonical_json': schedule_canonical,
            'fault_count': len(faults),
            'faults': faults,
            'schema_version': 1,
            'sha256': schedule_sha256,
        },
    }
    from robotest_missions import artifacts as mission_artifacts

    mission['identity'].update(
        {
            'action_name': '/robotest/follow_waypoints',
            'created_utc': '2026-01-01T00:00:00Z',
            'fault_seed': scenario_document.get('fault_seed'),
            'mission_file': str(repository / plan['scenario_path']),
            'mission_name': plan['scenario_name'],
            'mission_seed': scenario_document.get('mission_seed', 42),
            'resolved_action_name': '/robotest/follow_waypoints',
            'scenario_controller_seed': scenario_document.get('scenario_controller_seed', 42),
            'simulator_seed': scenario_document['simulator_seed'],
        }
    )
    mission['targets'].update(
        {
            'expected_outcome': scenario_document.get('expected_outcome', 'SUCCEEDED'),
            'fault_event_trace_capacity': mission_artifacts.FAULT_EVENT_TRACE_CAPACITY,
            'feedback_trace_capacity': mission_artifacts.FEEDBACK_TRACE_CAPACITY,
            'mission_event_trace_capacity': mission_artifacts.MISSION_EVENT_TRACE_CAPACITY,
        }
    )
    mission['measurements'].update(
        {
            'accepted_goal_stamp_source': mission_artifacts.ACCEPTED_GOAL_STAMP_SOURCE,
            'completion_time_sim_s': (terminal_action_stamp_ns - accepted_goal_stamp_ns)
            / 1_000_000_000,
            'feedback_count': 3,
            'goal_response_stamp_ns': accepted_goal_stamp_ns,
            'goal_submission_stamp_ns': accepted_goal_stamp_ns - 100_000_000,
            'mission_wall_duration_s': 30.0,
            'nav2_error_code': 0,
            'nav2_error_message': '',
        }
    )
    mission['quality'].update(
        {
            'cancel_acknowledged': False,
            'fault_event_overflow_count': 0,
            'fault_protocol_status': 'RESET_CONFIRMED_AFTER_GOAL',
            'feedback_trace_overflow_count': 0,
        }
    )
    mission['verdict'].update(
        {
            'reason': None,
            'scenario1_acceptance_status': mission_artifacts.PHASE3_BENCHMARK_NOT_EVALUATED,
        }
    )
    capture = _phase3_capture_fixture(
        metrics_fixture,
        scenario_id=scenario_id,
        goal_uuid=goal_uuid,
        accepted_goal_stamp_ns=accepted_goal_stamp_ns,
        terminal_action_stamp_ns=terminal_action_stamp_ns,
        fault_events=fault_events,
        fault=fault,
        support_pair=(
            yaml.safe_load(
                (repository / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
            )['support_pairs'][0]['robot_collision'],
            yaml.safe_load(
                (repository / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
            )['support_pairs'][0]['environment_collision'],
        ),
    )
    if scenario_id in {2, 3}:
        scenario_fixture = release_module._load_repository_module(
            repository,
            'src/robotest_metrics/test/test_candidate_validation.py',
            f'Phase 3 Scenario {scenario_id} producer fixture',
        )
        _, scenario_wrapper, _ = getattr(scenario_fixture, f'_scenario{scenario_id}')()
        scenario = copy.deepcopy(scenario_wrapper['result'])
        scenario['identity'] = {
            key: identity[key]
            for key in (
                'candidate_id',
                'repetition_index',
                'run_id',
                'scenario_id',
                'scenario_name',
                'scenario_sha256',
                'suite_index',
            )
        }
        scenario['binding'].update(
            {
                'accepted_goal_stamp_ns': accepted_goal_stamp_ns,
                'goal_uuid': goal_uuid,
                'terminal_status': 4,
            }
        )
        if scenario_id == 3:
            scenario['binding'].update(
                {
                    'terminal_observed_sequence': 500,
                    'terminal_observed_stamp_ns': 34_000_000_000,
                }
            )
    else:
        scenario = metrics_fixture._scenario_result(identity, mission_measurements)
    orchestrator = copy.deepcopy(request_fixture['orchestrator'])
    orchestrator['identity'].update(
        {
            'candidate_id': plan['candidate_id'],
            'gz_partition': plan['gz_partition'],
            'repetition_index': plan['repetition_index'],
            'ros_domain_id': plan['ros_domain_id'],
            'run_id': plan['run_id'],
            'scenario_id': scenario_id,
            'scenario_sha256': plan['scenario_sha256'],
            'suite_index': plan['suite_index'],
        }
    )
    orchestrator['git'] = {
        'dirty': False,
        'end_head': git_sha,
        'start_head': git_sha,
        'status_porcelain': '',
    }
    orchestrator['source_binding'].update(
        {
            'collector_configuration_sha256': build_binding['collector_configuration_sha256'],
            'contact_aggregator_binary': build_binding['contact_aggregator_binary'],
            'contact_gate_binary': build_binding['contact_gate_binary'],
            'install_end_sha256': build_binding['install']['aggregate_sha256'],
            'install_start_sha256': build_binding['install']['aggregate_sha256'],
            'metrics_contract_sha256': build_binding['metrics_contract_sha256'],
            'source_configuration_sha256': build_binding['source_configuration_sha256'],
            'source_end_sha256': build_binding['source']['aggregate_sha256'],
            'source_start_sha256': build_binding['source']['aggregate_sha256'],
            'target_set_sha256': build_binding['target_set_sha256'],
        }
    )
    run_root = directory.parent
    mission_path = run_root / 'mission-result.json'
    mission_csv_path = run_root / 'mission-result.csv'
    scenario_path = run_root / 'scenario-result.json'
    capture_path = run_root / 'capture.json'
    orchestrator_path = run_root / 'orchestrator.json'
    mission_artifacts.write_result_artifacts(mission, mission_path, mission_csv_path)
    Path(f'{mission_path}.sha256').write_text(
        f'{phase5_module.file_sha256(mission_path)}  {mission_path.name}\n',
        encoding='ascii',
    )
    _canonical_file(scenario_path, scenario, sidecar=True)
    _canonical_file(capture_path, capture, sidecar=True)
    contact_items = capture['streams']['contacts']['items']
    qualifying_contact_stamp_ns = contact_items[-1]['stamp_ns']
    contact_drain = {
        'clock_first_stamp_ns': 0,
        'clock_latest_stamp_ns': qualifying_contact_stamp_ns,
        'clock_message_count': 4,
        'clock_minus_qualifying_contact_ns': 0,
        'clock_regression_count': 0,
        'contact_message_count': len(contact_items),
        'contact_record_count_violation_count': 0,
        'contact_stamp_duplicate_count': 0,
        'contact_stamp_regression_count': 0,
        'first_contact_stamp_ns': contact_items[0]['stamp_ns'],
        'gate_node_present': True,
        'latest_contact_stamp_ns': qualifying_contact_stamp_ns,
        'limits': {
            'heartbeat_period_ns': 200_000_000,
            'max_clock_lag_ns': 220_000_000,
            'max_public_gap_ns': 220_000_000,
            'release_gap_ns': 250_000_000,
        },
        'maximum_contact_record_count': 1,
        'maximum_contact_source_gap_ns': 200_000_000,
        'minimum_contact_record_count': 1,
        'minimum_same_pair_set_interval_ns': 200_000_000,
        'producer': 'robotest_phase3/contact_drain_observer',
        'public_publisher_nodes': ['/robotest/contact_stream_gate'],
        'public_topic': '/robotest/validation/contacts',
        'qualifying_contact_snapshot_stamp_ns': qualifying_contact_stamp_ns,
        'same_pair_set_interval_violation_count': 0,
        'schema_version': 3,
        'target_stamp_ns': terminal_action_stamp_ns + 250_000_000,
        'terminal_action_stamp_ns': terminal_action_stamp_ns,
    }
    _canonical_file(run_root / 'contact-drain.json', contact_drain, sidecar=True)
    _canonical_file(
        run_root / 'contact-progress.json',
        {
            'latest_retained_stamp_ns': qualifying_contact_stamp_ns,
            'producer': 'robotest_metrics/metrics_collector',
            'public_topic': '/robotest/validation/contacts',
            'retained_message_count': len(contact_items),
            'schema_version': 1,
        },
    )
    _write_phase3_contact_gate_reobservation(
        repository,
        run_root,
        orchestration=orchestration,
        build_binding=build_binding,
        ros_domain_id=plan['ros_domain_id'],
        gz_partition=plan['gz_partition'],
    )
    empty_gate = {
        'attempt_count': 1,
        'elapsed_wall_s': 0.05,
        'mode': 'empty',
        'nodes': ['/robotest/evidence/phase3_runtime_gate'],
        'producer': 'robotest_phase3/runtime_gate',
        'remaining_nodes': [],
        'schema_version': 1,
        'verdict': 'PASS',
    }
    _canonical_file(run_root / 'domain-preflight.json', empty_gate, sidecar=True)
    _canonical_file(run_root / 'domain-cleanup.json', empty_gate, sidecar=True)
    _canonical_file(run_root / 'lifecycle-startup-result.json', {'verdict': 'PASS'})
    _canonical_file(run_root / 'startup-gate.json', {'verdict': 'PASS'})
    lifecycle_states = {
        node_name: {
            'attempts': 1,
            'label': 'active',
            'service': f'/robotest/{node_name}/get_state',
            'service_seen': True,
            'state_id': 3,
            'timed_out_attempts': 0,
        }
        for node_name in orchestration.REQUIRED_LIFECYCLE_NODES
    }
    _write_phase3_lifecycle_ready(
        repository,
        run_root,
        {
            'failure': None,
            'namespace': '/robotest',
            'required_state': {'id': 3, 'label': 'active'},
            'states': lifecycle_states,
            'verdict': 'PASS',
            'wall_timeout_s': 110.0,
            'watch_pid': 40_000 + plan['ros_domain_id'],
        },
    )
    for node_name in orchestration.REQUIRED_LIFECYCLE_NODES:
        (run_root / f'lifecycle-ready-{node_name}.txt').write_text(
            'active [3]\n',
            encoding='utf-8',
        )
    _canonical_file(run_root / 'metrics.ready.json', {'status': 'READY'})
    (run_root / 'metrics.stop').write_text(
        'terminal-contact-drain-complete\n',
        encoding='utf-8',
    )
    _canonical_file(run_root / 'scenario.ready.json', {'status': 'READY'})
    (run_root / 'goal-observer.arm').write_text('arm-next-new-goal\n', encoding='utf-8')
    _canonical_file(run_root / 'goal-observer.ready.json', {'status': 'READY'})
    _canonical_file(run_root / 'goal-observer.armed.json', {'status': 'ARMED'})
    goal_observer = {
        'accepted_goal_stamp_ns': accepted_goal_stamp_ns,
        'accepted_goal_uuid': goal_uuid,
    }
    _canonical_file(
        run_root / 'goal-observer.json',
        goal_observer,
        sidecar=True,
    )
    _canonical_file(
        run_root / 'component-exits.json',
        {'mission_exit_code': 0, 'scenario_exit_code': 0},
    )
    _canonical_file(
        run_root / 'goal-binding-reconciliation.json',
        orchestration.reconcile_goal_binding(goal_observer, mission, scenario),
    )
    resource_samples = [
        {
            'affinity_checked_pid_count': 1,
            'affinity_escape_count': 0,
            'affinity_escape_prefix': [],
            'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5],
            'affinity_unreadable_count': 0,
            'affinity_unreadable_pid_prefix': [],
            'cpu_percent': 50.0,
            'missing_count': 0,
            'oom_kill': False,
            'phase': 'before_launch',
            'pid_reuse_detected': False,
            'rss_sum_bytes': 1_000_000_000,
            'wsl_memory_bytes': 2_000_000_000,
            'wsl_swap_bytes': 0,
        },
        {
            'affinity_checked_pid_count': 1,
            'affinity_escape_count': 0,
            'affinity_escape_prefix': [],
            'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5],
            'affinity_unreadable_count': 0,
            'affinity_unreadable_pid_prefix': [],
            'cpu_percent': 25.0,
            'missing_count': 0,
            'oom_kill': False,
            'phase': 'after_shutdown',
            'pid_reuse_detected': False,
            'rss_sum_bytes': 500_000_000,
            'wsl_memory_bytes': 1_500_000_000,
            'wsl_swap_bytes': 0,
        },
    ]
    resource_path = run_root / 'resources.jsonl'
    resource_path.write_text(
        ''.join(json.dumps(sample, sort_keys=True) + '\n' for sample in resource_samples),
        encoding='utf-8',
    )
    resource_summary = orchestration.summarize_resources(resource_path)
    orchestrator['resources'] = {
        field: resource_summary[field] for field in orchestration.RESOURCE_METRIC_FIELDS
    }
    lifecycle_path = None
    if scenario_id == 4:
        lifecycle_stamps = [13_000_000_000, 14_000_000_000]
        schedule = {
            'lifecycle_schedule_schema_version': 1,
            'requested_stamps_ns': lifecycle_stamps,
            'run_id': plan['run_id'],
        }
        _canonical_file(run_root / 'lifecycle-schedule.json', schedule, sidecar=True)
        _canonical_file(run_root / 'lifecycle-sampler.ready.json', {'status': 'READY'})
        (run_root / 'lifecycle-sampler.stop').write_text(
            'mission-complete\n',
            encoding='utf-8',
        )
        records = []
        for round_index, stamp_ns in enumerate(lifecycle_stamps):
            for node in orchestration.REQUIRED_LIFECYCLE_NODES:
                records.append(
                    {
                        'collector_sequence': len(records) + 1,
                        'error': None,
                        'missed': False,
                        'node': node,
                        'request_stamp_ns': stamp_ns,
                        'requested_stamp_ns': stamp_ns,
                        'response_stamp_ns': stamp_ns,
                        'round_index': round_index,
                        'state_id': 3,
                        'state_label': 'active',
                        'success': True,
                    }
                )
        samples = [
            {
                'collector_sequence': record['collector_sequence'],
                'node': record['node'],
                'stamp_ns': record['response_stamp_ns'],
                'state': record['state_label'],
            }
            for record in records
        ]
        lifecycle = {
            'capacity': {
                'round_capacity': 96,
                'sample_capacity': 864,
                'scheduled_round_count': len(lifecycle_stamps),
                'scheduled_sample_count': len(records),
            },
            'clock': {
                'count': len(lifecycle_stamps),
                'latest_stamp_ns': lifecycle_stamps[-1],
                'regression_count': 0,
            },
            'errors': [],
            'identity': {
                'run_id': plan['run_id'],
                'sampler_node': 'lifecycle_sampler',
                'schedule_sha256': release_module._canonical_sha256(schedule),
            },
            'lifecycle_snapshot_schema_version': 1,
            'quality': {
                'collector_overflow': False,
                'complete': True,
                'error_count': 0,
                'error_counts': {},
                'finished_steady_wall_ns': 2_000,
                'first_overflow_sequence': None,
                'first_overflow_stamp_ns': None,
                'late_response_count': 0,
                'missed_count': 0,
                'overflow_count': 0,
                'pending_count': 0,
                'request_count': len(records),
                'response_count': len(records),
                'retained_count': len(records),
                'runtime_error': None,
                'started_steady_wall_ns': 1,
                'stop_reason': 'stop_file',
                'success_count': len(records),
            },
            'records': records,
            'samples': samples,
            'schedule': {
                'nodes': list(orchestration.REQUIRED_LIFECYCLE_NODES),
                'requested_stamps_ns': lifecycle_stamps,
                'service_names': [
                    f'{node}/get_state' for node in orchestration.REQUIRED_LIFECYCLE_NODES
                ],
            },
        }
        lifecycle_path = run_root / 'lifecycle-snapshot.json'
        _canonical_file(lifecycle_path, lifecycle, sidecar=True)
    pre_graph_binding, mission_graph_binding = _write_phase3_graph_prerequisites(
        repository,
        run_root,
        orchestration=orchestration,
        plan=plan,
        full_stack_stdout_bytes=full_stack_stdout_bytes,
    )
    _write_phase3_final_launch_log_gate(repository, run_root)
    mission_process = json.loads(
        (run_root / 'processes/mission_runner.process.json').read_text(encoding='utf-8')
    )
    orchestrator['execution'] = {
        'command': shlex.join(mission_process['command']),
        'exit_code': mission_process['returncode'],
        'wall_duration_s': (
            mission_process['finished_steady_ns'] - mission_process['started_steady_ns']
        )
        / 1_000_000_000,
        'wall_timed_out': False,
        'wall_timeout_s': 300.0,
        'working_directory': str(repository),
    }
    trial_context = orchestration.make_trial_context(
        plan,
        workspace=repository,
        git_sha=git_sha,
        build=build_binding,
        positive=json.loads(positive_binding_path.read_text(encoding='utf-8')),
    )
    _canonical_file(run_root / 'trial-context.json', trial_context, sidecar=True)
    prerequisite_paths = sorted(path for path in run_root.rglob('*') if path.is_file())
    prerequisite_manifest = orchestration.component_manifest(prerequisite_paths, run_root)
    prerequisite_manifest_path = run_root / 'prerequisite-manifest.json'
    _canonical_file(prerequisite_manifest_path, prerequisite_manifest, sidecar=True)
    assert orchestration.verify_component_manifest(prerequisite_manifest, run_root)
    prerequisite_records = prerequisite_manifest['artifacts']
    orchestrator['artifacts'] = {
        'mission_graph_sha256': mission_graph_binding['graph_json_sha256'],
        'pre_mission_graph_sha256': pre_graph_binding['graph_json_sha256'],
        'prerequisite_artifact_count': len(prerequisite_records),
        'prerequisite_checksums_verified': True,
        'prerequisite_manifest_sha256': phase5_module.file_sha256(prerequisite_manifest_path),
        'prerequisite_maximum_file_bytes': max(record['bytes'] for record in prerequisite_records),
        'prerequisite_total_bytes': prerequisite_manifest['total_bytes'],
        'prerequisites_finalized': True,
        'prerequisites_within_caps': True,
        'runtime_stderr_bytes': sum(
            record['bytes']
            for record in prerequisite_records
            if record['path'].endswith('.stderr.log')
        ),
        'runtime_stdout_bytes': sum(
            record['bytes']
            for record in prerequisite_records
            if record['path'].endswith('.stdout.log')
        ),
    }
    _canonical_file(orchestrator_path, orchestrator, sidecar=True)
    analysis_request = orchestration.compose_analysis_request(
        workspace=repository,
        plan=plan,
        mission_path=mission_path,
        scenario_path=scenario_path,
        capture_path=capture_path,
        positive_binding_path=positive_binding_path,
        orchestrator_path=orchestrator_path,
        contact_drain_path=run_root / 'contact-drain.json',
        contact_progress_path=run_root / 'contact-progress.json',
        lifecycle_snapshot_path=lifecycle_path,
    )
    _canonical_file(run_root / 'analysis-request.json', analysis_request, sidecar=True)
    from robotest_metrics.analysis import analyze_run
    from robotest_metrics.artifacts import one_row_csv_bytes as phase3_run_csv_bytes

    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'PASS', {
        'quality': result['quality'],
        'verdict': result['verdict'],
    }
    result_path = directory / 'run-result.json'
    _canonical_file(result_path, result)
    (directory / 'run-result.csv').write_bytes(phase3_run_csv_bytes(result))
    (directory / 'report.md').write_text('PASS\n', encoding='utf-8')
    (directory / 'report.html').write_text('<p>PASS</p>\n', encoding='utf-8')
    png = _phase5_fixture_png()
    for name in (
        'localization-error.png',
        'measurement-summary.png',
        'real-time-factor.png',
        'trajectory.png',
    ):
        (directory / name).write_bytes(png)
    records = [
        {
            'bytes': path.stat().st_size,
            'path': path.name,
            'sha256': phase5_module.file_sha256(path),
        }
        for path in sorted(directory.iterdir())
    ]
    result_sha = phase5_module.file_sha256(result_path)
    manifest = {
        'artifacts': records,
        'identity': {'run_id': plan['run_id'], 'run_result_sha256': result_sha},
        'producer': 'robotest_metrics/metrics_analyze',
        'quality': {
            'artifact_bytes_excluding_manifest': sum(record['bytes'] for record in records),
            'artifact_count': len(records),
            'caps_within_limits': True,
            'hashes_verified': True,
            'path_set_complete': True,
        },
        'schema_version': 1,
    }
    _canonical_file(directory / 'run-artifacts.manifest.json', manifest, sidecar=True)
    return result_sha


def _refresh_phase3_bundle(directory: Path) -> str:
    from robotest_metrics.artifacts import one_row_csv_bytes as phase3_run_csv_bytes

    result_path = directory / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    _canonical_file(result_path, result)
    (directory / 'run-result.csv').write_bytes(phase3_run_csv_bytes(result))
    artifacts = [
        path
        for path in sorted(directory.iterdir())
        if path.name not in {'run-artifacts.manifest.json', 'run-artifacts.manifest.json.sha256'}
    ]
    records = [
        {
            'bytes': path.stat().st_size,
            'path': path.name,
            'sha256': phase5_module.file_sha256(path),
        }
        for path in artifacts
    ]
    result_sha = phase5_module.file_sha256(result_path)
    manifest_path = directory / 'run-artifacts.manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['artifacts'] = records
    manifest['identity']['run_result_sha256'] = result_sha
    manifest['quality']['artifact_bytes_excluding_manifest'] = sum(
        record['bytes'] for record in records
    )
    manifest['quality']['artifact_count'] = len(records)
    _canonical_file(manifest_path, manifest, sidecar=True)
    return result_sha


def _rebind_phase3_prerequisites(
    repository: Path,
    run_root: Path,
    *,
    omitted_paths: frozenset[str] = frozenset(),
) -> None:
    """Rebind a tampered raw set so tests reach semantic graph replay."""
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 prerequisite rebind fixture',
    )
    manifest_path = run_root / 'prerequisite-manifest.json'
    previous = json.loads(manifest_path.read_text(encoding='utf-8'))
    paths = [
        run_root / record['path']
        for record in previous['artifacts']
        if record['path'] not in omitted_paths
    ]
    manifest = orchestration.component_manifest(paths, run_root)
    _canonical_file(manifest_path, manifest, sidecar=True)
    records = manifest['artifacts']
    orchestrator_path = run_root / 'orchestrator.json'
    orchestrator = json.loads(orchestrator_path.read_text(encoding='utf-8'))
    orchestrator['artifacts'].update(
        {
            'mission_graph_sha256': phase5_module.file_sha256(run_root / 'mission-graph.json'),
            'pre_mission_graph_sha256': phase5_module.file_sha256(run_root / 'graph.json'),
            'prerequisite_artifact_count': len(records),
            'prerequisite_manifest_sha256': phase5_module.file_sha256(manifest_path),
            'prerequisite_maximum_file_bytes': max(record['bytes'] for record in records),
            'prerequisite_total_bytes': manifest['total_bytes'],
            'runtime_stderr_bytes': sum(
                record['bytes'] for record in records if record['path'].endswith('.stderr.log')
            ),
            'runtime_stdout_bytes': sum(
                record['bytes'] for record in records if record['path'].endswith('.stdout.log')
            ),
        }
    )
    _canonical_file(orchestrator_path, orchestrator, sidecar=True)


def _recompose_phase3_smoke_result(repository: Path, candidate_root: Path) -> None:
    """Cascade a rebound smoke prerequisite set through analysis and PASS output."""
    release_module._activate_repository_packages(repository)
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 smoke analysis rebind fixture',
    )
    from robotest_metrics.analysis import analyze_run

    suite_plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))
    first_trial = suite_plan['trials'][0]
    smoke = suite_plan['smoke']
    plan = {
        **first_trial,
        'candidate_id': f'{suite_plan["candidate_id"]}-smoke',
        'gz_partition': smoke['gz_partition'],
        'ros_domain_id': smoke['ros_domain_id'],
        'run_id': smoke['run_id'],
    }
    run_root = candidate_root / 'smoke'
    request = orchestration.compose_analysis_request(
        workspace=repository,
        plan=plan,
        mission_path=run_root / 'mission-result.json',
        scenario_path=run_root / 'scenario-result.json',
        capture_path=run_root / 'capture.json',
        positive_binding_path=candidate_root / 'positive-control/positive-binding.json',
        orchestrator_path=run_root / 'orchestrator.json',
        contact_drain_path=run_root / 'contact-drain.json',
        contact_progress_path=run_root / 'contact-progress.json',
        lifecycle_snapshot_path=(
            run_root / 'lifecycle-snapshot.json' if int(plan['scenario_id']) == 4 else None
        ),
    )
    _canonical_file(run_root / 'analysis-request.json', request, sidecar=True)
    result = analyze_run(request)
    assert result['verdict']['automated_status'] == 'PASS', result['verdict']
    result_directory = run_root / 'result'
    _canonical_file(result_directory / 'run-result.json', result)
    result_sha = _refresh_phase3_bundle(result_directory)
    pass_path = run_root / 'PASS.json'
    marker = json.loads(pass_path.read_text(encoding='utf-8'))
    marker['run_result_sha256'] = result_sha
    _canonical_file(pass_path, marker, sidecar=True)


def _rebind_phase3_smoke_runtime_gate(
    repository: Path,
    candidate_root: Path,
    run_root: Path,
) -> None:
    """Cascade a smoke gate mutation through reobservation and the raw manifest."""
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 smoke runtime-gate rebind fixture',
    )
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    suite_plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))
    smoke = suite_plan['smoke']
    smoke_plan = {
        **suite_plan['trials'][0],
        'candidate_id': f'{suite_plan["candidate_id"]}-smoke',
        'gz_partition': smoke['gz_partition'],
        'ros_domain_id': smoke['ros_domain_id'],
        'run_id': smoke['run_id'],
    }
    _canonical_file(
        run_root / 'contact-gate-revalidation.json',
        orchestration.reconcile_contact_gate_reobservation(
            run_root / 'runtime-gate.json',
            run_root / 'contact-stream-final-gate.json',
            build_binding=build_binding,
            expected_domain_id=smoke_plan['ros_domain_id'],
            expected_gz_partition=smoke_plan['gz_partition'],
        ),
        sidecar=True,
    )
    _rebind_phase3_prerequisites(repository, run_root)


def _refresh_phase4_manifest(run_directory: Path) -> None:
    manifest_path = run_directory / 'SHA256SUMS'
    records = [
        f'{phase5_module.file_sha256(path)}  {path.relative_to(run_directory).as_posix()}'
        for path in sorted(run_directory.rglob('*'))
        if path.is_file() and path != manifest_path
    ]
    manifest_path.write_text('\n'.join(records) + '\n', encoding='ascii')


def _refresh_phase4_result(run_directory: Path) -> None:
    result_path = run_directory / 'scenario6-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['quality']['raw_evidence_sha256'] = {
        path.relative_to(run_directory).as_posix(): phase5_module.file_sha256(path)
        for path in sorted(run_directory.rglob('*'))
        if path.is_file()
        and path.relative_to(run_directory).as_posix()
        not in {'SHA256SUMS', 'scenario6-result.csv', 'scenario6-result.json'}
    }
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)


def _refresh_phase4_overlay_manifest_hashes(run_directory: Path) -> None:
    staging = json.loads((run_directory / 'runtime-staging.json').read_text(encoding='utf-8'))
    staging['source_manifest_sha256'] = phase5_module.file_sha256(
        run_directory / 'overlay-source-manifest.json'
    )
    staging['install_manifest_sha256'] = phase5_module.file_sha256(
        run_directory / 'overlay-install-manifest.json'
    )
    _canonical_file(run_directory / 'runtime-staging.json', staging)
    _canonical_file(run_directory / 'overlay-provenance.json', staging)


def _refresh_remote_proof(path: Path, document: object) -> None:
    _canonical_file(path, document)
    path.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(path)}  {path.name}\n', encoding='ascii'
    )
    path.with_suffix('.checksum-validation.txt').write_text(f'{path.name}: OK\n', encoding='utf-8')


def _refresh_phase3_positive_binding(
    candidate_root: Path,
    binding: dict,
    *,
    rebind_coverage: bool = False,
) -> None:
    coverage = binding['coverage_manifest']
    positive_control = binding['positive_control']
    benchmark_binding = binding['benchmark_binding']
    if rebind_coverage:
        semantic = dict(coverage)
        semantic.pop('manifest_sha256', None)
        coverage_sha = release_module._canonical_sha256(semantic)
        coverage['manifest_sha256'] = coverage_sha
        positive_control['configuration']['coverage_manifest_sha256'] = coverage_sha
        for name in ('benchmark_provenance', 'positive_control_provenance'):
            benchmark_binding[name]['coverage_manifest_sha256'] = coverage_sha
    positive_sha = release_module._canonical_sha256(positive_control)
    binding['positive_control_json_sha256'] = positive_sha
    benchmark_binding['positive_control_json_sha256'] = positive_sha
    path = candidate_root / 'positive-control/positive-binding.json'
    _canonical_file(path, binding, sidecar=True)
    marker_path = candidate_root / 'positive-control/PASS.json'
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    marker['positive_binding_sha256'] = phase5_module.file_sha256(path)
    _canonical_file(marker_path, marker, sidecar=True)


def _refresh_phase3_positive_component_manifest(
    positive_directory: Path,
    orchestration: object,
) -> None:
    excluded_outputs = {
        'PASS.json',
        'PASS.json.sha256',
        'component-manifest.json',
        'component-manifest.json.sha256',
        'positive-binding.json',
        'positive-binding.json.sha256',
    }
    component_paths = sorted(
        path
        for path in positive_directory.rglob('*')
        if path.is_file()
        and path.relative_to(positive_directory).as_posix() not in excluded_outputs
    )
    _canonical_file(
        positive_directory / 'component-manifest.json',
        orchestration.component_manifest(component_paths, positive_directory),
        sidecar=True,
    )


def _refresh_phase3_positive_component_manifest_from_repository(
    candidate_root: Path,
    repository: Path,
) -> None:
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration component-manifest fixture',
    )
    _refresh_phase3_positive_component_manifest(
        candidate_root / 'positive-control',
        orchestration,
    )


def _rebind_phase3_positive_runtime_gate(
    candidate_root: Path,
    repository: Path,
) -> None:
    """Cascade a changed runtime-gate hash through ARM, ACK, result, and binding."""
    positive_directory = candidate_root / 'positive-control'
    runtime_gate_sha256 = phase5_module.file_sha256(positive_directory / 'runtime-gate.json')
    request_path = positive_directory / 'contact-control.arm.json'
    request = json.loads(request_path.read_text(encoding='utf-8'))
    request['runtime_gate_sha256'] = runtime_gate_sha256
    _canonical_file(request_path, request)
    request_sha256 = phase5_module.file_sha256(request_path)

    acknowledgment_path = positive_directory / 'contact-control.armed.json'
    acknowledgment = json.loads(acknowledgment_path.read_text(encoding='utf-8'))
    acknowledgment['arm_request_sha256'] = request_sha256
    acknowledgment['runtime_gate_sha256'] = runtime_gate_sha256
    _canonical_file(acknowledgment_path, acknowledgment)
    acknowledgment_sha256 = phase5_module.file_sha256(acknowledgment_path)

    result_path = positive_directory / 'contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result_arm = result['control']['arm']
    result_arm.update(
        {
            'acknowledgment': acknowledgment,
            'acknowledgment_sha256': acknowledgment_sha256,
            'request': request,
            'request_sha256': request_sha256,
        }
    )
    _canonical_file(result_path, result, sidecar=True)

    binding_path = positive_directory / 'positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['positive_control'] = copy.deepcopy(result)
    _refresh_phase3_positive_binding(candidate_root, binding)
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration runtime-gate rebind fixture',
    )
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))
    positive_plan = plan['positive_control']
    _canonical_file(
        positive_directory / 'contact-gate-revalidation.json',
        orchestration.reconcile_contact_gate_reobservation(
            positive_directory / 'runtime-gate.json',
            positive_directory / 'contact-stream-final-gate.json',
            build_binding=build_binding,
            expected_domain_id=positive_plan['ros_domain_id'],
            expected_gz_partition=positive_plan['gz_partition'],
        ),
        sidecar=True,
    )
    _refresh_phase3_positive_component_manifest(positive_directory, orchestration)


def _rebind_phase3_positive_command_probe(
    candidate_root: Path,
    repository: Path,
) -> None:
    """Cascade command-progress and ARMED bytes through the positive binding."""
    positive_directory = candidate_root / 'positive-control'
    progress_path = positive_directory / 'command-progress.json'
    progress = json.loads(progress_path.read_text(encoding='utf-8'))
    progress_sha256 = phase5_module.file_sha256(progress_path)
    acknowledgment_path = positive_directory / 'contact-control.armed.json'
    acknowledgment = json.loads(acknowledgment_path.read_text(encoding='utf-8'))
    result_path = positive_directory / 'contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['control']['arm']['acknowledgment'] = copy.deepcopy(acknowledgment)
    result['control']['arm']['acknowledgment_sha256'] = phase5_module.file_sha256(
        acknowledgment_path
    )
    _canonical_file(result_path, result, sidecar=True)

    binding_path = positive_directory / 'positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['positive_control'] = copy.deepcopy(result)
    external_quality = binding['benchmark_binding']['positive_control_external_quality']
    external_quality['collector_command_progress_sha256'] = progress_sha256
    reconciliation = binding['collector_reconciliation']
    reconciliation.update(
        {
            'command_progress_artifact_sha256': progress_sha256,
            'command_progress_observed_steady_ns': progress['observed_steady_ns'],
            'command_progress_stamp_ns': progress['stamp_ns'],
        }
    )
    _refresh_phase3_positive_binding(candidate_root, binding)
    _refresh_phase3_positive_component_manifest_from_repository(
        candidate_root,
        repository,
    )


def _rebind_phase3_positive_capture(
    candidate_root: Path,
    repository: Path,
) -> None:
    """Cascade changed capture bytes while retaining downstream semantic replay."""
    positive_directory = candidate_root / 'positive-control'
    capture_sha256 = phase5_module.file_sha256(positive_directory / 'capture.json')
    binding_path = positive_directory / 'positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['capture_sha256'] = capture_sha256
    binding['benchmark_binding']['positive_control_external_quality'][
        'collector_capture_sha256'
    ] = capture_sha256
    _refresh_phase3_positive_binding(candidate_root, binding)
    _refresh_phase3_positive_component_manifest_from_repository(
        candidate_root,
        repository,
    )


def _rewrite_process_command(process: dict, command: list[str]) -> None:
    process['command'] = command
    process['wrapped_command'] = [*process['wrapped_command'][:4], *command]


def _rebind_phase3_positive_raw(candidate_root: Path, repository: Path) -> None:
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration test helper',
    )
    positive_directory = candidate_root / 'positive-control'
    _refresh_phase3_positive_component_manifest(positive_directory, orchestration)
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    positive_binding_path = positive_directory / 'positive-binding.json'
    binding = orchestration.reconcile_positive_control(
        workspace=repository,
        build_binding=build_binding,
        result_path=positive_directory / 'contact-control-result.json',
        capture_path=positive_directory / 'capture.json',
        contact_progress_path=positive_directory / 'contact-progress.json',
        command_progress_path=positive_directory / 'command-progress.json',
        driver_ready_path=positive_directory / 'contact-control.ready.json',
        arm_request_path=positive_directory / 'contact-control.arm.json',
        armed_ack_path=positive_directory / 'contact-control.armed.json',
        runtime_gate_path=positive_directory / 'runtime-gate.json',
        manifest_path=repository / 'config/collision-coverage.yaml',
        collector_configuration_sha256=build_binding['collector_configuration_sha256'],
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    _canonical_file(positive_binding_path, binding, sidecar=True)
    marker_path = positive_directory / 'PASS.json'
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    marker['positive_binding_sha256'] = phase5_module.file_sha256(positive_binding_path)
    _canonical_file(marker_path, marker, sidecar=True)


def _remote_proof(
    repository: Path,
    *,
    sha: str,
    mode: str,
    run_id: int,
    completed_at: str,
    created_at: str,
    checked_at: str,
) -> dict[str, object]:
    return {
        'checked_at': checked_at,
        'provenance': {
            'command': {
                'argv': ['scripts/verify_phase5.sh', mode, sha],
                'cwd': str(repository),
            },
            'local_resolved_sha': sha,
            'platform': {
                'github_runner': {
                    'architecture': 'X64',
                    'environment': 'github-hosted',
                    'image_os': 'ubuntu24',
                    'image_version': '20260826.1',
                    'operating_system': 'Linux',
                },
                'os_release': {
                    'ID': 'ubuntu',
                    'PRETTY_NAME': 'Ubuntu 24.04 LTS',
                    'VERSION_ID': '24.04',
                },
                'python': {'executable': '/usr/bin/python3', 'version': '3.12.3'},
                'ros_distro': 'jazzy',
                'uname': ['Linux', 'runner', '6.8.0', '', 'x86_64', 'x86_64'],
                'wsl_distro_name': None,
            },
            'tools': {
                'gh': {
                    'argv': ['gh', '--version'],
                    'executable': '/usr/bin/gh',
                    'output': 'gh version fixture',
                },
                'git': {
                    'argv': ['git', '--version'],
                    'executable': '/usr/bin/git',
                    'output': 'git version fixture',
                },
                'sha256sum': {
                    'argv': ['sha256sum', '--version'],
                    'executable': '/usr/bin/sha256sum',
                    'output': 'sha256sum fixture',
                },
            },
        },
        'repository': {
            'name_with_owner': 'example/robotest',
            'url': 'https://github.com/example/robotest',
            'visibility': 'PUBLIC',
        },
        'run': {
            'conclusion': 'success',
            'completed_at': completed_at,
            'created_at': created_at,
            'head_sha': sha,
            'run_id': run_id,
            'run_url': f'https://github.com/example/robotest/actions/runs/{run_id}',
            'status': 'completed',
            'workflow_name': 'RoboTest CI',
        },
        'schema_version': 1,
        'status': 'PASS',
        'verification_scope': release_module.REMOTE_PROOF_SCOPE,
    }


def _commit_all(repository: Path, message: str) -> None:
    subprocess.run(['git', 'add', '-A'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit',
            '-q',
            '--no-gpg-sign',
            '-m',
            message,
        ],
        cwd=repository,
        check=True,
    )


def _amend_all(repository: Path) -> str:
    subprocess.run(['git', 'add', '-A'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit',
            '-q',
            '--amend',
            '--no-edit',
            '--no-gpg-sign',
        ],
        cwd=repository,
        check=True,
    )
    return subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()


def _producer_collision_fixture(repository: Path) -> tuple[dict, dict, dict]:
    release_module._activate_repository_packages(repository)
    package_root = repository / 'src/robotest_metrics'
    package_root_text = str(package_root)
    if package_root_text not in sys.path:
        sys.path.insert(0, package_root_text)
    fixture_path = package_root / 'test/conftest.py'
    spec = importlib.util.spec_from_file_location('_phase5_collision_fixture', fixture_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load collision fixture: {fixture_path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _, positive_control, benchmark_binding = module.collision_fixture()
    manifest = yaml.safe_load(
        (repository / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
    )
    coverage_sha = manifest['manifest_sha256']
    chassis = next(
        item['name'] for item in manifest['robot_collisions'] if item['role'] == 'chassis'
    )
    wall = 'phase3_contact_control_wall::link::collision'
    expected_pair = sorted((chassis, wall))
    support_entry = manifest['support_pairs'][0]
    support_pair = sorted(
        (support_entry['robot_collision'], support_entry['environment_collision'])
    )
    positive_control['configuration']['coverage_manifest_sha256'] = coverage_sha
    positive_control['configuration']['coverage_manifest_provenance'] = {
        field: manifest[field]
        for field in (
            'bridge_sha256',
            'contact_configuration_sha256',
            'rendered_sdf_sha256',
            'robot_description_sha256',
            'world_source_sha256',
        )
    } | {'coverage_manifest_sha256': coverage_sha}
    positive_control['configuration']['coverage_manifest_path'] = str(
        repository / 'config/collision-coverage.yaml'
    )
    positive_control['configuration']['expected_pair'] = expected_pair
    positive_control['configuration']['fixture']['expected_pair'] = expected_pair
    fixture_sha = release_module._canonical_sha256(positive_control['configuration']['fixture'])
    positive_control['configuration']['fixture_sha256'] = fixture_sha
    positive_control['identity']['scenario_sha256'] = fixture_sha
    contact = positive_control['control']['contact']
    contact['expected_pair'] = expected_pair
    contact['first_qualifying_contact']['normalized_pair'] = expected_pair
    contact['episodes'][0]['normalized_pairs'] = [expected_pair]
    for record in contact['snapshot_records']:
        if record['disposition'] == 'counted':
            record.update(
                {
                    'counterpart_collision': wall,
                    'counterpart_model': 'phase3_contact_control_wall',
                    'normalized_pair': expected_pair,
                    'robot_collision': chassis,
                }
            )
        elif record['disposition'] == 'support_ground_excluded':
            record['normalized_pair'] = support_pair
    for snapshot in contact['snapshots']:
        for record in snapshot['counted_snapshot_records']:
            record.update(
                {
                    'counterpart_model': 'phase3_contact_control_wall',
                    'normalized_pair': expected_pair,
                }
            )
    for name in ('benchmark_provenance', 'positive_control_provenance'):
        benchmark_binding[name].update(
            {
                'bridge_sha256': manifest['bridge_sha256'],
                'contact_configuration_sha256': manifest['contact_configuration_sha256'],
                'coverage_manifest_sha256': coverage_sha,
                'rendered_sdf_sha256': manifest['rendered_sdf_sha256'],
                'robot_description_sha256': manifest['robot_description_sha256'],
                'world_source_sha256': manifest['world_source_sha256'],
            }
        )
    benchmark_binding['positive_control_scenario_sha256'] = fixture_sha
    return manifest, positive_control, benchmark_binding


def _portfolio_replay_fixture(
    repository: Path,
    candidate_sha: str,
    command: dict[str, object],
    output: Path,
) -> dict[str, object]:
    """Write a producer-shaped PASS replay without executing a README command."""
    output.mkdir(mode=0o700, parents=True)
    output.parent.chmod(0o700)
    output.chmod(0o700)
    assert output.parent.lstat().st_mode & 0o7777 == 0o700
    assert output.lstat().st_mode & 0o7777 == 0o700
    empty_sha256 = hashlib.sha256(b'').hexdigest()
    assignments, argv = portfolio_module._parse_replay_argv(
        str(command['id']),
        str(command['executable']),
    )
    candidate = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    allowed = list(command['allowed_tracked_changes'])
    postcondition_payload = b'fixture postcondition\n'
    result: dict[str, object] = {
        'allowed_tracked_changes': allowed,
        'authorization': command['authorization'],
        'argv': argv,
        'command_sha256': command['command_sha256'],
        'documented_root': portfolio_module.DOCUMENTED_REPOSITORY,
        'effective_checkout_policy': portfolio_module.REPLAY_MAPPING_POLICY,
        'environment_assignments': assignments,
        'execution_role': 'fresh_detached_candidate_worktree',
        'expected_exit_code': command['expected_exit_code'],
        'failure_reasons': [],
        'id': command['id'],
        'observed_tracked_changes': [],
        'ordinal': command['ordinal'],
        'postcondition_files': [
            {
                'bytes': len(postcondition_payload),
                'path': path,
                'sha256': hashlib.sha256(postcondition_payload).hexdigest(),
            }
            for path in allowed
        ],
        'repository_preflight': {
            'effective_filter_configuration_absent': True,
            'info_attributes_absent': True,
            'sanitized_git_environment': True,
            'schema_version': 1,
        },
        'returncode': command['expected_exit_code'],
        'schema_version': 1,
        'status': 'PASS',
        'stderr': {
            'bytes': 0,
            'maximum_bytes': command['maximum_stderr_bytes'],
            'observed_bytes': 0,
            'overflow': False,
            'path': f'documentation/{output.name}/stderr.log',
            'sha256': empty_sha256,
        },
        'stdout': {
            'bytes': 0,
            'maximum_bytes': command['maximum_stdout_bytes'],
            'observed_bytes': 0,
            'overflow': False,
            'path': f'documentation/{output.name}/stdout.log',
            'sha256': empty_sha256,
        },
        'timed_out': False,
        'wall_duration_ns': 1,
        'worktree_head': candidate_sha,
    }
    for stream in ('stdout', 'stderr'):
        portfolio_module._write_once(output / f'{stream}.log', b'', mode=0o600)
    portfolio_module._write_json_once(
        output / 'environment.json',
        {
            'forbidden_overlay_keys': list(portfolio_module.FORBIDDEN_REPLAY_ENVIRONMENT),
            'inherited_environment': False,
            'policy': 'env -i with an explicit non-secret allowlist',
            'schema_version': 1,
            'set_keys': [
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
        },
    )
    if command['id'] in portfolio_module.RUFF_REQUIRED_REPLAY_IDS:
        tooling = {
            'bytes': portfolio_module.RUFF_BYTES,
            'destination': '.venv/bin/ruff',
            'required': True,
            'schema_version': 1,
            'sha256': portfolio_module.RUFF_SHA256,
            'source': '.venv/bin/ruff',
            'version': f'ruff {portfolio_module.RUFF_VERSION}',
        }
    else:
        tooling = {'required': False, 'schema_version': 1}
    portfolio_module._write_json_once(output / 'tooling-seed.json', tooling)
    before_tree = {
        'allowed_modified_paths': [],
        'candidate_tree_entry_count': candidate['candidate_tree_entry_count'],
        'candidate_tree_listing_sha256': candidate['candidate_tree_listing_sha256'],
        'checked_immutable_path_count': candidate['candidate_tree_entry_count'],
        'schema_version': 1,
        'status': 'PASS',
    }
    after_tree = {
        **before_tree,
        'allowed_modified_paths': allowed,
        'checked_immutable_path_count': candidate['candidate_tree_entry_count'] - len(allowed),
    }
    before_state = {
        'candidate_git_sha': candidate_sha,
        'changed_tracked_paths': [],
        'detached_head': True,
        'git_head': candidate_sha,
        'status_bytes': 0,
        'status_sha256': empty_sha256,
        'tracked_tree': before_tree,
        'untracked_paths': [],
    }
    after_state = {**before_state, 'tracked_tree': after_tree}
    portfolio_module._write_json_once(output / 'source-state-before.json', before_state)
    portfolio_module._write_json_once(output / 'source-state-after.json', after_state)
    portfolio_module._write_json_once(output / 'result.json', result)
    return result


def _portfolio_svg(path: Path, label: str) -> None:
    path.write_text(
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 960 540">'
        f'<title>{label}</title><rect width="960" height="540" fill="#ffffff"/>'
        '<path d="M 80 270 L 880 270" stroke="#111111" stroke-width="8"/>'
        '</svg>\n',
        encoding='utf-8',
    )


def _phase5_fixture_png() -> bytes:
    def chunk(kind: bytes, payload: bytes) -> bytes:
        return (
            struct.pack('>I', len(payload))
            + kind
            + payload
            + struct.pack('>I', zlib.crc32(kind + payload) & 0xFFFFFFFF)
        )

    width, height = 960, 540
    row = b'\x00' + (b'\xf4\xf4\xf4' * width)
    return (
        b'\x89PNG\r\n\x1a\n'
        + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
        + chunk(b'IDAT', zlib.compress(row * height, level=9))
        + chunk(b'IEND', b'')
    )


def _finalize_portfolio_fixture(
    repository: Path,
    candidate_root: Path,
    candidate_sha: str,
) -> tuple[Path, Path, str]:
    """Create one finalized raw portfolio attempt without executing documentation units."""
    attempt_id = '20260826T000200Z-1'
    original_replay = portfolio_module._replay_one_command
    portfolio_module._replay_one_command = _portfolio_replay_fixture
    try:
        prepare = portfolio_module.prepare_portfolio(
            repository,
            candidate_sha,
            candidate_root,
            authorize_mutating_commands=True,
            attempt_id=attempt_id,
        )
    finally:
        portfolio_module._replay_one_command = original_replay
    assert prepare['status'] == 'REVIEW_REQUIRED'
    architecture_svg = repository.parent / 'architecture-fixture.svg'
    release_flow_svg = repository.parent / 'release-flow-fixture.svg'
    _portfolio_svg(architecture_svg, 'Architecture fixture')
    _portfolio_svg(release_flow_svg, 'Release flow fixture')
    candidate = portfolio_module.validate_candidate_contract(repository, candidate_sha)
    prepared = datetime.fromisoformat(str(prepare['prepared_utc']).replace('Z', '+00:00'))
    rendered_utc = prepared.astimezone(UTC).isoformat().replace('+00:00', 'Z')
    reviewed_utc = rendered_utc
    render_paths = {
        'architecture': architecture_svg,
        'release-flow': release_flow_svg,
    }
    review = {
        'attempt_id': attempt_id,
        'candidate_git_sha': candidate_sha,
        'diagrams': [
            {
                'diagram_id': identifier,
                'render_sha256': phase5_module.file_sha256(render_paths[identifier]),
                'rendered_utc': rendered_utc,
                'renderer': {
                    'identity': 'fixture-mermaid-renderer',
                    'mode': 'argv',
                    'reference': ['fixture-mermaid-renderer', '--input', f'{identifier}.mmd'],
                    'version': '1.0.0-fixture',
                },
                'source_sha256': next(
                    item['source_sha256']
                    for item in candidate['diagrams']
                    if item['id'] == identifier
                ),
                'verdict': 'PASS',
            }
            for identifier in portfolio_module.DIAGRAM_IDS
        ],
        'reviewed_utc': reviewed_utc,
        'reviewer': 'Phase 5 human fixture reviewer',
        'schema_version': 1,
    }
    visual_review = repository.parent / 'portfolio-visual-review.json'
    _canonical_file(visual_review, review)
    finalized = portfolio_module.finalize_portfolio(
        repository,
        candidate_sha,
        candidate_root,
        attempt_id,
        architecture_svg,
        release_flow_svg,
        visual_review,
    )
    assert finalized['status'] == 'PASS'
    portfolio_root = repository / portfolio_module.PORTFOLIO_RAW_PREFIX / candidate_sha
    raw_proof = repository / str(finalized['portfolio_proof_path'])
    return portfolio_root, raw_proof, attempt_id


def _set_local_aggregate_after_portfolio(
    local_aggregate: Path,
    portfolio_raw_proof: Path,
) -> None:
    """Place the bare aggregate deterministically after raw portfolio finalization."""
    aggregate = json.loads(local_aggregate.read_text(encoding='utf-8'))
    portfolio = json.loads(portfolio_raw_proof.read_text(encoding='utf-8'))
    finalized = datetime.fromisoformat(str(portfolio['finalized_utc']).replace('Z', '+00:00'))
    aggregate['checked_at'] = (
        (finalized + timedelta(seconds=1)).astimezone(UTC).isoformat().replace('+00:00', 'Z')
    )
    _canonical_file(local_aggregate, aggregate)
    local_aggregate.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(local_aggregate)}  {local_aggregate.name}\n',
        encoding='ascii',
    )
    local_aggregate.with_suffix('.checksum-validation.txt').write_text(
        f'{local_aggregate.name}: OK\n',
        encoding='utf-8',
    )


def _release_fixture_install_root() -> Path:
    configured = os.environ.get(PHASE5_FIXTURE_INSTALL_ROOT_ENV)
    if configured is None:
        candidate = REPOSITORY / 'install'
    else:
        if not configured or configured.strip() != configured:
            raise RuntimeError(f'{PHASE5_FIXTURE_INSTALL_ROOT_ENV} is invalid')
        candidate = Path(configured)
    if not candidate.is_absolute():
        raise RuntimeError(f'{PHASE5_FIXTURE_INSTALL_ROOT_ENV} must be absolute')
    if candidate.is_symlink():
        raise RuntimeError(f'{PHASE5_FIXTURE_INSTALL_ROOT_ENV} must not be a symlink')
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as error:
        raise RuntimeError(f'{PHASE5_FIXTURE_INSTALL_ROOT_ENV} is unavailable') from error
    if not resolved.is_dir():
        raise RuntimeError(f'{PHASE5_FIXTURE_INSTALL_ROOT_ENV} must be a directory')
    return resolved


def _build_release_fixture(tmp_path: Path) -> dict[str, Path | str]:
    repository = tmp_path / 'repository'
    repository.mkdir()
    shutil.copy2(REPOSITORY / '.gitignore', repository / '.gitignore')
    ignored = shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.pyc')
    for directory in ('config', 'docs', 'packaging', 'scenarios', 'scripts', 'supervisor', 'tests'):
        shutil.copytree(REPOSITORY / directory, repository / directory, ignore=ignored)
    fixture_install_root = _release_fixture_install_root()
    for package in release_module.PHASE3_RUNTIME_PACKAGES:
        shutil.copytree(
            REPOSITORY / f'src/{package}',
            repository / f'src/{package}',
            ignore=ignored,
        )
        installed_package = fixture_install_root / package
        if not installed_package.is_dir() or installed_package.is_symlink():
            raise RuntimeError(f'missing regular installed fixture package: {package}')
        shutil.copytree(
            installed_package,
            repository / f'install/{package}',
            symlinks=False,
            ignore=ignored,
        )
        python_source = repository / f'src/{package}/{package}'
        if python_source.is_dir():
            site_packages = next(
                (repository / f'install/{package}/lib').glob('python*/site-packages')
            )
            for egg_link in site_packages.glob('*.egg-link'):
                egg_link.unlink()
            installed_python = site_packages / package
            if installed_python.exists():
                if installed_python.is_symlink() or not installed_python.is_dir():
                    raise RuntimeError(
                        f'installed Python package is not a regular directory: {installed_python}'
                    )
                shutil.rmtree(installed_python)
            shutil.copytree(python_source, installed_python)
    for name in ('.editorconfig', '.gitattributes', 'LICENSE', 'README.md', 'pyproject.toml'):
        shutil.copy2(REPOSITORY / name, repository / name)
    gate_fixture_source = Path(sys.executable).resolve(strict=True)
    gate_build = repository / 'build/robotest_sim/contact_stream_gate'
    gate_install = repository / 'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
    aggregator_build = repository / 'build/robotest_sim/librobotest_contact_aggregator_system.so'
    aggregator_install = repository / (
        'install/robotest_sim/lib/robotest_sim/librobotest_contact_aggregator_system.so'
    )
    gate_build.parent.mkdir(parents=True, exist_ok=True)
    gate_install.parent.mkdir(parents=True, exist_ok=True)
    for binary_path in (gate_build, gate_install, aggregator_build, aggregator_install):
        shutil.copy2(gate_fixture_source, binary_path)
    gate_build.chmod(0o755)
    gate_install.chmod(0o755)
    phase0_version = repository / 'artifacts/evidence/phase0/phase0-versions.json'
    _canonical_file(phase0_version, {'checked_at': 'before', 'schema_version': 1})
    call_log = tmp_path / 'release-calls.jsonl'
    fake = """#!/usr/bin/env bash
python3 - "$0" "$@" <<'PY'
import json
import os
import pathlib
import sys

with pathlib.Path(os.environ['CALL_LOG']).open('a', encoding='utf-8') as output:
    record = {'script': pathlib.Path(sys.argv[1]).name, 'args': sys.argv[2:]}
    output.write(json.dumps(record) + '\\n')
PY
"""
    for phase in (3, 4, 5):
        verifier = repository / f'scripts/verify_phase{phase}.sh'
        verifier.write_text(fake, encoding='utf-8')
        verifier.chmod(0o755)
    (repository / 'scripts/verify_all.sh').chmod(0o755)
    subprocess.run(['git', 'init', '-q'], cwd=repository, check=True)
    _commit_all(repository, 'candidate')
    candidate_sha = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()
    subprocess.run(
        ['git', 'remote', 'add', 'origin', 'https://github.com/example/robotest.git'],
        cwd=repository,
        check=True,
    )
    _canonical_file(phase0_version, {'checked_at': 'after', 'schema_version': 1})

    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration fixture',
    )
    gate_source_inventory_sha256 = orchestration.canonical_sha256(
        orchestration.contact_gate_source_inventory(repository)
    )
    embedded_gate_source = (
        b'ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
        + gate_source_inventory_sha256.encode('ascii')
        + b'\0'
    )
    for binary_path in (gate_build, gate_install, aggregator_build, aggregator_install):
        with binary_path.open('ab') as stream:
            stream.write(embedded_gate_source)
    build_binding = orchestration.build_binding(
        repository,
        git_sha=candidate_sha,
        git_status_porcelain='',
    )

    local_aggregate = repository / 'artifacts/evidence/phase0/verify-all.json'
    _canonical_file(
        local_aggregate,
        {
            'checked_at': '2026-08-26T00:00:00+00:00',
            'command': {'argv': ['scripts/verify_all.sh'], 'cwd': str(repository)},
            'mode': 'static',
            'outstanding_gates': list(release_module.LOCAL_OUTSTANDING_GATES),
            'release_eligible': False,
            'release_evidence': None,
            'results': [
                {
                    'exit_code': '0',
                    'phase': str(phase),
                    'status': 'passed',
                    'verifier': f'scripts/verify_phase{phase}.sh',
                }
                for phase in range(6)
            ],
            'schema_version': 4,
            'source': {
                'git_dirty_end': True,
                'git_dirty_start': False,
                'git_sha_end': candidate_sha,
                'git_sha_start': candidate_sha,
                'git_status_porcelain_end': (' M artifacts/evidence/phase0/phase0-versions.json'),
                'git_status_porcelain_start': '',
                'generated_evidence_delta': {
                    'allowed_paths': list(release_module.ALLOWED_GENERATED_DELTA),
                    'changed_files': [
                        {
                            'bytes': phase0_version.stat().st_size,
                            'path': 'artifacts/evidence/phase0/phase0-versions.json',
                            'sha256': phase5_module.file_sha256(phase0_version),
                        }
                    ],
                },
                'source_unchanged': True,
            },
            'status': 'incomplete',
            'verification_scope': release_module.LOCAL_AGGREGATE_SCOPE,
        },
    )
    local_aggregate.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(local_aggregate)}  {local_aggregate.name}\n',
        encoding='ascii',
    )
    local_aggregate.with_suffix('.checksum-validation.txt').write_text(
        f'{local_aggregate.name}: OK\n', encoding='utf-8'
    )

    candidate_id = 'candidate-1'
    candidate_root = repository / f'artifacts/evidence/phase3-benchmarks/{candidate_id}'
    coverage_manifest, positive_control, benchmark_binding = _producer_collision_fixture(repository)
    for name in ('benchmark_provenance', 'positive_control_provenance'):
        benchmark_binding[name]['collector_configuration_sha256'] = build_binding[
            'collector_configuration_sha256'
        ]
    positive_control['identity']['run_id'] = f'{candidate_id}-positive-control'
    scenario_names = release_module.PHASE3_SCENARIO_NAMES
    scenario_paths = {
        1: 'scenarios/phase3_s1_baseline.yaml',
        2: 'scenarios/phase3_s2_static_obstacle.yaml',
        3: 'scenarios/phase3_s3_dynamic_obstacle.yaml',
        4: 'scenarios/phase3_s4_lidar_dropout.yaml',
        5: 'scenarios/phase3_s5_odom_drift.yaml',
    }
    trials = []
    for index in range(15):
        scenario_id = index // 3 + 1
        repetition = index % 3
        run_id = f'{candidate_id}-s{scenario_id}-r{repetition}-i{index:02d}'
        scenario_path = repository / scenario_paths[scenario_id]
        scenario_sha = phase5_module.file_sha256(scenario_path)
        trials.append(
            {
                'candidate_id': candidate_id,
                'gz_partition': f'robotest_p3_{candidate_id}_{index:02d}',
                'repetition_index': repetition,
                'ros_domain_id': 100 + index,
                'run_id': run_id,
                'scenario_id': scenario_id,
                'scenario_name': scenario_names[scenario_id],
                'scenario_path': scenario_paths[scenario_id],
                'scenario_sha256': scenario_sha,
                'suite_index': index,
            }
        )
    smoke_id = f'{candidate_id}-smoke-s1-r0'
    plan_path = candidate_root / 'suite-plan.json'
    binding_path = candidate_root / 'build-binding.json'
    _canonical_file(
        plan_path,
        {
            'aggregate_metrics': list(release_module.PHASE3_AGGREGATE_METRICS),
            'candidate_id': candidate_id,
            'cpu_affinity': [0, 1, 2, 3, 4, 5],
            'domain_base': 100,
            'positive_control': {
                'gz_partition': f'robotest_p3_{candidate_id}_positive_control',
                'ros_domain_id': 115,
                'run_id': f'{candidate_id}-positive-control',
            },
            'producer': 'robotest_phase3/benchmark_orchestrator',
            'schema_version': 1,
            'smoke': {
                'gz_partition': f'robotest_p3_{candidate_id}-smoke_00',
                'ros_domain_id': 116,
                'run_id': smoke_id,
                'scenario_path': scenario_paths[1],
            },
            'trials': trials,
        },
        sidecar=True,
    )
    _canonical_file(binding_path, build_binding, sidecar=True)
    _canonical_file(
        candidate_root / 'prepared.json',
        {
            'build_binding_sha256': phase5_module.file_sha256(binding_path),
            'candidate_id': candidate_id,
            'git_sha': candidate_sha,
            'producer': 'robotest_phase3/benchmark_orchestrator',
            'suite_plan_sha256': phase5_module.file_sha256(plan_path),
        },
        sidecar=True,
    )
    positive_directory = candidate_root / 'positive-control'
    positive_result_path = positive_directory / 'contact-control-result.json'
    positive_capture_path = positive_directory / 'capture.json'
    positive_binding = positive_directory / 'positive-binding.json'
    _write_phase3_positive_handshake(
        repository,
        positive_directory,
        orchestration=orchestration,
        build_binding=build_binding,
        positive_control=positive_control,
        ros_domain_id=115,
        gz_partition=f'robotest_p3_{candidate_id}_positive_control',
    )
    positive_sha = release_module._canonical_sha256(positive_control)
    benchmark_binding['positive_control_json_sha256'] = positive_sha
    benchmark_binding['positive_control_run_id'] = positive_control['identity']['run_id']
    _canonical_file(positive_result_path, positive_control, sidecar=True)
    metrics_fixture = release_module._load_repository_module(
        repository,
        'src/robotest_metrics/test/conftest.py',
        'Phase 3 positive-control collector fixture',
    )
    core = metrics_fixture.CollectorCore()
    expected_pair = positive_control['control']['contact']['expected_pair']
    support_entry = coverage_manifest['support_pairs'][0]
    support_contact = {
        'collision1': support_entry['robot_collision'],
        'collision2': support_entry['environment_collision'],
        'maximum_normal_force_n': 5.0,
        'maximum_penetration_depth_m': 0.01,
    }
    for snapshot in positive_control['control']['contact']['snapshots']:
        stamp_ns = snapshot['sim_stamp_ns']
        contacts = [support_contact]
        if snapshot['exact_pair_count']:
            contacts.append(
                {
                    'collision1': expected_pair[0],
                    'collision2': expected_pair[1],
                    'maximum_normal_force_n': 2.0,
                    'maximum_penetration_depth_m': 0.005,
                }
            )
        core.observe_clock(stamp_ns)
        core.record(
            'contacts',
            {
                'contacts': contacts,
                'delivery_clock_offset_ns': 0,
                'delivery_clock_stamp_ns': stamp_ns,
                'frame_id': '',
                'stamp_ns': stamp_ns,
            },
        )
    command_progress = json.loads(
        (positive_directory / 'command-progress.json').read_text(encoding='utf-8')
    )
    core.record(
        'cmd_vel',
        {
            'angular_z_rad_s': command_progress['angular_z_rad_s'],
            'linear_x_m_s': command_progress['linear_x_m_s'],
            'linear_y_m_s': command_progress['linear_y_m_s'],
            'stamp_ns': command_progress['stamp_ns'],
        },
    )
    for command in positive_control['control']['command_trace']:
        core.record(
            'cmd_vel',
            {
                'angular_z_rad_s': command['angular_z'],
                'linear_x_m_s': command['linear_x'],
                'linear_y_m_s': 0.0,
                'stamp_ns': command['sim_stamp_ns'],
            },
        )
    core.observe_clock(3_500_000_000)
    positive_capture = core.snapshot()
    positive_capture.update(
        {
            'capture_schema_version': 1,
            'finished_steady_wall_ns': 3_500_000_000,
            'started_steady_wall_ns': 0,
            'stop_reason': 'stop_file',
        }
    )
    _canonical_file(positive_capture_path, positive_capture)
    positive_contact_items = positive_capture['streams']['contacts']['items']
    _canonical_file(
        positive_directory / 'contact-progress.json',
        {
            'latest_retained_stamp_ns': positive_contact_items[-1]['stamp_ns'],
            'producer': 'robotest_metrics/metrics_collector',
            'public_topic': '/robotest/validation/contacts',
            'retained_message_count': len(positive_contact_items),
            'schema_version': 1,
        },
    )
    resource_path = positive_directory / 'resources.jsonl'
    resource_samples = [
        {
            'affinity_checked_pid_count': 1,
            'affinity_escape_count': 0,
            'affinity_escape_prefix': [],
            'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5],
            'affinity_unreadable_count': 0,
            'affinity_unreadable_pid_prefix': [],
            'cpu_percent': 50.0,
            'missing_count': 0,
            'oom_kill': False,
            'phase': 'before_launch',
            'pid_reuse_detected': False,
            'rss_sum_bytes': 1024,
            'wsl_memory_bytes': 2048,
            'wsl_swap_bytes': 0,
        },
        {
            'affinity_checked_pid_count': 1,
            'affinity_escape_count': 0,
            'affinity_escape_prefix': [],
            'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5],
            'affinity_unreadable_count': 0,
            'affinity_unreadable_pid_prefix': [],
            'cpu_percent': 25.0,
            'missing_count': 0,
            'oom_kill': False,
            'phase': 'after_shutdown',
            'pid_reuse_detected': False,
            'rss_sum_bytes': 512,
            'wsl_memory_bytes': 1536,
            'wsl_swap_bytes': 0,
        },
    ]
    resource_path.write_text(
        ''.join(json.dumps(sample, sort_keys=True) + '\n' for sample in resource_samples),
        encoding='utf-8',
    )
    resource_summary = orchestration.summarize_resources(resource_path)
    component_manifest_path = positive_directory / 'component-manifest.json'
    component_document = orchestration.component_manifest(
        sorted(path for path in positive_directory.rglob('*') if path.is_file()),
        positive_directory,
    )
    _canonical_file(component_manifest_path, component_document, sidecar=True)
    assert orchestration.verify_component_manifest(component_document, positive_directory)
    positive_binding_document = orchestration.reconcile_positive_control(
        workspace=repository,
        build_binding=build_binding,
        result_path=positive_result_path,
        capture_path=positive_capture_path,
        contact_progress_path=positive_directory / 'contact-progress.json',
        command_progress_path=positive_directory / 'command-progress.json',
        driver_ready_path=positive_directory / 'contact-control.ready.json',
        arm_request_path=positive_directory / 'contact-control.arm.json',
        armed_ack_path=positive_directory / 'contact-control.armed.json',
        runtime_gate_path=positive_directory / 'runtime-gate.json',
        manifest_path=repository / 'config/collision-coverage.yaml',
        collector_configuration_sha256=build_binding['collector_configuration_sha256'],
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    assert positive_binding_document['positive_control_json_sha256'] == positive_sha
    _canonical_file(positive_binding, positive_binding_document, sidecar=True)
    _canonical_file(
        positive_directory / 'PASS.json',
        {
            'positive_binding_sha256': phase5_module.file_sha256(positive_binding),
            'producer': 'robotest_phase3/benchmark_orchestrator',
            'resource_summary': resource_summary,
            'status': 'PASS',
        },
        sidecar=True,
    )
    run_results = []
    run_ids = []
    result_hashes = []
    for trial in trials:
        index = trial['suite_index']
        result_directory = candidate_root / f'runs/{index:02d}/result'
        result_hashes.append(
            _phase3_bundle(
                repository,
                result_directory,
                orchestration=orchestration,
                plan=trial,
                git_sha=candidate_sha,
                build_binding=build_binding,
                positive_binding_path=positive_binding,
            )
        )
        run_results.append(
            json.loads((result_directory / 'run-result.json').read_text(encoding='utf-8'))
        )
        run_ids.append(trial['run_id'])
    smoke_plan = {
        **trials[0],
        'candidate_id': f'{candidate_id}-smoke',
        'gz_partition': f'robotest_p3_{candidate_id}-smoke_00',
        'ros_domain_id': 116,
        'run_id': smoke_id,
    }
    smoke_sha = _phase3_bundle(
        repository,
        candidate_root / 'smoke/result',
        orchestration=orchestration,
        plan=smoke_plan,
        git_sha=candidate_sha,
        build_binding=build_binding,
        positive_binding_path=positive_binding,
        full_stack_stdout_bytes=_phase3_contact_profile_stdout(
            linux_tid=40_000 + smoke_plan['ros_domain_id']
        ),
    )
    _canonical_file(
        candidate_root / 'smoke/PASS.json',
        {
            'producer': 'robotest_phase3/benchmark_orchestrator',
            'run_result_sha256': smoke_sha,
            'status': 'PASS',
        },
        sidecar=True,
    )
    smoke_profile_path, smoke_profile_sha256 = _write_phase3_smoke_profile_fixture(
        repository,
        candidate_root,
        candidate_sha=candidate_sha,
    )
    aggregate = release_module._recompute_phase3_aggregate(repository, run_results, result_hashes)
    aggregate_path = candidate_root / 'aggregate/aggregate-result.json'
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )

    phase4_run = repository / 'artifacts/evidence/phase4/runs/phase4-20260826T100000Z-1'
    phase4_run.mkdir(parents=True)
    raw = phase4_run / 'timeline.jsonl'
    raw.write_text('{"kind":"cleanup_complete"}\n' * 10, encoding='utf-8')
    events = phase4_run / 'supervisor-events.jsonl'
    events.write_text('{"event":"fixture"}\n' * 10, encoding='utf-8')
    _canonical_file(phase4_run / 'supervisor-events.meta.json', {'dropped_events': 0})
    context = {
        'active_overlay_target': '/opt/robotest/overlay',
        'baseline_package': {'path': '/tmp/robotest-baseline.deb', 'sha256': 'a' * 64},
        'cpuset': '0-5',
        'isolation': {
            'domain_was_unused': True,
            'gz_partition': 'robotest_p4_fixture',
            'inspected_processes': 20,
            'partition_was_unused': True,
            'ros_domain_id': 120,
            'run_id': phase4_run.name,
            'schema_version': 1,
            'unreadable_process_environments': 0,
        },
        'lifecycle_evidence': {'path': '/tmp/lifecycle.json', 'sha256': 'b' * 64},
        'package_directory': '/tmp/packages',
        'run_id': phase4_run.name,
        'schema_version': 1,
        'source_git_commit': candidate_sha,
        'source_git_dirty': False,
        'started_utc': '2026-08-26T00:00:00+00:00',
        'upgrade_package': {'path': '/tmp/robotest-upgrade.deb', 'sha256': 'c' * 64},
    }
    _canonical_file(phase4_run / 'context.json', context)
    for name in sorted(
        release_module.PHASE4_REQUIRED_RAW
        - {
            'context.json',
            'supervisor-events.jsonl',
            'supervisor-events.meta.json',
            'timeline.jsonl',
        }
    ):
        path = phase4_run / name
        if path.suffix == '.json':
            _canonical_file(path, {'fixture': name})
        else:
            path.write_text('fixture\n', encoding='utf-8')
    raw_hashes = {
        path.relative_to(phase4_run).as_posix(): phase5_module.file_sha256(path)
        for path in sorted(phase4_run.rglob('*'))
        if path.is_file()
    }
    scenario6 = {
        'identity': {
            'completed_utc': '2026-08-26T00:01:00+00:00',
            'gz_partition': 'robotest_p4_fixture',
            'managed_child': 'robotest-stack',
            'ros_domain_id': 120,
            'run_id': phase4_run.name,
            'source_git_commit': candidate_sha,
            'source_git_dirty': False,
            'started_utc': '2026-08-26T00:00:00+00:00',
            'supervisor_unit': 'robotest-supervisor.service',
        },
        'measurements': {
            'actual_restart_backoff_wall_s': 1.0,
            'followup_mission_exit_code': 0,
            'interrupted_mission_exit_code': 1,
            'observed_ready_restore_after_503_wall_s': 2.0,
            'original_child_pgid': 1001,
            'original_child_pid': 1001,
            'original_group_empty_after_injection_wall_s': 1.0,
            'ready_503_after_injection_wall_s': 1.0,
            'replacement_child_pgid': 1002,
            'replacement_child_pid': 1002,
            'replacement_child_start_count': 1,
            'restart_scheduled_count': 1,
            'supervisor_main_pid': 1000,
            'supervisor_recovery_time_wall_s': 2.0,
        },
        'producer': 'robotest_phase4/acceptance_verifier',
        'quality': {
            'checks': {name: True for name in sorted(release_module.PHASE4_CHECKS)},
            'event_count': 10,
            'event_trace_dropped': 0,
            'raw_evidence_sha256': raw_hashes,
            'timeline_count': 10,
        },
        'schema_version': 1,
        'targets': {
            'heartbeat_period_wall_s': 0.5,
            'heartbeat_stale_wall_s': 2.0,
            'ready_failure_wall_s_max': 3.0,
            'ready_restore_after_detection_wall_s_max': 30.0,
            'restart_attempt_limit': 4,
            'restart_backoff_wall_s': [1.0, 2.0, 4.0, 8.0],
            'restart_window_wall_s': 60.0,
            'stable_reset_wall_s': 60.0,
            'termination_allowance_wall_s': 5.0,
        },
        'verdict': {'accepted': True, 'failure_count': 0, 'failures': [], 'status': 'PASS'},
    }
    scenario6_path = phase4_run / 'scenario6-result.json'
    _canonical_file(scenario6_path, scenario6)
    (phase4_run / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(scenario6))
    phase4_records = [
        f'{phase5_module.file_sha256(path)}  {path.relative_to(phase4_run).as_posix()}'
        for path in sorted(phase4_run.rglob('*'))
        if path.is_file()
    ]
    (phase4_run / 'SHA256SUMS').write_text('\n'.join(phase4_records) + '\n', encoding='ascii')

    phase4_run = repository / 'artifacts/evidence/phase4/runs/phase4-20260826T000000Z-999'
    phase4_fixture_module = release_module._load_repository_module(
        REPOSITORY,
        'tests/phase4_acceptance_test.py',
        'Phase 4 producer fixture',
    )
    phase4_fixture_module._write_pass_fixture(phase4_run)
    context_path = phase4_run / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    context['cpuset'] = '0-5'
    context['source_git_commit'] = candidate_sha
    context['source_git_dirty'] = False
    context['isolation'].update(
        {
            'inspected_processes': 20,
            'unreadable_process_environments': 0,
        }
    )
    _canonical_file(context_path, context)
    scenario6_path = _rebind_phase4_evidence(
        repository,
        phase4_run,
        completed_utc='2026-08-26T00:01:00.000000Z',
    )
    deferred_local_paths = (
        phase0_version,
        local_aggregate,
        local_aggregate.with_suffix('.SHA256SUMS'),
        local_aggregate.with_suffix('.checksum-validation.txt'),
    )
    deferred_local_payloads = {path: path.read_bytes() for path in deferred_local_paths}
    phase0_version.write_bytes(
        subprocess.run(
            ['git', 'show', f'{candidate_sha}:artifacts/evidence/phase0/phase0-versions.json'],
            cwd=repository,
            capture_output=True,
            check=True,
        ).stdout
    )
    for path in deferred_local_paths[1:]:
        path.unlink()
    try:
        portfolio_root, portfolio_raw_proof, portfolio_attempt_id = _finalize_portfolio_fixture(
            repository,
            candidate_root,
            candidate_sha,
        )
    finally:
        for path, payload in deferred_local_payloads.items():
            path.write_bytes(payload)
    _set_local_aggregate_after_portfolio(local_aggregate, portfolio_raw_proof)

    remote_root = repository / 'docs/results/phase-5'
    remote_path = remote_root / f'remote-{candidate_sha}.json'
    _canonical_file(
        remote_path,
        _remote_proof(
            repository,
            sha=candidate_sha,
            mode='--remote',
            run_id=42,
            completed_at='2026-08-26T00:00:30Z',
            created_at='2026-08-26T00:00:00Z',
            checked_at='2026-08-26T00:01:00+00:00',
        ),
    )
    remote_manifest = remote_path.with_suffix('.SHA256SUMS')
    remote_manifest.write_text(
        f'{phase5_module.file_sha256(remote_path)}  {remote_path.name}\n', encoding='ascii'
    )
    remote_path.with_suffix('.checksum-validation.txt').write_text(
        f'{remote_path.name}: OK\n', encoding='utf-8'
    )
    release_docs_module.write_release_documents(
        repository,
        aggregate_path,
        scenario6_path,
        candidate_root,
        portfolio_root,
        portfolio_raw_proof,
    )
    portfolio_proof = repository / portfolio_module.portfolio_projection_paths(candidate_sha)[0]
    _commit_all(repository, 'evidence')
    evidence_sha = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()
    evidence_remote = (
        repository / f'artifacts/evidence/phase5/remote-evidence-commit/remote-{evidence_sha}.json'
    )
    evidence_proof = _remote_proof(
        repository,
        sha=evidence_sha,
        mode='--remote-evidence-commit',
        run_id=43,
        completed_at='2026-08-26T00:10:30Z',
        created_at='2026-08-26T00:10:00Z',
        checked_at='2026-08-26T00:11:00+00:00',
    )
    _canonical_file(evidence_remote, evidence_proof)
    evidence_remote.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(evidence_remote)}  {evidence_remote.name}\n',
        encoding='ascii',
    )
    evidence_remote.with_suffix('.checksum-validation.txt').write_text(
        f'{evidence_remote.name}: OK\n', encoding='utf-8'
    )
    return {
        'aggregate': aggregate_path,
        'call_log': call_log,
        'candidate_root': candidate_root,
        'candidate_sha': candidate_sha,
        'evidence_remote': evidence_remote,
        'evidence_sha': evidence_sha,
        'local_aggregate': local_aggregate,
        'phase4_run': phase4_run,
        'portfolio_attempt_id': portfolio_attempt_id,
        'portfolio_proof': portfolio_proof,
        'portfolio_raw_proof': portfolio_raw_proof,
        'portfolio_root': portfolio_root,
        'remote': remote_path,
        'repository': repository,
        'scenario6': scenario6_path,
        'smoke_profile': smoke_profile_path,
        'smoke_profile_sha256': smoke_profile_sha256,
    }


def _relocated_value(value: object, old_root: str, new_root: str) -> object:
    if isinstance(value, dict):
        return {key: _relocated_value(item, old_root, new_root) for key, item in value.items()}
    if isinstance(value, list):
        return [_relocated_value(item, old_root, new_root) for item in value]
    if isinstance(value, str):
        return value.replace(old_root, new_root)
    return value


def _relocate_json(
    path: Path,
    old_root: str,
    new_root: str,
    *,
    sidecar: bool = False,
) -> dict:
    document = json.loads(path.read_text(encoding='utf-8'))
    relocated = _relocated_value(document, old_root, new_root)
    assert isinstance(relocated, dict)
    _canonical_file(path, relocated, sidecar=sidecar)
    return relocated


def _rebind_phase3_smoke_profile_outputs(
    repository: Path,
    candidate_root: Path,
) -> tuple[Path, str]:
    """Rebind valid full-stack outputs while preserving sampled host identity."""
    profiler = release_module._load_repository_module(
        repository,
        'tests/phase3_smoke_host_profiler.py',
        'rebound Phase 3 smoke host profiler',
    )
    candidate_id = candidate_root.name
    profile_path = (
        repository
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{candidate_id}-smoke-profile.json'
    )
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    anchor = profile['anchor']
    full_stack = profiler.wait_for_full_stack_close(
        candidate_root / 'smoke',
        anchor,
        monotonic=lambda: 0.0,
        sleep=lambda _duration: None,
        profile_started_monotonic_ns=profile['profile_started_monotonic_ns'],
    )
    full_stack['pre_smoke_outputs_absent'] = True
    profile_started_epoch_ns = profiler._profile_utc(
        profile['profile_started_utc'],
        'profile start UTC',
    )
    profile['finished_utc'] = profiler._utc(
        profile_started_epoch_ns
        + full_stack['finished_steady_ns']
        - profile['profile_started_monotonic_ns']
    )
    profile['full_stack_process'] = full_stack
    host_thread_bindings = profile['contact_profile']['host_thread_bindings']
    contact = profiler.parse_contact_profile_logs(candidate_root / 'smoke')
    profiler.reconcile_contact_log_sources(contact, full_stack)
    contact['host_thread_bindings'] = host_thread_bindings
    profile['contact_profile'] = contact
    _canonical_file(profile_path, profile, sidecar=True)
    binding = profiler.validate_campaign_smoke_profile(repository, candidate_root, candidate_id)
    digest = phase5_module.file_sha256(profile_path)
    assert binding['profile_sha256'] == digest
    return profile_path, digest


def _relocate_phase3_smoke_profile(
    repository: Path,
    candidate_root: Path,
    *,
    old_root: str,
) -> tuple[Path, str]:
    """Rebind the copied profile to replayed clone-local smoke artifacts."""
    profiler = release_module._load_repository_module(
        repository,
        'tests/phase3_smoke_host_profiler.py',
        'relocated Phase 3 smoke host profiler',
    )
    candidate_id = candidate_root.name
    profile_path = (
        repository
        / 'artifacts/evidence/phase3/performance-profiles'
        / f'{candidate_id}-smoke-profile.json'
    )
    profile = _relocate_json(profile_path, old_root, str(repository))
    anchor = profile['anchor']
    anchor['mapping_fingerprint_sha256'] = hashlib.sha256(
        profiler.canonical_json_bytes(anchor['mappings'])
    ).hexdigest()
    full_stack = profiler.wait_for_full_stack_close(
        candidate_root / 'smoke',
        anchor,
        monotonic=lambda: 0.0,
        sleep=lambda _duration: None,
        profile_started_monotonic_ns=profile['profile_started_monotonic_ns'],
    )
    full_stack['pre_smoke_outputs_absent'] = True
    profile['full_stack_process'] = full_stack
    host_thread_bindings = profile['contact_profile']['host_thread_bindings']
    contact = profiler.parse_contact_profile_logs(candidate_root / 'smoke')
    profiler.reconcile_contact_log_sources(contact, full_stack)
    contact['host_thread_bindings'] = host_thread_bindings
    profile['contact_profile'] = contact
    _canonical_file(profile_path, profile, sidecar=True)
    assert old_root.encode() not in profile_path.read_bytes()
    binding = profiler.validate_campaign_smoke_profile(repository, candidate_root, candidate_id)
    digest = phase5_module.file_sha256(profile_path)
    assert binding['profile_sha256'] == digest
    return profile_path, digest


def _rebind_phase3_gate_attestation_workspace(
    path: Path,
    *,
    repository: Path,
    build_binding: dict,
    orchestration: object,
) -> None:
    """Refresh clone-local live binary identities in one runtime-gate document."""
    gate = json.loads(path.read_text(encoding='utf-8'))
    gate_binding = build_binding['contact_gate_binary']
    gate_path = (repository / gate_binding['installed_path']).resolve(strict=True)
    gate_stat = gate_path.stat()
    gate_attestation = gate['contact_gate_binary_attestation']
    gate_attestation.update(
        {
            'installed_device': gate_stat.st_dev,
            'installed_inode': gate_stat.st_ino,
            'live_cmdline_sha256': _fixture_contact_gate_cmdline_sha256(gate_path),
            'live_device': gate_stat.st_dev,
            'live_executable_link': str(gate_path),
            'live_executable_path': str(gate_path),
            'live_inode': gate_stat.st_ino,
            'live_size_bytes': gate_stat.st_size,
        }
    )

    aggregator_binding = build_binding['contact_aggregator_binary']
    aggregator_path = (repository / aggregator_binding['installed_path']).resolve(strict=True)
    aggregator_stat = aggregator_path.stat()
    mapping_paths = [str(aggregator_path)]
    mapping_count, mapping_fingerprint = _fixture_contact_aggregator_mapping_identity(
        aggregator_path,
        device=aggregator_stat.st_dev,
        inode=aggregator_stat.st_ino,
        orchestration=orchestration,
    )
    aggregator_attestation = gate['contact_aggregator_binary_attestation']
    aggregator_attestation.update(
        {
            'installed_device': aggregator_stat.st_dev,
            'installed_inode': aggregator_stat.st_ino,
            'installed_size_bytes': aggregator_stat.st_size,
            'live_mapping_count': mapping_count,
            'live_mapping_device': aggregator_stat.st_dev,
            'live_mapping_fingerprint_sha256': mapping_fingerprint,
            'live_mapping_inode': aggregator_stat.st_ino,
            'live_mapping_paths': mapping_paths,
        }
    )
    stable_identity = {
        field: aggregator_attestation[field]
        for field in orchestration.CONTACT_AGGREGATOR_STABLE_IDENTITY_FIELDS
    }
    aggregator_attestation['stable_identity'] = stable_identity
    aggregator_attestation['stable_identity_sha256'] = orchestration.canonical_sha256(
        stable_identity
    )
    _canonical_file(path, gate, sidecar=True)


def _relocate_phase3_evidence(
    repository: Path,
    candidate_root: Path,
    *,
    candidate_sha: str,
    old_root: str,
) -> Path:
    release_module._activate_repository_packages(repository)
    from robotest_missions.artifacts import write_result_artifacts

    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration fixture clone',
    )
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    positive_directory = candidate_root / 'positive-control'
    positive_result_path = positive_directory / 'contact-control-result.json'
    _relocate_json(positive_result_path, old_root, str(repository), sidecar=True)
    for role in release_module.PHASE3_POSITIVE_PROCESS_ROLES:
        _relocate_json(
            positive_directory / f'processes/{role}.process.json',
            old_root,
            str(repository),
        )
    for gate_name in ('runtime-gate.json', 'contact-stream-final-gate.json'):
        gate_path = positive_directory / gate_name
        _relocate_json(gate_path, old_root, str(repository), sidecar=True)
        _rebind_phase3_gate_attestation_workspace(
            gate_path,
            repository=repository,
            build_binding=build_binding,
            orchestration=orchestration,
        )
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)
    excluded_outputs = {
        'PASS.json',
        'PASS.json.sha256',
        'component-manifest.json',
        'component-manifest.json.sha256',
        'positive-binding.json',
        'positive-binding.json.sha256',
    }
    component_paths = sorted(
        path
        for path in positive_directory.rglob('*')
        if path.is_file()
        and path.relative_to(positive_directory).as_posix() not in excluded_outputs
    )
    component_manifest_path = positive_directory / 'component-manifest.json'
    component_manifest = orchestration.component_manifest(
        component_paths,
        positive_directory,
    )
    _canonical_file(component_manifest_path, component_manifest, sidecar=True)
    positive_binding_path = positive_directory / 'positive-binding.json'
    positive_binding = orchestration.reconcile_positive_control(
        workspace=repository,
        build_binding=build_binding,
        result_path=positive_result_path,
        capture_path=positive_directory / 'capture.json',
        contact_progress_path=positive_directory / 'contact-progress.json',
        command_progress_path=positive_directory / 'command-progress.json',
        driver_ready_path=positive_directory / 'contact-control.ready.json',
        arm_request_path=positive_directory / 'contact-control.arm.json',
        armed_ack_path=positive_directory / 'contact-control.armed.json',
        runtime_gate_path=positive_directory / 'runtime-gate.json',
        manifest_path=repository / 'config/collision-coverage.yaml',
        collector_configuration_sha256=build_binding['collector_configuration_sha256'],
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    _canonical_file(positive_binding_path, positive_binding, sidecar=True)
    positive_marker_path = positive_directory / 'PASS.json'
    positive_marker = json.loads(positive_marker_path.read_text(encoding='utf-8'))
    positive_marker['positive_binding_sha256'] = phase5_module.file_sha256(positive_binding_path)
    _canonical_file(positive_marker_path, positive_marker, sidecar=True)

    metrics_root = str(repository / 'src/robotest_metrics')
    if metrics_root not in sys.path:
        sys.path.insert(0, metrics_root)
    analyze_run = importlib.import_module('robotest_metrics.analysis').analyze_run

    def replay(plan: dict, run_root: Path) -> tuple[dict, str]:
        result_directory = run_root / 'result'
        lifecycle_path = run_root / 'lifecycle-snapshot.json'
        mission_path = run_root / 'mission-result.json'
        mission_document = _relocate_json(
            mission_path,
            old_root,
            str(repository),
        )
        write_result_artifacts(
            mission_document,
            mission_path,
            run_root / 'mission-result.csv',
        )
        Path(f'{mission_path}.sha256').write_text(
            f'{phase5_module.file_sha256(mission_path)}  {mission_path.name}\n',
            encoding='ascii',
        )
        _relocate_json(
            run_root / 'scenario-result.json',
            old_root,
            str(repository),
            sidecar=True,
        )
        process_roles = set(release_module.PHASE3_PREREQUISITE_PROCESS_ROLES)
        if (run_root / 'processes/lifecycle_sampler.process.json').is_file():
            process_roles.add('lifecycle_sampler')
        for role in process_roles:
            _relocate_json(
                run_root / f'processes/{role}.process.json',
                old_root,
                str(repository),
            )
        previous_final_log_gate = json.loads(
            (run_root / 'full-stack-final-log-gate.json').read_text(encoding='utf-8')
        )
        _write_phase3_final_launch_log_gate(
            repository,
            run_root,
            launch_stopped_utc=previous_final_log_gate['launch_stopped_utc'],
            scanned_utc=previous_final_log_gate['scanned_utc'],
        )
        for gate_name in ('runtime-gate.json', 'contact-stream-final-gate.json'):
            gate_path = run_root / gate_name
            _relocate_json(gate_path, old_root, str(repository), sidecar=True)
            _rebind_phase3_gate_attestation_workspace(
                gate_path,
                repository=repository,
                build_binding=build_binding,
                orchestration=orchestration,
            )
        contact_gate_revalidation = orchestration.reconcile_contact_gate_reobservation(
            run_root / 'runtime-gate.json',
            run_root / 'contact-stream-final-gate.json',
            build_binding=build_binding,
            expected_domain_id=plan['ros_domain_id'],
            expected_gz_partition=plan['gz_partition'],
        )
        _canonical_file(
            run_root / 'contact-gate-revalidation.json',
            contact_gate_revalidation,
            sidecar=True,
        )
        context = orchestration.make_trial_context(
            plan,
            workspace=repository,
            git_sha=candidate_sha,
            build=build_binding,
            positive=positive_binding,
        )
        _canonical_file(run_root / 'trial-context.json', context, sidecar=True)
        prerequisite_manifest_path = run_root / 'prerequisite-manifest.json'
        previous_manifest = json.loads(prerequisite_manifest_path.read_text(encoding='utf-8'))
        prerequisite_paths = [
            run_root / record['path'] for record in previous_manifest['artifacts']
        ]
        prerequisite_manifest = orchestration.component_manifest(
            prerequisite_paths,
            run_root,
        )
        _canonical_file(prerequisite_manifest_path, prerequisite_manifest, sidecar=True)
        assert orchestration.verify_component_manifest(prerequisite_manifest, run_root)
        orchestrator_path = run_root / 'orchestrator.json'
        orchestrator = _relocate_json(
            orchestrator_path,
            old_root,
            str(repository),
            sidecar=True,
        )
        full_stack_process = release_module._phase3_bounded_process_record(
            repository,
            run_root,
            role='full_stack',
        )
        mission_process = release_module._phase3_bounded_process_record(
            repository,
            run_root,
            role='mission_runner',
        )
        pre_graph = orchestration.validate_phase3_graph_artifacts(
            run_root,
            mission_client=False,
            expected_watch_pid=full_stack_process['pid'],
        )
        mission_graph = orchestration.validate_phase3_graph_artifacts(
            run_root,
            mission_client=True,
            expected_watch_pid=mission_process['pid'],
        )
        prerequisite_records = prerequisite_manifest['artifacts']
        orchestrator['artifacts'] = {
            'mission_graph_sha256': mission_graph['graph_json_sha256'],
            'pre_mission_graph_sha256': pre_graph['graph_json_sha256'],
            'prerequisite_artifact_count': len(prerequisite_records),
            'prerequisite_checksums_verified': True,
            'prerequisite_manifest_sha256': phase5_module.file_sha256(prerequisite_manifest_path),
            'prerequisite_maximum_file_bytes': max(
                record['bytes'] for record in prerequisite_records
            ),
            'prerequisite_total_bytes': prerequisite_manifest['total_bytes'],
            'prerequisites_finalized': True,
            'prerequisites_within_caps': True,
            'runtime_stderr_bytes': sum(
                record['bytes']
                for record in prerequisite_records
                if record['path'].endswith('.stderr.log')
            ),
            'runtime_stdout_bytes': sum(
                record['bytes']
                for record in prerequisite_records
                if record['path'].endswith('.stdout.log')
            ),
        }
        _canonical_file(orchestrator_path, orchestrator, sidecar=True)
        request = orchestration.compose_analysis_request(
            workspace=repository,
            plan=plan,
            mission_path=run_root / 'mission-result.json',
            scenario_path=run_root / 'scenario-result.json',
            capture_path=run_root / 'capture.json',
            positive_binding_path=positive_binding_path,
            orchestrator_path=orchestrator_path,
            contact_drain_path=run_root / 'contact-drain.json',
            contact_progress_path=run_root / 'contact-progress.json',
            lifecycle_snapshot_path=(lifecycle_path if int(plan['scenario_id']) == 4 else None),
        )
        _canonical_file(run_root / 'analysis-request.json', request, sidecar=True)
        result = analyze_run(request)
        assert result['verdict']['automated_status'] == 'PASS', result['verdict']
        _canonical_file(result_directory / 'run-result.json', result)
        result_sha = _refresh_phase3_bundle(result_directory)
        stale_json_paths = [
            path.relative_to(run_root).as_posix()
            for path in run_root.rglob('*.json')
            if old_root.encode('utf-8') in path.read_bytes()
        ]
        assert not stale_json_paths, {
            'old_root': old_root,
            'stale_phase3_json_paths': stale_json_paths,
        }
        return result, result_sha

    suite_plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))
    results: list[dict] = []
    result_hashes: list[str] = []
    for trial in suite_plan['trials']:
        result, result_sha = replay(
            trial,
            candidate_root / f'runs/{int(trial["suite_index"]):02d}',
        )
        results.append(result)
        result_hashes.append(result_sha)
    first_trial = suite_plan['trials'][0]
    smoke = suite_plan['smoke']
    smoke_plan = {
        **first_trial,
        'candidate_id': f'{suite_plan["candidate_id"]}-smoke',
        'gz_partition': smoke['gz_partition'],
        'ros_domain_id': smoke['ros_domain_id'],
        'run_id': smoke['run_id'],
    }
    _, smoke_sha = replay(smoke_plan, candidate_root / 'smoke')
    smoke_marker_path = candidate_root / 'smoke/PASS.json'
    smoke_marker = json.loads(smoke_marker_path.read_text(encoding='utf-8'))
    smoke_marker['run_result_sha256'] = smoke_sha
    _canonical_file(smoke_marker_path, smoke_marker, sidecar=True)
    _relocate_phase3_smoke_profile(
        repository,
        candidate_root,
        old_root=old_root,
    )

    aggregate = release_module._recompute_phase3_aggregate(
        repository,
        results,
        result_hashes,
    )
    aggregate_path = candidate_root / 'aggregate/aggregate-result.json'
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )
    return aggregate_path


def _relocate_phase4_evidence(
    repository: Path,
    run_directory: Path,
    *,
    old_root: str,
) -> Path:
    context_path = run_directory / 'context.json'
    _relocate_json(context_path, old_root, str(repository))
    followup_path = run_directory / 'followup-result.json'
    followup = _relocate_json(followup_path, old_root, str(repository))
    missions_root = str(repository / 'src/robotest_missions')
    if missions_root not in sys.path:
        sys.path.insert(0, missions_root)
    mission_artifacts = importlib.import_module('robotest_missions.artifacts')
    mission_artifacts.write_result_artifacts(
        followup,
        followup_path,
        run_directory / 'followup-result.csv',
    )
    return _rebind_phase4_evidence(repository, run_directory)


def _rebind_phase4_runtime_staging(
    repository: Path,
    run_directory: Path,
    phase4: object,
    context: dict,
) -> None:
    source_snapshot = phase4.source_snapshot(repository)
    for name in ('source-snapshot-before.json', 'source-snapshot-after.json'):
        _canonical_file(run_directory / name, source_snapshot)

    source_manifest_path = run_directory / 'overlay-source-manifest.json'
    install_manifest_path = run_directory / 'overlay-install-manifest.json'
    _canonical_file(source_manifest_path, phase4.runtime_source_manifest(repository))
    install_manifest = json.loads(install_manifest_path.read_text(encoding='utf-8'))
    _canonical_file(install_manifest_path, install_manifest)

    git_commit = context['source_git_commit']
    git_dirty = context['source_git_dirty']
    source_manifest_sha = phase5_module.file_sha256(source_manifest_path)
    install_manifest_sha = phase5_module.file_sha256(install_manifest_path)
    release_id = f'{git_commit[:12]}-{source_manifest_sha[:16]}-{str(git_dirty).lower()}'
    release_path = f'/opt/robotest-lab-releases/{release_id}'
    previous_staging = json.loads(
        (run_directory / 'runtime-staging.json').read_text(encoding='utf-8')
    )
    staging = {
        'active_path': '/opt/robotest-lab',
        'build_command': [
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
        ],
        'created_utc': previous_staging['created_utc'],
        'git_commit': git_commit,
        'git_dirty': git_dirty,
        'install_manifest_sha256': install_manifest_sha,
        'packages': list(phase4.OVERLAY_PACKAGES),
        'release_id': release_id,
        'schema_version': 1,
        'source_manifest_sha256': source_manifest_sha,
        'source_workspace': str(repository),
    }
    _canonical_file(run_directory / 'runtime-staging.json', staging)
    _canonical_file(run_directory / 'overlay-provenance.json', staging)
    context['active_overlay_target'] = release_path
    _canonical_file(run_directory / 'context.json', context)


def _rebind_phase4_evidence(
    repository: Path,
    run_directory: Path,
    *,
    completed_utc: str | None = None,
) -> Path:
    phase4 = release_module._load_repository_module(
        repository,
        'tests/phase4_acceptance.py',
        'Phase 4 production acceptance fixture clone',
    )
    context_path = run_directory / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    _canonical_file(run_directory / 'isolation.json', context['isolation'])
    _rebind_phase4_runtime_staging(repository, run_directory, phase4, context)

    package_directory = Path(context['package_directory'])
    upgrade_package = Path(context['upgrade_package']['path'])
    baseline_package = Path(context['baseline_package']['path'])
    _canonical_file(
        run_directory / 'package-integrity.json',
        phase4.verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            repository,
        ),
    )
    package_manifest = package_directory / 'build-a/SOURCE-MANIFEST.json'
    _canonical_file(
        run_directory / 'package-binding.json',
        phase4.package_source_binding(repository, package_manifest),
    )
    if completed_utc is None:
        previous_result = json.loads(
            (run_directory / 'scenario6-result.json').read_text(encoding='utf-8')
        )
        completed_utc = previous_result['identity']['completed_utc']
    original_utc_now = phase4.utc_now
    phase4.utc_now = lambda: completed_utc
    try:
        result = phase4.write_result(run_directory)
    finally:
        phase4.utc_now = original_utc_now
    assert result['verdict']['status'] == 'PASS', result['verdict']
    phase4.write_checksums(run_directory)
    return run_directory / 'scenario6-result.json'


def _release_fixture_template() -> dict[str, Path | str]:
    global _RELEASE_FIXTURE_TEMPLATE, _RELEASE_FIXTURE_TEMPLATE_DIRECTORY
    if _RELEASE_FIXTURE_TEMPLATE is None:
        directory = tempfile.TemporaryDirectory(prefix='robotest-phase5-release-fixture-')
        try:
            fixture = _build_release_fixture(Path(directory.name))
        except BaseException:
            directory.cleanup()
            raise
        _RELEASE_FIXTURE_TEMPLATE_DIRECTORY = directory
        _RELEASE_FIXTURE_TEMPLATE = fixture
    return _RELEASE_FIXTURE_TEMPLATE


def _release_fixture(tmp_path: Path) -> dict[str, Path | str]:
    global _RELEASE_FIXTURE_LAST_REPOSITORY
    template = _release_fixture_template()
    template_repository = Path(template['repository'])
    tmp_path.mkdir(parents=True, exist_ok=True)
    repository = tmp_path / 'repository'
    previous_repository = _RELEASE_FIXTURE_LAST_REPOSITORY
    if previous_repository is not None and previous_repository.is_dir():
        assert previous_repository.name == 'repository'
        shutil.rmtree(previous_repository)
    repository.mkdir()
    subprocess.run(
        ['cp', '-a', '--', f'{template_repository}/.', str(repository)],
        check=True,
    )
    _RELEASE_FIXTURE_LAST_REPOSITORY = repository
    assert not any(path.is_symlink() for path in (repository / 'install').rglob('*'))
    old_root = str(template_repository)
    candidate_sha = str(template['candidate_sha'])
    old_evidence_sha = str(template['evidence_sha'])

    old_evidence_remote = (
        repository
        / f'artifacts/evidence/phase5/remote-evidence-commit/remote-{old_evidence_sha}.json'
    )
    for path in (
        old_evidence_remote,
        old_evidence_remote.with_suffix('.SHA256SUMS'),
        old_evidence_remote.with_suffix('.checksum-validation.txt'),
    ):
        path.unlink(missing_ok=True)

    subprocess.run(
        ['git', 'switch', '--detach', '--quiet', candidate_sha],
        cwd=repository,
        check=True,
    )
    try:
        candidate_root = repository / 'artifacts/evidence/phase3-benchmarks/candidate-1'
        aggregate_path = _relocate_phase3_evidence(
            repository,
            candidate_root,
            candidate_sha=candidate_sha,
            old_root=old_root,
        )
        phase4_run = repository / 'artifacts/evidence/phase4/runs/phase4-20260826T000000Z-999'
        scenario6_path = _relocate_phase4_evidence(
            repository,
            phase4_run,
            old_root=old_root,
        )
        copied_portfolio_root = repository / portfolio_module.PORTFOLIO_RAW_PREFIX / candidate_sha
        assert copied_portfolio_root.is_dir() and not copied_portfolio_root.is_symlink()
        assert copied_portfolio_root.resolve().is_relative_to(repository.resolve())
        shutil.rmtree(copied_portfolio_root)
        portfolio_root, portfolio_raw_proof, portfolio_attempt_id = _finalize_portfolio_fixture(
            repository,
            candidate_root,
            candidate_sha,
        )
    finally:
        subprocess.run(
            ['git', 'switch', '--detach', '--quiet', old_evidence_sha],
            cwd=repository,
            check=True,
        )

    local_aggregate = repository / 'artifacts/evidence/phase0/verify-all.json'
    _relocate_json(local_aggregate, old_root, str(repository))
    _set_local_aggregate_after_portfolio(local_aggregate, portfolio_raw_proof)
    remote_path = repository / f'docs/results/phase-5/remote-{candidate_sha}.json'
    _refresh_remote_proof(
        remote_path,
        _remote_proof(
            repository,
            sha=candidate_sha,
            mode='--remote',
            run_id=42,
            completed_at='2026-08-26T00:00:30Z',
            created_at='2026-08-26T00:00:00Z',
            checked_at='2026-08-26T00:01:00+00:00',
        ),
    )
    for relative in ('README.md', 'config/release-claims.json'):
        candidate_bytes = subprocess.run(
            ['git', 'show', f'{candidate_sha}:{relative}'],
            cwd=repository,
            capture_output=True,
            check=True,
        ).stdout
        (repository / relative).write_bytes(candidate_bytes)
    for path in (
        repository / 'docs/results/phase-3/candidate-1.csv',
        repository / 'docs/results/phase-3/candidate-1.json',
        repository / 'docs/results/phase-3/candidate-1.md',
        repository / f'docs/results/phase-4/{phase4_run.name}.csv',
        repository / f'docs/results/phase-4/{phase4_run.name}.json',
        repository / f'docs/results/phase-4/{phase4_run.name}.md',
        *(
            repository / relative
            for relative in portfolio_module.portfolio_projection_paths(candidate_sha)
        ),
    ):
        path.unlink(missing_ok=True)
    release_docs_module.write_release_documents(
        repository,
        aggregate_path,
        scenario6_path,
        candidate_root,
        portfolio_root,
        portfolio_raw_proof,
    )
    portfolio_proof = repository / portfolio_module.portfolio_projection_paths(candidate_sha)[0]
    subprocess.run(['git', 'add', '-A'], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit',
            '-q',
            '--amend',
            '--no-edit',
            '--no-gpg-sign',
        ],
        cwd=repository,
        check=True,
    )
    evidence_sha = subprocess.run(
        ['git', 'rev-parse', 'HEAD'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()
    evidence_remote = (
        repository / f'artifacts/evidence/phase5/remote-evidence-commit/remote-{evidence_sha}.json'
    )
    _refresh_remote_proof(
        evidence_remote,
        _remote_proof(
            repository,
            sha=evidence_sha,
            mode='--remote-evidence-commit',
            run_id=43,
            completed_at='2026-08-26T00:10:30Z',
            created_at='2026-08-26T00:10:00Z',
            checked_at='2026-08-26T00:11:00+00:00',
        ),
    )
    smoke_profile = (
        repository / 'artifacts/evidence/phase3/performance-profiles/candidate-1-smoke-profile.json'
    )
    return {
        'aggregate': aggregate_path,
        'call_log': tmp_path / 'release-calls.jsonl',
        'candidate_root': candidate_root,
        'candidate_sha': candidate_sha,
        'evidence_remote': evidence_remote,
        'evidence_sha': evidence_sha,
        'local_aggregate': local_aggregate,
        'phase4_run': phase4_run,
        'portfolio_attempt_id': portfolio_attempt_id,
        'portfolio_proof': portfolio_proof,
        'portfolio_raw_proof': portfolio_raw_proof,
        'portfolio_root': portfolio_root,
        'remote': remote_path,
        'repository': repository,
        'scenario6': scenario6_path,
        'smoke_profile': smoke_profile,
        'smoke_profile_sha256': phase5_module.file_sha256(smoke_profile),
    }


def _validate_release_fixture(fixture: dict[str, Path | str]) -> dict[str, object]:
    return validate_release_evidence(
        Path(fixture['repository']),
        Path(fixture['local_aggregate']),
        Path(fixture['candidate_root']),
        Path(fixture['aggregate']),
        Path(fixture['phase4_run']),
        Path(fixture['scenario6']),
        Path(fixture['portfolio_root']),
        Path(fixture['portfolio_proof']),
        Path(fixture['remote']),
        Path(fixture['evidence_remote']),
    )


def _fixture_smoke_profile(fixture: dict[str, Path | str]) -> Path:
    return Path(fixture['smoke_profile'])


def _rewrite_fixture_smoke_profile(
    fixture: dict[str, Path | str],
    mutate: object,
) -> None:
    profile_path = _fixture_smoke_profile(fixture)
    document = json.loads(profile_path.read_text(encoding='utf-8'))
    assert callable(mutate)
    mutate(document)
    _canonical_file(profile_path, document, sidecar=True)


def _rebind_phase3_smoke_profile_candidate_inputs(
    fixture: dict[str, Path | str],
    *,
    plugin_source_inventory_sha256: str | None = None,
) -> None:
    """Cascade candidate-input hashes through prepared and profile evidence."""
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'build-binding.json'
    plan_path = candidate_root / 'suite-plan.json'
    prepared_path = candidate_root / 'prepared.json'
    prepared = json.loads(prepared_path.read_text(encoding='utf-8'))
    prepared['build_binding_sha256'] = phase5_module.file_sha256(binding_path)
    prepared['suite_plan_sha256'] = phase5_module.file_sha256(plan_path)
    _canonical_file(prepared_path, prepared, sidecar=True)

    profile_path = _fixture_smoke_profile(fixture)
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    profile_candidate = profile['candidate']
    profile_candidate['build_binding']['sha256'] = phase5_module.file_sha256(binding_path)
    profile_candidate['suite_plan']['sha256'] = phase5_module.file_sha256(plan_path)
    profile_candidate['prepared_marker']['sha256'] = phase5_module.file_sha256(prepared_path)
    if plugin_source_inventory_sha256 is not None:
        profile_candidate['plugin']['source_inventory_sha256'] = plugin_source_inventory_sha256
    _canonical_file(profile_path, profile, sidecar=True)

    profiler = release_module._load_repository_module(
        repository,
        'tests/phase3_smoke_host_profiler.py',
        'rebound candidate-input smoke host profiler',
    )
    profile_binding = profiler.validate_campaign_smoke_profile(
        repository,
        candidate_root,
        candidate_root.name,
    )
    assert profile_binding['profile_sha256'] == phase5_module.file_sha256(profile_path)


def _release_evidence_command(fixture: dict[str, Path | str]) -> list[str]:
    return [
        'bash',
        str(Path(fixture['repository']) / 'scripts/verify_all.sh'),
        '--release-evidence',
        '--local-aggregate',
        str(fixture['local_aggregate']),
        '--phase3-candidate-root',
        str(fixture['candidate_root']),
        '--phase3-aggregate',
        str(fixture['aggregate']),
        '--phase4-run-directory',
        str(fixture['phase4_run']),
        '--phase4-scenario6',
        str(fixture['scenario6']),
        '--phase5-portfolio-root',
        str(fixture['portfolio_root']),
        '--phase5-portfolio-proof',
        str(fixture['portfolio_proof']),
        '--phase5-remote-proof',
        str(fixture['remote']),
        '--phase5-evidence-commit-remote-proof',
        str(fixture['evidence_remote']),
    ]


def test_release_evidence_mode_passes_only_exact_selected_artifacts(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    report = _validate_release_fixture(fixture)
    assert report['status'] == 'PASS'
    assert report['schema_version'] == 2
    assert report['release_eligible'] is True
    assert report['candidate_git_sha'] == fixture['candidate_sha']
    assert report['phase3']['smoke_profile_path'] == str(fixture['smoke_profile'])
    assert report['phase3']['smoke_profile_sha256'] == fixture['smoke_profile_sha256']
    assert report['portfolio']['status'] == 'PASS'
    assert report['portfolio']['projection_paths'] == list(
        portfolio_module.portfolio_projection_paths(str(fixture['candidate_sha']))
    )
    assert report['portfolio']['attempt_id'] == fixture['portfolio_attempt_id']
    assert datetime.fromisoformat(report['phase5']['completed_at']) <= datetime.fromisoformat(
        report['portfolio']['prepared_utc'].replace('Z', '+00:00')
    )
    assert datetime.fromisoformat(report['phase4']['completed_utc']) <= datetime.fromisoformat(
        report['portfolio']['prepared_utc'].replace('Z', '+00:00')
    )
    assert datetime.fromisoformat(
        report['portfolio']['finalized_utc'].replace('Z', '+00:00')
    ) <= datetime.fromisoformat(report['local_aggregate']['checked_at'])
    assert 'clone-local Phase 3 smoke host profile' in report['verification_scope']
    phase4_context = json.loads(
        (Path(fixture['phase4_run']) / 'context.json').read_text(encoding='utf-8')
    )
    assert Path(phase4_context['upgrade_package']['path']).name == (
        'robotest-supervisor_0.1.1_amd64.deb'
    )
    command = _release_evidence_command(fixture)
    local_paths = [
        Path(fixture['local_aggregate']),
        Path(fixture['local_aggregate']).with_suffix('.SHA256SUMS'),
        Path(fixture['local_aggregate']).with_suffix('.checksum-validation.txt'),
    ]
    local_before = {path: path.read_bytes() for path in local_paths}
    result = subprocess.run(
        command,
        cwd=fixture['repository'],
        env=os.environ | {'CALL_LOG': str(fixture['call_log'])},
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    assert {path: path.read_bytes() for path in local_paths} == local_before
    aggregate = json.loads(Path(fixture['local_aggregate']).read_text(encoding='utf-8'))
    assert aggregate['status'] == 'incomplete'
    assert aggregate['release_eligible'] is False
    release_result = json.loads(
        (
            Path(fixture['repository']) / 'artifacts/evidence/phase5/release/release-evidence.json'
        ).read_text(encoding='utf-8')
    )
    assert release_result['status'] == 'PASS'
    assert release_result['schema_version'] == 2
    assert release_result['release_eligible'] is True
    assert release_result['candidate_git_sha'] == fixture['candidate_sha']
    assert not Path(fixture['call_log']).exists()


@pytest.mark.parametrize('summary_phase', ('phase-3', 'phase-4'))
def test_release_docs_preflights_stale_summaries_before_portfolio_projection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    summary_phase: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    expected_portfolio_paths = portfolio_module.portfolio_projection_paths(
        str(fixture['candidate_sha'])
    )
    for relative in expected_portfolio_paths:
        (repository / relative).unlink()
    stale = repository / f'docs/results/{summary_phase}/stale-direct-file.txt'
    stale.write_text('stale\n', encoding='utf-8')
    project_calls: list[tuple[object, ...]] = []
    monkeypatch.setattr(
        release_docs_module,
        'project_portfolio',
        lambda *arguments: project_calls.append(arguments),
    )

    with pytest.raises(EvidenceError, match='release summary directory contains stale files'):
        release_docs_module.write_release_documents(
            repository,
            Path(fixture['aggregate']),
            Path(fixture['scenario6']),
            Path(fixture['candidate_root']),
            Path(fixture['portfolio_root']),
            Path(fixture['portfolio_raw_proof']),
        )

    assert project_calls == []
    assert all(not (repository / relative).exists() for relative in expected_portfolio_paths)


def test_release_docs_bounds_raw_portfolio_reads_before_projection(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    expected_portfolio_paths = portfolio_module.portfolio_projection_paths(
        str(fixture['candidate_sha'])
    )
    for relative in expected_portfolio_paths:
        (repository / relative).unlink()
    raw_projection_root = Path(fixture['portfolio_raw_proof']).parent
    oversized = raw_projection_root / Path(expected_portfolio_paths[-1]).name
    with oversized.open('r+b') as stream:
        stream.truncate(release_docs_module.MAX_JSON_BYTES + 1)
    project_calls: list[tuple[object, ...]] = []
    monkeypatch.setattr(
        release_docs_module,
        'project_portfolio',
        lambda *arguments: project_calls.append(arguments),
    )

    with pytest.raises(EvidenceError, match='raw portfolio projection exceeds its size bound'):
        release_docs_module.write_release_documents(
            repository,
            Path(fixture['aggregate']),
            Path(fixture['scenario6']),
            Path(fixture['candidate_root']),
            Path(fixture['portfolio_root']),
            Path(fixture['portfolio_raw_proof']),
        )

    assert project_calls == []
    assert all(not (repository / relative).exists() for relative in expected_portfolio_paths)


@pytest.mark.parametrize(
    'failure_point',
    ('late_validation', 'immediate_lstat', 'pre_mode_capture', 'post_mode_capture'),
)
def test_release_docs_rolls_back_transient_failure_and_retries_under_restrictive_umask(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    failure_point: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    portfolio_paths = [
        repository / relative
        for relative in portfolio_module.portfolio_projection_paths(candidate_sha)
    ]
    summary_paths = [
        repository / f'docs/results/phase-3/{Path(fixture["candidate_root"]).name}.{suffix}'
        for suffix in ('csv', 'json', 'md')
    ] + [
        repository / f'docs/results/phase-4/{Path(fixture["phase4_run"]).name}.{suffix}'
        for suffix in ('csv', 'json', 'md')
    ]
    for path in [*portfolio_paths, *summary_paths]:
        path.unlink()
    for relative in ('README.md', 'config/release-claims.json'):
        (repository / relative).write_bytes(
            subprocess.run(
                ['git', 'show', f'{candidate_sha}:{relative}'],
                cwd=repository,
                capture_output=True,
                check=True,
            ).stdout
        )
    readme = repository / 'README.md'
    claims = repository / 'config/release-claims.json'
    phase5_root = repository / 'docs/results/phase-5'
    direct_roots = [
        repository / 'docs/results/phase-3',
        repository / 'docs/results/phase-4',
        phase5_root,
    ]
    guarded_paths = [
        *portfolio_paths,
        *summary_paths,
        readme,
        claims,
        *(path for path in phase5_root.iterdir()),
    ]

    def before_tree() -> dict[str, object]:
        return {
            'directories': {
                str(root): sorted(path.name for path in root.iterdir()) for root in direct_roots
            },
            'files': {
                str(path): (
                    None
                    if not path.exists() and not path.is_symlink()
                    else {
                        'bytes': path.read_bytes(),
                        'mode': path.stat().st_mode & 0o7777,
                        'regular': path.is_file() and not path.is_symlink(),
                    }
                )
                for path in guarded_paths
            },
        }

    initial = before_tree()
    original_validate = release_docs_module.validate_release_documents

    def fail_after_validation(*args: object, **kwargs: object) -> dict[str, object]:
        original_validate(*args, **kwargs)
        raise EvidenceError('injected late release validation failure')

    original_capture = release_docs_module._capture_written_file
    capture_failed = False
    capture_failure_labels = {
        'pre_mode_capture': 'release transaction pre-mode',
        'post_mode_capture': 'release transaction',
    }

    def fail_capture(
        path: Path,
        payload: bytes,
        mode: int,
        label: str,
    ) -> release_docs_module._WrittenFile:
        nonlocal capture_failed
        if label == capture_failure_labels[failure_point] and not capture_failed:
            capture_failed = True
            raise OSError(f'injected {failure_point.replace("_", "-")} failure')
        return original_capture(path, payload, mode, label)

    immediate_lstat_target = next(
        path for path in summary_paths if path.parent.name == 'phase-3' and path.suffix == '.json'
    )
    original_atomic_write = release_docs_module.atomic_write_bytes
    original_lstat = Path.lstat
    immediate_lstat_armed = False
    immediate_lstat_failed = False

    def arm_immediate_lstat(path: Path, payload: bytes) -> None:
        nonlocal immediate_lstat_armed
        original_atomic_write(path, payload)
        if path == immediate_lstat_target:
            immediate_lstat_armed = True

    def fail_immediate_lstat(path: Path) -> os.stat_result:
        nonlocal immediate_lstat_failed
        if path == immediate_lstat_target and immediate_lstat_armed and not immediate_lstat_failed:
            immediate_lstat_failed = True
            raise OSError('injected immediate-lstat failure')
        return original_lstat(path)

    with monkeypatch.context() as patch:
        if failure_point == 'late_validation':
            patch.setattr(release_docs_module, 'validate_release_documents', fail_after_validation)
            expected_failure = 'injected late release validation failure'
        elif failure_point == 'immediate_lstat':
            patch.setattr(release_docs_module, 'atomic_write_bytes', arm_immediate_lstat)
            patch.setattr(Path, 'lstat', fail_immediate_lstat)
            expected_failure = 'injected immediate-lstat failure'
        else:
            patch.setattr(release_docs_module, '_capture_written_file', fail_capture)
            expected_failure = f'injected {failure_point.replace("_", "-")} failure'
        with pytest.raises((EvidenceError, OSError), match=expected_failure):
            release_docs_module.write_release_documents(
                repository,
                Path(fixture['aggregate']),
                Path(fixture['scenario6']),
                Path(fixture['candidate_root']),
                Path(fixture['portfolio_root']),
                Path(fixture['portfolio_raw_proof']),
            )

    assert before_tree() == initial
    previous_umask = os.umask(0o077)
    try:
        report = release_docs_module.write_release_documents(
            repository,
            Path(fixture['aggregate']),
            Path(fixture['scenario6']),
            Path(fixture['candidate_root']),
            Path(fixture['portfolio_root']),
            Path(fixture['portfolio_raw_proof']),
        )
    finally:
        os.umask(previous_umask)

    assert report['portfolio_projection_paths'] == [
        path.relative_to(repository).as_posix() for path in portfolio_paths
    ]
    assert all((path.stat().st_mode & 0o7777) == 0o644 for path in portfolio_paths)


def test_release_evidence_rejects_raw_tracked_portfolio_mismatch(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    architecture = (
        repository / portfolio_module.portfolio_projection_paths(str(fixture['candidate_sha']))[3]
    )
    architecture.write_bytes(architecture.read_bytes() + b'<!-- tracked-only drift -->\n')

    with pytest.raises(EvidenceError, match='tracked portfolio projection differs from raw'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_manipulated_portfolio_validator_return(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fixture = _release_fixture(tmp_path)
    original = release_module.validate_portfolio_evidence

    def manipulated(*args: object, **kwargs: object) -> dict[str, object]:
        report = dict(original(*args, **kwargs))
        report['projection_paths'] = [
            *report['projection_paths'][:-1],
            'docs/results/phase-5/attacker-selected.png',
        ]
        return report

    monkeypatch.setattr(release_module, 'validate_portfolio_evidence', manipulated)
    with pytest.raises(EvidenceError, match='manipulated projection contract'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_missing_portfolio_addition(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    validated = release_module.validate_portfolio_evidence(
        repository,
        Path(fixture['portfolio_root']),
        Path(fixture['portfolio_proof']),
        Path(fixture['candidate_root']),
        str(fixture['candidate_sha']),
    )
    missing = (
        repository / portfolio_module.portfolio_projection_paths(str(fixture['candidate_sha']))[-1]
    )
    subprocess.run(['git', 'rm', '-q', '--', str(missing)], cwd=repository, check=True)
    _amend_all(repository)
    monkeypatch.setattr(
        release_module,
        'validate_portfolio_evidence',
        lambda *_args, **_kwargs: copy.deepcopy(validated),
    )

    with pytest.raises(EvidenceError, match='outside the exact release allowlist'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_extra_evidence_commit_path(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    extra = repository / 'docs/results/phase-5/unexpected-evidence.txt'
    extra.write_text('not selected evidence\n', encoding='utf-8')
    _amend_all(repository)

    with pytest.raises(EvidenceError, match='outside the exact release allowlist'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_executable_portfolio_projection(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    architecture = (
        repository / portfolio_module.portfolio_projection_paths(str(fixture['candidate_sha']))[3]
    )
    architecture.chmod(0o755)
    subprocess.run(
        ['git', 'update-index', '--chmod=+x', '--', str(architecture)],
        cwd=repository,
        check=True,
    )
    _amend_all(repository)

    with pytest.raises(
        EvidenceError,
        match='tracked portfolio projection type, link count, or mode changed',
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_portfolio_matrix_source_drift(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    matrix = repository / 'config/phase5-readme-replay.json'
    matrix.write_bytes(matrix.read_bytes() + b'\n')
    _amend_all(repository)

    with pytest.raises(
        EvidenceError, match='Scenario 6 result is not the exact production evaluation replay'
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_candidate_ci_completed_after_portfolio_prepare(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    proof_path = Path(fixture['remote'])
    proof = json.loads(proof_path.read_text(encoding='utf-8'))
    portfolio = json.loads(Path(fixture['portfolio_proof']).read_text(encoding='utf-8'))
    prepared = datetime.fromisoformat(portfolio['prepared_utc'].replace('Z', '+00:00'))
    proof['run']['completed_at'] = (
        (prepared + timedelta(seconds=1)).astimezone(UTC).isoformat().replace('+00:00', 'Z')
    )
    proof['checked_at'] = (
        (prepared + timedelta(seconds=2)).astimezone(UTC).isoformat().replace('+00:00', 'Z')
    )
    _refresh_remote_proof(proof_path, proof)

    with pytest.raises(EvidenceError, match='preparation predates completed candidate CI'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase4_completed_after_portfolio_prepare(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    portfolio = json.loads(Path(fixture['portfolio_proof']).read_text(encoding='utf-8'))
    prepared = datetime.fromisoformat(portfolio['prepared_utc'].replace('Z', '+00:00'))
    completed = (prepared + timedelta(seconds=1)).astimezone(UTC).isoformat().replace('+00:00', 'Z')
    _rebind_phase4_evidence(
        Path(fixture['repository']),
        Path(fixture['phase4_run']),
        completed_utc=completed,
    )

    with pytest.raises(EvidenceError, match='preparation predates completed Phase 4 evidence'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_bare_aggregate_before_portfolio_finalize(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    aggregate_path = Path(fixture['local_aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    portfolio = json.loads(Path(fixture['portfolio_proof']).read_text(encoding='utf-8'))
    finalized = datetime.fromisoformat(portfolio['finalized_utc'].replace('Z', '+00:00'))
    aggregate['checked_at'] = (
        (finalized - timedelta(seconds=1)).astimezone(UTC).isoformat().replace('+00:00', 'Z')
    )
    _canonical_file(aggregate_path, aggregate)
    aggregate_path.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(aggregate_path)}  {aggregate_path.name}\n',
        encoding='ascii',
    )

    with pytest.raises(EvidenceError, match='aggregate predates portfolio finalization'):
        _validate_release_fixture(fixture)


def test_release_evidence_shell_ignores_latest_and_performs_no_network_or_verifiers(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    latest = repository / 'artifacts/evidence/phase3-benchmarks/latest'
    assert not latest.exists() and not latest.is_symlink()
    latest.symlink_to(tmp_path / 'deliberately-missing-latest')
    fake_bin = tmp_path / 'fake-bin'
    fake_bin.mkdir()
    network_log = tmp_path / 'network.log'
    fake_network = """#!/bin/sh
printf '%s\\n' "$0 $*" >>"${NETWORK_LOG}"
exit 99
"""
    for command in ('curl', 'gh', 'wget'):
        executable = fake_bin / command
        executable.write_text(fake_network, encoding='utf-8')
        executable.chmod(0o755)

    result = subprocess.run(
        _release_evidence_command(fixture),
        cwd=repository,
        env=os.environ
        | {
            'CALL_LOG': str(fixture['call_log']),
            'NETWORK_LOG': str(network_log),
            'PATH': f'{fake_bin}:{os.environ["PATH"]}',
        },
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert not network_log.exists()
    assert not Path(fixture['call_log']).exists()
    assert latest.is_symlink()
    assert os.readlink(latest) == str(tmp_path / 'deliberately-missing-latest')


@pytest.mark.parametrize(
    'case',
    [
        'missing',
        'profile_symlink',
        'sidecar_symlink',
        'noncanonical',
        'bad_sidecar',
        'failed_status',
        'schema',
        'producer',
    ],
)
def test_release_evidence_rejects_invalid_phase3_smoke_profile_artifact(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    profile_path = _fixture_smoke_profile(fixture)
    sidecar_path = Path(f'{profile_path}.sha256')
    if case == 'missing':
        profile_path.unlink()
    elif case == 'profile_symlink':
        payload = profile_path.read_bytes()
        profile_path.unlink()
        target = profile_path.with_name('foreign-profile-target.json')
        target.write_bytes(payload)
        profile_path.symlink_to(target)
    elif case == 'sidecar_symlink':
        payload = sidecar_path.read_bytes()
        sidecar_path.unlink()
        target = sidecar_path.with_name('foreign-profile-sidecar')
        target.write_bytes(payload)
        sidecar_path.symlink_to(target)
    elif case == 'noncanonical':
        document = json.loads(profile_path.read_text(encoding='utf-8'))
        profile_path.write_text(json.dumps(document, indent=2) + '\n', encoding='utf-8')
        sidecar_path.write_text(
            f'{phase5_module.file_sha256(profile_path)}  {profile_path.name}\n',
            encoding='ascii',
        )
    elif case == 'bad_sidecar':
        sidecar_path.write_text(f'{"0" * 64}  {profile_path.name}\n', encoding='ascii')
    elif case == 'failed_status':
        _rewrite_fixture_smoke_profile(
            fixture, lambda document: document.__setitem__('status', 'FAIL')
        )
    elif case == 'schema':
        _rewrite_fixture_smoke_profile(
            fixture, lambda document: document.__setitem__('schema_version', 2)
        )
    else:
        _rewrite_fixture_smoke_profile(
            fixture,
            lambda document: document.__setitem__('producer', 'forged/smoke_host_profiler'),
        )

    with pytest.raises(EvidenceError, match='smoke host profile failed validation'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'candidate_id',
        'smoke_identity',
        'smoke_marker_hash',
        'smoke_result_hash',
        'build_binding_hash',
        'suite_plan_hash',
        'prepared_marker_hash',
        'git_sha',
        'plugin_hash',
        'source_inventory_hash',
        'profiler_hash',
    ],
)
def test_release_evidence_rejects_phase3_smoke_profile_identity_hash_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    if case == 'smoke_marker_hash':
        marker_path = candidate_root / 'smoke/PASS.json'
        marker = json.loads(marker_path.read_text(encoding='utf-8'))
        marker['run_result_sha256'] = '0' * 64
        _canonical_file(marker_path, marker, sidecar=True)
    elif case == 'smoke_result_hash':
        result_path = candidate_root / 'smoke/result/run-result.json'
        result = json.loads(result_path.read_text(encoding='utf-8'))
        result['quality']['infrastructure_failure'] = 'forged'
        _canonical_file(result_path, result)
    else:

        def mutate(document: dict) -> None:
            candidate = document['candidate']
            if case == 'candidate_id':
                candidate['candidate_id'] = 'foreign-candidate'
            elif case == 'smoke_identity':
                candidate['smoke']['run_id'] = 'foreign-smoke-run'
            elif case == 'build_binding_hash':
                candidate['build_binding']['sha256'] = '0' * 64
            elif case == 'suite_plan_hash':
                candidate['suite_plan']['sha256'] = '0' * 64
            elif case == 'prepared_marker_hash':
                candidate['prepared_marker']['sha256'] = '0' * 64
            elif case == 'git_sha':
                candidate['git']['sha'] = '0' * 40
            elif case == 'plugin_hash':
                candidate['plugin']['sha256'] = '0' * 64
            elif case == 'source_inventory_hash':
                candidate['plugin']['source_inventory_sha256'] = '0' * 64
            else:
                candidate['profiler_producer']['sha256'] = '0' * 64

        _rewrite_fixture_smoke_profile(fixture, mutate)

    with pytest.raises(EvidenceError, match='smoke host profile failed validation'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_foreign_renamed_phase3_profile_candidate(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    foreign_root = candidate_root.with_name('candidate-foreign')
    candidate_root.rename(foreign_root)
    profile_path = _fixture_smoke_profile(fixture)
    foreign_profile = profile_path.with_name('candidate-foreign-smoke-profile.json')
    profile_path.rename(foreign_profile)
    original_sidecar = Path(f'{profile_path}.sha256')
    foreign_sidecar = Path(f'{foreign_profile}.sha256')
    original_sidecar.rename(foreign_sidecar)
    foreign_sidecar.write_text(
        f'{phase5_module.file_sha256(foreign_profile)}  {foreign_profile.name}\n',
        encoding='ascii',
    )
    fixture['candidate_root'] = foreign_root
    fixture['aggregate'] = foreign_root / 'aggregate/aggregate-result.json'
    fixture['smoke_profile'] = foreign_profile

    with pytest.raises(EvidenceError, match='smoke host profile failed validation'):
        _validate_release_fixture(fixture)


def test_release_fixture_clones_are_self_contained_and_reload_producers(
    tmp_path: Path,
) -> None:
    first = _release_fixture(tmp_path / 'first')
    first_repository = Path(first['repository'])
    first_report = _validate_release_fixture(first)
    first_profile = _fixture_smoke_profile(first)
    template_repository = Path(_release_fixture_template()['repository'])
    assert first_report['phase3']['smoke_profile_sha256'] == phase5_module.file_sha256(
        first_profile
    )
    assert str(template_repository).encode() not in first_profile.read_bytes()
    assert not any(path.is_symlink() for path in (first_repository / 'install').rglob('*'))
    release_module._activate_repository_packages(first_repository)
    first_analysis = importlib.import_module('robotest_metrics.analysis')
    first_scenario_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert Path(first_analysis.__file__).resolve().is_relative_to(first_repository)
    assert Path(first_scenario_provenance.__file__).resolve().is_relative_to(first_repository)

    second = _release_fixture(tmp_path / 'second')
    second_repository = Path(second['repository'])
    second_report = _validate_release_fixture(second)
    second_profile = _fixture_smoke_profile(second)
    assert second_report['phase3']['smoke_profile_sha256'] == phase5_module.file_sha256(
        second_profile
    )
    assert str(template_repository).encode() not in second_profile.read_bytes()
    assert str(first_repository).encode() not in second_profile.read_bytes()
    release_module._activate_repository_packages(second_repository)
    second_analysis = importlib.import_module('robotest_metrics.analysis')
    second_scenario_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert second_analysis is not first_analysis
    assert second_scenario_provenance is not first_scenario_provenance
    assert Path(second_analysis.__file__).resolve().is_relative_to(second_repository)
    assert Path(second_scenario_provenance.__file__).resolve().is_relative_to(second_repository)
    final_log_scanner = release_module._load_repository_module(
        second_repository,
        'tests/phase2_startup_gate.py',
        'relocated Phase 3 final launch-log scanner',
    )
    assert Path(final_log_scanner.__file__).resolve().is_relative_to(second_repository)
    for relative_run_root in ('smoke', 'runs/00'):
        run_root = Path(second['candidate_root']) / relative_run_root
        combined_path = run_root / 'full-stack-combined.log'
        final_gate = json.loads(
            (run_root / 'full-stack-final-log-gate.json').read_text(encoding='utf-8')
        )
        assert final_gate['launch_log_path'] == str(combined_path.resolve(strict=True))
        assert combined_path.read_bytes() == final_log_scanner.combined_launch_log_bytes(
            run_root / 'processes/full_stack.stdout.log',
            run_root / 'processes/full_stack.stderr.log',
            maximum_stream_bytes=8 * 1024 * 1024,
        )

    candidate_root = Path(second['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    positive_control = json.loads(
        (positive_directory / 'contact-control-result.json').read_text(encoding='utf-8')
    )
    benchmark_binding = json.loads(
        (positive_directory / 'positive-binding.json').read_text(encoding='utf-8')
    )['benchmark_binding']
    coverage_manifest = yaml.safe_load(
        (second_repository / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
    )
    wall_asset = second_repository / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
    collision_module = importlib.import_module('robotest_metrics.collision_metrics')
    assert (
        collision_module.validate_collision_qualification(
            coverage_manifest,
            positive_control,
            benchmark_binding,
            wall_asset_path=wall_asset,
        )['status']
        == 'PASS'
    )
    wall_asset_bytes = wall_asset.read_bytes()
    wall_asset.write_bytes(wall_asset_bytes + b'\n')
    try:
        with pytest.raises(collision_module.MetricUnavailable, match='wall asset hash'):
            collision_module.validate_collision_qualification(
                coverage_manifest,
                positive_control,
                benchmark_binding,
                wall_asset_path=wall_asset,
            )
    finally:
        wall_asset.write_bytes(wall_asset_bytes)

    analysis_path = second_repository / 'src/robotest_metrics/robotest_metrics/analysis.py'
    analysis_path.write_text(
        analysis_path.read_text(encoding='utf-8') + '\n_CACHE_RELOAD_SENTINEL = True\n',
        encoding='utf-8',
    )
    release_module._activate_repository_packages(second_repository)
    reloaded_analysis = importlib.import_module('robotest_metrics.analysis')
    assert reloaded_analysis is not second_analysis
    assert reloaded_analysis._CACHE_RELOAD_SENTINEL is True

    provenance_path = second_repository / 'src/robotest_scenarios/robotest_scenarios/provenance.py'
    initial_source_binding = second_scenario_provenance.contact_source_binding()
    provenance_path.write_text(
        provenance_path.read_text(encoding='utf-8') + '\n_CACHE_RELOAD_SENTINEL = True\n',
        encoding='utf-8',
    )
    release_module._activate_repository_packages(second_repository)
    reloaded_scenario_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert reloaded_scenario_provenance is not second_scenario_provenance
    assert reloaded_scenario_provenance._CACHE_RELOAD_SENTINEL is True
    provenance_source_binding = reloaded_scenario_provenance.contact_source_binding()
    assert provenance_source_binding != initial_source_binding

    scenario_schema = (
        second_repository / 'src/robotest_scenarios/schema/contact-control-result.schema.json'
    )
    scenario_schema.write_text(
        scenario_schema.read_text(encoding='utf-8') + '\n',
        encoding='utf-8',
    )
    release_module._activate_repository_packages(second_repository)
    schema_reloaded_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert schema_reloaded_provenance.contact_source_binding() != provenance_source_binding

    phase4_path = second_repository / 'tests/phase4_acceptance.py'
    original_phase4 = release_module._load_repository_module(
        second_repository,
        'tests/phase4_acceptance.py',
        'Phase 4 reload fixture',
    )
    phase4_path.write_text(
        phase4_path.read_text(encoding='utf-8') + '\n_CACHE_RELOAD_SENTINEL = True\n',
        encoding='utf-8',
    )
    reloaded_phase4 = release_module._load_repository_module(
        second_repository,
        'tests/phase4_acceptance.py',
        'Phase 4 reload fixture',
    )
    assert reloaded_phase4 is not original_phase4
    assert reloaded_phase4._CACHE_RELOAD_SENTINEL is True


def test_release_evidence_final_claim_extension_is_exact(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    report = validate_release_claims(Path(fixture['repository']))
    assert report['claim_count'] == 27
    assert report['evidence_file_count'] == 5


def test_release_markdown_scalars_use_canonical_json_spelling() -> None:
    assert release_docs_module._markdown_scalar(True) == 'true'
    assert release_docs_module._markdown_scalar(False) == 'false'
    assert release_docs_module._markdown_scalar('text') == '"text"'
    assert release_docs_module._markdown_scalar({'value': True}) == '{"value":true}'


@pytest.mark.parametrize(
    'marker',
    [
        release_docs_module.RELEASE_STATUS_START,
        release_docs_module.RELEASE_STATUS_END,
        release_docs_module.RELEASE_ROADMAP_START,
        release_docs_module.RELEASE_ROADMAP_END,
    ],
)
@pytest.mark.parametrize('mutation', ['missing', 'duplicated'])
def test_release_readme_renderer_rejects_marker_corruption(marker: str, mutation: str) -> None:
    source = (REPOSITORY / 'README.md').read_text(encoding='utf-8')
    if mutation == 'missing':
        source = source.replace(marker, '', 1)
    else:
        source = source.replace(marker, f'{marker}\n{marker}', 1)

    with pytest.raises(EvidenceError, match='candidate README marker'):
        release_docs_module.render_final_readme(
            source.encode(),
            candidate_id='candidate-1',
            phase4_run_id='phase4-20260826T100000Z-1',
            git_sha='a' * 40,
        )


def test_release_evidence_shell_does_not_create_python_caches(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    for cache in list(repository.rglob('__pycache__')):
        shutil.rmtree(cache)

    result = subprocess.run(
        _release_evidence_command(fixture),
        cwd=repository,
        env=os.environ | {'CALL_LOG': str(fixture['call_log'])},
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert not list(repository.rglob('__pycache__'))


def test_release_evidence_shell_rejects_symlinked_result_destination(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    release_root = repository / 'artifacts/evidence/phase5/release'
    external = tmp_path / 'external-release'
    external.mkdir()
    sentinel = external / 'sentinel.txt'
    sentinel.write_text('preserve\n', encoding='utf-8')
    release_root.parent.mkdir(parents=True, exist_ok=True)
    release_root.symlink_to(external, target_is_directory=True)

    result = subprocess.run(
        _release_evidence_command(fixture),
        cwd=repository,
        env=os.environ | {'CALL_LOG': str(fixture['call_log'])},
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0
    assert sentinel.read_text(encoding='utf-8') == 'preserve\n'
    assert list(external.iterdir()) == [sentinel]
    assert 'contains a symlink' in result.stderr


def test_release_evidence_shell_fails_closed_without_mutating_local_aggregate(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    local_paths = [
        Path(fixture['local_aggregate']),
        Path(fixture['local_aggregate']).with_suffix('.SHA256SUMS'),
        Path(fixture['local_aggregate']).with_suffix('.checksum-validation.txt'),
    ]
    local_before = {path: path.read_bytes() for path in local_paths}
    (Path(fixture['phase4_run']) / 'timeline.jsonl').write_text(
        'tampered after finalization\n', encoding='utf-8'
    )

    result = subprocess.run(
        _release_evidence_command(fixture),
        cwd=fixture['repository'],
        env=os.environ | {'CALL_LOG': str(fixture['call_log'])},
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode != 0, result.stdout + result.stderr
    assert {path: path.read_bytes() for path in local_paths} == local_before
    release_root = Path(fixture['repository']) / 'artifacts/evidence/phase5/release'
    release_result = json.loads(
        (release_root / 'release-evidence.json').read_text(encoding='utf-8')
    )
    assert release_result['status'] == 'FAIL'
    assert release_result['release_eligible'] is False
    checksum = subprocess.run(
        ['sha256sum', '-c', '--strict', 'release-evidence.SHA256SUMS'],
        cwd=release_root,
        capture_output=True,
        text=True,
        check=False,
    )
    assert checksum.returncode == 0, checksum.stdout + checksum.stderr
    assert not Path(fixture['call_log']).exists()


def test_release_evidence_rejects_missing_selected_aggregate(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    Path(fixture['aggregate']).unlink()
    with pytest.raises(EvidenceError, match='missing regular Phase 3 aggregate'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_missing_local_aggregate(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    Path(fixture['local_aggregate']).unlink()
    with pytest.raises(EvidenceError, match='missing regular local aggregate summary'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_local_delta_bytes(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    phase0_version = repository / 'artifacts/evidence/phase0/phase0-versions.json'
    phase0_version.write_bytes(b'x')
    aggregate_path = Path(fixture['local_aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    record = aggregate['source']['generated_evidence_delta']['changed_files'][0]
    record['bytes'] = True
    record['sha256'] = phase5_module.file_sha256(phase0_version)
    _canonical_file(aggregate_path, aggregate)
    aggregate_path.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(aggregate_path)}  {aggregate_path.name}\n',
        encoding='ascii',
    )
    aggregate_path.with_suffix('.checksum-validation.txt').write_text(
        f'{aggregate_path.name}: OK\n', encoding='utf-8'
    )

    with pytest.raises(EvidenceError, match='generated Phase 0 evidence changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_missing_evidence_commit_remote_proof(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    Path(fixture['evidence_remote']).unlink()
    with pytest.raises(EvidenceError, match='missing regular evidence-commit remote proof'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_wrong_evidence_commit_remote_sha(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['run']['head_sha'] = fixture['candidate_sha']
    _refresh_remote_proof(path, proof)
    with pytest.raises(EvidenceError, match='verdict or identity is invalid'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_failed_evidence_commit_remote_run(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['run']['conclusion'] = 'failure'
    _refresh_remote_proof(path, proof)
    with pytest.raises(EvidenceError, match='verdict or identity is invalid'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_reduced_remote_proof_shape(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    del proof['provenance']['platform']
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='provenance schema changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_requires_exact_remote_completion_timestamp(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    del proof['run']['completed_at']
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='verdict or identity is invalid'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_non_distinct_second_ci_run(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['run']['run_id'] = 42
    proof['run']['run_url'] = 'https://github.com/example/robotest/actions/runs/42'
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='not a distinct later workflow run'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_remote_run_id(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['run']['run_id'] = True
    proof['run']['run_url'] = 'https://github.com/example/robotest/actions/runs/True'
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='run ID is invalid'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_remote_schema_version(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['schema_version'] = True
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='top-level producer contract changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_second_ci_before_candidate_proof(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    path = Path(fixture['evidence_remote'])
    proof = json.loads(path.read_text(encoding='utf-8'))
    proof['run']['created_at'] = '2026-08-26T00:00:30Z'
    _refresh_remote_proof(path, proof)

    with pytest.raises(EvidenceError, match='not a distinct later workflow run'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_checksum_tampering(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    (Path(fixture['phase4_run']) / 'timeline.jsonl').write_text('tampered\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='Phase 4 checksum mismatch'):
        _validate_release_fixture(fixture)


def test_release_evidence_requires_an_evidence_only_head_commit(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    (repository / 'unexpected.txt').write_text('not evidence only\n', encoding='utf-8')
    _commit_all(repository, 'unexpected source change')
    with pytest.raises(
        EvidenceError, match='single-parent evidence-only commit directly after the candidate'
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_tampered_generated_summary(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    summary = (
        Path(fixture['repository'])
        / f'docs/results/phase-3/{Path(fixture["candidate_root"]).name}.md'
    )
    summary.write_text(summary.read_text(encoding='utf-8') + 'forged\n', encoding='utf-8')

    with pytest.raises(EvidenceError, match='release summary differs'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_extra_generated_summary(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    extra = Path(fixture['repository']) / 'docs/results/phase-3/extra.md'
    extra.write_text('extra\n', encoding='utf-8')

    with pytest.raises(EvidenceError, match='release summary path set is not exact'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_symlinked_summary_root(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    summary_root = repository / 'docs/results/phase-3'
    external = tmp_path / 'external-summary'
    summary_root.rename(external)
    summary_root.symlink_to(external, target_is_directory=True)

    with pytest.raises(EvidenceError, match='release summary root contains a symlink'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_readme_change_outside_markers(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    readme = Path(fixture['repository']) / 'README.md'
    readme.write_text('forged prefix\n' + readme.read_text(encoding='utf-8'), encoding='utf-8')

    with pytest.raises(EvidenceError, match='README changed outside'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_executable_generated_document(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    document = repository / f'docs/results/phase-3/{Path(fixture["candidate_root"]).name}.md'
    document.chmod(0o755)
    subprocess.run(['git', 'update-index', '--chmod=+x', str(document)], cwd=repository, check=True)
    subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit',
            '-q',
            '--amend',
            '--no-edit',
            '--no-gpg-sign',
        ],
        cwd=repository,
        check=True,
    )

    with pytest.raises(EvidenceError, match='not a regular 100644 blob'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_symlinked_phase3_evidence_root(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    evidence_root = repository / 'artifacts/evidence/phase3-benchmarks'
    external = tmp_path / 'external-phase3'
    evidence_root.rename(external)
    evidence_root.symlink_to(external, target_is_directory=True)

    with pytest.raises(EvidenceError, match='Phase 3 evidence root contains a symlink'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_hidden_index_flags(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    subprocess.run(
        ['git', 'update-index', '--assume-unchanged', 'README.md'],
        cwd=repository,
        check=True,
    )

    with pytest.raises(EvidenceError, match='release index contains'):
        _validate_release_fixture(fixture)


def test_release_git_ignores_inherited_redirects_path_and_fsmonitor(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    decoy = tmp_path / 'decoy-repository'
    decoy.mkdir()
    (decoy / 'README.md').write_text('decoy repository\n', encoding='utf-8')
    subprocess.run(['git', 'init', '-q'], cwd=decoy, check=True)
    _commit_all(decoy, 'decoy')

    marker = tmp_path / 'hostile-git-invoked'
    fsmonitor_marker = tmp_path / 'hostile-fsmonitor-invoked'
    hostile_bin = tmp_path / 'hostile-bin'
    hostile_bin.mkdir()
    hostile_git = hostile_bin / 'git'
    hostile_git.write_text(
        f'#!/bin/sh\nprintf invoked > {shlex.quote(str(marker))}\nexit 97\n',
        encoding='utf-8',
    )
    hostile_git.chmod(0o755)
    fsmonitor = tmp_path / 'hostile-fsmonitor'
    fsmonitor.write_text(
        f'#!/bin/sh\nprintf invoked > {shlex.quote(str(fsmonitor_marker))}\nexit 0\n',
        encoding='utf-8',
    )
    fsmonitor.chmod(0o755)
    inherited_git = {
        'GIT_CONFIG_COUNT': '1',
        'GIT_CONFIG_KEY_0': 'core.fsmonitor',
        'GIT_CONFIG_VALUE_0': str(fsmonitor),
        'GIT_DIR': str(decoy / '.git'),
        'GIT_INDEX_FILE': str(decoy / '.git/index'),
        'GIT_WORK_TREE': str(decoy),
        'HOME': str(tmp_path / 'hostile-home'),
    }
    candidate_sha = str(fixture['candidate_sha'])
    expected_readme = subprocess.run(
        ['git', 'show', f'{candidate_sha}:README.md'],
        cwd=repository,
        capture_output=True,
        check=True,
    ).stdout

    with monkeypatch.context() as poisoned:
        for name, value in inherited_git.items():
            poisoned.setenv(name, value)
        poisoned.setenv('PATH', str(hostile_bin))
        top_level = release_module._git(repository, ['rev-parse', '--show-toplevel'])
        candidate_readme = release_module._git_bytes(
            repository,
            ['show', f'{candidate_sha}:README.md'],
        )
        assert top_level.returncode == 0
        assert top_level.stdout.strip() == str(repository)
        assert candidate_readme.returncode == 0
        assert candidate_readme.stdout == expected_readme
        assert not marker.exists()
        assert not fsmonitor_marker.exists()

    with monkeypatch.context() as poisoned:
        for name, value in inherited_git.items():
            poisoned.setenv(name, value)
        assert _validate_release_fixture(fixture)['status'] == 'PASS'
        assert not fsmonitor_marker.exists()


def test_release_evidence_rejects_git_replace_tree_illusion(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_sha = str(fixture['candidate_sha'])
    evidence_tree = subprocess.run(
        ['git', 'rev-parse', 'HEAD^{tree}'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()
    replacement_sha = subprocess.run(
        [
            'git',
            '-c',
            'user.name=Phase5 Test',
            '-c',
            'user.email=phase5@example.invalid',
            'commit-tree',
            evidence_tree,
        ],
        cwd=repository,
        input='replacement tree\n',
        capture_output=True,
        check=True,
        text=True,
    ).stdout.strip()
    subprocess.run(
        ['git', 'replace', candidate_sha, replacement_sha],
        cwd=repository,
        check=True,
    )

    replaced_readme = subprocess.run(
        ['git', 'show', f'{candidate_sha}:README.md'],
        cwd=repository,
        capture_output=True,
        check=True,
    ).stdout
    actual_readme = release_module._git_bytes(
        repository,
        ['show', f'{candidate_sha}:README.md'],
    )
    assert actual_readme.returncode == 0
    assert replaced_readme != actual_readme.stdout
    with pytest.raises(EvidenceError, match='repository contains Git replacement refs'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_git_grafted_ancestry(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    evidence_sha = str(fixture['evidence_sha'])
    before = subprocess.run(
        ['git', 'rev-list', '--parents', '-n', '1', evidence_sha],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.split()
    assert len(before) == 2
    grafts_path = repository / '.git/info/grafts'
    grafts_path.write_text(f'{evidence_sha}\n', encoding='ascii')
    grafted = subprocess.run(
        ['git', 'rev-list', '--parents', '-n', '1', evidence_sha],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    ).stdout.split()
    assert grafted == [evidence_sha]

    with pytest.raises(EvidenceError, match='repository Git graft metadata is present'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_git_local_clean_filter_illusion(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    attributes_path = repository / '.git/info/attributes'
    attributes_path.write_text('README.md filter=mask\n', encoding='utf-8')
    subprocess.run(
        ['git', 'config', 'filter.mask.clean', "sed '/^evil$/d'"],
        cwd=repository,
        check=True,
    )
    readme_path = repository / 'README.md'
    readme_path.write_text(
        readme_path.read_text(encoding='utf-8') + 'evil\n',
        encoding='utf-8',
    )
    subprocess.run(['git', 'add', '--', 'README.md'], cwd=repository, check=True)
    staged = subprocess.run(
        ['git', 'diff', '--cached', '--quiet', '--', 'README.md'],
        cwd=repository,
        check=False,
    )
    assert staged.returncode == 0
    raw_status = subprocess.run(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    )
    assert raw_status.stdout == ''

    with pytest.raises(EvidenceError, match='repository Git local attributes are present'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_git_local_exclude_illusion(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    forged_path = repository / 'forged-untracked.txt'
    forged_path.write_text('hidden from release status\n', encoding='utf-8')
    visible = subprocess.run(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    )
    assert visible.stdout == '?? forged-untracked.txt\n'
    exclude_path = repository / '.git/info/exclude'
    exclude_path.write_text(
        exclude_path.read_text(encoding='utf-8') + 'forged-untracked.txt\n',
        encoding='utf-8',
    )
    hidden = subprocess.run(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    )
    assert hidden.stdout == ''

    with pytest.raises(EvidenceError, match='Git local excludes contain active rules'):
        _validate_release_fixture(fixture)


def test_release_git_forces_file_mode_tracking(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    readme_path = repository / 'README.md'
    readme_path.chmod(0o755)
    subprocess.run(
        ['git', 'config', 'core.fileMode', 'false'],
        cwd=repository,
        check=True,
    )
    raw_status = subprocess.run(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    )
    assert raw_status.stdout == ''
    sanitized_status = release_module._git(
        repository,
        ['status', '--porcelain=v1', '--untracked-files=all'],
    )
    assert sanitized_status.returncode == 0
    assert sanitized_status.stdout == ' M README.md\n'

    with pytest.raises(EvidenceError, match='release worktree is not clean'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_tracked_ignore_hidden_runtime_source(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    hidden_source = repository / 'src/robotest_missions/robotest_missions/evil.so'
    hidden_source.write_bytes(b'ignored hostile runtime source\n')
    ignored = subprocess.run(
        ['git', 'check-ignore', '-q', '--', str(hidden_source.relative_to(repository))],
        cwd=repository,
        check=False,
    )
    status = subprocess.run(
        ['git', 'status', '--porcelain=v1', '--untracked-files=all'],
        cwd=repository,
        capture_output=True,
        check=True,
        text=True,
    )
    assert ignored.returncode == 0
    assert status.stdout == ''
    _rebind_phase4_evidence(repository, Path(fixture['phase4_run']))

    with pytest.raises(
        EvidenceError,
        match=(
            'ignored untracked source input is forbidden: '
            r'src/robotest_missions/robotest_missions/evil\.so'
        ),
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_wrong_campaign_verdict(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    aggregate_path = Path(fixture['aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    aggregate['verdict']['automated_status'] = 'FAIL'
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )
    with pytest.raises(EvidenceError, match='aggregate verdict is not PASS'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_schema_invalid_phase3_bundle(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    bundle = Path(fixture['candidate_root']) / 'runs/00/result'
    result_path = bundle / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    del result['events']
    _canonical_file(result_path, result)
    _refresh_phase3_bundle(bundle)
    with pytest.raises(EvidenceError, match='result bundle verification failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_schema_invalid_phase3_manifest(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    manifest_path = Path(fixture['candidate_root']) / 'runs/00/result/run-artifacts.manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    manifest['producer'] = 'forged/producer'
    _canonical_file(manifest_path, manifest, sidecar=True)

    with pytest.raises(EvidenceError, match='result bundle verification failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase3_manifest_missing_graph_projection(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 incomplete prerequisite fixture',
    )
    manifest_path = run_root / 'prerequisite-manifest.json'
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    paths = [
        run_root / record['path']
        for record in manifest['artifacts']
        if record['path'] != 'mission-actions.txt'
    ]
    _canonical_file(
        manifest_path,
        orchestration.component_manifest(paths, run_root),
        sidecar=True,
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='exact production pre-manifest snapshot'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('role', 'case'),
    [
        ('full_stack', 'command'),
        ('full_stack', 'timeout'),
        ('mission_runner', 'command'),
        ('mission_runner', 'timeout'),
        ('mission_graph_gate', 'internal_timeout'),
        ('mission_graph_gate', 'outer_timeout'),
        ('runtime_gate', 'command'),
    ],
)
def test_release_evidence_rejects_rebound_phase3_process_contract_forgery(
    tmp_path: Path,
    role: str,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    process_path = run_root / f'processes/{role}.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    if case == 'command':
        command = list(process['command'])
        command[0] = 'forged-executable'
        _rewrite_process_command(process, command)
    elif case == 'internal_timeout':
        wall_timeout_index = process['command'].index('--wall-timeout') + 1
        process['command'][wall_timeout_index] = '90'
        process['wrapped_command'][4 + wall_timeout_index] = '90'
    elif case == 'outer_timeout':
        process['wall_timeout_s'] = 100.0
        process['wrapped_command'][3] = '100.000s'
    else:
        process['wall_timeout_s'] += 1.0
        process['wrapped_command'][3] = f'{process["wall_timeout_s"]:.3f}s'
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    if role == 'full_stack':
        with pytest.raises(
            EvidenceError,
            match='smoke host profile failed validation: full-stack recorded projection differs',
        ):
            _validate_release_fixture(fixture)
        _rebind_phase3_smoke_profile_outputs(repository, candidate_root)

    with pytest.raises(EvidenceError, match=rf'{role} process command, timeout, or outcome'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_partial_retained_stream(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    process_path = run_root / 'processes/full_stack.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    process['stdout']['observed_bytes'] = process['stdout']['retained_bytes'] + 128
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: full-stack recorded projection differs',
    ):
        _validate_release_fixture(fixture)
    profile_path = _fixture_smoke_profile(fixture)
    profile = json.loads(profile_path.read_text(encoding='utf-8'))
    profile['full_stack_process']['sha256'] = phase5_module.file_sha256(process_path)
    _canonical_file(profile_path, profile, sidecar=True)
    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: full-stack stdout was not retained',
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_combined_launch_log_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    (run_root / 'full-stack-combined.log').write_bytes(b'forged combined bytes\n')
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='differs from finalized full_stack streams'):
        _validate_release_fixture(fixture)


def test_release_evidence_accepts_recovered_phase3_lifecycle_timeout(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'

    gate = _write_phase3_recovered_lifecycle_timeout_gate(repository, run_root)
    recovery = gate['lifecycle_timeout_recovery']
    assert gate['schema_version'] == 2
    assert gate['match_count'] == 1
    assert gate['matches'][0]['signature_ids'] == ['dds_response_timeout']
    assert recovery['recovered_line_count'] == 1
    assert recovery['recovered_lines'][0]['service_name'] == (
        '/robotest/collision_monitor/get_state'
    )
    assert recovery['recovered_lines'][0]['timed_out_attempts'] == 1

    _rebind_phase3_prerequisites(repository, run_root)
    _recompose_phase3_smoke_result(repository, candidate_root)
    _rebind_phase3_smoke_profile_outputs(repository, candidate_root)

    _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    (
        'bad_lifecycle_hash',
        'bad_lifecycle_path',
        'timeout_count_deficit',
        'timeout_count_exceeds_attempts',
        'inactive_state',
        'residual_error',
        'unknown_service',
        'comixed_dds_timeout',
        'mixed_fatal_signature',
        'wrong_watch_pid',
        'wrong_wall_timeout',
        'missing_node',
        'extra_node',
    ),
)
def test_release_evidence_rejects_fully_rebound_lifecycle_timeout_recovery_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'full-stack-final-log-gate.json'
    gate = _write_phase3_recovered_lifecycle_timeout_gate(repository, run_root)

    if case == 'bad_lifecycle_hash':
        gate['lifecycle_timeout_recovery']['lifecycle_ready_sha256'] = '0' * 64
    elif case == 'bad_lifecycle_path':
        gate['lifecycle_timeout_recovery']['lifecycle_ready_path'] = str(
            run_root / 'forged-lifecycle-ready.json'
        )
    else:
        lifecycle_path = run_root / 'lifecycle-ready.json'
        lifecycle = json.loads(lifecycle_path.read_text(encoding='utf-8'))
        state = lifecycle['states']['collision_monitor']
        if case == 'timeout_count_deficit':
            state['timed_out_attempts'] = 0
        elif case == 'timeout_count_exceeds_attempts':
            state.update({'attempts': 2, 'timed_out_attempts': 2})
        elif case == 'inactive_state':
            state.update({'label': 'inactive', 'state_id': 2})
        elif case == 'residual_error':
            state['error'] = 'lifecycle state response exceeded 2.000s'
        elif case == 'wrong_watch_pid':
            lifecycle['watch_pid'] += 1
        elif case == 'wrong_wall_timeout':
            lifecycle['wall_timeout_s'] = 109.0
        elif case == 'missing_node':
            lifecycle['states'].pop('map_server')
        elif case == 'extra_node':
            lifecycle['states']['rogue_node'] = {
                **lifecycle['states']['map_server'],
                'service': '/robotest/rogue_node/get_state',
            }
        elif case == 'unknown_service':
            stream_path = run_root / 'processes/full_stack.stdout.log'
            retained = stream_path.read_bytes().replace(
                b'/robotest/collision_monitor/get_state',
                b'/robotest/unknown_service/get_state',
            )
            stream_path.write_bytes(retained)
            process_path = run_root / 'processes/full_stack.process.json'
            process = json.loads(process_path.read_text(encoding='utf-8'))
            process['stdout']['observed_bytes'] = len(retained)
            process['stdout']['retained_bytes'] = len(retained)
            _canonical_file(process_path, process)
        elif case == 'comixed_dds_timeout':
            stream_path = run_root / 'processes/full_stack.stdout.log'
            retained = stream_path.read_bytes().replace(
                b'client will not receive response\n',
                (
                    b'client will not receive response; failed to send response '
                    b'for unrelated request (timeout)\n'
                ),
            )
            stream_path.write_bytes(retained)
            process_path = run_root / 'processes/full_stack.process.json'
            process = json.loads(process_path.read_text(encoding='utf-8'))
            process['stdout']['observed_bytes'] = len(retained)
            process['stdout']['retained_bytes'] = len(retained)
            _canonical_file(process_path, process)
        else:
            _append_phase3_full_stack_log(run_root, b'process FATAL error\n')
        _write_phase3_lifecycle_ready(repository, run_root, lifecycle)
        gate = _write_phase3_final_launch_log_gate(
            repository,
            run_root,
            expected_verdict='FAIL',
        )
        gate.update({'failure_kind': None, 'failure_message': None, 'verdict': 'PASS'})

    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)
    _recompose_phase3_smoke_result(repository, candidate_root)
    _rebind_phase3_smoke_profile_outputs(repository, candidate_root)

    with pytest.raises(EvidenceError, match='differs from clone-local scanner replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('stream_name', 'payload'),
    [
        ('stdout', b'Lifecycle STARTUP was rejected\n'),
        ('stderr', b'Failed to bring up all requested nodes\n'),
        ('stdout', b'service client: async_send_request failed\n'),
        ('stderr', b'failed to send response for request (timeout)\n'),
        ('stdout', b'process FATAL error\n'),
    ],
)
def test_release_evidence_rejects_coordinated_phase3_launch_signature_forgery(
    tmp_path: Path,
    stream_name: str,
    payload: bytes,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'full-stack-final-log-gate.json'
    recorded_gate = json.loads(gate_path.read_text(encoding='utf-8'))

    stream_path = run_root / f'processes/full_stack.{stream_name}.log'
    forged_stream = stream_path.read_bytes() + payload
    stream_path.write_bytes(forged_stream)
    process_path = run_root / 'processes/full_stack.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    process[stream_name]['observed_bytes'] = len(forged_stream)
    process[stream_name]['retained_bytes'] = len(forged_stream)
    _canonical_file(process_path, process)

    scanner = release_module._load_repository_module(
        repository,
        'tests/phase2_startup_gate.py',
        'coordinated final launch-log forgery scanner',
    )
    combined_path = run_root / 'full-stack-combined.log'
    combined_path.write_bytes(
        scanner.combined_launch_log_bytes(
            run_root / 'processes/full_stack.stdout.log',
            run_root / 'processes/full_stack.stderr.log',
            maximum_stream_bytes=8 * 1024 * 1024,
        )
    )
    replayed = scanner.scan_final_launch_log(
        combined_path.resolve(strict=True),
        recorded_gate['launch_stopped_utc'],
        scanned_utc=recorded_gate['scanned_utc'],
        lifecycle_ready_path=(run_root / 'lifecycle-ready.json').resolve(strict=True),
        **_phase3_lifecycle_replay_contract(repository, run_root),
    )
    assert replayed['verdict'] == 'FAIL'
    assert replayed['match_count'] >= 1
    replayed.update(
        {
            'failure_kind': None,
            'failure_message': None,
            'match_count': 0,
            'matches': [],
            'verdict': 'PASS',
        }
    )
    _canonical_file(gate_path, replayed, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: full-stack recorded projection differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_outputs(repository, candidate_root)
    with pytest.raises(EvidenceError, match='differs from clone-local scanner replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    ['extra_field', 'schema_version', 'hash', 'path', 'signature_order'],
)
def test_release_evidence_rejects_rebound_phase3_final_launch_gate_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    gate_path = run_root / 'full-stack-final-log-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'extra_field':
        gate['forged'] = True
    elif case == 'schema_version':
        gate['schema_version'] = 1
    elif case == 'hash':
        gate['launch_log_sha256'] = '0' * 64
    elif case == 'path':
        gate['launch_log_path'] = str(run_root / 'processes/full_stack.stdout.log')
    else:
        gate['signature_definitions'].reverse()
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='final launch-log signature gate'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['mission_before_graph', 'stack_ends_before_final_gate'])
def test_release_evidence_rejects_rebound_phase3_process_timeline_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    if case == 'mission_before_graph':
        process_path = run_root / 'processes/mission_runner.process.json'
        process = json.loads(process_path.read_text(encoding='utf-8'))
        process['started_steady_ns'] = 6_500_000_000
    else:
        process_path = run_root / 'processes/full_stack.process.json'
        process = json.loads(process_path.read_text(encoding='utf-8'))
        process['finished_steady_ns'] = 28_500_000_000
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    if case == 'stack_ends_before_final_gate':
        with pytest.raises(
            EvidenceError,
            match='smoke host profile failed validation: full-stack recorded projection differs',
        ):
            _validate_release_fixture(fixture)
        _rebind_phase3_smoke_profile_outputs(repository, candidate_root)

    with pytest.raises(EvidenceError, match='successful process timeline is invalid'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('command', 'forged mission'),
        ('exit_code', 7),
        ('wall_duration_s', 123.0),
        ('wall_timeout_s', 301.0),
        ('working_directory', '/tmp/forged'),
    ],
)
def test_release_evidence_rejects_phase3_orchestrator_execution_forgery(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestrator_path = run_root / 'orchestrator.json'
    orchestrator = json.loads(orchestrator_path.read_text(encoding='utf-8'))
    orchestrator['execution'][field] = value
    _canonical_file(orchestrator_path, orchestrator, sidecar=True)

    with pytest.raises(EvidenceError, match='exactly bind the mission process'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'omitted_path',
    [
        'resources.jsonl',
        'lifecycle-ready-map_server.txt',
        'full-stack-combined.log',
        'full-stack-final-log-gate.json',
        'full-stack-final-log-gate.json.sha256',
    ],
)
def test_release_evidence_rejects_deleted_rebound_phase3_prerequisite(
    tmp_path: Path,
    omitted_path: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    (run_root / omitted_path).unlink()
    _rebind_phase3_prerequisites(
        repository,
        run_root,
        omitted_paths=frozenset({omitted_path}),
    )

    with pytest.raises(EvidenceError, match='exact successful-run snapshot'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_deleted_rebound_phase3_process_triplet(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    omitted_paths = frozenset(
        {
            'processes/runtime_gate.process.json',
            'processes/runtime_gate.stderr.log',
            'processes/runtime_gate.stdout.log',
        }
    )
    for relative in omitted_paths:
        (run_root / relative).unlink()
    _rebind_phase3_prerequisites(repository, run_root, omitted_paths=omitted_paths)

    with pytest.raises(EvidenceError, match='production role set'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_scenario4_sampler_command(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'runs/09'
    process_path = run_root / 'processes/lifecycle_sampler.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    command = list(process['command'])
    command[command.index('--schedule') + 1] = str(run_root / 'forged-schedule.json')
    _rewrite_process_command(process, command)
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError,
        match='lifecycle_sampler process command, timeout, or outcome',
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_mission_csv_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    csv_path = run_root / 'mission-result.csv'
    rows = list(csv.reader(csv_path.read_text(encoding='utf-8').splitlines()))
    rows[1][0] = 'forged-run-id'
    csv_path.write_text(
        ''.join(','.join(row) + '\n' for row in rows),
        encoding='utf-8',
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='mission JSON/CSV reconciliation failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_resource_summary_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    resource_path = run_root / 'resources.jsonl'
    samples = [json.loads(line) for line in resource_path.read_text(encoding='utf-8').splitlines()]
    samples[0]['cpu_percent'] += 1.0
    resource_path.write_text(
        ''.join(json.dumps(sample, sort_keys=True) + '\n' for sample in samples),
        encoding='utf-8',
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='resources do not exactly match the raw resource'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_goal_binding_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    observer_path = run_root / 'goal-observer.json'
    observer = json.loads(observer_path.read_text(encoding='utf-8'))
    observer['accepted_goal_uuid'] = 'forged-goal-uuid'
    _canonical_file(observer_path, observer, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='goal UUID/T0 evidence differs'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_empty_domain_gate_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    gate_path = run_root / 'domain-cleanup.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    gate['nodes'].append('/robotest/forged_survivor')
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError,
        match=r'domain-cleanup\.json is not an exact empty-domain PASS',
    ):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('verdict', 'FAIL'),
        ('qos_contract_pass', False),
        ('namespace_isolation_pass', False),
        ('cmd_vel_owner_pass', False),
        ('validation_autonomy_isolation_pass', False),
    ],
)
def test_release_evidence_rejects_rebound_phase3_candidate_gate_contradiction(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    gate[field] = value
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError, match='candidate runtime gate is not a complete claimed PASS'
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_accepts_factual_unknown_phase3_qos_introspection(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    gate_path = Path(fixture['candidate_root']) / 'smoke/runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    assert gate['bounded_depth_live_proven_for_all_endpoints'] is False
    assert gate['qos_introspection_complete'] is False
    assert all(
        evidence['publisher_qos_pass'] is True and evidence['subscriber_qos_pass'] is True
        for evidence in gate['topics'].values()
    )

    report = _validate_release_fixture(fixture)
    assert report['status'] == 'PASS'


@pytest.mark.parametrize(
    'case',
    ['bounded_aggregate', 'introspection_aggregate', 'nested_bounded'],
)
def test_release_evidence_rejects_rebound_phase3_qos_introspection_inconsistency(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'bounded_aggregate':
        gate['bounded_depth_live_proven_for_all_endpoints'] = True
        expected = 'claims differ from source-bound replay'
    elif case == 'introspection_aggregate':
        gate['qos_introspection_complete'] = True
        expected = 'claims differ from source-bound replay'
    else:
        first_topic = next(iter(gate['topics'].values()))
        first_topic['bounded_depth_live_proven'] = True
        expected = 'differs from endpoint/QoS replay'
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match=expected):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('node_name', 'mutation', 'expected_error'),
    (
        (
            '/robotest/metrics_collector',
            'missing',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/scenario_controller',
            'missing',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/metrics_collector',
            'duplicate',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/scenario_controller',
            'duplicate',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/metrics_collector',
            'wrong_type',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/scenario_controller',
            'wrong_type',
            'fused /clock subscriber ownership failed',
        ),
        (
            '/robotest/metrics_collector',
            'malformed_gid',
            'not an exact runtime-gate endpoint record',
        ),
        (
            '/robotest/scenario_controller',
            'cross_subscriber_duplicate_gid',
            'fused /clock subscriber ownership failed',
        ),
    ),
)
def test_release_evidence_rejects_invalid_fused_clock_reader(
    tmp_path: Path,
    node_name: str,
    mutation: str,
    expected_error: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    clock = gate['topics']['/clock']
    subscriber = next(item for item in clock['subscribers'] if item['node'] == node_name)
    subscriber_check = next(
        item
        for item in clock['qos_checks']
        if item['side'] == 'subscriber' and item['node'] == node_name
    )
    if mutation == 'missing':
        clock['subscribers'].remove(subscriber)
        clock['qos_checks'].remove(subscriber_check)
    elif mutation == 'duplicate':
        duplicate = copy.deepcopy(subscriber)
        duplicate['gid'] = ('a' if node_name.endswith('metrics_collector') else 'b') * 32
        clock['subscribers'].append(duplicate)
        clock['qos_checks'].append(copy.deepcopy(subscriber_check))
    elif mutation == 'wrong_type':
        subscriber['topic_type'] = 'std_msgs/msg/String'
    elif mutation == 'malformed_gid':
        subscriber['gid'] = 'ab'
    else:
        assert mutation == 'cross_subscriber_duplicate_gid'
        other_reader = copy.deepcopy(subscriber)
        other_reader.update({'gid': 'c' * 32, 'node': '/robotest/amcl'})
        clock['subscribers'].append(other_reader)
        subscriber['gid'] = other_reader['gid']
        other_check = copy.deepcopy(subscriber_check)
        other_check['node'] = other_reader['node']
        clock['qos_checks'].append(other_check)
    if mutation in {'duplicate', 'wrong_type', 'cross_subscriber_duplicate_gid'}:
        clock['subscribers'].sort(key=lambda item: (item['node'], item['topic_type']))
        clock['qos_checks'].sort(key=lambda item: (item['side'], item['node']))
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match=expected_error):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    ['cross_authoritative_gid', 'rogue_publisher', 'wrong_contact_type'],
)
def test_release_evidence_rejects_cascaded_phase3_candidate_endpoint_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'cross_authoritative_gid':
        gate['topics']['/robotest/validation/ground_truth']['publishers'][0]['gid'] = gate[
            'topics'
        ]['/clock']['publishers'][0]['gid']
        expected = 'authoritative publisher ownership failed'
    elif case == 'rogue_publisher':
        topic = gate['topics']['/robotest/map']
        topic['publishers'][0]['node'] = '/robotest/forged_map_server'
        publisher_check = next(
            check for check in topic['qos_checks'] if check['side'] == 'publisher'
        )
        publisher_check['node'] = '/robotest/forged_map_server'
        expected = 'publisher_ownership differs from endpoint ownership replay'
    else:
        gate['topics']['/robotest/internal/raw_contacts']['publishers'][0]['topic_type'] = (
            'std_msgs/msg/String'
        )
        expected = 'publisher_ownership differs from endpoint ownership replay'
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match=expected):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_phase3_candidate_qos_override(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    topic = gate['topics']['/robotest/cmd_vel']
    topic['expected']['endpoint_depth_overrides'][0]['depth'] = 2_048
    gate['exact_static_qos_depth_contract']['/robotest/cmd_vel'] = copy.deepcopy(topic['expected'])
    endpoint = next(
        item for item in topic['subscribers'] if item['node'] == '/robotest/metrics_collector'
    )
    endpoint.update({'depth': 2_048, 'history': 'KEEP_LAST'})
    check = next(
        item
        for item in topic['qos_checks']
        if item['side'] == 'subscriber' and item['node'] == '/robotest/metrics_collector'
    )
    check.update(
        {
            'bounded_depth_live_proven': True,
            'exact_depth_live_proven': True,
            'expected_depth': 2_048,
            'introspection_complete': True,
            'policy_contract_pass': True,
        }
    )
    topic['bounded_depth_live_proven'] = all(
        item['bounded_depth_live_proven'] for item in topic['qos_checks']
    )
    topic['exact_depth_live_proven'] = all(
        item['exact_depth_live_proven'] for item in topic['qos_checks']
    )
    topic['qos_introspection_complete'] = all(
        item['introspection_complete'] for item in topic['qos_checks']
    )
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='differs from endpoint/QoS replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['publisher_type', 'raw_subscriber_owner'])
def test_release_evidence_rejects_cascaded_phase3_final_contact_endpoint_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'contact-stream-final-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    raw_contacts = gate['topics']['/robotest/internal/raw_contacts']
    if case == 'publisher_type':
        raw_contacts['publishers'][0]['topic_type'] = 'std_msgs/msg/String'
    else:
        raw_contacts['subscribers'][0]['node'] = '/robotest/forged_contact_sink'
        subscriber_check = next(
            check for check in raw_contacts['qos_checks'] if check['side'] == 'subscriber'
        )
        subscriber_check['node'] = '/robotest/forged_contact_sink'
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='final runtime-gate claims differ'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_candidate_node_snapshot_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    gate['nodes'].append('/robotest/forged_persistent_node')
    gate['nodes'].sort()
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='runtime-gate/graph node replay failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_phase3_candidate_endpoint_type_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    gate['topics']['/robotest/map']['publishers'][0]['topic_type'] = 'std_msgs/msg/String'
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='endpoint types differ from the validated graph'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_phase3_unknown_endpoint_node(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    clock = gate['topics']['/clock']
    subscriber = copy.deepcopy(clock['subscribers'][0])
    subscriber.update({'gid': 'e' * 32, 'node': '/robotest/forged_unknown_node'})
    clock['subscribers'].append(subscriber)
    clock['subscribers'].sort(key=lambda item: (item['node'], item['topic_type']))
    subscriber_check = copy.deepcopy(
        next(check for check in clock['qos_checks'] if check['side'] == 'subscriber')
    )
    subscriber_check['node'] = subscriber['node']
    clock['qos_checks'].append(subscriber_check)
    clock['qos_checks'].sort(key=lambda item: (item['side'], item['node']))
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='endpoint nodes differ from the validated graph'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_phase3_final_wrong_type_subscriber(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / 'contact-stream-final-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    contacts = gate['topics']['/robotest/validation/contacts']
    forged_subscriber = copy.deepcopy(contacts['subscribers'][0])
    forged_subscriber.update(
        {
            'gid': 'd' * 32,
            'node': '/robotest/fault_proxy',
            'topic_type': 'std_msgs/msg/String',
        }
    )
    contacts['subscribers'].append(forged_subscriber)
    contacts['subscribers'].sort(key=lambda item: (item['node'], item['topic_type']))
    forged_check = copy.deepcopy(
        next(check for check in contacts['qos_checks'] if check['side'] == 'subscriber')
    )
    forged_check['node'] = forged_subscriber['node']
    contacts['qos_checks'].append(forged_check)
    contacts['qos_checks'].sort(key=lambda item: (item['side'], item['node']))
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='endpoint types differ from the validated graph'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_legacy_service_graph_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 legacy-service graph forgery fixture',
    )
    graph_path = run_root / 'graph.json'
    graph = orchestration._load_phase3_graph_json(graph_path)
    graph['observed']['services']['/robotest/faults/load_schedule'] = [
        'robotest_interfaces/srv/LoadFaultSchedule'
    ]
    graph_path.write_bytes(orchestration._phase3_graph_json_bytes(graph))
    (run_root / 'services.txt').write_text(
        orchestration._phase3_graph_text(graph['observed']['services']),
        encoding='utf-8',
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='service claims differ from the validated graph'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('gate_name', 'case'),
    [
        ('runtime-gate.json', 'extra'),
        ('runtime-gate.json', 'missing'),
        ('contact-stream-final-gate.json', 'extra'),
        ('contact-stream-final-gate.json', 'missing'),
    ],
)
def test_release_evidence_rejects_rebound_phase3_runtime_gate_shape_forgery(
    tmp_path: Path,
    gate_name: str,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    run_root = candidate_root / 'smoke'
    gate_path = run_root / gate_name
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'extra':
        gate['unexpected_claim'] = True
    elif gate_name == 'runtime-gate.json':
        del gate['scenario_services_missing']
    else:
        del gate['raw_subscriber_ownership']
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_smoke_runtime_gate(repository, candidate_root, run_root)

    with pytest.raises(EvidenceError, match='runtime gate is not a complete claimed PASS'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_graph_semantic_tamper(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 graph tamper fixture',
    )
    graph_path = run_root / 'graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    action_name = orchestration.PHASE3_GRAPH_ACTION_NAME
    action_type = orchestration.PHASE3_GRAPH_ACTION_TYPE
    graph['observed']['action_clients'] = {
        action_name: {'/robotest/metrics_collector': [action_type]}
    }
    ownership, mismatches = orchestration._phase3_graph_action_results(
        graph['contracts'],
        graph['observed']['action_clients'],
        graph['observed']['action_servers'],
    )
    graph['results']['action_ownership'] = ownership
    graph['action_ownership_mismatches'] = mismatches
    graph_path.write_bytes(orchestration._phase3_graph_json_bytes(graph))
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='semantic PASS'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_graph_text_tamper(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    projection = run_root / 'mission-actions.txt'
    projection.write_text(
        projection.read_text(encoding='utf-8') + '/forged [example_msgs/action/Forged]\n',
        encoding='utf-8',
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='does not match the graph JSON projection'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('prefix', 'field', 'value'),
    [
        ('', 'elapsed_wall_seconds', 4.999),
        ('mission-', 'attempt_count', 1),
    ],
)
def test_release_evidence_rejects_rebound_phase3_graph_quiet_window_forgery(
    tmp_path: Path,
    prefix: str,
    field: str,
    value: int | float,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 quiet-window graph forgery fixture',
    )
    graph_path = run_root / f'{prefix}graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    graph[field] = value
    _rewrite_phase3_graph_fixture(
        run_root,
        orchestration,
        graph,
        mission_client=bool(prefix),
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='graph artifact replay failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('projection', 'name', 'types'),
    [
        ('topics', '/robotest/forged_mission_topic', ['std_msgs/msg/String']),
        (
            'actions',
            '/robotest/forged_mission_action',
            ['nav2_msgs/action/NavigateToPose'],
        ),
    ],
)
def test_release_evidence_rejects_rebound_phase3_mission_graph_growth(
    tmp_path: Path,
    projection: str,
    name: str,
    types: list[str],
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 mission graph growth forgery fixture',
    )
    graph_path = run_root / 'mission-graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    graph['observed'][projection][name] = types
    _rewrite_phase3_graph_fixture(
        run_root,
        orchestration,
        graph,
        mission_client=True,
        projections=(projection,),
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='graph pair replay failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'unauthorized_node',
        'unauthorized_service',
        'wrong_transition_service_type',
        'transition_prefix_confusion',
    ],
)
def test_release_evidence_rejects_rebound_phase3_graph_transition_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 graph transition forgery fixture',
    )
    graph_path = run_root / 'mission-graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    projections: tuple[str, ...]
    if case == 'unauthorized_node':
        _add_phase3_graph_node(graph, '/robotest/mission_runner_helper')
        projections = ('nodes',)
    elif case == 'unauthorized_service':
        graph['observed']['services']['/robotest/unrelated/get_parameters'] = [
            'rcl_interfaces/srv/GetParameters'
        ]
        projections = ('services',)
    elif case == 'wrong_transition_service_type':
        graph['observed']['services']['/robotest/mission_runner/get_parameters'] = [
            'rcl_interfaces/srv/SetParameters'
        ]
        projections = ('services',)
    else:
        graph['observed']['services']['/robotest/mission_runner_helper/get_parameters'] = [
            'rcl_interfaces/srv/GetParameters'
        ]
        projections = ('services',)
    _rewrite_phase3_graph_fixture(
        run_root,
        orchestration,
        graph,
        mission_client=True,
        projections=projections,
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='graph pair replay failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_mission_only_hidden_node(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 hidden-node graph forgery fixture',
    )
    graph_path = run_root / 'mission-graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    _add_phase3_graph_node(
        graph,
        '/robotest/_mission_only_hidden_node',
        hidden=True,
    )
    _rewrite_phase3_graph_fixture(
        run_root,
        orchestration,
        graph,
        mission_client=True,
    )
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='graph pair replay failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_graph_duration_contradiction(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 graph duration forgery fixture',
    )
    process_path = run_root / 'processes/graph_gate.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    process['finished_steady_ns'] = process['started_steady_ns'] + int(
        (orchestration.PHASE3_GRAPH_QUIET_WINDOW_S - 0.25) * 1_000_000_000
    )
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(
        EvidenceError, match='graph elapsed time exceeds its owned process duration'
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_phase3_graph_watch_pid_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    process_path = run_root / 'processes/graph_gate.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    watch_index = process['command'].index('--watch-pid') + 1
    forged_watch_pid = int(process['command'][watch_index]) + 97
    process['command'][watch_index] = str(forged_watch_pid)
    process['wrapped_command'] = [
        'timeout',
        '--signal=TERM',
        '--kill-after=10s',
        f'{process["wall_timeout_s"]:.3f}s',
        *process['command'],
    ]
    _canonical_file(process_path, process)
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 cascaded watch-PID tamper fixture',
    )
    graph_path = run_root / 'graph.json'
    graph = json.loads(graph_path.read_text(encoding='utf-8'))
    graph['watch_pid'] = forged_watch_pid
    graph_path.write_bytes(orchestration._phase3_graph_json_bytes(graph))
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match='graph_gate process command, timeout, or outcome'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase3_orchestrator_graph_hash_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_root = Path(fixture['candidate_root']) / 'smoke'
    orchestrator_path = run_root / 'orchestrator.json'
    orchestrator = json.loads(orchestrator_path.read_text(encoding='utf-8'))
    orchestrator['artifacts']['pre_mission_graph_sha256'] = 'f' * 64
    _canonical_file(orchestrator_path, orchestrator, sidecar=True)

    with pytest.raises(EvidenceError, match='does not match its prerequisite manifest'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_incomplete_phase3_pass_vector(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    bundle = Path(fixture['candidate_root']) / 'runs/00/result'
    result_path = bundle / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['verdict']['mission_success'] = False
    _canonical_file(result_path, result)
    _refresh_phase3_bundle(bundle)

    with pytest.raises(EvidenceError, match='is not a full canonical PASS'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase3_plan_identity_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    bundle = Path(fixture['candidate_root']) / 'runs/00/result'
    result_path = bundle / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['identity']['ros_domain_id'] = 200
    _canonical_file(result_path, result)
    result_sha = _refresh_phase3_bundle(bundle)
    aggregate_path = Path(fixture['aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    aggregate['identity']['ordered_source_json_sha256'][0] = result_sha
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )
    with pytest.raises(EvidenceError, match='identity mismatch for ros_domain_id'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase3_target_binding_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    bundle = Path(fixture['candidate_root']) / 'runs/00/result'
    result_path = bundle / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['targets']['collector_configuration_sha256'] = 'f' * 64
    result['quality']['candidate_identity']['hashes']['collector_configuration_sha256'] = 'f' * 64
    _canonical_file(result_path, result)
    result_sha = _refresh_phase3_bundle(bundle)
    aggregate_path = Path(fixture['aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    aggregate['identity']['ordered_source_json_sha256'][0] = result_sha
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )

    with pytest.raises(EvidenceError, match='production analysis replay'):
        _validate_release_fixture(fixture)


def _contact_aggregator_binary_fixture(*, symlink_install: bool) -> dict[str, object]:
    source_inventory_sha256 = '5' * 64
    binary_sha256 = '7' * 64
    build_id = 'b' * 40
    return {
        'build_embedded_source_inventory_match': True,
        'build_embedded_source_inventory_sha256': source_inventory_sha256,
        'build_elf_build_id': build_id,
        'build_install_build_id_match': True,
        'build_install_embedded_source_inventory_match': True,
        'build_install_samefile': symlink_install,
        'build_install_sha256_match': True,
        'build_path': release_module.CONTACT_AGGREGATOR_BUILD_PATH,
        'build_regular_file': True,
        'build_sha256': binary_sha256,
        'installed_declared_is_symlink': symlink_install,
        'installed_declared_path': release_module.CONTACT_AGGREGATOR_INSTALL_PATH,
        'installed_embedded_source_inventory_match': True,
        'installed_embedded_source_inventory_sha256': source_inventory_sha256,
        'installed_elf_build_id': build_id,
        'installed_path': (
            release_module.CONTACT_AGGREGATOR_BUILD_PATH
            if symlink_install
            else release_module.CONTACT_AGGREGATOR_INSTALL_PATH
        ),
        'installed_regular_file': True,
        'installed_sha256': binary_sha256,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': source_inventory_sha256,
    }


@pytest.mark.parametrize('symlink_install', [False, True])
def test_release_contact_aggregator_binding_accepts_copy_and_symlink_installs(
    symlink_install: bool,
) -> None:
    release_module._validate_contact_aggregator_binary_binding(
        _contact_aggregator_binary_fixture(symlink_install=symlink_install)
    )


@pytest.mark.parametrize(
    ('field', 'value', 'message'),
    [
        ('unexpected', True, 'binding changed'),
        ('build_regular_file', False, 'binding changed'),
        ('schema_version', True, 'binding changed'),
        ('installed_declared_is_symlink', 1, 'binding changed'),
        ('build_install_samefile', False, 'declared/resolved install path'),
        (
            'installed_path',
            release_module.CONTACT_AGGREGATOR_INSTALL_PATH,
            'declared/resolved install path',
        ),
        ('installed_sha256', '8' * 64, 'build/install hash differs'),
        ('installed_elf_build_id', 'c' * 40, 'ELF build ID binding is invalid'),
        (
            'installed_embedded_source_inventory_sha256',
            '8' * 64,
            'embedded source or build/install hash differs',
        ),
    ],
)
def test_release_contact_aggregator_binding_rejects_tampering(
    field: str, value: object, message: str
) -> None:
    binding = _contact_aggregator_binary_fixture(symlink_install=True)
    binding[field] = value
    with pytest.raises(EvidenceError, match=message):
        release_module._validate_contact_aggregator_binary_binding(binding)


def test_release_evidence_rejects_unshared_contact_binary_inventory(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    aggregator = binding['contact_aggregator_binary']
    rebound_inventory_sha256 = 'f' * 64
    aggregator['source_inventory_sha256'] = rebound_inventory_sha256
    aggregator['build_embedded_source_inventory_sha256'] = rebound_inventory_sha256
    aggregator['installed_embedded_source_inventory_sha256'] = rebound_inventory_sha256
    _canonical_file(binding_path, binding, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate build_binding differs',
    ):
        _validate_release_fixture(fixture)

    _rebind_phase3_smoke_profile_candidate_inputs(
        fixture,
        plugin_source_inventory_sha256=rebound_inventory_sha256,
    )

    with pytest.raises(EvidenceError, match='do not share one source inventory'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase3_drain_stamp(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_root = Path(fixture['candidate_root']) / 'runs/00'
    request_path = run_root / 'analysis-request.json'
    request = json.loads(request_path.read_text(encoding='utf-8'))
    request['collision']['drain_completed_stamp_ns'] += 1
    _canonical_file(request_path, request, sidecar=True)
    analysis = importlib.import_module('robotest_metrics.analysis')
    result_path = run_root / 'result/run-result.json'
    _canonical_file(result_path, analysis.analyze_run(request))
    _refresh_phase3_bundle(result_path.parent)

    with pytest.raises(EvidenceError, match='strict authoritative post-terminal snapshot'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_coordinated_phase3_aggregate_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    aggregate_path = Path(fixture['aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    aggregate['scenarios']['1']['metrics']['measurements.completion_time_sim_s']['median'] = 9.0
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )

    with pytest.raises(EvidenceError, match='does not exactly recompute from runs'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase3_aggregate_number(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    aggregate_path = Path(fixture['aggregate'])
    aggregate = json.loads(aggregate_path.read_text(encoding='utf-8'))
    aggregate['scenarios']['1']['success_rate'] = True
    _canonical_file(aggregate_path, aggregate)
    (aggregate_path.parent / 'aggregate-result.csv').write_bytes(
        release_module._one_row_csv_bytes(aggregate)
    )

    with pytest.raises(EvidenceError, match='does not exactly recompute from runs'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase3_plan_schema(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    plan_path = Path(fixture['candidate_root']) / 'suite-plan.json'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    plan['schema_version'] = True
    _canonical_file(plan_path, plan, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate suite_plan differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_candidate_inputs(fixture)
    with pytest.raises(EvidenceError, match='suite plan contract changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase3_manifest_count(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['source']['file_count'] = True
    _canonical_file(binding_path, binding, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate build_binding differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_candidate_inputs(fixture)
    with pytest.raises(EvidenceError, match='counters or aggregate hash do not reconcile'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_positive_control_exit_code(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'positive-control/positive-binding.json'
    marker_path = candidate_root / 'positive-control/PASS.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['positive_control']['verdict']['exit_code'] = False
    positive_sha = release_module._canonical_sha256(binding['positive_control'])
    binding['positive_control_json_sha256'] = positive_sha
    binding['benchmark_binding']['positive_control_json_sha256'] = positive_sha
    _canonical_file(binding_path, binding, sidecar=True)
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    marker['positive_binding_sha256'] = phase5_module.file_sha256(binding_path)
    _canonical_file(marker_path, marker, sidecar=True)

    with pytest.raises(EvidenceError, match='positive-control result schema failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_treats_passive_release_clock_offset_as_diagnostic() -> None:
    reconciliation = {
        'release_delivery_clock_offset_ns': 278_000_000,
        'release_delivery_clock_stamp_ns': 578_000_000,
        'release_qualified_snapshot_stamp_ns': 300_000_000,
    }
    assert release_module._passive_release_clock_offset_is_consistent(reconciliation)

    reconciliation['release_delivery_clock_offset_ns'] += 1
    assert not release_module._passive_release_clock_offset_is_consistent(reconciliation)


def test_phase5_clone_local_recomposition_accepts_precontrol_contact_delivery_skew(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    result_path = candidate_root / 'positive-control/contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    control = result['control']
    first_forward_sequence = control['command_trace'][0]['collector_sequence']
    precontrol_snapshot = next(
        snapshot
        for snapshot in control['contact']['snapshots']
        if snapshot['collector_sequence'] < first_forward_sequence
    )
    precontrol_snapshot['delivery_clock_offset_ns'] = -300_000_000
    precontrol_snapshot['delivery_clock_stamp_ns'] = (
        precontrol_snapshot['sim_stamp_ns'] - 300_000_000
    )
    _canonical_file(result_path, result, sidecar=True)
    _rebind_phase3_positive_raw(candidate_root, repository)

    binding = json.loads(
        (candidate_root / 'positive-control/positive-binding.json').read_text(encoding='utf-8')
    )
    assert (
        binding['collector_reconciliation'][
            'contact_delivery_offset_strict_from_collector_sequence'
        ]
        == first_forward_sequence
    )
    assert (
        binding['positive_control']['control']['contact']['snapshots'][0][
            'delivery_clock_offset_ns'
        ]
        == -300_000_000
    )


def test_release_evidence_rejects_rebound_active_contact_delivery_skew(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    result_path = candidate_root / 'positive-control/contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    control = result['control']
    first_forward_sequence = control['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot
        for snapshot in control['contact']['snapshots']
        if snapshot['collector_sequence'] >= first_forward_sequence
    )
    active_snapshot['delivery_clock_offset_ns'] = 221_000_000
    active_snapshot['delivery_clock_stamp_ns'] = active_snapshot['sim_stamp_ns'] + 221_000_000
    _canonical_file(result_path, result, sidecar=True)
    _refresh_phase3_positive_component_manifest_from_repository(
        candidate_root,
        repository,
    )

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_active_snapshot_resequenced_before_forward(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    result_path = candidate_root / 'positive-control/contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    control = result['control']
    contact = control['contact']
    first_forward_sequence = control['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot
        for snapshot in contact['snapshots']
        if snapshot['collector_sequence'] >= first_forward_sequence
    )
    original_snapshot_sequence = active_snapshot['collector_sequence']
    forged_snapshot_sequence = first_forward_sequence - 1
    active_snapshot['collector_sequence'] = forged_snapshot_sequence
    active_snapshot['delivery_clock_offset_ns'] = 221_000_000
    active_snapshot['delivery_clock_stamp_ns'] = active_snapshot['sim_stamp_ns'] + 221_000_000
    for record in contact['snapshot_records']:
        if record['snapshot_sequence'] == original_snapshot_sequence:
            record['snapshot_sequence'] = forged_snapshot_sequence
    for record in active_snapshot['counted_snapshot_records']:
        record['snapshot_sequence'] = forged_snapshot_sequence
    _canonical_file(result_path, result, sidecar=True)
    _refresh_phase3_positive_component_manifest_from_repository(
        candidate_root,
        repository,
    )

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_cascaded_active_skew_boundary_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    result_path = positive_directory / 'contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    control = result['control']
    original_boundary = control['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot
        for snapshot in control['contact']['snapshots']
        if snapshot['collector_sequence'] >= original_boundary
    )
    active_snapshot['delivery_clock_offset_ns'] = 221_000_000
    active_snapshot['delivery_clock_stamp_ns'] = active_snapshot['sim_stamp_ns'] + 221_000_000
    forged_boundary = active_snapshot['collector_sequence'] + 1
    control['command_trace'][0]['collector_sequence'] = forged_boundary
    _canonical_file(result_path, result, sidecar=True)

    binding_path = positive_directory / 'positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['positive_control'] = copy.deepcopy(result)
    binding['collector_reconciliation'][
        'contact_delivery_offset_strict_from_collector_sequence'
    ] = forged_boundary
    _refresh_phase3_positive_binding(candidate_root, binding)
    _refresh_phase3_positive_component_manifest_from_repository(
        candidate_root,
        repository,
    )

    with pytest.raises(EvidenceError, match='Phase 3 collision qualification failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_missing_collision_coverage_sha(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'positive-control/positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    del binding['coverage_manifest']['bridge_sha256']
    for name in ('benchmark_provenance', 'positive_control_provenance'):
        binding['benchmark_binding'][name]['bridge_sha256'] = None
    _refresh_phase3_positive_binding(candidate_root, binding, rebind_coverage=True)

    with pytest.raises(EvidenceError, match='Phase 3 collision qualification failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('section', ['configuration', 'control', 'quality'])
def test_release_evidence_rejects_incomplete_positive_control(tmp_path: Path, section: str) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'positive-control/positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    del binding['positive_control'][section]
    _refresh_phase3_positive_binding(candidate_root, binding)

    with pytest.raises(EvidenceError, match='positive-control result schema failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_positive_control_raw_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    result_path = candidate_root / 'positive-control/contact-control-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    record = result['control']['contact']['snapshot_records'][0]
    record.update(
        {
            'counterpart_collision': None,
            'counterpart_model': None,
            'disposition': 'non_robot_pair_ignored',
            'robot_collision': None,
        }
    )
    _canonical_file(result_path, result, sidecar=True)
    _rebind_phase3_positive_raw(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control result schema failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_deleted_positive_control_arm_request(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    (candidate_root / 'positive-control/contact-control.arm.json').unlink()

    with pytest.raises(EvidenceError, match='positive-control arm request'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['gate_tamper', 'boolean', 'order', 'hash'])
def test_release_evidence_rejects_positive_control_arm_forgeries(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    positive_directory = Path(fixture['candidate_root']) / 'positive-control'
    if case == 'gate_tamper':
        path = positive_directory / 'runtime-gate.json'
        document = json.loads(path.read_text(encoding='utf-8'))
        document['qos_contract_pass'] = False
        _canonical_file(path, document, sidecar=True)
    elif case == 'boolean':
        path = positive_directory / 'contact-control.armed.json'
        document = json.loads(path.read_text(encoding='utf-8'))
        document['schema_version'] = True
        _canonical_file(path, document)
    elif case == 'order':
        path = positive_directory / 'contact-control.armed.json'
        document = json.loads(path.read_text(encoding='utf-8'))
        document['arm_observed_steady_ns'] = 3_999_999
        _canonical_file(path, document)
    else:
        path = positive_directory / 'contact-control.arm.json'
        document = json.loads(path.read_text(encoding='utf-8'))
        document['arm_protocol_sha256'] = '0' * 64
        _canonical_file(path, document)

    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 production orchestration arm-forgery fixture',
    )
    _refresh_phase3_positive_component_manifest(positive_directory, orchestration)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['missing', 'symlink'])
def test_release_evidence_requires_regular_positive_command_progress(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    progress_path = Path(fixture['candidate_root']) / 'positive-control/command-progress.json'
    if case == 'missing':
        progress_path.unlink()
    else:
        target = tmp_path / 'external-command-progress.json'
        target.write_bytes(progress_path.read_bytes())
        progress_path.unlink()
        progress_path.symlink_to(target)

    with pytest.raises(
        EvidenceError,
        match=r'command[- ]progress|component manifest|component artifact',
    ):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'wrong_run',
        'nonzero',
        'retained_count_two',
        'matched_count_one',
        'matched_count_three',
        'callback_before_publish',
        'probe_before_observed_sim',
        'probe_after_armed_sim',
        'sim_lag',
        'valid_progress_capture_mismatch',
        'wrong_hash',
    ],
)
def test_release_evidence_rejects_rebound_positive_command_probe(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    progress_path = positive_directory / 'command-progress.json'
    progress = json.loads(progress_path.read_text(encoding='utf-8'))
    acknowledgment_path = positive_directory / 'contact-control.armed.json'
    acknowledgment = json.loads(acknowledgment_path.read_text(encoding='utf-8'))
    probe = acknowledgment['command_delivery_probe']
    if case == 'wrong_run':
        progress['run_id'] = 'foreign-positive-control'
    elif case == 'nonzero':
        progress['linear_x_m_s'] = 0.01
    elif case == 'retained_count_two':
        progress['retained_command_count'] = 2
    elif case == 'matched_count_one':
        probe['matched_subscription_count'] = 1
    elif case == 'matched_count_three':
        probe['matched_subscription_count'] = 3
    elif case == 'callback_before_publish':
        progress['observed_steady_ns'] = probe['probe_publish_started_steady_ns'] - 1
    elif case == 'probe_before_observed_sim':
        probe['probe_sim_stamp_ns'] = acknowledgment['arm_observed_sim_stamp_ns']
        progress['stamp_ns'] = probe['probe_sim_stamp_ns']
    elif case == 'probe_after_armed_sim':
        probe['probe_sim_stamp_ns'] = acknowledgment['armed_sim_stamp_ns'] + 1
        progress['stamp_ns'] = probe['probe_sim_stamp_ns']
    elif case == 'sim_lag':
        progress['stamp_ns'] = probe['probe_sim_stamp_ns'] + 100_000_001
    elif case == 'valid_progress_capture_mismatch':
        progress['stamp_ns'] += 1
    else:
        probe['collector_progress_sha256'] = '0' * 64
    _canonical_file(progress_path, progress)
    if case != 'wrong_hash':
        probe.update(
            {
                'collector_progress_observed_steady_ns': progress['observed_steady_ns'],
                'collector_progress_sha256': phase5_module.file_sha256(progress_path),
                'collector_progress_stamp_ns': progress['stamp_ns'],
            }
        )
    _canonical_file(acknowledgment_path, acknowledgment)
    _rebind_phase3_positive_command_probe(candidate_root, repository)

    if case == 'valid_progress_capture_mismatch':
        expected_error = 'distinct command-delivery zero probe'
    elif case in {'matched_count_one', 'matched_count_three'}:
        expected_error = 'positive-control result schema failed'
    elif case in {
        'callback_before_publish',
        'probe_before_observed_sim',
        'probe_after_armed_sim',
        'sim_lag',
        'wrong_hash',
    }:
        expected_error = 'collision qualification failed'
    else:
        expected_error = 'positive-control recomposition failed'
    with pytest.raises(EvidenceError, match=expected_error):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['missing_probe', 'duplicate_probe', 'trailing_extra'])
def test_release_evidence_rejects_rebound_positive_command_capture(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    capture_path = candidate_root / 'positive-control/capture.json'
    capture = json.loads(capture_path.read_text(encoding='utf-8'))
    stream = capture['streams']['cmd_vel']
    items = stream['items']
    quality = stream['quality']
    if case == 'missing_probe':
        items.pop(0)
        quality['ingress_count'] -= 1
        quality['retained_count'] -= 1
    elif case == 'duplicate_probe':
        duplicate = copy.deepcopy(items[0])
        duplicate['collector_sequence'] += 100_000
        items.insert(1, duplicate)
        quality['ingress_count'] += 1
        quality['retained_count'] += 1
    else:
        trailing = copy.deepcopy(items[-1])
        trailing['collector_sequence'] += 100_000
        trailing['stamp_ns'] += 1
        items.append(trailing)
        quality['ingress_count'] += 1
        quality['retained_count'] += 1
    _canonical_file(capture_path, capture)
    _rebind_phase3_positive_capture(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'publisher_missing',
        'publisher_extra',
        'publisher_non_boolean',
        'subscriber_missing',
        'subscriber_wrong',
        'subscriber_non_boolean',
    ],
)
def test_release_evidence_rejects_rebound_runtime_gate_ownership_keys(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    gate_path = candidate_root / 'positive-control/runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'publisher_missing':
        del gate['contact_publisher_ownership']['/robotest/internal/raw_contacts']
    elif case == 'publisher_extra':
        gate['contact_publisher_ownership']['/robotest/forged_contacts'] = True
    elif case == 'publisher_non_boolean':
        gate['contact_publisher_ownership']['/robotest/internal/raw_contacts'] = 1
    elif case == 'subscriber_missing':
        gate['contact_subscriber_ownership'] = {}
    elif case == 'subscriber_non_boolean':
        gate['contact_subscriber_ownership']['/robotest/internal/raw_contacts'] = 1
    else:
        gate['contact_subscriber_ownership'] = {'/robotest/validation/contacts': True}
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['missing', 'false', 'extra'])
def test_release_evidence_rejects_cascaded_authoritative_projection_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    gate_path = candidate_root / 'positive-control/runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    projection = gate['authoritative_publisher_ownership']
    topic = '/robotest/validation/ground_truth'
    if case == 'missing':
        del projection[topic]
    elif case == 'false':
        projection[topic] = False
    else:
        projection['/robotest/validation/forged_source'] = True
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    ['extra', 'missing', 'wrong_type', 'duplicate_gid', 'cross_topic_duplicate_gid'],
)
def test_release_evidence_rejects_cascaded_authoritative_source_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    gate_path = candidate_root / 'positive-control/runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'cross_topic_duplicate_gid':
        gate['topics']['/robotest/validation/ground_truth']['publishers'][0]['gid'] = gate[
            'topics'
        ]['/clock']['publishers'][0]['gid']
    else:
        topic = gate['topics']['/robotest/validation/scenario_entity_poses']
        publishers = topic['publishers']
        publisher_checks = [check for check in topic['qos_checks'] if check['side'] == 'publisher']
        subscriber_checks = [
            check for check in topic['qos_checks'] if check['side'] == 'subscriber'
        ]
        if case == 'extra':
            forged_endpoint = copy.deepcopy(publishers[-1])
            forged_endpoint['gid'] = 'e' * 32
            publishers.append(forged_endpoint)
            publisher_checks.append(copy.deepcopy(publisher_checks[-1]))
        elif case == 'missing':
            publishers.pop()
            publisher_checks.pop()
        elif case == 'wrong_type':
            publishers[0]['topic_type'] = 'std_msgs/msg/String'
        else:
            publishers[1]['gid'] = publishers[0]['gid']
        publishers.sort(key=lambda item: (item['node'], item['topic_type']))
        topic['qos_checks'] = sorted(
            [*publisher_checks, *subscriber_checks],
            key=lambda item: (item['side'], item['node']),
        )
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'rebound_cmd_vel_override',
        'duplicate_endpoint_gid',
        'wrong_contact_type',
        'extra_contact_owner',
    ],
)
def test_release_evidence_rejects_cascaded_nested_runtime_gate_qos_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    gate_path = candidate_root / 'positive-control/runtime-gate.json'
    gate = json.loads(gate_path.read_text(encoding='utf-8'))
    if case == 'rebound_cmd_vel_override':
        topic = gate['topics']['/robotest/cmd_vel']
        topic['expected']['endpoint_depth_overrides'][0]['depth'] = 2_048
        gate['exact_static_qos_depth_contract']['/robotest/cmd_vel'] = copy.deepcopy(
            topic['expected']
        )
        endpoint = next(
            item for item in topic['subscribers'] if item['node'] == '/robotest/metrics_collector'
        )
        endpoint['depth'] = 2_048
        check = next(
            item
            for item in topic['qos_checks']
            if item['side'] == 'subscriber' and item['node'] == '/robotest/metrics_collector'
        )
        check['expected_depth'] = 2_048
    elif case == 'duplicate_endpoint_gid':
        topic = gate['topics']['/robotest/cmd_vel']
        topic['subscribers'][0]['gid'] = topic['publishers'][0]['gid']
    elif case == 'wrong_contact_type':
        topic = gate['topics']['/robotest/internal/raw_contacts']
        topic['publishers'][0]['topic_type'] = 'std_msgs/msg/String'
    else:
        topic = gate['topics']['/robotest/internal/raw_contacts']
        forged_endpoint = copy.deepcopy(topic['publishers'][0])
        forged_endpoint.update(
            {
                'gid': 'f' * 32,
                'node': '/robotest/forged_parameter_bridge',
            }
        )
        topic['publishers'].append(forged_endpoint)
        topic['publishers'].sort(key=lambda item: (item['node'], item['topic_type']))
        forged_check = copy.deepcopy(
            next(item for item in topic['qos_checks'] if item['side'] == 'publisher')
        )
        forged_check['node'] = '/robotest/forged_parameter_bridge'
        topic['qos_checks'].append(forged_check)
        topic['qos_checks'].sort(key=lambda item: (item['side'], item['node']))
    _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)

    with pytest.raises(EvidenceError, match='positive-control recomposition failed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'case',
    [
        'wrong_executable',
        'wrong_script',
        'wrong_mode',
        'wrong_workspace',
        'wrong_inner_timeout',
        'wrong_outer_timeout',
        'wrong_watch_pid',
        'wrong_launch_pid',
        'wrong_domain',
        'wrong_partition',
        'missing_runtime_cwd',
        'extra_runtime_field',
        'missing_runtime_stream_field',
        'extra_runtime_stream_field',
        'missing_driver_cwd',
        'missing_driver_command_progress',
        'wrong_collector_command_progress_run_id',
        'missing_sim_stream_field',
    ],
)
def test_release_evidence_rejects_rebound_runtime_gate_process_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    process_directory = candidate_root / 'positive-control/processes'
    process_path = process_directory / 'runtime_gate.process.json'
    if case in {'missing_driver_cwd', 'missing_driver_command_progress'}:
        process_path = process_directory / 'contact_control_driver.process.json'
    elif case == 'wrong_collector_command_progress_run_id':
        process_path = process_directory / 'metrics_collector.process.json'
    elif case == 'missing_sim_stream_field':
        process_path = process_directory / 'sim_launch.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    command = list(process['command'])
    if case == 'wrong_executable':
        command[0] = 'python'
        _rewrite_process_command(process, command)
    elif case == 'wrong_script':
        command[1] = str(repository / 'tests/forged_runtime_gate.py')
        _rewrite_process_command(process, command)
    elif case == 'wrong_mode':
        command[command.index('--mode') + 1] = 'candidate'
        _rewrite_process_command(process, command)
    elif case == 'wrong_workspace':
        forged_workspace = repository.parent.resolve()
        process['cwd'] = str(forged_workspace)
        command[1] = str(forged_workspace / 'tests/phase3_runtime_gate.py')
        command[command.index('--workspace') + 1] = str(forged_workspace)
        _rewrite_process_command(process, command)
    elif case == 'wrong_inner_timeout':
        command[command.index('--wall-timeout-s') + 1] = '19.0'
        _rewrite_process_command(process, command)
    elif case == 'wrong_outer_timeout':
        process['wall_timeout_s'] = 24.0
        process['wrapped_command'][3] = '24.000s'
    elif case == 'wrong_watch_pid':
        command[command.index('--watch-pid') + 1] = str(
            int(command[command.index('--watch-pid') + 1]) + 1
        )
        _rewrite_process_command(process, command)
    elif case == 'wrong_launch_pid':
        command[command.index('--launch-pid') + 1] = str(
            int(command[command.index('--launch-pid') + 1]) + 1
        )
        _rewrite_process_command(process, command)
    elif case == 'wrong_domain':
        command[command.index('--expected-domain-id') + 1] = str(
            int(command[command.index('--expected-domain-id') + 1]) + 1
        )
        _rewrite_process_command(process, command)
    elif case == 'wrong_partition':
        partition_index = command.index('--expected-gz-partition') + 1
        command[partition_index] = f'{command[partition_index]}-forged'
        _rewrite_process_command(process, command)
    elif case in {'missing_runtime_cwd', 'missing_driver_cwd'}:
        del process['cwd']
    elif case == 'missing_driver_command_progress':
        progress_index = command.index('--command-progress-file')
        del command[progress_index : progress_index + 2]
        _rewrite_process_command(process, command)
    elif case == 'wrong_collector_command_progress_run_id':
        run_index = command.index('--command-progress-run-id') + 1
        command[run_index] = f'{command[run_index]}-forged'
        _rewrite_process_command(process, command)
    elif case == 'extra_runtime_field':
        process['unexpected'] = 'forged'
    elif case == 'missing_runtime_stream_field':
        del process['stdout']['maximum_bytes']
    elif case == 'extra_runtime_stream_field':
        process['stdout']['unexpected'] = 0
    else:
        del process['stderr']['retained_bytes']
    _canonical_file(process_path, process)
    _refresh_phase3_positive_component_manifest_from_repository(candidate_root, repository)

    with pytest.raises(EvidenceError, match=r'positive-control .* process'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['missing_log', 'missing_process', 'extra_artifact'])
def test_release_evidence_requires_exact_positive_component_artifact_set(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    if case == 'missing_log':
        (positive_directory / 'processes/partition_cleanup.stdout.log').unlink()
    elif case == 'missing_process':
        (positive_directory / 'processes/partition_cleanup.process.json').unlink()
    else:
        (positive_directory / 'forged-extra.log').write_text('forged\n', encoding='utf-8')
    _refresh_phase3_positive_component_manifest_from_repository(candidate_root, repository)

    with pytest.raises(EvidenceError, match='component artifact path set is not exact'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('attestation_kind', ['contact_gate', 'contact_aggregator'])
def test_release_evidence_rejects_rebound_foreign_runtime_gate_workspace_paths(
    tmp_path: Path,
    attestation_kind: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 foreign runtime-gate path forgery fixture',
    )
    for gate_name in ('runtime-gate.json', 'contact-stream-final-gate.json'):
        gate_path = positive_directory / gate_name
        gate = json.loads(gate_path.read_text(encoding='utf-8'))
        if attestation_kind == 'contact_gate':
            attestation = gate['contact_gate_binary_attestation']
            foreign_path = repository.parent / 'foreign/build/robotest_sim/contact_stream_gate'
            attestation['live_executable_link'] = str(foreign_path)
            attestation['live_executable_path'] = str(foreign_path)
        else:
            attestation = gate['contact_aggregator_binary_attestation']
            foreign_path = (
                repository.parent
                / 'foreign/build/robotest_sim/librobotest_contact_aggregator_system.so'
            )
            attestation['live_mapping_paths'] = [str(foreign_path)]
            stable_identity = {
                field: attestation[field]
                for field in orchestration.CONTACT_AGGREGATOR_STABLE_IDENTITY_FIELDS
            }
            attestation['stable_identity'] = stable_identity
            attestation['stable_identity_sha256'] = orchestration.canonical_sha256(stable_identity)
        _canonical_file(gate_path, gate, sidecar=True)
    _rebind_phase3_positive_runtime_gate(candidate_root, repository)

    with pytest.raises(EvidenceError, match='attestation is not bound to this workspace'):
        _validate_release_fixture(fixture)


def test_release_workspace_gate_binding_accepts_launch_argv_and_multisegment_dso(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    gate_path = (repository / build_binding['contact_gate_binary']['installed_path']).resolve(
        strict=True
    )
    legacy_cmdline_sha256 = hashlib.sha256(f'{gate_path}\0--ros-args\0'.encode()).hexdigest()

    for gate_name in ('runtime-gate.json', 'contact-stream-final-gate.json'):
        gate = json.loads((positive_directory / gate_name).read_text(encoding='utf-8'))
        gate_attestation = gate['contact_gate_binary_attestation']
        aggregator_attestation = gate['contact_aggregator_binary_attestation']
        assert gate_attestation['live_cmdline_sha256'] == (
            _fixture_contact_gate_cmdline_sha256(gate_path)
        )
        assert gate_attestation['live_cmdline_sha256'] != legacy_cmdline_sha256
        assert aggregator_attestation['live_mapping_count'] == 5
        assert len(aggregator_attestation['live_mapping_paths']) == 1
        release_module._validate_positive_gate_workspace_paths(
            gate,
            repository=repository,
            build_binding=build_binding,
            label=f'fixture {gate_name}',
        )


@pytest.mark.parametrize(
    'case',
    [
        'malformed_gate_cmdline_sha256',
        'malformed_mapping_fingerprint_sha256',
        'mapping_count_zero',
        'mapping_count_boolean',
        'mapping_count_non_integer',
        'mapping_count_above_bound',
        'mapping_path_drift',
        'mapping_identity_drift',
        'mapping_fingerprint_stable_mismatch',
    ],
)
def test_release_workspace_gate_binding_rejects_invalid_attestation(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    gate = json.loads(
        (candidate_root / 'positive-control/runtime-gate.json').read_text(encoding='utf-8')
    )
    gate_attestation = gate['contact_gate_binary_attestation']
    aggregator_attestation = gate['contact_aggregator_binary_attestation']
    stable_identity = aggregator_attestation['stable_identity']

    if case == 'malformed_gate_cmdline_sha256':
        gate_attestation['live_cmdline_sha256'] = 'not-a-sha256'
    elif case == 'malformed_mapping_fingerprint_sha256':
        aggregator_attestation['live_mapping_fingerprint_sha256'] = 'not-a-sha256'
        stable_identity['live_mapping_fingerprint_sha256'] = 'not-a-sha256'
    elif case == 'mapping_count_zero':
        aggregator_attestation['live_mapping_count'] = 0
    elif case == 'mapping_count_boolean':
        aggregator_attestation['live_mapping_count'] = True
    elif case == 'mapping_count_non_integer':
        aggregator_attestation['live_mapping_count'] = '5'
    elif case == 'mapping_count_above_bound':
        aggregator_attestation['live_mapping_count'] = 65
    elif case == 'mapping_path_drift':
        foreign_path = repository.parent / 'foreign/librobotest_contact_aggregator_system.so'
        aggregator_attestation['live_mapping_paths'] = [str(foreign_path)]
        stable_identity['live_mapping_paths'] = [str(foreign_path)]
    elif case == 'mapping_identity_drift':
        forged_inode = int(aggregator_attestation['installed_inode']) + 1
        aggregator_attestation['installed_inode'] = forged_inode
        aggregator_attestation['live_mapping_inode'] = forged_inode
        stable_identity['installed_inode'] = forged_inode
        stable_identity['live_mapping_inode'] = forged_inode
    else:
        stable_identity['live_mapping_fingerprint_sha256'] = 'f' * 64

    with pytest.raises(EvidenceError, match='attestation is not bound to this workspace'):
        release_module._validate_positive_gate_workspace_paths(
            gate,
            repository=repository,
            build_binding=build_binding,
            label='forged fixture gate',
        )


@pytest.mark.parametrize(
    ('case', 'message'),
    [
        ('gate_cmdline', 'contact gate process/binary identity changed before final drain'),
        (
            'mapping_fingerprint',
            'contact aggregator process/DSO identity changed before final drain',
        ),
    ],
)
def test_phase3_reobservation_rejects_valid_digest_identity_drift(
    tmp_path: Path,
    case: str,
    message: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    candidate_root = Path(fixture['candidate_root'])
    positive_directory = candidate_root / 'positive-control'
    final_gate_path = positive_directory / 'contact-stream-final-gate.json'
    final_gate = json.loads(final_gate_path.read_text(encoding='utf-8'))
    orchestration = release_module._load_repository_module(
        repository,
        'tests/phase3_orchestration.py',
        'Phase 3 gate digest drift fixture',
    )

    if case == 'gate_cmdline':
        final_gate['contact_gate_binary_attestation']['live_cmdline_sha256'] = 'f' * 64
    else:
        aggregator_attestation = final_gate['contact_aggregator_binary_attestation']
        aggregator_attestation['live_mapping_fingerprint_sha256'] = 'f' * 64
        aggregator_attestation['stable_identity']['live_mapping_fingerprint_sha256'] = 'f' * 64
        aggregator_attestation['stable_identity_sha256'] = orchestration.canonical_sha256(
            aggregator_attestation['stable_identity']
        )
    _canonical_file(final_gate_path, final_gate, sidecar=True)
    build_binding = json.loads((candidate_root / 'build-binding.json').read_text(encoding='utf-8'))
    plan = json.loads((candidate_root / 'suite-plan.json').read_text(encoding='utf-8'))

    with pytest.raises(orchestration.EvidenceError, match=message):
        orchestration.reconcile_contact_gate_reobservation(
            positive_directory / 'runtime-gate.json',
            final_gate_path,
            build_binding=build_binding,
            expected_domain_id=plan['positive_control']['ros_domain_id'],
            expected_gz_partition=plan['positive_control']['gz_partition'],
        )


@pytest.mark.parametrize('field', ['source', 'install', 'source_install'])
def test_release_evidence_rejects_incomplete_phase3_build_binding(
    tmp_path: Path, field: str
) -> None:
    fixture = _release_fixture(tmp_path)
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    del binding[field]
    _canonical_file(binding_path, binding, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate build_binding differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_candidate_inputs(fixture)
    with pytest.raises(EvidenceError, match='build binding producer or schema changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_empty_phase3_build_manifest(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['source'] = {
        'aggregate_sha256': release_module._canonical_sha256([]),
        'file_count': 0,
        'files': [],
        'total_bytes': 0,
    }
    _canonical_file(binding_path, binding, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate build_binding differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_candidate_inputs(fixture)
    with pytest.raises(EvidenceError, match='source tree manifest is empty'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_shallow_phase3_positive_binding(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    binding_path = candidate_root / 'positive-control/positive-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    del binding['capture_sha256']
    _canonical_file(binding_path, binding, sidecar=True)
    marker_path = candidate_root / 'positive-control/PASS.json'
    marker = json.loads(marker_path.read_text(encoding='utf-8'))
    marker['positive_binding_sha256'] = phase5_module.file_sha256(binding_path)
    _canonical_file(marker_path, marker, sidecar=True)

    with pytest.raises(EvidenceError, match='positive-control binding contract changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase3_plan_partition_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    candidate_root = Path(fixture['candidate_root'])
    plan_path = candidate_root / 'suite-plan.json'
    plan = json.loads(plan_path.read_text(encoding='utf-8'))
    plan['trials'][0]['gz_partition'] = 'robotest_p3_forged_00'
    _canonical_file(plan_path, plan, sidecar=True)
    prepared_path = candidate_root / 'prepared.json'
    prepared = json.loads(prepared_path.read_text(encoding='utf-8'))
    prepared['suite_plan_sha256'] = phase5_module.file_sha256(plan_path)
    _canonical_file(prepared_path, prepared, sidecar=True)

    with pytest.raises(
        EvidenceError,
        match='smoke host profile failed validation: profile candidate suite_plan differs',
    ):
        _validate_release_fixture(fixture)
    _rebind_phase3_smoke_profile_candidate_inputs(fixture)
    with pytest.raises(EvidenceError, match='trial order changed'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('field', 'value'),
    [('schema_version', 2), ('producer', 'forged/acceptance_verifier')],
)
def test_release_evidence_rejects_phase4_producer_schema_forgery(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result[field] = value
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='producer schema changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_incomplete_phase4_check_set(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    del result['quality']['checks']['cleanup_complete_and_owned']
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)
    with pytest.raises(EvidenceError, match='check set is incomplete or failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_unexpected_phase4_check(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['quality']['checks']['forged_check'] = True
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='check set is incomplete or failed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_incomplete_phase4_raw_hashes(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    del result['quality']['raw_evidence_sha256']['timeline.jsonl']
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)
    with pytest.raises(EvidenceError, match='raw evidence hash coverage is not exact'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase4_csv_measurement_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    csv_path = run_directory / 'scenario6-result.csv'
    with csv_path.open(encoding='utf-8', newline='') as source:
        rows = list(csv.DictReader(source))
        fieldnames = list(rows[0])
    rows[0]['actual_restart_backoff_wall_s'] = '9.0'
    with csv_path.open('w', encoding='utf-8', newline='') as target:
        writer = csv.DictWriter(target, fieldnames=fieldnames, lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    _refresh_phase4_manifest(run_directory)
    with pytest.raises(EvidenceError, match='Scenario 6 CSV differs from JSON'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_self_consistent_phase4_measurement_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['measurements']['ready_503_after_injection_wall_s'] = True
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='measurements violate frozen bounds'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('replacement_child_start_count', True),
        ('restart_scheduled_count', True),
        ('interrupted_mission_exit_code', True),
        ('followup_mission_exit_code', False),
        ('actual_restart_backoff_wall_s', True),
    ],
)
def test_release_evidence_rejects_boolean_phase4_measurements(
    tmp_path: Path, field: str, value: bool
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['measurements'][field] = value
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='measurements violate frozen bounds'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase4_target(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['targets']['restart_backoff_wall_s'][0] = True
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='frozen targets changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase4_quality_count(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['quality']['event_trace_dropped'] = False
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='quality counters are invalid'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase4_event_meta_count(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    meta_path = run_directory / 'supervisor-events.meta.json'
    meta = json.loads(meta_path.read_text(encoding='utf-8'))
    meta['dropped_events'] = False
    _canonical_file(meta_path, meta)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='raw evidence counters do not reconcile'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase4_verdict_count(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    result_path = Path(fixture['scenario6'])
    result = json.loads(result_path.read_text(encoding='utf-8'))
    result['verdict']['failure_count'] = False
    _canonical_file(result_path, result)
    (run_directory / 'scenario6-result.csv').write_bytes(release_module._phase4_csv_bytes(result))
    _refresh_phase4_manifest(run_directory)

    with pytest.raises(EvidenceError, match='verdict is not canonical PASS'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_self_consistent_phase4_context_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    context_path = run_directory / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    context['isolation']['domain_was_unused'] = False
    _canonical_file(context_path, context)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='context paths or isolation changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase4_raw_forgery(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    timeline_path = run_directory / 'timeline.jsonl'
    timeline = [json.loads(line) for line in timeline_path.read_text(encoding='utf-8').splitlines()]
    unavailable = next(item for item in timeline if item['kind'] == 'ready_unavailable')
    unavailable['details']['http_status'] = 200
    timeline_path.write_text(
        ''.join(
            json.dumps(item, ensure_ascii=False, separators=(',', ':'), sort_keys=True) + '\n'
            for item in timeline
        ),
        encoding='utf-8',
    )
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase4_context_schema(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    context_path = run_directory / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    context['schema_version'] = True
    _canonical_file(context_path, context)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='clean exact candidate'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'name',
    (
        'lifecycle-startup-result.json',
        'package-lifecycle.json',
        'runtime-affinity-initial.json',
        'runtime-affinity-restored.json',
        'supervisor-config-check.txt',
    ),
)
def test_release_evidence_rejects_missing_phase4_producer_raw_file(
    tmp_path: Path,
    name: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    (run_directory / name).unlink()
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='raw evidence hash coverage is not exact'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase4_out_of_cpuset_process(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    affinity_path = run_directory / 'runtime-affinity-initial.json'
    affinity = json.loads(affinity_path.read_text(encoding='utf-8'))
    affinity['processes'][0]['cpus_allowed'] = '0000007f'
    affinity['processes'][0]['cpus_allowed_list'] = '0-6'
    _canonical_file(affinity_path, affinity)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='CPU mask is invalid'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('name', 'field'),
    (
        ('runtime-affinity-initial.json', 'main_pid'),
        ('runtime-affinity-restored.json', 'managed_child_pgid'),
    ),
)
def test_release_evidence_rejects_rebound_phase4_affinity_identity(
    tmp_path: Path,
    name: str,
    field: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    affinity_path = run_directory / name
    affinity = json.loads(affinity_path.read_text(encoding='utf-8'))
    affinity[field] += 10_000
    _canonical_file(affinity_path, affinity)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='identity or unit affinity changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase4_lifecycle_startup_failure(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    lifecycle_path = run_directory / 'lifecycle-startup-result.json'
    lifecycle = json.loads(lifecycle_path.read_text(encoding='utf-8'))
    lifecycle['accepted'] = False
    _canonical_file(lifecycle_path, lifecycle)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='lifecycle startup result is not canonical PASS'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    'forgery',
    (
        'restart_value',
        'restart_boolean',
        'timing_value',
        'timing_boolean',
        'event_cap_value',
        'event_cap_boolean',
        'extra_key',
        'missing_key',
        'child_argv',
        'child_environment_extra',
        'child_required_integer',
    ),
)
def test_release_evidence_rejects_rebound_phase4_supervisor_config_forgery(
    tmp_path: Path,
    forgery: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    config_path = run_directory / 'supervisor-config.json'
    config_check_path = run_directory / 'supervisor-config-check.txt'
    config = json.loads(config_path.read_text(encoding='utf-8'))
    child = config['children'][0]
    if forgery == 'restart_value':
        config['restart']['initial_backoff_ms'] = 1001
    elif forgery == 'restart_boolean':
        config['restart']['maximum_attempts'] = True
    elif forgery == 'timing_value':
        config['heartbeat_startup_timeout_ms'] = 109999
    elif forgery == 'timing_boolean':
        config['heartbeat_poll_ms'] = True
    elif forgery == 'event_cap_value':
        config['maximum_event_bytes'] = 8388609
    elif forgery == 'event_cap_boolean':
        config['maximum_event_entries'] = True
    elif forgery == 'extra_key':
        config['forged'] = 'retained-evidence-bypass'
    elif forgery == 'missing_key':
        del config['shutdown_timeout_ms']
    elif forgery == 'child_argv':
        child['argv'].append('--forged')
    elif forgery == 'child_environment_extra':
        child['environment']['FORGED'] = '1'
    else:
        assert forgery == 'child_required_integer'
        child['required'] = 1
    _canonical_file(config_path, config)
    _refresh_phase4_result(run_directory)

    assert config_check_path.read_bytes() == b'configuration valid\n'
    with pytest.raises(EvidenceError, match='run-scoped supervisor config is not exact'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_rebound_phase4_config_check_forgery(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    (run_directory / 'supervisor-config-check.txt').write_text(
        'configuration forged\n',
        encoding='utf-8',
    )
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='config check did not pass exactly'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase4_upgrade_version_before_replay(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_directory = Path(fixture['phase4_run'])
    hostile_root = repository / 'artifacts/phase4-hostile-packages'
    hostile_root.mkdir(parents=True)
    phase4_fixture = release_module._load_repository_module(
        repository,
        'tests/phase4_acceptance_test.py',
        'Phase 4 hostile package fixture',
    )
    downgrade = phase4_fixture._build_test_package(
        hostile_root,
        'robotest-supervisor_0.1.0_amd64.deb',
        '0.1.0',
        '127.0.0.1:9080',
    )
    context_path = run_directory / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    context['upgrade_package'] = {
        'path': str(downgrade),
        'sha256': phase5_module.file_sha256(downgrade),
    }
    _canonical_file(context_path, context)
    _refresh_phase4_result(run_directory)

    with pytest.raises(
        EvidenceError,
        match=(
            'Phase 4 production replay failed: selected upgrade version does not match '
            'the repository Debian changelog'
        ),
    ):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_phase4_decoy_candidate_manifest(
    tmp_path: Path,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_directory = Path(fixture['phase4_run'])
    context = json.loads((run_directory / 'context.json').read_text(encoding='utf-8'))
    canonical_manifest = Path(context['package_directory']) / 'build-a/SOURCE-MANIFEST.json'
    decoy_manifest = repository / 'artifacts/phase4-decoy/SOURCE-MANIFEST.json'
    decoy_manifest.parent.mkdir(parents=True)
    shutil.copy2(canonical_manifest, decoy_manifest)
    phase4 = release_module._load_repository_module(
        repository,
        'tests/phase4_acceptance.py',
        'Phase 4 hostile package-binding replay',
    )
    forged_binding = phase4.package_source_binding(repository, decoy_manifest)
    assert forged_binding['verdict'] == 'PASS'
    _canonical_file(run_directory / 'package-binding.json', forged_binding)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('forgery', ('stored_field', 'extra_key'))
def test_release_evidence_rejects_phase4_forged_stored_package_binding(
    tmp_path: Path,
    forgery: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    binding_path = run_directory / 'package-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    if forgery == 'stored_field':
        binding['file_count'] += 1
    else:
        assert forgery == 'extra_key'
        binding['forged'] = 'retained-evidence-bypass'
    _canonical_file(binding_path, binding)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize(
    ('manifest_name', 'forgery'),
    (
        ('overlay-source-manifest.json', 'empty'),
        ('overlay-source-manifest.json', 'noncanonical'),
        ('overlay-install-manifest.json', 'empty'),
        ('overlay-install-manifest.json', 'noncanonical'),
    ),
)
def test_release_evidence_rejects_phase4_overlay_manifest_forgery(
    tmp_path: Path,
    manifest_name: str,
    forgery: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    manifest_path = run_directory / manifest_name
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    if forgery == 'empty':
        manifest['files'] = []
        _canonical_file(manifest_path, manifest)
    else:
        assert forgery == 'noncanonical'
        manifest_path.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
            encoding='utf-8',
        )
    _refresh_phase4_overlay_manifest_hashes(run_directory)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('forgery', ('minimal', 'wrong_source_workspace'))
def test_release_evidence_rejects_phase4_overlay_provenance_forgery(
    tmp_path: Path,
    forgery: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    staging_path = run_directory / 'runtime-staging.json'
    staging = json.loads(staging_path.read_text(encoding='utf-8'))
    if forgery == 'minimal':
        staging = {'schema_version': 1}
    else:
        assert forgery == 'wrong_source_workspace'
        staging['source_workspace'] = str(tmp_path / 'decoy-workspace')
    _canonical_file(staging_path, staging)
    _canonical_file(run_directory / 'overlay-provenance.json', staging)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('forgery', ('wrong_parent', 'wrong_release_id'))
def test_release_evidence_rejects_phase4_active_overlay_identity_forgery(
    tmp_path: Path,
    forgery: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    context_path = run_directory / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    staging_path = run_directory / 'runtime-staging.json'
    staging = json.loads(staging_path.read_text(encoding='utf-8'))
    if forgery == 'wrong_parent':
        context['active_overlay_target'] = f'/opt/robotest-releases/{staging["release_id"]}'
    else:
        assert forgery == 'wrong_release_id'
        old_release_id = staging['release_id']
        forged_release_id = f'forged-{old_release_id}'
        staging['release_id'] = forged_release_id
        staging['build_command'] = [
            value.replace(old_release_id, forged_release_id) for value in staging['build_command']
        ]
        _canonical_file(staging_path, staging)
        _canonical_file(run_directory / 'overlay-provenance.json', staging)
        context['active_overlay_target'] = f'/opt/robotest-lab-releases/{forged_release_id}'
    _canonical_file(context_path, context)
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='exact production evaluation replay'):
        _validate_release_fixture(fixture)
