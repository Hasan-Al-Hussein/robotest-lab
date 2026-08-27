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
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest
import yaml

TESTS = Path(__file__).resolve().parent
REPOSITORY = TESTS.parent
sys.path.insert(0, str(TESTS))

import phase5_ci as phase5_module  # noqa: E402
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

_RELEASE_FIXTURE_TEMPLATE_DIRECTORY: tempfile.TemporaryDirectory | None = None
_RELEASE_FIXTURE_TEMPLATE: dict[str, Path | str] | None = None
_RELEASE_FIXTURE_LAST_REPOSITORY: Path | None = None


def test_phase5_docs_freeze_clean_candidate_release_order() -> None:
    document = (REPOSITORY / 'docs/testing/phase5-ci.md').read_text(encoding='utf-8')
    ordered_steps = (
        'Create and push the clean candidate commit C.',
        'run the separately authorized Phase 3 campaign and\n   Phase 4 acceptance workflow',
        'run bare `scripts/verify_all.sh`',
        'capture its candidate remote proof',
        'create the exact evidence-only child commit E',
        "Capture E's successful workflow",
        'Run `scripts/verify_all.sh --release-evidence`',
    )
    offsets = [document.index(step) for step in ordered_steps]
    assert offsets == sorted(offsets)
    assert 'Do not rerun a clean-start live or bare gate after this point.' in document


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
    assert dependency_inventory['apt_package_count'] == 42
    assert dependency_inventory['github_action_count'] == 3
    assert dependency_inventory['python_distribution_count'] == 1
    assert dependency_inventory['ros_dependency_count'] == 61
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
    assert report['claim_count'] == 12
    assert report['evidence_file_count'] == 2


def test_release_claim_audit_rejects_missing_evidence_text(tmp_path: Path) -> None:
    repository = tmp_path / 'repository'
    for relative in (
        'README.md',
        'config/release-claims.json',
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
) -> dict[str, object]:
    return {
        'conclusion': conclusion,
        'createdAt': f'2026-08-26T00:00:{run_id:02d}Z',
        'databaseId': run_id,
        'headSha': sha,
        'status': status,
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
    assert 'run_check pure-python-tests 600s python3 -m pytest' in script_text
    result = subprocess.run(['bash', str(script)], capture_output=True, text=True, check=False)
    assert result.returncode == 2
    assert 'There is deliberately no implicit mode.' in result.stderr
    help_result = subprocess.run(
        ['bash', str(script), '--help'], capture_output=True, text=True, check=False
    )
    assert help_result.returncode == 0
    assert 'never start Gazebo' in help_result.stdout


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
    result = subprocess.run(
        ['bash', str(script), '--local'],
        cwd=repository,
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


def _canonical_file(path: Path, value: object, *, sidecar: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(phase5_module.canonical_json_bytes(value))
    if sidecar:
        Path(f'{path}.sha256').write_text(
            f'{phase5_module.file_sha256(path)}  {path.name}\n',
            encoding='ascii',
        )


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
        '/clock': {'/robotest/metrics_collector'},
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
    return {
        'command_subscriber_ownership': command_subscriber_ownership,
        'contact_subscriber_ownership': contact_subscriber_ownership,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in topics.items()
        },
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


def _write_bounded_process_artifacts(directory: Path, process: dict) -> None:
    """Write one production-shaped process record and its retained log files."""
    role = process['role']
    process_directory = directory / 'processes'
    _canonical_file(process_directory / f'{role}.process.json', process)
    for stream_name in ('stdout', 'stderr'):
        stream = process[stream_name]
        assert stream['retained_bytes'] == 0
        (process_directory / f'{role}.{stream_name}.log').write_bytes(b'')


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
        'live_cmdline_sha256': hashlib.sha256(
            f'{installed_path}\0--ros-args\0'.encode()
        ).hexdigest(),
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
        'live_mapping_count': len(live_mapping_paths),
        'live_mapping_device': aggregator_installed_stat.st_dev,
        'live_mapping_fingerprint_sha256': hashlib.sha256(
            (
                f'{aggregator_installed_stat.st_dev}:'
                f'{aggregator_installed_stat.st_ino}:'
                f'{aggregator_installed_stat.st_size}:'
                f'{aggregator_installed_path}'
            ).encode()
        ).hexdigest(),
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
    if mission_client:
        services.update(
            {
                '/robotest/mission_runner/describe_parameters': [
                    'rcl_interfaces/srv/DescribeParameters'
                ],
                '/robotest/mission_runner/get_parameter_types': [
                    'rcl_interfaces/srv/GetParameterTypes'
                ],
                '/robotest/mission_runner/get_parameters': ['rcl_interfaces/srv/GetParameters'],
                '/robotest/mission_runner/list_parameters': ['rcl_interfaces/srv/ListParameters'],
                '/robotest/mission_runner/set_parameters': ['rcl_interfaces/srv/SetParameters'],
                '/robotest/mission_runner/set_parameters_atomically': [
                    'rcl_interfaces/srv/SetParametersAtomically'
                ],
            }
        )
    actions = {name: [type_name] for name, type_name in contracts['actions'].items()}
    node_names = sorted(
        {
            *persistent_nodes,
            *passive_clients,
            orchestration.PHASE3_GRAPH_ACTION_SERVER_NODE,
            *([orchestration.PHASE3_GRAPH_MISSION_CLIENT_NODE] if mission_client else []),
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
        'attempt_count': 2,
        'contracts': contracts,
        'duplicate_node_names': [],
        'elapsed_wall_seconds': 0.25,
        'failure': None,
        'failure_kind': None,
        'limits': {
            'maximum_graph_names': orchestration.PHASE3_GRAPH_MAXIMUM_GRAPH_NAMES,
            'maximum_graph_nodes': orchestration.PHASE3_GRAPH_MAXIMUM_GRAPH_NODES,
            'maximum_types_per_name': orchestration.PHASE3_GRAPH_MAXIMUM_TYPES_PER_NAME,
            'wall_timeout_seconds': orchestration.PHASE3_GRAPH_WALL_TIMEOUT_S,
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


def _write_phase3_graph_prerequisites(
    repository: Path,
    run_root: Path,
    *,
    orchestration: object,
    plan: dict,
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
        'domain_preflight': (100_000, 200_000),
        'partition_preflight': (300_000, 400_000),
        'full_stack': (500_000, 17_000_000),
        'startup_gate': (600_000, 700_000),
        'lifecycle_gate': (800_000, 900_000),
        'metrics_collector': (1_000_000, 15_000_000),
        'scenario_controller': (1_100_000, 10_000_000),
        'goal_observer': (1_200_000, 2_500_000),
        'runtime_gate': (1_300_000, 1_400_000),
        'graph_gate': (1_500_000, 1_600_000),
        'mission_runner': (2_000_000, 9_000_000),
        'lifecycle_sampler': (2_600_000, 16_000_000),
        'mission_graph_gate': (3_000_000, 3_100_000),
        'contact_drain': (11_000_000, 12_000_000),
        'contact_stream_final_gate': (13_000_000, 14_000_000),
        'domain_cleanup': (18_000_000, 18_500_000),
        'partition_cleanup': (19_000_000, 19_500_000),
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
        )
    pre_binding = _write_phase3_graph_evidence(
        run_root,
        orchestration=orchestration,
        mission_client=False,
        persistent_nodes=persistent_nodes,
        watch_pid=pre_watch_pid,
    )
    mission_binding = _write_phase3_graph_evidence(
        run_root,
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
            {'x_m': 1.0, 'y_m': 0.0},
            {'x_m': 2.0, 'y_m': 0.0},
            {'x_m': 3.0, 'y_m': 0.0},
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
        node_name: {'label': 'active', 'state_id': 3}
        for node_name in orchestration.REQUIRED_LIFECYCLE_NODES
    }
    _canonical_file(
        run_root / 'lifecycle-ready.json',
        {'states': lifecycle_states, 'verdict': 'PASS'},
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
    )
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

    result = analyze_run(analysis_request)
    assert result['verdict']['automated_status'] == 'PASS', {
        'quality': result['quality'],
        'verdict': result['verdict'],
    }
    result_path = directory / 'run-result.json'
    _canonical_file(result_path, result)
    (directory / 'run-result.csv').write_bytes(release_module._one_row_csv_bytes(result))
    (directory / 'report.md').write_text('PASS\n', encoding='utf-8')
    (directory / 'report.html').write_text('<p>PASS</p>\n', encoding='utf-8')
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
    result_path = directory / 'run-result.json'
    result = json.loads(result_path.read_text(encoding='utf-8'))
    _canonical_file(result_path, result)
    (directory / 'run-result.csv').write_bytes(release_module._one_row_csv_bytes(result))
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
        'gz_partition': f'robotest_p3_{suite_plan["candidate_id"]}-smoke_00',
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


def _build_release_fixture(tmp_path: Path) -> dict[str, Path | str]:
    repository = tmp_path / 'repository'
    repository.mkdir()
    shutil.copy2(REPOSITORY / '.gitignore', repository / '.gitignore')
    ignored = shutil.ignore_patterns('__pycache__', '.pytest_cache', '*.pyc')
    for directory in ('config', 'docs', 'packaging', 'scenarios', 'scripts', 'supervisor', 'tests'):
        shutil.copytree(REPOSITORY / directory, repository / directory, ignore=ignored)
    for package in release_module.PHASE3_RUNTIME_PACKAGES:
        shutil.copytree(
            REPOSITORY / f'src/{package}',
            repository / f'src/{package}',
            ignore=ignored,
        )
        shutil.copytree(
            REPOSITORY / f'install/{package}',
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
            shutil.copytree(python_source, site_packages / package)
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
                'gz_partition': f'robotest_p3_{candidate_id}_smoke',
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
    for name in ('runtime-staging.json', 'overlay-provenance.json'):
        path = phase4_run / name
        document = json.loads(path.read_text(encoding='utf-8'))
        document['git_commit'] = candidate_sha
        document['git_dirty'] = False
        _canonical_file(path, document)
    phase4_module = release_module._load_repository_module(
        repository,
        'tests/phase4_acceptance.py',
        'Phase 4 production acceptance module',
    )
    package_manifest = phase4_run / 'packages/build-a/SOURCE-MANIFEST.json'
    _canonical_file(
        phase4_run / 'package-binding.json',
        phase4_module.package_source_binding(repository, package_manifest),
    )
    original_utc_now = phase4_module.utc_now
    phase4_module.utc_now = lambda: '2026-08-26T00:01:00.000000Z'
    try:
        scenario6 = phase4_module.write_result(phase4_run)
    finally:
        phase4_module.utc_now = original_utc_now
    assert scenario6['verdict']['status'] == 'PASS', scenario6['verdict']
    phase4_module.write_checksums(phase4_run)
    scenario6_path = phase4_run / 'scenario6-result.json'

    remote_root = repository / 'docs/results/phase-5'
    remote_path = remote_root / f'remote-{candidate_sha}.json'
    _canonical_file(
        remote_path,
        _remote_proof(
            repository,
            sha=candidate_sha,
            mode='--remote',
            run_id=42,
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
    release_docs_module.write_release_documents(repository, aggregate_path, scenario6_path)
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
        'remote': remote_path,
        'repository': repository,
        'scenario6': scenario6_path,
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
            'live_cmdline_sha256': hashlib.sha256(
                f'{gate_path}\0--ros-args\0'.encode()
            ).hexdigest(),
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
    mapping_fingerprint = hashlib.sha256(
        (
            f'{aggregator_stat.st_dev}:{aggregator_stat.st_ino}:'
            f'{aggregator_stat.st_size}:{aggregator_path}'
        ).encode()
    ).hexdigest()
    aggregator_attestation = gate['contact_aggregator_binary_attestation']
    aggregator_attestation.update(
        {
            'installed_device': aggregator_stat.st_dev,
            'installed_inode': aggregator_stat.st_ino,
            'installed_size_bytes': aggregator_stat.st_size,
            'live_mapping_count': 1,
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
        'gz_partition': f'robotest_p3_{suite_plan["candidate_id"]}-smoke_00',
        'ros_domain_id': smoke['ros_domain_id'],
        'run_id': smoke['run_id'],
    }
    _, smoke_sha = replay(smoke_plan, candidate_root / 'smoke')
    smoke_marker_path = candidate_root / 'smoke/PASS.json'
    smoke_marker = json.loads(smoke_marker_path.read_text(encoding='utf-8'))
    smoke_marker['run_result_sha256'] = smoke_sha
    _canonical_file(smoke_marker_path, smoke_marker, sidecar=True)

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
    context = _relocate_json(context_path, old_root, str(repository))
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
    phase4 = release_module._load_repository_module(
        repository,
        'tests/phase4_acceptance.py',
        'Phase 4 production acceptance fixture clone',
    )
    package_directory = Path(context['package_directory'])
    upgrade_package = Path(context['upgrade_package']['path'])
    baseline_package = Path(context['baseline_package']['path'])
    _canonical_file(
        run_directory / 'package-integrity.json',
        phase4.verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
        ),
    )
    package_manifest = package_directory / 'build-a/SOURCE-MANIFEST.json'
    _canonical_file(
        run_directory / 'package-binding.json',
        phase4.package_source_binding(repository, package_manifest),
    )
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

    local_aggregate = repository / 'artifacts/evidence/phase0/verify-all.json'
    _relocate_json(local_aggregate, old_root, str(repository))
    local_aggregate.with_suffix('.SHA256SUMS').write_text(
        f'{phase5_module.file_sha256(local_aggregate)}  {local_aggregate.name}\n',
        encoding='ascii',
    )
    local_aggregate.with_suffix('.checksum-validation.txt').write_text(
        f'{local_aggregate.name}: OK\n',
        encoding='utf-8',
    )

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
    remote_path = repository / f'docs/results/phase-5/remote-{candidate_sha}.json'
    _refresh_remote_proof(
        remote_path,
        _remote_proof(
            repository,
            sha=candidate_sha,
            mode='--remote',
            run_id=42,
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
    ):
        path.unlink(missing_ok=True)
    release_docs_module.write_release_documents(repository, aggregate_path, scenario6_path)
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
            created_at='2026-08-26T00:10:00Z',
            checked_at='2026-08-26T00:11:00+00:00',
        ),
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
        'remote': remote_path,
        'repository': repository,
        'scenario6': scenario6_path,
    }


def _validate_release_fixture(fixture: dict[str, Path | str]) -> dict[str, object]:
    return validate_release_evidence(
        Path(fixture['repository']),
        Path(fixture['local_aggregate']),
        Path(fixture['candidate_root']),
        Path(fixture['aggregate']),
        Path(fixture['phase4_run']),
        Path(fixture['scenario6']),
        Path(fixture['remote']),
        Path(fixture['evidence_remote']),
    )


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
        '--phase5-remote-proof',
        str(fixture['remote']),
        '--phase5-evidence-commit-remote-proof',
        str(fixture['evidence_remote']),
    ]


def test_release_evidence_mode_passes_only_exact_selected_artifacts(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    report = _validate_release_fixture(fixture)
    assert report['status'] == 'PASS'
    assert report['release_eligible'] is True
    assert report['candidate_git_sha'] == fixture['candidate_sha']
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
    assert release_result['release_eligible'] is True
    assert release_result['candidate_git_sha'] == fixture['candidate_sha']
    assert not Path(fixture['call_log']).exists()


def test_release_fixture_clones_are_self_contained_and_reload_producers(
    tmp_path: Path,
) -> None:
    first = _release_fixture(tmp_path / 'first')
    first_repository = Path(first['repository'])
    assert not any(path.is_symlink() for path in (first_repository / 'install').rglob('*'))
    release_module._activate_repository_packages(first_repository)
    first_analysis = importlib.import_module('robotest_metrics.analysis')
    first_scenario_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert Path(first_analysis.__file__).resolve().is_relative_to(first_repository)
    assert Path(first_scenario_provenance.__file__).resolve().is_relative_to(first_repository)

    second = _release_fixture(tmp_path / 'second')
    second_repository = Path(second['repository'])
    release_module._activate_repository_packages(second_repository)
    second_analysis = importlib.import_module('robotest_metrics.analysis')
    second_scenario_provenance = importlib.import_module('robotest_scenarios.provenance')
    assert second_analysis is not first_analysis
    assert second_scenario_provenance is not first_scenario_provenance
    assert Path(second_analysis.__file__).resolve().is_relative_to(second_repository)
    assert Path(second_scenario_provenance.__file__).resolve().is_relative_to(second_repository)

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
    assert report['claim_count'] == 15
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
    run_root = Path(fixture['candidate_root']) / 'smoke'
    process_path = run_root / f'processes/{role}.process.json'
    process = json.loads(process_path.read_text(encoding='utf-8'))
    if case == 'command':
        command = list(process['command'])
        command[0] = 'forged-executable'
        _rewrite_process_command(process, command)
    else:
        process['wall_timeout_s'] += 1.0
        process['wrapped_command'][3] = f'{process["wall_timeout_s"]:.3f}s'
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

    with pytest.raises(EvidenceError, match=rf'{role} process command, timeout, or outcome'):
        _validate_release_fixture(fixture)


@pytest.mark.parametrize('case', ['mission_before_graph', 'stack_ends_before_final_gate'])
def test_release_evidence_rejects_rebound_phase3_process_timeline_forgery(
    tmp_path: Path,
    case: str,
) -> None:
    fixture = _release_fixture(tmp_path)
    repository = Path(fixture['repository'])
    run_root = Path(fixture['candidate_root']) / 'smoke'
    if case == 'mission_before_graph':
        process_path = run_root / 'processes/mission_runner.process.json'
        process = json.loads(process_path.read_text(encoding='utf-8'))
        process['started_steady_ns'] = 1_550_000
    else:
        process_path = run_root / 'processes/full_stack.process.json'
        process = json.loads(process_path.read_text(encoding='utf-8'))
        process['finished_steady_ns'] = 13_500_000
    _canonical_file(process_path, process)
    _rebind_phase3_prerequisites(repository, run_root)

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


@pytest.mark.parametrize('omitted_path', ['resources.jsonl', 'lifecycle-ready-map_server.txt'])
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

    with pytest.raises(EvidenceError, match='node snapshot differs from the validated graph'):
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
    subscriber = clock['subscribers'][0]
    previous_node = subscriber['node']
    subscriber['node'] = '/robotest/forged_unknown_node'
    subscriber_check = next(
        check
        for check in clock['qos_checks']
        if check['side'] == 'subscriber' and check['node'] == previous_node
    )
    subscriber_check['node'] = subscriber['node']
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
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    aggregator = binding['contact_aggregator_binary']
    rebound_inventory_sha256 = 'f' * 64
    aggregator['source_inventory_sha256'] = rebound_inventory_sha256
    aggregator['build_embedded_source_inventory_sha256'] = rebound_inventory_sha256
    aggregator['installed_embedded_source_inventory_sha256'] = rebound_inventory_sha256
    _canonical_file(binding_path, binding, sidecar=True)

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

    with pytest.raises(EvidenceError, match='suite plan contract changed'):
        _validate_release_fixture(fixture)


def test_release_evidence_rejects_boolean_phase3_manifest_count(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    binding['source']['file_count'] = True
    _canonical_file(binding_path, binding, sidecar=True)

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
    if case == 'missing_driver_cwd':
        process_path = process_directory / 'contact_control_driver.process.json'
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
            attestation['live_cmdline_sha256'] = hashlib.sha256(
                f'{foreign_path}\0--ros-args\0'.encode()
            ).hexdigest()
        else:
            attestation = gate['contact_aggregator_binary_attestation']
            foreign_path = (
                repository.parent
                / 'foreign/build/robotest_sim/librobotest_contact_aggregator_system.so'
            )
            attestation['live_mapping_paths'] = [str(foreign_path)]
            attestation['live_mapping_fingerprint_sha256'] = hashlib.sha256(
                (
                    f'{attestation["live_mapping_device"]}:'
                    f'{attestation["live_mapping_inode"]}:'
                    f'{attestation["installed_size_bytes"]}:{foreign_path}'
                ).encode()
            ).hexdigest()
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


@pytest.mark.parametrize('field', ['source', 'install', 'source_install'])
def test_release_evidence_rejects_incomplete_phase3_build_binding(
    tmp_path: Path, field: str
) -> None:
    fixture = _release_fixture(tmp_path)
    binding_path = Path(fixture['candidate_root']) / 'build-binding.json'
    binding = json.loads(binding_path.read_text(encoding='utf-8'))
    del binding[field]
    _canonical_file(binding_path, binding, sidecar=True)

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
    unavailable['monotonic_ns'] += 100_000_000
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


def test_release_evidence_rejects_missing_phase4_producer_raw_file(tmp_path: Path) -> None:
    fixture = _release_fixture(tmp_path)
    run_directory = Path(fixture['phase4_run'])
    (run_directory / 'package-lifecycle.json').unlink()
    _refresh_phase4_result(run_directory)

    with pytest.raises(EvidenceError, match='raw evidence hash coverage is not exact'):
        _validate_release_fixture(fixture)
