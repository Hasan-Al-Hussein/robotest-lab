# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Focused pure tests for the Phase 3 contact-aggregator DSO binding."""

from __future__ import annotations

import copy
import importlib.util
import shutil
import sys
from pathlib import Path
from typing import Any

import pytest

MODULE_PATH = Path(__file__).with_name('phase3_orchestration.py')
SPEC = importlib.util.spec_from_file_location(
    'phase3_orchestration_aggregator_binding', MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
orchestration = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = orchestration
SPEC.loader.exec_module(orchestration)

AGGREGATOR_BINDING_FIELDS = set(orchestration.CONTACT_AGGREGATOR_BINARY_FIELDS)


def _write_contact_sources(workspace: Path) -> str:
    for relative in orchestration.CONTACT_GATE_SOURCE_PATHS:
        source = workspace / relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(f'{relative}\n', encoding='utf-8')
    return orchestration.canonical_sha256(orchestration.contact_gate_source_inventory(workspace))


def _write_fake_aggregator_install(
    workspace: Path,
    *,
    symlink_install: bool,
    embedded_inventory_sha256: str | None = None,
) -> tuple[Path, Path, str]:
    source_inventory_sha256 = _write_contact_sources(workspace)
    embedded = embedded_inventory_sha256 or source_inventory_sha256
    build = workspace / orchestration.CONTACT_AGGREGATOR_BUILD_PATH
    installed = workspace / orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH
    build.parent.mkdir(parents=True, exist_ok=True)
    installed.parent.mkdir(parents=True, exist_ok=True)
    build.write_bytes(
        Path(sys.executable).read_bytes()
        + b'\0ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
        + embedded.encode('ascii')
        + b'\0'
    )
    shutil.copymode(sys.executable, build)
    if symlink_install:
        installed.symlink_to(build)
    else:
        shutil.copy2(build, installed)
    return build, installed, source_inventory_sha256


@pytest.mark.parametrize('symlink_install', (False, True))
def test_contact_aggregator_binding_records_declared_and_resolved_install_identity(
    tmp_path: Path,
    symlink_install: bool,
) -> None:
    build, installed, source_inventory_sha256 = _write_fake_aggregator_install(
        tmp_path,
        symlink_install=symlink_install,
    )

    binding = orchestration.contact_aggregator_build_install_binding(tmp_path)

    assert set(binding) == AGGREGATOR_BINDING_FIELDS
    assert binding['schema_version'] == 1
    assert binding['package'] == 'robotest_sim'
    assert binding['build_path'] == orchestration.CONTACT_AGGREGATOR_BUILD_PATH
    assert binding['installed_declared_path'] == orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH
    assert binding['installed_declared_is_symlink'] is symlink_install
    assert binding['installed_path'] == (
        orchestration.CONTACT_AGGREGATOR_BUILD_PATH
        if symlink_install
        else orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH
    )
    assert binding['build_install_samefile'] is symlink_install
    assert binding['build_regular_file'] is True
    assert binding['installed_regular_file'] is True
    assert binding['build_sha256'] == orchestration.file_sha256(build)
    assert binding['installed_sha256'] == orchestration.file_sha256(installed)
    assert binding['build_elf_build_id'] == binding['installed_elf_build_id']
    assert binding['source_inventory_sha256'] == source_inventory_sha256
    for field in (
        'build_embedded_source_inventory_match',
        'build_install_build_id_match',
        'build_install_embedded_source_inventory_match',
        'build_install_sha256_match',
        'installed_embedded_source_inventory_match',
    ):
        assert binding[field] is True


def test_contact_aggregator_binding_rejects_copy_with_different_bytes(tmp_path: Path) -> None:
    _, installed, _ = _write_fake_aggregator_install(tmp_path, symlink_install=False)
    with installed.open('ab') as stream:
        stream.write(b'changed-after-install')

    with pytest.raises(orchestration.EvidenceError, match='hashes differ'):
        orchestration.contact_aggregator_build_install_binding(tmp_path)


def test_contact_aggregator_binding_rejects_nonshared_embedded_inventory(tmp_path: Path) -> None:
    _write_fake_aggregator_install(
        tmp_path,
        symlink_install=True,
        embedded_inventory_sha256='f' * 64,
    )

    with pytest.raises(orchestration.EvidenceError, match='exact shared source inventory'):
        orchestration.contact_aggregator_build_install_binding(tmp_path)


def _aggregator_record(digest: str = 'a' * 64) -> dict[str, Any]:
    return {
        field: True
        for field in AGGREGATOR_BINDING_FIELDS
        if field.endswith('_match') or field.endswith('_file') or field == 'build_install_samefile'
    } | {
        'build_elf_build_id': 'build-id',
        'build_embedded_source_inventory_sha256': digest,
        'build_path': orchestration.CONTACT_AGGREGATOR_BUILD_PATH,
        'build_sha256': digest,
        'installed_declared_is_symlink': True,
        'installed_declared_path': orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH,
        'installed_elf_build_id': 'build-id',
        'installed_embedded_source_inventory_sha256': digest,
        'installed_path': orchestration.CONTACT_AGGREGATOR_BUILD_PATH,
        'installed_sha256': digest,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': digest,
    }


def _runtime_attestation(frozen: dict[str, Any]) -> dict[str, Any]:
    stable_identity = {
        'build_elf_build_id': frozen['build_elf_build_id'],
        'build_path': frozen['build_path'],
        'build_sha256': frozen['build_sha256'],
        'installed_declared_path': frozen['installed_declared_path'],
        'installed_device': 7,
        'installed_elf_build_id': frozen['installed_elf_build_id'],
        'installed_embedded_source_inventory_sha256': frozen[
            'installed_embedded_source_inventory_sha256'
        ],
        'installed_inode': 8,
        'installed_path': frozen['installed_path'],
        'installed_sha256': frozen['installed_sha256'],
        'launch_root_pid': 100,
        'live_cmdline_sha256': 'b' * 64,
        'live_executable_link': '/usr/bin/gz',
        'live_executable_path': '/usr/bin/gz',
        'live_mapping_device': 7,
        'live_mapping_fingerprint_sha256': 'c' * 64,
        'live_mapping_inode': 8,
        'live_mapping_paths': ['/workspace/contact_aggregator.so'],
        'live_pgid': 100,
        'live_pid': 101,
        'live_ppid': 100,
        'live_sid': 100,
        'live_start_ticks': 1234,
        'observed_gz_partition': 'robotest_p3_candidate_trial',
        'observed_ros_domain_id': '100',
        'source_inventory_sha256': frozen['source_inventory_sha256'],
    }
    return {
        **frozen,
        'attestation_method': 'proc_maps_exact_device_inode',
        'exact_live_process_count': 1,
        'identity_revalidated_after_hashing': True,
        'installed_device': 7,
        'installed_identity_revalidated_after_hashing': True,
        'installed_inode': 8,
        'installed_size_bytes': 1_000,
        'launch_root_pid': 100,
        'live_cmdline_sha256': 'b' * 64,
        'live_elf_build_id': frozen['installed_elf_build_id'],
        'live_embedded_source_inventory_match': True,
        'live_embedded_source_inventory_sha256': frozen[
            'installed_embedded_source_inventory_sha256'
        ],
        'live_executable_link': '/usr/bin/gz',
        'live_executable_path': '/usr/bin/gz',
        'live_installed_build_id_match': True,
        'live_installed_inode_match': True,
        'live_installed_sha256_match': True,
        'live_mapping_count': 5,
        'live_mapping_device': 7,
        'live_mapping_fingerprint_sha256': 'c' * 64,
        'live_mapping_has_executable': True,
        'live_mapping_has_offset_zero': True,
        'live_mapping_inode': 8,
        'live_mapping_paths': ['/workspace/contact_aggregator.so'],
        'live_pgid': 100,
        'live_pid': 101,
        'live_ppid': 100,
        'live_sid': 100,
        'live_start_ticks': 1234,
        'maps_revalidated_after_hashing': True,
        'observed_gz_partition': 'robotest_p3_candidate_trial',
        'observed_ros_domain_id': '100',
        'process_identity_match': True,
        'stable_identity': stable_identity,
        'stable_identity_sha256': orchestration.canonical_sha256(stable_identity),
        'verdict': 'PASS',
    }


@pytest.mark.parametrize('symlink_install', (False, True))
def test_runtime_attestation_joins_exact_frozen_dso_and_stable_mapping(
    symlink_install: bool,
) -> None:
    frozen = _aggregator_record()
    if not symlink_install:
        frozen['installed_declared_is_symlink'] = False
        frozen['build_install_samefile'] = False
        frozen['installed_path'] = orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH
    attestation = _runtime_attestation(frozen)

    validated = orchestration._validated_contact_aggregator_attestation(
        attestation,
        frozen_binary=frozen,
        expected_domain_id=100,
        expected_gz_partition='robotest_p3_candidate_trial',
        label='initial',
    )

    assert set(validated) == orchestration.CONTACT_AGGREGATOR_ATTESTATION_FIELDS
    assert validated['build_install_samefile'] is symlink_install

    rebound = copy.deepcopy(attestation)
    rebound['stable_identity']['live_start_ticks'] += 1
    rebound['stable_identity_sha256'] = orchestration.canonical_sha256(rebound['stable_identity'])
    with pytest.raises(orchestration.EvidenceError, match='stable identity conflicts'):
        orchestration._validated_contact_aggregator_attestation(
            rebound,
            frozen_binary=frozen,
            expected_domain_id=100,
            expected_gz_partition='robotest_p3_candidate_trial',
            label='final',
        )


def _gate_binary() -> dict[str, Any]:
    return {
        'build_embedded_source_inventory_match': True,
        'build_embedded_source_inventory_sha256': 'a' * 64,
        'build_elf_build_id': 'gate-build-id',
        'build_install_build_id_match': True,
        'build_install_samefile': False,
        'build_install_sha256_match': True,
        'build_path': 'build/robotest_sim/contact_stream_gate',
        'build_regular_executable': True,
        'build_sha256': 'd' * 64,
        'installed_declared_is_symlink': False,
        'installed_declared_path': 'install/robotest_sim/lib/robotest_sim/contact_stream_gate',
        'installed_declared_samefile': True,
        'installed_embedded_source_inventory_match': True,
        'installed_embedded_source_inventory_sha256': 'a' * 64,
        'installed_elf_build_id': 'gate-build-id',
        'installed_path': 'install/robotest_sim/lib/robotest_sim/contact_stream_gate',
        'installed_regular_executable': True,
        'installed_sha256': 'd' * 64,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': 'a' * 64,
    }


def _gate_attestation(frozen: dict[str, Any]) -> dict[str, Any]:
    attestation = {field: 0 for field in orchestration.CONTACT_GATE_ATTESTATION_FIELDS}
    attestation.update(
        {
            **{
                field: value
                for field, value in frozen.items()
                if field in orchestration.CONTACT_GATE_ATTESTATION_FIELDS
            },
            'exact_live_process_count': 1,
            'identity_revalidated_after_hashing': True,
            'installed_device': 7,
            'installed_inode': 8,
            'launch_root_pid': 100,
            'live_cmdline_sha256': 'e' * 64,
            'live_device': 7,
            'live_elf_build_id': frozen['installed_elf_build_id'],
            'live_embedded_source_inventory_match': True,
            'live_embedded_source_inventory_sha256': frozen[
                'installed_embedded_source_inventory_sha256'
            ],
            'live_executable_link': '/workspace/contact_stream_gate',
            'live_executable_path': '/workspace/contact_stream_gate',
            'live_executable_sha256': frozen['installed_sha256'],
            'live_inode': 8,
            'live_installed_build_id_match': True,
            'live_installed_inode_match': True,
            'live_installed_sha256_match': True,
            'live_pgid': 100,
            'live_pid': 102,
            'live_ppid': 100,
            'live_sid': 100,
            'live_size_bytes': 1_000,
            'live_start_ticks': 2345,
            'observed_gz_partition': 'robotest_p3_candidate_trial',
            'observed_ros_domain_id': '100',
            'process_identity_match': True,
            'verdict': 'PASS',
        }
    )
    return attestation


def test_reobservation_rejects_aggregator_dso_identity_change(
    tmp_path: Path,
) -> None:
    gate_binary = _gate_binary()
    aggregator_binary = _aggregator_record()
    document = {
        'contact_aggregator_binary_attestation': _runtime_attestation(aggregator_binary),
        'contact_gate_binary_attestation': _gate_attestation(gate_binary),
        'verdict': 'PASS',
    }
    initial = tmp_path / 'initial.json'
    final = tmp_path / 'final.json'
    orchestration.atomic_write_json(initial, document, sidecar=True)
    orchestration.atomic_write_json(final, document, sidecar=True)

    evidence = orchestration.reconcile_contact_gate_reobservation(
        initial,
        final,
        build_binding={
            'contact_aggregator_binary': aggregator_binary,
            'contact_gate_binary': gate_binary,
        },
        expected_domain_id=100,
        expected_gz_partition='robotest_p3_candidate_trial',
    )
    assert evidence['stable_aggregator_identity'] is True
    assert evidence['live_aggregator_pid'] == 101

    rebound = copy.deepcopy(document)
    aggregator_attestation = rebound['contact_aggregator_binary_attestation']
    aggregator_attestation['live_start_ticks'] += 1
    aggregator_attestation['stable_identity']['live_start_ticks'] += 1
    aggregator_attestation['stable_identity_sha256'] = orchestration.canonical_sha256(
        aggregator_attestation['stable_identity']
    )
    orchestration.atomic_write_json(final, rebound, sidecar=True)
    with pytest.raises(
        orchestration.EvidenceError, match='aggregator process/DSO identity changed'
    ):
        orchestration.reconcile_contact_gate_reobservation(
            initial,
            final,
            build_binding={
                'contact_aggregator_binary': aggregator_binary,
                'contact_gate_binary': gate_binary,
            },
            expected_domain_id=100,
            expected_gz_partition='robotest_p3_candidate_trial',
        )


def test_build_binding_requires_exact_contact_aggregator_record(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = {
        'aggregate_sha256': '1' * 64,
        'file_count': 1,
        'files': [],
        'total_bytes': 1,
    }
    correspondence = {
        'aggregate_sha256': '2' * 64,
        'all_match': True,
        'file_count': 1,
        'records': [],
    }
    gate_binary = {'source_inventory_sha256': 'a' * 64}
    aggregator_binary = _aggregator_record()
    monkeypatch.setattr(orchestration, 'tree_manifest', lambda *_args: copy.deepcopy(manifest))
    monkeypatch.setattr(
        orchestration, 'install_manifest', lambda _workspace: copy.deepcopy(manifest)
    )
    monkeypatch.setattr(
        orchestration,
        'source_install_correspondence',
        lambda _workspace: copy.deepcopy(correspondence),
    )
    monkeypatch.setattr(
        orchestration,
        'contact_gate_build_install_binding',
        lambda _workspace: copy.deepcopy(gate_binary),
    )
    monkeypatch.setattr(
        orchestration,
        'contact_aggregator_build_install_binding',
        lambda _workspace: copy.deepcopy(aggregator_binary),
    )
    monkeypatch.setattr(orchestration, 'configuration_hash', lambda *_args: '3' * 64)
    monkeypatch.setattr(orchestration, 'file_sha256', lambda _path: '4' * 64)

    binding = orchestration.build_binding(
        tmp_path,
        git_sha='5' * 40,
        git_status_porcelain='',
    )
    assert binding['contact_aggregator_binary'] == aggregator_binary
    assert (
        orchestration.validate_build_binding(
            tmp_path,
            binding,
            git_sha='5' * 40,
            git_status_porcelain='',
        )
        == binding
    )

    forged = copy.deepcopy(binding)
    forged['contact_aggregator_binary']['installed_sha256'] = '9' * 64
    with pytest.raises(orchestration.EvidenceError, match='aggregator binary differs'):
        orchestration.validate_build_binding(
            tmp_path,
            forged,
            git_sha='5' * 40,
            git_status_porcelain='',
        )


def _build_binding(aggregator_binary: dict[str, Any]) -> dict[str, Any]:
    return {
        'collector_configuration_sha256': '1' * 64,
        'contact_aggregator_binary': copy.deepcopy(aggregator_binary),
        'contact_gate_binary': {'source_inventory_sha256': 'a' * 64},
        'install': {'aggregate_sha256': '2' * 64},
        'metrics_contract_sha256': '3' * 64,
        'source': {'aggregate_sha256': '4' * 64},
        'source_configuration_sha256': '5' * 64,
        'source_install': {'aggregate_sha256': '6' * 64, 'all_match': True},
        'target_set_sha256': '7' * 64,
    }


def test_orchestrator_source_binding_freezes_aggregator_across_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    aggregator_binary = _aggregator_record()
    build_start = _build_binding(aggregator_binary)
    build_end = copy.deepcopy(build_start)
    monkeypatch.setattr(orchestration, '_artifact_sizes', lambda *_args, **_kwargs: {})
    resource_summary = {field: 0 for field in orchestration.RESOURCE_METRIC_FIELDS} | {
        'affinity_escape_count': 0,
        'affinity_observed_cpu_union': [0],
        'affinity_unreadable_count': 0,
    }
    plan = {
        'candidate_id': 'candidate',
        'gz_partition': 'partition',
        'repetition_index': 0,
        'ros_domain_id': 100,
        'run_id': 'run',
        'scenario_id': 1,
        'scenario_sha256': '8' * 64,
        'suite_index': 0,
    }
    execution = {
        'command': 'command',
        'exit_code': 0,
        'wall_duration_s': 1.0,
        'wall_timed_out': False,
        'wall_timeout_s': 2.0,
        'working_directory': '/workspace',
    }
    process = {
        field: True
        for field in (
            'cold_stack',
            'fresh_fault_generation',
            'fresh_localization',
            'new_process_group',
            'partition_unused_before_start',
            'previous_trial_gone',
            'ros_domain_unused_before_start',
        )
    }
    cleanup = {
        field: True
        for field in ('all_owned_processes_exited', 'discovery_endpoints_gone', 'no_orphans')
    }
    gates = {
        field: True
        for field in (
            'graph_contract_pass',
            'namespace_isolation_pass',
            'qos_contract_pass',
            'source_install_binding_pass',
            'validation_autonomy_isolation_pass',
        )
    }

    evidence = orchestration.make_orchestrator_evidence(
        plan=plan,
        build_start=build_start,
        build_end=build_end,
        git_sha='9' * 40,
        git_status_porcelain='',
        resource_summary=resource_summary,
        execution=execution,
        process=process,
        cleanup=cleanup,
        gates=gates,
        run_dir=tmp_path,
        component_manifest_sha256='b' * 64,
    )
    assert evidence['source_binding']['contact_aggregator_binary'] == aggregator_binary

    changed_end = copy.deepcopy(build_end)
    changed_end['contact_aggregator_binary']['build_sha256'] = 'c' * 64
    with pytest.raises(
        orchestration.EvidenceError, match='aggregator build/install binding changed'
    ):
        orchestration.make_orchestrator_evidence(
            plan=plan,
            build_start=build_start,
            build_end=changed_end,
            git_sha='9' * 40,
            git_status_porcelain='',
            resource_summary=resource_summary,
            execution=execution,
            process=process,
            cleanup=cleanup,
            gates=gates,
            run_dir=tmp_path,
            component_manifest_sha256='b' * 64,
        )
