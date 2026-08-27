#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Adversarial deterministic tests for the Phase 4 acceptance verifier."""

from __future__ import annotations

import copy
import csv
import hashlib
import json
import signal
import subprocess
import sys
from pathlib import Path

import pytest

TESTS = Path(__file__).resolve().parent
REPOSITORY = TESTS.parent
sys.path.insert(0, str(TESTS))
sys.path.insert(0, str(REPOSITORY / 'src/robotest_missions'))

from phase4_acceptance import (  # noqa: E402
    ACTIVE_GOAL_SCHEMA_VERSION,
    FOLLOW_WAYPOINTS_ACTION,
    FOLLOW_WAYPOINTS_ACTION_TYPE,
    FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE,
    FOLLOW_WAYPOINTS_SEND_GOAL_TYPE,
    EvidenceError,
    action_client_evidence_from_graph_entries,
    assert_process_identity,
    atomic_write_json,
    canonical_json_bytes,
    evaluate_run,
    file_sha256,
    find_exact_controller,
    mission_pair,
    package_artifact_descriptor,
    package_source_binding,
    process_group_members,
    render_followup_mission,
    render_supervisor_config,
    repository_package_manifest,
    signal_process_identity,
    validate_lifecycle_evidence,
    verify_package_candidate,
    write_checksums,
    write_result,
)
from robotest_missions.artifacts import build_result, write_result_artifacts  # noqa: E402
from robotest_missions.execution import (  # noqa: E402
    ExecutionRecord,
    ExitCode,
    GoalStatusCode,
    TerminalActionResult,
)

from robotest_missions.schema import load_mission  # noqa: E402


def test_goal_capability_comes_only_from_exact_send_goal_service_client() -> None:
    projected = [(FOLLOW_WAYPOINTS_ACTION, [FOLLOW_WAYPOINTS_ACTION_TYPE])]
    passive_services = [
        (
            f'{FOLLOW_WAYPOINTS_ACTION}/_action/cancel_goal',
            ['action_msgs/srv/CancelGoal'],
        ),
        (
            f'{FOLLOW_WAYPOINTS_ACTION}/_action/get_result',
            [f'{FOLLOW_WAYPOINTS_ACTION_TYPE}_GetResult'],
        ),
    ]
    passive = action_client_evidence_from_graph_entries(passive_services, projected)
    assert passive['mission_runner_is_goal_capable_action_client'] is False
    assert passive['goal_capable_action_clients'] == {}
    assert passive['projected_action_client_participants'] == {
        FOLLOW_WAYPOINTS_ACTION: {'/robotest/mission_runner': [FOLLOW_WAYPOINTS_ACTION_TYPE]}
    }

    exact = action_client_evidence_from_graph_entries(
        [
            *passive_services,
            (FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE, [FOLLOW_WAYPOINTS_SEND_GOAL_TYPE]),
        ],
        projected,
    )
    assert exact['mission_runner_is_goal_capable_action_client'] is True
    assert exact['goal_capable_action_clients'] == {
        FOLLOW_WAYPOINTS_ACTION: {'/robotest/mission_runner': [FOLLOW_WAYPOINTS_ACTION_TYPE]}
    }
    assert exact['send_goal_service_name'] == FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE
    assert exact['send_goal_service_type'] == FOLLOW_WAYPOINTS_SEND_GOAL_TYPE


@pytest.mark.parametrize(
    'wrong_types',
    (
        ['example_interfaces/srv/AddTwoInts'],
        [FOLLOW_WAYPOINTS_SEND_GOAL_TYPE, 'example_interfaces/srv/AddTwoInts'],
    ),
)
def test_send_goal_service_type_must_be_exact(wrong_types: list[str]) -> None:
    evidence = action_client_evidence_from_graph_entries(
        [(FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE, wrong_types)],
        [(FOLLOW_WAYPOINTS_ACTION, [FOLLOW_WAYPOINTS_ACTION_TYPE])],
    )
    assert evidence['mission_runner_is_goal_capable_action_client'] is False


def test_goal_capability_graph_snapshot_is_bounded_and_structural() -> None:
    with pytest.raises(EvidenceError, match='exactly a name and type list'):
        action_client_evidence_from_graph_entries([['only-a-name']], [])
    with pytest.raises(EvidenceError, match='16-type cap'):
        action_client_evidence_from_graph_entries(
            [(FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE, [f'pkg/srv/T{index}' for index in range(17)])],
            [],
        )


def _proc_stat(
    pid: int,
    comm: str,
    ppid: int,
    pgid: int,
    start: int,
    *,
    state: str = 'S',
) -> str:
    fields = [
        state,
        str(ppid),
        str(pgid),
        str(pgid),
        '0',
        '-1',
        '0',
        '0',
        '0',
        '0',
        '0',
        '0',
        '0',
        '0',
        '0',
        '20',
        '0',
        '1',
        '0',
        str(start),
    ]
    return f'{pid} ({comm}) ' + ' '.join(fields) + '\n'


def _fake_process(
    proc: Path,
    pid: int,
    *,
    comm: str,
    ppid: int,
    pgid: int,
    start: int,
    executable: str,
    command: list[str],
    domain: str = '177',
    partition: str = 'robotest_phase4_test',
    unit: str = 'robotest-supervisor.service',
    state: str = 'S',
) -> None:
    root = proc / str(pid)
    root.mkdir()
    (root / 'stat').write_text(
        _proc_stat(pid, comm, ppid, pgid, start, state=state), encoding='utf-8'
    )
    (root / 'cmdline').write_bytes(b'\0'.join(item.encode() for item in command) + b'\0')
    (root / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0GZ_PARTITION={partition}\0'.encode())
    (root / 'cgroup').write_text(f'0::/system.slice/{unit}\n', encoding='utf-8')
    (root / 'exe').symlink_to(executable)


def _controller_tree(proc: Path) -> dict:
    proc.mkdir()
    _fake_process(
        proc,
        100,
        comm='wrapper',
        ppid=1,
        pgid=100,
        start=10,
        executable='/bin/bash',
        command=['/bin/bash', 'start-robotest-stack'],
    )
    _fake_process(
        proc,
        110,
        comm='launch',
        ppid=100,
        pgid=100,
        start=11,
        executable='/usr/bin/python3',
        command=['python3', 'ros2', 'launch'],
    )
    _fake_process(
        proc,
        120,
        comm='controller_serv',
        ppid=110,
        pgid=100,
        start=12,
        executable='/opt/ros/jazzy/lib/nav2_controller/controller_server',
        command=['/opt/ros/jazzy/lib/nav2_controller/controller_server'],
    )
    return find_exact_controller(100, 100, 177, 'robotest_phase4_test', proc_root=proc)


def test_exact_nested_controller_requires_lineage_group_cgroup_and_environment(
    tmp_path: Path,
) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    assert identity['pid'] == 120
    assert identity['root_pid'] == 100
    assert_process_identity(identity, proc_root=proc)
    assert {item['pid'] for item in process_group_members(100, proc)} == {100, 110, 120}

    (proc / '120/stat').write_text(
        _proc_stat(120, 'controller_serv', 110, 100, 99), encoding='utf-8'
    )
    with pytest.raises(EvidenceError, match='identity changed'):
        assert_process_identity(identity, proc_root=proc)


def test_controller_cmdline_spoof_without_exact_executable_is_rejected(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    proc.mkdir()
    _fake_process(
        proc,
        100,
        comm='wrapper',
        ppid=1,
        pgid=100,
        start=10,
        executable='/bin/bash',
        command=['bash'],
    )
    _fake_process(
        proc,
        120,
        comm='controller_serv',
        ppid=100,
        pgid=100,
        start=12,
        executable='/usr/bin/python3',
        command=['python3', '/tmp/controller_server'],
    )
    with pytest.raises(EvidenceError, match='found 0'):
        find_exact_controller(100, 100, 177, 'robotest_phase4_test', proc_root=proc)


@pytest.mark.parametrize(
    'drift',
    ('cgroup', 'environment', 'zombie', 'lineage', 'root_pgid'),
)
def test_immediate_process_identity_revalidation_rejects_drift(
    tmp_path: Path,
    drift: str,
) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    if drift == 'cgroup':
        (proc / '120/cgroup').write_text('0::/system.slice/spoof.service\n', encoding='utf-8')
    elif drift == 'environment':
        (proc / '120/environ').write_bytes(b'ROS_DOMAIN_ID=177\0GZ_PARTITION=drifted_partition\0')
    elif drift == 'zombie':
        (proc / '120/stat').write_text(
            _proc_stat(120, 'controller_serv', 110, 100, 12, state='Z'),
            encoding='utf-8',
        )
    elif drift == 'lineage':
        (proc / '120/stat').write_text(
            _proc_stat(120, 'controller_serv', 100, 100, 12), encoding='utf-8'
        )
    else:
        (proc / '100/stat').write_text(_proc_stat(100, 'wrapper', 1, 999, 10), encoding='utf-8')
    with pytest.raises(EvidenceError):
        assert_process_identity(identity, proc_root=proc)


def test_identity_is_revalidated_immediately_before_exact_signal(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    calls: list[tuple[object, ...]] = []

    def open_pidfd(pid: int, flags: int) -> int:
        calls.append(('open', pid, flags))
        return 42

    def send_pidfd(descriptor: int, signum: int, siginfo: object, flags: int) -> None:
        calls.append(('signal', descriptor, signum, siginfo, flags))

    def close_pidfd(descriptor: int) -> None:
        calls.append(('close', descriptor))

    assert (
        signal_process_identity(
            identity,
            signal.SIGTERM,
            proc_root=proc,
            pidfd_opener=open_pidfd,
            pidfd_signaler=send_pidfd,
            fd_closer=close_pidfd,
        )
        == 120
    )
    assert calls == [
        ('open', 120, 0),
        ('signal', 42, signal.SIGTERM, None, 0),
        ('close', 42),
    ]

    calls.clear()
    (proc / '120/environ').write_bytes(b'ROS_DOMAIN_ID=177\0GZ_PARTITION=drifted\0')
    with pytest.raises(EvidenceError):
        signal_process_identity(
            identity,
            signal.SIGKILL,
            proc_root=proc,
            pidfd_opener=open_pidfd,
            pidfd_signaler=send_pidfd,
            fd_closer=close_pidfd,
        )
    assert calls == [('open', 120, 0), ('close', 42)]


def test_pidfd_open_precedes_validation_and_pid_reuse_never_signals(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    calls: list[tuple[object, ...]] = []

    def open_then_reuse(pid: int, flags: int) -> int:
        calls.append(('open', pid, flags))
        (proc / '120/stat').write_text(
            _proc_stat(120, 'controller_serv', 110, 100, 99), encoding='utf-8'
        )
        return 43

    with pytest.raises(EvidenceError, match='identity changed'):
        signal_process_identity(
            identity,
            signal.SIGTERM,
            proc_root=proc,
            pidfd_opener=open_then_reuse,
            pidfd_signaler=lambda *_args: calls.append(('signal',)),
            fd_closer=lambda descriptor: calls.append(('close', descriptor)),
        )
    assert calls == [('open', 120, 0), ('close', 43)]


@pytest.mark.parametrize('missing_api', ('open', 'signal', 'close'))
def test_pidfd_api_unavailable_fails_without_numeric_fallback(
    tmp_path: Path,
    missing_api: str,
) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    calls: list[tuple[object, ...]] = []
    opener = None if missing_api == 'open' else lambda pid, flags: 44
    sender = None if missing_api == 'signal' else lambda *_args: calls.append(('signal',))
    closer = (
        None if missing_api == 'close' else lambda descriptor: calls.append(('close', descriptor))
    )
    with pytest.raises(EvidenceError, match='pidfd signaling APIs are unavailable'):
        signal_process_identity(
            identity,
            signal.SIGTERM,
            proc_root=proc,
            pidfd_opener=opener,
            pidfd_signaler=sender,
            fd_closer=closer,
        )
    assert calls == []


def test_pidfd_signal_error_closes_descriptor_and_fails_closed(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    identity = _controller_tree(proc)
    closed: list[int] = []

    def fail_signal(*_args: object) -> None:
        raise ProcessLookupError('pinned task exited')

    with pytest.raises(EvidenceError, match='could not signal pidfd'):
        signal_process_identity(
            identity,
            signal.SIGTERM,
            proc_root=proc,
            pidfd_opener=lambda _pid, _flags: 45,
            pidfd_signaler=fail_signal,
            fd_closer=closed.append,
        )
    assert closed == [45]


def test_duplicate_or_wrong_partition_controller_fails_closed(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    proc.mkdir()
    _fake_process(
        proc,
        100,
        comm='wrapper',
        ppid=1,
        pgid=100,
        start=10,
        executable='/bin/bash',
        command=['bash'],
    )
    for pid in (120, 121):
        _fake_process(
            proc,
            pid,
            comm='controller',
            ppid=100,
            pgid=100,
            start=pid,
            executable='/opt/ros/jazzy/lib/nav2_controller/controller_server',
            command=['controller_server'],
        )
    with pytest.raises(EvidenceError, match='found 2'):
        find_exact_controller(100, 100, 177, 'robotest_phase4_test', proc_root=proc)
    (proc / '121/environ').write_bytes(b'ROS_DOMAIN_ID=177\0GZ_PARTITION=some_other_partition\0')
    assert (
        find_exact_controller(100, 100, 177, 'robotest_phase4_test', proc_root=proc)['pid'] == 120
    )


def test_rendered_config_changes_only_owned_runtime_identity(tmp_path: Path) -> None:
    output = tmp_path / 'config.json'
    rendered = render_supervisor_config(
        REPOSITORY / 'packaging/debian/config.json',
        output,
        '/var/lib/robotest-supervisor/phase4-20260826T000000Z-1',
        177,
        'robotest_phase4_test',
    )
    assert rendered['heartbeat_poll_ms'] == 500
    assert rendered['heartbeat_stale_ms'] == 2000
    assert rendered['termination_grace_ms'] == 5000
    assert rendered['restart'] == {
        'initial_backoff_ms': 1000,
        'maximum_backoff_ms': 8000,
        'maximum_attempts': 4,
        'window_ms': 60000,
        'stable_reset_ms': 60000,
    }
    environment = rendered['children'][0]['environment']
    assert environment['ROS_DOMAIN_ID'] == '177'
    assert environment['GZ_PARTITION'] == 'robotest_phase4_test'
    assert output.stat().st_mode & 0o777 == 0o640


def _mission_result(mission_path: Path) -> dict:
    record = ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='/robotest/follow_waypoints',
        started_sim_stamp_ns=1_000_000_000,
        started_steady_s=10.0,
        goal_submission_stamp_ns=1_000_000_000,
        goal_response_stamp_ns=0,
        accepted_goal_uuid='00000000-0000-0000-0000-000000000001',
        accepted_goal_stamp_ns=2_000_000_000,
        accepted_goal_wall_offset_s=0.2,
        terminal_action_stamp_ns=5_000_000_000,
        terminal_wall_offset_s=3.4,
        terminal_result=TerminalActionResult(
            status_code=int(GoalStatusCode.SUCCEEDED),
            error_code=0,
            error_message='',
            missed_waypoints=(),
        ),
        exit_code=ExitCode.SUCCESS,
        reason='succeeded',
    )
    return build_result(
        load_mission(mission_path),
        record,
        run_id='phase2-mission-phase4-test',
        created_utc='2026-08-26T00:00:00.000000Z',
    )


def _write_jsonl(path: Path, values: list[dict]) -> None:
    path.write_bytes(b''.join(canonical_json_bytes(value) for value in values))


def _timeline_record(sequence: int, kind: str, monotonic_ns: int, **details: object) -> dict:
    return {
        'schema_version': 1,
        'sequence': sequence,
        'timestamp_utc': '2026-08-26T00:00:00.000000Z',
        'monotonic_ns': monotonic_ns,
        'kind': kind,
        'details': details,
    }


def _build_test_package(
    root: Path,
    filename: str,
    version: str,
    listen_address: str,
) -> Path:
    package_root = root.parent / f'.{root.name}-{filename}-root'
    (package_root / 'DEBIAN').mkdir(parents=True)
    (package_root / 'etc/robotest-supervisor').mkdir(parents=True)
    (package_root / 'usr/bin').mkdir(parents=True)
    (package_root / 'usr/libexec/robotest-supervisor').mkdir(parents=True)
    (package_root / 'usr/lib/systemd/system').mkdir(parents=True)
    (package_root / 'DEBIAN/control').write_text(
        '\n'.join(
            (
                'Package: robotest-supervisor',
                f'Version: {version}',
                'Architecture: amd64',
                'Maintainer: RoboTest Test <robotest@example.invalid>',
                'Section: misc',
                'Priority: optional',
                'Description: deterministic Phase 4 verifier fixture',
                '',
            )
        ),
        encoding='utf-8',
    )
    (package_root / 'etc/robotest-supervisor/config.json').write_text(
        json.dumps(
            {'children': [], 'listen_address': listen_address},
            indent=2,
            sort_keys=True,
        )
        + '\n',
        encoding='utf-8',
    )
    binary = package_root / 'usr/bin/robotest-supervisor'
    binary.write_text(f'fixture-binary-{version}\n', encoding='utf-8')
    binary.chmod(0o755)
    helper = package_root / 'usr/libexec/robotest-supervisor/start-robotest-stack'
    helper.write_text(f'#!/bin/sh\n# fixture-helper-{version}\n', encoding='utf-8')
    helper.chmod(0o755)
    (package_root / 'usr/lib/systemd/system/robotest-supervisor.service').write_text(
        f'[Service]\nExecStart=/usr/bin/robotest-supervisor # {version}\n',
        encoding='utf-8',
    )
    output = root / filename
    subprocess.run(
        ['dpkg-deb', '--build', '--root-owner-group', str(package_root), str(output)],
        check=True,
        capture_output=True,
        text=True,
        timeout=30,
    )
    return output


def _write_package_sha256sums(directory: Path) -> None:
    files = sorted(
        path for path in directory.iterdir() if path.is_file() and path.name != 'SHA256SUMS'
    )
    (directory / 'SHA256SUMS').write_text(
        ''.join(f'{file_sha256(path)}  {path.name}\n' for path in files),
        encoding='utf-8',
    )


def _build_reproducible_package_fixture(root: Path) -> tuple[Path, Path, Path]:
    package_directory = root / 'packages'
    build_a = package_directory / 'build-a'
    build_b = package_directory / 'build-b'
    build_a.mkdir(parents=True)
    build_b.mkdir()
    binary_name = 'robotest-supervisor_0.1.0_amd64.deb'
    debug_name = 'robotest-supervisor-dbgsym_0.1.0_amd64.ddeb'
    buildinfo_name = 'robotest-supervisor_0.1.0_amd64.buildinfo'
    changes_name = 'robotest-supervisor_0.1.0_amd64.changes'

    upgrade_package = _build_test_package(
        build_a,
        binary_name,
        '0.1.0',
        '127.0.0.1:9080',
    )
    (build_b / binary_name).write_bytes(upgrade_package.read_bytes())
    for directory in (build_a, build_b):
        (directory / debug_name).write_bytes(b'phase4-debug-symbol-fixture\n')
        (directory / buildinfo_name).write_text(
            'Format: 1.0\nBuild-Date: Tue, 26 Aug 2026 00:00:00 +0000\n',
            encoding='utf-8',
        )
        (directory / changes_name).write_text(
            'Checksums-Sha1:\n'
            '  111 1 robotest-supervisor_0.1.0_amd64.buildinfo\n'
            'Checksums-Sha256:\n'
            '  222 2 robotest-supervisor_0.1.0_amd64.buildinfo\n'
            'Files:\n'
            '  333 misc optional robotest-supervisor_0.1.0_amd64.buildinfo\n',
            encoding='utf-8',
        )
        atomic_write_json(
            directory / 'SOURCE-MANIFEST.json',
            repository_package_manifest(REPOSITORY),
        )
        _write_package_sha256sums(directory)

    records = []
    for first in sorted(path for path in build_a.iterdir() if path.is_file()):
        second = build_b / first.name
        first_payload = first.read_bytes()
        second_payload = second.read_bytes()
        assert first_payload == second_payload
        digest = hashlib.sha256(first_payload).hexdigest()
        records.append(
            {
                'build_a_sha256': digest,
                'build_b_sha256': digest,
                'comparison': 'byte_identical',
                'name': first.name,
                'normalized_sha256': digest,
                'size_bytes': len(first_payload),
            }
        )
    atomic_write_json(
        package_directory / 'reproducibility.json',
        {
            'binary_packages_byte_identical': True,
            'builds': ['build-a', 'build-b'],
            'completed_utc': '2026-08-26T00:00:00Z',
            'files': records,
            'generated_metadata_policy': (
                'dpkg .buildinfo Build-Date is wall-clock metadata; .changes and SHA256SUMS '
                'may differ only through the cascading .buildinfo/.changes checksums'
            ),
            'schema_version': 1,
            'source_manifest_byte_identical': True,
            'verdict': 'PASS',
        },
    )

    baseline_package = _build_test_package(
        package_directory,
        'robotest-supervisor_0.0.9_baseline_amd64.deb',
        '0.0.9',
        '127.0.0.1:9079',
    )
    upgrade = package_artifact_descriptor(upgrade_package)
    baseline = package_artifact_descriptor(baseline_package)
    atomic_write_json(
        Path(f'{baseline_package}.fixture.json'),
        {
            'created_utc': '2026-08-26T00:00:00Z',
            'final_package': {
                'name': upgrade_package.name,
                'sha256': upgrade['sha256'],
                'version': upgrade['version'],
            },
            'fixture_kind': 'genuine_lower_version_different_default_conffile',
            'fixture_package': {
                'listen_address': baseline['vendor_listen_address'],
                'name': baseline_package.name,
                'sha256': baseline['sha256'],
                'version': baseline['version'],
            },
            'schema_version': 1,
        },
    )
    return package_directory, upgrade_package, baseline_package


def _lifecycle_value(
    upgrade: dict[str, str],
    baseline: dict[str, str],
) -> dict:
    return {
        'baseline_package': {
            key: baseline[key]
            for key in (
                'architecture',
                'name',
                'sha256',
                'vendor_config_sha256',
                'version',
            )
        },
        'completed_utc': '2026-08-26T00:00:00+00:00',
        'conffile': {
            'modified_sha256': baseline['modified_config_sha256'],
            'modified_value': {'listen_address': '127.0.0.1:9081'},
            'preserved_during_remove': True,
            'preserved_during_upgrade': True,
            'removed_during_purge': True,
            'upgrade_vendor_default_restored_on_final_install': True,
        },
        'final_state': {
            'configuration_mode_owner': '644:root:root',
            'dpkg_verify_clean': True,
            'installed_binary_sha256': upgrade['installed_binary_sha256'],
            'installed_helper_sha256': upgrade['installed_helper_sha256'],
            'installed_smoke_contract_passed': True,
            'installed_unit_sha256': upgrade['installed_unit_sha256'],
            'log_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
            'package_installed': True,
            'service_account_render_group': True,
            'service_account_video_group': True,
            'service_active': False,
            'service_enabled': False,
            'stale_dpkg_conffile_artifacts': 0,
            'state_directory_mode_owner': ('750:robotest-supervisor:robotest-supervisor'),
            'systemd_unit_verified': True,
        },
        'quality': {
            'baseline_provenance_verified': True,
            'both_package_manifests_exact': True,
            'genuine_versioned_upgrade': True,
            'legacy_noble_lintian_static_built_using_warning': True,
            'lintian_unexpected_diagnostics': 0,
        },
        'schema_version': 1,
        'state': {
            'log_directory_preserved': True,
            'sentinel_preserved_during_purge': True,
            'sentinel_preserved_during_remove': True,
            'state_directory_preserved': True,
        },
        'upgrade_package': {
            key: upgrade[key]
            for key in (
                'architecture',
                'name',
                'sha256',
                'vendor_config_sha256',
                'version',
            )
        },
        'verdict': 'PASS',
    }


def _installed_state_value(lifecycle: dict, upgrade: dict[str, str]) -> dict:
    final = lifecycle['final_state']
    return {
        'architecture': upgrade['architecture'],
        'captured_utc': '2026-08-26T00:00:00+00:00',
        'configuration_mode_owner': final['configuration_mode_owner'],
        'configuration_sha256': upgrade['vendor_config_sha256'],
        'dpkg_verify_clean': final['dpkg_verify_clean'],
        'installed_binary_sha256': final['installed_binary_sha256'],
        'installed_helper_sha256': final['installed_helper_sha256'],
        'installed_smoke_contract_passed': final['installed_smoke_contract_passed'],
        'installed_unit_sha256': final['installed_unit_sha256'],
        'log_directory_mode_owner': final['log_directory_mode_owner'],
        'package_installed': final['package_installed'],
        'package_name': upgrade['name'],
        'schema_version': 1,
        'service_account_render_group': final['service_account_render_group'],
        'service_account_video_group': final['service_account_video_group'],
        'service_active': final['service_active'],
        'service_enabled': final['service_enabled'],
        'stale_dpkg_conffile_artifacts': final['stale_dpkg_conffile_artifacts'],
        'state_directory_mode_owner': final['state_directory_mode_owner'],
        'systemd_unit_verified': final['systemd_unit_verified'],
        'verdict': 'PASS',
        'version': upgrade['version'],
    }


def _descriptor_pair() -> tuple[dict[str, str], dict[str, str]]:
    baseline = {
        'architecture': 'amd64',
        'installed_binary_sha256': '1' * 64,
        'installed_helper_sha256': '2' * 64,
        'installed_unit_sha256': '3' * 64,
        'modified_config_sha256': '4' * 64,
        'name': 'robotest-supervisor',
        'sha256': '5' * 64,
        'vendor_config_sha256': '6' * 64,
        'version': '0.0.9',
    }
    upgrade = {
        'architecture': 'amd64',
        'installed_binary_sha256': '7' * 64,
        'installed_helper_sha256': '8' * 64,
        'installed_unit_sha256': '9' * 64,
        'modified_config_sha256': 'a' * 64,
        'name': 'robotest-supervisor',
        'sha256': 'b' * 64,
        'vendor_config_sha256': 'c' * 64,
        'version': '0.1.0',
    }
    return upgrade, baseline


def _write_pass_fixture(root: Path) -> None:
    run_id = 'phase4-20260826T000000Z-999'
    isolation = {
        'schema_version': 1,
        'run_id': run_id,
        'ros_domain_id': 177,
        'gz_partition': 'robotest_phase4_20260826T000000Z_999',
        'domain_was_unused': True,
        'partition_was_unused': True,
    }
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(root)
    upgrade_descriptor = package_artifact_descriptor(upgrade_package)
    baseline_descriptor = package_artifact_descriptor(baseline_package)
    upgrade_sha = upgrade_descriptor['sha256']
    baseline_sha = baseline_descriptor['sha256']
    lifecycle_value = _lifecycle_value(upgrade_descriptor, baseline_descriptor)
    lifecycle_source = root / 'lifecycle-source.json'
    atomic_write_json(lifecycle_source, lifecycle_value)
    atomic_write_json(root / 'package-lifecycle.json', lifecycle_value)
    atomic_write_json(
        root / 'installed-package-state.json',
        _installed_state_value(lifecycle_value, upgrade_descriptor),
    )

    candidate_manifest = package_directory / 'build-a/SOURCE-MANIFEST.json'
    atomic_write_json(
        root / 'package-binding.json',
        package_source_binding(REPOSITORY, candidate_manifest),
    )
    atomic_write_json(
        root / 'package-integrity.json',
        verify_package_candidate(package_directory, upgrade_package, baseline_package),
    )

    snapshot_files: list[dict] = []
    snapshot_hash = hashlib.sha256(canonical_json_bytes(snapshot_files)).hexdigest()
    snapshot = {
        'schema_version': 1,
        'snapshot_sha256': snapshot_hash,
        'files': snapshot_files,
    }
    atomic_write_json(root / 'source-snapshot-before.json', snapshot)
    atomic_write_json(root / 'source-snapshot-after.json', snapshot)

    atomic_write_json(root / 'overlay-source-manifest.json', {'files': []})
    atomic_write_json(root / 'overlay-install-manifest.json', {'files': []})
    staging = {
        'schema_version': 1,
        'active_path': '/opt/robotest-lab',
        'source_manifest_sha256': file_sha256(root / 'overlay-source-manifest.json'),
        'install_manifest_sha256': file_sha256(root / 'overlay-install-manifest.json'),
        'git_commit': 'a' * 40,
        'git_dirty': True,
        'release_id': 'release-test',
    }
    atomic_write_json(root / 'runtime-staging.json', staging)
    atomic_write_json(root / 'overlay-provenance.json', staging)
    render_supervisor_config(
        REPOSITORY / 'packaging/debian/config.json',
        root / 'supervisor-config.json',
        f'/var/lib/robotest-supervisor/{run_id}',
        177,
        isolation['gz_partition'],
    )
    atomic_write_json(
        root / 'context.json',
        {
            'schema_version': 1,
            'run_id': run_id,
            'started_utc': '2026-08-26T00:00:00.000000Z',
            'source_git_commit': 'a' * 40,
            'source_git_dirty': True,
            'package_directory': str(package_directory),
            'isolation': isolation,
            'upgrade_package': {'path': str(upgrade_package), 'sha256': upgrade_sha},
            'baseline_package': {'path': str(baseline_package), 'sha256': baseline_sha},
            'lifecycle_evidence': {
                'path': str(lifecycle_source),
                'sha256': file_sha256(lifecycle_source),
            },
            'active_overlay_target': '/opt/releases/release-test',
        },
    )
    timeline = [
        _timeline_record(
            1,
            'initial_ready',
            0,
            main_pid=10,
            child_pid=100,
            child_pgid=100,
            systemd_nrestarts=0,
            systemd_owned_ros_service_count=1,
        ),
        _timeline_record(
            2,
            'mission_started',
            200_000_000,
            pid=400,
            pgid=400,
        ),
        _timeline_record(
            3,
            'active_goal_probe_started',
            400_000_000,
            pid=500,
            pgid=500,
        ),
        _timeline_record(
            4,
            'failure_injected',
            1_000_000_000,
            target_pid=120,
            original_pgid=100,
            mission_pid=400,
            event_sequence_before=2,
        ),
        _timeline_record(5, 'health_probe', 1_100_000_000, http_status=200),
        _timeline_record(6, 'ready_probe', 1_500_000_000, http_status=503),
        _timeline_record(7, 'ready_unavailable', 1_500_000_000, http_status=503),
        _timeline_record(8, 'health_probe', 2_000_000_000, http_status=200),
        _timeline_record(9, 'ready_probe', 2_000_000_000, http_status=503),
        _timeline_record(10, 'ready_probe', 3_000_000_000, http_status=503),
        _timeline_record(11, 'original_group_empty', 4_000_000_000, member_count=0),
        _timeline_record(12, 'health_probe', 4_100_000_000, http_status=200),
        _timeline_record(13, 'ready_probe', 5_000_000_000, http_status=503),
        _timeline_record(
            14,
            'ready_restored',
            6_000_000_000,
            http_status=200,
            main_pid=10,
            child_pid=300,
            child_pgid=300,
            systemd_nrestarts=0,
            systemd_owned_ros_service_count=1,
            event_sequence_after_recovery=8,
        ),
        _timeline_record(
            15,
            'interrupted_mission_finished',
            6_200_000_000,
            exit_code=143,
            terminated_by_harness=True,
            process_group_empty=True,
        ),
        _timeline_record(16, 'followup_started', 6_400_000_000, pid=600, pgid=600),
        _timeline_record(17, 'followup_finished', 6_800_000_000, exit_code=0),
        _timeline_record(
            18,
            'service_stopped',
            7_000_000_000,
            event_sequence_before_stop=8,
        ),
        _timeline_record(19, 'cleanup_complete', 8_000_000_000),
    ]
    _write_jsonl(root / 'timeline.jsonl', timeline)
    atomic_write_json(
        root / 'controller-target.json',
        {
            'pid': 120,
            'ppid': 110,
            'pgid': 100,
            'root_pid': 100,
            'start_time_ticks': 12,
            'executable': '/opt/ros/jazzy/lib/nav2_controller/controller_server',
            'cmdline': ['controller_server'],
            'cmdline_sha256': 'b' * 64,
            'cgroup': ['0::/system.slice/robotest-supervisor.service'],
            'ros_domain_id': '177',
            'gz_partition': isolation['gz_partition'],
            'unit': 'robotest-supervisor.service',
            'lineage': [
                {'pid': 100, 'ppid': 1, 'start_time_ticks': 10},
                {'pid': 110, 'ppid': 100, 'start_time_ticks': 11},
                {'pid': 120, 'ppid': 110, 'start_time_ticks': 12},
            ],
        },
    )
    active_client_evidence = action_client_evidence_from_graph_entries(
        [
            (
                f'{FOLLOW_WAYPOINTS_ACTION}/_action/cancel_goal',
                ['action_msgs/srv/CancelGoal'],
            ),
            (
                f'{FOLLOW_WAYPOINTS_ACTION}/_action/get_result',
                [f'{FOLLOW_WAYPOINTS_ACTION_TYPE}_GetResult'],
            ),
            (FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE, [FOLLOW_WAYPOINTS_SEND_GOAL_TYPE]),
        ],
        [(FOLLOW_WAYPOINTS_ACTION, [FOLLOW_WAYPOINTS_ACTION_TYPE])],
    )
    atomic_write_json(
        root / 'active-goal.json',
        {
            'schema_version': ACTIVE_GOAL_SCHEMA_VERSION,
            'verdict': 'PASS',
            'captured_utc': '2026-08-26T00:00:00.500000Z',
            'observed_monotonic_ns': 500_000_000,
            'elapsed_wall_s': 0.2,
            'mission_pid': 400,
            'mission_process_alive': True,
            'mission_node': '/robotest/mission_runner',
            'action_name': FOLLOW_WAYPOINTS_ACTION,
            'action_type': FOLLOW_WAYPOINTS_ACTION_TYPE,
            **active_client_evidence,
            'goal_uuid': '00000000-0000-0000-0000-000000000001',
            'accepted_goal_stamp_ns': 1_000_000_000,
            'goal_status_code': 2,
            'goal_status': 'EXECUTING',
            'status_entry_count': 1,
            'attempt_count': 4,
            'ros_domain_id': '177',
            'gz_partition': isolation['gz_partition'],
        },
    )
    events = [
        {
            'schema_version': 1,
            'sequence': 1,
            'timestamp_utc': '2026-08-26T00:00:00Z',
            'steady_wall_ns': 0,
            'kind': 'child_started',
            'child': 'robotest-stack',
            'pid': 100,
            'pgid': 100,
        },
        {
            'schema_version': 1,
            'sequence': 2,
            'timestamp_utc': '2026-08-26T00:00:01Z',
            'steady_wall_ns': 1_000_000_000,
            'kind': 'readiness_changed',
            'ready': True,
        },
        {
            'schema_version': 1,
            'sequence': 3,
            'timestamp_utc': '2026-08-26T00:00:02Z',
            'steady_wall_ns': 2_000_000_000,
            'kind': 'failure_detected',
            'child': 'robotest-stack',
            'pid': 100,
            'pgid': 100,
            'exit_code': -1,
            'failure_kind': 'unexpected_exit',
        },
        {
            'schema_version': 1,
            'sequence': 4,
            'timestamp_utc': '2026-08-26T00:00:02.1Z',
            'steady_wall_ns': 2_100_000_000,
            'kind': 'readiness_changed',
            'ready': False,
        },
        {
            'schema_version': 1,
            'sequence': 5,
            'timestamp_utc': '2026-08-26T00:00:02.2Z',
            'steady_wall_ns': 2_200_000_000,
            'kind': 'restart_scheduled',
            'child': 'robotest-stack',
            'restart_attempt': 1,
            'backoff_ms': 1000,
        },
        {
            'schema_version': 1,
            'sequence': 6,
            'timestamp_utc': '2026-08-26T00:00:03.3Z',
            'steady_wall_ns': 3_300_000_000,
            'kind': 'child_started',
            'child': 'robotest-stack',
            'pid': 300,
            'pgid': 300,
        },
        {
            'schema_version': 1,
            'sequence': 7,
            'timestamp_utc': '2026-08-26T00:00:05Z',
            'steady_wall_ns': 5_000_000_000,
            'kind': 'heartbeat_fresh',
            'child': 'robotest-stack',
        },
        {
            'schema_version': 1,
            'sequence': 8,
            'timestamp_utc': '2026-08-26T00:00:06Z',
            'steady_wall_ns': 6_000_000_000,
            'kind': 'readiness_changed',
            'ready': True,
        },
        {
            'schema_version': 1,
            'sequence': 9,
            'timestamp_utc': '2026-08-26T00:00:06.85Z',
            'steady_wall_ns': 6_850_000_000,
            'kind': 'shutdown_requested',
        },
        {
            'schema_version': 1,
            'sequence': 10,
            'timestamp_utc': '2026-08-26T00:00:06.86Z',
            'steady_wall_ns': 6_860_000_000,
            'kind': 'child_stop_requested',
            'child': 'robotest-stack',
            'pgid': 300,
        },
        {
            'schema_version': 1,
            'sequence': 11,
            'timestamp_utc': '2026-08-26T00:00:06.90Z',
            'steady_wall_ns': 6_900_000_000,
            'kind': 'child_stopped',
            'child': 'robotest-stack',
            'exit_code': 143,
        },
        {
            'schema_version': 1,
            'sequence': 12,
            'timestamp_utc': '2026-08-26T00:00:06.91Z',
            'steady_wall_ns': 6_910_000_000,
            'kind': 'readiness_changed',
            'ready': False,
        },
        {
            'schema_version': 1,
            'sequence': 13,
            'timestamp_utc': '2026-08-26T00:00:06.92Z',
            'steady_wall_ns': 6_920_000_000,
            'kind': 'supervisor_stopped',
        },
    ]
    _write_jsonl(root / 'supervisor-events.jsonl', events)
    atomic_write_json(
        root / 'supervisor-events.meta.json',
        {
            'schema_version': 1,
            'saturated': False,
            'dropped_events': 0,
            'last_attempted_sequence': len(events),
        },
    )
    atomic_write_json(
        root / 'interrupted-outcome.json',
        {
            'schema_version': 1,
            'captured_utc': '2026-08-26T00:00:06Z',
            'process_exit_code': 143,
            'json_present': False,
            'csv_present': False,
            'bounded_wait': True,
            'terminated_by_harness': True,
            'process_group_empty': True,
        },
    )
    mission_path = root / 'followup-mission.json'
    render_followup_mission(mission_path)
    write_result_artifacts(
        _mission_result(mission_path),
        root / 'followup-result.json',
        root / 'followup-result.csv',
    )
    atomic_write_json(
        root / 'ready-restored-status.json',
        {
            'schema_version': 1,
            'healthy': True,
            'ready': True,
            'children': [
                {
                    'name': 'robotest-stack',
                    'running': True,
                    'heartbeat_fresh': True,
                    'restart_count': 1,
                }
            ],
        },
    )
    atomic_write_json(
        root / 'service-final.json',
        {
            'main_pid_before_stop': 10,
            'nrestarts_before_stop': 0,
            'active_before_stop': True,
            'enabled_before_stop': False,
        },
    )
    atomic_write_json(
        root / 'cleanup.json',
        {
            'schema_version': 1,
            'captured_utc': '2026-08-26T00:00:08Z',
            'owned_paths_only': True,
            'service_inactive': True,
            'service_disabled': True,
            'dropin_absent': True,
            'run_state_directory_absent': True,
            'heartbeat_absent': True,
            'startup_result_absent': True,
            'mission_process_group_empty': True,
            'interrupted_process_group_empty': True,
            'followup_process_group_empty': True,
            'probe_process_group_empty': True,
            'replacement_process_group_empty': True,
            'original_process_group_empty': True,
            'daemon_reloaded': True,
            'source_unchanged': True,
            'interrupted_last_pgid': 400,
            'followup_last_pgid': 600,
            'probe_last_pgid': 500,
            'original_child_pgid': 100,
            'replacement_child_pgid': 300,
        },
    )


def test_pass_fixture_reconciles_every_scenario6_gate(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    result = evaluate_run(tmp_path)
    assert result['verdict'] == {
        'status': 'PASS',
        'accepted': True,
        'failure_count': 0,
        'failures': [],
    }
    assert result['measurements']['restart_scheduled_count'] == 1
    assert result['measurements']['supervisor_recovery_time_wall_s'] == 4.0


@pytest.mark.parametrize('mutation', ('projected_only', 'wrong_send_goal_type', 'legacy_schema'))
def test_active_goal_requires_schema_v2_exact_send_goal_ownership(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'active-goal.json'
    active_goal = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'projected_only':
        active_goal['service_clients'] = [
            entry
            for entry in active_goal['service_clients']
            if entry[0] != FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE
        ]
    elif mutation == 'wrong_send_goal_type':
        send_goal = next(
            entry
            for entry in active_goal['service_clients']
            if entry[0] == FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE
        )
        send_goal[1] = ['example_interfaces/srv/AddTwoInts']
    else:
        active_goal['schema_version'] = 1
    atomic_write_json(path, active_goal)

    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'mission_was_active_before_injection' in result['verdict']['failures']


def test_self_consistent_non_goal_capable_evidence_fails_closed(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'active-goal.json'
    active_goal = json.loads(path.read_text(encoding='utf-8'))
    passive_service_clients = [
        entry
        for entry in active_goal['service_clients']
        if entry[0] != FOLLOW_WAYPOINTS_SEND_GOAL_SERVICE
    ]
    rebound = action_client_evidence_from_graph_entries(
        passive_service_clients,
        active_goal['projected_action_clients'],
    )
    active_goal.update(rebound)
    atomic_write_json(path, active_goal)

    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'mission_was_active_before_injection' in result['verdict']['failures']


def test_active_goal_legacy_projected_ownership_shape_fails_closed(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'active-goal.json'
    active_goal = json.loads(path.read_text(encoding='utf-8'))
    for key in (
        'goal_capable_action_clients',
        'mission_runner_is_goal_capable_action_client',
        'projected_action_client_participants',
        'projected_action_clients',
        'send_goal_service_name',
        'send_goal_service_type',
        'service_clients',
    ):
        del active_goal[key]
    active_goal['mission_runner_is_action_client'] = True
    active_goal['action_clients'] = [[FOLLOW_WAYPOINTS_ACTION, [FOLLOW_WAYPOINTS_ACTION_TYPE]]]
    active_goal['schema_version'] = 1
    atomic_write_json(path, active_goal)

    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'active goal schema mismatch' in result['verdict']['failures'][0]


def _load_records(path: Path) -> list[dict]:
    return [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]


def _write_ordered_records(path: Path, records: list[dict]) -> None:
    records.sort(key=lambda value: (value['monotonic_ns'], value['sequence']))
    for index, record in enumerate(records, 1):
        record['sequence'] = index
    _write_jsonl(path, records)


def _bind_timeline_event_window(root: Path, events: list[dict]) -> None:
    timeline_path = root / 'timeline.jsonl'
    timeline = _load_records(timeline_path)
    ready_sequence = max(
        event['sequence']
        for event in events
        if event['kind'] == 'readiness_changed' and event.get('ready') is True
    )
    next(record for record in timeline if record['kind'] == 'ready_restored')['details'][
        'event_sequence_after_recovery'
    ] = ready_sequence
    pre_stop_sequence = (
        next(event['sequence'] for event in events if event['kind'] == 'shutdown_requested') - 1
    )
    next(record for record in timeline if record['kind'] == 'service_stopped')['details'][
        'event_sequence_before_stop'
    ] = pre_stop_sequence
    _write_jsonl(timeline_path, timeline)


def _install_interrupted_artifact(
    root: Path,
    artifact_exit: object,
    *,
    terminal_success: bool = False,
) -> None:
    result = copy.deepcopy(_mission_result(root / 'followup-mission.json'))
    verdict = result['verdict']
    verdict['exit_code'] = artifact_exit
    verdict['expected_outcome_met'] = False
    verdict['phase2_action_integration_status'] = 'FAIL'
    verdict['reason'] = 'interrupted'
    if terminal_success:
        verdict['mission_status'] = 'SUCCEEDED'
    else:
        verdict['mission_status'] = 'INTERRUPTED'
        measurements = result['measurements']
        for field in (
            'goal_status',
            'goal_status_code',
            'nav2_error_code',
            'nav2_error_message',
            'terminal_action_stamp',
            'terminal_action_stamp_ns',
        ):
            measurements[field] = None
        result['quality']['terminal_result_observed'] = False
    write_result_artifacts(
        result,
        root / 'interrupted-result.json',
        root / 'interrupted-result.csv',
    )
    outcome_path = root / 'interrupted-outcome.json'
    outcome = json.loads(outcome_path.read_text(encoding='utf-8'))
    outcome['json_present'] = True
    outcome['csv_present'] = True
    atomic_write_json(outcome_path, outcome)


@pytest.mark.parametrize(
    ('mutation', 'failure_fragment'),
    (
        ('missing', 'exactly one followup_finished, found 0'),
        ('duplicate', 'exactly one followup_finished, found 2'),
        ('nonzero', 'exactly_one_successful_followup_completion'),
        ('followup_mismatch', 'exactly_one_successful_followup_completion'),
        ('interrupted_mismatch', 'interrupted_completion_matches_outcome'),
    ),
)
def test_mission_completion_evidence_fails_closed(
    tmp_path: Path,
    mutation: str,
    failure_fragment: str,
) -> None:
    _write_pass_fixture(tmp_path)
    timeline_path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(timeline_path)
    followup = next(record for record in timeline if record['kind'] == 'followup_finished')
    if mutation == 'missing':
        timeline.remove(followup)
    elif mutation == 'duplicate':
        duplicate = copy.deepcopy(followup)
        duplicate['monotonic_ns'] += 1
        timeline.append(duplicate)
    elif mutation == 'nonzero':
        followup['details']['exit_code'] = 9
    elif mutation == 'followup_mismatch':
        followup['details']['exit_code'] = 1
    else:
        interrupted = next(
            record for record in timeline if record['kind'] == 'interrupted_mission_finished'
        )
        interrupted['details']['process_group_empty'] = False
    _write_ordered_records(timeline_path, timeline)

    if mutation == 'nonzero':
        result_path = tmp_path / 'followup-result.json'
        result_value = json.loads(result_path.read_text(encoding='utf-8'))
        result_value['verdict'].update(
            {
                'exit_code': 9,
                'expected_outcome_met': False,
                'mission_status': 'FOLLOWUP_FAILED',
                'phase2_action_integration_status': 'FAIL',
                'reason': 'followup_failed',
            }
        )
        for field in (
            'goal_status',
            'goal_status_code',
            'nav2_error_code',
            'nav2_error_message',
            'terminal_action_stamp',
            'terminal_action_stamp_ns',
        ):
            result_value['measurements'][field] = None
        result_value['quality']['terminal_result_observed'] = False
        write_result_artifacts(
            result_value,
            result_path,
            tmp_path / 'followup-result.csv',
        )
    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert any(failure_fragment in failure for failure in result['verdict']['failures'])


def test_unreported_interrupted_result_artifacts_are_rejected(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    mission_path = tmp_path / 'followup-mission.json'
    write_result_artifacts(
        _mission_result(mission_path),
        tmp_path / 'interrupted-result.json',
        tmp_path / 'interrupted-result.csv',
    )
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'interrupted_mission_pair_is_consistent' in result['verdict']['failures']


def test_interrupted_artifact_exit_must_equal_actual_and_timeline_exit(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    _install_interrupted_artifact(tmp_path, 3)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'interrupted_mission_pair_is_consistent' in result['verdict']['failures']


@pytest.mark.parametrize('mismatch_source', ('timeline', 'outcome'))
def test_interrupted_exit_three_way_reconciliation_rejects_each_mismatch(
    tmp_path: Path,
    mismatch_source: str,
) -> None:
    _write_pass_fixture(tmp_path)
    _install_interrupted_artifact(tmp_path, 143)
    if mismatch_source == 'timeline':
        timeline_path = tmp_path / 'timeline.jsonl'
        timeline = _load_records(timeline_path)
        next(record for record in timeline if record['kind'] == 'interrupted_mission_finished')[
            'details'
        ]['exit_code'] = 142
        _write_jsonl(timeline_path, timeline)
    else:
        outcome_path = tmp_path / 'interrupted-outcome.json'
        outcome = json.loads(outcome_path.read_text(encoding='utf-8'))
        outcome['process_exit_code'] = 142
        atomic_write_json(outcome_path, outcome)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert (
        'interrupted_mission_pair_is_consistent' in result['verdict']['failures']
        or 'interrupted_completion_matches_outcome' in result['verdict']['failures']
    )


def test_interrupted_artifact_bool_exit_is_not_accepted_as_zero(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    _install_interrupted_artifact(tmp_path, False)
    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'verdict.exit_code must be an integer' in result['verdict']['failures'][0]


def test_interrupted_terminal_status_cannot_contradict_nonzero_verdict(
    tmp_path: Path,
) -> None:
    _write_pass_fixture(tmp_path)
    _install_interrupted_artifact(tmp_path, 143, terminal_success=True)
    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'reports SUCCEEDED with a nonzero exit code' in result['verdict']['failures'][0]


@pytest.mark.parametrize(
    'mutation',
    (
        'wrong_original_pid',
        'wrong_failure_kind',
        'wrong_replacement_pgid',
        'duplicate_transition',
        'unrelated_transition',
        'non_strict_time',
        'transition_after_recovery',
    ),
)
def test_supervisor_causal_chain_rejects_spoof_drift_and_duplicates(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.jsonl'
    events = _load_records(path)
    failure = next(event for event in events if event['kind'] == 'failure_detected')
    replacement = next(
        event for event in events if event['kind'] == 'child_started' and event.get('pid') == 300
    )
    if mutation == 'wrong_original_pid':
        failure['pid'] = 999
    elif mutation == 'wrong_failure_kind':
        failure['failure_kind'] = 'heartbeat_stale'
    elif mutation == 'wrong_replacement_pgid':
        replacement['pgid'] = 301
    elif mutation == 'duplicate_transition':
        duplicate = copy.deepcopy(
            next(event for event in events if event['kind'] == 'restart_scheduled')
        )
        duplicate['steady_wall_ns'] += 1
        events.append(duplicate)
    elif mutation == 'unrelated_transition':
        events.append(
            {
                'schema_version': 1,
                'sequence': 999,
                'timestamp_utc': '2026-08-26T00:00:02.5Z',
                'steady_wall_ns': 2_500_000_000,
                'kind': 'child_started',
                'child': 'unrelated-child',
                'pid': 700,
                'pgid': 700,
            }
        )
    elif mutation == 'non_strict_time':
        restart = next(event for event in events if event['kind'] == 'restart_scheduled')
        restart['steady_wall_ns'] = 2_100_000_000
    else:
        events.append(
            {
                'schema_version': 1,
                'sequence': 999,
                'timestamp_utc': '2026-08-26T00:00:06.1Z',
                'steady_wall_ns': 6_100_000_000,
                'kind': 'restart_scheduled',
                'child': 'unrelated-child',
                'restart_attempt': 1,
                'backoff_ms': 1000,
            }
        )
    events.sort(key=lambda value: (value['steady_wall_ns'], value['sequence']))
    for index, event in enumerate(events, 1):
        event['sequence'] = index
    _write_jsonl(path, events)
    _bind_timeline_event_window(tmp_path, events)
    metadata = json.loads((tmp_path / 'supervisor-events.meta.json').read_text())
    metadata['last_attempted_sequence'] = len(events)
    atomic_write_json(tmp_path / 'supervisor-events.meta.json', metadata)

    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'exact_causal_supervisor_event_chain' in result['verdict']['failures']


def _set_exact_acceptance_boundaries(root: Path, *, recovery_ns: int) -> None:
    timeline_path = root / 'timeline.jsonl'
    timeline = _load_records(timeline_path)
    timestamps = {
        'ready_unavailable': 4_000_000_000,
        'original_group_empty': 6_000_000_000,
        'ready_restored': recovery_ns,
        'interrupted_mission_finished': recovery_ns + 200_000_000,
        'followup_started': recovery_ns + 400_000_000,
        'followup_finished': recovery_ns + 800_000_000,
        'service_stopped': recovery_ns + 1_000_000_000,
        'cleanup_complete': recovery_ns + 2_000_000_000,
    }
    for record in timeline:
        if record['kind'] in timestamps:
            record['monotonic_ns'] = timestamps[record['kind']]
    _write_ordered_records(timeline_path, timeline)


def test_acceptance_time_boundaries_are_inclusive(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    _set_exact_acceptance_boundaries(tmp_path, recovery_ns=34_000_000_000)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'PASS'
    assert result['measurements']['ready_503_after_injection_wall_s'] == 3.0
    assert result['measurements']['original_group_empty_after_injection_wall_s'] == 5.0
    assert result['measurements']['observed_ready_restore_after_503_wall_s'] == 30.0


def test_recovery_timeout_one_nanosecond_over_boundary_fails(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    _set_exact_acceptance_boundaries(tmp_path, recovery_ns=34_000_000_001)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'observed_recovery_is_bounded' in result['verdict']['failures']


@pytest.mark.parametrize(
    ('field', 'replacement'),
    (
        ('interrupted_process_group_empty', False),
        ('followup_process_group_empty', False),
        ('probe_process_group_empty', False),
        ('replacement_process_group_empty', False),
        ('interrupted_last_pgid', 401),
        ('followup_last_pgid', 601),
        ('probe_last_pgid', 501),
        ('replacement_child_pgid', 301),
    ),
)
def test_cleanup_rejects_residual_or_unbound_last_process_group(
    tmp_path: Path,
    field: str,
    replacement: object,
) -> None:
    _write_pass_fixture(tmp_path)
    cleanup = json.loads((tmp_path / 'cleanup.json').read_text(encoding='utf-8'))
    cleanup[field] = replacement
    atomic_write_json(tmp_path / 'cleanup.json', cleanup)
    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'cleanup.' in result['verdict']['failures'][0]


def test_duplicate_cleanup_finalizer_evidence_is_rejected(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    cleanup = copy.deepcopy(
        next(record for record in timeline if record['kind'] == 'cleanup_complete')
    )
    cleanup['monotonic_ns'] += 1
    timeline.append(cleanup)
    _write_ordered_records(path, timeline)
    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'exactly one cleanup_complete, found 2' in result['verdict']['failures'][0]


def test_live_harness_has_atomic_ownership_and_idempotent_finalizer_contract() -> None:
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    for required in (
        'STATE_STAGING_DIRECTORY="${STATE_RUN_DIRECTORY}.staging-${RUN_ID}"',
        'mv --no-target-directory -- "${STATE_STAGING_DIRECTORY}" "${STATE_RUN_DIRECTORY}"',
        'mv --no-target-directory -- "${DROPIN_STAGING_FILE}" "${DROPIN_FILE}"',
        'if ((FINALIZATION_STATE == 2)); then',
        'Refusing re-entrant finalization.',
        'prove_service_inactive',
        'if ((SERVICE_INACTIVITY_PROVEN))',
        '[[ ! -e "${RUN_DIRECTORY}/cleanup.json" ]] || return 0',
        'begin_process_publication',
        'PENDING_PUBLICATION_SIGNAL=143',
        'MISSION_LAST_PGID="${MISSION_PGID}"',
        'FOLLOWUP_LAST_PGID="${FOLLOWUP_PGID}"',
        'PROBE_LAST_PGID="${PROBE_PGID}"',
        'RESTORED_PGID="${restored_child_pgid}"',
    ):
        assert required in source


def test_package_candidate_rejects_tamper_with_regenerated_local_checksums(
    tmp_path: Path,
) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    assert (
        verify_package_candidate(package_directory, upgrade_package, baseline_package)['verdict']
        == 'PASS'
    )
    with upgrade_package.open('ab') as target:
        target.write(b'tamper-after-build\n')
    _write_package_sha256sums(package_directory / 'build-a')
    with pytest.raises(EvidenceError):
        verify_package_candidate(package_directory, upgrade_package, baseline_package)


def test_package_candidate_rejects_rebound_baseline_sidecar(tmp_path: Path) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    sidecar_path = Path(f'{baseline_package}.fixture.json')
    sidecar = json.loads(sidecar_path.read_text(encoding='utf-8'))
    sidecar['fixture_package']['sha256'] = '0' * 64
    atomic_write_json(sidecar_path, sidecar)
    with pytest.raises(EvidenceError, match='does not bind'):
        verify_package_candidate(package_directory, upgrade_package, baseline_package)


def test_package_candidate_rejects_tampered_reproducibility_json(tmp_path: Path) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    evidence_path = package_directory / 'reproducibility.json'
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    evidence['files'][0]['normalized_sha256'] = '0' * 64
    atomic_write_json(evidence_path, evidence)
    with pytest.raises(EvidenceError, match='does not reconcile'):
        verify_package_candidate(package_directory, upgrade_package, baseline_package)


def test_complete_lifecycle_schema_accepts_all_required_proof_facts() -> None:
    upgrade, baseline = _descriptor_pair()
    validate_lifecycle_evidence(
        _lifecycle_value(upgrade, baseline),
        upgrade,
        baseline,
    )


@pytest.mark.parametrize(
    ('path', 'replacement'),
    (
        pytest.param(('state',), None, id='truncated-top-level'),
        pytest.param(('schema_version',), True, id='boolean-schema-version'),
        pytest.param(
            ('conffile', 'preserved_during_upgrade'),
            None,
            id='truncated-conffile-proof',
        ),
        pytest.param(
            ('conffile', 'preserved_during_remove'),
            False,
            id='tampered-remove-preservation',
        ),
        pytest.param(
            ('conffile', 'removed_during_purge'),
            False,
            id='tampered-purge-removal',
        ),
        pytest.param(
            ('conffile', 'upgrade_vendor_default_restored_on_final_install'),
            False,
            id='tampered-final-config-restoration',
        ),
        pytest.param(
            ('state', 'sentinel_preserved_during_purge'),
            False,
            id='tampered-sentinel-preservation',
        ),
        pytest.param(
            ('state', 'log_directory_preserved'),
            False,
            id='tampered-log-preservation',
        ),
        pytest.param(
            ('final_state', 'state_directory_mode_owner'),
            '755:root:root',
            id='tampered-mode-owner',
        ),
        pytest.param(
            ('final_state', 'installed_smoke_contract_passed'),
            False,
            id='tampered-installed-smoke',
        ),
        pytest.param(
            ('final_state', 'stale_dpkg_conffile_artifacts'),
            1,
            id='tampered-stale-dpkg-artifact-count',
        ),
        pytest.param(
            ('final_state', 'service_active'),
            True,
            id='tampered-final-active-state',
        ),
        pytest.param(
            ('final_state', 'service_enabled'),
            0,
            id='non-boolean-service-state',
        ),
        pytest.param(
            ('final_state', 'stale_dpkg_conffile_artifacts'),
            False,
            id='non-integer-stale-artifact-count',
        ),
        pytest.param(
            ('upgrade_package', 'version'),
            '9.9.9',
            id='tampered-final-version',
        ),
        pytest.param(
            ('quality', 'lintian_unexpected_diagnostics'),
            1,
            id='tampered-quality',
        ),
        pytest.param(('unexpected',), True, id='extra-top-level-field'),
    ),
)
def test_lifecycle_pass_json_rejects_truncation_and_tampering(
    path: tuple[str, ...],
    replacement: object,
) -> None:
    upgrade, baseline = _descriptor_pair()
    lifecycle = copy.deepcopy(_lifecycle_value(upgrade, baseline))
    target = lifecycle
    for component in path[:-1]:
        target = target[component]
    if replacement is None:
        target.pop(path[-1])
    else:
        target[path[-1]] = replacement
    with pytest.raises(EvidenceError):
        validate_lifecycle_evidence(lifecycle, upgrade, baseline)


def test_compositor_rejects_rebound_tampered_lifecycle_pass_json(
    tmp_path: Path,
) -> None:
    _write_pass_fixture(tmp_path)
    source = tmp_path / 'lifecycle-source.json'
    lifecycle = json.loads(source.read_text(encoding='utf-8'))
    lifecycle['final_state']['installed_smoke_contract_passed'] = False
    atomic_write_json(source, lifecycle)
    atomic_write_json(tmp_path / 'package-lifecycle.json', lifecycle)
    context = json.loads((tmp_path / 'context.json').read_text(encoding='utf-8'))
    context['lifecycle_evidence']['sha256'] = file_sha256(source)
    atomic_write_json(tmp_path / 'context.json', context)

    result = write_result(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert result['verdict']['accepted'] is False
    assert result['verdict']['failure_count'] == 1
    assert 'final_state' in result['verdict']['failures'][0]


def test_late_ready_503_and_duplicate_restart_fail_closed(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    timeline = [json.loads(line) for line in (tmp_path / 'timeline.jsonl').read_text().splitlines()]
    for record in timeline:
        if record['kind'] == 'ready_unavailable':
            record['monotonic_ns'] = 4_100_000_000
    timeline.sort(key=lambda value: (value['monotonic_ns'], value['sequence']))
    for index, record in enumerate(timeline, 1):
        record['sequence'] = index
    _write_jsonl(tmp_path / 'timeline.jsonl', timeline)
    events = [
        json.loads(line) for line in (tmp_path / 'supervisor-events.jsonl').read_text().splitlines()
    ]
    duplicate = dict(events[4])
    duplicate['sequence'] = 6
    duplicate['steady_wall_ns'] = 2_300_000_000
    events.insert(5, duplicate)
    for index, event in enumerate(events, 1):
        event['sequence'] = index
    _write_jsonl(tmp_path / 'supervisor-events.jsonl', events)
    _bind_timeline_event_window(tmp_path, events)
    metadata = json.loads((tmp_path / 'supervisor-events.meta.json').read_text())
    metadata['last_attempted_sequence'] = len(events)
    atomic_write_json(tmp_path / 'supervisor-events.meta.json', metadata)

    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'ready_503_within_3s' in result['verdict']['failures']
    assert 'exactly_one_restart_scheduled' in result['verdict']['failures']


def test_mission_csv_tamper_is_rejected(tmp_path: Path) -> None:
    mission_path = tmp_path / 'followup.json'
    render_followup_mission(mission_path)
    write_result_artifacts(
        _mission_result(mission_path), tmp_path / 'result.json', tmp_path / 'result.csv'
    )
    text = (tmp_path / 'result.csv').read_text(encoding='utf-8')
    (tmp_path / 'result.csv').write_text(text.replace('SUCCEEDED', 'ABORTED'), encoding='utf-8')
    result, matches = mission_pair(tmp_path / 'result.json', tmp_path / 'result.csv')
    assert result['verdict']['exit_code'] == 0
    assert matches is False


def test_package_source_manifest_binding_detects_byte_drift(tmp_path: Path) -> None:
    candidate = tmp_path / 'SOURCE-MANIFEST.json'
    atomic_write_json(candidate, repository_package_manifest(REPOSITORY))
    report = package_source_binding(REPOSITORY, candidate)
    assert report['verdict'] == 'PASS'
    assert report['missing_paths'] == []
    assert report['extra_paths'] == []
    assert report['mismatched_paths'] == []
    value = json.loads(candidate.read_text(encoding='utf-8'))
    value['files'][0]['sha256'] = '0' * 64
    atomic_write_json(candidate, value)
    drifted = package_source_binding(REPOSITORY, candidate)
    assert drifted['verdict'] == 'FAIL'
    assert drifted['mismatched_paths'] == [value['files'][0]['path']]


def test_checksum_manifest_is_sorted_and_rejects_symlinks(tmp_path: Path) -> None:
    (tmp_path / 'z.txt').write_text('z\n', encoding='utf-8')
    (tmp_path / 'a.txt').write_text('a\n', encoding='utf-8')
    write_checksums(tmp_path)
    rows = list(csv.reader((tmp_path / 'SHA256SUMS').read_text().splitlines(), delimiter=' '))
    names = [row[-1] for row in rows]
    assert names == ['a.txt', 'z.txt']
    (tmp_path / 'link').symlink_to(tmp_path / 'a.txt')
    with pytest.raises(EvidenceError, match='symbolic links'):
        write_checksums(tmp_path)
