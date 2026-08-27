#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Adversarial deterministic tests for the Phase 4 acceptance verifier."""

from __future__ import annotations

import copy
import csv
import hashlib
import json
import os
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
    capture_runtime_affinity,
    evaluate_run,
    expected_supervisor_config,
    file_sha256,
    find_exact_controller,
    import_package_source_rebuild_attestation,
    mission_pair,
    package_artifact_descriptor,
    package_source_binding,
    package_source_rebuild_attestation,
    process_group_members,
    publish_run_evidence,
    render_followup_mission,
    render_overlay_stage_script,
    render_supervisor_config,
    repository_changelog_version,
    repository_package_manifest,
    runtime_source_manifest,
    signal_process_identity,
    source_snapshot,
    validate_lifecycle_evidence,
    validate_runtime_affinity_evidence,
    validate_stable_runtime_ownership_capture,
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
    cpus_allowed: str = '0000003f',
    cpus_allowed_list: str = '0-5',
) -> None:
    root = proc / str(pid)
    root.mkdir()
    (root / 'stat').write_text(
        _proc_stat(pid, comm, ppid, pgid, start, state=state), encoding='utf-8'
    )
    (root / 'cmdline').write_bytes(b'\0'.join(item.encode() for item in command) + b'\0')
    (root / 'environ').write_bytes(f'ROS_DOMAIN_ID={domain}\0GZ_PARTITION={partition}\0'.encode())
    (root / 'cgroup').write_text(f'0::/system.slice/{unit}\n', encoding='utf-8')
    (root / 'status').write_text(
        f'Name:\t{comm}\n'
        f'Pid:\t{pid}\n'
        f'PPid:\t{ppid}\n'
        f'Cpus_allowed:\t{cpus_allowed}\n'
        f'Cpus_allowed_list:\t{cpus_allowed_list}\n',
        encoding='utf-8',
    )
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


def _affinity_tree(proc: Path) -> None:
    proc.mkdir()
    _fake_process(
        proc,
        10,
        comm='supervisor',
        ppid=1,
        pgid=10,
        start=5,
        executable='/usr/bin/robotest-supervisor',
        command=['/usr/bin/robotest-supervisor'],
    )
    _fake_process(
        proc,
        100,
        comm='wrapper',
        ppid=10,
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
        comm='controller',
        ppid=110,
        pgid=100,
        start=12,
        executable='/opt/ros/jazzy/lib/nav2_controller/controller_server',
        command=['/opt/ros/jazzy/lib/nav2_controller/controller_server'],
    )


def test_runtime_affinity_capture_is_stable_exact_and_bounded(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    _affinity_tree(proc)
    value = capture_runtime_affinity(
        10,
        100,
        100,
        177,
        'robotest_phase4_test',
        'initial',
        proc_root=proc,
    )
    assert value['snapshot_stable'] is True
    assert value['controller_pid'] == 120
    assert value['process_count'] == 4
    assert [item['pid'] for item in value['processes']] == [10, 100, 110, 120]
    assert {item['role'] for item in value['processes']} == {
        'supervisor_main',
        'managed_child',
        'unit_descendant',
        'controller',
    }
    joined = validate_runtime_affinity_evidence(
        value,
        phase='initial',
        main_pid=10,
        managed_child_pid=100,
        managed_child_pgid=100,
    )
    assert joined['limited'] is True

    widened = copy.deepcopy(value)
    controller = next(item for item in widened['processes'] if item['role'] == 'controller')
    controller['cpus_allowed'] = '0000007f'
    controller['cpus_allowed_list'] = '0-6'
    assert (
        validate_runtime_affinity_evidence(
            widened,
            phase='initial',
            main_pid=10,
            managed_child_pid=100,
            managed_child_pgid=100,
        )['limited']
        is False
    )


def test_runtime_affinity_rejects_disconnected_parent_cycle(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    _affinity_tree(proc)
    value = capture_runtime_affinity(
        10,
        100,
        100,
        177,
        'robotest_phase4_test',
        'initial',
        proc_root=proc,
    )
    launch = next(item for item in value['processes'] if item['role'] == 'unit_descendant')
    controller = next(item for item in value['processes'] if item['role'] == 'controller')
    launch['ppid'] = controller['pid']
    controller['ppid'] = launch['pid']

    with pytest.raises(EvidenceError, match='does not descend from MainPID'):
        validate_runtime_affinity_evidence(
            value,
            phase='initial',
            main_pid=10,
            managed_child_pid=100,
            managed_child_pgid=100,
        )


def test_runtime_affinity_rejects_nested_or_drifting_unit_cgroup(tmp_path: Path) -> None:
    proc = tmp_path / 'proc'
    _affinity_tree(proc)
    (proc / '120/cgroup').write_text(
        '0::/system.slice/robotest-supervisor.service/nested\n', encoding='utf-8'
    )
    with pytest.raises(EvidenceError, match=r'exact unit-cgroup process set|found 0'):
        capture_runtime_affinity(
            10,
            100,
            100,
            177,
            'robotest_phase4_test',
            'initial',
            proc_root=proc,
        )


def test_rendered_config_changes_only_owned_runtime_identity(tmp_path: Path) -> None:
    output = tmp_path / 'config.json'
    state_directory = '/var/lib/robotest-supervisor/phase4-20260826T000000Z-1'
    rendered = render_supervisor_config(
        REPOSITORY / 'packaging/debian/config.json',
        output,
        state_directory,
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
    assert set(environment) == {
        'GZ_PARTITION',
        'RCUTILS_LOGGING_BUFFERED_STREAM',
        'ROBOTEST_CPUSET',
        'ROBOTEST_NAMESPACE',
        'ROBOTEST_RUNTIME_STATE_DIRECTORY',
        'ROS_DOMAIN_ID',
    }
    assert environment['ROS_DOMAIN_ID'] == '177'
    assert environment['GZ_PARTITION'] == 'robotest_phase4_test'
    assert environment['RCUTILS_LOGGING_BUFFERED_STREAM'] == '1'
    assert environment['ROBOTEST_CPUSET'] == '0-5'
    assert environment['ROBOTEST_NAMESPACE'] == 'robotest'
    assert environment['ROBOTEST_RUNTIME_STATE_DIRECTORY'] == state_directory
    assert rendered['state_directory'] == state_directory
    assert rendered['children'][0]['heartbeat_file'] == (
        f'{state_directory}/robotest-stack.heartbeat'
    )
    assert output.stat().st_mode & 0o777 == 0o640
    assert rendered == expected_supervisor_config(
        state_directory,
        177,
        'robotest_phase4_test',
    )


@pytest.mark.parametrize(
    'mutation',
    (
        'schema_version',
        'listen_address',
        'heartbeat_stale_ms',
        'termination_grace_ms',
        'maximum_event_entries',
        'maximum_event_bytes',
        'restart.maximum_attempts',
        'child.required',
        'extra_key',
    ),
)
def test_rendered_config_rejects_every_complete_contract_mutation(
    tmp_path: Path,
    mutation: str,
) -> None:
    template = tmp_path / 'template.json'
    value = json.loads((REPOSITORY / 'packaging/debian/config.json').read_text(encoding='utf-8'))
    if mutation == 'schema_version':
        value['schema_version'] = True
    elif mutation == 'listen_address':
        value['listen_address'] = '127.0.0.1:9081'
    elif mutation == 'heartbeat_stale_ms':
        value['heartbeat_stale_ms'] = 2001
    elif mutation == 'termination_grace_ms':
        value['termination_grace_ms'] = 5001
    elif mutation == 'maximum_event_entries':
        value['maximum_event_entries'] = 4095
    elif mutation == 'maximum_event_bytes':
        value['maximum_event_bytes'] = 8388607
    elif mutation == 'restart.maximum_attempts':
        value['restart']['maximum_attempts'] = 5
    elif mutation == 'child.required':
        value['children'][0]['required'] = False
    else:
        value['unexpected'] = True
    atomic_write_json(template, value)
    with pytest.raises(EvidenceError, match='complete frozen contract'):
        render_supervisor_config(
            template,
            tmp_path / 'config.json',
            '/var/lib/robotest-supervisor/phase4-20260826T000000Z-1',
            177,
            'robotest_phase4_test',
        )


@pytest.mark.parametrize(
    'state_directory',
    (
        '/var/lib/robotest-supervisor',
        '/var/lib/robotest-supervisor/phase4-20260826T000000Z-1/nested',
        '/var/lib/robotest-supervisor/phase4-20260826T000000Z-1/../escape',
        '/tmp/phase4-20260826T000000Z-1',
    ),
)
def test_rendered_config_rejects_non_run_scoped_state_directory(
    tmp_path: Path,
    state_directory: str,
) -> None:
    with pytest.raises(EvidenceError, match='run state directory'):
        render_supervisor_config(
            REPOSITORY / 'packaging/debian/config.json',
            tmp_path / 'config.json',
            state_directory,
            177,
            'robotest_phase4_test',
        )


def test_overlay_stage_script_redirects_only_frozen_assignments_to_live_scratch(
    tmp_path: Path,
) -> None:
    output = tmp_path / 'stage_runtime_overlay.sh'
    evidence = tmp_path / 'live/overlay-stage-scratch/evidence'
    render_overlay_stage_script(
        REPOSITORY / 'scripts/stage_runtime_overlay.sh',
        output,
        REPOSITORY,
        evidence,
    )
    rendered = output.read_text(encoding='utf-8')
    assert f'SCRIPT_DIR={REPOSITORY}/scripts' in rendered
    assert f'PROJECT_ROOT={REPOSITORY}' in rendered
    assert f'readonly EVIDENCE_DIR={evidence}' in rendered
    assert 'readonly EVIDENCE_DIR="${PROJECT_ROOT}/artifacts/evidence/phase4"' not in rendered
    assert output.stat().st_mode & 0o777 == 0o700


def test_overlay_stage_script_rejects_assignment_drift(tmp_path: Path) -> None:
    template = tmp_path / 'stage.sh'
    source = (REPOSITORY / 'scripts/stage_runtime_overlay.sh').read_text(encoding='utf-8')
    template.write_text(
        source.replace('readonly EVIDENCE_DIR=', 'EVIDENCE_DIR=', 1), encoding='utf-8'
    )
    with pytest.raises(EvidenceError, match='assignment changed'):
        render_overlay_stage_script(
            template,
            tmp_path / 'rendered.sh',
            REPOSITORY,
            tmp_path / 'live/evidence',
        )


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


def _http_probe_details(endpoint: str, status: int) -> dict[str, object]:
    if endpoint == '/healthz':
        body = 'ok\n' if status == 200 else ''
    else:
        body = {0: '', 200: 'ready\n', 503: 'not ready\n'}[status]
    return {
        'url': f'http://127.0.0.1:9080{endpoint}',
        'http_status': status,
        'body': body,
        'body_sha256': hashlib.sha256(body.encode()).hexdigest(),
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
    version = '0.1.1'
    binary_name = f'robotest-supervisor_{version}_amd64.deb'
    debug_name = f'robotest-supervisor-dbgsym_{version}_amd64.ddeb'
    buildinfo_name = f'robotest-supervisor_{version}_amd64.buildinfo'
    changes_name = f'robotest-supervisor_{version}_amd64.changes'

    upgrade_package = _build_test_package(
        build_a,
        binary_name,
        version,
        '127.0.0.1:9080',
    )
    (build_b / binary_name).write_bytes(upgrade_package.read_bytes())
    for directory in (build_a, build_b):
        (directory / debug_name).write_bytes(b'phase4-debug-symbol-fixture\n')
        (directory / buildinfo_name).write_text(
            'Format: 1.0\n'
            'Source: robotest-supervisor\n'
            'Binary: robotest-supervisor robotest-supervisor-dbgsym\n'
            'Architecture: amd64\n'
            f'Version: {version}\n'
            'Build-Date: Tue, 26 Aug 2026 00:00:00 +0000\n',
            encoding='utf-8',
        )
        (directory / changes_name).write_text(
            'Format: 1.8\n'
            'Source: robotest-supervisor\n'
            'Binary: robotest-supervisor robotest-supervisor-dbgsym\n'
            'Architecture: amd64\n'
            f'Version: {version}\n'
            'Checksums-Sha1:\n'
            f'  111 1 robotest-supervisor_{version}_amd64.buildinfo\n'
            'Checksums-Sha256:\n'
            f'  222 2 robotest-supervisor_{version}_amd64.buildinfo\n'
            'Files:\n'
            f'  333 misc optional robotest-supervisor_{version}_amd64.buildinfo\n',
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


def _runtime_affinity_value(phase: str, child_pid: int) -> dict:
    child_start = 10 if phase == 'initial' else 30
    captured_utc = '2026-08-26T00:00:10Z' if phase == 'initial' else '2026-08-26T00:00:20Z'
    launch_pid = child_pid + 10
    controller_pid = child_pid + 20
    cgroup = ['0::/system.slice/robotest-supervisor.service']
    processes = [
        {
            'role': 'supervisor_main',
            'pid': 10,
            'ppid': 1,
            'pgid': 10,
            'start_time_ticks': 5,
            'executable': '/usr/bin/robotest-supervisor',
            'cgroup': cgroup,
            'cpus_allowed': '0000003f',
            'cpus_allowed_list': '0-5',
        },
        {
            'role': 'managed_child',
            'pid': child_pid,
            'ppid': 10,
            'pgid': child_pid,
            'start_time_ticks': child_start,
            'executable': '/usr/bin/bash',
            'cgroup': cgroup,
            'cpus_allowed': '0000003f',
            'cpus_allowed_list': '0-5',
        },
        {
            'role': 'unit_descendant',
            'pid': launch_pid,
            'ppid': child_pid,
            'pgid': child_pid,
            'start_time_ticks': child_start + 1,
            'executable': '/usr/bin/python3',
            'cgroup': cgroup,
            'cpus_allowed': '0000003f',
            'cpus_allowed_list': '0-5',
        },
        {
            'role': 'controller',
            'pid': controller_pid,
            'ppid': launch_pid,
            'pgid': child_pid,
            'start_time_ticks': child_start + 2,
            'executable': '/opt/ros/jazzy/lib/nav2_controller/controller_server',
            'cgroup': cgroup,
            'cpus_allowed': '0000003f',
            'cpus_allowed_list': '0-5',
        },
    ]
    return {
        'schema_version': 1,
        'captured_utc': captured_utc,
        'phase': phase,
        'unit': 'robotest-supervisor.service',
        'unit_cgroup': '/system.slice/robotest-supervisor.service',
        'expected_cpuset': '0-5',
        'main_pid': 10,
        'managed_child_pid': child_pid,
        'managed_child_pgid': child_pid,
        'controller_pid': controller_pid,
        'controller_count': 1,
        'process_count': len(processes),
        'snapshot_stable': True,
        'processes': processes,
        'verdict': 'PASS',
    }


def _supervisor_status_value(
    child_pid: int,
    restart_count: int,
    event_count: int,
    *,
    final: bool = False,
) -> dict:
    child = {
        'name': 'robotest-stack',
        'required': True,
        'running': not final,
        'heartbeat_fresh': not final,
        'heartbeat_age_ms': 0,
        'pid': 0 if final else child_pid,
        'pgid': 0 if final else child_pid,
        'restart_count': restart_count,
        'circuit_open': False,
        'last_ready_utc': '2026-08-26T00:00:05Z',
        'started_utc': '2026-08-26T00:00:03.3Z',
    }
    if final:
        child['last_exit_code'] = 143
    return {
        'schema_version': 1,
        'supervisor_utc': '2026-08-26T00:00:06.92Z',
        'healthy': True,
        'ready': not final,
        'persistence_healthy': True,
        'shutting_down': final,
        'event_count': event_count,
        'dropped_events': 0,
        'children': [child],
    }


def _systemd_owners_value(isolation: dict, child_pid: int) -> dict:
    return {
        'schema_version': 1,
        'captured_utc': '2026-08-26T00:00:05Z',
        'ros_domain_id': isolation['ros_domain_id'],
        'gz_partition': isolation['gz_partition'],
        'units': {'robotest-supervisor.service': [child_pid, child_pid + 10, child_pid + 20]},
        'unit_count': 1,
        'verdict': 'PASS',
    }


def _write_pass_fixture(root: Path) -> None:
    run_id = 'phase4-20260826T000000Z-999'
    isolation = {
        'schema_version': 1,
        'run_id': run_id,
        'ros_domain_id': 177,
        'gz_partition': 'robotest_phase4_20260826T000000Z_999',
        'inspected_processes': 42,
        'unreadable_process_environments': 0,
        'domain_was_unused': True,
        'partition_was_unused': True,
    }
    atomic_write_json(root / 'isolation.json', isolation)
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
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        ),
    )
    atomic_write_json(
        root / 'package-source-rebuild.json',
        package_source_rebuild_attestation(
            REPOSITORY,
            package_directory,
            upgrade_package,
            package_directory / 'build-a',
        ),
    )

    snapshot = source_snapshot(REPOSITORY)
    atomic_write_json(root / 'source-snapshot-before.json', snapshot)
    atomic_write_json(root / 'source-snapshot-after.json', snapshot)

    atomic_write_json(root / 'overlay-source-manifest.json', runtime_source_manifest(REPOSITORY))
    atomic_write_json(
        root / 'overlay-install-manifest.json',
        {
            'schema_version': 1,
            'files': [
                {
                    'mode': '0755',
                    'path': 'robotest_missions/lib/robotest_missions/mission_runner',
                    'sha256': 'd' * 64,
                    'size_bytes': 4096,
                }
            ],
        },
    )
    source_manifest_sha = file_sha256(root / 'overlay-source-manifest.json')
    install_manifest_sha = file_sha256(root / 'overlay-install-manifest.json')
    git_commit = 'a' * 40
    git_dirty = False
    release_id = f'{git_commit[:12]}-{source_manifest_sha[:16]}-false'
    release_path = f'/opt/robotest-lab-releases/{release_id}'
    staging = {
        'schema_version': 1,
        'active_path': '/opt/robotest-lab',
        'build_command': [
            'colcon',
            '--log-base',
            f'{release_path}/.log',
            'build',
            '--base-paths',
            f'{REPOSITORY}/src',
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
        'created_utc': '2026-08-26T00:00:00Z',
        'git_commit': git_commit,
        'git_dirty': git_dirty,
        'install_manifest_sha256': install_manifest_sha,
        'packages': [
            'robotest_description',
            'robotest_faults',
            'robotest_interfaces',
            'robotest_metrics',
            'robotest_missions',
            'robotest_navigation',
            'robotest_scenarios',
            'robotest_sim',
        ],
        'release_id': release_id,
        'source_manifest_sha256': source_manifest_sha,
        'source_workspace': str(REPOSITORY),
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
    (root / 'systemd-dropin.conf').write_text(
        '[Service]\n'
        'ExecStart=\n'
        'ExecStart=/usr/bin/robotest-supervisor --config '
        f'/var/lib/robotest-supervisor/{run_id}/config.json\n',
        encoding='utf-8',
    )
    (root / 'supervisor-config-check.txt').write_bytes(b'configuration valid\n')
    atomic_write_json(
        root / 'lifecycle-startup-result.json',
        {
            'schema_version': 1,
            'verdict': 'PASS',
            'accepted': True,
            'exit_code': 0,
            'service_name': '/robotest/lifecycle_manager_navigation/manage_nodes',
            'command': 0,
            'elapsed_wall_sec': 8.5,
            'discovery_grace_sec': 4.0,
            'service_timeout_sec': 20.0,
            'response_timeout_sec': 60.0,
            'watch_pid': None,
            'failure_kind': None,
            'failure_message': None,
            'started_utc': '2026-08-26T00:00:00Z',
            'completed_utc': '2026-08-26T00:00:08.5Z',
        },
    )
    atomic_write_json(
        root / 'runtime-affinity-initial.json', _runtime_affinity_value('initial', 100)
    )
    atomic_write_json(
        root / 'runtime-affinity-restored.json', _runtime_affinity_value('restored', 300)
    )
    atomic_write_json(root / 'initial-status.json', _supervisor_status_value(100, 0, 4))
    atomic_write_json(root / 'systemd-owners-initial.json', _systemd_owners_value(isolation, 100))
    atomic_write_json(root / 'systemd-owners-restored.json', _systemd_owners_value(isolation, 300))
    atomic_write_json(
        root / 'original-group-empty.json',
        {
            'schema_version': 1,
            'captured_utc': '2026-08-26T00:00:04Z',
            'pgid': 100,
            'member_count': 0,
            'members': [],
        },
    )
    atomic_write_json(
        root / 'context.json',
        {
            'schema_version': 1,
            'run_id': run_id,
            'started_utc': '2026-08-26T00:00:00.000000Z',
            'source_git_commit': git_commit,
            'source_git_dirty': git_dirty,
            'package_directory': str(package_directory),
            'isolation': isolation,
            'upgrade_package': {'path': str(upgrade_package), 'sha256': upgrade_sha},
            'baseline_package': {'path': str(baseline_package), 'sha256': baseline_sha},
            'lifecycle_evidence': {
                'path': str(lifecycle_source),
                'sha256': file_sha256(lifecycle_source),
            },
            'active_overlay_target': release_path,
            'cpuset': '0-5',
        },
    )
    timeline = [
        _timeline_record(
            0,
            'startup_health_probe',
            0,
            **_http_probe_details('/healthz', 200),
        ),
        _timeline_record(
            0,
            'startup_ready_probe',
            50_000_000,
            **_http_probe_details('/readyz', 200),
        ),
        _timeline_record(
            1,
            'initial_ready',
            100_000_000,
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
            event_sequence_before=4,
        ),
        _timeline_record(5, 'health_probe', 1_100_000_000, **_http_probe_details('/healthz', 200)),
        _timeline_record(6, 'ready_probe', 1_500_000_000, **_http_probe_details('/readyz', 503)),
        _timeline_record(7, 'ready_unavailable', 1_500_000_000, http_status=503),
        _timeline_record(8, 'health_probe', 2_000_000_000, **_http_probe_details('/healthz', 200)),
        _timeline_record(9, 'ready_probe', 2_000_000_000, **_http_probe_details('/readyz', 503)),
        _timeline_record(0, 'health_probe', 2_900_000_000, **_http_probe_details('/healthz', 200)),
        _timeline_record(10, 'ready_probe', 3_000_000_000, **_http_probe_details('/readyz', 503)),
        _timeline_record(11, 'original_group_empty', 4_000_000_000, member_count=0),
        _timeline_record(12, 'health_probe', 4_100_000_000, **_http_probe_details('/healthz', 200)),
        _timeline_record(13, 'ready_probe', 5_000_000_000, **_http_probe_details('/readyz', 503)),
        _timeline_record(0, 'health_probe', 5_800_000_000, **_http_probe_details('/healthz', 200)),
        _timeline_record(0, 'ready_probe', 5_900_000_000, **_http_probe_details('/readyz', 200)),
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
            event_sequence_after_recovery=10,
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
            event_sequence_before_stop=10,
        ),
        _timeline_record(19, 'cleanup_complete', 8_000_000_000),
    ]
    for sequence, record in enumerate(timeline, 1):
        record['sequence'] = sequence
    _write_jsonl(root / 'timeline.jsonl', timeline)
    atomic_write_json(
        root / 'controller-target.json',
        {
            'pid': 120,
            'ppid': 110,
            'pgid': 100,
            'comm': 'controller_server',
            'state': 'S',
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
            'kind': 'supervisor_started',
        },
        {
            'schema_version': 1,
            'sequence': 2,
            'timestamp_utc': '2026-08-26T00:00:00.1Z',
            'steady_wall_ns': 100_000_000,
            'kind': 'child_started',
            'child': 'robotest-stack',
            'pid': 100,
            'pgid': 100,
        },
        {
            'schema_version': 1,
            'sequence': 3,
            'timestamp_utc': '2026-08-26T00:00:00.2Z',
            'steady_wall_ns': 200_000_000,
            'kind': 'heartbeat_fresh',
            'child': 'robotest-stack',
        },
        {
            'schema_version': 1,
            'sequence': 4,
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
            'details': {'error': 'signal: terminated'},
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
    for sequence, event in enumerate(events, 1):
        event['sequence'] = sequence
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
        _supervisor_status_value(300, 1, 10),
    )
    atomic_write_json(
        root / 'supervisor-status-final.json',
        _supervisor_status_value(300, 1, 15, final=True),
    )
    atomic_write_json(
        root / 'service-final.json',
        {
            'schema_version': 1,
            'captured_utc': '2026-08-26T00:00:07Z',
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
    assert result['quality']['checks']['lifecycle_startup_result_is_exact'] is True
    assert result['quality']['checks']['runtime_cpu_affinity_is_limited'] is True


def test_lifecycle_startup_result_tamper_fails_named_check(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'lifecycle-startup-result.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    value['watch_pid'] = 999
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'lifecycle_startup_result_is_exact' in result['verdict']['failures']


def test_runtime_affinity_escape_fails_named_check(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'runtime-affinity-restored.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    controller = next(item for item in value['processes'] if item['role'] == 'controller')
    controller['cpus_allowed'] = '0000007f'
    controller['cpus_allowed_list'] = '0-6'
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'runtime_cpu_affinity_is_limited' in result['verdict']['failures']


def test_supervisor_config_check_and_run_paths_are_composed(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    (tmp_path / 'supervisor-config-check.txt').write_text(
        'configuration valid\nextra\n', encoding='utf-8'
    )
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'run_scoped_supervisor_config_is_exact' in result['verdict']['failures']


@pytest.mark.parametrize(
    'mutation',
    (
        'schema_version',
        'listen_address',
        'heartbeat_stale_ms',
        'termination_grace_ms',
        'maximum_event_entries',
        'maximum_event_bytes',
        'restart.maximum_attempts',
        'child.argv',
        'extra_key',
    ),
)
def test_compositor_rejects_any_supervisor_config_contract_drift(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-config.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'schema_version':
        value['schema_version'] = True
    elif mutation == 'listen_address':
        value['listen_address'] = '127.0.0.1:9081'
    elif mutation == 'heartbeat_stale_ms':
        value['heartbeat_stale_ms'] = 2001
    elif mutation == 'termination_grace_ms':
        value['termination_grace_ms'] = 5001
    elif mutation == 'maximum_event_entries':
        value['maximum_event_entries'] = 4095
    elif mutation == 'maximum_event_bytes':
        value['maximum_event_bytes'] = 8388607
    elif mutation == 'restart.maximum_attempts':
        value['restart']['maximum_attempts'] = 5
    elif mutation == 'child.argv':
        value['children'][0]['argv'].append('--unexpected')
    else:
        value['unexpected'] = True
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'run_scoped_supervisor_config_is_exact' in result['verdict']['failures']


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
    baseline_sequence = (
        next(event['sequence'] for event in events if event['kind'] == 'failure_detected') - 1
    )
    next(record for record in timeline if record['kind'] == 'failure_injected')['details'][
        'event_sequence_before'
    ] = baseline_sequence
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
    for filename, event_count in (
        ('initial-status.json', baseline_sequence),
        ('ready-restored-status.json', ready_sequence),
        ('supervisor-status-final.json', len(events)),
    ):
        path = root / filename
        if path.exists():
            value = json.loads(path.read_text(encoding='utf-8'))
            value['event_count'] = event_count
            atomic_write_json(path, value)
    metadata_path = root / 'supervisor-events.meta.json'
    if metadata_path.exists():
        metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
        metadata['last_attempted_sequence'] = len(events)
        atomic_write_json(metadata_path, metadata)


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
        failure.pop('details')
        failure.pop('exit_code')
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
                'child': 'robotest-stack',
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
                'child': 'robotest-stack',
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
    health_probes = [record for record in timeline if record['kind'] == 'health_probe']
    ready_probes = [record for record in timeline if record['kind'] == 'ready_probe']
    for record, monotonic_ns in zip(
        health_probes,
        (1_100_000_000, 4_100_000_000, 4_300_000_000, 6_100_000_000, recovery_ns - 100_000_000),
        strict=True,
    ):
        record['monotonic_ns'] = monotonic_ns
    for record, monotonic_ns in zip(
        ready_probes,
        (4_000_000_000, 4_200_000_000, 4_400_000_000, 6_200_000_000, recovery_ns),
        strict=True,
    ):
        record['monotonic_ns'] = monotonic_ns
    timestamps = {
        'ready_unavailable': 4_000_000_000,
        'original_group_empty': 6_000_000_000,
        'ready_restored': recovery_ns + 100_000_000,
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
        'HEARTBEAT="${STATE_RUN_DIRECTORY}/robotest-stack.heartbeat"',
        'STARTUP_RESULT="${STATE_RUN_DIRECTORY}/lifecycle-startup-result.json"',
        'mv --no-target-directory -- "${STATE_STAGING_DIRECTORY}" "${STATE_RUN_DIRECTORY}"',
        'mv --no-target-directory -- "${DROPIN_STAGING_FILE}" "${DROPIN_FILE}"',
        'if ((FINALIZATION_STATE == 2)); then',
        'Refusing re-entrant finalization.',
        'prove_service_inactive',
        'if ((SERVICE_INACTIVITY_PROVEN))',
        '[[ ! -e "${RUN_DIRECTORY}/cleanup.json" ]] || return 0',
        'begin_process_publication',
        'latch_process_publication_signal 143',
        'PUBLICATION_SIGNAL_TRAPS_CLEARED',
        'MISSION_LAST_PGID="${MISSION_PGID}"',
        'FOLLOWUP_LAST_PGID="${FOLLOWUP_PGID}"',
        'PROBE_LAST_PGID="${PROBE_PGID}"',
        'RESTORED_PGID="${restored_child_pgid}"',
        'check_extracted_candidate_config',
        'supervisor-config-check.txt',
        'capture-runtime-affinity',
        'runtime-affinity-initial.json',
        'runtime-affinity-restored.json',
        'lifecycle-startup-result.json',
        'readonly LIVE_RUN_ROOT="/run/robotest-phase4-verifier"',
        'RUN_DIRECTORY="${LIVE_RUN_ROOT}/${RUN_ID}"',
        'PUBLIC_RUN_DIRECTORY="${PROJECT_ROOT}/artifacts/evidence/phase4/runs/${RUN_ID}"',
        '.phase4-live-owned',
        'freeze_live_evidence',
        'root:${PROJECT_GROUP}:550',
        'root:${PROJECT_GROUP}:710',
        'runuser -u "${PROJECT_USER}" -- env -i',
        'publish-evidence',
        'remove_published_live_evidence',
        'Evidence published atomically: ${PUBLIC_RUN_DIRECTORY}',
        'PHASE 4 PASS: ${PUBLIC_RUN_DIRECTORY}',
        'stage_runtime_overlay_safely',
        'render-overlay-stage-script',
        'OVERLAY_STAGE_EVIDENCE="${OVERLAY_STAGE_SCRATCH}/evidence"',
        'install -m 0644 -o root -g root -- "${result}" "${RUN_DIRECTORY}/runtime-staging.json"',
        'remove_overlay_stage_scratch',
        'CLEANUP_STATUS == 0 && CHECKSUM_STATUS == 0 &&',
        'FREEZE_STATUS == 0 && PUBLICATION_STATUS == 0',
    ):
        assert required in source
    assert 'chown -R "${PROJECT_USER}:${PROJECT_GROUP}" -- "${RUN_DIRECTORY}"' not in source
    assert '"${SCRIPT_DIR}/stage_runtime_overlay.sh" --apply' not in source


@pytest.mark.parametrize('invocation_mode', ('explicit', 'exit-handler'))
@pytest.mark.parametrize(
    ('signal_name', 'expected_status'),
    (('HUP', 129), ('INT', 130), ('TERM', 143)),
)
def test_authoritative_finalizer_defers_signals_until_terminal_state(
    tmp_path: Path,
    invocation_mode: str,
    signal_name: str,
    expected_status: int,
) -> None:
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    signal_functions = source[
        source.index('latch_process_publication_signal() {') : source.index('stop_owned_group() {')
    ].replace(
        'end_process_publication() {',
        'original_end_process_publication() {',
        1,
    )
    finalizer = source[source.index('finalize_authoritative_run() {') : source.index('on_exit() {')]
    probe = tmp_path / 'finalizer-signal-probe.sh'
    trace = tmp_path / 'trace.txt'
    probe.write_text(
        f"""#!/usr/bin/env bash
set -Eeuo pipefail
{signal_functions}
{finalizer}
SIGNAL_NAME="$1"
TRACE_FILE="$2"
INVOCATION_MODE="$3"
PENDING_PUBLICATION_SIGNAL=0
PUBLICATION_SIGNAL_TRAPS_CLEARED=0
FINALIZATION_STATE=0
FINALIZATION_STATUS=0
FINALIZED=0
CLEANUP_STATUS=0
COMPOSE_STATUS=0
CHECKSUM_STATUS=0
FREEZE_STATUS=0
PUBLICATION_STATUS=0
LIVE_REMOVAL_STATUS=0
RUN_DIRECTORY=/run/robotest-phase4-verifier/test-run
PUBLIC_RUN_DIRECTORY=/tmp/test-run
HELPER=phase4_acceptance.py
record() {{ printf '%s\n' "$1" >>"${{TRACE_FILE}}"; }}
log() {{ :; }}
finalize_live_cleanup() {{
  record cleanup
  kill "-${{SIGNAL_NAME}}" "$$"
  record cleanup-after-signal
}}
python3() {{
  case " $* " in
    *' compose '*) record compose ;;
    *' checksums '*) record checksums ;;
    *) return 97 ;;
  esac
}}
freeze_live_evidence() {{ record freeze; }}
publish_live_evidence() {{ record publish; }}
remove_published_live_evidence() {{ record remove; }}
end_process_publication() {{
  record "terminal-state:${{FINALIZATION_STATE}}:${{FINALIZED}}"
  original_end_process_publication
}}
if [[ "${{INVOCATION_MODE}}" == explicit ]]; then
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM
else
  trap - EXIT HUP INT TERM
fi
finalize_authoritative_run
record unexpected-return
exit 99
""",
        encoding='utf-8',
    )

    completed = subprocess.run(
        ['/bin/bash', str(probe), signal_name, str(trace), invocation_mode],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == expected_status, completed.stderr
    assert trace.read_text(encoding='utf-8').splitlines() == [
        'cleanup',
        'cleanup-after-signal',
        'compose',
        'checksums',
        'freeze',
        'publish',
        'remove',
        'terminal-state:2:1',
    ]


@pytest.mark.parametrize('invocation_mode', ('explicit', 'exit-handler'))
@pytest.mark.parametrize(
    ('signal_name', 'expected_status'),
    (('HUP', 129), ('INT', 130), ('TERM', 143)),
)
def test_publication_signal_at_end_boundary_is_not_swallowed(
    tmp_path: Path,
    invocation_mode: str,
    signal_name: str,
    expected_status: int,
) -> None:
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    signal_functions = source[
        source.index('latch_process_publication_signal() {') : source.index('stop_owned_group() {')
    ]
    needle = 'end_process_publication() {\n  local pending_signal\n'
    assert signal_functions.count(needle) == 1
    signal_functions = signal_functions.replace(
        needle,
        needle + '  kill "-${SIGNAL_NAME}" "$$"\n',
        1,
    )
    probe = tmp_path / 'publication-boundary-probe.sh'
    probe.write_text(
        f"""#!/usr/bin/env bash
set -Eeuo pipefail
{signal_functions}
SIGNAL_NAME="$1"
INVOCATION_MODE="$2"
PENDING_PUBLICATION_SIGNAL=0
PUBLICATION_SIGNAL_TRAPS_CLEARED=0
if [[ "${{INVOCATION_MODE}}" == explicit ]]; then
  trap 'exit 129' HUP
  trap 'exit 130' INT
  trap 'exit 143' TERM
else
  trap - EXIT HUP INT TERM
fi
begin_process_publication
end_process_publication
exit 99
""",
        encoding='utf-8',
    )

    completed = subprocess.run(
        ['/bin/bash', str(probe), signal_name, invocation_mode],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == expected_status, completed.stderr


@pytest.mark.parametrize(('mode', 'expected_status'), (('churn', 0), ('never', 1)))
def test_runtime_ownership_capture_retries_until_stable(
    tmp_path: Path,
    mode: str,
    expected_status: int,
) -> None:
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    function = source[
        source.index('capture_stable_runtime_ownership() {') : source.index('record_timeline() {')
    ]
    probe = tmp_path / 'ownership-retry-probe.sh'
    run_directory = tmp_path / 'run'
    run_directory.mkdir()
    probe.write_text(
        f"""#!/usr/bin/env bash
set -Eeuo pipefail
{function}
MODE="$1"
RUN_DIRECTORY="$2"
ROS_DOMAIN_ID=177
GZ_PARTITION=robotest_phase4_test
HELPER=phase4_acceptance.py
ATTEMPT=0
log() {{ printf '%s\n' "$*"; }}
sleep() {{ :; }}
python3() {{
  if [[ "$1" == - ]]; then
    printf '1 3\n'
    return 0
  fi
  if [[ " $* " == *' validate-runtime-ownership '* ]]; then
    ((ATTEMPT += 1))
    if [[ "${{MODE}}" == never || "${{ATTEMPT}}" -lt 2 ]]; then
      return 1
    fi
    return 0
  fi
  local output=''
  while (($#)); do
    if [[ "$1" == --output ]]; then
      output="$2"
      shift 2
    else
      shift
    fi
  done
  [[ -n "${{output}}" ]] || return 2
  printf '{{}}\n' >"${{output}}"
}}
set +e
counts="$(capture_stable_runtime_ownership \
  initial 10 100 100 \
  "${{RUN_DIRECTORY}}/owners.json" \
  "${{RUN_DIRECTORY}}/affinity.json")"
status=$?
set -e
printf 'status=%s counts=%s\n' "${{status}}" "${{counts}}"
exit "${{status}}"
""",
        encoding='utf-8',
    )
    completed = subprocess.run(
        ['/bin/bash', str(probe), mode, str(run_directory)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == expected_status, completed.stderr
    if mode == 'churn':
        assert 'status=0 counts=1 3' in completed.stdout
        assert (run_directory / 'owners.json').is_file()
        assert (run_directory / 'affinity.json').is_file()
    else:
        assert not (run_directory / 'owners.json').exists()
        assert not (run_directory / 'affinity.json').exists()


def test_root_static_worker_is_unprivileged_sanitized_and_rechecked() -> None:
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    worker = source[
        source.index('run_static_checks_as_project_user()') : source.index(
            'remove_overlay_stage_scratch()'
        )
    ]
    assert 'runuser -u "${PROJECT_USER}" -- env -i' in worker
    for variable in (
        'HOME=',
        'TMPDIR=',
        'XDG_CACHE_HOME=',
        'GOCACHE=',
        'GOTMPDIR=',
        'PYTHONPYCACHEPREFIX=',
        'LANG=C.UTF-8',
        'PATH=/usr/local/go/bin:/usr/bin:/bin',
    ):
        assert variable in worker
    assert 'bash "${SCRIPT_DIR}/verify_phase4.sh"' in worker
    assert 'Static worker output contains a symbolic link.' in worker
    assert 'Static worker output contains a special file.' in worker
    assert 'Static worker output contains a multiply linked file.' in worker
    assert worker.index('runuser -u') < worker.index(
        'python3 "${HELPER}" import-package-source-rebuild'
    )
    main = source[source.index('PROJECT_USER="$(stat') :]
    root_static_rejection = main.index('Static-only verification refuses root')
    first_resolve = main.index('resolve_package_directory', root_static_rejection)
    early_clean = main.index('verify_apply_source_clean')
    static_dispatch = main.index('run_static_checks_as_project_user')
    final_clean = main.index('verify_apply_source_clean', early_clean + 1)
    live_lock = main.index('exec 9>"${LOCK_FILE}"')
    assert root_static_rejection < first_resolve
    assert early_clean < static_dispatch < final_clean < live_lock


@pytest.mark.parametrize(
    ('index_mode', 'expected_status'),
    (('clean', 0), ('assume-unchanged', 1), ('skip-worktree', 1)),
)
def test_apply_clean_gate_rejects_hidden_git_index_flags(
    tmp_path: Path,
    index_mode: str,
    expected_status: int,
) -> None:
    repository = tmp_path / 'repository'
    repository.mkdir()
    subprocess.run(['git', 'init', '-q', str(repository)], check=True)
    subprocess.run(['git', '-C', str(repository), 'config', 'user.name', 'Phase4 Test'], check=True)
    subprocess.run(
        ['git', '-C', str(repository), 'config', 'user.email', 'phase4@example.invalid'],
        check=True,
    )
    tracked = repository / 'tracked.txt'
    tracked.write_text('committed\n', encoding='utf-8')
    subprocess.run(['git', '-C', str(repository), 'add', 'tracked.txt'], check=True)
    subprocess.run(['git', '-C', str(repository), 'commit', '-qm', 'fixture'], check=True)
    if index_mode != 'clean':
        subprocess.run(
            ['git', '-C', str(repository), 'update-index', f'--{index_mode}', 'tracked.txt'],
            check=True,
        )
        tracked.write_text('hidden mutation\n', encoding='utf-8')

    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    function = source[
        source.index('sanitized_git() {') : source.index(
            'validated_remove_package_rebuild_scratch() {'
        )
    ]
    probe = tmp_path / 'clean-gate-probe.sh'
    probe.write_text(
        f"""#!/usr/bin/env bash
set -Eeuo pipefail
{function}
die() {{ printf '%s\n' "$*" >&2; exit 1; }}
PROJECT_ROOT="$1"
APPLY_SOURCE_HEAD=""
verify_apply_source_clean
""",
        encoding='utf-8',
    )
    completed = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == expected_status, completed.stderr


def test_sanitized_git_ignores_fsmonitor_and_inherited_repository_redirects(
    tmp_path: Path,
) -> None:
    repository = tmp_path / 'repository'
    decoy = tmp_path / 'decoy'
    for path in (repository, decoy):
        path.mkdir()
        subprocess.run(['git', 'init', '-q', str(path)], check=True)
        subprocess.run(['git', '-C', str(path), 'config', 'user.name', 'Phase4 Test'], check=True)
        subprocess.run(
            ['git', '-C', str(path), 'config', 'user.email', 'phase4@example.invalid'],
            check=True,
        )
        (path / 'tracked.txt').write_text(f'{path.name}\n', encoding='utf-8')
        subprocess.run(['git', '-C', str(path), 'add', 'tracked.txt'], check=True)
        subprocess.run(['git', '-C', str(path), 'commit', '-qm', 'fixture'], check=True)
    marker = tmp_path / 'fsmonitor-executed'
    monitor = tmp_path / 'fsmonitor.sh'
    monitor.write_text(
        f'#!/bin/sh\ntouch "{marker}"\nprintf "\n"\n',
        encoding='utf-8',
    )
    monitor.chmod(0o755)
    subprocess.run(
        ['git', '-C', str(repository), 'config', 'core.fsmonitor', str(monitor)],
        check=True,
    )
    source = (REPOSITORY / 'scripts/verify_phase4.sh').read_text(encoding='utf-8')
    functions = source[
        source.index('sanitized_git() {') : source.index(
            'validated_remove_package_rebuild_scratch() {'
        )
    ]
    assert 'GIT_NO_REPLACE_OBJECTS=1' in functions
    probe = tmp_path / 'sanitized-git-probe.sh'
    probe.write_text(
        f"""#!/usr/bin/env bash
set -Eeuo pipefail
{functions}
die() {{ printf '%s\n' "$*" >&2; exit 1; }}
PROJECT_ROOT="$1"
APPLY_SOURCE_HEAD=""
verify_apply_source_clean
""",
        encoding='utf-8',
    )
    hostile_environment = {
        **os.environ,
        'GIT_DIR': str(decoy / '.git'),
        'GIT_WORK_TREE': str(decoy),
        'GIT_INDEX_FILE': str(decoy / '.git/index'),
    }
    clean = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert clean.returncode == 0, clean.stderr
    assert not marker.exists()

    filter_marker = tmp_path / 'clean-filter-executed'
    clean_filter = tmp_path / 'clean-filter.sh'
    clean_filter.write_text(
        f'#!/bin/sh\ntouch "{filter_marker}"\nprintf "repository\\n"\n',
        encoding='utf-8',
    )
    clean_filter.chmod(0o755)
    info_attributes = repository / '.git/info/attributes'
    info_attributes.write_text('tracked.txt filter=mask\n', encoding='utf-8')
    subprocess.run(
        ['git', '-C', str(repository), 'config', 'filter.mask.clean', str(clean_filter)],
        check=True,
    )
    (repository / 'tracked.txt').write_text('masked source\n', encoding='utf-8')
    attributes_blocked = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert attributes_blocked.returncode != 0
    assert not filter_marker.exists()
    info_attributes.unlink()
    subprocess.run(
        ['git', '-C', str(repository), 'config', '--unset-all', 'filter.mask.clean'],
        check=True,
    )
    subprocess.run(
        ['git', '-C', str(repository), 'reset', '--hard', '-q', 'HEAD'],
        check=True,
    )
    marker.unlink(missing_ok=True)

    ignored_source = repository / 'src/hidden.py'
    ignored_source.parent.mkdir()
    ignored_source.write_text('hostile source\n', encoding='utf-8')
    (repository / '.git/info/exclude').write_text('/src/\n', encoding='utf-8')
    ignored_blocked = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert ignored_blocked.returncode != 0
    ignored_source.unlink()
    cache_directory = repository / 'src/pkg/__pycache__'
    cache_directory.mkdir(parents=True)
    (cache_directory / 'cached.pyc').write_bytes(b'cache-only\n')
    allowed_cache = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert allowed_cache.returncode == 0, allowed_cache.stderr
    assert not marker.exists()

    for relative in (
        'src/pkg/build/evil.py',
        'src/pkg/.mypy_cache/evil.py',
        'packaging/debian/build/evil.py',
        'supervisor/internal/__pycache__/evil.pyc',
    ):
        forbidden = repository / relative
        forbidden.parent.mkdir(parents=True, exist_ok=True)
        forbidden.write_text('forbidden ignored input\n', encoding='utf-8')
        (repository / '.git/info/exclude').write_text(
            f'/{relative}\n/src/pkg/__pycache__/\n',
            encoding='utf-8',
        )
        forbidden_result = subprocess.run(
            ['/bin/bash', str(probe), str(repository)],
            check=False,
            capture_output=True,
            text=True,
            env=hostile_environment,
        )
        assert forbidden_result.returncode != 0, relative
        forbidden.unlink()
    (repository / '.git/info/exclude').write_text('/src/\n', encoding='utf-8')

    (repository / 'tracked.txt').write_text('real dirty source\n', encoding='utf-8')
    dirty = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert dirty.returncode != 0
    assert not marker.exists()

    subprocess.run(
        ['git', '-C', str(repository), 'reset', '--hard', '-q', 'HEAD'],
        check=True,
    )
    original_commit = subprocess.run(
        ['git', '-C', str(repository), 'rev-parse', 'HEAD'],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    (repository / 'tracked.txt').write_text('replacement source\n', encoding='utf-8')
    subprocess.run(
        ['git', '-C', str(repository), 'commit', '-qam', 'replacement tree'],
        check=True,
    )
    replacement_commit = subprocess.run(
        ['git', '-C', str(repository), 'rev-parse', 'HEAD'],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    subprocess.run(
        ['git', '-C', str(repository), 'reset', '--hard', '-q', original_commit],
        check=True,
    )
    subprocess.run(
        ['git', '-C', str(repository), 'replace', original_commit, replacement_commit],
        check=True,
    )
    subprocess.run(
        ['git', '-C', str(repository), 'reset', '--hard', '-q', original_commit],
        check=True,
    )
    marker.unlink(missing_ok=True)
    assert (repository / 'tracked.txt').read_text(encoding='utf-8') == 'replacement source\n'
    replaced = subprocess.run(
        ['/bin/bash', str(probe), str(repository)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert replaced.returncode != 0
    assert not marker.exists()


def test_overlay_stager_uses_the_same_sanitized_git_boundary() -> None:
    source = (REPOSITORY / 'scripts/stage_runtime_overlay.sh').read_text(encoding='utf-8')
    assert 'sanitized_git() {' in source
    assert 'GIT_CONFIG_GLOBAL=/dev/null' in source
    assert 'GIT_ATTR_NOSYSTEM=1' in source
    assert 'GIT_NO_REPLACE_OBJECTS=1' in source
    assert '--git-dir="${PROJECT_ROOT}/.git"' in source
    assert '-c core.attributesFile=/dev/null' in source
    assert '-c core.fileMode=true' in source
    assert '-c core.fsmonitor=false' in source
    assert 'verify_ignored_source_clean' in source
    assert 'git -c safe.directory=' not in source
    assert 'git_commit="$(sanitized_git rev-parse HEAD)"' in source
    allow_function = source[
        source.index('ignored_source_path_is_allowed() {') : source.index(
            'verify_ignored_source_clean() {'
        )
    ]
    allowed = subprocess.run(
        [
            '/bin/bash',
            '-c',
            f'{allow_function}\nignored_source_path_is_allowed "$1"',
            'probe',
            'src/pkg/__pycache__/cached.pyc',
        ],
        check=False,
    )
    forbidden = subprocess.run(
        [
            '/bin/bash',
            '-c',
            f'{allow_function}\nignored_source_path_is_allowed "$1"',
            'probe',
            'src/pkg/build/evil.py',
        ],
        check=False,
    )
    assert allowed.returncode == 0
    assert forbidden.returncode != 0


def _run_embedded_python(
    script: Path,
    function_name: str,
    arguments: list[str],
) -> bytes:
    source = script.read_text(encoding='utf-8')
    function_start = source.index(f'{function_name}() {{')
    body_start = source.index("<<'PY'\n", function_start) + len("<<'PY'\n")
    body_end = source.index('\nPY\n', body_start)
    completed = subprocess.run(
        [sys.executable, '-', *arguments],
        input=source[body_start:body_end],
        check=True,
        capture_output=True,
        text=True,
    )
    return completed.stdout.encode()


def test_package_and_overlay_manifest_producers_emit_canonical_replay_bytes(
    tmp_path: Path,
) -> None:
    package_payload = _run_embedded_python(
        REPOSITORY / 'scripts/build_debian_package.sh',
        'write_source_manifest',
        ['repository', str(REPOSITORY)],
    )
    package_manifest = json.loads(package_payload)
    assert package_payload == canonical_json_bytes(package_manifest)
    assert package_manifest == repository_package_manifest(REPOSITORY)

    stage_script = REPOSITORY / 'scripts/stage_runtime_overlay.sh'
    source_payload = _run_embedded_python(
        stage_script,
        'write_source_manifest',
        [str(REPOSITORY)],
    )
    source_manifest = json.loads(source_payload)
    assert source_payload == canonical_json_bytes(source_manifest)
    assert source_manifest == runtime_source_manifest(REPOSITORY)

    install_root = tmp_path / 'install'
    install_root.mkdir()
    (install_root / 'alpha.txt').write_text('alpha\n', encoding='utf-8')
    nested = install_root / 'nested'
    nested.mkdir()
    (nested / 'beta.txt').write_text('beta\n', encoding='utf-8')
    install_payload = _run_embedded_python(
        stage_script,
        'write_manifest',
        [str(install_root), str(install_root)],
    )
    install_manifest = json.loads(install_payload)
    assert install_payload == canonical_json_bytes(install_manifest)
    assert [row['path'] for row in install_manifest['files']] == [
        'alpha.txt',
        'nested/beta.txt',
    ]


@pytest.mark.parametrize(
    ('script_name', 'root_marker'),
    (
        ('verify_phase4.sh', 'ROBOTEST_PHASE4_VERIFY_ROOT_ENV'),
        ('stage_runtime_overlay.sh', 'ROBOTEST_PHASE4_STAGE_ROOT_ENV'),
    ),
)
def test_privileged_script_entry_reexecs_with_an_exact_environment(
    tmp_path: Path,
    script_name: str,
    root_marker: str,
) -> None:
    source = (REPOSITORY / 'scripts' / script_name).read_text(encoding='utf-8')
    assert source.startswith('#!/bin/bash -p\n')
    guard_end = source.index('set -Eeuo pipefail') + len('set -Eeuo pipefail')
    guard = source[:guard_end]
    assert 'exec /usr/bin/env -i' in guard
    assert 'HOME=/root' in guard
    assert 'LANG=C.UTF-8' in guard
    assert 'LC_ALL=C.UTF-8' in guard
    assert 'PATH=/usr/sbin:/usr/bin:/sbin:/bin' in guard
    assert 'PYTHONDONTWRITEBYTECODE=1' in guard
    assert 'PYTHONNOUSERSITE=1' in guard
    assert f'{root_marker}=1' in guard
    assert '/bin/bash -p -- "$0" "$@"' in guard

    captured_environment = tmp_path / f'{script_name}.environment'
    pollution_marker = tmp_path / f'{script_name}.bash-env-ran'
    bash_environment = tmp_path / f'{script_name}.bash-env'
    bash_environment.write_text(
        f'#!/bin/bash\n/usr/bin/touch -- "{pollution_marker}"\n',
        encoding='utf-8',
    )
    probe = tmp_path / f'{script_name}.probe'
    transformed_guard = guard.replace('if ((EUID == 0)); then', 'if ((1)); then', 1)
    probe.write_text(
        f'{transformed_guard}\n/usr/bin/env -0 >"$1"\n',
        encoding='utf-8',
    )
    probe.chmod(0o755)
    hostile_environment = {
        **os.environ,
        'BASH_ENV': str(bash_environment),
        'DPKG_ROOT': str(tmp_path / 'dpkg-root'),
        'GIT_DIR': str(tmp_path / 'decoy.git'),
        'HTTPS_PROXY': 'http://127.0.0.1:1',
        'PATH': str(tmp_path),
        'PYTHONPATH': str(tmp_path / 'python'),
        'SYSTEMD_EDITOR': str(tmp_path / 'editor'),
    }
    hostile_environment.pop('ROBOTEST_PHASE4_VERIFY_ROOT_ENV', None)
    hostile_environment.pop('ROBOTEST_PHASE4_STAGE_ROOT_ENV', None)
    completed = subprocess.run(
        [str(probe), str(captured_environment)],
        check=False,
        capture_output=True,
        text=True,
        env=hostile_environment,
    )
    assert completed.returncode == 0, completed.stderr
    assert not pollution_marker.exists()
    entries = captured_environment.read_bytes().rstrip(b'\0').split(b'\0')
    captured = dict(entry.decode().split('=', 1) for entry in entries)
    assert set(captured) == {
        'HOME',
        'LANG',
        'LC_ALL',
        'PATH',
        'PWD',
        'PYTHONDONTWRITEBYTECODE',
        'PYTHONNOUSERSITE',
        root_marker,
        'SHLVL',
        '_',
    }
    assert captured['HOME'] == '/root'
    assert captured['PATH'] == '/usr/sbin:/usr/bin:/sbin:/bin'
    assert captured[root_marker] == '1'

    captured_environment.unlink()
    forged_marker_environment = {
        'HOME': '/root',
        'LANG': 'C.UTF-8',
        'LC_ALL': 'C.UTF-8',
        'PATH': '/usr/sbin:/usr/bin:/sbin:/bin',
        'PYTHONDONTWRITEBYTECODE': '1',
        'PYTHONNOUSERSITE': '1',
        root_marker: '1',
        'PYTHONPATH': str(tmp_path / 'hostile-python'),
    }
    forged_marker = subprocess.run(
        [str(probe), str(captured_environment)],
        check=False,
        capture_output=True,
        text=True,
        env=forged_marker_environment,
    )
    assert forged_marker.returncode != 0
    assert not captured_environment.exists()


def _publication_fixture(tmp_path: Path) -> tuple[Path, Path, Path, Path]:
    run_id = 'phase4-20260826T000000Z-777'
    live_root = tmp_path / 'live'
    source = live_root / run_id
    repository = tmp_path / 'repository'
    phase4_root = repository / 'artifacts/evidence/phase4'
    live_root.mkdir(mode=0o700)
    source.mkdir(mode=0o700)
    phase4_root.mkdir(parents=True)
    (source / '.phase4-live-owned').write_text(f'{run_id}\n', encoding='utf-8')
    (source / 'raw.txt').write_bytes(b'root-frozen-evidence\n')
    nested = source / 'nested'
    nested.mkdir()
    (nested / 'value.json').write_bytes(b'{"schema_version":1}\n')
    write_checksums(source)
    for path in source.rglob('*'):
        if path.is_file():
            path.chmod(0o440)
    nested.chmod(0o550)
    source.chmod(0o550)
    live_root.chmod(0o710)
    destination = phase4_root / 'runs' / run_id
    return live_root, source, repository, destination


def _publish_fixture(
    live_root: Path,
    source: Path,
    repository: Path,
    destination: Path,
    **kwargs: object,
) -> dict:
    return publish_run_evidence(
        source,
        destination,
        repository,
        live_root=live_root,
        expected_source_uid=os.geteuid(),
        require_unprivileged=False,
        **kwargs,
    )


def test_unprivileged_publication_is_byte_exact_and_no_replace(tmp_path: Path) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    result = _publish_fixture(
        live_root,
        source,
        repository,
        destination,
        temporary_name_factory=lambda: 'deterministic',
    )
    assert result['destination'] == str(destination)
    assert result['file_count'] == 4
    assert destination.is_dir() and not destination.is_symlink()
    source_files = {
        path.relative_to(source).as_posix(): path.read_bytes()
        for path in source.rglob('*')
        if path.is_file()
    }
    destination_files = {
        path.relative_to(destination).as_posix(): path.read_bytes()
        for path in destination.rglob('*')
        if path.is_file()
    }
    assert destination_files == source_files
    assert destination.stat().st_mode & 0o777 == 0o750
    assert all(
        path.stat().st_mode & 0o777 == 0o640 for path in destination.rglob('*') if path.is_file()
    )


def test_unprivileged_publication_preserves_checksummed_fail_evidence(tmp_path: Path) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    source.chmod(0o750)
    for path in source.rglob('*'):
        if path.is_file():
            path.chmod(0o640)
    atomic_write_json(
        source / 'scenario6-result.json',
        {
            'schema_version': 1,
            'verdict': {
                'status': 'FAIL',
                'accepted': False,
                'failure_count': 1,
                'failures': ['deterministic_failure'],
            },
        },
    )
    write_checksums(source)
    for path in source.rglob('*'):
        if path.is_file():
            path.chmod(0o440)
    for path in sorted(source.rglob('*'), reverse=True):
        if path.is_dir():
            path.chmod(0o550)
    source.chmod(0o550)
    _publish_fixture(live_root, source, repository, destination)
    published = json.loads((destination / 'scenario6-result.json').read_text(encoding='utf-8'))
    assert published['verdict']['status'] == 'FAIL'
    manifest = (destination / 'SHA256SUMS').read_text(encoding='ascii')
    assert '  scenario6-result.json\n' in manifest


@pytest.mark.parametrize('mutation', ('file_symlink', 'fifo', 'hardlink', 'source_symlink'))
def test_publication_rejects_linked_or_special_source(
    tmp_path: Path,
    mutation: str,
) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    source.chmod(0o750)
    if mutation == 'file_symlink':
        (source / 'linked').symlink_to(source / 'raw.txt')
    elif mutation == 'fifo':
        os.mkfifo(source / 'special')
    elif mutation == 'hardlink':
        os.link(source / 'raw.txt', source / 'linked')
    else:
        source.chmod(0o550)
        moved = live_root / f'{source.name}.real'
        source.rename(moved)
        source.symlink_to(moved, target_is_directory=True)
    if mutation != 'source_symlink':
        source.chmod(0o550)
    with pytest.raises(EvidenceError):
        _publish_fixture(live_root, source, repository, destination)
    assert not destination.exists() and not destination.is_symlink()


@pytest.mark.parametrize('target_kind', ('file', 'directory', 'symlink', 'fifo'))
def test_publication_rejects_preexisting_public_destination_without_overwrite(
    tmp_path: Path,
    target_kind: str,
) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    destination.parent.mkdir()
    outside = tmp_path / 'outside.txt'
    outside.write_text('unchanged\n', encoding='utf-8')
    if target_kind == 'file':
        destination.write_text('existing\n', encoding='utf-8')
    elif target_kind == 'directory':
        destination.mkdir()
        (destination / 'sentinel').write_text('existing\n', encoding='utf-8')
    elif target_kind == 'symlink':
        destination.symlink_to(outside)
    else:
        os.mkfifo(destination)
    with pytest.raises(EvidenceError, match='already exists'):
        _publish_fixture(live_root, source, repository, destination)
    assert outside.read_text(encoding='utf-8') == 'unchanged\n'
    if target_kind == 'file':
        assert destination.read_text(encoding='utf-8') == 'existing\n'
    elif target_kind == 'directory':
        assert (destination / 'sentinel').read_text(encoding='utf-8') == 'existing\n'


def test_publication_rejects_symlink_public_parent(tmp_path: Path) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    outside = tmp_path / 'outside'
    outside.mkdir()
    destination.parent.symlink_to(outside, target_is_directory=True)
    with pytest.raises(EvidenceError, match='non-symlink directory'):
        _publish_fixture(live_root, source, repository, destination)
    assert list(outside.iterdir()) == []


def test_publication_rejects_precreated_random_sibling(tmp_path: Path) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    destination.parent.mkdir()
    collision = destination.parent / f'.{source.name}.publish-fixed'
    collision.mkdir()
    sentinel = collision / 'sentinel'
    sentinel.write_text('existing\n', encoding='utf-8')
    with pytest.raises(EvidenceError, match='temporary directory already exists'):
        _publish_fixture(
            live_root,
            source,
            repository,
            destination,
            temporary_name_factory=lambda: 'fixed',
        )
    assert sentinel.read_text(encoding='utf-8') == 'existing\n'


def test_publication_no_replace_closes_destination_creation_race(tmp_path: Path) -> None:
    live_root, source, repository, destination = _publication_fixture(tmp_path)
    outside = tmp_path / 'outside.txt'
    outside.write_text('unchanged\n', encoding='utf-8')

    def race_create(path: Path) -> None:
        path.symlink_to(outside)

    with pytest.raises(EvidenceError, match='appeared before no-replace rename'):
        _publish_fixture(
            live_root,
            source,
            repository,
            destination,
            temporary_name_factory=lambda: 'race',
            before_rename=race_create,
        )
    assert destination.is_symlink()
    assert outside.read_text(encoding='utf-8') == 'unchanged\n'
    assert source.is_dir()
    assert (source / '.phase4-live-owned').read_text(encoding='utf-8') == f'{source.name}\n'


def test_package_candidate_rejects_tamper_with_regenerated_local_checksums(
    tmp_path: Path,
) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    assert (
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        )['verdict']
        == 'PASS'
    )
    with upgrade_package.open('ab') as target:
        target.write(b'tamper-after-build\n')
    _write_package_sha256sums(package_directory / 'build-a')
    with pytest.raises(EvidenceError):
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        )


def test_package_candidate_rejects_rebound_baseline_sidecar(tmp_path: Path) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    sidecar_path = Path(f'{baseline_package}.fixture.json')
    sidecar = json.loads(sidecar_path.read_text(encoding='utf-8'))
    sidecar['fixture_package']['sha256'] = '0' * 64
    atomic_write_json(sidecar_path, sidecar)
    with pytest.raises(EvidenceError, match='does not bind'):
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        )


def test_package_candidate_rejects_tampered_reproducibility_json(tmp_path: Path) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    evidence_path = package_directory / 'reproducibility.json'
    evidence = json.loads(evidence_path.read_text(encoding='utf-8'))
    evidence['files'][0]['normalized_sha256'] = '0' * 64
    atomic_write_json(evidence_path, evidence)
    with pytest.raises(EvidenceError, match='does not reconcile'):
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        )


def test_package_candidate_version_must_match_repository_changelog(tmp_path: Path) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    repository = tmp_path / 'repository'
    changelog = repository / 'packaging/debian/changelog'
    changelog.parent.mkdir(parents=True)
    changelog.write_text(
        (REPOSITORY / 'packaging/debian/changelog')
        .read_text(encoding='utf-8')
        .replace('(0.1.1)', '(9.9.9)', 1),
        encoding='utf-8',
    )
    assert repository_changelog_version(REPOSITORY) == '0.1.1'
    with pytest.raises(EvidenceError, match=r'does not match.*changelog'):
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            repository,
        )


def test_package_candidate_rejects_version_drift_in_generated_metadata(
    tmp_path: Path,
) -> None:
    package_directory, upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    for build_name in ('build-a', 'build-b'):
        build = package_directory / build_name
        buildinfo = build / 'robotest-supervisor_0.1.1_amd64.buildinfo'
        buildinfo.write_text(
            buildinfo.read_text(encoding='utf-8').replace('Version: 0.1.1', 'Version: 9.9.9'),
            encoding='utf-8',
        )
        _write_package_sha256sums(build)
    with pytest.raises(EvidenceError, match='wrong Version field'):
        verify_package_candidate(
            package_directory,
            upgrade_package,
            baseline_package,
            REPOSITORY,
        )


def test_package_candidate_rejects_consistently_renamed_binary(tmp_path: Path) -> None:
    package_directory, _upgrade_package, baseline_package = _build_reproducible_package_fixture(
        tmp_path
    )
    wrong_name = 'robotest-supervisor_9.9.9_amd64.deb'
    for build_name in ('build-a', 'build-b'):
        build = package_directory / build_name
        (build / 'robotest-supervisor_0.1.1_amd64.deb').rename(build / wrong_name)
    with pytest.raises(EvidenceError, match='filenames do not match'):
        verify_package_candidate(
            package_directory,
            package_directory / 'build-a' / wrong_name,
            baseline_package,
            REPOSITORY,
        )


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
    duplicate = copy.deepcopy(
        next(event for event in events if event['kind'] == 'restart_scheduled')
    )
    duplicate['steady_wall_ns'] = 2_300_000_000
    events.append(duplicate)
    events.sort(key=lambda event: (event['steady_wall_ns'], event['sequence']))
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


@pytest.mark.parametrize(
    'mutation',
    ('extra-field', 'duplicate-row', 'noncanonical-bytes', 'duplicate-object-key'),
)
def test_package_source_binding_requires_exact_canonical_document(
    tmp_path: Path,
    mutation: str,
) -> None:
    candidate = tmp_path / 'SOURCE-MANIFEST.json'
    value = repository_package_manifest(REPOSITORY)
    atomic_write_json(candidate, value)
    if mutation == 'extra-field':
        value['extra'] = True
        atomic_write_json(candidate, value)
    elif mutation == 'duplicate-row':
        value['files'].append(copy.deepcopy(value['files'][0]))
        atomic_write_json(candidate, value)
    elif mutation == 'noncanonical-bytes':
        candidate.write_text(json.dumps(value, indent=2) + '\n', encoding='utf-8')
    else:
        payload = candidate.read_text(encoding='utf-8')
        candidate.write_text('{"schema_version":1,' + payload[1:], encoding='utf-8')
    assert package_source_binding(REPOSITORY, candidate)['verdict'] == 'FAIL'


@pytest.mark.parametrize('mutation', ('field', 'extra-key'))
def test_evaluator_rejects_nonexact_stored_package_binding(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'package-binding.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'field':
        value['file_count'] += 1
    else:
        value['unexpected'] = True
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'package_source_binding_passed' in result['verdict']['failures']


def test_evaluator_rejects_decoy_candidate_manifest_path(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    decoy = tmp_path / 'decoy-SOURCE-MANIFEST.json'
    atomic_write_json(decoy, repository_package_manifest(REPOSITORY))
    atomic_write_json(
        tmp_path / 'package-binding.json',
        package_source_binding(REPOSITORY, decoy),
    )
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'package_source_binding_passed' in result['verdict']['failures']


def test_evaluator_rejects_tampered_package_source_rebuild_attestation(
    tmp_path: Path,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'package-source-rebuild.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    value['rebuilt_artifacts'][0]['sha256'] = '0' * 64
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'package_source_rebuild_is_exact' in result['verdict']['failures']


def test_rebuild_import_preserves_worker_time_but_rejects_non_time_drift(
    tmp_path: Path,
) -> None:
    package_directory, upgrade_package, _baseline = _build_reproducible_package_fixture(tmp_path)
    rebuilt_directory = package_directory / 'build-b'
    worker = package_source_rebuild_attestation(
        REPOSITORY,
        package_directory,
        upgrade_package,
        rebuilt_directory,
    )
    worker['completed_utc'] = '2026-08-27T00:00:00.000001Z'
    worker_path = tmp_path / 'worker-rebuild.json'
    atomic_write_json(worker_path, worker)
    imported = import_package_source_rebuild_attestation(
        REPOSITORY,
        package_directory,
        upgrade_package,
        rebuilt_directory,
        worker_path,
    )
    assert imported == worker

    worker['build_script_sha256'] = '0' * 64
    atomic_write_json(worker_path, worker)
    with pytest.raises(EvidenceError, match='differs from the root replay'):
        import_package_source_rebuild_attestation(
            REPOSITORY,
            package_directory,
            upgrade_package,
            rebuilt_directory,
            worker_path,
        )


@pytest.mark.parametrize(
    'mutation',
    ('empty-source', 'duplicate-install', 'minimal-provenance', 'wrong-parent', 'release-id'),
)
def test_runtime_staging_replay_rejects_hostile_projection(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    staging_path = tmp_path / 'runtime-staging.json'
    provenance_path = tmp_path / 'overlay-provenance.json'
    staging = json.loads(staging_path.read_text(encoding='utf-8'))
    if mutation == 'empty-source':
        atomic_write_json(
            tmp_path / 'overlay-source-manifest.json',
            {'schema_version': 1, 'files': []},
        )
    elif mutation == 'duplicate-install':
        install_path = tmp_path / 'overlay-install-manifest.json'
        install = json.loads(install_path.read_text(encoding='utf-8'))
        install['files'].append(copy.deepcopy(install['files'][0]))
        atomic_write_json(install_path, install)
        staging['install_manifest_sha256'] = file_sha256(install_path)
        atomic_write_json(staging_path, staging)
        atomic_write_json(provenance_path, staging)
    elif mutation == 'minimal-provenance':
        staging.pop('packages')
        atomic_write_json(staging_path, staging)
        atomic_write_json(provenance_path, staging)
    elif mutation == 'wrong-parent':
        context_path = tmp_path / 'context.json'
        context = json.loads(context_path.read_text(encoding='utf-8'))
        context['active_overlay_target'] = f'/opt/wrong/{staging["release_id"]}'
        atomic_write_json(context_path, context)
    else:
        staging['release_id'] = 'wrong-release-id'
        atomic_write_json(staging_path, staging)
        atomic_write_json(provenance_path, staging)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'runtime_staging_is_bound' in result['verdict']['failures']


@pytest.mark.parametrize(
    'mutation',
    ('empty', 'extra-key', 'duplicate-path', 'wrong-repository', 'wrong-count'),
)
def test_source_snapshot_requires_exact_current_source_projection(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    before_path = tmp_path / 'source-snapshot-before.json'
    value = json.loads(before_path.read_text(encoding='utf-8'))
    if mutation == 'empty':
        value['files'] = []
        value['file_count'] = 0
    elif mutation == 'extra-key':
        value['unexpected'] = True
    elif mutation == 'duplicate-path':
        value['files'].append(copy.deepcopy(value['files'][0]))
        value['file_count'] += 1
    elif mutation == 'wrong-repository':
        value['repository'] = '/tmp/decoy'
    else:
        value['file_count'] += 1
    value['snapshot_sha256'] = hashlib.sha256(canonical_json_bytes(value['files'])).hexdigest()
    atomic_write_json(before_path, value)
    atomic_write_json(tmp_path / 'source-snapshot-after.json', value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'source_snapshot_unchanged_and_self_consistent' in result['verdict']['failures']


def test_generated_claims_and_results_are_excluded_from_source_manifests(
    tmp_path: Path,
) -> None:
    repository = tmp_path / 'repository'
    for name in (
        'config',
        'docs',
        'packaging',
        'scenarios',
        'scripts',
        'src',
        'supervisor',
        'tests',
    ):
        (repository / name).mkdir(parents=True)
    runtime = repository / 'config/runtime.json'
    runtime.write_text('{}\n', encoding='utf-8')
    claim = repository / 'config/release-claims.json'
    claim.write_text('{"generated":1}\n', encoding='utf-8')
    result = repository / 'docs/results/phase-4.md'
    result.parent.mkdir(parents=True)
    result.write_text('generated one\n', encoding='utf-8')
    source_before = source_snapshot(repository)
    runtime_before = runtime_source_manifest(repository)
    claim.write_text('{"generated":2}\n', encoding='utf-8')
    result.write_text('generated two\n', encoding='utf-8')
    assert source_snapshot(repository) == source_before
    assert runtime_source_manifest(repository) == runtime_before
    runtime.write_text('{"changed":true}\n', encoding='utf-8')
    assert source_snapshot(repository) != source_before
    assert runtime_source_manifest(repository) != runtime_before


@pytest.mark.parametrize(
    'missing',
    (
        'initial-status.json',
        'isolation.json',
        'systemd-owners-initial.json',
        'systemd-owners-restored.json',
        'original-group-empty.json',
        'service-final.json',
        'supervisor-status-final.json',
        'systemd-dropin.conf',
        'package-source-rebuild.json',
    ),
)
def test_required_raw_projection_omission_is_rejected(tmp_path: Path, missing: str) -> None:
    _write_pass_fixture(tmp_path)
    (tmp_path / missing).unlink()
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


def test_raw_status_group_and_dropin_mutations_fail_named_checks(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    initial_path = tmp_path / 'initial-status.json'
    initial = json.loads(initial_path.read_text(encoding='utf-8'))
    initial['children'][0]['pid'] = 999
    atomic_write_json(initial_path, initial)
    group_path = tmp_path / 'original-group-empty.json'
    group = json.loads(group_path.read_text(encoding='utf-8'))
    group['pgid'] = 999
    atomic_write_json(group_path, group)
    (tmp_path / 'systemd-dropin.conf').write_text(
        '[Service]\nExecStart=/usr/bin/robotest-supervisor --config /etc/default.json\n',
        encoding='utf-8',
    )
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'supervisor_status_snapshots_are_exact' in result['verdict']['failures']
    assert 'original_process_group_empty_within_5s' in result['verdict']['failures']
    assert 'run_scoped_supervisor_config_is_exact' in result['verdict']['failures']


@pytest.mark.parametrize(
    ('filename', 'event_count'),
    (('initial-status.json', 5), ('ready-restored-status.json', 11)),
)
def test_status_event_count_must_join_exact_capture_boundary(
    tmp_path: Path,
    filename: str,
    event_count: int,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / filename
    value = json.loads(path.read_text(encoding='utf-8'))
    value['event_count'] = event_count
    atomic_write_json(path, value)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'supervisor_status_snapshots_are_exact' in result['verdict']['failures']


@pytest.mark.parametrize('mutation', ('extra', 'bool-schema', 'bool-dropped'))
def test_event_metadata_schema_is_type_exact(tmp_path: Path, mutation: str) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.meta.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'extra':
        value['unexpected'] = True
    elif mutation == 'bool-schema':
        value['schema_version'] = True
    else:
        value['dropped_events'] = False
    atomic_write_json(path, value)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


@pytest.mark.parametrize('mutation', ('extra-top', 'extra-detail', 'bool-schema'))
def test_timeline_schema_is_type_exact(tmp_path: Path, mutation: str) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    if mutation == 'extra-top':
        timeline[0]['unexpected'] = True
    elif mutation == 'extra-detail':
        next(record for record in timeline if record['kind'] == 'initial_ready')['details'][
            'unexpected'
        ] = True
    else:
        timeline[0]['schema_version'] = True
    _write_jsonl(path, timeline)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


@pytest.mark.parametrize(
    'mutation',
    ('extra-top', 'extra-isolation', 'bool-schema', 'descriptor-extra', 'run-id-drift'),
)
def test_context_and_retained_isolation_are_type_exact(tmp_path: Path, mutation: str) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'context.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'extra-top':
        value['unexpected'] = True
    elif mutation == 'extra-isolation':
        value['isolation']['unexpected'] = True
    elif mutation == 'bool-schema':
        value['schema_version'] = True
    elif mutation == 'descriptor-extra':
        value['upgrade_package']['unexpected'] = True
    else:
        value['isolation']['run_id'] = 'phase4-20260826T000000Z-123'
    atomic_write_json(path, value)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


@pytest.mark.parametrize('mutation', ('extra-target', 'extra-lineage', 'broken-parent'))
def test_controller_target_schema_and_lineage_are_exact(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'controller-target.json'
    value = json.loads(path.read_text(encoding='utf-8'))
    if mutation == 'extra-target':
        value['unexpected'] = True
    elif mutation == 'extra-lineage':
        value['lineage'][1]['unexpected'] = True
    else:
        value['lineage'][1]['ppid'] = 999
    atomic_write_json(path, value)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


@pytest.mark.parametrize('mutation', ('waypoint', 'extra-key'))
def test_followup_mission_must_equal_frozen_recovery_mission(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    mission_path = tmp_path / 'followup-mission.json'
    mission = json.loads(mission_path.read_text(encoding='utf-8'))
    if mutation == 'waypoint':
        mission['waypoints'][0]['x'] = 9.0
    else:
        mission['unexpected'] = True
    atomic_write_json(mission_path, mission)
    result_path = tmp_path / 'followup-result.json'
    mission_result = json.loads(result_path.read_text(encoding='utf-8'))
    mission_result['identity']['mission_sha256'] = file_sha256(mission_path)
    write_result_artifacts(
        mission_result,
        result_path,
        tmp_path / 'followup-result.csv',
    )
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'fresh_followup_mission_succeeded' in result['verdict']['failures']


def test_ready_unavailable_marker_may_follow_probe_within_adjacent_bracket(
    tmp_path: Path,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    next(record for record in timeline if record['kind'] == 'ready_unavailable')['monotonic_ns'] = (
        1_600_000_000
    )
    _write_jsonl(path, timeline)
    assert evaluate_run(tmp_path)['verdict']['status'] == 'PASS'


def test_ready_probe_200_then_503_flap_fails_unavailable_interval(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    ready_probes = [record for record in timeline if record['kind'] == 'ready_probe']
    ready_probes[1]['details'] = _http_probe_details('/readyz', 200)
    _write_jsonl(path, timeline)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'readiness_false_throughout_unavailable_interval' in result['verdict']['failures']


@pytest.mark.parametrize('mutation', ('url', 'body', 'hash'))
def test_http_probe_exact_details_reject_tampering(tmp_path: Path, mutation: str) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    probe = next(record for record in timeline if record['kind'] == 'health_probe')
    if mutation == 'url':
        probe['details']['url'] = 'http://127.0.0.1:9080/readyz'
    elif mutation == 'body':
        probe['details']['body'] = 'forged\n'
        probe['details']['body_sha256'] = hashlib.sha256(b'forged\n').hexdigest()
    else:
        probe['details']['body_sha256'] = '0' * 64
    _write_jsonl(path, timeline)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


def test_startup_status_zero_empty_body_can_precede_final_ready_pair(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    startup_health = next(record for record in timeline if record['kind'] == 'startup_health_probe')
    startup_ready = next(record for record in timeline if record['kind'] == 'startup_ready_probe')
    startup_health['details'] = _http_probe_details('/healthz', 0)
    startup_ready['details'] = _http_probe_details('/readyz', 0)
    timeline.extend(
        [
            _timeline_record(
                0,
                'startup_health_probe',
                75_000_000,
                **_http_probe_details('/healthz', 200),
            ),
            _timeline_record(
                0,
                'startup_ready_probe',
                90_000_000,
                **_http_probe_details('/readyz', 200),
            ),
        ]
    )
    timeline.sort(key=lambda record: (record['monotonic_ns'], record['sequence']))
    for sequence, record in enumerate(timeline, 1):
        record['sequence'] = sequence
    _write_jsonl(path, timeline)
    assert evaluate_run(tmp_path)['verdict']['status'] == 'PASS'


def test_startup_final_health_zero_ready_200_is_rejected(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = _load_records(path)
    health = next(record for record in timeline if record['kind'] == 'startup_health_probe')
    health['details'] = _http_probe_details('/healthz', 0)
    _write_jsonl(path, timeline)
    with pytest.raises(EvidenceError, match='do not end ready'):
        evaluate_run(tmp_path)


def test_missing_startup_http_probe_pair_is_rejected(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'timeline.jsonl'
    timeline = [
        record
        for record in _load_records(path)
        if record['kind'] not in {'startup_health_probe', 'startup_ready_probe'}
    ]
    for sequence, record in enumerate(timeline, 1):
        record['sequence'] = sequence
    _write_jsonl(path, timeline)
    with pytest.raises(EvidenceError, match='missing or unbalanced'):
        evaluate_run(tmp_path)


def test_post_observation_restart_event_fails_shutdown_suffix(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.jsonl'
    events = _load_records(path)
    events.append(
        {
            'schema_version': 1,
            'sequence': 999,
            'timestamp_utc': '2026-08-26T00:00:06.84Z',
            'steady_wall_ns': 6_840_000_000,
            'kind': 'restart_scheduled',
            'child': 'robotest-stack',
            'restart_attempt': 2,
            'backoff_ms': 2000,
        }
    )
    events.sort(key=lambda event: (event['steady_wall_ns'], event['sequence']))
    for sequence, event in enumerate(events, 1):
        event['sequence'] = sequence
    _write_jsonl(path, events)
    metadata_path = tmp_path / 'supervisor-events.meta.json'
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    metadata['last_attempted_sequence'] = len(events)
    atomic_write_json(metadata_path, metadata)
    final_path = tmp_path / 'supervisor-status-final.json'
    final = json.loads(final_path.read_text(encoding='utf-8'))
    final['event_count'] = len(events)
    atomic_write_json(final_path, final)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'post_observation_shutdown_is_exact' in result['verdict']['failures']


@pytest.mark.parametrize('mutation', ('missing', 'extra', 'reordered'))
def test_supervisor_startup_event_prefix_is_exact(tmp_path: Path, mutation: str) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.jsonl'
    events = _load_records(path)
    if mutation == 'missing':
        events.pop(2)
    elif mutation == 'extra':
        events.insert(
            3,
            {
                'schema_version': 1,
                'sequence': 0,
                'timestamp_utc': '2026-08-26T00:00:00.3Z',
                'steady_wall_ns': 300_000_000,
                'kind': 'heartbeat_stale',
                'child': 'robotest-stack',
            },
        )
    else:
        child = events[1]
        heartbeat = events[2]
        events[1] = {
            **heartbeat,
            'sequence': child['sequence'],
            'timestamp_utc': child['timestamp_utc'],
            'steady_wall_ns': child['steady_wall_ns'],
        }
        events[2] = {
            **child,
            'sequence': heartbeat['sequence'],
            'timestamp_utc': heartbeat['timestamp_utc'],
            'steady_wall_ns': heartbeat['steady_wall_ns'],
        }
    for sequence, event in enumerate(events, 1):
        event['sequence'] = sequence
    _write_jsonl(path, events)
    _bind_timeline_event_window(tmp_path, events)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'exact_causal_supervisor_event_chain' in result['verdict']['failures']


def test_replacement_heartbeat_event_is_required(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.jsonl'
    events = _load_records(path)
    events = [
        event
        for event in events
        if not (event['kind'] == 'heartbeat_fresh' and event['sequence'] > 4)
    ]
    for sequence, event in enumerate(events, 1):
        event['sequence'] = sequence
    _write_jsonl(path, events)
    _bind_timeline_event_window(tmp_path, events)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'exact_causal_supervisor_event_chain' in result['verdict']['failures']


@pytest.mark.parametrize('mutation', ('decoy-child', 'missing-failure-details'))
def test_supervisor_event_schema_rejects_decoy_child_or_incomplete_exit(
    tmp_path: Path,
    mutation: str,
) -> None:
    _write_pass_fixture(tmp_path)
    path = tmp_path / 'supervisor-events.jsonl'
    events = _load_records(path)
    if mutation == 'decoy-child':
        next(
            event
            for event in events
            if event['kind'] == 'child_started' and event.get('pid') == 300
        )['child'] = 'decoy-child'
    else:
        next(event for event in events if event['kind'] == 'failure_detected').pop('details')
    _write_jsonl(path, events)
    with pytest.raises(EvidenceError):
        evaluate_run(tmp_path)


def test_runtime_staging_rejects_coherently_dirty_source(tmp_path: Path) -> None:
    _write_pass_fixture(tmp_path)
    context_path = tmp_path / 'context.json'
    context = json.loads(context_path.read_text(encoding='utf-8'))
    staging_path = tmp_path / 'runtime-staging.json'
    staging = json.loads(staging_path.read_text(encoding='utf-8'))
    clean_release_id = staging['release_id']
    dirty_release_id = clean_release_id.removesuffix('-false') + '-true'
    staging['git_dirty'] = True
    staging['release_id'] = dirty_release_id
    staging['build_command'] = [
        item.replace(clean_release_id, dirty_release_id) for item in staging['build_command']
    ]
    context['source_git_dirty'] = True
    context['active_overlay_target'] = f'/opt/robotest-lab-releases/{dirty_release_id}'
    atomic_write_json(context_path, context)
    atomic_write_json(staging_path, staging)
    atomic_write_json(tmp_path / 'overlay-provenance.json', staging)
    result = evaluate_run(tmp_path)
    assert result['verdict']['status'] == 'FAIL'
    assert 'runtime_staging_is_bound' in result['verdict']['failures']


def test_stable_runtime_ownership_validator_rejects_churn() -> None:
    isolation = {
        'ros_domain_id': 177,
        'gz_partition': 'robotest_phase4_20260826T000000Z_999',
    }
    affinity = _runtime_affinity_value('initial', 100)
    stable = _systemd_owners_value(isolation, 100)
    validate_stable_runtime_ownership_capture(stable, copy.deepcopy(stable), affinity)
    churned = copy.deepcopy(stable)
    churned['units']['robotest-supervisor.service'][-1] = 999
    with pytest.raises(EvidenceError, match='changed across affinity'):
        validate_stable_runtime_ownership_capture(churned, stable, affinity)


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
