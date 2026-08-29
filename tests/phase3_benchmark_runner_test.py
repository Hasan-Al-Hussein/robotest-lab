# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Process-ownership tests for the Phase 3 benchmark runner."""

from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest

TEST_DIR = Path(__file__).parent


def _load(name: str, filename: str):
    spec = importlib.util.spec_from_file_location(name, TEST_DIR / filename)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


orchestration = _load('phase3_orchestration', 'phase3_orchestration.py')
startup_gate = _load('phase2_startup_gate', 'phase2_startup_gate.py')
profiler = _load('phase3_smoke_host_profiler', 'phase3_smoke_host_profiler.py')
runner = _load('phase3_benchmark_runner', 'phase3_benchmark_runner.py')
runtime_gate = _load('phase3_runtime_gate', 'phase3_runtime_gate.py')
metrics_constants = _load(
    'robotest_metrics_source_constants',
    '../src/robotest_metrics/robotest_metrics/constants.py',
)


def _finalized_launch(
    root: Path,
    *,
    stdout: bytes,
    stderr: bytes,
    logs_within_cap: bool = True,
) -> SimpleNamespace:
    process_dir = root / 'processes'
    process_dir.mkdir(parents=True, exist_ok=True)
    stdout_path = process_dir / 'full_stack.stdout.log'
    stderr_path = process_dir / 'full_stack.stderr.log'
    metadata_path = process_dir / 'full_stack.process.json'
    stdout_path.write_bytes(stdout)
    stderr_path.write_bytes(stderr)
    metadata_path.write_text('{}\n', encoding='utf-8')
    return SimpleNamespace(
        _group_confirmed_empty=True,
        finished_steady_ns=20,
        logs_within_cap=logs_within_cap,
        metadata_path=metadata_path,
        poll=lambda: -15,
        stderr_path=stderr_path,
        stdout_path=stdout_path,
    )


def _write_lifecycle_ready(
    root: Path,
    *,
    node_names: tuple[str, ...] = runner.LIFECYCLE_NODES,
    timeout_node: str = 'amcl',
    attempts: int = 2,
    timed_out_attempts: int = 1,
    watch_pid: int = 1234,
    wall_timeout_s: float = runner.LIFECYCLE_READY_WALL_TIMEOUT_S,
) -> Path:
    path = root / 'lifecycle-ready.json'
    document = {
        'failure': None,
        'namespace': '/robotest',
        'required_state': {'id': 3, 'label': 'active'},
        'states': {
            node_name: {
                'attempts': attempts if node_name == timeout_node else 1,
                'label': 'active',
                'service': f'/robotest/{node_name}/get_state',
                'service_seen': True,
                'state_id': 3,
                'timed_out_attempts': (timed_out_attempts if node_name == timeout_node else 0),
            }
            for node_name in node_names
        },
        'verdict': 'PASS',
        'wall_timeout_s': wall_timeout_s,
        'watch_pid': watch_pid,
    }
    path.write_text(startup_gate.serialize_result(document), encoding='utf-8')
    return path


def _lifecycle_expectations(
    *,
    watch_pid: int = 1234,
    wall_timeout_s: float = runner.LIFECYCLE_READY_WALL_TIMEOUT_S,
    node_names: tuple[str, ...] = runner.LIFECYCLE_NODES,
) -> dict[str, object]:
    return {
        'expected_lifecycle_watch_pid': watch_pid,
        'expected_lifecycle_wall_timeout_s': wall_timeout_s,
        'expected_lifecycle_nodes': node_names,
    }


def test_finalized_component_artifact_gets_one_verified_sidecar(tmp_path: Path) -> None:
    artifact = tmp_path / 'capture.json'
    artifact.write_bytes(b'{"capture_schema_version":1}\n')

    digest = runner._write_existing_artifact_sidecar(artifact)

    assert digest == runner.file_sha256(artifact)
    assert (tmp_path / 'capture.json.sha256').read_text(encoding='ascii') == (
        f'{digest}  capture.json\n'
    )
    assert runner.verify_json_sidecar(artifact) == digest
    with pytest.raises(runner.EvidenceError, match='already exists'):
        runner._write_existing_artifact_sidecar(artifact)
    with pytest.raises(runner.EvidenceError, match='missing or non-regular'):
        runner._write_existing_artifact_sidecar(tmp_path / 'missing.json')


def test_final_launch_log_gate_combines_closed_streams_and_replays_exactly(
    tmp_path: Path,
) -> None:
    launch = _finalized_launch(
        tmp_path,
        stdout=b'full stack healthy',
        stderr=b'stderr healthy\n',
    )
    launch_stopped_utc = '2026-08-27T07:00:00Z'
    scanned_utc = '2026-08-27T07:00:01Z'
    lifecycle_ready = _write_lifecycle_ready(tmp_path)

    evidence = runner._finalize_full_stack_log_gate(
        launch,
        tmp_path,
        launch_stopped_utc=launch_stopped_utc,
        scanned_utc=scanned_utc,
        lifecycle_ready_path=lifecycle_ready,
        **_lifecycle_expectations(),
    )

    combined = tmp_path / runner.FULL_STACK_COMBINED_LOG_NAME
    gate = tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME
    assert combined.read_bytes() == b'full stack healthy\nstderr healthy\n'
    assert orchestration.load_canonical_json(gate) == evidence
    assert orchestration.verify_json_sidecar(gate) == orchestration.file_sha256(gate)
    assert evidence['verdict'] == 'PASS'
    assert evidence['launch_stopped_utc'] == launch_stopped_utc
    assert evidence['scanned_utc'] == scanned_utc
    assert evidence['launch_log_path'] == str(combined)
    assert evidence['launch_log_sha256'] == orchestration.file_sha256(combined)
    assert evidence['schema_version'] == 2
    assert evidence['lifecycle_timeout_recovery'] == {
        'lifecycle_ready_path': str(lifecycle_ready.resolve()),
        'lifecycle_ready_sha256': orchestration.file_sha256(lifecycle_ready),
        'lifecycle_ready_size_bytes': lifecycle_ready.stat().st_size,
        'recovered_line_count': 0,
        'recovered_lines': [],
    }
    assert (
        startup_gate.scan_final_launch_log(
            combined,
            evidence['launch_stopped_utc'],
            scanned_utc=evidence['scanned_utc'],
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )
        == evidence
    )


def test_final_launch_log_gate_recovers_only_counter_witnessed_lifecycle_timeout(
    tmp_path: Path,
) -> None:
    timeout_line = (
        '[collision_monitor-15] failed to send response to '
        '/robotest/amcl/get_state (timeout): client will not receive response'
    )
    launch = _finalized_launch(
        tmp_path,
        stdout=f'{timeout_line}\n'.encode(),
        stderr=b'',
    )
    lifecycle_ready = _write_lifecycle_ready(
        tmp_path,
        attempts=3,
        timed_out_attempts=2,
    )

    evidence = runner._finalize_full_stack_log_gate(
        launch,
        tmp_path,
        launch_stopped_utc='2026-08-27T07:00:00Z',
        scanned_utc='2026-08-27T07:00:01Z',
        lifecycle_ready_path=lifecycle_ready,
        **_lifecycle_expectations(),
    )

    assert evidence['verdict'] == 'PASS'
    assert evidence['match_count'] == 1
    assert evidence['matches'] == [
        {
            'line_number': 1,
            'signature_ids': ['dds_response_timeout'],
            'text': timeout_line,
        }
    ]
    recovery = evidence['lifecycle_timeout_recovery']
    assert recovery['recovered_line_count'] == 1
    assert recovery['recovered_lines'] == [
        {
            'line_number': 1,
            'service_name': '/robotest/amcl/get_state',
            'text': timeout_line,
            'timed_out_attempts': 2,
        }
    ]
    assert recovery['lifecycle_ready_path'] == str(lifecycle_ready.resolve())
    assert recovery['lifecycle_ready_sha256'] == orchestration.file_sha256(lifecycle_ready)
    assert recovery['lifecycle_ready_size_bytes'] == lifecycle_ready.stat().st_size


def test_final_launch_log_gate_rejects_lifecycle_timeout_comixed_with_generic_timeout(
    tmp_path: Path,
) -> None:
    line = (
        'failed to send response to /robotest/amcl/get_state (timeout); '
        'failed to send response for unrelated request (timeout)'
    )
    launch = _finalized_launch(tmp_path, stdout=f'{line}\n'.encode(), stderr=b'')
    lifecycle_ready = _write_lifecycle_ready(tmp_path)

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )

    assert captured.value.kind == 'launch_log_signature_detected'
    evidence = orchestration.load_canonical_json(tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME)
    assert evidence['verdict'] == 'FAIL'
    assert evidence['match_count'] == 1
    assert evidence['matches'][0]['signature_ids'] == ['dds_response_timeout']
    assert evidence['lifecycle_timeout_recovery']['recovered_line_count'] == 0
    assert evidence['lifecycle_timeout_recovery']['recovered_lines'] == []


@pytest.mark.parametrize(
    'line',
    (
        'FAILED TO SEND RESPONSE TO /robotest/amcl/get_state (timeout)',
        'failed to send response to /other/amcl/get_state (timeout)',
        'failed to send response to /robotest/amcl/set_parameters (timeout)',
        'failed to send response to /robotest/unlisted_node/get_state (timeout)',
        (
            'failed to send response to /robotest/amcl/get_state (timeout); '
            'failed to send response to /robotest/amcl/get_state (timeout)'
        ),
        'failed to send response to /robotest/amcl/get_state (timeout): fatal condition',
        'Failed to bring up all requested nodes',
    ),
)
def test_final_launch_log_gate_refuses_nonexact_or_comixed_timeout_recovery(
    tmp_path: Path,
    line: str,
) -> None:
    launch = _finalized_launch(tmp_path, stdout=f'{line}\n'.encode(), stderr=b'')
    lifecycle_ready = _write_lifecycle_ready(tmp_path)

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )

    assert captured.value.kind == 'launch_log_signature_detected'
    evidence = orchestration.load_canonical_json(tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME)
    assert evidence['verdict'] == 'FAIL'
    assert evidence['match_count'] == 1
    assert evidence['lifecycle_timeout_recovery']['recovered_line_count'] == 0
    assert evidence['lifecycle_timeout_recovery']['recovered_lines'] == []


def test_final_launch_log_gate_rejects_timeout_lines_in_excess_of_witness(
    tmp_path: Path,
) -> None:
    timeout_line = 'failed to send response to /robotest/amcl/get_state (timeout)'
    launch = _finalized_launch(
        tmp_path,
        stdout=f'{timeout_line}\n{timeout_line}\n'.encode(),
        stderr=b'',
    )
    lifecycle_ready = _write_lifecycle_ready(tmp_path)

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )

    assert captured.value.kind == 'launch_log_signature_detected'
    evidence = orchestration.load_canonical_json(tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME)
    assert evidence['match_count'] == 2
    recovery = evidence['lifecycle_timeout_recovery']
    assert recovery['recovered_line_count'] == 1
    assert recovery['recovered_lines'][0]['line_number'] == 1


def test_final_launch_log_gate_rejects_exact_timeout_without_counter_witness(
    tmp_path: Path,
) -> None:
    timeout_line = 'failed to send response to /robotest/amcl/get_state (timeout)'
    launch = _finalized_launch(tmp_path, stdout=f'{timeout_line}\n'.encode(), stderr=b'')
    lifecycle_ready = _write_lifecycle_ready(
        tmp_path,
        attempts=1,
        timed_out_attempts=0,
    )

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )

    assert captured.value.kind == 'launch_log_signature_detected'
    evidence = orchestration.load_canonical_json(tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME)
    assert evidence['match_count'] == 1
    assert evidence['lifecycle_timeout_recovery']['recovered_line_count'] == 0


@pytest.mark.parametrize(
    'malformation',
    (
        'noncanonical',
        'invalid_counters',
        'wrong_service',
        'failed_verdict',
        'wrong_pid',
        'wrong_timeout',
        'missing_node',
        'extra_node',
    ),
)
def test_final_launch_log_gate_rejects_malformed_lifecycle_recovery_artifact(
    tmp_path: Path,
    malformation: str,
) -> None:
    launch = _finalized_launch(tmp_path, stdout=b'healthy\n', stderr=b'')
    node_names = runner.LIFECYCLE_NODES
    if malformation == 'missing_node':
        node_names = runner.LIFECYCLE_NODES[:-1]
    elif malformation == 'extra_node':
        node_names = (*runner.LIFECYCLE_NODES, 'unrelated_node')
    lifecycle_ready = _write_lifecycle_ready(
        tmp_path,
        node_names=node_names,
        attempts=1 if malformation == 'invalid_counters' else 2,
        timed_out_attempts=1,
        watch_pid=4321 if malformation == 'wrong_pid' else 1234,
        wall_timeout_s=(
            109.0 if malformation == 'wrong_timeout' else runner.LIFECYCLE_READY_WALL_TIMEOUT_S
        ),
    )
    if malformation == 'noncanonical':
        lifecycle_ready.write_text(
            lifecycle_ready.read_text(encoding='utf-8') + ' ',
            encoding='utf-8',
        )
    elif malformation == 'wrong_service':
        lifecycle_ready.write_text(
            lifecycle_ready.read_text(encoding='utf-8').replace(
                '/robotest/amcl/get_state',
                '/robotest/other/get_state',
            ),
            encoding='utf-8',
        )
    elif malformation == 'failed_verdict':
        lifecycle_ready.write_text(
            lifecycle_ready.read_text(encoding='utf-8').replace(
                '"verdict": "PASS"',
                '"verdict": "FAIL"',
            ),
            encoding='utf-8',
        )

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
            lifecycle_ready_path=lifecycle_ready,
            **_lifecycle_expectations(),
        )

    assert captured.value.kind == 'launch_log_recovery_invalid'
    evidence = orchestration.load_canonical_json(tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME)
    assert evidence['verdict'] == 'FAIL'
    assert evidence['match_count'] == 0
    assert evidence['lifecycle_timeout_recovery'] == {
        'lifecycle_ready_path': None,
        'lifecycle_ready_sha256': None,
        'lifecycle_ready_size_bytes': None,
        'recovered_line_count': 0,
        'recovered_lines': [],
    }


@pytest.mark.parametrize('stream', ('stdout', 'stderr'))
@pytest.mark.parametrize(
    ('signature_id', 'line'),
    (
        ('lifecycle_startup_rejection', 'Lifecycle STARTUP rejection'),
        ('nav2_bringup_failure', 'Failed to bring up all requested nodes'),
        ('lifecycle_async_send_request_failure', 'service client: async_send_request failed'),
        (
            'dds_response_timeout',
            'failed to send response to /robotest/amcl/get_state (timeout)',
        ),
        ('fatal_process_signature', 'fatal process condition'),
    ),
)
def test_final_launch_log_gate_rejects_every_signature_from_either_stream(
    tmp_path: Path,
    stream: str,
    signature_id: str,
    line: str,
) -> None:
    stdout = f'{line}\n'.encode() if stream == 'stdout' else b'stdout healthy\n'
    stderr = f'{line}\n'.encode() if stream == 'stderr' else b'stderr healthy\n'
    launch = _finalized_launch(tmp_path, stdout=stdout, stderr=stderr)

    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            launch,
            tmp_path,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
        )

    assert captured.value.stage == 'final_launch_log_gate'
    assert captured.value.kind == 'launch_log_signature_detected'
    gate = tmp_path / runner.FINAL_LAUNCH_LOG_GATE_NAME
    evidence = orchestration.load_canonical_json(gate)
    assert orchestration.verify_json_sidecar(gate) == orchestration.file_sha256(gate)
    assert evidence['verdict'] == 'FAIL'
    assert evidence['match_count'] == 1
    assert evidence['matches'][0]['signature_ids'] == [signature_id]


def test_final_launch_log_gate_rejects_empty_or_incomplete_closed_evidence(
    tmp_path: Path,
) -> None:
    empty_root = tmp_path / 'empty'
    empty_launch = _finalized_launch(empty_root, stdout=b'', stderr=b'')
    with pytest.raises(runner.StageFailure) as captured:
        runner._finalize_full_stack_log_gate(
            empty_launch,
            empty_root,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
        )
    assert captured.value.kind == 'launch_log_invalid'
    empty_gate = empty_root / runner.FINAL_LAUNCH_LOG_GATE_NAME
    assert orchestration.load_canonical_json(empty_gate)['verdict'] == 'FAIL'
    assert orchestration.verify_json_sidecar(empty_gate) == orchestration.file_sha256(empty_gate)

    overflow_root = tmp_path / 'overflow'
    overflow_launch = _finalized_launch(
        overflow_root,
        stdout=b'retained prefix\n',
        stderr=b'',
        logs_within_cap=False,
    )
    with pytest.raises(runner.EvidenceError, match='complete finalized process evidence'):
        runner._finalize_full_stack_log_gate(
            overflow_launch,
            overflow_root,
            launch_stopped_utc='2026-08-27T07:00:00Z',
            scanned_utc='2026-08-27T07:00:01Z',
        )
    assert not (overflow_root / runner.FULL_STACK_COMBINED_LOG_NAME).exists()


def test_combined_launch_log_rejects_symlinked_source(tmp_path: Path) -> None:
    target = tmp_path / 'target.log'
    target.write_text('healthy\n', encoding='utf-8')
    stdout = tmp_path / 'stdout.log'
    stdout.symlink_to(target)
    stderr = tmp_path / 'stderr.log'
    stderr.write_bytes(b'')

    with pytest.raises(startup_gate.GateError, match='regular non-symlink'):
        startup_gate.combined_launch_log_bytes(stdout, stderr)


def test_combined_launch_log_rejects_oversized_or_changing_stream(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    stdout = tmp_path / 'stdout.log'
    stderr = tmp_path / 'stderr.log'
    stdout.write_bytes(b'x' * 17)
    stderr.write_bytes(b'')
    with pytest.raises(startup_gate.GateError, match=r'outside 0\.\.16'):
        startup_gate.combined_launch_log_bytes(
            stdout,
            stderr,
            maximum_stream_bytes=16,
        )

    stdout.write_bytes(b'stable prefix\n')
    original_read_bytes = Path.read_bytes

    def mutate_after_read(path: Path) -> bytes:
        payload = original_read_bytes(path)
        if path == stdout:
            path.write_bytes(payload + b'mutated\n')
        return payload

    monkeypatch.setattr(Path, 'read_bytes', mutate_after_read)
    with pytest.raises(startup_gate.GateError, match='changed while being combined'):
        startup_gate.combined_launch_log_bytes(stdout, stderr)


def test_final_launch_log_scan_rejects_scan_time_before_shutdown(tmp_path: Path) -> None:
    launch_log = tmp_path / 'full-stack-combined.log'
    launch_log.write_text('healthy\n', encoding='utf-8')

    evidence = startup_gate.scan_final_launch_log(
        launch_log,
        '2026-08-27T07:00:01Z',
        scanned_utc='2026-08-27T07:00:00Z',
    )

    assert evidence['verdict'] == 'FAIL'
    assert evidence['failure_kind'] == 'launch_log_invalid'
    assert evidence['failure_message'] == 'scanned_utc must not precede launch_stopped_utc'
    assert evidence['launch_log_sha256'] is None


def test_final_log_failure_is_sticky_without_hiding_a_new_failure() -> None:
    prior = runner.StageFailure('goal_binding', 'observer_exit', 'observer failed')
    scan = runner.StageFailure(
        'final_launch_log_gate',
        'launch_log_signature_detected',
        'prohibited signature',
    )

    assert runner._sticky_final_log_failure(prior, scan) is prior
    assert runner._sticky_final_log_failure(None, scan) is scan
    invalid = runner._sticky_final_log_failure(
        None,
        startup_gate.GateError('launch_log_invalid', 'missing closed log'),
    )
    assert invalid.stage == 'final_launch_log_gate'
    assert invalid.kind == 'invalid_evidence'


def test_final_log_gate_runs_after_stop_and_before_domain_cleanup() -> None:
    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    trial_start = source.index('    def _run_trial(')
    trial_source = source[trial_start:]

    stop_all = trial_source.index('cleanup_ok = registry.stop_all()')
    final_gate_try = trial_source.index('try:', stop_all)
    stopped_timestamp = trial_source.index('launch_stopped_utc = startup_gate_utc_now()', stop_all)
    final_gate = trial_source.index('_finalize_full_stack_log_gate(', stop_all)
    final_gate_except = trial_source.index('except (', final_gate)
    domain_cleanup = trial_source.index("'domain_cleanup',", final_gate)

    assert (
        stop_all
        < final_gate_try
        < stopped_timestamp
        < final_gate
        < final_gate_except
        < domain_cleanup
    )
    assert "registry.run_checked(\n                    'final_launch_log_gate'" not in trial_source
    assert "lifecycle_ready_path=run_dir / 'lifecycle-ready.json'" in trial_source
    assert 'expected_lifecycle_watch_pid=launch.pid' in trial_source
    assert 'expected_lifecycle_wall_timeout_s=LIFECYCLE_READY_WALL_TIMEOUT_S' in trial_source
    assert 'expected_lifecycle_nodes=LIFECYCLE_NODES' in trial_source
    assert format(runner.LIFECYCLE_READY_WALL_TIMEOUT_S, 'g') == '110'
    assert (
        "'--wall-timeout',\n                    "
        "format(LIFECYCLE_READY_WALL_TIMEOUT_S, 'g')" in trial_source
    )


def test_component_sidecars_are_finalized_before_prerequisite_manifest() -> None:
    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    trial_start = source.index('    def _run_trial(')
    trial_source = source[trial_start:]

    sidecar_targets = trial_source.index("sidecar_targets = [run_dir / 'capture.json'")
    scenario_four_target = trial_source.index(
        "run_dir / 'lifecycle-snapshot.json'", sidecar_targets
    )
    write_sidecar = trial_source.index('_write_existing_artifact_sidecar(target)', sidecar_targets)
    snapshot_paths = trial_source.index(
        "core_paths = sorted(item for item in run_dir.rglob('*') if item.is_file())",
        write_sidecar,
    )

    assert sidecar_targets < scenario_four_target < write_sidecar < snapshot_paths


def test_graph_probe_binds_exact_goal_capable_action_clients(tmp_path: Path) -> None:
    def flag_values(command: list[str], flag: str) -> list[str]:
        return [command[index + 1] for index, value in enumerate(command[:-1]) if value == flag]

    pre_mission = runner._graph_probe_command(
        tmp_path,
        tmp_path / 'pre-mission',
        watch_pid=101,
        mission_client=False,
    )
    with_mission = runner._graph_probe_command(
        tmp_path,
        tmp_path / 'mission',
        watch_pid=202,
        mission_client=True,
    )
    pre_contracts = orchestration.phase3_graph_contracts(False)
    mission_contracts = orchestration.phase3_graph_contracts(True)
    assert flag_values(pre_mission, '--topic') == [
        f'{name}={type_name}' for name, type_name in pre_contracts['topics'].items()
    ]
    assert flag_values(pre_mission, '--service') == [
        f'{name}={type_name}' for name, type_name in pre_contracts['services'].items()
    ]
    assert flag_values(pre_mission, '--action') == [
        '/robotest/follow_waypoints=nav2_msgs/action/FollowWaypoints'
    ]
    assert flag_values(pre_mission, '--action-server') == [
        '/robotest/follow_waypoints=/robotest/waypoint_follower'
    ]
    assert flag_values(pre_mission, '--action-client') == []
    assert flag_values(with_mission, '--action-client') == [
        '/robotest/follow_waypoints=/robotest/mission_runner',
    ]
    assert flag_values(pre_mission, '--wall-timeout') == ['90']
    assert flag_values(with_mission, '--wall-timeout') == ['20']
    assert pre_contracts['topics'] == mission_contracts['topics']
    assert pre_contracts['services'] == mission_contracts['services']


def test_graph_artifacts_are_validated_before_each_trial_can_advance() -> None:
    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    trial_start = source.index('    def _run_trial(')
    trial_source = source[trial_start:]

    graph_run = trial_source.index("'graph_gate',")
    graph_validation = trial_source.index(
        'graph_binding = validate_phase3_graph_artifacts(', graph_run
    )
    runtime_node_join = trial_source.index(
        'validate_phase3_runtime_graph_node_join(', graph_validation
    )
    observer_arm = trial_source.index('atomic_write_bytes(observer_arm,', runtime_node_join)
    mission_graph_run = trial_source.index("'mission_graph_gate',", observer_arm)
    mission_graph_timeout = trial_source.index('wall_timeout_s=30.0,', mission_graph_run)
    mission_graph_validation = trial_source.index(
        'mission_graph_binding = validate_phase3_graph_artifacts(', mission_graph_run
    )
    pair_validation = trial_source.index(
        'validate_phase3_graph_pair(',
        mission_graph_validation,
    )
    auxiliary_contract = trial_source.index('expected_mission_auxiliary_nodes=(', pair_validation)
    pair_check = trial_source.index("'inconsistent_graph_pair'", mission_graph_validation)
    mission_wait = trial_source.index('mission_status = mission.wait(', pair_check)

    assert graph_run < graph_validation < runtime_node_join < observer_arm
    assert (
        mission_graph_run
        < mission_graph_timeout
        < mission_graph_validation
        < pair_validation
        < auxiliary_contract
        < pair_check
        < mission_wait
    )


def test_goal_observer_arm_ack_is_validated_before_mission_launch() -> None:
    prearm_hash = orchestration.canonical_sha256([])
    acknowledgment = {
        'armed_steady_ns': 1,
        'prearm_uuid_set_sha256': prearm_hash,
        'producer': 'robotest_phase3/goal_observer',
        'schema_version': 1,
    }

    assert (
        runner._validate_goal_observer_armed(
            acknowledgment,
            {'prearm_uuid_set_sha256': prearm_hash},
        )
        == acknowledgment
    )
    tampered = dict(acknowledgment, prearm_uuid_set_sha256='0' * 64)
    with pytest.raises(runner.EvidenceError, match='changed after readiness'):
        runner._validate_goal_observer_armed(
            tampered,
            {'prearm_uuid_set_sha256': prearm_hash},
        )
    with pytest.raises(runner.EvidenceError, match='schema is invalid'):
        runner._validate_goal_observer_armed(
            dict(acknowledgment, schema_version=True),
            {'prearm_uuid_set_sha256': prearm_hash},
        )

    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    trial_start = source.index('    def _run_trial(')
    trial_source = source[trial_start:]
    arm_request = trial_source.index('atomic_write_bytes(observer_arm,')
    arm_ack = trial_source.index("stage='goal_observer_armed'")
    mission_launch = trial_source.index("'mission_runner',")
    assert arm_request < arm_ack < mission_launch


def test_positive_control_runtime_gate_finishes_before_motion_arm() -> None:
    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    positive_start = source.index('    def positive_control(')
    positive_end = source.index('    def smoke(', positive_start)
    positive_source = source[positive_start:positive_end]

    driver_ready = positive_source.index("stage='positive_driver_ready'")
    runtime_gate = positive_source.index('runtime_gate_process = registry.run_checked(')
    group_empty = positive_source.index('if _group_alive(runtime_gate_process.pgid):')
    owners_alive = positive_source.index(
        "(launch, collector, driver), stage='positive_runtime_gate'"
    )
    arm_write = positive_source.index('arm_request_sha256 = atomic_write_json(')
    arm_ack = positive_source.index("stage='positive_driver_armed'")
    driver_wait = positive_source.index('driver_status = driver.wait(50.0)')
    progress_validation = positive_source.index('command_progress=load_canonical_json(')

    assert (
        driver_ready < runtime_gate
        and runtime_gate < group_empty
        and group_empty < owners_alive
        and owners_alive < arm_write
        and arm_write < arm_ack
        and arm_ack < progress_validation
        and progress_validation < driver_wait
        and arm_ack < driver_wait
    )
    assert "'--arm-file'" in positive_source
    assert "'--armed-file'" in positive_source
    assert "'--command-progress-file'" in positive_source
    assert 'timeout_s=CONTACT_CONTROL_READY_WAIT_TIMEOUT_S' in positive_source
    assert 'wall_timeout_s=CONTACT_CONTROL_PROCESS_WALL_TIMEOUT_S' in positive_source
    assert (
        runner.CONTACT_CONTROL_WALL_TIMEOUT_S
        < runner.CONTACT_CONTROL_READY_WAIT_TIMEOUT_S
        < runner.CONTACT_CONTROL_PROCESS_WALL_TIMEOUT_S
    )


def test_contact_control_arm_paths_must_be_fresh_and_distinct(tmp_path: Path) -> None:
    arm_path = tmp_path / 'arm.json'
    armed_path = tmp_path / 'armed.json'
    runner._require_fresh_distinct_paths(
        (arm_path, armed_path), label='contact-control arm handshake'
    )

    with pytest.raises(runner.EvidenceError, match='distinct'):
        runner._require_fresh_distinct_paths(
            (arm_path, arm_path), label='contact-control arm handshake'
        )
    arm_path.write_text('{}\n', encoding='utf-8')
    with pytest.raises(runner.EvidenceError, match='already exists'):
        runner._require_fresh_distinct_paths(
            (arm_path, armed_path), label='contact-control arm handshake'
        )


def test_bounded_process_is_group_leader_and_records_exact_command(tmp_path: Path) -> None:
    process = runner.BoundedProcess(
        role='probe',
        command=['python3', '-c', 'print("ready")'],
        cwd=tmp_path,
        env=os.environ,
        evidence_dir=tmp_path / 'evidence',
        wall_timeout_s=10.0,
    )
    assert process.pid == process.pgid
    assert process.wait(12.0) == 0
    metadata = orchestration.load_json(process.metadata_path)
    assert metadata['command'] == ['python3', '-c', 'print("ready")']
    assert metadata['pid'] == metadata['pgid']
    assert process.stdout_path.read_text(encoding='utf-8') == 'ready\n'


def test_bounded_process_drains_beyond_prefix_and_marks_overflow(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.setattr(runner, 'LOG_MAX_BYTES', 128)
    process = runner.BoundedProcess(
        role='overflow_probe',
        command=['python3', '-c', 'import sys; sys.stdout.write("x" * 4096)'],
        cwd=tmp_path,
        env=os.environ,
        evidence_dir=tmp_path / 'evidence',
        wall_timeout_s=10.0,
    )
    assert process.wait(12.0) == 0
    assert process.stdout_state.observed_bytes == 4096
    assert process.stdout_state.retained_bytes == 128
    assert process.stdout_state.overflow is True
    assert process.stdout_path.stat().st_size == 128


def test_registry_stops_only_its_owned_group(tmp_path: Path) -> None:
    unrelated = subprocess.Popen(
        ['python3', '-c', 'import time; time.sleep(60)'],
        start_new_session=True,
    )
    try:
        registry = runner.ProcessRegistry(tmp_path / 'evidence', tmp_path, os.environ)
        process = registry.start(
            'sleeper',
            ['python3', '-c', 'import time; time.sleep(60)'],
            wall_timeout_s=70.0,
        )
        assert runner._group_alive(process.pgid)
        assert registry.stop_all() is True
        assert not runner._group_alive(process.pgid)
        assert unrelated.poll() is None
    finally:
        if unrelated.poll() is None:
            os.killpg(unrelated.pid, signal.SIGTERM)
        unrelated.wait(timeout=5.0)


def test_bounded_process_stop_observes_delayed_group_disappearance(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = SimpleNamespace(now=0.0)
    killed = SimpleNamespace(value=False, checks=0)
    signals: list[int] = []

    def group_alive(_pgid: int) -> bool:
        if not killed.value:
            return True
        killed.checks += 1
        return killed.checks < 3

    def kill_group(_pgid: int, requested_signal: int) -> None:
        signals.append(requested_signal)
        if requested_signal == signal.SIGKILL:
            killed.value = True

    monkeypatch.setattr(runner, '_group_alive', group_alive)
    monkeypatch.setattr(runner, 'GROUP_TERM_GRACE_S', 0.0)
    monkeypatch.setattr(runner, 'PROCESS_KILL_GRACE_S', 0.5)
    monkeypatch.setattr(runner.os, 'getpgid', lambda _pid: 41)
    monkeypatch.setattr(runner, '_process_start_ticks', lambda _pid: 73)
    monkeypatch.setattr(runner.os, 'killpg', kill_group)
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(
        runner.time, 'sleep', lambda duration: setattr(clock, 'now', clock.now + duration)
    )

    direct = SimpleNamespace(returncode=None)
    direct.poll = lambda: direct.returncode

    def wait_direct(*, timeout: float) -> int:
        assert timeout == runner.PROCESS_KILL_GRACE_S
        clock.now += 0.2
        direct.returncode = -signal.SIGTERM
        return direct.returncode

    direct.wait = wait_direct
    process = runner.BoundedProcess.__new__(runner.BoundedProcess)
    process.role = 'delayed_group'
    process.pid = 41
    process.pgid = 41
    process.start_ticks = 73
    process.process = direct
    process.returncode = None
    process._group_confirmed_empty = False
    process._finish = lambda: None

    assert process.stop() is True
    assert process._group_confirmed_empty is True
    assert signals == [signal.SIGTERM, signal.SIGKILL]
    assert killed.checks == 3
    assert clock.now == pytest.approx(0.4)


def test_bounded_process_stop_fails_closed_for_persistent_group(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = SimpleNamespace(now=0.0)
    signals: list[int] = []

    monkeypatch.setattr(runner, '_group_alive', lambda _pgid: True)
    monkeypatch.setattr(runner, 'GROUP_TERM_GRACE_S', 0.0)
    monkeypatch.setattr(runner, 'PROCESS_KILL_GRACE_S', 0.25)
    monkeypatch.setattr(runner.os, 'getpgid', lambda _pid: 43)
    monkeypatch.setattr(runner, '_process_start_ticks', lambda _pid: 79)
    monkeypatch.setattr(
        runner.os,
        'killpg',
        lambda _pgid, requested_signal: signals.append(requested_signal),
    )
    monkeypatch.setattr(runner.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(
        runner.time, 'sleep', lambda duration: setattr(clock, 'now', clock.now + duration)
    )

    direct = SimpleNamespace(returncode=None)
    direct.poll = lambda: direct.returncode

    def wait_direct(*, timeout: float) -> int:
        assert timeout == runner.PROCESS_KILL_GRACE_S
        clock.now += 0.1
        direct.returncode = -signal.SIGTERM
        return direct.returncode

    direct.wait = wait_direct
    process = runner.BoundedProcess.__new__(runner.BoundedProcess)
    process.role = 'persistent_group'
    process.pid = 43
    process.pgid = 43
    process.start_ticks = 79
    process.process = direct
    process.returncode = None
    process._group_confirmed_empty = False
    process._finish = lambda: None

    assert process.stop() is False
    assert process._group_confirmed_empty is False
    assert signals == [signal.SIGTERM, signal.SIGKILL]
    assert clock.now == pytest.approx(runner.PROCESS_KILL_GRACE_S)


def test_resource_sampler_spans_before_launch_through_shutdown(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    inherited_affinity = sorted(os.sched_getaffinity(0))
    monkeypatch.setattr(runner, 'CPU_AFFINITY', inherited_affinity)
    monkeypatch.setattr(orchestration, 'CPU_AFFINITY', inherited_affinity)
    registry = runner.ProcessRegistry(tmp_path / 'evidence', tmp_path, os.environ)
    sampler = runner.ResourceSampler(registry, tmp_path / 'resources.jsonl')
    sampler.start()
    process = registry.start(
        'short_sleeper',
        ['python3', '-c', 'import time; time.sleep(1.2)'],
        wall_timeout_s=5.0,
    )
    time.sleep(1.05)
    process.wait(3.0)
    assert registry.stop_all() is True
    sampler.stop_after_shutdown()
    summary = orchestration.summarize_resources(tmp_path / 'resources.jsonl')
    assert summary['sample_count'] >= 2
    assert summary['sampler_started_before_launch'] is True
    assert summary['sampler_stopped_after_shutdown'] is True
    assert summary['pid_reuse_detected'] is False
    assert summary['affinity_checked_pid_count'] >= 1
    assert summary['affinity_observed_cpu_union'] == inherited_affinity
    assert summary['affinity_escape_count'] == 0
    assert summary['affinity_unreadable_count'] == 0


def test_runner_has_no_global_kill_primitive() -> None:
    source = (TEST_DIR / 'phase3_benchmark_runner.py').read_text(encoding='utf-8')
    prohibited = ('p' + 'kill', 'kill' + 'all')
    assert all(token not in source for token in prohibited)
    assert 'os.killpg' in source


def test_campaign_authorization_literal_is_nonempty_and_exact_token() -> None:
    assert runner.CAMPAIGN_AUTHORIZATION == ('I_AUTHORIZE_EXACTLY_15_COLD_STACK_TRIALS_NO_RETRIES')


def test_campaign_profile_failure_precedes_trial_or_aggregate_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    benchmark = runner.BenchmarkRunner.__new__(runner.BenchmarkRunner)
    benchmark.authorization = runner.CAMPAIGN_AUTHORIZATION
    benchmark.workspace = tmp_path / 'workspace'
    benchmark.candidate_id = 'phase3-deadbee-001'
    benchmark.candidate_root = (
        benchmark.workspace / 'artifacts/evidence/phase3-benchmarks' / benchmark.candidate_id
    )
    benchmark.candidate_root.mkdir(parents=True)
    smoke_marker = benchmark.candidate_root / 'smoke/PASS.json'
    orchestration.atomic_write_json(
        smoke_marker,
        {'producer': runner.PRODUCER, 'run_result_sha256': '1' * 64, 'status': 'PASS'},
        sidecar=True,
    )
    benchmark._load_state = lambda: ({'trials': []}, {})
    benchmark._load_positive = lambda: {}
    benchmark._run_trial = lambda *_args, **_kwargs: pytest.fail('trial process started')

    def reject_profile(*_args, **_kwargs):
        raise profiler.ProfileError('missing_identity', 'profile is absent')

    monkeypatch.setattr(runner, 'validate_campaign_smoke_profile', reject_profile)
    monkeypatch.setattr(
        runner,
        'ProcessRegistry',
        lambda *_args, **_kwargs: pytest.fail('campaign process registry created'),
    )

    with pytest.raises(orchestration.EvidenceError, match='valid profiled smoke'):
        benchmark.campaign()

    assert not (benchmark.candidate_root / 'runs').exists()
    assert not (benchmark.candidate_root / 'aggregate').exists()


@pytest.mark.parametrize(
    ('mode', 'profile_value'),
    (
        ('prepare', None),
        ('positive-control', None),
        ('campaign', None),
        ('smoke', None),
        ('smoke', '1'),
    ),
)
def test_contact_profiler_mode_guard_accepts_only_intended_states(
    mode: str,
    profile_value: str | None,
) -> None:
    environment = {'PATH': '/usr/bin'}
    if profile_value is not None:
        environment[runner.CONTACT_PROFILE_ENV] = profile_value

    frozen = runner._base_environment_for_mode(mode, environment)

    assert frozen is not environment
    assert frozen['PATH'] == '/usr/bin'
    if profile_value is None:
        assert runner.CONTACT_PROFILE_ENV not in frozen
    else:
        assert frozen[runner.CONTACT_PROFILE_ENV] == '1'


@pytest.mark.parametrize(
    ('mode', 'profile_value'),
    (
        ('prepare', ''),
        ('prepare', '1'),
        ('positive-control', ''),
        ('positive-control', '0'),
        ('positive-control', '1'),
        ('positive-control', 'true'),
        ('campaign', ''),
        ('campaign', '0'),
        ('campaign', '1'),
        ('campaign', 'other'),
        ('smoke', ''),
        ('smoke', '0'),
        ('smoke', 'true'),
        ('smoke', ' 1'),
        ('smoke', '1 '),
    ),
)
def test_contact_profiler_mode_guard_rejects_present_invalid_states(
    mode: str,
    profile_value: str,
) -> None:
    with pytest.raises(orchestration.EvidenceError, match=runner.CONTACT_PROFILE_ENV):
        runner._base_environment_for_mode(
            mode,
            {runner.CONTACT_PROFILE_ENV: profile_value},
        )


def _benchmark_argv(tmp_path: Path, mode: str) -> list[str]:
    workspace = tmp_path / 'workspace'
    arguments = [
        '--mode',
        mode,
        '--workspace',
        str(workspace),
        '--candidate-id',
        'phase3-deadbee-001',
        '--domain-base',
        '100',
        '--output-root',
        str(workspace / 'artifacts/evidence/phase3-benchmarks'),
        '--build-binding',
        str(workspace / 'build-binding.json'),
    ]
    if mode == 'campaign':
        arguments.extend(['--authorization', runner.CAMPAIGN_AUTHORIZATION])
    return arguments


@pytest.mark.parametrize(
    ('mode', 'profile_value'),
    (
        ('prepare', '1'),
        ('positive-control', ''),
        ('smoke', '0'),
        ('campaign', '1'),
    ),
)
def test_main_rejects_profile_state_before_stage_or_evidence_mutation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
    mode: str,
    profile_value: str,
) -> None:
    stage_name = mode.replace('-', '_')
    called = False

    def unexpected_stage(_benchmark: runner.BenchmarkRunner) -> None:
        nonlocal called
        called = True

    monkeypatch.setenv(runner.CONTACT_PROFILE_ENV, profile_value)
    monkeypatch.setattr(runner.BenchmarkRunner, stage_name, unexpected_stage)
    monkeypatch.setattr(runner.signal, 'signal', lambda *_args: None)

    assert runner.main(_benchmark_argv(tmp_path, mode)) == 2
    assert called is False
    assert not (tmp_path / 'workspace').exists()
    assert runner.CONTACT_PROFILE_ENV in capsys.readouterr().err


@pytest.mark.parametrize(
    ('mode', 'profile_value'),
    (
        ('prepare', None),
        ('positive-control', None),
        ('smoke', None),
        ('smoke', '1'),
        ('campaign', None),
    ),
)
def test_main_freezes_allowed_profile_state_for_child_processes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    mode: str,
    profile_value: str | None,
) -> None:
    stage_name = mode.replace('-', '_')
    observed: list[str | None] = []
    if profile_value is None:
        monkeypatch.delenv(runner.CONTACT_PROFILE_ENV, raising=False)
    else:
        monkeypatch.setenv(runner.CONTACT_PROFILE_ENV, profile_value)

    def inspect_stage(benchmark: runner.BenchmarkRunner) -> None:
        if profile_value is None:
            monkeypatch.setenv(runner.CONTACT_PROFILE_ENV, '1')
        else:
            monkeypatch.delenv(runner.CONTACT_PROFILE_ENV)
        child = runner._environment(100, 'robotest_test', benchmark.base_environment)
        observed.append(child.get(runner.CONTACT_PROFILE_ENV))

    monkeypatch.setattr(runner.BenchmarkRunner, stage_name, inspect_stage)
    monkeypatch.setattr(runner.signal, 'signal', lambda *_args: None)

    assert runner.main(_benchmark_argv(tmp_path, mode)) == 0
    assert observed == [profile_value]


def test_smoke_trial_uses_the_exact_suite_plan_partition() -> None:
    plan = orchestration.suite_document(TEST_DIR.parent, 'phase3-deadbee-001', 100)

    smoke = runner._smoke_trial_plan(plan, plan['candidate_id'])

    assert smoke['candidate_id'] == 'phase3-deadbee-001-smoke'
    assert smoke['gz_partition'] == plan['smoke']['gz_partition']
    assert smoke['gz_partition'] == 'robotest_p3_phase3-deadbee-001-smoke_00'
    assert smoke['ros_domain_id'] == 116
    assert smoke['run_id'] == 'phase3-deadbee-001-smoke-s1-r0'


def test_frozen_candidate_evidence_tree_is_ignored_by_git() -> None:
    workspace = TEST_DIR.parent
    probe = 'artifacts/evidence/phase3-benchmarks/.probe'
    result = subprocess.run(
        ['git', '-C', str(workspace), 'check-ignore', '--verbose', '--', probe],
        capture_output=True,
        check=False,
        text=True,
        timeout=5.0,
    )
    assert result.returncode == 0, result.stderr
    rule, ignored_path = result.stdout.rstrip('\n').split('\t', maxsplit=1)
    assert rule.endswith(':/artifacts/evidence/phase3-benchmarks/')
    assert ignored_path == probe


def test_campaign_aggregate_requires_zero_exit_and_canonical_pass(tmp_path: Path) -> None:
    aggregate = tmp_path / 'aggregate-result.json'
    orchestration.atomic_write_json(aggregate, {'verdict': {'automated_status': 'PASS'}})
    runner._require_campaign_aggregate_pass(0, aggregate, cleanup_ok=True)

    orchestration.atomic_write_json(aggregate, {'verdict': {'automated_status': 'FAIL'}})
    with pytest.raises(orchestration.EvidenceError, match='aggregate verdict'):
        runner._require_campaign_aggregate_pass(30, aggregate, cleanup_ok=True)
    assert orchestration.load_json(aggregate)['verdict']['automated_status'] == 'FAIL'


def test_campaign_aggregate_requires_owned_group_cleanup(tmp_path: Path) -> None:
    aggregate = tmp_path / 'aggregate-result.json'
    orchestration.atomic_write_json(aggregate, {'verdict': {'automated_status': 'PASS'}})
    with pytest.raises(orchestration.EvidenceError, match='did not shut down cleanly'):
        runner._require_campaign_aggregate_pass(0, aggregate, cleanup_ok=False)


def test_failure_reason_is_bounded_by_utf8_bytes() -> None:
    value = runner._bounded_reason('\u20ac' * 4096)
    assert value.endswith('...')
    assert len(value.encode('utf-8')) <= 4096


def test_trial_collector_and_drain_wait_share_exact_contact_progress_path(
    tmp_path: Path,
) -> None:
    contact_progress_path = tmp_path / 'contact-progress.json'
    command = runner._metrics_collector_command(
        capture_path=tmp_path / 'capture.json',
        ready_path=tmp_path / 'metrics.ready.json',
        stop_path=tmp_path / 'metrics.stop',
        contact_progress_path=contact_progress_path,
    )
    progress_option = command.index('--contact-progress-file')
    assert command[progress_option + 1] == str(contact_progress_path)
    assert command.count(str(contact_progress_path)) == 1

    command_progress_path = tmp_path / 'command-progress.json'
    positive_command = runner._metrics_collector_command(
        capture_path=tmp_path / 'positive-capture.json',
        ready_path=tmp_path / 'positive-metrics.ready.json',
        stop_path=tmp_path / 'positive-metrics.stop',
        contact_progress_path=tmp_path / 'positive-contact-progress.json',
        command_progress_path=command_progress_path,
        command_progress_run_id='candidate-positive-control',
    )
    command_progress_option = positive_command.index('--command-progress-file')
    command_progress_run_option = positive_command.index('--command-progress-run-id')
    assert positive_command[command_progress_option + 1] == str(command_progress_path)
    assert positive_command[command_progress_run_option + 1] == 'candidate-positive-control'
    assert command_progress_option < command_progress_run_option
    with pytest.raises(runner.EvidenceError, match='supplied together'):
        runner._metrics_collector_command(
            capture_path=tmp_path / 'invalid-capture.json',
            ready_path=tmp_path / 'invalid-metrics.ready.json',
            stop_path=tmp_path / 'invalid-metrics.stop',
            contact_progress_path=tmp_path / 'invalid-contact-progress.json',
            command_progress_path=command_progress_path,
        )

    orchestration.atomic_write_json(
        contact_progress_path,
        {
            'latest_retained_stamp_ns': 600_000_000,
            'producer': 'robotest_metrics/metrics_collector',
            'public_topic': '/robotest/validation/contacts',
            'retained_message_count': 3,
            'schema_version': 1,
        },
    )
    progress = runner._wait_for_contact_progress(
        contact_progress_path,
        qualifying_stamp_ns=500_000_000,
        timeout_s=0.1,
        watched=(),
    )
    assert progress['latest_retained_stamp_ns'] == 600_000_000


def test_contact_progress_wait_polls_until_atomic_marker_advances(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    progress_path = tmp_path / 'contact-progress.json'
    document = {
        'latest_retained_stamp_ns': 400_000_000,
        'producer': 'robotest_metrics/metrics_collector',
        'public_topic': '/robotest/validation/contacts',
        'retained_message_count': 2,
        'schema_version': 1,
    }
    orchestration.atomic_write_json(progress_path, document)
    now = 0.0
    sleep_count = 0

    def monotonic() -> float:
        return now

    def sleep(duration: float) -> None:
        nonlocal now, sleep_count
        now += duration
        sleep_count += 1
        if sleep_count == 1:
            orchestration.atomic_write_json(
                progress_path,
                {
                    **document,
                    'latest_retained_stamp_ns': 600_000_000,
                    'retained_message_count': 3,
                },
            )

    monkeypatch.setattr(runner.time, 'monotonic', monotonic)
    monkeypatch.setattr(runner.time, 'sleep', sleep)

    progress = runner._wait_for_contact_progress(
        progress_path,
        qualifying_stamp_ns=500_000_000,
        timeout_s=1.0,
        watched=(),
    )

    assert sleep_count == 1
    assert progress['latest_retained_stamp_ns'] == 600_000_000
    assert progress['retained_message_count'] == 3


def test_positive_control_allows_sim_fault_proxy_but_forbids_navigation() -> None:
    assert 'fault_proxy' not in runtime_gate.POSITIVE_FORBIDDEN_NODES
    assert 'collision_monitor' in runtime_gate.POSITIVE_FORBIDDEN_NODES
    assert 'waypoint_follower' in runtime_gate.POSITIVE_FORBIDDEN_NODES


def test_candidate_gate_freezes_exact_command_and_tf_owners() -> None:
    assert runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS['/robotest/cmd_vel'] == {
        '/robotest/collision_monitor'
    }
    assert runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS['/tf'] == {
        '/robotest/amcl',
        '/robotest/fault_proxy',
        '/robotest/robot_state_publisher',
    }
    assert (
        runtime_gate.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS['/robotest/cmd_vel_behavior_unused']
        == set()
    )
    assert runtime_gate.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS['/robotest/cmd_vel'] == {
        '/robotest/metrics_collector',
        '/robotest/parameter_bridge',
    }


def test_runtime_gate_freezes_metrics_only_command_history_override() -> None:
    assert runtime_gate._endpoint_qos_contract(
        '/robotest/cmd_vel',
        'publisher',
        '/robotest/contact_control_driver',
    ) == ('RELIABLE', 'VOLATILE', 1)
    assert runtime_gate._endpoint_qos_contract(
        '/robotest/cmd_vel',
        'subscriber',
        '/robotest/parameter_bridge',
    ) == ('RELIABLE', 'VOLATILE', 1)
    metrics_contract = runtime_gate._endpoint_qos_contract(
        '/robotest/cmd_vel',
        'subscriber',
        '/robotest/metrics_collector',
    )
    assert metrics_contract == (
        'RELIABLE',
        'VOLATILE',
        metrics_constants.COMMAND_QOS_DEPTH,
    )
    assert runtime_gate._qos_matches(
        {
            'depth': 4_096,
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST',
            'node': '/robotest/metrics_collector',
            'reliability': 'RELIABLE',
            'topic_type': 'geometry_msgs/msg/Twist',
        },
        metrics_contract,
    )
    assert not runtime_gate._qos_matches(
        {
            'depth': 1,
            'durability': 'VOLATILE',
            'history': 'KEEP_LAST',
            'node': '/robotest/metrics_collector',
            'reliability': 'RELIABLE',
            'topic_type': 'geometry_msgs/msg/Twist',
        },
        metrics_contract,
    )


def test_positive_command_ownership_rejects_suffix_spoof_and_wrong_type() -> None:
    evidence = {
        'publishers': [
            {
                'gid': '01' * 16,
                'node': '/robotest/contact_control_driver',
                'topic_type': runtime_gate.COMMAND_MESSAGE_TYPE,
            }
        ],
        'subscribers': [
            {
                'gid': '02' * 16,
                'node': '/robotest/metrics_collector',
                'topic_type': runtime_gate.COMMAND_MESSAGE_TYPE,
            },
            {
                'gid': '03' * 16,
                'node': '/robotest/parameter_bridge',
                'topic_type': runtime_gate.COMMAND_MESSAGE_TYPE,
            },
        ],
    }
    assert runtime_gate._positive_command_ownership(evidence) == {
        'publisher': True,
        'subscribers': True,
    }

    evidence['publishers'][0]['node'] = '/robotest/nested/contact_control_driver'
    assert runtime_gate._positive_command_ownership(evidence)['publisher'] is False
    evidence['publishers'][0]['node'] = '/robotest/contact_control_driver'
    evidence['publishers'][0]['topic_type'] = 'example_interfaces/msg/String'
    assert runtime_gate._positive_command_ownership(evidence)['publisher'] is False


def test_runtime_gate_applies_command_depth_per_endpoint() -> None:
    def endpoint(node_name: str, depth: int, gid: int) -> SimpleNamespace:
        return SimpleNamespace(
            endpoint_gid=bytes([gid]) * 16,
            node_name=node_name,
            node_namespace='/robotest',
            qos_profile=SimpleNamespace(
                depth=depth,
                durability='VOLATILE',
                history='KEEP_LAST',
                reliability='RELIABLE',
            ),
            topic_type='geometry_msgs/msg/Twist',
        )

    metrics = endpoint('metrics_collector', 4_096, 1)
    bridge = endpoint('parameter_bridge', 1, 2)
    publisher = endpoint('collision_monitor', 1, 3)
    node = SimpleNamespace(
        get_publishers_info_by_topic=lambda _topic: [publisher],
        get_subscriptions_info_by_topic=lambda _topic: [metrics, bridge],
    )

    evidence = runtime_gate._topic_evidence(node, '/robotest/cmd_vel')
    assert evidence['publisher_qos_pass'] is True
    assert evidence['subscriber_qos_pass'] is True
    assert evidence['expected']['endpoint_depth_overrides'] == [
        {'depth': 4_096, 'node': '/robotest/metrics_collector', 'side': 'subscriber'}
    ]
    metrics.qos_profile.depth = 1
    assert not runtime_gate._topic_evidence(node, '/robotest/cmd_vel')['subscriber_qos_pass']


def test_runtime_qos_unknown_depth_is_inconclusive_not_a_false_mismatch() -> None:
    record = {
        'depth': 0,
        'durability': 'VOLATILE',
        'history': 'UNKNOWN',
        'node': '/robotest/source',
        'reliability': 'RELIABLE',
        'topic_type': 'std_msgs/msg/String',
    }
    status = runtime_gate._qos_status(record, ('RELIABLE', 'VOLATILE', 10))
    assert status['policy_contract_pass'] is True
    assert status['introspection_complete'] is False
    assert status['bounded_depth_live_proven'] is False
    assert status['exact_depth_live_proven'] is False


def test_runtime_qos_explicit_keep_all_is_rejected() -> None:
    record = {
        'depth': 0,
        'durability': 'VOLATILE',
        'history': 'KEEP_ALL',
        'node': '/robotest/source',
        'reliability': 'RELIABLE',
        'topic_type': 'std_msgs/msg/String',
    }
    status = runtime_gate._qos_status(record, ('RELIABLE', 'VOLATILE', 10))
    assert status['explicit_keep_all'] is True
    assert status['policy_contract_pass'] is False


def test_runtime_gate_rejects_duplicate_endpoints_with_the_same_fqn() -> None:
    endpoint = {
        'gid': '01',
        'node': '/robotest/contact_stream_gate',
        'topic_type': runtime_gate.CONTACT_MESSAGE_TYPE,
    }
    assert runtime_gate._exact_endpoint_owners(
        [endpoint],
        {'/robotest/contact_stream_gate'},
        expected_type=runtime_gate.CONTACT_MESSAGE_TYPE,
    )
    assert not runtime_gate._exact_endpoint_owners(
        [endpoint, {**endpoint, 'gid': '02'}],
        {'/robotest/contact_stream_gate'},
        expected_type=runtime_gate.CONTACT_MESSAGE_TYPE,
    )


def test_runtime_gate_proc_stat_parser_handles_parentheses_in_comm() -> None:
    assert runtime_gate._proc_parent_pid('123 (gate ) worker) S 42 7 7 0') == 42
