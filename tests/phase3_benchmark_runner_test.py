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
runner = _load('phase3_benchmark_runner', 'phase3_benchmark_runner.py')
runtime_gate = _load('phase3_runtime_gate', 'phase3_runtime_gate.py')
metrics_constants = _load(
    'robotest_metrics_source_constants',
    '../src/robotest_metrics/robotest_metrics/constants.py',
)


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
    observer_arm = trial_source.index('atomic_write_bytes(observer_arm,', graph_validation)
    mission_graph_run = trial_source.index("'mission_graph_gate',", observer_arm)
    mission_graph_validation = trial_source.index(
        'mission_graph_binding = validate_phase3_graph_artifacts(', mission_graph_run
    )
    pair_validation = trial_source.index(
        'validate_phase3_graph_pair(graph_binding, mission_graph_binding)',
        mission_graph_validation,
    )
    pair_check = trial_source.index("'inconsistent_graph_pair'", mission_graph_validation)
    mission_wait = trial_source.index('mission_status = mission.wait(', pair_check)

    assert graph_run < graph_validation < observer_arm
    assert (
        mission_graph_run < mission_graph_validation < pair_validation < pair_check < mission_wait
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

    assert (
        driver_ready < runtime_gate
        and runtime_gate < group_empty
        and group_empty < owners_alive
        and owners_alive < arm_write
        and arm_write < arm_ack
        and arm_ack < driver_wait
    )
    assert "'--arm-file'" in positive_source
    assert "'--armed-file'" in positive_source


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
