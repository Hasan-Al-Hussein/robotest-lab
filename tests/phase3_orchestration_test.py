# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Pure tests for the Phase 3 orchestration and evidence lane."""

from __future__ import annotations

import copy
import importlib.util
from itertools import pairwise
import json
from pathlib import Path
import sys

from jsonschema import Draft202012Validator
import pytest
import yaml

MODULE_PATH = Path(__file__).with_name('phase3_orchestration.py')
SPEC = importlib.util.spec_from_file_location('phase3_orchestration', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
orchestration = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = orchestration
SPEC.loader.exec_module(orchestration)


def _scenario(path: Path, scenario_id: int, scenario_name: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        yaml.safe_dump(
            {
                'fault_schedule_sha256': str(scenario_id) * 64,
                'mission_timeout_sim_s': 180.0,
                'retries': 0,
                'scenario_id': scenario_id,
                'scenario_name': scenario_name,
                'simulator_seed': 42,
                'wall_escape_timeout_s': 300.0,
            },
            sort_keys=True,
        ),
        encoding='utf-8',
    )


def _workspace(tmp_path: Path) -> Path:
    for scenario_id, scenario_name, relative in orchestration.SCENARIOS:
        _scenario(tmp_path / relative, scenario_id, scenario_name)
    return tmp_path


@pytest.mark.parametrize(
    'package_name',
    ('robotest_missions', 'robotest_metrics', 'robotest_scenarios'),
)
def test_ament_python_manifests_do_not_publish_an_invalid_rosdep_key(
    package_name: str,
) -> None:
    package_xml = (Path(__file__).parents[1] / 'src' / package_name / 'package.xml').read_text(
        encoding='utf-8'
    )
    assert '<buildtool_depend>ament_python</buildtool_depend>' not in package_xml
    assert '<build_type>ament_python</build_type>' in package_xml


def test_suite_plan_is_exact_ordered_and_isolated(tmp_path: Path) -> None:
    workspace = _workspace(tmp_path)
    plan = orchestration.suite_document(workspace, 'candidate-1', 100)
    assert len(plan['trials']) == 15
    assert [item['suite_index'] for item in plan['trials']] == list(range(15))
    assert [item['ros_domain_id'] for item in plan['trials']] == list(range(100, 115))
    assert plan['trials'][0]['run_id'] == 'candidate-1-s1-r0-i00'
    assert plan['trials'][-1]['run_id'] == 'candidate-1-s5-r2-i14'
    assert plan['positive_control']['ros_domain_id'] == 115
    assert plan['smoke']['ros_domain_id'] == 116


@pytest.mark.parametrize(
    ('candidate_id', 'domain_base'),
    [('bad value', 100), ('ok', -1), ('ok', 217)],
)
def test_suite_plan_rejects_unsafe_identity_or_domain(
    tmp_path: Path, candidate_id: str, domain_base: int
) -> None:
    workspace = _workspace(tmp_path)
    with pytest.raises(orchestration.EvidenceError):
        orchestration.plan_suite(workspace, candidate_id, domain_base)


def test_canonical_json_is_strict_and_newline_terminated() -> None:
    assert orchestration.canonical_json_bytes({'b': 2, 'a': 1}) == b'{"a":1,"b":2}\n'
    with pytest.raises(orchestration.EvidenceError):
        orchestration.canonical_json_bytes({'bad': float('nan')})


def test_utf8_string_bound_is_bytes_not_codepoints() -> None:
    assert orchestration.require_bounded_string('x' * 4096, 'value')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.require_bounded_string('\u00e9' * 4096, 'value')


def test_lifecycle_schedule_has_exact_absolute_dense_window() -> None:
    schedule = orchestration.lifecycle_schedule('run-1', 1_000_000_000)
    stamps = schedule['requested_stamps_ns']
    assert len(stamps) == 96
    assert stamps[0] == 12_600_000_000
    assert stamps[-1] == 31_600_000_000
    assert all(right - left == 200_000_000 for left, right in pairwise(stamps))


def test_acceptance_thresholds_are_scenario_specific() -> None:
    s1, required1 = orchestration.acceptance_for_scenario(1)
    s2, _ = orchestration.acceptance_for_scenario(2)
    _s3, required3 = orchestration.acceptance_for_scenario(3)
    _s4, required4 = orchestration.acceptance_for_scenario(4)
    assert s1['measurements.path_efficiency']['minimum'] == 0.75
    assert s1['measurements.rtf_median']['minimum'] == 0.80
    assert s2['measurements.path_efficiency']['minimum'] == 0.60
    assert 'measurements.scenario3_stop_command' in required3
    assert 'measurements.sensor_recovery' in required4
    assert 'measurements.scenario3_stop_command' not in required1


def test_tree_manifest_uses_relative_path_and_content(tmp_path: Path) -> None:
    (tmp_path / 'tree').mkdir()
    (tmp_path / 'tree/a.txt').write_text('a\n', encoding='utf-8')
    (tmp_path / 'tree/b.txt').write_text('b\n', encoding='utf-8')
    first = orchestration.tree_manifest(tmp_path, ['tree'])
    second = orchestration.tree_manifest(tmp_path, ['tree'])
    assert first == second
    assert [item['path'] for item in first['files']] == ['tree/a.txt', 'tree/b.txt']
    (tmp_path / 'tree/b.txt').write_text('changed\n', encoding='utf-8')
    assert (
        orchestration.tree_manifest(tmp_path, ['tree'])['aggregate_sha256']
        != first['aggregate_sha256']
    )


def test_source_install_correspondence_requires_matching_runtime_bytes(
    tmp_path: Path,
) -> None:
    for package in orchestration.RUNTIME_PACKAGES:
        source = tmp_path / 'src' / package / 'package.xml'
        installed = tmp_path / 'install' / package / 'share' / package / 'package.xml'
        source.parent.mkdir(parents=True, exist_ok=True)
        installed.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(f'<package>{package}</package>\n', encoding='utf-8')
        installed.write_bytes(source.read_bytes())
    module = tmp_path / 'src/robotest_metrics/robotest_metrics/example.py'
    installed_module = (
        tmp_path
        / 'install/robotest_metrics/lib/python3.12/site-packages/'
        / 'robotest_metrics/example.py'
    )
    module.parent.mkdir(parents=True, exist_ok=True)
    installed_module.parent.mkdir(parents=True, exist_ok=True)
    module.write_text('VALUE = 1\n', encoding='utf-8')
    installed_module.write_bytes(module.read_bytes())
    result = orchestration.source_install_correspondence(tmp_path)
    assert result['all_match'] is True
    assert result['file_count'] == len(orchestration.RUNTIME_PACKAGES) + 1
    installed_module.write_text('VALUE = 2\n', encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.source_install_correspondence(tmp_path)


def test_resource_summary_is_bounded_and_computes_nearest_rank_p95(tmp_path: Path) -> None:
    trace = tmp_path / 'resources.jsonl'
    rows = []
    for index, cpu in enumerate((0.0, 10.0, 20.0, 30.0)):
        rows.append(
            {
                'affinity_checked_pid_count': 1 if index == 1 else 0,
                'affinity_escape_count': 0,
                'affinity_escape_prefix': [],
                'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5] if index == 1 else [],
                'affinity_unreadable_count': 0,
                'affinity_unreadable_pid_prefix': [],
                'cpu_percent': cpu,
                'missing_count': 0,
                'oom_kill': False,
                'phase': (
                    'before_launch' if index == 0 else 'after_shutdown' if index == 3 else 'run'
                ),
                'pid_reuse_detected': False,
                'rss_sum_bytes': 100 + index,
                'wsl_memory_bytes': 1000 + index,
                'wsl_swap_bytes': 0,
            }
        )
    trace.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')
    result = orchestration.summarize_resources(trace)
    assert result['sample_count'] == 4
    assert result['cpu_percent_mean'] == 15.0
    assert result['cpu_percent_p95'] == 30.0
    assert result['peak_rss_sum_bytes'] == 103
    assert result['sampler_started_before_launch'] is True
    assert result['sampler_stopped_after_shutdown'] is True
    assert result['affinity_checked_pid_count'] == 1
    assert result['affinity_observed_cpu_union'] == [0, 1, 2, 3, 4, 5]


def test_resource_summary_rejects_a_bounded_child_affinity_escape(tmp_path: Path) -> None:
    trace = tmp_path / 'resources.jsonl'
    base = {
        'affinity_checked_pid_count': 0,
        'affinity_escape_count': 0,
        'affinity_escape_prefix': [],
        'affinity_observed_cpu_union': [],
        'affinity_unreadable_count': 0,
        'affinity_unreadable_pid_prefix': [],
        'cpu_percent': 0.0,
        'missing_count': 0,
        'oom_kill': False,
        'pid_reuse_detected': False,
        'rss_sum_bytes': 0,
        'wsl_memory_bytes': 0,
        'wsl_swap_bytes': 0,
    }
    before = {**base, 'phase': 'before_launch'}
    escaped = {
        **base,
        'affinity_checked_pid_count': 1,
        'affinity_escape_count': 1,
        'affinity_escape_prefix': [{'allowed_cpus': [0, 6], 'escaped_cpus': [6], 'pid': 42}],
        'affinity_observed_cpu_union': [0, 6],
        'phase': 'run',
    }
    after = {**base, 'phase': 'after_shutdown'}
    trace.write_text(
        ''.join(json.dumps(row) + '\n' for row in (before, escaped, after)),
        encoding='utf-8',
    )
    with pytest.raises(orchestration.EvidenceError, match='affinity escape'):
        orchestration.summarize_resources(trace)


def test_positive_control_reconciliation_binds_semantic_manifest(tmp_path: Path) -> None:
    unsigned = {
        'bridge_sha256': '1' * 64,
        'contact_configuration_sha256': '2' * 64,
        'contact_topic': '/robotest/validation/contacts',
        'covered_collisions': [],
        'rendered_robot_collisions': [],
        'rendered_sdf_sha256': '3' * 64,
        'robot_collisions': [],
        'robot_description_sha256': '4' * 64,
        'robot_model': 'robotest',
        'schema_version': 2,
        'support_pairs': [],
        'world_source_sha256': '5' * 64,
    }
    manifest = {**unsigned, 'manifest_sha256': orchestration.canonical_sha256(unsigned)}
    manifest_path = tmp_path / 'coverage.yaml'
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=True), encoding='utf-8')
    result = {
        'cleanup': {},
        'configuration': {
            'coverage_manifest_provenance': {
                key: manifest[key]
                for key in (
                    'bridge_sha256',
                    'contact_configuration_sha256',
                    'rendered_sdf_sha256',
                    'robot_description_sha256',
                    'world_source_sha256',
                )
            }
            | {'coverage_manifest_sha256': manifest['manifest_sha256']},
            'coverage_manifest_sha256': manifest['manifest_sha256'],
        },
        'control': {
            'command_trace': [
                {
                    'angular_z': 0.0,
                    'collector_sequence': 1,
                    'linear_x': 0.05,
                    'phase': 'forward',
                    'sim_stamp_ns': 10,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 2,
                    'linear_x': 0.0,
                    'phase': 'final_zero',
                    'sim_stamp_ns': 20,
                },
            ],
            'contact': {
                'exact_pair_raw_count': 1,
                'expected_pair': ['robotest::base', 'wall::collision'],
                'first_qualifying_contact': {'sim_stamp_ns': 15},
            },
            'timeline': {'release_required_through_stamp_ns': 25},
        },
        'identity': {'run_id': 'positive', 'scenario_sha256': '6' * 64},
        'producer': 'robotest_scenarios/contact_control_driver',
        'quality': {'overflow_free': True},
        'schema_version': 1,
        'status': 'PASS',
        'verdict': {
            'authority': 'component_only',
            'benchmark_pass': None,
            'exit_code': 0,
            'reason': 'complete',
        },
    }
    result_path = tmp_path / 'contact-control-result.json'
    orchestration.atomic_write_json(result_path, result, sidecar=True)
    capture = {
        'clock': {'latest_stamp_ns': 30, 'regression_count': 0},
        'quality': {'collector_overflow': False},
        'stop_reason': 'stop_file',
        'streams': {
            'cmd_vel': {
                'items': [
                    {
                        'angular_z_rad_s': 0.0,
                        'linear_x_m_s': 0.05,
                        'stamp_ns': 10,
                    },
                    {
                        'angular_z_rad_s': 0.0,
                        'linear_x_m_s': 0.0,
                        'stamp_ns': 20,
                    },
                ]
            },
            'contacts': {
                'items': [
                    {
                        'contacts': [
                            {
                                'collision1': 'robotest::base',
                                'collision2': 'wall::collision',
                            }
                        ],
                        'stamp_ns': 15,
                    }
                ]
            },
        },
    }
    capture_path = tmp_path / 'capture.json'
    orchestration.atomic_write_json(capture_path, capture)
    bound = orchestration.reconcile_positive_control(
        result_path=result_path,
        capture_path=capture_path,
        manifest_path=manifest_path,
        collector_configuration_sha256='7' * 64,
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    assert bound['benchmark_binding']['positive_control_run_id'] == 'positive'
    assert (
        bound['benchmark_binding']['benchmark_provenance']['coverage_manifest_sha256']
        == manifest['manifest_sha256']
    )
    tampered = copy.deepcopy(manifest)
    tampered['bridge_sha256'] = '8' * 64
    manifest_path.write_text(yaml.safe_dump(tampered, sort_keys=True), encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.reconcile_positive_control(
            result_path=result_path,
            capture_path=capture_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )


def test_component_manifest_rejects_escape_and_duplicates(tmp_path: Path) -> None:
    run = tmp_path / 'run'
    run.mkdir()
    artifact = run / 'a.json'
    artifact.write_text('{}\n', encoding='utf-8')
    result = orchestration.component_manifest([artifact], run)
    assert result['artifact_count'] == 1
    assert orchestration.verify_component_manifest(result, run) is True
    artifact.write_text('{"changed":true}\n', encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.verify_component_manifest(result, run)
    artifact.write_text('{}\n', encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.component_manifest([artifact, artifact], run)
    outside = tmp_path / 'outside.json'
    outside.write_text('{}\n', encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.component_manifest([outside], run)


def test_trial_context_matches_metrics_schema(tmp_path: Path) -> None:
    workspace = _workspace(tmp_path)
    plan = orchestration.suite_document(workspace, 'candidate-1', 100)['trials'][0]
    build = {
        'collector_configuration_sha256': '1' * 64,
        'metrics_contract_sha256': '2' * 64,
        'source_configuration_sha256': '3' * 64,
        'target_set_sha256': '4' * 64,
    }
    positive = {
        'coverage_manifest': {'manifest_sha256': '5' * 64},
        'positive_control_json_sha256': '6' * 64,
    }
    context = orchestration.make_trial_context(
        plan,
        workspace=workspace,
        git_sha='a' * 40,
        build=build,
        positive=positive,
    )
    schema = json.loads(
        (
            Path(__file__).parents[1] / 'src/robotest_metrics/schema/trial-context.schema.json'
        ).read_text(encoding='utf-8')
    )
    assert list(Draft202012Validator(schema).iter_errors(context)) == []
    failed = orchestration.make_trial_context(
        plan,
        workspace=workspace,
        git_sha='a' * 40,
        build=build,
        positive=positive,
        failure={
            'evidence_sha256': '7' * 64,
            'exit_code': 124,
            'kind': 'readiness_timeout',
            'reason': 'bounded test failure',
            'stage': 'scenario_ready',
            'wall_timed_out': True,
        },
    )
    assert list(Draft202012Validator(schema).iter_errors(failed)) == []


def test_orchestrator_evidence_matches_metrics_schema(tmp_path: Path) -> None:
    run = tmp_path / 'run'
    run.mkdir()
    core = run / 'core.json'
    core.write_text('{}\n', encoding='utf-8')
    manifest = orchestration.component_manifest([core], run)
    manifest_path = run / 'prerequisite-manifest.json'
    orchestration.atomic_write_json(manifest_path, manifest, sidecar=True)
    (run / 'probe.stdout.log').write_text('ok\n', encoding='utf-8')
    (run / 'probe.stderr.log').write_text('', encoding='utf-8')
    plan = {
        'candidate_id': 'candidate-1',
        'gz_partition': 'robotest_p3_candidate-1_00',
        'repetition_index': 0,
        'ros_domain_id': 100,
        'run_id': 'candidate-1-s1-r0-i00',
        'scenario_id': 1,
        'scenario_sha256': '1' * 64,
        'suite_index': 0,
    }
    build = {
        'collector_configuration_sha256': '2' * 64,
        'install': {'aggregate_sha256': '3' * 64},
        'metrics_contract_sha256': '4' * 64,
        'source': {'aggregate_sha256': '5' * 64},
        'source_configuration_sha256': '6' * 64,
        'source_install': {'aggregate_sha256': '8' * 64, 'all_match': True},
        'target_set_sha256': '7' * 64,
    }
    evidence = orchestration.make_orchestrator_evidence(
        plan=plan,
        build_start=build,
        build_end=copy.deepcopy(build),
        git_sha='a' * 40,
        git_status_porcelain='',
        resource_summary={
            'affinity_checked_pid_count': 3,
            'affinity_escape_count': 0,
            'affinity_observed_cpu_union': [0, 1, 2, 3, 4, 5],
            'affinity_unreadable_count': 0,
            'cpu_percent_mean': 1.0,
            'cpu_percent_p95': 2.0,
            'cpu_percent_peak': 3.0,
            'missing_sample_count': 0,
            'oom_kill': False,
            'overflow_free': True,
            'peak_rss_sum_bytes': 1024,
            'pid_reuse_detected': False,
            'sample_count': 3,
            'sampler_started_before_launch': True,
            'sampler_stopped_after_shutdown': True,
            'wsl_peak_memory_bytes': 2048,
            'wsl_peak_swap_bytes': 0,
        },
        execution={
            'command': 'scripts/run_benchmarks.sh --mode campaign',
            'exit_code': 0,
            'wall_duration_s': 10.0,
            'wall_timed_out': False,
            'wall_timeout_s': 1200.0,
            'working_directory': str(tmp_path),
        },
        process={
            'cold_stack': True,
            'fresh_fault_generation': True,
            'fresh_localization': True,
            'new_process_group': True,
            'partition_unused_before_start': True,
            'previous_trial_gone': True,
            'ros_domain_unused_before_start': True,
        },
        cleanup={
            'all_owned_processes_exited': True,
            'discovery_endpoints_gone': True,
            'no_orphans': True,
        },
        gates={
            'graph_contract_pass': True,
            'namespace_isolation_pass': True,
            'qos_contract_pass': True,
            'source_install_binding_pass': True,
            'validation_autonomy_isolation_pass': True,
        },
        run_dir=run,
        component_manifest_sha256=orchestration.file_sha256(manifest_path),
    )
    schema = json.loads(
        (
            Path(__file__).parents[1] / 'src/robotest_metrics/schema/analysis-request.schema.json'
        ).read_text(encoding='utf-8')
    )
    wrapper = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/orchestrator',
    }
    assert list(Draft202012Validator(wrapper).iter_errors(evidence)) == []


def test_goal_binding_reconciliation_is_exact() -> None:
    observer = {'accepted_goal_stamp_ns': 123, 'accepted_goal_uuid': 'goal-1'}
    mission = {'measurements': {'accepted_goal_stamp_ns': 123, 'accepted_goal_uuid': 'goal-1'}}
    scenario = {'binding': {'accepted_goal_stamp_ns': 123, 'goal_uuid': 'goal-1'}}
    assert (
        orchestration.reconcile_goal_binding(observer, mission, scenario)['immutable_exact_match']
        is True
    )
    scenario['binding']['accepted_goal_stamp_ns'] = 124
    with pytest.raises(orchestration.EvidenceError):
        orchestration.reconcile_goal_binding(observer, mission, scenario)
