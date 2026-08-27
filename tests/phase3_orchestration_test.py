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
import shutil
import sys
from types import SimpleNamespace

from jsonschema import Draft202012Validator
import pytest
import yaml

MODULE_PATH = Path(__file__).with_name('phase3_orchestration.py')
SPEC = importlib.util.spec_from_file_location('phase3_orchestration', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
orchestration = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = orchestration
SPEC.loader.exec_module(orchestration)

OBSERVER_PATH = Path(__file__).with_name('phase3_runtime_observer.py')
OBSERVER_SPEC = importlib.util.spec_from_file_location('phase3_runtime_observer', OBSERVER_PATH)
assert OBSERVER_SPEC is not None and OBSERVER_SPEC.loader is not None
runtime_observer = importlib.util.module_from_spec(OBSERVER_SPEC)
sys.modules[OBSERVER_SPEC.name] = runtime_observer
OBSERVER_SPEC.loader.exec_module(runtime_observer)

RUNTIME_GATE_PATH = Path(__file__).with_name('phase3_runtime_gate.py')
RUNTIME_GATE_MODULE = 'phase3_runtime_gate'
RUNTIME_GATE_SPEC = importlib.util.spec_from_file_location(
    RUNTIME_GATE_MODULE,
    RUNTIME_GATE_PATH,
)
assert RUNTIME_GATE_SPEC is not None and RUNTIME_GATE_SPEC.loader is not None
runtime_gate = importlib.util.module_from_spec(RUNTIME_GATE_SPEC)
sys.modules[RUNTIME_GATE_SPEC.name] = runtime_gate
RUNTIME_GATE_SPEC.loader.exec_module(runtime_gate)


def _contact_stream_contract() -> dict:
    workspace = Path(__file__).parents[1]
    policy = copy.deepcopy(orchestration.EXPECTED_CONTACT_STREAM_POLICY)
    source_paths = orchestration.CONTACT_GATE_SOURCE_PATHS
    source_inventory = {
        'schema_version': 1,
        'sources': [
            {
                'path': path,
                'sha256': orchestration.file_sha256(workspace / path),
            }
            for path in source_paths
        ],
    }
    return {
        'gate': {
            'executable': 'contact_stream_gate',
            'launch_sha256': orchestration.file_sha256(
                workspace / 'src/robotest_sim/launch/sim.launch.py'
            ),
            'package': 'robotest_sim',
            'source_inventory': source_inventory,
            'source_inventory_sha256': orchestration.canonical_sha256(source_inventory),
        },
        'policy': policy,
        'policy_sha256': orchestration.canonical_sha256(policy),
        'qos': {
            'private_raw_ros': {
                'depth': 64,
                'durability': 'VOLATILE',
                'history': 'KEEP_LAST',
                'reliability': 'RELIABLE',
            },
            'public_ros': {
                'depth': 10,
                'durability': 'VOLATILE',
                'history': 'KEEP_LAST',
                'reliability': 'RELIABLE',
            },
        },
        'schema_version': 1,
        'topics': {
            'gazebo_raw': '/robotest/internal/contact_aggregate',
            'private_raw_ros': '/robotest/internal/raw_contacts',
            'public_ros': '/robotest/validation/contacts',
        },
    }


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


def test_goal_observer_readiness_requires_endpoint_not_idle_status_message() -> None:
    publisher_count = 1
    node = SimpleNamespace(
        clock_ns=1,
        count_publishers=lambda _topic: publisher_count,
        status_message_count=0,
    )

    assert runtime_observer._goal_observer_ready(node)
    node.clock_ns = 0
    assert not runtime_observer._goal_observer_ready(node)
    node.clock_ns = 1
    publisher_count = 0
    assert not runtime_observer._goal_observer_ready(node)


def test_goal_observer_binds_first_executing_status_after_empty_prearm_window() -> None:
    observer = runtime_observer.GoalObserver()

    observer.arm()
    observer.observe(uuid='new', status=2, stamp_ns=2, clock_ns=3)

    assert observer.prearm_uuids == set()
    assert observer.bound_uuid == 'new'
    assert observer.bound_t0_ns == 2
    observer.observe(uuid='new', status=4, stamp_ns=2, clock_ns=4)
    with pytest.raises(orchestration.EvidenceError, match='UUID/T0 changed'):
        observer.observe(uuid='new', status=4, stamp_ns=3, clock_ns=5)


def test_goal_observer_rejects_terminal_first_status() -> None:
    observer = runtime_observer.GoalObserver()

    observer.arm()
    with pytest.raises(orchestration.EvidenceError, match='first observed only after'):
        observer.observe(uuid='new', status=4, stamp_ns=2, clock_ns=3)

    invalid = runtime_observer.GoalObserver()
    invalid.arm()
    with pytest.raises(orchestration.EvidenceError, match='invalid action status'):
        invalid.observe(uuid='new', status=0, stamp_ns=2, clock_ns=3)


def test_goal_observer_accepts_only_exact_bounded_arm_request(tmp_path: Path) -> None:
    arm_path = tmp_path / 'goal-observer.arm'

    assert not runtime_observer._arm_requested(arm_path)
    arm_path.write_bytes(runtime_observer.ARM_REQUEST)
    assert runtime_observer._arm_requested(arm_path)
    arm_path.write_bytes(b'wrong\n')
    with pytest.raises(orchestration.EvidenceError, match='payload is invalid'):
        runtime_observer._arm_requested(arm_path)


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
    expected = b'{"a":1,"b":2}\n'
    assert orchestration.canonical_json_bytes({'b': 2, 'a': 1}) == expected
    with pytest.raises(orchestration.EvidenceError):
        orchestration.canonical_json_bytes({'bad': float('nan')})


def test_contact_stream_manifest_requires_exact_v3_contract() -> None:
    workspace = Path(__file__).parents[1]
    manifest = {'schema_version': 3, 'contact_stream': _contact_stream_contract()}
    expected_contact_stream = manifest['contact_stream']
    actual_contact_stream = orchestration._contact_stream_manifest_v3(manifest, workspace)
    assert actual_contact_stream == expected_contact_stream

    with pytest.raises(orchestration.EvidenceError, match='schema_version 3'):
        orchestration._contact_stream_manifest_v3(
            {'schema_version': 2, 'contact_stream': _contact_stream_contract()},
            workspace,
        )

    mutations = []
    changed_policy = copy.deepcopy(manifest)
    changed_policy['contact_stream']['policy']['heartbeat_period_ns'] = 200_000_001
    changed_policy['contact_stream']['policy_sha256'] = orchestration.canonical_sha256(
        changed_policy['contact_stream']['policy']
    )
    mutations.append(changed_policy)
    changed_release = copy.deepcopy(manifest)
    changed_release['contact_stream']['policy']['release_comparison'] = (
        'completed_absent_stamp_greater_than_or_equal_to_last_seen_plus_gap'
    )
    changed_release['contact_stream']['policy_sha256'] = orchestration.canonical_sha256(
        changed_release['contact_stream']['policy']
    )
    mutations.append(changed_release)
    changed_topic = copy.deepcopy(manifest)
    changed_topic['contact_stream']['topics']['private_raw_ros'] = '/robotest/raw_contacts'
    mutations.append(changed_topic)
    changed_qos = copy.deepcopy(manifest)
    changed_qos['contact_stream']['qos']['public_ros']['depth'] = 11
    mutations.append(changed_qos)
    changed_owner = copy.deepcopy(manifest)
    changed_owner['contact_stream']['gate']['executable'] = 'legacy_contact_gate'
    mutations.append(changed_owner)
    changed_source = copy.deepcopy(manifest)
    changed_inventory = changed_source['contact_stream']['gate']['source_inventory']
    changed_inventory['sources'][0]['sha256'] = '0' * 64
    changed_source['contact_stream']['gate']['source_inventory_sha256'] = (
        orchestration.canonical_sha256(changed_inventory)
    )
    mutations.append(changed_source)

    for changed in mutations:
        with pytest.raises(orchestration.EvidenceError):
            orchestration._contact_stream_manifest_v3(changed, workspace)


def test_generated_contact_stream_configuration_round_trips_v3_contract() -> None:
    workspace = Path(__file__).parents[1]
    generator_relative_path = 'src/robotest_description/tools/generate_collision_coverage.py'
    generator_path = workspace / generator_relative_path
    spec = importlib.util.spec_from_file_location(
        'collision_coverage_generator_v3',
        generator_path,
    )
    assert spec is not None and spec.loader is not None
    generator = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(generator)
    contact_stream = generator.contact_stream_configuration(workspace)
    manifest = {'schema_version': 3, 'contact_stream': contact_stream}
    assert orchestration._contact_stream_manifest_v3(manifest, workspace) == contact_stream


def _contact_aggregator_reobservation_fixture(
    *, source_inventory_sha256: str = '4' * 64
) -> tuple[dict, dict]:
    digest = source_inventory_sha256
    binary = {
        field: True
        for field in orchestration.CONTACT_AGGREGATOR_BINARY_FIELDS
        if field.endswith('_match') or field.endswith('_file')
    } | {
        'build_elf_build_id': 'b2',
        'build_embedded_source_inventory_sha256': digest,
        'build_install_samefile': True,
        'build_path': orchestration.CONTACT_AGGREGATOR_BUILD_PATH,
        'build_sha256': '6' * 64,
        'installed_declared_is_symlink': True,
        'installed_declared_path': orchestration.CONTACT_AGGREGATOR_INSTALLED_PATH,
        'installed_elf_build_id': 'b2',
        'installed_embedded_source_inventory_sha256': digest,
        'installed_path': orchestration.CONTACT_AGGREGATOR_BUILD_PATH,
        'installed_sha256': '6' * 64,
        'package': 'robotest_sim',
        'schema_version': 1,
        'source_inventory_sha256': digest,
    }
    stable_identity = {
        'build_elf_build_id': binary['build_elf_build_id'],
        'build_path': binary['build_path'],
        'build_sha256': binary['build_sha256'],
        'installed_declared_path': binary['installed_declared_path'],
        'installed_device': 9,
        'installed_elf_build_id': binary['installed_elf_build_id'],
        'installed_embedded_source_inventory_sha256': digest,
        'installed_inode': 10,
        'installed_path': binary['installed_path'],
        'installed_sha256': binary['installed_sha256'],
        'launch_root_pid': 100,
        'live_cmdline_sha256': '7' * 64,
        'live_executable_link': '/usr/bin/gz',
        'live_executable_path': '/usr/bin/gz',
        'live_mapping_device': 9,
        'live_mapping_fingerprint_sha256': '8' * 64,
        'live_mapping_inode': 10,
        'live_mapping_paths': ['/workspace/librobotest_contact_aggregator_system.so'],
        'live_pgid': 100,
        'live_pid': 103,
        'live_ppid': 100,
        'live_sid': 100,
        'live_start_ticks': 3456,
        'observed_gz_partition': 'robotest_p3_candidate_trial',
        'observed_ros_domain_id': '100',
        'source_inventory_sha256': digest,
    }
    attestation = {
        **binary,
        'attestation_method': 'proc_maps_exact_device_inode',
        'exact_live_process_count': 1,
        'identity_revalidated_after_hashing': True,
        'installed_device': 9,
        'installed_identity_revalidated_after_hashing': True,
        'installed_inode': 10,
        'installed_size_bytes': 2_000,
        'launch_root_pid': 100,
        'live_cmdline_sha256': '7' * 64,
        'live_elf_build_id': binary['installed_elf_build_id'],
        'live_embedded_source_inventory_match': True,
        'live_embedded_source_inventory_sha256': digest,
        'live_executable_link': '/usr/bin/gz',
        'live_executable_path': '/usr/bin/gz',
        'live_installed_build_id_match': True,
        'live_installed_inode_match': True,
        'live_installed_sha256_match': True,
        'live_mapping_count': 5,
        'live_mapping_device': 9,
        'live_mapping_fingerprint_sha256': '8' * 64,
        'live_mapping_has_executable': True,
        'live_mapping_has_offset_zero': True,
        'live_mapping_inode': 10,
        'live_mapping_paths': stable_identity['live_mapping_paths'],
        'live_pgid': 100,
        'live_pid': 103,
        'live_ppid': 100,
        'live_sid': 100,
        'live_start_ticks': 3456,
        'maps_revalidated_after_hashing': True,
        'observed_gz_partition': 'robotest_p3_candidate_trial',
        'observed_ros_domain_id': '100',
        'process_identity_match': True,
        'stable_identity': stable_identity,
        'stable_identity_sha256': orchestration.canonical_sha256(stable_identity),
        'verdict': 'PASS',
    }
    return binary, attestation


def test_contact_gate_reobservation_binds_build_and_rejects_rebound(
    tmp_path: Path,
) -> None:
    attestation = {field: 0 for field in orchestration.CONTACT_GATE_ATTESTATION_FIELDS}
    attestation.update(
        {
            'build_embedded_source_inventory_match': True,
            'build_embedded_source_inventory_sha256': '4' * 64,
            'build_elf_build_id': 'a1',
            'build_install_build_id_match': True,
            'build_install_samefile': False,
            'build_install_sha256_match': True,
            'build_path': 'build/robotest_sim/contact_stream_gate',
            'build_regular_executable': True,
            'build_sha256': '2' * 64,
            'exact_live_process_count': 1,
            'identity_revalidated_after_hashing': True,
            'installed_declared_is_symlink': False,
            'installed_declared_path': 'install/robotest_sim/lib/robotest_sim/contact_stream_gate',
            'installed_declared_samefile': True,
            'installed_device': 7,
            'installed_embedded_source_inventory_match': True,
            'installed_embedded_source_inventory_sha256': '4' * 64,
            'installed_elf_build_id': 'a1',
            'installed_inode': 8,
            'installed_path': 'install/robotest_sim/lib/robotest_sim/contact_stream_gate',
            'installed_regular_executable': True,
            'installed_sha256': '2' * 64,
            'launch_root_pid': 100,
            'live_cmdline_sha256': '3' * 64,
            'live_device': 7,
            'live_embedded_source_inventory_match': True,
            'live_embedded_source_inventory_sha256': '4' * 64,
            'live_elf_build_id': 'a1',
            'live_executable_link': (
                '/workspace/install/robotest_sim/lib/robotest_sim/contact_stream_gate'
            ),
            'live_executable_path': (
                '/workspace/install/robotest_sim/lib/robotest_sim/contact_stream_gate'
            ),
            'live_executable_sha256': '2' * 64,
            'live_inode': 8,
            'live_installed_build_id_match': True,
            'live_installed_inode_match': True,
            'live_installed_sha256_match': True,
            'live_pgid': 100,
            'live_pid': 101,
            'live_ppid': 100,
            'live_sid': 100,
            'live_size_bytes': 1_000,
            'live_start_ticks': 1234,
            'observed_gz_partition': 'robotest_p3_candidate_trial',
            'observed_ros_domain_id': '100',
            'package': 'robotest_sim',
            'process_identity_match': True,
            'schema_version': 1,
            'source_inventory_sha256': '4' * 64,
            'verdict': 'PASS',
        }
    )
    aggregator_binary, aggregator_attestation = _contact_aggregator_reobservation_fixture()
    initial_path = tmp_path / 'runtime-gate.json'
    final_path = tmp_path / 'contact-stream-final-gate.json'
    document = {
        'contact_aggregator_binary_attestation': aggregator_attestation,
        'contact_gate_binary_attestation': attestation,
        'verdict': 'PASS',
    }
    orchestration.atomic_write_json(initial_path, document, sidecar=True)
    orchestration.atomic_write_json(final_path, document, sidecar=True)
    build_binding = {
        'contact_aggregator_binary': aggregator_binary,
        'contact_gate_binary': {
            'build_embedded_source_inventory_match': True,
            'build_embedded_source_inventory_sha256': '4' * 64,
            'build_elf_build_id': 'a1',
            'build_install_build_id_match': True,
            'build_install_samefile': False,
            'build_install_sha256_match': True,
            'build_path': 'build/robotest_sim/contact_stream_gate',
            'build_regular_executable': True,
            'build_sha256': '2' * 64,
            'installed_declared_is_symlink': False,
            'installed_declared_path': (
                'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
            ),
            'installed_declared_samefile': True,
            'installed_embedded_source_inventory_match': True,
            'installed_embedded_source_inventory_sha256': '4' * 64,
            'installed_elf_build_id': 'a1',
            'installed_path': 'install/robotest_sim/lib/robotest_sim/contact_stream_gate',
            'installed_regular_executable': True,
            'installed_sha256': '2' * 64,
            'package': 'robotest_sim',
            'schema_version': 1,
            'source_inventory_sha256': '4' * 64,
        },
    }
    evidence = orchestration.reconcile_contact_gate_reobservation(
        initial_path,
        final_path,
        build_binding=build_binding,
        expected_domain_id=100,
        expected_gz_partition='robotest_p3_candidate_trial',
    )
    assert evidence['stable_identity'] is True

    rebound = copy.deepcopy(document)
    rebound['contact_gate_binary_attestation']['live_start_ticks'] = 1235
    orchestration.atomic_write_json(final_path, rebound, sidecar=True)
    with pytest.raises(orchestration.EvidenceError, match='identity changed'):
        orchestration.reconcile_contact_gate_reobservation(
            initial_path,
            final_path,
            build_binding=build_binding,
            expected_domain_id=100,
            expected_gz_partition='robotest_p3_candidate_trial',
        )

    orchestration.atomic_write_json(final_path, document, sidecar=True)
    forged_build = copy.deepcopy(build_binding)
    forged_build['contact_gate_binary']['installed_sha256'] = '9' * 64
    with pytest.raises(orchestration.EvidenceError, match='prelaunch build binding'):
        orchestration.reconcile_contact_gate_reobservation(
            initial_path,
            final_path,
            build_binding=forged_build,
            expected_domain_id=100,
            expected_gz_partition='robotest_p3_candidate_trial',
        )


@pytest.mark.parametrize(
    'extractor',
    (
        orchestration._elf_embedded_source_inventory_sha256,
        runtime_gate._elf_embedded_source_inventory_sha256,
    ),
)
def test_contact_gate_tagged_digest_extraction_is_distinct_and_chunk_safe(
    tmp_path: Path, extractor: object
) -> None:
    tag = b'ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
    first = b'1' * 64
    second = b'2' * 64
    path = tmp_path / 'gate.elf'

    path.write_bytes(tag + first + b'\0debug-copy\0' + tag + first)
    assert extractor(path) == first.decode('ascii')

    path.write_bytes(tag + first + b'\0' + tag + second)
    with pytest.raises(orchestration.EvidenceError, match='conflicting tagged'):
        extractor(path)

    path.write_bytes(b'x' * (1024 * 1024 - 10) + tag + first + b'\0')
    assert extractor(path) == first.decode('ascii')


def test_contact_gate_build_install_rejects_same_build_id_with_appended_byte(
    tmp_path: Path,
) -> None:
    for relative in orchestration.CONTACT_GATE_SOURCE_PATHS:
        source = tmp_path / relative
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(f'{relative}\n', encoding='utf-8')
    inventory_sha256 = orchestration.canonical_sha256(
        orchestration.contact_gate_source_inventory(tmp_path)
    )
    build = tmp_path / 'build/robotest_sim/contact_stream_gate'
    installed = tmp_path / 'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
    build.parent.mkdir(parents=True)
    installed.parent.mkdir(parents=True)
    tagged_elf = Path(sys.executable).read_bytes() + (
        b'\0ROBOTEST_CONTACT_GATE_SOURCE_INVENTORY_SHA256='
        + inventory_sha256.encode('ascii')
        + b'\0'
    )
    build.write_bytes(tagged_elf)
    shutil.copymode(sys.executable, build)
    shutil.copy2(build, installed)
    binding = orchestration.contact_gate_build_install_binding(tmp_path)
    assert binding['build_install_sha256_match'] is True
    assert binding['installed_declared_is_symlink'] is False
    assert binding['build_install_samefile'] is False
    assert binding['installed_path'] == orchestration.CONTACT_GATE_INSTALLED_PATH

    with installed.open('ab') as stream:
        stream.write(b'x')
    assert orchestration._elf_build_id(build) == orchestration._elf_build_id(installed)
    assert orchestration._elf_embedded_source_inventory_sha256(
        build
    ) == orchestration._elf_embedded_source_inventory_sha256(installed)
    with pytest.raises(orchestration.EvidenceError, match='hashes differ'):
        orchestration.contact_gate_build_install_binding(tmp_path)


def _contact_gate_binary_analysis_schema() -> dict[str, object]:
    schema = json.loads(
        (
            Path(__file__).parents[1] / 'src/robotest_metrics/schema/analysis-request.schema.json'
        ).read_text(encoding='utf-8')
    )
    gate_schema = schema['$defs']['orchestrator']['properties']['source_binding']['properties'][
        'contact_gate_binary'
    ]
    return {**gate_schema, '$defs': schema['$defs']}


def test_current_contact_gate_build_binding_matches_analysis_schema() -> None:
    workspace = Path(__file__).parents[1]
    binding = orchestration.contact_gate_build_install_binding(workspace)
    assert binding['installed_declared_is_symlink'] is True
    assert binding['build_install_samefile'] is True
    assert binding['installed_path'] == orchestration.CONTACT_GATE_BUILD_PATH
    assert (
        list(Draft202012Validator(_contact_gate_binary_analysis_schema()).iter_errors(binding))
        == []
    )


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('installed_declared_is_symlink', 1),
        ('installed_path', orchestration.CONTACT_GATE_INSTALLED_PATH),
        ('build_install_samefile', False),
    ],
)
def test_contact_gate_analysis_schema_rejects_symlink_binding_tampering(
    field: str,
    value: object,
) -> None:
    binding = orchestration.contact_gate_build_install_binding(Path(__file__).parents[1])
    binding[field] = value
    assert list(Draft202012Validator(_contact_gate_binary_analysis_schema()).iter_errors(binding))


def test_utf8_string_bound_is_bytes_not_codepoints() -> None:
    assert orchestration.require_bounded_string('x' * 4096, 'value')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.require_bounded_string('\u00e9' * 4096, 'value')


def test_contact_episode_projection_groups_all_pairs_by_external_counterpart() -> None:
    manifest = yaml.safe_load(
        (Path(__file__).parents[1] / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
    )
    first_robot, second_robot = [entry['name'] for entry in manifest['robot_collisions'][:2]]
    wall_pair_a = tuple(sorted((first_robot, 'wall::link::collision')))
    wall_pair_b = tuple(sorted((second_robot, 'wall::link::collision')))
    support = manifest['support_pairs'][0]
    support_pair = tuple(sorted((support['robot_collision'], support['environment_collision'])))

    simultaneous = orchestration._captured_contact_episodes(
        [
            (100, tuple(sorted((wall_pair_a, wall_pair_b)))),
            (200, (support_pair,)),
        ],
        manifest=manifest,
    )
    assert simultaneous == [
        {
            'counterpart_model': 'wall',
            'end_stamp_ns': 200,
            'normalized_pairs': [list(pair) for pair in sorted((wall_pair_a, wall_pair_b))],
            'snapshot_record_count': 2,
            'start_stamp_ns': 100,
        }
    ]

    migration = orchestration._captured_contact_episodes(
        [(100, (wall_pair_a,)), (200, (wall_pair_b,)), (300, (support_pair,))],
        manifest=manifest,
    )
    assert len(migration) == 1
    assert migration[0]['start_stamp_ns'] == 100
    assert migration[0]['end_stamp_ns'] == 300
    assert migration[0]['snapshot_record_count'] == 2
    assert migration[0]['normalized_pairs'] == [
        list(pair) for pair in sorted((wall_pair_a, wall_pair_b))
    ]

    distinct_counterparts = orchestration._captured_contact_episodes(
        [
            (
                100,
                tuple(
                    sorted(
                        (
                            wall_pair_a,
                            tuple(sorted((second_robot, 'box::link::collision'))),
                        )
                    )
                ),
            ),
            (200, (support_pair,)),
        ],
        manifest=manifest,
    )
    assert [episode['counterpart_model'] for episode in distinct_counterparts] == [
        'box',
        'wall',
    ]


def test_contact_drain_requires_strict_snapshot_and_clock_catch_up() -> None:
    state = runtime_observer.ContactDrainObserver(1_000_000_000)
    pair_a = frozenset({('robot::link::collision', 'wall::link::collision')})
    pair_b = frozenset({('robot::link::collision', 'box::link::collision')})
    state.observe_contact(stamp_ns=1_250_000_000, frame_id='', pair_set=pair_a, record_count=1)
    assert state.qualifying_contact_snapshot_stamp_ns is None
    state.observe_contact(stamp_ns=1_250_000_001, frame_id='', pair_set=pair_b, record_count=1)
    assert state.qualifying_contact_snapshot_stamp_ns == 1_250_000_001
    state.observe_clock(1_250_000_000)
    assert state.complete() is False
    state.observe_clock(1_470_000_001)
    assert state.complete() is True
    assert state.evidence()['clock_minus_qualifying_contact_ns'] == 220_000_000


def test_contact_drain_source_gap_and_clock_lag_boundaries() -> None:
    pair = frozenset({('robot::link::collision', 'wall::link::collision')})
    accepted = runtime_observer.ContactDrainObserver(100_000_000)
    accepted.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    accepted.observe_contact(stamp_ns=520_000_000, frame_id='', pair_set=pair, record_count=1)
    accepted.observe_clock(740_000_000)
    assert accepted.complete() is True

    gap_failure = runtime_observer.ContactDrainObserver(1)
    gap_failure.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    with pytest.raises(orchestration.EvidenceError, match='source gap'):
        gap_failure.observe_contact(
            stamp_ns=520_000_001, frame_id='', pair_set=pair, record_count=1
        )

    lag_failure = runtime_observer.ContactDrainObserver(1)
    lag_failure.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    lag_failure.observe_clock(520_000_001)
    with pytest.raises(orchestration.EvidenceError, match='220 ms'):
        lag_failure.complete()


def test_contact_drain_rejects_duplicate_regression_frame_and_early_heartbeat() -> None:
    pair = frozenset({('robot::link::collision', 'wall::link::collision')})
    duplicate = runtime_observer.ContactDrainObserver(1)
    duplicate.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    with pytest.raises(orchestration.EvidenceError, match='duplicated'):
        duplicate.observe_contact(
            stamp_ns=300_000_000,
            frame_id='',
            pair_set=pair,
            record_count=1,
        )

    regression = runtime_observer.ContactDrainObserver(1)
    regression.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    with pytest.raises(orchestration.EvidenceError, match='regressed'):
        regression.observe_contact(
            stamp_ns=299_999_999,
            frame_id='',
            pair_set=pair,
            record_count=1,
        )

    frame = runtime_observer.ContactDrainObserver(1)
    with pytest.raises(orchestration.EvidenceError, match='frame_id'):
        frame.observe_contact(
            stamp_ns=300_000_000,
            frame_id='world',
            pair_set=pair,
            record_count=1,
        )

    heartbeat = runtime_observer.ContactDrainObserver(1)
    heartbeat.observe_contact(stamp_ns=300_000_000, frame_id='', pair_set=pair, record_count=1)
    with pytest.raises(orchestration.EvidenceError, match='before 200 ms'):
        heartbeat.observe_contact(stamp_ns=499_999_999, frame_id='', pair_set=pair, record_count=1)

    records = runtime_observer.ContactDrainObserver(1)
    with pytest.raises(orchestration.EvidenceError, match='1 to 16'):
        records.observe_contact(
            stamp_ns=300_000_000, frame_id='', pair_set=frozenset(), record_count=0
        )
    with pytest.raises(orchestration.EvidenceError, match='1 to 16'):
        records.observe_contact(stamp_ns=300_000_001, frame_id='', pair_set=pair, record_count=17)


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
        'contact_stream': _contact_stream_contract(),
        'contact_configuration_sha256': '2' * 64,
        'contact_topic': '/robotest/validation/contacts',
        'covered_collisions': [],
        'rendered_robot_collisions': [],
        'rendered_sdf_sha256': '3' * 64,
        'robot_collisions': [
            {'name': 'robotest::base::collision'},
            {'name': 'robotest::wheel::collision'},
        ],
        'robot_description_sha256': '4' * 64,
        'robot_model': 'robotest',
        'schema_version': 3,
        'support_pairs': [
            {
                'environment_collision': 'ground::plane::collision',
                'robot_collision': 'robotest::wheel::collision',
            }
        ],
        'world_source_sha256': '5' * 64,
    }
    manifest = {**unsigned, 'manifest_sha256': orchestration.canonical_sha256(unsigned)}
    source_inventory_sha256 = manifest['contact_stream']['gate']['source_inventory_sha256']
    gate_binary = {'source_inventory_sha256': source_inventory_sha256}
    positive_build_binding = {
        'contact_aggregator_binary': {
            'source_inventory_sha256': source_inventory_sha256,
        },
        'contact_gate_binary': gate_binary,
    }
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
                'episodes': [
                    {
                        'counterpart_model': 'wall',
                        'end_stamp_ns': 300_000_000,
                        'normalized_pairs': [
                            ['robotest::base::collision', 'wall::link::collision']
                        ],
                        'snapshot_record_count': 1,
                        'start_stamp_ns': 100_000_000,
                    }
                ],
                'exact_pair_snapshot_record_count': 1,
                'expected_pair': [
                    'robotest::base::collision',
                    'wall::link::collision',
                ],
                'first_qualifying_contact': {'sim_stamp_ns': 100_000_000},
                'snapshot_records': [
                    {
                        'collector_sequence': 2,
                        'counterpart_collision': 'wall::link::collision',
                        'counterpart_model': 'wall',
                        'disposition': 'counted',
                        'normalized_pair': [
                            'robotest::base::collision',
                            'wall::link::collision',
                        ],
                        'robot_collision': 'robotest::base::collision',
                        'sim_stamp_ns': 100_000_000,
                        'snapshot_sequence': 1,
                    },
                    {
                        'collector_sequence': 4,
                        'counterpart_collision': None,
                        'counterpart_model': None,
                        'disposition': 'support_ground_excluded',
                        'normalized_pair': [
                            'ground::plane::collision',
                            'robotest::wheel::collision',
                        ],
                        'robot_collision': None,
                        'sim_stamp_ns': 300_000_000,
                        'snapshot_sequence': 3,
                    },
                ],
                'snapshots': [
                    {
                        'classified_count': 1,
                        'collector_sequence': 1,
                        'counted_snapshot_records': [
                            {
                                'counterpart_model': 'wall',
                                'normalized_pair': [
                                    'robotest::base::collision',
                                    'wall::link::collision',
                                ],
                                'record_sequence': 2,
                                'snapshot_sequence': 1,
                            }
                        ],
                        'delivery_clock_offset_ns': 0,
                        'delivery_clock_stamp_ns': 100_000_000,
                        'exact_pair_count': 1,
                        'sim_stamp_ns': 100_000_000,
                        'snapshot_record_count': 1,
                    },
                    {
                        'classified_count': 0,
                        'collector_sequence': 3,
                        'counted_snapshot_records': [],
                        'delivery_clock_offset_ns': 200_000_000,
                        'delivery_clock_stamp_ns': 500_000_000,
                        'exact_pair_count': 0,
                        'sim_stamp_ns': 300_000_000,
                        'snapshot_record_count': 1,
                    },
                ],
            },
            'timeline': {
                'release_qualified_snapshot_stamp_ns': 300_000_000,
                'release_required_through_stamp_ns': 250_000_000,
            },
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
        'clock': {'latest_stamp_ns': 600_000_000, 'regression_count': 0},
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
                                'collision1': 'robotest::base::collision',
                                'collision2': 'wall::link::collision',
                            }
                        ],
                        'delivery_clock_offset_ns': 0,
                        'delivery_clock_stamp_ns': 100_000_000,
                        'frame_id': '',
                        'stamp_ns': 100_000_000,
                    },
                    {
                        'contacts': [
                            {
                                'collision1': 'robotest::wheel::collision',
                                'collision2': 'ground::plane::collision',
                            }
                        ],
                        'delivery_clock_offset_ns': 200_000_000,
                        'delivery_clock_stamp_ns': 500_000_000,
                        'frame_id': '',
                        'stamp_ns': 300_000_000,
                    },
                ]
            },
        },
    }
    capture_path = tmp_path / 'capture.json'
    orchestration.atomic_write_json(capture_path, capture)
    contact_progress_path = tmp_path / 'contact-progress.json'
    contact_progress = {
        'latest_retained_stamp_ns': 300_000_000,
        'producer': 'robotest_metrics/metrics_collector',
        'public_topic': '/robotest/validation/contacts',
        'retained_message_count': 2,
        'schema_version': 1,
    }
    orchestration.atomic_write_json(contact_progress_path, contact_progress)
    bound = orchestration.reconcile_positive_control(
        workspace=Path(__file__).parents[1],
        build_binding=positive_build_binding,
        result_path=result_path,
        capture_path=capture_path,
        contact_progress_path=contact_progress_path,
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
    assert bound['collector_reconciliation'] == {
        'captured_command_count': 2,
        'captured_exact_pair_count': 1,
        'captured_release_expected_pair_count': 0,
        'captured_release_snapshot_count': 1,
        'contact_projection_episode_count': 1,
        'contact_projection_first_stamp_ns': 100_000_000,
        'contact_projection_record_count': 2,
        'contact_projection_sha256': orchestration.canonical_sha256(
            [
                {
                    'normalized_pairs': [['robotest::base::collision', 'wall::link::collision']],
                    'stamp_ns': 100_000_000,
                },
                {
                    'normalized_pairs': [
                        ['ground::plane::collision', 'robotest::wheel::collision']
                    ],
                    'stamp_ns': 300_000_000,
                },
            ]
        ),
        'contact_projection_snapshot_count': 2,
        'contact_progress_artifact_sha256': orchestration.file_sha256(contact_progress_path),
        'contact_progress_latest_retained_stamp_ns': 300_000_000,
        'contact_progress_retained_message_count': 2,
        'component_command_count': 2,
        'component_exact_pair_count': 1,
        'latest_clock_stamp_ns': 600_000_000,
        'release_delivery_clock_offset_ns': 200_000_000,
        'release_delivery_clock_stamp_ns': 500_000_000,
        'release_qualified_snapshot_stamp_ns': 300_000_000,
        'release_required_through_stamp_ns': 250_000_000,
    }

    missing_repeated_command = copy.deepcopy(result)
    missing_repeated_command['control']['command_trace'][1]['collector_sequence'] = 3
    missing_repeated_command['control']['command_trace'].insert(
        1,
        {
            'angular_z': 0.0,
            'collector_sequence': 2,
            'linear_x': 0.05,
            'phase': 'forward',
            'sim_stamp_ns': 15,
        },
    )
    orchestration.atomic_write_json(result_path, missing_repeated_command, sidecar=True)
    with pytest.raises(orchestration.EvidenceError, match='complete component subsequence'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )

    result_with_support_suffix = copy.deepcopy(result)
    result_with_support_suffix['control']['contact']['snapshot_records'].append(
        {
            'collector_sequence': 6,
            'counterpart_collision': None,
            'counterpart_model': None,
            'disposition': 'robot_internal_excluded',
            'normalized_pair': [
                'robotest::base::collision',
                'robotest::wheel::collision',
            ],
            'robot_collision': None,
            'sim_stamp_ns': 500_000_000,
            'snapshot_sequence': 5,
        }
    )
    result_with_support_suffix['control']['contact']['snapshots'].append(
        {
            'classified_count': 0,
            'collector_sequence': 5,
            'counted_snapshot_records': [],
            'delivery_clock_offset_ns': 0,
            'delivery_clock_stamp_ns': 500_000_000,
            'exact_pair_count': 0,
            'sim_stamp_ns': 500_000_000,
            'snapshot_record_count': 1,
        }
    )
    capture_with_support_suffix = copy.deepcopy(capture)
    capture_with_support_suffix['streams']['contacts']['items'].append(
        {
            'contacts': [
                {
                    'collision1': 'robotest::base::collision',
                    'collision2': 'robotest::wheel::collision',
                }
            ],
            'delivery_clock_offset_ns': 0,
            'delivery_clock_stamp_ns': 500_000_000,
            'frame_id': '',
            'stamp_ns': 500_000_000,
        }
    )
    support_suffix_progress = {
        **contact_progress,
        'latest_retained_stamp_ns': 500_000_000,
        'retained_message_count': 3,
    }
    orchestration.atomic_write_json(result_path, result_with_support_suffix, sidecar=True)
    orchestration.atomic_write_json(capture_path, capture_with_support_suffix)
    orchestration.atomic_write_json(contact_progress_path, support_suffix_progress)
    support_suffix_bound = orchestration.reconcile_positive_control(
        workspace=Path(__file__).parents[1],
        build_binding=positive_build_binding,
        result_path=result_path,
        capture_path=capture_path,
        contact_progress_path=contact_progress_path,
        manifest_path=manifest_path,
        collector_configuration_sha256='7' * 64,
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    assert (
        support_suffix_bound['collector_reconciliation']['contact_projection_snapshot_count'] == 2
    )
    assert (
        support_suffix_bound['collector_reconciliation'][
            'contact_progress_latest_retained_stamp_ns'
        ]
        == 500_000_000
    )

    missing_qualified = copy.deepcopy(result)
    missing_snapshots = missing_qualified['control']['contact']['snapshots']
    missing_records = missing_qualified['control']['contact']['snapshot_records']
    missing_snapshots[1]['sim_stamp_ns'] = 301_000_000
    missing_snapshots[1]['delivery_clock_stamp_ns'] = 501_000_000
    missing_records[1]['sim_stamp_ns'] = 301_000_000
    orchestration.atomic_write_json(result_path, missing_qualified, sidecar=True)
    orchestration.atomic_write_json(capture_path, capture)
    orchestration.atomic_write_json(contact_progress_path, contact_progress)
    with pytest.raises(orchestration.EvidenceError, match='not present exactly once'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )

    duplicated_qualified = copy.deepcopy(result_with_support_suffix)
    duplicated_qualified['control']['contact']['snapshots'][2]['sim_stamp_ns'] = 300_000_000
    duplicated_qualified['control']['contact']['snapshots'][2]['delivery_clock_stamp_ns'] = (
        300_000_000
    )
    duplicated_qualified['control']['contact']['snapshot_records'][2]['sim_stamp_ns'] = 300_000_000
    orchestration.atomic_write_json(result_path, duplicated_qualified, sidecar=True)
    with pytest.raises(orchestration.EvidenceError, match='ordering/liveness'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )

    countable_recontact = copy.deepcopy(result_with_support_suffix)
    countable_recontact_record = countable_recontact['control']['contact']['snapshot_records'][2]
    countable_recontact_record.update(
        {
            'counterpart_collision': 'second_wall::link::collision',
            'counterpart_model': 'second_wall',
            'disposition': 'counted',
            'normalized_pair': [
                'robotest::wheel::collision',
                'second_wall::link::collision',
            ],
            'robot_collision': 'robotest::wheel::collision',
        }
    )
    orchestration.atomic_write_json(result_path, countable_recontact, sidecar=True)
    with pytest.raises(orchestration.EvidenceError, match='countable contact'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )

    orchestration.atomic_write_json(result_path, result, sidecar=True)
    orchestration.atomic_write_json(capture_path, capture)
    orchestration.atomic_write_json(contact_progress_path, contact_progress)

    diagnostic_offset = copy.deepcopy(capture)
    diagnostic_offset['streams']['contacts']['items'][1]['delivery_clock_stamp_ns'] = 578_000_000
    diagnostic_offset['streams']['contacts']['items'][1]['delivery_clock_offset_ns'] = 278_000_000
    orchestration.atomic_write_json(capture_path, diagnostic_offset)
    diagnostic_bound = orchestration.reconcile_positive_control(
        workspace=Path(__file__).parents[1],
        build_binding=positive_build_binding,
        result_path=result_path,
        capture_path=capture_path,
        contact_progress_path=contact_progress_path,
        manifest_path=manifest_path,
        collector_configuration_sha256='7' * 64,
        owned_process_group_shutdown=True,
        checksum_verified=True,
    )
    assert (
        diagnostic_bound['collector_reconciliation']['release_delivery_clock_offset_ns']
        == 278_000_000
    )

    inconsistent_offset = copy.deepcopy(diagnostic_offset)
    inconsistent_contact_items = inconsistent_offset['streams']['contacts']['items']
    inconsistent_contact_items[1]['delivery_clock_offset_ns'] = 278_000_001
    orchestration.atomic_write_json(capture_path, inconsistent_offset)
    with pytest.raises(orchestration.EvidenceError, match='offset is inconsistent'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )

    projection_mutations = []
    extra_recontact = copy.deepcopy(capture)
    extra_recontact['streams']['contacts']['items'][1]['contacts'][0]['collision2'] = (
        'wall::link::collision'
    )
    projection_mutations.append(extra_recontact)
    missing_snapshot = copy.deepcopy(capture)
    missing_snapshot['streams']['contacts']['items'].pop(0)
    projection_mutations.append(missing_snapshot)
    membership_mismatch = copy.deepcopy(capture)
    membership_mismatch['streams']['contacts']['items'][1]['contacts'][0]['collision2'] = (
        'robotest::other::collision'
    )
    projection_mutations.append(membership_mismatch)
    duplicate_multiplicity = copy.deepcopy(capture)
    duplicate_multiplicity['streams']['contacts']['items'][0]['contacts'].append(
        copy.deepcopy(duplicate_multiplicity['streams']['contacts']['items'][0]['contacts'][0])
    )
    projection_mutations.append(duplicate_multiplicity)
    for changed_capture in projection_mutations:
        orchestration.atomic_write_json(capture_path, changed_capture)
        with pytest.raises(orchestration.EvidenceError, match='not bijective'):
            orchestration.reconcile_positive_control(
                workspace=Path(__file__).parents[1],
                build_binding=positive_build_binding,
                result_path=result_path,
                capture_path=capture_path,
                contact_progress_path=contact_progress_path,
                manifest_path=manifest_path,
                collector_configuration_sha256='7' * 64,
                owned_process_group_shutdown=True,
                checksum_verified=True,
            )
    orchestration.atomic_write_json(capture_path, capture)

    rebound_progress = {**contact_progress, 'latest_retained_stamp_ns': 300_000_001}
    orchestration.atomic_write_json(contact_progress_path, rebound_progress)
    with pytest.raises(orchestration.EvidenceError, match='final capture'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )
    count_mismatch = {**contact_progress, 'retained_message_count': 3}
    orchestration.atomic_write_json(contact_progress_path, count_mismatch)
    with pytest.raises(orchestration.EvidenceError, match='capture length'):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
            manifest_path=manifest_path,
            collector_configuration_sha256='7' * 64,
            owned_process_group_shutdown=True,
            checksum_verified=True,
        )
    orchestration.atomic_write_json(contact_progress_path, contact_progress)

    tampered = copy.deepcopy(manifest)
    tampered['bridge_sha256'] = '8' * 64
    manifest_path.write_text(yaml.safe_dump(tampered, sort_keys=True), encoding='utf-8')
    with pytest.raises(orchestration.EvidenceError):
        orchestration.reconcile_positive_control(
            workspace=Path(__file__).parents[1],
            build_binding=positive_build_binding,
            result_path=result_path,
            capture_path=capture_path,
            contact_progress_path=contact_progress_path,
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
        'contact_aggregator_binary': _contact_aggregator_reobservation_fixture(
            source_inventory_sha256='8' * 64
        )[0],
        'contact_gate_binary': {
            'build_embedded_source_inventory_match': True,
            'build_embedded_source_inventory_sha256': '8' * 64,
            'build_elf_build_id': 'a1',
            'build_install_build_id_match': True,
            'build_install_samefile': True,
            'build_install_sha256_match': True,
            'build_path': 'build/robotest_sim/contact_stream_gate',
            'build_regular_executable': True,
            'build_sha256': '9' * 64,
            'installed_declared_is_symlink': True,
            'installed_declared_path': (
                'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
            ),
            'installed_declared_samefile': True,
            'installed_embedded_source_inventory_match': True,
            'installed_embedded_source_inventory_sha256': '8' * 64,
            'installed_elf_build_id': 'a1',
            'installed_path': 'build/robotest_sim/contact_stream_gate',
            'installed_regular_executable': True,
            'installed_sha256': '9' * 64,
            'package': 'robotest_sim',
            'schema_version': 1,
            'source_inventory_sha256': '8' * 64,
        },
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
