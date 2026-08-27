# Copyright 2026 Hasan Ahmed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import copy
import json
from pathlib import Path
from typing import Any

import pytest
import robotest_metrics.collision_metrics as collision_module
from conftest import collision_fixture
from jsonschema import Draft202012Validator
from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.collision_metrics import (
    analyze_collisions,
    classify_contact_pair,
    validate_collision_qualification,
)
from robotest_metrics.errors import MetricUnavailable
from robotest_scenarios.provenance import (
    contact_control_configuration,
    contact_control_configuration_sha256,
    file_sha256,
)


def _rebind_positive_control(positive: dict[str, Any], binding: dict[str, Any]) -> None:
    fixture_sha256 = canonical_sha256(positive['configuration']['fixture'])
    positive['configuration']['fixture_sha256'] = fixture_sha256
    positive['identity']['scenario_sha256'] = fixture_sha256
    binding['positive_control_scenario_sha256'] = fixture_sha256
    binding['positive_control_json_sha256'] = canonical_sha256(positive)


def _contact(counterpart: str, *, depth: float = 0.01, force: float = 2.0) -> dict[str, Any]:
    return {
        'collision1': 'robotest::base_link::base_collision',
        'collision2': counterpart,
        'maximum_penetration_depth_m': depth,
        'maximum_normal_force_n': force,
    }


def _support_contact() -> dict[str, Any]:
    return {
        'collision1': 'robotest::left_wheel_link::left_wheel_collision',
        'collision2': 'ground_plane::ground_link::ground_collision',
        'maximum_penetration_depth_m': 0.001,
        'maximum_normal_force_n': 1.0,
    }


def _message(stamp: int, sequence: int, *contacts: dict[str, Any]) -> dict[str, Any]:
    return {
        'collector_sequence': sequence,
        'contacts': [_support_contact(), *contacts],
        'delivery_clock_offset_ns': 0,
        'delivery_clock_stamp_ns': stamp,
        'frame_id': '',
        'stamp_ns': stamp,
    }


def test_coverage_requires_exact_complete_collision_union() -> None:
    manifest, positive, binding = collision_fixture()
    broken = copy.deepcopy(manifest)
    broken['covered_collisions'] = []
    with pytest.raises(MetricUnavailable, match='coverage'):
        validate_collision_qualification(broken, positive, binding)


def test_positive_control_is_hash_bound_and_must_pass() -> None:
    manifest, positive, binding = collision_fixture()
    assert validate_collision_qualification(manifest, positive, binding)['status'] == 'PASS'
    broken = copy.deepcopy(binding)
    broken['benchmark_provenance']['rendered_sdf_sha256'] = '9' * 64
    with pytest.raises(MetricUnavailable, match='hash mismatch'):
        validate_collision_qualification(manifest, positive, broken)
    failed = copy.deepcopy(positive)
    failed['status'] = 'FAIL'
    with pytest.raises(MetricUnavailable, match='not PASS'):
        validate_collision_qualification(manifest, failed, binding)


def test_positive_control_precontrol_callback_skew_is_diagnostic() -> None:
    manifest, positive, binding = collision_fixture()
    snapshots = positive['control']['contact']['snapshots']
    first_forward_sequence = positive['control']['command_trace'][0]['collector_sequence']
    precontrol_snapshot = snapshots[0]
    assert precontrol_snapshot['collector_sequence'] < first_forward_sequence
    precontrol_snapshot['delivery_clock_offset_ns'] = -278_000_000
    precontrol_snapshot['delivery_clock_stamp_ns'] = (
        precontrol_snapshot['sim_stamp_ns'] + precontrol_snapshot['delivery_clock_offset_ns']
    )
    positive['quality']['public_contact_snapshot_heartbeat']['future_delivery_count'] = 1
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    assert validate_collision_qualification(manifest, positive, binding)['status'] == 'PASS'


def test_contact_snapshot_boundary_accepts_caught_up_precontrol_skew() -> None:
    snapshots: list[dict[str, Any]] = []
    records: list[dict[str, Any]] = []
    snapshot_inputs = (
        (1, 2, 500_000_000, 778_000_000),
        (3, 4, 700_000_000, 800_000_000),
        (5, 6, 900_000_000, 1_000_000_000),
        (11, 12, 1_100_000_000, 1_000_000_000),
    )
    for summary_sequence, record_sequence, stamp, delivery_clock in snapshot_inputs:
        records.append(
            {
                'collector_sequence': record_sequence,
                'disposition': 'support_ground_excluded',
                'normalized_pair': [
                    'ground::plane::collision',
                    'robotest::wheel::collision',
                ],
                'sim_stamp_ns': stamp,
                'snapshot_sequence': summary_sequence,
            }
        )
        snapshots.append(
            {
                'classified_count': 0,
                'collector_sequence': summary_sequence,
                'counted_snapshot_records': [],
                'delivery_clock_offset_ns': delivery_clock - stamp,
                'delivery_clock_stamp_ns': delivery_clock,
                'exact_pair_count': 0,
                'sim_stamp_ns': stamp,
                'snapshot_record_count': 1,
            }
        )

    episodes, records_by_snapshot = collision_module._reconcile_contact_snapshots(
        snapshots,
        records,
        {},
        ('robotest::base::collision', 'wall::link::collision'),
        command_sequences=(10,),
        first_forward_sequence=10,
        first_forward_stamp_ns=1_000_000_000,
    )

    assert episodes == []
    assert sorted(records_by_snapshot) == [1, 3, 5, 11]


def test_positive_control_active_callback_skew_fails_closed() -> None:
    manifest, positive, binding = collision_fixture()
    snapshots = positive['control']['contact']['snapshots']
    first_forward_sequence = positive['control']['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot
        for snapshot in snapshots
        if snapshot['collector_sequence'] >= first_forward_sequence
        and snapshot['exact_pair_count'] == 0
    )
    active_snapshot['delivery_clock_offset_ns'] = 278_000_000
    active_snapshot['delivery_clock_stamp_ns'] = (
        active_snapshot['sim_stamp_ns'] + active_snapshot['delivery_clock_offset_ns']
    )
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='snapshot ordering/liveness'):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_active_snapshot_summary_cannot_cross_forward_records() -> None:
    manifest, positive, binding = collision_fixture()
    contact = positive['control']['contact']
    first_forward_sequence = positive['control']['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot for snapshot in contact['snapshots'] if snapshot['exact_pair_count'] == 1
    )
    original_sequence = active_snapshot['collector_sequence']
    forged_sequence = first_forward_sequence - 1
    active_snapshot['collector_sequence'] = forged_sequence
    active_snapshot['delivery_clock_offset_ns'] = 278_000_000
    active_snapshot['delivery_clock_stamp_ns'] = (
        active_snapshot['sim_stamp_ns'] + active_snapshot['delivery_clock_offset_ns']
    )
    for record in contact['snapshot_records']:
        if record['snapshot_sequence'] == original_sequence:
            record['snapshot_sequence'] = forged_sequence
    for record in active_snapshot['counted_snapshot_records']:
        record['snapshot_sequence'] = forged_sequence
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='sequence block is not contiguous'):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_atomic_snapshot_block_cannot_straddle_forward() -> None:
    manifest, positive, binding = collision_fixture()
    contact = positive['control']['contact']
    first_forward_sequence = positive['control']['command_trace'][0]['collector_sequence']
    active_snapshot = next(
        snapshot for snapshot in contact['snapshots'] if snapshot['exact_pair_count'] == 1
    )
    original_sequence = active_snapshot['collector_sequence']
    linked_records = [
        record
        for record in contact['snapshot_records']
        if record['snapshot_sequence'] == original_sequence
    ]
    old_to_new_record_sequence = {
        record['collector_sequence']: first_forward_sequence + index
        for index, record in enumerate(linked_records)
    }
    active_snapshot['collector_sequence'] = first_forward_sequence - 1
    for record in linked_records:
        record['snapshot_sequence'] = first_forward_sequence - 1
        record['collector_sequence'] = old_to_new_record_sequence[record['collector_sequence']]
    for record in active_snapshot['counted_snapshot_records']:
        record['snapshot_sequence'] = first_forward_sequence - 1
        record['record_sequence'] = old_to_new_record_sequence[record['record_sequence']]
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='block crosses the active-control boundary'):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_active_snapshot_block_cannot_move_wholly_before_forward() -> None:
    manifest, positive, binding = collision_fixture()
    contact = positive['control']['contact']
    precontrol_snapshot = contact['snapshots'][0]
    active_snapshot = next(
        snapshot for snapshot in contact['snapshots'] if snapshot['exact_pair_count'] == 1
    )
    for snapshot, summary_sequence, record_sequences in (
        (precontrol_snapshot, 1, (2,)),
        (active_snapshot, 4, (5, 6)),
    ):
        original_sequence = snapshot['collector_sequence']
        linked_records = [
            record
            for record in contact['snapshot_records']
            if record['snapshot_sequence'] == original_sequence
        ]
        old_to_new_record_sequence = {
            record['collector_sequence']: new_sequence
            for record, new_sequence in zip(linked_records, record_sequences, strict=True)
        }
        snapshot['collector_sequence'] = summary_sequence
        for record in linked_records:
            record['snapshot_sequence'] = summary_sequence
            record['collector_sequence'] = old_to_new_record_sequence[record['collector_sequence']]
        for record in snapshot['counted_snapshot_records']:
            record['snapshot_sequence'] = summary_sequence
            record['record_sequence'] = old_to_new_record_sequence[record['record_sequence']]
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='chronology crosses the active-control boundary'):
        validate_collision_qualification(manifest, positive, binding)


def _rehash_arm_acknowledgment(positive: dict[str, Any]) -> None:
    arm = positive['control']['arm']
    arm['acknowledgment_sha256'] = canonical_sha256(arm['acknowledgment'])


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (lambda positive: positive['control'].pop('arm'), 'arm proof'),
        (
            lambda positive: positive['control']['arm']['request'].__setitem__(
                'schema_version', True
            ),
            'must be an integer',
        ),
        (
            lambda positive: positive['control']['arm'].__setitem__('request_sha256', '0' * 64),
            'request hash mismatch',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment'].__setitem__(
                    'arm_observed_steady_ns', 3_999_999
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'steady-time ordering',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment'].__setitem__(
                    'armed_clock_sample_count', 10
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'fresh clock sample',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment']['command_delivery_probe'].__setitem__(
                    'matched_subscription_count', 1
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'match count',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment']['command_delivery_probe'].__setitem__(
                    'collector_progress_observed_steady_ns', 5_199_999
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'probe ordering',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment']['command_delivery_probe'].__setitem__(
                    'collector_progress_stamp_ns', 950_000_001
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'probe lag',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment']['command_delivery_probe'].__setitem__(
                    'probe_sim_stamp_ns', 900_000_001
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'fresh-clock bracket',
        ),
        (
            lambda positive: (
                positive['control']['arm']['acknowledgment']['command_delivery_probe'].__setitem__(
                    'collector_progress_sha256', '0' * 64
                ),
                _rehash_arm_acknowledgment(positive),
            ),
            'command-progress hash binding',
        ),
    ],
)
def test_positive_control_arm_handshake_fails_closed(
    mutation: Any,
    message: str,
) -> None:
    manifest, positive, binding = collision_fixture()
    mutation(positive)
    _rebind_positive_control(positive, binding)

    with pytest.raises(MetricUnavailable, match=message):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_graph_pins_jazzy_endpoint_gid_width() -> None:
    _manifest, positive, _binding = collision_fixture()
    snapshot = positive['quality']['contact_graph_topology']['first_snapshot']
    collision_module._validate_contact_graph_snapshot(snapshot, 'positive-control')
    invalid = copy.deepcopy(snapshot)
    invalid['public_snapshot_publishers'][0]['endpoint_gid'] = 'ab' * 24
    with pytest.raises(MetricUnavailable, match='owner/type/QoS changed'):
        collision_module._validate_contact_graph_snapshot(invalid, 'positive-control')


@pytest.mark.parametrize(
    'mutation',
    [
        lambda positive: positive['control']['command_trace'][0].pop('phase'),
        lambda positive: positive['control']['contact'].__setitem__('first_qualifying_contact', {}),
        lambda positive: positive['control'].__setitem__('command_trace', []),
        lambda positive: positive['control']['contact'].__setitem__('snapshot_records', []),
        lambda positive: positive['control']['contact'].__setitem__(
            'first_qualifying_contact', None
        ),
        lambda positive: positive['control']['contact'].__setitem__(
            'exact_pair_snapshot_record_count', 0
        ),
        lambda positive: positive['control'].__setitem__(
            'command_trace',
            [
                command
                for command in positive['control']['command_trace']
                if command['phase'] != 'HOLD'
            ],
        ),
        lambda positive: positive['configuration'].__setitem__('control_configuration', {}),
        lambda positive: positive['configuration']['source_binding'].__setitem__('files', []),
        lambda positive: positive['configuration'].__setitem__('fixture', {}),
        lambda positive: positive['control']['setup'].__setitem__('observed_wall', {}),
        lambda positive: positive['cleanup'].__setitem__('proof', {}),
        lambda positive: positive['control'].__setitem__('metrics_owned', {}),
        lambda positive: positive['quality'].__setitem__('forbidden_nodes', ['planner_server']),
    ],
)
def test_positive_control_schema_rejects_incomplete_pass_evidence(mutation: Any) -> None:
    _, positive, _ = collision_fixture()
    schema_path = (
        Path(__file__).resolve().parents[2]
        / 'robotest_scenarios/schema/contact-control-result.schema.json'
    )
    schema = json.loads(schema_path.read_text(encoding='utf-8'))
    validator = Draft202012Validator(schema)
    assert not list(validator.iter_errors(positive))

    mutation(positive)

    assert list(validator.iter_errors(positive))


@pytest.mark.parametrize(
    ('section', 'field'),
    [
        ('positive_control_external_quality', 'owned_process_group_shutdown'),
        ('positive_control_external_quality', 'collector_reconciled'),
        ('positive_control_external_quality', 'checksum_verified'),
    ],
)
def test_positive_control_external_gates_fail_closed(section: str, field: str) -> None:
    manifest, positive, binding = collision_fixture()
    binding[section][field] = False
    with pytest.raises(MetricUnavailable, match='external gate'):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_requires_frozen_ground_truth_start_proof() -> None:
    manifest, positive, binding = collision_fixture()
    positive['control']['observed_robot_start']['x'] = 0.02
    binding['positive_control_json_sha256'] = canonical_sha256(positive)
    with pytest.raises(MetricUnavailable, match='start proof'):
        validate_collision_qualification(manifest, positive, binding)


@pytest.mark.parametrize(
    ('path', 'value'),
    [
        (('control', 'observed_robot_start', 'alignment_error_ns'), -1),
        (('control', 'observed_robot_start', 'position_error_m'), -0.01),
        (('quality', 'buffers', 'ground_truth', 'ingress_count'), 2),
        (('quality', 'buffers', 'ground_truth', 'first_overflow_sequence'), 1),
    ],
)
def test_positive_control_start_and_buffer_proofs_fail_closed(
    path: tuple[str, ...],
    value: Any,
) -> None:
    manifest, positive, binding = collision_fixture()
    current = positive
    for component in path[:-1]:
        current = current[component]
    current[path[-1]] = value
    binding['positive_control_json_sha256'] = canonical_sha256(positive)
    with pytest.raises(MetricUnavailable, match=r'start proof|buffer ground_truth'):
        validate_collision_qualification(manifest, positive, binding)


@pytest.mark.parametrize(
    ('path', 'value', 'message'),
    [
        (('control', 'criteria', 'release_source_spanned'), False, 'criteria'),
        (('control', 'setup', 'observed_wall'), None, 'setup'),
        (('quality', 'source_streams_live'), False, 'source_streams_live'),
        (
            ('quality', 'public_contact_snapshot_heartbeat', 'latest_stamp_ns'),
            3_000_000_000,
            'heartbeat',
        ),
    ],
)
def test_positive_control_source_liveness_proofs_fail_closed(
    path: tuple[str, ...],
    value: Any,
    message: str,
) -> None:
    manifest, positive, binding = collision_fixture()
    current = positive
    for component in path[:-1]:
        current = current[component]
    current[path[-1]] = value
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match=message):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_coverage_provenance_is_exact() -> None:
    manifest, positive, binding = collision_fixture()
    positive['configuration']['coverage_manifest_provenance']['bridge_sha256'] = '9' * 64
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='coverage provenance mismatch'):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_actor_state_buffer_allows_genuine_deduplication() -> None:
    manifest, positive, binding = collision_fixture()
    actor_state = positive['quality']['buffers']['actor_state']
    actor_state.update(
        {
            'accepted_count': 1_100,
            'capacity': 1_024,
            'ingress_count': 1_100,
            'retained_count': 1,
        }
    )
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    assert validate_collision_qualification(manifest, positive, binding)['status'] == 'PASS'


def test_positive_control_keeps_driver_and_rendered_configuration_hashes_distinct() -> None:
    manifest, positive, binding = collision_fixture()
    configuration = positive['configuration']
    assert configuration['control_configuration'] == contact_control_configuration()
    assert configuration['control_configuration_sha256'] == contact_control_configuration_sha256()
    assert configuration['control_configuration_sha256'] != manifest['contact_configuration_sha256']
    assert validate_collision_qualification(manifest, positive, binding)['status'] == 'PASS'

    swapped = copy.deepcopy(positive)
    swapped_binding = copy.deepcopy(binding)
    swapped['configuration']['control_configuration_sha256'] = manifest[
        'contact_configuration_sha256'
    ]
    _rebind_positive_control(swapped, swapped_binding)
    with pytest.raises(MetricUnavailable, match='driver control configuration'):
        validate_collision_qualification(manifest, swapped, swapped_binding)

    equalized_binding = copy.deepcopy(binding)
    for name in ('benchmark_provenance', 'positive_control_provenance'):
        equalized_binding[name]['contact_configuration_sha256'] = configuration[
            'control_configuration_sha256'
        ]
    with pytest.raises(MetricUnavailable, match='manifest provenance mismatch'):
        validate_collision_qualification(manifest, positive, equalized_binding)


def test_positive_control_default_wall_asset_ignores_evidence_path_redirection(
    tmp_path: Path,
) -> None:
    manifest, positive, binding = collision_fixture()
    fake_manifest = tmp_path / 'config/collision-coverage.yaml'
    fake_manifest.parent.mkdir(parents=True)
    fake_manifest.write_text('attacker-controlled: true\n', encoding='utf-8')
    fake_asset = tmp_path / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
    fake_asset.parent.mkdir(parents=True)
    fake_asset.write_text('<sdf version="1.10"><model name="forged"/></sdf>\n', encoding='utf-8')
    fake_asset_sha256 = file_sha256(fake_asset)
    positive['configuration']['coverage_manifest_path'] = str(fake_manifest)
    positive['configuration']['wall_asset_sha256'] = fake_asset_sha256
    positive['configuration']['fixture']['entity']['asset_sha256'] = fake_asset_sha256
    _rebind_positive_control(positive, binding)

    with pytest.raises(MetricUnavailable, match='wall asset hash'):
        validate_collision_qualification(manifest, positive, binding)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda positive, _manifest: positive['configuration'].__setitem__(
                'control_configuration', {}
            ),
            'driver control configuration',
        ),
        (
            lambda positive, _manifest: positive['configuration'].__setitem__(
                'source_binding',
                {
                    'aggregate_sha256': canonical_sha256([{'name': 'fake.py', 'sha256': 'a' * 64}]),
                    'files': [{'name': 'fake.py', 'sha256': 'a' * 64}],
                },
            ),
            'source binding',
        ),
        (
            lambda positive, _manifest: positive['configuration']['fixture']['entity'].__setitem__(
                'name', 'forged_wall'
            ),
            'fixture binding',
        ),
        (
            lambda positive, _manifest: (
                positive['configuration'].__setitem__('wall_asset_sha256', 'f' * 64),
                positive['configuration']['fixture']['entity'].__setitem__(
                    'asset_sha256', 'f' * 64
                ),
            ),
            'wall asset hash',
        ),
        (
            lambda positive, _manifest: positive['control']['setup']['observed_wall'].update(
                {'position_error_m': 0.1, 'x': 0.8}
            ),
            'setup proof',
        ),
        (
            lambda positive, _manifest: positive['control']['setup']['observed_wall'].__setitem__(
                'stamp_ns', 1_100_000_000
            ),
            'timeline',
        ),
        (
            lambda positive, _manifest: positive['control']['observed_robot_start'].__setitem__(
                'alignment_error_ns', 0
            ),
            'setup/control sequence',
        ),
        (
            lambda positive, _manifest: positive['cleanup'].__setitem__('proof', {}),
            'cleanup proof',
        ),
        (
            lambda positive, _manifest: positive['cleanup']['proof'].__setitem__(
                'request_stamp_ns', 3_600_000_000
            ),
            'cleanup proof',
        ),
        (
            lambda positive, _manifest: positive['control'].__setitem__('metrics_owned', {}),
            'metrics ownership',
        ),
        (
            lambda positive, _manifest: positive['quality'].__setitem__(
                'forbidden_nodes', ['planner_server']
            ),
            'forbidden navigation nodes',
        ),
        (
            lambda positive, _manifest: positive['control']['command_trace'][1].__setitem__(
                'sim_stamp_ns',
                positive['control']['command_trace'][1]['sim_stamp_ns'] + 1,
            ),
            'command anchors',
        ),
        (
            lambda positive, _manifest: (
                positive['control']['contact']['first_qualifying_contact'].__setitem__(
                    'collector_sequence', 6
                ),
                next(
                    record
                    for record in positive['control']['contact']['snapshot_records']
                    if record['disposition'] == 'counted'
                ).__setitem__('collector_sequence', 6),
                positive['control']['command_trace'][1].__setitem__('collector_sequence', 7),
            ),
            'command trace is not ordered',
        ),
    ],
)
def test_positive_control_rejects_rebound_producer_contract_tampering(
    mutation: Any,
    message: str,
) -> None:
    manifest, positive, binding = collision_fixture()
    mutation(positive, manifest)
    setup_start = positive['control']['setup']['observed_robot_start']
    control_start = positive['control']['observed_robot_start']
    if setup_start != control_start:
        positive['control']['setup']['observed_robot_start'] = copy.deepcopy(control_start)
    _rebind_positive_control(positive, binding)

    with pytest.raises(MetricUnavailable, match=message):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_rejects_rebound_command_trace_without_hold() -> None:
    manifest, positive, binding = collision_fixture()
    commands = positive['control']['command_trace']
    positive['control']['command_trace'] = [
        command for command in commands if command['phase'] != 'HOLD'
    ]
    command_buffer = positive['quality']['buffers']['command']
    for field in ('accepted_count', 'ingress_count', 'retained_count'):
        command_buffer[field] = len(positive['control']['command_trace'])
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='command anchors'):
        validate_collision_qualification(manifest, positive, binding)


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda positive: positive['control']['contact'].__setitem__('snapshot_records', []),
            'retained contact records',
        ),
        (
            lambda positive: positive['quality']['buffers']['command'].update(
                {'accepted_count': 3, 'ingress_count': 3, 'retained_count': 3}
            ),
            'retained buffers',
        ),
        (
            lambda positive: positive['control']['timeline'].__setitem__(
                'hold_complete_stamp_ns', 2_000_000_000
            ),
            'timeline',
        ),
        (
            lambda positive: positive['control']['timeline'].__setitem__(
                'release_required_through_stamp_ns', 2_250_000_000
            ),
            'timeline',
        ),
        (
            lambda positive: positive['control']['timeline'].__setitem__(
                'release_contact_snapshot_start_count', 0
            ),
            'timeline',
        ),
        (
            lambda positive: positive['quality']['public_contact_snapshot_heartbeat'].__setitem__(
                'first_stamp_ns', 2_500_000_000
            ),
            'heartbeat',
        ),
        (
            lambda positive: positive['quality']['public_contact_snapshot_heartbeat'].__setitem__(
                'pre_clock_discard_count', -1
            ),
            'heartbeat',
        ),
        (
            lambda positive: positive['quality'].__setitem__(
                'source_publisher_missing_observation_count', -1
            ),
            'source publisher miss count',
        ),
    ],
)
def test_positive_control_derived_evidence_relations_fail_closed(
    mutation: Any,
    message: str,
) -> None:
    manifest, positive, binding = collision_fixture()
    mutation(positive)
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match=message):
        validate_collision_qualification(manifest, positive, binding)


def test_positive_control_contact_records_are_reclassified_from_retained_bytes() -> None:
    manifest, positive, binding = collision_fixture()
    record = next(
        record
        for record in positive['control']['contact']['snapshot_records']
        if record['disposition'] == 'counted'
    )
    record.update(
        {
            'counterpart_collision': None,
            'counterpart_model': None,
            'disposition': 'non_robot_pair_ignored',
            'normalized_pair': ['box::link::collision', 'wall::link::collision'],
            'robot_collision': None,
        }
    )
    binding['positive_control_json_sha256'] = canonical_sha256(positive)

    with pytest.raises(MetricUnavailable, match='non-robot pair'):
        validate_collision_qualification(manifest, positive, binding)


def test_pair_classification_has_only_exact_frozen_exclusions() -> None:
    manifest, _, _ = collision_fixture()
    wheel = 'robotest::left_wheel::wheel_collision'
    manifest['robot_collisions'].append(
        {'name': wheel, 'role': 'left_wheel', 'source': 'all_robot_contacts'}
    )
    manifest['covered_collisions'].append(wheel)
    manifest['rendered_robot_collisions'].append(wheel)
    ground = 'ground_plane::ground_link::ground_collision'
    manifest['support_pairs'] = [{'environment_collision': ground, 'robot_collision': wheel}]
    manifest_body = dict(manifest)
    manifest_body.pop('manifest_sha256')
    manifest['manifest_sha256'] = canonical_sha256(manifest_body)
    assert classify_contact_pair(wheel, ground, manifest)['reason'] == 'allowlisted_support_contact'
    assert (
        classify_contact_pair(wheel, manifest['robot_collisions'][0]['name'], manifest)['reason']
        == 'robot_internal'
    )
    with pytest.raises(MetricUnavailable, match='non-robot pair'):
        classify_contact_pair('box::a::collision', 'wall::b::collision', manifest)
    counted = classify_contact_pair(wheel, 'wall::link::collision', manifest)
    assert counted['counted'] is True
    assert counted['counterpart_model'] == 'wall'


@pytest.mark.parametrize(
    'invalid_name',
    ['model::collision', 'a::::b', '::a::b', 'a::b::'],
)
def test_pair_classification_rejects_impossible_scoped_names(invalid_name: str) -> None:
    manifest, _, _ = collision_fixture()
    with pytest.raises(MetricUnavailable, match='model::link::collision scoped'):
        classify_contact_pair(manifest['robot_collisions'][0]['name'], invalid_name, manifest)


def test_support_allowlist_rejects_chassis_or_non_ground_exclusions() -> None:
    manifest, _, _ = collision_fixture()
    chassis = manifest['robot_collisions'][0]['name']
    for counterpart in (
        'ground_plane::ground_link::ground_collision',
        'wall::link::collision',
    ):
        broken = copy.deepcopy(manifest)
        broken['support_pairs'] = [
            {'environment_collision': counterpart, 'robot_collision': chassis}
        ]
        body = dict(broken)
        body.pop('manifest_sha256')
        broken['manifest_sha256'] = canonical_sha256(body)
        with pytest.raises(MetricUnavailable, match='wheel/caster'):
            classify_contact_pair(chassis, counterpart, broken)


def test_contact_episode_dedup_release_and_pre_action_diagnostics() -> None:
    manifest, positive, binding = collision_fixture()
    wall = 'wall::link::collision'
    messages = [
        _message(50, 1, _contact('prewall::link::collision')),
        _message(100, 2, _contact(wall, depth=0.01, force=2.0)),
        _message(200, 3, _contact(wall, depth=0.02, force=3.0)),
        _message(450, 4, _contact(wall, depth=0.03, force=4.0)),
        _message(500, 5),
        _message(800, 6),
    ]
    result = analyze_collisions(
        messages, 100, 500, 800, manifest, positive, binding, release_gap_ns=250
    )
    assert result['collision_count'] == 1
    assert result['pre_action_snapshot_record_count'] == 1
    assert result['events'][0]['sampled_snapshot_record_count'] == 3
    assert result['events'][0]['end_stamp_ns'] == 500
    assert result['events'][0]['maximum_delivered_snapshot_penetration_depth_m'] == pytest.approx(
        0.03
    )


def test_contact_episode_that_began_before_t0_remains_diagnostic() -> None:
    manifest, positive, binding = collision_fixture()
    wall = 'wall::link::collision'
    result = analyze_collisions(
        [
            _message(50, 1, _contact(wall)),
            _message(100, 2, _contact(wall)),
            _message(200, 3, _contact(wall)),
            _message(451, 4),
        ],
        100,
        200,
        451,
        manifest,
        positive,
        binding,
        release_gap_ns=250,
    )
    assert result['pre_action_snapshot_record_count'] == 1
    assert result['collision_count'] == 0


def test_simultaneous_counterpart_models_are_distinct_events() -> None:
    manifest, positive, binding = collision_fixture()
    messages = [
        _message(
            100,
            1,
            _contact('wall::link::collision'),
            _contact('box::link::collision'),
        ),
        _message(200, 2),
        _message(451, 3),
    ]
    result = analyze_collisions(
        messages, 100, 200, 451, manifest, positive, binding, release_gap_ns=250
    )
    assert result['collision_count'] == 2
    assert {event['counterpart_model'] for event in result['events']} == {'box', 'wall'}


def test_post_terminal_contacts_do_not_start_counted_events() -> None:
    manifest, positive, binding = collision_fixture()
    messages = [
        _message(100, 1),
        _message(201, 2, _contact('wall::link::collision')),
        _message(451, 3),
    ]
    result = analyze_collisions(
        messages, 100, 200, 451, manifest, positive, binding, release_gap_ns=250
    )
    assert result['collision_count'] == 0
    assert result['post_terminal_snapshot_record_count'] == 1


def test_post_terminal_continuation_without_full_release_is_censored_at_drain() -> None:
    manifest, positive, binding = collision_fixture()
    wall = 'wall::link::collision'
    result = analyze_collisions(
        [
            _message(100, 1),
            _message(300, 2, _contact(wall)),
            _message(551, 3, _contact(wall)),
        ],
        100,
        300,
        551,
        manifest,
        positive,
        binding,
        release_gap_ns=250,
    )
    assert result['collision_count'] == 1
    assert result['events'][0]['censored_at_drain'] is True
    assert result['events'][0]['end_stamp_ns'] == 551


def test_collision_requires_full_drain_and_non_silent_mission_topic() -> None:
    manifest, positive, binding = collision_fixture()
    with pytest.raises(MetricUnavailable, match='drain'):
        analyze_collisions(
            [_message(100, 1), _message(450, 2)],
            100,
            200,
            450,
            manifest,
            positive,
            binding,
            release_gap_ns=250,
        )
    with pytest.raises(MetricUnavailable, match='silent'):
        analyze_collisions(
            [_message(99, 1), _message(451, 2)],
            100,
            200,
            451,
            manifest,
            positive,
            binding,
            release_gap_ns=250,
        )


def test_collision_rejects_negative_depth_and_record_overflow(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest, positive, binding = collision_fixture()
    with pytest.raises(MetricUnavailable, match='negative'):
        analyze_collisions(
            [
                _message(100, 1, _contact('wall::link::collision', depth=-0.1)),
                _message(451, 2),
            ],
            100,
            200,
            451,
            manifest,
            positive,
            binding,
            release_gap_ns=250,
        )
    monkeypatch.setattr(collision_module, 'CONTACT_RECORD_CAPACITY', 1)
    with pytest.raises(MetricUnavailable, match='capacity'):
        analyze_collisions(
            [
                _message(
                    100,
                    1,
                    _contact('wall::link::collision'),
                    _contact('box::link::collision'),
                ),
                _message(451, 2),
            ],
            100,
            200,
            451,
            manifest,
            positive,
            binding,
            release_gap_ns=250,
        )


def test_snapshot_presence_equality_pair_migration_and_absence_recontact() -> None:
    manifest, positive, binding = collision_fixture()
    wall = 'wall::link::collision'
    chassis = 'robotest::base_link::base_collision'
    wheel = 'robotest::left_wheel_link::left_wheel_collision'
    migrated = _contact(wall)
    migrated['collision1'] = wheel
    result = analyze_collisions(
        [
            _message(100, 1, _contact(wall)),
            _message(350, 2, migrated),
            _message(351, 3),
            _message(600, 4, _contact(wall)),
            _message(601, 5),
            _message(951, 6),
        ],
        100,
        700,
        951,
        manifest,
        positive,
        binding,
        release_gap_ns=250,
    )
    assert result['collision_count'] == 2
    assert result['events'][0]['start_stamp_ns'] == 100
    assert result['events'][0]['end_stamp_ns'] == 351
    assert result['events'][0]['sampled_snapshot_record_count'] == 2
    assert result['events'][0]['pairs'] == [
        {'counterpart_collision': wall, 'robot_collision': chassis},
        {'counterpart_collision': wall, 'robot_collision': wheel},
    ]
    assert result['events'][1]['start_stamp_ns'] == 600
    assert result['events'][1]['end_stamp_ns'] == 601


def test_public_snapshot_shape_ordering_and_delivery_offset_identity_fail_closed() -> None:
    manifest, positive, binding = collision_fixture()
    base = [_message(100, 1), _message(451, 2)]
    duplicate = copy.deepcopy(base)
    duplicate[1]['stamp_ns'] = 100
    duplicate[1]['delivery_clock_stamp_ns'] = 100
    with pytest.raises(MetricUnavailable, match='strictly increasing'):
        analyze_collisions(
            duplicate, 100, 200, 451, manifest, positive, binding, release_gap_ns=250
        )

    empty = copy.deepcopy(base)
    empty[0]['contacts'] = []
    with pytest.raises(MetricUnavailable, match='between one and 16'):
        analyze_collisions(empty, 100, 200, 451, manifest, positive, binding, release_gap_ns=250)

    oversized = copy.deepcopy(base)
    oversized[0]['contacts'] = [_support_contact() for _ in range(17)]
    with pytest.raises(MetricUnavailable, match='between one and 16'):
        analyze_collisions(
            oversized, 100, 200, 451, manifest, positive, binding, release_gap_ns=250
        )

    diagnostic_offset = copy.deepcopy(base)
    diagnostic_offset[0]['delivery_clock_stamp_ns'] = 278_000_100
    diagnostic_offset[0]['delivery_clock_offset_ns'] = 278_000_000
    analyze_collisions(
        diagnostic_offset,
        100,
        200,
        451,
        manifest,
        positive,
        binding,
        release_gap_ns=250,
    )

    inconsistent_offset = copy.deepcopy(diagnostic_offset)
    inconsistent_offset[0]['delivery_clock_offset_ns'] = 278_000_001
    with pytest.raises(MetricUnavailable, match='delivery clock offset is inconsistent'):
        analyze_collisions(
            inconsistent_offset,
            100,
            200,
            451,
            manifest,
            positive,
            binding,
            release_gap_ns=250,
        )
