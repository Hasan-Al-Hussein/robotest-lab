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

"""Coverage classification and episode de-duplication tests."""

import copy
import importlib.util
import sys
from pathlib import Path

import pytest
import yaml
from robotest_scenarios.constants import CONTROL_WALL_COLLISION
from robotest_scenarios.contact_evidence import (
    EXPECTED_CONTACT_GATE_SOURCE_PATHS,
    EXPECTED_CONTACT_STREAM_POLICY,
    ContactEpisodeTracker,
    load_coverage_manifest,
    reconcile_contact_snapshot_pair_traces,
)
from robotest_scenarios.errors import ProtocolError, ValidationError

REPOSITORY = Path(__file__).resolve().parents[3]
MANIFEST = REPOSITORY / 'config' / 'collision-coverage.yaml'
GROUND = 'ground_plane::ground_link::ground_collision'


def _load_generator_module():
    path = REPOSITORY / 'src' / 'robotest_description' / 'tools' / 'generate_collision_coverage.py'
    spec = importlib.util.spec_from_file_location('contact_coverage_generator_contract', path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_manifest_freezes_exact_expected_pair_and_exclusions() -> None:
    manifest = load_coverage_manifest(str(MANIFEST))
    assert manifest.public_contact_snapshot_topic == '/robotest/validation/contacts'
    assert manifest.private_raw_contact_topic == '/robotest/internal/raw_contacts'
    assert manifest.authoritative_contact_snapshots is True
    assert manifest.contact_snapshot_max_gap_ns == 220_000_000
    value = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    assert value['contact_stream']['policy'] == EXPECTED_CONTACT_STREAM_POLICY
    assert [
        source['path'] for source in value['contact_stream']['gate']['source_inventory']['sources']
    ] == list(EXPECTED_CONTACT_GATE_SOURCE_PATHS)
    assert manifest.expected_control_pair == tuple(
        sorted((manifest.chassis_collision, CONTROL_WALL_COLLISION))
    )
    chassis_wall = manifest.classify(manifest.chassis_collision, CONTROL_WALL_COLLISION)
    assert chassis_wall is not None
    assert tuple(chassis_wall['normalized_pair']) == manifest.expected_control_pair
    left_wheel = next(
        collision for collision in manifest.robot_collisions if 'left_wheel_link' in collision
    )
    assert manifest.classify(left_wheel, GROUND) is None
    assert manifest.classify(manifest.chassis_collision, GROUND) is not None


def test_generator_policy_roundtrips_through_the_runtime_manifest_loader() -> None:
    generator = _load_generator_module()
    assert generator.CONTACT_STREAM_POLICY == EXPECTED_CONTACT_STREAM_POLICY
    generated = generator.build_manifest(REPOSITORY)
    committed = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    assert generated == committed
    manifest = load_coverage_manifest(str(MANIFEST))
    assert manifest.contact_stream_policy_sha256 == generator.canonical_sha256(
        EXPECTED_CONTACT_STREAM_POLICY
    )


def test_tracker_closes_one_episode_on_authoritative_absence_snapshot() -> None:
    pair = ('phase3_contact_control_wall::link::collision', 'robotest::link::collision')
    tracker = ContactEpisodeTracker('phase3_contact_control_wall')
    tracker.observe_snapshot(1_000_000_000, [pair])
    tracker.observe_snapshot(1_100_000_000, [pair])
    tracker.observe_snapshot(1_350_000_001, [])
    assert tracker.active_start_ns is None
    assert tracker.episodes == [
        {
            'counterpart_model': 'phase3_contact_control_wall',
            'end_stamp_ns': 1_350_000_001,
            'normalized_pairs': [list(pair)],
            'snapshot_record_count': 2,
            'start_stamp_ns': 1_000_000_000,
        }
    ]


def test_tracker_starts_a_second_episode_after_release() -> None:
    pair = ('phase3_contact_control_wall::link::collision', 'robotest::link::collision')
    tracker = ContactEpisodeTracker('phase3_contact_control_wall')
    tracker.observe_snapshot(1_000_000_000, [pair])
    tracker.observe_snapshot(1_250_000_001, [])
    tracker.observe_snapshot(1_500_000_000, [pair])
    tracker.observe_snapshot(1_750_000_001, [])
    assert len(tracker.episodes) == 2


def test_tracker_rejects_legacy_record_evidence_and_snapshot_regression() -> None:
    pair = ('phase3_contact_control_wall::link::collision', 'robotest::link::collision')
    tracker = ContactEpisodeTracker('phase3_contact_control_wall')
    with pytest.raises(ProtocolError, match='legacy'):
        tracker.observe(1_000_000_000, pair)
    with pytest.raises(ProtocolError, match='legacy'):
        tracker.advance(1_000_000_000)
    tracker.observe_snapshot(1_000_000_000, [pair])
    with pytest.raises(ProtocolError, match='strictly advance'):
        tracker.observe_snapshot(1_000_000_000, [pair])


def _snapshot_trace_fixture() -> tuple[dict[str, object], list[dict[str, object]], int]:
    support = tuple(sorted(('robotest::wheel::collision', GROUND)))
    internal = tuple(sorted(('robotest::base::collision', 'robotest::sensor::collision')))
    wall = tuple(sorted(('robotest::base::collision', CONTROL_WALL_COLLISION)))
    component = {
        'snapshots': [
            {'collector_sequence': 10, 'sim_stamp_ns': 100, 'snapshot_record_count': 3},
            {'collector_sequence': 20, 'sim_stamp_ns': 200, 'snapshot_record_count': 1},
            {'collector_sequence': 30, 'sim_stamp_ns': 300, 'snapshot_record_count': 1},
        ],
        'snapshot_records': [
            {'snapshot_sequence': 10, 'sim_stamp_ns': 100, 'normalized_pair': list(support)},
            {'snapshot_sequence': 10, 'sim_stamp_ns': 100, 'normalized_pair': list(internal)},
            {'snapshot_sequence': 10, 'sim_stamp_ns': 100, 'normalized_pair': list(support)},
            {'snapshot_sequence': 20, 'sim_stamp_ns': 200, 'normalized_pair': list(wall)},
            {'snapshot_sequence': 30, 'sim_stamp_ns': 300, 'normalized_pair': list(support)},
        ],
    }
    capture = [
        {
            'stamp_ns': 50,
            'contacts': [{'collision1': support[0], 'collision2': support[1]}],
        },
        {
            'stamp_ns': 100,
            'contacts': [
                {'collision1': support[1], 'collision2': support[0]},
                {'collision1': support[0], 'collision2': support[1]},
                {'collision1': internal[1], 'collision2': internal[0]},
            ],
        },
        {
            'stamp_ns': 200,
            'contacts': [{'collision1': wall[1], 'collision2': wall[0]}],
        },
        {
            'stamp_ns': 300,
            'contacts': [{'collision1': support[0], 'collision2': support[1]}],
        },
        {
            'stamp_ns': 350,
            'contacts': [{'collision1': support[0], 'collision2': support[1]}],
        },
    ]
    return component, capture, 300


def test_component_and_collector_snapshot_pair_traces_are_bijective() -> None:
    component, capture, q = _snapshot_trace_fixture()
    trace = reconcile_contact_snapshot_pair_traces(component, capture, q)
    assert [stamp for stamp, _ in trace] == [100, 200, 300]
    assert len(trace[0][1]) == 3
    assert trace[0][1][0] == trace[0][1][1]


@pytest.mark.parametrize(
    'mutation',
    ('missing_stamp', 'extra_recontact', 'pair_replacement', 'duplicate_multiplicity'),
)
def test_snapshot_pair_trace_bijection_rejects_observer_disagreement(mutation: str) -> None:
    component, capture, q = _snapshot_trace_fixture()
    capture = copy.deepcopy(capture)
    if mutation == 'missing_stamp':
        del capture[2]
    elif mutation == 'extra_recontact':
        capture.insert(
            3,
            {
                'stamp_ns': 250,
                'contacts': [
                    {
                        'collision1': 'robotest::base::collision',
                        'collision2': CONTROL_WALL_COLLISION,
                    }
                ],
            },
        )
    elif mutation == 'pair_replacement':
        capture[2]['contacts'][0]['collision2'] = 'box::link::collision'
    else:
        del capture[1]['contacts'][1]
    with pytest.raises(ValidationError, match='traces differ'):
        reconcile_contact_snapshot_pair_traces(component, capture, q)


def test_component_snapshot_pair_trace_rejects_record_count_mismatch() -> None:
    component, capture, q = _snapshot_trace_fixture()
    component['snapshots'][0]['snapshot_record_count'] = 2
    with pytest.raises(ValidationError, match='records do not reconcile'):
        reconcile_contact_snapshot_pair_traces(component, capture, q)


@pytest.mark.parametrize('mutation', ('missing_q', 'duplicate_q'))
def test_snapshot_pair_trace_requires_exact_q_once(mutation: str) -> None:
    component, capture, q = _snapshot_trace_fixture()
    capture = copy.deepcopy(capture)
    if mutation == 'missing_q':
        del capture[3]
    else:
        capture.insert(4, copy.deepcopy(capture[3]))
    with pytest.raises(ValidationError, match=r'span|strictly ordered'):
        reconcile_contact_snapshot_pair_traces(component, capture, q)


def test_manifest_rejects_unknown_rendered_robot_collision() -> None:
    manifest = load_coverage_manifest(str(MANIFEST))
    with pytest.raises(ProtocolError, match='unknown rendered robot collision'):
        manifest.classify('robotest::unknown_link::unknown_collision', CONTROL_WALL_COLLISION)


@pytest.mark.parametrize(
    'invalid_name',
    ['model::collision', 'a::::b', '::a::b', 'a::b::'],
)
def test_manifest_rejects_impossible_scoped_collision_names(invalid_name: str) -> None:
    manifest = load_coverage_manifest(str(MANIFEST))
    with pytest.raises(ProtocolError, match='invalid scoped collision name'):
        manifest.classify(manifest.chassis_collision, invalid_name)


def test_manifest_self_hash_rejects_tampering(tmp_path: Path) -> None:
    value = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    value['robot_collisions'][0]['contact_sensor'] = 'tampered_sensor'
    path = tmp_path / 'tampered.yaml'
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValidationError, match=r'source inventory|self-hash'):
        load_coverage_manifest(str(path))
