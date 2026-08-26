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

from pathlib import Path

import pytest
import yaml
from robotest_scenarios.constants import CONTROL_WALL_COLLISION
from robotest_scenarios.contact_evidence import ContactEpisodeTracker, load_coverage_manifest
from robotest_scenarios.errors import ProtocolError, ValidationError

REPOSITORY = Path(__file__).resolve().parents[3]
MANIFEST = REPOSITORY / 'config' / 'collision-coverage.yaml'
GROUND = 'ground_plane::ground_link::ground_collision'


def test_manifest_freezes_exact_expected_pair_and_exclusions() -> None:
    manifest = load_coverage_manifest(str(MANIFEST))
    assert manifest.raw_contact_topic == '/robotest/validation/contacts'
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


def test_tracker_closes_one_episode_only_after_full_release_gap() -> None:
    pair = ('phase3_contact_control_wall::link::collision', 'robotest::link::collision')
    tracker = ContactEpisodeTracker('phase3_contact_control_wall')
    tracker.observe(1_000_000_000, pair)
    tracker.observe(1_100_000_000, pair)
    tracker.advance(1_349_999_999)
    assert not tracker.episodes
    tracker.advance(1_350_000_000)
    assert tracker.active_start_ns is None
    assert tracker.episodes == [
        {
            'counterpart_model': 'phase3_contact_control_wall',
            'end_stamp_ns': 1_350_000_000,
            'normalized_pairs': [list(pair)],
            'sample_count': 2,
            'start_stamp_ns': 1_000_000_000,
        }
    ]


def test_tracker_starts_a_second_episode_after_release() -> None:
    pair = ('phase3_contact_control_wall::link::collision', 'robotest::link::collision')
    tracker = ContactEpisodeTracker('phase3_contact_control_wall')
    tracker.observe(1_000_000_000, pair)
    tracker.advance(1_250_000_000)
    tracker.observe(1_500_000_000, pair)
    tracker.advance(1_750_000_000)
    assert len(tracker.episodes) == 2


def test_manifest_rejects_unknown_rendered_robot_collision() -> None:
    manifest = load_coverage_manifest(str(MANIFEST))
    with pytest.raises(ProtocolError, match='unknown rendered robot collision'):
        manifest.classify('robotest::unknown_link::unknown_collision', CONTROL_WALL_COLLISION)


def test_manifest_self_hash_rejects_tampering(tmp_path: Path) -> None:
    value = yaml.safe_load(MANIFEST.read_text(encoding='utf-8'))
    value['robot_collisions'][0]['contact_sensor'] = 'tampered_sensor'
    path = tmp_path / 'tampered.yaml'
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValidationError, match='self-hash'):
        load_coverage_manifest(str(path))
