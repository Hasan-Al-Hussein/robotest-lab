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

"""Strict mission schema tests."""

from pathlib import Path

import pytest
import yaml

from robotest_missions.schema import MissionValidationError, load_mission

REPOSITORY = Path(__file__).resolve().parents[3]
BASELINE = REPOSITORY / 'scenarios' / 'phase2_baseline.yaml'


def baseline_payload() -> dict:
    return yaml.safe_load(BASELINE.read_text(encoding='utf-8'))


def write_mission(tmp_path: Path, payload: object) -> Path:
    path = tmp_path / 'mission.yaml'
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    return path


def test_baseline_is_exact_and_strictly_valid() -> None:
    document = load_mission(BASELINE)
    config = document.config
    assert config.frame_id == 'map'
    assert [(pose.x, pose.y) for pose in config.waypoints] == [
        (-2.0, -3.5),
        (0.0, 0.0),
        (0.0, 3.5),
    ]
    assert config.fault_schedule is None
    assert config.fault_seed is None
    assert config.simulator_seed == 42


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (lambda data: data.update({'unknown': 1}), 'Additional properties'),
        (lambda data: data.update({'frame_id': '/map'}), "'map' was expected"),
        (lambda data: data.update({'fault_schedule': 'faults.yaml'}), "type 'null'"),
        (lambda data: data.update({'fault_seed': 7}), "type 'null'"),
        (lambda data: data.update({'retries': 4}), 'maximum of 3'),
        (lambda data: data.update({'mission_seed': True}), 'integer'),
    ],
)
def test_invalid_contract_values_are_rejected(tmp_path, mutation, message) -> None:
    payload = baseline_payload()
    mutation(payload)
    with pytest.raises(MissionValidationError, match=message):
        load_mission(write_mission(tmp_path, payload))


def test_duplicate_yaml_keys_are_rejected(tmp_path: Path) -> None:
    raw = BASELINE.read_text(encoding='utf-8') + '\nmission_seed: 99\n'
    path = tmp_path / 'duplicate.yaml'
    path.write_text(raw, encoding='utf-8')
    with pytest.raises(MissionValidationError, match='duplicate key'):
        load_mission(path)


def test_nonfinite_yaml_number_is_rejected(tmp_path: Path) -> None:
    payload = baseline_payload()
    payload['waypoints'][0]['x'] = float('nan')
    with pytest.raises(MissionValidationError, match='non-finite'):
        load_mission(write_mission(tmp_path, payload))


def test_missing_mission_is_a_bounded_input_error(tmp_path: Path) -> None:
    with pytest.raises((MissionValidationError, FileNotFoundError)):
        load_mission(tmp_path / 'missing.yaml')
