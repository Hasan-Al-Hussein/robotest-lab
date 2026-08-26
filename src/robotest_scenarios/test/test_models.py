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

"""Strict scenario-input contract tests."""

from pathlib import Path

import pytest
import yaml
from robotest_scenarios.errors import ValidationError
from robotest_scenarios.models import EXPECTED_NAMES, load_scenario

REPOSITORY = Path(__file__).resolve().parents[3]
SCENARIOS = tuple(sorted((REPOSITORY / 'scenarios').glob('phase3_s*.yaml')))


@pytest.mark.parametrize('path', SCENARIOS, ids=lambda path: path.stem)
def test_all_frozen_phase3_documents_load(path: Path) -> None:
    document = load_scenario(str(path))
    assert document.scenario_name == EXPECTED_NAMES[document.scenario_id]
    assert document.controller_seed == 42
    assert document.sha256
    assert document.has_actor is (document.scenario_id in {2, 3})


def test_unknown_root_field_is_rejected(tmp_path: Path) -> None:
    payload = yaml.safe_load(SCENARIOS[0].read_text(encoding='utf-8'))
    payload['unfrozen_extension'] = True
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValidationError, match='schema violation'):
        load_scenario(str(path))


def test_actor_pose_mutation_is_rejected(tmp_path: Path) -> None:
    source = REPOSITORY / 'scenarios' / 'phase3_s2_static_obstacle.yaml'
    payload = yaml.safe_load(source.read_text(encoding='utf-8'))
    payload['entity']['world_pose']['x_m'] = -0.9
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValidationError, match='schema violation'):
        load_scenario(str(path))


def test_scenario3_only_fields_are_rejected_from_baseline(tmp_path: Path) -> None:
    payload = yaml.safe_load(SCENARIOS[0].read_text(encoding='utf-8'))
    payload.update(
        {
            'control_rate_hz': 10.0,
            'dwell_duration_sim_s': 4.0,
            'move_1_duration_sim_s': 4.0,
            'move_2_duration_sim_s': 4.0,
        }
    )
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    with pytest.raises(ValidationError, match='schema violation'):
        load_scenario(str(path))
