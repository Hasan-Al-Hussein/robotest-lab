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

"""Strict discriminated validation for all five frozen Phase 3 scenarios."""

from pathlib import Path

import pytest
import yaml

from robotest_missions.schema import MissionValidationError, load_mission

REPOSITORY = Path(__file__).resolve().parents[3]
SCENARIOS = tuple(sorted((REPOSITORY / 'scenarios').glob('phase3_s*.yaml')))


@pytest.mark.parametrize('path', SCENARIOS, ids=lambda path: path.stem)
def test_frozen_phase3_scenario_loads_with_computed_schedule(path: Path) -> None:
    document = load_mission(path)
    assert document.config.schema_version == 1
    assert document.config.scenario_id in range(1, 6)
    assert document.config.scenario_controller_seed == 42
    assert document.config.fault_schedule is not None
    assert (
        document.config.fault_schedule.sha256
        == yaml.safe_load(path.read_text(encoding='utf-8'))['fault_schedule_sha256']
    )


def _write(tmp_path: Path, payload: dict) -> Path:
    path = tmp_path / 'scenario.yaml'
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding='utf-8')
    return path


@pytest.mark.parametrize(
    ('filename', 'mutate'),
    [
        ('phase3_s1_baseline.yaml', lambda p: p.update({'unknown': True})),
        (
            'phase3_s2_static_obstacle.yaml',
            lambda p: p['entity']['world_pose'].update({'x_m': -0.9}),
        ),
        ('phase3_s3_dynamic_obstacle.yaml', lambda p: p.update({'control_rate_hz': 9.0})),
        (
            'phase3_s4_lidar_dropout.yaml',
            lambda p: p['fault'].update({'duration_ns': 2_000_000_001}),
        ),
        (
            'phase3_s5_odom_drift.yaml',
            lambda p: p['fault']['parameters'].update({'x_rate_nm_per_s': 9_000_000}),
        ),
    ],
)
def test_frozen_scenario_mutation_is_rejected(tmp_path: Path, filename: str, mutate) -> None:
    payload = yaml.safe_load((REPOSITORY / 'scenarios' / filename).read_text(encoding='utf-8'))
    mutate(payload)
    with pytest.raises(MissionValidationError):
        load_mission(_write(tmp_path, payload))


def test_scenario_hash_claim_is_recomputed_not_trusted(tmp_path: Path) -> None:
    payload = yaml.safe_load(SCENARIOS[0].read_text(encoding='utf-8'))
    payload['fault_schedule_sha256'] = '0' * 64
    with pytest.raises(MissionValidationError):
        load_mission(_write(tmp_path, payload))


def test_boolean_scenario_id_does_not_alias_integer_one(tmp_path: Path) -> None:
    payload = yaml.safe_load(SCENARIOS[0].read_text(encoding='utf-8'))
    payload['scenario_id'] = True
    with pytest.raises(MissionValidationError):
        load_mission(_write(tmp_path, payload))
