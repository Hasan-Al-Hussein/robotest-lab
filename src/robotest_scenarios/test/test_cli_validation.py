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

"""Fail-closed CLI identity, artifact-path, and frozen-timeout tests."""

from pathlib import Path

import pytest
from robotest_scenarios.contact_control_driver import _inputs as contact_inputs
from robotest_scenarios.errors import ValidationError
from robotest_scenarios.scenario_controller import _inputs as scenario_inputs

REPOSITORY = Path(__file__).resolve().parents[3]
SCENARIO = REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'
MANIFEST = REPOSITORY / 'config' / 'collision-coverage.yaml'


def _scenario_args(tmp_path: Path) -> list[str]:
    output = tmp_path / 'scenario-result.json'
    return [
        'scenario_controller',
        '--scenario',
        str(SCENARIO),
        '--output',
        str(output),
        '--ready-file',
        str(tmp_path / 'scenario-ready.json'),
        '--run-id',
        'run-1',
        '--candidate-id',
        'candidate-1',
        '--repetition-index',
        '0',
        '--suite-index',
        '0',
    ]


def _contact_args(tmp_path: Path) -> list[str]:
    return [
        'contact_control_driver',
        '--output',
        str(tmp_path / 'contact-result.json'),
        '--ready-file',
        str(tmp_path / 'contact-ready.json'),
        '--run-id',
        'control-1',
        '--coverage-manifest',
        str(MANIFEST),
    ]


def test_scenario_rejects_ready_path_aliasing_result_sidecar(tmp_path: Path) -> None:
    args = _scenario_args(tmp_path)
    args[args.index('--ready-file') + 1] = str(tmp_path / 'scenario-result.json.sha256')
    with pytest.raises(ValidationError, match='checksum sidecar'):
        scenario_inputs(args)


def test_contact_rejects_ready_path_aliasing_result_sidecar(tmp_path: Path) -> None:
    args = _contact_args(tmp_path)
    args[args.index('--ready-file') + 1] = str(tmp_path / 'contact-result.json.sha256')
    with pytest.raises(ValidationError, match='checksum sidecar'):
        contact_inputs(args)


def test_release_wall_timeouts_cannot_be_shortened(tmp_path: Path) -> None:
    scenario_args = [*_scenario_args(tmp_path), '--wall-timeout-s', '299']
    with pytest.raises(ValidationError, match=r'frozen at 300\.0'):
        scenario_inputs(scenario_args)
    contact_args = [*_contact_args(tmp_path), '--wall-timeout-s', '29']
    with pytest.raises(ValidationError, match=r'frozen at 30\.0'):
        contact_inputs(contact_args)
