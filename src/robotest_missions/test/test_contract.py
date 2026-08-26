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

"""Static package and namespace-safety contract tests."""

import inspect
from argparse import Namespace
from pathlib import Path

import pytest
from robotest_missions.execution import ExitCode
from robotest_missions.fault_control import (
    ARM_SERVICE,
    EVENT_TOPIC,
    FAULT_EVENT_QOS,
    PRELOAD_SERVICE,
    RESET_SERVICE,
    RclpyFaultControlDriver,
)
from robotest_missions.mission_runner import CliError, _trial_identity, main
from robotest_missions.ros_action import (
    ACTION_NAME,
    ACTION_STATUS_TOPIC,
    RclpyFollowWaypointsDriver,
)

from robotest_missions.schema import load_mission

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parents[1]


def test_action_client_is_direct_relative_and_goal_fields_are_fixed() -> None:
    source = inspect.getsource(RclpyFollowWaypointsDriver)
    assert ACTION_NAME == 'follow_waypoints'
    assert ACTION_STATUS_TOPIC == 'follow_waypoints/_action/status'
    assert 'BasicNavigator' not in source
    assert 'goal.number_of_loops = 0' in source
    assert 'goal.goal_index = 0' in source
    assert 'ActionClient(node, FollowWaypoints, ACTION_NAME)' in source
    assert 'qos_profile_action_status_default' in source


def test_stable_exit_code_contract() -> None:
    assert {int(item) for item in ExitCode} == {0, 10, 20, 21, 22, 23, 24, 25}


def test_fault_control_names_are_relative_and_transport_is_direct() -> None:
    source = inspect.getsource(RclpyFaultControlDriver)
    assert RESET_SERVICE == 'faults/reset'
    assert PRELOAD_SERVICE == 'faults/preload_schedule'
    assert ARM_SERVICE == 'faults/arm_schedule'
    assert EVENT_TOPIC == 'faults/events'
    assert 'create_client' in source
    assert 'create_subscription' in source
    assert 'wait_for_service' not in source
    assert FAULT_EVENT_QOS.depth == 100
    assert FAULT_EVENT_QOS.reliability.name == 'RELIABLE'
    assert FAULT_EVENT_QOS.durability.name == 'VOLATILE'


def test_cli_validation_is_stable_and_does_not_initialize_ros() -> None:
    assert main([]) == ExitCode.VALIDATION_ERROR
    assert (
        main(
            [
                '--mission',
                str(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml'),
                '--json',
                '/tmp/robotest-phase3-contract.json',
                '--csv',
                '/tmp/robotest-phase3-contract.csv',
            ]
        )
        == ExitCode.VALIDATION_ERROR
    )


def test_package_metadata_and_schema_are_installed() -> None:
    setup = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    package_xml = (PACKAGE / 'package.xml').read_text(encoding='utf-8')
    assert 'mission_runner = robotest_missions.mission_runner:main' in setup
    assert "glob('schema/*.json')" in setup
    assert '<exec_depend>nav2_msgs</exec_depend>' in package_xml
    assert '<exec_depend>robotest_interfaces</exec_depend>' in package_xml
    assert '<exec_depend>std_srvs</exec_depend>' in package_xml


def _phase3_cli(**changes) -> Namespace:
    values = {
        'run_id': 'p3-candidate-a-s04-r1',
        'candidate_id': 'candidate-a',
        'repetition_index': 1,
        'suite_index': 10,
    }
    values.update(changes)
    return Namespace(**values)


def test_phase3_trial_identity_enforces_suite_formula() -> None:
    document = load_mission(REPOSITORY / 'scenarios' / 'phase3_s4_lidar_dropout.yaml')
    identity = _trial_identity(document, _phase3_cli())
    assert identity['suite_index'] == 10
    with pytest.raises(CliError, match='suite-index'):
        _trial_identity(document, _phase3_cli(suite_index=9))


@pytest.mark.parametrize('field', ['run_id', 'candidate_id', 'repetition_index', 'suite_index'])
def test_phase3_trial_identity_requires_every_field(field: str) -> None:
    document = load_mission(REPOSITORY / 'scenarios' / 'phase3_s1_baseline.yaml')
    with pytest.raises(CliError, match='requires'):
        _trial_identity(document, _phase3_cli(**{field: None}))


def test_phase2_rejects_phase3_identity_without_changing_default_cli() -> None:
    document = load_mission(REPOSITORY / 'scenarios' / 'phase2_baseline.yaml')
    with pytest.raises(CliError, match='require a Phase 3'):
        _trial_identity(document, _phase3_cli())
