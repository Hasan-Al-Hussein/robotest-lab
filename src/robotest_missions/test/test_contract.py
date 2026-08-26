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
from pathlib import Path

from robotest_missions.execution import ExitCode
from robotest_missions.mission_runner import main
from robotest_missions.ros_action import (
    ACTION_NAME,
    ACTION_STATUS_TOPIC,
    RclpyFollowWaypointsDriver,
)

PACKAGE = Path(__file__).resolve().parents[1]


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


def test_cli_validation_is_stable_and_does_not_initialize_ros() -> None:
    assert main([]) == ExitCode.VALIDATION_ERROR


def test_package_metadata_and_schema_are_installed() -> None:
    setup = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    package_xml = (PACKAGE / 'package.xml').read_text(encoding='utf-8')
    assert 'mission_runner = robotest_missions.mission_runner:main' in setup
    assert "glob('schema/*.json')" in setup
    assert '<exec_depend>nav2_msgs</exec_depend>' in package_xml
