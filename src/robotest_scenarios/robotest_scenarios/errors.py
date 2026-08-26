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

"""Typed failures projected into stable process exit codes."""

from __future__ import annotations

from robotest_scenarios.constants import ExitCode


class RobotestScenarioError(RuntimeError):
    """Base class for deterministic scenario failures."""

    exit_code = ExitCode.PROTOCOL_ERROR


class ValidationError(RobotestScenarioError):
    """Configuration or command-line input is invalid."""

    exit_code = ExitCode.VALIDATION_ERROR


class WallTimeoutError(RobotestScenarioError):
    """A steady-wall escape deadline elapsed."""

    exit_code = ExitCode.WALL_TIMEOUT


class ProtocolError(RobotestScenarioError):
    """Observed data violates the frozen ordering or boundedness contract."""

    exit_code = ExitCode.PROTOCOL_ERROR


class InfrastructureError(RobotestScenarioError):
    """A required ROS interface or transaction failed."""

    exit_code = ExitCode.INFRASTRUCTURE_ERROR


class ArtifactError(RobotestScenarioError):
    """A required artifact could not be safely finalized."""

    exit_code = ExitCode.ARTIFACT_ERROR


class ScenarioFailureError(RobotestScenarioError):
    """The intended interaction was observed but did not satisfy its contract."""

    exit_code = ExitCode.SCENARIO_FAILED
