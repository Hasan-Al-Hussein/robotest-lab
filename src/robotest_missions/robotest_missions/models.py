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

"""Immutable mission-domain models independent of the ROS graph."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True, slots=True)
class PlanarPose:
    """A finite planar pose in the mission frame."""

    x: float
    y: float
    yaw: float

    def as_dict(self) -> dict[str, float]:
        """Return the stable JSON representation."""
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw}


@dataclass(frozen=True, slots=True)
class FaultSpec:
    """One normalized deterministic fault specification."""

    schema_version: int
    fault_id: str
    target: int
    mode: int
    start_offset_ns: int
    duration_ns: int
    seed: int
    parameters: tuple[tuple[str, int], ...]

    def as_dict(self) -> dict[str, Any]:
        """Return the canonical property order frozen by ADR 0005."""
        return {
            'schema_version': self.schema_version,
            'fault_id': self.fault_id,
            'target': self.target,
            'mode': self.mode,
            'start_offset_ns': self.start_offset_ns,
            'duration_ns': self.duration_ns,
            'seed': self.seed,
            'parameters': dict(self.parameters),
        }


@dataclass(frozen=True, slots=True)
class FaultSchedule:
    """Canonical fault schedule plus its independently computed identity."""

    schema_version: int
    faults: tuple[FaultSpec, ...]
    canonical_json: str
    sha256: str

    @property
    def earliest_start_offset_ns(self) -> int | None:
        """Return the first configured activation offset, if any."""
        return None if not self.faults else self.faults[0].start_offset_ns

    def as_dict(self) -> dict[str, Any]:
        """Return normalized schedule content without derived identity."""
        return {
            'schema_version': self.schema_version,
            'faults': [fault.as_dict() for fault in self.faults],
        }


@dataclass(frozen=True, slots=True)
class MissionConfig:
    """Validated mission input used by both pure logic and ROS transport."""

    schema_version: int
    mission_name: str
    mission_seed: int
    simulator_seed: int
    frame_id: str
    start_pose: PlanarPose
    waypoints: tuple[PlanarPose, ...]
    mission_timeout_sim_s: float
    wall_escape_timeout_s: float
    allowed_collision_count: int
    fault_schedule: FaultSchedule | None
    fault_seed: int | None
    expected_outcome: str
    retries: int
    scenario_id: int | None = None
    scenario_controller_seed: int | None = None
    scenario_contract: dict[str, Any] | None = None

    @property
    def is_phase3(self) -> bool:
        """Return whether this is one of the frozen Phase 3 scenarios."""
        return self.scenario_id is not None

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-safe representation without runtime metadata."""
        return {
            'schema_version': self.schema_version,
            'mission_name': self.mission_name,
            'mission_seed': self.mission_seed,
            'simulator_seed': self.simulator_seed,
            'frame_id': self.frame_id,
            'start_pose': self.start_pose.as_dict(),
            'waypoints': [waypoint.as_dict() for waypoint in self.waypoints],
            'mission_timeout_sim_s': self.mission_timeout_sim_s,
            'wall_escape_timeout_s': self.wall_escape_timeout_s,
            'allowed_collision_count': self.allowed_collision_count,
            'fault_schedule': (
                None if self.fault_schedule is None else self.fault_schedule.as_dict()
            ),
            'fault_seed': self.fault_seed,
            'expected_outcome': self.expected_outcome,
            'retries': self.retries,
            'scenario_id': self.scenario_id,
            'scenario_controller_seed': self.scenario_controller_seed,
            'scenario_contract': self.scenario_contract,
        }


@dataclass(frozen=True, slots=True)
class MissionDocument:
    """Validated mission plus immutable source provenance."""

    config: MissionConfig
    source_path: Path
    source_sha256: str
    schema_sha256: str
