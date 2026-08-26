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
    fault_schedule: None
    fault_seed: None
    expected_outcome: str
    retries: int

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
            'fault_schedule': self.fault_schedule,
            'fault_seed': self.fault_seed,
            'expected_outcome': self.expected_outcome,
            'retries': self.retries,
        }


@dataclass(frozen=True, slots=True)
class MissionDocument:
    """Validated mission plus immutable source provenance."""

    config: MissionConfig
    source_path: Path
    source_sha256: str
    schema_sha256: str
