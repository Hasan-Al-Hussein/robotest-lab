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

"""Strict Phase 3 scenario document loading."""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator

from robotest_scenarios.errors import ValidationError

MAX_SCENARIO_BYTES = 1_048_576
EXPECTED_NAMES = {
    1: 'baseline_navigation',
    2: 'deterministic_static_obstacle_replan',
    3: 'deterministic_dynamic_obstacle',
    4: 'temporary_lidar_dropout',
    5: 'deterministic_odometry_drift',
}
_PHASE3_FROZEN_CONTRACT = 'target-set revision 3 / ADR0007'
_PHASE3_WAYPOINTS = (
    (-2.0, -3.5, 0.0),
    (-0.2, 0.0, 0.0),
    (-0.2, 3.5, 0.0),
)


@dataclass(frozen=True, slots=True)
class PoseTarget:
    """One finite planar scenario pose with world z."""

    x: float
    y: float
    z: float
    yaw: float

    def as_dict(self) -> dict[str, float]:
        """Return stable artifact fields."""
        return {'x': self.x, 'y': self.y, 'yaw': self.yaw, 'z': self.z}


@dataclass(frozen=True, slots=True)
class Waypoint:
    """One ordered Nav2 waypoint."""

    x: float
    y: float
    yaw: float


@dataclass(frozen=True, slots=True)
class ScenarioDocument:
    """Validated scenario fields required by the controller."""

    path: Path
    sha256: str
    raw: dict[str, Any]
    scenario_id: int
    scenario_name: str
    controller_seed: int
    frame_id: str
    waypoints: tuple[Waypoint, ...]
    wall_escape_timeout_s: float
    actor_name: str | None
    actor_asset_name: str | None
    actor_initial_pose: PoseTarget | None

    @property
    def has_actor(self) -> bool:
        """Whether the scenario owns a Gazebo actor."""
        return self.actor_name is not None


def package_schema_path(name: str) -> Path:
    """Resolve a schema from source or an installed ament share directory."""
    source_candidate = Path(__file__).resolve().parents[1] / 'schema' / name
    if source_candidate.is_file():
        return source_candidate
    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('robotest_scenarios')) / 'schema' / name
    except Exception as exc:
        raise ValidationError(f'cannot resolve package schema {name}: {exc}') from exc
    if not installed.is_file():
        raise ValidationError(f'package schema is missing: {installed}')
    return installed


def _finite_float(value: Any, *, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValidationError(f'{label} must be numeric')
    converted = float(value)
    if not math.isfinite(converted):
        raise ValidationError(f'{label} must be finite')
    return converted


def _validate_schema(value: Any) -> None:
    path = package_schema_path('scenario-input.schema.json')
    try:
        schema = yaml.safe_load(path.read_text(encoding='utf-8'))
        errors = sorted(Draft202012Validator(schema).iter_errors(value), key=lambda item: item.path)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise ValidationError(f'cannot validate scenario schema: {exc}') from exc
    if errors:
        error = errors[0]
        location = '.'.join(str(part) for part in error.absolute_path) or '<root>'
        raise ValidationError(f'scenario schema violation at {location}: {error.message}')


def _assert_common_frozen(value: dict[str, Any]) -> tuple[Waypoint, ...]:
    actual_waypoints = tuple(
        (
            _finite_float(item['x'], label='waypoint.x'),
            _finite_float(item['y'], label='waypoint.y'),
            _finite_float(item['yaw'], label='waypoint.yaw'),
        )
        for item in value['waypoints']
    )
    if actual_waypoints != _PHASE3_WAYPOINTS:
        raise ValidationError(f'waypoints differ from the {_PHASE3_FROZEN_CONTRACT} frozen mission')
    frozen = {
        'allowed_collision_count': 0,
        'expected_outcome': 'succeeded',
        'frame_id': 'map',
        'mission_seed': 42,
        'mission_timeout_sim_s': 180.0,
        'retries': 0,
        'scenario_controller_seed': 42,
        'schema_version': 1,
        'simulator_seed': 42,
        'wall_escape_timeout_s': 300.0,
    }
    for key, expected in frozen.items():
        if value[key] != expected:
            raise ValidationError(
                f'{key} differs from the {_PHASE3_FROZEN_CONTRACT} frozen value {expected!r}'
            )
    start = value['start_pose']
    if (start['x'], start['y'], start['yaw']) != (0.0, -3.5, 0.0):
        raise ValidationError(f'start_pose differs from the {_PHASE3_FROZEN_CONTRACT} frozen pose')
    return tuple(Waypoint(*item) for item in actual_waypoints)


def _actor_fields(value: dict[str, Any]) -> tuple[str | None, str | None, PoseTarget | None]:
    scenario_id = value['scenario_id']
    if scenario_id == 2:
        actor = value['entity']
        pose = actor['world_pose']
        return (
            actor['name'],
            'phase3_static_block.sdf',
            PoseTarget(pose['x_m'], pose['y_m'], pose['z_m'], pose['yaw_rad']),
        )
    if scenario_id == 3:
        actor = value['entity']
        pose = actor['initial_world_pose']
        return (
            actor['name'],
            'phase3_dynamic_block.sdf',
            PoseTarget(pose['x_m'], pose['y_m'], pose['z_m'], pose['yaw_rad']),
        )
    return None, None, None


def load_scenario(path_value: str) -> ScenarioDocument:
    """Load one exact Phase 3 scenario without accepting YAML extensions."""
    path = Path(path_value).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValidationError(f'scenario path is not a regular file: {path}')
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise ValidationError(f'cannot read scenario {path}: {exc}') from exc
    if len(payload) > MAX_SCENARIO_BYTES:
        raise ValidationError(f'scenario exceeds {MAX_SCENARIO_BYTES} bytes')
    try:
        text = payload.decode('utf-8', errors='strict')
        value = yaml.safe_load(text)
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValidationError(f'invalid UTF-8 YAML scenario: {exc}') from exc
    _validate_schema(value)
    if not isinstance(value, dict):
        raise ValidationError('scenario root must be an object')
    scenario_id = value['scenario_id']
    if value['scenario_name'] != EXPECTED_NAMES[scenario_id]:
        raise ValidationError('scenario_id and scenario_name do not match')
    waypoints = _assert_common_frozen(value)
    actor_name, actor_asset_name, actor_initial_pose = _actor_fields(value)
    return ScenarioDocument(
        path=path,
        sha256=hashlib.sha256(payload).hexdigest(),
        raw=value,
        scenario_id=scenario_id,
        scenario_name=value['scenario_name'],
        controller_seed=value['scenario_controller_seed'],
        frame_id=value['frame_id'],
        waypoints=waypoints,
        wall_escape_timeout_s=float(value['wall_escape_timeout_s']),
        actor_name=actor_name,
        actor_asset_name=actor_asset_name,
        actor_initial_pose=actor_initial_pose,
    )
