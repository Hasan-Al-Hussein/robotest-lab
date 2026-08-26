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

"""Strict YAML loading and JSON Schema validation for mission inputs."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import yaml
from jsonschema import Draft202012Validator
from yaml.constructor import ConstructorError

from robotest_missions.fault_schedule import FaultScheduleError, build_fault_schedule
from robotest_missions.models import MissionConfig, MissionDocument, PlanarPose

MAX_MISSION_BYTES = 1024 * 1024
SCHEMA_FILENAME = 'mission.schema.json'


class MissionValidationError(ValueError):
    """Raised when mission bytes do not satisfy the complete input contract."""


class _UniqueKeySafeLoader(yaml.SafeLoader):
    """Safe YAML loader that rejects duplicate mapping keys."""


def _construct_unique_mapping(
    loader: _UniqueKeySafeLoader,
    node: yaml.MappingNode,
    deep: bool = False,
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in result
        except TypeError as exc:
            raise ConstructorError(
                'while constructing a mapping',
                node.start_mark,
                'found an unhashable mapping key',
                key_node.start_mark,
            ) from exc
        if duplicate:
            raise ConstructorError(
                'while constructing a mapping',
                node.start_mark,
                f'found duplicate key {key!r}',
                key_node.start_mark,
            )
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeySafeLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
    _construct_unique_mapping,
)


def _schema_path() -> Path:
    source_candidate = Path(__file__).resolve().parents[1] / 'schema' / SCHEMA_FILENAME
    if source_candidate.is_file():
        return source_candidate

    try:
        from ament_index_python.packages import get_package_share_directory

        installed = Path(get_package_share_directory('robotest_missions')) / 'schema'
        installed = installed / SCHEMA_FILENAME
    except (ImportError, LookupError):
        installed = Path()
    if installed.is_file():
        return installed
    raise MissionValidationError(f'mission schema is unavailable: {SCHEMA_FILENAME}')


def _load_schema() -> tuple[dict[str, Any], str]:
    path = _schema_path()
    raw = path.read_bytes()
    try:
        schema = json.loads(raw)
        Draft202012Validator.check_schema(schema)
    except (json.JSONDecodeError, ValueError) as exc:
        raise MissionValidationError(f'invalid installed mission schema: {exc}') from exc
    return schema, hashlib.sha256(raw).hexdigest()


def _reject_non_finite(value: Any, path: str = '$') -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise MissionValidationError(f'{path}: non-finite numbers are forbidden')
    if isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite(item, f'{path}.{key}')
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite(item, f'{path}[{index}]')


def _format_path(parts: Any) -> str:
    result = '$'
    for part in parts:
        result += f'[{part}]' if isinstance(part, int) else f'.{part}'
    return result


def _pose(value: dict[str, Any]) -> PlanarPose:
    return PlanarPose(x=float(value['x']), y=float(value['y']), yaw=float(value['yaw']))


def _most_specific_error(error: Any) -> Any:
    """Choose a useful leaf from a oneOf error without weakening validation."""
    if not error.context:
        return error
    leaves = [_most_specific_error(child) for child in error.context]
    return max(
        leaves,
        key=lambda child: (
            len(child.absolute_path),
            len(child.absolute_schema_path),
            -len(child.message),
        ),
    )


def load_mission(path: str | Path) -> MissionDocument:
    """Load one regular UTF-8 YAML file and validate it without coercion."""
    try:
        source = Path(path).expanduser().resolve(strict=True)
    except OSError as exc:
        raise MissionValidationError(f'mission cannot be resolved: {exc}') from exc
    if not source.is_file():
        raise MissionValidationError(f'mission is not a regular file: {source}')
    size = source.stat().st_size
    if size <= 0 or size > MAX_MISSION_BYTES:
        raise MissionValidationError(
            f'mission size must be between 1 and {MAX_MISSION_BYTES} bytes: {size}'
        )

    raw = source.read_bytes()
    try:
        text = raw.decode('utf-8')
    except UnicodeDecodeError as exc:
        raise MissionValidationError('mission must be UTF-8') from exc
    try:
        payload = yaml.load(text, Loader=_UniqueKeySafeLoader)
    except yaml.YAMLError as exc:
        raise MissionValidationError(f'invalid mission YAML: {exc}') from exc

    _reject_non_finite(payload)
    schema, schema_sha256 = _load_schema()
    validator = Draft202012Validator(schema)
    if isinstance(payload, dict) and 'scenario_id' in payload:
        scenario_id = payload.get('scenario_id')
        valid_scenario_id = (
            isinstance(scenario_id, int)
            and not isinstance(scenario_id, bool)
            and scenario_id in (1, 2, 3, 4, 5)
        )
        branch_name = f'phase3_s{scenario_id}' if valid_scenario_id else None
        validation_schema = (
            {'$ref': f'#/$defs/{branch_name}'} if branch_name is not None else schema
        )
    else:
        validation_schema = {'$ref': '#/$defs/phase2'}
    errors = sorted(
        validator.evolve(schema=validation_schema).iter_errors(payload),
        key=lambda error: tuple(str(part) for part in error.absolute_path),
    )
    if errors:
        first = _most_specific_error(errors[0])
        raise MissionValidationError(f'{_format_path(first.absolute_path)}: {first.message}')

    try:
        json.dumps(payload, allow_nan=False, sort_keys=True)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f'mission is not canonical JSON-compatible: {exc}') from exc

    scenario_id = payload.get('scenario_id')
    schedule = None
    scenario_contract = None
    if scenario_id is not None:
        raw_faults = (
            [payload['fault']] if 'fault' in payload else payload['fault_schedule']['faults']
        )
        try:
            schedule = build_fault_schedule(
                raw_faults,
                claimed_sha256=payload['fault_schedule_sha256'],
            )
        except FaultScheduleError as exc:
            raise MissionValidationError(f'$.fault_schedule: {exc}') from exc
        scenario_contract = json.loads(
            json.dumps(payload, allow_nan=False, separators=(',', ':'), sort_keys=True)
        )

    config = MissionConfig(
        schema_version=payload['schema_version'],
        mission_name=payload.get('mission_name', payload.get('scenario_name')),
        mission_seed=payload['mission_seed'],
        simulator_seed=payload['simulator_seed'],
        frame_id=payload['frame_id'],
        start_pose=_pose(payload['start_pose']),
        waypoints=tuple(_pose(value) for value in payload['waypoints']),
        mission_timeout_sim_s=float(payload['mission_timeout_sim_s']),
        wall_escape_timeout_s=float(payload['wall_escape_timeout_s']),
        allowed_collision_count=payload['allowed_collision_count'],
        fault_schedule=schedule,
        fault_seed=payload['fault_seed'],
        expected_outcome=payload['expected_outcome'],
        retries=payload['retries'],
        scenario_id=scenario_id,
        scenario_controller_seed=payload.get('scenario_controller_seed'),
        scenario_contract=scenario_contract,
    )
    return MissionDocument(
        config=config,
        source_path=source,
        source_sha256=hashlib.sha256(raw).hexdigest(),
        schema_sha256=schema_sha256,
    )
