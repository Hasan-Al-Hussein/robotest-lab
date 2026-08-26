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

"""Pure collision-coverage and positive-control episode logic."""

from __future__ import annotations

import hashlib
import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

from robotest_scenarios.constants import CONTACT_RELEASE_GAP_NS, CONTROL_WALL_COLLISION
from robotest_scenarios.errors import ProtocolError, ValidationError

EXPECTED_ROLES = {
    'chassis',
    'front_caster',
    'imu_body',
    'left_wheel',
    'lidar_body',
    'rear_caster',
    'right_wheel',
}
EXPECTED_SCOPED_COLLISIONS = {
    'chassis': (
        'robotest::base_footprint::base_footprint_fixed_joint_lump__base_link_collision_collision'
    ),
    'front_caster': (
        'robotest::front_caster_link::'
        'front_caster_link_fixed_joint_lump__front_caster_collision_collision'
    ),
    'imu_body': 'robotest::imu_link::imu_link_collision_collision',
    'left_wheel': (
        'robotest::left_wheel_link::'
        'left_wheel_link_fixed_joint_lump__left_wheel_collision_collision'
    ),
    'lidar_body': 'robotest::lidar_link::lidar_link_collision_collision',
    'rear_caster': (
        'robotest::rear_caster_link::'
        'rear_caster_link_fixed_joint_lump__rear_caster_collision_collision'
    ),
    'right_wheel': (
        'robotest::right_wheel_link::'
        'right_wheel_link_fixed_joint_lump__right_wheel_collision_collision'
    ),
}
EXPECTED_GEOMETRIES = {
    'chassis': {
        'link': 'base_footprint',
        'collision': 'base_footprint_fixed_joint_lump__base_link_collision_collision',
        'contact_sensor': 'chassis_contact_sensor',
    },
    'left_wheel': {
        'link': 'left_wheel_link',
        'collision': 'left_wheel_link_fixed_joint_lump__left_wheel_collision_collision',
        'contact_sensor': 'left_wheel_contact_sensor',
    },
    'right_wheel': {
        'link': 'right_wheel_link',
        'collision': 'right_wheel_link_fixed_joint_lump__right_wheel_collision_collision',
        'contact_sensor': 'right_wheel_contact_sensor',
    },
    'front_caster': {
        'link': 'front_caster_link',
        'collision': ('front_caster_link_fixed_joint_lump__front_caster_collision_collision'),
        'contact_sensor': 'front_caster_contact_sensor',
    },
    'rear_caster': {
        'link': 'rear_caster_link',
        'collision': 'rear_caster_link_fixed_joint_lump__rear_caster_collision_collision',
        'contact_sensor': 'rear_caster_contact_sensor',
    },
    'lidar_body': {
        'link': 'lidar_link',
        'collision': 'lidar_link_collision_collision',
        'contact_sensor': 'lidar_body_contact_sensor',
    },
    'imu_body': {
        'link': 'imu_link',
        'collision': 'imu_link_collision_collision',
        'contact_sensor': 'imu_body_contact_sensor',
    },
}
GROUND_COLLISION = 'ground_plane::ground_link::ground_collision'
FROZEN_CONTACT_TOPIC = '/robotest/validation/contacts'
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


@dataclass(frozen=True, slots=True)
class CoverageManifest:
    """Exact rendered robot collision set and support allowlist."""

    path: Path
    sha256: str
    raw_contact_topic: str
    robot_collisions: frozenset[str]
    support_exclusions: frozenset[frozenset[str]]
    chassis_collision: str
    bridge_sha256: str
    contact_configuration_sha256: str
    rendered_sdf_sha256: str
    robot_description_sha256: str
    world_source_sha256: str

    def provenance(self) -> dict[str, str]:
        """Return the verified manifest and source provenance bindings."""
        return {
            'bridge_sha256': self.bridge_sha256,
            'contact_configuration_sha256': self.contact_configuration_sha256,
            'coverage_manifest_sha256': self.sha256,
            'rendered_sdf_sha256': self.rendered_sdf_sha256,
            'robot_description_sha256': self.robot_description_sha256,
            'world_source_sha256': self.world_source_sha256,
        }

    @property
    def expected_control_pair(self) -> tuple[str, str]:
        """Return the normalized exact chassis-to-wall pair."""
        return tuple(sorted((self.chassis_collision, CONTROL_WALL_COLLISION)))

    def classify(self, collision_a: str, collision_b: str) -> dict[str, Any] | None:
        """Normalize one pair and apply only the frozen exclusions."""
        top_level_model(collision_a)
        top_level_model(collision_b)
        unknown_robot = [
            collision
            for collision in (collision_a, collision_b)
            if collision.partition('::')[0] == 'robotest' and collision not in self.robot_collisions
        ]
        if unknown_robot:
            raise ProtocolError(
                f'contact names unknown rendered robot collision: {unknown_robot[0]!r}'
            )
        pair_set = frozenset((collision_a, collision_b))
        a_robot = collision_a in self.robot_collisions
        b_robot = collision_b in self.robot_collisions
        if a_robot and b_robot:
            return None
        if pair_set in self.support_exclusions:
            return None
        if a_robot == b_robot:
            return None
        robot_collision = collision_a if a_robot else collision_b
        counterpart_collision = collision_b if a_robot else collision_a
        return {
            'counterpart_collision': counterpart_collision,
            'counterpart_model': top_level_model(counterpart_collision),
            'normalized_pair': sorted((robot_collision, counterpart_collision)),
            'robot_collision': robot_collision,
        }


def _bounded_string(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or not value or len(value.encode('utf-8')) > 4096:
        raise ValidationError(f'{label} must be non-empty UTF-8 within 4096 bytes')
    return value


def _sha256(value: Any, *, label: str) -> str:
    text = _bounded_string(value, label=label)
    if _SHA256_PATTERN.fullmatch(text) is None:
        raise ValidationError(f'{label} must be a lowercase SHA-256 digest')
    return text


def _canonical_manifest_sha256(value: dict[str, Any]) -> str:
    unsigned = dict(value)
    unsigned.pop('manifest_sha256', None)
    try:
        encoded = (
            json.dumps(
                unsigned,
                allow_nan=False,
                ensure_ascii=False,
                separators=(',', ':'),
                sort_keys=True,
            )
            + '\n'
        ).encode('utf-8')
    except (TypeError, ValueError) as exc:
        raise ValidationError(f'coverage manifest is not canonical-JSON-safe: {exc}') from exc
    return hashlib.sha256(encoded).hexdigest()


def load_coverage_manifest(path_value: str) -> CoverageManifest:
    """Load and self-verify the exact revision-2 collision coverage manifest."""
    if len(path_value.encode('utf-8')) > 4096:
        raise ValidationError('coverage manifest path exceeds 4096 UTF-8 bytes')
    path = Path(path_value).expanduser().resolve(strict=True)
    if not path.is_file():
        raise ValidationError(f'coverage manifest is not a regular file: {path}')
    payload = path.read_bytes()
    if len(payload) > 1_048_576:
        raise ValidationError('coverage manifest exceeds 1 MiB')
    try:
        value = yaml.safe_load(payload.decode('utf-8', errors='strict'))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValidationError(f'invalid coverage manifest YAML: {exc}') from exc
    root_keys = {
        'bridge_sha256',
        'contact_configuration_sha256',
        'contact_topic',
        'covered_collisions',
        'manifest_sha256',
        'rendered_robot_collisions',
        'rendered_sdf_sha256',
        'robot_collisions',
        'robot_description_sha256',
        'robot_model',
        'schema_version',
        'support_pairs',
        'world_source_sha256',
    }
    if not isinstance(value, dict) or set(value) != root_keys:
        raise ValidationError('coverage manifest root fields do not match revision 2')
    if value['schema_version'] != 2 or value['robot_model'] != 'robotest':
        raise ValidationError('coverage manifest schema/model does not match RoboTest')
    if value['contact_topic'] != FROZEN_CONTACT_TOPIC:
        raise ValidationError('coverage manifest contact topic differs from the frozen topic')
    declared_sha256 = _sha256(value['manifest_sha256'], label='manifest SHA-256')
    if declared_sha256 != _canonical_manifest_sha256(value):
        raise ValidationError('coverage manifest self-hash does not match canonical content')
    geometries = value['robot_collisions']
    if not isinstance(geometries, list) or len(geometries) != 7:
        raise ValidationError('coverage manifest must contain exactly seven robot geometries')
    roles: set[str] = set()
    collisions: set[str] = set()
    chassis: str | None = None
    required_geometry_keys = {'name', 'role', 'source', 'link', 'collision', 'contact_sensor'}
    for item in geometries:
        if not isinstance(item, dict) or set(item) != required_geometry_keys:
            raise ValidationError('coverage geometry fields do not match revision 1')
        role = _bounded_string(item['role'], label='coverage role')
        scoped = _bounded_string(item['name'], label='scoped collision')
        if role in roles or scoped in collisions:
            raise ValidationError('coverage roles and scoped collisions must be unique')
        if scoped != EXPECTED_SCOPED_COLLISIONS.get(role):
            raise ValidationError('coverage role does not bind its frozen scoped collision')
        expected_geometry = EXPECTED_GEOMETRIES.get(role)
        if expected_geometry is None or any(
            _bounded_string(item[key], label=f'{role} {key}') != expected_geometry[key]
            for key in ('link', 'collision', 'contact_sensor')
        ):
            raise ValidationError('coverage geometry proof differs from the frozen rendering')
        if item['source'] != FROZEN_CONTACT_TOPIC:
            raise ValidationError('coverage geometry source differs from the frozen topic')
        roles.add(role)
        collisions.add(scoped)
        if role == 'chassis':
            chassis = scoped
    if roles != EXPECTED_ROLES or chassis is None:
        raise ValidationError('coverage manifest does not enumerate the frozen rendered roles')
    for key in ('covered_collisions', 'rendered_robot_collisions'):
        observed = value[key]
        if (
            not isinstance(observed, list)
            or len(observed) != len(collisions)
            or any(not isinstance(item, str) for item in observed)
            or set(observed) != collisions
        ):
            raise ValidationError(f'{key} must equal the exact rendered collision set')
    exclusions = value['support_pairs']
    if not isinstance(exclusions, list) or len(exclusions) != 4:
        raise ValidationError('coverage manifest must contain four support-ground exclusions')
    support_pairs: set[frozenset[str]] = set()
    for item in exclusions:
        if not isinstance(item, dict) or set(item) != {
            'robot_collision',
            'environment_collision',
        }:
            raise ValidationError('support-ground exclusion fields are invalid')
        robot_collision = _bounded_string(item['robot_collision'], label='support collision')
        counterpart = _bounded_string(item['environment_collision'], label='ground collision')
        if robot_collision not in collisions:
            raise ValidationError('support-ground exclusion names an unknown robot collision')
        support_pairs.add(frozenset((robot_collision, counterpart)))
    if len(support_pairs) != 4:
        raise ValidationError('support-ground exclusions must be unique')
    expected_support_pairs = {
        frozenset((EXPECTED_SCOPED_COLLISIONS[role], GROUND_COLLISION))
        for role in ('left_wheel', 'right_wheel', 'front_caster', 'rear_caster')
    }
    if support_pairs != expected_support_pairs:
        raise ValidationError('support-ground exclusions differ from the frozen exact pairs')
    return CoverageManifest(
        path=path,
        sha256=declared_sha256,
        raw_contact_topic=_bounded_string(value['contact_topic'], label='contact topic'),
        robot_collisions=frozenset(collisions),
        support_exclusions=frozenset(support_pairs),
        chassis_collision=chassis,
        bridge_sha256=_sha256(value['bridge_sha256'], label='bridge SHA-256'),
        contact_configuration_sha256=_sha256(
            value['contact_configuration_sha256'],
            label='contact configuration SHA-256',
        ),
        rendered_sdf_sha256=_sha256(value['rendered_sdf_sha256'], label='rendered SDF SHA-256'),
        robot_description_sha256=_sha256(
            value['robot_description_sha256'], label='robot description SHA-256'
        ),
        world_source_sha256=_sha256(value['world_source_sha256'], label='world source SHA-256'),
    )


def top_level_model(scoped_collision: str) -> str:
    """Extract an exact top-level Gazebo model name."""
    model, separator, remainder = scoped_collision.partition('::')
    if not separator or not model or not remainder:
        raise ProtocolError(f'invalid scoped collision name: {scoped_collision!r}')
    return model


@dataclass(slots=True)
class ContactEpisodeTracker:
    """De-duplicate one counterpart model using the frozen release gap."""

    counterpart_model: str
    release_gap_ns: int = CONTACT_RELEASE_GAP_NS
    active_start_ns: int | None = None
    last_contact_ns: int | None = None
    active_pairs: set[tuple[str, str]] = field(default_factory=set)
    active_sample_count: int = 0
    episodes: list[dict[str, Any]] = field(default_factory=list)

    def observe(self, stamp_ns: int, normalized_pair: tuple[str, str]) -> None:
        """Observe one normalized non-excluded contact record."""
        if stamp_ns <= 0:
            raise ProtocolError('contact stamp must be positive')
        if self.last_contact_ns is not None and stamp_ns < self.last_contact_ns:
            raise ProtocolError('contact stamp regressed')
        if (
            self.active_start_ns is not None
            and self.last_contact_ns is not None
            and stamp_ns - self.last_contact_ns >= self.release_gap_ns
        ):
            self._close(self.last_contact_ns + self.release_gap_ns)
        if self.active_start_ns is None:
            self.active_start_ns = stamp_ns
            self.active_pairs = set()
            self.active_sample_count = 0
        self.last_contact_ns = stamp_ns
        self.active_pairs.add(normalized_pair)
        self.active_sample_count += 1

    def advance(self, stamp_ns: int) -> None:
        """Close an active episode after a full quiet release gap."""
        if (
            self.active_start_ns is not None
            and self.last_contact_ns is not None
            and stamp_ns >= self.last_contact_ns + self.release_gap_ns
        ):
            self._close(self.last_contact_ns + self.release_gap_ns)

    def _close(self, end_stamp_ns: int) -> None:
        assert self.active_start_ns is not None
        self.episodes.append(
            {
                'counterpart_model': self.counterpart_model,
                'end_stamp_ns': end_stamp_ns,
                'normalized_pairs': [list(pair) for pair in sorted(self.active_pairs)],
                'sample_count': self.active_sample_count,
                'start_stamp_ns': self.active_start_ns,
            }
        )
        self.active_start_ns = None
        self.last_contact_ns = None
        self.active_pairs = set()
        self.active_sample_count = 0
