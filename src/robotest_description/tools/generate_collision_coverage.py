#!/usr/bin/env python3
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

"""Generate and verify the canonical Phase 3 collision-coverage manifest."""

# ruff: noqa: I001

from __future__ import annotations

import argparse
from collections.abc import Mapping, Sequence
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET

import yaml

SCHEMA_VERSION = 2
ROBOT_MODEL = 'robotest'
CONTACT_TOPIC = '/robotest/validation/contacts'
GROUND_COLLISION = 'ground_plane::ground_link::ground_collision'
CONTACT_UPDATE_RATE_HZ = 5.0
RENDER_ARGUMENTS = {
    'enable_ground_truth': 'true',
    'namespace': '/robotest',
    'use_gazebo': 'true',
}
ROLE_BINDINGS = (
    ('chassis', 'base_footprint', 'chassis_contact_sensor'),
    ('left_wheel', 'left_wheel_link', 'left_wheel_contact_sensor'),
    ('right_wheel', 'right_wheel_link', 'right_wheel_contact_sensor'),
    ('front_caster', 'front_caster_link', 'front_caster_contact_sensor'),
    ('rear_caster', 'rear_caster_link', 'rear_caster_contact_sensor'),
    ('lidar_body', 'lidar_link', 'lidar_body_contact_sensor'),
    ('imu_body', 'imu_link', 'imu_body_contact_sensor'),
)
SUPPORT_ROLES = ('left_wheel', 'right_wheel', 'front_caster', 'rear_caster')
BRIDGE_CONTACT_ENTRY = {
    'ros_topic_name': 'validation/contacts',
    'gz_topic_name': CONTACT_TOPIC,
    'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
    'gz_type_name': 'gz.msgs.Contacts',
    'direction': 'GZ_TO_ROS',
    'publisher_queue': 10,
    'subscriber_queue': 10,
    'qos_profile': 'SERVICES',
}


class CoverageGenerationError(RuntimeError):
    """Raised when source evidence cannot produce the frozen manifest."""


def canonical_json_bytes(value: Any) -> bytes:
    """Return the manifest's exact sorted, compact UTF-8 JSON representation."""
    try:
        text = json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(',', ':'),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise CoverageGenerationError(f'value is not canonicalizable: {exc}') from exc
    return (text + '\n').encode('utf-8')


def canonical_sha256(value: Any) -> str:
    """Hash the canonical JSON bytes used by all semantic bindings."""
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def file_sha256(path: Path) -> str:
    """Hash a source file without normalizing its bytes."""
    digest = hashlib.sha256()
    try:
        with path.open('rb') as source:
            while block := source.read(1024 * 1024):
                digest.update(block)
    except OSError as exc:
        raise CoverageGenerationError(f'cannot hash {path}: {exc}') from exc
    return digest.hexdigest()


def _run(command: Sequence[str], *, label: str) -> bytes:
    environment = os.environ.copy()
    environment.update({'LANG': 'C.UTF-8', 'LC_ALL': 'C.UTF-8', 'TZ': 'UTC'})
    try:
        result = subprocess.run(
            list(command),
            check=True,
            capture_output=True,
            env=environment,
        )
    except FileNotFoundError as exc:
        raise CoverageGenerationError(f'{label} executable is unavailable: {command[0]}') from exc
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.decode('utf-8', errors='replace').strip()
        raise CoverageGenerationError(f'{label} failed: {detail}') from exc
    if not result.stdout:
        raise CoverageGenerationError(f'{label} produced no output')
    return result.stdout


def robot_description_sha256(repository_root: Path) -> str:
    """Hash every Xacro source and the exact render arguments as one inventory."""
    urdf_directory = repository_root / 'src' / 'robotest_description' / 'urdf'
    sources = sorted(urdf_directory.glob('*.xacro'))
    if not sources:
        raise CoverageGenerationError('robot description contains no Xacro sources')
    inventory = {
        'render_arguments': RENDER_ARGUMENTS,
        'source_files': [
            {
                'path': source.relative_to(repository_root).as_posix(),
                'sha256': file_sha256(source),
            }
            for source in sources
        ],
    }
    return canonical_sha256(inventory)


def render_robot_sdf(repository_root: Path) -> bytes:
    """Render the launch-equivalent Xacro and print its deterministic SDF 1.11 form."""
    xacro_path = repository_root / 'src' / 'robotest_description' / 'urdf' / 'robotest.urdf.xacro'
    if not xacro_path.is_file():
        raise CoverageGenerationError(f'robot Xacro is missing: {xacro_path}')
    urdf = _run(
        [
            'xacro',
            str(xacro_path),
            'namespace:=/robotest',
            'use_gazebo:=true',
            'enable_ground_truth:=true',
        ],
        label='Xacro rendering',
    )
    temporary_name: str | None = None
    try:
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as temporary:
            temporary.write(urdf)
            temporary.flush()
            os.fsync(temporary.fileno())
            temporary_name = temporary.name
        rendered = _run(
            ['gz', 'sdf', '--precision', '17', '-p', temporary_name],
            label='SDFormat rendering',
        )
    finally:
        if temporary_name is not None:
            Path(temporary_name).unlink(missing_ok=True)
    try:
        root = ET.fromstring(rendered)
    except ET.ParseError as exc:
        raise CoverageGenerationError(f'rendered SDF is invalid XML: {exc}') from exc
    if root.tag != 'sdf' or root.get('version') != '1.11':
        raise CoverageGenerationError('rendered SDF must be the frozen SDF 1.11 form')
    return rendered


def extract_rendered_coverage(rendered_sdf: bytes) -> tuple[list[dict[str, str]], str]:
    """Validate rendered collisions/contact sensors and return their exact projection."""
    try:
        root = ET.fromstring(rendered_sdf)
    except ET.ParseError as exc:
        raise CoverageGenerationError(f'rendered SDF is invalid XML: {exc}') from exc
    models = root.findall('model')
    if len(models) != 1 or models[0].get('name') != ROBOT_MODEL:
        raise CoverageGenerationError('rendered SDF must contain exactly the robotest model')
    model = models[0]
    links = {link.get('name'): link for link in model.findall('link')}
    expected_links = {link_name for _, link_name, _ in ROLE_BINDINGS}
    collision_links = {
        link_name
        for link_name, link in links.items()
        if link is not None and link.findall('collision')
    }
    if collision_links != expected_links:
        raise CoverageGenerationError(
            'rendered collision links do not equal the frozen seven-link set'
        )
    geometries: list[dict[str, str]] = []
    contact_projection: list[dict[str, Any]] = []
    for role, link_name, sensor_name in ROLE_BINDINGS:
        link = links.get(link_name)
        if link is None:
            raise CoverageGenerationError(f'rendered link is missing: {link_name}')
        collisions = link.findall('collision')
        if len(collisions) != 1:
            raise CoverageGenerationError(
                f'{link_name} must contain exactly one rendered collision'
            )
        collision_name = collisions[0].get('name')
        if not collision_name:
            raise CoverageGenerationError(f'{link_name} rendered collision has no name')
        contact_sensors = [
            sensor for sensor in link.findall('sensor') if sensor.get('type') == 'contact'
        ]
        if len(contact_sensors) != 1 or contact_sensors[0].get('name') != sensor_name:
            raise CoverageGenerationError(
                f'{link_name} must have exactly the frozen contact sensor {sensor_name}'
            )
        sensor = contact_sensors[0]
        target_collision = sensor.findtext('contact/collision')
        topic = sensor.findtext('contact/topic')
        update_rate_text = sensor.findtext('update_rate')
        try:
            update_rate = float(update_rate_text) if update_rate_text is not None else math.nan
        except ValueError as exc:
            raise CoverageGenerationError(f'{sensor_name} update rate is invalid') from exc
        if (
            target_collision != collision_name
            or topic != CONTACT_TOPIC
            or sensor.findtext('always_on') != 'true'
            or not math.isfinite(update_rate)
            or update_rate != CONTACT_UPDATE_RATE_HZ
        ):
            raise CoverageGenerationError(
                f'{sensor_name} does not exactly cover its collision on the frozen topic/rate'
            )
        scoped_name = f'{ROBOT_MODEL}::{link_name}::{collision_name}'
        geometries.append(
            {
                'name': scoped_name,
                'role': role,
                'source': CONTACT_TOPIC,
                'link': link_name,
                'collision': collision_name,
                'contact_sensor': sensor_name,
            }
        )
        contact_projection.append(
            {
                'collision': collision_name,
                'contact_sensor': sensor_name,
                'link': link_name,
                'source': CONTACT_TOPIC,
                'update_rate_hz': update_rate,
            }
        )
    if len({item['name'] for item in geometries}) != len(ROLE_BINDINGS):
        raise CoverageGenerationError('rendered scoped collision names are not unique')
    contact_configuration = {
        'contact_topic': CONTACT_TOPIC,
        'robot_model': ROBOT_MODEL,
        'schema_version': 1,
        'sensors': contact_projection,
    }
    return geometries, canonical_sha256(contact_configuration)


def bridge_sha256(repository_root: Path) -> str:
    """Validate the exact contact bridge entry and hash the complete bridge source."""
    path = repository_root / 'src' / 'robotest_sim' / 'config' / 'bridge.yaml'
    try:
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise CoverageGenerationError(f'cannot load bridge configuration: {exc}') from exc
    if not isinstance(document, list):
        raise CoverageGenerationError('bridge configuration root must be a list')
    contact_entries = [
        entry
        for entry in document
        if isinstance(entry, Mapping)
        and (
            entry.get('ros_topic_name') == BRIDGE_CONTACT_ENTRY['ros_topic_name']
            or entry.get('gz_topic_name') == CONTACT_TOPIC
        )
    ]
    if len(contact_entries) != 1 or dict(contact_entries[0]) != BRIDGE_CONTACT_ENTRY:
        raise CoverageGenerationError('contact bridge entry differs from the frozen mapping')
    return file_sha256(path)


def world_source_sha256(repository_root: Path) -> str:
    """Validate the Contact system and exact ground collision, then hash the world."""
    path = repository_root / 'src' / 'robotest_sim' / 'worlds' / 'robotest_lab.sdf'
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        raise CoverageGenerationError(f'cannot parse simulation world: {exc}') from exc
    world = root.find("./world[@name='robotest_lab']")
    if world is None:
        raise CoverageGenerationError('robotest_lab world is missing')
    contact_plugins = [
        plugin
        for plugin in world.findall('plugin')
        if plugin.get('filename') == 'gz-sim-contact-system'
        and plugin.get('name') == 'gz::sim::systems::Contact'
    ]
    ground = world.find(
        "./model[@name='ground_plane']/link[@name='ground_link']/"
        "collision[@name='ground_collision']"
    )
    if len(contact_plugins) != 1 or ground is None:
        raise CoverageGenerationError(
            'world lacks the exact Contact system or ground support collision'
        )
    return file_sha256(path)


def build_manifest(repository_root: Path) -> dict[str, Any]:
    """Derive the complete manifest from current rendered/source evidence."""
    root = repository_root.expanduser().resolve(strict=True)
    rendered_sdf = render_robot_sdf(root)
    geometries, contact_configuration_hash = extract_rendered_coverage(rendered_sdf)
    names = [item['name'] for item in geometries]
    by_role = {item['role']: item['name'] for item in geometries}
    manifest: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'robot_model': ROBOT_MODEL,
        'contact_topic': CONTACT_TOPIC,
        'robot_collisions': geometries,
        'covered_collisions': list(names),
        'rendered_robot_collisions': list(names),
        'support_pairs': [
            {
                'robot_collision': by_role[role],
                'environment_collision': GROUND_COLLISION,
            }
            for role in SUPPORT_ROLES
        ],
        'bridge_sha256': bridge_sha256(root),
        'contact_configuration_sha256': contact_configuration_hash,
        'rendered_sdf_sha256': hashlib.sha256(rendered_sdf).hexdigest(),
        'robot_description_sha256': robot_description_sha256(root),
        'world_source_sha256': world_source_sha256(root),
    }
    manifest['manifest_sha256'] = canonical_sha256(manifest)
    return manifest


def manifest_yaml_bytes(manifest: Mapping[str, Any]) -> bytes:
    """Encode the human-readable source form deterministically."""
    generated = yaml.safe_dump(
        dict(manifest),
        allow_unicode=True,
        default_flow_style=False,
        sort_keys=False,
        width=1000,
    )
    text = (
        '# Generated by robotest_description/generate_collision_coverage.py.\n'
        '# Regenerate from the repository root; do not edit hashes by hand.\n' + generated
    )
    return text.encode('utf-8')


def _write_atomic(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f'.{path.name}.', dir=path.parent)
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'wb') as output:
            output.write(payload)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repository-root', type=Path, default=Path.cwd())
    parser.add_argument('--output', type=Path)
    parser.add_argument('--mode', choices=('check', 'print', 'write'), default='check')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Check, print, or atomically regenerate the canonical manifest."""
    arguments = _parser().parse_args(argv)
    try:
        root = arguments.repository_root.expanduser().resolve(strict=True)
        output = arguments.output or root / 'config' / 'collision-coverage.yaml'
        manifest = build_manifest(root)
        payload = manifest_yaml_bytes(manifest)
        if arguments.mode == 'check':
            try:
                existing = output.read_bytes()
            except OSError as exc:
                raise CoverageGenerationError(f'cannot read manifest: {exc}') from exc
            if existing != payload:
                raise CoverageGenerationError(
                    'collision coverage is stale; regenerate with --mode write'
                )
        elif arguments.mode == 'write':
            _write_atomic(output, payload)
        else:
            sys.stdout.buffer.write(payload)
        summary = {
            'manifest_sha256': manifest['manifest_sha256'],
            'mode': arguments.mode,
            'output': str(output),
            'rendered_sdf_sha256': manifest['rendered_sdf_sha256'],
            'status': 'PASS',
        }
        stream = sys.stderr if arguments.mode == 'print' else sys.stdout
        print(json.dumps(summary, sort_keys=True), file=stream)
        return 0
    except CoverageGenerationError as exc:
        print(f'collision coverage error: {exc}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
