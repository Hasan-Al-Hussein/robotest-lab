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

SCHEMA_VERSION = 3
ROBOT_MODEL = 'robotest'
GAZEBO_CONTACT_TOPIC = '/robotest/internal/contact_aggregate'
CONTACT_SENSOR_TOPIC_PREFIX = '/robotest/internal/contact_sources'
PRIVATE_RAW_CONTACT_TOPIC = '/robotest/internal/raw_contacts'
PUBLIC_CONTACT_TOPIC = '/robotest/validation/contacts'
CONTACT_TOPIC = PUBLIC_CONTACT_TOPIC
GROUND_COLLISION = 'ground_plane::ground_link::ground_collision'
DECLARED_CONTACT_UPDATE_RATE_HZ = 5.0
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
    'ros_topic_name': 'internal/raw_contacts',
    'gz_topic_name': GAZEBO_CONTACT_TOPIC,
    'ros_type_name': 'ros_gz_interfaces/msg/Contacts',
    'gz_type_name': 'gz.msgs.Contacts',
    'direction': 'GZ_TO_ROS',
    'publisher_queue': 64,
    'subscriber_queue': 64,
    'qos_profile': 'SERVICES',
}
CONTACT_GATE_SOURCE_PATHS = (
    'src/robotest_description/urdf/robotest_gazebo.xacro',
    'src/robotest_sim/CMakeLists.txt',
    'src/robotest_sim/config/bridge.yaml',
    'src/robotest_sim/include/robotest_sim/contact_aggregator.hpp',
    'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
    'src/robotest_sim/launch/sim.launch.py',
    'src/robotest_sim/src/contact_aggregator.cpp',
    'src/robotest_sim/src/contact_aggregator_system.cpp',
    'src/robotest_sim/src/contact_stream_gate.cpp',
    'src/robotest_sim/src/contact_stream_gate_node.cpp',
    'src/robotest_sim/worlds/robotest_lab.sdf',
)
CONTACT_STREAM_POLICY = {
    'active_pair_expiry_ns': 250_000_000,
    'active_pair_scope': 'support_robot_internal_and_countable_robot_external',
    'accepted_run_scope': 'bounded_complete_aggregate_and_public_source_liveness_required',
    'aggregate_interval_ns': 20_000_000,
    'aggregate_interval_pair_membership': 'union_of_every_physics_step_in_open_closed_interval',
    'aggregate_record_reduction': (
        'latest_complete_physics_step_group_per_normalized_pair_preserving_record_order'
    ),
    'capacity_claim_scope': 'unchanged_pair_set_heartbeat_only',
    'completed_stamp_batching': 'one_complete_interval_aggregate_per_stamp',
    'delivery_semantics': 'authoritative_delivered_active_pair_snapshot',
    'emission_policy': 'immediate_active_pair_set_transition_else_heartbeat_at_or_after_200ms',
    'heartbeat_period_ns': 200_000_000,
    'initial_finalized_stamp_suppressed': True,
    'ingress_memory_bound_scope': '_'.join(
        ('post_dds_deserialization_of_trusted_sole', 'private_aggregate_bridge_input')
    ),
    'max_pending_batch_clock_lag_ns': 220_000_000,
    'max_public_snapshot_gap_ns': 220_000_000,
    'max_public_snapshot_clock_lag_ns': 220_000_000,
    'max_raw_clock_lag_ns': 220_000_000,
    'passive_callback_clock_offset_semantics': 'diagnostic_noncausal',
    'public_snapshots_require_nonempty_contacts': True,
    'public_snapshot_cardinality': 'required_nonempty_1_to_16',
    'public_snapshot_clock_lag_scope': (
        'active_positive_control_and_explicit_caught_up_brackets_only'
    ),
    'public_snapshots_per_finalized_stamp': 'at_most_one',
    'limits': {
        'max_active_contact_pairs': 16,
        'max_active_contact_records': 16,
        'max_active_string_bytes': 65_536,
        'max_body_name_bytes': 4_096,
        'max_collision_name_bytes': 4_096,
        'max_contact_points_per_record': 64,
        'max_contact_records_per_pair': 4,
        'max_contact_string_bytes': 8_192,
        'max_frame_id_bytes': 256,
        'max_raw_contact_records': 16,
        'max_raw_messages_per_completed_stamp': 1,
        'max_raw_string_bytes_per_completed_stamp': 65_536,
    },
    'release_comparison': 'completed_absent_stamp_strictly_greater_than_last_seen_plus_gap',
    'raw_contact_positions': 'required_nonempty_1_to_64',
    'raw_stamp_gap_semantics': (
        'sole_source_grid_is_exact_20ms_and_gate_rejects_delivered_gaps_greater_than_20ms'
    ),
    'raw_messages_require_nonempty_contacts': True,
    'required_raw_frame_id': '',
    'retained_contact_payload': (
        'exact_ros_projected_nested_copy_of_latest_pair_group_selected_by_interval_reduction'
    ),
    'semantic_fatal_delivery': 'best_effort_diagnostic_snapshot_before_process_failure',
    'string_budget_accounting': ('payload_strings_plus_normalized_pair_key_once_per_stored_pair'),
    'synthesized_envelope': [
        'container',
        'interval_boundary_container_and_contact_headers',
        'normalized_pair_order',
    ],
    'synchronization': 'second_finalized_stamp_seeds_public_stream',
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


def extract_rendered_coverage(
    rendered_sdf: bytes,
) -> tuple[list[dict[str, str]], list[dict[str, Any]]]:
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
        expected_sensor_topic = f'{CONTACT_SENSOR_TOPIC_PREFIX}/{sensor_name}'
        update_rate_text = sensor.findtext('update_rate')
        try:
            update_rate = float(update_rate_text) if update_rate_text is not None else math.nan
        except ValueError as exc:
            raise CoverageGenerationError(f'{sensor_name} update rate is invalid') from exc
        if (
            target_collision != collision_name
            or topic != expected_sensor_topic
            or sensor.findtext('always_on') != 'true'
            or not math.isfinite(update_rate)
            or update_rate != DECLARED_CONTACT_UPDATE_RATE_HZ
        ):
            raise CoverageGenerationError(
                f'{sensor_name} does not exactly cover its collision on the frozen topic/rate'
            )
        scoped_name = f'{ROBOT_MODEL}::{link_name}::{collision_name}'
        geometries.append(
            {
                'name': scoped_name,
                'role': role,
                'source': PUBLIC_CONTACT_TOPIC,
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
                'declared_update_rate_hz': update_rate,
                'source': GAZEBO_CONTACT_TOPIC,
                'sensor_topic': expected_sensor_topic,
            }
        )
    if len({item['name'] for item in geometries}) != len(ROLE_BINDINGS):
        raise CoverageGenerationError('rendered scoped collision names are not unique')
    return geometries, contact_projection


def contact_gate_source_inventory(repository_root: Path) -> dict[str, Any]:
    """Return the canonical compiled contact-gate source/link inventory."""
    return {
        'schema_version': 1,
        'sources': [
            {
                'path': relative_path,
                'sha256': file_sha256(repository_root / relative_path),
            }
            for relative_path in CONTACT_GATE_SOURCE_PATHS
        ],
    }


def contact_gate_source_sha256(repository_root: Path) -> str:
    """Hash the complete compiled contact-gate source/link inventory."""
    return canonical_sha256(contact_gate_source_inventory(repository_root))


def contact_stream_configuration(repository_root: Path) -> dict[str, Any]:
    """Bind the private bridge, compiled gate, and public snapshot contract."""
    launch_path = repository_root / 'src' / 'robotest_sim' / 'launch' / 'sim.launch.py'
    try:
        launch_source = launch_path.read_text(encoding='utf-8')
    except (OSError, UnicodeError) as exc:
        raise CoverageGenerationError(f'cannot read contact gate launch source: {exc}') from exc
    required_launch_tokens = (
        "package='robotest_sim'",
        "executable='contact_stream_gate'",
        'namespace=namespace',
        "name='contact_stream_gate'",
        "on_exit=Shutdown(reason='contact stream gate exited')",
        "on_exit=Shutdown(reason='parameter bridge exited')",
    )
    if any(token not in launch_source for token in required_launch_tokens):
        raise CoverageGenerationError(
            ' '.join(('contact gate launch wiring differs from the', 'frozen topology'))
        )
    policy = dict(CONTACT_STREAM_POLICY)
    source_inventory = contact_gate_source_inventory(repository_root)
    return {
        'schema_version': 1,
        'topics': {
            'gazebo_raw': GAZEBO_CONTACT_TOPIC,
            'private_raw_ros': PRIVATE_RAW_CONTACT_TOPIC,
            'public_ros': PUBLIC_CONTACT_TOPIC,
        },
        'qos': {
            'private_raw_ros': {
                'depth': 64,
                'durability': 'VOLATILE',
                'history': 'KEEP_LAST',
                'reliability': 'RELIABLE',
            },
            'public_ros': {
                'depth': 10,
                'durability': 'VOLATILE',
                'history': 'KEEP_LAST',
                'reliability': 'RELIABLE',
            },
        },
        'gate': {
            'executable': 'contact_stream_gate',
            'launch_sha256': file_sha256(launch_path),
            'package': 'robotest_sim',
            'source_inventory': source_inventory,
            'source_inventory_sha256': canonical_sha256(source_inventory),
        },
        'policy': policy,
        'policy_sha256': canonical_sha256(policy),
    }


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
            or entry.get('ros_topic_name') == 'validation/contacts'
            or entry.get('gz_topic_name') == GAZEBO_CONTACT_TOPIC
        )
    ]
    if len(contact_entries) != 1 or dict(contact_entries[0]) != BRIDGE_CONTACT_ENTRY:
        raise CoverageGenerationError('contact bridge entry differs from the frozen mapping')
    return file_sha256(path)


def world_source_sha256(repository_root: Path) -> str:
    """Validate the aggregate system and exact ground collision, then hash the world."""
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
        if plugin.get('filename') == 'robotest_contact_aggregator_system'
        and plugin.get('name') == 'robotest_sim::ContactAggregatorSystem'
    ]
    stock_contact_plugins = [
        plugin
        for plugin in world.findall('plugin')
        if plugin.get('filename') == 'gz-sim-contact-system'
        or plugin.get('name') == 'gz::sim::systems::Contact'
    ]
    ground = world.find(
        "./model[@name='ground_plane']/link[@name='ground_link']/"
        "collision[@name='ground_collision']"
    )
    plugin = contact_plugins[0] if len(contact_plugins) == 1 else None
    plugin_contract_matches = plugin is not None and (
        plugin.findtext('output_topic') == GAZEBO_CONTACT_TOPIC
        and plugin.findtext('publish_period_ns') == '20000000'
        and plugin.findtext('robot_model_name') == ROBOT_MODEL
    )
    if stock_contact_plugins or not plugin_contract_matches or ground is None:
        raise CoverageGenerationError(
            'world lacks the exact contact aggregate system or ground support collision'
        )
    return file_sha256(path)


def build_manifest(repository_root: Path) -> dict[str, Any]:
    """Derive the complete manifest from current rendered/source evidence."""
    root = repository_root.expanduser().resolve(strict=True)
    rendered_sdf = render_robot_sdf(root)
    geometries, sensor_projection = extract_rendered_coverage(rendered_sdf)
    contact_stream = contact_stream_configuration(root)
    contact_configuration = {
        'contact_stream': contact_stream,
        'declared_sensor_update_rate_authoritative': False,
        'gazebo_contact_topic': GAZEBO_CONTACT_TOPIC,
        'robot_model': ROBOT_MODEL,
        'schema_version': 2,
        'sensors': sensor_projection,
    }
    names = [item['name'] for item in geometries]
    by_role = {item['role']: item['name'] for item in geometries}
    manifest: dict[str, Any] = {
        'schema_version': SCHEMA_VERSION,
        'robot_model': ROBOT_MODEL,
        'contact_topic': PUBLIC_CONTACT_TOPIC,
        'contact_stream': contact_stream,
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
        'contact_configuration_sha256': canonical_sha256(contact_configuration),
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
