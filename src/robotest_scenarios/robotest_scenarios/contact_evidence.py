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
from collections.abc import Mapping, Sequence
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
FROZEN_PRIVATE_CONTACT_TOPIC = '/robotest/internal/raw_contacts'
EXPECTED_CONTACT_GATE_SOURCE_PATHS = (
    'src/robotest_sim/CMakeLists.txt',
    'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
    'src/robotest_sim/src/contact_stream_gate.cpp',
    'src/robotest_sim/src/contact_stream_gate_node.cpp',
)
EXPECTED_CONTACT_STREAM_POLICY = {
    'active_pair_expiry_ns': 250_000_000,
    'active_pair_scope': 'support_robot_internal_and_countable_robot_external',
    'accepted_run_scope': 'bounded_pending_raw_and_public_source_liveness_required',
    'capacity_claim_scope': 'unchanged_pair_set_heartbeat_only',
    'completed_stamp_batching': 'finalize_on_strictly_greater_raw_stamp',
    'delivery_semantics': 'authoritative_delivered_active_pair_snapshot',
    'emission_policy': 'immediate_active_pair_set_transition_else_heartbeat_at_or_after_200ms',
    'heartbeat_period_ns': 200_000_000,
    'initial_finalized_stamp_suppressed': True,
    'ingress_memory_bound_scope': ('post_dds_deserialization_of_trusted_sole_private_bridge_input'),
    'max_pending_batch_clock_lag_ns': 220_000_000,
    'max_public_snapshot_gap_ns': 220_000_000,
    'max_public_snapshot_clock_lag_ns': 220_000_000,
    'max_raw_clock_lag_ns': 220_000_000,
    'public_snapshots_require_nonempty_contacts': True,
    'public_snapshot_cardinality': 'required_nonempty_1_to_16',
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
        'max_raw_messages_per_completed_stamp': 7,
        'max_raw_string_bytes_per_completed_stamp': 65_536,
    },
    'release_comparison': 'completed_absent_stamp_strictly_greater_than_last_seen_plus_gap',
    'raw_contact_positions': 'required_nonempty_1_to_64',
    'raw_stamp_gap_semantics': (
        'consecutive_nonempty_raw_gaps_are_not_absence_evidence_and_have_no_independent_bound'
    ),
    'raw_messages_require_nonempty_contacts': True,
    'required_raw_frame_id': '',
    'retained_contact_payload': 'exact_nested_contact_record_copy',
    'semantic_fatal_delivery': 'best_effort_diagnostic_snapshot_before_process_failure',
    'string_budget_accounting': ('payload_strings_plus_normalized_pair_key_once_per_stored_pair'),
    'synthesized_envelope': ['container', 'current_completed_header', 'normalized_pair_order'],
    'synchronization': 'second_finalized_stamp_seeds_public_stream',
}
_SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')


@dataclass(frozen=True, slots=True)
class CoverageManifest:
    """Exact rendered robot collision set and support allowlist."""

    path: Path
    sha256: str
    public_contact_snapshot_topic: str
    private_raw_contact_topic: str
    authoritative_contact_snapshots: bool
    contact_snapshot_release_gap_ns: int
    contact_snapshot_max_gap_ns: int
    contact_snapshot_max_clock_lag_ns: int
    contact_stream_policy_sha256: str
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
    """Load and self-verify the exact revision-3 collision coverage manifest."""
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
        'contact_stream',
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
        raise ValidationError('coverage manifest root fields do not match revision 3')
    if value['schema_version'] != 3 or value['robot_model'] != 'robotest':
        raise ValidationError('coverage manifest schema/model does not match RoboTest')
    if value['contact_topic'] != FROZEN_CONTACT_TOPIC:
        raise ValidationError('coverage manifest contact topic differs from the frozen topic')
    contact_stream = value['contact_stream']
    if not isinstance(contact_stream, dict) or set(contact_stream) != {
        'gate',
        'policy',
        'policy_sha256',
        'qos',
        'schema_version',
        'topics',
    }:
        raise ValidationError('coverage contact stream fields do not match revision 1')
    if contact_stream['schema_version'] != 1:
        raise ValidationError('coverage contact stream schema is not revision 1')
    topics = contact_stream['topics']
    if topics != {
        'gazebo_raw': FROZEN_CONTACT_TOPIC,
        'private_raw_ros': FROZEN_PRIVATE_CONTACT_TOPIC,
        'public_ros': FROZEN_CONTACT_TOPIC,
    }:
        raise ValidationError('coverage contact stream topics differ from the frozen topology')
    expected_qos = {
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
    }
    if contact_stream['qos'] != expected_qos:
        raise ValidationError('coverage contact stream QoS differs from the frozen topology')
    gate = contact_stream['gate']
    if not isinstance(gate, dict) or set(gate) != {
        'executable',
        'launch_sha256',
        'package',
        'source_inventory',
        'source_inventory_sha256',
    }:
        raise ValidationError('coverage contact gate fields are invalid')
    if gate['package'] != 'robotest_sim' or gate['executable'] != 'contact_stream_gate':
        raise ValidationError('coverage contact gate executable differs from the frozen owner')
    _sha256(gate['launch_sha256'], label='contact gate launch SHA-256')
    source_inventory = gate['source_inventory']
    if (
        not isinstance(source_inventory, dict)
        or set(source_inventory) != {'schema_version', 'sources'}
        or source_inventory.get('schema_version') != 1
        or not isinstance(source_inventory.get('sources'), list)
        or len(source_inventory['sources']) != len(EXPECTED_CONTACT_GATE_SOURCE_PATHS)
    ):
        raise ValidationError('coverage contact gate source inventory is invalid')
    observed_paths: list[str] = []
    repository_root = path.parent.parent
    for index, source in enumerate(source_inventory['sources']):
        if not isinstance(source, dict) or set(source) != {'path', 'sha256'}:
            raise ValidationError('coverage contact gate source inventory entry is invalid')
        source_path = _bounded_string(source['path'], label='contact gate source path')
        observed_paths.append(source_path)
        declared_source_sha = _sha256(source['sha256'], label='contact gate source file SHA-256')
        expected_path = EXPECTED_CONTACT_GATE_SOURCE_PATHS[index]
        candidate_path = repository_root / expected_path
        if source_path != expected_path or not candidate_path.is_file():
            raise ValidationError('coverage contact gate source inventory path is unavailable')
        if hashlib.sha256(candidate_path.read_bytes()).hexdigest() != declared_source_sha:
            raise ValidationError('coverage contact gate source file SHA-256 is stale')
    if tuple(observed_paths) != EXPECTED_CONTACT_GATE_SOURCE_PATHS:
        raise ValidationError('coverage contact gate source inventory order changed')
    source_inventory_sha = _sha256(
        gate['source_inventory_sha256'], label='contact gate source SHA-256'
    )
    try:
        canonical_inventory = (
            json.dumps(
                source_inventory,
                allow_nan=False,
                ensure_ascii=False,
                separators=(',', ':'),
                sort_keys=True,
            )
            + '\n'
        ).encode('utf-8')
    except (TypeError, ValueError) as exc:
        raise ValidationError('contact gate source inventory is not canonical') from exc
    if source_inventory_sha != hashlib.sha256(canonical_inventory).hexdigest():
        raise ValidationError('coverage contact gate source inventory SHA-256 is invalid')
    policy = contact_stream['policy']
    if policy != EXPECTED_CONTACT_STREAM_POLICY:
        raise ValidationError('coverage contact stream policy differs from revision 3')
    policy_sha256 = _sha256(contact_stream['policy_sha256'], label='contact stream policy SHA-256')
    try:
        canonical_policy = (
            json.dumps(
                policy,
                allow_nan=False,
                ensure_ascii=False,
                separators=(',', ':'),
                sort_keys=True,
            )
            + '\n'
        ).encode('utf-8')
    except (TypeError, ValueError) as exc:
        raise ValidationError(f'contact stream policy is not canonical-JSON-safe: {exc}') from exc
    if policy_sha256 != hashlib.sha256(canonical_policy).hexdigest():
        raise ValidationError('contact stream policy SHA-256 does not match its content')
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
        public_contact_snapshot_topic=_bounded_string(
            topics['public_ros'], label='public contact snapshot topic'
        ),
        private_raw_contact_topic=_bounded_string(
            topics['private_raw_ros'], label='private raw contact topic'
        ),
        authoritative_contact_snapshots=True,
        contact_snapshot_release_gap_ns=EXPECTED_CONTACT_STREAM_POLICY['active_pair_expiry_ns'],
        contact_snapshot_max_gap_ns=EXPECTED_CONTACT_STREAM_POLICY['max_public_snapshot_gap_ns'],
        contact_snapshot_max_clock_lag_ns=EXPECTED_CONTACT_STREAM_POLICY[
            'max_public_snapshot_clock_lag_ns'
        ],
        contact_stream_policy_sha256=policy_sha256,
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
    if not isinstance(scoped_collision, str):
        raise ProtocolError(f'invalid scoped collision name: {scoped_collision!r}')
    segments = scoped_collision.split('::')
    if len(segments) < 3 or any(not segment for segment in segments):
        raise ProtocolError(f'invalid scoped collision name: {scoped_collision!r}')
    return segments[0]


ContactSnapshotPairTrace = tuple[
    tuple[int, tuple[tuple[str, str], ...]],
    ...,
]


def _trace_int(value: Any, *, label: str, positive: bool = False) -> int:
    if type(value) is not int or (positive and value <= 0):
        qualifier = 'positive ' if positive else ''
        raise ValidationError(f'{label} must be a {qualifier}integer')
    return value


def _trace_pair(value: Any, *, label: str, normalized: bool) -> tuple[str, str]:
    if (
        not isinstance(value, Sequence)
        or isinstance(value, (str, bytes))
        or len(value) != 2
        or any(not isinstance(item, str) or not item for item in value)
    ):
        raise ValidationError(f'{label} must contain two nonempty collision names')
    pair = (value[0], value[1])
    for collision in pair:
        top_level_model(collision)
    canonical = tuple(sorted(pair))
    if normalized and pair != canonical:
        raise ValidationError(f'{label} is not normalized')
    return canonical


def component_contact_snapshot_pair_trace(
    contact: Any,
    qualified_snapshot_stamp_ns: int,
) -> ContactSnapshotPairTrace:
    """Project all component snapshots through ``q`` to pair multisets."""
    q = _trace_int(
        qualified_snapshot_stamp_ns,
        label='qualified contact snapshot stamp',
        positive=True,
    )
    if not isinstance(contact, Mapping):
        raise ValidationError('component contact evidence must be an object')
    snapshots = contact.get('snapshots')
    records = contact.get('snapshot_records')
    if not isinstance(snapshots, list) or not snapshots:
        raise ValidationError('component contact snapshots are missing')
    if not isinstance(records, list) or not records:
        raise ValidationError('component contact snapshot records are missing')

    records_by_snapshot: dict[int, list[tuple[int, tuple[str, str]]]] = {}
    for index, record in enumerate(records):
        if not isinstance(record, Mapping):
            raise ValidationError(f'component snapshot record {index} is not an object')
        sequence = _trace_int(
            record.get('snapshot_sequence'),
            label=f'component snapshot record {index} sequence',
            positive=True,
        )
        stamp = _trace_int(
            record.get('sim_stamp_ns'),
            label=f'component snapshot record {index} stamp',
            positive=True,
        )
        pair = _trace_pair(
            record.get('normalized_pair'),
            label=f'component snapshot record {index} pair',
            normalized=True,
        )
        records_by_snapshot.setdefault(sequence, []).append((stamp, pair))

    trace: list[tuple[int, tuple[tuple[str, str], ...]]] = []
    included_sequences: set[int] = set()
    previous_sequence = 0
    previous_stamp = 0
    for index, summary in enumerate(snapshots):
        if not isinstance(summary, Mapping):
            raise ValidationError(f'component contact snapshot {index} is not an object')
        sequence = _trace_int(
            summary.get('collector_sequence'),
            label=f'component contact snapshot {index} sequence',
            positive=True,
        )
        stamp = _trace_int(
            summary.get('sim_stamp_ns'),
            label=f'component contact snapshot {index} stamp',
            positive=True,
        )
        if sequence <= previous_sequence or stamp <= previous_stamp:
            raise ValidationError('component contact snapshots are not strictly ordered')
        previous_sequence = sequence
        previous_stamp = stamp
        if stamp > q:
            continue
        snapshot_records = records_by_snapshot.get(sequence, [])
        declared_count = _trace_int(
            summary.get('snapshot_record_count'),
            label=f'component contact snapshot {index} record count',
            positive=True,
        )
        if (
            not 1 <= declared_count <= 16
            or len(snapshot_records) != declared_count
            or any(record_stamp != stamp for record_stamp, _ in snapshot_records)
        ):
            raise ValidationError('component contact snapshot records do not reconcile')
        included_sequences.add(sequence)
        trace.append((stamp, tuple(sorted(pair for _, pair in snapshot_records))))

    if not trace or trace[-1][0] != q:
        raise ValidationError('component contact trace does not end at qualified snapshot q')
    start = trace[0][0]
    for sequence, snapshot_records in records_by_snapshot.items():
        if sequence in included_sequences:
            continue
        if any(start <= stamp <= q for stamp, _ in snapshot_records):
            raise ValidationError('component contact record references a missing snapshot')
    return tuple(trace)


def captured_contact_snapshot_pair_trace(
    contact_items: Any,
    *,
    first_component_stamp_ns: int,
    qualified_snapshot_stamp_ns: int,
) -> ContactSnapshotPairTrace:
    """Project the collector's closed component-first-through-``q`` span."""
    start = _trace_int(
        first_component_stamp_ns,
        label='first component contact snapshot stamp',
        positive=True,
    )
    q = _trace_int(
        qualified_snapshot_stamp_ns,
        label='qualified contact snapshot stamp',
        positive=True,
    )
    if start > q or not isinstance(contact_items, list):
        raise ValidationError('collector contact snapshot span is invalid')
    trace: list[tuple[int, tuple[tuple[str, str], ...]]] = []
    previous_stamp = 0
    for index, item in enumerate(contact_items):
        if not isinstance(item, Mapping):
            raise ValidationError(f'collector contact snapshot {index} is not an object')
        stamp = _trace_int(
            item.get('stamp_ns'),
            label=f'collector contact snapshot {index} stamp',
            positive=True,
        )
        if not start <= stamp <= q:
            continue
        if stamp <= previous_stamp:
            raise ValidationError('collector contact snapshots are not strictly ordered')
        previous_stamp = stamp
        contacts = item.get('contacts')
        if not isinstance(contacts, list) or not 1 <= len(contacts) <= 16:
            raise ValidationError('collector contact snapshot record count is invalid')
        pairs: list[tuple[str, str]] = []
        for record_index, record in enumerate(contacts):
            if not isinstance(record, Mapping):
                raise ValidationError(
                    f'collector contact snapshot {index} record {record_index} is invalid'
                )
            pairs.append(
                _trace_pair(
                    (record.get('collision1'), record.get('collision2')),
                    label=f'collector contact snapshot {index} record {record_index} pair',
                    normalized=False,
                )
            )
        trace.append((stamp, tuple(sorted(pairs))))
    if not trace or trace[0][0] != start or trace[-1][0] != q:
        raise ValidationError('collector contact trace does not exactly span component-first to q')
    return tuple(trace)


def reconcile_contact_snapshot_pair_traces(
    contact: Any,
    contact_items: Any,
    qualified_snapshot_stamp_ns: int,
) -> ContactSnapshotPairTrace:
    """Require a complete component-to-collector snapshot-trace bijection."""
    component = component_contact_snapshot_pair_trace(
        contact,
        qualified_snapshot_stamp_ns,
    )
    captured = captured_contact_snapshot_pair_trace(
        contact_items,
        first_component_stamp_ns=component[0][0],
        qualified_snapshot_stamp_ns=qualified_snapshot_stamp_ns,
    )
    if captured != component:
        raise ValidationError('component and collector contact snapshot traces differ')
    return component


@dataclass(slots=True)
class ContactEpisodeTracker:
    """Track one counterpart from authoritative v3 active-set snapshots."""

    counterpart_model: str
    release_gap_ns: int = CONTACT_RELEASE_GAP_NS
    active_start_ns: int | None = None
    last_contact_ns: int | None = None
    last_snapshot_stamp_ns: int | None = None
    active_pairs: set[tuple[str, str]] = field(default_factory=set)
    active_snapshot_record_count: int = 0
    episodes: list[dict[str, Any]] = field(default_factory=list)

    def observe(self, stamp_ns: int, normalized_pair: tuple[str, str]) -> None:
        """Reject legacy record-wise evidence that lacks absence snapshots."""
        del stamp_ns, normalized_pair
        raise ProtocolError('legacy record-wise contact evidence is not valid for manifest v3')

    def advance(self, stamp_ns: int) -> None:
        """Reject legacy clock-gap inference without a v3 absence snapshot."""
        del stamp_ns
        raise ProtocolError('legacy clock-gap contact inference is not valid for manifest v3')

    def observe_snapshot(
        self,
        stamp_ns: int,
        normalized_pairs: list[tuple[str, str]],
    ) -> None:
        """Apply one authoritative, strictly ordered active-set snapshot."""
        if stamp_ns <= 0:
            raise ProtocolError('contact snapshot stamp must be positive')
        if self.last_snapshot_stamp_ns is not None and stamp_ns <= self.last_snapshot_stamp_ns:
            raise ProtocolError('contact snapshot stamp did not strictly advance')
        if not normalized_pairs:
            if self.active_start_ns is not None:
                self.last_snapshot_stamp_ns = stamp_ns
                self._close(stamp_ns)
            else:
                self.last_snapshot_stamp_ns = stamp_ns
            return
        self.last_snapshot_stamp_ns = stamp_ns
        if self.active_start_ns is None:
            self.active_start_ns = stamp_ns
            self.active_pairs = set()
            self.active_snapshot_record_count = 0
        self.last_contact_ns = stamp_ns
        self.active_pairs.update(normalized_pairs)
        self.active_snapshot_record_count += len(normalized_pairs)

    def _close(self, end_stamp_ns: int) -> None:
        assert self.active_start_ns is not None
        self.episodes.append(
            {
                'counterpart_model': self.counterpart_model,
                'end_stamp_ns': end_stamp_ns,
                'normalized_pairs': [list(pair) for pair in sorted(self.active_pairs)],
                'snapshot_record_count': self.active_snapshot_record_count,
                'start_stamp_ns': self.active_start_ns,
            }
        )
        self.active_start_ns = None
        self.last_contact_ns = None
        self.active_pairs = set()
        self.active_snapshot_record_count = 0
