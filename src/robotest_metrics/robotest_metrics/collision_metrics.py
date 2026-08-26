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

"""Collision coverage qualification and deterministic contact de-duplication."""

from __future__ import annotations

import math
import re
from collections import Counter
from collections.abc import Mapping, Sequence
from itertools import pairwise
from pathlib import Path
from typing import Any

from robotest_scenarios.constants import (
    ACTOR_CLEANUP_QUIET_NS,
    ACTOR_INITIAL_POSITION_TOLERANCE_M,
    ACTOR_YAW_TOLERANCE_RAD,
    CONTROL_ROBOT_START,
    CONTROL_WALL_POSE,
)
from robotest_scenarios.geometry import shortest_yaw_error
from robotest_scenarios.provenance import (
    contact_control_configuration,
    contact_control_configuration_sha256,
    contact_source_binding,
    file_sha256,
)

from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.constants import CONTACT_RECORD_CAPACITY, CONTACT_RELEASE_GAP_NS
from robotest_metrics.errors import ArtifactError, MetricUnavailable
from robotest_metrics.geometry import require_finite, require_int

_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_GROUND_COLLISION = 'ground_plane::ground_link::ground_collision'
_SUPPORT_ROLES = {'front_caster', 'left_wheel', 'rear_caster', 'right_wheel'}
_PROVENANCE_HASHES = (
    'bridge_sha256',
    'collector_configuration_sha256',
    'contact_configuration_sha256',
    'coverage_manifest_sha256',
    'rendered_sdf_sha256',
    'robot_description_sha256',
    'world_source_sha256',
)
_POSITIVE_CONTROL_CRITERIA = (
    'contact_before_deadline',
    'episode_reconciled',
    'exactly_one_counterpart_episode',
    'final_command_zero',
    'graph_isolated',
    'hold_completed',
    'overflow_free',
    'raw_expected_contact_observed',
    'release_completed',
    'release_source_spanned',
    'reverse_completed',
    'robot_start_verified',
    'sole_cmd_vel_publisher',
    'stop_within_100ms',
    'wall_deleted',
)
_POSITIVE_BUFFER_CAPACITIES = {
    'actor_state': 1_024,
    'command': 4_096,
    'contact_records': CONTACT_RECORD_CAPACITY,
    'contact_summaries': 8_192,
    'ground_truth': 8_192,
    'raw_contact_stream': CONTACT_RECORD_CAPACITY,
}
_CONTROL_CONTACT_DEADLINE_NS = 12_000_000_000
_CONTROL_HOLD_NS = 250_000_000
_CONTROL_REVERSE_NS = 1_000_000_000
_CONTROL_STOP_DEADLINE_NS = 100_000_000
_FIXTURE_ID = 'collision_positive_control'
_WALL_NAME = 'phase3_contact_control_wall'
_CONFIGURATION_FIELDS = {
    'control_configuration',
    'control_configuration_sha256',
    'coverage_manifest_path',
    'coverage_manifest_provenance',
    'coverage_manifest_sha256',
    'expected_pair',
    'fixture',
    'fixture_sha256',
    'service_timeout_s',
    'source_binding',
    'wall_asset_sha256',
    'wall_timeout_s',
}
_METRICS_OWNED = {
    'collector_reconciliation_and_process_group_termination': {
        'owner': 'robotest_metrics_and_orchestrator',
        'required': True,
        'scenario_controller_value': None,
    }
}
_CONTACT_RECORD_FIELDS = {
    'collector_sequence',
    'counterpart_collision',
    'counterpart_model',
    'disposition',
    'normalized_pair',
    'robot_collision',
    'sim_stamp_ns',
}
_CONTACT_DISPOSITIONS = {
    'allowlisted_support_contact': 'support_ground_excluded',
    'robot_internal': 'robot_internal_excluded',
    'unrelated_environment_contact': 'non_robot_pair_ignored',
}


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise MetricUnavailable(f'{name} must be a lowercase SHA-256')
    return value


def _require_bounded_buffer(
    value: Any,
    name: str,
    *,
    retain_every_accepted_item: bool,
) -> dict[str, int]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable(f'positive-control buffer {name} is invalid')
    accepted = require_int(value.get('accepted_count'), f'positive_control.{name}.accepted')
    capacity = require_int(value.get('capacity'), f'positive_control.{name}.capacity')
    ingress = require_int(value.get('ingress_count'), f'positive_control.{name}.ingress')
    retained = require_int(value.get('retained_count'), f'positive_control.{name}.retained')
    invalid = require_int(value.get('invalid_count'), f'positive_control.{name}.invalid')
    overflow = require_int(value.get('overflow_count'), f'positive_control.{name}.overflow')
    expected_capacity = _POSITIVE_BUFFER_CAPACITIES[name]
    if (
        capacity != expected_capacity
        or min(accepted, ingress, invalid, overflow, retained) < 0
        or ingress != accepted + invalid + overflow
        or retained > accepted
        or retained > capacity
        or (retain_every_accepted_item and retained != accepted)
        or value.get('overflow') is not False
        or overflow != 0
        or invalid != 0
        or value.get('first_overflow_sequence') is not None
        or value.get('first_overflow_stamp_ns') is not None
    ):
        raise MetricUnavailable(f'positive-control buffer {name} is incomplete')
    return {
        'accepted_count': accepted,
        'capacity': capacity,
        'ingress_count': ingress,
        'retained_count': retained,
    }


def _reconcile_contact_records(
    records: Any,
    manifest: Mapping[str, Any],
    expected: tuple[str, str],
) -> tuple[list[Mapping[str, Any]], list[Mapping[str, Any]], Mapping[str, Any]]:
    if not isinstance(records, list) or not records:
        raise MetricUnavailable('positive-control retained contact records are missing')
    counted: list[Mapping[str, Any]] = []
    exact: list[Mapping[str, Any]] = []
    previous_sequence = 0
    previous_stamp = 0
    for index, value in enumerate(records):
        if not isinstance(value, Mapping) or set(value) != _CONTACT_RECORD_FIELDS:
            raise MetricUnavailable(f'positive-control contact.records[{index}] is invalid')
        sequence = require_int(
            value.get('collector_sequence'),
            f'positive_control.contact.records[{index}].collector_sequence',
        )
        stamp = require_int(
            value.get('sim_stamp_ns'),
            f'positive_control.contact.records[{index}].sim_stamp_ns',
        )
        pair = value.get('normalized_pair')
        if (
            sequence <= previous_sequence
            or stamp <= 0
            or stamp < previous_stamp
            or not isinstance(pair, list)
            or len(pair) != 2
            or any(not isinstance(item, str) or not item for item in pair)
            or pair != sorted(pair)
        ):
            raise MetricUnavailable('positive-control contact record ordering is invalid')
        previous_sequence = sequence
        previous_stamp = stamp
        classification = classify_contact_pair(pair[0], pair[1], manifest)
        if classification.get('counted') is True:
            if (
                value.get('disposition') != 'counted'
                or value.get('robot_collision') != classification.get('robot_collision')
                or value.get('counterpart_collision') != classification.get('counterpart_collision')
                or value.get('counterpart_model') != classification.get('counterpart_model')
            ):
                raise MetricUnavailable('positive-control counted contact classification changed')
            counted.append(value)
            if tuple(pair) == expected:
                exact.append(value)
        else:
            if value.get('disposition') != _CONTACT_DISPOSITIONS.get(
                classification.get('reason')
            ) or any(
                value.get(field) is not None
                for field in ('robot_collision', 'counterpart_collision', 'counterpart_model')
            ):
                raise MetricUnavailable('positive-control excluded contact classification changed')
    if not counted or not exact:
        raise MetricUnavailable('positive-control retained expected contact is missing')
    return counted, exact, exact[0]


def _validate_command_trace(
    commands: Any,
    *,
    control_started_stamp: int,
    first_contact_sequence: int,
    stop_command_stamp: int,
    reverse_start_stamp: int,
    final_zero_stamp: int,
) -> None:
    if not isinstance(commands, list) or len(commands) < 4:
        raise MetricUnavailable('positive-control command trace is incomplete')
    expected_values = {
        'CONTACT_STOP': 0.0,
        'FINAL_ZERO': 0.0,
        'FORWARD': 0.05,
        'HOLD': 0.0,
        'REVERSE': -0.05,
    }
    phases: list[str] = []
    previous_order: tuple[int, int] | None = None
    previous_sequence: int | None = None
    for index, command in enumerate(commands):
        if not isinstance(command, Mapping) or set(command) != {
            'angular_z',
            'collector_sequence',
            'linear_x',
            'phase',
            'sim_stamp_ns',
        }:
            raise MetricUnavailable(f'positive-control command_trace[{index}] is invalid')
        stamp = require_int(
            command.get('sim_stamp_ns'),
            f'positive_control.command_trace[{index}].sim_stamp_ns',
        )
        sequence = require_int(
            command.get('collector_sequence'),
            f'positive_control.command_trace[{index}].collector_sequence',
        )
        order = (stamp, sequence)
        if (
            stamp <= 0
            or (previous_order is not None and order <= previous_order)
            or (previous_sequence is not None and sequence <= previous_sequence)
        ):
            raise MetricUnavailable('positive-control command trace is not ordered')
        previous_order = order
        previous_sequence = sequence
        phase = command.get('phase')
        if (
            phase not in expected_values
            or require_finite(
                command.get('linear_x'), f'positive_control.command_trace[{index}].linear_x'
            )
            != expected_values[phase]
            or require_finite(
                command.get('angular_z'), f'positive_control.command_trace[{index}].angular_z'
            )
            != 0.0
        ):
            raise MetricUnavailable('positive-control command phase/value contract changed')
        phases.append(phase)
    phase_rank = {'FORWARD': 0, 'CONTACT_STOP': 1, 'HOLD': 2, 'REVERSE': 3, 'FINAL_ZERO': 4}
    hold_commands = [command for command in commands if command['phase'] == 'HOLD']
    stop_command = next(command for command in commands if command['phase'] == 'CONTACT_STOP')
    if (
        [phase_rank[phase] for phase in phases] != sorted(phase_rank[phase] for phase in phases)
        or phases[0] != 'FORWARD'
        or phases[-1] != 'FINAL_ZERO'
        or phases.count('CONTACT_STOP') != 1
        or phases.count('FINAL_ZERO') != 1
        or not hold_commands
        or 'REVERSE' not in phases
        or commands[0]['sim_stamp_ns'] != control_started_stamp
        or stop_command['sim_stamp_ns'] != stop_command_stamp
        or stop_command['collector_sequence'] != first_contact_sequence + 1
        or next(item['sim_stamp_ns'] for item in commands if item['phase'] == 'REVERSE')
        != reverse_start_stamp
        or any(
            not stop_command_stamp < command['sim_stamp_ns'] < reverse_start_stamp
            for command in hold_commands
        )
        or commands[-1]['sim_stamp_ns'] != final_zero_stamp
    ):
        raise MetricUnavailable('positive-control command anchors do not reconcile')


def _collision_names(manifest: Mapping[str, Any]) -> tuple[str, set[str], set[tuple[str, str]]]:
    robot_model = manifest.get('robot_model')
    if not isinstance(robot_model, str) or not robot_model:
        raise MetricUnavailable('coverage manifest robot_model is missing')
    entries = manifest.get('robot_collisions')
    if not isinstance(entries, list) or not entries:
        raise MetricUnavailable('coverage manifest has no robot collisions')
    names: set[str] = set()
    roles: dict[str, str] = {}
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping):
            raise MetricUnavailable(f'robot_collisions[{index}] must be an object')
        name = entry.get('name')
        role = entry.get('role')
        source = entry.get('source')
        if not all(isinstance(value, str) and value for value in (name, role, source)):
            raise MetricUnavailable(f'robot_collisions[{index}] is incomplete')
        if name in names:
            raise MetricUnavailable(f'duplicate robot collision {name}')
        names.add(name)
        roles[name] = role
    covered = manifest.get('covered_collisions')
    if not isinstance(covered, list) or set(covered) != names or len(covered) != len(names):
        raise MetricUnavailable('contact-source coverage does not equal robot collision set')
    rendered = manifest.get('rendered_robot_collisions')
    if not isinstance(rendered, list) or set(rendered) != names or len(rendered) != len(names):
        raise MetricUnavailable('rendered-SDF collision set does not equal coverage manifest')
    support_pairs_raw = manifest.get('support_pairs', [])
    if not isinstance(support_pairs_raw, list):
        raise MetricUnavailable('support_pairs must be a list')
    support_pairs: set[tuple[str, str]] = set()
    for index, pair in enumerate(support_pairs_raw):
        if not isinstance(pair, Mapping):
            raise MetricUnavailable(f'support_pairs[{index}] must be an object')
        robot_collision = pair.get('robot_collision')
        environment_collision = pair.get('environment_collision')
        if robot_collision not in names or not isinstance(environment_collision, str):
            raise MetricUnavailable(f'support_pairs[{index}] is invalid')
        if roles[robot_collision] not in _SUPPORT_ROLES:
            raise MetricUnavailable('only exact wheel/caster roles may be support-allowlisted')
        if environment_collision != _GROUND_COLLISION:
            raise MetricUnavailable('support allowlist counterpart must be exact ground collision')
        normalized_pair = (robot_collision, environment_collision)
        if normalized_pair in support_pairs:
            raise MetricUnavailable(f'duplicate support_pairs[{index}]')
        support_pairs.add(normalized_pair)
    for field in (
        'bridge_sha256',
        'contact_configuration_sha256',
        'manifest_sha256',
        'robot_description_sha256',
        'rendered_sdf_sha256',
        'world_source_sha256',
    ):
        _sha256(manifest.get(field), f'coverage.{field}')
    manifest_body = dict(manifest)
    declared_manifest_hash = manifest_body.pop('manifest_sha256')
    try:
        calculated_manifest_hash = canonical_sha256(manifest_body)
    except ArtifactError as exc:
        raise MetricUnavailable('coverage manifest is not canonicalizable') from exc
    if calculated_manifest_hash != declared_manifest_hash:
        raise MetricUnavailable('coverage manifest canonical hash mismatch')
    return robot_model, names, support_pairs


def _positive_control_evidence(
    positive_control: Mapping[str, Any],
    manifest: Mapping[str, Any],
    expected_wall_asset_sha256: str,
) -> dict[str, Any]:
    if positive_control.get('schema_version') != 1 or positive_control.get('producer') != (
        'robotest_scenarios/contact_control_driver'
    ):
        raise MetricUnavailable('positive-control producer/schema is invalid')
    if positive_control.get('status') != 'PASS':
        raise MetricUnavailable('positive-control status is not PASS')
    verdict = positive_control.get('verdict')
    if not isinstance(verdict, Mapping) or (
        verdict.get('authority') != 'component_only'
        or verdict.get('benchmark_pass') is not None
        or verdict.get('exit_code') != 0
    ):
        raise MetricUnavailable('positive-control component authority is invalid')
    identity = positive_control.get('identity')
    if (
        not isinstance(identity, Mapping)
        or not isinstance(identity.get('run_id'), str)
        or not identity.get('run_id')
        or identity.get('fixture_id') != 'collision_positive_control'
    ):
        raise MetricUnavailable('positive-control run ID is missing')
    scenario_hash = _sha256(
        identity.get('scenario_sha256'),
        'positive_control.identity.scenario_sha256',
    )
    configuration = positive_control.get('configuration')
    if not isinstance(configuration, Mapping) or set(configuration) != _CONFIGURATION_FIELDS:
        raise MetricUnavailable('positive-control configuration is missing')
    for field in (
        'control_configuration_sha256',
        'coverage_manifest_sha256',
        'fixture_sha256',
        'wall_asset_sha256',
    ):
        _sha256(configuration.get(field), f'positive_control.configuration.{field}')
    if identity.get('scenario_sha256') != configuration.get('fixture_sha256'):
        raise MetricUnavailable('positive-control fixture/scenario hash mismatch')
    if configuration.get('wall_asset_sha256') != expected_wall_asset_sha256:
        raise MetricUnavailable('positive-control wall asset hash mismatch')
    expected_control_configuration = contact_control_configuration()
    declared_control_hash = _sha256(
        configuration.get('control_configuration_sha256'),
        'positive_control.configuration.control_configuration_sha256',
    )
    embedded_control_configuration = configuration.get('control_configuration')
    try:
        embedded_control_hash = canonical_sha256(embedded_control_configuration)
        expected_control_hash = canonical_sha256(expected_control_configuration)
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control control configuration is not canonical') from exc
    if (
        not isinstance(embedded_control_configuration, Mapping)
        or declared_control_hash != contact_control_configuration_sha256()
        or embedded_control_hash != expected_control_hash
        or embedded_control_hash != declared_control_hash
    ):
        raise MetricUnavailable('positive-control driver control configuration changed')
    coverage_provenance = configuration.get('coverage_manifest_provenance')
    expected_coverage_provenance = {
        field: manifest.get(field)
        for field in (
            'bridge_sha256',
            'contact_configuration_sha256',
            'rendered_sdf_sha256',
            'robot_description_sha256',
            'world_source_sha256',
        )
    } | {'coverage_manifest_sha256': manifest.get('manifest_sha256')}
    if not isinstance(coverage_provenance, Mapping) or set(coverage_provenance) != set(
        expected_coverage_provenance
    ):
        raise MetricUnavailable('positive-control coverage provenance is incomplete')
    for field, expected in expected_coverage_provenance.items():
        if (
            _sha256(
                coverage_provenance.get(field),
                f'positive_control.configuration.coverage_manifest_provenance.{field}',
            )
            != expected
        ):
            raise MetricUnavailable(f'positive-control coverage provenance mismatch for {field}')
    source_binding = configuration.get('source_binding')
    expected_source_binding = contact_source_binding()
    try:
        source_binding_matches = canonical_sha256(source_binding) == canonical_sha256(
            expected_source_binding
        )
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control source binding is not canonical') from exc
    if not isinstance(source_binding, Mapping) or not source_binding_matches:
        raise MetricUnavailable('positive-control source binding is missing')
    driver_source_sha = _sha256(
        source_binding.get('aggregate_sha256'),
        'positive_control.configuration.source_binding.aggregate_sha256',
    )
    if configuration.get('coverage_manifest_sha256') != manifest.get('manifest_sha256'):
        raise MetricUnavailable('positive-control coverage manifest hash mismatch')
    control = positive_control.get('control')
    if not isinstance(control, Mapping):
        raise MetricUnavailable('positive-control control evidence is missing')
    criteria = control.get('criteria')
    if not isinstance(criteria, Mapping) or set(criteria) != set(_POSITIVE_CONTROL_CRITERIA):
        raise MetricUnavailable('positive-control criteria fields are incomplete')
    if any(criteria.get(field) is not True for field in _POSITIVE_CONTROL_CRITERIA):
        raise MetricUnavailable('positive-control frozen criteria did not all pass')
    try:
        metrics_owned_matches = canonical_sha256(control.get('metrics_owned')) == canonical_sha256(
            _METRICS_OWNED
        )
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control metrics ownership is not canonical') from exc
    if not metrics_owned_matches:
        raise MetricUnavailable('positive-control metrics ownership contract changed')
    observed_start = control.get('observed_robot_start')
    if not isinstance(observed_start, Mapping):
        raise MetricUnavailable('positive-control observed robot start is missing')
    start_x = require_finite(observed_start.get('x'), 'positive_control.robot_start.x')
    start_y = require_finite(observed_start.get('y'), 'positive_control.robot_start.y')
    start_yaw = require_finite(observed_start.get('yaw'), 'positive_control.robot_start.yaw')
    start_alignment = require_int(
        observed_start.get('alignment_error_ns'),
        'positive_control.robot_start.alignment_error_ns',
    )
    start_sequence = require_int(
        observed_start.get('collector_sequence'),
        'positive_control.robot_start.collector_sequence',
    )
    start_stamp = require_int(
        observed_start.get('sim_stamp_ns'), 'positive_control.robot_start.sim_stamp_ns'
    )
    position_error = require_finite(
        observed_start.get('position_error_m'),
        'positive_control.robot_start.position_error_m',
    )
    yaw_error = require_finite(
        observed_start.get('yaw_error_rad'),
        'positive_control.robot_start.yaw_error_rad',
    )
    expected_start_position_error = math.hypot(
        start_x - CONTROL_ROBOT_START[0],
        start_y - CONTROL_ROBOT_START[1],
    )
    expected_start_yaw_error = shortest_yaw_error(start_yaw, CONTROL_ROBOT_START[2])
    if (
        position_error != expected_start_position_error
        or yaw_error != expected_start_yaw_error
        or position_error > ACTOR_INITIAL_POSITION_TOLERANCE_M
        or yaw_error > ACTOR_YAW_TOLERANCE_RAD
        or not 0 <= start_alignment <= 250_000_000
    ):
        raise MetricUnavailable('positive-control robot start proof exceeds tolerance')
    setup = control.get('setup')
    if not isinstance(setup, Mapping):
        raise MetricUnavailable('positive-control setup evidence is missing')
    setup_start = setup.get('observed_robot_start')
    observed_wall = setup.get('observed_wall')
    spawn = setup.get('spawn')
    start_fields = {
        'alignment_error_ns',
        'collector_sequence',
        'position_error_m',
        'sim_stamp_ns',
        'x',
        'y',
        'yaw',
        'yaw_error_rad',
    }
    wall_fields = {
        'collector_sequence',
        'position_error_m',
        'stamp_ns',
        'x',
        'y',
        'yaw',
        'yaw_error_rad',
        'z',
    }
    spawn_fields = {
        'attempt_count',
        'error',
        'request_sequence',
        'request_stamp_ns',
        'response_sequence',
        'response_stamp_ns',
        'success',
    }
    try:
        setup_start_matches = canonical_sha256(setup_start) == canonical_sha256(observed_start)
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control setup evidence is not canonical') from exc
    if (
        set(setup) != {'observed_robot_start', 'observed_wall', 'spawn'}
        or not isinstance(setup_start, Mapping)
        or set(setup_start) != start_fields
        or not setup_start_matches
        or not isinstance(observed_wall, Mapping)
        or set(observed_wall) != wall_fields
        or not isinstance(spawn, Mapping)
        or set(spawn) != spawn_fields
        or spawn.get('success') is not True
        or spawn.get('error') is not None
    ):
        raise MetricUnavailable('positive-control successful setup proof is incomplete')
    request_sequence = require_int(
        spawn.get('request_sequence'), 'positive_control.spawn.request_sequence'
    )
    response_sequence = require_int(
        spawn.get('response_sequence'), 'positive_control.spawn.response_sequence'
    )
    request_stamp = require_int(
        spawn.get('request_stamp_ns'), 'positive_control.spawn.request_stamp_ns'
    )
    response_stamp = require_int(
        spawn.get('response_stamp_ns'), 'positive_control.spawn.response_stamp_ns'
    )
    wall_sequence = require_int(
        observed_wall.get('collector_sequence'),
        'positive_control.observed_wall.collector_sequence',
    )
    wall_stamp = require_int(
        observed_wall.get('stamp_ns'), 'positive_control.observed_wall.stamp_ns'
    )
    wall_x = require_finite(observed_wall.get('x'), 'positive_control.observed_wall.x')
    wall_y = require_finite(observed_wall.get('y'), 'positive_control.observed_wall.y')
    wall_z = require_finite(observed_wall.get('z'), 'positive_control.observed_wall.z')
    wall_yaw = require_finite(observed_wall.get('yaw'), 'positive_control.observed_wall.yaw')
    wall_position_error = require_finite(
        observed_wall.get('position_error_m'),
        'positive_control.observed_wall.position_error_m',
    )
    wall_yaw_error = require_finite(
        observed_wall.get('yaw_error_rad'),
        'positive_control.observed_wall.yaw_error_rad',
    )
    expected_wall_position_error = math.sqrt(
        (wall_x - CONTROL_WALL_POSE[0]) ** 2
        + (wall_y - CONTROL_WALL_POSE[1]) ** 2
        + (wall_z - CONTROL_WALL_POSE[2]) ** 2
    )
    expected_wall_yaw_error = shortest_yaw_error(wall_yaw, CONTROL_WALL_POSE[3])
    if (
        require_int(spawn.get('attempt_count'), 'positive_control.spawn.attempt_count') != 1
        or request_sequence < 1
        or response_sequence <= request_sequence
        or request_stamp < 0
        or response_stamp < request_stamp
        or wall_sequence <= request_sequence
        or wall_stamp < request_stamp
        or wall_position_error != expected_wall_position_error
        or wall_yaw_error != expected_wall_yaw_error
        or not 0.0 <= wall_position_error <= ACTOR_INITIAL_POSITION_TOLERANCE_M
        or not 0.0 <= wall_yaw_error <= ACTOR_YAW_TOLERANCE_RAD
    ):
        raise MetricUnavailable('positive-control successful setup proof is incomplete')
    contact = control.get('contact')
    if not isinstance(contact, Mapping):
        raise MetricUnavailable('positive-control contact evidence is missing')
    expected_pair = contact.get('expected_pair')
    if not isinstance(expected_pair, list) or len(expected_pair) != 2:
        raise MetricUnavailable('positive-control expected pair is invalid')
    if any(not isinstance(value, str) or not value for value in expected_pair):
        raise MetricUnavailable('positive-control expected pair is invalid')
    expected = tuple(sorted(expected_pair))
    configuration_pair = configuration.get('expected_pair')
    if not isinstance(configuration_pair, list) or configuration_pair != list(expected):
        raise MetricUnavailable('positive-control configured and observed pairs differ')
    expected_fixture = {
        'control': expected_control_configuration,
        'entity': {
            'asset_sha256': configuration['wall_asset_sha256'],
            'name': _WALL_NAME,
            'pose': {
                'x': CONTROL_WALL_POSE[0],
                'y': CONTROL_WALL_POSE[1],
                'yaw': CONTROL_WALL_POSE[3],
                'z': CONTROL_WALL_POSE[2],
            },
        },
        'expected_pair': list(expected),
        'fixture_id': _FIXTURE_ID,
        'robot_start': {
            'x': CONTROL_ROBOT_START[0],
            'y': CONTROL_ROBOT_START[1],
            'yaw': CONTROL_ROBOT_START[2],
        },
        'schema_version': 1,
    }
    try:
        expected_fixture_hash = canonical_sha256(expected_fixture)
        fixture_matches = canonical_sha256(configuration.get('fixture')) == expected_fixture_hash
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control fixture is not canonical') from exc
    if (
        not fixture_matches
        or configuration.get('fixture_sha256') != expected_fixture_hash
        or scenario_hash != expected_fixture_hash
    ):
        raise MetricUnavailable('positive-control fixture binding changed')
    classification = classify_contact_pair(expected[0], expected[1], manifest)
    if classification.get('counted') is not True:
        raise MetricUnavailable('positive-control expected pair is excluded by classification')
    counted_records, exact_records, first_exact_record = _reconcile_contact_records(
        contact.get('records'), manifest, expected
    )
    exact_count = require_int(
        contact.get('exact_pair_raw_count'),
        'positive_control.exact_pair_raw_count',
    )
    raw_count = require_int(
        contact.get('raw_contact_record_count'),
        'positive_control.raw_contact_record_count',
    )
    classified_count = require_int(
        contact.get('classified_record_count'),
        'positive_control.classified_record_count',
    )
    if (
        raw_count != len(contact['records'])
        or classified_count != len(counted_records)
        or exact_count != len(exact_records)
        or not 0 < exact_count <= classified_count <= raw_count
    ):
        raise MetricUnavailable('positive-control contact record counters do not reconcile')
    counterpart_models = {record['counterpart_model'] for record in counted_records}
    if (
        require_int(
            contact.get('active_counterpart_count'),
            'positive_control.active_counterpart_count',
        )
        != 0
        or require_int(
            contact.get('counterpart_tracker_count'),
            'positive_control.counterpart_tracker_count',
        )
        != len(counterpart_models)
        or counterpart_models != {classification['counterpart_model']}
    ):
        raise MetricUnavailable('positive-control counterpart tracker did not close exactly once')
    first_contact = contact.get('first_qualifying_contact')
    first_observed_stamp = (
        first_contact.get('observed_sim_stamp_ns') if isinstance(first_contact, Mapping) else None
    )
    if (
        not isinstance(first_contact, Mapping)
        or set(first_contact)
        != {
            'clock_delivery_offset_ns',
            'collector_sequence',
            'normalized_pair',
            'observed_sim_stamp_ns',
            'sim_stamp_ns',
        }
        or first_contact.get('collector_sequence') != first_exact_record['collector_sequence']
        or first_contact.get('normalized_pair') != list(expected)
        or first_contact.get('sim_stamp_ns') != first_exact_record['sim_stamp_ns']
        or require_int(
            first_observed_stamp,
            'positive_control.first_qualifying_contact.observed_sim_stamp_ns',
        )
        < first_exact_record['sim_stamp_ns']
        or require_int(
            first_contact.get('clock_delivery_offset_ns'),
            'positive_control.first_qualifying_contact.clock_delivery_offset_ns',
        )
        != first_observed_stamp - first_exact_record['sim_stamp_ns']
    ):
        raise MetricUnavailable('positive-control first qualifying contact is missing')
    first_contact_sequence = require_int(
        first_contact.get('collector_sequence'),
        'positive_control.first_qualifying_contact.collector_sequence',
    )
    first_contact_stamp = require_int(
        first_contact.get('sim_stamp_ns'),
        'positive_control.first_qualifying_contact.sim_stamp_ns',
    )
    episodes = contact.get('episodes')
    if not isinstance(episodes, list) or len(episodes) != 1:
        raise MetricUnavailable('positive-control must contain exactly one contact episode')
    episode = episodes[0]
    counted_stamps = [record['sim_stamp_ns'] for record in counted_records]
    if any(
        later - earlier >= CONTACT_RELEASE_GAP_NS for earlier, later in pairwise(counted_stamps)
    ):
        raise MetricUnavailable('positive-control counted records imply multiple episodes')
    normalized_pairs = sorted({tuple(record['normalized_pair']) for record in counted_records})
    expected_episode = {
        'counterpart_model': classification['counterpart_model'],
        'end_stamp_ns': max(counted_stamps) + CONTACT_RELEASE_GAP_NS,
        'normalized_pairs': [list(pair) for pair in normalized_pairs],
        'sample_count': len(counted_records),
        'start_stamp_ns': min(counted_stamps),
    }
    if not isinstance(episode, Mapping) or dict(episode) != expected_episode:
        raise MetricUnavailable('positive-control episode boundaries do not reconcile')
    timeline = control.get('timeline')
    if not isinstance(timeline, Mapping):
        raise MetricUnavailable('positive-control timeline is missing')
    control_started_stamp = require_int(
        timeline.get('control_started_stamp_ns'),
        'positive_control.timeline.control_started_stamp_ns',
    )
    stop_command_stamp = require_int(
        timeline.get('stop_command_stamp_ns'), 'positive_control.timeline.stop_command_stamp_ns'
    )
    hold_complete_stamp = require_int(
        timeline.get('hold_complete_stamp_ns'), 'positive_control.timeline.hold_complete_stamp_ns'
    )
    reverse_start_stamp = require_int(
        timeline.get('reverse_start_stamp_ns'), 'positive_control.timeline.reverse_start_stamp_ns'
    )
    final_zero_stamp = require_int(
        timeline.get('final_zero_stamp_ns'), 'positive_control.timeline.final_zero_stamp_ns'
    )
    release_complete_stamp = require_int(
        timeline.get('release_complete_stamp_ns'),
        'positive_control.timeline.release_complete_stamp_ns',
    )
    stop_latency = require_int(timeline.get('stop_latency_ns'), 'positive_control.stop_latency_ns')
    release_start_count = require_int(
        timeline.get('release_contact_message_start_count'),
        'positive_control.release_contact_message_start_count',
    )
    release_end_count = require_int(
        timeline.get('release_contact_message_end_count'),
        'positive_control.release_contact_message_end_count',
    )
    release_required_stamp = require_int(
        timeline.get('release_required_through_stamp_ns'),
        'positive_control.release_required_through_stamp_ns',
    )
    expected_release_stamp = max(
        final_zero_stamp + CONTACT_RELEASE_GAP_NS,
        expected_episode['end_stamp_ns'],
    )
    if (
        control_started_stamp <= 0
        or wall_stamp > control_started_stamp
        or not control_started_stamp
        <= first_contact_stamp
        <= control_started_stamp + _CONTROL_CONTACT_DEADLINE_NS
        or first_observed_stamp != stop_command_stamp
        or stop_latency != max(0, stop_command_stamp - first_contact_stamp)
        or stop_latency > _CONTROL_STOP_DEADLINE_NS
        or hold_complete_stamp - stop_command_stamp < _CONTROL_HOLD_NS
        or reverse_start_stamp != hold_complete_stamp
        or final_zero_stamp - reverse_start_stamp < _CONTROL_REVERSE_NS
        or release_required_stamp != expected_release_stamp
        or release_complete_stamp < release_required_stamp
        or release_start_count < 1
        or release_end_count <= release_start_count
    ):
        raise MetricUnavailable('positive-control timeline does not match the frozen driver')
    commands = control.get('command_trace')
    _validate_command_trace(
        commands,
        control_started_stamp=control_started_stamp,
        first_contact_sequence=first_contact_sequence,
        stop_command_stamp=stop_command_stamp,
        reverse_start_stamp=reverse_start_stamp,
        final_zero_stamp=final_zero_stamp,
    )
    if (
        response_stamp > control_started_stamp
        or start_stamp > control_started_stamp
        or commands[0]['collector_sequence']
        <= max(response_sequence, wall_sequence, start_sequence)
        or first_contact_sequence
        < max(
            command['collector_sequence'] for command in commands if command['phase'] == 'FORWARD'
        )
        + 2
        or start_stamp + start_alignment != control_started_stamp
    ):
        raise MetricUnavailable('positive-control setup/control sequence does not reconcile')
    cleanup = positive_control.get('cleanup')
    cleanup_fields = {
        'actor_absent',
        'delete_attempt_count',
        'delete_success',
        'proof',
        'required',
    }
    cleanup_proof_fields = {
        'kind',
        'pose_source_publishers_after',
        'pose_source_publishers_before',
        'post_delete_pose_count',
        'post_delete_pose_source_heartbeat_count',
        'post_delete_pose_source_latest_sim_stamp_ns',
        'quiet_until_sim_stamp_ns',
        'request_sequence',
        'request_stamp_ns',
        'response_sequence',
        'response_stamp_ns',
    }
    if (
        not isinstance(cleanup, Mapping)
        or set(cleanup) != cleanup_fields
        or cleanup.get('required') is not True
        or cleanup.get('delete_attempt_count') != 1
        or cleanup.get('delete_success') is not True
        or cleanup.get('actor_absent') is not True
    ):
        raise MetricUnavailable('positive-control actor cleanup is incomplete')
    cleanup_proof = cleanup.get('proof')
    if (
        not isinstance(cleanup_proof, Mapping)
        or set(cleanup_proof) != cleanup_proof_fields
        or cleanup_proof.get('kind') != 'successful_delete_response_and_pose_quiet_interval'
    ):
        raise MetricUnavailable('positive-control actor cleanup proof is incomplete')
    cleanup_request_sequence = require_int(
        cleanup_proof.get('request_sequence'),
        'positive_control.cleanup.request_sequence',
    )
    cleanup_response_sequence = require_int(
        cleanup_proof.get('response_sequence'),
        'positive_control.cleanup.response_sequence',
    )
    cleanup_request_stamp = require_int(
        cleanup_proof.get('request_stamp_ns'),
        'positive_control.cleanup.request_stamp_ns',
    )
    cleanup_response_stamp = require_int(
        cleanup_proof.get('response_stamp_ns'),
        'positive_control.cleanup.response_stamp_ns',
    )
    cleanup_quiet_until = require_int(
        cleanup_proof.get('quiet_until_sim_stamp_ns'),
        'positive_control.cleanup.quiet_until_sim_stamp_ns',
    )
    cleanup_latest_pose_stamp = require_int(
        cleanup_proof.get('post_delete_pose_source_latest_sim_stamp_ns'),
        'positive_control.cleanup.post_delete_pose_source_latest_sim_stamp_ns',
    )
    max_pre_cleanup_sequence = max(
        response_sequence,
        wall_sequence,
        start_sequence,
        *(command['collector_sequence'] for command in commands),
        *(record['collector_sequence'] for record in contact['records']),
    )
    if (
        cleanup_request_sequence <= max_pre_cleanup_sequence
        or cleanup_request_sequence
        < commands[-1]['collector_sequence'] + (release_end_count - release_start_count) + 1
        or cleanup_response_sequence <= cleanup_request_sequence
        or cleanup_request_stamp != release_complete_stamp
        or cleanup_response_stamp < cleanup_request_stamp
        or cleanup_quiet_until != cleanup_response_stamp + ACTOR_CLEANUP_QUIET_NS
        or cleanup_latest_pose_stamp < cleanup_quiet_until
        or require_int(
            cleanup_proof.get('post_delete_pose_source_heartbeat_count'),
            'positive_control.cleanup.post_delete_pose_source_heartbeat_count',
        )
        < 1
        or require_int(
            cleanup_proof.get('post_delete_pose_count'),
            'positive_control.cleanup.post_delete_pose_count',
        )
        != 0
        or require_int(
            cleanup_proof.get('pose_source_publishers_before'),
            'positive_control.cleanup.pose_source_publishers_before',
        )
        < 1
        or require_int(
            cleanup_proof.get('pose_source_publishers_after'),
            'positive_control.cleanup.pose_source_publishers_after',
        )
        < 1
    ):
        raise MetricUnavailable('positive-control actor cleanup proof is incomplete')
    quality = positive_control.get('quality')
    if not isinstance(quality, Mapping):
        raise MetricUnavailable('positive-control quality is missing')
    for field in (
        'all_buffers_bounded',
        'collision_monitor_absent',
        'nav2_absent',
        'overflow_free',
        'relative_project_names',
        'sole_cmd_vel_publisher',
        'source_streams_live',
    ):
        if quality.get(field) is not True:
            raise MetricUnavailable(f'positive-control quality gate {field} did not pass')
    if quality.get('cmd_vel_publisher_count') != 1 or quality.get('protocol_error_count') != 0:
        raise MetricUnavailable('positive-control graph/protocol counters are invalid')
    if quality.get('forbidden_nodes') != []:
        raise MetricUnavailable('positive-control forbidden navigation nodes were present')
    source_publishers = quality.get('source_publisher_counts')
    if not isinstance(source_publishers, Mapping) or set(source_publishers) != {
        'contacts',
        'entity_pose',
        'ground_truth',
    }:
        raise MetricUnavailable('positive-control source publisher evidence is incomplete')
    if any(
        require_int(value, f'positive_control.source_publisher_counts.{name}') < 1
        for name, value in source_publishers.items()
    ):
        raise MetricUnavailable('positive-control observation source is not live')
    if require_int(
        cleanup_proof.get('pose_source_publishers_after'),
        'positive_control.cleanup.pose_source_publishers_after',
    ) != require_int(
        source_publishers.get('entity_pose'),
        'positive_control.source_publisher_counts.entity_pose',
    ):
        raise MetricUnavailable('positive-control cleanup/source publishers differ')
    heartbeat = quality.get('contact_message_heartbeat')
    if not isinstance(heartbeat, Mapping) or set(heartbeat) != {
        'first_stamp_ns',
        'latest_stamp_ns',
        'max_gap_ns',
        'message_count',
    }:
        raise MetricUnavailable('positive-control contact heartbeat is incomplete')
    heartbeat_first = require_int(
        heartbeat.get('first_stamp_ns'), 'positive_control.contact_heartbeat.first_stamp_ns'
    )
    heartbeat_latest = require_int(
        heartbeat.get('latest_stamp_ns'), 'positive_control.contact_heartbeat.latest_stamp_ns'
    )
    heartbeat_count = require_int(
        heartbeat.get('message_count'), 'positive_control.contact_heartbeat.message_count'
    )
    heartbeat_gap = require_int(
        heartbeat.get('max_gap_ns'), 'positive_control.contact_heartbeat.max_gap_ns'
    )
    if (
        heartbeat_first < 0
        or heartbeat_latest < heartbeat_first
        or heartbeat_first > min(record['sim_stamp_ns'] for record in contact['records'])
        or not heartbeat_first <= first_contact_stamp <= heartbeat_latest
        or heartbeat_latest < release_required_stamp
        or heartbeat_count != release_end_count
        or heartbeat_gap < 0
    ):
        raise MetricUnavailable('positive-control contact heartbeat does not span release')
    clock = quality.get('clock')
    if not isinstance(clock, Mapping):
        raise MetricUnavailable('positive-control clock evidence is missing')
    clock_latest = require_int(
        clock.get('latest_stamp_ns'), 'positive_control.clock.latest_stamp_ns'
    )
    if (
        require_int(clock.get('sample_count'), 'positive_control.clock.sample_count') < 1
        or require_int(clock.get('regression_count'), 'positive_control.clock.regression_count')
        != 0
        or require_int(clock.get('max_gap_ns'), 'positive_control.clock.max_gap_ns') < 0
        or clock_latest
        < max(release_complete_stamp, cleanup_quiet_until, cleanup_latest_pose_stamp)
    ):
        raise MetricUnavailable('positive-control clock source evidence is invalid')
    buffers = quality.get('buffers')
    if not isinstance(buffers, Mapping) or set(buffers) != {
        'actor_state',
        'command',
        'contact_records',
        'contact_summaries',
        'ground_truth',
    }:
        raise MetricUnavailable('positive-control bounded-buffer evidence is incomplete')
    buffer_counts = {
        name: _require_bounded_buffer(
            buffer,
            name,
            retain_every_accepted_item=name != 'actor_state',
        )
        for name, buffer in buffers.items()
    }
    raw_stream = _require_bounded_buffer(
        quality.get('raw_contact_stream'),
        'raw_contact_stream',
        retain_every_accepted_item=True,
    )
    if (
        buffer_counts['command']['retained_count'] != len(commands)
        or buffer_counts['contact_records']['retained_count'] != raw_count
        or raw_stream['retained_count'] != raw_count
        or raw_stream != buffer_counts['contact_records']
        or buffer_counts['contact_summaries']['retained_count'] != heartbeat_count
        or buffer_counts['ground_truth']['retained_count'] < 1
        or buffer_counts['actor_state']['retained_count'] < 1
    ):
        raise MetricUnavailable('positive-control retained buffers do not match evidence')
    return {
        'control_configuration_sha256': configuration['control_configuration_sha256'],
        'driver_source_sha256': driver_source_sha,
        'fixture_sha256': configuration['fixture_sha256'],
        'run_id': identity['run_id'],
        'scenario_sha256': scenario_hash,
        'wall_asset_sha256': configuration['wall_asset_sha256'],
    }


def validate_collision_qualification(
    manifest: Mapping[str, Any],
    positive_control: Mapping[str, Any],
    benchmark_binding: Mapping[str, Any],
    *,
    wall_asset_path: Path | None = None,
) -> dict[str, Any]:
    """Require complete static coverage and a hash-identical positive control."""
    _, names, _ = _collision_names(manifest)
    if wall_asset_path is None:
        module_path = Path(__file__).resolve()
        source_suffix = (
            'src',
            'robotest_metrics',
            'robotest_metrics',
            'collision_metrics.py',
        )
        source_layout = (
            module_path.parents[2].name,
            module_path.parents[1].name,
            module_path.parent.name,
            module_path.name,
        )
        if source_layout == source_suffix:
            repository = module_path.parents[3]
            source_asset = (
                repository / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
            ).resolve()
            if source_asset.is_file() and source_asset.is_relative_to(repository):
                wall_asset_path = source_asset
        if wall_asset_path is None:
            from ament_index_python.packages import get_package_share_directory

            wall_asset_path = (
                Path(get_package_share_directory('robotest_sim'))
                / 'models/phase3_contact_control_wall.sdf'
            )
    wall_asset_path = wall_asset_path.resolve()
    if not wall_asset_path.is_file():
        raise MetricUnavailable('positive-control wall asset is unavailable')
    positive_identity = _positive_control_evidence(
        positive_control,
        manifest,
        file_sha256(wall_asset_path),
    )
    positive_provenance = benchmark_binding.get('positive_control_provenance')
    benchmark_provenance = benchmark_binding.get('benchmark_provenance')
    if not isinstance(positive_provenance, Mapping) or not isinstance(
        benchmark_provenance, Mapping
    ):
        raise MetricUnavailable('control/benchmark provenance bindings are missing')
    if set(positive_provenance) != set(_PROVENANCE_HASHES) or set(benchmark_provenance) != set(
        _PROVENANCE_HASHES
    ):
        raise MetricUnavailable('control/benchmark provenance fields are incomplete')
    for name in _PROVENANCE_HASHES:
        control_value = _sha256(
            positive_provenance.get(name),
            f'positive_control_provenance.{name}',
        )
        benchmark_value = _sha256(
            benchmark_provenance.get(name),
            f'benchmark_provenance.{name}',
        )
        if control_value != benchmark_value:
            raise MetricUnavailable(f'collision qualification hash mismatch for {name}')
    manifest_expected = {
        'bridge_sha256': manifest['bridge_sha256'],
        'contact_configuration_sha256': manifest['contact_configuration_sha256'],
        'coverage_manifest_sha256': manifest['manifest_sha256'],
        'rendered_sdf_sha256': manifest['rendered_sdf_sha256'],
        'robot_description_sha256': manifest['robot_description_sha256'],
        'world_source_sha256': manifest['world_source_sha256'],
    }
    for name, expected_value in manifest_expected.items():
        if positive_provenance[name] != expected_value:
            raise MetricUnavailable(f'collision manifest provenance mismatch for {name}')
    external_quality = benchmark_binding.get('positive_control_external_quality')
    if not isinstance(external_quality, Mapping):
        raise MetricUnavailable('positive-control external quality is missing')
    for field in (
        'checksum_verified',
        'collector_reconciled',
        'owned_process_group_shutdown',
    ):
        if external_quality.get(field) is not True:
            raise MetricUnavailable(f'positive-control external gate {field} did not pass')
    _sha256(
        external_quality.get('collector_capture_sha256'),
        'positive_control_external_quality.collector_capture_sha256',
    )
    positive_json_sha = _sha256(
        benchmark_binding.get('positive_control_json_sha256'),
        'benchmark_binding.positive_control_json_sha256',
    )
    try:
        calculated_positive_hash = canonical_sha256(positive_control)
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control JSON is not canonicalizable') from exc
    if positive_json_sha != calculated_positive_hash:
        raise MetricUnavailable('positive-control canonical JSON hash mismatch')
    if benchmark_binding.get('positive_control_run_id') != positive_identity['run_id']:
        raise MetricUnavailable('positive-control run ID mismatch')
    scenario_hash = _sha256(
        benchmark_binding.get('positive_control_scenario_sha256'),
        'benchmark_binding.positive_control_scenario_sha256',
    )
    if scenario_hash != positive_identity['scenario_sha256']:
        raise MetricUnavailable('positive-control scenario hash mismatch')
    return {
        'covered_collision_count': len(names),
        'coverage_manifest_sha256': manifest['manifest_sha256'],
        'positive_control_json_sha256': positive_json_sha,
        'positive_control_run_id': positive_identity['run_id'],
        'positive_control_scenario_sha256': scenario_hash,
        'status': 'PASS',
    }


def _top_model(scoped_collision: str) -> str:
    stripped = scoped_collision.strip('/')
    return stripped.split('::', maxsplit=1)[0]


def classify_contact_pair(
    collision1: Any,
    collision2: Any,
    manifest: Mapping[str, Any],
) -> dict[str, Any]:
    """Classify one unordered contact pair using only exact manifest names."""
    robot_model, robot_names, support_pairs = _collision_names(manifest)
    return _classify_contact_pair_resolved(
        collision1,
        collision2,
        robot_model,
        robot_names,
        support_pairs,
    )


def _classify_contact_pair_resolved(
    collision1: Any,
    collision2: Any,
    robot_model: str,
    robot_names: set[str],
    support_pairs: set[tuple[str, str]],
) -> dict[str, Any]:
    """Classify against a manifest that was already validated and resolved."""
    if not isinstance(collision1, str) or not isinstance(collision2, str):
        raise MetricUnavailable('contact collision names must be strings')
    first_robot = collision1 in robot_names
    second_robot = collision2 in robot_names
    if first_robot and second_robot:
        return {'counted': False, 'reason': 'robot_internal'}
    if not first_robot and not second_robot:
        return {'counted': False, 'reason': 'unrelated_environment_contact'}
    robot_collision = collision1 if first_robot else collision2
    counterpart_collision = collision2 if first_robot else collision1
    if (robot_collision, counterpart_collision) in support_pairs:
        return {'counted': False, 'reason': 'allowlisted_support_contact'}
    counterpart_model = _top_model(counterpart_collision)
    if not counterpart_model or counterpart_model == robot_model:
        raise MetricUnavailable('contact counterpart cannot be resolved')
    return {
        'counted': True,
        'counterpart_collision': counterpart_collision,
        'counterpart_model': counterpart_model,
        'robot_collision': robot_collision,
    }


def _maximum_optional(values: Any, name: str) -> float | None:
    if values is None:
        return None
    if not isinstance(values, list):
        raise MetricUnavailable(f'{name} must be a list')
    if not values:
        return None
    normalized = [require_finite(value, name) for value in values]
    if any(value < 0.0 for value in normalized):
        raise MetricUnavailable(f'{name} cannot contain negative depths')
    return max(normalized)


def analyze_collisions(
    messages: Sequence[Mapping[str, Any]],
    accepted_goal_stamp_ns: int,
    terminal_action_stamp_ns: int,
    drain_completed_stamp_ns: int,
    manifest: Mapping[str, Any],
    positive_control: Mapping[str, Any],
    benchmark_binding: Mapping[str, Any],
    *,
    release_gap_ns: int = CONTACT_RELEASE_GAP_NS,
) -> dict[str, Any]:
    """Classify and de-duplicate collision episodes through terminal drain."""
    qualification = validate_collision_qualification(manifest, positive_control, benchmark_binding)
    robot_model, robot_names, support_pairs = _collision_names(manifest)
    start = require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns')
    terminal = require_int(terminal_action_stamp_ns, 'terminal_action_stamp_ns')
    drain = require_int(drain_completed_stamp_ns, 'drain_completed_stamp_ns')
    if terminal <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    if release_gap_ns <= 0:
        raise ValueError('release_gap_ns must be positive')
    required_drain = terminal + release_gap_ns
    if drain < required_drain:
        raise MetricUnavailable('contact terminal drain is incomplete')
    active: dict[str, dict[str, Any]] = {}
    events: list[dict[str, Any]] = []
    exclusions: Counter[str] = Counter()
    pre_action_contacts = 0
    post_terminal_contacts = 0
    previous_order: tuple[int, int] | None = None
    contact_record_count = 0
    in_action_message_count = 0

    def close_stale(now_ns: int) -> None:
        for key, event in list(active.items()):
            if now_ns - event['last_contact_stamp_ns'] >= release_gap_ns:
                event['end_stamp_ns'] = event['last_contact_stamp_ns'] + release_gap_ns
                event['duration_s'] = (
                    event['end_stamp_ns'] - event['start_stamp_ns']
                ) / 1_000_000_000
                event['censored_at_drain'] = False
                events.append(event)
                del active[key]

    for message_index, message in enumerate(messages):
        stamp = require_int(message.get('stamp_ns'), f'messages[{message_index}].stamp_ns')
        sequence = require_int(
            message.get('collector_sequence'),
            f'messages[{message_index}].collector_sequence',
        )
        order = (stamp, sequence)
        if previous_order is not None and order <= previous_order:
            raise MetricUnavailable('contact messages are not in collector order')
        previous_order = order
        if stamp > drain:
            continue
        if start <= stamp <= terminal:
            in_action_message_count += 1
        close_stale(stamp)
        contacts = message.get('contacts')
        if not isinstance(contacts, list):
            raise MetricUnavailable('contact message contacts must be a list')
        contact_record_count += len(contacts)
        if contact_record_count > CONTACT_RECORD_CAPACITY:
            raise MetricUnavailable('normalized contact record capacity exceeded')
        grouped: dict[str, dict[str, Any]] = {}
        for contact_index, contact in enumerate(contacts):
            if not isinstance(contact, Mapping):
                raise MetricUnavailable(
                    f'messages[{message_index}].contacts[{contact_index}] must be an object'
                )
            classification = _classify_contact_pair_resolved(
                contact.get('collision1'),
                contact.get('collision2'),
                robot_model,
                robot_names,
                support_pairs,
            )
            if not classification['counted']:
                exclusions[classification['reason']] += 1
                continue
            counterpart = classification['counterpart_model']
            group = grouped.setdefault(
                counterpart,
                {
                    'maximum_normal_force_n': None,
                    'maximum_penetration_depth_m': None,
                    'pairs': set(),
                    'record_count': 0,
                },
            )
            group['pairs'].add(
                (
                    classification['robot_collision'],
                    classification['counterpart_collision'],
                )
            )
            group['record_count'] += 1
            if 'maximum_penetration_depth_m' in contact:
                depth_value = contact.get('maximum_penetration_depth_m')
                depth = (
                    None
                    if depth_value is None
                    else require_finite(depth_value, 'contact.maximum_penetration_depth_m')
                )
                if depth is not None and depth < 0.0:
                    raise MetricUnavailable('contact depth cannot be negative')
            else:
                depth = _maximum_optional(contact.get('depths_m'), 'contact.depths_m')
            force = contact.get('maximum_normal_force_n')
            if force is not None:
                force = require_finite(force, 'contact.maximum_normal_force_n')
                if force < 0.0:
                    raise MetricUnavailable('contact force cannot be negative')
            if depth is not None:
                group['maximum_penetration_depth_m'] = max(
                    depth,
                    group['maximum_penetration_depth_m'] or 0.0,
                )
            if force is not None:
                group['maximum_normal_force_n'] = max(
                    force,
                    group['maximum_normal_force_n'] or 0.0,
                )
        if stamp < start:
            pre_action_contacts += sum(group['record_count'] for group in grouped.values())
        for counterpart, group in grouped.items():
            if stamp > terminal and counterpart not in active:
                post_terminal_contacts += group['record_count']
                continue
            event = active.get(counterpart)
            if event is None:
                event = {
                    'counterpart_model': counterpart,
                    'last_contact_stamp_ns': stamp,
                    'maximum_normal_force_n': group['maximum_normal_force_n'],
                    'maximum_penetration_depth_m': group['maximum_penetration_depth_m'],
                    'pairs': set(group['pairs']),
                    'post_terminal_sample_count': 0,
                    'sample_count': group['record_count'],
                    'start_stamp_ns': stamp,
                }
                active[counterpart] = event
            else:
                event['last_contact_stamp_ns'] = stamp
                event['pairs'].update(group['pairs'])
                event['sample_count'] += group['record_count']
                for field in ('maximum_normal_force_n', 'maximum_penetration_depth_m'):
                    value = group[field]
                    if value is not None:
                        event[field] = max(value, event[field] or 0.0)
            if stamp > terminal:
                event['post_terminal_sample_count'] += group['record_count']
    if in_action_message_count == 0:
        raise MetricUnavailable('contact stream is silent during the mission interval')
    close_stale(drain)
    for event in active.values():
        event['end_stamp_ns'] = drain
        event['duration_s'] = (drain - event['start_stamp_ns']) / 1_000_000_000
        event['censored_at_drain'] = True
        events.append(event)
    normalized_events: list[dict[str, Any]] = []
    for event in sorted(
        events, key=lambda value: (value['start_stamp_ns'], value['counterpart_model'])
    ):
        if event['start_stamp_ns'] < start or event['start_stamp_ns'] > terminal:
            continue
        normalized_events.append(
            {
                **event,
                'pairs': [
                    {'counterpart_collision': pair[1], 'robot_collision': pair[0]}
                    for pair in sorted(event['pairs'])
                ],
            }
        )
    return {
        'collision_count': len(normalized_events),
        'contact_record_count': contact_record_count,
        'drain_completed_stamp_ns': drain,
        'events': normalized_events,
        'excluded_contact_counts': dict(sorted(exclusions.items())),
        'in_action_message_count': in_action_message_count,
        'pre_action_contact_record_count': pre_action_contacts,
        'post_terminal_contact_record_count': post_terminal_contacts,
        'qualification': qualification,
        'required_drain_stamp_ns': required_drain,
    }
