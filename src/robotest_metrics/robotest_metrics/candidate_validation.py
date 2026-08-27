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

"""Fail-closed binding of Phase 3 component and orchestrator evidence."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from typing import Any

from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.constants import (
    LOG_MAX_BYTES,
    PER_RUN_DIRECTORY_MAX_BYTES,
    PER_RUN_JSON_MAX_BYTES,
)
from robotest_metrics.errors import ArtifactError, MetricUnavailable
from robotest_metrics.geometry import require_finite, require_int

_SHA256 = re.compile(r'^[0-9a-f]{64}$')
_GIT_SHA = re.compile(r'^[0-9a-f]{40}$')
_FAULT_EVENT_FIELDS = (
    'accepted',
    'actual_stamp_ns',
    'affected_message_count',
    'arm_commit_stamp_ns',
    'arm_margin_ns',
    'bound_goal_uuid',
    'bound_t0_ns',
    'committed_fault_count',
    'committed_generation',
    'committed_schedule_hash',
    'configured_activation_stamp_ns',
    'configured_deactivation_stamp_ns',
    'event_sequence',
    'event_type',
    'fault_id',
    'header_stamp_ns',
    'input_sequence',
    'mode',
    'raw_input_count',
    'replayed',
    'requested_fault_count',
    'requested_generation',
    'requested_goal_uuid',
    'requested_schedule_hash',
    'requested_t0_ns',
    'schema_version',
    'seed',
    'state_after',
    'state_before',
    'target',
    'validated_output_count',
)
_IDENTITY_FIELDS = (
    'run_id',
    'candidate_id',
    'repetition_index',
    'suite_index',
    'scenario_id',
)
_SCENARIO_NAMES = {
    1: 'baseline_navigation',
    2: 'deterministic_static_obstacle_replan',
    3: 'deterministic_dynamic_obstacle',
    4: 'temporary_lidar_dropout',
    5: 'deterministic_odometry_drift',
}
_S2_CRITERIA = {
    'all_segments_avoid',
    'clearance_passed',
    'geometry_hash_changed',
    'observed_by_deadline',
    'plan_before_request',
    'replan_observed_after_request',
    'replan_stamp_after_request',
    'spawn_exactly_once',
}
_S3_CRITERIA = {
    'all_targets_observed',
    'all_transactions_succeeded',
    'exact_schedule',
    'exact_single_attempts',
    'exact_target_count',
    'final_endpoint',
    'finished_before_terminal_status_observation',
    'latency_within_limit',
    'monotonic_and_dwell',
    'no_early_motion',
    'ordered_feedback_anchor',
    'pose_errors_within_limit',
    'preloaded_before_goal',
}


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable(f'{name} must be an object')
    return value


def _array(value: Any, name: str) -> list[Any]:
    if not isinstance(value, list):
        raise MetricUnavailable(f'{name} must be an array')
    return value


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value or len(value.encode('utf-8')) > 4096:
        raise MetricUnavailable(f'{name} must be a bounded non-empty string')
    return value


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise MetricUnavailable(f'{name} must be a lowercase SHA-256')
    return value


def _passed(value: Any, name: str) -> None:
    if value is not True:
        raise MetricUnavailable(f'{name} did not pass')


def _finite_equal(value: Any, expected: float, name: str, *, tolerance: float = 1e-12) -> None:
    if abs(require_finite(value, name) - expected) > tolerance:
        raise MetricUnavailable(f'{name} differs from the frozen value')


def _plan_proof(value: Any, name: str) -> Mapping[str, Any]:
    plan = _mapping(value, name)
    _sha256(plan.get('geometry_sha256'), f'{name}.geometry_sha256')
    if require_int(plan.get('collector_sequence'), f'{name}.collector_sequence') <= 0:
        raise MetricUnavailable(f'{name} collector sequence is invalid')
    if require_int(plan.get('sim_stamp_ns'), f'{name}.sim_stamp_ns') <= 0:
        raise MetricUnavailable(f'{name} simulation stamp is invalid')
    if require_int(plan.get('point_count'), f'{name}.point_count') < 2:
        raise MetricUnavailable(f'{name} has fewer than two points')
    if require_finite(plan.get('length_m'), f'{name}.length_m') <= 0.000001:
        raise MetricUnavailable(f'{name} length is not valid')
    endpoint = require_finite(plan.get('endpoint_error_m'), f'{name}.endpoint_error_m')
    if not 0.0 <= endpoint <= 0.05:
        raise MetricUnavailable(f'{name} endpoint is outside tolerance')
    return plan


def _scenario2_proof(
    interaction: Mapping[str, Any],
    mission_measurements: Mapping[str, Any],
) -> None:
    criteria = _mapping(interaction.get('criteria'), 'scenario.interaction.criteria')
    if set(criteria) != _S2_CRITERIA or any(
        criteria.get(name) is not True for name in _S2_CRITERIA
    ):
        raise MetricUnavailable('Scenario 2 criteria are incomplete or failed')
    actor = _mapping(interaction.get('actor'), 'scenario.interaction.actor')
    if actor.get('name') != 'phase3_static_block':
        raise MetricUnavailable('Scenario 2 actor identity is invalid')
    _sha256(actor.get('asset_sha256'), 'scenario.interaction.actor.asset_sha256')
    target = _mapping(actor.get('target_pose'), 'scenario.interaction.actor.target_pose')
    for field, expected in {'x': -1.0, 'y': -3.5, 'z': 0.4, 'yaw': 0.0}.items():
        _finite_equal(target.get(field), expected, f'Scenario 2 target_pose.{field}')
    initial = _plan_proof(interaction.get('initial_plan'), 'scenario.interaction.initial_plan')
    replan = _mapping(interaction.get('replan'), 'scenario.interaction.replan')
    changed = _plan_proof(replan.get('plan'), 'scenario.interaction.replan.plan')
    if (
        replan.get('found') is not True
        or replan.get('hash_changed') is not True
        or replan.get('all_segments_avoid') is not True
        or initial.get('geometry_sha256') == changed.get('geometry_sha256')
    ):
        raise MetricUnavailable('Scenario 2 changed safe leg-0 plan proof is invalid')
    rectangle = _mapping(
        replan.get('exclusion_rectangle'),
        'scenario.interaction.replan.exclusion_rectangle',
    )
    for field, expected in {
        'x_min': -1.55,
        'x_max': -0.45,
        'y_min': -4.05,
        'y_max': -2.95,
    }.items():
        _finite_equal(rectangle.get(field), expected, f'Scenario 2 rectangle.{field}')
    trigger = _mapping(interaction.get('trigger'), 'scenario.interaction.trigger')
    t0 = require_int(
        mission_measurements.get('accepted_goal_stamp_ns'),
        'mission.accepted_goal_stamp_ns',
    )
    target_stamp = require_int(trigger.get('target_stamp_ns'), 'Scenario 2 target stamp')
    request_stamp = require_int(trigger.get('request_stamp_ns'), 'Scenario 2 request stamp')
    request_sequence = require_int(
        trigger.get('request_sequence'),
        'Scenario 2 request sequence',
    )
    observed_stamp = require_int(
        trigger.get('first_observed_stamp_ns'),
        'Scenario 2 observed stamp',
    )
    if (
        target_stamp != t0 + 2_000_000_000
        or request_stamp < target_stamp
        or observed_stamp > t0 + 2_250_000_000
        or trigger.get('within_window') is not True
        or initial['sim_stamp_ns'] > request_stamp
        or initial['collector_sequence'] >= request_sequence
    ):
        raise MetricUnavailable('Scenario 2 insertion ordering/window proof is invalid')
    spawn = _mapping(interaction.get('spawn'), 'scenario.interaction.spawn')
    if spawn.get('attempt_count') != 1 or spawn.get('success') is not True:
        raise MetricUnavailable('Scenario 2 spawn was not exactly-once successful')
    response_sequence = require_int(
        spawn.get('response_sequence'),
        'Scenario 2 spawn response sequence',
    )
    if changed['collector_sequence'] <= response_sequence:
        raise MetricUnavailable('Scenario 2 replan did not follow actor insertion')
    clearance = _mapping(interaction.get('clearance'), 'scenario.interaction.clearance')
    clearance_distance = require_finite(clearance.get('distance_m'), 'Scenario 2 clearance')
    clearance_stamp = require_int(
        clearance.get('ground_truth_stamp_ns'),
        'Scenario 2 clearance stamp',
    )
    if (
        clearance.get('passed') is not True
        or clearance.get('minimum_m') != 0.70
        or clearance_distance < 0.70
        or not 0 <= request_stamp - clearance_stamp <= 250_000_000
    ):
        raise MetricUnavailable('Scenario 2 clearance proof is invalid')
    observed = _mapping(interaction.get('observed_pose'), 'scenario.interaction.observed_pose')
    if (
        observed.get('passed') is not True
        or require_finite(observed.get('position_error_m'), 'Scenario 2 position error') > 0.01
        or require_finite(observed.get('yaw_error_rad'), 'Scenario 2 yaw error') > 0.01
    ):
        raise MetricUnavailable('Scenario 2 observed actor pose is outside tolerance')


def _scenario3_target(index: int, anchor_stamp_ns: int) -> tuple[int, float]:
    stamp = anchor_stamp_ns + index * 100_000_000
    if index <= 40:
        x_m = -0.8 + 0.02 * index
    elif index <= 80:
        x_m = 0.0
    else:
        x_m = 0.02 * (index - 80)
    return stamp, x_m


def _scenario3_proof(
    interaction: Mapping[str, Any],
    binding: Mapping[str, Any],
    mission_measurements: Mapping[str, Any],
) -> dict[str, Any]:
    criteria = _mapping(interaction.get('criteria'), 'scenario.interaction.criteria')
    if set(criteria) != _S3_CRITERIA or any(
        criteria.get(name) is not True for name in _S3_CRITERIA
    ):
        raise MetricUnavailable('Scenario 3 criteria are incomplete or failed')
    actor = _mapping(interaction.get('actor'), 'scenario.interaction.actor')
    if actor.get('name') != 'phase3_dynamic_block':
        raise MetricUnavailable('Scenario 3 actor identity is invalid')
    _sha256(actor.get('asset_sha256'), 'scenario.interaction.actor.asset_sha256')
    parked = _mapping(actor.get('parked_pose'), 'scenario.interaction.actor.parked_pose')
    for field, expected in {'x': -0.8, 'y': 1.5, 'z': 0.4, 'yaw': 0.0}.items():
        _finite_equal(parked.get(field), expected, f'Scenario 3 parked_pose.{field}')
    preload = _mapping(interaction.get('preload'), 'scenario.interaction.preload')
    if (
        preload.get('spawn_attempt_count') != 1
        or preload.get('spawn_success') is not True
        or preload.get('observed_parked_before_ready') is not True
        or require_finite(preload.get('position_error_m'), 'Scenario 3 preload position error')
        > 0.01
        or require_finite(preload.get('yaw_error_rad'), 'Scenario 3 preload yaw error') > 0.01
    ):
        raise MetricUnavailable('Scenario 3 parked preload proof is invalid')
    anchor = _mapping(interaction.get('feedback_anchor'), 'scenario.interaction.feedback_anchor')
    anchor_stamp = require_int(anchor.get('stamp_ns'), 'Scenario 3 feedback anchor stamp')
    if (
        anchor.get('from_index') != 1
        or anchor.get('to_index') != 2
        or anchor.get('ordered_indices_0_1_2') is not True
        or anchor_stamp <= 0
    ):
        raise MetricUnavailable('Scenario 3 feedback anchor is invalid')
    trajectory = _mapping(interaction.get('trajectory'), 'scenario.interaction.trajectory')
    targets = _array(trajectory.get('targets'), 'scenario.interaction.trajectory.targets')
    if (
        trajectory.get('expected_target_count') != 121
        or trajectory.get('set_attempt_count') != 121
        or trajectory.get('set_success_count') != 121
        or trajectory.get('matched_observation_count') != 121
        or len(targets) != 121
    ):
        raise MetricUnavailable('Scenario 3 trajectory counts do not equal 121')
    for field in (
        'early_motion_count',
        'invalid_value_count',
        'late_observation_count',
        'missing_observation_count',
    ):
        if trajectory.get(field) != 0:
            raise MetricUnavailable(f'Scenario 3 trajectory {field} is nonzero')
    for field in (
        'dwell_constant',
        'final_endpoint_passed',
        'finished_before_terminal_status_observation',
        'monotonic_move_1',
        'monotonic_move_2',
    ):
        _passed(trajectory.get(field), f'Scenario 3 trajectory.{field}')
    if trajectory.get('terminal_basis') != 'follow_waypoints_status_observation':
        raise MetricUnavailable('Scenario 3 trajectory terminal basis is invalid')
    _sha256(trajectory.get('target_trajectory_sha256'), 'Scenario 3 target trajectory hash')
    _sha256(trajectory.get('observed_trajectory_sha256'), 'Scenario 3 observed trajectory hash')
    observed_pairs: list[tuple[int, int]] = []
    for expected_index, value in enumerate(targets):
        target = _mapping(value, f'Scenario 3 target[{expected_index}]')
        if (
            target.get('attempt_count') != 1
            or target.get('index') != expected_index
            or target.get('outcome') != 'OBSERVED'
            or target.get('request_lateness_ns') != 0
            or target.get('response_success') is not True
        ):
            raise MetricUnavailable('Scenario 3 target index/response is invalid')
        expected_stamp, expected_x = _scenario3_target(expected_index, anchor_stamp)
        target_stamp = require_int(target.get('target_stamp_ns'), 'Scenario 3 target stamp')
        request_stamp = require_int(target.get('request_stamp_ns'), 'Scenario 3 request stamp')
        response_stamp = require_int(target.get('response_stamp_ns'), 'Scenario 3 response stamp')
        if (
            target_stamp != expected_stamp
            or request_stamp != target_stamp
            or response_stamp < request_stamp
        ):
            raise MetricUnavailable('Scenario 3 target transaction timing is invalid')
        target_pose = _mapping(target.get('target_pose'), 'Scenario 3 target pose')
        for field, expected in {
            'x': expected_x,
            'y': 1.5,
            'z': 0.4,
            'yaw': 0.0,
        }.items():
            _finite_equal(target_pose.get(field), expected, f'Scenario 3 target pose.{field}')
        observed = _mapping(target.get('observed_pose'), 'Scenario 3 observed pose')
        observed_stamp = require_int(observed.get('stamp_ns'), 'Scenario 3 observed stamp')
        observed_sequence = require_int(
            observed.get('collector_sequence'),
            'Scenario 3 observed collector sequence',
        )
        latency = require_int(observed.get('latency_ns'), 'Scenario 3 observation latency')
        if (
            observed_sequence <= 0
            or latency != observed_stamp - target_stamp
            or not 0 <= latency <= 200_000_000
            or require_finite(observed.get('position_error_m'), 'Scenario 3 position error') > 0.02
            or require_finite(observed.get('yaw_error_rad'), 'Scenario 3 yaw error') > 0.01
        ):
            raise MetricUnavailable('Scenario 3 observed target is outside tolerance')
        observed_pairs.append((observed_stamp, observed_sequence))
    finished_stamp = require_int(
        trajectory.get('finished_stamp_ns'),
        'Scenario 3 finished stamp',
    )
    finished_sequence = require_int(
        trajectory.get('finished_collector_sequence'),
        'Scenario 3 finished collector sequence',
    )
    if finished_sequence <= 0 or (finished_stamp, finished_sequence) != max(observed_pairs):
        raise MetricUnavailable('Scenario 3 trajectory finish does not match observations')
    terminal_observed_stamp = require_int(
        binding.get('terminal_observed_stamp_ns'),
        'Scenario 3 terminal status observation stamp',
    )
    terminal_observed_sequence = require_int(
        binding.get('terminal_observed_sequence'),
        'Scenario 3 terminal status observation sequence',
    )
    if (finished_stamp, finished_sequence) >= (
        terminal_observed_stamp,
        terminal_observed_sequence,
    ):
        raise MetricUnavailable('Scenario 3 trajectory did not precede terminal status evidence')
    terminal_stamp = require_int(
        mission_measurements.get('terminal_action_stamp_ns'),
        'mission terminal stamp',
    )
    if finished_stamp >= terminal_stamp:
        raise MetricUnavailable(
            'Scenario 3 trajectory did not precede the authoritative mission action result'
        )
    return {
        'authoritative_action_result_ordering': {
            'mission_terminal_action_stamp_ns': terminal_stamp,
            'owner': 'robotest_metrics',
            'passed': True,
            'source': 'mission.result.measurements.terminal_action_stamp_ns',
            'trajectory_finished_collector_sequence': finished_sequence,
            'trajectory_finished_stamp_ns': finished_stamp,
        }
    }


def validate_candidate_identity(
    identity: Mapping[str, Any],
    targets: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate immutable per-trial and source/hash identity."""
    run_id = _string(identity.get('run_id'), 'identity.run_id')
    candidate_id = _string(identity.get('candidate_id'), 'identity.candidate_id')
    scenario_id = require_int(identity.get('scenario_id'), 'identity.scenario_id')
    scenario_index = require_int(identity.get('scenario_index'), 'identity.scenario_index')
    repetition = require_int(identity.get('repetition_index'), 'identity.repetition_index')
    suite_index = require_int(identity.get('suite_index'), 'identity.suite_index')
    if scenario_id != scenario_index or scenario_id not in _SCENARIO_NAMES:
        raise MetricUnavailable('scenario identity is outside the frozen Phase 3 suite')
    if repetition not in (0, 1, 2) or suite_index != (scenario_id - 1) * 3 + repetition:
        raise MetricUnavailable('suite/repetition identity does not match frozen order')
    if identity.get('scenario_name') != _SCENARIO_NAMES[scenario_id]:
        raise MetricUnavailable('scenario name does not match frozen scenario ID')
    if identity.get('git_dirty') is not False or identity.get('cold_stack') is not True:
        raise MetricUnavailable('candidate identity must prove clean Git and a cold stack')
    git_sha = identity.get('git_sha')
    if not isinstance(git_sha, str) or _GIT_SHA.fullmatch(git_sha) is None:
        raise MetricUnavailable('identity.git_sha must be a lowercase 40-character commit')
    domain = require_int(identity.get('ros_domain_id'), 'identity.ros_domain_id')
    if not 0 <= domain <= 232:
        raise MetricUnavailable('ROS domain is outside [0, 232]')
    partition = _string(identity.get('gz_partition'), 'identity.gz_partition')
    if not partition.startswith(f'robotest_p3_{candidate_id}_'):
        raise MetricUnavailable('Gazebo partition is not candidate-bound')
    required_hashes = (
        'collector_configuration_sha256',
        'collision_coverage_manifest_sha256',
        'fault_schedule_sha256',
        'metrics_contract_sha256',
        'positive_control_json_sha256',
        'scenario_sha256',
        'source_configuration_sha256',
        'target_set_sha256',
    )
    hashes = {name: _sha256(targets.get(name), f'targets.{name}') for name in required_hashes}
    if identity.get('scenario_sha256') != hashes['scenario_sha256']:
        raise MetricUnavailable('identity and target scenario hashes differ')
    return {
        'candidate_id': candidate_id,
        'git_sha': git_sha,
        'gz_partition': partition,
        'hashes': hashes,
        'ros_domain_id': domain,
        'run_id': run_id,
        'scenario_id': scenario_id,
        'status': 'PASS',
    }


def _match_identity(
    component: Mapping[str, Any],
    identity: Mapping[str, Any],
    name: str,
) -> None:
    for field in _IDENTITY_FIELDS:
        if component.get(field) != identity.get(field):
            raise MetricUnavailable(f'{name} identity mismatch for {field}')
    if component.get('scenario_sha256') != identity.get('scenario_sha256'):
        raise MetricUnavailable(f'{name} identity mismatch for scenario_sha256')


def validate_mission_component(
    mission: Mapping[str, Any],
    identity: Mapping[str, Any],
) -> dict[str, Any]:
    """Require a UUID-bound observed terminal result from the mission component."""
    mission_identity = _mapping(mission.get('identity'), 'mission.identity')
    for field in _IDENTITY_FIELDS:
        if mission_identity.get(field) != identity.get(field):
            raise MetricUnavailable(f'mission identity mismatch for {field}')
    if mission_identity.get('mission_sha256') != identity.get('scenario_sha256'):
        raise MetricUnavailable('mission scenario hash mismatch')
    measurements = _mapping(mission.get('measurements'), 'mission.measurements')
    targets = _mapping(mission.get('targets'), 'mission.targets')
    quality = _mapping(mission.get('quality'), 'mission.quality')
    verdict = _mapping(mission.get('verdict'), 'mission.verdict')
    _string(measurements.get('accepted_goal_uuid'), 'mission accepted_goal_uuid')
    if measurements.get('goal_status') != 'SUCCEEDED' or measurements.get('goal_status_code') != 4:
        raise MetricUnavailable('mission did not observe a SUCCEEDED action result')
    waypoint_count = require_int(targets.get('waypoint_count'), 'mission.targets.waypoint_count')
    if measurements.get('completed_waypoint_count') != waypoint_count or (
        measurements.get('missed_waypoint_count') != 0
    ):
        raise MetricUnavailable('mission waypoint completion evidence is incomplete')
    required_true = ('goal_accepted', 'terminal_result_observed')
    for field in required_true:
        _passed(quality.get(field), f'mission.quality.{field}')
    if (
        quality.get('cancellation_requested') is not False
        or quality.get('deadline_kind') not in (None, '')
        or quality.get('feedback_trace_overflow') is not False
        or quality.get('event_trace_overflow') is not False
        or quality.get('fault_event_overflow') is not False
        or quality.get('invalid_feedback_count') != 0
    ):
        raise MetricUnavailable('mission quality gate failed')
    if (
        verdict.get('exit_code') != 0
        or verdict.get('expected_outcome_met') is not True
        or verdict.get('phase2_action_integration_status') != 'PASS'
    ):
        raise MetricUnavailable('mission component verdict is not successful')
    return {
        'artifact_scope': 'UUID-bound FollowWaypoints action result',
        'identity': dict(mission_identity),
        'quality': dict(quality),
        'status': 'PASS',
        'verdict': dict(verdict),
    }


def validate_scenario_component(
    wrapper: Mapping[str, Any],
    identity: Mapping[str, Any],
    mission_measurements: Mapping[str, Any],
) -> dict[str, Any]:
    """Bind the bounded component artifact and enforce scenario-owned criteria."""
    result = _mapping(wrapper.get('result'), 'scenario.result')
    declared_hash = _sha256(wrapper.get('artifact_sha256'), 'scenario.artifact_sha256')
    try:
        calculated_hash = canonical_sha256(result)
    except ArtifactError as exc:
        raise MetricUnavailable('scenario result is not canonicalizable') from exc
    if declared_hash != calculated_hash:
        raise MetricUnavailable('scenario component artifact hash mismatch')
    if result.get('schema_version') != 1 or result.get('producer') != (
        'robotest_scenarios/scenario_controller'
    ):
        raise MetricUnavailable('scenario component producer/schema is invalid')
    if result.get('status') != 'PASS':
        raise MetricUnavailable('scenario component status is not PASS')
    verdict = _mapping(result.get('verdict'), 'scenario.result.verdict')
    if (
        verdict.get('authority') != 'component_only'
        or verdict.get('benchmark_pass') is not None
        or verdict.get('exit_code') != 0
    ):
        raise MetricUnavailable('scenario component verdict authority is invalid')
    component_identity = _mapping(result.get('identity'), 'scenario.result.identity')
    _match_identity(component_identity, identity, 'scenario')
    if component_identity.get('scenario_name') != identity.get('scenario_name'):
        raise MetricUnavailable('scenario component name mismatch')
    binding = _mapping(result.get('binding'), 'scenario.result.binding')
    if binding.get('goal_uuid') != mission_measurements.get('accepted_goal_uuid'):
        raise MetricUnavailable('scenario goal UUID does not match mission')
    if binding.get('accepted_goal_stamp_ns') != mission_measurements.get('accepted_goal_stamp_ns'):
        raise MetricUnavailable('scenario T0 does not match mission')
    terminal_status = require_int(
        binding.get('terminal_status'), 'scenario.result.binding.terminal_status'
    )
    mission_status_code = require_int(
        mission_measurements.get('goal_status_code'), 'mission.measurements.goal_status_code'
    )
    if terminal_status != mission_status_code:
        raise MetricUnavailable('scenario terminal status does not match mission')
    quality = _mapping(result.get('quality'), 'scenario.result.quality')
    for field in (
        'all_buffers_bounded',
        'immutable_t0',
        'overflow_free',
        'relative_project_names',
        'single_goal_binding',
    ):
        _passed(quality.get(field), f'scenario.quality.{field}')
    if quality.get('protocol_error_count') != 0:
        raise MetricUnavailable('scenario component has protocol errors')
    buffers = _mapping(quality.get('buffers'), 'scenario.result.quality.buffers')
    for name in ('actor_state', 'feedback', 'ground_truth', 'plans', 'status'):
        buffer = _mapping(buffers.get(name), f'scenario.quality.buffers.{name}')
        capacity = require_int(buffer.get('capacity'), f'scenario.{name}.capacity')
        retained = require_int(buffer.get('retained_count'), f'scenario.{name}.retained_count')
        if capacity <= 0 or not 0 <= retained <= capacity or buffer.get('overflow') is not False:
            raise MetricUnavailable(f'scenario buffer {name} is not complete')
        if buffer.get('overflow_count') != 0 or buffer.get('invalid_count') != 0:
            raise MetricUnavailable(f'scenario buffer {name} has invalid/overflow evidence')
    cleanup = _mapping(result.get('cleanup'), 'scenario.result.cleanup')
    _passed(cleanup.get('actor_absent'), 'scenario.cleanup.actor_absent')
    if cleanup.get('required') is True and cleanup.get('delete_success') is not True:
        raise MetricUnavailable('scenario actor cleanup did not succeed')
    interaction = _mapping(result.get('interaction'), 'scenario.result.interaction')
    scenario_id = require_int(identity.get('scenario_id'), 'identity.scenario_id')
    expected_kind = {2: 'static_obstacle', 3: 'pose_controlled_obstacle'}.get(scenario_id, 'none')
    if interaction.get('kind') != expected_kind:
        raise MetricUnavailable('scenario interaction kind does not match scenario')
    metrics_validation: dict[str, Any] = {}
    if scenario_id == 2:
        _scenario2_proof(interaction, mission_measurements)
    if scenario_id == 3:
        owned = _mapping(interaction.get('metrics_owned'), 'scenario.interaction.metrics_owned')
        correlation = _mapping(
            owned.get('collision_monitor_stop_final_command_correlation'),
            'scenario.interaction.metrics_owned.correlation',
        )
        if (
            correlation.get('owner') != 'robotest_metrics'
            or correlation.get('required') is not True
            or correlation.get('scenario_controller_value') is not None
        ):
            raise MetricUnavailable('Scenario 3 metrics ownership declaration is invalid')
        action_ordering = _mapping(
            owned.get('authoritative_action_result_ordering'),
            'scenario.interaction.metrics_owned.authoritative_action_result_ordering',
        )
        if (
            action_ordering.get('owner') != 'mission_runner_and_robotest_metrics'
            or action_ordering.get('required') is not True
            or action_ordering.get('scenario_controller_value') is not None
        ):
            raise MetricUnavailable(
                'Scenario 3 authoritative result ownership declaration is invalid'
            )
        metrics_validation = _scenario3_proof(
            interaction,
            binding,
            mission_measurements,
        )
    return {
        'artifact_sha256': declared_hash,
        'binding': dict(binding),
        'cleanup': dict(cleanup),
        'identity': dict(component_identity),
        'interaction': dict(interaction),
        'metrics_validation': metrics_validation,
        'quality': dict(quality),
        'status': 'PASS',
        'verdict': dict(verdict),
    }


def _fault_stream(capture: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    streams = _mapping(capture.get('streams'), 'capture.streams')
    fault_stream = _mapping(streams.get('fault_events'), 'capture.streams.fault_events')
    items = _array(fault_stream.get('items'), 'capture.streams.fault_events.items')
    if any(not isinstance(item, Mapping) for item in items):
        raise MetricUnavailable('captured fault events must be objects')
    return items


def validate_fault_component(
    mission: Mapping[str, Any],
    capture: Mapping[str, Any],
    identity: Mapping[str, Any],
    targets: Mapping[str, Any],
) -> dict[str, Any]:
    """Validate every Phase 3 reset/preload/arm/event/teardown proof."""
    mission_identity = _mapping(mission.get('identity'), 'mission.identity')
    for field in _IDENTITY_FIELDS:
        if mission_identity.get(field) != identity.get(field):
            raise MetricUnavailable(f'mission identity mismatch for {field}')
    if mission_identity.get('mission_sha256') != identity.get('scenario_sha256'):
        raise MetricUnavailable('mission scenario hash does not match candidate identity')
    fault = _mapping(mission.get('fault'), 'mission.fault')
    schedule = _mapping(fault.get('schedule'), 'mission.fault.schedule')
    control = _mapping(fault.get('control'), 'mission.fault.control')
    events = _array(fault.get('events'), 'mission.fault.events')
    schedule_hash = _sha256(schedule.get('sha256'), 'mission.fault.schedule.sha256')
    if schedule_hash != targets.get(
        'fault_schedule_sha256'
    ) or schedule_hash != mission_identity.get('fault_schedule_hash'):
        raise MetricUnavailable('fault schedule hash binding mismatch')
    canonical = schedule.get('canonical_json')
    if not isinstance(canonical, str):
        raise MetricUnavailable('fault schedule canonical JSON is missing')
    try:
        decoded = json.loads(canonical)
    except json.JSONDecodeError as exc:
        raise MetricUnavailable('fault schedule canonical JSON is invalid') from exc
    expected_schedule = {
        'schema_version': schedule.get('schema_version'),
        'faults': schedule.get('faults'),
    }
    if decoded != expected_schedule or hashlib.sha256(canonical.encode('utf-8')).hexdigest() != (
        schedule_hash
    ):
        raise MetricUnavailable('fault schedule canonical JSON/hash does not reconcile')
    fault_count = require_int(schedule.get('fault_count'), 'mission.fault.schedule.fault_count')
    if not isinstance(schedule.get('faults'), list) or fault_count != len(schedule['faults']):
        raise MetricUnavailable('fault schedule count does not reconcile')
    for field in ('reset_before_goal', 'reset_after_goal'):
        _passed(control.get(field), f'mission.fault.control.{field}')
    if control.get('protocol_status') != 'RESET_CONFIRMED_AFTER_GOAL':
        raise MetricUnavailable('fault-control protocol did not complete teardown reset')
    generation = require_int(control.get('generation'), 'mission.fault.control.generation')
    if generation <= 0 or control.get('event_trace_overflow') is not False:
        raise MetricUnavailable('fault-control generation/event quality is invalid')
    if control.get('event_trace_overflow_count') != 0 or control.get('event_count') != len(events):
        raise MetricUnavailable('fault-control event counters do not reconcile')
    margin = require_int(control.get('arm_margin_ns'), 'mission.fault.control.arm_margin_ns')
    if (fault_count and margin < 500_000_000) or (not fault_count and margin != 0):
        raise MetricUnavailable('fault arm margin does not satisfy the frozen contract')
    measurements = _mapping(mission.get('measurements'), 'mission.measurements')
    goal_uuid = measurements.get('accepted_goal_uuid')
    t0 = measurements.get('accepted_goal_stamp_ns')
    captured = _fault_stream(capture)
    if len(captured) != len(events):
        raise MetricUnavailable('captured and mission fault-event counts differ')
    previous_sequence: int | None = None
    for index, (mission_event, captured_event) in enumerate(zip(events, captured, strict=True)):
        mission_item = _mapping(mission_event, f'mission.fault.events[{index}]')
        sequence = require_int(
            mission_item.get('event_sequence'),
            f'fault.events[{index}].sequence',
        )
        if previous_sequence is not None and sequence != previous_sequence + 1:
            raise MetricUnavailable('fault event sequence is not contiguous')
        previous_sequence = sequence
        for field in _FAULT_EVENT_FIELDS:
            if mission_item.get(field) != captured_event.get(field):
                raise MetricUnavailable(f'captured fault event mismatch at {index}.{field}')
    armed = next((event for event in events if event.get('event_type') == 7), None)
    if not isinstance(armed, Mapping):
        raise MetricUnavailable('fault armed event is missing')
    if (
        armed.get('accepted') is not True
        or armed.get('bound_goal_uuid') != goal_uuid
        or armed.get('bound_t0_ns') != t0
        or armed.get('committed_schedule_hash') != schedule_hash
        or armed.get('committed_generation') != generation
    ):
        raise MetricUnavailable('fault armed event is not UUID/T0/hash/generation bound')
    if sum(1 for event in events if event.get('event_type') == 3) < 2:
        raise MetricUnavailable('fault reset events do not prove pre-run and teardown reset')
    return {
        'control': dict(control),
        'event_count': len(events),
        'events_sha256': canonical_sha256(events),
        'schedule_sha256': schedule_hash,
        'status': 'PASS',
    }


def validate_orchestrator_evidence(
    evidence: Mapping[str, Any],
    identity: Mapping[str, Any],
    targets: Mapping[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Validate full-lifetime process, resource, provenance, and cleanup gates."""
    if evidence.get('schema_version') != 1:
        raise MetricUnavailable('orchestrator schema_version must be 1')
    orchestrator_identity = _mapping(evidence.get('identity'), 'orchestrator.identity')
    _match_identity(orchestrator_identity, identity, 'orchestrator')
    if orchestrator_identity.get('ros_domain_id') != identity.get('ros_domain_id') or (
        orchestrator_identity.get('gz_partition') != identity.get('gz_partition')
    ):
        raise MetricUnavailable('orchestrator domain/partition identity mismatch')
    git = _mapping(evidence.get('git'), 'orchestrator.git')
    if (
        git.get('start_head') != identity.get('git_sha')
        or git.get('end_head') != identity.get('git_sha')
        or git.get('dirty') is not False
        or git.get('status_porcelain') != ''
    ):
        raise MetricUnavailable('orchestrator Git immutability gate failed')
    source = _mapping(evidence.get('source_binding'), 'orchestrator.source_binding')
    for field in (
        'source_install_match',
        'source_unchanged',
        'install_unchanged',
    ):
        _passed(source.get(field), f'orchestrator.source_binding.{field}')
    for field in (
        'source_start_sha256',
        'source_end_sha256',
        'install_start_sha256',
        'install_end_sha256',
    ):
        _sha256(source.get(field), f'orchestrator.source_binding.{field}')
    if source['source_start_sha256'] != source['source_end_sha256'] or (
        source['install_start_sha256'] != source['install_end_sha256']
    ):
        raise MetricUnavailable('source/install changed during the trial')
    for field in (
        'collector_configuration_sha256',
        'metrics_contract_sha256',
        'source_configuration_sha256',
        'target_set_sha256',
    ):
        value = _sha256(source.get(field), f'orchestrator.source_binding.{field}')
        if value != targets.get(field):
            raise MetricUnavailable(f'orchestrator target binding mismatch for {field}')
    process = _mapping(evidence.get('process'), 'orchestrator.process')
    affinity = _array(process.get('cpu_affinity'), 'orchestrator.process.cpu_affinity')
    if affinity != [0, 1, 2, 3, 4, 5]:
        raise MetricUnavailable('runtime CPU affinity must be exactly logical CPUs 0-5')
    for field in (
        'cold_stack',
        'fresh_fault_generation',
        'fresh_localization',
        'new_process_group',
        'partition_unused_before_start',
        'previous_trial_gone',
        'ros_domain_unused_before_start',
    ):
        _passed(process.get(field), f'orchestrator.process.{field}')
    resources = _mapping(evidence.get('resources'), 'orchestrator.resources')
    sample_count = require_int(resources.get('sample_count'), 'orchestrator.resources.sample_count')
    missing_count = require_int(
        resources.get('missing_sample_count'),
        'orchestrator.resources.missing_sample_count',
    )
    if sample_count < 2 or missing_count != 0:
        raise MetricUnavailable('resource sampler is incomplete')
    for field in (
        'overflow_free',
        'sampler_started_before_launch',
        'sampler_stopped_after_shutdown',
    ):
        _passed(resources.get(field), f'orchestrator.resources.{field}')
    if resources.get('pid_reuse_detected') is not False or resources.get('oom_kill') is not False:
        raise MetricUnavailable('resource sampler detected PID reuse or OOM')
    resource_values = {
        'cpu_percent_mean': require_finite(resources.get('cpu_percent_mean'), 'cpu mean'),
        'cpu_percent_p95': require_finite(resources.get('cpu_percent_p95'), 'cpu p95'),
        'cpu_percent_peak': require_finite(resources.get('cpu_percent_peak'), 'cpu peak'),
        'peak_rss_sum_bytes': require_int(resources.get('peak_rss_sum_bytes'), 'peak RSS'),
        'wsl_peak_memory_bytes': require_int(
            resources.get('wsl_peak_memory_bytes'), 'WSL peak memory'
        ),
        'wsl_peak_swap_bytes': require_int(resources.get('wsl_peak_swap_bytes'), 'WSL peak swap'),
    }
    if any(resource_values[name] < 0 for name in resource_values):
        raise MetricUnavailable('resource values cannot be negative')
    if resource_values['peak_rss_sum_bytes'] > 6 * 1024**3:
        raise MetricUnavailable('process-tree peak RSS exceeds 6.0 GiB')
    execution = _mapping(evidence.get('execution'), 'orchestrator.execution')
    _string(execution.get('command'), 'orchestrator.execution.command')
    _string(execution.get('working_directory'), 'orchestrator.execution.working_directory')
    duration = require_finite(execution.get('wall_duration_s'), 'execution.wall_duration_s')
    timeout = require_finite(execution.get('wall_timeout_s'), 'execution.wall_timeout_s')
    if (
        duration < 0.0
        or timeout <= 0.0
        or duration > timeout
        or execution.get('wall_timed_out') is not False
        or execution.get('exit_code') != 0
    ):
        raise MetricUnavailable('orchestrator execution/timeout gate failed')
    cleanup = _mapping(evidence.get('cleanup'), 'orchestrator.cleanup')
    for field in (
        'all_owned_processes_exited',
        'discovery_endpoints_gone',
        'no_orphans',
    ):
        _passed(cleanup.get(field), f'orchestrator.cleanup.{field}')
    artifacts = _mapping(evidence.get('artifacts'), 'orchestrator.artifacts')
    for field in (
        'prerequisite_checksums_verified',
        'prerequisites_finalized',
        'prerequisites_within_caps',
    ):
        _passed(artifacts.get(field), f'orchestrator.artifacts.{field}')
    _sha256(
        artifacts.get('prerequisite_manifest_sha256'),
        'prerequisite artifact manifest hash',
    )
    pre_mission_graph_sha256 = _sha256(
        artifacts.get('pre_mission_graph_sha256'),
        'pre-mission graph artifact hash',
    )
    mission_graph_sha256 = _sha256(
        artifacts.get('mission_graph_sha256'),
        'mission graph artifact hash',
    )
    if pre_mission_graph_sha256 == mission_graph_sha256:
        raise MetricUnavailable('pre-mission and mission graph artifacts must be distinct')
    artifact_count = require_int(
        artifacts.get('prerequisite_artifact_count'),
        'orchestrator.artifacts.prerequisite_artifact_count',
    )
    if not 1 <= artifact_count <= 256:
        raise MetricUnavailable('prerequisite artifact count is outside [1, 256]')
    size_limits = {
        'prerequisite_maximum_file_bytes': PER_RUN_JSON_MAX_BYTES,
        'prerequisite_total_bytes': PER_RUN_DIRECTORY_MAX_BYTES,
        'runtime_stderr_bytes': LOG_MAX_BYTES,
        'runtime_stdout_bytes': LOG_MAX_BYTES,
    }
    for field, maximum in size_limits.items():
        value = require_int(artifacts.get(field), f'orchestrator.artifacts.{field}')
        if not 0 <= value <= maximum:
            raise MetricUnavailable(f'orchestrator artifact {field} exceeds its cap')
    gates = _mapping(evidence.get('gates'), 'orchestrator.gates')
    for field in (
        'graph_contract_pass',
        'namespace_isolation_pass',
        'qos_contract_pass',
        'source_install_binding_pass',
        'validation_autonomy_isolation_pass',
    ):
        _passed(gates.get(field), f'orchestrator.gates.{field}')
    quality = {
        'artifacts': dict(artifacts),
        'cleanup': dict(cleanup),
        'execution': dict(execution),
        'gates': dict(gates),
        'git': dict(git),
        'process': dict(process),
        'resources': dict(resources),
        'source_binding': dict(source),
        'status': 'PASS',
    }
    return quality, resource_values
