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

"""Pure composition of one canonical Phase 3 run result."""

from __future__ import annotations

import copy
import math
from collections.abc import Callable, Mapping, Sequence
from typing import Any

from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.candidate_validation import (
    validate_candidate_identity,
    validate_fault_component,
    validate_mission_component,
    validate_orchestrator_evidence,
    validate_scenario_component,
)
from robotest_metrics.collision_metrics import analyze_collisions
from robotest_metrics.constants import (
    BUFFER_CAPACITIES,
    CONTACT_RECORD_CAPACITY,
    CONTRACT_REVISION,
    LOG_MAX_BYTES,
    NAV2_LIFECYCLE_NODES,
    PER_RUN_CSV_MAX_BYTES,
    PER_RUN_DIRECTORY_MAX_BYTES,
    PER_RUN_JSON_MAX_BYTES,
    PLAN_MESSAGE_CAPACITY,
    PLAN_POSE_CAPACITY,
    PNG_MAX_BYTES,
    PNG_MAX_COUNT,
    STRING_FIELD_MAX_BYTES,
)
from robotest_metrics.errors import ArtifactError, MetricUnavailable
from robotest_metrics.geometry import require_int
from robotest_metrics.localization_metrics import localization_error_metrics
from robotest_metrics.odometry_metrics import analyze_odometry_drift
from robotest_metrics.path_metrics import actual_path_metrics, analyze_plans, path_efficiency
from robotest_metrics.recovery_metrics import (
    analyze_lidar_dropout,
    sensor_recovery_metrics,
    supervisor_recovery_metrics,
)
from robotest_metrics.safety_metrics import (
    lidar_dropout_command_safety_metrics,
    scenario3_stop_command_metrics,
)
from robotest_metrics.statistics import real_time_factor_metrics


def _mapping(value: Any, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MetricUnavailable(f'{name} must be an object')
    return value


def _sequence(value: Any, name: str) -> Sequence[Mapping[str, Any]]:
    if not isinstance(value, list) or any(not isinstance(item, Mapping) for item in value):
        raise MetricUnavailable(f'{name} must be an array of objects')
    return value


def _stream(capture: Mapping[str, Any], name: str) -> list[Mapping[str, Any]]:
    streams = _mapping(capture.get('streams'), 'capture.streams')
    stream = _mapping(streams.get(name), f'capture.streams.{name}')
    return list(_sequence(stream.get('items'), f'capture.streams.{name}.items'))


def _capture_quality(capture: Mapping[str, Any]) -> tuple[bool, dict[str, Any]]:
    failures: list[str] = []
    any_overflow = False
    streams = _mapping(capture.get('streams'), 'capture.streams')
    if set(streams) != {*BUFFER_CAPACITIES, 'plans'}:
        raise MetricUnavailable('capture stream set does not match the frozen collector')
    limits = _mapping(capture.get('limits'), 'capture.limits')
    if (
        limits.get('stream_capacities') != dict(sorted(BUFFER_CAPACITIES.items()))
        or limits.get('contact_record_capacity') != CONTACT_RECORD_CAPACITY
        or limits.get('plan_message_capacity') != PLAN_MESSAGE_CAPACITY
        or limits.get('plan_pose_capacity') != PLAN_POSE_CAPACITY
        or limits.get('string_field_max_bytes') != STRING_FIELD_MAX_BYTES
    ):
        raise MetricUnavailable('capture limits do not match metrics-contract revision 2')
    per_stream: dict[str, Any] = {}
    for name in sorted(streams):
        stream = _mapping(streams[name], f'capture.streams.{name}')
        quality = _mapping(stream.get('quality'), f'capture.streams.{name}.quality')
        overflowed = quality.get('overflowed') is True
        any_overflow = any_overflow or overflowed
        invalid_count = quality.get('invalid_count', 0)
        if isinstance(invalid_count, bool) or not isinstance(invalid_count, int):
            raise MetricUnavailable(f'capture.streams.{name}.quality.invalid_count is invalid')
        if overflowed:
            failures.append(f'{name}: overflow')
        if invalid_count:
            failures.append(f'{name}: {invalid_count} invalid observations')
        expected_capacity = PLAN_MESSAGE_CAPACITY if name == 'plans' else BUFFER_CAPACITIES[name]
        actual_capacity = quality.get('message_capacity', quality.get('capacity'))
        if actual_capacity != expected_capacity:
            raise MetricUnavailable(f'{name} retained capacity does not match contract')
        ingress = quality.get('ingress_count')
        retained = quality.get('retained_count')
        overflow_count = quality.get('overflow_count')
        if any(
            isinstance(value, bool) or not isinstance(value, int)
            for value in (
                ingress,
                retained,
                overflow_count,
            )
        ):
            raise MetricUnavailable(f'{name} quality counters are invalid')
        if ingress < retained + invalid_count or retained > expected_capacity:
            raise MetricUnavailable(f'{name} quality counters are inconsistent')
        per_stream[name] = {
            'ingress_count': quality.get('ingress_count'),
            'invalid_count': invalid_count,
            'overflow_count': quality.get('overflow_count'),
            'overflowed': overflowed,
            'retained_count': quality.get('retained_count'),
        }
    quality = _mapping(capture.get('quality'), 'capture.quality')
    contact_records = require_int(
        quality.get('contact_record_count'),
        'capture.quality.contact_record_count',
    )
    contact_record_overflow = require_int(
        quality.get('contact_record_overflow_count'),
        'capture.quality.contact_record_overflow_count',
    )
    if contact_records > CONTACT_RECORD_CAPACITY:
        raise MetricUnavailable('capture retained too many normalized contact records')
    if contact_record_overflow:
        failures.append(f'contacts: {contact_record_overflow} nested record overflows')
    if quality.get('collector_overflow') is True:
        any_overflow = True
        failures.append('capture quality reports collector overflow')
    if capture.get('stop_reason') != 'stop_file':
        failures.append(f'capture stop reason is {capture.get("stop_reason")!r}')
    started_wall = require_int(
        capture.get('started_steady_wall_ns'),
        'capture.started_steady_wall_ns',
    )
    finished_wall = require_int(
        capture.get('finished_steady_wall_ns'),
        'capture.finished_steady_wall_ns',
    )
    if finished_wall < started_wall:
        raise MetricUnavailable('capture steady-wall interval regressed')
    clock = _mapping(capture.get('clock'), 'capture.clock')
    regressions = clock.get('regression_count', 0)
    if isinstance(regressions, bool) or not isinstance(regressions, int):
        raise MetricUnavailable('capture.clock.regression_count is invalid')
    if regressions:
        failures.append(f'clock: {regressions} regressions')
    return not failures, {
        'clock': copy.deepcopy(dict(clock)),
        'failures': failures,
        'overflowed': any_overflow,
        'status': 'PASS' if not failures else 'FAIL',
        'streams': per_stream,
    }


def _feedback(
    capture: Mapping[str, Any],
    accepted_goal_uuid: str | None = None,
) -> list[dict[str, Any]]:
    feedback: list[dict[str, Any]] = []
    for event in _stream(capture, 'state_events'):
        if event.get('kind') != 'waypoint_feedback':
            continue
        if accepted_goal_uuid is not None and event.get('subject') != accepted_goal_uuid:
            continue
        feedback.append(
            {
                'collector_sequence': event.get('collector_sequence'),
                'current_waypoint': event.get('value'),
                'previous_waypoint': event.get('previous_value'),
                'stamp_ns': event.get('stamp_ns'),
            }
        )
    return feedback


def _lookup(document: Mapping[str, Any], path: str) -> Any:
    value: Any = document
    for component in path.split('.'):
        if not isinstance(value, Mapping) or component not in value:
            return None
        value = value[component]
    return value


def _evaluate_thresholds(
    result: Mapping[str, Any],
    acceptance: Mapping[str, Any],
) -> list[dict[str, Any]]:
    checks: list[dict[str, Any]] = []
    for path in sorted(acceptance):
        rule = _mapping(acceptance[path], f'targets.acceptance.{path}')
        value = _lookup(result, path)
        passed = isinstance(value, (int, float)) and not isinstance(value, bool)
        if passed:
            numeric = float(value)
            passed = math.isfinite(numeric)
            minimum = rule.get('minimum')
            maximum = rule.get('maximum')
            if minimum is not None:
                passed = passed and numeric >= float(minimum)
            if maximum is not None:
                passed = passed and numeric <= float(maximum)
        checks.append(
            {
                'actual': value,
                'maximum': rule.get('maximum'),
                'minimum': rule.get('minimum'),
                'passed': passed,
                'path': path,
            }
        )
    return checks


def _collision_metrics(
    collision_request: Mapping[str, Any],
    capture: Mapping[str, Any],
    targets: Mapping[str, Any],
    start: int,
    terminal: int,
) -> dict[str, Any]:
    manifest = _mapping(
        collision_request.get('coverage_manifest'),
        'collision.coverage_manifest',
    )
    binding = _mapping(
        collision_request.get('benchmark_binding'),
        'collision.benchmark_binding',
    )
    if manifest.get('manifest_sha256') != targets.get('collision_coverage_manifest_sha256'):
        raise MetricUnavailable('collision coverage manifest does not match target binding')
    if binding.get('positive_control_json_sha256') != targets.get('positive_control_json_sha256'):
        raise MetricUnavailable('positive-control result does not match target binding')
    for provenance_name in ('benchmark_provenance', 'positive_control_provenance'):
        provenance = _mapping(binding.get(provenance_name), f'collision.{provenance_name}')
        if provenance.get('collector_configuration_sha256') != targets.get(
            'collector_configuration_sha256'
        ):
            raise MetricUnavailable('collision collector configuration target binding differs')
    return analyze_collisions(
        _stream(capture, 'contacts'),
        start,
        terminal,
        require_int(
            collision_request.get('drain_completed_stamp_ns'),
            'collision.drain_completed_stamp_ns',
        ),
        manifest,
        _mapping(collision_request.get('positive_control'), 'collision.positive_control'),
        binding,
    )


def analyze_run(request: Mapping[str, Any]) -> dict[str, Any]:
    """Compute all requested metrics without ROS or scenario actuation."""
    identity = copy.deepcopy(dict(_mapping(request.get('identity'), 'identity')))
    targets = copy.deepcopy(dict(_mapping(request.get('targets'), 'targets')))
    candidate_identity = validate_candidate_identity(identity, targets)
    mission_wrapper = _mapping(request.get('mission'), 'mission')
    mission = _mapping(mission_wrapper.get('result'), 'mission.result')
    mission_hash = mission_wrapper.get('artifact_sha256')
    if not isinstance(mission_hash, str) or mission_hash != canonical_sha256(mission):
        raise MetricUnavailable('mission component artifact hash mismatch')
    mission_measurements = _mapping(mission.get('measurements'), 'mission.result.measurements')
    mission_targets = _mapping(mission.get('targets'), 'mission.result.targets')
    capture = _mapping(request.get('capture'), 'capture')
    start = require_int(
        mission_measurements.get('accepted_goal_stamp_ns'),
        'mission.measurements.accepted_goal_stamp_ns',
    )
    terminal = require_int(
        mission_measurements.get('terminal_action_stamp_ns'),
        'mission.measurements.terminal_action_stamp_ns',
    )
    if terminal <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    waypoints = list(_sequence(mission_targets.get('waypoints'), 'mission.targets.waypoints'))
    completed = require_int(
        mission_measurements.get('completed_waypoint_count'),
        'mission.measurements.completed_waypoint_count',
    )
    terminal_status = mission_measurements.get('terminal_status')
    if terminal_status is None:
        terminal_status = mission_measurements.get('goal_status')
    if terminal_status is None:
        terminal_status = mission_measurements.get('mission_action_status')
    if not isinstance(terminal_status, str) or not terminal_status:
        raise MetricUnavailable('mission terminal status must be a non-empty string')
    accepted_goal_uuid = mission_measurements.get('accepted_goal_uuid')
    if not isinstance(accepted_goal_uuid, str) or not accepted_goal_uuid:
        raise MetricUnavailable('mission accepted_goal_uuid must be a non-empty string')
    capture_complete, capture_details = _capture_quality(capture)
    component_failures: dict[str, str] = {}

    def component_gate(name: str, function: Callable[[], Any]) -> Any:
        try:
            return function()
        except (ArtifactError, MetricUnavailable) as exc:
            component_failures[name] = str(exc)
            return None

    mission_component = component_gate(
        'mission',
        lambda: validate_mission_component(mission, identity),
    )
    fault_component = component_gate(
        'fault_control',
        lambda: validate_fault_component(mission, capture, identity, targets),
    )
    scenario_component = component_gate(
        'scenario',
        lambda: validate_scenario_component(
            _mapping(request.get('scenario'), 'scenario'),
            identity,
            mission_measurements,
        ),
    )
    orchestrator_pair = component_gate(
        'orchestrator',
        lambda: validate_orchestrator_evidence(
            _mapping(request.get('orchestrator'), 'orchestrator'),
            identity,
            targets,
        ),
    )
    orchestrator_quality: dict[str, Any] | None = None
    resource_values: dict[str, Any] = {}
    if orchestrator_pair is not None:
        orchestrator_quality, resource_values = orchestrator_pair
    measurements: dict[str, Any] = {
        'accepted_goal_stamp_ns': start,
        'completed_waypoint_count': completed,
        'completion_time_sim_s': (terminal - start) / 1_000_000_000,
        'mission_action_status': terminal_status,
        'nav2_recovery_count': None,
        'terminal_action_stamp_ns': terminal,
    }
    for name in (
        'cpu_percent_mean',
        'cpu_percent_p95',
        'cpu_percent_peak',
        'peak_rss_sum_bytes',
        'wsl_peak_memory_bytes',
        'wsl_peak_swap_bytes',
    ):
        measurements[name] = resource_values.get(name)
    events: list[dict[str, Any]] = []
    unavailable: dict[str, str] = {
        'measurements.nav2_recovery_count': (
            'installed Jazzy FollowWaypoints feedback has no structured recovery counter'
        )
    }
    if orchestrator_pair is None:
        reason = component_failures.get('orchestrator', 'orchestrator evidence is unavailable')
        for name in (
            'cpu_percent_mean',
            'cpu_percent_p95',
            'cpu_percent_peak',
            'peak_rss_sum_bytes',
            'wsl_peak_memory_bytes',
            'wsl_peak_swap_bytes',
        ):
            unavailable[f'measurements.{name}'] = reason

    def calculate(
        name: str,
        function: Callable[[], Any],
        *,
        dependencies: Sequence[str] = (),
        requires_sim_clock: bool = True,
    ) -> Any:
        try:
            if requires_sim_clock and capture_details['clock'].get('regression_count', 0):
                raise MetricUnavailable('clock regression invalidates simulation-time evidence')
            streams = _mapping(capture.get('streams'), 'capture.streams')
            for dependency in dependencies:
                stream = _mapping(streams.get(dependency), f'capture.streams.{dependency}')
                quality = _mapping(
                    stream.get('quality'),
                    f'capture.streams.{dependency}.quality',
                )
                if quality.get('overflowed') is True:
                    raise MetricUnavailable(f'{dependency} collector prefix overflowed')
                invalid_count = quality.get('invalid_count', 0)
                if invalid_count:
                    raise MetricUnavailable(
                        f'{dependency} collector contains {invalid_count} invalid observations'
                    )
            return function()
        except MetricUnavailable as exc:
            unavailable[f'measurements.{name}'] = str(exc)
            return None

    actual = calculate(
        'actual_path',
        lambda: actual_path_metrics(_stream(capture, 'ground_truth'), start, terminal),
        dependencies=('ground_truth',),
    )
    measurements['actual_path'] = actual
    measurements['actual_path_length_m'] = (
        None if actual is None else actual['actual_path_length_m']
    )
    if actual is None:
        unavailable['measurements.actual_path_length_m'] = unavailable['measurements.actual_path']

    plans = calculate(
        'planned_paths',
        lambda: analyze_plans(
            _stream(capture, 'plans'),
            _feedback(capture, accepted_goal_uuid),
            waypoints,
            start,
            terminal,
        ),
        dependencies=('plans', 'state_events'),
    )
    measurements['planned_paths'] = plans
    for field in (
        'initial_planned_path_length_m',
        'latest_planned_path_length_m',
        'replan_count',
    ):
        measurements[field] = None if plans is None else plans[field]
        if plans is None:
            unavailable[f'measurements.{field}'] = unavailable['measurements.planned_paths']
    efficiency = None
    if plans is not None and actual is not None:
        efficiency = calculate(
            'path_efficiency',
            lambda: path_efficiency(
                plans['initial_planned_path_length_m'],
                actual['actual_path_length_m'],
            ),
        )
    else:
        unavailable['measurements.path_efficiency'] = (
            'required planned or actual path is unavailable'
        )
    measurements['path_efficiency'] = None if efficiency is None else efficiency['path_efficiency']
    measurements['path_overrun_ratio'] = (
        None if efficiency is None else efficiency['path_overrun_ratio']
    )
    if efficiency is None:
        unavailable.setdefault(
            'measurements.path_overrun_ratio',
            unavailable['measurements.path_efficiency'],
        )

    collision_request = request.get('collision')
    if isinstance(collision_request, Mapping):
        collisions = calculate(
            'collisions',
            lambda: _collision_metrics(
                collision_request,
                capture,
                targets,
                start,
                terminal,
            ),
            dependencies=('contacts',),
        )
    else:
        unavailable['measurements.collisions'] = 'collision qualification input is missing'
        collisions = None
    measurements['collisions'] = collisions
    measurements['collision_count'] = None if collisions is None else collisions['collision_count']
    if collisions is None:
        unavailable['measurements.collision_count'] = unavailable['measurements.collisions']
    else:
        events.extend({'kind': 'collision', **event} for event in collisions['events'])

    localization = calculate(
        'localization',
        lambda: localization_error_metrics(
            _stream(capture, 'ground_truth'),
            _stream(capture, 'tf_map_odom'),
            _stream(capture, 'tf_odom_base_footprint'),
            start,
            terminal,
            world_to_map=targets.get('world_to_map'),
        ),
        dependencies=('ground_truth', 'tf_map_odom', 'tf_odom_base_footprint'),
    )
    measurements['localization'] = localization
    localization_scalars = {
        'localization_coverage_ratio': ('coverage_ratio',),
        'localization_position_maximum_m': ('position_error_m', 'maximum'),
        'localization_position_p95_m': ('position_error_m', 'p95'),
        'localization_position_rmse_m': ('position_error_m', 'rmse'),
        'localization_yaw_maximum_rad': ('yaw_error_rad', 'maximum'),
        'localization_yaw_p95_rad': ('yaw_error_rad', 'p95'),
        'localization_yaw_rmse_rad': ('yaw_error_rad', 'rmse'),
    }
    for output_name, path in localization_scalars.items():
        value: Any = localization
        for key in path:
            value = value[key] if isinstance(value, Mapping) else None
        measurements[output_name] = value
        if value is None:
            unavailable[f'measurements.{output_name}'] = unavailable.get(
                'measurements.localization',
                'localization evidence is unavailable',
            )

    rtf = calculate(
        'real_time_factor',
        lambda: real_time_factor_metrics(_stream(capture, 'world_stats')),
        dependencies=('world_stats',),
    )
    measurements['real_time_factor'] = rtf
    measurements['rtf_median'] = None if rtf is None else rtf['median']
    measurements['rtf_p5'] = None if rtf is None else rtf['p5']
    if rtf is None:
        unavailable['measurements.rtf_median'] = unavailable['measurements.real_time_factor']
        unavailable['measurements.rtf_p5'] = unavailable['measurements.real_time_factor']

    if scenario_component is not None:
        measurements['scenario_interaction'] = copy.deepcopy(scenario_component['interaction'])
    if identity['scenario_id'] == 3 and scenario_component is not None:
        scenario3_safety = calculate(
            'scenario3_stop_command',
            lambda: scenario3_stop_command_metrics(
                _stream(capture, 'state_events'),
                _stream(capture, 'cmd_vel'),
                scenario_component['interaction'],
                terminal,
            ),
            dependencies=('cmd_vel', 'state_events'),
        )
        measurements['scenario3_stop_command'] = scenario3_safety

    fault = _mapping(request.get('fault'), 'fault')
    mission_fault = _mapping(mission.get('fault'), 'mission.result.fault')
    fault_schedule = _mapping(mission_fault.get('schedule'), 'mission.result.fault.schedule')
    fault_specs = _sequence(fault_schedule.get('faults'), 'mission.result.fault.schedule.faults')
    mission_fault_events = _sequence(
        mission_fault.get('events'),
        'mission.result.fault.events',
    )
    if identity['scenario_id'] == 4:
        if fault.get('kind') != 'lidar_dropout' or len(fault_specs) != 1:
            raise MetricUnavailable('Scenario 4 requires one lidar_dropout analysis binding')
        fault_spec = fault_specs[0]
        if fault_spec.get('target') != 1 or fault_spec.get('mode') != 1:
            raise MetricUnavailable('Scenario 4 schedule is not the frozen LiDAR dropout mode')
        activation = start + require_int(fault_spec.get('start_offset_ns'), 'fault start offset')
        deactivation = activation + require_int(fault_spec.get('duration_ns'), 'fault duration')
        first_affected_event = next(
            (event for event in mission_fault_events if event.get('event_type') == 5),
            None,
        )
        restored_event = next(
            (event for event in mission_fault_events if event.get('event_type') == 9),
            None,
        )
        if first_affected_event is None or restored_event is None:
            component_failures.setdefault(
                'fault_control',
                'LiDAR first-affected/restored fault events are missing',
            )
        raw_active_stamps = [
            require_int(sample.get('stamp_ns'), 'raw_scan.stamp_ns')
            for sample in _stream(capture, 'raw_scan')
            if activation <= require_int(sample.get('stamp_ns'), 'raw_scan.stamp_ns') < deactivation
        ]
        dropout = calculate(
            'lidar_dropout',
            lambda: analyze_lidar_dropout(
                _stream(capture, 'raw_scan'),
                _stream(capture, 'scan'),
                activation,
                deactivation,
                proxy_affected_message_count=(
                    None if restored_event is None else restored_event.get('affected_message_count')
                ),
                proxy_first_affected_stamp_ns=(
                    None
                    if first_affected_event is None
                    else first_affected_event.get('actual_stamp_ns')
                ),
                proxy_last_affected_stamp_ns=(raw_active_stamps[-1] if raw_active_stamps else None),
                proxy_restored_stamp_ns=(
                    None if restored_event is None else restored_event.get('actual_stamp_ns')
                ),
            ),
            dependencies=('fault_events', 'raw_scan', 'scan'),
        )
        measurements['lidar_dropout'] = dropout
        command_safety = None
        if dropout is not None:
            command_safety = calculate(
                'lidar_dropout_command_safety',
                lambda: lidar_dropout_command_safety_metrics(
                    _stream(capture, 'cmd_vel'),
                    _stream(capture, 'scan'),
                    activation,
                    dropout['actual_deactivation_stamp_ns'],
                    collision_monitor_source_timeout_s=fault.get(
                        'collision_monitor_source_timeout_s'
                    ),
                ),
                dependencies=('cmd_vel', 'scan'),
            )
        measurements['lidar_dropout_command_safety'] = command_safety
        recovery = None
        if dropout is not None and collisions is not None:
            required_lifecycle_nodes = fault.get('required_lifecycle_nodes')
            if required_lifecycle_nodes != list(NAV2_LIFECYCLE_NODES):
                component_failures['lifecycle_snapshot'] = (
                    'required lifecycle nodes do not match the exact nine-node contract'
                )
                required_lifecycle_nodes = []
            lifecycle_snapshot = _mapping(
                fault.get('lifecycle_snapshot'),
                'fault.lifecycle_snapshot',
            )
            lifecycle_quality = _mapping(
                lifecycle_snapshot.get('quality'),
                'fault.lifecycle_snapshot.quality',
            )
            lifecycle_identity = _mapping(
                lifecycle_snapshot.get('identity'),
                'fault.lifecycle_snapshot.identity',
            )
            if lifecycle_quality.get('complete') is not True or (
                lifecycle_identity.get('run_id') != identity.get('run_id')
            ):
                component_failures['lifecycle_snapshot'] = (
                    'bounded lifecycle snapshot artifact is incomplete or run-mismatched'
                )
                lifecycle_samples: list[Mapping[str, Any]] = []
            else:
                lifecycle_samples = list(
                    _sequence(
                        lifecycle_snapshot.get('samples'),
                        'fault.lifecycle_snapshot.samples',
                    )
                )
            recovery = calculate(
                'sensor_recovery',
                lambda: sensor_recovery_metrics(
                    dropout['actual_deactivation_stamp_ns'],
                    _stream(capture, 'scan'),
                    lifecycle_samples,
                    list(required_lifecycle_nodes),
                    _stream(capture, 'ground_truth'),
                    _feedback(capture, accepted_goal_uuid),
                    collisions['events'],
                    {'stamp_ns': terminal, 'status': terminal_status},
                ),
                dependencies=('contacts', 'ground_truth', 'scan', 'state_events'),
            )
        elif dropout is None:
            unavailable['measurements.sensor_recovery'] = 'LiDAR dropout evidence is unavailable'
        else:
            unavailable['measurements.sensor_recovery'] = 'collision evidence is unavailable'
        measurements['sensor_recovery'] = recovery
        measurements['sensor_recovery_time_sim_s'] = (
            None if recovery is None else recovery['sensor_recovery_time_sim_s']
        )
        if recovery is None:
            unavailable['measurements.sensor_recovery_time_sim_s'] = unavailable[
                'measurements.sensor_recovery'
            ]
    elif identity['scenario_id'] == 5:
        if fault.get('kind') != 'odometry_drift' or len(fault_specs) != 1:
            raise MetricUnavailable('Scenario 5 requires one odometry_drift analysis binding')
        fault_spec = fault_specs[0]
        parameters = _mapping(fault_spec.get('parameters'), 'odometry drift parameters')
        activation = start + require_int(fault_spec.get('start_offset_ns'), 'fault start offset')
        deactivation = activation + require_int(fault_spec.get('duration_ns'), 'fault duration')
        drift = calculate(
            'odometry_drift',
            lambda: analyze_odometry_drift(
                _stream(capture, 'raw_odom'),
                _stream(capture, 'odom'),
                _stream(capture, 'tf_odom_base_footprint'),
                activation,
                deactivation,
                x_rate_nm_per_s=require_int(
                    parameters.get('x_rate_nm_per_s'),
                    'x_rate_nm_per_s',
                ),
                yaw_rate_nrad_per_s=require_int(
                    parameters.get('yaw_rate_nrad_per_s'),
                    'yaw_rate_nrad_per_s',
                ),
            ),
            dependencies=('fault_events', 'odom', 'raw_odom', 'tf_odom_base_footprint'),
        )
        measurements['odometry_drift'] = drift
    elif fault.get('kind') != 'none' or fault_specs:
        raise MetricUnavailable('Scenarios 1-3 require an empty fault schedule and kind none')

    supervisor_events = request.get('supervisor_events')
    if supervisor_events is not None:
        supervisor = calculate(
            'supervisor_recovery',
            lambda: supervisor_recovery_metrics(_sequence(supervisor_events, 'supervisor_events')),
            requires_sim_clock=False,
        )
        measurements['supervisor_recovery'] = supervisor
        measurements['supervisor_recovery_time_wall_s'] = (
            None if supervisor is None else supervisor['supervisor_recovery_time_wall_s']
        )
        measurements['restart_count'] = None if supervisor is None else supervisor['restart_count']

    result: dict[str, Any] = {
        'events': events,
        'identity': identity,
        'measurements': measurements,
        'quality': {
            'artifact_projection_preflight': 'PASS',
            'capture': capture_details,
            'candidate_identity': candidate_identity,
            'collector_overflow': capture_details['overflowed'],
            'component_failures': dict(sorted(component_failures.items())),
            'components': {
                'fault_control': fault_component,
                'mission': mission_component,
                'mission_artifact_sha256': mission_hash,
                'orchestrator': orchestrator_quality,
                'scenario': scenario_component,
            },
            'contract_revision': CONTRACT_REVISION,
            'infrastructure_failure': None,
            'artifact_caps': {
                'canonical_csv_max_bytes': PER_RUN_CSV_MAX_BYTES,
                'canonical_json_max_bytes': PER_RUN_JSON_MAX_BYTES,
                'log_max_bytes': LOG_MAX_BYTES,
                'png_max_bytes': PNG_MAX_BYTES,
                'png_max_count': PNG_MAX_COUNT,
                'run_directory_max_bytes': PER_RUN_DIRECTORY_MAX_BYTES,
            },
            'metric_unavailable_reasons': dict(sorted(unavailable.items())),
        },
        'targets': targets,
        'verdict': {},
    }
    required = targets.get('required_metrics')
    if (
        not isinstance(required, list)
        or not required
        or any(not isinstance(path, str) or not path for path in required)
    ):
        raise MetricUnavailable('targets.required_metrics must be a non-empty string array')
    required_checks = [
        {
            'passed': _lookup(result, path) is not None,
            'path': path,
            'reason': unavailable.get(path),
        }
        for path in required
    ]
    mission_check = (
        mission_component is not None
        and terminal_status == 'SUCCEEDED'
        and completed == len(waypoints)
    )
    acceptance = targets.get('acceptance', {})
    acceptance_mapping = _mapping(acceptance, 'targets.acceptance')
    if 'measurements.collision_count' not in acceptance_mapping:
        raise MetricUnavailable('collision allowance is missing from targets.acceptance')
    collision_rule = _mapping(
        acceptance_mapping['measurements.collision_count'],
        'targets.acceptance.measurements.collision_count',
    )
    if collision_rule.get('maximum') is None:
        raise MetricUnavailable('collision allowance must define a maximum')
    threshold_checks = _evaluate_thresholds(result, acceptance_mapping)
    scenario_id = identity['scenario_id']
    scenario_metric_check = (
        scenario_id in (1, 2)
        or (scenario_id == 3 and measurements.get('scenario3_stop_command') is not None)
        or (
            scenario_id == 4
            and measurements.get('lidar_dropout') is not None
            and measurements.get('lidar_dropout_command_safety') is not None
            and measurements.get('sensor_recovery') is not None
            and measurements.get('sensor_recovery_time_sim_s', math.inf) <= 10.0
        )
        or (scenario_id == 5 and measurements.get('odometry_drift') is not None)
    )
    components_complete = (
        mission_component is not None
        and fault_component is not None
        and scenario_component is not None
        and orchestrator_pair is not None
        and not component_failures
    )
    passed = (
        capture_complete
        and components_complete
        and mission_check
        and scenario_metric_check
        and all(check['passed'] for check in required_checks)
        and all(check['passed'] for check in threshold_checks)
    )
    result['verdict'] = {
        'automated_status': 'PASS' if passed else 'FAIL',
        'capture_integrity': capture_complete,
        'components_complete': components_complete,
        'exit_code': 0 if passed else 30,
        'mission_success': mission_check,
        'scenario_metric_gate': scenario_metric_check,
        'required_metric_checks': required_checks,
        'threshold_checks': threshold_checks,
    }
    return result
