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

from __future__ import annotations

import copy
from typing import Any

import pytest
from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.candidate_validation import validate_scenario_component
from robotest_metrics.errors import MetricUnavailable


def _buffer() -> dict[str, Any]:
    return {
        'capacity': 128,
        'first_overflow_sequence': None,
        'first_overflow_stamp_ns': None,
        'ingress_count': 1,
        'invalid_count': 0,
        'overflow': False,
        'overflow_count': 0,
        'retained_count': 1,
    }


def _component(
    scenario_id: int,
    interaction: dict[str, Any],
    measurements: dict[str, Any],
) -> tuple[dict[str, Any], dict[str, Any]]:
    names = {
        2: 'deterministic_static_obstacle_replan',
        3: 'deterministic_dynamic_obstacle',
    }
    identity = {
        'candidate_id': 'candidate-1',
        'repetition_index': 0,
        'run_id': f'run-{scenario_id}',
        'scenario_id': scenario_id,
        'scenario_name': names[scenario_id],
        'scenario_sha256': str(scenario_id) * 64,
        'suite_index': (scenario_id - 1) * 3,
    }
    result = {
        'binding': {
            'accepted_goal_stamp_ns': measurements['accepted_goal_stamp_ns'],
            'feedback_trace': [],
            'goal_uuid': measurements['accepted_goal_uuid'],
            'ready_sim_stamp_ns': 1,
            'status_trace': [],
            'terminal_observed_sequence': measurements.get('terminal_observed_sequence'),
            'terminal_observed_stamp_ns': measurements.get('terminal_observed_stamp_ns'),
            'terminal_status': measurements['goal_status_code'],
        },
        'cleanup': {
            'actor_absent': True,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': {},
            'required': True,
        },
        'configuration': {},
        'identity': copy.deepcopy(identity),
        'interaction': interaction,
        'producer': 'robotest_scenarios/scenario_controller',
        'quality': {
            'all_buffers_bounded': True,
            'buffers': {
                name: _buffer()
                for name in ('actor_state', 'feedback', 'ground_truth', 'plans', 'status')
            },
            'immutable_t0': True,
            'overflow_free': True,
            'protocol_error_count': 0,
            'relative_project_names': True,
            'single_goal_binding': True,
        },
        'schema_version': 1,
        'status': 'PASS',
        'verdict': {
            'authority': 'component_only',
            'benchmark_pass': None,
            'exit_code': 0,
            'reason': 'component complete',
        },
    }
    wrapper = {'artifact_sha256': canonical_sha256(result), 'result': result}
    return identity, wrapper


def _plan(stamp_ns: int, sequence: int, hash_digit: str) -> dict[str, Any]:
    return {
        'collector_sequence': sequence,
        'endpoint_error_m': 0.01,
        'geometry_sha256': hash_digit * 64,
        'length_m': 2.0,
        'point_count': 3,
        'sim_stamp_ns': stamp_ns,
    }


def _scenario2() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    measurements = {
        'accepted_goal_stamp_ns': 1_000_000_000,
        'accepted_goal_uuid': 'goal-2',
        'goal_status': 'SUCCEEDED',
        'goal_status_code': 4,
        'terminal_action_stamp_ns': 10_000_000_000,
    }
    interaction = {
        'actor': {
            'asset_sha256': 'a' * 64,
            'name': 'phase3_static_block',
            'target_pose': {'x': -1.0, 'y': -3.5, 'yaw': 0.0, 'z': 0.4},
        },
        'clearance': {
            'distance_m': 0.8,
            'ground_truth_stamp_ns': 2_900_000_000,
            'minimum_m': 0.70,
            'passed': True,
        },
        'criteria': {
            'all_segments_avoid': True,
            'clearance_passed': True,
            'geometry_hash_changed': True,
            'observed_by_deadline': True,
            'plan_before_request': True,
            'replan_observed_after_request': True,
            'replan_stamp_after_request': True,
            'spawn_exactly_once': True,
        },
        'initial_plan': _plan(2_000_000_000, 10, '1'),
        'kind': 'static_obstacle',
        'observed_pose': {
            'passed': True,
            'position_error_m': 0.005,
            'yaw_error_rad': 0.005,
        },
        'replan': {
            'all_segments_avoid': True,
            'exclusion_rectangle': {
                'x_max': -0.45,
                'x_min': -1.55,
                'y_max': -2.95,
                'y_min': -4.05,
            },
            'found': True,
            'hash_changed': True,
            'plan': _plan(4_000_000_000, 30, '2'),
        },
        'spawn': {
            'attempt_count': 1,
            'response_sequence': 22,
            'response_stamp_ns': 3_100_000_000,
            'success': True,
        },
        'trigger': {
            'first_observed_stamp_ns': 3_100_000_000,
            'request_sequence': 20,
            'request_stamp_ns': 3_000_000_000,
            'target_stamp_ns': 3_000_000_000,
            'within_window': True,
        },
    }
    identity, wrapper = _component(2, interaction, measurements)
    return identity, wrapper, measurements


def _scenario3_target(index: int, anchor: int) -> dict[str, Any]:
    if index <= 40:
        x_m = -0.8 + 0.02 * index
    elif index <= 80:
        x_m = 0.0
    else:
        x_m = 0.02 * (index - 80)
    stamp = anchor + index * 100_000_000
    return {
        'attempt_count': 1,
        'index': index,
        'observed_pose': {
            'collector_sequence': index + 10,
            'latency_ns': 0,
            'position_error_m': 0.0,
            'stamp_ns': stamp,
            'yaw_error_rad': 0.0,
        },
        'outcome': 'OBSERVED',
        'request_lateness_ns': 0,
        'request_stamp_ns': stamp,
        'response_stamp_ns': stamp,
        'response_success': True,
        'target_pose': {'x': x_m, 'y': 1.5, 'yaw': 0.0, 'z': 0.4},
        'target_stamp_ns': stamp,
    }


def _scenario3() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    anchor = 2_000_000_000
    measurements = {
        'accepted_goal_stamp_ns': 1_000_000_000,
        'accepted_goal_uuid': 'goal-3',
        'goal_status': 'SUCCEEDED',
        'goal_status_code': 4,
        'terminal_action_stamp_ns': 15_000_000_000,
        'terminal_observed_sequence': 500,
        'terminal_observed_stamp_ns': 14_500_000_000,
    }
    interaction = {
        'actor': {
            'asset_sha256': 'a' * 64,
            'name': 'phase3_dynamic_block',
            'parked_pose': {'x': -0.8, 'y': 1.5, 'yaw': 0.0, 'z': 0.4},
        },
        'criteria': {
            'all_targets_observed': True,
            'all_transactions_succeeded': True,
            'exact_schedule': True,
            'exact_single_attempts': True,
            'exact_target_count': True,
            'final_endpoint': True,
            'finished_before_terminal_status_observation': True,
            'latency_within_limit': True,
            'monotonic_and_dwell': True,
            'no_early_motion': True,
            'ordered_feedback_anchor': True,
            'pose_errors_within_limit': True,
            'preloaded_before_goal': True,
        },
        'feedback_anchor': {
            'from_index': 1,
            'ordered_indices_0_1_2': True,
            'stamp_ns': anchor,
            'to_index': 2,
        },
        'kind': 'pose_controlled_obstacle',
        'metrics_owned': {
            'authoritative_action_result_ordering': {
                'owner': 'mission_runner_and_robotest_metrics',
                'required': True,
                'scenario_controller_value': None,
            },
            'collision_monitor_stop_final_command_correlation': {
                'owner': 'robotest_metrics',
                'required': True,
                'scenario_controller_value': None,
            },
        },
        'preload': {
            'observed_parked_before_ready': True,
            'position_error_m': 0.0,
            'spawn_attempt_count': 1,
            'spawn_success': True,
            'yaw_error_rad': 0.0,
        },
        'trajectory': {
            'dwell_constant': True,
            'early_motion_count': 0,
            'expected_target_count': 121,
            'final_endpoint_passed': True,
            'finished_before_terminal_status_observation': True,
            'finished_collector_sequence': 130,
            'finished_stamp_ns': 14_000_000_000,
            'invalid_value_count': 0,
            'late_observation_count': 0,
            'matched_observation_count': 121,
            'missing_observation_count': 0,
            'monotonic_move_1': True,
            'monotonic_move_2': True,
            'observed_trajectory_sha256': 'b' * 64,
            'set_attempt_count': 121,
            'set_success_count': 121,
            'target_trajectory_sha256': 'c' * 64,
            'targets': [_scenario3_target(index, anchor) for index in range(121)],
            'terminal_basis': 'follow_waypoints_status_observation',
        },
    }
    identity, wrapper = _component(3, interaction, measurements)
    return identity, wrapper, measurements


def _rehash(wrapper: dict[str, Any]) -> None:
    wrapper['artifact_sha256'] = canonical_sha256(wrapper['result'])


def test_scenario2_component_requires_complete_changed_safe_leg_zero_plan() -> None:
    identity, wrapper, measurements = _scenario2()
    assert measurements['goal_status'] == 'SUCCEEDED'
    assert measurements['goal_status_code'] == 4
    assert wrapper['result']['binding']['terminal_status'] == 4
    assert validate_scenario_component(wrapper, identity, measurements)['status'] == 'PASS'


@pytest.mark.parametrize('failure', ['same_hash', 'ordering', 'clearance', 'criteria'])
def test_scenario2_component_fails_closed(failure: str) -> None:
    identity, wrapper, measurements = _scenario2()
    interaction = wrapper['result']['interaction']
    if failure == 'same_hash':
        interaction['replan']['plan']['geometry_sha256'] = interaction['initial_plan'][
            'geometry_sha256'
        ]
    elif failure == 'ordering':
        interaction['replan']['plan']['collector_sequence'] = 20
    elif failure == 'clearance':
        interaction['clearance']['distance_m'] = 0.6
    else:
        del interaction['criteria']['plan_before_request']
    _rehash(wrapper)
    with pytest.raises(MetricUnavailable):
        validate_scenario_component(wrapper, identity, measurements)


@pytest.mark.parametrize(
    'criterion',
    ('replan_observed_after_request', 'replan_stamp_after_request'),
)
def test_scenario2_component_requires_each_post_request_replan_criterion(
    criterion: str,
) -> None:
    identity, wrapper, measurements = _scenario2()
    del wrapper['result']['interaction']['criteria'][criterion]
    _rehash(wrapper)
    with pytest.raises(MetricUnavailable, match='criteria'):
        validate_scenario_component(wrapper, identity, measurements)


@pytest.mark.parametrize(
    ('terminal_status', 'goal_status_code'),
    ((5, 4), ('SUCCEEDED', 4), (4, '4')),
)
def test_scenario_component_rejects_terminal_status_code_mismatch_or_non_numeric(
    terminal_status: object,
    goal_status_code: object,
) -> None:
    identity, wrapper, measurements = _scenario2()
    wrapper['result']['binding']['terminal_status'] = terminal_status
    measurements['goal_status_code'] = goal_status_code
    _rehash(wrapper)
    with pytest.raises(MetricUnavailable):
        validate_scenario_component(wrapper, identity, measurements)


def test_scenario3_component_requires_exact_121_target_observed_trajectory() -> None:
    identity, wrapper, measurements = _scenario3()
    result = validate_scenario_component(wrapper, identity, measurements)
    assert result['status'] == 'PASS'
    ordering = result['metrics_validation']['authoritative_action_result_ordering']
    assert ordering['owner'] == 'robotest_metrics'
    assert ordering['passed'] is True
    assert ordering['mission_terminal_action_stamp_ns'] == 15_000_000_000


@pytest.mark.parametrize(
    'failure',
    [
        'count',
        'pose',
        'latency',
        'mission_terminal',
        'status_terminal',
        'finish_pair',
        'status_criterion',
        'legacy_terminal_key',
        'terminal_basis',
        'collision_ownership',
        'ordering_ownership',
        'ordering_value',
        'exact_schedule',
    ],
)
def test_scenario3_component_fails_closed(failure: str) -> None:
    identity, wrapper, measurements = _scenario3()
    interaction = wrapper['result']['interaction']
    if failure == 'count':
        interaction['trajectory']['targets'].pop()
    elif failure == 'pose':
        interaction['trajectory']['targets'][50]['target_pose']['x'] = 0.1
    elif failure == 'latency':
        item = interaction['trajectory']['targets'][1]
        item['observed_pose']['stamp_ns'] += 300_000_000
        item['observed_pose']['latency_ns'] = 300_000_000
    elif failure == 'mission_terminal':
        interaction['trajectory']['finished_stamp_ns'] = measurements['terminal_action_stamp_ns']
    elif failure == 'status_terminal':
        wrapper['result']['binding']['terminal_observed_stamp_ns'] = interaction['trajectory'][
            'finished_stamp_ns'
        ]
        wrapper['result']['binding']['terminal_observed_sequence'] = interaction['trajectory'][
            'finished_collector_sequence'
        ]
    elif failure == 'finish_pair':
        interaction['trajectory']['finished_collector_sequence'] -= 1
    elif failure == 'status_criterion':
        interaction['criteria']['finished_before_terminal_status_observation'] = False
    elif failure == 'legacy_terminal_key':
        del interaction['criteria']['finished_before_terminal_status_observation']
        interaction['criteria']['finished_before_terminal'] = True
    elif failure == 'terminal_basis':
        interaction['trajectory']['terminal_basis'] = 'scenario_controller_guess'
    elif failure == 'collision_ownership':
        correlation = interaction['metrics_owned'][
            'collision_monitor_stop_final_command_correlation'
        ]
        correlation['owner'] = 'scenario_controller'
    elif failure == 'ordering_ownership':
        interaction['metrics_owned']['authoritative_action_result_ordering']['owner'] = (
            'robotest_metrics'
        )
    elif failure == 'ordering_value':
        interaction['metrics_owned']['authoritative_action_result_ordering'][
            'scenario_controller_value'
        ] = True
    else:
        interaction['criteria']['exact_schedule'] = False
    _rehash(wrapper)
    with pytest.raises(MetricUnavailable):
        validate_scenario_component(wrapper, identity, measurements)
