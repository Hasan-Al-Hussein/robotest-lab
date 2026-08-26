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
from pathlib import Path
from typing import Any

import pytest
import yaml
from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.collector import CollectorCore
from robotest_scenarios.constants import CONTROL_ROBOT_START, CONTROL_WALL_POSE
from robotest_scenarios.provenance import (
    contact_control_configuration,
    contact_control_configuration_sha256,
    contact_source_binding,
    file_sha256,
)

EMPTY_FAULT_SCHEDULE_SHA256 = '26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8'
SCENARIO_SHA256 = '9' * 64


def _fault_event(
    sequence: int,
    event_type: int,
    stamp_ns: int,
    *,
    goal_uuid: str = 'goal-1',
) -> dict[str, Any]:
    committed_hash = EMPTY_FAULT_SCHEDULE_SHA256 if event_type in (1, 7) else ''
    generation = 1 if event_type in (1, 7) else 0
    return {
        'accepted': True,
        'actual_stamp_ns': 0,
        'affected_message_count': 0,
        'arm_commit_stamp_ns': stamp_ns if event_type == 7 else 0,
        'arm_margin_ns': 0,
        'bound_goal_uuid': goal_uuid if event_type == 7 else '00000000-0000-0000-0000-000000000000',
        'bound_t0_ns': 200_000_000 if event_type == 7 else 0,
        'committed_fault_count': 0,
        'committed_generation': generation,
        'committed_schedule_hash': committed_hash,
        'configured_activation_stamp_ns': 0,
        'configured_deactivation_stamp_ns': 0,
        'detail': 'test fixture',
        'event_sequence': sequence,
        'event_type': event_type,
        'fault_id': '',
        'header_stamp_ns': stamp_ns,
        'input_sequence': 0,
        'mode': 0,
        'raw_input_count': 0,
        'replayed': False,
        'requested_fault_count': 0,
        'requested_generation': generation,
        'requested_goal_uuid': (
            goal_uuid if event_type == 7 else '00000000-0000-0000-0000-000000000000'
        ),
        'requested_schedule_hash': committed_hash,
        'requested_t0_ns': 200_000_000 if event_type == 7 else 0,
        'schema_version': 2,
        'seed': 0,
        'state_after': 2 if event_type == 7 else 0,
        'state_before': 1 if event_type == 7 else 0,
        'target': 0,
        'validated_output_count': 0,
    }


def _scenario_result(identity: dict[str, Any], measurements: dict[str, Any]) -> dict[str, Any]:
    buffer = {
        'capacity': 16,
        'first_overflow_sequence': None,
        'first_overflow_stamp_ns': None,
        'ingress_count': 1,
        'invalid_count': 0,
        'overflow': False,
        'overflow_count': 0,
        'retained_count': 1,
    }
    return {
        'binding': {
            'accepted_goal_stamp_ns': measurements['accepted_goal_stamp_ns'],
            'feedback_trace': [],
            'goal_uuid': measurements['accepted_goal_uuid'],
            'ready_sim_stamp_ns': 1,
            'status_trace': [],
            'terminal_status': measurements['goal_status_code'],
        },
        'cleanup': {
            'actor_absent': True,
            'delete_attempt_count': 0,
            'delete_success': True,
            'proof': {},
            'required': False,
        },
        'configuration': {},
        'identity': {
            key: identity[key]
            for key in (
                'candidate_id',
                'repetition_index',
                'run_id',
                'scenario_id',
                'scenario_name',
                'scenario_sha256',
                'suite_index',
            )
        },
        'interaction': {'kind': 'none'},
        'producer': 'robotest_scenarios/scenario_controller',
        'quality': {
            'all_buffers_bounded': True,
            'buffers': {
                name: copy.deepcopy(buffer)
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
            'reason': 'completed',
        },
    }


def _orchestrator(identity: dict[str, Any]) -> dict[str, Any]:
    return {
        'artifacts': {
            'prerequisite_artifact_count': 6,
            'prerequisite_checksums_verified': True,
            'prerequisite_manifest_sha256': 'a' * 64,
            'prerequisite_maximum_file_bytes': 100_000,
            'prerequisite_total_bytes': 200_000,
            'prerequisites_finalized': True,
            'prerequisites_within_caps': True,
            'runtime_stderr_bytes': 100,
            'runtime_stdout_bytes': 100,
        },
        'cleanup': {
            'all_owned_processes_exited': True,
            'discovery_endpoints_gone': True,
            'no_orphans': True,
        },
        'execution': {
            'command': 'scripts/verify_phase3.sh',
            'exit_code': 0,
            'wall_duration_s': 30.0,
            'wall_timed_out': False,
            'wall_timeout_s': 300.0,
            'working_directory': '/home/hasan/robotest-lab',
        },
        'gates': {
            'graph_contract_pass': True,
            'namespace_isolation_pass': True,
            'qos_contract_pass': True,
            'source_install_binding_pass': True,
            'validation_autonomy_isolation_pass': True,
        },
        'git': {
            'dirty': False,
            'end_head': identity['git_sha'],
            'start_head': identity['git_sha'],
            'status_porcelain': '',
        },
        'identity': {
            **{
                key: identity[key]
                for key in (
                    'candidate_id',
                    'repetition_index',
                    'run_id',
                    'scenario_id',
                    'scenario_sha256',
                    'suite_index',
                )
            },
            'gz_partition': identity['gz_partition'],
            'ros_domain_id': identity['ros_domain_id'],
        },
        'process': {
            'cold_stack': True,
            'cpu_affinity': [0, 1, 2, 3, 4, 5],
            'fresh_fault_generation': True,
            'fresh_localization': True,
            'new_process_group': True,
            'partition_unused_before_start': True,
            'previous_trial_gone': True,
            'ros_domain_unused_before_start': True,
        },
        'resources': {
            'cpu_percent_mean': 100.0,
            'cpu_percent_p95': 200.0,
            'cpu_percent_peak': 300.0,
            'missing_sample_count': 0,
            'oom_kill': False,
            'overflow_free': True,
            'peak_rss_sum_bytes': 1_000_000_000,
            'pid_reuse_detected': False,
            'sample_count': 10,
            'sampler_started_before_launch': True,
            'sampler_stopped_after_shutdown': True,
            'wsl_peak_memory_bytes': 2_000_000_000,
            'wsl_peak_swap_bytes': 0,
        },
        'schema_version': 1,
        'source_binding': {
            'collector_configuration_sha256': '1' * 64,
            'install_end_sha256': 'b' * 64,
            'install_start_sha256': 'b' * 64,
            'install_unchanged': True,
            'metrics_contract_sha256': '2' * 64,
            'source_configuration_sha256': '3' * 64,
            'source_end_sha256': 'c' * 64,
            'source_install_match': True,
            'source_start_sha256': 'c' * 64,
            'source_unchanged': True,
            'target_set_sha256': '4' * 64,
        },
    }


def collision_fixture() -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    repository = Path(__file__).resolve().parents[3]
    rendered_manifest = yaml.safe_load(
        (repository / 'config/collision-coverage.yaml').read_text(encoding='utf-8')
    )
    control_configuration = contact_control_configuration()
    control_configuration_sha256 = contact_control_configuration_sha256()
    wall_asset_sha256 = file_sha256(
        repository / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
    )
    manifest = {
        'bridge_sha256': '1' * 64,
        'contact_configuration_sha256': rendered_manifest['contact_configuration_sha256'],
        'covered_collisions': ['robotest::base_link::base_collision'],
        'rendered_sdf_sha256': '4' * 64,
        'rendered_robot_collisions': ['robotest::base_link::base_collision'],
        'robot_description_sha256': '7' * 64,
        'robot_collisions': [
            {
                'name': 'robotest::base_link::base_collision',
                'role': 'chassis',
                'source': 'all_robot_contacts',
            }
        ],
        'robot_model': 'robotest',
        'support_pairs': [],
        'world_source_sha256': '8' * 64,
    }
    manifest['manifest_sha256'] = canonical_sha256(manifest)
    expected_pair = [
        'phase3_contact_control_wall::link::collision',
        'robotest::base_link::base_collision',
    ]
    fixture = {
        'control': control_configuration,
        'entity': {
            'asset_sha256': wall_asset_sha256,
            'name': 'phase3_contact_control_wall',
            'pose': {
                'x': CONTROL_WALL_POSE[0],
                'y': CONTROL_WALL_POSE[1],
                'yaw': CONTROL_WALL_POSE[3],
                'z': CONTROL_WALL_POSE[2],
            },
        },
        'expected_pair': expected_pair,
        'fixture_id': 'collision_positive_control',
        'robot_start': {
            'x': CONTROL_ROBOT_START[0],
            'y': CONTROL_ROBOT_START[1],
            'yaw': CONTROL_ROBOT_START[2],
        },
        'schema_version': 1,
    }
    fixture_sha256 = canonical_sha256(fixture)

    def positive_buffer(capacity: int, count: int) -> dict[str, Any]:
        return {
            'accepted_count': count,
            'capacity': capacity,
            'first_overflow_sequence': None,
            'first_overflow_stamp_ns': None,
            'ingress_count': count,
            'invalid_count': 0,
            'overflow': False,
            'overflow_count': 0,
            'retained_count': count,
        }

    positive = {
        'cleanup': {
            'actor_absent': True,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': {
                'kind': 'successful_delete_response_and_pose_quiet_interval',
                'pose_source_publishers_after': 1,
                'pose_source_publishers_before': 1,
                'post_delete_pose_count': 0,
                'post_delete_pose_source_heartbeat_count': 1,
                'post_delete_pose_source_latest_sim_stamp_ns': 3_900_000_000,
                'quiet_until_sim_stamp_ns': 3_900_000_000,
                'request_sequence': 14,
                'request_stamp_ns': 3_500_000_000,
                'response_sequence': 15,
                'response_stamp_ns': 3_650_000_000,
            },
            'required': True,
        },
        'configuration': {
            'control_configuration': control_configuration,
            'control_configuration_sha256': control_configuration_sha256,
            'coverage_manifest_path': str(repository / 'config/collision-coverage.yaml'),
            'coverage_manifest_provenance': {
                'bridge_sha256': manifest['bridge_sha256'],
                'contact_configuration_sha256': manifest['contact_configuration_sha256'],
                'coverage_manifest_sha256': manifest['manifest_sha256'],
                'rendered_sdf_sha256': manifest['rendered_sdf_sha256'],
                'robot_description_sha256': manifest['robot_description_sha256'],
                'world_source_sha256': manifest['world_source_sha256'],
            },
            'coverage_manifest_sha256': manifest['manifest_sha256'],
            'expected_pair': expected_pair,
            'fixture': fixture,
            'fixture_sha256': fixture_sha256,
            'service_timeout_s': 2.0,
            'source_binding': contact_source_binding(),
            'wall_asset_sha256': wall_asset_sha256,
            'wall_timeout_s': 30.0,
        },
        'control': {
            'command_trace': [
                {
                    'angular_z': 0.0,
                    'collector_sequence': 5,
                    'linear_x': 0.05,
                    'phase': 'FORWARD',
                    'sim_stamp_ns': 1_000_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 8,
                    'linear_x': 0.0,
                    'phase': 'CONTACT_STOP',
                    'sim_stamp_ns': 2_000_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 9,
                    'linear_x': 0.0,
                    'phase': 'HOLD',
                    'sim_stamp_ns': 2_100_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 10,
                    'linear_x': -0.05,
                    'phase': 'REVERSE',
                    'sim_stamp_ns': 2_250_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 12,
                    'linear_x': 0.0,
                    'phase': 'FINAL_ZERO',
                    'sim_stamp_ns': 3_250_000_000,
                },
            ],
            'contact': {
                'active_counterpart_count': 0,
                'classified_record_count': 1,
                'counterpart_tracker_count': 1,
                'episodes': [
                    {
                        'counterpart_model': 'phase3_contact_control_wall',
                        'end_stamp_ns': 2_250_000_000,
                        'normalized_pairs': [expected_pair],
                        'sample_count': 1,
                        'start_stamp_ns': 2_000_000_000,
                    }
                ],
                'exact_pair_raw_count': 1,
                'expected_pair': expected_pair,
                'first_qualifying_contact': {
                    'clock_delivery_offset_ns': 0,
                    'collector_sequence': 7,
                    'normalized_pair': expected_pair,
                    'observed_sim_stamp_ns': 2_000_000_000,
                    'sim_stamp_ns': 2_000_000_000,
                },
                'raw_contact_record_count': 1,
                'records': [
                    {
                        'collector_sequence': 7,
                        'counterpart_collision': expected_pair[0],
                        'counterpart_model': 'phase3_contact_control_wall',
                        'disposition': 'counted',
                        'normalized_pair': expected_pair,
                        'robot_collision': expected_pair[1],
                        'sim_stamp_ns': 2_000_000_000,
                    }
                ],
            },
            'observed_robot_start': {
                'alignment_error_ns': 100_000_000,
                'collector_sequence': 4,
                'position_error_m': 0.0,
                'sim_stamp_ns': 900_000_000,
                'x': 0.0,
                'y': -3.5,
                'yaw': 0.0,
                'yaw_error_rad': 0.0,
            },
            'criteria': {
                name: True
                for name in (
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
            },
            'metrics_owned': {
                'collector_reconciliation_and_process_group_termination': {
                    'owner': 'robotest_metrics_and_orchestrator',
                    'required': True,
                    'scenario_controller_value': None,
                }
            },
            'setup': {
                'observed_robot_start': {
                    'alignment_error_ns': 100_000_000,
                    'collector_sequence': 4,
                    'position_error_m': 0.0,
                    'sim_stamp_ns': 900_000_000,
                    'x': 0.0,
                    'y': -3.5,
                    'yaw': 0.0,
                    'yaw_error_rad': 0.0,
                },
                'observed_wall': {
                    'collector_sequence': 3,
                    'position_error_m': 0.0,
                    'stamp_ns': 300_000_000,
                    'x': CONTROL_WALL_POSE[0],
                    'y': CONTROL_WALL_POSE[1],
                    'yaw': CONTROL_WALL_POSE[3],
                    'yaw_error_rad': 0.0,
                    'z': CONTROL_WALL_POSE[2],
                },
                'spawn': {
                    'attempt_count': 1,
                    'error': None,
                    'request_sequence': 1,
                    'request_stamp_ns': 100_000_000,
                    'response_sequence': 2,
                    'response_stamp_ns': 200_000_000,
                    'success': True,
                },
            },
            'timeline': {
                'control_started_stamp_ns': 1_000_000_000,
                'final_zero_stamp_ns': 3_250_000_000,
                'hold_complete_stamp_ns': 2_250_000_000,
                'release_complete_stamp_ns': 3_500_000_000,
                'release_contact_message_end_count': 3,
                'release_contact_message_start_count': 2,
                'release_required_through_stamp_ns': 3_500_000_000,
                'reverse_start_stamp_ns': 2_250_000_000,
                'stop_command_stamp_ns': 2_000_000_000,
                'stop_latency_ns': 0,
            },
        },
        'identity': {
            'fixture_id': 'collision_positive_control',
            'run_id': 'positive-control-1',
            'scenario_sha256': fixture_sha256,
        },
        'producer': 'robotest_scenarios/contact_control_driver',
        'quality': {
            'all_buffers_bounded': True,
            'buffers': {
                'actor_state': positive_buffer(1_024, 1),
                'command': positive_buffer(4_096, 5),
                'contact_records': positive_buffer(32_768, 1),
                'contact_summaries': positive_buffer(8_192, 3),
                'ground_truth': positive_buffer(8_192, 1),
            },
            'cmd_vel_publisher_count': 1,
            'collision_monitor_absent': True,
            'clock': {
                'first_stamp_ns': 0,
                'latest_stamp_ns': 3_900_000_000,
                'max_gap_ns': 100_000_000,
                'regression_count': 0,
                'sample_count': 36,
            },
            'contact_message_heartbeat': {
                'first_stamp_ns': 1_000_000_000,
                'latest_stamp_ns': 3_500_000_000,
                'max_gap_ns': 500_000_000,
                'message_count': 3,
            },
            'forbidden_nodes': [],
            'nav2_absent': True,
            'overflow_free': True,
            'protocol_error_count': 0,
            'raw_contact_stream': positive_buffer(32_768, 1),
            'relative_project_names': True,
            'sole_cmd_vel_publisher': True,
            'source_publisher_counts': {
                'contacts': 1,
                'entity_pose': 1,
                'ground_truth': 1,
            },
            'source_streams_live': True,
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
    provenance = {
        'bridge_sha256': manifest['bridge_sha256'],
        'collector_configuration_sha256': '1' * 64,
        'contact_configuration_sha256': manifest['contact_configuration_sha256'],
        'coverage_manifest_sha256': manifest['manifest_sha256'],
        'rendered_sdf_sha256': manifest['rendered_sdf_sha256'],
        'robot_description_sha256': manifest['robot_description_sha256'],
        'world_source_sha256': manifest['world_source_sha256'],
    }
    binding = {
        'benchmark_provenance': copy.deepcopy(provenance),
        'positive_control_external_quality': {
            'checksum_verified': True,
            'collector_capture_sha256': 'e' * 64,
            'collector_reconciled': True,
            'owned_process_group_shutdown': True,
        },
        'positive_control_json_sha256': canonical_sha256(positive),
        'positive_control_provenance': provenance,
        'positive_control_run_id': positive['identity']['run_id'],
        'positive_control_scenario_sha256': positive['identity']['scenario_sha256'],
    }
    return manifest, positive, binding


def complete_request() -> dict[str, Any]:
    core = CollectorCore()
    for index in range(11):
        stamp = index * 200_000_000
        x_m = index * 0.2
        core.record(
            'ground_truth',
            {'frame_id': 'world', 'stamp_ns': stamp, 'x_m': x_m, 'y_m': 0.0, 'yaw_rad': 0.0},
        )
        core.record(
            'tf_map_odom',
            {'frame_id': 'map', 'stamp_ns': stamp, 'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        )
        core.record(
            'tf_odom_base_footprint',
            {'frame_id': 'odom', 'stamp_ns': stamp, 'x_m': x_m, 'y_m': 0.0, 'yaw_rad': 0.0},
        )
    core.record_state_transition(
        kind='waypoint_feedback',
        subject='goal-1',
        value=0,
        stamp_ns=200_000_000,
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [{'x_m': 0.2, 'y_m': 0.0}, {'x_m': 1.0, 'y_m': 0.0}],
            'stamp_ns': 300_000_000,
        }
    )
    core.record_state_transition(
        kind='waypoint_feedback',
        subject='goal-1',
        value=1,
        stamp_ns=1_000_000_000,
    )
    core.record_plan(
        {
            'frame_id': 'map',
            'poses': [{'x_m': 1.0, 'y_m': 0.0}, {'x_m': 1.8, 'y_m': 0.0}],
            'stamp_ns': 1_100_000_000,
        }
    )
    core.record('contacts', {'contacts': [], 'stamp_ns': 400_000_000})
    core.record('contacts', {'contacts': [], 'stamp_ns': 1_800_000_000})
    for sequence, stamp in enumerate((200_000_000, 1_000_000_000, 1_800_000_000)):
        core.record(
            'world_stats',
            {
                'paused': False,
                'reported_real_time_factor': 1.0,
                'sim_stamp_ns': stamp,
                'stamp_ns': stamp,
                'steady_wall_ns': sequence * 800_000_000,
            },
        )
    for stamp in (0, 1_000_000_000, 2_000_000_000):
        core.observe_clock(stamp)
    fault_events = [
        _fault_event(1, 3, 100_000_000),
        _fault_event(2, 1, 120_000_000),
        _fault_event(3, 7, 200_000_000),
        _fault_event(4, 3, 1_900_000_000),
    ]
    for event in fault_events:
        core.record('fault_events', {**event, 'stamp_ns': event['header_stamp_ns']})
    capture = core.snapshot()
    capture['capture_schema_version'] = 1
    capture['started_steady_wall_ns'] = 0
    capture['finished_steady_wall_ns'] = 2_000_000_000
    capture['stop_reason'] = 'stop_file'
    manifest, positive, binding = collision_fixture()
    identity = {
        'candidate_id': 'candidate-1',
        'cold_stack': True,
        'git_dirty': False,
        'git_sha': 'd' * 40,
        'gz_partition': 'robotest_p3_candidate-1_00',
        'repetition_index': 0,
        'ros_domain_id': 100,
        'run_id': 'run-1',
        'scenario_id': 1,
        'scenario_index': 1,
        'scenario_name': 'baseline_navigation',
        'scenario_sha256': SCENARIO_SHA256,
        'suite_index': 0,
    }
    mission_measurements = {
        'accepted_goal_stamp_ns': 200_000_000,
        'accepted_goal_uuid': 'goal-1',
        'completed_waypoint_count': 2,
        'goal_status': 'SUCCEEDED',
        'goal_status_code': 4,
        'missed_waypoint_count': 0,
        'terminal_action_stamp_ns': 1_800_000_000,
    }
    mission_result = {
        'events': [],
        'fault': {
            'control': {
                'arm_commit_stamp_ns': 200_000_000,
                'arm_margin_ns': 0,
                'arm_replayed': False,
                'event_count': len(fault_events),
                'event_trace_capacity': 512,
                'event_trace_overflow': False,
                'event_trace_overflow_count': 0,
                'generation': 1,
                'preload_replayed': False,
                'protocol_status': 'RESET_CONFIRMED_AFTER_GOAL',
                'reset_after_goal': True,
                'reset_before_goal': True,
            },
            'events': fault_events,
            'schedule': {
                'canonical_json': '{"schema_version":1,"faults":[]}',
                'fault_count': 0,
                'faults': [],
                'schema_version': 1,
                'sha256': EMPTY_FAULT_SCHEDULE_SHA256,
            },
        },
        'identity': {
            'candidate_id': identity['candidate_id'],
            'fault_schedule_hash': EMPTY_FAULT_SCHEDULE_SHA256,
            'mission_sha256': SCENARIO_SHA256,
            'repetition_index': identity['repetition_index'],
            'run_id': identity['run_id'],
            'scenario_id': identity['scenario_id'],
            'suite_index': identity['suite_index'],
        },
        'measurements': mission_measurements,
        'quality': {
            'cancellation_requested': False,
            'deadline_kind': None,
            'event_trace_overflow': False,
            'fault_event_overflow': False,
            'feedback_trace_overflow': False,
            'goal_accepted': True,
            'invalid_feedback_count': 0,
            'terminal_result_observed': True,
        },
        'targets': {
            'waypoint_count': 2,
            'waypoints': [{'x_m': 1.0, 'y_m': 0.0}, {'x_m': 1.8, 'y_m': 0.0}],
        },
        'verdict': {
            'exit_code': 0,
            'expected_outcome_met': True,
            'phase2_action_integration_status': 'PASS',
        },
    }
    scenario_result = _scenario_result(identity, mission_measurements)
    return {
        'capture': capture,
        'collision': {
            'benchmark_binding': binding,
            'coverage_manifest': manifest,
            'drain_completed_stamp_ns': 2_050_000_000,
            'positive_control': positive,
        },
        'fault': {'kind': 'none'},
        'identity': identity,
        'mission': {
            'artifact_sha256': canonical_sha256(mission_result),
            'result': mission_result,
        },
        'orchestrator': _orchestrator(identity),
        'scenario': {
            'artifact_sha256': canonical_sha256(scenario_result),
            'result': scenario_result,
        },
        'targets': {
            'acceptance': {
                'measurements.collision_count': {'maximum': 0},
                'measurements.rtf_p5': {'minimum': 0.5},
            },
            'required_metrics': [
                'measurements.actual_path_length_m',
                'measurements.collision_count',
                'measurements.initial_planned_path_length_m',
                'measurements.localization_position_rmse_m',
                'measurements.path_efficiency',
                'measurements.replan_count',
                'measurements.rtf_median',
                'measurements.peak_rss_sum_bytes',
            ],
            'collector_configuration_sha256': '1' * 64,
            'collision_coverage_manifest_sha256': manifest['manifest_sha256'],
            'fault_schedule_sha256': EMPTY_FAULT_SCHEDULE_SHA256,
            'metrics_contract_sha256': '2' * 64,
            'positive_control_json_sha256': binding['positive_control_json_sha256'],
            'scenario_sha256': SCENARIO_SHA256,
            'source_configuration_sha256': '3' * 64,
            'target_set_sha256': '4' * 64,
            'world_to_map': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        },
    }


@pytest.fixture
def analysis_request() -> dict[str, Any]:
    return copy.deepcopy(complete_request())
