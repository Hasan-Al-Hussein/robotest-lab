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
from robotest_metrics.artifacts import canonical_sha256
from robotest_metrics.collector import CollectorCore
from robotest_scenarios.constants import CONTROL_ROBOT_START, CONTROL_WALL_POSE
from robotest_scenarios.contact_evidence import (
    EXPECTED_CONTACT_GATE_SOURCE_PATHS,
    EXPECTED_CONTACT_STREAM_POLICY,
)
from robotest_scenarios.provenance import (
    configuration_sha256,
    contact_control_configuration,
    contact_control_configuration_sha256,
    contact_source_binding,
    controller_configuration,
    file_sha256,
    source_binding,
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
            'delete_success': None,
            'proof': {'kind': 'scenario_declares_no_actor'},
            'required': False,
        },
        'configuration': {
            'actor_asset_sha256': None,
            'controller_configuration': controller_configuration(),
            'controller_configuration_sha256': configuration_sha256(),
            'service_timeout_s': 2.0,
            'source_binding': source_binding(),
            'wall_timeout_s': 300.0,
        },
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
            'mission_graph_sha256': 'b' * 64,
            'pre_mission_graph_sha256': 'c' * 64,
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
            'contact_aggregator_binary': {
                'build_embedded_source_inventory_match': True,
                'build_embedded_source_inventory_sha256': '5' * 64,
                'build_elf_build_id': 'b' * 40,
                'build_install_build_id_match': True,
                'build_install_embedded_source_inventory_match': True,
                'build_install_samefile': True,
                'build_install_sha256_match': True,
                'build_path': ('build/robotest_sim/librobotest_contact_aggregator_system.so'),
                'build_regular_file': True,
                'build_sha256': '7' * 64,
                'installed_declared_is_symlink': True,
                'installed_declared_path': (
                    'install/robotest_sim/lib/robotest_sim/librobotest_contact_aggregator_system.so'
                ),
                'installed_embedded_source_inventory_match': True,
                'installed_embedded_source_inventory_sha256': '5' * 64,
                'installed_elf_build_id': 'b' * 40,
                'installed_path': ('build/robotest_sim/librobotest_contact_aggregator_system.so'),
                'installed_regular_file': True,
                'installed_sha256': '7' * 64,
                'package': 'robotest_sim',
                'schema_version': 1,
                'source_inventory_sha256': '5' * 64,
            },
            'contact_gate_binary': {
                'build_embedded_source_inventory_match': True,
                'build_embedded_source_inventory_sha256': '5' * 64,
                'build_elf_build_id': 'a' * 40,
                'build_install_build_id_match': True,
                'build_install_samefile': True,
                'build_install_sha256_match': True,
                'build_path': 'build/robotest_sim/contact_stream_gate',
                'build_regular_executable': True,
                'build_sha256': '6' * 64,
                'installed_declared_is_symlink': True,
                'installed_declared_path': (
                    'install/robotest_sim/lib/robotest_sim/contact_stream_gate'
                ),
                'installed_declared_samefile': True,
                'installed_embedded_source_inventory_match': True,
                'installed_embedded_source_inventory_sha256': '5' * 64,
                'installed_elf_build_id': 'a' * 40,
                'installed_path': 'build/robotest_sim/contact_stream_gate',
                'installed_regular_executable': True,
                'installed_sha256': '6' * 64,
                'package': 'robotest_sim',
                'schema_version': 1,
                'source_inventory_sha256': '5' * 64,
            },
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
    control_configuration = contact_control_configuration()
    control_configuration_sha256 = contact_control_configuration_sha256()
    wall_asset_sha256 = file_sha256(
        repository / 'src/robotest_sim/models/phase3_contact_control_wall.sdf'
    )
    chassis_collision = 'robotest::base_link::base_collision'
    support_collision = 'robotest::left_wheel_link::left_wheel_collision'
    gate_source_inventory = {
        'schema_version': 1,
        'sources': [
            {'path': path, 'sha256': file_sha256(repository / path)}
            for path in EXPECTED_CONTACT_GATE_SOURCE_PATHS
        ],
    }
    contact_stream = {
        'gate': {
            'executable': 'contact_stream_gate',
            'launch_sha256': 'a' * 64,
            'package': 'robotest_sim',
            'source_inventory': gate_source_inventory,
            'source_inventory_sha256': canonical_sha256(gate_source_inventory),
        },
        'policy': copy.deepcopy(EXPECTED_CONTACT_STREAM_POLICY),
        'policy_sha256': canonical_sha256(EXPECTED_CONTACT_STREAM_POLICY),
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
        'schema_version': 1,
        'topics': {
            'gazebo_raw': '/robotest/internal/contact_aggregate',
            'private_raw_ros': '/robotest/internal/raw_contacts',
            'public_ros': '/robotest/validation/contacts',
        },
    }
    manifest = {
        'bridge_sha256': '1' * 64,
        'contact_configuration_sha256': '2' * 64,
        'contact_stream': contact_stream,
        'contact_topic': '/robotest/validation/contacts',
        'covered_collisions': [chassis_collision, support_collision],
        'rendered_sdf_sha256': '4' * 64,
        'rendered_robot_collisions': [chassis_collision, support_collision],
        'robot_description_sha256': '7' * 64,
        'robot_collisions': [
            {
                'name': chassis_collision,
                'role': 'chassis',
                'source': '/robotest/validation/contacts',
            },
            {
                'name': support_collision,
                'role': 'left_wheel',
                'source': '/robotest/validation/contacts',
            },
        ],
        'robot_model': 'robotest',
        'schema_version': 3,
        'support_pairs': [
            {
                'environment_collision': 'ground_plane::ground_link::ground_collision',
                'robot_collision': support_collision,
            }
        ],
        'world_source_sha256': '8' * 64,
    }
    manifest['manifest_sha256'] = canonical_sha256(manifest)
    expected_pair = [
        'phase3_contact_control_wall::link::collision',
        chassis_collision,
    ]
    support_pair = sorted([support_collision, 'ground_plane::ground_link::ground_collision'])
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

    snapshot_layout = (
        (4, 5, None, 900_000_000),
        (8, 9, 10, 1_100_000_000),
        (13, 14, 15, 1_300_000_000),
        (17, 18, None, 1_500_000_000),
        (19, 20, None, 1_700_000_000),
        (21, 22, None, 1_900_000_000),
        (23, 24, None, 2_100_000_000),
        (25, 26, None, 2_300_000_000),
        (28, 29, None, 2_500_000_000),
        (30, 31, None, 2_700_000_000),
    )
    snapshot_records: list[dict[str, Any]] = []
    snapshots: list[dict[str, Any]] = []
    for summary_sequence, support_sequence, wall_sequence, stamp_ns in snapshot_layout:
        snapshot_records.append(
            {
                'collector_sequence': support_sequence,
                'counterpart_collision': None,
                'counterpart_model': None,
                'disposition': 'support_ground_excluded',
                'normalized_pair': support_pair,
                'robot_collision': None,
                'sim_stamp_ns': stamp_ns,
                'snapshot_sequence': summary_sequence,
            }
        )
        counted_records: list[dict[str, Any]] = []
        if wall_sequence is not None:
            snapshot_records.append(
                {
                    'collector_sequence': wall_sequence,
                    'counterpart_collision': expected_pair[0],
                    'counterpart_model': 'phase3_contact_control_wall',
                    'disposition': 'counted',
                    'normalized_pair': expected_pair,
                    'robot_collision': expected_pair[1],
                    'sim_stamp_ns': stamp_ns,
                    'snapshot_sequence': summary_sequence,
                }
            )
            counted_records.append(
                {
                    'counterpart_model': 'phase3_contact_control_wall',
                    'normalized_pair': expected_pair,
                    'record_sequence': wall_sequence,
                    'snapshot_sequence': summary_sequence,
                }
            )
        snapshots.append(
            {
                'classified_count': len(counted_records),
                'collector_sequence': summary_sequence,
                'counted_snapshot_records': counted_records,
                'delivery_clock_offset_ns': 0,
                'delivery_clock_stamp_ns': stamp_ns,
                'exact_pair_count': len(counted_records),
                'sim_stamp_ns': stamp_ns,
                'snapshot_record_count': 1 + len(counted_records),
            }
        )

    def graph_endpoint(node_fqn: str, depth: int, gid_byte: str) -> dict[str, Any]:
        return {
            'endpoint_gid': gid_byte * 16,
            'node_fqn': node_fqn,
            'qos': {
                'depth': depth,
                'durability': 'VOLATILE',
                'history': 'KEEP_LAST',
                'reliability': 'RELIABLE',
            },
            'qos_status': {
                'depth_matches_or_unknown': True,
                'durability_volatile': True,
                'history_keep_last_or_unknown': True,
                'reliability_reliable': True,
            },
            'topic_type': 'ros_gz_interfaces/msg/Contacts',
        }

    graph_snapshot = {
        'private_raw_publishers': [graph_endpoint('/robotest/parameter_bridge', 64, '1a')],
        'private_raw_subscribers': [graph_endpoint('/robotest/contact_stream_gate', 64, '2b')],
        'public_snapshot_publishers': [graph_endpoint('/robotest/contact_stream_gate', 10, '3c')],
        'topics': {
            'private_raw_contact_topic': '/robotest/internal/raw_contacts',
            'public_contact_snapshot_topic': '/robotest/validation/contacts',
        },
    }
    graph_snapshot_sha256 = canonical_sha256(graph_snapshot)

    arm_protocol = control_configuration['arm_protocol']
    arm_request = {
        'action': arm_protocol['action'],
        'arm_protocol_sha256': canonical_sha256(arm_protocol),
        'arm_requested_steady_ns': 4_000_000,
        'producer': arm_protocol['request_producer'],
        'ready_sha256': 'd' * 64,
        'run_id': 'positive-control-1',
        'runtime_gate_sha256': 'e' * 64,
        'schema_version': arm_protocol['schema_version'],
    }
    arm_request_sha256 = canonical_sha256(arm_request)
    armed_acknowledgment = {
        'arm_observed_clock_sample_count': 10,
        'arm_observed_sim_stamp_ns': 800_000_000,
        'arm_observed_steady_ns': 5_000_000,
        'arm_protocol_sha256': canonical_sha256(arm_protocol),
        'arm_request_sha256': arm_request_sha256,
        'arm_requested_steady_ns': arm_request['arm_requested_steady_ns'],
        'armed_clock_sample_count': 11,
        'armed_sim_stamp_ns': 900_000_000,
        'armed_steady_ns': 6_000_000,
        'command_delivery_probe': {
            'collector_progress_observed_steady_ns': 5_400_000,
            'collector_progress_sha256': 'f' * 64,
            'collector_progress_stamp_ns': 850_000_000,
            'matched_subscription_count': 2,
            'match_observed_steady_ns': 5_100_000,
            'probe_publish_returned_steady_ns': 5_300_000,
            'probe_publish_started_steady_ns': 5_200_000,
            'probe_sim_stamp_ns': 850_000_000,
            'required_subscription_count': 2,
        },
        'producer': arm_protocol['ack_producer'],
        'ready_sha256': arm_request['ready_sha256'],
        'run_id': arm_request['run_id'],
        'runtime_gate_sha256': arm_request['runtime_gate_sha256'],
        'schema_version': arm_protocol['schema_version'],
    }

    positive = {
        'cleanup': {
            'actor_absent': True,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': {
                'kind': 'successful_blocking_delete_and_bounded_pose_absence',
                'dds_drain_complete_steady_ns': 4_050_000_000,
                'dds_drain_grace_ns': 50_000_000,
                'dds_drain_spin_count': 3,
                'dds_drain_start_steady_ns': 4_000_000_000,
                'observation_deadline_sim_stamp_ns': 3_790_000_000,
                'pose_source_publishers_after': 1,
                'pose_source_publishers_before': 1,
                'post_delete_pose_count': 0,
                'post_delete_pose_first_sequence': None,
                'post_delete_pose_first_sim_stamp_ns': None,
                'post_delete_pose_latest_sequence': None,
                'post_delete_pose_latest_sim_stamp_ns': None,
                'post_delete_pose_source_heartbeat_count': 2,
                'post_delete_pose_source_latest_sequence': 35,
                'post_delete_pose_source_latest_sim_stamp_ns': 3_050_000_000,
                'quiet_restart_count': 0,
                'quiet_start_sim_stamp_ns': 2_800_000_000,
                'quiet_until_sim_stamp_ns': 3_050_000_000,
                'request_sequence': 32,
                'request_stamp_ns': 2_710_000_000,
                'response_sequence': 33,
                'response_stamp_ns': 2_790_000_000,
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
            'arm': {
                'acknowledgment': armed_acknowledgment,
                'acknowledgment_sha256': canonical_sha256(armed_acknowledgment),
                'first_nonzero_publish_returned_steady_ns': 8_000_000,
                'first_nonzero_publish_started_steady_ns': 7_000_000,
                'request': arm_request,
                'request_sha256': arm_request_sha256,
            },
            'command_trace': [
                {
                    'angular_z': 0.0,
                    'collector_sequence': 7,
                    'linear_x': 0.05,
                    'phase': 'FORWARD',
                    'sim_stamp_ns': 1_000_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 11,
                    'linear_x': 0.0,
                    'phase': 'CONTACT_STOP',
                    'sim_stamp_ns': 1_100_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 12,
                    'linear_x': 0.0,
                    'phase': 'HOLD',
                    'sim_stamp_ns': 1_200_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 16,
                    'linear_x': -0.05,
                    'phase': 'REVERSE',
                    'sim_stamp_ns': 1_350_000_000,
                },
                {
                    'angular_z': 0.0,
                    'collector_sequence': 27,
                    'linear_x': 0.0,
                    'phase': 'FINAL_ZERO',
                    'sim_stamp_ns': 2_350_000_000,
                },
            ],
            'contact': {
                'active_counterpart_count': 0,
                'classified_record_count': 2,
                'contact_clock_bracket': {
                    'clock_stamp_ns': 2_710_000_000,
                    'lag_ns': 10_000_000,
                    'limit_ns': 220_000_000,
                    'snapshot_stamp_ns': 2_700_000_000,
                },
                'counterpart_tracker_count': 1,
                'episodes': [
                    {
                        'counterpart_model': 'phase3_contact_control_wall',
                        'end_stamp_ns': 1_500_000_000,
                        'normalized_pairs': [expected_pair],
                        'snapshot_record_count': 2,
                        'start_stamp_ns': 1_100_000_000,
                    }
                ],
                'exact_pair_snapshot_record_count': 2,
                'expected_pair': expected_pair,
                'first_qualifying_contact': {
                    'callback_clock_offset_ns': 0,
                    'callback_clock_stamp_ns': 1_100_000_000,
                    'collector_sequence': 10,
                    'normalized_pair': expected_pair,
                    'sim_stamp_ns': 1_100_000_000,
                    'stop_latency_clock_stamp_ns': 1_100_000_000,
                    'stop_latency_upper_bound_ns': 0,
                },
                'release_snapshot': {
                    'collector_sequence': 30,
                    'sim_stamp_ns': 2_700_000_000,
                },
                'snapshot_contact_record_count': len(snapshot_records),
                'snapshot_records': snapshot_records,
                'snapshots': snapshots,
            },
            'observed_robot_start': {
                'alignment_error_ns': 100_000_000,
                'collector_sequence': 6,
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
                    'expected_contact_snapshot_observed',
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
                    'collector_sequence': 6,
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
                'control_started_steady_ns': 7_000_000,
                'control_started_stamp_ns': 1_000_000_000,
                'final_zero_stamp_ns': 2_350_000_000,
                'hold_complete_stamp_ns': 1_350_000_000,
                'release_complete_stamp_ns': 2_700_000_000,
                'release_contact_snapshot_end_count': 10,
                'release_contact_snapshot_start_count': 8,
                'release_observed_clock_stamp_ns': 2_710_000_000,
                'release_qualified_snapshot_stamp_ns': 2_700_000_000,
                'release_required_through_stamp_ns': 2_600_000_000,
                'reverse_start_stamp_ns': 1_350_000_000,
                'stop_command_stamp_ns': 1_100_000_000,
                'stop_latency_clock_stamp_ns': 1_100_000_000,
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
                'contact_snapshot_records': positive_buffer(32_768, len(snapshot_records)),
                'contact_snapshots': positive_buffer(8_192, len(snapshots)),
                'ground_truth': positive_buffer(8_192, 1),
            },
            'cmd_vel_publisher_count': 1,
            'collision_monitor_absent': True,
            'contact_graph_topology': {
                'audit_count': 10,
                'first_sha256': graph_snapshot_sha256,
                'first_snapshot': graph_snapshot,
                'last_sha256': graph_snapshot_sha256,
                'last_snapshot': copy.deepcopy(graph_snapshot),
            },
            'clock': {
                'first_stamp_ns': 0,
                'latest_stamp_ns': 3_050_000_000,
                'max_gap_ns': 100_000_000,
                'regression_count': 0,
                'sample_count': 36,
            },
            'public_contact_snapshot_heartbeat': {
                'first_stamp_ns': 900_000_000,
                'future_delivery_count': 0,
                'latest_stamp_ns': 2_700_000_000,
                'max_gap_ns': 200_000_000,
                'pre_clock_discard_count': 0,
                'snapshot_count': len(snapshots),
            },
            'forbidden_nodes': [],
            'nav2_absent': True,
            'overflow_free': True,
            'protocol_error_count': 0,
            'public_contact_snapshot_stream': positive_buffer(32_768, len(snapshot_records)),
            'relative_project_names': True,
            'sole_cmd_vel_publisher': True,
            'source_publisher_counts': {
                'contacts': 1,
                'entity_pose': 1,
                'ground_truth': 1,
            },
            'source_publisher_missing_observation_count': 0,
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
            'collector_command_progress_sha256': 'f' * 64,
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
    support_contact = {
        'collision1': 'robotest::left_wheel_link::left_wheel_collision',
        'collision2': 'ground_plane::ground_link::ground_collision',
        'maximum_normal_force_n': 5.0,
        'maximum_penetration_depth_m': 0.01,
    }
    for stamp in range(200_000_000, 2_200_000_001, 200_000_000):
        core.record(
            'contacts',
            {
                'contacts': [support_contact],
                'delivery_clock_offset_ns': 0,
                'delivery_clock_stamp_ns': stamp,
                'frame_id': '',
                'stamp_ns': stamp,
            },
        )
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
    for stamp in (0, 1_000_000_000, 2_000_000_000, 2_200_000_000):
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
            'waypoints': [
                {'x': 1.0, 'y': 0.0, 'yaw': 0.0},
                {'x': 1.8, 'y': 0.0, 'yaw': 0.0},
            ],
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
            'contact_drain': {
                'clock_first_stamp_ns': 0,
                'clock_latest_stamp_ns': 2_200_000_000,
                'clock_message_count': 4,
                'clock_minus_qualifying_contact_ns': 0,
                'clock_regression_count': 0,
                'contact_message_count': 11,
                'contact_record_count_violation_count': 0,
                'contact_stamp_duplicate_count': 0,
                'contact_stamp_regression_count': 0,
                'first_contact_stamp_ns': 200_000_000,
                'gate_node_present': True,
                'latest_contact_stamp_ns': 2_200_000_000,
                'limits': {
                    'heartbeat_period_ns': 200_000_000,
                    'max_clock_lag_ns': 220_000_000,
                    'max_public_gap_ns': 220_000_000,
                    'release_gap_ns': 250_000_000,
                },
                'maximum_contact_record_count': 1,
                'maximum_contact_source_gap_ns': 200_000_000,
                'minimum_contact_record_count': 1,
                'minimum_same_pair_set_interval_ns': 200_000_000,
                'producer': 'robotest_phase3/contact_drain_observer',
                'public_publisher_nodes': ['/robotest/contact_stream_gate'],
                'public_topic': '/robotest/validation/contacts',
                'qualifying_contact_snapshot_stamp_ns': 2_200_000_000,
                'same_pair_set_interval_violation_count': 0,
                'schema_version': 3,
                'target_stamp_ns': 2_050_000_000,
                'terminal_action_stamp_ns': 1_800_000_000,
            },
            'contact_drain_ack': {
                'artifact_sha256': 'f' * 64,
                'latest_retained_stamp_ns': 2_200_000_000,
                'producer': 'robotest_metrics/metrics_collector',
                'public_topic': '/robotest/validation/contacts',
                'retained_message_count': 11,
                'schema_version': 1,
            },
            'coverage_manifest': manifest,
            'drain_completed_stamp_ns': 2_200_000_000,
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
