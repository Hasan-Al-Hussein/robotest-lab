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

"""Installed result-schema shape tests."""

from pathlib import Path
from typing import Any

import pytest
from robotest_scenarios.artifacts import load_schema, validate_against_schema
from robotest_scenarios.errors import ArtifactError

SCHEMA_DIR = Path(__file__).resolve().parents[1] / 'schema'


def _buffer() -> dict:
    return {
        'accepted_count': 0,
        'capacity': 1,
        'first_overflow_sequence': None,
        'first_overflow_stamp_ns': None,
        'ingress_count': 0,
        'invalid_count': 0,
        'overflow': False,
        'overflow_count': 0,
        'retained_count': 0,
    }


def _cleanup_proof() -> dict:
    return {
        'kind': 'successful_blocking_delete_and_bounded_pose_absence',
        'dds_drain_complete_steady_ns': 70_000_000,
        'dds_drain_grace_ns': 50_000_000,
        'dds_drain_spin_count': 3,
        'dds_drain_start_steady_ns': 20_000_000,
        'observation_deadline_sim_stamp_ns': 17_100_000_000,
        'pose_source_publishers_after': 1,
        'pose_source_publishers_before': 1,
        'post_delete_pose_count': 0,
        'post_delete_pose_first_sequence': None,
        'post_delete_pose_first_sim_stamp_ns': None,
        'post_delete_pose_latest_sequence': None,
        'post_delete_pose_latest_sim_stamp_ns': None,
        'post_delete_pose_source_heartbeat_count': 3,
        'post_delete_pose_source_latest_sequence': 205,
        'post_delete_pose_source_latest_sim_stamp_ns': 16_600_000_000,
        'quiet_restart_count': 0,
        'quiet_start_sim_stamp_ns': 16_300_000_000,
        'quiet_until_sim_stamp_ns': 16_550_000_000,
        'request_sequence': 200,
        'request_stamp_ns': 16_000_000_000,
        'response_sequence': 201,
        'response_stamp_ns': 16_100_000_000,
    }


def _scenario_pass_result(scenario_id: int) -> dict:
    actor_scenario = scenario_id in {2, 3}
    plan_pose_stream = _buffer()
    plan_pose_stream['capacity'] = 65_536
    if scenario_id == 2:
        interaction = {
            'actor': {},
            'clearance': {},
            'criteria': {
                name: True
                for name in (
                    'all_segments_avoid',
                    'clearance_passed',
                    'geometry_hash_changed',
                    'observed_by_deadline',
                    'plan_before_request',
                    'replan_observed_after_request',
                    'replan_stamp_after_request',
                    'spawn_exactly_once',
                )
            },
            'initial_plan': {},
            'kind': 'static_obstacle',
            'observed_pose': {},
            'replan': {},
            'spawn': {},
            'trigger': {},
        }
    elif scenario_id == 3:
        interaction = {
            'actor': {},
            'criteria': {
                name: True
                for name in (
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
                )
            },
            'feedback_anchor': {},
            'kind': 'pose_controlled_obstacle',
            'metrics_owned': {},
            'preload': {},
            'spawn': {},
            'trajectory': {
                'expected_target_count': 121,
                'set_attempt_count': 121,
                'set_success_count': 121,
                'targets': [{} for _ in range(121)],
                'terminal_basis': 'follow_waypoints_status_observation',
            },
        }
    else:
        interaction = {
            'criteria': {'bound_goal_terminal_succeeded': True},
            'kind': 'none',
        }
    if actor_scenario:
        cleanup = {
            'actor_absent': True,
            'delete_attempt_count': 1,
            'delete_success': True,
            'proof': _cleanup_proof(),
            'required': True,
        }
    else:
        cleanup = {
            'actor_absent': True,
            'delete_attempt_count': 0,
            'delete_success': None,
            'proof': {'kind': 'scenario_declares_no_actor'},
            'required': False,
        }
    return {
        'binding': {},
        'cleanup': cleanup,
        'configuration': {
            'actor_asset_sha256': 'a' * 64 if actor_scenario else None,
            'controller_configuration': {'delete_service_mode': 'blocking'},
            'controller_configuration_sha256': 'b' * 64,
            'service_timeout_s': 2.0,
            'source_binding': {
                'aggregate_sha256': 'c' * 64,
                'files': [{'name': 'scenario_controller.py', 'sha256': 'd' * 64}],
            },
            'wall_timeout_s': 300.0,
        },
        'identity': {
            'candidate_id': 'candidate-1',
            'repetition_index': 0,
            'run_id': f'run-{scenario_id}',
            'scenario_id': scenario_id,
            'scenario_name': (
                {
                    1: 'baseline_navigation',
                    2: 'deterministic_static_obstacle_replan',
                    3: 'deterministic_dynamic_obstacle',
                }[scenario_id]
            ),
            'scenario_sha256': str(scenario_id) * 64,
            'suite_index': 0,
        },
        'interaction': interaction,
        'producer': 'robotest_scenarios/scenario_controller',
        'quality': {
            'all_buffers_bounded': True,
            'buffers': {
                name: _buffer()
                for name in ('actor_state', 'feedback', 'ground_truth', 'plans', 'status')
            },
            'clock': {
                'first_stamp_ns': 1,
                'latest_stamp_ns': 2,
                'max_gap_ns': 1,
                'regression_count': 0,
                'sample_count': 2,
            },
            'immutable_t0': True,
            'overflow_free': True,
            'plan_pose_stream': plan_pose_stream,
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


@pytest.mark.parametrize('scenario_id', [2, 3])
def test_scenario_pass_schema_requires_bounded_actor_cleanup_proof(scenario_id: int) -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(scenario_id)
    validate_against_schema(value, schema)

    del value['cleanup']['proof']['quiet_restart_count']
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(value, schema)


@pytest.mark.parametrize(
    'mutation',
    [
        lambda configuration: configuration.pop('controller_configuration_sha256'),
        lambda configuration: configuration.__setitem__('unexpected', True),
        lambda configuration: configuration.__setitem__('source_binding', {}),
        lambda configuration: configuration.__setitem__('service_timeout_s', 0.0),
        lambda configuration: configuration.__setitem__('wall_timeout_s', 301.0),
    ],
)
def test_scenario_configuration_schema_rejects_invalid_shape(mutation: Any) -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(2)
    mutation(value['configuration'])

    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(value, schema)


@pytest.mark.parametrize(
    ('field', 'value'),
    [
        ('kind', 'successful_blocking_delete_and_pose_quiet_interval'),
        ('dds_drain_grace_ns', 49_999_999),
        ('post_delete_pose_source_heartbeat_count', 1),
        ('post_delete_pose_source_latest_sequence', 0),
        ('pose_source_publishers_after', 0),
    ],
)
def test_scenario_cleanup_proof_schema_rejects_mutations(field: str, value: object) -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    proof_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/cleanupProof',
    }
    proof = _cleanup_proof()
    validate_against_schema(proof, proof_schema)
    proof[field] = value
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(proof, proof_schema)


@pytest.mark.parametrize(
    'schema_name',
    ['contact-control-result.schema.json', 'scenario-result.schema.json'],
)
def test_pending_cleanup_proof_allows_source_to_disappear_before_observation(
    schema_name: str,
) -> None:
    schema = load_schema(SCHEMA_DIR / schema_name)
    proof_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/pendingCleanupProof',
    }
    proof = _cleanup_proof()
    proof.pop('pose_source_publishers_after')
    proof.update(
        {
            'kind': 'successful_blocking_delete_response_cleanup_anchor_pending',
            'dds_drain_complete_steady_ns': None,
            'dds_drain_spin_count': 0,
            'dds_drain_start_steady_ns': None,
            'pose_source_publishers_before': 0,
            'post_delete_pose_source_heartbeat_count': 0,
            'post_delete_pose_source_latest_sequence': None,
            'post_delete_pose_source_latest_sim_stamp_ns': None,
            'quiet_start_sim_stamp_ns': None,
            'quiet_until_sim_stamp_ns': None,
        }
    )
    validate_against_schema(proof, proof_schema)


@pytest.mark.parametrize(
    'schema_name',
    ['contact-control-result.schema.json', 'scenario-result.schema.json'],
)
def test_pending_cleanup_proof_preserves_completed_drain_when_source_disappears(
    schema_name: str,
) -> None:
    schema = load_schema(SCHEMA_DIR / schema_name)
    proof_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/pendingCleanupProof',
    }
    proof = _cleanup_proof()
    proof.pop('pose_source_publishers_after')
    proof['kind'] = 'successful_blocking_delete_response_cleanup_drain_pending'
    validate_against_schema(proof, proof_schema)


def test_scenario_failure_schema_accepts_source_loss_after_completed_drain() -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(2)
    proof = _cleanup_proof()
    proof.pop('pose_source_publishers_after')
    proof['kind'] = 'successful_blocking_delete_response_cleanup_drain_pending'
    value['status'] = 'FAIL'
    value['verdict']['exit_code'] = 24
    value['verdict']['reason'] = 'entity-pose source publisher was missing'
    value['cleanup'] = {
        'actor_absent': False,
        'delete_attempt_count': 1,
        'delete_success': True,
        'proof': proof,
        'required': True,
    }
    validate_against_schema(value, schema)


def test_actorless_scenario_schema_preserves_no_delete_cleanup() -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(1)
    validate_against_schema(value, schema)

    value['cleanup']['proof'] = {}
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(value, schema)


def test_scenario_failure_schema_requires_structured_delete_transaction_proof() -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(2)
    value['status'] = 'FAIL'
    value['verdict']['exit_code'] = 22
    value['verdict']['reason'] = 'delete response timed out'
    value['cleanup'] = {
        'actor_absent': False,
        'delete_attempt_count': 1,
        'delete_success': None,
        'proof': {
            'error': 'scenario/delete_entity response timed out',
            'failure_stage': 'response_timeout',
            'kind': 'delete_transaction_failed',
            'request_sequence': 20,
            'request_stamp_ns': 1_000_000_000,
            'response_sequence': None,
            'response_stamp_ns': None,
        },
        'required': True,
    }
    validate_against_schema(value, schema)

    value['cleanup']['proof'] = {}
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(value, schema)


def test_scenario_failure_schema_accepts_spawn_sent_cleanup_pending_proof() -> None:
    schema = load_schema(SCHEMA_DIR / 'scenario-result.schema.json')
    value = _scenario_pass_result(2)
    value['status'] = 'FAIL'
    value['verdict']['exit_code'] = 22
    value['verdict']['reason'] = 'pose-source graph query failed before delete'
    value['cleanup'] = {
        'actor_absent': False,
        'delete_attempt_count': 0,
        'delete_success': None,
        'proof': {'kind': 'spawn_request_sent_cleanup_pending'},
        'required': True,
    }
    validate_against_schema(value, schema)


@pytest.mark.parametrize(
    ('schema_name', 'definition_name'),
    [
        (
            'contact-control-result.schema.json',
            'simpleCleanupFailureProof',
        ),
        (
            'scenario-result.schema.json',
            'simpleActorFailureProof',
        ),
    ],
)
def test_failure_schema_accepts_ambiguous_spawn_send_cleanup_obligation(
    schema_name: str,
    definition_name: str,
) -> None:
    schema = load_schema(SCHEMA_DIR / schema_name)
    validate_against_schema(
        {'kind': 'spawn_request_send_attempted_cleanup_pending'},
        schema['$defs'][definition_name],
    )


def _contact_graph_endpoint(endpoint_gid: str) -> dict:
    return {
        'endpoint_gid': endpoint_gid,
        'node_fqn': '/robotest/contact_stream_gate',
        'qos': {
            'depth': 10,
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


def test_contact_graph_endpoint_schema_pins_jazzy_gid_width() -> None:
    schema = load_schema(SCHEMA_DIR / 'contact-control-result.schema.json')
    endpoint_schema = schema['$defs']['contactGraphEndpoint']
    validate_against_schema(_contact_graph_endpoint('ab' * 16), endpoint_schema)
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(_contact_graph_endpoint('ab' * 24), endpoint_schema)


def test_contact_snapshot_schema_accepts_unbounded_integer_delivery_diagnostic() -> None:
    schema = load_schema(SCHEMA_DIR / 'contact-control-result.schema.json')
    snapshot_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/contactSnapshot',
    }
    snapshot = {
        'classified_count': 0,
        'collector_sequence': 1,
        'counted_snapshot_records': [],
        'delivery_clock_offset_ns': 220_000_001,
        'delivery_clock_stamp_ns': 1_220_000_001,
        'exact_pair_count': 0,
        'sim_stamp_ns': 1_000_000_000,
        'snapshot_record_count': 1,
    }

    validate_against_schema(snapshot, snapshot_schema)
    snapshot['delivery_clock_offset_ns'] = -220_000_001
    validate_against_schema(snapshot, snapshot_schema)

    snapshot['delivery_clock_offset_ns'] = '220000001'
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(snapshot, snapshot_schema)


def test_contact_arm_evidence_schema_requires_complete_passing_proof() -> None:
    schema = load_schema(SCHEMA_DIR / 'contact-control-result.schema.json')
    passing_schema = {
        '$schema': schema['$schema'],
        '$defs': schema['$defs'],
        '$ref': '#/$defs/passingArmEvidence',
    }
    sha = 'a' * 64
    request = {
        'action': 'start_positive_control_motion',
        'arm_protocol_sha256': sha,
        'arm_requested_steady_ns': 20,
        'producer': 'robotest_phase3/benchmark_runner',
        'ready_sha256': sha,
        'run_id': 'control-1',
        'runtime_gate_sha256': sha,
        'schema_version': 2,
    }
    acknowledgment = {
        'arm_observed_clock_sample_count': 3,
        'arm_observed_sim_stamp_ns': 100,
        'arm_observed_steady_ns': 30,
        'arm_protocol_sha256': sha,
        'arm_request_sha256': sha,
        'arm_requested_steady_ns': 20,
        'armed_clock_sample_count': 4,
        'armed_sim_stamp_ns': 101,
        'armed_steady_ns': 40,
        'command_delivery_probe': {
            'collector_progress_observed_steady_ns': 39,
            'collector_progress_sha256': sha,
            'collector_progress_stamp_ns': 101,
            'matched_subscription_count': 2,
            'match_observed_steady_ns': 35,
            'probe_publish_returned_steady_ns': 38,
            'probe_publish_started_steady_ns': 37,
            'probe_sim_stamp_ns': 100,
            'required_subscription_count': 2,
        },
        'producer': 'robotest_scenarios/contact_control_driver',
        'ready_sha256': sha,
        'run_id': 'control-1',
        'runtime_gate_sha256': sha,
        'schema_version': 2,
    }
    evidence = {
        'acknowledgment': acknowledgment,
        'acknowledgment_sha256': sha,
        'first_nonzero_publish_returned_steady_ns': 51,
        'first_nonzero_publish_started_steady_ns': 50,
        'request': request,
        'request_sha256': sha,
    }
    validate_against_schema(evidence, passing_schema)

    evidence['acknowledgment'] = None
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(evidence, passing_schema)


def test_contact_failure_artifact_shape_is_valid() -> None:
    sha = 'a' * 64
    criteria_names = {
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
    }
    pair = ['phase3_contact_control_wall::link::collision', 'robotest::link::collision']
    value = {
        'cleanup': {
            'actor_absent': True,
            'delete_attempt_count': 0,
            'delete_success': None,
            'proof': {'kind': 'spawn_not_committed'},
            'required': False,
        },
        'configuration': {
            'control_configuration': {},
            'control_configuration_sha256': sha,
            'coverage_manifest_path': '/tmp/coverage.yaml',
            'coverage_manifest_provenance': {
                'bridge_sha256': sha,
                'contact_configuration_sha256': sha,
                'coverage_manifest_sha256': sha,
                'rendered_sdf_sha256': sha,
                'robot_description_sha256': sha,
                'world_source_sha256': sha,
            },
            'coverage_manifest_sha256': sha,
            'expected_pair': pair,
            'fixture': {},
            'fixture_sha256': sha,
            'service_timeout_s': 2.0,
            'source_binding': {},
            'wall_asset_sha256': sha,
            'wall_timeout_s': 30.0,
        },
        'control': {
            'arm': {
                'acknowledgment': None,
                'acknowledgment_sha256': None,
                'first_nonzero_publish_returned_steady_ns': None,
                'first_nonzero_publish_started_steady_ns': None,
                'request': None,
                'request_sha256': None,
            },
            'command_trace': [],
            'contact': {
                'active_counterpart_count': 0,
                'classified_record_count': 0,
                'contact_clock_bracket': None,
                'counterpart_tracker_count': 1,
                'episodes': [],
                'exact_pair_snapshot_record_count': 0,
                'expected_pair': pair,
                'first_qualifying_contact': None,
                'release_snapshot': None,
                'snapshot_contact_record_count': 0,
                'snapshot_records': [],
                'snapshots': [],
            },
            'criteria': dict.fromkeys(criteria_names, False),
            'metrics_owned': {},
            'observed_robot_start': None,
            'setup': {
                'observed_robot_start': None,
                'observed_wall': None,
                'spawn': {
                    'attempt_count': 0,
                    'error': None,
                    'request_sequence': None,
                    'request_stamp_ns': None,
                    'response_sequence': None,
                    'response_stamp_ns': None,
                    'success': None,
                },
            },
            'timeline': {
                'control_started_steady_ns': None,
                'control_started_stamp_ns': None,
                'final_zero_stamp_ns': None,
                'hold_complete_stamp_ns': None,
                'release_complete_stamp_ns': None,
                'release_contact_snapshot_end_count': 0,
                'release_contact_snapshot_start_count': None,
                'release_observed_clock_stamp_ns': None,
                'release_qualified_snapshot_stamp_ns': None,
                'release_required_through_stamp_ns': None,
                'reverse_start_stamp_ns': None,
                'stop_command_stamp_ns': None,
                'stop_latency_clock_stamp_ns': None,
                'stop_latency_ns': None,
            },
        },
        'identity': {
            'fixture_id': 'collision_positive_control',
            'run_id': 'control-1',
            'scenario_sha256': sha,
        },
        'producer': 'robotest_scenarios/contact_control_driver',
        'quality': {
            'all_buffers_bounded': True,
            'buffers': {
                'actor_state': _buffer(),
                'command': _buffer(),
                'contact_snapshot_records': _buffer(),
                'contact_snapshots': _buffer(),
                'ground_truth': _buffer(),
            },
            'cmd_vel_publisher_count': 1,
            'collision_monitor_absent': True,
            'contact_graph_topology': {
                'audit_count': 0,
                'first_sha256': None,
                'first_snapshot': None,
                'last_sha256': None,
                'last_snapshot': None,
            },
            'clock': {
                'first_stamp_ns': None,
                'latest_stamp_ns': None,
                'max_gap_ns': 0,
                'regression_count': 0,
                'sample_count': 0,
            },
            'public_contact_snapshot_heartbeat': {
                'first_stamp_ns': None,
                'future_delivery_count': 0,
                'latest_stamp_ns': None,
                'max_gap_ns': 0,
                'pre_clock_discard_count': 0,
                'snapshot_count': 0,
            },
            'forbidden_nodes': [],
            'nav2_absent': True,
            'overflow_free': True,
            'protocol_error_count': 0,
            'public_contact_snapshot_stream': _buffer(),
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
        'status': 'FAIL',
        'verdict': {
            'authority': 'component_only',
            'benchmark_pass': None,
            'exit_code': 24,
            'reason': 'fixture failed',
        },
    }
    schema = load_schema(SCHEMA_DIR / 'contact-control-result.schema.json')
    validate_against_schema(value, schema)

    value['cleanup'] = {
        'actor_absent': False,
        'delete_attempt_count': 1,
        'delete_success': None,
        'proof': {
            'error': 'scenario/delete_entity response timed out',
            'failure_stage': 'response_timeout',
            'kind': 'delete_transaction_failed',
            'request_sequence': 20,
            'request_stamp_ns': 1_000_000_000,
            'response_sequence': None,
            'response_stamp_ns': None,
        },
        'required': True,
    }
    validate_against_schema(value, schema)

    del value['cleanup']['proof']['failure_stage']
    with pytest.raises(ArtifactError, match='schema violation'):
        validate_against_schema(value, schema)
