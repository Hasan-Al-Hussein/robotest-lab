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
            'proof': {},
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
