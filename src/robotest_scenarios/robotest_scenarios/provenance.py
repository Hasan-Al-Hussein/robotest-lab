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

"""Hash-bound controller configuration and installed source provenance."""

from __future__ import annotations

import hashlib
from pathlib import Path

from robotest_scenarios import constants
from robotest_scenarios.artifacts import canonical_json_bytes
from robotest_scenarios.models import package_schema_path


def file_sha256(path: Path) -> str:
    """Hash one regular file without following a mutable result artifact."""
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1_048_576), b''):
            digest.update(chunk)
    return digest.hexdigest()


def controller_configuration() -> dict[str, object]:
    """Return every scenario policy literal used outside the input document."""
    return {
        'actor_cleanup_observation_ns': constants.ACTOR_CLEANUP_OBSERVATION_NS,
        'actor_cleanup_quiet_ns': constants.ACTOR_CLEANUP_QUIET_NS,
        'actor_initial_position_tolerance_m': (constants.ACTOR_INITIAL_POSITION_TOLERANCE_M),
        'actor_observation_latency_ns': constants.ACTOR_OBSERVATION_LATENCY_NS,
        'actor_position_tolerance_m': constants.ACTOR_POSITION_TOLERANCE_M,
        'actor_yaw_tolerance_rad': constants.ACTOR_YAW_TOLERANCE_RAD,
        'capacities': {
            'actor_state': constants.ACTOR_STATE_CAPACITY,
            'feedback': constants.FEEDBACK_CAPACITY,
            'ground_truth': constants.GROUND_TRUTH_CAPACITY,
            'plan_poses': constants.PLAN_POSE_CAPACITY,
            'plans': constants.PLAN_CAPACITY,
            'status': constants.STATUS_CAPACITY,
        },
        'ground_truth_alignment_ns': constants.GROUND_TRUTH_ALIGNMENT_NS,
        'dds_drain_grace_s': constants.DDS_DRAIN_GRACE_S,
        'delete_service_mode': constants.DELETE_SERVICE_MODE,
        'names': {
            'action': constants.ACTION_NAME,
            'cancel_service': constants.ACTION_CANCEL_SERVICE,
            'clock': constants.CLOCK_TOPIC,
            'delete_service': constants.DELETE_SERVICE,
            'entity_pose': constants.ENTITY_POSE_TOPIC,
            'ground_truth': constants.GROUND_TRUTH_TOPIC,
            'plan': constants.PLAN_TOPIC,
            'set_pose_service': constants.SET_POSE_SERVICE,
            'spawn_service': constants.SPAWN_SERVICE,
        },
        'plan_endpoint_tolerance_m': constants.PLAN_ENDPOINT_TOLERANCE_M,
        'plan_length_epsilon_m': constants.PATH_LENGTH_EPSILON_M,
        's2_clearance_m': constants.S2_CLEARANCE_M,
        's2_exclusion_rectangle': list(constants.S2_EXCLUSION_RECT),
        's2_observation_deadline_offset_ns': constants.S2_OBSERVATION_DEADLINE_OFFSET_NS,
        's2_trigger_offset_ns': constants.S2_TRIGGER_OFFSET_NS,
        's3_period_ns': constants.S3_PERIOD_NS,
        's3_target_count': constants.S3_TARGET_COUNT,
    }


def configuration_sha256() -> str:
    """Hash the normalized controller policy."""
    return hashlib.sha256(canonical_json_bytes(controller_configuration())).hexdigest()


def source_binding() -> dict[str, object]:
    """Hash the installed implementation modules and schemas used by the executable."""
    module_dir = Path(__file__).resolve().parent
    names = (
        '__init__.py',
        'artifacts.py',
        'bounded.py',
        'constants.py',
        'errors.py',
        'geometry.py',
        'models.py',
        'provenance.py',
        'scenario_controller.py',
    )
    files: list[dict[str, str]] = []
    for name in names:
        path = module_dir / name
        if path.is_file():
            files.append({'name': name, 'sha256': file_sha256(path)})
    for schema_name in ('scenario-input.schema.json', 'scenario-result.schema.json'):
        schema_path = package_schema_path(schema_name)
        files.append({'name': f'schema/{schema_name}', 'sha256': file_sha256(schema_path)})
    files.sort(key=lambda item: item['name'])
    aggregate = hashlib.sha256(canonical_json_bytes(files)).hexdigest()
    return {'aggregate_sha256': aggregate, 'files': files}


def contact_control_arm_protocol() -> dict[str, object]:
    """Return the frozen runner-to-driver motion-arm protocol."""
    return {
        'ack_max_bytes': constants.CONTROL_ARM_ACK_MAX_BYTES,
        'ack_producer': constants.CONTROL_ARM_ACK_PRODUCER,
        'action': constants.CONTROL_ARM_ACTION,
        'command_delivery_probe': {
            'max_sim_lag_ns': constants.CONTROL_COMMAND_DELIVERY_PROBE_MAX_LAG_NS,
            'policy': constants.CONTROL_COMMAND_DELIVERY_PROBE_POLICY,
            'progress_max_bytes': constants.CONTROL_COMMAND_PROGRESS_MAX_BYTES,
            'progress_producer': constants.CONTROL_COMMAND_PROGRESS_PRODUCER,
            'progress_schema_version': constants.CONTROL_COMMAND_PROGRESS_SCHEMA_VERSION,
            'public_topic': constants.CONTROL_COMMAND_PROGRESS_TOPIC,
            'required_subscription_count': (constants.CONTROL_COMMAND_REQUIRED_SUBSCRIPTION_COUNT),
            'safe_zero': {
                'angular_z_rad_s': 0.0,
                'linear_x_m_s': 0.0,
                'linear_y_m_s': 0.0,
            },
        },
        'fresh_clock_policy': constants.CONTROL_ARM_FRESH_CLOCK_POLICY,
        'request_max_bytes': constants.CONTROL_ARM_REQUEST_MAX_BYTES,
        'request_producer': constants.CONTROL_ARM_REQUEST_PRODUCER,
        'schema_version': constants.CONTROL_ARM_SCHEMA_VERSION,
        'wait_deadline_policy': constants.CONTROL_ARM_WAIT_DEADLINE_POLICY,
    }


def contact_control_arm_protocol_sha256() -> str:
    """Hash the canonical positive-control motion-arm protocol."""
    return hashlib.sha256(canonical_json_bytes(contact_control_arm_protocol())).hexdigest()


def contact_control_configuration() -> dict[str, object]:
    """Return every frozen policy literal used by the positive-control driver."""
    return {
        'actor_cleanup_observation_ns': constants.ACTOR_CLEANUP_OBSERVATION_NS,
        'arm_protocol': contact_control_arm_protocol(),
        'capacities': {
            'actor_state': constants.ACTOR_STATE_CAPACITY,
            'command': constants.COMMAND_CAPACITY,
            'contact_snapshot_records': constants.CONTACT_RECORD_CAPACITY,
            'contact_snapshots': constants.CONTACT_SUMMARY_CAPACITY,
            'ground_truth': constants.GROUND_TRUTH_CAPACITY,
        },
        'cleanup_reserve_s': constants.CONTROL_CLEANUP_RESERVE_S,
        'contact_deadline_ns': constants.CONTROL_CONTACT_DEADLINE_NS,
        'dds_drain_grace_s': constants.DDS_DRAIN_GRACE_S,
        'delete_service_mode': constants.DELETE_SERVICE_MODE,
        'contact_release_gap_ns': constants.CONTACT_RELEASE_GAP_NS,
        'forward_mps': constants.CONTROL_FORWARD_MPS,
        'hold_ns': constants.CONTROL_HOLD_NS,
        'names': {
            'clock': constants.CLOCK_TOPIC,
            'cmd_vel': constants.FINAL_COMMAND_TOPIC,
            'contacts': constants.CONTACT_TOPIC,
            'delete_service': constants.DELETE_SERVICE,
            'entity_pose': constants.ENTITY_POSE_TOPIC,
            'spawn_service': constants.SPAWN_SERVICE,
        },
        'publish_period_ns': constants.CONTROL_COMMAND_PERIOD_NS,
        'reverse_mps': constants.CONTROL_REVERSE_MPS,
        'reverse_ns': constants.CONTROL_REVERSE_NS,
        'robot_start': list(constants.CONTROL_ROBOT_START),
        'source_graph_missing_confirmation_ns': (constants.CONTROL_SOURCE_GRAPH_MISSING_CONFIRM_NS),
        'spawn_response_deadline_policy': (constants.CONTROL_SPAWN_RESPONSE_DEADLINE_POLICY),
        'stop_deadline_ns': constants.CONTROL_STOP_DEADLINE_NS,
        'wall_collision': constants.CONTROL_WALL_COLLISION,
        'wall_pose': list(constants.CONTROL_WALL_POSE),
    }


def contact_control_configuration_sha256() -> str:
    """Hash the normalized positive-control policy."""
    return hashlib.sha256(canonical_json_bytes(contact_control_configuration())).hexdigest()


def contact_source_binding() -> dict[str, object]:
    """Hash the positive-control implementation and result schema."""
    module_dir = Path(__file__).resolve().parent
    names = (
        '__init__.py',
        'artifacts.py',
        'bounded.py',
        'constants.py',
        'contact_control_driver.py',
        'contact_evidence.py',
        'errors.py',
        'geometry.py',
        'models.py',
        'provenance.py',
    )
    files: list[dict[str, str]] = []
    for name in names:
        path = module_dir / name
        if path.is_file():
            files.append({'name': name, 'sha256': file_sha256(path)})
    schema_path = package_schema_path('contact-control-result.schema.json')
    files.append(
        {
            'name': 'schema/contact-control-result.schema.json',
            'sha256': file_sha256(schema_path),
        }
    )
    files.sort(key=lambda item: item['name'])
    aggregate = hashlib.sha256(canonical_json_bytes(files)).hexdigest()
    return {'aggregate_sha256': aggregate, 'files': files}
