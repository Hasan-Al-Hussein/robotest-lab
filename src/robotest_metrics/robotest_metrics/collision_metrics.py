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
from pathlib import Path
from typing import Any

from robotest_scenarios.constants import (
    ACTOR_CLEANUP_QUIET_NS,
    ACTOR_INITIAL_POSITION_TOLERANCE_M,
    ACTOR_YAW_TOLERANCE_RAD,
    CONTROL_ROBOT_START,
    CONTROL_WALL_POSE,
)
from robotest_scenarios.contact_evidence import (
    EXPECTED_CONTACT_GATE_SOURCE_PATHS,
    EXPECTED_CONTACT_STREAM_POLICY,
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
    'expected_contact_snapshot_observed',
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
    'contact_snapshot_records': CONTACT_RECORD_CAPACITY,
    'contact_snapshots': 8_192,
    'ground_truth': 8_192,
    'public_contact_snapshot_stream': CONTACT_RECORD_CAPACITY,
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
    'snapshot_sequence',
}
_CONTACT_DISPOSITIONS = {
    'allowlisted_support_contact': 'support_ground_excluded',
    'robot_internal': 'robot_internal_excluded',
}
_MAX_CONTACT_SNAPSHOT_GAP_NS = 220_000_000
_ARM_REQUEST_FIELDS = {
    'action',
    'arm_protocol_sha256',
    'arm_requested_steady_ns',
    'producer',
    'ready_sha256',
    'run_id',
    'runtime_gate_sha256',
    'schema_version',
}
_ARM_ACKNOWLEDGMENT_FIELDS = {
    'arm_observed_clock_sample_count',
    'arm_observed_sim_stamp_ns',
    'arm_observed_steady_ns',
    'arm_protocol_sha256',
    'arm_request_sha256',
    'arm_requested_steady_ns',
    'armed_clock_sample_count',
    'armed_sim_stamp_ns',
    'armed_steady_ns',
    'command_delivery_probe',
    'producer',
    'ready_sha256',
    'run_id',
    'runtime_gate_sha256',
    'schema_version',
}
_COMMAND_DELIVERY_PROBE_FIELDS = {
    'collector_progress_observed_steady_ns',
    'collector_progress_sha256',
    'collector_progress_stamp_ns',
    'matched_subscription_count',
    'match_observed_steady_ns',
    'probe_publish_returned_steady_ns',
    'probe_publish_started_steady_ns',
    'probe_sim_stamp_ns',
    'required_subscription_count',
}
_ARM_RESULT_FIELDS = {
    'acknowledgment',
    'acknowledgment_sha256',
    'first_nonzero_publish_returned_steady_ns',
    'first_nonzero_publish_started_steady_ns',
    'request',
    'request_sha256',
}


def _sha256(value: Any, name: str) -> str:
    if not isinstance(value, str) or _SHA256.fullmatch(value) is None:
        raise MetricUnavailable(f'{name} must be a lowercase SHA-256')
    return value


def _positive_int(value: Any, name: str) -> int:
    parsed = require_int(value, name)
    if parsed <= 0:
        raise MetricUnavailable(f'{name} must be positive')
    return parsed


def _validate_contact_control_arm(
    control: Mapping[str, Any],
    *,
    expected_run_id: str,
    control_configuration: Mapping[str, Any],
) -> str:
    """Validate the component-owned motion authorization and fresh-clock barrier."""
    arm = control.get('arm')
    if not isinstance(arm, Mapping) or set(arm) != _ARM_RESULT_FIELDS:
        raise MetricUnavailable('positive-control arm proof fields are incomplete')
    protocol = control_configuration.get('arm_protocol')
    if not isinstance(protocol, Mapping):
        raise MetricUnavailable('positive-control arm protocol is missing')
    protocol_sha256 = canonical_sha256(protocol)

    request = arm.get('request')
    if not isinstance(request, Mapping) or set(request) != _ARM_REQUEST_FIELDS:
        raise MetricUnavailable('positive-control arm request fields are incomplete')
    request_schema_version = require_int(
        request.get('schema_version'), 'positive_control.control.arm.request.schema_version'
    )
    requested_steady_ns = _positive_int(
        request.get('arm_requested_steady_ns'),
        'positive_control.control.arm.request.arm_requested_steady_ns',
    )
    request_ready_sha256 = _sha256(
        request.get('ready_sha256'), 'positive_control.control.arm.request.ready_sha256'
    )
    request_runtime_gate_sha256 = _sha256(
        request.get('runtime_gate_sha256'),
        'positive_control.control.arm.request.runtime_gate_sha256',
    )
    if (
        request.get('action') != protocol.get('action')
        or request.get('producer') != protocol.get('request_producer')
        or request_schema_version != protocol.get('schema_version')
        or request.get('run_id') != expected_run_id
        or request.get('arm_protocol_sha256') != protocol_sha256
    ):
        raise MetricUnavailable('positive-control arm request binding changed')
    request_sha256 = _sha256(
        arm.get('request_sha256'), 'positive_control.control.arm.request_sha256'
    )
    if canonical_sha256(request) != request_sha256:
        raise MetricUnavailable('positive-control arm request hash mismatch')

    acknowledgment = arm.get('acknowledgment')
    if not isinstance(acknowledgment, Mapping) or set(acknowledgment) != _ARM_ACKNOWLEDGMENT_FIELDS:
        raise MetricUnavailable('positive-control arm acknowledgment fields are incomplete')
    acknowledgment_schema_version = require_int(
        acknowledgment.get('schema_version'),
        'positive_control.control.arm.acknowledgment.schema_version',
    )
    observed_steady_ns = _positive_int(
        acknowledgment.get('arm_observed_steady_ns'),
        'positive_control.control.arm.acknowledgment.arm_observed_steady_ns',
    )
    armed_steady_ns = _positive_int(
        acknowledgment.get('armed_steady_ns'),
        'positive_control.control.arm.acknowledgment.armed_steady_ns',
    )
    observed_clock_count = _positive_int(
        acknowledgment.get('arm_observed_clock_sample_count'),
        'positive_control.control.arm.acknowledgment.arm_observed_clock_sample_count',
    )
    armed_clock_count = _positive_int(
        acknowledgment.get('armed_clock_sample_count'),
        'positive_control.control.arm.acknowledgment.armed_clock_sample_count',
    )
    observed_sim_stamp_ns = _positive_int(
        acknowledgment.get('arm_observed_sim_stamp_ns'),
        'positive_control.control.arm.acknowledgment.arm_observed_sim_stamp_ns',
    )
    armed_sim_stamp_ns = _positive_int(
        acknowledgment.get('armed_sim_stamp_ns'),
        'positive_control.control.arm.acknowledgment.armed_sim_stamp_ns',
    )
    probe_protocol = protocol.get('command_delivery_probe')
    if not isinstance(probe_protocol, Mapping):
        raise MetricUnavailable('positive-control command-delivery probe protocol is missing')
    probe = acknowledgment.get('command_delivery_probe')
    if not isinstance(probe, Mapping) or set(probe) != _COMMAND_DELIVERY_PROBE_FIELDS:
        raise MetricUnavailable('positive-control command-delivery probe fields are incomplete')
    progress_sha256 = _sha256(
        probe.get('collector_progress_sha256'),
        'positive_control.control.arm.command_delivery_probe.collector_progress_sha256',
    )
    progress_stamp_ns = _positive_int(
        probe.get('collector_progress_stamp_ns'),
        'positive_control.control.arm.command_delivery_probe.collector_progress_stamp_ns',
    )
    progress_observed_steady_ns = _positive_int(
        probe.get('collector_progress_observed_steady_ns'),
        'positive_control.control.arm.command_delivery_probe.collector_progress_observed_steady_ns',
    )
    matched_subscription_count = _positive_int(
        probe.get('matched_subscription_count'),
        'positive_control.control.arm.command_delivery_probe.matched_subscription_count',
    )
    match_observed_steady_ns = _positive_int(
        probe.get('match_observed_steady_ns'),
        'positive_control.control.arm.command_delivery_probe.match_observed_steady_ns',
    )
    probe_publish_started_steady_ns = _positive_int(
        probe.get('probe_publish_started_steady_ns'),
        'positive_control.control.arm.command_delivery_probe.probe_publish_started_steady_ns',
    )
    probe_publish_returned_steady_ns = _positive_int(
        probe.get('probe_publish_returned_steady_ns'),
        'positive_control.control.arm.command_delivery_probe.probe_publish_returned_steady_ns',
    )
    probe_sim_stamp_ns = _positive_int(
        probe.get('probe_sim_stamp_ns'),
        'positive_control.control.arm.command_delivery_probe.probe_sim_stamp_ns',
    )
    required_subscription_count = _positive_int(
        probe.get('required_subscription_count'),
        'positive_control.control.arm.command_delivery_probe.required_subscription_count',
    )
    protocol_required_count = _positive_int(
        probe_protocol.get('required_subscription_count'),
        'positive_control.control.arm.command_delivery_probe.protocol_required_count',
    )
    max_sim_lag_ns = _positive_int(
        probe_protocol.get('max_sim_lag_ns'),
        'positive_control.control.arm.command_delivery_probe.max_sim_lag_ns',
    )
    if (
        acknowledgment.get('arm_protocol_sha256') != protocol_sha256
        or acknowledgment.get('arm_request_sha256') != request_sha256
        or acknowledgment.get('arm_requested_steady_ns') != requested_steady_ns
        or acknowledgment.get('producer') != protocol.get('ack_producer')
        or acknowledgment.get('ready_sha256') != request_ready_sha256
        or acknowledgment.get('run_id') != expected_run_id
        or acknowledgment.get('runtime_gate_sha256') != request_runtime_gate_sha256
        or acknowledgment_schema_version != protocol.get('schema_version')
    ):
        raise MetricUnavailable('positive-control arm acknowledgment binding changed')
    acknowledgment_sha256 = _sha256(
        arm.get('acknowledgment_sha256'),
        'positive_control.control.arm.acknowledgment_sha256',
    )
    if canonical_sha256(acknowledgment) != acknowledgment_sha256:
        raise MetricUnavailable('positive-control arm acknowledgment hash mismatch')
    if armed_clock_count <= observed_clock_count or armed_sim_stamp_ns <= observed_sim_stamp_ns:
        raise MetricUnavailable('positive-control arm acknowledgment lacks a fresh clock sample')
    if (
        matched_subscription_count != protocol_required_count
        or required_subscription_count != protocol_required_count
        or protocol_required_count != 2
    ):
        raise MetricUnavailable('positive-control command-delivery probe match count changed')
    if not (
        observed_steady_ns
        <= match_observed_steady_ns
        <= probe_publish_started_steady_ns
        <= probe_publish_returned_steady_ns
        <= armed_steady_ns
        and probe_publish_started_steady_ns <= progress_observed_steady_ns <= armed_steady_ns
    ):
        raise MetricUnavailable('positive-control command-delivery probe ordering is invalid')
    if abs(progress_stamp_ns - probe_sim_stamp_ns) > max_sim_lag_ns:
        raise MetricUnavailable('positive-control command-delivery probe lag is invalid')
    if not observed_sim_stamp_ns < probe_sim_stamp_ns <= armed_sim_stamp_ns:
        raise MetricUnavailable(
            'positive-control command-delivery probe fresh-clock bracket is invalid'
        )

    first_publish_started_ns = _positive_int(
        arm.get('first_nonzero_publish_started_steady_ns'),
        'positive_control.control.arm.first_nonzero_publish_started_steady_ns',
    )
    first_publish_returned_ns = _positive_int(
        arm.get('first_nonzero_publish_returned_steady_ns'),
        'positive_control.control.arm.first_nonzero_publish_returned_steady_ns',
    )
    timeline = control.get('timeline')
    if not isinstance(timeline, Mapping):
        raise MetricUnavailable('positive-control timeline is missing')
    control_started_steady_ns = _positive_int(
        timeline.get('control_started_steady_ns'),
        'positive_control.timeline.control_started_steady_ns',
    )
    if not (
        requested_steady_ns
        <= observed_steady_ns
        <= armed_steady_ns
        <= control_started_steady_ns
        == first_publish_started_ns
        <= first_publish_returned_ns
    ):
        raise MetricUnavailable('positive-control arm steady-time ordering is invalid')
    return progress_sha256


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


def _validate_contact_graph_snapshot(value: Any, label: str) -> None:
    if not isinstance(value, Mapping) or set(value) != {
        'private_raw_publishers',
        'private_raw_subscribers',
        'public_snapshot_publishers',
        'topics',
    }:
        raise MetricUnavailable(f'{label} contact graph snapshot is invalid')
    if value.get('topics') != {
        'private_raw_contact_topic': '/robotest/internal/raw_contacts',
        'public_contact_snapshot_topic': '/robotest/validation/contacts',
    }:
        raise MetricUnavailable(f'{label} contact graph topics changed')
    expected = {
        'private_raw_publishers': ('/robotest/parameter_bridge', 64),
        'private_raw_subscribers': ('/robotest/contact_stream_gate', 64),
        'public_snapshot_publishers': ('/robotest/contact_stream_gate', 10),
    }
    for key, (node_fqn, expected_depth) in expected.items():
        endpoints = value.get(key)
        if not isinstance(endpoints, list) or len(endpoints) != 1:
            raise MetricUnavailable(f'{label}.{key} cardinality is not exactly one')
        endpoint = endpoints[0]
        if not isinstance(endpoint, Mapping) or set(endpoint) != {
            'endpoint_gid',
            'node_fqn',
            'qos',
            'qos_status',
            'topic_type',
        }:
            raise MetricUnavailable(f'{label}.{key} endpoint is invalid')
        qos = endpoint.get('qos')
        if not isinstance(qos, Mapping) or set(qos) != {
            'depth',
            'durability',
            'history',
            'reliability',
        }:
            raise MetricUnavailable(f'{label}.{key} QoS is invalid')
        depth = require_int(qos.get('depth'), f'{label}.{key}.qos.depth')
        expected_status = {
            'depth_matches_or_unknown': depth <= 0 or depth == expected_depth,
            'durability_volatile': qos.get('durability') == 'VOLATILE',
            'history_keep_last_or_unknown': qos.get('history')
            in {'KEEP_LAST', 'SYSTEM_DEFAULT', 'UNKNOWN'},
            'reliability_reliable': qos.get('reliability') == 'RELIABLE',
        }
        if (
            not isinstance(endpoint.get('endpoint_gid'), str)
            or re.fullmatch(r'[0-9a-f]{32}', endpoint['endpoint_gid']) is None
            or endpoint.get('node_fqn') != node_fqn
            or endpoint.get('topic_type') != 'ros_gz_interfaces/msg/Contacts'
            or endpoint.get('qos_status') != expected_status
            or not all(expected_status.values())
        ):
            raise MetricUnavailable(f'{label}.{key} owner/type/QoS changed')


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
            raise MetricUnavailable(
                f'positive-control contact.snapshot_records[{index}] is invalid'
            )
        sequence = require_int(
            value.get('collector_sequence'),
            f'positive_control.contact.snapshot_records[{index}].collector_sequence',
        )
        stamp = require_int(
            value.get('sim_stamp_ns'),
            f'positive_control.contact.snapshot_records[{index}].sim_stamp_ns',
        )
        pair = value.get('normalized_pair')
        snapshot_sequence = require_int(
            value.get('snapshot_sequence'),
            f'positive_control.contact.snapshot_records[{index}].snapshot_sequence',
        )
        if (
            sequence <= previous_sequence
            or snapshot_sequence < 1
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


def _reconcile_contact_snapshots(
    snapshots: Any,
    records: Sequence[Mapping[str, Any]],
    manifest: Mapping[str, Any],
    expected: tuple[str, str],
) -> tuple[list[dict[str, Any]], dict[int, list[Mapping[str, Any]]]]:
    """Validate v3 snapshot summaries and recompute presence/absence episodes."""
    if not isinstance(snapshots, list) or not snapshots:
        raise MetricUnavailable('positive-control authoritative contact snapshots are missing')
    records_by_snapshot: dict[int, list[Mapping[str, Any]]] = {}
    for record in records:
        snapshot_sequence = require_int(
            record.get('snapshot_sequence'), 'positive_control.snapshot_record.snapshot_sequence'
        )
        records_by_snapshot.setdefault(snapshot_sequence, []).append(record)

    previous_stamp: int | None = None
    previous_sequence = 0
    active: dict[str, dict[str, Any]] = {}
    episodes: list[dict[str, Any]] = []
    seen_summary_sequences: set[int] = set()
    summary_fields = {
        'classified_count',
        'collector_sequence',
        'counted_snapshot_records',
        'delivery_clock_offset_ns',
        'delivery_clock_stamp_ns',
        'exact_pair_count',
        'sim_stamp_ns',
        'snapshot_record_count',
    }
    for index, summary in enumerate(snapshots):
        if not isinstance(summary, Mapping) or set(summary) != summary_fields:
            raise MetricUnavailable(f'positive-control contact.snapshots[{index}] is invalid')
        sequence = require_int(
            summary.get('collector_sequence'),
            f'positive_control.contact.snapshots[{index}].collector_sequence',
        )
        stamp = require_int(
            summary.get('sim_stamp_ns'),
            f'positive_control.contact.snapshots[{index}].sim_stamp_ns',
        )
        delivery_clock = require_int(
            summary.get('delivery_clock_stamp_ns'),
            f'positive_control.contact.snapshots[{index}].delivery_clock_stamp_ns',
        )
        delivery_offset = require_int(
            summary.get('delivery_clock_offset_ns'),
            f'positive_control.contact.snapshots[{index}].delivery_clock_offset_ns',
        )
        if (
            sequence <= previous_sequence
            or stamp <= 0
            or (
                previous_stamp is not None and stamp - previous_stamp > _MAX_CONTACT_SNAPSHOT_GAP_NS
            )
            or (previous_stamp is not None and stamp <= previous_stamp)
            or delivery_clock - stamp != delivery_offset
            or abs(delivery_offset) > _MAX_CONTACT_SNAPSHOT_GAP_NS
        ):
            raise MetricUnavailable(
                'positive-control contact snapshot ordering/liveness is invalid'
            )
        previous_sequence = sequence
        previous_stamp = stamp
        seen_summary_sequences.add(sequence)
        snapshot_records = records_by_snapshot.get(sequence, [])
        if any(
            record.get('sim_stamp_ns') != stamp
            or require_int(record.get('collector_sequence'), 'snapshot record sequence') <= sequence
            for record in snapshot_records
        ):
            raise MetricUnavailable('positive-control snapshot record linkage is invalid')
        counted_records = [
            record for record in snapshot_records if record.get('disposition') == 'counted'
        ]
        exact_records = [
            record
            for record in counted_records
            if tuple(record.get('normalized_pair', ())) == expected
        ]
        expected_counted_summary = [
            {
                'counterpart_model': record['counterpart_model'],
                'normalized_pair': record['normalized_pair'],
                'record_sequence': record['collector_sequence'],
                'snapshot_sequence': sequence,
            }
            for record in counted_records
        ]
        if (
            not 1 <= len(snapshot_records) <= 16
            or require_int(summary.get('snapshot_record_count'), 'snapshot_record_count')
            != len(snapshot_records)
            or require_int(summary.get('classified_count'), 'classified_count')
            != len(counted_records)
            or require_int(summary.get('exact_pair_count'), 'exact_pair_count')
            != len(exact_records)
            or summary.get('counted_snapshot_records') != expected_counted_summary
        ):
            raise MetricUnavailable('positive-control contact snapshot summary does not reconcile')

        present: dict[str, list[Mapping[str, Any]]] = {}
        for record in counted_records:
            present.setdefault(str(record['counterpart_model']), []).append(record)
        for counterpart in list(active):
            if counterpart in present:
                continue
            episode = active.pop(counterpart)
            episode['end_stamp_ns'] = stamp
            episodes.append(episode)
        for counterpart, present_records in present.items():
            episode = active.get(counterpart)
            if episode is None:
                episode = {
                    'counterpart_model': counterpart,
                    'normalized_pairs': set(),
                    'snapshot_record_count': 0,
                    'start_stamp_ns': stamp,
                }
                active[counterpart] = episode
            episode['normalized_pairs'].update(
                tuple(record['normalized_pair']) for record in present_records
            )
            episode['snapshot_record_count'] += len(present_records)

    if not set(records_by_snapshot).issubset(seen_summary_sequences):
        raise MetricUnavailable('positive-control snapshot record references a missing summary')
    if active:
        raise MetricUnavailable('positive-control contact episode lacks an absence snapshot')
    normalized_episodes = [
        {
            **episode,
            'normalized_pairs': [list(pair) for pair in sorted(episode['normalized_pairs'])],
        }
        for episode in episodes
    ]
    return normalized_episodes, records_by_snapshot


def _validate_command_trace(
    commands: Any,
    *,
    control_started_stamp: int,
    first_contact_sequence: int,
    qualifying_snapshot_max_record_sequence: int,
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
        or stop_command['collector_sequence'] != qualifying_snapshot_max_record_sequence + 1
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
    if manifest.get('schema_version') != 3:
        raise MetricUnavailable('coverage manifest must use authoritative contact schema v3')
    contact_stream = manifest.get('contact_stream')
    if not isinstance(contact_stream, Mapping) or contact_stream.get('schema_version') != 1:
        raise MetricUnavailable('coverage manifest contact_stream v1 is missing')
    topics = contact_stream.get('topics')
    if (
        topics
        != {
            'gazebo_raw': '/robotest/internal/contact_aggregate',
            'private_raw_ros': '/robotest/internal/raw_contacts',
            'public_ros': '/robotest/validation/contacts',
        }
        or manifest.get('contact_topic') != '/robotest/validation/contacts'
    ):
        raise MetricUnavailable('coverage manifest contact topology is not frozen v3')
    policy = contact_stream.get('policy')
    if policy != EXPECTED_CONTACT_STREAM_POLICY:
        raise MetricUnavailable('coverage manifest contact snapshot policy is not frozen v3')
    try:
        policy_sha256 = canonical_sha256(policy)
    except ArtifactError as exc:
        raise MetricUnavailable('coverage contact policy is not canonicalizable') from exc
    if contact_stream.get('policy_sha256') != policy_sha256:
        raise MetricUnavailable('coverage contact policy hash mismatch')
    if contact_stream.get('qos') != {
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
    }:
        raise MetricUnavailable('coverage contact QoS is not frozen v3')
    gate = contact_stream.get('gate')
    if (
        not isinstance(gate, Mapping)
        or set(gate)
        != {
            'executable',
            'launch_sha256',
            'package',
            'source_inventory',
            'source_inventory_sha256',
        }
        or gate.get('package') != 'robotest_sim'
        or gate.get('executable') != 'contact_stream_gate'
    ):
        raise MetricUnavailable('coverage contact gate ownership is invalid')
    _sha256(gate.get('launch_sha256'), 'coverage.contact_stream.gate.launch_sha256')
    inventory_sha = _sha256(
        gate.get('source_inventory_sha256'),
        'coverage.contact_stream.gate.source_inventory_sha256',
    )
    inventory = gate.get('source_inventory')
    if (
        not isinstance(inventory, Mapping)
        or set(inventory) != {'schema_version', 'sources'}
        or inventory.get('schema_version') != 1
        or not isinstance(inventory.get('sources'), list)
        or [item.get('path') for item in inventory['sources'] if isinstance(item, Mapping)]
        != list(EXPECTED_CONTACT_GATE_SOURCE_PATHS)
        or any(
            not isinstance(item, Mapping)
            or set(item) != {'path', 'sha256'}
            or not isinstance(item.get('sha256'), str)
            or _SHA256.fullmatch(item['sha256']) is None
            for item in inventory.get('sources', [])
        )
    ):
        raise MetricUnavailable('coverage contact gate source inventory is invalid')
    try:
        calculated_inventory_sha = canonical_sha256(inventory)
    except ArtifactError as exc:
        raise MetricUnavailable('coverage contact gate source inventory is not canonical') from exc
    if inventory_sha != calculated_inventory_sha:
        raise MetricUnavailable('coverage contact gate source inventory hash mismatch')
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
    command_progress_sha256 = _validate_contact_control_arm(
        control,
        expected_run_id=identity['run_id'],
        control_configuration=expected_control_configuration,
    )
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
        contact.get('snapshot_records'), manifest, expected
    )
    computed_episodes, _records_by_snapshot = _reconcile_contact_snapshots(
        contact.get('snapshots'), contact['snapshot_records'], manifest, expected
    )
    exact_count = require_int(
        contact.get('exact_pair_snapshot_record_count'),
        'positive_control.exact_pair_snapshot_record_count',
    )
    snapshot_record_count = require_int(
        contact.get('snapshot_contact_record_count'),
        'positive_control.snapshot_contact_record_count',
    )
    classified_count = require_int(
        contact.get('classified_record_count'),
        'positive_control.classified_record_count',
    )
    if (
        snapshot_record_count != len(contact['snapshot_records'])
        or classified_count != len(counted_records)
        or exact_count != len(exact_records)
        or not 0 < exact_count <= classified_count <= snapshot_record_count
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
    if (
        not isinstance(first_contact, Mapping)
        or set(first_contact)
        != {
            'callback_clock_offset_ns',
            'callback_clock_stamp_ns',
            'collector_sequence',
            'normalized_pair',
            'sim_stamp_ns',
            'stop_latency_clock_stamp_ns',
            'stop_latency_upper_bound_ns',
        }
        or first_contact.get('collector_sequence') != first_exact_record['collector_sequence']
        or first_contact.get('normalized_pair') != list(expected)
        or first_contact.get('sim_stamp_ns') != first_exact_record['sim_stamp_ns']
        or require_int(
            first_contact.get('callback_clock_stamp_ns'),
            'positive_control.first_qualifying_contact.callback_clock_stamp_ns',
        )
        - first_exact_record['sim_stamp_ns']
        != require_int(
            first_contact.get('callback_clock_offset_ns'),
            'positive_control.first_qualifying_contact.callback_clock_offset_ns',
        )
        or require_int(
            first_contact.get('stop_latency_clock_stamp_ns'),
            'positive_control.first_qualifying_contact.stop_latency_clock_stamp_ns',
        )
        < first_exact_record['sim_stamp_ns']
    ):
        raise MetricUnavailable('positive-control first qualifying contact is missing')
    callback_clock_stamp = require_int(
        first_contact.get('callback_clock_stamp_ns'),
        'positive_control.first_qualifying_contact.callback_clock_stamp_ns',
    )
    callback_clock_offset = require_int(
        first_contact.get('callback_clock_offset_ns'),
        'positive_control.first_qualifying_contact.callback_clock_offset_ns',
    )
    stop_latency_clock_stamp = require_int(
        first_contact.get('stop_latency_clock_stamp_ns'),
        'positive_control.first_qualifying_contact.stop_latency_clock_stamp_ns',
    )
    stop_latency_upper_bound = require_int(
        first_contact.get('stop_latency_upper_bound_ns'),
        'positive_control.first_qualifying_contact.stop_latency_upper_bound_ns',
    )
    if (
        callback_clock_stamp - first_exact_record['sim_stamp_ns'] != callback_clock_offset
        or abs(callback_clock_offset) > _MAX_CONTACT_SNAPSHOT_GAP_NS
        or stop_latency_clock_stamp - first_exact_record['sim_stamp_ns'] != stop_latency_upper_bound
        or not 0 <= stop_latency_upper_bound <= _CONTROL_STOP_DEADLINE_NS
    ):
        raise MetricUnavailable('positive-control contact delivery/stop bracket is invalid')
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
    if computed_episodes != episodes:
        raise MetricUnavailable('positive-control episode boundaries do not reconcile')
    expected_episode = computed_episodes[0]
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
    timeline_stop_latency_clock = require_int(
        timeline.get('stop_latency_clock_stamp_ns'),
        'positive_control.stop_latency_clock_stamp_ns',
    )
    release_start_count = require_int(
        timeline.get('release_contact_snapshot_start_count'),
        'positive_control.release_contact_snapshot_start_count',
    )
    release_end_count = require_int(
        timeline.get('release_contact_snapshot_end_count'),
        'positive_control.release_contact_snapshot_end_count',
    )
    release_observed_clock_stamp = require_int(
        timeline.get('release_observed_clock_stamp_ns'),
        'positive_control.release_observed_clock_stamp_ns',
    )
    release_qualified_snapshot_stamp = require_int(
        timeline.get('release_qualified_snapshot_stamp_ns'),
        'positive_control.release_qualified_snapshot_stamp_ns',
    )
    release_required_stamp = require_int(
        timeline.get('release_required_through_stamp_ns'),
        'positive_control.release_required_through_stamp_ns',
    )
    expected_release_stamp = final_zero_stamp + CONTACT_RELEASE_GAP_NS
    release_snapshot = contact.get('release_snapshot')
    contact_clock_bracket = contact.get('contact_clock_bracket')
    if (
        not isinstance(release_snapshot, Mapping)
        or set(release_snapshot) != {'collector_sequence', 'sim_stamp_ns'}
        or not isinstance(contact_clock_bracket, Mapping)
        or set(contact_clock_bracket)
        != {'clock_stamp_ns', 'lag_ns', 'limit_ns', 'snapshot_stamp_ns'}
    ):
        raise MetricUnavailable('positive-control release snapshot/bracket is missing')
    release_snapshot_sequence = require_int(
        release_snapshot.get('collector_sequence'),
        'positive_control.release_snapshot.collector_sequence',
    )
    release_snapshot_stamp = require_int(
        release_snapshot.get('sim_stamp_ns'),
        'positive_control.release_snapshot.sim_stamp_ns',
    )
    release_summary = next(
        (
            summary
            for summary in contact['snapshots']
            if summary.get('collector_sequence') == release_snapshot_sequence
        ),
        None,
    )
    bracket_clock = require_int(
        contact_clock_bracket.get('clock_stamp_ns'),
        'positive_control.contact_clock_bracket.clock_stamp_ns',
    )
    bracket_lag = require_int(
        contact_clock_bracket.get('lag_ns'),
        'positive_control.contact_clock_bracket.lag_ns',
    )
    bracket_limit = require_int(
        contact_clock_bracket.get('limit_ns'),
        'positive_control.contact_clock_bracket.limit_ns',
    )
    bracket_snapshot = require_int(
        contact_clock_bracket.get('snapshot_stamp_ns'),
        'positive_control.contact_clock_bracket.snapshot_stamp_ns',
    )
    qualifying_snapshot_sequence = require_int(
        first_exact_record.get('snapshot_sequence'),
        'positive_control.first_qualifying_contact.snapshot_sequence',
    )
    qualifying_snapshot_records = [
        record
        for record in contact['snapshot_records']
        if record.get('snapshot_sequence') == qualifying_snapshot_sequence
    ]
    if not qualifying_snapshot_records:
        raise MetricUnavailable('positive-control qualifying snapshot records are missing')
    qualifying_snapshot_max_record_sequence = max(
        require_int(record.get('collector_sequence'), 'qualifying snapshot record sequence')
        for record in qualifying_snapshot_records
    )
    absence_span_summaries = [
        summary
        for summary in contact['snapshots']
        if expected_episode['end_stamp_ns']
        <= require_int(summary.get('sim_stamp_ns'), 'absence snapshot stamp')
        <= release_snapshot_stamp
    ]
    if (
        control_started_stamp <= 0
        or wall_stamp > control_started_stamp
        or not control_started_stamp
        <= first_contact_stamp
        <= control_started_stamp + _CONTROL_CONTACT_DEADLINE_NS
        or callback_clock_stamp != stop_command_stamp
        or stop_latency != stop_latency_upper_bound
        or timeline_stop_latency_clock != stop_latency_clock_stamp
        or hold_complete_stamp - timeline_stop_latency_clock < _CONTROL_HOLD_NS
        or reverse_start_stamp != hold_complete_stamp
        or final_zero_stamp - reverse_start_stamp < _CONTROL_REVERSE_NS
        or release_required_stamp != expected_release_stamp
        or release_snapshot_stamp <= release_required_stamp
        or release_complete_stamp != release_snapshot_stamp
        or release_qualified_snapshot_stamp != release_snapshot_stamp
        or not first_contact_stamp < expected_episode['end_stamp_ns'] <= release_snapshot_stamp
        or not absence_span_summaries
        or any(
            item.get('counterpart_model') == classification['counterpart_model']
            for summary in absence_span_summaries
            for item in summary.get('counted_snapshot_records', [])
        )
        or release_summary is None
        or release_summary.get('sim_stamp_ns') != release_snapshot_stamp
        or any(
            item.get('counterpart_model') == classification['counterpart_model']
            for item in release_summary.get('counted_snapshot_records', [])
        )
        or release_observed_clock_stamp != bracket_clock
        or bracket_snapshot != release_snapshot_stamp
        or bracket_lag != bracket_clock - bracket_snapshot
        or bracket_limit != _MAX_CONTACT_SNAPSHOT_GAP_NS
        or not 0 <= bracket_lag <= bracket_limit
        or release_start_count < 1
        or release_end_count <= release_start_count
    ):
        raise MetricUnavailable('positive-control timeline does not match the frozen driver')
    commands = control.get('command_trace')
    _validate_command_trace(
        commands,
        control_started_stamp=control_started_stamp,
        first_contact_sequence=first_contact_sequence,
        qualifying_snapshot_max_record_sequence=qualifying_snapshot_max_record_sequence,
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
        *(record['collector_sequence'] for record in contact['snapshot_records']),
        *(summary['collector_sequence'] for summary in contact['snapshots']),
    )
    if (
        cleanup_request_sequence <= max_pre_cleanup_sequence
        or cleanup_response_sequence <= cleanup_request_sequence
        or cleanup_request_stamp != release_observed_clock_stamp
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
    if (
        require_int(
            quality.get('source_publisher_missing_observation_count'),
            'positive_control.source_publisher_missing_observation_count',
        )
        < 0
    ):
        raise MetricUnavailable('positive-control source publisher miss count is invalid')
    if quality.get('forbidden_nodes') != []:
        raise MetricUnavailable('positive-control forbidden navigation nodes were present')
    topology = quality.get('contact_graph_topology')
    if not isinstance(topology, Mapping) or set(topology) != {
        'audit_count',
        'first_sha256',
        'first_snapshot',
        'last_sha256',
        'last_snapshot',
    }:
        raise MetricUnavailable('positive-control contact graph audit is missing')
    first_graph_snapshot = topology.get('first_snapshot')
    last_graph_snapshot = topology.get('last_snapshot')
    _validate_contact_graph_snapshot(first_graph_snapshot, 'positive_control.first_graph')
    _validate_contact_graph_snapshot(last_graph_snapshot, 'positive_control.last_graph')
    try:
        first_graph_sha = canonical_sha256(first_graph_snapshot)
        last_graph_sha = canonical_sha256(last_graph_snapshot)
    except ArtifactError as exc:
        raise MetricUnavailable('positive-control contact graph snapshot is not canonical') from exc
    if (
        require_int(topology.get('audit_count'), 'positive_control.contact_graph.audit_count') < 2
        or topology.get('first_sha256') != first_graph_sha
        or topology.get('last_sha256') != last_graph_sha
    ):
        raise MetricUnavailable('positive-control contact graph audit hash/count is invalid')
    endpoint_groups = (
        'private_raw_publishers',
        'private_raw_subscribers',
        'public_snapshot_publishers',
    )
    if any(
        first_graph_snapshot[group][0]['endpoint_gid']
        != last_graph_snapshot[group][0]['endpoint_gid']
        for group in endpoint_groups
    ):
        raise MetricUnavailable('positive-control contact graph endpoint identity changed')
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
    heartbeat = quality.get('public_contact_snapshot_heartbeat')
    if not isinstance(heartbeat, Mapping) or set(heartbeat) != {
        'first_stamp_ns',
        'future_delivery_count',
        'latest_stamp_ns',
        'max_gap_ns',
        'pre_clock_discard_count',
        'snapshot_count',
    }:
        raise MetricUnavailable('positive-control contact heartbeat is incomplete')
    heartbeat_first = require_int(
        heartbeat.get('first_stamp_ns'), 'positive_control.contact_heartbeat.first_stamp_ns'
    )
    heartbeat_latest = require_int(
        heartbeat.get('latest_stamp_ns'), 'positive_control.contact_heartbeat.latest_stamp_ns'
    )
    heartbeat_count = require_int(
        heartbeat.get('snapshot_count'), 'positive_control.contact_heartbeat.snapshot_count'
    )
    heartbeat_gap = require_int(
        heartbeat.get('max_gap_ns'), 'positive_control.contact_heartbeat.max_gap_ns'
    )
    heartbeat_future_count = require_int(
        heartbeat.get('future_delivery_count'),
        'positive_control.contact_heartbeat.future_delivery_count',
    )
    heartbeat_pre_clock_discard_count = require_int(
        heartbeat.get('pre_clock_discard_count'),
        'positive_control.contact_heartbeat.pre_clock_discard_count',
    )
    if heartbeat_pre_clock_discard_count < 0:
        raise MetricUnavailable('positive-control contact heartbeat discard count is invalid')
    expected_future_count = sum(
        summary['delivery_clock_offset_ns'] < 0 for summary in contact['snapshots']
    )
    if (
        heartbeat_first < 0
        or heartbeat_latest < heartbeat_first
        or heartbeat_first != contact['snapshots'][0]['sim_stamp_ns']
        or heartbeat_latest != contact['snapshots'][-1]['sim_stamp_ns']
        or not heartbeat_first <= first_contact_stamp <= heartbeat_latest
        or heartbeat_latest < release_snapshot_stamp
        or heartbeat_count != release_end_count
        or heartbeat_count != len(contact['snapshots'])
        or heartbeat_future_count != expected_future_count
        or not 0 <= heartbeat_gap <= _MAX_CONTACT_SNAPSHOT_GAP_NS
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
        'contact_snapshot_records',
        'contact_snapshots',
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
    public_snapshot_stream = _require_bounded_buffer(
        quality.get('public_contact_snapshot_stream'),
        'public_contact_snapshot_stream',
        retain_every_accepted_item=True,
    )
    if (
        buffer_counts['command']['retained_count'] != len(commands)
        or buffer_counts['contact_snapshot_records']['retained_count'] != snapshot_record_count
        or public_snapshot_stream['retained_count'] != snapshot_record_count
        or public_snapshot_stream != buffer_counts['contact_snapshot_records']
        or buffer_counts['contact_snapshots']['retained_count'] != heartbeat_count
        or buffer_counts['ground_truth']['retained_count'] < 1
        or buffer_counts['actor_state']['retained_count'] < 1
    ):
        raise MetricUnavailable('positive-control retained buffers do not match evidence')
    return {
        'command_progress_sha256': command_progress_sha256,
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
    external_progress_sha256 = _sha256(
        external_quality.get('collector_command_progress_sha256'),
        'positive_control_external_quality.collector_command_progress_sha256',
    )
    if external_progress_sha256 != positive_identity['command_progress_sha256']:
        raise MetricUnavailable('positive-control command-progress hash binding mismatch')
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
    if not isinstance(scoped_collision, str):
        raise MetricUnavailable('contact collision name must be a string')
    segments = scoped_collision.split('::')
    if len(segments) < 3 or any(not segment for segment in segments):
        raise MetricUnavailable('contact collision name is not model::link::collision scoped')
    return segments[0]


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
    first_model = _top_model(collision1)
    second_model = _top_model(collision2)
    if (first_model == robot_model and collision1 not in robot_names) or (
        second_model == robot_model and collision2 not in robot_names
    ):
        raise MetricUnavailable('contact references an unknown rendered robot collision')
    first_robot = collision1 in robot_names
    second_robot = collision2 in robot_names
    if first_robot and second_robot:
        return {'counted': False, 'reason': 'robot_internal'}
    if not first_robot and not second_robot:
        raise MetricUnavailable('manifest-v3 contact snapshot contains a non-robot pair')
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
    """Analyze authoritative v3 active-pair snapshots through the proven drain."""
    qualification = validate_collision_qualification(manifest, positive_control, benchmark_binding)
    robot_model, robot_names, support_pairs = _collision_names(manifest)
    start = require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns')
    terminal = require_int(terminal_action_stamp_ns, 'terminal_action_stamp_ns')
    # Compatibility scalar: v3 freezes this as the exact retained qualifying
    # public snapshot stamp, never a synthesized terminal+gap target.
    drain = require_int(drain_completed_stamp_ns, 'drain_completed_stamp_ns')
    if terminal <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    if release_gap_ns <= 0:
        raise ValueError('release_gap_ns must be positive')
    required_drain = terminal + release_gap_ns
    if drain <= required_drain:
        raise MetricUnavailable(
            'contact terminal drain lacks a snapshot strictly beyond terminal + release gap'
        )
    active: dict[str, dict[str, Any]] = {}
    events: list[dict[str, Any]] = []
    exclusions: Counter[str] = Counter()
    pre_action_snapshot_records = 0
    post_terminal_snapshot_records = 0
    previous_sequence = 0
    previous_stamp: int | None = None
    maximum_public_gap_ns = 0
    snapshot_contact_record_count = 0
    public_snapshot_count = 0
    in_action_snapshot_count = 0
    drain_snapshot_observed = False
    latest_snapshot_at_or_before_start: int | None = None

    def close_absent(counterpart: str, stamp_ns: int) -> None:
        event = active.pop(counterpart)
        event['end_stamp_ns'] = stamp_ns
        event['duration_s'] = (stamp_ns - event['start_stamp_ns']) / 1_000_000_000
        event['censored_at_drain'] = False
        events.append(event)

    for message_index, message in enumerate(messages):
        stamp = require_int(message.get('stamp_ns'), f'messages[{message_index}].stamp_ns')
        sequence = require_int(
            message.get('collector_sequence'),
            f'messages[{message_index}].collector_sequence',
        )
        if stamp <= 0 or sequence <= previous_sequence:
            raise MetricUnavailable('contact snapshot collector ordering is invalid')
        if previous_stamp is not None:
            if stamp <= previous_stamp:
                raise MetricUnavailable(
                    'authoritative contact snapshot stamps are not strictly increasing'
                )
            gap_ns = stamp - previous_stamp
            maximum_public_gap_ns = max(maximum_public_gap_ns, gap_ns)
            if gap_ns > _MAX_CONTACT_SNAPSHOT_GAP_NS:
                raise MetricUnavailable('authoritative contact snapshot gap exceeded 220 ms')
        previous_sequence = sequence
        previous_stamp = stamp
        if message.get('frame_id') != '':
            raise MetricUnavailable('authoritative contact snapshot frame_id must be empty')
        delivery_clock_stamp = require_int(
            message.get('delivery_clock_stamp_ns'),
            f'messages[{message_index}].delivery_clock_stamp_ns',
        )
        delivery_clock_offset = require_int(
            message.get('delivery_clock_offset_ns'),
            f'messages[{message_index}].delivery_clock_offset_ns',
        )
        if delivery_clock_stamp - stamp != delivery_clock_offset:
            raise MetricUnavailable(
                'authoritative contact snapshot delivery clock offset is inconsistent'
            )
        contacts = message.get('contacts')
        if not isinstance(contacts, list) or not 1 <= len(contacts) <= 16:
            raise MetricUnavailable(
                'authoritative contact snapshot must contain between one and 16 records'
            )
        public_snapshot_count += 1
        snapshot_contact_record_count += len(contacts)
        if snapshot_contact_record_count > CONTACT_RECORD_CAPACITY:
            raise MetricUnavailable('normalized contact snapshot record capacity exceeded')
        if stamp == drain:
            drain_snapshot_observed = True
        if stamp <= start:
            latest_snapshot_at_or_before_start = stamp
        if stamp > drain:
            continue
        if start <= stamp <= terminal:
            in_action_snapshot_count += 1
        grouped: dict[str, dict[str, Any]] = {}
        for contact_index, contact in enumerate(contacts):
            if not isinstance(contact, Mapping) or set(contact) != {
                'collision1',
                'collision2',
                'maximum_normal_force_n',
                'maximum_penetration_depth_m',
            }:
                raise MetricUnavailable(
                    f'messages[{message_index}].contacts[{contact_index}] is not a v3 '
                    'snapshot record'
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
                    'maximum_delivered_snapshot_normal_force_n': None,
                    'maximum_delivered_snapshot_penetration_depth_m': None,
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
            depth_value = contact.get('maximum_penetration_depth_m')
            depth = (
                None
                if depth_value is None
                else require_finite(depth_value, 'contact.maximum_penetration_depth_m')
            )
            if depth is not None and depth < 0.0:
                raise MetricUnavailable('contact depth cannot be negative')
            force = contact.get('maximum_normal_force_n')
            if force is not None:
                force = require_finite(force, 'contact.maximum_normal_force_n')
                if force < 0.0:
                    raise MetricUnavailable('contact force cannot be negative')
            if depth is not None:
                group['maximum_delivered_snapshot_penetration_depth_m'] = max(
                    depth,
                    group['maximum_delivered_snapshot_penetration_depth_m'] or 0.0,
                )
            if force is not None:
                group['maximum_delivered_snapshot_normal_force_n'] = max(
                    force,
                    group['maximum_delivered_snapshot_normal_force_n'] or 0.0,
                )
        for counterpart in sorted(set(active) - set(grouped)):
            close_absent(counterpart, stamp)
        if stamp < start:
            pre_action_snapshot_records += sum(group['record_count'] for group in grouped.values())
        for counterpart, group in grouped.items():
            if stamp > terminal and counterpart not in active:
                post_terminal_snapshot_records += group['record_count']
                continue
            event = active.get(counterpart)
            if event is None:
                event = {
                    'counterpart_model': counterpart,
                    'last_snapshot_stamp_ns': stamp,
                    'maximum_delivered_snapshot_normal_force_n': group[
                        'maximum_delivered_snapshot_normal_force_n'
                    ],
                    'maximum_delivered_snapshot_penetration_depth_m': group[
                        'maximum_delivered_snapshot_penetration_depth_m'
                    ],
                    'pairs': set(group['pairs']),
                    'post_terminal_sampled_snapshot_record_count': 0,
                    'sampled_snapshot_record_count': group['record_count'],
                    'start_stamp_ns': stamp,
                }
                active[counterpart] = event
            else:
                event['last_snapshot_stamp_ns'] = stamp
                event['pairs'].update(group['pairs'])
                event['sampled_snapshot_record_count'] += group['record_count']
                for field in (
                    'maximum_delivered_snapshot_normal_force_n',
                    'maximum_delivered_snapshot_penetration_depth_m',
                ):
                    value = group[field]
                    if value is not None:
                        event[field] = max(value, event[field] or 0.0)
            if stamp > terminal:
                event['post_terminal_sampled_snapshot_record_count'] += group['record_count']
    if not drain_snapshot_observed:
        raise MetricUnavailable('capture does not contain the exact qualifying drain snapshot')
    if (
        latest_snapshot_at_or_before_start is None
        or start - latest_snapshot_at_or_before_start > _MAX_CONTACT_SNAPSHOT_GAP_NS
    ):
        raise MetricUnavailable(
            'accepted-goal T0 lacks a fresh authoritative snapshot at or before T0'
        )
    if in_action_snapshot_count == 0:
        raise MetricUnavailable('contact snapshot stream is silent during the mission interval')
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
        'contact_stream_semantics': 'authoritative_delivered_active_pair_snapshot',
        'drain_completed_stamp_ns': drain,
        'events': normalized_events,
        'excluded_contact_counts': dict(sorted(exclusions.items())),
        'in_action_snapshot_count': in_action_snapshot_count,
        'maximum_public_snapshot_gap_ns': maximum_public_gap_ns,
        'post_terminal_snapshot_record_count': post_terminal_snapshot_records,
        'pre_action_snapshot_record_count': pre_action_snapshot_records,
        'public_snapshot_count': public_snapshot_count,
        'qualification': qualification,
        'qualifying_contact_snapshot_stamp_ns': drain,
        'required_drain_stamp_ns': required_drain,
        'snapshot_contact_record_count': snapshot_contact_record_count,
    }
