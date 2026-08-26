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

"""LiDAR-dropout reconciliation and conservative recovery-window analysis."""

from __future__ import annotations

import bisect
from collections.abc import Mapping, Sequence
from itertools import pairwise
from typing import Any

from robotest_metrics.constants import (
    ALIGNMENT_GAP_NS,
    RECOVERY_LIFECYCLE_MARGIN_NS,
    RECOVERY_PROGRESS_METRES,
    RECOVERY_PROGRESS_YAW_RAD,
    RECOVERY_SCAN_GAP_NS,
    RECOVERY_WINDOW_NS,
)
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import normalize_angle, planar_distance, require_int
from robotest_metrics.path_metrics import interpolate_pose

_FAILURE_STATUSES = {
    'ABORTED',
    'CANCELED',
    'INFRASTRUCTURE_ERROR',
    'REJECTED',
    'TIMED_OUT',
}


def supervisor_recovery_metrics(events: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Measure one structured supervisor outage and count proven restart cycles."""
    starts = [
        index for index, event in enumerate(events) if event.get('kind') == 'supervisor_started'
    ]
    if not starts:
        raise MetricUnavailable('supervisor_started event is missing')
    segment_start = starts[-1]
    segment = events[segment_start:]
    ordered: list[tuple[int, int, Mapping[str, Any]]] = []
    previous_sequence: int | None = None
    previous_steady_wall_ns: int | None = None
    for offset, event in enumerate(segment):
        index = segment_start + offset
        if event.get('schema_version') != 1:
            raise MetricUnavailable(f'events[{index}].schema_version must be 1')
        sequence = require_int(event.get('sequence'), f'events[{index}].sequence')
        steady_wall_ns = require_int(
            event.get('steady_wall_ns'),
            f'events[{index}].steady_wall_ns',
        )
        if sequence < 0 or steady_wall_ns < 0:
            raise MetricUnavailable('supervisor event counters must be nonnegative')
        if previous_sequence is not None and sequence <= previous_sequence:
            raise MetricUnavailable('supervisor event sequence is not strictly increasing')
        if previous_steady_wall_ns is not None and steady_wall_ns < previous_steady_wall_ns:
            raise MetricUnavailable(
                'supervisor steady-wall time regressed within the latest segment'
            )
        ordered.append((sequence, steady_wall_ns, event))
        previous_sequence = sequence
        previous_steady_wall_ns = steady_wall_ns
    failure = next((item for item in ordered if item[2].get('kind') == 'failure_detected'), None)
    if failure is None:
        raise MetricUnavailable('supervisor failure_detected event is missing')
    restored = next(
        (
            item
            for item in ordered
            if item[0] > failure[0]
            and item[2].get('kind') == 'readiness_changed'
            and item[2].get('ready') is True
        ),
        None,
    )
    if restored is None:
        raise MetricUnavailable('supervisor readiness restoration is missing')
    interval = [item for item in ordered if failure[0] <= item[0] <= restored[0]]
    false_transition = next(
        (
            item
            for item in interval
            if item[2].get('kind') == 'readiness_changed' and item[2].get('ready') is False
        ),
        None,
    )
    if false_transition is None:
        raise MetricUnavailable('readiness false transition is missing during outage')
    premature_true = [
        item
        for item in interval
        if item[0] < restored[0]
        and item[2].get('kind') == 'readiness_changed'
        and item[2].get('ready') is True
    ]
    if premature_true:
        raise MetricUnavailable('readiness became true before the selected restoration')
    restart_count = 0
    pending_schedule = False
    restart_pairs: list[dict[str, int]] = []
    scheduled_stamp: int | None = None
    for _, stamp, event in interval:
        kind = event.get('kind')
        if kind == 'restart_scheduled':
            if pending_schedule:
                raise MetricUnavailable('restart scheduled twice without a child start')
            pending_schedule = True
            scheduled_stamp = stamp
        elif kind == 'child_started' and pending_schedule:
            assert scheduled_stamp is not None
            restart_count += 1
            restart_pairs.append(
                {'child_started_wall_ns': stamp, 'restart_scheduled_wall_ns': scheduled_stamp}
            )
            pending_schedule = False
            scheduled_stamp = None
    if pending_schedule:
        raise MetricUnavailable('restart schedule has no child start before readiness restoration')
    if restart_count == 0:
        raise MetricUnavailable('no scheduled restart followed by a new child start')
    return {
        'failure_detected_wall_ns': failure[1],
        'latest_supervisor_started_sequence': ordered[0][0],
        'readiness_false_wall_ns': false_transition[1],
        'ready_restored_wall_ns': restored[1],
        'restart_count': restart_count,
        'restart_pairs': restart_pairs,
        'supervisor_recovery_time_wall_s': (restored[1] - failure[1]) / 1_000_000_000,
    }


def _strict_stamps(samples: Sequence[Mapping[str, Any]], name: str) -> list[int]:
    stamps: list[int] = []
    previous: int | None = None
    for index, sample in enumerate(samples):
        stamp = require_int(sample.get('stamp_ns'), f'{name}[{index}].stamp_ns')
        if previous is not None and stamp <= previous:
            raise MetricUnavailable(f'{name} stamps must be unique and increasing')
        stamps.append(stamp)
        previous = stamp
    return stamps


def _ordered_callback_samples(samples: Sequence[Mapping[str, Any]], name: str) -> None:
    previous: tuple[int, int] | None = None
    for index, sample in enumerate(samples):
        stamp = require_int(sample.get('stamp_ns'), f'{name}[{index}].stamp_ns')
        sequence = require_int(
            sample.get('collector_sequence'),
            f'{name}[{index}].collector_sequence',
        )
        order = (stamp, sequence)
        if previous is not None and order <= previous:
            raise MetricUnavailable(f'{name} is not in deterministic callback order')
        previous = order


def analyze_lidar_dropout(
    raw_scans: Sequence[Mapping[str, Any]],
    validated_scans: Sequence[Mapping[str, Any]],
    configured_activation_stamp_ns: int,
    configured_deactivation_stamp_ns: int,
    *,
    proxy_affected_message_count: int | None = None,
    proxy_first_affected_stamp_ns: int | None = None,
    proxy_last_affected_stamp_ns: int | None = None,
    proxy_restored_stamp_ns: int | None = None,
) -> dict[str, Any]:
    """Prove omission during a half-open interval and exact first restoration."""
    activation = require_int(configured_activation_stamp_ns, 'configured_activation_stamp_ns')
    deactivation = require_int(
        configured_deactivation_stamp_ns,
        'configured_deactivation_stamp_ns',
    )
    if deactivation <= activation:
        raise MetricUnavailable('dropout deactivation must be after activation')
    raw_stamps = _strict_stamps(raw_scans, 'raw_scans')
    validated_stamps = _strict_stamps(validated_scans, 'validated_scans')
    affected = [stamp for stamp in raw_stamps if activation <= stamp < deactivation]
    leaked = [stamp for stamp in validated_stamps if activation <= stamp < deactivation]
    if not affected:
        raise MetricUnavailable('raw scans do not continue through the dropout interval')
    if leaked:
        raise MetricUnavailable('validated scans were published during LiDAR dropout')
    if proxy_affected_message_count is not None:
        count = require_int(proxy_affected_message_count, 'proxy_affected_message_count')
        if count != len(affected):
            raise MetricUnavailable('proxy affected-message count does not reconcile')
    if proxy_first_affected_stamp_ns is None or proxy_last_affected_stamp_ns is None:
        raise MetricUnavailable('proxy affected-message boundary events are missing')
    if (
        require_int(proxy_first_affected_stamp_ns, 'proxy_first_affected_stamp_ns') != affected[0]
        or require_int(proxy_last_affected_stamp_ns, 'proxy_last_affected_stamp_ns') != affected[-1]
    ):
        raise MetricUnavailable('proxy affected-message boundary stamps do not reconcile')
    restored_raw = next(
        (
            sample
            for sample in raw_scans
            if require_int(sample.get('stamp_ns'), 'raw.stamp_ns') >= deactivation
        ),
        None,
    )
    if restored_raw is None:
        raise MetricUnavailable('no raw scan exists at or after dropout deactivation')
    restored_stamp = require_int(restored_raw.get('stamp_ns'), 'restored_raw.stamp_ns')
    restored_validated = next(
        (
            sample
            for sample in validated_scans
            if require_int(sample.get('stamp_ns'), 'validated.stamp_ns') == restored_stamp
        ),
        None,
    )
    if restored_validated is None:
        raise MetricUnavailable('first post-dropout raw scan was not restored')
    raw_hash = restored_raw.get('payload_sha256')
    validated_hash = restored_validated.get('payload_sha256')
    if not isinstance(raw_hash, str) or raw_hash != validated_hash:
        raise MetricUnavailable('first restored scan does not match raw payload')
    if proxy_restored_stamp_ns is None:
        raise MetricUnavailable('proxy restoration event is missing')
    if require_int(proxy_restored_stamp_ns, 'proxy_restored_stamp_ns') != restored_stamp:
        raise MetricUnavailable('proxy restoration stamp does not reconcile')
    return {
        'actual_deactivation_stamp_ns': restored_stamp,
        'actual_first_affected_stamp_ns': affected[0],
        'actual_last_affected_stamp_ns': affected[-1],
        'affected_message_count': len(affected),
        'configured_activation_stamp_ns': activation,
        'configured_deactivation_stamp_ns': deactivation,
        'raw_active_interval_count': len(affected),
        'proxy_restored_stamp_ns': proxy_restored_stamp_ns,
        'validated_active_interval_count': 0,
    }


def _scan_window(
    stamps: Sequence[int],
    start: int,
    end: int,
    maximum_gap_ns: int,
) -> tuple[bool, dict[str, Any]]:
    before_index = bisect.bisect_right(stamps, start) - 1
    if before_index < 0:
        return False, {'reason': 'no_restored_scan_at_or_before_window_start'}
    end_index = bisect.bisect_right(stamps, end)
    relevant = list(stamps[before_index:end_index])
    if not relevant or end - relevant[-1] > maximum_gap_ns:
        return False, {'reason': 'restored_scan_is_stale_at_window_end'}
    gaps = [right - left for left, right in pairwise(relevant)]
    if any(gap <= 0 or gap > maximum_gap_ns for gap in gaps):
        return False, {'reason': 'restored_scan_gap_exceeded'}
    return True, {
        'maximum_gap_ns': max(gaps, default=0),
        'sample_count': len(relevant),
    }


def _lifecycle_window(
    samples: Sequence[Mapping[str, Any]],
    required_nodes: Sequence[str],
    start: int,
    end: int,
    margin_ns: int,
) -> tuple[bool, dict[str, Any]]:
    evidence: dict[str, Any] = {}
    for node in required_nodes:
        node_samples = [sample for sample in samples if sample.get('node') == node]
        before = [
            sample
            for sample in node_samples
            if start - margin_ns
            <= require_int(sample.get('stamp_ns'), 'lifecycle.stamp_ns')
            <= start
        ]
        after = [
            sample
            for sample in node_samples
            if end <= require_int(sample.get('stamp_ns'), 'lifecycle.stamp_ns') <= end + margin_ns
        ]
        between = [
            sample
            for sample in node_samples
            if start <= require_int(sample.get('stamp_ns'), 'lifecycle.stamp_ns') <= end
        ]
        if not before or before[-1].get('state') != 'active':
            return False, {'node': node, 'reason': 'missing_active_start_snapshot'}
        if not after or after[0].get('state') != 'active':
            return False, {'node': node, 'reason': 'missing_active_end_snapshot'}
        if any(sample.get('state') != 'active' for sample in between):
            return False, {'node': node, 'reason': 'transitioned_away_from_active'}
        evidence[node] = {
            'end_snapshot_stamp_ns': require_int(after[0].get('stamp_ns'), 'after.stamp_ns'),
            'start_snapshot_stamp_ns': require_int(before[-1].get('stamp_ns'), 'before.stamp_ns'),
        }
    return True, evidence


def _progress_window(
    ground_truth: Sequence[Mapping[str, Any]],
    feedback: Sequence[Mapping[str, Any]],
    mission_terminal: Mapping[str, Any] | None,
    start: int,
    end: int,
) -> tuple[bool, dict[str, Any]]:
    if mission_terminal is not None:
        terminal_stamp = require_int(mission_terminal.get('stamp_ns'), 'mission_terminal.stamp_ns')
        if mission_terminal.get('status') == 'SUCCEEDED' and terminal_stamp <= end:
            return True, {'predicate': 'mission_succeeded', 'stamp_ns': terminal_stamp}
    try:
        first = interpolate_pose(ground_truth, start, max_gap_ns=ALIGNMENT_GAP_NS)
        last = interpolate_pose(ground_truth, end, max_gap_ns=ALIGNMENT_GAP_NS)
        displacement = planar_distance(first, last)
        yaw_delta = abs(normalize_angle(last.get('yaw_rad', 0.0) - first.get('yaw_rad', 0.0)))
        if displacement >= RECOVERY_PROGRESS_METRES:
            return True, {'predicate': 'planar_displacement', 'value_m': displacement}
        if yaw_delta >= RECOVERY_PROGRESS_YAW_RAD:
            return True, {'predicate': 'yaw_change', 'value_rad': yaw_delta}
    except MetricUnavailable:
        pass
    transitions = [
        sample
        for sample in feedback
        if start <= require_int(sample.get('stamp_ns'), 'feedback.stamp_ns') <= end
    ]
    for sample in transitions:
        current = require_int(sample.get('current_waypoint'), 'feedback.current_waypoint')
        previous = sample.get('previous_waypoint')
        if previous is None:
            previous = sample.get('previous_value')
        if previous is not None:
            prior = require_int(previous, 'feedback.previous_waypoint')
            if current > prior:
                return True, {
                    'from': prior,
                    'predicate': 'waypoint_advance',
                    'to': current,
                }
    indices = [
        require_int(sample.get('current_waypoint'), 'feedback.current_waypoint')
        for sample in transitions
    ]
    if indices and max(indices) > min(indices):
        return True, {'from': min(indices), 'predicate': 'waypoint_advance', 'to': max(indices)}
    return False, {'reason': 'no_navigation_progress'}


def sensor_recovery_metrics(
    actual_deactivation_stamp_ns: int,
    validated_scans: Sequence[Mapping[str, Any]],
    lifecycle_samples: Sequence[Mapping[str, Any]],
    required_lifecycle_nodes: Sequence[str],
    ground_truth: Sequence[Mapping[str, Any]],
    feedback: Sequence[Mapping[str, Any]],
    collision_events: Sequence[Mapping[str, Any]],
    mission_terminal: Mapping[str, Any] | None,
    *,
    window_ns: int = RECOVERY_WINDOW_NS,
    maximum_scan_gap_ns: int = RECOVERY_SCAN_GAP_NS,
    lifecycle_margin_ns: int = RECOVERY_LIFECYCLE_MARGIN_NS,
) -> dict[str, Any]:
    """Find the first fully observed stable recovery window."""
    deactivation = require_int(actual_deactivation_stamp_ns, 'actual_deactivation_stamp_ns')
    if window_ns <= 0 or maximum_scan_gap_ns <= 0 or lifecycle_margin_ns < 0:
        raise ValueError('recovery timing parameters are invalid')
    if not required_lifecycle_nodes:
        raise MetricUnavailable('required lifecycle node set is empty')
    if mission_terminal is not None and mission_terminal.get('status') in _FAILURE_STATUSES:
        raise MetricUnavailable('bound mission eventually failed')
    _ordered_callback_samples(lifecycle_samples, 'lifecycle_samples')
    _ordered_callback_samples(feedback, 'feedback')
    last_waypoint: int | None = None
    for sample in feedback:
        current_waypoint = require_int(
            sample.get('current_waypoint'),
            'feedback.current_waypoint',
        )
        if last_waypoint is not None and current_waypoint < last_waypoint:
            raise MetricUnavailable('feedback waypoint index regressed')
        last_waypoint = current_waypoint
    scan_stamps = _strict_stamps(validated_scans, 'validated_scans')
    restored_stamps = [stamp for stamp in scan_stamps if stamp >= deactivation]
    if not restored_stamps:
        raise MetricUnavailable('no restored validated scans exist')
    candidate_ends = {deactivation + window_ns}
    for collection in (validated_scans, ground_truth, feedback, lifecycle_samples):
        for sample in collection:
            stamp = require_int(sample.get('stamp_ns'), 'candidate.stamp_ns')
            if stamp >= deactivation + window_ns:
                candidate_ends.add(stamp)
    if mission_terminal is not None:
        stamp = require_int(mission_terminal.get('stamp_ns'), 'mission_terminal.stamp_ns')
        if stamp >= deactivation + window_ns:
            candidate_ends.add(stamp)
    rejected: list[dict[str, Any]] = []
    for end in sorted(candidate_ends):
        start = end - window_ns
        scan_ok, scan_evidence = _scan_window(
            restored_stamps,
            start,
            end,
            maximum_scan_gap_ns,
        )
        if not scan_ok:
            rejected.append({'end_stamp_ns': end, **scan_evidence})
            continue
        lifecycle_ok, lifecycle_evidence = _lifecycle_window(
            lifecycle_samples,
            required_lifecycle_nodes,
            start,
            end,
            lifecycle_margin_ns,
        )
        if not lifecycle_ok:
            rejected.append({'end_stamp_ns': end, **lifecycle_evidence})
            continue
        collision = next(
            (
                event
                for event in collision_events
                if start
                <= require_int(event.get('start_stamp_ns'), 'collision.start_stamp_ns')
                <= end
            ),
            None,
        )
        if collision is not None:
            rejected.append({'end_stamp_ns': end, 'reason': 'collision_in_window'})
            continue
        progress_ok, progress_evidence = _progress_window(
            ground_truth,
            feedback,
            mission_terminal,
            start,
            end,
        )
        if not progress_ok:
            rejected.append({'end_stamp_ns': end, **progress_evidence})
            continue
        return {
            'lifecycle_evidence': lifecycle_evidence,
            'progress': progress_evidence,
            'recovered_stamp_ns': end,
            'rejected_candidate_count': len(rejected),
            'rejected_candidates': rejected,
            'scan_evidence': scan_evidence,
            'sensor_recovery_time_sim_s': (end - deactivation) / 1_000_000_000,
            'window_end_stamp_ns': end,
            'window_start_stamp_ns': start,
        }
    raise MetricUnavailable('no qualifying sensor-recovery window')
