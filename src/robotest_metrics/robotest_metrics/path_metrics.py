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

"""Ground-truth path and per-waypoint Nav2 plan analysis."""

from __future__ import annotations

import hashlib
import math
import struct
from bisect import bisect_left
from collections.abc import Mapping, Sequence
from itertools import pairwise
from typing import Any

from robotest_metrics.constants import (
    ALIGNMENT_GAP_NS,
    MICROMETRES_PER_METRE,
    PATH_LENGTH_EPSILON_M,
    PLAN_GOAL_TOLERANCE_M,
    PLAN_HASH_VERSION,
)
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import (
    ensure_strictly_increasing,
    interpolate_angle,
    planar_distance,
    require_finite,
    require_int,
)

_INT64_MAX = (1 << 63) - 1
_INT64_MIN = -(1 << 63)
_TERMINAL_SEQUENCE = (1 << 63) - 1


def _pose(sample: Mapping[str, Any], name: str) -> dict[str, float]:
    result = {
        'x_m': require_finite(sample.get('x_m'), f'{name}.x_m'),
        'y_m': require_finite(sample.get('y_m'), f'{name}.y_m'),
    }
    if 'yaw_rad' in sample:
        result['yaw_rad'] = require_finite(sample.get('yaw_rad'), f'{name}.yaw_rad')
    return result


def interpolate_pose(
    samples: Sequence[Mapping[str, Any]],
    stamp_ns: int,
    *,
    max_gap_ns: int = ALIGNMENT_GAP_NS,
) -> dict[str, Any]:
    """Return an exact or linearly interpolated planar pose at ``stamp_ns``."""
    if max_gap_ns <= 0:
        raise ValueError('max_gap_ns must be positive')
    if not samples:
        raise MetricUnavailable('pose stream is empty')
    ensure_strictly_increasing(samples)
    target = require_int(stamp_ns, 'stamp_ns')
    stamps = [require_int(sample.get('stamp_ns'), 'sample.stamp_ns') for sample in samples]
    index = bisect_left(stamps, target)
    if index < len(samples) and stamps[index] == target:
        exact = _pose(samples[index], f'samples[{index}]')
        return {
            **exact,
            'stamp_ns': target,
            'interpolated': False,
            'source_before_stamp_ns': target,
            'source_after_stamp_ns': target,
            'source_gap_ns': 0,
        }
    if index == 0 or index == len(samples):
        raise MetricUnavailable(f'pose stream does not bracket {target}')
    before = samples[index - 1]
    after = samples[index]
    before_stamp = stamps[index - 1]
    after_stamp = stamps[index]
    gap = after_stamp - before_stamp
    if gap > max_gap_ns:
        raise MetricUnavailable(f'pose bracket gap {gap} exceeds {max_gap_ns}')
    fraction = (target - before_stamp) / gap
    before_pose = _pose(before, f'samples[{index - 1}]')
    after_pose = _pose(after, f'samples[{index}]')
    result: dict[str, Any] = {
        'x_m': before_pose['x_m'] + fraction * (after_pose['x_m'] - before_pose['x_m']),
        'y_m': before_pose['y_m'] + fraction * (after_pose['y_m'] - before_pose['y_m']),
        'stamp_ns': target,
        'interpolated': True,
        'source_before_stamp_ns': before_stamp,
        'source_after_stamp_ns': after_stamp,
        'source_gap_ns': gap,
    }
    if 'yaw_rad' in before_pose and 'yaw_rad' in after_pose:
        result['yaw_rad'] = interpolate_angle(
            before_pose['yaw_rad'],
            after_pose['yaw_rad'],
            fraction,
        )
    return result


def actual_path_metrics(
    samples: Sequence[Mapping[str, Any]],
    accepted_goal_stamp_ns: int,
    terminal_action_stamp_ns: int,
    *,
    max_gap_ns: int = ALIGNMENT_GAP_NS,
) -> dict[str, Any]:
    """Integrate the exact ground-truth polyline over the action interval."""
    start = require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns')
    end = require_int(terminal_action_stamp_ns, 'terminal_action_stamp_ns')
    if end <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    ensure_strictly_increasing(samples)
    start_pose = interpolate_pose(samples, start, max_gap_ns=max_gap_ns)
    end_pose = interpolate_pose(samples, end, max_gap_ns=max_gap_ns)
    interval = [start_pose]
    maximum_gap_ns = 0
    previous_stamp = start
    raw_inside = 0
    for index, sample in enumerate(samples):
        stamp = require_int(sample.get('stamp_ns'), f'samples[{index}].stamp_ns')
        if start < stamp < end:
            pose = _pose(sample, f'samples[{index}]')
            interval.append({'stamp_ns': stamp, **pose, 'interpolated': False})
            raw_inside += 1
            maximum_gap_ns = max(maximum_gap_ns, stamp - previous_stamp)
            previous_stamp = stamp
    maximum_gap_ns = max(maximum_gap_ns, end - previous_stamp)
    if maximum_gap_ns > max_gap_ns:
        raise MetricUnavailable(f'ground-truth gap {maximum_gap_ns} exceeds {max_gap_ns}')
    interval.append(end_pose)
    length_m = sum(planar_distance(first, second) for first, second in pairwise(interval))
    if not math.isfinite(length_m):
        raise MetricUnavailable('actual path length is non-finite')
    return {
        'actual_path_length_m': length_m,
        'boundary_end': end_pose,
        'boundary_start': start_pose,
        'first_stamp_ns': start,
        'in_interval_raw_sample_count': raw_inside,
        'integrated_sample_count': len(interval),
        'last_stamp_ns': end,
        'maximum_gap_ns': maximum_gap_ns,
        'raw_sample_count': len(samples),
        'samples': interval,
    }


def plan_length_m(poses: Sequence[Mapping[str, Any]]) -> float:
    """Sum raw finite planar distances in one plan."""
    if len(poses) < 2:
        raise MetricUnavailable('a valid plan requires at least two poses')
    normalized = [_pose(pose, f'poses[{index}]') for index, pose in enumerate(poses)]
    length = sum(planar_distance(first, second) for first, second in pairwise(normalized))
    if length <= PATH_LENGTH_EPSILON_M:
        raise MetricUnavailable('plan length is not above path_length_epsilon_m')
    return length


def quantize_micrometres(value_m: Any) -> int:
    """Round metres to signed integer micrometres, half away from zero."""
    value = require_finite(value_m, 'plan coordinate')
    if value == 0.0:
        return 0
    scaled = abs(value) * MICROMETRES_PER_METRE
    if not math.isfinite(scaled):
        raise MetricUnavailable('quantized plan coordinate does not fit signed int64')
    magnitude = math.floor(scaled + 0.5)
    result = magnitude if value > 0.0 else -magnitude
    if not _INT64_MIN <= result <= _INT64_MAX:
        raise MetricUnavailable('quantized plan coordinate does not fit signed int64')
    return result


def canonical_plan_points(poses: Sequence[Mapping[str, Any]]) -> list[tuple[int, int]]:
    """Quantize and collapse consecutive identical planar points."""
    points: list[tuple[int, int]] = []
    for index, pose in enumerate(poses):
        point = (
            quantize_micrometres(pose.get('x_m')),
            quantize_micrometres(pose.get('y_m')),
        )
        if not points or points[-1] != point:
            points.append(point)
        if index >= (1 << 63) - 1:
            raise MetricUnavailable('plan pose count exceeds uint64')
    return points


def plan_geometry_hash(frame_id: Any, poses: Sequence[Mapping[str, Any]]) -> str:
    """Hash versioned, quantized planar path geometry."""
    if frame_id != 'map':
        raise MetricUnavailable('plan frame_id must be map')
    frame = frame_id.encode('utf-8')
    if len(frame) > (1 << 32) - 1:
        raise MetricUnavailable('plan frame_id is too long')
    points = canonical_plan_points(poses)
    digest = hashlib.sha256()
    digest.update(PLAN_HASH_VERSION)
    digest.update(struct.pack('!I', len(frame)))
    digest.update(frame)
    digest.update(struct.pack('!Q', len(points)))
    for x_um, y_um in points:
        digest.update(struct.pack('!qq', x_um, y_um))
    return digest.hexdigest()


def _event_order(sample: Mapping[str, Any], name: str) -> tuple[int, int]:
    return (
        require_int(sample.get('stamp_ns'), f'{name}.stamp_ns'),
        require_int(sample.get('collector_sequence'), f'{name}.collector_sequence'),
    )


def _feedback_transitions(
    feedback: Sequence[Mapping[str, Any]],
    waypoint_count: int,
    start: int,
    end: int,
) -> list[dict[str, int]]:
    if waypoint_count <= 0:
        raise MetricUnavailable('waypoint list is empty')
    transitions: list[dict[str, int]] = []
    previous_order: tuple[int, int] | None = None
    last_index: int | None = None
    for position, sample in enumerate(feedback):
        order = _event_order(sample, f'feedback[{position}]')
        if previous_order is not None and order <= previous_order:
            raise MetricUnavailable('feedback is not in deterministic collector order')
        previous_order = order
        if order[0] < start or order[0] > end:
            continue
        waypoint_index = require_int(
            sample.get('current_waypoint'),
            f'feedback[{position}].current_waypoint',
        )
        if not 0 <= waypoint_index < waypoint_count:
            raise MetricUnavailable('feedback waypoint index is out of range')
        if last_index is None:
            if waypoint_index != 0:
                raise MetricUnavailable('feedback must begin at waypoint zero')
            transitions.append(
                {
                    'collector_sequence': order[1],
                    'current_waypoint': waypoint_index,
                    'stamp_ns': order[0],
                }
            )
            last_index = waypoint_index
            continue
        if waypoint_index == last_index:
            continue
        if waypoint_index != last_index + 1:
            raise MetricUnavailable('feedback waypoint index regressed or skipped')
        transitions.append(
            {
                'collector_sequence': order[1],
                'current_waypoint': waypoint_index,
                'stamp_ns': order[0],
            }
        )
        last_index = waypoint_index
    if last_index != waypoint_count - 1:
        raise MetricUnavailable('feedback does not establish every waypoint leg')
    return transitions


def analyze_plans(
    plans: Sequence[Mapping[str, Any]],
    feedback: Sequence[Mapping[str, Any]],
    waypoints: Sequence[Mapping[str, Any]],
    accepted_goal_stamp_ns: int,
    terminal_action_stamp_ns: int,
) -> dict[str, Any]:
    """Compute cumulative first-valid-per-leg plan metrics and replans."""
    start = require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns')
    end = require_int(terminal_action_stamp_ns, 'terminal_action_stamp_ns')
    if end <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    transitions = _feedback_transitions(feedback, len(waypoints), start, end)
    starts: list[tuple[int, int]] = [(start, -1)]
    starts.extend(
        (transition['stamp_ns'], transition['collector_sequence'])
        for transition in transitions
        if transition['current_waypoint'] > 0
    )
    ends = [*starts[1:], (end, _TERMINAL_SEQUENCE)]
    per_leg: list[list[dict[str, Any]]] = [[] for _ in waypoints]
    diagnostics: list[dict[str, Any]] = []
    previous_order: tuple[int, int] | None = None
    for plan_index, plan in enumerate(plans):
        order = _event_order(plan, f'plans[{plan_index}]')
        if previous_order is not None and order <= previous_order:
            raise MetricUnavailable('plans are not in deterministic collector order')
        previous_order = order
        if order < starts[0] or order > ends[-1]:
            diagnostics.append({'plan_index': plan_index, 'reason': 'outside_action_interval'})
            continue
        leg_index = next(
            (
                index
                for index, (leg_start, leg_end) in enumerate(zip(starts, ends, strict=True))
                if leg_start <= order < leg_end
            ),
            None,
        )
        if leg_index is None:
            diagnostics.append({'plan_index': plan_index, 'reason': 'no_feedback_leg'})
            continue
        try:
            frame_id = plan.get('frame_id')
            poses = plan.get('poses')
            if not isinstance(poses, list):
                raise MetricUnavailable('plan poses must be a list')
            length = plan_length_m(poses)
            geometry_hash = plan_geometry_hash(frame_id, poses)
            endpoint_error = planar_distance(poses[-1], waypoints[leg_index])
            if endpoint_error > PLAN_GOAL_TOLERANCE_M:
                raise MetricUnavailable('plan endpoint does not match feedback-derived waypoint')
        except MetricUnavailable as exc:
            diagnostics.append({'plan_index': plan_index, 'reason': str(exc)})
            continue
        per_leg[leg_index].append(
            {
                'collector_sequence': order[1],
                'endpoint_error_m': endpoint_error,
                'geometry_sha256': geometry_hash,
                'length_m': length,
                'plan_index': plan_index,
                'stamp_ns': order[0],
                'waypoint_index': leg_index,
            }
        )
    missing = [index for index, entries in enumerate(per_leg) if not entries]
    if missing:
        raise MetricUnavailable(f'no valid matching plan for waypoint legs {missing}')
    leg_results: list[dict[str, Any]] = []
    flattened: list[dict[str, Any]] = []
    initial_total = 0.0
    latest_total = 0.0
    total_replans = 0
    for leg_index, entries in enumerate(per_leg):
        replans = 0
        last_hash = entries[0]['geometry_sha256']
        for entry in entries[1:]:
            if entry['geometry_sha256'] != last_hash:
                replans += 1
                last_hash = entry['geometry_sha256']
        initial_total += entries[0]['length_m']
        latest_total += entries[-1]['length_m']
        total_replans += replans
        leg_results.append(
            {
                'initial_plan': entries[0],
                'latest_plan': entries[-1],
                'plan_count': len(entries),
                'replan_count': replans,
                'waypoint_index': leg_index,
            }
        )
        flattened.extend(entries)
    return {
        'diagnostics': diagnostics,
        'initial_planned_path_length_m': initial_total,
        'latest_planned_path_length_m': latest_total,
        'leg_results': leg_results,
        'planned_path_lengths_m': flattened,
        'replan_count': total_replans,
    }


def path_efficiency(
    initial_planned_path_length_m: Any, actual_path_length_m: Any
) -> dict[str, float]:
    """Return unclamped efficiency and overrun ratios."""
    planned = require_finite(initial_planned_path_length_m, 'initial_planned_path_length_m')
    actual = require_finite(actual_path_length_m, 'actual_path_length_m')
    if planned <= PATH_LENGTH_EPSILON_M or actual <= PATH_LENGTH_EPSILON_M:
        raise MetricUnavailable('path lengths must be above path_length_epsilon_m')
    return {
        'path_efficiency': planned / actual,
        'path_overrun_ratio': actual / planned,
    }
