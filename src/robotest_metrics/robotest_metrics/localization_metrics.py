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

"""Ground-truth-grid localization error using exact-stamp TF composition."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from robotest_metrics.constants import ALIGNMENT_GAP_NS, LOCALIZATION_MINIMUM_COVERAGE
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import (
    compose_se2,
    normalize_angle,
    require_finite,
    require_int,
)
from robotest_metrics.path_metrics import interpolate_pose
from robotest_metrics.statistics import numeric_summary


def _truth_pose(sample: Mapping[str, Any], index: int) -> dict[str, float]:
    return {
        'x_m': require_finite(sample.get('x_m'), f'ground_truth[{index}].x_m'),
        'y_m': require_finite(sample.get('y_m'), f'ground_truth[{index}].y_m'),
        'yaw_rad': require_finite(sample.get('yaw_rad'), f'ground_truth[{index}].yaw_rad'),
    }


def localization_error_metrics(
    ground_truth: Sequence[Mapping[str, Any]],
    map_to_odom: Sequence[Mapping[str, Any]],
    odom_to_base: Sequence[Mapping[str, Any]],
    accepted_goal_stamp_ns: int,
    terminal_action_stamp_ns: int,
    *,
    max_gap_ns: int = ALIGNMENT_GAP_NS,
    minimum_coverage: float = LOCALIZATION_MINIMUM_COVERAGE,
    world_to_map: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Evaluate TF localization on the ground-truth timestamp grid."""
    if not 0.0 < minimum_coverage <= 1.0:
        raise ValueError('minimum_coverage must be in (0, 1]')
    alignment = world_to_map or {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
    if any(
        require_finite(alignment.get(field), f'world_to_map.{field}') != 0.0
        for field in ('x_m', 'y_m', 'yaw_rad')
    ):
        raise MetricUnavailable('world_to_map alignment must be identity')
    start = require_int(accepted_goal_stamp_ns, 'accepted_goal_stamp_ns')
    terminal = require_int(terminal_action_stamp_ns, 'terminal_action_stamp_ns')
    if terminal <= start:
        raise MetricUnavailable('terminal action stamp must be after accepted goal stamp')
    eligible: list[tuple[int, Mapping[str, Any], int]] = []
    previous_stamp: int | None = None
    for index, sample in enumerate(ground_truth):
        stamp = require_int(sample.get('stamp_ns'), f'ground_truth[{index}].stamp_ns')
        if previous_stamp is not None and stamp <= previous_stamp:
            raise MetricUnavailable('ground-truth stamps must be unique and increasing')
        previous_stamp = stamp
        if start <= stamp <= terminal:
            eligible.append((stamp, sample, index))
    if not eligible:
        raise MetricUnavailable('no eligible ground-truth localization samples')
    aligned: list[dict[str, Any]] = []
    unavailable: list[dict[str, Any]] = []
    position_errors: list[float] = []
    yaw_errors: list[float] = []
    for stamp, truth_sample, truth_index in eligible:
        truth = _truth_pose(truth_sample, truth_index)
        try:
            first = interpolate_pose(map_to_odom, stamp, max_gap_ns=max_gap_ns)
            second = interpolate_pose(odom_to_base, stamp, max_gap_ns=max_gap_ns)
            estimate = compose_se2(first, second)
        except MetricUnavailable as exc:
            unavailable.append({'reason': str(exc), 'stamp_ns': stamp})
            continue
        position_error = math.hypot(
            estimate['x_m'] - truth['x_m'],
            estimate['y_m'] - truth['y_m'],
        )
        yaw_error = abs(normalize_angle(estimate['yaw_rad'] - truth['yaw_rad']))
        position_errors.append(position_error)
        yaw_errors.append(yaw_error)
        aligned.append(
            {
                'estimate': estimate,
                'map_to_odom_bracket': {
                    'after_stamp_ns': first['source_after_stamp_ns'],
                    'before_stamp_ns': first['source_before_stamp_ns'],
                    'gap_ns': first['source_gap_ns'],
                },
                'odom_to_base_bracket': {
                    'after_stamp_ns': second['source_after_stamp_ns'],
                    'before_stamp_ns': second['source_before_stamp_ns'],
                    'gap_ns': second['source_gap_ns'],
                },
                'position_error_m': position_error,
                'stamp_ns': stamp,
                'truth': truth,
                'yaw_error_rad': yaw_error,
            }
        )
    coverage = len(aligned) / len(eligible)
    if coverage < minimum_coverage:
        raise MetricUnavailable(
            f'localization coverage {coverage:.6f} is below {minimum_coverage:.6f}'
        )
    position = numeric_summary(position_errors, include_rmse=True)
    yaw = numeric_summary(yaw_errors, include_rmse=True)
    return {
        'aligned_samples': aligned,
        'available_count': len(aligned),
        'coverage_ratio': coverage,
        'eligible_count': len(eligible),
        'position_error_m': position,
        'unavailable_count': len(unavailable),
        'unavailable_samples': unavailable,
        'world_to_map': {'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        'yaw_error_rad': yaw,
    }
