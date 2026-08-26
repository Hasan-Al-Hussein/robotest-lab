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

import math

import pytest
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.localization_metrics import localization_error_metrics
from robotest_metrics.statistics import (
    deterministic_median,
    nearest_rank,
    numeric_summary,
    real_time_factor_metrics,
)


def _pose(stamp: int, x_m: float, y_m: float, yaw_rad: float) -> dict[str, float | int]:
    return {'stamp_ns': stamp, 'x_m': x_m, 'y_m': y_m, 'yaw_rad': yaw_rad}


def test_localization_composes_both_tf_edges_at_ground_truth_stamp() -> None:
    truth = [_pose(100, 1.0, 1.0, math.pi - 0.1)]
    map_odom = [_pose(0, 0.0, 0.0, math.pi / 2), _pose(200, 0.0, 0.0, math.pi / 2)]
    odom_base = [_pose(0, 1.0, -1.0, math.pi / 2 - 0.1), _pose(200, 1.0, -1.0, math.pi / 2 - 0.1)]
    result = localization_error_metrics(truth, map_odom, odom_base, 50, 150)
    assert result['coverage_ratio'] == 1.0
    assert result['aligned_samples'][0]['map_to_odom_bracket']['gap_ns'] == 200
    assert result['position_error_m']['rmse'] == pytest.approx(0.0, abs=1e-12)
    assert result['yaw_error_rad']['maximum'] == pytest.approx(0.0, abs=1e-12)


def test_localization_shortest_arc_yaw_error_wraps() -> None:
    truth = [_pose(100, 0.0, 0.0, -math.pi + 0.01)]
    map_odom = [_pose(100, 0.0, 0.0, 0.0)]
    odom_base = [_pose(100, 0.0, 0.0, math.pi - 0.01)]
    result = localization_error_metrics(truth, map_odom, odom_base, 100, 101)
    assert result['yaw_error_rad']['maximum'] == pytest.approx(0.02)


def test_localization_requires_identity_alignment_and_95_percent_coverage() -> None:
    truth = [_pose(100, 0.0, 0.0, 0.0), _pose(400, 0.0, 0.0, 0.0)]
    edge = [_pose(0, 0.0, 0.0, 0.0), _pose(200, 0.0, 0.0, 0.0)]
    with pytest.raises(MetricUnavailable, match='coverage'):
        localization_error_metrics(truth, edge, edge, 100, 400)
    with pytest.raises(MetricUnavailable, match='identity'):
        localization_error_metrics(
            truth[:1],
            edge,
            edge,
            100,
            101,
            world_to_map={'x_m': 0.1, 'y_m': 0.0, 'yaw_rad': 0.0},
        )


def test_nearest_rank_and_median_have_no_library_interpolation() -> None:
    assert nearest_rank([1.0, 2.0, 3.0], 0.95) == 3.0
    assert nearest_rank([1.0, 2.0, 3.0], 0.05) == 1.0
    assert deterministic_median([1.0, 2.0, 9.0, 10.0]) == 5.5
    assert numeric_summary([3.0, 4.0], include_rmse=True)['rmse'] == pytest.approx(math.sqrt(12.5))
    with pytest.raises(MetricUnavailable, match='empty'):
        nearest_rank([], 0.95)


def test_numeric_summary_remains_finite_for_finite_extremes() -> None:
    maximum = float.fromhex('0x1.fffffffffffffp+1023')
    summary = numeric_summary([maximum, maximum], include_rmse=True)
    assert deterministic_median([maximum, maximum]) == maximum
    assert summary['mean'] == maximum
    assert summary['rmse'] == maximum


def test_rtf_uses_adjacent_unpaused_sim_and_steady_wall_deltas() -> None:
    samples = [
        {'collector_sequence': 1, 'paused': False, 'sim_stamp_ns': 0, 'steady_wall_ns': 0},
        {
            'collector_sequence': 2,
            'paused': False,
            'sim_stamp_ns': 1_000_000_000,
            'steady_wall_ns': 2_000_000_000,
        },
        {
            'collector_sequence': 3,
            'paused': True,
            'sim_stamp_ns': 2_000_000_000,
            'steady_wall_ns': 3_000_000_000,
        },
        {
            'collector_sequence': 4,
            'paused': False,
            'sim_stamp_ns': 3_000_000_000,
            'steady_wall_ns': 4_000_000_000,
        },
        {
            'collector_sequence': 5,
            'paused': False,
            'sim_stamp_ns': 4_000_000_000,
            'steady_wall_ns': 4_000_000_000,
        },
    ]
    result = real_time_factor_metrics(samples)
    assert result['calculated_series'] == [0.5]
    assert [item['reason'] for item in result['excluded_intervals']] == [
        'declared_pause',
        'declared_pause',
        'non_positive_delta',
    ]
    assert result['p5'] == 0.5


def test_rtf_rejects_non_deterministic_order_and_empty_intervals() -> None:
    with pytest.raises(MetricUnavailable, match='collector order'):
        real_time_factor_metrics(
            [
                {'collector_sequence': 2, 'paused': False, 'sim_stamp_ns': 1, 'steady_wall_ns': 1},
                {'collector_sequence': 1, 'paused': False, 'sim_stamp_ns': 0, 'steady_wall_ns': 2},
            ]
        )
    with pytest.raises(MetricUnavailable, match='no valid'):
        real_time_factor_metrics(
            [
                {'collector_sequence': 1, 'paused': True, 'sim_stamp_ns': 0, 'steady_wall_ns': 0},
                {'collector_sequence': 2, 'paused': True, 'sim_stamp_ns': 1, 'steady_wall_ns': 1},
            ]
        )


def test_rtf_records_simulation_regression_as_non_positive_exclusion() -> None:
    result = real_time_factor_metrics(
        [
            {'collector_sequence': 1, 'paused': False, 'sim_stamp_ns': 10, 'steady_wall_ns': 1},
            {'collector_sequence': 2, 'paused': False, 'sim_stamp_ns': 9, 'steady_wall_ns': 2},
            {'collector_sequence': 3, 'paused': False, 'sim_stamp_ns': 11, 'steady_wall_ns': 4},
        ]
    )
    assert result['calculated_series'] == [1.0]
    assert result['excluded_intervals'] == [{'interval_index': 0, 'reason': 'non_positive_delta'}]
