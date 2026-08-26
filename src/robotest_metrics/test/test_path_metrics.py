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
from robotest_metrics.path_metrics import (
    actual_path_metrics,
    analyze_plans,
    canonical_plan_points,
    path_efficiency,
    plan_geometry_hash,
    plan_length_m,
    quantize_micrometres,
)


def test_actual_path_inserts_exact_linear_boundaries_without_smoothing() -> None:
    samples = [
        {'stamp_ns': 0, 'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0},
        {'stamp_ns': 250_000_000, 'x_m': 0.25, 'y_m': 0.0, 'yaw_rad': 0.0},
        {'stamp_ns': 500_000_000, 'x_m': 0.25, 'y_m': 0.25, 'yaw_rad': math.pi / 2},
    ]
    result = actual_path_metrics(samples, 125_000_000, 375_000_000)
    assert result['boundary_start']['interpolated'] is True
    assert result['boundary_end']['interpolated'] is True
    assert result['boundary_start']['x_m'] == pytest.approx(0.125)
    assert result['boundary_end']['y_m'] == pytest.approx(0.125)
    assert result['actual_path_length_m'] == pytest.approx(0.25)
    assert [sample['stamp_ns'] for sample in result['samples']] == [
        125_000_000,
        250_000_000,
        375_000_000,
    ]


@pytest.mark.parametrize(
    'samples,reason',
    [
        (
            [
                {'stamp_ns': 0, 'x_m': 0.0, 'y_m': 0.0},
                {'stamp_ns': 0, 'x_m': 1.0, 'y_m': 0.0},
            ],
            'unique',
        ),
        (
            [
                {'stamp_ns': 0, 'x_m': 0.0, 'y_m': 0.0},
                {'stamp_ns': 300_000_000, 'x_m': 1.0, 'y_m': 0.0},
            ],
            'gap',
        ),
    ],
)
def test_actual_path_rejects_duplicate_stamps_and_large_gaps(
    samples: list[dict[str, float | int]], reason: str
) -> None:
    with pytest.raises(MetricUnavailable, match=reason):
        actual_path_metrics(samples, 0, 100_000_000)


@pytest.mark.parametrize(
    'metres,expected',
    [
        (0.0, 0),
        (0.00000049, 0),
        (0.00000050, 1),
        (-0.00000049, 0),
        (-0.00000050, -1),
        (1.2345675, 1_234_568),
        (-1.2345675, -1_234_568),
    ],
)
def test_plan_hash_quantization_is_half_away_from_zero(metres: float, expected: int) -> None:
    assert quantize_micrometres(metres) == expected


def test_plan_hash_quantization_rejects_finite_coordinate_outside_int64() -> None:
    with pytest.raises(MetricUnavailable, match='signed int64'):
        quantize_micrometres(float.fromhex('0x1.fffffffffffffp+1023'))


def test_hash_collapses_quantized_duplicates_but_length_uses_raw_coordinates() -> None:
    poses = [
        {'x_m': 0.0, 'y_m': 0.0},
        {'x_m': 0.0000004, 'y_m': 0.0},
        {'x_m': 1.0, 'y_m': 0.0},
    ]
    assert canonical_plan_points(poses) == [(0, 0), (1_000_000, 0)]
    assert plan_length_m(poses) == pytest.approx(1.0)


def test_geometry_hash_excludes_stamps_z_and_orientation() -> None:
    first = [
        {'x_m': 0.0, 'y_m': 0.0, 'stamp_ns': 1, 'z_m': 100.0},
        {'x_m': 1.0, 'y_m': 0.0, 'stamp_ns': 2, 'yaw_rad': 2.0},
    ]
    second = [
        {'x_m': 0.0, 'y_m': 0.0, 'stamp_ns': 99, 'z_m': -1.0},
        {'x_m': 1.0, 'y_m': 0.0, 'stamp_ns': 100, 'yaw_rad': -2.0},
    ]
    assert plan_geometry_hash('map', first) == plan_geometry_hash('map', second)
    with pytest.raises(MetricUnavailable, match='frame_id'):
        plan_geometry_hash('odom', first)


def _plan(stamp: int, sequence: int, points: list[float]) -> dict[str, object]:
    return {
        'collector_sequence': sequence,
        'frame_id': 'map',
        'poses': [{'x_m': x_m, 'y_m': 0.0} for x_m in points],
        'stamp_ns': stamp,
    }


def test_per_leg_first_plan_and_replan_chains_are_independent() -> None:
    feedback = [
        {'collector_sequence': 1, 'current_waypoint': 0, 'stamp_ns': 0},
        {'collector_sequence': 10, 'current_waypoint': 1, 'stamp_ns': 10},
    ]
    plans = [
        _plan(1, 2, [0.0, 2.0]),
        _plan(2, 3, [0.0, 2.0]),
        _plan(3, 4, [0.0, 1.0, 2.0]),
        _plan(10, 11, [2.0, 4.0]),
        _plan(12, 12, [2.0, 3.0, 4.0]),
    ]
    result = analyze_plans(
        plans,
        feedback,
        [{'x_m': 2.0, 'y_m': 0.0}, {'x_m': 4.0, 'y_m': 0.0}],
        0,
        20,
    )
    assert result['initial_planned_path_length_m'] == pytest.approx(4.0)
    assert result['latest_planned_path_length_m'] == pytest.approx(4.0)
    assert result['replan_count'] == 2
    assert [leg['replan_count'] for leg in result['leg_results']] == [1, 1]
    assert result['leg_results'][1]['initial_plan']['collector_sequence'] == 11


def test_equal_stamp_plan_before_feedback_remains_in_previous_leg_diagnostics() -> None:
    feedback = [
        {'collector_sequence': 1, 'current_waypoint': 0, 'stamp_ns': 0},
        {'collector_sequence': 10, 'current_waypoint': 1, 'stamp_ns': 10},
    ]
    plans = [_plan(1, 2, [0.0, 2.0]), _plan(10, 9, [2.0, 4.0]), _plan(11, 11, [2.0, 4.0])]
    result = analyze_plans(
        plans,
        feedback,
        [{'x_m': 2.0, 'y_m': 0.0}, {'x_m': 4.0, 'y_m': 0.0}],
        0,
        20,
    )
    assert result['leg_results'][1]['plan_count'] == 1
    assert any(item['reason'].startswith('plan endpoint') for item in result['diagnostics'])


@pytest.mark.parametrize(
    'feedback',
    [
        [{'collector_sequence': 1, 'current_waypoint': 1, 'stamp_ns': 0}],
        [
            {'collector_sequence': 1, 'current_waypoint': 0, 'stamp_ns': 0},
            {'collector_sequence': 2, 'current_waypoint': 2, 'stamp_ns': 1},
        ],
    ],
)
def test_ambiguous_feedback_and_missing_leg_fail_closed(
    feedback: list[dict[str, int]],
) -> None:
    with pytest.raises(MetricUnavailable):
        analyze_plans(
            [_plan(1, 2, [0.0, 1.0])],
            feedback,
            [{'x_m': 1.0, 'y_m': 0.0}, {'x_m': 2.0, 'y_m': 0.0}],
            0,
            20,
        )


def test_path_efficiency_is_unclamped_and_rejects_zero() -> None:
    assert path_efficiency(2.0, 1.0) == {'path_efficiency': 2.0, 'path_overrun_ratio': 0.5}
    with pytest.raises(MetricUnavailable):
        path_efficiency(0.0, 1.0)
