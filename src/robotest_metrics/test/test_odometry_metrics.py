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
import math
from typing import Any

import pytest
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.odometry_metrics import analyze_odometry_drift

ACTIVATION_NS = 1_000_000_000
DEACTIVATION_NS = 2_000_000_000
X_RATE_NM_PER_S = 100_000_000
YAW_RATE_NRAD_PER_S = 50_000_000


def _integrity() -> dict[str, Any]:
    return {
        'pose_covariance_sha256': 'a' * 64,
        'twist': {
            'angular': [0.0, 0.0, 0.1],
            'linear': [0.2, 0.0, 0.0],
        },
        'twist_covariance_sha256': 'b' * 64,
        'z_m': 0.2,
    }


def _quaternion(yaw: float) -> list[float]:
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def _pose_sample(
    stamp_ns: int,
    sequence: int,
    *,
    raw: bool,
    restored: bool = False,
) -> dict[str, Any]:
    raw_x = 1.0 + (stamp_ns - ACTIVATION_NS) / 10_000_000_000
    raw_y = 0.5
    raw_yaw = 0.2
    elapsed_s = (stamp_ns - ACTIVATION_NS) / 1_000_000_000
    dx = 0.0 if restored else 0.1 * elapsed_s
    dyaw = 0.0 if restored else 0.05 * elapsed_s
    if raw:
        x_m, y_m, yaw = raw_x, raw_y, raw_yaw
    else:
        x_m = math.cos(dyaw) * raw_x - math.sin(dyaw) * raw_y + dx
        y_m = math.sin(dyaw) * raw_x + math.cos(dyaw) * raw_y
        yaw = raw_yaw + dyaw
    return {
        'child_frame_id': 'base_footprint',
        'collector_sequence': sequence,
        'frame_id': 'odom',
        'nonplanar_integrity': _integrity(),
        'orientation_xyzw': _quaternion(yaw),
        'stamp_ns': stamp_ns,
        'x_m': x_m,
        'y_m': y_m,
        'yaw_rad': yaw,
    }


def _inputs() -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    stamps = [1_000_000_000, 1_500_000_000, 1_900_000_000, 2_000_000_000]
    raw = [
        _pose_sample(stamp, index, raw=True, restored=stamp >= DEACTIVATION_NS)
        for index, stamp in enumerate(stamps, start=1)
    ]
    validated = [
        _pose_sample(stamp, index, raw=False, restored=stamp >= DEACTIVATION_NS)
        for index, stamp in enumerate(stamps, start=1)
    ]
    transforms = [
        {
            key: value
            for key, value in sample.items()
            if key
            in {
                'child_frame_id',
                'collector_sequence',
                'frame_id',
                'orientation_xyzw',
                'stamp_ns',
                'x_m',
                'y_m',
                'yaw_rad',
            }
        }
        for sample in validated
    ]
    return raw, validated, transforms


def _analyze(
    raw: list[dict[str, Any]],
    validated: list[dict[str, Any]],
    transforms: list[dict[str, Any]],
) -> dict[str, Any]:
    return analyze_odometry_drift(
        raw,
        validated,
        transforms,
        ACTIVATION_NS,
        DEACTIVATION_NS,
        x_rate_nm_per_s=X_RATE_NM_PER_S,
        yaw_rate_nrad_per_s=YAW_RATE_NRAD_PER_S,
    )


def test_odometry_drift_proves_left_composition_endpoint_tf_and_restoration() -> None:
    result = _analyze(*_inputs())
    assert result['active_sample_count'] == 3
    assert result['final_active_gap_ns'] == 100_000_000
    assert result['full_duration_target'] == {
        'x_m': pytest.approx(0.1),
        'y_m': 0.0,
        'yaw_rad': pytest.approx(0.05),
    }
    assert result['nonplanar_raw_integrity'] == 'PASS'
    assert result['odom_tf_exact_stamp_identity'] == 'PASS'
    assert result['restored_offset']['x_m'] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize(
    'failure',
    [
        'stamp_mismatch',
        'final_stale',
        'left_composition',
        'nonplanar_mutation',
        'tf_mismatch',
        'unnormalized_quaternion',
        'latched_restoration',
        'collector_order',
    ],
)
def test_odometry_drift_fails_closed_for_every_frozen_integrity_gate(failure: str) -> None:
    raw, validated, transforms = _inputs()
    if failure == 'stamp_mismatch':
        validated.pop(1)
    elif failure == 'final_stale':
        raw.pop(2)
        validated.pop(2)
        transforms.pop(2)
    elif failure == 'left_composition':
        validated[1]['x_m'] += 0.03
        transforms[1]['x_m'] += 0.03
    elif failure == 'nonplanar_mutation':
        validated[1]['nonplanar_integrity']['z_m'] = 0.3
    elif failure == 'tf_mismatch':
        transforms[1]['x_m'] += 0.01
    elif failure == 'unnormalized_quaternion':
        validated[1]['orientation_xyzw'] = [0.0, 0.0, 0.0, 2.0]
    elif failure == 'latched_restoration':
        validated[-1]['x_m'] += 0.01
        transforms[-1]['x_m'] += 0.01
    else:
        raw[1]['collector_sequence'] = raw[0]['collector_sequence']
    with pytest.raises(MetricUnavailable):
        _analyze(raw, validated, transforms)


def test_odometry_drift_rejects_boolean_rate_and_does_not_mutate_inputs() -> None:
    raw, validated, transforms = _inputs()
    before = copy.deepcopy((raw, validated, transforms))
    with pytest.raises(MetricUnavailable):
        analyze_odometry_drift(
            raw,
            validated,
            transforms,
            ACTIVATION_NS,
            DEACTIVATION_NS,
            x_rate_nm_per_s=True,
            yaw_rate_nrad_per_s=YAW_RATE_NRAD_PER_S,
        )
    assert (raw, validated, transforms) == before
