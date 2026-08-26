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

"""Exact Scenario 5 odometry-drift and odom/TF identity metrics."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import normalize_angle, require_finite, require_int

_FINAL_ACTIVE_GAP_NS = 250_000_000
_POSITION_TOLERANCE_M = 0.020
_YAW_TOLERANCE_RAD = 0.010
_IDENTITY_TOLERANCE = 1e-9


def _ordered_index(
    samples: Sequence[Mapping[str, Any]],
    name: str,
) -> dict[int, Mapping[str, Any]]:
    indexed: dict[int, Mapping[str, Any]] = {}
    previous_sequence: int | None = None
    previous_stamp: int | None = None
    for index, sample in enumerate(samples):
        stamp = require_int(sample.get('stamp_ns'), f'{name}[{index}].stamp_ns')
        sequence = require_int(
            sample.get('collector_sequence'),
            f'{name}[{index}].collector_sequence',
        )
        if (previous_sequence is not None and sequence <= previous_sequence) or (
            previous_stamp is not None and stamp <= previous_stamp
        ):
            raise MetricUnavailable(f'{name} is not in deterministic collector order')
        if stamp in indexed:
            raise MetricUnavailable(f'{name} contains duplicate stamps')
        indexed[stamp] = sample
        previous_sequence = sequence
        previous_stamp = stamp
    return indexed


def _pose(sample: Mapping[str, Any], name: str) -> tuple[float, float, float]:
    return (
        require_finite(sample.get('x_m'), f'{name}.x_m'),
        require_finite(sample.get('y_m'), f'{name}.y_m'),
        require_finite(sample.get('yaw_rad'), f'{name}.yaw_rad'),
    )


def _measured_offset(
    raw: Mapping[str, Any],
    validated: Mapping[str, Any],
    name: str,
) -> dict[str, float]:
    raw_x, raw_y, raw_yaw = _pose(raw, f'{name}.raw')
    validated_x, validated_y, validated_yaw = _pose(validated, f'{name}.validated')
    yaw = normalize_angle(validated_yaw - raw_yaw)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return {
        'x_m': validated_x - cosine * raw_x + sine * raw_y,
        'y_m': validated_y - sine * raw_x - cosine * raw_y,
        'yaw_rad': yaw,
    }


def _require_nonplanar_identity(
    raw: Mapping[str, Any],
    validated: Mapping[str, Any],
    name: str,
) -> None:
    if raw.get('frame_id') != 'odom' or validated.get('frame_id') != 'odom':
        raise MetricUnavailable(f'{name} odometry frame must be odom')
    if raw.get('child_frame_id') != 'base_footprint' or validated.get('child_frame_id') != (
        'base_footprint'
    ):
        raise MetricUnavailable(f'{name} odometry child frame must be base_footprint')
    raw_integrity = raw.get('nonplanar_integrity')
    validated_integrity = validated.get('nonplanar_integrity')
    if not isinstance(raw_integrity, Mapping) or not isinstance(validated_integrity, Mapping):
        raise MetricUnavailable(f'{name} nonplanar odometry integrity evidence is missing')
    if dict(raw_integrity) != dict(validated_integrity):
        raise MetricUnavailable(f'{name} validated odometry mutated twist/z/covariance')
    quaternion = validated.get('orientation_xyzw')
    if (
        not isinstance(quaternion, list)
        or len(quaternion) != 4
        or any(not isinstance(value, (int, float)) for value in quaternion)
    ):
        raise MetricUnavailable(f'{name} validated quaternion evidence is invalid')
    norm = math.sqrt(sum(require_finite(value, f'{name}.quaternion') ** 2 for value in quaternion))
    if abs(norm - 1.0) > _IDENTITY_TOLERANCE:
        raise MetricUnavailable(f'{name} validated quaternion is not normalized')


def _require_tf_identity(
    validated: Mapping[str, Any],
    transform: Mapping[str, Any],
    name: str,
) -> None:
    if transform.get('frame_id') != 'odom' or transform.get('child_frame_id') != 'base_footprint':
        raise MetricUnavailable(f'{name} TF frame identity is invalid')
    odom_pose = _pose(validated, f'{name}.validated')
    tf_pose = _pose(transform, f'{name}.tf')
    if (
        math.hypot(tf_pose[0] - odom_pose[0], tf_pose[1] - odom_pose[1]) > _IDENTITY_TOLERANCE
        or abs(normalize_angle(tf_pose[2] - odom_pose[2])) > _IDENTITY_TOLERANCE
    ):
        raise MetricUnavailable(f'{name} validated odometry and TF differ')


def analyze_odometry_drift(
    raw_odometry: Sequence[Mapping[str, Any]],
    validated_odometry: Sequence[Mapping[str, Any]],
    odom_to_base_tf: Sequence[Mapping[str, Any]],
    configured_activation_stamp_ns: int,
    configured_deactivation_stamp_ns: int,
    *,
    x_rate_nm_per_s: int,
    yaw_rate_nrad_per_s: int,
    position_tolerance_m: float = _POSITION_TOLERANCE_M,
    yaw_tolerance_rad: float = _YAW_TOLERANCE_RAD,
    final_active_gap_ns: int = _FINAL_ACTIVE_GAP_NS,
) -> dict[str, Any]:
    """Prove left composition, endpoint, raw integrity, restoration, and TF identity."""
    activation = require_int(configured_activation_stamp_ns, 'configured activation')
    deactivation = require_int(configured_deactivation_stamp_ns, 'configured deactivation')
    x_rate = require_int(x_rate_nm_per_s, 'x_rate_nm_per_s') * 1e-9
    yaw_rate = require_int(yaw_rate_nrad_per_s, 'yaw_rate_nrad_per_s') * 1e-9
    if deactivation <= activation:
        raise MetricUnavailable('odometry-drift interval is invalid')
    if position_tolerance_m <= 0.0 or yaw_tolerance_rad <= 0.0 or final_active_gap_ns <= 0:
        raise ValueError('odometry-drift tolerances must be positive')
    raw = _ordered_index(raw_odometry, 'raw_odometry')
    validated = _ordered_index(validated_odometry, 'validated_odometry')
    transforms = _ordered_index(odom_to_base_tf, 'odom_to_base_tf')
    active_stamps = sorted(stamp for stamp in raw if activation <= stamp < deactivation)
    validated_active = sorted(stamp for stamp in validated if activation <= stamp < deactivation)
    if not active_stamps or active_stamps != validated_active:
        raise MetricUnavailable('raw/validated active odometry stamps do not exactly match')
    if any(stamp not in transforms for stamp in active_stamps):
        raise MetricUnavailable('active validated odometry lacks exact-stamp TF identity evidence')
    samples: list[dict[str, Any]] = []
    for stamp in active_stamps:
        raw_sample = raw[stamp]
        validated_sample = validated[stamp]
        _require_nonplanar_identity(raw_sample, validated_sample, f'active[{stamp}]')
        _require_tf_identity(validated_sample, transforms[stamp], f'active[{stamp}]')
        elapsed_s = (stamp - activation) / 1_000_000_000
        measured = _measured_offset(raw_sample, validated_sample, f'active[{stamp}]')
        expected = {'x_m': x_rate * elapsed_s, 'y_m': 0.0, 'yaw_rad': yaw_rate * elapsed_s}
        samples.append(
            {
                'expected_offset': expected,
                'measured_offset': measured,
                'sample_error': {
                    'position_m': math.hypot(
                        measured['x_m'] - expected['x_m'],
                        measured['y_m'],
                    ),
                    'yaw_rad': abs(normalize_angle(measured['yaw_rad'] - expected['yaw_rad'])),
                },
                'stamp_ns': stamp,
            }
        )
    if any(
        sample['sample_error']['position_m'] > position_tolerance_m
        or sample['sample_error']['yaw_rad'] > yaw_tolerance_rad
        for sample in samples
    ):
        raise MetricUnavailable('odometry-drift left-composition sample exceeds tolerance')
    final = samples[-1]
    final_gap = deactivation - final['stamp_ns']
    if not 0 < final_gap <= final_active_gap_ns:
        raise MetricUnavailable('final active odometry sample is not within 0.25 s of interval end')
    duration_s = (deactivation - activation) / 1_000_000_000
    full_target = {'x_m': x_rate * duration_s, 'y_m': 0.0, 'yaw_rad': yaw_rate * duration_s}
    measured_final = final['measured_offset']
    full_position_error = math.hypot(
        measured_final['x_m'] - full_target['x_m'],
        measured_final['y_m'],
    )
    full_yaw_error = abs(normalize_angle(measured_final['yaw_rad'] - full_target['yaw_rad']))
    if full_position_error > position_tolerance_m or full_yaw_error > yaw_tolerance_rad:
        raise MetricUnavailable('odometry-drift endpoint exceeds frozen tolerances')
    restored_stamp = next((stamp for stamp in sorted(raw) if stamp >= deactivation), None)
    if (
        restored_stamp is None
        or restored_stamp not in validated
        or restored_stamp not in transforms
    ):
        raise MetricUnavailable('post-drift restored odometry/TF sample is missing')
    restored_raw = raw[restored_stamp]
    restored_validated = validated[restored_stamp]
    _require_nonplanar_identity(restored_raw, restored_validated, 'restored')
    _require_tf_identity(restored_validated, transforms[restored_stamp], 'restored')
    restored_offset = _measured_offset(restored_raw, restored_validated, 'restored')
    if (
        math.hypot(restored_offset['x_m'], restored_offset['y_m']) > _IDENTITY_TOLERANCE
        or abs(restored_offset['yaw_rad']) > _IDENTITY_TOLERANCE
    ):
        raise MetricUnavailable('odometry drift remained latched after the half-open interval')
    return {
        'active_sample_count': len(samples),
        'configured_activation_stamp_ns': activation,
        'configured_deactivation_stamp_ns': deactivation,
        'final_active_gap_ns': final_gap,
        'final_active_stamp_ns': final['stamp_ns'],
        'full_duration_target': full_target,
        'full_target_position_error_m': full_position_error,
        'full_target_yaw_error_rad': full_yaw_error,
        'left_composition_samples': samples,
        'maximum_sample_position_error_m': max(
            sample['sample_error']['position_m'] for sample in samples
        ),
        'maximum_sample_yaw_error_rad': max(
            sample['sample_error']['yaw_rad'] for sample in samples
        ),
        'nonplanar_raw_integrity': 'PASS',
        'odom_tf_exact_stamp_identity': 'PASS',
        'position_tolerance_m': position_tolerance_m,
        'restored_offset': restored_offset,
        'restored_stamp_ns': restored_stamp,
        'yaw_tolerance_rad': yaw_tolerance_rad,
    }
