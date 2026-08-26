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

"""Small deterministic SE(2) and numeric helpers."""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from robotest_metrics.errors import MetricUnavailable


def require_finite(value: Any, name: str) -> float:
    """Convert a finite real value or reject it as unavailable evidence."""
    if isinstance(value, bool):
        raise MetricUnavailable(f'{name} must be a finite number')
    try:
        converted = float(value)
    except (TypeError, ValueError) as exc:
        raise MetricUnavailable(f'{name} must be a finite number') from exc
    if not math.isfinite(converted):
        raise MetricUnavailable(f'{name} must be finite')
    return converted


def require_int(value: Any, name: str) -> int:
    """Return an integer while rejecting booleans and lossy conversions."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise MetricUnavailable(f'{name} must be an integer')
    return value


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi)."""
    value = require_finite(angle_rad, 'angle_rad')
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def interpolate_angle(start: float, end: float, fraction: float) -> float:
    """Interpolate along the shortest angular arc."""
    delta = normalize_angle(end - start)
    return normalize_angle(start + fraction * delta)


def compose_se2(first: Mapping[str, Any], second: Mapping[str, Any]) -> dict[str, float]:
    """Compose two planar transforms ``first * second``."""
    x1 = require_finite(first.get('x_m'), 'first.x_m')
    y1 = require_finite(first.get('y_m'), 'first.y_m')
    yaw1 = require_finite(first.get('yaw_rad'), 'first.yaw_rad')
    x2 = require_finite(second.get('x_m'), 'second.x_m')
    y2 = require_finite(second.get('y_m'), 'second.y_m')
    yaw2 = require_finite(second.get('yaw_rad'), 'second.yaw_rad')
    cosine = math.cos(yaw1)
    sine = math.sin(yaw1)
    return {
        'x_m': x1 + cosine * x2 - sine * y2,
        'y_m': y1 + sine * x2 + cosine * y2,
        'yaw_rad': normalize_angle(yaw1 + yaw2),
    }


def planar_distance(first: Mapping[str, Any], second: Mapping[str, Any]) -> float:
    """Return Euclidean x/y distance between two mappings."""
    x1 = require_finite(first.get('x_m'), 'first.x_m')
    y1 = require_finite(first.get('y_m'), 'first.y_m')
    x2 = require_finite(second.get('x_m'), 'second.x_m')
    y2 = require_finite(second.get('y_m'), 'second.y_m')
    return math.hypot(x2 - x1, y2 - y1)


def ensure_strictly_increasing(samples: Sequence[Mapping[str, Any]]) -> None:
    """Require unique, strictly increasing ``stamp_ns`` fields."""
    previous: int | None = None
    for index, sample in enumerate(samples):
        stamp = require_int(sample.get('stamp_ns'), f'samples[{index}].stamp_ns')
        if previous is not None and stamp <= previous:
            raise MetricUnavailable('sample stamps must be unique and strictly increasing')
        previous = stamp
