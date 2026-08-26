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

"""Deterministic statistics shared by run and aggregate analysis."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping, Sequence
from itertools import pairwise
from typing import Any

from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import require_finite, require_int


def finite_values(values: Iterable[Any], name: str = 'values') -> list[float]:
    """Normalize a non-empty iterable of finite values."""
    normalized = [require_finite(value, name) for value in values]
    if not normalized:
        raise MetricUnavailable(f'{name} is empty')
    return normalized


def deterministic_median(values: Iterable[Any]) -> float:
    """Return the contract median without relying on library defaults."""
    ordered = sorted(finite_values(values))
    count = len(ordered)
    middle = count // 2
    if count % 2:
        return ordered[middle]
    # Divide first so the median of two finite extremes cannot overflow.
    return ordered[middle - 1] / 2.0 + ordered[middle] / 2.0


def nearest_rank(values: Iterable[Any], fraction: float) -> float:
    """Return the one-based nearest-rank percentile."""
    if not 0.0 < fraction <= 1.0:
        raise ValueError('fraction must be in (0, 1]')
    ordered = sorted(finite_values(values))
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[index]


def numeric_summary(values: Iterable[Any], *, include_rmse: bool = False) -> dict[str, float | int]:
    """Summarize finite values with explicitly defined percentiles."""
    normalized = finite_values(values)
    count = len(normalized)
    mean = math.fsum(value / count for value in normalized)
    result: dict[str, float | int] = {
        'count': count,
        'maximum': max(normalized),
        'mean': mean,
        'median': deterministic_median(normalized),
        'minimum': min(normalized),
        'p5': nearest_rank(normalized, 0.05),
        'p95': nearest_rank(normalized, 0.95),
    }
    if include_rmse:
        scale = max(abs(value) for value in normalized)
        result['rmse'] = (
            0.0
            if scale == 0.0
            else scale * math.sqrt(math.fsum((value / scale) ** 2 for value in normalized) / count)
        )
    return result


def real_time_factor_metrics(samples: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Calculate RTF from adjacent unpaused simulation and steady-wall samples."""
    if len(samples) < 2:
        raise MetricUnavailable('at least two world-statistics samples are required')
    values: list[float] = []
    exclusions: list[dict[str, Any]] = []
    previous_sequence: int | None = None
    for index, sample in enumerate(samples):
        require_int(sample.get('sim_stamp_ns'), f'samples[{index}].sim_stamp_ns')
        sequence = require_int(
            sample.get('collector_sequence'),
            f'samples[{index}].collector_sequence',
        )
        if previous_sequence is not None and sequence <= previous_sequence:
            raise MetricUnavailable('world-statistics samples are not in collector order')
        previous_sequence = sequence
    for index, (previous, current) in enumerate(pairwise(samples)):
        delta_sim = require_int(current.get('sim_stamp_ns'), 'current.sim_stamp_ns') - require_int(
            previous.get('sim_stamp_ns'), 'previous.sim_stamp_ns'
        )
        delta_wall = require_int(
            current.get('steady_wall_ns'), 'current.steady_wall_ns'
        ) - require_int(previous.get('steady_wall_ns'), 'previous.steady_wall_ns')
        if previous.get('paused') is True or current.get('paused') is True:
            exclusions.append({'interval_index': index, 'reason': 'declared_pause'})
            continue
        if delta_sim <= 0 or delta_wall <= 0:
            exclusions.append({'interval_index': index, 'reason': 'non_positive_delta'})
            continue
        values.append(delta_sim / delta_wall)
    if not values:
        raise MetricUnavailable('no valid real-time-factor intervals')
    summary = numeric_summary(values)
    return {
        'calculated_series': values,
        'excluded_intervals': exclusions,
        'interval_count': len(values),
        'mean': summary['mean'],
        'median': summary['median'],
        'minimum': summary['minimum'],
        'p5': summary['p5'],
    }
