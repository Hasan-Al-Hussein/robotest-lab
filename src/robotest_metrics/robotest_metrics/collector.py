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

"""ROS-independent core for the bounded live evidence collector."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from robotest_metrics.buffers import PlanPrefixBuffer, PrefixBuffer
from robotest_metrics.constants import (
    BUFFER_CAPACITIES,
    CONTACT_RECORD_CAPACITY,
    PLAN_MESSAGE_CAPACITY,
    PLAN_POSE_CAPACITY,
    STRING_FIELD_MAX_BYTES,
)
from robotest_metrics.errors import EvidenceOverflow, MetricUnavailable
from robotest_metrics.geometry import require_int

_MISSING = object()


def _validate_bounded_values(value: Any, path: str = 'item') -> None:
    if isinstance(value, str):
        if len(value.encode('utf-8')) > STRING_FIELD_MAX_BYTES:
            raise MetricUnavailable(f'{path} exceeds the UTF-8 string bound')
        return
    if isinstance(value, Mapping):
        if len(value) > 65_536:
            raise MetricUnavailable(f'{path} exceeds the object-item bound')
        for key, nested in value.items():
            if not isinstance(key, str):
                raise MetricUnavailable(f'{path} has a non-string key')
            _validate_bounded_values(key, f'{path}.key')
            _validate_bounded_values(nested, f'{path}.{key}')
        return
    if isinstance(value, (list, tuple)):
        if len(value) > 65_536:
            raise MetricUnavailable(f'{path} exceeds the array-item bound')
        for index, nested in enumerate(value):
            _validate_bounded_values(nested, f'{path}[{index}]')
        return
    if value is None or isinstance(value, bool):
        return
    if isinstance(value, int):
        if not -(1 << 63) <= value <= (1 << 64) - 1:
            raise MetricUnavailable(f'{path} integer exceeds the 64-bit evidence bound')
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise MetricUnavailable(f'{path} must be finite')
        return
    raise MetricUnavailable(f'{path} has unsupported type {type(value).__name__}')


@dataclass
class ClockSummary:
    """Constant-space `/clock` evidence."""

    count: int = 0
    duplicate_count: int = 0
    first_stamp_ns: int | None = None
    latest_stamp_ns: int | None = None
    maximum_positive_gap_ns: int = 0
    regression_count: int = 0

    def observe(self, stamp_ns: int) -> None:
        """Observe one clock stamp without retaining a raw sample."""
        stamp = require_int(stamp_ns, 'clock.stamp_ns')
        self.count += 1
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp
            self.latest_stamp_ns = stamp
            return
        assert self.latest_stamp_ns is not None
        delta = stamp - self.latest_stamp_ns
        if delta < 0:
            self.regression_count += 1
        elif delta == 0:
            self.duplicate_count += 1
        else:
            self.maximum_positive_gap_ns = max(self.maximum_positive_gap_ns, delta)
        self.latest_stamp_ns = stamp

    def as_dict(self) -> dict[str, int | None]:
        """Return the JSON-safe clock summary."""
        return {
            'count': self.count,
            'duplicate_count': self.duplicate_count,
            'first_stamp_ns': self.first_stamp_ns,
            'latest_stamp_ns': self.latest_stamp_ns,
            'maximum_positive_gap_ns': self.maximum_positive_gap_ns,
            'regression_count': self.regression_count,
        }


class CollectorCore:
    """Own every bounded prefix and one global callback-order sequence."""

    def __init__(self) -> None:
        """Create all frozen-capacity evidence streams."""
        self.buffers = {
            name: PrefixBuffer[dict[str, Any]](name=name, capacity=capacity)
            for name, capacity in BUFFER_CAPACITIES.items()
        }
        self.plans = PlanPrefixBuffer(PLAN_MESSAGE_CAPACITY, PLAN_POSE_CAPACITY)
        self.clock = ClockSummary()
        self._sequence = 0
        self._last_state: dict[tuple[str, str], Any] = {}
        self._contact_record_count = 0
        self._contact_record_overflow_count = 0

    @property
    def next_sequence(self) -> int:
        """Reserve and return the next global collector sequence."""
        self._sequence += 1
        return self._sequence

    def observe_clock(self, stamp_ns: int) -> None:
        """Update the constant-space clock summary."""
        _ = self.next_sequence
        self.clock.observe(stamp_ns)

    def record(self, stream: str, item: Mapping[str, Any]) -> bool:
        """Validate and append one normalized evidence item."""
        if stream not in self.buffers:
            raise KeyError(f'unknown evidence stream {stream}')
        sequence = self.next_sequence
        buffer = self.buffers[stream]
        contacts: Any = None
        if stream == 'contacts':
            contacts = item.get('contacts')
            if not isinstance(contacts, list):
                buffer.reject('contacts must be a list')
                return False
            if self._contact_record_count + len(contacts) > CONTACT_RECORD_CAPACITY:
                self._contact_record_overflow_count += 1
                buffer.mark_overflow(sequence=sequence, stamp_ns=item.get('stamp_ns'))
                return False
        try:
            _validate_bounded_values(item)
            stamp = item.get('stamp_ns')
            if stamp is not None:
                require_int(stamp, f'{stream}.stamp_ns')
        except MetricUnavailable as exc:
            buffer.reject(str(exc))
            return False
        normalized = dict(item)
        normalized['collector_sequence'] = sequence
        if stream == 'contacts':
            retained = buffer.append(
                normalized,
                sequence=sequence,
                stamp_ns=normalized.get('stamp_ns'),
            )
            if retained:
                self._contact_record_count += len(contacts)
            return retained
        return buffer.append(
            normalized,
            sequence=sequence,
            stamp_ns=normalized.get('stamp_ns'),
        )

    def record_plan(self, item: Mapping[str, Any]) -> bool:
        """Record one normalized plan under message and total-pose limits."""
        sequence = self.next_sequence
        poses = item.get('poses')
        if not isinstance(poses, list):
            self.plans.reject('plan poses must be a list')
            return False
        if (
            len(self.plans.items) >= self.plans.message_capacity
            or self.plans.retained_pose_count + len(poses) > self.plans.pose_capacity
        ):
            normalized = dict(item)
            normalized['collector_sequence'] = sequence
            return self.plans.append(normalized)
        try:
            _validate_bounded_values(item)
            require_int(item.get('stamp_ns'), 'plans.stamp_ns')
        except MetricUnavailable as exc:
            self.plans.reject(str(exc))
            return False
        normalized = dict(item)
        normalized['collector_sequence'] = sequence
        return self.plans.append(normalized)

    def record_state_transition(
        self,
        *,
        kind: str,
        subject: str,
        value: Any,
        stamp_ns: int,
        details: Mapping[str, Any] | None = None,
    ) -> bool:
        """Retain only actual state changes while counting unchanged observations."""
        key = (kind, subject)
        buffer = self.buffers['state_events']
        previous = self._last_state.get(key, _MISSING)
        if previous is not _MISSING and previous == value:
            _ = self.next_sequence
            buffer.observe_without_retaining()
            return False
        retained = self.record(
            'state_events',
            {
                'details': dict(details or {}),
                'kind': kind,
                'previous_value': None if previous is _MISSING else previous,
                'stamp_ns': stamp_ns,
                'subject': subject,
                'value': value,
            },
        )
        if retained:
            self._last_state[key] = value
        return retained

    def require_complete(self) -> None:
        """Fail if any configured evidence prefix overflowed or clock regressed."""
        failures: list[str] = []
        for buffer in self.buffers.values():
            try:
                buffer.require_complete()
            except EvidenceOverflow as exc:
                failures.append(str(exc))
        try:
            self.plans.require_complete()
        except EvidenceOverflow as exc:
            failures.append(str(exc))
        if self.clock.regression_count:
            failures.append(f'clock regressed {self.clock.regression_count} times')
        if failures:
            raise EvidenceOverflow('; '.join(failures))

    def snapshot(self) -> dict[str, Any]:
        """Return one JSON-safe atomic capture document body."""
        streams = {name: buffer.snapshot() for name, buffer in sorted(self.buffers.items())}
        streams['plans'] = self.plans.snapshot()
        return {
            'clock': self.clock.as_dict(),
            'collector_sequence': self._sequence,
            'limits': {
                'contact_record_capacity': CONTACT_RECORD_CAPACITY,
                'plan_message_capacity': PLAN_MESSAGE_CAPACITY,
                'plan_pose_capacity': PLAN_POSE_CAPACITY,
                'stream_capacities': dict(sorted(BUFFER_CAPACITIES.items())),
                'string_field_max_bytes': STRING_FIELD_MAX_BYTES,
            },
            'quality': {
                'collector_overflow': any(
                    buffer.quality.overflowed for buffer in self.buffers.values()
                )
                or self.plans.overflow_count > 0,
                'contact_record_count': self._contact_record_count,
                'contact_record_overflow_count': self._contact_record_overflow_count,
            },
            'streams': streams,
        }
