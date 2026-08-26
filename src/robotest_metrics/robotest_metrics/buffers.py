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

"""Prefix-retaining bounded evidence buffers."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Generic, TypeVar

from robotest_metrics.errors import EvidenceOverflow

T = TypeVar('T')


@dataclass
class BufferQuality:
    """Constant-space evidence-quality counters."""

    capacity: int
    ingress_count: int = 0
    retained_count: int = 0
    invalid_count: int = 0
    overflow_count: int = 0
    first_overflow_sequence: int | None = None
    first_overflow_stamp_ns: int | None = None
    first_invalid_reason: str | None = None

    @property
    def overflowed(self) -> bool:
        """Return whether at least one item exceeded the capacity."""
        return self.overflow_count > 0

    def as_dict(self) -> dict[str, Any]:
        """Return a JSON-safe quality record."""
        return {
            'capacity': self.capacity,
            'first_invalid_reason': self.first_invalid_reason,
            'first_overflow_sequence': self.first_overflow_sequence,
            'first_overflow_stamp_ns': self.first_overflow_stamp_ns,
            'ingress_count': self.ingress_count,
            'invalid_count': self.invalid_count,
            'overflow_count': self.overflow_count,
            'overflowed': self.overflowed,
            'retained_count': self.retained_count,
        }


@dataclass
class PrefixBuffer(Generic[T]):  # noqa: UP046 - Jazzy packages retain 3.10-compatible syntax.
    """Retain the first ``capacity`` valid items and fail closed afterwards."""

    name: str
    capacity: int
    items: list[T] = field(default_factory=list)
    quality: BufferQuality = field(init=False)

    def __post_init__(self) -> None:
        """Validate capacity and initialize constant-space counters."""
        if self.capacity <= 0:
            raise ValueError('capacity must be positive')
        self.quality = BufferQuality(capacity=self.capacity)

    def append(self, item: T, *, sequence: int, stamp_ns: int | None) -> bool:
        """Append when capacity permits; otherwise preserve the prefix."""
        self.quality.ingress_count += 1
        if len(self.items) >= self.capacity:
            self.quality.overflow_count += 1
            if self.quality.first_overflow_sequence is None:
                self.quality.first_overflow_sequence = sequence
                self.quality.first_overflow_stamp_ns = stamp_ns
            return False
        self.items.append(item)
        self.quality.retained_count += 1
        return True

    def reject(self, reason: str) -> None:
        """Record invalid evidence without consuming retained capacity."""
        self.quality.ingress_count += 1
        self.quality.invalid_count += 1
        if self.quality.first_invalid_reason is None:
            self.quality.first_invalid_reason = reason

    def observe_without_retaining(self) -> None:
        """Count an intentionally collapsed observation such as unchanged state."""
        self.quality.ingress_count += 1

    def mark_overflow(self, *, sequence: int, stamp_ns: int | None) -> None:
        """Record an external nested-limit overflow without changing the prefix."""
        self.quality.ingress_count += 1
        self.quality.overflow_count += 1
        if self.quality.first_overflow_sequence is None:
            self.quality.first_overflow_sequence = sequence
            self.quality.first_overflow_stamp_ns = stamp_ns

    def require_complete(self) -> None:
        """Raise when the buffer overflowed."""
        if self.quality.overflowed:
            raise EvidenceOverflow(f'{self.name} overflowed by {self.quality.overflow_count} items')

    def snapshot(self) -> dict[str, Any]:
        """Return retained items and quality without mutating the buffer."""
        return {'items': list(self.items), 'quality': self.quality.as_dict()}


@dataclass
class PlanPrefixBuffer:
    """Bound both plan-message count and total retained pose count."""

    message_capacity: int
    pose_capacity: int
    items: list[dict[str, Any]] = field(default_factory=list)
    ingress_count: int = 0
    retained_pose_count: int = 0
    invalid_count: int = 0
    first_invalid_reason: str | None = None
    overflow_count: int = 0
    first_overflow_sequence: int | None = None
    first_overflow_stamp_ns: int | None = None

    def __post_init__(self) -> None:
        """Validate both independent plan capacities."""
        if self.message_capacity <= 0 or self.pose_capacity <= 0:
            raise ValueError('plan capacities must be positive')

    def append(self, item: dict[str, Any]) -> bool:
        """Retain a complete plan or reject it without retaining a partial plan."""
        self.ingress_count += 1
        poses = item.get('poses')
        if not isinstance(poses, list):
            self.invalid_count += 1
            if self.first_invalid_reason is None:
                self.first_invalid_reason = 'plan poses must be a list'
            return False
        pose_count = len(poses)
        over_messages = len(self.items) >= self.message_capacity
        over_poses = self.retained_pose_count + pose_count > self.pose_capacity
        if over_messages or over_poses:
            self.overflow_count += 1
            if self.first_overflow_sequence is None:
                self.first_overflow_sequence = item.get('collector_sequence')
                self.first_overflow_stamp_ns = item.get('stamp_ns')
            return False
        self.items.append(item)
        self.retained_pose_count += pose_count
        return True

    def reject(self, reason: str) -> None:
        """Count a normalized plan rejected before capacity evaluation."""
        self.ingress_count += 1
        self.invalid_count += 1
        if self.first_invalid_reason is None:
            self.first_invalid_reason = reason

    def require_complete(self) -> None:
        """Raise when either plan limit overflowed."""
        if self.overflow_count:
            raise EvidenceOverflow(f'plans overflowed by {self.overflow_count} messages')

    def snapshot(self) -> dict[str, Any]:
        """Return retained plan evidence and both limits."""
        return {
            'items': list(self.items),
            'quality': {
                'first_overflow_sequence': self.first_overflow_sequence,
                'first_overflow_stamp_ns': self.first_overflow_stamp_ns,
                'first_invalid_reason': self.first_invalid_reason,
                'ingress_count': self.ingress_count,
                'invalid_count': self.invalid_count,
                'message_capacity': self.message_capacity,
                'overflow_count': self.overflow_count,
                'overflowed': self.overflow_count > 0,
                'pose_capacity': self.pose_capacity,
                'retained_count': len(self.items),
                'retained_pose_count': self.retained_pose_count,
            },
        }
