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

"""Prefix-retaining evidence containers with fail-closed overflow."""

from __future__ import annotations

from dataclasses import dataclass, field

from robotest_scenarios.errors import ProtocolError


@dataclass(slots=True)
class PrefixBuffer[T]:
    """Retain a fixed prefix and reject the first item beyond capacity."""

    name: str
    capacity: int
    items: list[T] = field(default_factory=list)
    ingress_count: int = 0
    accepted_count: int = 0
    invalid_count: int = 0
    overflow_count: int = 0
    first_overflow_sequence: int | None = None
    first_overflow_stamp_ns: int | None = None

    def __post_init__(self) -> None:
        if self.capacity <= 0:
            raise ValueError('prefix buffer capacity must be positive')

    def add(self, item: T, *, sequence: int, stamp_ns: int | None) -> None:
        """Append one item, preserving the existing prefix if capacity is exceeded."""
        self.ingress_count += 1
        if len(self.items) >= self.capacity:
            self.overflow_count += 1
            if self.first_overflow_sequence is None:
                self.first_overflow_sequence = sequence
                self.first_overflow_stamp_ns = stamp_ns
            raise ProtocolError(f'{self.name} prefix buffer overflowed at sequence {sequence}')
        self.items.append(item)
        self.accepted_count += 1

    def reject_invalid(self) -> None:
        """Record one rejected item without consuming retained capacity."""
        self.ingress_count += 1
        self.invalid_count += 1

    def observe_unretained(self) -> None:
        """Count an unchanged state sample without duplicating retained evidence."""
        self.ingress_count += 1
        self.accepted_count += 1

    def invalidate_retained(self, item: T) -> None:
        """Reclassify one provisionally retained item as invalid without new ingress."""
        for index, retained in enumerate(self.items):
            if retained is item:
                self.items.pop(index)
                self.accepted_count -= 1
                self.invalid_count += 1
                return
        raise ValueError(f'{self.name} item is not retained')

    def quality(self) -> dict[str, int | bool | None]:
        """Return stable machine-readable capacity evidence."""
        return {
            'accepted_count': self.accepted_count,
            'capacity': self.capacity,
            'first_overflow_sequence': self.first_overflow_sequence,
            'first_overflow_stamp_ns': self.first_overflow_stamp_ns,
            'ingress_count': self.ingress_count,
            'invalid_count': self.invalid_count,
            'overflow': self.overflow_count > 0,
            'overflow_count': self.overflow_count,
            'retained_count': len(self.items),
        }
