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

import pytest
import robotest_metrics.collector as collector_module
from robotest_metrics.buffers import PlanPrefixBuffer, PrefixBuffer
from robotest_metrics.collector import ClockSummary, CollectorCore
from robotest_metrics.constants import BUFFER_CAPACITIES, CONTACT_MESSAGE_CAPACITY
from robotest_metrics.errors import EvidenceOverflow


def test_prefix_buffer_retains_prefix_and_records_first_overflow() -> None:
    buffer = PrefixBuffer[int]('test', 2)
    assert buffer.append(10, sequence=1, stamp_ns=100)
    assert buffer.append(20, sequence=2, stamp_ns=200)
    assert not buffer.append(30, sequence=3, stamp_ns=300)
    assert not buffer.append(40, sequence=4, stamp_ns=400)
    assert buffer.items == [10, 20]
    assert buffer.quality.first_overflow_sequence == 3
    assert buffer.quality.first_overflow_stamp_ns == 300
    assert buffer.quality.overflow_count == 2
    with pytest.raises(EvidenceOverflow, match='overflowed by 2'):
        buffer.require_complete()


def test_invalid_evidence_does_not_consume_prefix_capacity() -> None:
    buffer = PrefixBuffer[int]('test', 1)
    buffer.reject('bad')
    assert buffer.append(1, sequence=2, stamp_ns=20)
    assert buffer.quality.ingress_count == 2
    assert buffer.quality.invalid_count == 1
    assert buffer.quality.first_invalid_reason == 'bad'


@pytest.mark.parametrize('message_capacity,pose_capacity', [(1, 9), (9, 2)])
def test_plan_buffer_enforces_both_limits(message_capacity: int, pose_capacity: int) -> None:
    buffer = PlanPrefixBuffer(message_capacity, pose_capacity)
    first = {'collector_sequence': 1, 'poses': [{}, {}], 'stamp_ns': 10}
    second = {'collector_sequence': 2, 'poses': [{}], 'stamp_ns': 20}
    assert buffer.append(first)
    assert not buffer.append(second)
    assert buffer.items == [first]
    assert buffer.overflow_count == 1
    with pytest.raises(EvidenceOverflow):
        buffer.require_complete()


def test_plan_invalid_shape_is_not_an_overflow() -> None:
    buffer = PlanPrefixBuffer(1, 1)
    assert not buffer.append({'poses': 'not-an-array'})
    assert buffer.invalid_count == 1
    assert buffer.overflow_count == 0


def test_clock_summary_is_constant_space_and_exposes_regression() -> None:
    clock = ClockSummary()
    for stamp in (10, 10, 25, 20):
        clock.observe(stamp)
    assert clock.as_dict() == {
        'count': 4,
        'duplicate_count': 1,
        'first_stamp_ns': 10,
        'latest_stamp_ns': 20,
        'maximum_positive_gap_ns': 15,
        'regression_count': 1,
    }


def test_collector_declares_every_contract_capacity() -> None:
    core = CollectorCore()
    assert set(core.buffers) == set(BUFFER_CAPACITIES)
    assert core.buffers['contacts'].quality.capacity == CONTACT_MESSAGE_CAPACITY
    assert core.snapshot()['limits']['stream_capacities'] == dict(sorted(BUFFER_CAPACITIES.items()))


def test_collector_collapses_unchanged_state_but_preserves_previous_value() -> None:
    core = CollectorCore()
    assert core.record_state_transition(
        kind='lifecycle', subject='planner', value='active', stamp_ns=10
    )
    assert not core.record_state_transition(
        kind='lifecycle', subject='planner', value='active', stamp_ns=20
    )
    assert core.record_state_transition(
        kind='lifecycle', subject='planner', value='inactive', stamp_ns=30
    )
    stream = core.snapshot()['streams']['state_events']
    assert stream['quality']['ingress_count'] == 3
    assert [item['previous_value'] for item in stream['items']] == [None, 'active']


def test_collector_rejects_oversized_strings_without_truncating() -> None:
    core = CollectorCore()
    assert not core.record('fault_events', {'detail': 'x' * 4097, 'stamp_ns': 1})
    stream = core.snapshot()['streams']['fault_events']
    assert stream['items'] == []
    assert stream['quality']['invalid_count'] == 1


def test_contact_nested_limit_marks_stream_overflow(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(collector_module, 'CONTACT_RECORD_CAPACITY', 2)
    core = CollectorCore()
    records = [{'collision1': 'a', 'collision2': 'b'}] * 2
    assert core.record('contacts', {'contacts': records, 'stamp_ns': 1})
    assert not core.record('contacts', {'contacts': [records[0]], 'stamp_ns': 2})
    snapshot = core.snapshot()
    assert snapshot['streams']['contacts']['items'][0]['contacts'] == records
    assert snapshot['quality']['collector_overflow'] is True
    assert snapshot['quality']['contact_record_overflow_count'] == 1
    with pytest.raises(EvidenceOverflow):
        core.require_complete()
