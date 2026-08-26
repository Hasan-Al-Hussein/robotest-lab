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

"""Bounded canonical artifact tests."""

import hashlib
from pathlib import Path

import pytest
from robotest_scenarios.artifacts import (
    bounded_diagnostic,
    canonical_json_bytes,
    validate_new_output,
    write_canonical_json,
)
from robotest_scenarios.bounded import PrefixBuffer
from robotest_scenarios.errors import ProtocolError, ValidationError


def test_prefix_buffer_retains_prefix_and_reports_overflow() -> None:
    buffer = PrefixBuffer[int]('sample', 2)
    buffer.add(1, sequence=1, stamp_ns=10)
    buffer.add(2, sequence=2, stamp_ns=20)
    with pytest.raises(ProtocolError, match='overflowed'):
        buffer.add(3, sequence=3, stamp_ns=30)
    assert buffer.items == [1, 2]
    assert buffer.quality() == {
        'accepted_count': 2,
        'capacity': 2,
        'first_overflow_sequence': 3,
        'first_overflow_stamp_ns': 30,
        'ingress_count': 3,
        'invalid_count': 0,
        'overflow': True,
        'overflow_count': 1,
        'retained_count': 2,
    }


def test_json_and_sidecar_are_canonical_and_create_only(tmp_path: Path) -> None:
    path = tmp_path / 'result.json'
    digest = write_canonical_json(path, {'z': 1, 'a': 'value'})
    expected = b'{"a":"value","z":1}\n'
    assert path.read_bytes() == expected
    assert digest == hashlib.sha256(expected).hexdigest()
    assert (tmp_path / 'result.json.sha256').read_text(encoding='ascii') == (
        f'{digest}  result.json\n'
    )
    with pytest.raises(ValidationError, match='already exists'):
        validate_new_output(str(path), label='result')


def test_canonical_json_rejects_nonfinite_numbers() -> None:
    with pytest.raises(RuntimeError, match='canonical JSON'):
        canonical_json_bytes({'bad': float('nan')})


def test_bounded_diagnostic_respects_utf8_byte_limit() -> None:
    value = bounded_diagnostic('😀' * 2_000)
    assert len(value.encode('utf-8')) <= 4_096
    assert value == '😀' * 1_024
