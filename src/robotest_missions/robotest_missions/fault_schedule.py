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

"""Pure ADR 0005 schedule validation, normalization, and hashing."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping, Sequence
from typing import Any

from robotest_missions.models import FaultSchedule, FaultSpec

MAX_FAULTS = 16
MAX_TIME_NS = 3_600_000_000_000
MIN_START_OFFSET_NS = 500_000_000
MAX_RATE = 1_000_000_000
UINT64_MAX = 18_446_744_073_709_551_615
FAULT_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$')
SHA256_PATTERN = re.compile(r'^[0-9a-f]{64}$')

TARGET_SCAN = 1
TARGET_ODOM = 2
TARGET_IMU = 3
MODE_PASS_THROUGH = 0
MODE_LIDAR_DROPOUT = 1
MODE_ODOM_DRIFT = 4


class FaultScheduleError(ValueError):
    """Raised when a schedule cannot satisfy the frozen wire contract."""


def _integer(value: Any, name: str, minimum: int, maximum: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool):
        raise FaultScheduleError(f'{name} must be an integer')
    if not minimum <= value <= maximum:
        raise FaultScheduleError(f'{name} must be in [{minimum}, {maximum}]')
    return value


def _parameters(target: int, mode: int, value: Any) -> tuple[tuple[str, int], ...]:
    if not isinstance(value, Mapping):
        raise FaultScheduleError('parameters must be an object')
    if mode in (MODE_PASS_THROUGH, MODE_LIDAR_DROPOUT):
        if value:
            raise FaultScheduleError('pass-through and LiDAR dropout parameters must be empty')
        return ()
    if mode == MODE_ODOM_DRIFT and target == TARGET_ODOM:
        expected = {'x_rate_nm_per_s', 'yaw_rate_nrad_per_s'}
        if set(value) != expected:
            raise FaultScheduleError('odometry drift parameters must contain exactly both rates')
        x_rate = _integer(value['x_rate_nm_per_s'], 'x_rate_nm_per_s', -MAX_RATE, MAX_RATE)
        yaw_rate = _integer(
            value['yaw_rate_nrad_per_s'],
            'yaw_rate_nrad_per_s',
            -MAX_RATE,
            MAX_RATE,
        )
        if x_rate == 0 and yaw_rate == 0:
            raise FaultScheduleError('at least one odometry drift rate must be nonzero')
        return (
            ('x_rate_nm_per_s', x_rate),
            ('yaw_rate_nrad_per_s', yaw_rate),
        )
    raise FaultScheduleError(f'unsupported target/mode pair: {target}/{mode}')


def _fault(value: Any) -> FaultSpec:
    if not isinstance(value, Mapping):
        raise FaultScheduleError('each fault must be an object')
    expected = {
        'schema_version',
        'fault_id',
        'target',
        'mode',
        'start_offset_ns',
        'duration_ns',
        'seed',
        'parameters',
    }
    if set(value) != expected:
        raise FaultScheduleError('fault fields do not match the canonical contract')
    if value['schema_version'] != 1:
        raise FaultScheduleError('fault schema_version must be 1')
    fault_id = value['fault_id']
    if not isinstance(fault_id, str) or FAULT_ID_PATTERN.fullmatch(fault_id) is None:
        raise FaultScheduleError('fault_id does not match the bounded ASCII contract')
    target = _integer(value['target'], 'target', TARGET_SCAN, TARGET_IMU)
    mode = _integer(value['mode'], 'mode', MODE_PASS_THROUGH, MODE_ODOM_DRIFT)
    valid_pair = (
        mode == MODE_PASS_THROUGH
        or (target == TARGET_SCAN and mode == MODE_LIDAR_DROPOUT)
        or (target == TARGET_ODOM and mode == MODE_ODOM_DRIFT)
    )
    if not valid_pair:
        raise FaultScheduleError(f'unsupported target/mode pair: {target}/{mode}')
    start = _integer(value['start_offset_ns'], 'start_offset_ns', 0, MAX_TIME_NS)
    duration = _integer(value['duration_ns'], 'duration_ns', 1, MAX_TIME_NS)
    if start < MIN_START_OFFSET_NS:
        raise FaultScheduleError('non-empty fault start_offset_ns must be at least 500000000')
    if start + duration > MAX_TIME_NS:
        raise FaultScheduleError('fault interval exceeds the 3600-second horizon')
    seed = _integer(value['seed'], 'seed', 0, UINT64_MAX)
    return FaultSpec(
        schema_version=1,
        fault_id=fault_id,
        target=target,
        mode=mode,
        start_offset_ns=start,
        duration_ns=duration,
        seed=seed,
        parameters=_parameters(target, mode, value['parameters']),
    )


def build_fault_schedule(
    raw_faults: Sequence[Any],
    *,
    claimed_sha256: str | None = None,
) -> FaultSchedule:
    """Build canonical bytes and digest after complete schedule validation."""
    if isinstance(raw_faults, (str, bytes)) or not isinstance(raw_faults, Sequence):
        raise FaultScheduleError('faults must be an array')
    if len(raw_faults) > MAX_FAULTS:
        raise FaultScheduleError(f'fault count exceeds {MAX_FAULTS}')
    faults = sorted(
        (_fault(value) for value in raw_faults),
        key=lambda fault: (fault.start_offset_ns, fault.target, fault.fault_id.encode('ascii')),
    )
    identifiers = [fault.fault_id for fault in faults]
    if len(identifiers) != len(set(identifiers)):
        raise FaultScheduleError('fault_id values must be unique')
    by_target: dict[int, list[FaultSpec]] = {}
    for fault in faults:
        previous = by_target.setdefault(fault.target, [])
        if (
            previous
            and previous[-1].start_offset_ns + previous[-1].duration_ns > fault.start_offset_ns
        ):
            raise FaultScheduleError('same-target fault intervals must not overlap')
        previous.append(fault)

    content = {'schema_version': 1, 'faults': [fault.as_dict() for fault in faults]}
    canonical_json = json.dumps(content, ensure_ascii=True, separators=(',', ':'))
    digest = hashlib.sha256(canonical_json.encode('utf-8')).hexdigest()
    if claimed_sha256 is not None:
        if not isinstance(claimed_sha256, str) or SHA256_PATTERN.fullmatch(claimed_sha256) is None:
            raise FaultScheduleError('claimed schedule hash must be lowercase SHA-256')
        if claimed_sha256 != digest:
            raise FaultScheduleError(
                f'claimed schedule hash {claimed_sha256} does not match computed {digest}'
            )
    return FaultSchedule(
        schema_version=1,
        faults=tuple(faults),
        canonical_json=canonical_json,
        sha256=digest,
    )
