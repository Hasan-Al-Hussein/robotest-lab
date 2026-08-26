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

"""Known-answer and hostile-boundary tests for ADR 0005 canonical schedules."""

from copy import deepcopy

import pytest
from robotest_missions.fault_schedule import FaultScheduleError, build_fault_schedule

EMPTY_BYTES = '{"schema_version":1,"faults":[]}'
EMPTY_HASH = '26080d7dc8f4108a369962ecad1d2e29941a991af68beb415067eae1dc1de6f8'
LIDAR_BYTES = (
    '{"schema_version":1,"faults":[{"schema_version":1,"fault_id":"scan_drop",'
    '"target":1,"mode":1,"start_offset_ns":10000000000,"duration_ns":2000000000,'
    '"seed":42,"parameters":{}}]}'
)
LIDAR_HASH = '5f93838ca7c0be214858ffe8f62fa351b82d260f6223f139dfaf6bd3dcd224c8'
ODOM_BYTES = (
    '{"schema_version":1,"faults":[{"schema_version":1,"fault_id":"odom_drift",'
    '"target":2,"mode":4,"start_offset_ns":10000000000,"duration_ns":20000000000,'
    '"seed":42,"parameters":{"x_rate_nm_per_s":10000000,'
    '"yaw_rate_nrad_per_s":5000000}}]}'
)
ODOM_HASH = '3c72bc48e05221a52058112e58c94b578550deb2ee1ff4ad0d719675917335eb'

LIDAR = {
    'schema_version': 1,
    'fault_id': 'scan_drop',
    'target': 1,
    'mode': 1,
    'start_offset_ns': 10_000_000_000,
    'duration_ns': 2_000_000_000,
    'seed': 42,
    'parameters': {},
}
ODOM = {
    'parameters': {'yaw_rate_nrad_per_s': 5_000_000, 'x_rate_nm_per_s': 10_000_000},
    'duration_ns': 20_000_000_000,
    'seed': 42,
    'mode': 4,
    'target': 2,
    'fault_id': 'odom_drift',
    'start_offset_ns': 10_000_000_000,
    'schema_version': 1,
}


@pytest.mark.parametrize(
    ('faults', 'expected_bytes', 'expected_hash'),
    [
        ([], EMPTY_BYTES, EMPTY_HASH),
        ([LIDAR], LIDAR_BYTES, LIDAR_HASH),
        ([ODOM], ODOM_BYTES, ODOM_HASH),
    ],
)
def test_known_answer_bytes_and_hashes(faults, expected_bytes, expected_hash) -> None:
    schedule = build_fault_schedule(faults, claimed_sha256=expected_hash)
    assert schedule.canonical_json == expected_bytes
    assert schedule.sha256 == expected_hash
    assert not schedule.canonical_json.endswith('\n')


def test_input_order_and_parameter_order_normalize_deterministically() -> None:
    later_lidar = dict(LIDAR, start_offset_ns=40_000_000_000)
    schedule = build_fault_schedule([later_lidar, ODOM])
    assert [fault.fault_id for fault in schedule.faults] == ['odom_drift', 'scan_drop']
    assert '"parameters":{"x_rate_nm_per_s":10000000,"yaw_rate_nrad_per_s":5000000}' in (
        schedule.canonical_json
    )


@pytest.mark.parametrize(
    ('mutation', 'message'),
    [
        (
            lambda faults: faults.extend(
                [
                    dict(
                        LIDAR,
                        fault_id=f'f{i}',
                        target=1,
                        start_offset_ns=(i + 1) * 3_000_000_000,
                    )
                    for i in range(17)
                ]
            ),
            'exceeds 16',
        ),
        (lambda faults: faults.append(dict(LIDAR, start_offset_ns=499_999_999)), 'at least'),
        (lambda faults: faults.append(dict(LIDAR, duration_ns=0)), 'duration_ns'),
        (lambda faults: faults.append(dict(LIDAR, fault_id='bad id')), 'fault_id'),
        (lambda faults: faults.append(dict(LIDAR, target=2)), 'unsupported'),
        (
            lambda faults: faults.append(
                dict(
                    ODOM,
                    parameters={'x_rate_nm_per_s': 0, 'yaw_rate_nrad_per_s': 0},
                )
            ),
            'nonzero',
        ),
        (
            lambda faults: faults.extend(
                [
                    LIDAR,
                    dict(
                        LIDAR,
                        fault_id='scan_drop_2',
                        start_offset_ns=11_000_000_000,
                    ),
                ]
            ),
            'overlap',
        ),
        (lambda faults: faults.extend([LIDAR, dict(LIDAR, fault_id='scan_drop')]), 'unique'),
    ],
)
def test_invalid_schedule_contract_is_rejected(mutation, message) -> None:
    faults = []
    mutation(faults)
    with pytest.raises(FaultScheduleError, match=message):
        build_fault_schedule(faults)


def test_touching_same_target_intervals_are_allowed() -> None:
    second = deepcopy(LIDAR)
    second['fault_id'] = 'scan_drop_2'
    second['start_offset_ns'] = LIDAR['start_offset_ns'] + LIDAR['duration_ns']
    schedule = build_fault_schedule([second, LIDAR])
    assert len(schedule.faults) == 2


def test_claimed_hash_mismatch_is_rejected() -> None:
    with pytest.raises(FaultScheduleError, match='does not match'):
        build_fault_schedule([], claimed_sha256='0' * 64)


def test_seed_uses_the_uint64_wire_bound() -> None:
    schedule = build_fault_schedule([dict(LIDAR, seed=18_446_744_073_709_551_615)])
    assert schedule.faults[0].seed == 18_446_744_073_709_551_615
    with pytest.raises(FaultScheduleError, match='seed'):
        build_fault_schedule([dict(LIDAR, seed=18_446_744_073_709_551_616)])
