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
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.recovery_metrics import (
    analyze_lidar_dropout,
    sensor_recovery_metrics,
    supervisor_recovery_metrics,
)


def _scan(stamp: int, payload: str = 'a') -> dict[str, object]:
    return {'payload_sha256': payload * 64, 'stamp_ns': stamp}


def test_lidar_dropout_reconciles_half_open_affected_interval_and_restoration() -> None:
    raw = [_scan(stamp) for stamp in (0, 100, 200, 300, 400)]
    validated = [_scan(stamp) for stamp in (0, 300, 400)]
    result = analyze_lidar_dropout(
        raw,
        validated,
        100,
        300,
        proxy_affected_message_count=2,
        proxy_first_affected_stamp_ns=100,
        proxy_last_affected_stamp_ns=200,
        proxy_restored_stamp_ns=300,
    )
    assert result['affected_message_count'] == 2
    assert result['actual_first_affected_stamp_ns'] == 100
    assert result['actual_last_affected_stamp_ns'] == 200
    assert result['actual_deactivation_stamp_ns'] == 300


@pytest.mark.parametrize('failure', ['leak', 'counter', 'payload'])
def test_lidar_dropout_fails_closed_on_reconciliation_errors(failure: str) -> None:
    raw = [_scan(stamp) for stamp in (0, 100, 200, 300)]
    validated = [_scan(0), _scan(300)]
    count = 2
    if failure == 'leak':
        validated.insert(1, _scan(200))
    elif failure == 'counter':
        count = 3
    else:
        validated[-1] = _scan(300, 'b')
    with pytest.raises(MetricUnavailable):
        analyze_lidar_dropout(
            raw,
            validated,
            100,
            300,
            proxy_affected_message_count=count,
            proxy_first_affected_stamp_ns=100,
            proxy_last_affected_stamp_ns=200,
            proxy_restored_stamp_ns=300,
        )


def _recovery_inputs() -> dict[str, object]:
    stamps = list(range(0, 1_000_000_001, 200_000_000))
    return {
        'actual_deactivation_stamp_ns': 0,
        'collision_events': [],
        'feedback': [],
        'ground_truth': [
            {'stamp_ns': stamp, 'x_m': stamp / 10_000_000_000, 'y_m': 0.0, 'yaw_rad': 0.0}
            for stamp in stamps
        ],
        'lifecycle_samples': [
            {
                'collector_sequence': 1,
                'node': 'planner_server',
                'stamp_ns': 0,
                'state': 'active',
            },
            {
                'collector_sequence': 2,
                'node': 'planner_server',
                'stamp_ns': 1_000_000_000,
                'state': 'active',
            },
        ],
        'mission_terminal': {'stamp_ns': 2_000_000_000, 'status': 'SUCCEEDED'},
        'required_lifecycle_nodes': ['planner_server'],
        'validated_scans': [_scan(stamp) for stamp in stamps],
    }


def test_sensor_recovery_requires_complete_one_second_stability_window() -> None:
    result = sensor_recovery_metrics(**_recovery_inputs())
    assert result['window_start_stamp_ns'] == 0
    assert result['window_end_stamp_ns'] == 1_000_000_000
    assert result['sensor_recovery_time_sim_s'] == 1.0
    assert result['scan_evidence']['sample_count'] == 6
    assert result['progress']['predicate'] == 'planar_displacement'


@pytest.mark.parametrize('failure', ['stale_scan', 'lifecycle', 'mission', 'collision'])
def test_recovery_window_resets_on_each_safety_break(failure: str) -> None:
    inputs = _recovery_inputs()
    if failure == 'stale_scan':
        inputs['validated_scans'] = [_scan(0), _scan(500_000_000), _scan(1_000_000_000)]
    elif failure == 'lifecycle':
        inputs['lifecycle_samples'].insert(
            1,
            {
                'collector_sequence': 2,
                'node': 'planner_server',
                'stamp_ns': 500_000_000,
                'state': 'inactive',
            },
        )
        inputs['lifecycle_samples'][-1]['collector_sequence'] = 3
    elif failure == 'mission':
        inputs['mission_terminal'] = {'stamp_ns': 500_000_000, 'status': 'ABORTED'}
    else:
        inputs['collision_events'] = [{'start_stamp_ns': 500_000_000}]
    expected = 'eventually failed' if failure == 'mission' else 'no qualifying'
    with pytest.raises(MetricUnavailable, match=expected):
        sensor_recovery_metrics(**inputs)


def test_single_feedback_transition_can_prove_monotonic_waypoint_progress() -> None:
    inputs = _recovery_inputs()
    inputs['ground_truth'] = [
        {'stamp_ns': stamp, 'x_m': 0.0, 'y_m': 0.0, 'yaw_rad': 0.0}
        for stamp in range(0, 1_000_000_001, 200_000_000)
    ]
    inputs['feedback'] = [
        {
            'collector_sequence': 3,
            'current_waypoint': 1,
            'previous_waypoint': 0,
            'stamp_ns': 500_000_000,
        }
    ]
    result = sensor_recovery_metrics(**inputs)
    assert result['progress'] == {'from': 0, 'predicate': 'waypoint_advance', 'to': 1}


def test_later_terminal_failure_invalidates_an_earlier_stable_window() -> None:
    inputs = _recovery_inputs()
    inputs['mission_terminal'] = {'stamp_ns': 2_000_000_000, 'status': 'ABORTED'}
    with pytest.raises(MetricUnavailable, match='eventually failed'):
        sensor_recovery_metrics(**inputs)


def test_recovery_rejects_nonmonotonic_lifecycle_and_feedback_evidence() -> None:
    inputs = _recovery_inputs()
    inputs['lifecycle_samples'] = list(reversed(inputs['lifecycle_samples']))
    with pytest.raises(MetricUnavailable, match='callback order'):
        sensor_recovery_metrics(**inputs)
    inputs = _recovery_inputs()
    inputs['feedback'] = [
        {'collector_sequence': 1, 'current_waypoint': 1, 'stamp_ns': 100},
        {'collector_sequence': 2, 'current_waypoint': 0, 'stamp_ns': 200},
    ]
    with pytest.raises(MetricUnavailable, match='regressed'):
        sensor_recovery_metrics(**inputs)
    inputs = _recovery_inputs()
    inputs['required_lifecycle_nodes'] = []
    with pytest.raises(MetricUnavailable, match='node set is empty'):
        sensor_recovery_metrics(**inputs)


def test_supervisor_recovery_counts_only_scheduled_new_child_starts() -> None:
    events = [
        {'kind': 'supervisor_started', 'schema_version': 1, 'sequence': 1, 'steady_wall_ns': 0},
        {'kind': 'child_started', 'schema_version': 1, 'sequence': 2, 'steady_wall_ns': 0},
        {
            'kind': 'failure_detected',
            'schema_version': 1,
            'sequence': 3,
            'steady_wall_ns': 1_000_000_000,
        },
        {
            'kind': 'readiness_changed',
            'ready': False,
            'schema_version': 1,
            'sequence': 4,
            'steady_wall_ns': 1_100_000_000,
        },
        {
            'kind': 'restart_scheduled',
            'schema_version': 1,
            'sequence': 5,
            'steady_wall_ns': 1_200_000_000,
        },
        {
            'kind': 'child_started',
            'schema_version': 1,
            'sequence': 6,
            'steady_wall_ns': 1_500_000_000,
        },
        {
            'kind': 'readiness_changed',
            'ready': True,
            'schema_version': 1,
            'sequence': 7,
            'steady_wall_ns': 2_000_000_000,
        },
    ]
    result = supervisor_recovery_metrics(events)
    assert result['restart_count'] == 1
    assert result['supervisor_recovery_time_wall_s'] == 1.0


def test_supervisor_recovery_rejects_unpaired_restart_or_missing_false_state() -> None:
    base = [
        {'kind': 'supervisor_started', 'schema_version': 1, 'sequence': 1, 'steady_wall_ns': 0},
        {'kind': 'failure_detected', 'schema_version': 1, 'sequence': 2, 'steady_wall_ns': 1},
        {
            'kind': 'readiness_changed',
            'ready': True,
            'schema_version': 1,
            'sequence': 3,
            'steady_wall_ns': 2,
        },
    ]
    with pytest.raises(MetricUnavailable, match='false'):
        supervisor_recovery_metrics(base)
    with pytest.raises(MetricUnavailable, match='no scheduled restart'):
        supervisor_recovery_metrics(
            [
                base[0],
                base[1],
                {
                    'kind': 'readiness_changed',
                    'ready': False,
                    'schema_version': 1,
                    'sequence': 3,
                    'steady_wall_ns': 1,
                },
                {**base[2], 'sequence': 4},
            ]
        )


def test_supervisor_recovery_uses_only_latest_process_segment() -> None:
    old_segment = [
        {'kind': 'supervisor_started', 'schema_version': 1, 'sequence': 1, 'steady_wall_ns': 0},
        {
            'kind': 'failure_detected',
            'schema_version': 1,
            'sequence': 2,
            'steady_wall_ns': 9_000_000_000,
        },
    ]
    new_segment = [
        {'kind': 'supervisor_started', 'schema_version': 1, 'sequence': 1, 'steady_wall_ns': 0},
        {'kind': 'failure_detected', 'schema_version': 1, 'sequence': 2, 'steady_wall_ns': 10},
        {
            'kind': 'readiness_changed',
            'ready': False,
            'schema_version': 1,
            'sequence': 3,
            'steady_wall_ns': 20,
        },
        {'kind': 'restart_scheduled', 'schema_version': 1, 'sequence': 4, 'steady_wall_ns': 30},
        {'kind': 'child_started', 'schema_version': 1, 'sequence': 5, 'steady_wall_ns': 40},
        {
            'kind': 'readiness_changed',
            'ready': True,
            'schema_version': 1,
            'sequence': 6,
            'steady_wall_ns': 50,
        },
    ]
    result = supervisor_recovery_metrics([*old_segment, *new_segment])
    assert result['failure_detected_wall_ns'] == 10
    assert result['latest_supervisor_started_sequence'] == 1


@pytest.mark.parametrize('mutation', ['missing_start', 'schema', 'sequence', 'steady'])
def test_supervisor_recovery_rejects_invalid_latest_segment(mutation: str) -> None:
    events = [
        {'kind': 'supervisor_started', 'schema_version': 1, 'sequence': 1, 'steady_wall_ns': 0},
        {'kind': 'failure_detected', 'schema_version': 1, 'sequence': 2, 'steady_wall_ns': 10},
    ]
    if mutation == 'missing_start':
        events = events[1:]
    elif mutation == 'schema':
        events[1]['schema_version'] = 2
    elif mutation == 'sequence':
        events[1]['sequence'] = 1
    else:
        events[1]['steady_wall_ns'] = -1
    with pytest.raises(MetricUnavailable):
        supervisor_recovery_metrics(events)
