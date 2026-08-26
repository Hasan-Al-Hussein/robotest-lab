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

import copy
from typing import Any

import pytest
from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.safety_metrics import (
    lidar_dropout_command_safety_metrics,
    scenario3_stop_command_metrics,
)


def _command(stamp_ns: int, sequence: int, linear: float) -> dict[str, Any]:
    return {
        'angular_z_rad_s': 0.0,
        'collector_sequence': sequence,
        'linear_x_m_s': linear,
        'linear_y_m_s': 0.0,
        'stamp_ns': stamp_ns,
    }


def _scenario3_inputs() -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    state_events = [
        {
            'collector_sequence': 1,
            'kind': 'collision_monitor',
            'stamp_ns': 1_100_000_000,
            'value': 1,
        },
        {
            'collector_sequence': 2,
            'kind': 'collision_monitor',
            'stamp_ns': 1_500_000_000,
            'value': 0,
        },
    ]
    commands = [
        _command(900_000_000, 3, 0.2),
        _command(1_100_000_000, 4, 0.0),
        _command(1_300_000_000, 5, 0.0),
        _command(1_500_000_000, 6, 0.2),
    ]
    interaction = {
        'trajectory': {
            'targets': [
                {
                    'observed_pose': {'stamp_ns': 1_000_000_000},
                    'target_pose': {'x_m': -0.4},
                },
                {
                    'observed_pose': {'stamp_ns': 1_400_000_000},
                    'target_pose': {'x_m': 0.4},
                },
            ]
        }
    }
    return state_events, commands, interaction


def test_scenario3_proves_capture_derived_stop_and_final_zero_overlap() -> None:
    result = scenario3_stop_command_metrics(*_scenario3_inputs(), 2_000_000_000)
    assert result['maximum_continuous_stop_zero_ns'] == 300_000_000
    assert result['no_nonzero_command_during_stop'] is True


@pytest.mark.parametrize('failure', ['missing_stop', 'nonzero', 'short_overlap', 'order'])
def test_scenario3_safety_fails_closed(failure: str) -> None:
    events, commands, interaction = _scenario3_inputs()
    if failure == 'missing_stop':
        events = []
    elif failure == 'nonzero':
        commands[2]['linear_x_m_s'] = 0.1
    elif failure == 'short_overlap':
        events[0]['stamp_ns'] = 1_350_000_000
    else:
        commands[1]['collector_sequence'] = commands[0]['collector_sequence']
        commands[1]['stamp_ns'] = commands[0]['stamp_ns']
    with pytest.raises(MetricUnavailable):
        scenario3_stop_command_metrics(events, commands, interaction, 2_000_000_000)


def _scenario4_inputs() -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    commands = [
        _command(2_400_000_000, 1, 0.2),
        _command(3_000_000_000, 2, 0.0),
        _command(3_500_000_000, 3, 0.0),
        _command(4_100_000_000, 4, 0.2),
    ]
    scans = [
        {'collector_sequence': 5, 'stamp_ns': 1_700_000_000},
        {'collector_sequence': 6, 'stamp_ns': 1_900_000_000},
        {'collector_sequence': 7, 'stamp_ns': 4_000_000_000},
    ]
    return commands, scans


def test_lidar_dropout_reaches_zero_by_deadline_and_holds_to_restoration() -> None:
    commands, scans = _scenario4_inputs()
    result = lidar_dropout_command_safety_metrics(
        commands,
        scans,
        2_000_000_000,
        4_000_000_000,
        collision_monitor_source_timeout_s=0.60,
    )
    assert result['stale_threshold_stamp_ns'] == 2_500_000_000
    assert result['zero_command_stamp_ns'] == 3_000_000_000
    assert result['safe_until_restored_stamp_ns'] == 4_000_000_000


@pytest.mark.parametrize(
    'failure',
    ['timeout', 'late_zero', 'unsafe_hold', 'missing_scan', 'order'],
)
def test_lidar_dropout_command_safety_fails_closed(failure: str) -> None:
    commands, scans = _scenario4_inputs()
    timeout = 0.60
    if failure == 'timeout':
        timeout = 0.61
    elif failure == 'late_zero':
        commands[1]['stamp_ns'] = 3_600_000_000
    elif failure == 'unsafe_hold':
        commands[2]['linear_x_m_s'] = 0.1
    elif failure == 'missing_scan':
        scans = [scans[-1]]
    else:
        commands = copy.deepcopy(commands)
        commands[1]['collector_sequence'] = commands[0]['collector_sequence']
        commands[1]['stamp_ns'] = commands[0]['stamp_ns']
    with pytest.raises(MetricUnavailable):
        lidar_dropout_command_safety_metrics(
            commands,
            scans,
            2_000_000_000,
            4_000_000_000,
            collision_monitor_source_timeout_s=timeout,
        )
