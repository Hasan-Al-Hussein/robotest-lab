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

"""Capture-derived final-command safety gates for Scenarios 3 and 4."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from robotest_metrics.errors import MetricUnavailable
from robotest_metrics.geometry import require_finite, require_int

_STOP_ACTION = 1
_LINEAR_ZERO_TOLERANCE_M_S = 0.02
_ANGULAR_ZERO_TOLERANCE_RAD_S = 0.05
_MINIMUM_STOP_ZERO_NS = 200_000_000
_LIDAR_SOURCE_TIMEOUT_NS = 600_000_000
_LIDAR_ZERO_DEADLINE_NS = 1_000_000_000


def _ordered(
    samples: Sequence[Mapping[str, Any]],
    name: str,
) -> list[Mapping[str, Any]]:
    previous: tuple[int, int] | None = None
    result: list[Mapping[str, Any]] = []
    for index, sample in enumerate(samples):
        order = (
            require_int(sample.get('stamp_ns'), f'{name}[{index}].stamp_ns'),
            require_int(sample.get('collector_sequence'), f'{name}[{index}].collector_sequence'),
        )
        if previous is not None and order <= previous:
            raise MetricUnavailable(f'{name} is not in deterministic collector order')
        previous = order
        result.append(sample)
    return result


def _is_zero(command: Mapping[str, Any]) -> bool:
    return (
        abs(require_finite(command.get('linear_x_m_s'), 'command.linear_x_m_s'))
        <= _LINEAR_ZERO_TOLERANCE_M_S
        and abs(require_finite(command.get('linear_y_m_s'), 'command.linear_y_m_s'))
        <= _LINEAR_ZERO_TOLERANCE_M_S
        and abs(require_finite(command.get('angular_z_rad_s'), 'command.angular_z_rad_s'))
        <= _ANGULAR_ZERO_TOLERANCE_RAD_S
    )


def _held_command(
    commands: Sequence[Mapping[str, Any]],
    stamp_ns: int,
) -> Mapping[str, Any] | None:
    held: Mapping[str, Any] | None = None
    for command in commands:
        if require_int(command.get('stamp_ns'), 'command.stamp_ns') > stamp_ns:
            break
        held = command
    return held


def _stop_intervals(
    state_events: Sequence[Mapping[str, Any]],
    terminal_stamp_ns: int,
) -> list[tuple[int, int]]:
    events = [
        event
        for event in _ordered(state_events, 'state_events')
        if event.get('kind') == ('collision_monitor')
    ]
    intervals: list[tuple[int, int]] = []
    for index, event in enumerate(events):
        if event.get('value') != _STOP_ACTION:
            continue
        start = require_int(event.get('stamp_ns'), 'collision_monitor.stamp_ns')
        end = (
            require_int(events[index + 1].get('stamp_ns'), 'collision_monitor.next_stamp_ns')
            if index + 1 < len(events)
            else terminal_stamp_ns
        )
        if end > start:
            intervals.append((start, end))
    return intervals


def scenario3_stop_command_metrics(
    state_events: Sequence[Mapping[str, Any]],
    commands: Sequence[Mapping[str, Any]],
    interaction: Mapping[str, Any],
    terminal_stamp_ns: int,
) -> dict[str, Any]:
    """Prove STOP/final-zero correlation over the observed exposed crossing."""
    terminal = require_int(terminal_stamp_ns, 'terminal_stamp_ns')
    trajectory = interaction.get('trajectory')
    if not isinstance(trajectory, Mapping) or not isinstance(trajectory.get('targets'), list):
        raise MetricUnavailable('Scenario 3 bounded trajectory evidence is missing')
    exposed: list[tuple[int, int]] = []
    for index, target in enumerate(trajectory['targets']):
        if not isinstance(target, Mapping):
            raise MetricUnavailable(f'Scenario 3 target[{index}] is invalid')
        target_pose = target.get('target_pose')
        observation = target.get('observed_pose')
        if not isinstance(target_pose, Mapping) or not isinstance(observation, Mapping):
            raise MetricUnavailable(f'Scenario 3 target[{index}] lacks observed-pose proof')
        x_value = target_pose.get('x_m', target_pose.get('x'))
        x_m = require_finite(x_value, f'Scenario 3 target[{index}].x')
        stamp = require_int(observation.get('stamp_ns'), f'Scenario 3 target[{index}].stamp_ns')
        if -0.425 < x_m < 0.425:
            exposed.append((stamp, index))
    if len(exposed) < 2:
        raise MetricUnavailable('Scenario 3 observed exposed-crossing interval is missing')
    exposed.sort()
    exposed_start = exposed[0][0]
    exposed_end = exposed[-1][0]
    if exposed_end <= exposed_start:
        raise MetricUnavailable('Scenario 3 exposed-crossing interval has no duration')
    ordered_commands = _ordered(commands, 'cmd_vel')
    intervals = _stop_intervals(state_events, terminal)
    if not intervals:
        raise MetricUnavailable('collision monitor never reported STOP')
    maximum_overlap = 0
    interval_evidence: list[dict[str, Any]] = []
    for start, end in intervals:
        held = _held_command(ordered_commands, start)
        if held is None or not _is_zero(held):
            raise MetricUnavailable('nonzero or missing final command at STOP interval start')
        in_interval = [
            command
            for command in ordered_commands
            if start <= require_int(command.get('stamp_ns'), 'command.stamp_ns') < end
        ]
        if any(not _is_zero(command) for command in in_interval):
            raise MetricUnavailable('nonzero final command was published during STOP')
        overlap_start = max(start, exposed_start)
        overlap_end = min(end, exposed_end)
        overlap = max(0, overlap_end - overlap_start)
        maximum_overlap = max(maximum_overlap, overlap)
        interval_evidence.append(
            {
                'end_stamp_ns': end,
                'exposed_zero_overlap_ns': overlap,
                'start_stamp_ns': start,
            }
        )
    if maximum_overlap < _MINIMUM_STOP_ZERO_NS:
        raise MetricUnavailable('STOP/final-zero exposed overlap is below 0.20 s')
    return {
        'exposed_end_stamp_ns': exposed_end,
        'exposed_start_stamp_ns': exposed_start,
        'maximum_continuous_stop_zero_ns': maximum_overlap,
        'minimum_required_ns': _MINIMUM_STOP_ZERO_NS,
        'no_nonzero_command_during_stop': True,
        'stop_intervals': interval_evidence,
    }


def lidar_dropout_command_safety_metrics(
    commands: Sequence[Mapping[str, Any]],
    validated_scans: Sequence[Mapping[str, Any]],
    configured_activation_stamp_ns: int,
    restored_stamp_ns: int,
    *,
    collision_monitor_source_timeout_s: Any,
) -> dict[str, Any]:
    """Prove the frozen source-timeout zero-command deadline and safe hold."""
    source_timeout = require_finite(
        collision_monitor_source_timeout_s,
        'collision_monitor_source_timeout_s',
    )
    if source_timeout != 0.60:
        raise MetricUnavailable('collision-monitor source_timeout must be exactly 0.60 s')
    activation = require_int(configured_activation_stamp_ns, 'configured activation')
    restored = require_int(restored_stamp_ns, 'restored stamp')
    if restored <= activation:
        raise MetricUnavailable('restored stamp must follow dropout activation')
    scan_stamps = sorted(
        require_int(sample.get('stamp_ns'), 'validated_scan.stamp_ns') for sample in validated_scans
    )
    before = [stamp for stamp in scan_stamps if stamp < activation]
    if not before:
        raise MetricUnavailable('validated scan before dropout is missing')
    stale_stamp = before[-1] + _LIDAR_SOURCE_TIMEOUT_NS
    ordered_commands = _ordered(commands, 'cmd_vel')
    held = _held_command(ordered_commands, stale_stamp)
    zero_stamp: int | None = stale_stamp if held is not None and _is_zero(held) else None
    if zero_stamp is None:
        for command in ordered_commands:
            stamp = require_int(command.get('stamp_ns'), 'command.stamp_ns')
            if stamp >= stale_stamp and _is_zero(command):
                zero_stamp = stamp
                break
    if zero_stamp is None or zero_stamp > stale_stamp + _LIDAR_ZERO_DEADLINE_NS:
        raise MetricUnavailable('final command did not reach zero within dropout deadline')
    held_zero = _held_command(ordered_commands, zero_stamp)
    if held_zero is None or not _is_zero(held_zero):
        raise MetricUnavailable('zero-command hold could not be established')
    for command in ordered_commands:
        stamp = require_int(command.get('stamp_ns'), 'command.stamp_ns')
        if zero_stamp <= stamp <= restored and not _is_zero(command):
            raise MetricUnavailable('nonzero final command appeared before scan restoration')
    return {
        'collision_monitor_source_timeout_s': source_timeout,
        'last_validated_before_dropout_stamp_ns': before[-1],
        'safe_until_restored_stamp_ns': restored,
        'stale_threshold_stamp_ns': stale_stamp,
        'zero_command_deadline_stamp_ns': stale_stamp + _LIDAR_ZERO_DEADLINE_NS,
        'zero_command_stamp_ns': zero_stamp,
    }
