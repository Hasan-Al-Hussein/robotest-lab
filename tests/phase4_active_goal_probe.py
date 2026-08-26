#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Prove one exact FollowWaypoints goal is executing before Phase 4 injection."""

from __future__ import annotations

import argparse
import os
import sys
import time
import uuid
from collections.abc import Sequence
from pathlib import Path
from typing import Any

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from phase4_acceptance import atomic_write_json, utc_now
from rclpy.action.graph import get_action_client_names_and_types_by_node
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default

SCHEMA_VERSION = 1
ACTION_NAME = '/robotest/follow_waypoints'
ACTION_TYPE = 'nav2_msgs/action/FollowWaypoints'
MISSION_NODE_NAME = 'mission_runner'
MISSION_NODE_NAMESPACE = '/robotest'
MAX_STATUS_ENTRIES = 1024
SPIN_QUANTUM_S = 0.05


def process_alive(pid: int) -> bool:
    """Return whether a positive PID still exists."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def stamp_ns(stamp: Any) -> int:
    """Return a structurally valid positive ROS timestamp."""
    seconds = getattr(stamp, 'sec', None)
    nanoseconds = getattr(stamp, 'nanosec', None)
    if (
        isinstance(seconds, bool)
        or not isinstance(seconds, int)
        or isinstance(nanoseconds, bool)
        or not isinstance(nanoseconds, int)
        or seconds < 0
        or not 0 <= nanoseconds < 1_000_000_000
    ):
        raise ValueError('goal status contains an invalid timestamp')
    result = seconds * 1_000_000_000 + nanoseconds
    if result <= 0:
        raise ValueError('executing goal status timestamp is not positive')
    return result


class ActiveGoalProbe(Node):
    """Retain the latest bounded action-status sample."""

    def __init__(self) -> None:
        super().__init__('phase4_active_goal_probe', namespace='/robotest/evidence')
        self.latest: GoalStatusArray | None = None
        self.subscription = self.create_subscription(
            GoalStatusArray,
            ACTION_NAME + '/_action/status',
            self._on_status,
            qos_profile_action_status_default,
        )

    def _on_status(self, message: GoalStatusArray) -> None:
        if len(message.status_list) > MAX_STATUS_ENTRIES:
            raise RuntimeError('action status list exceeds the 1,024-entry bound')
        self.latest = message


def mission_client_present(node: Node) -> tuple[bool, list[list[Any]]]:
    """Return exact action-client ownership and a deterministic raw snapshot."""
    entries = get_action_client_names_and_types_by_node(
        node, MISSION_NODE_NAME, MISSION_NODE_NAMESPACE
    )
    normalized = sorted([name, sorted(set(types))] for name, types in entries)
    matches = [
        entry for entry in normalized if entry[0] == ACTION_NAME and entry[1] == [ACTION_TYPE]
    ]
    return len(matches) == 1, normalized


def probe(mission_pid: int, wall_timeout_s: float) -> dict[str, Any]:
    """Observe one unambiguous executing goal from the live mission client."""
    if mission_pid <= 0 or not 1.0 <= wall_timeout_s <= 60.0:
        raise ValueError('mission PID or wall timeout is outside its bound')
    started = time.monotonic()
    deadline = started + wall_timeout_s
    rclpy.init(args=None)
    node: ActiveGoalProbe | None = None
    executor: SingleThreadedExecutor | None = None
    try:
        node = ActiveGoalProbe()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        attempts = 0
        while rclpy.ok() and time.monotonic() < deadline:
            attempts += 1
            if attempts > 2048:
                raise RuntimeError('active-goal probe exceeded its attempt bound')
            if not process_alive(mission_pid):
                raise RuntimeError('mission process exited before active-goal proof')
            executor.spin_once(timeout_sec=SPIN_QUANTUM_S)
            if node.latest is None:
                continue
            executing = [
                status
                for status in node.latest.status_list
                if int(status.status) == GoalStatus.STATUS_EXECUTING
            ]
            if not executing:
                continue
            if len(executing) != 1:
                raise RuntimeError(
                    f'expected one executing FollowWaypoints goal, found {len(executing)}'
                )
            try:
                client_present, clients = mission_client_present(node)
            except Exception:  # Graph discovery can change between status and query.
                continue
            if not client_present:
                continue
            status = executing[0]
            raw_uuid = bytes(status.goal_info.goal_id.uuid)
            if len(raw_uuid) != 16:
                raise RuntimeError('executing goal UUID does not contain 16 bytes')
            accepted_stamp_ns = stamp_ns(status.goal_info.stamp)
            observed_monotonic_ns = time.monotonic_ns()
            return {
                'schema_version': SCHEMA_VERSION,
                'verdict': 'PASS',
                'captured_utc': utc_now(),
                'observed_monotonic_ns': observed_monotonic_ns,
                'elapsed_wall_s': time.monotonic() - started,
                'mission_pid': mission_pid,
                'mission_process_alive': process_alive(mission_pid),
                'mission_node': '/robotest/mission_runner',
                'mission_runner_is_action_client': True,
                'action_clients': clients,
                'action_name': ACTION_NAME,
                'action_type': ACTION_TYPE,
                'goal_uuid': str(uuid.UUID(bytes=raw_uuid)),
                'accepted_goal_stamp_ns': accepted_stamp_ns,
                'goal_status_code': int(status.status),
                'goal_status': 'EXECUTING',
                'status_entry_count': len(node.latest.status_list),
                'attempt_count': attempts,
                'ros_domain_id': os.environ.get('ROS_DOMAIN_ID'),
                'gz_partition': os.environ.get('GZ_PARTITION'),
            }
        raise RuntimeError('timed out before an exact executing mission goal was observed')
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def build_parser() -> argparse.ArgumentParser:
    """Build the strict probe CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission-pid', type=int, required=True)
    parser.add_argument('--wall-timeout', type=float, default=30.0)
    parser.add_argument('--output', type=Path, required=True)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the probe and always retain a bounded PASS/FAIL artifact."""
    args = build_parser().parse_args(argv)
    try:
        result = probe(args.mission_pid, args.wall_timeout)
    except Exception as exc:
        result = {
            'schema_version': SCHEMA_VERSION,
            'verdict': 'FAIL',
            'captured_utc': utc_now(),
            'mission_pid': args.mission_pid,
            'mission_process_alive': process_alive(args.mission_pid),
            'failure': f'{type(exc).__name__}: {exc}',
        }
        atomic_write_json(args.output, result)
        print(f'phase4_active_goal_probe: {result["failure"]}', file=sys.stderr)
        return 1
    atomic_write_json(args.output, result)
    print(
        f'phase4_active_goal_probe: goal={result["goal_uuid"]} status={result["goal_status"]}',
        file=sys.stderr,
    )
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
