#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Bounded ROS observers needed by the Phase 3 orchestration lane.

The goal observer is armed only after the scenario controller is ready.  It
binds the first newly ACCEPTED FollowWaypoints UUID, preserves the immutable
action-status goal stamp as T0, and optionally emits the absolute Scenario-4
lifecycle schedule.  The clock observer proves the collector remained alive
through the required terminal + 0.25 s contact drain.
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import os
from pathlib import Path
import time
from typing import Any, ClassVar
import uuid

from phase3_orchestration import (
    atomic_write_json,
    canonical_sha256,
    EvidenceError,
    lifecycle_schedule,
    require_bounded_string,
)

MAX_STATUS_SAMPLES = 4096
STATUS_ACCEPTED = 1


def _stamp_ns(stamp: Any) -> int:
    sec = getattr(stamp, 'sec', None)
    nanosec = getattr(stamp, 'nanosec', None)
    if (
        isinstance(sec, bool)
        or not isinstance(sec, int)
        or isinstance(nanosec, bool)
        or not isinstance(nanosec, int)
        or sec < 0
        or not 0 <= nanosec < 1_000_000_000
    ):
        raise EvidenceError('action-status stamp is malformed')
    return sec * 1_000_000_000 + nanosec


def _uuid_hex(value: Any) -> str:
    values = getattr(value, 'uuid', None)
    if not isinstance(values, Sequence) or len(values) != 16:
        raise EvidenceError('action-status UUID must contain exactly 16 bytes')
    try:
        octets = bytes(int(item) for item in values)
    except (TypeError, ValueError, OverflowError) as exc:
        raise EvidenceError('action-status UUID bytes are malformed') from exc
    if not any(octets):
        raise EvidenceError('action-status UUID cannot be all zero')
    return str(uuid.UUID(bytes=octets))


def _pid_alive(pid: int | None) -> bool:
    if pid is None:
        return True
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


class GoalObserver:
    """Small state machine separated from the rclpy node for pure tests."""

    def __init__(self) -> None:
        self.armed = False
        self.bound_uuid: str | None = None
        self.bound_t0_ns: int | None = None
        self.ingress_count = 0
        self.prearm_uuids: set[str] = set()
        self.postarm_new_uuids: set[str] = set()
        self.status_trace: list[dict[str, Any]] = []

    def arm(self) -> None:
        self.armed = True

    def observe(self, *, uuid: str, status: int, stamp_ns: int, clock_ns: int) -> None:
        self.ingress_count += 1
        if self.ingress_count > MAX_STATUS_SAMPLES:
            raise EvidenceError('goal observer exceeded 4,096 status samples')
        if stamp_ns <= 0:
            raise EvidenceError('goal observer saw a non-positive goal stamp')
        if not self.armed:
            self.prearm_uuids.add(uuid)
            return
        if uuid in self.prearm_uuids:
            return
        self.postarm_new_uuids.add(uuid)
        if len(self.postarm_new_uuids) > 1:
            raise EvidenceError('more than one new goal UUID appeared after arm')
        if self.bound_uuid is not None:
            if uuid != self.bound_uuid or stamp_ns != self.bound_t0_ns:
                raise EvidenceError('bound goal UUID/T0 changed after acceptance')
            return
        if status != STATUS_ACCEPTED:
            return
        if clock_ns <= 0:
            return
        self.bound_uuid = uuid
        self.bound_t0_ns = stamp_ns
        self.status_trace.append(
            {
                'goal_uuid': uuid,
                'goal_stamp_ns': stamp_ns,
                'observed_clock_ns': clock_ns,
                'status': status,
            }
        )


def _run_goal_observer(arguments: argparse.Namespace) -> int:
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_action_status_default,
    )
    from rosgraph_msgs.msg import Clock

    class NodeImpl(Node):
        def __init__(self) -> None:
            super().__init__('phase3_goal_observer')
            self.machine = GoalObserver()
            self.clock_ns = 0
            self.clock_count = 0
            self.clock_regression_count = 0
            self.last_clock_ns: int | None = None
            self.status_message_count = 0
            clock_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
            self.create_subscription(
                GoalStatusArray,
                'follow_waypoints/_action/status',
                self.on_status,
                qos_profile_action_status_default,
            )

        def on_clock(self, message: Clock) -> None:
            value = _stamp_ns(message.clock)
            self.clock_count += 1
            if self.last_clock_ns is not None and value < self.last_clock_ns:
                self.clock_regression_count += 1
                raise EvidenceError('simulation clock regressed in goal observer')
            self.last_clock_ns = value
            self.clock_ns = value

        def on_status(self, message: GoalStatusArray) -> None:
            self.status_message_count += 1
            if self.status_message_count > MAX_STATUS_SAMPLES:
                raise EvidenceError('goal observer exceeded 4,096 status messages')
            for item in message.status_list:
                self.machine.observe(
                    uuid=_uuid_hex(item.goal_info.goal_id),
                    status=int(item.status),
                    stamp_ns=_stamp_ns(item.goal_info.stamp),
                    clock_ns=self.clock_ns,
                )

    for path in (arguments.ready_file, arguments.output):
        if path.exists() or path.is_symlink():
            raise EvidenceError(f'goal observer output already exists: {path}')
    if arguments.schedule_output is not None and (
        arguments.schedule_output.exists() or arguments.schedule_output.is_symlink()
    ):
        raise EvidenceError(
            f'lifecycle schedule output already exists: {arguments.schedule_output}'
        )
    rclpy.init(args=list(arguments.ros_args))
    node = NodeImpl()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + int(arguments.wall_timeout_s * 1_000_000_000)
    ready_written = False
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            if not _pid_alive(arguments.watch_pid):
                raise EvidenceError('watched process exited before goal observation completed')
            executor.spin_once(timeout_sec=0.05)
            if (
                not ready_written
                and node.clock_ns > 0
                and node.status_message_count > 0
                and node.count_publishers('follow_waypoints/_action/status') >= 1
            ):
                atomic_write_json(
                    arguments.ready_file,
                    {
                        'prearm_uuid_set_sha256': canonical_sha256(
                            sorted(node.machine.prearm_uuids)
                        ),
                        'producer': 'robotest_phase3/goal_observer',
                        'ready_clock_ns': node.clock_ns,
                        'schema_version': 1,
                    },
                )
                ready_written = True
            if ready_written and arguments.arm_file.is_file() and not node.machine.armed:
                node.machine.arm()
            if node.machine.bound_uuid is not None:
                assert node.machine.bound_t0_ns is not None
                result = {
                    'accepted_goal_stamp_ns': node.machine.bound_t0_ns,
                    'accepted_goal_uuid': node.machine.bound_uuid,
                    'clock': {
                        'latest_stamp_ns': node.clock_ns,
                        'message_count': node.clock_count,
                        'regression_count': node.clock_regression_count,
                    },
                    'ingress_count': node.machine.ingress_count,
                    'postarm_new_uuid_count': len(node.machine.postarm_new_uuids),
                    'prearm_uuid_set_sha256': canonical_sha256(sorted(node.machine.prearm_uuids)),
                    'producer': 'robotest_phase3/goal_observer',
                    'run_id': require_bounded_string(arguments.run_id, 'run_id'),
                    'schema_version': 1,
                    'status_message_count': node.status_message_count,
                    'status_trace': node.machine.status_trace,
                }
                atomic_write_json(arguments.output, result, sidecar=True)
                if arguments.schedule_output is not None:
                    atomic_write_json(
                        arguments.schedule_output,
                        lifecycle_schedule(arguments.run_id, node.machine.bound_t0_ns),
                        sidecar=True,
                    )
                return 0
        raise EvidenceError('goal observer wall timeout expired')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_clock_waiter(arguments: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock

    class NodeImpl(Node):
        def __init__(self) -> None:
            super().__init__('phase3_clock_drain_observer')
            self.first_ns: int | None = None
            self.latest_ns: int | None = None
            self.count = 0
            self.regressions = 0
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Clock, '/clock', self.on_clock, qos)

        def on_clock(self, message: Clock) -> None:
            value = _stamp_ns(message.clock)
            self.count += 1
            if self.latest_ns is not None and value < self.latest_ns:
                self.regressions += 1
                raise EvidenceError('simulation clock regressed during terminal drain')
            self.first_ns = value if self.first_ns is None else self.first_ns
            self.latest_ns = value

    if arguments.output.exists() or arguments.output.is_symlink():
        raise EvidenceError(f'clock observer output already exists: {arguments.output}')
    rclpy.init(args=list(arguments.ros_args))
    node = NodeImpl()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + int(arguments.wall_timeout_s * 1_000_000_000)
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            if not _pid_alive(arguments.watch_pid):
                raise EvidenceError('watched stack exited before terminal contact drain')
            executor.spin_once(timeout_sec=0.05)
            if node.latest_ns is not None and node.latest_ns >= arguments.target_stamp_ns:
                atomic_write_json(
                    arguments.output,
                    {
                        'first_stamp_ns': node.first_ns,
                        'latest_stamp_ns': node.latest_ns,
                        'message_count': node.count,
                        'producer': 'robotest_phase3/clock_drain_observer',
                        'regression_count': node.regressions,
                        'schema_version': 1,
                        'target_stamp_ns': arguments.target_stamp_ns,
                    },
                    sidecar=True,
                )
                return 0
        raise EvidenceError('clock drain wall timeout expired')
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _self_test() -> int:
    class Stamp:
        sec = 1
        nanosec = 5

    class UUID:
        uuid: ClassVar[list[int]] = list(range(1, 17))

    assert _stamp_ns(Stamp()) == 1_000_000_005
    assert _uuid_hex(UUID()) == '01020304-0506-0708-090a-0b0c0d0e0f10'
    observer = GoalObserver()
    observer.observe(uuid='old', status=1, stamp_ns=1, clock_ns=1)
    observer.arm()
    observer.observe(uuid='new', status=1, stamp_ns=2, clock_ns=2)
    assert observer.bound_uuid == 'new'
    assert observer.bound_t0_ns == 2
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest='command', required=True)
    goal = subparsers.add_parser('observe-goal')
    goal.add_argument('--run-id', required=True)
    goal.add_argument('--ready-file', required=True, type=Path)
    goal.add_argument('--arm-file', required=True, type=Path)
    goal.add_argument('--output', required=True, type=Path)
    goal.add_argument('--schedule-output', type=Path)
    goal.add_argument('--watch-pid', required=True, type=int)
    goal.add_argument('--wall-timeout-s', type=float, default=300.0)
    goal.set_defaults(function=_run_goal_observer)
    clock = subparsers.add_parser('wait-clock')
    clock.add_argument('--target-stamp-ns', required=True, type=int)
    clock.add_argument('--output', required=True, type=Path)
    clock.add_argument('--watch-pid', required=True, type=int)
    clock.add_argument('--wall-timeout-s', type=float, default=30.0)
    clock.set_defaults(function=_run_clock_waiter)
    test = subparsers.add_parser('self-test')
    test.set_defaults(function=lambda _arguments: _self_test())
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run one bounded runtime observation command."""
    arguments, ros_args = _parser().parse_known_args(argv)
    arguments.ros_args = ros_args
    try:
        wall_timeout_s = getattr(arguments, 'wall_timeout_s', None)
        if wall_timeout_s is not None and not 0.0 < wall_timeout_s <= 300.0:
            raise EvidenceError('wall timeout must be in (0, 300] seconds')
        return int(arguments.function(arguments))
    except (EvidenceError, RuntimeError) as exc:
        print(f'phase3 runtime observer error: {exc}', file=os.sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
