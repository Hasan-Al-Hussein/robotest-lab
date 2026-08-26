#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Bounded ROS observers needed by the Phase 3 orchestration lane.

The goal observer is armed only after the scenario controller is ready.  It
binds the first newly ACCEPTED FollowWaypoints UUID, preserves the immutable
action-status goal stamp as T0, and optionally emits the absolute Scenario-4
lifecycle schedule.  The contact-drain observer binds the first authoritative
public snapshot strictly beyond terminal + 0.25 s to a caught-up /clock sample.
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
CONTACT_TOPIC = '/robotest/validation/contacts'
CONTACT_GATE_NODE = '/robotest/contact_stream_gate'
CONTACT_RELEASE_GAP_NS = 250_000_000
CONTACT_HEARTBEAT_NS = 200_000_000
CONTACT_MAX_PUBLIC_GAP_NS = 220_000_000
CONTACT_MAX_CLOCK_LAG_NS = 220_000_000
CONTACT_MAX_RECORDS = 16


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
        raise EvidenceError('ROS timestamp is malformed')
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


def _valid_scoped_collision_name(value: Any) -> bool:
    if not isinstance(value, str) or not value or len(value.encode('utf-8')) > 4096:
        return False
    segments = value.split('::')
    return len(segments) >= 3 and all(segments)


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


class ContactDrainObserver:
    """Pure state machine for authoritative terminal contact-drain evidence."""

    def __init__(self, terminal_action_stamp_ns: int) -> None:
        if (
            isinstance(terminal_action_stamp_ns, bool)
            or not isinstance(terminal_action_stamp_ns, int)
            or terminal_action_stamp_ns <= 0
        ):
            raise EvidenceError('terminal action stamp must be a positive integer')
        self.terminal_action_stamp_ns = terminal_action_stamp_ns
        self.target_stamp_ns = terminal_action_stamp_ns + CONTACT_RELEASE_GAP_NS
        self.first_contact_stamp_ns: int | None = None
        self.latest_contact_stamp_ns: int | None = None
        self.contact_message_count = 0
        self.contact_record_count_violation_count = 0
        self.minimum_contact_record_count: int | None = None
        self.maximum_contact_record_count: int | None = None
        self.maximum_contact_source_gap_ns = 0
        self.contact_stamp_regression_count = 0
        self.contact_stamp_duplicate_count = 0
        self.minimum_same_pair_set_interval_ns: int | None = None
        self.same_pair_set_interval_violation_count = 0
        self.previous_pair_set: frozenset[tuple[str, str]] | None = None
        self.qualifying_contact_snapshot_stamp_ns: int | None = None
        self.clock_first_stamp_ns: int | None = None
        self.clock_latest_stamp_ns: int | None = None
        self.clock_message_count = 0
        self.clock_regression_count = 0

    def observe_contact(
        self,
        *,
        stamp_ns: int,
        frame_id: str,
        pair_set: frozenset[tuple[str, str]],
        record_count: int,
    ) -> None:
        if stamp_ns <= 0:
            raise EvidenceError('public contact snapshot stamp must be positive')
        if frame_id:
            raise EvidenceError('public contact snapshot frame_id must be empty')
        if not 1 <= record_count <= CONTACT_MAX_RECORDS:
            self.contact_record_count_violation_count += 1
            raise EvidenceError('public contact snapshot must contain 1 to 16 records')
        self.contact_message_count += 1
        self.minimum_contact_record_count = (
            record_count
            if self.minimum_contact_record_count is None
            else min(self.minimum_contact_record_count, record_count)
        )
        self.maximum_contact_record_count = (
            record_count
            if self.maximum_contact_record_count is None
            else max(self.maximum_contact_record_count, record_count)
        )
        previous = self.latest_contact_stamp_ns
        if previous is not None:
            delta_ns = stamp_ns - previous
            if delta_ns < 0:
                self.contact_stamp_regression_count += 1
                raise EvidenceError('public contact snapshot stamp regressed')
            if delta_ns == 0:
                self.contact_stamp_duplicate_count += 1
                raise EvidenceError('public contact snapshot stamp was duplicated')
            self.maximum_contact_source_gap_ns = max(self.maximum_contact_source_gap_ns, delta_ns)
            if delta_ns > CONTACT_MAX_PUBLIC_GAP_NS:
                raise EvidenceError('public contact snapshot source gap exceeded 220 ms')
            if pair_set == self.previous_pair_set:
                self.minimum_same_pair_set_interval_ns = (
                    delta_ns
                    if self.minimum_same_pair_set_interval_ns is None
                    else min(self.minimum_same_pair_set_interval_ns, delta_ns)
                )
                if delta_ns < CONTACT_HEARTBEAT_NS:
                    self.same_pair_set_interval_violation_count += 1
                    raise EvidenceError('unchanged public contact pair set repeated before 200 ms')
        if self.first_contact_stamp_ns is None:
            self.first_contact_stamp_ns = stamp_ns
        self.latest_contact_stamp_ns = stamp_ns
        self.previous_pair_set = pair_set
        if self.qualifying_contact_snapshot_stamp_ns is None and stamp_ns > self.target_stamp_ns:
            self.qualifying_contact_snapshot_stamp_ns = stamp_ns

    def observe_clock(self, stamp_ns: int) -> None:
        if stamp_ns < 0:
            raise EvidenceError('simulation clock stamp cannot be negative')
        self.clock_message_count += 1
        if self.clock_latest_stamp_ns is not None and stamp_ns < self.clock_latest_stamp_ns:
            self.clock_regression_count += 1
            raise EvidenceError('simulation clock regressed during terminal contact drain')
        if self.clock_first_stamp_ns is None:
            self.clock_first_stamp_ns = stamp_ns
        self.clock_latest_stamp_ns = stamp_ns

    def complete(self) -> bool:
        qualifying = self.qualifying_contact_snapshot_stamp_ns
        latest_clock = self.clock_latest_stamp_ns
        if qualifying is None or latest_clock is None or latest_clock < qualifying:
            return False
        if latest_clock - qualifying > CONTACT_MAX_CLOCK_LAG_NS:
            raise EvidenceError('simulation clock is more than 220 ms beyond drain snapshot')
        return True

    def evidence(self) -> dict[str, Any]:
        if not self.complete():
            raise EvidenceError('terminal contact drain is not complete')
        qualifying = self.qualifying_contact_snapshot_stamp_ns
        latest_clock = self.clock_latest_stamp_ns
        assert qualifying is not None and latest_clock is not None
        interval_violations = self.same_pair_set_interval_violation_count
        return {
            'clock_first_stamp_ns': self.clock_first_stamp_ns,
            'clock_latest_stamp_ns': latest_clock,
            'clock_message_count': self.clock_message_count,
            'clock_minus_qualifying_contact_ns': latest_clock - qualifying,
            'clock_regression_count': self.clock_regression_count,
            'contact_message_count': self.contact_message_count,
            'contact_record_count_violation_count': (self.contact_record_count_violation_count),
            'contact_stamp_duplicate_count': self.contact_stamp_duplicate_count,
            'contact_stamp_regression_count': self.contact_stamp_regression_count,
            'first_contact_stamp_ns': self.first_contact_stamp_ns,
            'latest_contact_stamp_ns': self.latest_contact_stamp_ns,
            'limits': {
                'heartbeat_period_ns': CONTACT_HEARTBEAT_NS,
                'max_clock_lag_ns': CONTACT_MAX_CLOCK_LAG_NS,
                'max_public_gap_ns': CONTACT_MAX_PUBLIC_GAP_NS,
                'release_gap_ns': CONTACT_RELEASE_GAP_NS,
            },
            'maximum_contact_record_count': self.maximum_contact_record_count,
            'maximum_contact_source_gap_ns': self.maximum_contact_source_gap_ns,
            'minimum_contact_record_count': self.minimum_contact_record_count,
            'minimum_same_pair_set_interval_ns': self.minimum_same_pair_set_interval_ns,
            'producer': 'robotest_phase3/contact_drain_observer',
            'public_topic': CONTACT_TOPIC,
            'qualifying_contact_snapshot_stamp_ns': qualifying,
            'same_pair_set_interval_violation_count': interval_violations,
            'schema_version': 3,
            'target_stamp_ns': self.target_stamp_ns,
            'terminal_action_stamp_ns': self.terminal_action_stamp_ns,
        }


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


def _run_contact_drain_observer(arguments: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from ros_gz_interfaces.msg import Contacts
    from rosgraph_msgs.msg import Clock

    class NodeImpl(Node):
        def __init__(self) -> None:
            super().__init__('phase3_contact_drain_observer')
            self.machine = ContactDrainObserver(arguments.terminal_action_stamp_ns)
            clock_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            contact_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
            self.create_subscription(Contacts, CONTACT_TOPIC, self.on_contacts, contact_qos)

        def on_clock(self, message: Clock) -> None:
            self.machine.observe_clock(_stamp_ns(message.clock))

        def on_contacts(self, message: Contacts) -> None:
            pairs: set[tuple[str, str]] = set()
            for contact in message.contacts:
                first = contact.collision1.name
                second = contact.collision2.name
                if not _valid_scoped_collision_name(first) or not _valid_scoped_collision_name(
                    second
                ):
                    raise EvidenceError('public contact snapshot contains a malformed scoped name')
                pairs.add(tuple(sorted((first, second))))
            self.machine.observe_contact(
                stamp_ns=_stamp_ns(message.header.stamp),
                frame_id=message.header.frame_id,
                pair_set=frozenset(pairs),
                record_count=len(message.contacts),
            )

    if arguments.output.exists() or arguments.output.is_symlink():
        raise EvidenceError(f'contact drain output already exists: {arguments.output}')
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
            if node.machine.complete():
                publisher_endpoints = node.get_publishers_info_by_topic(CONTACT_TOPIC)
                publisher_nodes = []
                for endpoint in publisher_endpoints:
                    namespace = endpoint.node_namespace.rstrip('/')
                    node_name = f'{namespace}/{endpoint.node_name}'.replace('//', '/')
                    publisher_nodes.append(node_name)
                publisher_nodes.sort()
                graph_nodes = {
                    f'{namespace.rstrip("/")}/{name}'.replace('//', '/')
                    for name, namespace in node.get_node_names_and_namespaces()
                }
                if (
                    len(publisher_endpoints) != 1
                    or publisher_nodes != [CONTACT_GATE_NODE]
                    or publisher_endpoints[0].topic_type != 'ros_gz_interfaces/msg/Contacts'
                    or CONTACT_GATE_NODE not in graph_nodes
                ):
                    raise EvidenceError(
                        'contact stream gate did not survive through terminal drain'
                    )
                evidence = node.machine.evidence()
                evidence.update(
                    {
                        'gate_node_present': True,
                        'public_publisher_nodes': publisher_nodes,
                    }
                )
                atomic_write_json(
                    arguments.output,
                    evidence,
                    sidecar=True,
                )
                return 0
        raise EvidenceError('contact drain wall timeout expired')
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
    drain = ContactDrainObserver(1_000_000_000)
    pair_a = frozenset({('a::link::collision', 'b::link::collision')})
    pair_b = frozenset({('a::link::collision', 'c::link::collision')})
    drain.observe_contact(stamp_ns=1_250_000_000, frame_id='', pair_set=pair_a, record_count=1)
    assert drain.qualifying_contact_snapshot_stamp_ns is None
    drain.observe_contact(stamp_ns=1_250_000_001, frame_id='', pair_set=pair_b, record_count=1)
    assert drain.qualifying_contact_snapshot_stamp_ns == 1_250_000_001
    drain.observe_clock(1_250_000_000)
    assert drain.complete() is False
    drain.observe_clock(1_470_000_001)
    assert drain.complete() is True
    assert drain.evidence()['clock_minus_qualifying_contact_ns'] == 220_000_000
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
    drain = subparsers.add_parser('wait-contact-drain')
    drain.add_argument('--terminal-action-stamp-ns', required=True, type=int)
    drain.add_argument('--output', required=True, type=Path)
    drain.add_argument('--watch-pid', required=True, type=int)
    drain.add_argument('--wall-timeout-s', type=float, default=30.0)
    drain.set_defaults(function=_run_contact_drain_observer)
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
