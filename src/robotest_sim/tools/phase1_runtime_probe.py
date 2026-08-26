#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Bounded Phase 1 graph, data-plane, TF, motion, and RTF probe."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import signal
import statistics
import struct
import time
from array import array
from collections import defaultdict, deque
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ros_gz_interfaces.msg import Contacts, WorldStatistics
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

from robotest_interfaces.msg import FaultEvent

PHASE1_GROUND_COLLISION = 'ground_plane::ground_link::ground_collision'
PHASE1_COLLISION_BY_ROLE = {
    'chassis': (
        'robotest::base_footprint::base_footprint_fixed_joint_lump__base_link_collision_collision'
    ),
    'left_wheel': (
        'robotest::left_wheel_link::'
        'left_wheel_link_fixed_joint_lump__left_wheel_collision_collision'
    ),
    'right_wheel': (
        'robotest::right_wheel_link::'
        'right_wheel_link_fixed_joint_lump__right_wheel_collision_collision'
    ),
    'front_caster': (
        'robotest::front_caster_link::'
        'front_caster_link_fixed_joint_lump__front_caster_collision_collision'
    ),
    'rear_caster': (
        'robotest::rear_caster_link::'
        'rear_caster_link_fixed_joint_lump__rear_caster_collision_collision'
    ),
    'lidar_body': 'robotest::lidar_link::lidar_link_collision_collision',
    'imu_body': 'robotest::imu_link::imu_link_collision_collision',
}
PHASE1_SUPPORT_ROLES = frozenset({'front_caster', 'left_wheel', 'rear_caster', 'right_wheel'})
PHASE1_ROBOT_COLLISIONS = frozenset(PHASE1_COLLISION_BY_ROLE.values())
PHASE1_SUPPORT_PAIRS = frozenset(
    (PHASE1_COLLISION_BY_ROLE[role], PHASE1_GROUND_COLLISION) for role in PHASE1_SUPPORT_ROLES
)
PHASE1_ALLOWED_CONTACT_DISPOSITIONS = frozenset({'allowlisted_support_contact', 'robot_internal'})
PHASE1_CONTACT_DISPOSITIONS = (
    'allowlisted_support_contact',
    'robot_internal',
    'invalid_collision_name',
    'unexpected_non_robot_pair',
    'unexpected_robot_environment_contact',
)
CONTACT_EVIDENCE_CAPACITY = 64
CONTACT_NAME_EVIDENCE_LIMIT = 256
CONTACT_PUBLIC_HEARTBEAT_NS = 200_000_000
CONTACT_PUBLIC_MAX_GAP_NS = 220_000_000
CONTACT_PUBLIC_MAX_CLOCK_LAG_NS = 220_000_000
CONTACT_CLOCK_BRACKET_WALL_TIMEOUT_S = 2.0
CONTACT_PUBLIC_MAX_RECORDS = 16


def classify_phase1_contact_pair(collision1: Any, collision2: Any) -> str:
    """Classify an exact pair against the frozen collision-coverage policy."""
    if not valid_scoped_collision_name(collision1):
        return 'invalid_collision_name'
    if not valid_scoped_collision_name(collision2):
        return 'invalid_collision_name'
    first_robot = collision1 in PHASE1_ROBOT_COLLISIONS
    second_robot = collision2 in PHASE1_ROBOT_COLLISIONS
    if first_robot and second_robot:
        # The frozen Phase 3 classifier excludes exact covered robot/robot
        # pairs as internal contacts rather than external collision events.
        return 'robot_internal'
    if not first_robot and not second_robot:
        return 'unexpected_non_robot_pair'
    robot_collision = collision1 if first_robot else collision2
    counterpart_collision = collision2 if first_robot else collision1
    if (robot_collision, counterpart_collision) in PHASE1_SUPPORT_PAIRS:
        return 'allowlisted_support_contact'
    return 'unexpected_robot_environment_contact'


def valid_scoped_collision_name(value: Any) -> bool:
    """Require at least model, link, and collision scoped-name segments."""
    if not isinstance(value, str) or not value:
        return False
    segments = value.split('::')
    return len(segments) >= 3 and all(segments)


def valid_public_contact_record_count(value: Any) -> bool:
    """Require the bounded, nonempty v3 public snapshot cardinality."""
    return (
        isinstance(value, int)
        and not isinstance(value, bool)
        and 1 <= value <= CONTACT_PUBLIC_MAX_RECORDS
    )


def contact_name_evidence(value: Any) -> str:
    """Bound one untrusted collision name before retaining it in evidence."""
    if not isinstance(value, str):
        return f'<non-string:{type(value).__qualname__}>'
    if len(value) <= CONTACT_NAME_EVIDENCE_LIMIT:
        return value
    return value[:CONTACT_NAME_EVIDENCE_LIMIT] + '...'


def phase1_contact_policy() -> dict[str, Any]:
    """Return the canonical exact-name policy embedded in runtime evidence."""
    return {
        'allowed_dispositions': sorted(PHASE1_ALLOWED_CONTACT_DISPOSITIONS),
        'ground_collision': PHASE1_GROUND_COLLISION,
        'robot_collisions': sorted(PHASE1_ROBOT_COLLISIONS),
        'support_pairs': [list(pair) for pair in sorted(PHASE1_SUPPORT_PAIRS)],
    }


def phase1_contact_policy_sha256() -> str:
    payload = json.dumps(
        phase1_contact_policy(),
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def qos(depth: int, reliable: bool) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=(ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT),
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_ns(message: Any) -> int:
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def time_ns(value: Any) -> int:
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def gid_hex(value: Any) -> str:
    try:
        return bytes(value).hex()
    except (TypeError, ValueError):
        return str(value)


def canonical_semantic_value(value: Any) -> Any:
    """Return a CDR-padding-independent representation of a ROS message value."""
    if hasattr(value, 'get_fields_and_field_types'):
        fields = value.get_fields_and_field_types()
        return {
            'message_type': f'{type(value).__module__}.{type(value).__qualname__}',
            'fields': [
                [field_name, canonical_semantic_value(getattr(value, field_name))]
                for field_name in fields
            ],
        }
    if isinstance(value, bool):
        return ['bool', value]
    if isinstance(value, int):
        return ['int', str(value)]
    if isinstance(value, float):
        if math.isnan(value):
            return ['float', 'nan']
        if math.isinf(value):
            return ['float', '+inf' if value > 0.0 else '-inf']
        # Python exposes ROS float32 and float64 fields as float. Packing the
        # decoded value preserves every available bit, including signed zero.
        return ['float', struct.pack('>d', value).hex()]
    if isinstance(value, str):
        return ['str', value]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return ['bytes', bytes(value).hex()]
    if isinstance(value, (list, tuple, array)):
        return ['sequence', [canonical_semantic_value(item) for item in value]]
    if hasattr(value, 'tolist'):
        # ROS fixed-size numeric arrays may be generated as NumPy ndarrays.
        return canonical_semantic_value(value.tolist())
    if value is None:
        return ['none']
    raise TypeError(f'unsupported semantic fingerprint value: {type(value).__qualname__}')


def semantic_fingerprint(message: Any) -> str:
    """Hash message fields, never raw CDR bytes with nondeterministic padding."""
    canonical = canonical_semantic_value(message)
    payload = json.dumps(
        canonical,
        ensure_ascii=False,
        separators=(',', ':'),
        sort_keys=True,
    ).encode('utf-8')
    return hashlib.sha256(payload).hexdigest()


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def windowed_rtf(
    samples: list[tuple[int, float, bool, float]], sample_span: int = 5
) -> list[float]:
    """Calculate RTF across non-overlapping, multi-sample callback windows."""
    values: list[float] = []
    for start_index in range(0, len(samples) - sample_span, sample_span):
        window = samples[start_index : start_index + sample_span + 1]
        if any(sample[2] for sample in window):
            continue
        delta_sim = (window[-1][0] - window[0][0]) / 1e9
        delta_wall = window[-1][1] - window[0][1]
        if delta_sim > 0.0 and delta_wall > 0.0:
            values.append(delta_sim / delta_wall)
    return values


@dataclass
class StampTracker:
    """Constant-memory timestamp ordering and staleness accumulator."""

    sample_count: int = 0
    first_stamp_ns: int | None = None
    previous_stamp_ns: int | None = None
    latest_stamp_ns: int | None = None
    regression_count: int = 0
    duplicate_count: int = 0
    maximum_forward_gap_ns: int = 0
    receipt_age_sample_count: int = 0
    maximum_receipt_age_ns: int = 0
    future_relative_to_clock_count: int = 0

    def observe(self, stamp: int, latest_clock: int | None) -> None:
        self.sample_count += 1
        if self.first_stamp_ns is None:
            self.first_stamp_ns = stamp
        if self.previous_stamp_ns is not None:
            delta = stamp - self.previous_stamp_ns
            if delta < 0:
                self.regression_count += 1
            elif delta == 0:
                self.duplicate_count += 1
            else:
                self.maximum_forward_gap_ns = max(self.maximum_forward_gap_ns, delta)
        self.previous_stamp_ns = stamp
        self.latest_stamp_ns = (
            stamp if self.latest_stamp_ns is None else max(self.latest_stamp_ns, stamp)
        )

        if latest_clock is not None:
            receipt_age = latest_clock - stamp
            if receipt_age < 0:
                self.future_relative_to_clock_count += 1
            else:
                self.receipt_age_sample_count += 1
                self.maximum_receipt_age_ns = max(
                    self.maximum_receipt_age_ns,
                    receipt_age,
                )

    def evidence(
        self,
        latest_clock: int | None,
        maximum_gap_s: float,
        maximum_final_age_s: float,
        maximum_receipt_age_s: float,
    ) -> dict[str, Any]:
        final_age_ns = None
        if latest_clock is not None and self.latest_stamp_ns is not None:
            final_age_ns = latest_clock - self.latest_stamp_ns
        return {
            'sample_count': self.sample_count,
            'first_stamp_ns': self.first_stamp_ns,
            'latest_stamp_ns': self.latest_stamp_ns,
            'regression_count': self.regression_count,
            'duplicate_count': self.duplicate_count,
            'maximum_forward_gap_ns': self.maximum_forward_gap_ns,
            'maximum_forward_gap_s': self.maximum_forward_gap_ns / 1e9,
            'maximum_receipt_age_ns': self.maximum_receipt_age_ns,
            'maximum_receipt_age_s': self.maximum_receipt_age_ns / 1e9,
            'receipt_age_sample_count': self.receipt_age_sample_count,
            'future_relative_to_clock_count': self.future_relative_to_clock_count,
            'final_age_s': None if final_age_ns is None else final_age_ns / 1e9,
            'final_age_ns': final_age_ns,
            'limits': {
                'maximum_gap_s': maximum_gap_s,
                'maximum_gap_ns': round(maximum_gap_s * 1e9),
                'maximum_final_age_s': maximum_final_age_s,
                'maximum_final_age_ns': round(maximum_final_age_s * 1e9),
                'maximum_receipt_age_s': maximum_receipt_age_s,
                'maximum_receipt_age_ns': round(maximum_receipt_age_s * 1e9),
            },
        }


def qos_policy_failures(
    topic: str,
    snapshot: dict[str, Any],
    expected_reliability: str,
    expected_durability: str,
) -> list[str]:
    """Check both live endpoint directions for the normative QoS policies."""
    failures = []
    for side in ('publishers', 'subscribers'):
        endpoints = snapshot[side]
        if not endpoints:
            failures.append(f'{topic} has no live {side}')
            continue
        for endpoint in endpoints:
            reliability = endpoint['qos']['reliability']
            durability = endpoint['qos']['durability']
            if reliability != expected_reliability:
                failures.append(
                    f'{topic} {side[:-1]} {endpoint["node"]} reliability '
                    f'{reliability} != {expected_reliability}'
                )
            if durability != expected_durability:
                failures.append(
                    f'{topic} {side[:-1]} {endpoint["node"]} durability '
                    f'{durability} != {expected_durability}'
                )
    return failures


def stamp_evidence_failures(topic: str, evidence: dict[str, Any]) -> list[str]:
    """Apply explicit Phase 1 ordering, gap, and staleness bounds."""
    failures = []
    limits = evidence['limits']
    if evidence['sample_count'] < 3:
        failures.append(f'{topic} has fewer than three timestamped samples')
    if evidence['regression_count']:
        failures.append(f'{topic} has {evidence["regression_count"]} timestamp regressions')
    if evidence['duplicate_count']:
        failures.append(f'{topic} has {evidence["duplicate_count"]} duplicate timestamps')
    if evidence['maximum_forward_gap_ns'] > limits['maximum_gap_ns']:
        failures.append(
            f'{topic} maximum stamp gap {evidence["maximum_forward_gap_s"]:.6f} s '
            f'exceeds {limits["maximum_gap_s"]:.6f} s'
        )
    if evidence['maximum_receipt_age_ns'] > limits['maximum_receipt_age_ns']:
        failures.append(
            f'{topic} maximum receipt age {evidence["maximum_receipt_age_s"]:.6f} s '
            f'exceeds {limits["maximum_receipt_age_s"]:.6f} s'
        )
    final_age = evidence['final_age_s']
    if evidence['final_age_ns'] is None:
        failures.append(f'{topic} final timestamp age is unavailable')
    elif evidence['final_age_ns'] > limits['maximum_final_age_ns']:
        failures.append(
            f'{topic} final stamp age {final_age:.6f} s '
            f'exceeds {limits["maximum_final_age_s"]:.6f} s'
        )
    return failures


class Phase1Probe(Node):
    """Collect deterministic, machine-readable evidence without unbounded waits."""

    def __init__(self) -> None:
        super().__init__('phase1_runtime_probe')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.started_wall = time.monotonic()
        self.stop_requested = False
        self.counts: dict[str, int] = defaultdict(int)
        self.first_wall: dict[str, float] = {}
        self.last_wall: dict[str, float] = {}
        self.sim_counts: dict[str, int] = defaultdict(int)
        self.first_sim_ns: dict[str, int] = {}
        self.last_sim_ns: dict[str, int] = {}
        self.stamp_trackers: dict[str, StampTracker] = defaultdict(StampTracker)
        self.latest_clock_ns: int | None = None
        self.latest_ground_truth: Odometry | None = None
        self.last_cmd: Twist | None = None
        self.nonempty_contact_messages = 0
        self.contact_frame_id_violation_count = 0
        self.contact_record_count_violation_count = 0
        self.contact_same_pair_set_interval_violation_count = 0
        self.contact_minimum_same_pair_set_interval_ns: int | None = None
        self.previous_contact_pair_set: frozenset[tuple[str, str]] | None = None
        self.previous_contact_pair_set_stamp_ns: int | None = None
        self.contact_clock_bracket: dict[str, Any] | None = None
        self.contact_record_count = 0
        self.contact_dispositions: dict[str, int] = defaultdict(int)
        self.unexpected_contact_pair_count = 0
        self.unexpected_contact_pair_prefix: list[dict[str, Any]] = []
        self.unexpected_contact_pair_omitted_count = 0
        self.world_samples: list[tuple[int, float, bool, float]] = []
        self.tf_edges: set[tuple[str, str]] = set()
        self.tf_static_edges: set[tuple[str, str]] = set()
        self.pair_fingerprints: dict[str, dict[str, dict[int, str]]] = {
            key: {'raw': {}, 'validated': {}} for key in ('scan', 'odom', 'imu')
        }
        self.semantic_matches: dict[str, int] = defaultdict(int)
        self.semantic_mismatches: dict[str, int] = defaultdict(int)
        self.motion_command_publications = 0
        self.motion_command_wall_times: list[float] = []
        self.motion_trace_active = False
        self.motion_trace_started_wall: float | None = None
        self.motion_trace_started_sim_ns: int | None = None
        self.motion_ground_truth_trace: deque[dict[str, Any]] = deque(maxlen=512)
        self.motion_command_trace: deque[dict[str, Any]] = deque(maxlen=256)
        self.motion_ground_truth_trace_dropped = 0
        self.motion_command_trace_dropped = 0
        self.failures: list[str] = []
        self._probe_subscriptions = []

        self._subscribe_pair('scan', 'raw', '/robotest/raw/scan', LaserScan, qos(5, False))
        self._subscribe_pair('scan', 'validated', '/robotest/scan', LaserScan, qos(5, False))
        self._subscribe_pair('odom', 'raw', '/robotest/raw/odom', Odometry, qos(10, False))
        self._subscribe_pair('odom', 'validated', '/robotest/odom', Odometry, qos(10, False))
        self._subscribe_pair('imu', 'raw', '/robotest/raw/imu', Imu, qos(10, False))
        self._subscribe_pair('imu', 'validated', '/robotest/imu', Imu, qos(10, False))
        self._probe_subscriptions.extend(
            [
                self.create_subscription(Clock, '/clock', self._on_clock, qos(1, False)),
                self.create_subscription(
                    Odometry,
                    '/robotest/validation/ground_truth',
                    self._on_ground_truth,
                    qos(10, True),
                ),
                self.create_subscription(
                    Contacts,
                    '/robotest/validation/contacts',
                    self._on_contacts,
                    qos(10, True),
                ),
                self.create_subscription(
                    WorldStatistics,
                    '/robotest/validation/world_stats',
                    self._on_world_stats,
                    qos(10, True),
                ),
                self.create_subscription(
                    FaultEvent,
                    '/robotest/faults/events',
                    self._on_fault_event,
                    qos(100, True),
                ),
                self.create_subscription(Twist, '/robotest/cmd_vel', self._on_cmd, qos(1, True)),
                self.create_subscription(TFMessage, '/tf', self._on_tf, qos(100, True)),
                self.create_subscription(
                    TFMessage,
                    '/tf_static',
                    self._on_tf_static,
                    QoSProfile(
                        history=HistoryPolicy.KEEP_LAST,
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    ),
                ),
            ]
        )
        self.command_publisher = self.create_publisher(Twist, '/robotest/cmd_vel', qos(1, True))

    def _record(self, key: str, simulation_stamp_ns: int | None = None) -> None:
        now = time.monotonic()
        self.counts[key] += 1
        self.first_wall.setdefault(key, now)
        self.last_wall[key] = now
        if simulation_stamp_ns is not None:
            self.sim_counts[key] += 1
            self.first_sim_ns.setdefault(key, simulation_stamp_ns)
            self.last_sim_ns[key] = simulation_stamp_ns
            self.stamp_trackers[key].observe(simulation_stamp_ns, self.latest_clock_ns)

    def _subscribe_pair(
        self, stream: str, plane: str, topic: str, message_type: Any, profile: QoSProfile
    ) -> None:
        def callback(message: Any) -> None:
            key = f'{plane}_{stream}'
            stamp = stamp_ns(message)
            self._record(key, stamp)
            own = self.pair_fingerprints[stream][plane]
            other_plane = 'validated' if plane == 'raw' else 'raw'
            other = self.pair_fingerprints[stream][other_plane]
            own[stamp] = semantic_fingerprint(message)
            if stamp in other:
                if own[stamp] == other[stamp]:
                    self.semantic_matches[stream] += 1
                else:
                    self.semantic_mismatches[stream] += 1
                own.pop(stamp, None)
                other.pop(stamp, None)
            while len(own) > 100:
                own.pop(next(iter(own)))

        self._probe_subscriptions.append(
            self.create_subscription(message_type, topic, callback, profile)
        )

    def _on_clock(self, message: Clock) -> None:
        self.latest_clock_ns = time_ns(message.clock)
        self._record('clock', self.latest_clock_ns)

    def _on_ground_truth(self, message: Odometry) -> None:
        self._record('ground_truth', stamp_ns(message))
        self.latest_ground_truth = message
        if self.motion_trace_active:
            self._append_motion_ground_truth(message)

    def _on_world_stats(self, message: WorldStatistics) -> None:
        simulation_stamp = time_ns(message.sim_time)
        self._record('world_stats', simulation_stamp)
        self.world_samples.append(
            (
                simulation_stamp,
                time.monotonic(),
                bool(message.paused),
                float(message.real_time_factor),
            )
        )
        if len(self.world_samples) > 5000:
            self.world_samples = self.world_samples[-5000:]

    def _on_cmd(self, message: Twist) -> None:
        self._record('cmd_vel')
        self.last_cmd = message

    def _on_contacts(self, message: Contacts) -> None:
        simulation_stamp = stamp_ns(message)
        self._record('contacts', simulation_stamp)
        if message.header.frame_id:
            self.contact_frame_id_violation_count += 1
        if not valid_public_contact_record_count(len(message.contacts)):
            self.contact_record_count_violation_count += 1
        if message.contacts:
            self.nonempty_contact_messages += 1
        pair_set: set[tuple[str, str]] = set()
        for contact_index, contact in enumerate(message.contacts):
            collision1 = getattr(getattr(contact, 'collision1', None), 'name', None)
            collision2 = getattr(getattr(contact, 'collision2', None), 'name', None)
            if isinstance(collision1, str) and isinstance(collision2, str):
                pair_set.add(tuple(sorted((collision1, collision2))))
            disposition = classify_phase1_contact_pair(collision1, collision2)
            self.contact_record_count += 1
            self.contact_dispositions[disposition] += 1
            if disposition in PHASE1_ALLOWED_CONTACT_DISPOSITIONS:
                continue
            self.unexpected_contact_pair_count += 1
            if len(self.unexpected_contact_pair_prefix) < CONTACT_EVIDENCE_CAPACITY:
                self.unexpected_contact_pair_prefix.append(
                    {
                        'collision1': contact_name_evidence(collision1),
                        'collision2': contact_name_evidence(collision2),
                        'contact_index': contact_index,
                        'disposition': disposition,
                        'message_sequence': self.counts['contacts'],
                    }
                )
            else:
                self.unexpected_contact_pair_omitted_count += 1
        frozen_pair_set = frozenset(pair_set)
        if (
            self.previous_contact_pair_set is not None
            and self.previous_contact_pair_set_stamp_ns is not None
            and frozen_pair_set == self.previous_contact_pair_set
            and simulation_stamp > self.previous_contact_pair_set_stamp_ns
        ):
            interval_ns = simulation_stamp - self.previous_contact_pair_set_stamp_ns
            self.contact_minimum_same_pair_set_interval_ns = (
                interval_ns
                if self.contact_minimum_same_pair_set_interval_ns is None
                else min(self.contact_minimum_same_pair_set_interval_ns, interval_ns)
            )
            if interval_ns < CONTACT_PUBLIC_HEARTBEAT_NS:
                self.contact_same_pair_set_interval_violation_count += 1
        self.previous_contact_pair_set = frozen_pair_set
        self.previous_contact_pair_set_stamp_ns = simulation_stamp

    def _on_fault_event(self, message: FaultEvent) -> None:
        self._record('fault_events', stamp_ns(message))

    def _append_motion_ground_truth(self, message: Odometry) -> None:
        if len(self.motion_ground_truth_trace) == self.motion_ground_truth_trace.maxlen:
            self.motion_ground_truth_trace_dropped += 1
        self.motion_ground_truth_trace.append(
            {
                'stamp_ns': stamp_ns(message),
                'x_m': float(message.pose.pose.position.x),
                'y_m': float(message.pose.pose.position.y),
            }
        )

    def _append_motion_command(self, message: Twist, phase: str) -> None:
        if self.motion_trace_started_wall is None:
            return
        if len(self.motion_command_trace) == self.motion_command_trace.maxlen:
            self.motion_command_trace_dropped += 1
        self.motion_command_trace.append(
            {
                'wall_offset_s': time.monotonic() - self.motion_trace_started_wall,
                'simulation_stamp_ns': self.latest_clock_ns,
                'phase': phase,
                'linear_x': float(message.linear.x),
                'angular_z': float(message.angular.z),
            }
        )

    def _record_tf(self, target: set[tuple[str, str]], message: TFMessage) -> None:
        for transform in message.transforms:
            target.add(
                (transform.header.frame_id.lstrip('/'), transform.child_frame_id.lstrip('/'))
            )

    def _on_tf(self, message: TFMessage, _info: Any) -> None:
        self._record('tf')
        self._record_tf(self.tf_edges, message)

    def _on_tf_static(self, message: TFMessage, _info: Any) -> None:
        self._record('tf_static')
        self._record_tf(self.tf_static_edges, message)

    def spin_for(self, wall_seconds: float) -> None:
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_contact_clock_bracket(self) -> None:
        """Bind the latest public snapshot to a caught-up clock sample."""
        deadline = time.monotonic() + CONTACT_CLOCK_BRACKET_WALL_TIMEOUT_S
        while time.monotonic() < deadline and not self.stop_requested:
            latest_contact = self.stamp_trackers['contacts'].latest_stamp_ns
            latest_clock = self.latest_clock_ns
            if (
                latest_contact is not None
                and latest_clock is not None
                and latest_clock >= latest_contact
            ):
                lag_ns = latest_clock - latest_contact
                if lag_ns <= CONTACT_PUBLIC_MAX_CLOCK_LAG_NS:
                    self.contact_clock_bracket = {
                        'clock_stamp_ns': latest_clock,
                        'clock_minus_public_latest_ns': lag_ns,
                        'limits': {
                            'maximum_clock_lag_ns': CONTACT_PUBLIC_MAX_CLOCK_LAG_NS,
                            'maximum_public_gap_ns': CONTACT_PUBLIC_MAX_GAP_NS,
                        },
                        'public_latest_stamp_ns': latest_contact,
                    }
                    return
            rclpy.spin_once(self, timeout_sec=0.02)
        latest_contact = self.stamp_trackers['contacts'].latest_stamp_ns
        latest_clock = self.latest_clock_ns
        lag_ns = (
            latest_clock - latest_contact
            if latest_clock is not None and latest_contact is not None
            else None
        )
        self.contact_clock_bracket = {
            'clock_stamp_ns': latest_clock,
            'clock_minus_public_latest_ns': lag_ns,
            'limits': {
                'maximum_clock_lag_ns': CONTACT_PUBLIC_MAX_CLOCK_LAG_NS,
                'maximum_public_gap_ns': CONTACT_PUBLIC_MAX_GAP_NS,
            },
            'public_latest_stamp_ns': latest_contact,
        }
        self.failures.append(
            'contact public snapshot did not obtain a caught-up /clock bracket within '
            f'{CONTACT_CLOCK_BRACKET_WALL_TIMEOUT_S:.1f} s'
        )

    def publish_zero(self) -> None:
        zero = Twist()
        for _ in range(10):
            self.command_publisher.publish(zero)
            if self.motion_trace_active:
                self._append_motion_command(zero, 'final_zero')
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_test(self) -> float | None:
        if self.latest_ground_truth is None or self.latest_clock_ns is None:
            self.failures.append('ground truth or /clock absent before motion test')
            return None
        before = (
            self.latest_ground_truth.pose.pose.position.x,
            self.latest_ground_truth.pose.pose.position.y,
        )
        start_sim = self.latest_clock_ns
        self.motion_trace_started_wall = time.monotonic()
        self.motion_trace_started_sim_ns = start_sim
        self.motion_command_publications = 0
        self.motion_command_wall_times.clear()
        self.motion_ground_truth_trace.clear()
        self.motion_command_trace.clear()
        self.motion_ground_truth_trace_dropped = 0
        self.motion_command_trace_dropped = 0
        self.motion_trace_active = True
        self._append_motion_ground_truth(self.latest_ground_truth)
        deadline = time.monotonic() + 12.0
        command = Twist()
        command.linear.x = 0.15
        command_period_wall = 0.1
        next_command_wall = time.monotonic()
        try:
            while time.monotonic() < deadline and not self.stop_requested:
                if (
                    self.latest_clock_ns is not None
                    and self.latest_clock_ns - start_sim >= 2_000_000_000
                ):
                    break
                now = time.monotonic()
                if now >= next_command_wall:
                    self.command_publisher.publish(command)
                    self.motion_command_publications += 1
                    self.motion_command_wall_times.append(now)
                    self._append_motion_command(command, 'motion')
                    next_command_wall = now + command_period_wall
                # Process callbacks continuously during motion.  Adding an
                # unconditional sleep here caps the single-threaded executor's
                # callback throughput and can make reliable evidence streams
                # stale even while the simulator itself remains healthy.
                rclpy.spin_once(self, timeout_sec=0.01)
            else:
                self.failures.append('two-second simulated motion interval timed out')
        finally:
            self.publish_zero()
        self.spin_for(1.0)
        self.motion_trace_active = False
        if self.latest_ground_truth is None:
            self.failures.append('ground truth absent after motion test')
            return None
        after = (
            self.latest_ground_truth.pose.pose.position.x,
            self.latest_ground_truth.pose.pose.position.y,
        )
        displacement = math.hypot(after[0] - before[0], after[1] - before[1])
        if not 0.05 <= displacement <= 0.75:
            self.failures.append(
                f'bounded motion displacement {displacement:.4f} m outside [0.05, 0.75]'
            )
        return displacement

    def endpoint_snapshot(self, topic: str) -> dict[str, Any]:
        def encode(endpoint: Any) -> dict[str, Any]:
            profile = endpoint.qos_profile
            return {
                'node': f'{endpoint.node_namespace.rstrip("/")}/{endpoint.node_name}'.replace(
                    '//', '/'
                ),
                'gid': gid_hex(endpoint.endpoint_gid),
                'qos': {
                    'history': profile.history.name,
                    'depth': int(profile.depth),
                    'reliability': profile.reliability.name,
                    'durability': profile.durability.name,
                },
            }

        return {
            'publishers': [encode(item) for item in self.get_publishers_info_by_topic(topic)],
            'subscribers': [encode(item) for item in self.get_subscriptions_info_by_topic(topic)],
        }

    @staticmethod
    def qos_introspection_status(snapshot: dict[str, Any]) -> dict[str, Any]:
        """State what the live graph can and cannot prove about bounded history."""
        incomplete: list[dict[str, Any]] = []
        endpoint_count = 0
        bounded_keep_last = True
        for side in ('publishers', 'subscribers'):
            for endpoint in snapshot[side]:
                endpoint_count += 1
                reasons = []
                if endpoint['qos']['history'] in {'UNKNOWN', 'SYSTEM_DEFAULT'}:
                    reasons.append(f'history={endpoint["qos"]["history"]}')
                if endpoint['qos']['depth'] <= 0:
                    reasons.append(f'depth={endpoint["qos"]["depth"]}')
                if endpoint['qos']['history'] != 'KEEP_LAST' or endpoint['qos']['depth'] <= 0:
                    bounded_keep_last = False
                if reasons:
                    incomplete.append(
                        {
                            'side': side,
                            'node': endpoint['node'],
                            'gid': endpoint['gid'],
                            'reasons': reasons,
                        }
                    )
        complete = endpoint_count > 0 and not incomplete
        bounded_depth_live_proven = complete and bounded_keep_last
        return {
            'complete': complete,
            'bounded_depth_live_proven': bounded_depth_live_proven,
            'endpoint_count': endpoint_count,
            'incomplete_endpoints': incomplete,
            'note': (
                'UNKNOWN/SYSTEM_DEFAULT history or non-positive depth is reported as '
                'incomplete live introspection; bounded KEEP_LAST depth remains covered '
                'by source and structural tests.'
            ),
        }

    def wall_rate(self, key: str) -> float | None:
        first = self.first_wall.get(key)
        last = self.last_wall.get(key)
        count = self.counts.get(key, 0)
        if first is None or last is None or last <= first or count <= 1:
            return None
        return (count - 1) / (last - first)

    def simulation_rate(self, key: str) -> float | None:
        first = self.first_sim_ns.get(key)
        last = self.last_sim_ns.get(key)
        count = self.sim_counts.get(key, 0)
        if first is None or last is None or last <= first or count <= 1:
            return None
        return (count - 1) / ((last - first) / 1e9)

    def evaluate(self, displacement: float | None) -> dict[str, Any]:
        topics = {
            '/clock': (
                'clock',
                20.0,
                'wall',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            # Sensor minimums retain the original tolerance, but are measured
            # against header/simulation stamps so low RTF cannot create a false
            # rate failure (configured: scan 5, odom 20, IMU 50 Hz).
            '/robotest/raw/scan': (
                'raw_scan',
                4.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/scan': (
                'validated_scan',
                4.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/fault_proxy',
            ),
            '/robotest/raw/odom': (
                'raw_odom',
                10.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/odom': (
                'validated_odom',
                10.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/fault_proxy',
            ),
            '/robotest/raw/imu': (
                'raw_imu',
                20.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/imu': (
                'validated_imu',
                20.0,
                'simulation',
                'BEST_EFFORT',
                'VOLATILE',
                '/robotest/fault_proxy',
            ),
            '/robotest/validation/ground_truth': (
                'ground_truth',
                10.0,
                'wall',
                'RELIABLE',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/validation/contacts': (
                'contacts',
                4.0,
                'simulation',
                'RELIABLE',
                'VOLATILE',
                '/robotest/contact_stream_gate',
            ),
            '/robotest/internal/raw_contacts': (
                'private_raw_contacts',
                None,
                'none',
                'RELIABLE',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/validation/world_stats': (
                'world_stats',
                9.5,
                'wall',
                'RELIABLE',
                'VOLATILE',
                '/robotest/parameter_bridge',
            ),
            '/robotest/cmd_vel': (
                'cmd_vel',
                1.0,
                'wall',
                'RELIABLE',
                'VOLATILE',
                '/phase1_runtime_probe',
            ),
            '/robotest/faults/events': (
                'fault_events',
                None,
                'none',
                'RELIABLE',
                'VOLATILE',
                '/robotest/fault_proxy',
            ),
        }
        endpoint_evidence: dict[str, Any] = {}
        qos_introspection: dict[str, Any] = {}
        wall_rates: dict[str, float | None] = {}
        simulation_rates: dict[str, float | None] = {}
        rate_gates: dict[str, Any] = {}
        qos_contract_expectations: dict[str, Any] = {}
        for topic, (
            key,
            minimum_rate,
            rate_basis,
            expected_reliability,
            expected_durability,
            expected_owner,
        ) in topics.items():
            wall_rate = self.wall_rate(key)
            simulation_rate = self.simulation_rate(key)
            wall_rates[topic] = wall_rate
            simulation_rates[topic] = simulation_rate
            measured_rate = simulation_rate if rate_basis == 'simulation' else wall_rate
            rate_gates[topic] = {
                'basis': rate_basis,
                'minimum_hz': minimum_rate,
                'measured_hz': measured_rate,
            }
            if minimum_rate is not None and (measured_rate is None or measured_rate < minimum_rate):
                self.failures.append(
                    f'{topic} {rate_basis}-time rate {measured_rate} below {minimum_rate} Hz'
                )
            snapshot = self.endpoint_snapshot(topic)
            endpoint_evidence[topic] = snapshot
            qos_introspection[topic] = self.qos_introspection_status(snapshot)
            qos_contract_expectations[topic] = {
                'publishers_and_subscribers': {
                    'reliability': expected_reliability,
                    'durability': expected_durability,
                }
            }
            self.failures.extend(
                qos_policy_failures(
                    topic,
                    snapshot,
                    expected_reliability,
                    expected_durability,
                )
            )
            publishers = snapshot['publishers']
            if len(publishers) != 1:
                self.failures.append(
                    f'{topic} has {len(publishers)} publishers, expected exactly one'
                )
            elif publishers[0]['node'] != expected_owner:
                self.failures.append(
                    f'{topic} publisher {publishers[0]["node"]} != {expected_owner}'
                )
            for side in ('publishers', 'subscribers'):
                for endpoint in snapshot[side]:
                    if endpoint['qos']['history'] == 'KEEP_ALL':
                        self.failures.append(f'{topic} has prohibited KEEP_ALL endpoint')

        for stream in ('scan', 'odom', 'imu'):
            if self.semantic_matches[stream] < 3:
                self.failures.append(
                    f'{stream} pass-through has only '
                    f'{self.semantic_matches[stream]} semantic matches'
                )
            if self.semantic_mismatches[stream] != 0:
                self.failures.append(
                    f'{stream} pass-through has '
                    f'{self.semantic_mismatches[stream]} semantic mismatches'
                )

        if self.unexpected_contact_pair_count != 0:
            self.failures.append(
                f'observed {self.unexpected_contact_pair_count} unexpected contact records'
            )
        if self.contact_frame_id_violation_count:
            self.failures.append('public contact snapshot frame_id must be empty')
        if self.contact_record_count_violation_count:
            self.failures.append('public contact snapshots must contain 1 to 16 records')
        if self.contact_same_pair_set_interval_violation_count:
            self.failures.append(
                'public contact snapshot repeated an unchanged pair set before the 200 ms '
                'heartbeat interval'
            )

        raw_contact_subscribers = endpoint_evidence['/robotest/internal/raw_contacts'][
            'subscribers'
        ]
        raw_contact_subscriber_nodes = [endpoint['node'] for endpoint in raw_contact_subscribers]
        if len(raw_contact_subscribers) != 1 or raw_contact_subscriber_nodes != [
            '/robotest/contact_stream_gate'
        ]:
            self.failures.append(
                '/robotest/internal/raw_contacts subscribers '
                f'{sorted(raw_contact_subscriber_nodes)} != exactly one '
                "'/robotest/contact_stream_gate' endpoint"
            )

        validation_topics = [
            '/robotest/validation/ground_truth',
            '/robotest/validation/contacts',
            '/robotest/validation/world_stats',
        ]
        for topic in validation_topics:
            for endpoint in endpoint_evidence[topic]['subscribers']:
                if endpoint['node'] != '/phase1_runtime_probe':
                    self.failures.append(
                        f'unauthorized validation subscriber {endpoint["node"]} on {topic}'
                    )

        tf_expected_publishers = {
            '/robotest/fault_proxy',
            '/robotest/robot_state_publisher',
        }
        tf_static_expected_publishers = {'/robotest/robot_state_publisher'}
        for topic, expected_publishers, expected_reliability, expected_durability in (
            ('/tf', tf_expected_publishers, 'RELIABLE', 'VOLATILE'),
            (
                '/tf_static',
                tf_static_expected_publishers,
                'RELIABLE',
                'TRANSIENT_LOCAL',
            ),
        ):
            snapshot = self.endpoint_snapshot(topic)
            endpoint_evidence[topic] = snapshot
            qos_introspection[topic] = self.qos_introspection_status(snapshot)
            qos_contract_expectations[topic] = {
                'publishers_and_subscribers': {
                    'reliability': expected_reliability,
                    'durability': expected_durability,
                }
            }
            self.failures.extend(
                qos_policy_failures(
                    topic,
                    snapshot,
                    expected_reliability,
                    expected_durability,
                )
            )
            publishers = snapshot['publishers']
            actual_publishers = [endpoint['node'] for endpoint in publishers]
            if (
                len(actual_publishers) != len(expected_publishers)
                or set(actual_publishers) != expected_publishers
            ):
                self.failures.append(
                    f'{topic} publisher endpoints {sorted(actual_publishers)} '
                    f'!= {sorted(expected_publishers)}'
                )
            for side in ('publishers', 'subscribers'):
                for endpoint in snapshot[side]:
                    if endpoint['qos']['history'] == 'KEEP_ALL':
                        self.failures.append(f'{topic} has prohibited KEEP_ALL endpoint')

        odom_edge = ('odom', 'base_footprint')
        if odom_edge not in self.tf_edges:
            self.failures.append('missing dynamic TF edge odom->base_footprint')
        required_static = {
            ('base_footprint', 'base_link'),
            ('base_link', 'lidar_link'),
            ('base_link', 'imu_link'),
        }
        missing_static = sorted(required_static - self.tf_static_edges)
        if missing_static:
            self.failures.append(f'missing static TF edges: {missing_static}')

        stamp_contracts = {
            '/robotest/raw/scan': ('raw_scan', 0.50, 0.60, 0.75),
            '/robotest/scan': ('validated_scan', 0.50, 0.60, 0.75),
            '/robotest/raw/odom': ('raw_odom', 0.25, 0.35, 0.50),
            '/robotest/odom': ('validated_odom', 0.25, 0.35, 0.50),
            '/robotest/raw/imu': ('raw_imu', 0.15, 0.25, 0.50),
            '/robotest/imu': ('validated_imu', 0.15, 0.25, 0.50),
            '/robotest/validation/ground_truth': ('ground_truth', 0.25, 0.35, 0.50),
            '/robotest/validation/contacts': ('contacts', 0.22, 0.22, 0.22),
        }
        stamp_evidence: dict[str, Any] = {}
        for topic, (
            key,
            maximum_gap_s,
            maximum_final_age_s,
            maximum_receipt_age_s,
        ) in stamp_contracts.items():
            evidence = self.stamp_trackers[key].evidence(
                self.latest_clock_ns,
                maximum_gap_s,
                maximum_final_age_s,
                maximum_receipt_age_s,
            )
            stamp_evidence[topic] = evidence
            self.failures.extend(stamp_evidence_failures(topic, evidence))
        stamp_evidence['/robotest/validation/contacts'].update(
            {
                'clock_bracket': self.contact_clock_bracket,
                'frame_id_violation_count': self.contact_frame_id_violation_count,
                'record_count_violation_count': (self.contact_record_count_violation_count),
                'minimum_same_pair_set_interval_ns': (
                    self.contact_minimum_same_pair_set_interval_ns
                ),
                'same_pair_set_interval_violation_count': (
                    self.contact_same_pair_set_interval_violation_count
                ),
            }
        )

        rtf_sample_span = 5
        rtf_values = windowed_rtf(self.world_samples, rtf_sample_span)
        rtf_median = statistics.median(rtf_values) if rtf_values else None
        rtf_p5 = percentile(rtf_values, 0.05)
        native_rtf_values = [
            sample[3]
            for sample in self.world_samples
            if not sample[2] and math.isfinite(sample[3]) and sample[3] >= 0.0
        ]
        native_rtf_median = statistics.median(native_rtf_values) if native_rtf_values else None
        native_rtf_p5 = percentile(native_rtf_values, 0.05)
        if rtf_median is None or rtf_median < 0.80:
            self.failures.append(f'median calculated RTF {rtf_median} below 0.80')
        if rtf_p5 is None or rtf_p5 < 0.50:
            self.failures.append(f'p5 calculated RTF {rtf_p5} below 0.50')

        if (
            self.last_cmd is None
            or abs(self.last_cmd.linear.x) > 0.02
            or abs(self.last_cmd.angular.z) > 0.05
        ):
            self.failures.append('final observed cmd_vel is not within zero-command tolerance')

        motion_command_rate = None
        motion_command_min_interval = None
        if len(self.motion_command_wall_times) > 1:
            motion_intervals = [
                current - previous
                for previous, current in zip(
                    self.motion_command_wall_times,
                    self.motion_command_wall_times[1:],
                    strict=False,
                )
            ]
            motion_command_rate = (len(self.motion_command_wall_times) - 1) / (
                self.motion_command_wall_times[-1] - self.motion_command_wall_times[0]
            )
            motion_command_min_interval = min(motion_intervals)
            if motion_command_rate > 10.1 or motion_command_min_interval < 0.095:
                self.failures.append('motion cmd_vel cadence exceeded the 10 Hz steady-wall limit')

        motion_phases = {sample['phase'] for sample in self.motion_command_trace}
        if len(self.motion_ground_truth_trace) < 3:
            self.failures.append('motion trace has fewer than three ground-truth samples')
        if not {'motion', 'final_zero'} <= motion_phases:
            self.failures.append('motion trace does not contain motion and final-zero commands')
        if self.motion_ground_truth_trace_dropped or self.motion_command_trace_dropped:
            self.failures.append('motion trace exceeded its bounded evidence capacity')

        return {
            'verdict': 'PASS' if not self.failures else 'FAIL',
            'failures': sorted(set(self.failures)),
            'counts': dict(self.counts),
            'rates_hz_wall': wall_rates,
            'rates_hz_simulation': simulation_rates,
            'rate_gates': rate_gates,
            'pass_through_semantic_matches': dict(self.semantic_matches),
            'pass_through_semantic_mismatches': dict(self.semantic_mismatches),
            'nonempty_contact_messages': self.nonempty_contact_messages,
            'contact_classification': {
                'total_pair_count': self.contact_record_count,
                'excluded_support_pair_count': self.contact_dispositions.get(
                    'allowlisted_support_contact', 0
                ),
                'excluded_internal_pair_count': self.contact_dispositions.get('robot_internal', 0),
                'counted_collision_pair_count': self.contact_dispositions.get(
                    'unexpected_robot_environment_contact', 0
                ),
                'dispositions': {
                    disposition: self.contact_dispositions.get(disposition, 0)
                    for disposition in PHASE1_CONTACT_DISPOSITIONS
                },
                'unexpected_pair_count': self.unexpected_contact_pair_count,
                'unexpected_pair_evidence_capacity': CONTACT_EVIDENCE_CAPACITY,
                'unexpected_pair_omitted_count': self.unexpected_contact_pair_omitted_count,
                'unexpected_pair_prefix': self.unexpected_contact_pair_prefix,
                'exact_support_allowlist': [list(pair) for pair in sorted(PHASE1_SUPPORT_PAIRS)],
                'policy_projection': phase1_contact_policy(),
                'policy_sha256': phase1_contact_policy_sha256(),
            },
            'motion_command': {
                'publications': self.motion_command_publications,
                'wall_rate_hz': motion_command_rate,
                'minimum_wall_interval_s': motion_command_min_interval,
                'target_wall_rate_hz': 10.0,
            },
            'motion_trace': {
                'started_simulation_stamp_ns': self.motion_trace_started_sim_ns,
                'ended_simulation_stamp_ns': self.latest_clock_ns,
                'ground_truth_capacity': self.motion_ground_truth_trace.maxlen,
                'ground_truth_dropped': self.motion_ground_truth_trace_dropped,
                'ground_truth': list(self.motion_ground_truth_trace),
                'command_capacity': self.motion_command_trace.maxlen,
                'command_dropped': self.motion_command_trace_dropped,
                'commands': list(self.motion_command_trace),
            },
            'displacement_m': displacement,
            'sensor_stamp_evidence': stamp_evidence,
            'rtf': {
                'method': 'non-overlapping world-stats windows',
                'source_sample_count': len(self.world_samples),
                'sample_span_per_window': rtf_sample_span,
                'window_count': len(rtf_values),
                'median': rtf_median,
                'p5': rtf_p5,
                'gazebo_reported_secondary': {
                    'sample_count': len(native_rtf_values),
                    'median': native_rtf_median,
                    'p5': native_rtf_p5,
                },
            },
            'tf_edges_observed': sorted([list(edge) for edge in self.tf_edges]),
            'tf_static_edges_observed': sorted([list(edge) for edge in self.tf_static_edges]),
            'tf_edge_source_attribution': (
                'Jazzy rclpy message callbacks do not expose publisher GID; exact live '
                'endpoint node sets prove the allowed TF publishers and observed messages '
                'prove required edge presence, but each edge cannot be correlated to one '
                'endpoint from callback metadata.'
            ),
            'endpoints': endpoint_evidence,
            'qos_contract_expectations': qos_contract_expectations,
            'qos_introspection': qos_introspection,
            'bounded_depth_live_proven_for_all_endpoints': all(
                status['bounded_depth_live_proven'] for status in qos_introspection.values()
            ),
            'bounded_depth_static_proof': (
                'bridge.yaml and structural/source tests reject KEEP_ALL and require '
                'positive bounded queues even when Jazzy graph introspection reports '
                'UNKNOWN history or depth 0.'
            ),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--warmup-wall-seconds', type=float, default=15.0)
    args = parser.parse_args()
    rclpy.init()
    probe = Phase1Probe()

    def request_stop(_signum: int, _frame: Any) -> None:
        probe.stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    result: dict[str, Any]
    exit_code = 1
    try:
        probe.spin_for(args.warmup_wall_seconds)
        # Warmup proves liveness/rates but must not contaminate the bounded
        # motion-window performance measurement.
        probe.world_samples.clear()
        displacement = probe.move_test()
        probe.wait_for_contact_clock_bracket()
        result = probe.evaluate(displacement)
        exit_code = 0 if result['verdict'] == 'PASS' else 1
    except Exception as error:
        result = {
            'verdict': 'ERROR',
            'failures': [f'{type(error).__name__}: {error}'],
        }
    finally:
        with suppress(Exception):
            probe.publish_zero()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )
        print(json.dumps(result, indent=2, sort_keys=True))
        probe.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
