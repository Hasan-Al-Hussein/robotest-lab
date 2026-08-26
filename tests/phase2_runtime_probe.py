#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Bounded Phase 2 graph, command-chain, stamp, TF, and RTF probe.

This probe observes the no-fault navigation baseline.  It deliberately does
not calculate Scenario 1 collision, path-length, efficiency, or repeated-run
acceptance metrics; those remain Phase 3 responsibilities.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import time
from collections import Counter, defaultdict, deque
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.msg import Path as NavPath
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

PROBE_NODE = '/robotest/evidence/phase2_runtime_probe'
ZERO_LINEAR_MPS = 0.02
ZERO_ANGULAR_RADPS = 0.05
NANOSECONDS_PER_SECOND = 1_000_000_000
MINIMUM_RTF_WINDOWS = 5
RTF_WINDOW_SAMPLE_SPAN = 5
DEFAULT_TRACE_CAPACITY = 8192
MAXIMUM_TF_EDGES = 256
CONTACT_PUBLIC_HEARTBEAT_NS = 200_000_000
CONTACT_PUBLIC_MAX_GAP_NS = 220_000_000
CONTACT_PUBLIC_MAX_CLOCK_LAG_NS = 220_000_000
CONTACT_CLOCK_BRACKET_WALL_TIMEOUT_S = 2.0
CONTACT_PUBLIC_MAX_WALL_INTER_RECEIPT_GAP_S = CONTACT_CLOCK_BRACKET_WALL_TIMEOUT_S
CONTACT_PUBLIC_MAX_RECORDS = 16

# Gazebo advances this world in 2 ms steps at a configured target RTF of 1.0.
# /clock is unthrottled, BEST_EFFORT, and KEEP_LAST(1), so a subscriber may skip
# source ticks under load.  A skipped simulation-stamp interval is therefore not
# a clock freeze.  Frozen-clock evidence comes from steady-wall callback silence;
# the separate stamp-jump guard permits 250 source ticks of observer loss.
CLOCK_SOURCE_RATE_HZ = 500.0
CONFIGURED_REAL_TIME_FACTOR = 1.0
CLOCK_MAXIMUM_WALL_INTER_RECEIPT_GAP_S = 0.5
CLOCK_MAXIMUM_STAMP_GAP_SOURCE_TICKS = 250
CLOCK_MAXIMUM_STAMP_GAP_S = CLOCK_MAXIMUM_STAMP_GAP_SOURCE_TICKS / CLOCK_SOURCE_RATE_HZ

# Gazebo Sim 8.11 advertises world statistics at 10 messages per steady-wall
# second.  The bridge contract is RELIABLE KEEP_LAST(10), so one callback can
# legitimately trail the latest /clock callback by a full queue plus observer
# scheduling margin without the source having stopped.  Publisher continuity is
# gated independently by the wall inter-receipt gap and simulation-stamp gap.
WORLD_STATS_SOURCE_WALL_RATE_HZ = 10.0
WORLD_STATS_QOS_DEPTH = 10
WORLD_STATS_MAXIMUM_WALL_INTER_RECEIPT_GAP_S = 0.5
WORLD_STATS_MAXIMUM_RECEIPT_AGE_S = CONFIGURED_REAL_TIME_FACTOR * (
    WORLD_STATS_QOS_DEPTH / WORLD_STATS_SOURCE_WALL_RATE_HZ + CLOCK_MAXIMUM_WALL_INTER_RECEIPT_GAP_S
)

# These depths are mechanically fixed in project-owned source: bridge.yaml,
# robotest_faults/qos_profiles.hpp, and the explicit subscriptions above.  An
# endpoint absent from this table is not assigned an invented Nav2/default
# depth.  Fast DDS may report UNKNOWN history and depth 0; that is recorded as
# live-inconclusive while this source contract remains independently auditable.
STATIC_QOS_DEPTH_CONTRACT: dict[str, dict[str, dict[str, int]]] = {
    '/clock': {
        'publishers': {'/robotest/parameter_bridge': 1},
        'subscribers': {PROBE_NODE: 1},
    },
    '/robotest/map': {'subscribers': {PROBE_NODE: 1}},
    '/robotest/raw/scan': {
        'publishers': {'/robotest/parameter_bridge': 5},
        'subscribers': {'/robotest/fault_proxy': 5},
    },
    '/robotest/scan': {
        'publishers': {'/robotest/fault_proxy': 5},
        'subscribers': {PROBE_NODE: 5},
    },
    '/robotest/raw/odom': {
        'publishers': {'/robotest/parameter_bridge': 10},
        'subscribers': {'/robotest/fault_proxy': 10},
    },
    '/robotest/odom': {
        'publishers': {'/robotest/fault_proxy': 10},
        'subscribers': {PROBE_NODE: 10},
    },
    '/robotest/raw/imu': {
        'publishers': {'/robotest/parameter_bridge': 10},
        'subscribers': {'/robotest/fault_proxy': 10},
    },
    '/robotest/imu': {
        'publishers': {'/robotest/fault_proxy': 10},
        'subscribers': {PROBE_NODE: 10},
    },
    '/robotest/navigation/plan': {'subscribers': {PROBE_NODE: 5}},
    '/robotest/cmd_vel_nav': {'subscribers': {PROBE_NODE: 1}},
    '/robotest/cmd_vel_smoothed': {'subscribers': {PROBE_NODE: 1}},
    '/robotest/cmd_vel': {
        'subscribers': {
            '/robotest/parameter_bridge': 1,
            PROBE_NODE: 1,
        }
    },
    '/robotest/validation/ground_truth': {
        'publishers': {'/robotest/parameter_bridge': 10},
        'subscribers': {PROBE_NODE: 10},
    },
    '/robotest/validation/contacts': {
        'publishers': {'/robotest/contact_stream_gate': 10},
        'subscribers': {PROBE_NODE: 10},
    },
    '/robotest/internal/raw_contacts': {
        'publishers': {'/robotest/parameter_bridge': 64},
        'subscribers': {'/robotest/contact_stream_gate': 64},
    },
    '/robotest/validation/world_stats': {
        'publishers': {'/robotest/parameter_bridge': 10},
        'subscribers': {PROBE_NODE: 10},
    },
    '/robotest/faults/events': {
        'publishers': {'/robotest/fault_proxy': 100},
        'subscribers': {PROBE_NODE: 100},
    },
    '/tf': {'subscribers': {PROBE_NODE: 100}},
    '/tf_static': {'subscribers': {PROBE_NODE: 1}},
}

TF_EDGE_SOURCE_CONTRACT = {
    'dynamic': {
        'map->odom': '/robotest/amcl',
        'odom->base_footprint': '/robotest/fault_proxy',
    },
    'static': {
        'base_footprint->base_link': '/robotest/robot_state_publisher',
        'base_link->lidar_link': '/robotest/robot_state_publisher',
        'base_link->imu_link': '/robotest/robot_state_publisher',
    },
}


def qos(
    depth: int,
    reliability: ReliabilityPolicy,
    durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
) -> QoSProfile:
    """Construct an explicit bounded QoS profile."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability,
        durability=durability,
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


def full_node_name(namespace: str, name: str) -> str:
    return f'{namespace.rstrip("/")}/{name}'.replace('//', '/')


def percentile(values: list[float], fraction: float) -> float | None:
    """Return a conservative nearest-rank percentile."""
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def finite_values(values: dict[str, float]) -> tuple[bool, list[str]]:
    """Return whether named floating-point evidence is JSON-safe and finite."""
    invalid = sorted(name for name, value in values.items() if not math.isfinite(value))
    return not invalid, invalid


def cached_clock_offset_failures(
    topic: str,
    evidence: dict[str, Any],
    *,
    maximum_receipt_age_s: float,
    maximum_future_offset_s: float,
    enforce: bool,
) -> list[str]:
    """Validate a cached callback clock relation only when it is causal policy."""
    if not enforce:
        return []
    failures = []
    if evidence['maximum_receipt_age_s'] > maximum_receipt_age_s:
        failures.append(
            f'{topic} maximum receipt age {evidence["maximum_receipt_age_s"]} '
            f'exceeds {maximum_receipt_age_s} s'
        )
    if evidence['maximum_future_offset_s'] > maximum_future_offset_s:
        failures.append(
            f'{topic} maximum future offset {evidence["maximum_future_offset_s"]} '
            f'exceeds {maximum_future_offset_s} s'
        )
    return failures


def windowed_rtf(
    samples: list[tuple[int, float, bool, float]], sample_span: int = 5
) -> list[float]:
    """Calculate RTF over non-overlapping multi-sample steady-wall windows."""
    values: list[float] = []
    for start in range(0, len(samples) - sample_span, sample_span):
        window = samples[start : start + sample_span + 1]
        if any(sample[2] for sample in window):
            continue
        delta_sim = (window[-1][0] - window[0][0]) / 1e9
        delta_wall = window[-1][1] - window[0][1]
        if delta_sim > 0.0 and delta_wall > 0.0:
            values.append(delta_sim / delta_wall)
    return values


def exact_endpoint_failure(
    topic: str,
    side: str,
    actual_nodes: list[str],
    expected_nodes: set[str],
) -> str | None:
    """Require exact endpoint multiplicity as well as exact node names."""
    if len(actual_nodes) != len(expected_nodes) or set(actual_nodes) != expected_nodes:
        return f'{topic} {side} endpoints {sorted(actual_nodes)} != {sorted(expected_nodes)}'
    return None


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


def serialize_result(result: dict[str, Any]) -> str:
    """Serialize evidence while rejecting non-standard NaN/Infinity JSON."""
    return (
        json.dumps(
            result,
            allow_nan=False,
            indent=2,
            sort_keys=True,
        )
        + '\n'
    )


def write_result(path: Path, result: dict[str, Any]) -> str:
    """Atomically publish one complete UTF-8 evidence artifact."""
    serialized = serialize_result(result)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f'.{path.name}.tmp')
    try:
        temporary.write_text(serialized, encoding='utf-8')
        temporary.replace(path)
    finally:
        with suppress(FileNotFoundError):
            temporary.unlink()
    return serialized


@dataclass
class StampTracker:
    sample_count: int = 0
    first_stamp_ns: int | None = None
    previous_stamp_ns: int | None = None
    latest_stamp_ns: int | None = None
    regression_count: int = 0
    duplicate_count: int = 0
    maximum_forward_gap_ns: int = 0
    maximum_receipt_age_ns: int = 0
    future_relative_to_clock_count: int = 0
    maximum_future_offset_ns: int = 0
    previous_receipt_wall_s: float | None = None
    wall_inter_receipt_interval_count: int = 0
    maximum_wall_inter_receipt_gap_s: float = 0.0

    def observe(
        self,
        stamp: int,
        latest_clock: int | None,
        receipt_wall_s: float | None = None,
    ) -> None:
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
            stamp if self.latest_stamp_ns is None else max(stamp, self.latest_stamp_ns)
        )
        if receipt_wall_s is not None:
            if self.previous_receipt_wall_s is not None:
                wall_gap = receipt_wall_s - self.previous_receipt_wall_s
                self.wall_inter_receipt_interval_count += 1
                self.maximum_wall_inter_receipt_gap_s = max(
                    self.maximum_wall_inter_receipt_gap_s,
                    wall_gap,
                )
            self.previous_receipt_wall_s = receipt_wall_s
        if latest_clock is not None:
            receipt_age = latest_clock - stamp
            if receipt_age < 0:
                self.future_relative_to_clock_count += 1
                self.maximum_future_offset_ns = max(
                    self.maximum_future_offset_ns,
                    -receipt_age,
                )
            else:
                self.maximum_receipt_age_ns = max(
                    self.maximum_receipt_age_ns,
                    receipt_age,
                )

    def evidence(self, latest_clock: int | None) -> dict[str, Any]:
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
            'future_relative_to_clock_count': self.future_relative_to_clock_count,
            'maximum_future_offset_s': self.maximum_future_offset_ns / 1e9,
            'final_age_ns': final_age_ns,
            'final_age_s': None if final_age_ns is None else final_age_ns / 1e9,
            'wall_inter_receipt_interval_count': self.wall_inter_receipt_interval_count,
            'maximum_wall_inter_receipt_gap_s': (
                self.maximum_wall_inter_receipt_gap_s
                if self.wall_inter_receipt_interval_count
                else None
            ),
        }


class Phase2RuntimeProbe(Node):
    """Observe one bounded Phase 2 baseline without publishing autonomy data."""

    def __init__(self, trace_capacity: int = DEFAULT_TRACE_CAPACITY) -> None:
        super().__init__('phase2_runtime_probe', namespace='/robotest/evidence')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.started_wall = time.monotonic()
        self.stop_requested = False
        self.failures: list[str] = []
        self.counts: dict[str, int] = defaultdict(int)
        self.first_wall: dict[str, float] = {}
        self.last_wall: dict[str, float] = {}
        self.first_sim_ns: dict[str, int] = {}
        self.last_sim_ns: dict[str, int] = {}
        self.latest_clock_ns: int | None = None
        self.stamp_trackers: dict[str, StampTracker] = defaultdict(StampTracker)
        self.contact_frame_id_violation_count = 0
        self.contact_name_violation_count = 0
        self.contact_record_count_violation_count = 0
        self.contact_same_pair_set_interval_violation_count = 0
        self.contact_minimum_same_pair_set_interval_ns: int | None = None
        self.previous_contact_pair_set: frozenset[tuple[str, str]] | None = None
        self.previous_contact_pair_set_stamp_ns: int | None = None
        self.contact_clock_bracket: dict[str, Any] | None = None
        self.numeric_rejections: Counter[str] = Counter()
        self.world_samples: deque[tuple[int, float, bool, float]] = deque(maxlen=4096)
        self.world_samples_dropped = 0
        self.command_traces: dict[str, deque[dict[str, Any]]] = {
            name: deque(maxlen=trace_capacity)
            for name in ('cmd_vel_nav', 'cmd_vel_smoothed', 'cmd_vel')
        }
        self.command_trace_dropped: dict[str, int] = defaultdict(int)
        self.ground_truth_trace: deque[dict[str, Any]] = deque(maxlen=trace_capacity)
        self.ground_truth_trace_dropped = 0
        self.tf_edges: set[tuple[str, str]] = set()
        self.tf_static_edges: set[tuple[str, str]] = set()
        self.tf_edge_occurrences: Counter[tuple[str, str]] = Counter()
        self.tf_static_edge_occurrences: Counter[tuple[str, str]] = Counter()
        self.tf_duplicate_edges_within_message: Counter[tuple[str, str]] = Counter()
        self.tf_static_duplicate_edges_within_message: Counter[tuple[str, str]] = Counter()
        self.graph_audit_count = 0
        self.validation_subscribers_seen: dict[str, set[str]] = defaultdict(set)
        self.command_publishers_seen: dict[str, set[str]] = defaultdict(set)
        self.command_subscribers_seen: dict[str, set[str]] = defaultdict(set)
        self._subscriptions_kept: list[Any] = []

        best_effort = ReliabilityPolicy.BEST_EFFORT
        reliable = ReliabilityPolicy.RELIABLE
        volatile = DurabilityPolicy.VOLATILE
        transient = DurabilityPolicy.TRANSIENT_LOCAL
        self._subscriptions_kept.extend(
            [
                self.create_subscription(Clock, '/clock', self._on_clock, qos(1, best_effort)),
                self.create_subscription(
                    OccupancyGrid,
                    '/robotest/map',
                    self._on_map,
                    qos(1, reliable, transient),
                ),
                self.create_subscription(
                    LaserScan,
                    '/robotest/scan',
                    self._on_scan,
                    qos(5, best_effort, volatile),
                ),
                self.create_subscription(
                    Odometry,
                    '/robotest/odom',
                    self._on_odom,
                    qos(10, best_effort, volatile),
                ),
                self.create_subscription(
                    Imu,
                    '/robotest/imu',
                    self._on_imu,
                    qos(10, best_effort, volatile),
                ),
                self.create_subscription(
                    NavPath,
                    '/robotest/navigation/plan',
                    self._on_plan,
                    qos(5, reliable, volatile),
                ),
                self.create_subscription(
                    Twist,
                    '/robotest/cmd_vel_nav',
                    self._command_callback('cmd_vel_nav'),
                    qos(1, reliable, volatile),
                ),
                self.create_subscription(
                    Twist,
                    '/robotest/cmd_vel_smoothed',
                    self._command_callback('cmd_vel_smoothed'),
                    qos(1, reliable, volatile),
                ),
                self.create_subscription(
                    Twist,
                    '/robotest/cmd_vel',
                    self._command_callback('cmd_vel'),
                    qos(1, reliable, volatile),
                ),
                self.create_subscription(
                    Odometry,
                    '/robotest/validation/ground_truth',
                    self._on_ground_truth,
                    qos(10, reliable, volatile),
                ),
                self.create_subscription(
                    Contacts,
                    '/robotest/validation/contacts',
                    self._on_contacts,
                    qos(10, reliable, volatile),
                ),
                self.create_subscription(
                    WorldStatistics,
                    '/robotest/validation/world_stats',
                    self._on_world_stats,
                    qos(10, reliable, volatile),
                ),
                self.create_subscription(
                    FaultEvent,
                    '/robotest/faults/events',
                    self._on_fault_event,
                    qos(100, reliable, volatile),
                ),
                self.create_subscription(TFMessage, '/tf', self._on_tf, qos(100, reliable)),
                self.create_subscription(
                    TFMessage,
                    '/tf_static',
                    self._on_tf_static,
                    qos(1, reliable, transient),
                ),
            ]
        )

    def _record(self, key: str, simulation_stamp_ns: int | None = None) -> None:
        now = time.monotonic()
        self.counts[key] += 1
        self.first_wall.setdefault(key, now)
        self.last_wall[key] = now
        if simulation_stamp_ns is not None:
            self.first_sim_ns.setdefault(key, simulation_stamp_ns)
            self.last_sim_ns[key] = simulation_stamp_ns
            self.stamp_trackers[key].observe(
                simulation_stamp_ns,
                self.latest_clock_ns,
                receipt_wall_s=now,
            )

    def _on_clock(self, message: Clock) -> None:
        self.latest_clock_ns = time_ns(message.clock)
        self._record('clock', self.latest_clock_ns)

    def _on_map(self, message: OccupancyGrid) -> None:
        self._record('map', stamp_ns(message))

    def _on_scan(self, message: LaserScan) -> None:
        self._record('scan', stamp_ns(message))

    def _on_odom(self, message: Odometry) -> None:
        self._record('odom', stamp_ns(message))

    def _on_imu(self, message: Imu) -> None:
        self._record('imu', stamp_ns(message))

    def _on_plan(self, message: NavPath) -> None:
        self._record('plan', stamp_ns(message))

    def _command_callback(self, key: str) -> Callable[[Twist], None]:
        def callback(message: Twist) -> None:
            self._record(key)
            values = {
                'linear_x_mps': float(message.linear.x),
                'linear_y_mps': float(message.linear.y),
                'angular_z_radps': float(message.angular.z),
            }
            valid, invalid = finite_values(values)
            if not valid:
                self.numeric_rejections[f'/robotest/{key}:{",".join(invalid)}'] += 1
                return
            if self.latest_clock_ns is None:
                self.numeric_rejections[f'/robotest/{key}:missing_simulation_stamp'] += 1
                return
            trace = self.command_traces[key]
            if len(trace) == trace.maxlen:
                self.command_trace_dropped[key] += 1
            trace.append(
                {
                    'wall_offset_s': time.monotonic() - self.started_wall,
                    'simulation_stamp_ns': self.latest_clock_ns,
                    **values,
                }
            )

        return callback

    def _on_ground_truth(self, message: Odometry) -> None:
        stamp = stamp_ns(message)
        self._record('ground_truth', stamp)
        values = {
            'x_m': float(message.pose.pose.position.x),
            'y_m': float(message.pose.pose.position.y),
            'linear_x_mps': float(message.twist.twist.linear.x),
            'linear_y_mps': float(message.twist.twist.linear.y),
            'angular_z_radps': float(message.twist.twist.angular.z),
        }
        valid, invalid = finite_values(values)
        if not valid:
            self.numeric_rejections[f'/robotest/validation/ground_truth:{",".join(invalid)}'] += 1
            return
        if len(self.ground_truth_trace) == self.ground_truth_trace.maxlen:
            self.ground_truth_trace_dropped += 1
        self.ground_truth_trace.append(
            {
                'stamp_ns': stamp,
                **values,
            }
        )

    def _on_contacts(self, message: Contacts) -> None:
        simulation_stamp = stamp_ns(message)
        self._record('contacts', simulation_stamp)
        if message.header.frame_id:
            self.contact_frame_id_violation_count += 1
        if not valid_public_contact_record_count(len(message.contacts)):
            self.contact_record_count_violation_count += 1
        pairs: set[tuple[str, str]] = set()
        for contact in message.contacts:
            first = contact.collision1.name
            second = contact.collision2.name
            if not valid_scoped_collision_name(first) or not valid_scoped_collision_name(second):
                self.contact_name_violation_count += 1
                continue
            pairs.add(tuple(sorted((first, second))))
        pair_set = frozenset(pairs)
        if (
            self.previous_contact_pair_set is not None
            and self.previous_contact_pair_set_stamp_ns is not None
            and pair_set == self.previous_contact_pair_set
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
        self.previous_contact_pair_set = pair_set
        self.previous_contact_pair_set_stamp_ns = simulation_stamp

    def _on_fault_event(self, _message: FaultEvent) -> None:
        self._record('fault_events')

    def _on_world_stats(self, message: WorldStatistics) -> None:
        simulation_stamp = time_ns(message.sim_time)
        self._record('world_stats', simulation_stamp)
        real_time_factor = float(message.real_time_factor)
        if not math.isfinite(real_time_factor) or real_time_factor < 0.0:
            self.numeric_rejections['/robotest/validation/world_stats:real_time_factor'] += 1
            return
        if len(self.world_samples) == self.world_samples.maxlen:
            self.world_samples_dropped += 1
        self.world_samples.append(
            (
                simulation_stamp,
                time.monotonic(),
                bool(message.paused),
                real_time_factor,
            )
        )

    def _record_tf(
        self,
        target: set[tuple[str, str]],
        occurrences: Counter[tuple[str, str]],
        duplicates_within_message: Counter[tuple[str, str]],
        message: TFMessage,
    ) -> None:
        message_edges: Counter[tuple[str, str]] = Counter()
        for transform in message.transforms:
            edge = (
                transform.header.frame_id.lstrip('/'),
                transform.child_frame_id.lstrip('/'),
            )
            if not all(edge):
                self.numeric_rejections['TF:empty_frame_id'] += 1
                continue
            if edge not in target and len(target) >= MAXIMUM_TF_EDGES:
                self.failures.append(f'TF edge evidence exceeded its {MAXIMUM_TF_EDGES}-edge bound')
                return
            target.add(edge)
            occurrences[edge] += 1
            message_edges[edge] += 1
        for edge, count in message_edges.items():
            if count > 1:
                duplicates_within_message[edge] += count - 1

    def _on_tf(self, message: TFMessage) -> None:
        self._record('tf')
        self._record_tf(
            self.tf_edges,
            self.tf_edge_occurrences,
            self.tf_duplicate_edges_within_message,
            message,
        )

    def _on_tf_static(self, message: TFMessage) -> None:
        self._record('tf_static')
        self._record_tf(
            self.tf_static_edges,
            self.tf_static_edge_occurrences,
            self.tf_static_duplicate_edges_within_message,
            message,
        )

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
        self.contact_clock_bracket = {
            'clock_stamp_ns': latest_clock,
            'clock_minus_public_latest_ns': (
                latest_clock - latest_contact
                if latest_clock is not None and latest_contact is not None
                else None
            ),
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

    def endpoint_snapshot(self, topic: str) -> dict[str, Any]:
        def encode(endpoint: Any) -> dict[str, Any]:
            profile = endpoint.qos_profile
            return {
                'node': full_node_name(endpoint.node_namespace, endpoint.node_name),
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

    def audit_transient_graph_ownership(self) -> None:
        """Accumulate owners seen while the mission runner itself is alive."""
        self.graph_audit_count += 1
        for topic in (
            '/robotest/validation/ground_truth',
            '/robotest/validation/contacts',
            '/robotest/validation/world_stats',
        ):
            snapshot = self.endpoint_snapshot(topic)
            self.validation_subscribers_seen[topic].update(
                item['node'] for item in snapshot['subscribers']
            )
        for topic in (
            '/robotest/cmd_vel_nav',
            '/robotest/cmd_vel_smoothed',
            '/robotest/cmd_vel',
            '/robotest/cmd_vel_behavior_unused',
        ):
            snapshot = self.endpoint_snapshot(topic)
            self.command_publishers_seen[topic].update(
                item['node'] for item in snapshot['publishers']
            )
            self.command_subscribers_seen[topic].update(
                item['node'] for item in snapshot['subscribers']
            )

    @staticmethod
    def qos_introspection_status(
        snapshot: dict[str, Any],
        static_depths: dict[str, dict[str, int]],
    ) -> dict[str, Any]:
        incomplete: list[dict[str, Any]] = []
        exact_observations: list[dict[str, Any]] = []
        exact_mismatches: list[dict[str, Any]] = []
        missing_static_contract_endpoints: list[dict[str, Any]] = []
        bounded = True
        endpoint_count = 0
        for side in ('publishers', 'subscribers'):
            observed_nodes: set[str] = set()
            for endpoint in snapshot[side]:
                observed_nodes.add(endpoint['node'])
                endpoint_count += 1
                reasons = []
                history = endpoint['qos']['history']
                depth = endpoint['qos']['depth']
                expected_depth = static_depths.get(side, {}).get(endpoint['node'])
                if history in {'UNKNOWN', 'SYSTEM_DEFAULT'}:
                    reasons.append(f'history={history}')
                if depth <= 0:
                    reasons.append(f'depth={depth}')
                if history != 'KEEP_LAST' or depth <= 0:
                    bounded = False
                if expected_depth is not None and history == 'KEEP_LAST' and depth > 0:
                    observation = {
                        'side': side,
                        'node': endpoint['node'],
                        'gid': endpoint['gid'],
                        'expected_depth': expected_depth,
                        'observed_depth': depth,
                    }
                    if depth == expected_depth:
                        exact_observations.append(observation)
                    else:
                        exact_mismatches.append(observation)
                if reasons:
                    incomplete.append(
                        {
                            'side': side,
                            'node': endpoint['node'],
                            'gid': endpoint['gid'],
                            'expected_depth': expected_depth,
                            'reasons': reasons,
                        }
                    )
            for node, expected_depth in static_depths.get(side, {}).items():
                if node not in observed_nodes:
                    missing_static_contract_endpoints.append(
                        {
                            'side': side,
                            'node': node,
                            'expected_depth': expected_depth,
                        }
                    )
        complete = endpoint_count > 0 and not incomplete
        return {
            'complete': complete,
            'bounded_depth_live_proven': complete and bounded,
            'exact_static_depth_live_proven': (
                bool(exact_observations)
                and not exact_mismatches
                and not missing_static_contract_endpoints
                and not any(item['expected_depth'] is not None for item in incomplete)
            ),
            'endpoint_count': endpoint_count,
            'incomplete_endpoints': incomplete,
            'exact_depth_observations': exact_observations,
            'exact_depth_mismatches': exact_mismatches,
            'missing_static_contract_endpoints': missing_static_contract_endpoints,
            'note': (
                'UNKNOWN/SYSTEM_DEFAULT history or depth <= 0 is incomplete live '
                'introspection, not a mismatch. Positive KEEP_LAST depths are compared '
                'exactly where project source fixes an expected depth; unlisted Nav2 '
                'defaults are not guessed.'
            ),
        }

    def _check_qos(
        self,
        topic: str,
        snapshot: dict[str, Any],
        reliability: str,
        durability: str,
        static_depths: dict[str, dict[str, int]],
        require_subscribers: bool = True,
    ) -> None:
        for side in ('publishers', 'subscribers'):
            if not snapshot[side] and (side == 'publishers' or require_subscribers):
                self.failures.append(f'{topic} has no live {side}')
            for endpoint in snapshot[side]:
                observed = endpoint['qos']
                if observed['reliability'] != reliability:
                    self.failures.append(
                        f'{topic} {side[:-1]} {endpoint["node"]} reliability '
                        f'{observed["reliability"]} != {reliability}'
                    )
                if observed['durability'] != durability:
                    self.failures.append(
                        f'{topic} {side[:-1]} {endpoint["node"]} durability '
                        f'{observed["durability"]} != {durability}'
                    )
                if observed['history'] == 'KEEP_ALL':
                    self.failures.append(f'{topic} {side[:-1]} {endpoint["node"]} uses KEEP_ALL')
                expected_depth = static_depths.get(side, {}).get(endpoint['node'])
                if (
                    expected_depth is not None
                    and observed['history'] == 'KEEP_LAST'
                    and observed['depth'] > 0
                    and observed['depth'] != expected_depth
                ):
                    self.failures.append(
                        f'{topic} {side[:-1]} {endpoint["node"]} depth '
                        f'{observed["depth"]} != exact source depth {expected_depth}'
                    )
            observed_nodes = {endpoint['node'] for endpoint in snapshot[side]}
            missing = sorted(set(static_depths.get(side, {})) - observed_nodes)
            if missing:
                self.failures.append(f'{topic} is missing source-bound {side}: {missing}')

    def wall_rate(self, key: str) -> float | None:
        first = self.first_wall.get(key)
        last = self.last_wall.get(key)
        count = self.counts.get(key, 0)
        if first is None or last is None or last <= first or count < 2:
            return None
        return (count - 1) / (last - first)

    def simulation_rate(self, key: str) -> float | None:
        first = self.first_sim_ns.get(key)
        last = self.last_sim_ns.get(key)
        count = self.stamp_trackers[key].sample_count
        if first is None or last is None or last <= first or count < 2:
            return None
        return (count - 1) / ((last - first) / 1e9)

    def _check_stamp_stream(
        self,
        topic: str,
        key: str,
        minimum_simulation_rate: float,
        maximum_gap_s: float,
        maximum_final_age_s: float,
        maximum_receipt_age_s: float,
        maximum_future_offset_s: float,
        minimum_simulation_span_s: float = 1.0,
        minimum_sample_count: int = 3,
        maximum_wall_receipt_age_s: float = 1.0,
        maximum_wall_inter_receipt_gap_s: float | None = None,
        enforce_callback_clock_offset: bool = True,
    ) -> dict[str, Any]:
        evidence = self.stamp_trackers[key].evidence(self.latest_clock_ns)
        rate = self.simulation_rate(key)
        first_stamp = evidence['first_stamp_ns']
        latest_stamp = evidence['latest_stamp_ns']
        simulation_span_s = (
            (latest_stamp - first_stamp) / NANOSECONDS_PER_SECOND
            if first_stamp is not None and latest_stamp is not None
            else None
        )
        last_wall = self.last_wall.get(key)
        wall_receipt_age_s = time.monotonic() - last_wall if last_wall is not None else None
        evidence['simulation_rate_hz'] = rate
        evidence['simulation_span_s'] = simulation_span_s
        evidence['wall_receipt_age_s'] = wall_receipt_age_s
        evidence['callback_clock_offset_semantics'] = (
            'bounded_cached_clock_offset'
            if enforce_callback_clock_offset
            else 'diagnostic_noncausal_cached_clock_offset'
        )
        evidence['limits'] = {
            'minimum_simulation_rate_hz': minimum_simulation_rate,
            'maximum_gap_s': maximum_gap_s,
            'maximum_gap_ns': round(maximum_gap_s * NANOSECONDS_PER_SECOND),
            'maximum_final_age_s': maximum_final_age_s,
            'maximum_final_age_ns': round(maximum_final_age_s * NANOSECONDS_PER_SECOND),
            'maximum_receipt_age_s': maximum_receipt_age_s,
            'maximum_receipt_age_ns': round(maximum_receipt_age_s * NANOSECONDS_PER_SECOND),
            'maximum_future_offset_s': maximum_future_offset_s,
            'minimum_simulation_span_s': minimum_simulation_span_s,
            'minimum_sample_count': minimum_sample_count,
            'maximum_wall_receipt_age_s': maximum_wall_receipt_age_s,
            'maximum_wall_inter_receipt_gap_s': maximum_wall_inter_receipt_gap_s,
        }
        if evidence['sample_count'] < minimum_sample_count:
            self.failures.append(
                f'{topic} has {evidence["sample_count"]} timestamped samples, fewer than '
                f'{minimum_sample_count}'
            )
        if simulation_span_s is None or simulation_span_s < minimum_simulation_span_s:
            self.failures.append(
                f'{topic} simulation stamp span {simulation_span_s} below '
                f'{minimum_simulation_span_s} s'
            )
        if evidence['regression_count']:
            self.failures.append(f'{topic} has timestamp regressions')
        if evidence['duplicate_count']:
            self.failures.append(f'{topic} has duplicate timestamps')
        if rate is None or rate < minimum_simulation_rate:
            self.failures.append(
                f'{topic} simulation-time rate {rate} below {minimum_simulation_rate} Hz'
            )
        if evidence['maximum_forward_gap_ns'] > evidence['limits']['maximum_gap_ns']:
            self.failures.append(
                f'{topic} maximum stamp gap {evidence["maximum_forward_gap_s"]} '
                f'exceeds {maximum_gap_s} s'
            )
        self.failures.extend(
            cached_clock_offset_failures(
                topic,
                evidence,
                maximum_receipt_age_s=maximum_receipt_age_s,
                maximum_future_offset_s=maximum_future_offset_s,
                enforce=enforce_callback_clock_offset,
            )
        )
        final_age = evidence['final_age_s']
        final_age_ns = evidence['final_age_ns']
        if (
            final_age_ns is None
            or final_age_ns > evidence['limits']['maximum_final_age_ns']
            or final_age_ns < -round(maximum_future_offset_s * NANOSECONDS_PER_SECOND)
        ):
            self.failures.append(
                f'{topic} final stamp age {final_age} outside '
                f'[-{maximum_future_offset_s}, {maximum_final_age_s}] s'
            )
        if wall_receipt_age_s is None or wall_receipt_age_s > maximum_wall_receipt_age_s:
            self.failures.append(
                f'{topic} last wall receipt age {wall_receipt_age_s} exceeds '
                f'{maximum_wall_receipt_age_s} s'
            )
        observed_wall_gap_s = evidence['maximum_wall_inter_receipt_gap_s']
        if maximum_wall_inter_receipt_gap_s is not None and (
            observed_wall_gap_s is None or observed_wall_gap_s > maximum_wall_inter_receipt_gap_s
        ):
            self.failures.append(
                f'{topic} maximum wall inter-receipt gap {observed_wall_gap_s} exceeds '
                f'{maximum_wall_inter_receipt_gap_s} s'
            )
        return evidence

    def evaluate(self) -> dict[str, Any]:
        if self.graph_audit_count < 2:
            self.failures.append('fewer than two steady-wall transient graph audits completed')
        if self.numeric_rejections:
            self.failures.append(
                f'non-finite or unstamped evidence samples rejected: '
                f'{dict(sorted(self.numeric_rejections.items()))}'
            )
        topic_contracts = {
            '/clock': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/map': ('RELIABLE', 'TRANSIENT_LOCAL'),
            '/robotest/raw/scan': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/scan': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/raw/odom': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/odom': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/raw/imu': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/imu': ('BEST_EFFORT', 'VOLATILE'),
            '/robotest/navigation/plan': ('RELIABLE', 'VOLATILE'),
            '/robotest/cmd_vel_nav': ('RELIABLE', 'VOLATILE'),
            '/robotest/cmd_vel_smoothed': ('RELIABLE', 'VOLATILE'),
            '/robotest/cmd_vel': ('RELIABLE', 'VOLATILE'),
            '/robotest/cmd_vel_behavior_unused': ('RELIABLE', 'VOLATILE'),
            '/robotest/validation/ground_truth': ('RELIABLE', 'VOLATILE'),
            '/robotest/validation/contacts': ('RELIABLE', 'VOLATILE'),
            '/robotest/internal/raw_contacts': ('RELIABLE', 'VOLATILE'),
            '/robotest/validation/world_stats': ('RELIABLE', 'VOLATILE'),
            '/robotest/faults/events': ('RELIABLE', 'VOLATILE'),
            '/tf': ('RELIABLE', 'VOLATILE'),
            '/tf_static': ('RELIABLE', 'TRANSIENT_LOCAL'),
        }
        endpoints: dict[str, Any] = {}
        qos_status: dict[str, Any] = {}
        for topic, (reliability, durability) in topic_contracts.items():
            snapshot = self.endpoint_snapshot(topic)
            static_depths = STATIC_QOS_DEPTH_CONTRACT.get(topic, {})
            endpoints[topic] = snapshot
            qos_status[topic] = self.qos_introspection_status(snapshot, static_depths)
            self._check_qos(
                topic,
                snapshot,
                reliability,
                durability,
                static_depths,
                require_subscribers=(topic != '/robotest/cmd_vel_behavior_unused'),
            )
        if self.contact_frame_id_violation_count:
            self.failures.append('public contact snapshot frame_id must be empty')
        if self.contact_name_violation_count:
            self.failures.append('public contact snapshot contains a malformed scoped name')
        if self.contact_record_count_violation_count:
            self.failures.append('public contact snapshots must contain 1 to 16 records')
        if self.contact_same_pair_set_interval_violation_count:
            self.failures.append(
                'public contact snapshot repeated an unchanged pair set before the 200 ms '
                'heartbeat interval'
            )

        expected_publishers = {
            '/clock': {'/robotest/parameter_bridge'},
            '/robotest/map': {'/robotest/map_server'},
            '/robotest/raw/scan': {'/robotest/parameter_bridge'},
            '/robotest/scan': {'/robotest/fault_proxy'},
            '/robotest/raw/odom': {'/robotest/parameter_bridge'},
            '/robotest/odom': {'/robotest/fault_proxy'},
            '/robotest/raw/imu': {'/robotest/parameter_bridge'},
            '/robotest/imu': {'/robotest/fault_proxy'},
            '/robotest/navigation/plan': {'/robotest/planner_server'},
            '/robotest/cmd_vel_nav': {'/robotest/controller_server'},
            '/robotest/cmd_vel_smoothed': {'/robotest/velocity_smoother'},
            '/robotest/cmd_vel': {'/robotest/collision_monitor'},
            '/robotest/cmd_vel_behavior_unused': {'/robotest/behavior_server'},
            '/robotest/validation/ground_truth': {'/robotest/parameter_bridge'},
            '/robotest/validation/contacts': {'/robotest/contact_stream_gate'},
            '/robotest/internal/raw_contacts': {'/robotest/parameter_bridge'},
            '/robotest/validation/world_stats': {'/robotest/parameter_bridge'},
            '/robotest/faults/events': {'/robotest/fault_proxy'},
            '/tf': {
                '/robotest/amcl',
                '/robotest/fault_proxy',
                '/robotest/robot_state_publisher',
            },
            '/tf_static': {'/robotest/robot_state_publisher'},
        }
        for topic, expected in expected_publishers.items():
            actual = [item['node'] for item in endpoints[topic]['publishers']]
            failure = exact_endpoint_failure(topic, 'publisher', actual, expected)
            if failure:
                self.failures.append(failure)

        connected_subscribers = {
            '/robotest/cmd_vel_nav': {'/robotest/velocity_smoother'},
            '/robotest/cmd_vel_smoothed': {'/robotest/collision_monitor'},
            '/robotest/cmd_vel': {'/robotest/parameter_bridge'},
            '/robotest/cmd_vel_behavior_unused': set(),
            '/robotest/internal/raw_contacts': {'/robotest/contact_stream_gate'},
        }
        for topic, expected in connected_subscribers.items():
            actual = [
                item['node']
                for item in endpoints[topic]['subscribers']
                if item['node'] != PROBE_NODE
            ]
            failure = exact_endpoint_failure(topic, 'connected subscriber', actual, expected)
            if failure:
                self.failures.append(failure)

        for topic in (
            '/robotest/validation/ground_truth',
            '/robotest/validation/contacts',
            '/robotest/validation/world_stats',
        ):
            unauthorized = sorted(
                item['node']
                for item in endpoints[topic]['subscribers']
                if item['node'] != PROBE_NODE
            )
            if unauthorized:
                self.failures.append(
                    f'{topic} has unauthorized non-evidence subscribers: {unauthorized}'
                )
            historical_unauthorized = sorted(self.validation_subscribers_seen[topic] - {PROBE_NODE})
            if historical_unauthorized:
                self.failures.append(
                    f'{topic} had transient unauthorized subscribers during the mission: '
                    f'{historical_unauthorized}'
                )

        for topic, expected in expected_publishers.items():
            if topic not in self.command_publishers_seen:
                continue
            unexpected = sorted(self.command_publishers_seen[topic] - expected)
            if unexpected:
                self.failures.append(
                    f'{topic} had transient unexpected publishers during the mission: {unexpected}'
                )
        for topic, expected in connected_subscribers.items():
            seen = self.command_subscribers_seen[topic] - {PROBE_NODE}
            unexpected = sorted(seen - expected)
            if unexpected:
                self.failures.append(
                    f'{topic} had transient unexpected connected subscribers during the '
                    f'mission: {unexpected}'
                )

        node_names = [
            full_node_name(namespace, name)
            for name, namespace in self.get_node_names_and_namespaces()
        ]
        duplicates = sorted(name for name, count in Counter(node_names).items() if count > 1)
        required_names = set().union(*expected_publishers.values()) | {PROBE_NODE}
        required_duplicates = sorted(set(duplicates) & required_names)
        if required_duplicates:
            self.failures.append(f'duplicate required node names: {required_duplicates}')

        required_dynamic = {('map', 'odom'), ('odom', 'base_footprint')}
        missing_dynamic = sorted(required_dynamic - self.tf_edges)
        required_static = {
            ('base_footprint', 'base_link'),
            ('base_link', 'lidar_link'),
            ('base_link', 'imu_link'),
        }
        missing_static = sorted(required_static - self.tf_static_edges)
        if missing_dynamic:
            self.failures.append(f'missing dynamic TF edges: {missing_dynamic}')
        if missing_static:
            self.failures.append(f'missing static TF edges: {missing_static}')

        dynamic_static_overlap = sorted(self.tf_edges & self.tf_static_edges)
        if dynamic_static_overlap:
            self.failures.append(
                f'TF edges appeared on both /tf and /tf_static: {dynamic_static_overlap}'
            )
        duplicate_dynamic_messages = sorted(self.tf_duplicate_edges_within_message)
        duplicate_static_messages = sorted(self.tf_static_duplicate_edges_within_message)
        if duplicate_dynamic_messages:
            self.failures.append(
                'duplicate dynamic TF edges occurred within one TFMessage: '
                f'{duplicate_dynamic_messages}'
            )
        if duplicate_static_messages:
            self.failures.append(
                'duplicate static TF edges occurred within one TFMessage: '
                f'{duplicate_static_messages}'
            )
        parents_by_child: dict[str, set[str]] = defaultdict(set)
        for parent, child in self.tf_edges | self.tf_static_edges:
            parents_by_child[child].add(parent)
        multiple_parent_children = {
            child: sorted(parents)
            for child, parents in sorted(parents_by_child.items())
            if len(parents) > 1
        }
        if multiple_parent_children:
            self.failures.append(
                f'TF children observed with multiple parents: {multiple_parent_children}'
            )

        stamp_evidence = {
            '/clock': self._check_stamp_stream(
                '/clock',
                'clock',
                20.0,
                CLOCK_MAXIMUM_STAMP_GAP_S,
                0.05,
                0.05,
                0.05,
                minimum_simulation_span_s=2.0,
                minimum_sample_count=20,
                maximum_wall_receipt_age_s=0.5,
                maximum_wall_inter_receipt_gap_s=(CLOCK_MAXIMUM_WALL_INTER_RECEIPT_GAP_S),
            ),
            '/robotest/scan': self._check_stamp_stream(
                '/robotest/scan', 'scan', 4.0, 0.50, 0.60, 0.75, 0.25
            ),
            '/robotest/odom': self._check_stamp_stream(
                '/robotest/odom', 'odom', 10.0, 0.25, 0.35, 0.50, 0.25
            ),
            '/robotest/imu': self._check_stamp_stream(
                '/robotest/imu', 'imu', 20.0, 0.15, 0.25, 0.50, 0.25
            ),
            '/robotest/validation/ground_truth': self._check_stamp_stream(
                '/robotest/validation/ground_truth',
                'ground_truth',
                10.0,
                0.25,
                0.35,
                0.50,
                0.25,
            ),
            '/robotest/validation/contacts': self._check_stamp_stream(
                '/robotest/validation/contacts',
                'contacts',
                4.0,
                0.22,
                0.22,
                0.22,
                0.25,
                maximum_wall_inter_receipt_gap_s=(CONTACT_PUBLIC_MAX_WALL_INTER_RECEIPT_GAP_S),
                enforce_callback_clock_offset=False,
            ),
            '/robotest/validation/world_stats': self._check_stamp_stream(
                '/robotest/validation/world_stats',
                'world_stats',
                5.0,
                0.30,
                0.50,
                WORLD_STATS_MAXIMUM_RECEIPT_AGE_S,
                0.25,
                minimum_simulation_span_s=2.0,
                minimum_sample_count=(MINIMUM_RTF_WINDOWS * RTF_WINDOW_SAMPLE_SPAN + 1),
                maximum_wall_receipt_age_s=1.0,
                maximum_wall_inter_receipt_gap_s=(WORLD_STATS_MAXIMUM_WALL_INTER_RECEIPT_GAP_S),
            ),
        }
        stamp_evidence['/robotest/validation/contacts'].update(
            {
                'clock_bracket': self.contact_clock_bracket,
                'frame_id_violation_count': self.contact_frame_id_violation_count,
                'name_violation_count': self.contact_name_violation_count,
                'record_count_violation_count': (self.contact_record_count_violation_count),
                'minimum_same_pair_set_interval_ns': (
                    self.contact_minimum_same_pair_set_interval_ns
                ),
                'same_pair_set_interval_violation_count': (
                    self.contact_same_pair_set_interval_violation_count
                ),
            }
        )
        if self.counts.get('map', 0) < 1:
            self.failures.append('no map message observed')
        if self.counts.get('plan', 0) < 1:
            self.failures.append('no navigation plan observed')
        command_summary: dict[str, Any] = {}
        for key, trace in self.command_traces.items():
            samples = list(trace)
            nonzero = [
                sample
                for sample in samples
                if abs(sample['linear_x_mps']) > ZERO_LINEAR_MPS
                or abs(sample['angular_z_radps']) > ZERO_ANGULAR_RADPS
            ]
            command_summary[f'/robotest/{key}'] = {
                'capacity': trace.maxlen,
                'dropped': self.command_trace_dropped[key],
                'sample_count': len(samples),
                'nonzero_sample_count': len(nonzero),
                'first_nonzero_simulation_stamp_ns': (
                    nonzero[0]['simulation_stamp_ns'] if nonzero else None
                ),
                'last_sample': samples[-1] if samples else None,
                'samples': samples,
            }
            if not nonzero:
                self.failures.append(f'/robotest/{key} has no observed motion command')
            if self.command_trace_dropped[key]:
                self.failures.append(f'/robotest/{key} command trace exceeded its bound')

        final_samples = list(self.command_traces['cmd_vel'])
        if not final_samples:
            self.failures.append('no final cmd_vel message observed')
        else:
            last = final_samples[-1]
            if (
                abs(last['linear_x_mps']) > ZERO_LINEAR_MPS
                or abs(last['angular_z_radps']) > ZERO_ANGULAR_RADPS
            ):
                self.failures.append('final cmd_vel is outside zero-command tolerance')

        xs = [sample['x_m'] for sample in self.ground_truth_trace]
        ys = [sample['y_m'] for sample in self.ground_truth_trace]
        position_span = None
        if xs and ys:
            position_span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if position_span is None or position_span <= 0.10:
            self.failures.append(
                f'ground-truth motion liveness span {position_span} is not greater than 0.10 m'
            )
        final_ground_truth = self.ground_truth_trace[-1] if self.ground_truth_trace else None
        if final_ground_truth is None:
            self.failures.append('no final ground-truth sample is available for stop evidence')
        elif (
            math.hypot(
                final_ground_truth['linear_x_mps'],
                final_ground_truth['linear_y_mps'],
            )
            > ZERO_LINEAR_MPS
            or abs(final_ground_truth['angular_z_radps']) > ZERO_ANGULAR_RADPS
        ):
            self.failures.append('final ground-truth twist is outside stop tolerance')
        if self.ground_truth_trace_dropped:
            self.failures.append('ground-truth liveness trace exceeded its bound')

        rtf_values = windowed_rtf(
            list(self.world_samples),
            sample_span=RTF_WINDOW_SAMPLE_SPAN,
        )
        rtf_median = statistics.median(rtf_values) if rtf_values else None
        rtf_p5 = percentile(rtf_values, 0.05)
        native_values = [
            sample[3]
            for sample in self.world_samples
            if not sample[2] and math.isfinite(sample[3]) and sample[3] >= 0.0
        ]
        if rtf_median is None or rtf_median < 0.80:
            self.failures.append(f'median calculated RTF {rtf_median} below 0.80')
        if rtf_p5 is None or rtf_p5 < 0.50:
            self.failures.append(f'p5 calculated RTF {rtf_p5} below 0.50')
        if len(rtf_values) < MINIMUM_RTF_WINDOWS:
            self.failures.append(
                f'calculated RTF has {len(rtf_values)} valid windows, fewer than '
                f'{MINIMUM_RTF_WINDOWS}'
            )
        if self.world_samples_dropped:
            self.failures.append('world-statistics RTF samples exceeded their bound')

        return {
            'verdict': 'PASS' if not self.failures else 'FAIL',
            'failures': sorted(set(self.failures)),
            'scope': {
                'fault_schedule': 'EMPTY_NO_FAULT_BASELINE',
                'fault_arming': 'NOT_IMPLEMENTED_NOT_EXERCISED',
                'scenario1_metrics': 'DEFERRED_TO_PHASE3',
                'validation_truth_use': 'motion_liveness_and_evidence_only',
            },
            'counts': dict(self.counts),
            'numeric_evidence_rejections': dict(sorted(self.numeric_rejections.items())),
            'stamp_evidence': stamp_evidence,
            'timing_gate_basis': {
                'clock': {
                    'source': 'robotest_lab.sdf physics step/update rate',
                    'source_rate_hz': CLOCK_SOURCE_RATE_HZ,
                    'qos_effect': (
                        'BEST_EFFORT KEEP_LAST(1) may skip source ticks; simulation-stamp '
                        'gaps do not by themselves prove a frozen clock'
                    ),
                    'maximum_stamp_gap_source_tick_budget': (CLOCK_MAXIMUM_STAMP_GAP_SOURCE_TICKS),
                    'maximum_stamp_gap_s': CLOCK_MAXIMUM_STAMP_GAP_S,
                    'maximum_wall_inter_receipt_gap_s': (CLOCK_MAXIMUM_WALL_INTER_RECEIPT_GAP_S),
                },
                'world_stats': {
                    'source': 'Gazebo Sim 8.11 SimulationRunner transport throttle',
                    'source_wall_rate_hz': WORLD_STATS_SOURCE_WALL_RATE_HZ,
                    'qos_depth': WORLD_STATS_QOS_DEPTH,
                    'configured_real_time_factor': CONFIGURED_REAL_TIME_FACTOR,
                    'maximum_receipt_age_formula': (
                        'configured_real_time_factor * (qos_depth / source_wall_rate_hz '
                        '+ clock_maximum_wall_inter_receipt_gap_s)'
                    ),
                    'maximum_receipt_age_s': WORLD_STATS_MAXIMUM_RECEIPT_AGE_S,
                    'maximum_wall_inter_receipt_gap_s': (
                        WORLD_STATS_MAXIMUM_WALL_INTER_RECEIPT_GAP_S
                    ),
                },
            },
            'command_trace': command_summary,
            'ground_truth_motion_liveness': {
                'capacity': self.ground_truth_trace.maxlen,
                'dropped': self.ground_truth_trace_dropped,
                'sample_count': len(self.ground_truth_trace),
                'position_bounding_span_m': position_span,
                'final_sample': final_ground_truth,
                'stop_tolerance': {
                    'maximum_planar_linear_speed_mps': ZERO_LINEAR_MPS,
                    'maximum_absolute_angular_z_radps': ZERO_ANGULAR_RADPS,
                },
                'samples': list(self.ground_truth_trace),
            },
            'rtf': {
                'method': (
                    'non-overlapping world-statistics windows with '
                    f'{RTF_WINDOW_SAMPLE_SPAN} sample intervals'
                ),
                'source_sample_count': len(self.world_samples),
                'source_samples_dropped': self.world_samples_dropped,
                'window_count': len(rtf_values),
                'minimum_window_count': MINIMUM_RTF_WINDOWS,
                'median': rtf_median,
                'p5': rtf_p5,
                'gazebo_reported_secondary': {
                    'sample_count': len(native_values),
                    'median': statistics.median(native_values) if native_values else None,
                    'p5': percentile(native_values, 0.05),
                },
            },
            'tf_edges_observed': sorted([list(edge) for edge in self.tf_edges]),
            'tf_static_edges_observed': sorted([list(edge) for edge in self.tf_static_edges]),
            'tf_duplicate_edge_evidence': {
                'dynamic_static_overlap': [list(edge) for edge in dynamic_static_overlap],
                'multiple_parents_by_child': multiple_parent_children,
                'dynamic_duplicates_within_message': {
                    f'{parent}->{child}': count
                    for (parent, child), count in sorted(
                        self.tf_duplicate_edges_within_message.items()
                    )
                },
                'static_duplicates_within_message': {
                    f'{parent}->{child}': count
                    for (parent, child), count in sorted(
                        self.tf_static_duplicate_edges_within_message.items()
                    )
                },
                'dynamic_edge_occurrences': {
                    f'{parent}->{child}': count
                    for (parent, child), count in sorted(self.tf_edge_occurrences.items())
                },
                'static_edge_occurrences': {
                    f'{parent}->{child}': count
                    for (parent, child), count in sorted(self.tf_static_edge_occurrences.items())
                },
            },
            'tf_edge_source_contract': {
                **TF_EDGE_SOURCE_CONTRACT,
                'proof_scope': (
                    'static architecture/source-test ownership contract only; observed '
                    'messages cannot be correlated to publisher GIDs by Jazzy rclpy'
                ),
            },
            'tf_edge_source_attribution': (
                'Jazzy rclpy message callbacks do not expose publisher GID. Exact live '
                'TF publisher node sets and observed edge presence are separate proofs; '
                'this artifact does not claim per-edge endpoint attribution.'
            ),
            'node_names': sorted(node_names),
            'duplicate_node_names': duplicates,
            'transient_graph_audit': {
                'sample_count': self.graph_audit_count,
                'validation_subscribers_seen': {
                    topic: sorted(nodes)
                    for topic, nodes in self.validation_subscribers_seen.items()
                },
                'command_publishers_seen': {
                    topic: sorted(nodes) for topic, nodes in self.command_publishers_seen.items()
                },
                'command_subscribers_seen': {
                    topic: sorted(nodes) for topic, nodes in self.command_subscribers_seen.items()
                },
                'purpose': (
                    'union of live graph owners sampled on a steady-wall cadence while '
                    'the mission runner exists; catches transient bypass or validation leaks'
                ),
            },
            'endpoints': endpoints,
            'qos_introspection': qos_status,
            'bounded_depth_live_proven_for_all_endpoints': all(
                item['bounded_depth_live_proven'] for item in qos_status.values()
            ),
            'exact_static_depth_live_proven_for_all_bound_endpoints': all(
                item['exact_static_depth_live_proven']
                for topic, item in qos_status.items()
                if STATIC_QOS_DEPTH_CONTRACT.get(topic)
            ),
            'exact_static_qos_depth_contract': STATIC_QOS_DEPTH_CONTRACT,
            'bounded_depth_static_proof': (
                'The exact project-owned depths above come from bridge.yaml, '
                'robotest_faults/qos_profiles.hpp, and this probe. Nav2 defaults are '
                'intentionally omitted. Fast DDS UNKNOWN history/depth 0 remains '
                'live-inconclusive, not a false mismatch.'
            ),
            'mission_interval_filtering_contract': {
                'lower_bound_field': 'accepted_goal_stamp_ns',
                'upper_bound_field': 'terminal_action_stamp_ns',
                'bounds': 'inclusive',
                'command_sample_stamp_field': 'simulation_stamp_ns',
                'ground_truth_sample_stamp_field': 'stamp_ns',
                'required_preconditions': [
                    'accepted_goal_stamp_ns and terminal_action_stamp_ns are positive integers',
                    'accepted_goal_stamp_ns <= terminal_action_stamp_ns',
                    'every selected trace reports dropped == 0',
                ],
                'capacity_constraint': (
                    f'each command and ground-truth deque is bounded to '
                    f'{DEFAULT_TRACE_CAPACITY} samples by default; any overflow is a probe '
                    'failure, so filtering must reject incomplete traces'
                ),
                'command_stamp_limitation': (
                    'Twist has no header; command simulation_stamp_ns is the latest /clock '
                    'observed by this probe at callback receipt'
                ),
            },
        }


def run_self_test() -> int:
    assert percentile([], 0.05) is None
    assert percentile([4.0, 1.0, 3.0, 2.0], 0.50) == 2.0
    samples = [(index * 1_000_000_000, float(index), False, 1.0) for index in range(11)]
    assert windowed_rtf(samples, sample_span=5) == [1.0, 1.0]
    paused = list(samples)
    paused[2] = (paused[2][0], paused[2][1], True, paused[2][3])
    assert windowed_rtf(paused, sample_span=5) == [1.0]
    assert full_node_name('/robotest', 'controller_server') == '/robotest/controller_server'
    assert exact_endpoint_failure('/topic', 'publisher', ['/a'], {'/a'}) is None
    assert exact_endpoint_failure('/topic', 'publisher', ['/a', '/a'], {'/a'}) is not None
    assert valid_scoped_collision_name('model::link::collision')
    assert valid_scoped_collision_name('world::model::link::collision')
    assert not valid_scoped_collision_name('model::collision')
    assert not valid_scoped_collision_name('model::::collision')
    assert valid_public_contact_record_count(1)
    assert valid_public_contact_record_count(16)
    assert not valid_public_contact_record_count(0)
    assert not valid_public_contact_record_count(17)
    tracker = StampTracker()
    tracker.observe(1_000_000_000, 1_100_000_000, receipt_wall_s=10.0)
    tracker.observe(2_000_000_000, 2_100_000_000, receipt_wall_s=10.1)
    tracker.observe(2_000_000_000, 2_100_000_000, receipt_wall_s=10.2)
    tracker.observe(1_900_000_000, 2_100_000_000, receipt_wall_s=10.7)
    tracker.observe(2_300_000_000, 2_100_000_000, receipt_wall_s=10.8)
    tracker_evidence = tracker.evidence(2_300_000_000)
    assert tracker_evidence['regression_count'] == 1
    assert tracker_evidence['duplicate_count'] == 1
    assert tracker_evidence['maximum_future_offset_s'] == 0.2
    assert tracker_evidence['wall_inter_receipt_interval_count'] == 4
    assert math.isclose(tracker_evidence['maximum_wall_inter_receipt_gap_s'], 0.5)

    lagged_contacts = StampTracker()
    lagged_contacts.observe(10_000_000_000, 10_278_000_000, receipt_wall_s=20.0)
    lagged_contacts.observe(10_200_000_000, 10_300_000_000, receipt_wall_s=20.25)
    lagged_contacts.observe(10_400_000_000, 10_448_000_000, receipt_wall_s=20.50)
    lagged_evidence = lagged_contacts.evidence(10_448_000_000)
    assert lagged_evidence['maximum_forward_gap_ns'] == 200_000_000
    assert lagged_evidence['maximum_receipt_age_ns'] == 278_000_000
    assert lagged_evidence['final_age_ns'] == 48_000_000
    assert (
        cached_clock_offset_failures(
            '/robotest/validation/contacts',
            lagged_evidence,
            maximum_receipt_age_s=0.22,
            maximum_future_offset_s=0.25,
            enforce=False,
        )
        == []
    )
    assert cached_clock_offset_failures(
        '/robotest/scan',
        lagged_evidence,
        maximum_receipt_age_s=0.22,
        maximum_future_offset_s=0.25,
        enforce=True,
    )

    future_contacts = StampTracker()
    future_contacts.observe(10_000_000_000, 9_722_000_000, receipt_wall_s=30.0)
    future_contacts.observe(10_200_000_000, 10_000_000_000, receipt_wall_s=30.25)
    future_contacts.observe(10_400_000_000, 10_448_000_000, receipt_wall_s=30.50)
    future_evidence = future_contacts.evidence(10_448_000_000)
    assert future_evidence['maximum_future_offset_s'] == 0.278
    assert (
        cached_clock_offset_failures(
            '/robotest/validation/contacts',
            future_evidence,
            maximum_receipt_age_s=0.22,
            maximum_future_offset_s=0.25,
            enforce=False,
        )
        == []
    )
    assert math.isclose(CLOCK_MAXIMUM_STAMP_GAP_S, 0.5)
    assert math.isclose(WORLD_STATS_MAXIMUM_RECEIPT_AGE_S, 1.5)
    valid, invalid = finite_values({'finite': 1.0, 'nan': math.nan})
    assert not valid and invalid == ['nan']
    try:
        serialize_result({'invalid': math.nan})
    except ValueError:
        pass
    else:
        raise AssertionError('serialize_result accepted NaN')
    unknown_snapshot = {
        'publishers': [
            {
                'node': '/publisher',
                'gid': '00',
                'qos': {
                    'history': 'UNKNOWN',
                    'depth': 0,
                    'reliability': 'BEST_EFFORT',
                    'durability': 'VOLATILE',
                },
            }
        ],
        'subscribers': [],
    }
    qos_evidence = Phase2RuntimeProbe.qos_introspection_status(
        unknown_snapshot,
        {'publishers': {'/publisher': 5}},
    )
    assert not qos_evidence['exact_depth_mismatches']
    assert not qos_evidence['exact_static_depth_live_proven']
    print('phase2_runtime_probe self-test PASS')
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    parser.add_argument('--ready-file', type=Path)
    parser.add_argument('--stop-file', type=Path)
    parser.add_argument('--warmup-wall-seconds', type=float, default=10.0)
    parser.add_argument('--maximum-wall-seconds', type=float, default=330.0)
    parser.add_argument('--stop-grace-wall-seconds', type=float, default=2.0)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.output is None or args.ready_file is None or args.stop_file is None:
        parser.error('--output, --ready-file, and --stop-file are required')
    for value, name in (
        (args.warmup_wall_seconds, 'warmup'),
        (args.maximum_wall_seconds, 'maximum'),
        (args.stop_grace_wall_seconds, 'stop grace'),
    ):
        if not math.isfinite(value) or value <= 0.0:
            parser.error(f'{name} wall seconds must be finite and positive')
    if args.maximum_wall_seconds <= args.warmup_wall_seconds:
        parser.error('maximum wall seconds must exceed warmup')

    probe: Phase2RuntimeProbe | None = None
    initialized = False
    result: dict[str, Any] = {
        'verdict': 'ERROR',
        'failures': ['probe did not initialize'],
        'scope': {'scenario1_metrics': 'DEFERRED_TO_PHASE3'},
    }
    exit_code = 1
    try:
        rclpy.init()
        initialized = True
        probe = Phase2RuntimeProbe()
        hard_deadline = time.monotonic() + args.maximum_wall_seconds

        def request_stop(_signum: int, _frame: Any) -> None:
            if probe is not None:
                probe.stop_requested = True

        signal.signal(signal.SIGINT, request_stop)
        signal.signal(signal.SIGTERM, request_stop)
        probe.spin_for(args.warmup_wall_seconds)
        probe.world_samples.clear()
        probe.world_samples_dropped = 0
        args.ready_file.parent.mkdir(parents=True, exist_ok=True)
        args.ready_file.write_text('ready\n', encoding='utf-8')
        next_graph_audit = time.monotonic()
        while (
            time.monotonic() < hard_deadline
            and not probe.stop_requested
            and not args.stop_file.is_file()
        ):
            rclpy.spin_once(probe, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_graph_audit:
                probe.audit_transient_graph_ownership()
                next_graph_audit = now + 1.0
        if time.monotonic() >= hard_deadline:
            probe.failures.append('probe reached its steady-wall hard deadline')
        elif args.stop_file.is_file() and not probe.stop_requested:
            probe.spin_for(args.stop_grace_wall_seconds)
        probe.wait_for_contact_clock_bracket()
        result = probe.evaluate()
        exit_code = 0 if result['verdict'] == 'PASS' else 1
    except Exception as error:
        result = {
            'verdict': 'ERROR',
            'failures': [f'{type(error).__name__}: {error}'],
            'scope': {'scenario1_metrics': 'DEFERRED_TO_PHASE3'},
        }
    finally:
        if probe is not None:
            with suppress(Exception):
                probe.destroy_node()
        if initialized:
            with suppress(Exception):
                rclpy.shutdown()
        try:
            serialized = write_result(args.output, result)
        except (TypeError, ValueError) as error:
            result = {
                'verdict': 'ERROR',
                'failures': [f'evidence serialization rejected unsafe numeric data: {error}'],
                'scope': {'scenario1_metrics': 'DEFERRED_TO_PHASE3'},
            }
            serialized = write_result(args.output, result)
            exit_code = 1
        print(serialized, end='')
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
