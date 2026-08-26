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

"""Bounded observer-only snapshots of the Nav2 lifecycle state surface."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from rosgraph_msgs.msg import Clock

from robotest_metrics.artifacts import canonical_sha256, write_json_atomic
from robotest_metrics.buffers import PrefixBuffer
from robotest_metrics.constants import NAV2_LIFECYCLE_NODES
from robotest_metrics.errors import ArtifactError, MetricUnavailable

MAX_SCHEDULE_ROUNDS = 96
MAX_LIFECYCLE_SAMPLES = MAX_SCHEDULE_ROUNDS * len(NAV2_LIFECYCLE_NODES)
MAX_SCHEDULE_BYTES = 65_536
MAX_RUN_ID_BYTES = 256
MAX_STATE_LABEL_BYTES = 256
MAX_ERROR_BYTES = 512
MAX_STAMP_NS = (1 << 63) - 1
READY_FILE_MAX_BYTES = 16_384


def lifecycle_service_name(node: str) -> str:
    """Return the relative get-state service for one exact allowlisted node."""
    if node not in NAV2_LIFECYCLE_NODES:
        raise ValueError(f'unsupported lifecycle node: {node}')
    return f'{node}/get_state'


def _bounded_text(value: Any, field: str, maximum_bytes: int) -> str:
    if not isinstance(value, str) or not value:
        raise MetricUnavailable(f'{field} must be a non-empty string')
    if len(value.encode('utf-8')) > maximum_bytes:
        raise MetricUnavailable(f'{field} exceeds its {maximum_bytes}-byte UTF-8 bound')
    return value


def _stamp_ns(value: Any, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise MetricUnavailable(f'{field} must be an integer')
    if value < 0 or value > MAX_STAMP_NS:
        raise MetricUnavailable(f'{field} must be in [0, {MAX_STAMP_NS}]')
    return value


def _reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    document: dict[str, Any] = {}
    for key, value in pairs:
        if key in document:
            raise MetricUnavailable(f'duplicate JSON key: {key}')
        document[key] = value
    return document


@dataclass(frozen=True)
class LifecycleSchedule:
    """Validated immutable lifecycle sampling schedule."""

    run_id: str
    requested_stamps_ns: tuple[int, ...]

    @property
    def expected_sample_count(self) -> int:
        """Return the exact number of node snapshots requested by this schedule."""
        return len(self.requested_stamps_ns) * len(NAV2_LIFECYCLE_NODES)

    def as_dict(self) -> dict[str, Any]:
        """Return the canonical schedule document."""
        return {
            'lifecycle_schedule_schema_version': 1,
            'requested_stamps_ns': list(self.requested_stamps_ns),
            'run_id': self.run_id,
        }

    @property
    def sha256(self) -> str:
        """Hash the normalized canonical schedule."""
        return canonical_sha256(self.as_dict())


def validate_schedule(document: Any) -> LifecycleSchedule:
    """Validate the semantic constraints not expressible in JSON Schema."""
    if not isinstance(document, Mapping):
        raise MetricUnavailable('lifecycle schedule must be a JSON object')
    required = {
        'lifecycle_schedule_schema_version',
        'requested_stamps_ns',
        'run_id',
    }
    if set(document) != required:
        missing = sorted(required - set(document))
        extra = sorted(set(document) - required)
        raise MetricUnavailable(
            f'lifecycle schedule keys mismatch: missing={missing}, extra={extra}'
        )
    version = document['lifecycle_schedule_schema_version']
    if isinstance(version, bool) or version != 1:
        raise MetricUnavailable('lifecycle_schedule_schema_version must equal 1')
    run_id = _bounded_text(document['run_id'], 'run_id', MAX_RUN_ID_BYTES)
    raw_stamps = document['requested_stamps_ns']
    if not isinstance(raw_stamps, list):
        raise MetricUnavailable('requested_stamps_ns must be an array')
    if not 1 <= len(raw_stamps) <= MAX_SCHEDULE_ROUNDS:
        raise MetricUnavailable(f'requested_stamps_ns must contain 1..{MAX_SCHEDULE_ROUNDS} rounds')
    stamps = tuple(
        _stamp_ns(value, f'requested_stamps_ns[{index}]') for index, value in enumerate(raw_stamps)
    )
    if any(current <= previous for previous, current in pairwise(stamps)):
        raise MetricUnavailable('requested_stamps_ns must be strictly increasing')
    return LifecycleSchedule(run_id=run_id, requested_stamps_ns=stamps)


def load_schedule(path: str | Path) -> LifecycleSchedule:
    """Read one bounded UTF-8 JSON schedule and reject duplicate object keys."""
    source = Path(path)
    try:
        with source.open('rb') as stream:
            payload = stream.read(MAX_SCHEDULE_BYTES + 1)
    except OSError as exc:
        raise MetricUnavailable(f'cannot read lifecycle schedule: {exc}') from exc
    if len(payload) > MAX_SCHEDULE_BYTES:
        raise MetricUnavailable(
            f'lifecycle schedule exceeds its {MAX_SCHEDULE_BYTES}-byte file bound'
        )
    try:
        document = json.loads(payload.decode('utf-8'), object_pairs_hook=_reject_duplicate_keys)
    except UnicodeDecodeError as exc:
        raise MetricUnavailable('lifecycle schedule must be UTF-8') from exc
    except json.JSONDecodeError as exc:
        raise MetricUnavailable(f'lifecycle schedule is not valid JSON: {exc.msg}') from exc
    return validate_schedule(document)


@dataclass(frozen=True)
class SampleRequest:
    """One immutable service request generated from a schedule round."""

    token: str
    round_index: int
    node: str
    requested_stamp_ns: int
    request_stamp_ns: int | None
    deadline_wall_ns: int


class LifecycleSamplerCore:
    """ROS-independent bounded state machine for lifecycle sampling."""

    def __init__(self, schedule: LifecycleSchedule, *, request_timeout_ns: int) -> None:
        """Initialize exact schedule, capacity, and request timeout policy."""
        if isinstance(request_timeout_ns, bool) or not isinstance(request_timeout_ns, int):
            raise ValueError('request_timeout_ns must be an integer')
        if request_timeout_ns <= 0:
            raise ValueError('request_timeout_ns must be positive')
        self.schedule = schedule
        self.request_timeout_ns = request_timeout_ns
        self.records = PrefixBuffer[dict[str, Any]](
            name='lifecycle_samples',
            capacity=MAX_LIFECYCLE_SAMPLES,
        )
        self.latest_clock_ns: int | None = None
        self.clock_count = 0
        self.clock_regression_count = 0
        self.request_count = 0
        self.response_count = 0
        self.success_count = 0
        self.error_count = 0
        self.missed_count = 0
        self.late_response_count = 0
        self._sequence = 0
        self._next_round_index = 0
        self._pending: dict[str, SampleRequest] = {}
        self._finalized = False
        self._stop_reason: str | None = None
        self._finished_steady_wall_ns: int | None = None
        self._started_steady_wall_ns: int | None = None
        self._runtime_error: str | None = None

    @property
    def finalized(self) -> bool:
        """Return whether all scheduled slots have been terminally accounted for."""
        return self._finalized

    @property
    def pending_tokens(self) -> tuple[str, ...]:
        """Return pending request tokens in deterministic insertion order."""
        return tuple(self._pending)

    def set_started_wall_ns(self, stamp_ns: int) -> None:
        """Record the process start stamp exactly once."""
        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int) or stamp_ns < 0:
            raise ValueError('started wall stamp must be a nonnegative integer')
        if self._started_steady_wall_ns is not None:
            raise RuntimeError('started wall stamp is already set')
        self._started_steady_wall_ns = stamp_ns

    def observe_clock(self, stamp_ns: int, *, wall_ns: int) -> list[SampleRequest]:
        """Advance due rounds without blocking and return their service requests."""
        stamp = _stamp_ns(stamp_ns, 'clock.stamp_ns')
        if isinstance(wall_ns, bool) or not isinstance(wall_ns, int) or wall_ns < 0:
            raise ValueError('wall_ns must be a nonnegative integer')
        if self._finalized:
            return []
        self.clock_count += 1
        if self.latest_clock_ns is not None and stamp < self.latest_clock_ns:
            self.clock_regression_count += 1
            self.latest_clock_ns = stamp
            return []
        self.latest_clock_ns = stamp
        due: list[SampleRequest] = []
        while (
            self._next_round_index < len(self.schedule.requested_stamps_ns)
            and self.schedule.requested_stamps_ns[self._next_round_index] <= stamp
        ):
            round_index = self._next_round_index
            requested_stamp = self.schedule.requested_stamps_ns[round_index]
            for node in NAV2_LIFECYCLE_NODES:
                token = f'{round_index}:{node}'
                request = SampleRequest(
                    token=token,
                    round_index=round_index,
                    node=node,
                    requested_stamp_ns=requested_stamp,
                    request_stamp_ns=stamp,
                    deadline_wall_ns=wall_ns + self.request_timeout_ns,
                )
                self._pending[token] = request
                due.append(request)
            self.request_count += len(NAV2_LIFECYCLE_NODES)
            self._next_round_index += 1
        return due

    def _append_record(
        self,
        request: SampleRequest,
        *,
        response_stamp_ns: int | None,
        state_id: int | None,
        state_label: str | None,
        success: bool,
        error: str | None,
        missed: bool,
    ) -> None:
        self._sequence += 1
        record = {
            'collector_sequence': self._sequence,
            'error': error,
            'missed': missed,
            'node': request.node,
            'request_stamp_ns': request.request_stamp_ns,
            'requested_stamp_ns': request.requested_stamp_ns,
            'response_stamp_ns': response_stamp_ns,
            'round_index': request.round_index,
            'state_id': state_id,
            'state_label': state_label,
            'success': success,
        }
        self.records.append(
            record,
            sequence=self._sequence,
            stamp_ns=response_stamp_ns,
        )

    def _complete_error(
        self,
        request: SampleRequest,
        error: str,
        *,
        response_stamp_ns: int | None,
        response_received: bool,
        missed: bool = False,
    ) -> None:
        bounded_error = _bounded_text(error, 'error', MAX_ERROR_BYTES)
        if response_stamp_ns is not None:
            response_stamp_ns = _stamp_ns(response_stamp_ns, 'response_stamp_ns')
        self._append_record(
            request,
            response_stamp_ns=response_stamp_ns,
            state_id=None,
            state_label=None,
            success=False,
            error=bounded_error,
            missed=missed,
        )
        self.response_count += int(response_received)
        self.error_count += 1
        self.missed_count += int(missed)

    def record_response(
        self,
        token: str,
        *,
        response_stamp_ns: int,
        state_id: Any,
        state_label: Any,
    ) -> bool:
        """Resolve one pending request with a validated lifecycle response."""
        request = self._pending.pop(token, None)
        if request is None:
            self.late_response_count += 1
            return False
        try:
            response_stamp = _stamp_ns(response_stamp_ns, 'response_stamp_ns')
            if isinstance(state_id, bool) or not isinstance(state_id, int):
                raise MetricUnavailable('state_id must be an integer')
            if not 0 <= state_id <= 255:
                raise MetricUnavailable('state_id must be in [0, 255]')
            label = _bounded_text(state_label, 'state_label', MAX_STATE_LABEL_BYTES)
        except MetricUnavailable as exc:
            self._complete_error(
                request,
                f'invalid_response:{exc}',
                response_stamp_ns=self.latest_clock_ns,
                response_received=True,
            )
            return False
        self._append_record(
            request,
            response_stamp_ns=response_stamp,
            state_id=state_id,
            state_label=label,
            success=True,
            error=None,
            missed=False,
        )
        self.response_count += 1
        self.success_count += 1
        return True

    def record_error(
        self,
        token: str,
        error: str,
        *,
        response_stamp_ns: int | None,
        response_received: bool = False,
    ) -> bool:
        """Resolve one pending request with an explicit bounded failure."""
        request = self._pending.pop(token, None)
        if request is None:
            self.late_response_count += 1
            return False
        self._complete_error(
            request,
            error,
            response_stamp_ns=response_stamp_ns,
            response_received=response_received,
        )
        return True

    def expire_requests(self, wall_ns: int) -> tuple[str, ...]:
        """Resolve every wall-deadline-expired request and return its token."""
        if isinstance(wall_ns, bool) or not isinstance(wall_ns, int) or wall_ns < 0:
            raise ValueError('wall_ns must be a nonnegative integer')
        expired = tuple(
            token for token, request in self._pending.items() if request.deadline_wall_ns <= wall_ns
        )
        for token in expired:
            self.record_error(
                token,
                'request_wall_timeout',
                response_stamp_ns=self.latest_clock_ns,
            )
        return expired

    def mark_runtime_error(self, error: str) -> None:
        """Retain one bounded process-level error for the diagnostic snapshot."""
        if not isinstance(error, str) or not error:
            error = 'unspecified_runtime_error'
        encoded = error.encode('utf-8')[:MAX_ERROR_BYTES]
        self._runtime_error = encoded.decode('utf-8', errors='ignore') or 'runtime_error'

    def finalize(self, stop_reason: str, *, finished_wall_ns: int) -> tuple[str, ...]:
        """Account for pending and unreached slots without overwriting the prefix."""
        allowed_reasons = {'stop_file', 'wall_timeout', 'runtime_shutdown', 'runtime_error'}
        if stop_reason not in allowed_reasons:
            raise ValueError(f'unsupported stop reason: {stop_reason}')
        if isinstance(finished_wall_ns, bool) or not isinstance(finished_wall_ns, int):
            raise ValueError('finished_wall_ns must be an integer')
        if finished_wall_ns < 0:
            raise ValueError('finished_wall_ns must be nonnegative')
        if self._finalized:
            return ()
        canceled_tokens = tuple(self._pending)
        for token in canceled_tokens:
            self.record_error(
                token,
                f'response_pending_on_{stop_reason}',
                response_stamp_ns=self.latest_clock_ns,
            )
        while self._next_round_index < len(self.schedule.requested_stamps_ns):
            round_index = self._next_round_index
            requested_stamp = self.schedule.requested_stamps_ns[round_index]
            for node in NAV2_LIFECYCLE_NODES:
                request = SampleRequest(
                    token=f'{round_index}:{node}',
                    round_index=round_index,
                    node=node,
                    requested_stamp_ns=requested_stamp,
                    request_stamp_ns=None,
                    deadline_wall_ns=finished_wall_ns,
                )
                self._complete_error(
                    request,
                    f'requested_stamp_not_reached_before_{stop_reason}',
                    response_stamp_ns=self.latest_clock_ns,
                    response_received=False,
                    missed=True,
                )
            self._next_round_index += 1
        self._finished_steady_wall_ns = finished_wall_ns
        self._stop_reason = stop_reason
        self._finalized = True
        return canceled_tokens

    def snapshot(self, *, sampler_node: str = 'lifecycle_sampler') -> dict[str, Any]:
        """Project detailed evidence and the analysis-ready successful samples."""
        records = list(self.records.items)
        errors = [
            {
                'collector_sequence': record['collector_sequence'],
                'error': record['error'],
                'node': record['node'],
                'requested_stamp_ns': record['requested_stamp_ns'],
                'round_index': record['round_index'],
            }
            for record in records
            if not record['success']
        ]
        samples = [
            {
                'collector_sequence': record['collector_sequence'],
                'node': record['node'],
                'stamp_ns': record['response_stamp_ns'],
                'state': record['state_label'],
            }
            for record in records
            if record['success']
        ]
        error_counts = Counter(record['error'] for record in records if record['error'] is not None)
        expected_count = self.schedule.expected_sample_count
        buffer_quality = self.records.quality.as_dict()
        complete = (
            self._finalized
            and self._stop_reason == 'stop_file'
            and self.success_count == expected_count
            and self.error_count == 0
            and self.missed_count == 0
            and self.clock_regression_count == 0
            and not buffer_quality['overflowed']
            and self._runtime_error is None
        )
        return {
            'capacity': {
                'round_capacity': MAX_SCHEDULE_ROUNDS,
                'sample_capacity': MAX_LIFECYCLE_SAMPLES,
                'scheduled_round_count': len(self.schedule.requested_stamps_ns),
                'scheduled_sample_count': expected_count,
            },
            'clock': {
                'count': self.clock_count,
                'latest_stamp_ns': self.latest_clock_ns,
                'regression_count': self.clock_regression_count,
            },
            'errors': errors,
            'identity': {
                'run_id': self.schedule.run_id,
                'sampler_node': sampler_node,
                'schedule_sha256': self.schedule.sha256,
            },
            'lifecycle_snapshot_schema_version': 1,
            'quality': {
                'collector_overflow': buffer_quality['overflowed'],
                'complete': complete,
                'error_count': self.error_count,
                'error_counts': dict(sorted(error_counts.items())),
                'finished_steady_wall_ns': self._finished_steady_wall_ns,
                'first_overflow_sequence': buffer_quality['first_overflow_sequence'],
                'first_overflow_stamp_ns': buffer_quality['first_overflow_stamp_ns'],
                'late_response_count': self.late_response_count,
                'missed_count': self.missed_count,
                'overflow_count': buffer_quality['overflow_count'],
                'pending_count': len(self._pending),
                'request_count': self.request_count,
                'response_count': self.response_count,
                'retained_count': buffer_quality['retained_count'],
                'runtime_error': self._runtime_error,
                'started_steady_wall_ns': self._started_steady_wall_ns,
                'stop_reason': self._stop_reason,
                'success_count': self.success_count,
            },
            'records': records,
            'samples': samples,
            'schedule': {
                'nodes': list(NAV2_LIFECYCLE_NODES),
                'requested_stamps_ns': list(self.schedule.requested_stamps_ns),
                'service_names': [lifecycle_service_name(node) for node in NAV2_LIFECYCLE_NODES],
            },
        }


def _clock_stamp_ns(message: Clock) -> int:
    return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)


def _clock_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class LifecycleSamplerNode(Node):
    """Use one DDS participant to query the exact Nav2 lifecycle allowlist."""

    def __init__(self, core: LifecycleSamplerCore) -> None:
        """Create nine read-only clients and the global simulation clock observer."""
        super().__init__(
            'lifecycle_sampler',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
        )
        self.core = core
        self._clients = {
            node: self.create_client(GetState, lifecycle_service_name(node))
            for node in NAV2_LIFECYCLE_NODES
        }
        self._futures: dict[str, Any] = {}
        self._armed = False
        self._latest_observed_clock_ns: int | None = None
        self._clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self._on_clock,
            _clock_qos(),
        )

    def _on_clock(self, message: Clock) -> None:
        self._latest_observed_clock_ns = _clock_stamp_ns(message)
        if not self._armed:
            return
        self._dispatch_due(self._latest_observed_clock_ns)

    def _dispatch_due(self, stamp_ns: int) -> None:
        requests = self.core.observe_clock(stamp_ns, wall_ns=time.monotonic_ns())
        for request in requests:
            client = self._clients[request.node]
            if not client.service_is_ready():
                self.core.record_error(
                    request.token,
                    'service_unavailable',
                    response_stamp_ns=self.core.latest_clock_ns,
                )
                continue
            future = client.call_async(GetState.Request())
            self._futures[request.token] = future
            future.add_done_callback(
                lambda completed, token=request.token: self._on_response(token, completed)
            )

    def all_services_ready(self) -> bool:
        """Return whether every exact allowlisted read-only service is discovered."""
        return all(client.service_is_ready() for client in self._clients.values())

    def arm(self) -> None:
        """Enable schedule dispatch after the complete service-ready gate."""
        if self._armed:
            return
        if not self.all_services_ready():
            raise RuntimeError('cannot arm before all lifecycle services are ready')
        self._armed = True
        if self._latest_observed_clock_ns is not None:
            self._dispatch_due(self._latest_observed_clock_ns)

    def _on_response(self, token: str, future: Any) -> None:
        if self._futures.pop(token, None) is None:
            return
        try:
            response = future.result()
            if response is None:
                raise RuntimeError('empty service response')
            self.core.record_response(
                token,
                response_stamp_ns=self.core.latest_clock_ns,
                state_id=int(response.current_state.id),
                state_label=response.current_state.label,
            )
        except Exception as exc:  # rclpy futures surface transport failures here.
            self.core.record_error(
                token,
                f'service_response_error:{type(exc).__name__}',
                response_stamp_ns=self.core.latest_clock_ns,
                response_received=True,
            )

    def expire_requests(self, wall_ns: int) -> None:
        """Cancel runtime futures whose pure-core wall deadline expired."""
        for token in self.core.expire_requests(wall_ns):
            future = self._futures.pop(token, None)
            if future is not None:
                future.cancel()

    def cancel_pending(self, tokens: Sequence[str]) -> None:
        """Cancel every runtime future finalized by a stop condition."""
        for token in tokens:
            future = self._futures.pop(token, None)
            if future is not None:
                future.cancel()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Capture bounded observer-only Nav2 lifecycle snapshots.',
    )
    parser.add_argument('--schedule', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--ready-file', required=True, type=Path)
    parser.add_argument('--stop-file', required=True, type=Path)
    parser.add_argument('--wall-timeout-s', type=float, default=360.0)
    parser.add_argument('--request-timeout-s', type=float, default=2.0)
    return parser


def _validate_cli_paths(output: Path, ready_file: Path, stop_file: Path) -> None:
    paths = {'output': output, 'ready': ready_file, 'stop': stop_file}
    resolved = [path.resolve(strict=False) for path in paths.values()]
    if len(set(resolved)) != len(resolved):
        raise SystemExit('output, ready-file, and stop-file paths must be distinct')
    for label, path in paths.items():
        if path.exists():
            raise SystemExit(f'stale {label} file already exists: {path}')


def main(argv: Sequence[str] | None = None) -> int:
    """Run the sampler through an explicit stop or bounded wall deadline."""
    raw_arguments = list(sys.argv) if argv is None else [sys.argv[0], *argv]
    arguments = _parser().parse_args(remove_ros_args(args=raw_arguments)[1:])
    for label, value in (
        ('--wall-timeout-s', arguments.wall_timeout_s),
        ('--request-timeout-s', arguments.request_timeout_s),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise SystemExit(f'{label} must be finite and positive')
    _validate_cli_paths(arguments.output, arguments.ready_file, arguments.stop_file)
    try:
        schedule = load_schedule(arguments.schedule)
    except MetricUnavailable as exc:
        print(f'lifecycle sampler schedule error: {exc}', file=sys.stderr)
        return 24

    wall_timeout_ns = int(arguments.wall_timeout_s * 1_000_000_000)
    request_timeout_ns = int(arguments.request_timeout_s * 1_000_000_000)
    if wall_timeout_ns <= 0 or request_timeout_ns <= 0:
        raise SystemExit('timeout values must each be at least one nanosecond')
    rclpy.init(args=raw_arguments)
    node: LifecycleSamplerNode | None = None
    core = LifecycleSamplerCore(schedule, request_timeout_ns=request_timeout_ns)
    started_wall_ns = time.monotonic_ns()
    core.set_started_wall_ns(started_wall_ns)
    stop_reason = 'runtime_shutdown'
    exit_code = 23
    try:
        node = LifecycleSamplerNode(core)
        deadline_wall_ns = started_wall_ns + wall_timeout_ns
        services_ready = False
        while rclpy.ok() and not services_ready:
            now_wall_ns = time.monotonic_ns()
            if arguments.stop_file.exists():
                stop_reason = 'stop_file'
                exit_code = 0
                break
            if now_wall_ns >= deadline_wall_ns:
                stop_reason = 'wall_timeout'
                exit_code = 20
                break
            services_ready = node.all_services_ready()
            if not services_ready:
                rclpy.spin_once(node, timeout_sec=0.05)
        if services_ready:
            node.arm()
            write_json_atomic(
                {
                    'identity': {
                        'run_id': schedule.run_id,
                        'schedule_sha256': schedule.sha256,
                    },
                    'node_name': node.get_fully_qualified_name(),
                    'service_names': [
                        lifecycle_service_name(name) for name in NAV2_LIFECYCLE_NODES
                    ],
                    'started_steady_wall_ns': started_wall_ns,
                    'status': 'READY',
                },
                arguments.ready_file,
                maximum_bytes=READY_FILE_MAX_BYTES,
            )
        while rclpy.ok() and services_ready:
            now_wall_ns = time.monotonic_ns()
            node.expire_requests(now_wall_ns)
            if arguments.stop_file.exists():
                stop_reason = 'stop_file'
                exit_code = 0
                break
            if now_wall_ns >= deadline_wall_ns:
                stop_reason = 'wall_timeout'
                exit_code = 20
                break
            rclpy.spin_once(node, timeout_sec=0.05)
        if not rclpy.ok() and not arguments.stop_file.exists():
            stop_reason = 'runtime_shutdown'
            exit_code = 23
    except ArtifactError as exc:
        print(f'lifecycle sampler artifact error: {exc}', file=sys.stderr)
        return 22
    except Exception as exc:  # Preserve a canonical diagnostic on unexpected runtime failure.
        stop_reason = 'runtime_error'
        exit_code = 25
        core.mark_runtime_error(f'{type(exc).__name__}:{exc}')
    finally:
        finished_wall_ns = time.monotonic_ns()
        canceled = core.finalize(stop_reason, finished_wall_ns=finished_wall_ns)
        if node is not None:
            node.cancel_pending(canceled)
        try:
            write_json_atomic(
                core.snapshot(
                    sampler_node=(
                        node.get_fully_qualified_name() if node is not None else 'lifecycle_sampler'
                    )
                ),
                arguments.output,
            )
        except ArtifactError as exc:
            print(f'lifecycle sampler artifact error: {exc}', file=sys.stderr)
            exit_code = 22
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code == 0 and not core.snapshot()['quality']['complete']:
        return 21
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
