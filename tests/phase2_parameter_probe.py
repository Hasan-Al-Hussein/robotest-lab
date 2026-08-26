#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Boundedly prove ``use_sim_time`` on many ROS 2 nodes with one participant."""

from __future__ import annotations

import argparse
import contextlib
import json
import math
import os
import re
import sys
import tempfile
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, ParameterValue
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient

PARAMETER_NAME = 'use_sim_time'
PROBE_NODE_NAME = 'phase2_parameter_probe'
PROBE_NAMESPACE = '/robotest/evidence'
SCHEMA_VERSION = 1
SPIN_QUANTUM_S = 0.05
DEFAULT_DISCOVERY_GRACE_S = 3.0
DEFAULT_REQUEST_TIMEOUT_S = 2.0
MAX_REQUEST_ATTEMPTS = 2
RETRY_BACKOFF_S = 0.25

EXIT_PASS = 0
EXIT_PROBE_FAILURE = 1
EXIT_INTERNAL_ERROR = 3

_ROS_NAME_TOKEN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


@dataclass
class TargetRuntime:
    """Non-serializable client state for one remote node."""

    client: AsyncParameterClient


class ProbeAbortError(RuntimeError):
    """Global probe abort carrying a stable machine-readable failure kind."""

    def __init__(self, kind: str, message: str) -> None:
        super().__init__(message)
        self.kind = kind


def normalize_namespace(namespace: str) -> str:
    """Return a canonical absolute namespace after strict ROS-name validation."""
    stripped = namespace.strip('/')
    if not stripped:
        return '/'
    tokens = stripped.split('/')
    if any(_ROS_NAME_TOKEN.fullmatch(token) is None for token in tokens):
        raise ValueError(f'invalid ROS namespace: {namespace!r}')
    return '/' + '/'.join(tokens)


def validate_relative_node_name(name: str) -> str:
    """Validate a node name intended to be resolved directly below a namespace."""
    if _ROS_NAME_TOKEN.fullmatch(name) is None:
        raise ValueError(f'invalid relative ROS node name: {name!r}')
    return name


def full_node_name(namespace: str, node_name: str) -> str:
    """Join one validated relative node name to a canonical namespace."""
    canonical_namespace = normalize_namespace(namespace)
    validated_name = validate_relative_node_name(node_name)
    if canonical_namespace == '/':
        return f'/{validated_name}'
    return f'{canonical_namespace}/{validated_name}'


def parameter_type_name(type_id: int) -> str:
    """Return a stable symbolic name for an rcl_interfaces parameter type."""
    names = {
        ParameterType.PARAMETER_NOT_SET: 'PARAMETER_NOT_SET',
        ParameterType.PARAMETER_BOOL: 'PARAMETER_BOOL',
        ParameterType.PARAMETER_INTEGER: 'PARAMETER_INTEGER',
        ParameterType.PARAMETER_DOUBLE: 'PARAMETER_DOUBLE',
        ParameterType.PARAMETER_STRING: 'PARAMETER_STRING',
        ParameterType.PARAMETER_BYTE_ARRAY: 'PARAMETER_BYTE_ARRAY',
        ParameterType.PARAMETER_BOOL_ARRAY: 'PARAMETER_BOOL_ARRAY',
        ParameterType.PARAMETER_INTEGER_ARRAY: 'PARAMETER_INTEGER_ARRAY',
        ParameterType.PARAMETER_DOUBLE_ARRAY: 'PARAMETER_DOUBLE_ARRAY',
        ParameterType.PARAMETER_STRING_ARRAY: 'PARAMETER_STRING_ARRAY',
    }
    return names.get(type_id, f'UNKNOWN_PARAMETER_TYPE_{type_id}')


def serialize_result(result: dict[str, Any]) -> str:
    """Serialize canonical strict JSON, rejecting NaN and infinities."""
    return json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + '\n'


def atomic_write_text(path: Path, content: str) -> None:
    """Replace a text file atomically and remove its temporary on failure."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, pending_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f'.{path.name}.',
        suffix='.pending',
        text=True,
    )
    pending_path = Path(pending_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8', newline='\n') as stream:
            descriptor = -1
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        pending_path.replace(path)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        pending_path.unlink(missing_ok=True)


def watched_process_alive(pid: int) -> bool:
    """Return whether a positive PID still identifies a live or inaccessible process."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def evaluate_parameter_responses(
    describe_response: Any,
    get_response: Any,
) -> tuple[dict[str, Any], list[str]]:
    """Validate exact declaration, type, and true value from two service responses."""
    descriptors = list(describe_response.descriptors)
    values = list(get_response.values)
    evidence: dict[str, Any] = {
        'declared': False,
        'descriptor_count': len(descriptors),
        'descriptor_name': None,
        'descriptor_type': None,
        'descriptor_type_name': None,
        'value': None,
        'value_count': len(values),
        'value_type': None,
        'value_type_name': None,
    }
    failures: list[str] = []

    if len(descriptors) != 1:
        failures.append(
            f'describe_parameters returned {len(descriptors)} descriptors; expected exactly 1'
        )
    else:
        descriptor = descriptors[0]
        descriptor_type = int(descriptor.type)
        evidence.update(
            {
                'declared': (
                    descriptor.name == PARAMETER_NAME
                    and descriptor_type != ParameterType.PARAMETER_NOT_SET
                ),
                'descriptor_name': descriptor.name,
                'descriptor_type': descriptor_type,
                'descriptor_type_name': parameter_type_name(descriptor_type),
            }
        )
        if descriptor.name != PARAMETER_NAME:
            failures.append(f'descriptor name is {descriptor.name!r}; expected {PARAMETER_NAME!r}')
        if descriptor_type == ParameterType.PARAMETER_NOT_SET:
            failures.append(f'{PARAMETER_NAME} is not declared')
        elif descriptor_type != ParameterType.PARAMETER_BOOL:
            failures.append(
                f'{PARAMETER_NAME} descriptor type is '
                f'{parameter_type_name(descriptor_type)}; expected PARAMETER_BOOL'
            )

    if len(values) != 1:
        failures.append(f'get_parameters returned {len(values)} values; expected exactly 1')
    else:
        value = values[0]
        value_type = int(value.type)
        evidence.update(
            {
                'value': bool(value.bool_value)
                if value_type == ParameterType.PARAMETER_BOOL
                else None,
                'value_type': value_type,
                'value_type_name': parameter_type_name(value_type),
            }
        )
        if value_type != ParameterType.PARAMETER_BOOL:
            failures.append(
                f'{PARAMETER_NAME} value type is {parameter_type_name(value_type)}; '
                'expected PARAMETER_BOOL'
            )
        elif value.bool_value is not True:
            failures.append(f'{PARAMETER_NAME} is false; expected true')

    return evidence, failures


def initial_result(
    namespace: str,
    names: list[str],
    wall_timeout: float,
    watch_pid: int | None,
    discovery_grace: float = DEFAULT_DISCOVERY_GRACE_S,
    request_timeout: float = DEFAULT_REQUEST_TIMEOUT_S,
    max_request_attempts: int = MAX_REQUEST_ATTEMPTS,
) -> dict[str, Any]:
    """Build the JSON-safe public state before DDS discovery begins."""
    results: dict[str, Any] = {}
    for name in names:
        remote = full_node_name(namespace, name)
        results[name] = {
            'describe_service': f'{remote}/describe_parameters',
            'describe_attempts': [],
            'failures': [],
            'full_node_name': remote,
            'get_service': f'{remote}/get_parameters',
            'get_attempts': [],
            'parameter_services_ready': False,
            'requests_completed_after_wall_seconds': None,
            'requests_issued': False,
            'services_ready_after_wall_seconds': None,
            'status': 'PENDING',
            'validation': None,
        }
    return {
        'elapsed_wall_seconds': 0.0,
        'all_services_ready_after_wall_seconds': None,
        'discovery_grace_seconds': discovery_grace,
        'failure': None,
        'failure_kind': None,
        'namespace': normalize_namespace(namespace),
        'parameter': PARAMETER_NAME,
        'participant': f'{PROBE_NAMESPACE}/{PROBE_NODE_NAME}',
        'required': {
            'declared': True,
            'type': 'PARAMETER_BOOL',
            'value': True,
        },
        'results': results,
        'request_attempt_timeout_seconds': request_timeout,
        'max_request_attempts': max_request_attempts,
        'requests_started_after_wall_seconds': None,
        'schema_version': SCHEMA_VERSION,
        'verdict': 'FAIL',
        'wall_timeout_seconds': wall_timeout,
        'watch_pid': watch_pid,
    }


def _future_error(future: Any) -> tuple[Any | None, str | None]:
    """Consume one completed rclpy future without leaking middleware exceptions."""
    try:
        response = future.result()
    except Exception as error:  # pragma: no cover - middleware-specific exception types
        return None, f'{type(error).__name__}: {error}'
    if response is None:
        return None, 'parameter service future completed without a response'
    return response, None


def _cancel_future(future: Any) -> None:
    """Best-effort cancellation of a timed-out local future."""
    with contextlib.suppress(Exception):
        future.cancel()


def _spin_until(
    executor: SingleThreadedExecutor,
    deadline: float,
    abort_check: Callable[[], None],
    clock: Callable[[], float],
) -> None:
    """Spin with short wall bounds until a deadline while honoring global aborts."""
    while clock() < deadline:
        abort_check()
        remaining = deadline - clock()
        if remaining <= 0.0:
            break
        executor.spin_once(timeout_sec=min(SPIN_QUANTUM_S, remaining))


def request_with_retries(
    executor: SingleThreadedExecutor,
    request: Callable[[], Any],
    services_ready: Callable[[], bool],
    operation: str,
    *,
    request_timeout: float,
    max_attempts: int,
    global_deadline: float,
    probe_started: float,
    abort_check: Callable[[], None],
    clock: Callable[[], float] = time.monotonic,
    attempts: list[dict[str, Any]] | None = None,
) -> tuple[Any | None, list[dict[str, Any]], str | None]:
    """Issue one request at a time with explicit wall timeout and bounded retries."""
    if not math.isfinite(request_timeout) or request_timeout <= 0.0:
        raise ValueError('request_timeout must be finite and positive')
    if not 1 <= max_attempts <= MAX_REQUEST_ATTEMPTS:
        raise ValueError(f'max_attempts must be between 1 and {MAX_REQUEST_ATTEMPTS}')
    attempt_log: list[dict[str, Any]] = attempts if attempts is not None else []
    last_failure: str | None = None

    for attempt_number in range(1, max_attempts + 1):
        abort_check()
        attempt_started = clock()
        attempt: dict[str, Any] = {
            'attempt': attempt_number,
            'completed_after_wall_seconds': None,
            'duration_wall_seconds': None,
            'failure': None,
            'request_sent': False,
            'started_after_wall_seconds': round(attempt_started - probe_started, 9),
            'status': 'PENDING',
        }
        attempt_log.append(attempt)
        future = None

        if not services_ready():
            last_failure = f'{operation} parameter service was not ready'
            attempt['failure'] = last_failure
            attempt['status'] = 'SERVICE_UNAVAILABLE'
        else:
            try:
                future = request()
                attempt['request_sent'] = True
            except Exception as error:  # pragma: no cover - middleware-specific
                last_failure = f'{type(error).__name__}: {error}'
                attempt['failure'] = last_failure
                attempt['status'] = 'REQUEST_ERROR'

        if future is not None:
            attempt_deadline = min(attempt_started + request_timeout, global_deadline)
            while not future.done():
                abort_check()
                remaining = attempt_deadline - clock()
                if remaining <= 0.0:
                    _cancel_future(future)
                    last_failure = (
                        f'{operation} attempt {attempt_number} exceeded its '
                        f'{request_timeout:.3f}s wall timeout'
                    )
                    attempt['failure'] = last_failure
                    attempt['status'] = 'TIMEOUT'
                    break
                executor.spin_once(timeout_sec=min(SPIN_QUANTUM_S, remaining))
            else:
                response, error = _future_error(future)
                if error is None:
                    completed = clock()
                    attempt['completed_after_wall_seconds'] = round(
                        completed - probe_started,
                        9,
                    )
                    attempt['duration_wall_seconds'] = round(completed - attempt_started, 9)
                    attempt['status'] = 'PASS'
                    return response, attempt_log, None
                last_failure = error
                attempt['failure'] = error
                attempt['status'] = 'RESPONSE_ERROR'

        completed = clock()
        attempt['completed_after_wall_seconds'] = round(completed - probe_started, 9)
        attempt['duration_wall_seconds'] = round(completed - attempt_started, 9)
        if attempt_number < max_attempts:
            retry_deadline = min(clock() + RETRY_BACKOFF_S, global_deadline)
            _spin_until(executor, retry_deadline, abort_check, clock)

    return None, attempt_log, last_failure


def probe_parameters(
    probe: Node,
    executor: SingleThreadedExecutor,
    namespace: str,
    names: list[str],
    wall_timeout: float,
    watch_pid: int | None,
    discovery_grace: float = DEFAULT_DISCOVERY_GRACE_S,
    request_timeout: float = DEFAULT_REQUEST_TIMEOUT_S,
    max_request_attempts: int = MAX_REQUEST_ATTEMPTS,
) -> dict[str, Any]:
    """Discover all targets, stabilize DDS, then query with one in-flight request."""
    for field_name, value in (
        ('wall_timeout', wall_timeout),
        ('discovery_grace', discovery_grace),
        ('request_timeout', request_timeout),
    ):
        if not math.isfinite(value) or value <= 0.0:
            raise ValueError(f'{field_name} must be finite and positive')
    if not 1 <= max_request_attempts <= MAX_REQUEST_ATTEMPTS:
        raise ValueError(f'max_request_attempts must be between 1 and {MAX_REQUEST_ATTEMPTS}')
    result = initial_result(
        namespace,
        names,
        wall_timeout,
        watch_pid,
        discovery_grace,
        request_timeout,
        max_request_attempts,
    )
    result['participant'] = probe.get_fully_qualified_name()
    runtimes = {
        name: TargetRuntime(AsyncParameterClient(probe, full_node_name(namespace, name)))
        for name in names
    }
    started = time.monotonic()
    deadline = started + wall_timeout
    global_failure_kind: str | None = None
    global_failure: str | None = None

    def abort_check() -> None:
        """Raise when a global wall, owner, or ROS-context bound is crossed."""
        now = time.monotonic()
        if watch_pid is not None and not watched_process_alive(watch_pid):
            raise ProbeAbortError(
                'watch_pid_exited',
                f'watched process {watch_pid} exited before probe completion',
            )
        if not rclpy.ok():
            raise ProbeAbortError(
                'rclpy_shutdown',
                'rclpy context shut down before probe completion',
            )
        if now >= deadline:
            raise ProbeAbortError(
                'wall_timeout',
                f'probe exceeded its {wall_timeout:.3f}s monotonic wall deadline',
            )

    try:
        stable_services_since: float | None = None
        while True:
            abort_check()
            now = time.monotonic()
            elapsed = now - started
            all_ready = True
            for name, runtime in runtimes.items():
                public = result['results'][name]
                ready = runtime.client.services_are_ready()
                all_ready = all_ready and ready
                if ready and not public['parameter_services_ready']:
                    public['parameter_services_ready'] = True
                    public['services_ready_after_wall_seconds'] = round(elapsed, 9)

            if all_ready:
                if stable_services_since is None:
                    stable_services_since = now
                    if result['all_services_ready_after_wall_seconds'] is None:
                        result['all_services_ready_after_wall_seconds'] = round(elapsed, 9)
                if now - stable_services_since >= discovery_grace:
                    break
            else:
                stable_services_since = None

            executor.spin_once(
                timeout_sec=min(SPIN_QUANTUM_S, max(0.0, deadline - time.monotonic()))
            )

        result['requests_started_after_wall_seconds'] = round(time.monotonic() - started, 9)

        for name in names:
            abort_check()
            runtime = runtimes[name]
            public = result['results'][name]
            public['requests_issued'] = True

            describe_response, _, describe_error = request_with_retries(
                executor,
                lambda runtime=runtime: runtime.client.describe_parameters([PARAMETER_NAME]),
                runtime.client.services_are_ready,
                'describe_parameters',
                request_timeout=request_timeout,
                max_attempts=max_request_attempts,
                global_deadline=deadline,
                probe_started=started,
                abort_check=abort_check,
                attempts=public['describe_attempts'],
            )
            get_response, _, get_error = request_with_retries(
                executor,
                lambda runtime=runtime: runtime.client.get_parameters([PARAMETER_NAME]),
                runtime.client.services_are_ready,
                'get_parameters',
                request_timeout=request_timeout,
                max_attempts=max_request_attempts,
                global_deadline=deadline,
                probe_started=started,
                abort_check=abort_check,
                attempts=public['get_attempts'],
            )

            if describe_response is None:
                public['failures'].append(
                    'describe_parameters exhausted '
                    f'{max_request_attempts} attempts: {describe_error or "no response"}'
                )
            if get_response is None:
                public['failures'].append(
                    'get_parameters exhausted '
                    f'{max_request_attempts} attempts: {get_error or "no response"}'
                )
            if describe_response is not None and get_response is not None:
                validation, failures = evaluate_parameter_responses(
                    describe_response,
                    get_response,
                )
                public['validation'] = validation
                public['failures'].extend(failures)
            public['requests_completed_after_wall_seconds'] = round(
                time.monotonic() - started,
                9,
            )
            public['status'] = 'PASS' if not public['failures'] else 'FAIL'
    except ProbeAbortError as error:
        global_failure_kind = error.kind
        global_failure = str(error)

    elapsed = time.monotonic() - started
    result['elapsed_wall_seconds'] = round(elapsed, 9)
    if global_failure_kind is not None:
        result['failure_kind'] = global_failure_kind
        result['failure'] = global_failure
        for public in result['results'].values():
            if public['status'] == 'PENDING':
                public['status'] = (
                    'TIMEOUT' if global_failure_kind == 'wall_timeout' else 'INCOMPLETE'
                )
                if not public['parameter_services_ready']:
                    public['failures'].append('parameter services were not all discovered')
                elif public['requests_completed_after_wall_seconds'] is None:
                    public['failures'].append('parameter requests did not both complete')
    else:
        failed_nodes = [
            name for name, public in result['results'].items() if public['status'] != 'PASS'
        ]
        if failed_nodes:
            incomplete_nodes = [
                name for name in failed_nodes if result['results'][name]['validation'] is None
            ]
            result['failure_kind'] = (
                'parameter_query' if incomplete_nodes else 'parameter_validation'
            )
            result['failure'] = 'parameter proof failed for: ' + ', '.join(failed_nodes)

    if result['failure'] is None:
        result['verdict'] = 'PASS'
    return result


def per_node_text(public: dict[str, Any]) -> str:
    """Render grep-friendly compatibility evidence for one target."""
    if public['status'] == 'PASS':
        return 'Boolean value is: True\n'
    failures = '; '.join(public['failures']) or f'probe status is {public["status"]}'
    return 'Parameter validation failed: ' + failures.replace('\n', ' ') + '\n'


def write_evidence(
    result: dict[str, Any],
    output: Path,
    text_dir: Path | None,
    text_prefix: str,
) -> None:
    """Write optional per-node files first, then commit the canonical JSON record."""
    if text_dir is not None:
        for name, public in result['results'].items():
            atomic_write_text(text_dir / f'{text_prefix}{name}.txt', per_node_text(public))
    atomic_write_text(output, serialize_result(result))


def cleanup_ros(
    executor: SingleThreadedExecutor | None,
    nodes: list[Node],
) -> None:
    """Remove and destroy all owned ROS entities, even after a failed probe."""
    if executor is not None:
        for node in nodes:
            with contextlib.suppress(Exception):  # pragma: no cover - defensive cleanup
                executor.remove_node(node)
    for node in reversed(nodes):
        with contextlib.suppress(Exception):  # pragma: no cover - defensive cleanup
            node.destroy_node()
    if executor is not None:
        with contextlib.suppress(Exception):  # pragma: no cover - defensive cleanup
            executor.shutdown(timeout_sec=1.0)
    if rclpy.ok():
        with contextlib.suppress(Exception):  # pragma: no cover - defensive cleanup
            rclpy.shutdown()


def run_live_probe(
    namespace: str,
    names: list[str],
    wall_timeout: float,
    watch_pid: int | None,
    discovery_grace: float = DEFAULT_DISCOVERY_GRACE_S,
    request_timeout: float = DEFAULT_REQUEST_TIMEOUT_S,
    max_request_attempts: int = MAX_REQUEST_ATTEMPTS,
) -> dict[str, Any]:
    """Initialize one probe participant, run it, and clean up deterministically."""
    executor: SingleThreadedExecutor | None = None
    probe: Node | None = None
    rclpy.init()
    try:
        probe = Node(PROBE_NODE_NAME, namespace=PROBE_NAMESPACE)
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        return probe_parameters(
            probe,
            executor,
            namespace,
            names,
            wall_timeout,
            watch_pid,
            discovery_grace,
            request_timeout,
            max_request_attempts,
        )
    finally:
        cleanup_ros(executor, [probe] if probe is not None else [])


def run_self_test() -> int:
    """Exercise pure validation and serialization logic without creating a ROS graph."""

    class FakeClock:
        def __init__(self) -> None:
            self.now = 0.0

        def __call__(self) -> float:
            return self.now

    class FakeExecutor:
        def __init__(self, clock: FakeClock) -> None:
            self.clock = clock

        def spin_once(self, timeout_sec: float) -> None:
            assert 0.0 < timeout_sec <= SPIN_QUANTUM_S
            self.clock.now += timeout_sec

    class FakeFuture:
        def __init__(self, clock: FakeClock, done_after: float | None, response: Any) -> None:
            self.clock = clock
            self.done_at = math.inf if done_after is None else clock() + done_after
            self.response = response
            self.cancelled = False

        def done(self) -> bool:
            return self.clock() >= self.done_at

        def cancel(self) -> None:
            self.cancelled = True

        def result(self) -> Any:
            return self.response

    assert normalize_namespace('robotest/evidence/') == '/robotest/evidence'
    assert normalize_namespace('/') == '/'
    assert full_node_name('/robotest', 'map_server') == '/robotest/map_server'
    for invalid_name in ('', '/absolute', 'nested/name', '9starts_with_digit'):
        try:
            validate_relative_node_name(invalid_name)
        except ValueError:
            pass
        else:
            raise AssertionError(f'accepted invalid node name: {invalid_name!r}')

    descriptor = ParameterDescriptor(name=PARAMETER_NAME, type=ParameterType.PARAMETER_BOOL)
    true_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
    evidence, failures = evaluate_parameter_responses(
        type('DescribeResponse', (), {'descriptors': [descriptor]})(),
        type('GetResponse', (), {'values': [true_value]})(),
    )
    assert not failures
    assert evidence['declared'] is True
    assert evidence['value'] is True

    false_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False)
    _, failures = evaluate_parameter_responses(
        type('DescribeResponse', (), {'descriptors': [descriptor]})(),
        type('GetResponse', (), {'values': [false_value]})(),
    )
    assert failures == [f'{PARAMETER_NAME} is false; expected true']

    retry_clock = FakeClock()
    retry_executor = FakeExecutor(retry_clock)
    timed_out_future: FakeFuture | None = None
    request_count = 0
    expected_response = object()

    def timeout_then_succeed() -> FakeFuture:
        nonlocal request_count, timed_out_future
        request_count += 1
        if request_count == 1:
            timed_out_future = FakeFuture(retry_clock, None, None)
            return timed_out_future
        return FakeFuture(retry_clock, 0.0, expected_response)

    response, attempts, failure = request_with_retries(
        retry_executor,
        timeout_then_succeed,
        lambda: True,
        'get_parameters',
        request_timeout=0.1,
        max_attempts=2,
        global_deadline=2.0,
        probe_started=0.0,
        abort_check=lambda: None,
        clock=retry_clock,
    )
    assert response is expected_response
    assert failure is None
    assert [attempt['status'] for attempt in attempts] == ['TIMEOUT', 'PASS']
    assert timed_out_future is not None and timed_out_future.cancelled
    assert request_count == 2

    failure_clock = FakeClock()
    failure_executor = FakeExecutor(failure_clock)
    response, attempts, failure = request_with_retries(
        failure_executor,
        lambda: FakeFuture(failure_clock, None, None),
        lambda: True,
        'describe_parameters',
        request_timeout=0.1,
        max_attempts=2,
        global_deadline=2.0,
        probe_started=0.0,
        abort_check=lambda: None,
        clock=failure_clock,
    )
    assert response is None
    assert failure is not None
    assert [attempt['status'] for attempt in attempts] == ['TIMEOUT', 'TIMEOUT']
    try:
        request_with_retries(
            failure_executor,
            lambda: FakeFuture(failure_clock, None, None),
            lambda: True,
            'describe_parameters',
            request_timeout=0.1,
            max_attempts=3,
            global_deadline=2.0,
            probe_started=0.0,
            abort_check=lambda: None,
            clock=failure_clock,
        )
    except ValueError:
        pass
    else:
        raise AssertionError('accepted more than one retry')

    completed_invalid_clock = FakeClock()
    completed_invalid_executor = FakeExecutor(completed_invalid_clock)
    invalid_response = type('GetResponse', (), {'values': [false_value]})()
    response, attempts, failure = request_with_retries(
        completed_invalid_executor,
        lambda: FakeFuture(completed_invalid_clock, 0.0, invalid_response),
        lambda: True,
        'get_parameters',
        request_timeout=0.1,
        max_attempts=2,
        global_deadline=2.0,
        probe_started=0.0,
        abort_check=lambda: None,
        clock=completed_invalid_clock,
    )
    assert response is invalid_response and failure is None
    assert [attempt['status'] for attempt in attempts] == ['PASS']
    _, invalid_failures = evaluate_parameter_responses(
        type('DescribeResponse', (), {'descriptors': [descriptor]})(),
        response,
    )
    assert invalid_failures == [f'{PARAMETER_NAME} is false; expected true']

    try:
        serialize_result({'not_finite': math.nan})
    except ValueError:
        pass
    else:
        raise AssertionError('strict JSON serializer accepted NaN')
    print('phase2_parameter_probe self-test PASS')
    return EXIT_PASS


def run_live_smoke_test() -> int:
    """Prove success, injected retry, and bounded failure with one ROS participant."""

    class NeverDoneFuture:
        def __init__(self) -> None:
            self.cancelled = False

        def done(self) -> bool:
            return False

        def cancel(self) -> None:
            self.cancelled = True

    namespace = f'/robotest_parameter_probe_smoke_{os.getpid()}_{time.monotonic_ns()}'
    executor: SingleThreadedExecutor | None = None
    nodes: list[Node] = []
    rclpy.init()
    try:
        fake = Node('parameter_target', namespace=namespace)
        nodes.append(fake)
        if fake.has_parameter(PARAMETER_NAME):
            set_results = fake.set_parameters(
                [Parameter(PARAMETER_NAME, Parameter.Type.BOOL, True)]
            )
            if len(set_results) != 1 or not set_results[0].successful:
                raise RuntimeError(f'failed to configure fake node: {set_results!r}')
        else:
            fake.declare_parameter(PARAMETER_NAME, True)

        probe = Node(f'{PROBE_NODE_NAME}_{os.getpid()}', namespace=namespace)
        nodes.append(probe)
        executor = SingleThreadedExecutor()
        executor.add_node(fake)
        executor.add_node(probe)
        result = probe_parameters(
            probe,
            executor,
            namespace,
            ['parameter_target'],
            5.0,
            None,
            discovery_grace=0.2,
            request_timeout=0.5,
            max_request_attempts=2,
        )
        if result['verdict'] != 'PASS':
            raise RuntimeError(f'baseline live parameter proof failed: {result!r}')
        observed_grace = (
            result['requests_started_after_wall_seconds']
            - result['all_services_ready_after_wall_seconds']
        )
        if observed_grace < 0.19:
            raise RuntimeError(f'live discovery grace was too short: {observed_grace!r}')

        parameter_client = AsyncParameterClient(
            probe,
            full_node_name(namespace, 'parameter_target'),
        )
        discovery_deadline = time.monotonic() + 2.0

        def live_abort() -> None:
            if not rclpy.ok():
                raise RuntimeError('rclpy shut down during live retry smoke')
            if time.monotonic() >= discovery_deadline:
                raise RuntimeError('live retry smoke exceeded its wall deadline')

        while not parameter_client.services_are_ready():
            live_abort()
            executor.spin_once(
                timeout_sec=min(SPIN_QUANTUM_S, discovery_deadline - time.monotonic())
            )
        _spin_until(
            executor,
            min(time.monotonic() + 0.2, discovery_deadline),
            live_abort,
            time.monotonic,
        )

        injected_timeout = NeverDoneFuture()
        request_count = 0

        def timeout_then_real_request() -> Any:
            nonlocal request_count
            request_count += 1
            if request_count == 1:
                return injected_timeout
            return parameter_client.get_parameters([PARAMETER_NAME])

        retry_started = time.monotonic()
        response, retry_attempts, retry_failure = request_with_retries(
            executor,
            timeout_then_real_request,
            parameter_client.services_are_ready,
            'get_parameters',
            request_timeout=0.1,
            max_attempts=2,
            global_deadline=discovery_deadline,
            probe_started=retry_started,
            abort_check=live_abort,
        )
        if response is None or retry_failure is not None:
            raise RuntimeError(f'live retry did not recover: {retry_attempts!r}')
        if [attempt['status'] for attempt in retry_attempts] != ['TIMEOUT', 'PASS']:
            raise RuntimeError(f'unexpected live retry attempts: {retry_attempts!r}')
        if not injected_timeout.cancelled:
            raise RuntimeError('timed-out live smoke future was not cancelled')
        _, response_failures = evaluate_parameter_responses(
            type(
                'DescribeResponse',
                (),
                {
                    'descriptors': [
                        ParameterDescriptor(
                            name=PARAMETER_NAME,
                            type=ParameterType.PARAMETER_BOOL,
                        )
                    ]
                },
            )(),
            response,
        )
        if response_failures:
            raise RuntimeError(f'live retry response was invalid: {response_failures!r}')

        failure_started = time.monotonic()
        failure_deadline = failure_started + 1.0

        def failure_abort() -> None:
            if not rclpy.ok() or time.monotonic() >= failure_deadline:
                raise RuntimeError('bounded live failure smoke exceeded its outer bound')

        failed_response, failure_attempts, failure = request_with_retries(
            executor,
            NeverDoneFuture,
            lambda: True,
            'describe_parameters',
            request_timeout=0.05,
            max_attempts=2,
            global_deadline=failure_deadline,
            probe_started=failure_started,
            abort_check=failure_abort,
        )
        if failed_response is not None or failure is None:
            raise RuntimeError('bounded live failure smoke accepted an incomplete response')
        if [attempt['status'] for attempt in failure_attempts] != ['TIMEOUT', 'TIMEOUT']:
            raise RuntimeError(f'unexpected live failure attempts: {failure_attempts!r}')

        result['live_smoke_failure_attempt_statuses'] = [
            attempt['status'] for attempt in failure_attempts
        ]
        result['live_smoke_retry_attempt_statuses'] = [
            attempt['status'] for attempt in retry_attempts
        ]
        sys.stdout.write(serialize_result(result))
        return EXIT_PASS if result['verdict'] == 'PASS' else EXIT_PROBE_FAILURE
    finally:
        cleanup_ros(executor, nodes)


def build_parser() -> argparse.ArgumentParser:
    """Build the stable verifier-facing CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/robotest')
    parser.add_argument('--node', action='append', dest='nodes')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--text-dir', type=Path)
    parser.add_argument('--text-prefix', default='sim-time-')
    parser.add_argument('--wall-timeout', type=float, default=30.0)
    parser.add_argument('--discovery-grace', type=float, default=DEFAULT_DISCOVERY_GRACE_S)
    parser.add_argument('--request-timeout', type=float, default=DEFAULT_REQUEST_TIMEOUT_S)
    parser.add_argument(
        '--max-request-attempts',
        type=int,
        default=MAX_REQUEST_ATTEMPTS,
    )
    parser.add_argument('--watch-pid', type=int)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--self-test', action='store_true')
    mode.add_argument('--live-smoke-test', action='store_true')
    return parser


def main() -> int:
    """Run a self-test, fake-node smoke, or bounded live parameter proof."""
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.live_smoke_test:
        return run_live_smoke_test()
    if not args.nodes:
        parser.error('at least one --node is required')
    if args.output is None:
        parser.error('--output is required')
    if len(args.nodes) != len(set(args.nodes)):
        parser.error('--node values must be unique')
    try:
        namespace = normalize_namespace(args.namespace)
        names = [validate_relative_node_name(name) for name in args.nodes]
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0.0:
        parser.error('--wall-timeout must be finite and positive')
    if not math.isfinite(args.discovery_grace) or args.discovery_grace <= 0.0:
        parser.error('--discovery-grace must be finite and positive')
    if not math.isfinite(args.request_timeout) or args.request_timeout <= 0.0:
        parser.error('--request-timeout must be finite and positive')
    if not 1 <= args.max_request_attempts <= MAX_REQUEST_ATTEMPTS:
        parser.error(f'--max-request-attempts must be between 1 and {MAX_REQUEST_ATTEMPTS}')
    if args.watch_pid is not None and args.watch_pid <= 0:
        parser.error('--watch-pid must be a positive integer')
    if (
        not args.text_prefix
        or Path(args.text_prefix).name != args.text_prefix
        or args.text_prefix in {'.', '..'}
    ):
        parser.error('--text-prefix must be a non-empty filename prefix')

    try:
        result = run_live_probe(
            namespace,
            names,
            args.wall_timeout,
            args.watch_pid,
            args.discovery_grace,
            args.request_timeout,
            args.max_request_attempts,
        )
    except KeyboardInterrupt:
        print('phase2_parameter_probe interrupted', file=sys.stderr)
        return 130
    except Exception as error:  # pragma: no cover - middleware/environment-specific
        print(
            f'phase2_parameter_probe internal error: {type(error).__name__}: {error}',
            file=sys.stderr,
        )
        return EXIT_INTERNAL_ERROR

    try:
        write_evidence(result, args.output, args.text_dir, args.text_prefix)
    except Exception as error:
        print(
            f'failed to write parameter evidence: {type(error).__name__}: {error}',
            file=sys.stderr,
        )
        return EXIT_INTERNAL_ERROR
    sys.stdout.write(serialize_result(result))
    return EXIT_PASS if result['verdict'] == 'PASS' else EXIT_PROBE_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
