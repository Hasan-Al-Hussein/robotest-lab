#!/usr/bin/env python3
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

"""Bounded wall-clock trigger for Nav2 lifecycle-manager STARTUP."""

# Ruff and ROS ament-flake8 intentionally use different import-order models.
# ruff: noqa: I001

from __future__ import annotations

import argparse
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
import datetime
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time
from typing import Any, Protocol

from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

EXIT_SUCCESS = 0
EXIT_INVALID_CONFIGURATION = 2
EXIT_WATCH_PROCESS_ENDED = 3
EXIT_SERVICE_TIMEOUT = 4
EXIT_RESPONSE_TIMEOUT = 5
EXIT_STARTUP_REJECTED = 6
EXIT_RUNTIME_ERROR = 7
EXIT_INTERRUPTED = 130

RESULT_SCHEMA_VERSION = 1
RESULT_KEYS = frozenset(
    {
        'schema_version',
        'verdict',
        'accepted',
        'exit_code',
        'service_name',
        'command',
        'elapsed_wall_sec',
        'discovery_grace_sec',
        'service_timeout_sec',
        'response_timeout_sec',
        'watch_pid',
        'failure_kind',
        'failure_message',
        'started_utc',
        'completed_utc',
    }
)
FAILURE_KIND_BY_EXIT_CODE = {
    EXIT_INVALID_CONFIGURATION: 'invalid_configuration',
    EXIT_WATCH_PROCESS_ENDED: 'watch_process_ended',
    EXIT_SERVICE_TIMEOUT: 'service_timeout',
    EXIT_RESPONSE_TIMEOUT: 'response_timeout',
    EXIT_STARTUP_REJECTED: 'startup_rejected',
    EXIT_RUNTIME_ERROR: 'runtime_error',
    EXIT_INTERRUPTED: 'interrupted',
}

DEFAULT_MANAGER_NODE = 'lifecycle_manager_navigation'
DEFAULT_DISCOVERY_GRACE_SEC = 4.0
DEFAULT_SERVICE_TIMEOUT_SEC = 20.0
DEFAULT_RESPONSE_TIMEOUT_SEC = 60.0
DEFAULT_POLL_PERIOD_SEC = 0.05

_ROS_NAME_TOKEN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')


class ExecutorLike(Protocol):
    """Small executor surface used by the testable orchestration core."""

    def spin_once(self, timeout_sec: float | None = None) -> None:
        """Process callbacks for at most ``timeout_sec`` wall seconds."""


class ClientLike(Protocol):
    """Small service-client surface used by the testable orchestration core."""

    def service_is_ready(self) -> bool:
        """Return whether the manager service is currently discovered."""

    def call_async(self, request: ManageLifecycleNodes.Request):
        """Send one asynchronous lifecycle-manager request."""


@dataclass(frozen=True)
class StartupConfig:
    """Validated wall-clock bounds for one startup attempt."""

    service_name: str
    discovery_grace_sec: float = DEFAULT_DISCOVERY_GRACE_SEC
    service_timeout_sec: float = DEFAULT_SERVICE_TIMEOUT_SEC
    response_timeout_sec: float = DEFAULT_RESPONSE_TIMEOUT_SEC
    poll_period_sec: float = DEFAULT_POLL_PERIOD_SEC
    watch_pid: int | None = None

    def __post_init__(self) -> None:
        if not self.service_name.startswith('/'):
            raise ValueError('service_name must be absolute')
        for field_name in (
            'discovery_grace_sec',
            'service_timeout_sec',
            'response_timeout_sec',
            'poll_period_sec',
        ):
            value = getattr(self, field_name)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{field_name} must be finite and greater than zero')
        if self.watch_pid is not None and self.watch_pid <= 0:
            raise ValueError('watch_pid must be a positive process identifier')


@dataclass(frozen=True)
class StartupResult:
    """Evidence returned after a manager accepts STARTUP."""

    service_name: str
    elapsed_wall_sec: float
    discovery_grace_sec: float
    command: int


class StartupError(RuntimeError):
    """Expected bounded failure carrying a stable process exit code."""

    def __init__(self, message: str, exit_code: int) -> None:
        super().__init__(message)
        self.exit_code = exit_code


def utc_now() -> str:
    """Return one unambiguous, microsecond-resolution UTC timestamp."""
    return (
        datetime.datetime.now(datetime.UTC)
        .isoformat(timespec='microseconds')
        .replace('+00:00', 'Z')
    )


def build_result_document(
    *,
    accepted: bool,
    exit_code: int,
    service_name: str,
    elapsed_wall_sec: float,
    discovery_grace_sec: float,
    service_timeout_sec: float,
    response_timeout_sec: float,
    watch_pid: int | None,
    failure_kind: str | None,
    failure_message: str | None,
    started_utc: str,
    completed_utc: str,
) -> dict[str, Any]:
    """Build the exact strict startup-result schema."""
    if not math.isfinite(elapsed_wall_sec) or elapsed_wall_sec < 0.0:
        raise ValueError('elapsed_wall_sec must be finite and non-negative')
    passed = accepted and exit_code == EXIT_SUCCESS
    if accepted != (exit_code == EXIT_SUCCESS):
        raise ValueError('accepted must be true exactly when exit_code is zero')
    if passed:
        if failure_kind is not None or failure_message is not None:
            raise ValueError('PASS result cannot contain failure details')
    elif not failure_kind or failure_message is None:
        raise ValueError('FAIL result requires a failure kind and message')

    result = {
        'schema_version': RESULT_SCHEMA_VERSION,
        'verdict': 'PASS' if passed else 'FAIL',
        'accepted': accepted,
        'exit_code': exit_code,
        'service_name': service_name,
        'command': int(ManageLifecycleNodes.Request.STARTUP),
        'elapsed_wall_sec': round(elapsed_wall_sec, 9),
        'discovery_grace_sec': discovery_grace_sec,
        'service_timeout_sec': service_timeout_sec,
        'response_timeout_sec': response_timeout_sec,
        'watch_pid': watch_pid,
        'failure_kind': failure_kind,
        'failure_message': failure_message,
        'started_utc': started_utc,
        'completed_utc': completed_utc,
    }
    if frozenset(result) != RESULT_KEYS:  # pragma: no cover - defensive schema lock
        raise RuntimeError('startup-result schema key drift')
    # Validate finite JSON numbers and reject accidental non-standard values now.
    json.dumps(result, allow_nan=False, sort_keys=True)
    return result


def write_result_json_atomic(path: Path, result: dict[str, Any]) -> None:
    """Atomically replace one strict UTF-8/LF startup-result document."""
    if frozenset(result) != RESULT_KEYS:
        raise ValueError('startup-result document has an unexpected key set')
    content = json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + '\n'
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


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError('must be finite and greater than zero')
    return parsed


def _positive_pid(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError('must be a positive process identifier')
    return parsed


def normalize_namespace(namespace: str) -> str:
    """Return a validated absolute ROS namespace without a trailing slash."""
    stripped = namespace.strip()
    if stripped in ('', '/'):
        return '/'
    tokens = stripped.strip('/').split('/')
    if any(not _ROS_NAME_TOKEN.fullmatch(token) for token in tokens):
        raise ValueError(f'invalid ROS namespace: {namespace!r}')
    return '/' + '/'.join(tokens)


def lifecycle_manager_service(namespace: str, manager_node: str) -> str:
    """Resolve the project lifecycle manager's absolute ManageNodes service."""
    normalized_namespace = normalize_namespace(namespace)
    normalized_manager = manager_node.strip()
    if not _ROS_NAME_TOKEN.fullmatch(normalized_manager):
        raise ValueError(f'invalid lifecycle manager node name: {manager_node!r}')
    prefix = '' if normalized_namespace == '/' else normalized_namespace
    return f'{prefix}/{normalized_manager}/manage_nodes'


def process_is_alive(pid: int) -> bool:
    """Return whether a POSIX process still exists without signalling it."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _check_watch(config: StartupConfig, alive: Callable[[int], bool]) -> None:
    if config.watch_pid is not None and not alive(config.watch_pid):
        raise StartupError(
            f'watched process {config.watch_pid} ended before STARTUP completed',
            EXIT_WATCH_PROCESS_ENDED,
        )


def _bounded_spin(
    executor: ExecutorLike,
    deadline: float,
    config: StartupConfig,
    clock: Callable[[], float],
) -> None:
    remaining = deadline - clock()
    if remaining > 0.0:
        executor.spin_once(timeout_sec=min(config.poll_period_sec, remaining))


def trigger_startup(
    client: ClientLike,
    executor: ExecutorLike,
    config: StartupConfig,
    *,
    clock: Callable[[], float] = time.monotonic,
    alive: Callable[[int], bool] = process_is_alive,
) -> StartupResult:
    """Wait, stabilize discovery, and send exactly one bounded STARTUP request."""
    started_at = clock()
    service_deadline = started_at + config.service_timeout_sec

    while not client.service_is_ready():
        _check_watch(config, alive)
        if clock() >= service_deadline:
            raise StartupError(
                f'timed out discovering {config.service_name} after '
                f'{config.service_timeout_sec:.3f}s',
                EXIT_SERVICE_TIMEOUT,
            )
        _bounded_spin(executor, service_deadline, config, clock)

    grace_deadline = clock() + config.discovery_grace_sec
    while clock() < grace_deadline:
        _check_watch(config, alive)
        if not client.service_is_ready():
            raise StartupError(
                f'{config.service_name} disappeared during the '
                f'{config.discovery_grace_sec:.3f}s discovery grace',
                EXIT_SERVICE_TIMEOUT,
            )
        _bounded_spin(executor, grace_deadline, config, clock)

    _check_watch(config, alive)
    if not client.service_is_ready():
        raise StartupError(
            f'{config.service_name} was unavailable at STARTUP dispatch',
            EXIT_SERVICE_TIMEOUT,
        )

    request = ManageLifecycleNodes.Request()
    request.command = ManageLifecycleNodes.Request.STARTUP
    future = client.call_async(request)
    response_deadline = clock() + config.response_timeout_sec

    while not future.done():
        _check_watch(config, alive)
        if clock() >= response_deadline:
            with suppress(Exception):
                future.cancel()
            raise StartupError(
                f'timed out waiting {config.response_timeout_sec:.3f}s for '
                f'{config.service_name} STARTUP response',
                EXIT_RESPONSE_TIMEOUT,
            )
        _bounded_spin(executor, response_deadline, config, clock)

    if future.cancelled():
        raise StartupError('STARTUP response future was cancelled', EXIT_RUNTIME_ERROR)
    exception = future.exception()
    if exception is not None:
        raise StartupError(f'STARTUP request failed: {exception}', EXIT_RUNTIME_ERROR)
    response = future.result()
    if response is None or not response.success:
        raise StartupError(
            f'{config.service_name} rejected STARTUP',
            EXIT_STARTUP_REJECTED,
        )

    return StartupResult(
        service_name=config.service_name,
        elapsed_wall_sec=clock() - started_at,
        discovery_grace_sec=config.discovery_grace_sec,
        command=ManageLifecycleNodes.Request.STARTUP,
    )


def _cleanup_ros_resources(
    node: Node | None,
    executor: SingleThreadedExecutor | None,
) -> None:
    """Best-effort cleanup for every success and failure path."""
    if executor is not None:
        if node is not None:
            with suppress(Exception):
                executor.remove_node(node)
        with suppress(Exception):
            executor.shutdown(timeout_sec=1.0)
    if node is not None:
        with suppress(Exception):
            node.destroy_node()


def create_argument_parser() -> argparse.ArgumentParser:
    """Build the non-ROS CLI contract used by launch and diagnostics."""
    parser = argparse.ArgumentParser(
        description='Trigger bounded Nav2 lifecycle-manager STARTUP after discovery grace.'
    )
    parser.add_argument('--namespace', default='/robotest')
    parser.add_argument('--manager-node', default=DEFAULT_MANAGER_NODE)
    parser.add_argument(
        '--discovery-grace-sec',
        type=_positive_float,
        default=DEFAULT_DISCOVERY_GRACE_SEC,
    )
    parser.add_argument(
        '--service-timeout-sec',
        type=_positive_float,
        default=DEFAULT_SERVICE_TIMEOUT_SEC,
    )
    parser.add_argument(
        '--response-timeout-sec',
        type=_positive_float,
        default=DEFAULT_RESPONSE_TIMEOUT_SEC,
    )
    parser.add_argument(
        '--watch-pid',
        type=_positive_pid,
        help='Fail if this process exits before STARTUP completes.',
    )
    parser.add_argument(
        '--result-json',
        default='',
        help='Atomically write the strict startup result to PATH; empty disables output.',
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run one lifecycle-manager STARTUP attempt and return a stable exit code."""
    raw_args = sys.argv if argv is None else [sys.argv[0], *argv]
    try:
        cli_args = remove_ros_args(args=raw_args)[1:]
        options = create_argument_parser().parse_args(cli_args)
    except RuntimeError as exc:
        print(f'lifecycle_startup_trigger: invalid configuration: {exc}', file=sys.stderr)
        return EXIT_INVALID_CONFIGURATION

    started_wall = time.monotonic()
    started_utc = utc_now()
    result_path = Path(options.result_json) if options.result_json else None
    service_name = ''
    accepted = False
    exit_code = EXIT_RUNTIME_ERROR
    failure_kind: str | None = 'runtime_error'
    failure_message: str | None = 'startup trigger did not complete'
    node = None
    executor = None
    context = None
    rclpy_initialized = False
    try:
        service_name = lifecycle_manager_service(options.namespace, options.manager_node)
        config = StartupConfig(
            service_name=service_name,
            discovery_grace_sec=options.discovery_grace_sec,
            service_timeout_sec=options.service_timeout_sec,
            response_timeout_sec=options.response_timeout_sec,
            watch_pid=options.watch_pid,
        )
        rclpy.init(args=raw_args)
        rclpy_initialized = True
        node = Node(
            'lifecycle_startup_trigger',
            automatically_declare_parameters_from_overrides=True,
        )
        context = node.context
        if not node.has_parameter('use_sim_time'):
            node.declare_parameter('use_sim_time', True)
        if node.get_parameter('use_sim_time').value is not True:
            raise StartupError(
                'use_sim_time must be true for the Phase 2 startup trigger',
                EXIT_INVALID_CONFIGURATION,
            )

        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        client = node.create_client(ManageLifecycleNodes, service_name)
        node.get_logger().info(
            f'waiting up to {config.service_timeout_sec:.3f}s for {service_name}; '
            f'STARTUP follows a {config.discovery_grace_sec:.3f}s wall discovery grace'
        )
        result = trigger_startup(client, executor, config)
        node.get_logger().info(
            f'STARTUP accepted by {result.service_name} after '
            f'{result.elapsed_wall_sec:.3f}s wall time '
            f'(command={result.command}, grace={result.discovery_grace_sec:.3f}s)'
        )
        accepted = True
        exit_code = EXIT_SUCCESS
        failure_kind = None
        failure_message = None
    except ValueError as exc:
        exit_code = EXIT_INVALID_CONFIGURATION
        failure_kind = FAILURE_KIND_BY_EXIT_CODE[exit_code]
        failure_message = str(exc)
        print(f'lifecycle_startup_trigger: invalid configuration: {exc}', file=sys.stderr)
    except StartupError as exc:
        exit_code = exc.exit_code
        failure_kind = FAILURE_KIND_BY_EXIT_CODE.get(exit_code, 'runtime_error')
        failure_message = str(exc)
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f'lifecycle_startup_trigger: {exc}', file=sys.stderr)
    except KeyboardInterrupt:
        exit_code = EXIT_INTERRUPTED
        failure_kind = FAILURE_KIND_BY_EXIT_CODE[exit_code]
        failure_message = 'startup trigger interrupted'
    except Exception as exc:
        exit_code = EXIT_RUNTIME_ERROR
        failure_kind = FAILURE_KIND_BY_EXIT_CODE[exit_code]
        failure_message = f'{type(exc).__name__}: {exc}'
        if node is not None:
            node.get_logger().error(f'unexpected startup-trigger failure: {exc}')
        else:
            print(f'lifecycle_startup_trigger: unexpected failure: {exc}', file=sys.stderr)
    finally:
        _cleanup_ros_resources(node, executor)
        if rclpy_initialized:
            with suppress(Exception):
                if context is None:
                    rclpy.try_shutdown()
                elif rclpy.ok(context=context):
                    rclpy.shutdown(context=context)

    if result_path is not None:
        result_document = build_result_document(
            accepted=accepted,
            exit_code=exit_code,
            service_name=service_name,
            elapsed_wall_sec=max(0.0, time.monotonic() - started_wall),
            discovery_grace_sec=options.discovery_grace_sec,
            service_timeout_sec=options.service_timeout_sec,
            response_timeout_sec=options.response_timeout_sec,
            watch_pid=options.watch_pid,
            failure_kind=failure_kind,
            failure_message=failure_message,
            started_utc=started_utc,
            completed_utc=utc_now(),
        )
        try:
            write_result_json_atomic(result_path, result_document)
        except Exception as exc:
            print(
                f'lifecycle_startup_trigger: failed to write result JSON: '
                f'{type(exc).__name__}: {exc}',
                file=sys.stderr,
            )
            if exit_code == EXIT_SUCCESS:
                return EXIT_RUNTIME_ERROR
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
