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

"""Unit and structural tests for bounded lifecycle startup sequencing."""

# Ruff and ROS ament-flake8 intentionally use different import-order models.
# ruff: noqa: I001

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import re
import sys
from types import SimpleNamespace

from nav2_msgs.srv import ManageLifecycleNodes
import pytest

PACKAGE = Path(__file__).resolve().parents[1]
HELPER = PACKAGE / 'scripts' / 'lifecycle_startup_trigger.py'
LAUNCH = PACKAGE / 'launch' / 'phase2.launch.py'


def load_helper():
    """Load the project-owned executable as an importable test module."""
    spec = importlib.util.spec_from_file_location('robotest_lifecycle_startup_trigger', HELPER)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class FakeClock:
    """Deterministic monotonic clock advanced only by executor spins."""

    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeExecutor:
    """Executor double that records and advances every bounded spin."""

    def __init__(self, clock: FakeClock) -> None:
        self.clock = clock
        self.timeouts: list[float] = []

    def spin_once(self, timeout_sec: float | None = None) -> None:
        assert timeout_sec is not None and timeout_sec > 0.0
        self.timeouts.append(timeout_sec)
        self.clock.advance(timeout_sec)


class FakeFuture:
    """Future double completed at a configured monotonic instant."""

    def __init__(self, clock: FakeClock, done_at: float, success: bool) -> None:
        self.clock = clock
        self.done_at = done_at
        self.success = success
        self.was_cancelled = False

    def done(self) -> bool:
        return self.clock() >= self.done_at or self.was_cancelled

    def cancelled(self) -> bool:
        return self.was_cancelled

    def cancel(self) -> None:
        self.was_cancelled = True

    def exception(self):
        return None

    def result(self):
        return SimpleNamespace(success=self.success)


class FakeClient:
    """ManageNodes client double with deterministic discovery and response."""

    def __init__(
        self,
        clock: FakeClock,
        *,
        ready_at: float,
        response_delay: float = 0.1,
        success: bool = True,
        unavailable_at: float = math.inf,
    ) -> None:
        self.clock = clock
        self.ready_at = ready_at
        self.response_delay = response_delay
        self.success = success
        self.unavailable_at = unavailable_at
        self.requests: list[ManageLifecycleNodes.Request] = []
        self.future: FakeFuture | None = None

    def service_is_ready(self) -> bool:
        return self.ready_at <= self.clock() < self.unavailable_at

    def call_async(self, request: ManageLifecycleNodes.Request) -> FakeFuture:
        self.requests.append(request)
        self.future = FakeFuture(
            self.clock,
            self.clock() + self.response_delay,
            self.success,
        )
        return self.future


def make_config(module, **overrides):
    """Create one compact test configuration."""
    values = {
        'service_name': '/robotest/lifecycle_manager_navigation/manage_nodes',
        'discovery_grace_sec': 4.0,
        'service_timeout_sec': 2.0,
        'response_timeout_sec': 1.0,
        'poll_period_sec': 0.05,
    }
    values.update(overrides)
    return module.StartupConfig(**values)


class MainLogger:
    """No-op logger for main-path contract tests."""

    def info(self, _message: str) -> None:
        pass

    def error(self, _message: str) -> None:
        pass


class MainNode:
    """Minimal node double for result-artifact main-path tests."""

    def __init__(self, *_args, **_kwargs) -> None:
        self.context = object()
        self.logger = MainLogger()

    def has_parameter(self, _name: str) -> bool:
        return True

    def get_parameter(self, _name: str):
        return SimpleNamespace(value=True)

    def create_client(self, *_args, **_kwargs):
        return object()

    def get_logger(self) -> MainLogger:
        return self.logger

    def destroy_node(self) -> None:
        pass


class MainExecutor:
    """Minimal executor double for result-artifact main-path tests."""

    def __init__(self, **_kwargs) -> None:
        pass

    def add_node(self, _node) -> None:
        pass

    def remove_node(self, _node) -> None:
        pass

    def shutdown(self, *, timeout_sec: float) -> None:
        assert timeout_sec == 1.0


def install_main_doubles(module, monkeypatch, outcome) -> None:
    """Replace middleware setup and the core attempt with one fixed outcome."""
    monkeypatch.setattr(module, 'Node', MainNode)
    monkeypatch.setattr(module, 'SingleThreadedExecutor', MainExecutor)
    monkeypatch.setattr(module.rclpy, 'init', lambda **_kwargs: None)
    monkeypatch.setattr(module.rclpy, 'ok', lambda **_kwargs: False)

    def trigger(*_args, **_kwargs):
        if isinstance(outcome, BaseException):
            raise outcome
        return outcome

    monkeypatch.setattr(module, 'trigger_startup', trigger)


def test_exact_service_type_command_and_monotonic_grace() -> None:
    """One request is STARTUP and follows a full stable wall discovery grace."""
    module = load_helper()
    clock = FakeClock()
    executor = FakeExecutor(clock)
    client = FakeClient(clock, ready_at=0.25, response_delay=0.2)
    config = make_config(module)

    result = module.trigger_startup(client, executor, config, clock=clock)

    assert result.service_name == '/robotest/lifecycle_manager_navigation/manage_nodes'
    assert result.command == ManageLifecycleNodes.Request.STARTUP == 0
    assert 4.45 <= result.elapsed_wall_sec <= 4.50
    assert len(client.requests) == 1
    assert isinstance(client.requests[0], ManageLifecycleNodes.Request)
    assert client.requests[0].command == ManageLifecycleNodes.Request.STARTUP
    assert executor.timeouts
    assert max(executor.timeouts) <= config.poll_period_sec


def test_result_document_has_exact_schema_and_atomic_strict_json(tmp_path: Path) -> None:
    """The persisted PASS contract is exact, finite, and atomically replaced."""
    module = load_helper()
    timestamp = '2026-08-26T00:00:00.000000Z'
    result = module.build_result_document(
        accepted=True,
        exit_code=module.EXIT_SUCCESS,
        service_name='/robotest/lifecycle_manager_navigation/manage_nodes',
        elapsed_wall_sec=4.25,
        discovery_grace_sec=4.0,
        service_timeout_sec=20.0,
        response_timeout_sec=60.0,
        watch_pid=None,
        failure_kind=None,
        failure_message=None,
        started_utc=timestamp,
        completed_utc=timestamp,
    )
    assert frozenset(result) == module.RESULT_KEYS
    assert len(result) == 15
    assert result['verdict'] == 'PASS'
    assert result['accepted'] is True
    assert result['command'] == ManageLifecycleNodes.Request.STARTUP == 0
    assert result['failure_kind'] is None
    assert result['failure_message'] is None

    output = tmp_path / 'nested' / 'startup-result.json'
    output.parent.mkdir()
    output.write_text('{"stale": true}\n', encoding='utf-8')
    module.write_result_json_atomic(output, result)
    assert json.loads(output.read_text(encoding='utf-8')) == result
    assert output.read_bytes().endswith(b'\n')
    assert not list(output.parent.glob(f'.{output.name}.*.pending'))

    with pytest.raises(ValueError):
        module.build_result_document(
            accepted=True,
            exit_code=module.EXIT_SUCCESS,
            service_name='/robotest/lifecycle_manager_navigation/manage_nodes',
            elapsed_wall_sec=math.nan,
            discovery_grace_sec=4.0,
            service_timeout_sec=20.0,
            response_timeout_sec=60.0,
            watch_pid=None,
            failure_kind=None,
            failure_message=None,
            started_utc=timestamp,
            completed_utc=timestamp,
        )


def test_main_writes_pass_only_after_accepted_response(tmp_path: Path, monkeypatch) -> None:
    """A successful core result produces the exact accepted PASS artifact."""
    module = load_helper()
    outcome = module.StartupResult(
        service_name='/robotest/lifecycle_manager_navigation/manage_nodes',
        elapsed_wall_sec=4.1,
        discovery_grace_sec=4.0,
        command=ManageLifecycleNodes.Request.STARTUP,
    )
    install_main_doubles(module, monkeypatch, outcome)
    output = tmp_path / 'startup-pass.json'

    status = module.main(['--result-json', str(output)])

    result = json.loads(output.read_text(encoding='utf-8'))
    assert status == module.EXIT_SUCCESS
    assert frozenset(result) == module.RESULT_KEYS
    assert result['verdict'] == 'PASS'
    assert result['accepted'] is True
    assert result['exit_code'] == module.EXIT_SUCCESS
    assert result['service_name'] == '/robotest/lifecycle_manager_navigation/manage_nodes'
    assert result['failure_kind'] is None
    assert result['failure_message'] is None
    assert math.isfinite(result['elapsed_wall_sec']) and result['elapsed_wall_sec'] >= 0.0
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}T.*Z', result['started_utc'])
    assert re.fullmatch(r'\d{4}-\d{2}-\d{2}T.*Z', result['completed_utc'])


@pytest.mark.parametrize(
    ('exit_code', 'failure_kind'),
    [
        (3, 'watch_process_ended'),
        (4, 'service_timeout'),
        (5, 'response_timeout'),
        (6, 'startup_rejected'),
        (7, 'runtime_error'),
    ],
)
def test_main_writes_each_stable_startup_failure(
    tmp_path: Path,
    monkeypatch,
    exit_code: int,
    failure_kind: str,
) -> None:
    """Every bounded post-parse StartupError retains its exit and stable kind."""
    module = load_helper()
    install_main_doubles(module, monkeypatch, module.StartupError('injected failure', exit_code))
    output = tmp_path / f'startup-failure-{exit_code}.json'

    status = module.main(['--watch-pid', '42', '--result-json', str(output)])

    result = json.loads(output.read_text(encoding='utf-8'))
    assert status == exit_code
    assert frozenset(result) == module.RESULT_KEYS
    assert result['verdict'] == 'FAIL'
    assert result['accepted'] is False
    assert result['exit_code'] == exit_code
    assert result['failure_kind'] == failure_kind
    assert result['failure_message'] == 'injected failure'
    assert result['watch_pid'] == 42


def test_main_writes_invalid_configuration_and_interrupt(tmp_path: Path, monkeypatch) -> None:
    """Post-parse invalid configuration and interruption both produce FAIL JSON."""
    module = load_helper()
    invalid_output = tmp_path / 'invalid.json'
    invalid_status = module.main(
        [
            '--namespace',
            '/invalid-name',
            '--result-json',
            str(invalid_output),
        ]
    )
    invalid = json.loads(invalid_output.read_text(encoding='utf-8'))
    assert invalid_status == module.EXIT_INVALID_CONFIGURATION
    assert invalid['failure_kind'] == 'invalid_configuration'
    assert invalid['service_name'] == ''

    install_main_doubles(module, monkeypatch, KeyboardInterrupt())
    interrupt_output = tmp_path / 'interrupted.json'
    interrupt_status = module.main(['--result-json', str(interrupt_output)])
    interrupted = json.loads(interrupt_output.read_text(encoding='utf-8'))
    assert interrupt_status == module.EXIT_INTERRUPTED == 130
    assert interrupted['failure_kind'] == 'interrupted'
    assert interrupted['accepted'] is False


def test_service_discovery_and_stability_are_bounded() -> None:
    """Absent or disappearing manager services fail without sending STARTUP."""
    module = load_helper()
    clock = FakeClock()
    executor = FakeExecutor(clock)
    missing = FakeClient(clock, ready_at=math.inf)

    with pytest.raises(module.StartupError) as error:
        module.trigger_startup(missing, executor, make_config(module), clock=clock)
    assert error.value.exit_code == module.EXIT_SERVICE_TIMEOUT
    assert missing.requests == []

    clock = FakeClock()
    executor = FakeExecutor(clock)
    unstable = FakeClient(clock, ready_at=0.1, unavailable_at=1.0)
    with pytest.raises(module.StartupError) as error:
        module.trigger_startup(unstable, executor, make_config(module), clock=clock)
    assert error.value.exit_code == module.EXIT_SERVICE_TIMEOUT
    assert unstable.requests == []


def test_response_timeout_cancels_future_and_rejection_is_distinct() -> None:
    """Response timeout is finite and manager rejection has a separate result."""
    module = load_helper()
    clock = FakeClock()
    executor = FakeExecutor(clock)
    client = FakeClient(clock, ready_at=0.0, response_delay=10.0)
    config = make_config(module, discovery_grace_sec=0.1, response_timeout_sec=0.2)

    with pytest.raises(module.StartupError) as error:
        module.trigger_startup(client, executor, config, clock=clock)
    assert error.value.exit_code == module.EXIT_RESPONSE_TIMEOUT
    assert client.future is not None and client.future.was_cancelled

    clock = FakeClock()
    executor = FakeExecutor(clock)
    rejected = FakeClient(clock, ready_at=0.0, response_delay=0.05, success=False)
    with pytest.raises(module.StartupError) as error:
        module.trigger_startup(
            rejected,
            executor,
            make_config(module, discovery_grace_sec=0.1),
            clock=clock,
        )
    assert error.value.exit_code == module.EXIT_STARTUP_REJECTED


def test_watch_pid_aborts_before_dispatch() -> None:
    """A dead watched owner fails early and cannot dispatch STARTUP."""
    module = load_helper()
    clock = FakeClock()
    executor = FakeExecutor(clock)
    client = FakeClient(clock, ready_at=0.0)
    config = make_config(module, watch_pid=4242)

    with pytest.raises(module.StartupError) as error:
        module.trigger_startup(
            client,
            executor,
            config,
            clock=clock,
            alive=lambda _pid: clock() < 0.3,
        )
    assert error.value.exit_code == module.EXIT_WATCH_PROCESS_ENDED
    assert client.requests == []


def test_cleanup_attempts_every_owned_resource() -> None:
    """Executor and node cleanup both run even if one cleanup action raises."""
    module = load_helper()

    class CleanupExecutor:
        def __init__(self) -> None:
            self.removed = False
            self.shutdown_called = False

        def remove_node(self, _node) -> None:
            self.removed = True
            raise RuntimeError('injected remove failure')

        def shutdown(self, timeout_sec: float) -> None:
            assert timeout_sec == 1.0
            self.shutdown_called = True

    class CleanupNode:
        def __init__(self) -> None:
            self.destroyed = False

        def destroy_node(self) -> None:
            self.destroyed = True

    executor = CleanupExecutor()
    node = CleanupNode()
    module._cleanup_ros_resources(node, executor)
    assert executor.removed
    assert executor.shutdown_called
    assert node.destroyed


def test_namespace_validation_and_finite_cli_bounds() -> None:
    """Namespaces resolve deterministically and invalid bounds are rejected."""
    module = load_helper()
    assert module.lifecycle_manager_service('robotest', 'lifecycle_manager_navigation') == (
        '/robotest/lifecycle_manager_navigation/manage_nodes'
    )
    assert module.lifecycle_manager_service('/', 'manager') == '/manager/manage_nodes'
    for invalid in ('/robotest//bad', '/123bad', '/robot-est'):
        with pytest.raises(ValueError):
            module.normalize_namespace(invalid)
    for invalid in ('nan', 'inf', '0', '-1'):
        with pytest.raises(SystemExit):
            module.create_argument_parser().parse_args(['--discovery-grace-sec', invalid])
    assert module.create_argument_parser().parse_args(['--result-json', '']).result_json == ''


def test_launch_enforces_manual_manager_and_conditional_sim_time_trigger() -> None:
    """Public autostart controls only the project trigger, never manager autostart."""
    launch_text = LAUNCH.read_text(encoding='utf-8')
    helper_text = HELPER.read_text(encoding='utf-8')
    cmake_text = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')

    assert "'autostart': False" in launch_text
    assert 'condition=IfCondition(autostart)' in launch_text
    assert "executable='lifecycle_startup_trigger'" in launch_text
    assert "parameters=[{'use_sim_time': True}]" in launch_text
    assert "'--manager-node'" in launch_text
    assert "'--discovery-grace-sec'" in launch_text
    assert "'--service-timeout-sec'" in launch_text
    assert "'--response-timeout-sec'" in launch_text
    assert "'--result-json'" in launch_text
    assert "LaunchConfiguration('lifecycle_startup_result_path')" in launch_text
    assert 'ManageLifecycleNodes.Request.STARTUP' in helper_text
    assert 'time.monotonic' in helper_text
    assert 'get_clock(' not in helper_text
    assert 'subprocess' not in helper_text
    assert 'RENAME lifecycle_startup_trigger' in cmake_text
