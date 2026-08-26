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

"""Bounded command-line runner for one validated FollowWaypoints mission."""

from __future__ import annotations

import argparse
import sys
import time
from collections.abc import Sequence
from datetime import UTC, datetime
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args

from robotest_missions.artifacts import (
    ArtifactError,
    build_result,
    validate_artifact_paths,
    write_result_artifacts,
)
from robotest_missions.execution import ExecutionRecord, ExitCode, MissionExecutor
from robotest_missions.ros_action import RclpyFollowWaypointsDriver
from robotest_missions.schema import MissionValidationError, load_mission


class CliError(ValueError):
    """Raised instead of allowing argparse to terminate the process."""


class _ArgumentParser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise CliError(message)


class MissionRunnerNode(Node):
    """Node hosting only the direct FollowWaypoints action client."""

    def __init__(self) -> None:
        super().__init__(
            'mission_runner',
            parameter_overrides=[Parameter('use_sim_time', value=True)],
            automatically_declare_parameters_from_overrides=True,
        )


class RosMissionClock:
    """Simulation and steady clocks used by the bounded state machine."""

    def __init__(self, node: Node) -> None:
        self._node = node

    def sim_time_ns(self) -> int:
        return self._node.get_clock().now().nanoseconds

    def steady_time_s(self) -> float:
        return time.monotonic()


def _parser() -> argparse.ArgumentParser:
    parser = _ArgumentParser(
        prog='mission_runner',
        description='Run one strict RoboTest FollowWaypoints mission.',
    )
    parser.add_argument('--mission', required=True, help='strict mission YAML path')
    parser.add_argument('--json', required=True, dest='json_path', help='canonical JSON output')
    parser.add_argument(
        '--csv',
        required=True,
        dest='csv_path',
        help='matching one-row CSV output',
    )
    return parser


def _run_id(created: datetime) -> str:
    return f'phase2-mission-{created.strftime("%Y%m%dT%H%M%S.%fZ")}'


def _created_utc(created: datetime) -> str:
    return created.isoformat(timespec='microseconds').replace('+00:00', 'Z')


def _infrastructure_record(reason: str, clock: RosMissionClock) -> ExecutionRecord:
    return ExecutionRecord(
        action_name='follow_waypoints',
        resolved_action_name='follow_waypoints',
        started_sim_stamp_ns=clock.sim_time_ns(),
        started_steady_s=clock.steady_time_s(),
        terminal_wall_offset_s=0.0,
        exit_code=ExitCode.INFRASTRUCTURE_ERROR,
        reason=reason,
    )


def main(args: Sequence[str] | None = None) -> int:
    """Run one mission and return its stable process exit code."""
    raw_args = list(sys.argv if args is None else ['mission_runner', *args])
    try:
        cli = _parser().parse_args(remove_ros_args(args=raw_args)[1:])
        json_target, csv_target = validate_artifact_paths(cli.json_path, cli.csv_path)
        document = load_mission(cli.mission)
    except (ArtifactError, CliError, MissionValidationError, OSError, RuntimeError) as exc:
        print(f'mission_runner: validation error: {exc}', file=sys.stderr)
        return int(ExitCode.VALIDATION_ERROR)

    created = datetime.now(UTC)
    run_id = _run_id(created)
    node: MissionRunnerNode | None = None
    executor: SingleThreadedExecutor | None = None
    record: ExecutionRecord | None = None
    initialized = False
    try:
        rclpy.init(args=raw_args)
        initialized = True
        node = MissionRunnerNode()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        clock = RosMissionClock(node)
        driver = RclpyFollowWaypointsDriver(node)
        runner = MissionExecutor(
            document.config,
            driver,
            clock,
            lambda timeout: executor.spin_once(timeout_sec=timeout),
            runtime_ok=rclpy.ok,
        )
        record = runner.run()
    except Exception as exc:
        print(f'mission_runner: infrastructure error: {exc}', file=sys.stderr)
        if node is not None:
            clock = RosMissionClock(node)
            record = _infrastructure_record(f'ros_bootstrap_error: {exc}', clock)
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=1.0)
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()

    if record is None:
        return int(ExitCode.INFRASTRUCTURE_ERROR)

    try:
        result = build_result(
            document,
            record,
            run_id=run_id,
            created_utc=_created_utc(created),
        )
        write_result_artifacts(result, json_target, csv_target)
    except ArtifactError as exc:
        print(f'mission_runner: artifact error: {exc}', file=sys.stderr)
        return int(ExitCode.ARTIFACT_ERROR)

    print(
        f'mission_runner: exit={int(record.exit_code)} reason={record.reason} '
        f'json={Path(json_target)} csv={Path(csv_target)}',
        file=sys.stderr,
    )
    return int(record.exit_code)


if __name__ == '__main__':
    raise SystemExit(main())
