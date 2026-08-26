#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Boundedly prove Phase 3 namespace, QoS, ownership, and isolation gates."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import os
from pathlib import Path
import time
from typing import Any

from phase3_orchestration import atomic_write_json, EvidenceError

MAX_ENDPOINTS = 4096
MAX_ATTEMPTS = 4096

AUTONOMY_NODE_NAMES = {
    'amcl',
    'behavior_server',
    'bt_navigator',
    'collision_monitor',
    'controller_server',
    'fault_proxy',
    'lifecycle_manager_navigation',
    'map_server',
    'mission_runner',
    'planner_server',
    'velocity_smoother',
    'waypoint_follower',
}

VALIDATION_TOPICS = (
    '/robotest/validation/contacts',
    '/robotest/validation/ground_truth',
    '/robotest/validation/scenario_entity_poses',
    '/robotest/validation/world_stats',
)

QOS_CONTRACTS: dict[str, tuple[str, str, int]] = {
    '/clock': ('BEST_EFFORT', 'VOLATILE', 1),
    '/robotest/map': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
    '/robotest/raw/scan': ('BEST_EFFORT', 'VOLATILE', 5),
    '/robotest/scan': ('BEST_EFFORT', 'VOLATILE', 5),
    '/robotest/raw/odom': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/odom': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/raw/imu': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/imu': ('BEST_EFFORT', 'VOLATILE', 10),
    '/robotest/navigation/plan': ('RELIABLE', 'VOLATILE', 5),
    '/robotest/cmd_vel_nav': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel_smoothed': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/cmd_vel_behavior_unused': ('RELIABLE', 'VOLATILE', 1),
    '/robotest/collision_monitor_state': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/ground_truth': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/contacts': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/world_stats': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/validation/scenario_entity_poses': ('RELIABLE', 'VOLATILE', 10),
    '/robotest/faults/events': ('RELIABLE', 'VOLATILE', 100),
    '/tf': ('RELIABLE', 'VOLATILE', 100),
    '/tf_static': ('RELIABLE', 'TRANSIENT_LOCAL', 1),
}

CANDIDATE_REQUIRED_NODES = AUTONOMY_NODE_NAMES - {'mission_runner'} | {
    'metrics_collector',
    'parameter_bridge',
    'phase3_goal_observer',
    'robot_state_publisher',
    'scenario_bridge',
    'scenario_controller',
}

POSITIVE_FORBIDDEN_NODES = AUTONOMY_NODE_NAMES - {'fault_proxy'}

CANDIDATE_EXPECTED_PUBLISHERS: dict[str, set[str]] = {
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
    '/robotest/collision_monitor_state': {'/robotest/collision_monitor'},
    '/robotest/validation/ground_truth': {'/robotest/parameter_bridge'},
    '/robotest/validation/contacts': {'/robotest/parameter_bridge'},
    '/robotest/validation/world_stats': {'/robotest/parameter_bridge'},
    '/robotest/validation/scenario_entity_poses': {'/robotest/parameter_bridge'},
    '/robotest/faults/events': {'/robotest/fault_proxy'},
    '/tf': {
        '/robotest/amcl',
        '/robotest/fault_proxy',
        '/robotest/robot_state_publisher',
    },
    '/tf_static': {'/robotest/robot_state_publisher'},
}

CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS: dict[str, set[str]] = {
    '/robotest/cmd_vel_nav': {'/robotest/velocity_smoother'},
    '/robotest/cmd_vel_smoothed': {'/robotest/collision_monitor'},
    '/robotest/cmd_vel': {'/robotest/parameter_bridge'},
    '/robotest/cmd_vel_behavior_unused': set(),
}

SCENARIO_SERVICES = (
    '/robotest/scenario/delete_entity',
    '/robotest/scenario/set_entity_pose',
    '/robotest/scenario/spawn_entity',
)


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


def _fq_node_name(info: Any) -> str:
    namespace = str(info.node_namespace).rstrip('/')
    return f'{namespace}/{info.node_name}' if namespace else f'/{info.node_name}'


def _policy_name(value: Any) -> str:
    name = getattr(value, 'name', None)
    return str(name) if name is not None else str(value).rsplit('.', 1)[-1]


def _endpoint_record(info: Any) -> dict[str, Any]:
    qos = info.qos_profile
    return {
        'depth': int(qos.depth),
        'durability': _policy_name(qos.durability),
        'history': _policy_name(qos.history),
        'node': _fq_node_name(info),
        'reliability': _policy_name(qos.reliability),
        'topic_type': str(info.topic_type),
    }


def _qos_status(record: dict[str, Any], expected: tuple[str, str, int]) -> dict[str, Any]:
    """Classify live QoS without turning Fast DDS UNKNOWN/0 into a false mismatch."""
    reliability, durability, depth = expected
    history = record['history']
    observed_depth = record['depth']
    introspection_complete = history not in {'UNKNOWN', 'SYSTEM_DEFAULT'} and observed_depth > 0
    explicit_keep_all = history == 'KEEP_ALL'
    positive_keep_last = history == 'KEEP_LAST' and observed_depth > 0
    exact_depth_mismatch = positive_keep_last and observed_depth != depth
    policy_contract_pass = (
        record['reliability'] == reliability
        and record['durability'] == durability
        and not explicit_keep_all
        and not exact_depth_mismatch
    )
    return {
        'bounded_depth_live_proven': positive_keep_last,
        'exact_depth_live_proven': positive_keep_last and observed_depth == depth,
        'explicit_keep_all': explicit_keep_all,
        'expected_depth': depth,
        'introspection_complete': introspection_complete,
        'policy_contract_pass': policy_contract_pass,
    }


def _qos_matches(record: dict[str, Any], expected: tuple[str, str, int]) -> bool:
    return _qos_status(record, expected)['policy_contract_pass']


def _node_snapshot(node: Any) -> list[str]:
    names = []
    for name, namespace in node.get_node_names_and_namespaces():
        prefix = namespace.rstrip('/')
        names.append(f'{prefix}/{name}' if prefix else f'/{name}')
    if len(names) > MAX_ENDPOINTS:
        raise EvidenceError('node graph exceeds the 4,096-name bound')
    return sorted(set(names))


def _service_snapshot(node: Any) -> set[str]:
    services = {name for name, _types in node.get_service_names_and_types()}
    if len(services) > MAX_ENDPOINTS:
        raise EvidenceError('service graph exceeds the 4,096-name bound')
    return services


def _topic_evidence(node: Any, topic: str) -> dict[str, Any]:
    publishers = [_endpoint_record(item) for item in node.get_publishers_info_by_topic(topic)]
    subscribers = [_endpoint_record(item) for item in node.get_subscriptions_info_by_topic(topic)]
    if len(publishers) + len(subscribers) > MAX_ENDPOINTS:
        raise EvidenceError(f'{topic} endpoint graph exceeds the bound')
    expected = QOS_CONTRACTS[topic]
    checks = [
        {
            **_qos_status(endpoint, expected),
            'node': endpoint['node'],
            'side': side,
        }
        for side, endpoints in (('publisher', publishers), ('subscriber', subscribers))
        for endpoint in endpoints
    ]
    return {
        'bounded_depth_live_proven': bool(checks)
        and all(item['bounded_depth_live_proven'] for item in checks),
        'exact_depth_live_proven': bool(checks)
        and all(item['exact_depth_live_proven'] for item in checks),
        'expected': {
            'depth': expected[2],
            'durability': expected[1],
            'history': 'KEEP_LAST',
            'reliability': expected[0],
        },
        'publishers': sorted(publishers, key=lambda item: (item['node'], item['topic_type'])),
        'publisher_qos_pass': bool(publishers)
        and all(_qos_matches(item, expected) for item in publishers),
        'qos_checks': sorted(checks, key=lambda item: (item['side'], item['node'])),
        'qos_introspection_complete': bool(checks)
        and all(item['introspection_complete'] for item in checks),
        'subscribers': sorted(subscribers, key=lambda item: (item['node'], item['topic_type'])),
        'subscriber_qos_pass': all(_qos_matches(item, expected) for item in subscribers),
    }


def _candidate_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    short_names = {value.rsplit('/', 1)[-1] for value in nodes}
    services = _service_snapshot(node)
    topics = {name: _topic_evidence(node, name) for name in QOS_CONTRACTS}
    publisher_ownership = {
        topic: sorted({item['node'] for item in topics[topic]['publishers']}) == sorted(expected)
        for topic, expected in CANDIDATE_EXPECTED_PUBLISHERS.items()
    }
    command_subscriber_ownership = {
        topic: sorted({item['node'] for item in topics[topic]['subscribers']}) == sorted(expected)
        for topic, expected in CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS.items()
    }
    cmd_owner_pass = publisher_ownership['/robotest/cmd_vel']
    autonomy_leaks: list[dict[str, str]] = []
    for topic in VALIDATION_TOPICS:
        for subscriber in topics[topic]['subscribers']:
            short_name = subscriber['node'].rsplit('/', 1)[-1]
            if short_name in AUTONOMY_NODE_NAMES:
                autonomy_leaks.append({'node': subscriber['node'], 'topic': topic})
    project_endpoints = [
        endpoint
        for evidence in topics.values()
        for role in ('publishers', 'subscribers')
        for endpoint in evidence[role]
    ]
    endpoint_namespace_pass = all(
        endpoint['node'].startswith('/robotest/') for endpoint in project_endpoints
    )
    required_nodes_missing = sorted(CANDIDATE_REQUIRED_NODES - short_names)
    required_nodes_outside_namespace = sorted(
        value
        for value in nodes
        if value.rsplit('/', 1)[-1] in CANDIDATE_REQUIRED_NODES
        and not value.startswith('/robotest/')
    )
    namespace_pass = endpoint_namespace_pass and not required_nodes_outside_namespace
    services_missing = sorted(set(SCENARIO_SERVICES) - services)
    legacy_service_absent = '/robotest/faults/load_schedule' not in services
    qos_pass = all(
        evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
        for evidence in topics.values()
    )
    qos_introspection_complete = all(
        evidence['qos_introspection_complete'] for evidence in topics.values()
    )
    bounded_depth_live_proven = all(
        evidence['bounded_depth_live_proven'] for evidence in topics.values()
    )
    passed = (
        not required_nodes_missing
        and not required_nodes_outside_namespace
        and not services_missing
        and legacy_service_absent
        and cmd_owner_pass
        and all(publisher_ownership.values())
        and all(command_subscriber_ownership.values())
        and not autonomy_leaks
        and namespace_pass
        and qos_pass
    )
    return passed, {
        'autonomy_validation_leaks': autonomy_leaks,
        'bounded_depth_live_proven_for_all_endpoints': bounded_depth_live_proven,
        'cmd_vel_owner_pass': cmd_owner_pass,
        'command_subscriber_ownership': command_subscriber_ownership,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in topics.items()
        },
        'legacy_fault_service_absent': legacy_service_absent,
        'mode': 'candidate',
        'namespace_isolation_pass': namespace_pass,
        'nodes': nodes,
        'publisher_ownership': publisher_ownership,
        'qos_contract_pass': qos_pass,
        'qos_introspection_complete': qos_introspection_complete,
        'required_nodes_missing': required_nodes_missing,
        'required_nodes_outside_namespace': required_nodes_outside_namespace,
        'scenario_services_missing': services_missing,
        'topics': topics,
        'validation_autonomy_isolation_pass': not autonomy_leaks,
    }


def _positive_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    short_names = {value.rsplit('/', 1)[-1] for value in nodes}
    services = _service_snapshot(node)
    relevant = {
        name: _topic_evidence(node, name)
        for name in (
            '/clock',
            '/robotest/cmd_vel',
            '/robotest/validation/contacts',
            '/robotest/validation/ground_truth',
            '/robotest/validation/scenario_entity_poses',
            '/robotest/validation/world_stats',
        )
    }
    cmd_publishers = relevant['/robotest/cmd_vel']['publishers']
    cmd_owner_pass = len(cmd_publishers) == 1 and cmd_publishers[0]['node'].endswith(
        '/contact_control_driver'
    )
    required_nodes = {
        'contact_control_driver',
        'fault_proxy',
        'metrics_collector',
        'parameter_bridge',
        'robot_state_publisher',
        'scenario_bridge',
    }
    required_nodes_missing = sorted(required_nodes - short_names)
    forbidden_present = sorted(POSITIVE_FORBIDDEN_NODES & short_names)
    services_missing = sorted(
        {
            '/robotest/scenario/delete_entity',
            '/robotest/scenario/spawn_entity',
        }
        - services
    )
    namespace_pass = all(
        endpoint['node'].startswith('/robotest/')
        for evidence in relevant.values()
        for role in ('publishers', 'subscribers')
        for endpoint in evidence[role]
    )
    qos_pass = all(
        evidence['publisher_qos_pass'] and evidence['subscriber_qos_pass']
        for evidence in relevant.values()
    )
    qos_introspection_complete = all(
        evidence['qos_introspection_complete'] for evidence in relevant.values()
    )
    bounded_depth_live_proven = all(
        evidence['bounded_depth_live_proven'] for evidence in relevant.values()
    )
    passed = (
        cmd_owner_pass
        and not forbidden_present
        and not required_nodes_missing
        and not services_missing
        and namespace_pass
        and qos_pass
    )
    return passed, {
        'bounded_depth_live_proven_for_all_endpoints': bounded_depth_live_proven,
        'cmd_vel_owner_pass': cmd_owner_pass,
        'exact_static_qos_depth_contract': {
            topic: evidence['expected'] for topic, evidence in relevant.items()
        },
        'forbidden_nodes_present': forbidden_present,
        'mode': 'positive_control',
        'namespace_isolation_pass': namespace_pass,
        'nodes': nodes,
        'qos_contract_pass': qos_pass,
        'qos_introspection_complete': qos_introspection_complete,
        'required_nodes_missing': required_nodes_missing,
        'scenario_services_missing': services_missing,
        'topics': relevant,
        'validation_autonomy_isolation_pass': not forbidden_present,
    }


def _empty_evaluation(node: Any) -> tuple[bool, dict[str, Any]]:
    nodes = _node_snapshot(node)
    self_name = node.get_fully_qualified_name()
    remaining = [value for value in nodes if value != self_name]
    return not remaining, {
        'mode': 'empty',
        'nodes': nodes,
        'remaining_nodes': remaining,
    }


def _run(arguments: argparse.Namespace) -> int:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    if arguments.output.exists() or arguments.output.is_symlink():
        raise EvidenceError(f'runtime gate output already exists: {arguments.output}')
    if not 0.0 < arguments.wall_timeout_s <= 120.0:
        raise EvidenceError('wall timeout must be in (0, 120] seconds')
    rclpy.init(args=list(arguments.ros_args))
    node = Node('phase3_runtime_gate', namespace='/robotest/evidence')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    started_ns = time.monotonic_ns()
    deadline_ns = started_ns + int(arguments.wall_timeout_s * 1_000_000_000)
    attempt = 0
    last: dict[str, Any] = {}
    try:
        while rclpy.ok() and time.monotonic_ns() < deadline_ns:
            if not _pid_alive(arguments.watch_pid):
                raise EvidenceError('watched process exited before runtime gate converged')
            attempt += 1
            if attempt > MAX_ATTEMPTS:
                raise EvidenceError('runtime gate exceeded 4,096 graph attempts')
            executor.spin_once(timeout_sec=0.05)
            if arguments.mode == 'candidate':
                passed, last = _candidate_evaluation(node)
            elif arguments.mode == 'positive-control':
                passed, last = _positive_evaluation(node)
            else:
                passed, last = _empty_evaluation(node)
            if passed:
                result = {
                    **last,
                    'attempt_count': attempt,
                    'elapsed_wall_s': (time.monotonic_ns() - started_ns) / 1_000_000_000,
                    'producer': 'robotest_phase3/runtime_gate',
                    'schema_version': 1,
                    'verdict': 'PASS',
                }
                atomic_write_json(arguments.output, result, sidecar=True)
                return 0
        result = {
            **last,
            'attempt_count': attempt,
            'elapsed_wall_s': (time.monotonic_ns() - started_ns) / 1_000_000_000,
            'producer': 'robotest_phase3/runtime_gate',
            'schema_version': 1,
            'verdict': 'FAIL',
        }
        atomic_write_json(arguments.output, result, sidecar=True)
        return 1
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _self_test() -> int:
    class Policy:
        def __init__(self, name: str) -> None:
            self.name = name

    class Qos:
        history = Policy('KEEP_LAST')
        reliability = Policy('RELIABLE')
        durability = Policy('VOLATILE')
        depth = 10

    class Endpoint:
        node_namespace = '/robotest'
        node_name = 'metrics_collector'
        topic_type = 'std_msgs/msg/String'
        qos_profile = Qos()

    record = _endpoint_record(Endpoint())
    assert record['node'] == '/robotest/metrics_collector'
    assert _qos_matches(record, ('RELIABLE', 'VOLATILE', 10))
    assert not _qos_matches(record, ('BEST_EFFORT', 'VOLATILE', 10))
    unknown = {**record, 'depth': 0, 'history': 'UNKNOWN'}
    unknown_status = _qos_status(unknown, ('RELIABLE', 'VOLATILE', 10))
    assert unknown_status['policy_contract_pass']
    assert not unknown_status['introspection_complete']
    assert not unknown_status['bounded_depth_live_proven']
    keep_all = {**record, 'depth': 0, 'history': 'KEEP_ALL'}
    assert not _qos_status(keep_all, ('RELIABLE', 'VOLATILE', 10))['policy_contract_pass']
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('candidate', 'positive-control', 'empty'))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--watch-pid', type=int)
    parser.add_argument('--wall-timeout-s', type=float, default=90.0)
    parser.add_argument('--self-test', action='store_true')
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the runtime gate or its pure self-test."""
    arguments, ros_args = _parser().parse_known_args(argv)
    if arguments.self_test:
        return _self_test()
    if arguments.mode is None or arguments.output is None:
        print('phase3 runtime gate requires --mode and --output', file=os.sys.stderr)
        return 2
    arguments.ros_args = ros_args
    try:
        return _run(arguments)
    except (EvidenceError, RuntimeError) as exc:
        print(f'phase3 runtime gate error: {exc}', file=os.sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
