#!/usr/bin/env python3
# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0
# ruff: noqa: I001

"""Boundedly prove Phase 2 node, topic, service, and action graph contracts."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import suppress
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time
from typing import Any

from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav2_msgs.action import FollowWaypoints
import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.action.graph import (
    get_action_client_names_and_types_by_node,
    get_action_names_and_types,
    get_action_server_names_and_types_by_node,
)
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_action_status_default
from std_msgs.msg import String
from std_srvs.srv import Trigger

PROBE_NODE_NAME = 'phase2_graph_probe'
PROBE_NAMESPACE = '/robotest/evidence'
SCHEMA_VERSION = 2
SPIN_QUANTUM_S = 0.05
MAXIMUM_GRAPH_NAMES = 4096
MAXIMUM_TYPES_PER_NAME = 16
MAXIMUM_GRAPH_NODES = 1024
ACTION_SEND_GOAL_SERVICE_SUFFIX = '/_action/send_goal'
ACTION_SEND_GOAL_TYPE_SUFFIX = '_SendGoal'

EXIT_PASS = 0
EXIT_PROBE_FAILURE = 1
EXIT_INTERNAL_ERROR = 3

_GRAPH_NAME_TOKEN = re.compile(r'^[A-Za-z_][A-Za-z0-9_]*$')
_TYPE_TOKEN = r'[A-Za-z_][A-Za-z0-9_]*'
_TYPE_NAME = re.compile(rf'^{_TYPE_TOKEN}/(?P<kind>msg|srv|action)/{_TYPE_TOKEN}$')


def validate_graph_name(name: str) -> str:
    """Return one canonical absolute ROS graph name."""
    if not name.startswith('/') or name == '/' or name.endswith('/'):
        raise ValueError(f'graph name must be canonical and absolute: {name!r}')
    tokens = name[1:].split('/')
    if any(_GRAPH_NAME_TOKEN.fullmatch(token) is None for token in tokens):
        raise ValueError(f'invalid ROS graph name: {name!r}')
    return name


def parse_contract(value: str, expected_kind: str) -> tuple[str, str]:
    """Parse ``/absolute/name=pkg/msg-or-action/Type`` exactly."""
    if value.count('=') != 1:
        raise ValueError(f'contract must contain exactly one equals sign: {value!r}')
    name, type_name = value.split('=', 1)
    name = validate_graph_name(name)
    match = _TYPE_NAME.fullmatch(type_name)
    if match is None or match.group('kind') != expected_kind:
        raise ValueError(f'{name} must use canonical ROS {expected_kind} type: {type_name!r}')
    return name, type_name


def contracts_from_values(values: list[str] | None, kind: str) -> dict[str, str]:
    """Build a unique, sorted contract map from repeated CLI values."""
    contracts: dict[str, str] = {}
    for value in values or []:
        name, type_name = parse_contract(value, kind)
        if name in contracts:
            raise ValueError(f'duplicate {kind} contract: {name}')
        contracts[name] = type_name
    return dict(sorted(contracts.items()))


def endpoint_contracts_from_values(
    values: list[str] | None,
    action_contracts: dict[str, str],
    role: str,
) -> dict[str, list[str]]:
    """Build exact expected action endpoint owners for one role."""
    endpoints = {name: [] for name in action_contracts}
    seen: set[tuple[str, str]] = set()
    for value in values or []:
        if value.count('=') != 1:
            raise ValueError(
                f'action {role} contract must contain exactly one equals sign: {value!r}'
            )
        action_name, node_name = value.split('=', 1)
        action_name = validate_graph_name(action_name)
        node_name = validate_graph_name(node_name)
        if action_name not in action_contracts:
            raise ValueError(f'action {role} contract references undeclared action: {action_name}')
        pair = (action_name, node_name)
        if pair in seen:
            raise ValueError(f'duplicate action {role} contract: {value}')
        seen.add(pair)
        endpoints[action_name].append(node_name)
    return {name: sorted(nodes) for name, nodes in sorted(endpoints.items())}


def goal_action_clients_from_service_clients(
    action_contracts: dict[str, str],
    entries: list[tuple[str, list[str]]],
) -> dict[str, list[str]]:
    """Derive goal-capable actions from one node's SendGoal service clients."""
    if len(entries) > MAXIMUM_GRAPH_NAMES:
        raise ValueError(f'service-client count {len(entries)} exceeds {MAXIMUM_GRAPH_NAMES}')
    service_types: dict[str, set[str]] = {}
    for raw_name, raw_types in entries:
        name = validate_graph_name(raw_name)
        types = set(raw_types)
        if len(types) > MAXIMUM_TYPES_PER_NAME:
            raise ValueError(
                f'{name} service client has {len(types)} types; bound is {MAXIMUM_TYPES_PER_NAME}'
            )
        merged_types = service_types.setdefault(name, set())
        merged_types.update(types)
        if len(merged_types) > MAXIMUM_TYPES_PER_NAME:
            raise ValueError(
                f'{name} service client has more than {MAXIMUM_TYPES_PER_NAME} merged types'
            )

    goal_actions: dict[str, list[str]] = {}
    for action_name, expected_action_type in action_contracts.items():
        send_goal_name = f'{action_name}{ACTION_SEND_GOAL_SERVICE_SUFFIX}'
        if send_goal_name not in service_types:
            continue
        expected_send_goal_type = f'{expected_action_type}{ACTION_SEND_GOAL_TYPE_SUFFIX}'
        observed_action_types: set[str] = set()
        for service_type in service_types[send_goal_name]:
            if service_type == expected_send_goal_type:
                observed_action_types.add(expected_action_type)
            elif service_type.endswith(ACTION_SEND_GOAL_TYPE_SUFFIX):
                observed_action_types.add(service_type[: -len(ACTION_SEND_GOAL_TYPE_SUFFIX)])
            else:
                observed_action_types.add(service_type)
        goal_actions[action_name] = sorted(observed_action_types)
    return dict(sorted(goal_actions.items()))


def serialize_result(result: dict[str, Any]) -> str:
    """Serialize strict, canonical JSON."""
    return json.dumps(result, allow_nan=False, indent=2, sort_keys=True) + '\n'


def atomic_write_text(path: Path, content: str) -> None:
    """Atomically replace one UTF-8/LF evidence file."""
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
    """Return whether a positive PID still identifies a live process."""
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def is_hidden_graph_name(name: str) -> bool:
    """Match the ROS CLI default of omitting hidden graph names."""
    return any(token.startswith('_') for token in name.split('/') if token)


def normalize_snapshot(
    entries: list[tuple[str, list[str]]],
) -> dict[str, list[str]]:
    """Return a bounded public graph snapshot with deterministic ordering."""
    public = [(name, types) for name, types in entries if not is_hidden_graph_name(name)]
    if len(public) > MAXIMUM_GRAPH_NAMES:
        raise ValueError(
            f'public graph contains {len(public)} names; bound is {MAXIMUM_GRAPH_NAMES}'
        )
    snapshot: dict[str, list[str]] = {}
    for raw_name, raw_types in public:
        name = validate_graph_name(raw_name)
        types = sorted(set(raw_types))
        if len(types) > MAXIMUM_TYPES_PER_NAME:
            raise ValueError(f'{name} has {len(types)} types; bound is {MAXIMUM_TYPES_PER_NAME}')
        snapshot[name] = types
    return dict(sorted(snapshot.items()))


def full_node_name(name: str, namespace: str) -> str:
    """Join graph-provided node identity fields into one canonical FQN."""
    canonical_namespace = '/' + namespace.strip('/') if namespace.strip('/') else ''
    return validate_graph_name(f'{canonical_namespace}/{name}')


def node_name_multiplicity(node_names: list[str]) -> tuple[dict[str, int], list[str]]:
    """Return deterministic node-name counts and exact duplicates."""
    counts = dict(sorted(Counter(node_names).items()))
    duplicates = sorted(name for name, count in counts.items() if count > 1)
    return counts, duplicates


def node_identity_snapshot(
    probe: Node,
) -> tuple[
    list[tuple[str, str]],
    list[dict[str, Any]],
    list[str],
    dict[str, int],
    list[str],
]:
    """Retain raw identities and summarize all non-probe node-name multiplicity."""
    raw_identities = list(probe.get_node_names_and_namespaces())
    if len(raw_identities) > MAXIMUM_GRAPH_NODES:
        raise ValueError(
            f'graph contains {len(raw_identities)} node identities; bound is {MAXIMUM_GRAPH_NODES}'
        )
    participant = probe.get_fully_qualified_name()
    records: list[dict[str, Any]] = []
    all_node_names: list[str] = []
    public_node_names: list[str] = []
    for name, namespace in sorted(raw_identities):
        node_fqn = full_node_name(name, namespace)
        hidden = is_hidden_graph_name(node_fqn)
        is_participant = node_fqn == participant
        records.append(
            {
                'fully_qualified_name': node_fqn,
                'hidden': hidden,
                'is_probe_participant': is_participant,
                'name': name,
                'namespace': namespace,
            }
        )
        if not is_participant:
            all_node_names.append(node_fqn)
            if not hidden:
                public_node_names.append(node_fqn)
    public_node_names.sort()
    counts, duplicates = node_name_multiplicity(all_node_names)
    return raw_identities, records, public_node_names, counts, duplicates


def action_endpoint_snapshot(
    probe: Node,
    identities: list[tuple[str, str]],
    action_contracts: dict[str, str],
) -> tuple[
    dict[str, dict[str, list[str]]],
    dict[str, dict[str, list[str]]],
    dict[str, dict[str, list[str]]],
    list[str],
]:
    """Map goal clients, projected client participants, and action servers."""
    goal_clients: dict[str, dict[str, list[str]]] = {}
    client_participants: dict[str, dict[str, list[str]]] = {}
    servers: dict[str, dict[str, list[str]]] = {}
    query_errors: list[str] = []
    for remote_name, remote_namespace in sorted(set(identities)):
        node_fqn = full_node_name(remote_name, remote_namespace)
        for role, query, destination in (
            (
                'client participant',
                get_action_client_names_and_types_by_node,
                client_participants,
            ),
            ('server', get_action_server_names_and_types_by_node, servers),
        ):
            try:
                entries = query(probe, remote_name, remote_namespace)
            except Exception as error:  # pragma: no cover - graph changes during query
                query_errors.append(f'{role} query {node_fqn}: {type(error).__name__}: {error}')
                continue
            for action_name, raw_types in entries:
                action_name = validate_graph_name(action_name)
                types = sorted(set(raw_types))
                if len(types) > MAXIMUM_TYPES_PER_NAME:
                    raise ValueError(
                        f'{action_name} {role} has {len(types)} types; '
                        f'bound is {MAXIMUM_TYPES_PER_NAME}'
                    )
                destination.setdefault(action_name, {})[node_fqn] = types
        try:
            service_client_entries = probe.get_client_names_and_types_by_node(
                remote_name, remote_namespace
            )
        except Exception as error:  # pragma: no cover - graph changes during query
            query_errors.append(
                f'service-client query for {node_fqn}: {type(error).__name__}: {error}'
            )
            continue
        for action_name, types in goal_action_clients_from_service_clients(
            action_contracts, service_client_entries
        ).items():
            goal_clients.setdefault(action_name, {})[node_fqn] = types
    return (
        {name: dict(sorted(nodes.items())) for name, nodes in sorted(goal_clients.items())},
        {name: dict(sorted(nodes.items())) for name, nodes in sorted(client_participants.items())},
        {name: dict(sorted(nodes.items())) for name, nodes in sorted(servers.items())},
        sorted(query_errors),
    )


def evaluate_contracts(
    expected: dict[str, str],
    observed: dict[str, list[str]],
) -> tuple[dict[str, dict[str, Any]], list[str], list[dict[str, Any]]]:
    """Evaluate exact required type sets while allowing unrelated graph names."""
    results: dict[str, dict[str, Any]] = {}
    missing: list[str] = []
    mismatches: list[dict[str, Any]] = []
    for name, expected_type in expected.items():
        observed_types = observed.get(name, [])
        present = name in observed
        exact = observed_types == [expected_type]
        results[name] = {
            'exact_type_match': exact,
            'expected_type': expected_type,
            'observed_types': observed_types,
            'present': present,
            'status': 'PASS' if exact else 'MISSING' if not present else 'TYPE_MISMATCH',
        }
        if not present:
            missing.append(name)
        elif not exact:
            mismatches.append(
                {
                    'expected_type': expected_type,
                    'name': name,
                    'observed_types': observed_types,
                }
            )
    return results, missing, mismatches


def evaluate_action_ownership(
    action_contracts: dict[str, str],
    expected_clients: dict[str, list[str]],
    expected_servers: dict[str, list[str]],
    observed_clients: dict[str, dict[str, list[str]]],
    observed_servers: dict[str, dict[str, list[str]]],
) -> tuple[dict[str, dict[str, Any]], list[dict[str, Any]]]:
    """Require exact client/server FQN sets and exact endpoint types."""
    results: dict[str, dict[str, Any]] = {}
    mismatches: list[dict[str, Any]] = []
    for action_name, expected_type in action_contracts.items():
        clients = observed_clients.get(action_name, {})
        servers = observed_servers.get(action_name, {})
        client_nodes = sorted(clients)
        server_nodes = sorted(servers)
        expected_client_nodes = expected_clients[action_name]
        expected_server_nodes = expected_servers[action_name]
        client_type_mismatches = {
            node: types for node, types in clients.items() if types != [expected_type]
        }
        server_type_mismatches = {
            node: types for node, types in servers.items() if types != [expected_type]
        }
        exact = (
            client_nodes == expected_client_nodes
            and server_nodes == expected_server_nodes
            and not client_type_mismatches
            and not server_type_mismatches
        )
        result = {
            'client_type_mismatches': client_type_mismatches,
            'exact_ownership_and_types': exact,
            'expected_client_nodes': expected_client_nodes,
            'expected_server_nodes': expected_server_nodes,
            'expected_type': expected_type,
            'observed_clients': clients,
            'observed_servers': servers,
            'server_type_mismatches': server_type_mismatches,
            'status': 'PASS' if exact else 'OWNERSHIP_OR_TYPE_MISMATCH',
        }
        results[action_name] = result
        if not exact:
            mismatches.append({'action': action_name, **result})
    return results, mismatches


def graph_text(snapshot: dict[str, list[str]]) -> str:
    """Render output compatible with ``ros2 topic/action list -t``."""
    return ''.join(f'{name} [{", ".join(types)}]\n' for name, types in sorted(snapshot.items()))


def initial_result(
    topic_contracts: dict[str, str],
    service_contracts: dict[str, str],
    action_contracts: dict[str, str],
    expected_action_clients: dict[str, list[str]],
    expected_action_servers: dict[str, list[str]],
    wall_timeout: float,
    watch_pid: int | None,
) -> dict[str, Any]:
    """Build JSON-safe evidence before discovery begins."""
    return {
        'attempt_count': 0,
        'contracts': {
            'actions': action_contracts,
            'action_clients': expected_action_clients,
            'action_servers': expected_action_servers,
            'services': service_contracts,
            'topics': topic_contracts,
        },
        'elapsed_wall_seconds': 0.0,
        'failure': None,
        'failure_kind': None,
        'limits': {
            'maximum_graph_names': MAXIMUM_GRAPH_NAMES,
            'maximum_graph_nodes': MAXIMUM_GRAPH_NODES,
            'maximum_types_per_name': MAXIMUM_TYPES_PER_NAME,
            'wall_timeout_seconds': wall_timeout,
        },
        'missing_actions': sorted(action_contracts),
        'missing_services': sorted(service_contracts),
        'missing_topics': sorted(topic_contracts),
        'duplicate_node_names': [],
        'node_name_counts': {},
        'observed': {
            'action_clients': {},
            'action_client_participants': {},
            'action_servers': {},
            'actions': {},
            'node_identities': [],
            'node_names': [],
            'services': {},
            'topics': {},
        },
        'participant': f'{PROBE_NAMESPACE}/{PROBE_NODE_NAME}',
        'query_errors': [],
        'results': {'action_ownership': {}, 'actions': {}, 'services': {}, 'topics': {}},
        'schema_version': SCHEMA_VERSION,
        'type_mismatches': {'actions': [], 'services': [], 'topics': []},
        'action_ownership_mismatches': [],
        'verdict': 'FAIL',
        'watch_pid': watch_pid,
    }


def probe_graph(
    probe: Node,
    executor: SingleThreadedExecutor,
    topic_contracts: dict[str, str],
    service_contracts: dict[str, str],
    action_contracts: dict[str, str],
    expected_action_clients: dict[str, list[str]],
    expected_action_servers: dict[str, list[str]],
    wall_timeout: float,
    watch_pid: int | None,
) -> dict[str, Any]:
    """Observe the graph repeatedly under one monotonic deadline."""
    result = initial_result(
        topic_contracts,
        service_contracts,
        action_contracts,
        expected_action_clients,
        expected_action_servers,
        wall_timeout,
        watch_pid,
    )
    result['participant'] = probe.get_fully_qualified_name()
    started = time.monotonic()
    deadline = started + wall_timeout

    while True:
        if watch_pid is not None and not watched_process_alive(watch_pid):
            result['failure_kind'] = 'watch_pid_exited'
            result['failure'] = f'watched process {watch_pid} exited before graph readiness'
            break
        if not rclpy.ok():
            result['failure_kind'] = 'rclpy_shutdown'
            result['failure'] = 'rclpy context shut down before graph readiness'
            break

        topics = normalize_snapshot(probe.get_topic_names_and_types(no_demangle=False))
        services = normalize_snapshot(probe.get_service_names_and_types())
        actions = normalize_snapshot(get_action_names_and_types(probe))
        (
            raw_node_identities,
            node_identity_records,
            node_names,
            node_name_counts,
            duplicate_node_names,
        ) = node_identity_snapshot(probe)
        (
            action_clients,
            action_client_participants,
            action_servers,
            query_errors,
        ) = action_endpoint_snapshot(probe, raw_node_identities, action_contracts)
        topic_results, missing_topics, topic_mismatches = evaluate_contracts(
            topic_contracts, topics
        )
        service_results, missing_services, service_mismatches = evaluate_contracts(
            service_contracts, services
        )
        action_results, missing_actions, action_mismatches = evaluate_contracts(
            action_contracts, actions
        )
        ownership_results, ownership_mismatches = evaluate_action_ownership(
            action_contracts,
            expected_action_clients,
            expected_action_servers,
            action_clients,
            action_servers,
        )
        result['attempt_count'] += 1
        result['observed'] = {
            'action_clients': action_clients,
            'action_client_participants': action_client_participants,
            'action_servers': action_servers,
            'actions': actions,
            'node_identities': node_identity_records,
            'node_names': node_names,
            'services': services,
            'topics': topics,
        }
        result['duplicate_node_names'] = duplicate_node_names
        result['node_name_counts'] = node_name_counts
        result['results'] = {
            'action_ownership': ownership_results,
            'actions': action_results,
            'services': service_results,
            'topics': topic_results,
        }
        result['missing_actions'] = missing_actions
        result['missing_services'] = missing_services
        result['missing_topics'] = missing_topics
        result['type_mismatches'] = {
            'actions': action_mismatches,
            'services': service_mismatches,
            'topics': topic_mismatches,
        }
        result['action_ownership_mismatches'] = ownership_mismatches
        result['query_errors'] = query_errors
        if (
            not missing_topics
            and not missing_services
            and not missing_actions
            and not topic_mismatches
            and not service_mismatches
            and not action_mismatches
            and not ownership_mismatches
            and not duplicate_node_names
            and not query_errors
        ):
            result['verdict'] = 'PASS'
            break

        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            if result['duplicate_node_names']:
                result['failure_kind'] = 'duplicate_node_names'
                result['failure'] = (
                    'duplicate node names remained at the graph deadline: '
                    + ', '.join(result['duplicate_node_names'])
                )
            else:
                result['failure_kind'] = 'wall_timeout'
                result['failure'] = (
                    f'graph contracts did not converge within {wall_timeout:.3f} monotonic seconds'
                )
            break
        executor.spin_once(timeout_sec=min(SPIN_QUANTUM_S, remaining))

    result['elapsed_wall_seconds'] = round(time.monotonic() - started, 9)
    return result


def write_evidence(
    result: dict[str, Any],
    output: Path,
    topics_output: Path,
    services_output: Path,
    actions_output: Path,
    nodes_output: Path,
) -> None:
    """Write compatible final snapshots, then atomically commit canonical JSON."""
    observed = result['observed']
    atomic_write_text(topics_output, graph_text(observed['topics']))
    atomic_write_text(services_output, graph_text(observed['services']))
    atomic_write_text(actions_output, graph_text(observed['actions']))
    atomic_write_text(nodes_output, ''.join(f'{name}\n' for name in observed['node_names']))
    atomic_write_text(output, serialize_result(result))


def cleanup_ros(
    executor: SingleThreadedExecutor | None,
    nodes: list[Node],
    action_servers: list[ActionServer] | None = None,
    action_clients: list[ActionClient] | None = None,
) -> None:
    """Destroy every owned ROS entity on success and failure."""
    for client in reversed(action_clients or []):
        with suppress(Exception):  # pragma: no cover - defensive middleware cleanup
            client.destroy()
    for server in reversed(action_servers or []):
        with suppress(Exception):  # pragma: no cover - defensive middleware cleanup
            server.destroy()
    if executor is not None:
        for node in nodes:
            with suppress(Exception):  # pragma: no cover - defensive middleware cleanup
                executor.remove_node(node)
    for node in reversed(nodes):
        with suppress(Exception):  # pragma: no cover - defensive middleware cleanup
            node.destroy_node()
    if executor is not None:
        executor.shutdown(timeout_sec=1.0)
    if rclpy.ok():
        rclpy.shutdown()


def run_live_probe(
    topic_contracts: dict[str, str],
    service_contracts: dict[str, str],
    action_contracts: dict[str, str],
    expected_action_clients: dict[str, list[str]],
    expected_action_servers: dict[str, list[str]],
    wall_timeout: float,
    watch_pid: int | None,
) -> dict[str, Any]:
    """Create exactly one observer node and clean it up deterministically."""
    executor: SingleThreadedExecutor | None = None
    probe: Node | None = None
    rclpy.init()
    try:
        probe = Node(PROBE_NODE_NAME, namespace=PROBE_NAMESPACE)
        executor = SingleThreadedExecutor()
        executor.add_node(probe)
        return probe_graph(
            probe,
            executor,
            topic_contracts,
            service_contracts,
            action_contracts,
            expected_action_clients,
            expected_action_servers,
            wall_timeout,
            watch_pid,
        )
    finally:
        cleanup_ros(executor, [probe] if probe is not None else [])


def run_self_test() -> int:
    """Exercise pure parsing, exact matching, bounds, and serialization."""
    assert parse_contract('/robotest/scan=sensor_msgs/msg/LaserScan', 'msg') == (
        '/robotest/scan',
        'sensor_msgs/msg/LaserScan',
    )
    for invalid in (
        'relative=std_msgs/msg/String',
        '/bad-=std_msgs/msg/String',
        '/ok=std_msgs/action/String',
        '/ok=bad_type',
    ):
        try:
            parse_contract(invalid, 'msg')
        except ValueError:
            pass
        else:
            raise AssertionError(f'accepted invalid contract: {invalid!r}')

    counts, duplicates = node_name_multiplicity(
        ['/robotest/unique', '/robotest/duplicate', '/robotest/duplicate']
    )
    assert counts == {'/robotest/duplicate': 2, '/robotest/unique': 1}
    assert duplicates == ['/robotest/duplicate']

    results, missing, mismatches = evaluate_contracts(
        {'/required': 'std_msgs/msg/String'},
        {'/required': ['std_msgs/msg/String'], '/unrelated': ['std_msgs/msg/String']},
    )
    assert not missing and not mismatches
    assert results['/required']['status'] == 'PASS'
    _, missing, _ = evaluate_contracts({'/missing': 'std_msgs/msg/String'}, {})
    assert missing == ['/missing']
    _, _, mismatches = evaluate_contracts(
        {'/required': 'std_msgs/msg/String'},
        {'/required': ['std_msgs/msg/Bool', 'std_msgs/msg/String']},
    )
    assert mismatches[0]['name'] == '/required'
    ownership, mismatches = evaluate_action_ownership(
        {'/action': 'nav2_msgs/action/FollowWaypoints'},
        {'/action': ['/client']},
        {'/action': ['/server']},
        {'/action': {'/client': ['nav2_msgs/action/FollowWaypoints']}},
        {'/action': {'/server': ['nav2_msgs/action/FollowWaypoints']}},
    )
    assert not mismatches
    assert ownership['/action']['status'] == 'PASS'
    _, mismatches = evaluate_action_ownership(
        {'/action': 'nav2_msgs/action/FollowWaypoints'},
        {'/action': ['/client']},
        {'/action': ['/server']},
        {
            '/action': {
                '/client': ['nav2_msgs/action/FollowWaypoints'],
                '/unexpected_client': ['nav2_msgs/action/FollowWaypoints'],
            }
        },
        {'/action': {'/server': ['nav2_msgs/action/FollowWaypoints']}},
    )
    assert mismatches[0]['observed_clients'].keys() == {'/client', '/unexpected_client'}
    endpoint_contracts = endpoint_contracts_from_values(
        ['/action=/client'],
        {'/action': 'nav2_msgs/action/FollowWaypoints'},
        'client',
    )
    assert endpoint_contracts == {'/action': ['/client']}
    try:
        endpoint_contracts_from_values(
            ['/undeclared=/client'],
            {'/action': 'nav2_msgs/action/FollowWaypoints'},
            'client',
        )
    except ValueError:
        pass
    else:
        raise AssertionError('accepted action endpoint for an undeclared action')
    action_contracts = {'/action': 'nav2_msgs/action/FollowWaypoints'}
    assert goal_action_clients_from_service_clients(action_contracts, []) == {}
    assert (
        goal_action_clients_from_service_clients(
            action_contracts,
            [('/action/_action/cancel_goal', ['action_msgs/srv/CancelGoal'])],
        )
        == {}
    )
    assert goal_action_clients_from_service_clients(
        action_contracts,
        [
            (
                '/action/_action/send_goal',
                ['nav2_msgs/action/FollowWaypoints_SendGoal'],
            )
        ],
    ) == {'/action': ['nav2_msgs/action/FollowWaypoints']}
    assert goal_action_clients_from_service_clients(
        action_contracts,
        [('/action/_action/send_goal', ['example_interfaces/srv/AddTwoInts'])],
    ) == {'/action': ['example_interfaces/srv/AddTwoInts']}
    try:
        goal_action_clients_from_service_clients(
            action_contracts,
            [
                ('/action/_action/send_goal', [f'example_interfaces/srv/Type{index}'])
                for index in range(MAXIMUM_TYPES_PER_NAME + 1)
            ],
        )
    except ValueError:
        pass
    else:
        raise AssertionError('accepted a merged service-client type set above the bound')
    assert graph_text({'/topic': ['std_msgs/msg/String']}) == ('/topic [std_msgs/msg/String]\n')
    try:
        serialize_result({'not_finite': math.nan})
    except ValueError:
        pass
    else:
        raise AssertionError('strict JSON serializer accepted NaN')
    print('phase2_graph_probe self-test PASS')
    return EXIT_PASS


def run_live_smoke_test() -> int:
    """Prove topic, service, action, and endpoint discovery on a fake graph."""
    namespace = f'/robotest_graph_probe_smoke_{os.getpid()}_{time.monotonic_ns()}'
    topic_name = f'{namespace}/smoke_topic'
    service_name = f'{namespace}/smoke_service'
    action_name = f'{namespace}/smoke_action'
    executor: SingleThreadedExecutor | None = None
    nodes: list[Node] = []
    action_servers: list[ActionServer] = []
    action_clients: list[ActionClient] = []
    rclpy.init()
    try:
        server = Node('graph_server', namespace=namespace)
        nodes.append(server)
        publisher = server.create_publisher(String, 'smoke_topic', 1)

        def service_callback(
            _request: Trigger.Request, response: Trigger.Response
        ) -> Trigger.Response:
            response.success = True
            response.message = 'graph smoke service'
            return response

        service = server.create_service(Trigger, 'smoke_service', service_callback)

        def execute_callback(goal_handle: Any) -> FollowWaypoints.Result:
            goal_handle.succeed()
            return FollowWaypoints.Result()

        action_server = ActionServer(
            server,
            FollowWaypoints,
            'smoke_action',
            execute_callback=execute_callback,
        )
        action_servers.append(action_server)
        client = Node('graph_client', namespace=namespace)
        nodes.append(client)
        action_client = ActionClient(client, FollowWaypoints, 'smoke_action')
        action_clients.append(action_client)
        passive = Node('graph_passive_observer', namespace=namespace)
        nodes.append(passive)
        passive_status = passive.create_subscription(
            GoalStatusArray,
            f'{action_name}/_action/status',
            lambda _message: None,
            qos_profile_action_status_default,
        )
        passive_feedback = passive.create_subscription(
            FollowWaypoints.Impl.FeedbackMessage,
            f'{action_name}/_action/feedback',
            lambda _message: None,
            10,
        )
        passive_cancel = passive.create_client(CancelGoal, f'{action_name}/_action/cancel_goal')
        probe = Node(f'{PROBE_NODE_NAME}_{os.getpid()}', namespace=namespace)
        nodes.append(probe)
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client)
        executor.add_node(probe)
        result = probe_graph(
            probe,
            executor,
            {topic_name: 'std_msgs/msg/String'},
            {service_name: 'std_srvs/srv/Trigger'},
            {action_name: 'nav2_msgs/action/FollowWaypoints'},
            {action_name: [client.get_fully_qualified_name()]},
            {action_name: [server.get_fully_qualified_name()]},
            5.0,
            None,
        )
        assert publisher is not None and service is not None
        assert result['schema_version'] == SCHEMA_VERSION == 2
        assert result['duplicate_node_names'] == []
        assert result['node_name_counts'][server.get_fully_qualified_name()] == 1
        assert result['node_name_counts'][client.get_fully_qualified_name()] == 1
        assert result['node_name_counts'][passive.get_fully_qualified_name()] == 1
        assert probe.get_fully_qualified_name() not in result['observed']['node_names']
        participants = result['observed']['action_client_participants'][action_name]
        goal_clients = result['observed']['action_clients'][action_name]
        assert client.get_fully_qualified_name() in participants
        assert client.get_fully_qualified_name() in goal_clients
        assert passive.get_fully_qualified_name() in participants
        assert passive.get_fully_qualified_name() not in goal_clients
        assert passive_status is not None and passive_feedback is not None
        assert passive_cancel is not None
        sys.stdout.write(serialize_result(result))
        return EXIT_PASS if result['verdict'] == 'PASS' else EXIT_PROBE_FAILURE
    finally:
        cleanup_ros(executor, nodes, action_servers, action_clients)


def run_live_duplicate_node_test() -> int:
    """Prove two live public nodes with one FQN are a graph gate failure."""
    namespace = f'/robotest_graph_probe_duplicate_{os.getpid()}_{time.monotonic_ns()}'
    duplicate_fqn = f'{namespace}/duplicate_target'
    executor: SingleThreadedExecutor | None = None
    nodes: list[Node] = []
    rclpy.init()
    try:
        first = Node('duplicate_target', namespace=namespace)
        second = Node('duplicate_target', namespace=namespace)
        probe = Node(f'{PROBE_NODE_NAME}_{os.getpid()}', namespace=namespace)
        nodes.extend((first, second, probe))
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)

        discovery_deadline = time.monotonic() + 5.0
        while time.monotonic() < discovery_deadline:
            _, _, _, counts, _ = node_identity_snapshot(probe)
            if counts.get(duplicate_fqn) == 2:
                break
            executor.spin_once(timeout_sec=SPIN_QUANTUM_S)
        else:
            raise AssertionError('duplicate live nodes were not discovered within 5 seconds')

        result = probe_graph(probe, executor, {}, {}, {}, {}, {}, 0.2, None)
        assert result['verdict'] == 'FAIL'
        assert result['failure_kind'] == 'duplicate_node_names'
        assert result['node_name_counts'][duplicate_fqn] == 2
        assert result['duplicate_node_names'] == [duplicate_fqn]
        sys.stdout.write(serialize_result(result))
        return EXIT_PASS
    finally:
        cleanup_ros(executor, nodes)


def build_parser() -> argparse.ArgumentParser:
    """Build the verifier-facing CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', action='append', dest='topics')
    parser.add_argument('--service', action='append', dest='services')
    parser.add_argument('--action', action='append', dest='actions')
    parser.add_argument('--action-client', action='append', dest='action_clients')
    parser.add_argument('--action-server', action='append', dest='action_servers')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--topics-output', type=Path)
    parser.add_argument('--services-output', type=Path)
    parser.add_argument('--actions-output', type=Path)
    parser.add_argument('--nodes-output', type=Path)
    parser.add_argument('--wall-timeout', type=float, default=90.0)
    parser.add_argument('--watch-pid', type=int)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--self-test', action='store_true')
    mode.add_argument('--live-smoke-test', action='store_true')
    mode.add_argument('--live-duplicate-node-test', action='store_true')
    return parser


def main() -> int:
    """Run a pure test, fake-graph smoke, or bounded live graph proof."""
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    if args.live_smoke_test:
        return run_live_smoke_test()
    if args.live_duplicate_node_test:
        return run_live_duplicate_node_test()
    if not args.topics and not args.services and not args.actions:
        parser.error('at least one --topic, --service, or --action contract is required')
    for name in (
        'output',
        'topics_output',
        'services_output',
        'actions_output',
        'nodes_output',
    ):
        if getattr(args, name) is None:
            parser.error(f'--{name.replace("_", "-")} is required')
    try:
        topic_contracts = contracts_from_values(args.topics, 'msg')
        service_contracts = contracts_from_values(args.services, 'srv')
        action_contracts = contracts_from_values(args.actions, 'action')
        expected_action_clients = endpoint_contracts_from_values(
            args.action_clients, action_contracts, 'client'
        )
        expected_action_servers = endpoint_contracts_from_values(
            args.action_servers, action_contracts, 'server'
        )
    except ValueError as error:
        parser.error(str(error))
    if not math.isfinite(args.wall_timeout) or args.wall_timeout <= 0.0:
        parser.error('--wall-timeout must be finite and positive')
    if args.watch_pid is not None and args.watch_pid <= 0:
        parser.error('--watch-pid must be a positive integer')
    output_paths = [
        args.output,
        args.topics_output,
        args.services_output,
        args.actions_output,
        args.nodes_output,
    ]
    if len(set(output_paths)) != len(output_paths):
        parser.error('JSON and graph text outputs must be distinct')

    try:
        result = run_live_probe(
            topic_contracts,
            service_contracts,
            action_contracts,
            expected_action_clients,
            expected_action_servers,
            args.wall_timeout,
            args.watch_pid,
        )
    except KeyboardInterrupt:
        print('phase2_graph_probe interrupted', file=sys.stderr)
        return 130
    except Exception as error:  # pragma: no cover - middleware/environment-specific
        message = f'phase2_graph_probe internal error: {type(error).__name__}: {error}'
        print(message, file=sys.stderr)
        return EXIT_INTERNAL_ERROR

    try:
        write_evidence(
            result,
            args.output,
            args.topics_output,
            args.services_output,
            args.actions_output,
            args.nodes_output,
        )
    except Exception as error:
        print(f'failed to write graph evidence: {type(error).__name__}: {error}', file=sys.stderr)
        return EXIT_INTERNAL_ERROR
    sys.stdout.write(serialize_result(result))
    return EXIT_PASS if result['verdict'] == 'PASS' else EXIT_PROBE_FAILURE


if __name__ == '__main__':
    raise SystemExit(main())
