# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Pure tests for the Phase 3 live contact-aggregator DSO attestation."""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

import pytest

TESTS_ROOT = Path(__file__).resolve().parent

ORCHESTRATION_SPEC = importlib.util.spec_from_file_location(
    'phase3_orchestration',
    TESTS_ROOT / 'phase3_orchestration.py',
)
assert ORCHESTRATION_SPEC is not None and ORCHESTRATION_SPEC.loader is not None
orchestration = importlib.util.module_from_spec(ORCHESTRATION_SPEC)
sys.modules[ORCHESTRATION_SPEC.name] = orchestration
ORCHESTRATION_SPEC.loader.exec_module(orchestration)

RUNTIME_GATE_SPEC = importlib.util.spec_from_file_location(
    'phase3_runtime_gate_dso_test_module',
    TESTS_ROOT / 'phase3_runtime_gate.py',
)
assert RUNTIME_GATE_SPEC is not None and RUNTIME_GATE_SPEC.loader is not None
runtime_gate = importlib.util.module_from_spec(RUNTIME_GATE_SPEC)
sys.modules[RUNTIME_GATE_SPEC.name] = runtime_gate
RUNTIME_GATE_SPEC.loader.exec_module(runtime_gate)


def _write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def _fake_workspace(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    symlink_install: bool = True,
) -> tuple[Path, dict]:
    workspace = tmp_path / 'workspace'
    for index, relative in enumerate(runtime_gate.CONTACT_PIPELINE_SOURCE_PATHS, start=1):
        _write(workspace / relative, f'source-{index}\n'.encode())
    source_inventory_sha256 = runtime_gate._contact_gate_source_inventory_sha256(workspace)
    dso = workspace / runtime_gate.CONTACT_AGGREGATOR_BUILD_PATH
    _write(
        dso,
        b'\x7fELF-test\0'
        + runtime_gate.CONTACT_GATE_SOURCE_TAG
        + source_inventory_sha256.encode('ascii')
        + b'\0',
    )
    installed = workspace / runtime_gate.CONTACT_AGGREGATOR_INSTALLED_PATH
    installed.parent.mkdir(parents=True, exist_ok=True)
    if symlink_install:
        installed.symlink_to(dso.resolve())
    else:
        installed.write_bytes(dso.read_bytes())
    monkeypatch.setattr(runtime_gate, '_elf_build_id', lambda _path: 'deadbeef')
    return workspace, runtime_gate._contact_aggregator_build_install_record(workspace)


def _proc_stat_line(pid: int, ppid: int, pgid: int, sid: int, start_ticks: int) -> bytes:
    fields = ['S', str(ppid), str(pgid), str(sid), *(['0'] * 15), str(start_ticks)]
    return f'{pid} (gz sim server) '.encode() + ' '.join(fields).encode() + b'\n'


def _device_text(device: int) -> str:
    return f'{os.major(device):02x}:{os.minor(device):02x}'


def _dso_maps(
    dso: Path,
    *,
    deleted: bool = False,
    executable: bool = True,
    offset_zero: bool = True,
    address_shift: int = 0,
) -> bytes:
    stat = dso.stat()
    device = _device_text(stat.st_dev)
    path = str(dso) + (' (deleted)' if deleted else '')
    first_offset = 0 if offset_zero else 0x1000
    execute_permissions = 'r-xp' if executable else 'r--p'
    rows = (
        (0x1000 + address_shift, 0x2000 + address_shift, 'r--p', first_offset),
        (0x2000 + address_shift, 0x3000 + address_shift, execute_permissions, 0x1000),
        (0x3000 + address_shift, 0x4000 + address_shift, 'rw-p', 0x2000),
    )
    return ''.join(
        f'{start:08x}-{end:08x} {permissions} {offset:08x} {device} {stat.st_ino} {path}\n'
        for start, end, permissions, offset in rows
    ).encode()


def _unrelated_maps(path: str = '/fake/launch') -> bytes:
    return f'00400000-00401000 r--p 00000000 00:00 0 {path}\n'.encode()


class _Policy:
    def __init__(self, name: str) -> None:
        self.name = name


class _GraphQos:
    def __init__(self, topic: str) -> None:
        reliability, durability, _depth = runtime_gate.QOS_CONTRACTS[topic]
        self.depth = 0
        self.durability = _Policy(durability)
        self.history = _Policy('UNKNOWN')
        self.reliability = _Policy(reliability)


class _GraphEndpoint:
    def __init__(self, topic: str, node: str, topic_type: str, gid: int) -> None:
        namespace, name = node.rsplit('/', 1)
        self.endpoint_gid = bytes.fromhex(f'{gid:032x}')
        self.node_name = name
        self.node_namespace = namespace
        self.qos_profile = _GraphQos(topic)
        self.topic_type = topic_type


_TOPIC_TYPES = {
    '/clock': runtime_gate.CLOCK_MESSAGE_TYPE,
    '/robotest/cmd_vel': 'geometry_msgs/msg/Twist',
    '/robotest/cmd_vel_behavior_unused': 'geometry_msgs/msg/Twist',
    '/robotest/cmd_vel_nav': 'geometry_msgs/msg/Twist',
    '/robotest/cmd_vel_smoothed': 'geometry_msgs/msg/Twist',
    '/robotest/collision_monitor_state': 'nav2_msgs/msg/CollisionMonitorState',
    '/robotest/faults/events': 'robotest_interfaces/msg/FaultEvent',
    '/robotest/imu': 'sensor_msgs/msg/Imu',
    '/robotest/internal/raw_contacts': 'ros_gz_interfaces/msg/Contacts',
    '/robotest/map': 'nav_msgs/msg/OccupancyGrid',
    '/robotest/navigation/plan': 'nav_msgs/msg/Path',
    '/robotest/odom': 'nav_msgs/msg/Odometry',
    '/robotest/raw/imu': 'sensor_msgs/msg/Imu',
    '/robotest/raw/odom': 'nav_msgs/msg/Odometry',
    '/robotest/raw/scan': 'sensor_msgs/msg/LaserScan',
    '/robotest/scan': 'sensor_msgs/msg/LaserScan',
    '/robotest/validation/contacts': 'ros_gz_interfaces/msg/Contacts',
    '/robotest/validation/ground_truth': 'nav_msgs/msg/Odometry',
    '/robotest/validation/scenario_entity_poses': 'tf2_msgs/msg/TFMessage',
    '/robotest/validation/world_stats': 'ros_gz_interfaces/msg/WorldStatistics',
    '/tf': 'tf2_msgs/msg/TFMessage',
    '/tf_static': 'tf2_msgs/msg/TFMessage',
}


class _PositiveGraph:
    def __init__(self) -> None:
        source_contracts = runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS
        layouts = {
            '/clock': (
                [('/robotest/parameter_bridge', source_contracts['/clock'][1])],
                [],
            ),
            '/robotest/cmd_vel': (
                [('/robotest/contact_control_driver', runtime_gate.COMMAND_MESSAGE_TYPE)],
                [
                    ('/robotest/metrics_collector', runtime_gate.COMMAND_MESSAGE_TYPE),
                    ('/robotest/parameter_bridge', runtime_gate.COMMAND_MESSAGE_TYPE),
                ],
            ),
            '/robotest/internal/raw_contacts': (
                [('/robotest/parameter_bridge', runtime_gate.CONTACT_MESSAGE_TYPE)],
                [('/robotest/contact_stream_gate', runtime_gate.CONTACT_MESSAGE_TYPE)],
            ),
            '/robotest/validation/contacts': (
                [('/robotest/contact_stream_gate', runtime_gate.CONTACT_MESSAGE_TYPE)],
                [],
            ),
            '/robotest/validation/ground_truth': (
                [
                    (
                        '/robotest/parameter_bridge',
                        source_contracts['/robotest/validation/ground_truth'][1],
                    )
                ],
                [],
            ),
            '/robotest/validation/scenario_entity_poses': (
                [
                    (
                        '/robotest/parameter_bridge',
                        source_contracts['/robotest/validation/scenario_entity_poses'][1],
                    )
                ]
                * 4,
                [],
            ),
            '/robotest/validation/world_stats': (
                [
                    (
                        '/robotest/parameter_bridge',
                        source_contracts['/robotest/validation/world_stats'][1],
                    )
                ],
                [],
            ),
        }
        self.publishers: dict[str, list[_GraphEndpoint]] = {}
        self.subscribers: dict[str, list[_GraphEndpoint]] = {}
        gid = 1
        for topic, (publishers, subscribers) in layouts.items():
            self.publishers[topic] = []
            self.subscribers[topic] = []
            for node, topic_type in publishers:
                self.publishers[topic].append(_GraphEndpoint(topic, node, topic_type, gid))
                gid += 1
            for node, topic_type in subscribers:
                self.subscribers[topic].append(_GraphEndpoint(topic, node, topic_type, gid))
                gid += 1

    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        names = {f'/robotest/{name}' for name in runtime_gate.POSITIVE_REQUIRED_NODE_NAMES}
        for endpoints in (*self.publishers.values(), *self.subscribers.values()):
            for endpoint in endpoints:
                names.add(f'{endpoint.node_namespace}/{endpoint.node_name}')
        return [
            (fully_qualified.rsplit('/', 1)[1], fully_qualified.rsplit('/', 1)[0])
            for fully_qualified in sorted(names)
        ]

    def get_service_names_and_types(self) -> list[tuple[str, list[str]]]:
        return [
            ('/robotest/scenario/delete_entity', ['ros_gz_interfaces/srv/DeleteEntity']),
            ('/robotest/scenario/spawn_entity', ['ros_gz_interfaces/srv/SpawnEntity']),
        ]

    def get_publishers_info_by_topic(self, topic: str) -> list[_GraphEndpoint]:
        return self.publishers[topic]

    def get_subscriptions_info_by_topic(self, topic: str) -> list[_GraphEndpoint]:
        return self.subscribers[topic]


class _CandidateGraph:
    def __init__(self, *, entity_publisher_count: int = 4) -> None:
        entity_topic = '/robotest/validation/scenario_entity_poses'
        self.publishers: dict[str, list[_GraphEndpoint]] = {
            topic: [] for topic in runtime_gate.QOS_CONTRACTS
        }
        self.subscribers: dict[str, list[_GraphEndpoint]] = {
            topic: [] for topic in runtime_gate.QOS_CONTRACTS
        }
        gid = 1
        for topic, nodes in runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS.items():
            cardinality_per_node = entity_publisher_count if topic == entity_topic else 1
            for node in sorted(nodes):
                for _index in range(cardinality_per_node):
                    self.publishers[topic].append(
                        _GraphEndpoint(topic, node, _TOPIC_TYPES[topic], gid)
                    )
                    gid += 1

        subscriber_nodes = {
            topic: set(nodes)
            for topic, nodes in runtime_gate.CANDIDATE_EXPECTED_COMMAND_SUBSCRIBERS.items()
        }
        for topic, nodes in runtime_gate.CANDIDATE_EXPECTED_CONTACT_SUBSCRIBERS.items():
            subscriber_nodes.setdefault(topic, set()).update(nodes)
        for topic, nodes in {
            '/clock': runtime_gate.CANDIDATE_FUSED_CLOCK_SUBSCRIBERS,
            '/robotest/validation/contacts': {'/robotest/metrics_collector'},
            '/robotest/validation/ground_truth': {
                '/robotest/metrics_collector',
                '/robotest/scenario_controller',
            },
            entity_topic: {'/robotest/scenario_controller'},
            '/robotest/validation/world_stats': {'/robotest/metrics_collector'},
        }.items():
            subscriber_nodes.setdefault(topic, set()).update(nodes)
        for topic, nodes in subscriber_nodes.items():
            for node in sorted(nodes):
                self.subscribers[topic].append(
                    _GraphEndpoint(topic, node, _TOPIC_TYPES[topic], gid)
                )
                gid += 1

    def get_node_names_and_namespaces(self) -> list[tuple[str, str]]:
        names = {f'/robotest/{name}' for name in runtime_gate.CANDIDATE_REQUIRED_NODES}
        for endpoints in (*self.publishers.values(), *self.subscribers.values()):
            for endpoint in endpoints:
                names.add(f'{endpoint.node_namespace}/{endpoint.node_name}')
        return [
            (fully_qualified.rsplit('/', 1)[1], fully_qualified.rsplit('/', 1)[0])
            for fully_qualified in sorted(names)
        ]

    def get_service_names_and_types(self) -> list[tuple[str, list[str]]]:
        return [(service, ['test_msgs/srv/Fixture']) for service in runtime_gate.SCENARIO_SERVICES]

    def get_publishers_info_by_topic(self, topic: str) -> list[_GraphEndpoint]:
        return self.publishers[topic]

    def get_subscriptions_info_by_topic(self, topic: str) -> list[_GraphEndpoint]:
        return self.subscribers[topic]


def _write_process(
    proc_root: Path,
    pid: int,
    *,
    ppid: int,
    pgid: int,
    sid: int,
    start_ticks: int,
    maps: bytes,
    executable: Path,
    environment: bytes = b'',
) -> None:
    directory = proc_root / str(pid)
    directory.mkdir(parents=True)
    _write(directory / 'stat', _proc_stat_line(pid, ppid, pgid, sid, start_ticks))
    _write(directory / 'maps', maps)
    _write(directory / 'cmdline', b'gz\0sim\0-s\0robotest_lab.sdf\0')
    _write(directory / 'environ', environment)
    (directory / 'exe').symlink_to(executable.resolve())


def _fake_proc_tree(
    tmp_path: Path,
    dso: Path,
    *,
    child_maps: bytes | None = None,
) -> tuple[Path, int, int]:
    proc_root = tmp_path / 'proc'
    launch_pid = 100
    live_pid = 101
    host = tmp_path / 'gz-server-host'
    _write(host, b'fake Gazebo host executable')
    _write_process(
        proc_root,
        launch_pid,
        ppid=1,
        pgid=launch_pid,
        sid=launch_pid,
        start_ticks=1000,
        maps=_unrelated_maps(),
        executable=host,
    )
    _write_process(
        proc_root,
        live_pid,
        ppid=launch_pid,
        pgid=launch_pid,
        sid=launch_pid,
        start_ticks=2000,
        maps=child_maps if child_maps is not None else _dso_maps(dso),
        executable=host,
        environment=b'ROS_DOMAIN_ID=77\0GZ_PARTITION=robotest-test\0',
    )
    return proc_root, launch_pid, live_pid


def _assert_only_entity_publisher_ownership_failed(evidence: dict) -> None:
    entity_topic = '/robotest/validation/scenario_entity_poses'
    assert evidence['publisher_ownership'][entity_topic] is False
    assert all(
        accepted
        for topic, accepted in evidence['publisher_ownership'].items()
        if topic != entity_topic
    )


def test_candidate_and_positive_graphs_share_the_authoritative_source_contract() -> None:
    expected = {
        '/clock': ({'/robotest/parameter_bridge'}, 'rosgraph_msgs/msg/Clock', 1),
        '/robotest/validation/ground_truth': (
            {'/robotest/parameter_bridge'},
            'nav_msgs/msg/Odometry',
            1,
        ),
        '/robotest/validation/scenario_entity_poses': (
            {'/robotest/parameter_bridge'},
            'tf2_msgs/msg/TFMessage',
            4,
        ),
        '/robotest/validation/world_stats': (
            {'/robotest/parameter_bridge'},
            'ros_gz_interfaces/msg/WorldStatistics',
            1,
        ),
    }

    assert expected == runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS
    assert all(
        runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS[topic] == nodes
        for topic, (nodes, _topic_type, _cardinality) in expected.items()
    )


def test_candidate_graph_accepts_four_distinct_entity_pose_publishers() -> None:
    graph = _CandidateGraph()

    passed, evidence = runtime_gate._candidate_evaluation(graph)

    assert passed
    assert set(evidence['publisher_ownership']) == set(runtime_gate.CANDIDATE_EXPECTED_PUBLISHERS)
    assert all(evidence['publisher_ownership'].values())
    assert evidence['fused_clock_subscriber_ownership_pass']
    clock_subscribers = evidence['topics']['/clock']['subscribers']
    assert {subscriber['node'] for subscriber in clock_subscribers} == (
        runtime_gate.CANDIDATE_FUSED_CLOCK_SUBSCRIBERS
    )
    assert len(clock_subscribers) == len(runtime_gate.CANDIDATE_FUSED_CLOCK_SUBSCRIBERS)
    assert len({subscriber['gid'] for subscriber in clock_subscribers}) == len(clock_subscribers)
    entity_publishers = evidence['topics']['/robotest/validation/scenario_entity_poses'][
        'publishers'
    ]
    assert len(entity_publishers) == 4
    assert {publisher['node'] for publisher in entity_publishers} == {'/robotest/parameter_bridge'}
    assert {publisher['topic_type'] for publisher in entity_publishers} == {
        'tf2_msgs/msg/TFMessage'
    }
    assert len({publisher['gid'] for publisher in entity_publishers}) == 4


@pytest.mark.parametrize(
    ('node_name', 'mutation'),
    (
        ('/robotest/metrics_collector', 'missing'),
        ('/robotest/scenario_controller', 'missing'),
        ('/robotest/metrics_collector', 'duplicate'),
        ('/robotest/scenario_controller', 'duplicate'),
        ('/robotest/metrics_collector', 'wrong_type'),
        ('/robotest/scenario_controller', 'wrong_type'),
        ('/robotest/metrics_collector', 'malformed_gid'),
        ('/robotest/scenario_controller', 'cross_subscriber_duplicate_gid'),
    ),
)
def test_candidate_graph_rejects_invalid_fused_clock_reader(
    node_name: str,
    mutation: str,
) -> None:
    graph = _CandidateGraph()
    clock_subscribers = graph.subscribers['/clock']
    matching = next(
        endpoint
        for endpoint in clock_subscribers
        if f'{endpoint.node_namespace}/{endpoint.node_name}' == node_name
    )
    next_gid = (
        max(
            int.from_bytes(endpoint.endpoint_gid, byteorder='big')
            for endpoints in (*graph.publishers.values(), *graph.subscribers.values())
            for endpoint in endpoints
        )
        + 1
    )
    if mutation == 'missing':
        clock_subscribers.remove(matching)
    elif mutation == 'duplicate':
        clock_subscribers.append(
            _GraphEndpoint(
                '/clock',
                node_name,
                runtime_gate.CLOCK_MESSAGE_TYPE,
                next_gid,
            )
        )
    elif mutation == 'wrong_type':
        matching.topic_type = 'std_msgs/msg/String'
    elif mutation == 'malformed_gid':
        matching.endpoint_gid = b'\xab'
    else:
        assert mutation == 'cross_subscriber_duplicate_gid'
        other_reader = _GraphEndpoint(
            '/clock',
            '/robotest/amcl',
            runtime_gate.CLOCK_MESSAGE_TYPE,
            next_gid,
        )
        clock_subscribers.append(other_reader)
        matching.endpoint_gid = other_reader.endpoint_gid

    passed, evidence = runtime_gate._candidate_evaluation(graph)

    assert not passed
    assert evidence['fused_clock_subscriber_ownership_pass'] is False
    assert evidence['qos_contract_pass'] is True
    assert all(evidence['publisher_ownership'].values())


@pytest.mark.parametrize('publisher_count', (3, 5))
def test_candidate_graph_rejects_entity_pose_publisher_cardinality_drift(
    publisher_count: int,
) -> None:
    graph = _CandidateGraph(entity_publisher_count=publisher_count)

    passed, evidence = runtime_gate._candidate_evaluation(graph)

    assert not passed
    _assert_only_entity_publisher_ownership_failed(evidence)


def test_candidate_graph_rejects_duplicate_entity_pose_publisher_gid() -> None:
    graph = _CandidateGraph()
    publishers = graph.publishers['/robotest/validation/scenario_entity_poses']
    publishers[1].endpoint_gid = publishers[0].endpoint_gid

    passed, evidence = runtime_gate._candidate_evaluation(graph)

    assert not passed
    _assert_only_entity_publisher_ownership_failed(evidence)


@pytest.mark.parametrize('mutation', ('wrong_node', 'wrong_type'))
def test_candidate_graph_rejects_entity_pose_publisher_identity_drift(mutation: str) -> None:
    graph = _CandidateGraph()
    publisher = graph.publishers['/robotest/validation/scenario_entity_poses'][0]
    if mutation == 'wrong_node':
        publisher.node_name = 'rogue_parameter_bridge'
    else:
        publisher.topic_type = 'std_msgs/msg/String'

    passed, evidence = runtime_gate._candidate_evaluation(graph)

    assert not passed
    _assert_only_entity_publisher_ownership_failed(evidence)


def test_positive_graph_requires_the_configured_authoritative_publishers() -> None:
    graph = _PositiveGraph()

    passed, evidence = runtime_gate._positive_evaluation(graph)

    assert passed
    assert evidence['authoritative_publisher_ownership'] == {
        topic: True for topic in runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS
    }
    entity_publishers = evidence['topics']['/robotest/validation/scenario_entity_poses'][
        'publishers'
    ]
    assert len(entity_publishers) == 4
    assert {publisher['node'] for publisher in entity_publishers} == {'/robotest/parameter_bridge'}
    assert {publisher['topic_type'] for publisher in entity_publishers} == {
        'tf2_msgs/msg/TFMessage'
    }
    assert len({publisher['gid'] for publisher in entity_publishers}) == 4
    assert evidence['qos_contract_pass']
    assert not evidence['qos_introspection_complete']


@pytest.mark.parametrize('topic', tuple(runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS))
@pytest.mark.parametrize('mutation', ('extra', 'missing', 'wrong_type'))
def test_positive_graph_rejects_authoritative_publisher_drift(
    topic: str,
    mutation: str,
) -> None:
    graph = _PositiveGraph()
    _expected_nodes, expected_type, _expected_cardinality = (
        runtime_gate.AUTHORITATIVE_PUBLISHER_CONTRACTS[topic]
    )
    if mutation == 'extra':
        graph.publishers[topic].append(
            _GraphEndpoint(topic, '/robotest/rogue_source', expected_type, 10_000)
        )
    elif mutation == 'missing':
        graph.publishers[topic].pop()
    else:
        graph.publishers[topic][0].topic_type = 'std_msgs/msg/String'

    passed, evidence = runtime_gate._positive_evaluation(graph)

    assert not passed
    assert evidence['authoritative_publisher_ownership'][topic] is False
    assert all(
        accepted
        for other_topic, accepted in evidence['authoritative_publisher_ownership'].items()
        if other_topic != topic
    )


def test_positive_graph_rejects_duplicate_authoritative_publisher_gid() -> None:
    graph = _PositiveGraph()
    topic = '/robotest/validation/scenario_entity_poses'
    graph.publishers[topic][1].endpoint_gid = graph.publishers[topic][0].endpoint_gid

    passed, evidence = runtime_gate._positive_evaluation(graph)

    assert not passed
    assert evidence['authoritative_publisher_ownership'][topic] is False


def test_positive_graph_rejects_cross_topic_authoritative_publisher_gid() -> None:
    graph = _PositiveGraph()
    graph.publishers['/robotest/validation/ground_truth'][0].endpoint_gid = graph.publishers[
        '/clock'
    ][0].endpoint_gid

    passed, evidence = runtime_gate._positive_evaluation(graph)

    assert not passed
    assert evidence['authoritative_publisher_ownership']['/clock'] is False
    assert (
        evidence['authoritative_publisher_ownership']['/robotest/validation/ground_truth'] is False
    )


def test_contact_pipeline_source_inventory_is_the_canonical_eleven_files() -> None:
    assert runtime_gate.CONTACT_PIPELINE_SOURCE_PATHS == (
        'src/robotest_description/urdf/robotest_gazebo.xacro',
        'src/robotest_sim/CMakeLists.txt',
        'src/robotest_sim/config/bridge.yaml',
        'src/robotest_sim/include/robotest_sim/contact_aggregator.hpp',
        'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
        'src/robotest_sim/launch/sim.launch.py',
        'src/robotest_sim/src/contact_aggregator.cpp',
        'src/robotest_sim/src/contact_aggregator_system.cpp',
        'src/robotest_sim/src/contact_stream_gate.cpp',
        'src/robotest_sim/src/contact_stream_gate_node.cpp',
        'src/robotest_sim/worlds/robotest_lab.sdf',
    )


def test_proc_maps_parser_preserves_exact_device_inode_and_spaced_path(tmp_path: Path) -> None:
    mapped = tmp_path / 'path with spaces' / 'librobotest_contact_aggregator_system.so'
    _write(mapped, b'dso')
    entries = runtime_gate._parse_proc_maps(_dso_maps(mapped))

    assert len(entries) == 3
    assert {entry.device for entry in entries} == {mapped.stat().st_dev}
    assert {entry.inode for entry in entries} == {mapped.stat().st_ino}
    assert {entry.path for entry in entries} == {str(mapped)}
    assert any(entry.offset == 0 for entry in entries)
    assert any('x' in entry.permissions for entry in entries)


@pytest.mark.parametrize(
    'payload',
    [
        b'not-a-maps-line\n',
        b'00002000-00001000 r--p 00000000 00:00 0 /bad/range\n',
        b'00001000-00002000 rwxq 00000000 00:00 0 /bad/perms\n',
    ],
)
def test_proc_maps_parser_rejects_malformed_records(payload: bytes) -> None:
    with pytest.raises(orchestration.EvidenceError, match='/proc maps line'):
        runtime_gate._parse_proc_maps(payload)


def test_proc_maps_parser_rejects_oversized_input() -> None:
    with pytest.raises(orchestration.EvidenceError, match='byte bound'):
        runtime_gate._parse_proc_maps(b'x' * (runtime_gate.MAX_PROC_MAPS_BYTES + 1))


@pytest.mark.parametrize(
    ('options', 'message'),
    [
        ({'deleted': True}, 'deleted live mapping'),
        ({'offset_zero': False}, 'offset-zero segment'),
        ({'executable': False}, 'executable segment'),
    ],
)
def test_dso_owner_selection_rejects_invalid_mapping_sets(
    tmp_path: Path,
    options: dict[str, bool],
    message: str,
) -> None:
    dso = tmp_path / 'librobotest_contact_aggregator_system.so'
    _write(dso, b'dso')
    proc_root, _launch_pid, live_pid = _fake_proc_tree(
        tmp_path,
        dso,
        child_maps=_dso_maps(dso, **options),
    )
    with pytest.raises(orchestration.EvidenceError, match=message):
        runtime_gate._contact_aggregator_map_owners(
            proc_root,
            {live_pid},
            installed_device=dso.stat().st_dev,
            installed_inode=dso.stat().st_ino,
        )


def test_dso_owner_selection_rejects_multiple_launch_descendants(tmp_path: Path) -> None:
    dso = tmp_path / 'librobotest_contact_aggregator_system.so'
    _write(dso, b'dso')
    proc_root, launch_pid, live_pid = _fake_proc_tree(tmp_path, dso)
    host = tmp_path / 'gz-server-host'
    second_pid = 102
    _write_process(
        proc_root,
        second_pid,
        ppid=launch_pid,
        pgid=launch_pid,
        sid=launch_pid,
        start_ticks=3000,
        maps=_dso_maps(dso, address_shift=0x10000),
        executable=host,
        environment=b'ROS_DOMAIN_ID=77\0GZ_PARTITION=robotest-test\0',
    )

    with pytest.raises(orchestration.EvidenceError, match='exactly one'):
        runtime_gate._one_contact_aggregator_map_owner(
            proc_root,
            {live_pid, second_pid},
            installed_device=dso.stat().st_dev,
            installed_inode=dso.stat().st_ino,
        )


def test_build_install_record_accepts_an_identical_copied_dso(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    _workspace, record = _fake_workspace(
        tmp_path,
        monkeypatch,
        symlink_install=False,
    )

    assert record['build_install_samefile'] is False
    assert record['installed_declared_is_symlink'] is False
    assert record['build_install_sha256_match'] is True
    assert record['build_install_build_id_match'] is True
    assert record['build_install_embedded_source_inventory_match'] is True


def test_build_install_record_rejects_a_symlink_to_a_distinct_copy(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace, _record = _fake_workspace(
        tmp_path,
        monkeypatch,
        symlink_install=False,
    )
    build = workspace / runtime_gate.CONTACT_AGGREGATOR_BUILD_PATH
    installed = workspace / runtime_gate.CONTACT_AGGREGATOR_INSTALLED_PATH
    copied = workspace / 'alternate' / build.name
    _write(copied, build.read_bytes())
    installed.unlink()
    installed.symlink_to(copied.resolve())

    with pytest.raises(
        orchestration.EvidenceError,
        match='symlink install does not resolve to the build artifact',
    ):
        runtime_gate._contact_aggregator_build_install_record(workspace)


def test_contact_aggregator_attestation_is_stable_across_ready_and_final(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace, record = _fake_workspace(tmp_path, monkeypatch)
    dso = (workspace / record['installed_path']).resolve()
    proc_root, launch_pid, live_pid = _fake_proc_tree(tmp_path, dso)

    ready = runtime_gate._contact_aggregator_binary_attestation(
        workspace,
        launch_pid,
        77,
        'robotest-test',
        build_install_record=record,
        proc_root=proc_root,
    )
    final = runtime_gate._contact_aggregator_binary_attestation(
        workspace,
        launch_pid,
        77,
        'robotest-test',
        build_install_record=record,
        proc_root=proc_root,
    )

    assert ready['verdict'] == 'PASS'
    assert ready['live_pid'] == live_pid
    assert ready['live_mapping_count'] == 3
    assert ready['live_mapping_has_offset_zero'] is True
    assert ready['live_mapping_has_executable'] is True
    assert ready['live_installed_inode_match'] is True
    assert ready['maps_revalidated_after_hashing'] is True
    assert ready['identity_revalidated_after_hashing'] is True
    assert ready['installed_identity_revalidated_after_hashing'] is True
    assert ready['stable_identity'] == final['stable_identity']
    assert ready['stable_identity_sha256'] == final['stable_identity_sha256']


def test_contact_aggregator_attestation_rejects_maps_toctou(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace, record = _fake_workspace(tmp_path, monkeypatch)
    dso = (workspace / record['installed_path']).resolve()
    proc_root, launch_pid, live_pid = _fake_proc_tree(tmp_path, dso)
    original_read = runtime_gate._read_proc_maps
    live_reads = 0

    def changed_second_live_read(root: Path, pid: int):
        nonlocal live_reads
        if pid != live_pid:
            return original_read(root, pid)
        live_reads += 1
        if live_reads == 1:
            return original_read(root, pid)
        return runtime_gate._parse_proc_maps(_dso_maps(dso, address_shift=0x10000))

    monkeypatch.setattr(runtime_gate, '_read_proc_maps', changed_second_live_read)
    attestation = runtime_gate._contact_aggregator_binary_attestation(
        workspace,
        launch_pid,
        77,
        'robotest-test',
        build_install_record=record,
        proc_root=proc_root,
    )

    assert attestation['verdict'] == 'FAIL'
    assert attestation['maps_revalidated_after_hashing'] is False


def test_contact_aggregator_attestation_rejects_start_tick_toctou(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace, record = _fake_workspace(tmp_path, monkeypatch)
    dso = (workspace / record['installed_path']).resolve()
    proc_root, launch_pid, live_pid = _fake_proc_tree(tmp_path, dso)
    original_fields = runtime_gate._proc_process_fields
    live_reads = 0

    def changed_second_live_identity(root: Path, pid: int):
        nonlocal live_reads
        fields = original_fields(root, pid)
        if pid != live_pid:
            return fields
        live_reads += 1
        if live_reads == 1:
            return fields
        return fields[0], fields[1], fields[2], fields[3] + 1

    monkeypatch.setattr(runtime_gate, '_proc_process_fields', changed_second_live_identity)
    attestation = runtime_gate._contact_aggregator_binary_attestation(
        workspace,
        launch_pid,
        77,
        'robotest-test',
        build_install_record=record,
        proc_root=proc_root,
    )

    assert attestation['verdict'] == 'FAIL'
    assert attestation['identity_revalidated_after_hashing'] is False


def test_contact_aggregator_attestation_rejects_wrong_process_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    workspace, record = _fake_workspace(tmp_path, monkeypatch)
    dso = (workspace / record['installed_path']).resolve()
    proc_root, launch_pid, _live_pid = _fake_proc_tree(tmp_path, dso)

    attestation = runtime_gate._contact_aggregator_binary_attestation(
        workspace,
        launch_pid,
        78,
        'robotest-test',
        build_install_record=record,
        proc_root=proc_root,
    )

    assert attestation['verdict'] == 'FAIL'
    assert attestation['process_identity_match'] is False
