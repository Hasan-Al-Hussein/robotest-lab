# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Fast structural contract checks; no simulator is started here."""

from __future__ import annotations

import ast
import dataclasses
import hashlib
import importlib.util
import struct
import sys
import xml.etree.ElementTree as ET
from array import array
from pathlib import Path

import yaml
from nav_msgs.msg import Odometry
from ros_gz_interfaces.msg import Contact, Contacts
from sensor_msgs.msg import Imu, LaserScan

PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
CONTACT_AGGREGATOR_BUILD_INSTALL_FIELDS = frozenset(
    {
        'build_elf_build_id',
        'build_embedded_source_inventory_match',
        'build_embedded_source_inventory_sha256',
        'build_install_build_id_match',
        'build_install_embedded_source_inventory_match',
        'build_install_samefile',
        'build_install_sha256_match',
        'build_path',
        'build_regular_file',
        'build_sha256',
        'installed_declared_is_symlink',
        'installed_declared_path',
        'installed_elf_build_id',
        'installed_embedded_source_inventory_match',
        'installed_embedded_source_inventory_sha256',
        'installed_path',
        'installed_regular_file',
        'installed_sha256',
        'package',
        'schema_version',
        'source_inventory_sha256',
    }
)


def _source_hash_declarations(script_path: Path) -> tuple[list[Path], list[Path]]:
    text = script_path.read_text(encoding='utf-8')
    function = text.split('write_source_config_hashes() {', 1)[1]
    python_source = function.split("<<'PY'\n", 1)[1].split('\nPY\n}', 1)[0]
    tree = ast.parse(python_source, filename=str(script_path))

    assignments: dict[str, list[Path]] = {}
    for statement in tree.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if not isinstance(target, ast.Name) or not isinstance(statement.value, ast.List):
            continue
        paths = [
            WORKSPACE / element.right.value
            for element in statement.value.elts
            if (
                isinstance(element, ast.BinOp)
                and isinstance(element.op, ast.Div)
                and isinstance(element.left, ast.Name)
                and element.left.id == 'workspace'
                and isinstance(element.right, ast.Constant)
                and isinstance(element.right.value, str)
            )
        ]
        assignments[target.id] = paths
    explicit = assignments.get('explicit_files', assignments.get('explicit', []))
    roots = assignments.get('source_roots', assignments.get('roots', []))
    assert explicit and roots
    return explicit, roots


def _source_hash_evidence(
    script_path: Path, overrides: dict[Path, bytes] | None = None
) -> tuple[str, dict[str, str]]:
    explicit, roots = _source_hash_declarations(script_path)
    paths = {path for path in explicit if path.is_file() and not path.is_symlink()}
    for root in roots:
        paths.update(
            path
            for path in root.rglob('*')
            if path.is_file()
            and not path.is_symlink()
            and '__pycache__' not in path.parts
            and path.suffix != '.pyc'
        )
    aggregate = hashlib.sha256()
    entries: dict[str, str] = {}
    for path in sorted(paths):
        relative = path.relative_to(WORKSPACE).as_posix()
        content = overrides.get(path, path.read_bytes()) if overrides else path.read_bytes()
        digest = hashlib.sha256(content).hexdigest()
        entries[relative] = digest
        aggregate.update(relative.encode('utf-8') + b'\0' + digest.encode('ascii') + b'\n')
    return aggregate.hexdigest(), entries


def _runtime_attestor_python_source(script_path: Path) -> str:
    function = script_path.read_text(encoding='utf-8').split(
        'write_contact_gate_runtime_attestation() {', 1
    )[1]
    return function.split("<<'PY'\n", 1)[1].split('\nPY\n}', 1)[0]


def _aggregator_build_install_projection(script_path: Path):
    tree = ast.parse(_runtime_attestor_python_source(script_path), filename=str(script_path))
    fields_assignment = next(
        statement
        for statement in tree.body
        if (
            isinstance(statement, ast.Assign)
            and any(
                isinstance(target, ast.Name) and target.id == 'aggregator_build_install_fields'
                for target in statement.targets
            )
        )
    )
    projection = next(
        statement
        for statement in tree.body
        if (
            isinstance(statement, ast.FunctionDef)
            and statement.name == 'contact_aggregator_build_install_binding'
        )
    )
    selected = ast.Module(body=[fields_assignment, projection], type_ignores=[])
    ast.fix_missing_locations(selected)
    namespace: dict[str, object] = {}
    exec(compile(selected, str(script_path), 'exec'), namespace)
    return (
        namespace['aggregator_build_install_fields'],
        namespace['contact_aggregator_build_install_binding'],
    )


def load_runtime_probe_module():
    path = PACKAGE / 'tools' / 'phase1_runtime_probe.py'
    spec = importlib.util.spec_from_file_location('phase1_runtime_probe', path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_phase1_motion_loop_does_not_throttle_callback_dispatch() -> None:
    """The motion loop must not impose a fixed sleep after each ROS callback."""
    path = PACKAGE / 'tools' / 'phase1_runtime_probe.py'
    tree = ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
    probe_class = next(
        node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Phase1Probe'
    )
    move_test = next(
        node
        for node in probe_class.body
        if isinstance(node, ast.FunctionDef) and node.name == 'move_test'
    )
    calls = [node for node in ast.walk(move_test) if isinstance(node, ast.Call)]
    assert any(
        isinstance(call.func, ast.Attribute) and call.func.attr == 'spin_once' for call in calls
    )
    assert not any(
        isinstance(call.func, ast.Attribute)
        and isinstance(call.func.value, ast.Name)
        and call.func.value.id == 'time'
        and call.func.attr == 'sleep'
        for call in calls
    )


def test_phase1_and_phase2_hash_runtime_attestation_helper_bytes() -> None:
    helpers = (
        WORKSPACE / 'config/collision-coverage.yaml',
        WORKSPACE / 'tests/phase3_runtime_gate.py',
        WORKSPACE / 'tests/phase3_orchestration.py',
    )
    for phase in (1, 2):
        script = WORKSPACE / 'scripts' / f'verify_phase{phase}.sh'
        script_text = script.read_text(encoding='utf-8')
        assert 'source-config-hashes-end.json' in script_text
        assert 'source-config-mutation.json' in script_text
        assert script_text.count('write_source_config_mutation_evidence') >= 2
        explicit, _ = _source_hash_declarations(script)
        assert set(helpers) <= set(explicit)
        baseline_aggregate, baseline_entries = _source_hash_evidence(script)
        for helper in helpers:
            relative = helper.relative_to(WORKSPACE).as_posix()
            mutated_aggregate, mutated_entries = _source_hash_evidence(
                script, {helper: helper.read_bytes() + b'\n# mutation sentinel\n'}
            )
            assert relative in baseline_entries
            assert baseline_entries[relative] != mutated_entries[relative]
            assert baseline_aggregate != mutated_aggregate


def test_phase1_and_phase2_attest_the_loaded_contact_aggregator_dso() -> None:
    for phase in (1, 2):
        script = (WORKSPACE / 'scripts' / f'verify_phase{phase}.sh').read_text(encoding='utf-8')
        assert 'module._contact_aggregator_binary_attestation(' in script
        assert "'contact_aggregator_binary_attestation': aggregator_attestation" in script
        assert "initial['contact_aggregator_binary_attestation']" in script
        assert "get('stable_identity_sha256')" in script
        assert 'contact aggregator runtime identity changed before final evaluation' in script
        registration = 'sys.modules[spec.name] = module'
        execution = 'spec.loader.exec_module(module)'
        assert registration in script
        assert script.index(registration) < script.index(execution)
    phase2 = (WORKSPACE / 'scripts' / f'verify_phase{2}.sh').read_text(encoding='utf-8')
    assert "'library': 'librobotest_contact_aggregator_system.so'" in phase2
    assert "aggregator_record.get('build_install_samefile') is True" in phase2
    assert 'contact aggregator runtime differs from installed source binding' in phase2


def test_provenance_build_timeout_hierarchy_is_bounded_and_nested() -> None:
    build_test = (PACKAGE / 'test/test_contact_stream_gate_build.py').read_text(encoding='utf-8')
    cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    assert 'CONFIGURE_TIMEOUT_SECONDS = 180' in build_test
    assert 'BUILD_TIMEOUT_SECONDS = 360' in build_test
    assert 'TIMEOUT 960' in cmake

    expected_markers = {
        1: 'timeout --signal=TERM --kill-after=10s 1500s \\\n  colcon test',
        2: 'timeout --signal=TERM --kill-after=10s 1500s \\\n  colcon test',
        3: 'run_logged colcon-test 1500s \\\n  colcon test',
        5: 'run_check colcon-test 1500s \\\n  colcon --log-base',
    }
    for phase, marker in expected_markers.items():
        verifier = (WORKSPACE / 'scripts' / f'verify_phase{phase}.sh').read_text(encoding='utf-8')
        assert marker in verifier


def test_runtime_attestor_dynamic_import_supports_dataclass_annotations() -> None:
    tests_path = str(WORKSPACE / 'tests')
    module_name = 'robotest_contact_gate_attestor_structure_test'
    previous_module = sys.modules.get(module_name)
    orchestration_was_loaded = 'phase3_orchestration' in sys.modules
    sys.path.insert(0, tests_path)
    try:
        spec = importlib.util.spec_from_file_location(
            module_name,
            WORKSPACE / 'tests/phase3_runtime_gate.py',
        )
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        assert dataclasses.is_dataclass(module.ProcMapEntry)
    finally:
        sys.path.remove(tests_path)
        if previous_module is None:
            sys.modules.pop(module_name, None)
        else:
            sys.modules[module_name] = previous_module
        if not orchestration_was_loaded:
            sys.modules.pop('phase3_orchestration', None)


def test_phase1_and_phase2_fail_closed_on_aggregator_binding_drift() -> None:
    for phase in (1, 2):
        script_path = WORKSPACE / 'scripts' / f'verify_phase{phase}.sh'
        source = _runtime_attestor_python_source(script_path)
        fields, projection = _aggregator_build_install_projection(script_path)
        assert len(fields) == len(CONTACT_AGGREGATOR_BUILD_INSTALL_FIELDS) == 21
        assert set(fields) == CONTACT_AGGREGATOR_BUILD_INSTALL_FIELDS
        assert "get('installed_declared_is_symlink') is True" in source
        assert "get('build_install_samefile') is True" in source
        assert 'initial_aggregator_build_install != aggregator_build_install' in source
        assert "initial.get('contact_aggregator_build_install_sha256')" in source

        baseline_attestation = {field: f'baseline:{field}' for field in fields}
        baseline_binding = projection(baseline_attestation)
        for field in fields:
            tampered = dict(baseline_attestation)
            tampered[field] = f'tampered:{field}'
            assert projection(tampered) != baseline_binding

    phase2 = (WORKSPACE / 'scripts' / f'verify_phase{2}.sh').read_text(encoding='utf-8')
    assert 'and aggregator_installed_declared.is_symlink()' in phase2
    assert "initial.get('installed_source_binding_sha256')" in phase2
    assert "!= result['installed_source_binding_sha256']" in phase2
    assert 'installed source binding changed before final evaluation' in phase2


def load_sim_launch_module():
    path = PACKAGE / 'launch' / 'sim.launch.py'
    spec = importlib.util.spec_from_file_location('robotest_sim_launch', path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_package_is_apache_ament_cmake_and_installs_runtime_assets() -> None:
    root = ET.parse(PACKAGE / 'package.xml').getroot()
    assert root.findtext('name') == 'robotest_sim'
    assert root.findtext('license') == 'Apache-2.0'
    assert root.find('./export/build_type').text == 'ament_cmake'
    runtime_dependencies = {
        element.text for tag in ('depend', 'exec_depend') for element in root.findall(tag)
    }
    assert {'robotest_interfaces', 'ros_gz_interfaces', 'tf2_msgs'} <= runtime_dependencies
    cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    for directory in ('config', 'launch', 'models', 'rviz', 'worlds'):
        assert directory in cmake
    assert 'phase1_runtime_probe.py' in cmake


def test_contact_aggregator_avoids_unbounded_raw_message_staging() -> None:
    source = (PACKAGE / 'src' / 'contact_aggregator_system.cpp').read_text(encoding='utf-8')
    post_update = source.split('  void PostUpdate(', 1)[1].split('  void Reset(', 1)[0]

    assert 'ContactAggregateSources current_sources{};' in post_update
    assert 'current_sources[index] = &contacts->Data();' in post_update
    assert 'policy_.observe(stamp_ns, current_sources)' in post_update
    assert 'CopyFrom' not in post_update
    assert 'gz::msgs::Contacts aggregate' not in post_update


def test_world_is_local_enclosed_and_deterministic_friendly() -> None:
    world_path = PACKAGE / 'worlds' / 'robotest_lab.sdf'
    text = world_path.read_text(encoding='utf-8')
    root = ET.fromstring(text)
    assert root.attrib['version'] == '1.10'
    world = root.find('world')
    assert world is not None and world.attrib['name'] == 'robotest_lab'
    physics = world.find('physics')
    assert physics is not None
    assert float(physics.findtext('max_step_size')) == 0.002
    assert float(physics.findtext('real_time_factor')) == 1.0
    assert int(physics.findtext('real_time_update_rate')) == 500

    plugins = {item.attrib['filename'] for item in world.findall('plugin')}
    required = {
        'gz-sim-physics-system',
        'gz-sim-user-commands-system',
        'gz-sim-scene-broadcaster-system',
        'robotest_contact_aggregator_system',
        'gz-sim-sensors-system',
        'gz-sim-imu-system',
    }
    assert required <= plugins
    assert 'gz-sim-contact-system' not in plugins
    aggregator = world.find("./plugin[@name='robotest_sim::ContactAggregatorSystem']")
    assert aggregator is not None
    assert aggregator.attrib['filename'] == 'robotest_contact_aggregator_system'
    assert aggregator.findtext('output_topic') == '/robotest/internal/contact_aggregate'
    assert int(aggregator.findtext('publish_period_ns', default='0')) == 20_000_000
    assert aggregator.findtext('robot_model_name') == 'robotest'
    assert world.findall('include') == []
    assert 'fuel.gazebosim' not in text.lower()
    assert 'http://' not in text.lower() and 'https://' not in text.lower()
    model_names = {model.attrib['name'] for model in world.findall('model')}
    assert {'wall_north', 'wall_south', 'wall_east', 'wall_west'} <= model_names
    assert len({name for name in model_names if name.startswith('obstacle_')}) >= 3
    heartbeat = world.find(
        "./model[@name='ground_plane']/plugin[@name='gz::sim::systems::PosePublisher']"
    )
    assert heartbeat is not None
    assert heartbeat.attrib['filename'] == 'gz-sim-pose-publisher-system'
    assert heartbeat.findtext('publish_model_pose') == 'true'
    assert heartbeat.findtext('publish_link_pose') == 'false'
    assert heartbeat.findtext('publish_collision_pose') == 'false'
    assert heartbeat.findtext('publish_visual_pose') == 'false'
    assert heartbeat.findtext('publish_sensor_pose') == 'false'
    assert heartbeat.findtext('publish_nested_model_pose') == 'false'
    assert heartbeat.findtext('use_pose_vector_msg') == 'true'
    assert float(heartbeat.findtext('update_frequency')) == 10.0
    assert heartbeat.findtext('static_publisher') == 'false'
    assert heartbeat.find('topic') is None


def test_bridge_matches_the_frozen_data_plane() -> None:
    bridges = yaml.safe_load((PACKAGE / 'config' / 'bridge.yaml').read_text(encoding='utf-8'))
    assert isinstance(bridges, list)
    scenario_pose_bridges = [
        entry for entry in bridges if entry['ros_topic_name'] == 'validation/scenario_entity_poses'
    ]
    by_ros_name = {
        entry['ros_topic_name']: entry
        for entry in bridges
        if entry['ros_topic_name'] != 'validation/scenario_entity_poses'
    }
    expected = {
        '/clock': ('rosgraph_msgs/msg/Clock', 'gz.msgs.Clock', 'GZ_TO_ROS', 'CLOCK'),
        'raw/scan': (
            'sensor_msgs/msg/LaserScan',
            'gz.msgs.LaserScan',
            'GZ_TO_ROS',
            'SENSOR_DATA',
        ),
        'raw/odom': (
            'nav_msgs/msg/Odometry',
            'gz.msgs.Odometry',
            'GZ_TO_ROS',
            'SENSOR_DATA',
        ),
        'raw/imu': ('sensor_msgs/msg/Imu', 'gz.msgs.IMU', 'GZ_TO_ROS', 'SENSOR_DATA'),
        'joint_states': (
            'sensor_msgs/msg/JointState',
            'gz.msgs.Model',
            'GZ_TO_ROS',
            'SENSOR_DATA',
        ),
        'cmd_vel': ('geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ', 'SERVICES'),
        'validation/ground_truth': (
            'nav_msgs/msg/Odometry',
            'gz.msgs.Odometry',
            'GZ_TO_ROS',
            'SERVICES',
        ),
        'internal/raw_contacts': (
            'ros_gz_interfaces/msg/Contacts',
            'gz.msgs.Contacts',
            'GZ_TO_ROS',
            'SERVICES',
        ),
        'validation/world_stats': (
            'ros_gz_interfaces/msg/WorldStatistics',
            'gz.msgs.WorldStatistics',
            'GZ_TO_ROS',
            'SERVICES',
        ),
    }
    assert set(by_ros_name) == set(expected)
    for topic, (ros_type, gz_type, direction, qos_profile) in expected.items():
        entry = by_ros_name[topic]
        assert entry['ros_type_name'] == ros_type
        assert entry['gz_type_name'] == gz_type
        assert entry['direction'] == direction
        assert 1 <= int(entry['publisher_queue']) <= 100
        assert 1 <= int(entry['subscriber_queue']) <= 100
        assert entry['qos_profile'] == qos_profile

    assert by_ros_name['internal/raw_contacts']['gz_topic_name'] == (
        '/robotest/internal/contact_aggregate'
    )

    assert {entry['gz_topic_name'] for entry in scenario_pose_bridges} == {
        '/model/ground_plane/pose',
        '/model/phase3_static_block/pose',
        '/model/phase3_dynamic_block/pose',
        '/model/phase3_contact_control_wall/pose',
    }
    for entry in scenario_pose_bridges:
        assert entry['ros_type_name'] == 'tf2_msgs/msg/TFMessage'
        assert entry['gz_type_name'] == 'gz.msgs.Pose_V'
        assert entry['direction'] == 'GZ_TO_ROS'
        assert entry['publisher_queue'] == 10
        assert entry['subscriber_queue'] == 10
        assert entry['qos_profile'] == 'SERVICES'


def test_launch_uses_context_safe_headless_logic_and_all_core_stages() -> None:
    launch_path = PACKAGE / 'launch' / 'sim.launch.py'
    text = launch_path.read_text(encoding='utf-8')
    tree = ast.parse(text, filename=str(launch_path))
    assert 'headless ==' not in text
    for token in (
        'OpaqueFunction',
        'gz_sim.launch.py',
        'robot_state_publisher',
        'parameter_bridge',
        'scenario_bridge',
        'ros_gz_interfaces/srv/SpawnEntity',
        'ros_gz_interfaces/srv/SetEntityPose',
        'ros_gz_interfaces/srv/DeleteEntity',
        'scenario/spawn_entity',
        'scenario/set_entity_pose',
        'scenario/delete_entity',
        'fault_proxy.launch.py',
        'TimerAction',
        'spawn_robotest',
        'use_sim_time',
        'GZ_SIM_RESOURCE_PATH',
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        'get_package_prefix',
        "'lib', 'robotest_sim'",
        'rviz2',
    ):
        assert token in text
    string_constants = {
        node.targets[0].id: node.value.value
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
        and isinstance(node.value.value, str)
    }
    defaults: dict[str, str] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Name):
            continue
        if node.func.id != 'DeclareLaunchArgument' or not node.args:
            continue
        argument = node.args[0]
        default = next(
            (keyword.value for keyword in node.keywords if keyword.arg == 'default_value'),
            None,
        )
        if isinstance(argument, ast.Constant) and isinstance(argument.value, str):
            if isinstance(default, ast.Constant) and isinstance(default.value, str):
                defaults[argument.value] = default.value
            elif isinstance(default, ast.Name) and default.id in string_constants:
                defaults[argument.value] = string_constants[default.id]

    assert defaults['headless'] == 'true'
    assert defaults['rviz'] == 'true'
    assert defaults['seed'] == '42'

    gz_args_assignments = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == 'gz_args' for target in node.targets)
    ]
    assert len(gz_args_assignments) == 1
    gz_args_value = gz_args_assignments[0].value
    assert isinstance(gz_args_value, ast.List)
    assert any(
        isinstance(element, ast.Constant) and element.value == '--seed'
        for element in gz_args_value.elts
    )
    assert any(
        isinstance(element, ast.Call)
        and isinstance(element.func, ast.Name)
        and element.func.id == 'str'
        and len(element.args) == 1
        and isinstance(element.args[0], ast.Name)
        and element.args[0].id == 'simulator_seed'
        for element in gz_args_value.elts
    )


def test_phase3_actor_assets_are_fixed_static_boxes_with_observed_pose_publishers() -> None:
    expected = {
        'phase3_static_block': (0.40, 0.40, 0.80),
        'phase3_dynamic_block': (0.35, 0.35, 0.80),
        'phase3_contact_control_wall': (0.10, 0.80, 0.80),
    }
    for model_name, expected_size in expected.items():
        root = ET.parse(PACKAGE / 'models' / f'{model_name}.sdf').getroot()
        assert root.attrib['version'] == '1.10'
        model = root.find('model')
        assert model is not None and model.attrib['name'] == model_name
        assert model.findtext('static') == 'true'

        collision = model.find("./link[@name='link']/collision[@name='collision']")
        assert collision is not None
        size = tuple(float(value) for value in collision.findtext('./geometry/box/size').split())
        assert size == expected_size

        plugin = model.find("./plugin[@name='gz::sim::systems::PosePublisher']")
        assert plugin is not None
        assert plugin.attrib['filename'] == 'gz-sim-pose-publisher-system'
        assert plugin.findtext('publish_model_pose') == 'true'
        assert plugin.findtext('publish_link_pose') == 'false'
        assert plugin.findtext('use_pose_vector_msg') == 'true'
        assert float(plugin.findtext('update_frequency')) == 10.0
        assert plugin.findtext('static_publisher') == 'false'
        assert plugin.find('topic') is None


def test_launch_rejects_invalid_simulator_seeds() -> None:
    module = load_sim_launch_module()
    assert module._parse_simulator_seed('0') == 0
    assert module._parse_simulator_seed('42') == 42
    assert module._parse_simulator_seed(str(2**32 - 1)) == 2**32 - 1

    for invalid in ('not-an-integer', '-1', str(2**32)):
        try:
            module._parse_simulator_seed(invalid)
        except RuntimeError as error:
            assert str(error) == 'seed must be an integer in [0, 4294967295]'
        else:
            raise AssertionError(f'invalid simulator seed was accepted: {invalid}')


def test_generated_rviz_covers_robot_tf_and_phase1_sensors() -> None:
    text = (PACKAGE / 'rviz' / 'robotest.rviz').read_text(encoding='utf-8')
    assert 'Generated with ros2-sim/scripts/gen_rviz_config.py' in text
    assert 'rviz_default_plugins/RobotModel' in text
    assert 'Value: /robotest/robot_description' in text
    assert 'rviz_default_plugins/TF' in text
    assert 'rviz_default_plugins/LaserScan' in text
    assert 'Value: /robotest/scan' in text
    assert 'rviz_default_plugins/Odometry' in text
    assert 'Value: /robotest/odom' in text
    assert 'rviz_default_plugins/Imu' not in text
    readme = (PACKAGE / 'README.md').read_text(encoding='utf-8')
    assert 'rqt_plot /robotest/imu' in readme
    assert text.count('Reliability Policy: Best Effort') >= 2


def test_runtime_probe_has_bounded_motion_and_final_zero_guard() -> None:
    path = PACKAGE / 'tools' / 'phase1_runtime_probe.py'
    text = path.read_text(encoding='utf-8')
    ast.parse(text, filename=str(path))
    assert 'deadline = time.monotonic() + 12.0' in text
    assert 'publish_zero' in text
    assert 'KEEP_ALL' in text
    assert 'unauthorized validation subscriber' in text
    assert 'odom->base_footprint' in text
    assert 'command_period_wall = 0.1' in text
    assert "'simulation'" in text
    assert 'qos_introspection' in text
    assert '/robotest/robot_state_publisher' in text
    assert '/robotest/fault_proxy' in text
    assert 'publisher_gid' not in text
    assert 'tf_edges_by_publisher_gid' not in text
    assert 'serialize_message' not in text
    assert 'pass_through_semantic_matches' in text
    assert '/robotest/faults/events' in text
    assert 'qos_policy_failures' in text
    assert 'sensor_stamp_evidence' in text
    assert 'motion_trace' in text
    assert 'CONTACT_EVIDENCE_CAPACITY' in text
    assert 'unexpected_contact_pair_count' in text


def test_runtime_probe_contact_policy_matches_frozen_coverage_manifest() -> None:
    module = load_runtime_probe_module()
    manifest = yaml.safe_load(
        (PACKAGE.parents[1] / 'config' / 'collision-coverage.yaml').read_text(encoding='utf-8')
    )
    expected_robot_collisions = {entry['name'] for entry in manifest['robot_collisions']}
    expected_by_role = {entry['role']: entry['name'] for entry in manifest['robot_collisions']}
    expected_support_pairs = {
        (entry['robot_collision'], entry['environment_collision'])
        for entry in manifest['support_pairs']
    }

    assert expected_robot_collisions == module.PHASE1_ROBOT_COLLISIONS
    assert expected_by_role == module.PHASE1_COLLISION_BY_ROLE
    assert expected_support_pairs == module.PHASE1_SUPPORT_PAIRS
    assert module.phase1_contact_policy_sha256() == (
        'efdc004d1db75d4e7a1933db96124c1d02bf4a707f4d61ecf69b1b6183db8eac'
    )


def test_runtime_probe_contact_classifier_allows_only_support_and_internal_pairs() -> None:
    module = load_runtime_probe_module()
    support_robot, ground = next(iter(module.PHASE1_SUPPORT_PAIRS))
    chassis = next(
        collision
        for collision in module.PHASE1_ROBOT_COLLISIONS
        if 'base_link_collision' in collision
    )
    lidar = next(
        collision for collision in module.PHASE1_ROBOT_COLLISIONS if 'lidar_link' in collision
    )
    obstacle = 'obstacle_1::link::collision'

    assert (
        module.classify_phase1_contact_pair(support_robot, ground) == 'allowlisted_support_contact'
    )
    assert (
        module.classify_phase1_contact_pair(ground, support_robot) == 'allowlisted_support_contact'
    )
    assert module.classify_phase1_contact_pair(chassis, lidar) == 'robot_internal'
    assert (
        module.classify_phase1_contact_pair(chassis, ground)
        == 'unexpected_robot_environment_contact'
    )
    assert (
        module.classify_phase1_contact_pair(support_robot, obstacle)
        == 'unexpected_robot_environment_contact'
    )
    assert module.classify_phase1_contact_pair(obstacle, ground) == 'unexpected_non_robot_pair'
    assert module.classify_phase1_contact_pair('', ground) == 'invalid_collision_name'
    assert module.classify_phase1_contact_pair(None, ground) == 'invalid_collision_name'
    assert module.classify_phase1_contact_pair('model::collision', ground) == (
        'invalid_collision_name'
    )
    assert module.classify_phase1_contact_pair('model::::collision', ground) == (
        'invalid_collision_name'
    )
    assert module.valid_public_contact_record_count(1)
    assert module.valid_public_contact_record_count(16)
    assert not module.valid_public_contact_record_count(0)
    assert not module.valid_public_contact_record_count(17)


def test_runtime_probe_contact_evidence_names_are_bounded() -> None:
    module = load_runtime_probe_module()
    oversized = 'x' * (module.CONTACT_NAME_EVIDENCE_LIMIT + 100)
    retained = module.contact_name_evidence(oversized)

    assert retained.endswith('...')
    assert len(retained) == module.CONTACT_NAME_EVIDENCE_LIMIT + 3
    assert module.contact_name_evidence(None) == '<non-string:NoneType>'


def test_runtime_probe_contact_callback_counts_and_bounds_unexpected_prefix() -> None:
    module = load_runtime_probe_module()
    support_robot, ground = next(iter(module.PHASE1_SUPPORT_PAIRS))
    chassis = module.PHASE1_COLLISION_BY_ROLE['chassis']
    lidar = module.PHASE1_COLLISION_BY_ROLE['lidar_body']

    def contact(collision1: str, collision2: str) -> Contact:
        value = Contact()
        value.collision1.name = collision1
        value.collision2.name = collision2
        return value

    message = Contacts()
    message.contacts = [
        contact(support_robot, ground),
        contact(chassis, lidar),
        *[
            contact(support_robot, f'obstacle_{index}::link::collision')
            for index in range(module.CONTACT_EVIDENCE_CAPACITY + 1)
        ],
    ]

    class ProbeState:
        def __init__(self) -> None:
            self.counts: dict[str, int] = {}
            self.nonempty_contact_messages = 0
            self.contact_frame_id_violation_count = 0
            self.contact_record_count_violation_count = 0
            self.contact_same_pair_set_interval_violation_count = 0
            self.contact_minimum_same_pair_set_interval_ns: int | None = None
            self.previous_contact_pair_set = None
            self.previous_contact_pair_set_stamp_ns = None
            self.contact_record_count = 0
            self.contact_dispositions = {
                disposition: 0 for disposition in module.PHASE1_CONTACT_DISPOSITIONS
            }
            self.unexpected_contact_pair_count = 0
            self.unexpected_contact_pair_prefix: list[dict[str, object]] = []
            self.unexpected_contact_pair_omitted_count = 0

        def _record(self, key: str, _stamp_ns: int) -> None:
            self.counts[key] = self.counts.get(key, 0) + 1

    state = ProbeState()
    module.Phase1Probe._on_contacts(state, message)

    assert state.nonempty_contact_messages == 1
    assert state.contact_record_count == module.CONTACT_EVIDENCE_CAPACITY + 3
    assert {key: value for key, value in state.contact_dispositions.items() if value} == {
        'allowlisted_support_contact': 1,
        'robot_internal': 1,
        'unexpected_robot_environment_contact': module.CONTACT_EVIDENCE_CAPACITY + 1,
    }
    assert state.unexpected_contact_pair_count == module.CONTACT_EVIDENCE_CAPACITY + 1
    assert len(state.unexpected_contact_pair_prefix) == module.CONTACT_EVIDENCE_CAPACITY
    assert state.unexpected_contact_pair_omitted_count == 1
    assert state.contact_record_count_violation_count == 1

    module.Phase1Probe._on_contacts(state, Contacts())
    assert state.contact_record_count_violation_count == 2


def test_semantic_fingerprint_ignores_nan_payload_but_detects_value_changes() -> None:
    module = load_runtime_probe_module()
    first = LaserScan()
    second = LaserScan()
    first.header.frame_id = second.header.frame_id = 'lidar_link'
    first.angle_increment = second.angle_increment = 0.25
    nan_payload_one = struct.unpack('>f', bytes.fromhex('7fc00001'))[0]
    nan_payload_two = struct.unpack('>f', bytes.fromhex('7fc01234'))[0]
    first.ranges = array('f', [1.0, nan_payload_one, float('inf'), float('-inf')])
    second.ranges = array('f', [1.0, nan_payload_two, float('inf'), float('-inf')])

    assert module.semantic_fingerprint(first) == module.semantic_fingerprint(second)
    second.ranges = array('f', [1.0000001192092896, nan_payload_two, float('inf'), -float('inf')])
    assert module.semantic_fingerprint(first) != module.semantic_fingerprint(second)

    first_odom = Odometry()
    second_odom = Odometry()
    first_odom.pose.pose.position.x = second_odom.pose.pose.position.x = -0.0
    assert module.semantic_fingerprint(first_odom) == module.semantic_fingerprint(second_odom)
    second_odom.pose.pose.position.x = 0.0
    assert module.semantic_fingerprint(first_odom) != module.semantic_fingerprint(second_odom)

    first_imu = Imu()
    second_imu = Imu()
    first_imu.angular_velocity.z = second_imu.angular_velocity.z = float('inf')
    assert module.semantic_fingerprint(first_imu) == module.semantic_fingerprint(second_imu)
    second_imu.angular_velocity.z = float('-inf')
    assert module.semantic_fingerprint(first_imu) != module.semantic_fingerprint(second_imu)


def test_unknown_qos_introspection_never_claims_bounded_depth_proof() -> None:
    module = load_runtime_probe_module()
    snapshot = {
        'publishers': [
            {
                'node': '/publisher',
                'gid': '01',
                'qos': {'history': 'UNKNOWN', 'depth': 0},
            }
        ],
        'subscribers': [],
    }
    status = module.Phase1Probe.qos_introspection_status(snapshot)
    assert status['complete'] is False
    assert status['bounded_depth_live_proven'] is False
    assert status['incomplete_endpoints'][0]['reasons'] == ['history=UNKNOWN', 'depth=0']


def test_qos_policy_checks_publishers_and_subscribers() -> None:
    module = load_runtime_probe_module()
    snapshot = {
        'publishers': [
            {
                'node': '/publisher',
                'qos': {'reliability': 'RELIABLE', 'durability': 'VOLATILE'},
            }
        ],
        'subscribers': [
            {
                'node': '/subscriber',
                'qos': {'reliability': 'BEST_EFFORT', 'durability': 'TRANSIENT_LOCAL'},
            }
        ],
    }
    failures = module.qos_policy_failures('/test', snapshot, 'RELIABLE', 'VOLATILE')
    assert failures == [
        '/test subscriber /subscriber reliability BEST_EFFORT != RELIABLE',
        '/test subscriber /subscriber durability TRANSIENT_LOCAL != VOLATILE',
    ]


def test_stamp_tracker_reports_order_gap_and_staleness_bounds() -> None:
    module = load_runtime_probe_module()
    tracker = module.StampTracker()
    tracker.observe(1_000_000_000, 1_050_000_000)
    tracker.observe(1_100_000_000, 1_150_000_000)
    tracker.observe(1_200_000_000, 1_250_000_000)
    evidence = tracker.evidence(1_260_000_000, 0.15, 0.10, 0.10)
    assert evidence['regression_count'] == 0
    assert evidence['duplicate_count'] == 0
    assert evidence['maximum_forward_gap_s'] == 0.1
    assert evidence['maximum_receipt_age_s'] == 0.05
    assert evidence['final_age_s'] == 0.06
    assert module.stamp_evidence_failures('/sensor', evidence) == []

    tracker.observe(1_150_000_000, 1_300_000_000)
    tracker.observe(1_150_000_000, 1_300_000_000)
    violated = tracker.evidence(1_500_000_000, 0.15, 0.10, 0.10)
    failures = module.stamp_evidence_failures('/sensor', violated)
    assert any('timestamp regressions' in failure for failure in failures)
    assert any('duplicate timestamps' in failure for failure in failures)
    assert any('maximum receipt age' in failure for failure in failures)
    assert any('final stamp age' in failure for failure in failures)


def test_contact_stamp_tracker_accepts_220ms_and_rejects_one_ns_more() -> None:
    module = load_runtime_probe_module()
    boundary = module.StampTracker()
    boundary.observe(1_000_000_000, 1_000_000_000)
    boundary.observe(1_220_000_000, 1_220_000_000)
    boundary.observe(1_440_000_000, 1_440_000_000)
    evidence = boundary.evidence(1_440_000_000, 0.22, 0.22, 0.22)
    assert module.stamp_evidence_failures('/robotest/validation/contacts', evidence) == []

    exceeded = module.StampTracker()
    exceeded.observe(1_000_000_000, 1_000_000_000)
    exceeded.observe(1_220_000_000, 1_220_000_000)
    exceeded.observe(1_440_000_001, 1_440_000_001)
    evidence = exceeded.evidence(1_440_000_001, 0.22, 0.22, 0.22)
    assert any(
        'maximum stamp gap' in failure
        for failure in module.stamp_evidence_failures('/robotest/validation/contacts', evidence)
    )


def test_passive_contact_cached_clock_offset_is_diagnostic() -> None:
    module = load_runtime_probe_module()
    tracker = module.StampTracker()
    tracker.observe(10_000_000_000, 10_278_000_000, receipt_wall_s=10.0)
    tracker.observe(10_200_000_000, 10_300_000_000, receipt_wall_s=10.25)
    tracker.observe(10_400_000_000, 10_448_000_000, receipt_wall_s=10.50)
    evidence = tracker.evidence(
        10_448_000_000,
        0.22,
        0.22,
        0.22,
        module.CONTACT_PUBLIC_MAX_WALL_INTER_RECEIPT_GAP_S,
    )

    assert evidence['maximum_forward_gap_ns'] == 200_000_000
    assert evidence['maximum_receipt_age_ns'] == 278_000_000
    assert evidence['final_age_ns'] == 48_000_000
    assert (
        module.stamp_evidence_failures(
            '/robotest/validation/contacts',
            evidence,
            enforce_callback_clock_offset=False,
        )
        == []
    )
    assert any(
        'maximum receipt age' in failure
        for failure in module.stamp_evidence_failures('/robotest/validation/contacts', evidence)
    )


def test_passive_contact_source_final_and_wall_liveness_stay_fail_closed() -> None:
    module = load_runtime_probe_module()
    source_gap = module.StampTracker()
    source_gap.observe(1_000_000_000, 1_000_000_000, receipt_wall_s=10.0)
    source_gap.observe(1_220_000_001, 1_220_000_001, receipt_wall_s=10.1)
    source_gap.observe(1_440_000_001, 1_440_000_001, receipt_wall_s=10.2)
    evidence = source_gap.evidence(1_440_000_001, 0.22, 0.22, 0.22, 2.0)
    failures = module.stamp_evidence_failures(
        '/robotest/validation/contacts',
        evidence,
        enforce_callback_clock_offset=False,
    )
    assert any('maximum stamp gap' in failure for failure in failures)

    final_age = module.StampTracker()
    final_age.observe(1_000_000_000, 1_000_000_000, receipt_wall_s=10.0)
    final_age.observe(1_200_000_000, 1_200_000_000, receipt_wall_s=10.1)
    final_age.observe(1_400_000_000, 1_400_000_000, receipt_wall_s=10.2)
    boundary = final_age.evidence(1_620_000_000, 0.22, 0.22, 0.22, 2.0)
    assert (
        module.stamp_evidence_failures(
            '/robotest/validation/contacts',
            boundary,
            enforce_callback_clock_offset=False,
        )
        == []
    )
    evidence = final_age.evidence(1_620_000_001, 0.22, 0.22, 0.22, 2.0)
    failures = module.stamp_evidence_failures(
        '/robotest/validation/contacts',
        evidence,
        enforce_callback_clock_offset=False,
    )
    assert any('final stamp age' in failure for failure in failures)

    wall_gap = module.StampTracker()
    wall_gap.observe(1_000_000_000, 1_000_000_000, receipt_wall_s=10.0)
    wall_gap.observe(1_200_000_000, 1_200_000_000, receipt_wall_s=12.000_001)
    wall_gap.observe(1_400_000_000, 1_400_000_000, receipt_wall_s=12.1)
    evidence = wall_gap.evidence(1_400_000_000, 0.22, 0.22, 0.22, 2.0)
    failures = module.stamp_evidence_failures(
        '/robotest/validation/contacts',
        evidence,
        enforce_callback_clock_offset=False,
    )
    assert any('wall inter-receipt gap' in failure for failure in failures)


def test_windowed_rtf_uses_multi_sample_non_overlapping_intervals() -> None:
    module = load_runtime_probe_module()
    samples = [
        (index * 100_000_000, index * 0.125 + (0.03 if index % 2 else 0.0), False, 0.8)
        for index in range(11)
    ]
    values = module.windowed_rtf(samples, sample_span=5)
    assert len(values) == 2
    assert all(value > 0.7 for value in values)
