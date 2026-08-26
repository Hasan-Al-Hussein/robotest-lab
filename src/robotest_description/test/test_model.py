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

import hashlib
import pathlib
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import yaml

PACKAGE_ROOT = pathlib.Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
MODEL_XACRO = PACKAGE_ROOT / 'urdf' / 'robotest.urdf.xacro'
COLLISION_COVERAGE = REPOSITORY_ROOT / 'config' / 'collision-coverage.yaml'
CONTACT_SENSORS = {
    'chassis_contact_sensor': (
        'base_footprint',
        'base_footprint_fixed_joint_lump__base_link_collision_collision',
    ),
    'left_wheel_contact_sensor': (
        'left_wheel_link',
        'left_wheel_link_fixed_joint_lump__left_wheel_collision_collision',
    ),
    'right_wheel_contact_sensor': (
        'right_wheel_link',
        'right_wheel_link_fixed_joint_lump__right_wheel_collision_collision',
    ),
    'front_caster_contact_sensor': (
        'front_caster_link',
        'front_caster_link_fixed_joint_lump__front_caster_collision_collision',
    ),
    'rear_caster_contact_sensor': (
        'rear_caster_link',
        'rear_caster_link_fixed_joint_lump__rear_caster_collision_collision',
    ),
    'lidar_body_contact_sensor': ('lidar_link', 'lidar_link_collision_collision'),
    'imu_body_contact_sensor': ('imu_link', 'imu_link_collision_collision'),
}


def _render(
    namespace: str = '/robotest',
    use_gazebo: bool = True,
    enable_ground_truth: bool = True,
) -> str:
    command = [
        'xacro',
        str(MODEL_XACRO),
        f'namespace:={namespace}',
        f'use_gazebo:={str(use_gazebo).lower()}',
        f'enable_ground_truth:={str(enable_ground_truth).lower()}',
    ]
    return subprocess.run(
        command,
        check=True,
        text=True,
        capture_output=True,
    ).stdout


@pytest.fixture(scope='module')
def robot_xml() -> ET.Element:
    return ET.fromstring(_render())


def _plugins(robot_xml: ET.Element) -> dict[str, ET.Element]:
    return {plugin.attrib['name']: plugin for plugin in robot_xml.findall('./gazebo/plugin')}


def _sensor(robot_xml: ET.Element, name: str) -> ET.Element:
    matches = [
        sensor for sensor in robot_xml.findall('./gazebo/sensor') if sensor.attrib['name'] == name
    ]
    assert len(matches) == 1
    return matches[0]


def test_xacro_expands_and_urdfdom_accepts_tree() -> None:
    check_urdf = shutil.which('check_urdf')
    assert check_urdf is not None

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as output:
        output.write(_render())
        output.flush()
        result = subprocess.run(
            [check_urdf, output.name],
            check=True,
            text=True,
            capture_output=True,
        )

    assert 'Successfully Parsed XML' in result.stdout
    assert 'root Link: base_footprint' in result.stdout


def test_frame_and_joint_contract(robot_xml: ET.Element) -> None:
    links = {link.attrib['name'] for link in robot_xml.findall('link')}
    assert links == {
        'base_footprint',
        'base_link',
        'left_wheel_link',
        'right_wheel_link',
        'front_caster_link',
        'rear_caster_link',
        'lidar_link',
        'imu_link',
    }

    joints = {joint.attrib['name']: joint for joint in robot_xml.findall('joint')}
    base_joint = joints['base_footprint_joint']
    assert base_joint.attrib['type'] == 'fixed'
    assert base_joint.find('parent').attrib['link'] == 'base_footprint'
    assert base_joint.find('child').attrib['link'] == 'base_link'

    for side in ('left', 'right'):
        wheel_joint = joints[f'{side}_wheel_joint']
        assert wheel_joint.attrib['type'] == 'continuous'
        assert wheel_joint.find('parent').attrib['link'] == 'base_link'
        assert wheel_joint.find('axis').attrib['xyz'] == '0 1 0'

    assert joints['front_caster_joint'].find('child').attrib['link'] == ('front_caster_link')
    assert joints['rear_caster_joint'].find('child').attrib['link'] == ('rear_caster_link')


def test_every_physical_link_has_valid_dynamics(robot_xml: ET.Element) -> None:
    physical_links = [
        link for link in robot_xml.findall('link') if link.attrib['name'] != 'base_footprint'
    ]
    assert physical_links

    for link in physical_links:
        assert link.find('visual') is not None, link.attrib['name']
        assert link.find('collision') is not None, link.attrib['name']
        inertial = link.find('inertial')
        assert inertial is not None, link.attrib['name']

        mass = float(inertial.find('mass').attrib['value'])
        assert mass > 0.0, link.attrib['name']

        inertia = inertial.find('inertia')
        moments = [
            float(inertia.attrib['ixx']),
            float(inertia.attrib['iyy']),
            float(inertia.attrib['izz']),
        ]
        assert all(moment > 0.0 for moment in moments), link.attrib['name']
        assert moments[0] <= moments[1] + moments[2] + 1e-12
        assert moments[1] <= moments[0] + moments[2] + 1e-12
        assert moments[2] <= moments[0] + moments[1] + 1e-12


def test_sensor_contract(robot_xml: ET.Element) -> None:
    lidar = _sensor(robot_xml, 'lidar')
    assert lidar.attrib['type'] == 'gpu_lidar'
    assert lidar.findtext('topic') == '/robotest/raw/scan'
    assert lidar.findtext('gz_frame_id') == 'lidar_link'
    assert lidar.findtext('update_rate') == '5'
    assert lidar.findtext('lidar/scan/horizontal/samples') == '360'
    assert lidar.findtext('lidar/scan/horizontal/min_angle') == str(-3.141592653589793)
    assert lidar.findtext('lidar/scan/horizontal/max_angle') == str(3.141592653589793)

    imu = _sensor(robot_xml, 'imu')
    assert imu.attrib['type'] == 'imu'
    assert imu.findtext('topic') == '/robotest/raw/imu'
    assert imu.findtext('gz_frame_id') == 'imu_link'
    assert imu.findtext('update_rate') == '50'

    for sensor_name, (_, collision_name) in CONTACT_SENSORS.items():
        contact = _sensor(robot_xml, sensor_name)
        assert contact.attrib['type'] == 'contact'
        assert contact.findtext('update_rate') == '5'
        assert contact.findtext('contact/topic') == '/robotest/validation/contacts'
        assert contact.findtext('contact/collision') == collision_name


def test_collision_coverage_manifest_matches_contact_sensors() -> None:
    coverage = yaml.safe_load(COLLISION_COVERAGE.read_text(encoding='utf-8'))
    entries = coverage['robot_collisions']

    assert coverage['schema_version'] == 3
    assert coverage['robot_model'] == 'robotest'
    assert coverage['contact_topic'] == '/robotest/validation/contacts'
    contact_stream = coverage['contact_stream']
    assert contact_stream['schema_version'] == 1
    assert contact_stream['topics'] == {
        'gazebo_raw': '/robotest/validation/contacts',
        'private_raw_ros': '/robotest/internal/raw_contacts',
        'public_ros': '/robotest/validation/contacts',
    }
    assert contact_stream['policy']['delivery_semantics'] == (
        'authoritative_delivered_active_pair_snapshot'
    )
    assert contact_stream['policy']['string_budget_accounting'] == (
        'payload_strings_plus_normalized_pair_key_once_per_stored_pair'
    )
    assert contact_stream['qos']['private_raw_ros']['depth'] == 64
    assert contact_stream['qos']['public_ros']['depth'] == 10
    source_inventory = contact_stream['gate']['source_inventory']
    expected_source_paths = [
        'src/robotest_sim/CMakeLists.txt',
        'src/robotest_sim/include/robotest_sim/contact_stream_gate.hpp',
        'src/robotest_sim/src/contact_stream_gate.cpp',
        'src/robotest_sim/src/contact_stream_gate_node.cpp',
    ]
    assert [source['path'] for source in source_inventory['sources']] == expected_source_paths
    for source in source_inventory['sources']:
        assert (
            source['sha256']
            == hashlib.sha256((REPOSITORY_ROOT / source['path']).read_bytes()).hexdigest()
        )
    assert len(entries) == len(CONTACT_SENSORS)
    assert {entry['contact_sensor'] for entry in entries} == set(CONTACT_SENSORS)
    assert {entry['collision'] for entry in entries} == {
        collision_name for _, collision_name in CONTACT_SENSORS.values()
    }
    assert len({entry['name'] for entry in entries}) == len(entries)
    assert {entry['source'] for entry in entries} == {coverage['contact_topic']}
    names = {entry['name'] for entry in entries}
    assert set(coverage['covered_collisions']) == names
    assert set(coverage['rendered_robot_collisions']) == names

    excluded_roles = {
        entry['robot_collision'].split('::')[1] for entry in coverage['support_pairs']
    }
    assert excluded_roles == {
        'left_wheel_link',
        'right_wheel_link',
        'front_caster_link',
        'rear_caster_link',
    }
    assert {entry['environment_collision'] for entry in coverage['support_pairs']} == {
        'ground_plane::ground_link::ground_collision'
    }


def test_drive_joint_state_and_ground_truth_contract(
    robot_xml: ET.Element,
) -> None:
    plugins = _plugins(robot_xml)

    drive = plugins['gz::sim::systems::DiffDrive']
    assert drive.findtext('left_joint') == 'left_wheel_joint'
    assert drive.findtext('right_joint') == 'right_wheel_joint'
    assert drive.findtext('topic') == '/robotest/cmd_vel'
    assert drive.findtext('odom_topic') == '/robotest/raw/odom'
    assert drive.findtext('frame_id') == 'odom'
    assert drive.findtext('child_frame_id') == 'base_footprint'
    assert drive.findtext('tf_topic') == ('/robotest/internal/diff_drive_tf_unbridged')

    joint_state = plugins['gz::sim::systems::JointStatePublisher']
    assert joint_state.findtext('topic') == '/robotest/joint_states'
    assert [element.text for element in joint_state.findall('joint_name')] == [
        'left_wheel_joint',
        'right_wheel_joint',
    ]

    truth = plugins['gz::sim::systems::OdometryPublisher']
    assert truth.findtext('dimensions') == '2'
    assert truth.findtext('odom_frame') == 'world'
    assert truth.findtext('robot_base_frame') == 'base_footprint'
    assert truth.findtext('odom_topic') == ('/robotest/validation/ground_truth')
    assert truth.findtext('tf_topic') == ('/robotest/internal/ground_truth_tf_unbridged')


def test_namespace_and_feature_switches() -> None:
    alternate = ET.fromstring(_render(namespace='/robot_2'))
    topics = {
        element.text
        for element in alternate.iter()
        if element.tag in {'topic', 'odom_topic', 'tf_topic'}
    }
    assert {
        '/robot_2/cmd_vel',
        '/robot_2/raw/scan',
        '/robot_2/raw/odom',
        '/robot_2/raw/imu',
        '/robot_2/validation/contacts',
        '/robot_2/validation/ground_truth',
    }.issubset(topics)
    assert all(topic.startswith('/robot_2/') for topic in topics)

    without_truth = ET.fromstring(_render(enable_ground_truth=False))
    assert 'gz::sim::systems::OdometryPublisher' not in _plugins(without_truth)

    urdf_only = ET.fromstring(_render(use_gazebo=False))
    assert urdf_only.findall('./gazebo') == []


def test_sdformat_conversion_preserves_complete_contact_coverage() -> None:
    gz = shutil.which('gz')
    assert gz is not None

    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as output:
        output.write(_render())
        output.flush()
        conversion = subprocess.run(
            [gz, 'sdf', '-p', output.name],
            check=True,
            text=True,
            capture_output=True,
        )

    sdf = ET.fromstring(conversion.stdout)
    model = sdf.find('model')
    assert model is not None
    rendered_collisions = {
        collision.attrib['name']
        for link in model.findall('link')
        for collision in link.findall('collision')
    }
    covered_collisions = set()
    for sensor_name, (link_name, expected_collision) in CONTACT_SENSORS.items():
        link = model.find(f"./link[@name='{link_name}']")
        assert link is not None
        link_collisions = {collision.attrib['name'] for collision in link.findall('collision')}
        contact = link.find(f"./sensor[@name='{sensor_name}']")
        assert contact is not None
        target = contact.findtext('contact/collision')
        assert target == expected_collision
        assert target in link_collisions
        assert contact.findtext('contact/topic') == '/robotest/validation/contacts'
        covered_collisions.add(target)

    assert covered_collisions == rendered_collisions

    assert model.find("./link[@name='lidar_link']/sensor[@name='lidar']") is not None
    assert model.find("./link[@name='imu_link']/sensor[@name='imu']") is not None
