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

"""Static and launch-description contract tests for robotest_navigation."""

from __future__ import annotations

import ast
import importlib.util
from itertools import pairwise
import math
import os
from pathlib import Path
import stat
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch.actions import DeclareLaunchArgument
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
INITIAL_POSE = (0.0, -3.5)
WAYPOINTS = ((-2.0, -3.5), (0.0, 0.0), (0.0, 3.5))
FOOTPRINT_HALF_LENGTH = 0.24
FOOTPRINT_HALF_WIDTH = 0.21
FOOTPRINT_PADDING = 0.02


def load_module(name: str, path: Path):
    """Load a source module whose filename is not a normal Python module name."""
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def source_world() -> Path:
    """Resolve the project-owned SDF from the source tree."""
    world = PACKAGE.parent / 'robotest_sim' / 'worlds' / 'robotest_lab.sdf'
    assert world.is_file(), f'missing sibling world: {world}'
    return world


def parse_pgm(path: Path) -> tuple[int, int, list[int]]:
    """Read the deterministic ASCII PGM while ignoring comments."""
    tokens: list[str] = []
    for line in path.read_text(encoding='ascii').splitlines():
        if line.startswith('#'):
            continue
        tokens.extend(line.split())
    assert tokens[0] == 'P2'
    width, height, maximum = map(int, tokens[1:4])
    assert maximum == 255
    pixels = [int(token) for token in tokens[4:]]
    assert len(pixels) == width * height
    return width, height, pixels


def pixel_at(
    x: float,
    y: float,
    width: int,
    height: int,
    pixels: list[int],
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> int:
    """Return the PGM value covering one map-frame coordinate."""
    column = math.floor((x - origin_x) / resolution)
    map_row = math.floor((y - origin_y) / resolution)
    assert 0 <= column < width
    assert 0 <= map_row < height
    pgm_row = height - map_row - 1
    return pixels[pgm_row * width + column]


def sample_segment(start: tuple[float, float], end: tuple[float, float]):
    """Yield poses no farther than 0.05 m apart along a straight segment."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    steps = max(1, math.ceil(distance / 0.05))
    yaw = math.atan2(dy, dx)
    for step in range(steps + 1):
        fraction = step / steps
        yield start[0] + fraction * dx, start[1] + fraction * dy, yaw


def test_package_metadata_and_install_contract() -> None:
    """The package is Apache-2.0 ament_cmake and installs every runtime asset."""
    root = ET.parse(PACKAGE / 'package.xml').getroot()
    assert root.findtext('name') == 'robotest_navigation'
    assert root.findtext('license') == 'Apache-2.0'
    assert root.find('./export/build_type').text == 'ament_cmake'
    dependencies = {element.text for element in root.findall('exec_depend')}
    required = {
        'nav2_amcl',
        'nav2_behaviors',
        'nav2_behavior_tree',
        'nav2_bt_navigator',
        'nav2_collision_monitor',
        'nav2_controller',
        'nav2_dwb_controller',
        'nav2_map_server',
        'nav2_msgs',
        'nav2_navfn_planner',
        'nav2_planner',
        'nav2_velocity_smoother',
        'nav2_waypoint_follower',
        'rclpy',
        'robotest_sim',
    }
    assert required <= dependencies
    license_text = (PACKAGE / 'LICENSE').read_text(encoding='utf-8')
    assert 'Apache License' in license_text
    assert 'Version 2.0, January 2004' in license_text
    cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    for directory in ('behavior_trees', 'config', 'launch', 'maps'):
        assert directory in cmake
    assert 'generate_map.py' in cmake
    assert 'lifecycle_startup_trigger.py' in cmake
    assert 'FILES LICENSE README.md' in cmake


def test_installed_license_and_generator_modes() -> None:
    """The standalone package installs its license and an executable map tool."""
    source_generator = PACKAGE / 'tools' / 'generate_map.py'
    installed_generator = (
        Path(get_package_prefix('robotest_navigation'))
        / 'lib'
        / 'robotest_navigation'
        / 'generate_map.py'
    )
    installed_license = Path(get_package_share_directory('robotest_navigation')) / 'LICENSE'

    assert source_generator.is_file()
    assert installed_generator.is_file()
    assert stat.S_IMODE(source_generator.stat().st_mode) & 0o111 == 0o111
    assert stat.S_IMODE(installed_generator.stat().st_mode) & 0o111 == 0o111
    assert os.access(source_generator, os.X_OK)
    assert os.access(installed_generator, os.X_OK)
    assert installed_license.read_bytes() == (PACKAGE / 'LICENSE').read_bytes()


def test_map_is_byte_reproducible_from_world(tmp_path: Path) -> None:
    """The checked-in map is mechanical output of the current SDF collisions."""
    generator = load_module('robotest_map_generator', PACKAGE / 'tools' / 'generate_map.py')
    generator.write_map(source_world(), tmp_path)
    for filename in ('robotest_lab.pgm', 'robotest_lab.yaml'):
        assert (tmp_path / filename).read_bytes() == (PACKAGE / 'maps' / filename).read_bytes()


def test_map_geometry_identity_and_mission_clearance() -> None:
    """World obstacles are occupied and the frozen mission corridor is clear."""
    metadata = yaml.safe_load((PACKAGE / 'maps' / 'robotest_lab.yaml').read_text())
    assert metadata['resolution'] == 0.05
    assert metadata['origin'] == [-6.2, -6.2, 0.0]
    assert metadata['mode'] == 'trinary'
    width, height, pixels = parse_pgm(PACKAGE / 'maps' / 'robotest_lab.pgm')
    assert (width, height) == (248, 248)
    assert set(pixels) == {0, 254}

    def value(x: float, y: float) -> int:
        return pixel_at(
            x,
            y,
            width,
            height,
            pixels,
            metadata['origin'][0],
            metadata['origin'][1],
            metadata['resolution'],
        )

    for point in (
        (0.0, 6.05),
        (0.0, -6.05),
        (6.05, 0.0),
        (-6.05, 0.0),
        (-3.30, 1.50),
        (3.30, 1.50),
        (-3.8, -2.4),
        (3.7, 3.7),
        (2.7, -3.5),
    ):
        assert value(*point) == 0
    for point in (INITIAL_POSE, *WAYPOINTS, (0.0, 1.5)):
        assert value(*point) == 254

    route = (INITIAL_POSE, *WAYPOINTS)
    half_length = FOOTPRINT_HALF_LENGTH + FOOTPRINT_PADDING
    half_width = FOOTPRINT_HALF_WIDTH + FOOTPRINT_PADDING
    for start, end in pairwise(route):
        for center_x, center_y, yaw in sample_segment(start, end):
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            for local_x in (-half_length, 0.0, half_length):
                for local_y in (-half_width, 0.0, half_width):
                    x = center_x + cos_yaw * local_x - sin_yaw * local_y
                    y = center_y + sin_yaw * local_x + cos_yaw * local_y
                    assert value(x, y) == 254, (center_x, center_y, x, y)


def test_world_shape_set_is_complete() -> None:
    """The generator sees every non-ground static collision in robotest_lab."""
    generator = load_module('robotest_map_shapes', PACKAGE / 'tools' / 'generate_map.py')
    shapes = generator.collision_shapes(source_world())
    assert {shape.name.split('/')[0] for shape in shapes} == {
        'wall_north',
        'wall_south',
        'wall_east',
        'wall_west',
        'divider_west',
        'divider_east',
        'obstacle_box_west',
        'obstacle_box_east',
        'obstacle_cylinder',
    }


def test_nav2_plugins_frames_inputs_and_geometry() -> None:
    """Parameters use installed Jazzy IDs and only validated autonomy inputs."""
    params_path = PACKAGE / 'config' / 'nav2_params.yaml'
    text = params_path.read_text(encoding='utf-8')
    assert '/validation/' not in text
    assert '/raw/' not in text
    params = yaml.safe_load(text)

    amcl = params['amcl']['ros__parameters']
    assert amcl['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'
    assert amcl['base_frame_id'] == 'base_footprint'
    assert amcl['scan_topic'] == 'scan'
    assert amcl['initial_pose'] == {'x': 0.0, 'y': -3.5, 'z': 0.0, 'yaw': 0.0}

    planner = params['planner_server']['ros__parameters']['GridBased']
    assert planner['plugin'] == 'nav2_navfn_planner::NavfnPlanner'
    controller = params['controller_server']['ros__parameters']
    assert controller['FollowPath']['plugin'] == 'dwb_core::DWBLocalPlanner'
    assert 'ObstacleFootprint' in controller['FollowPath']['critics']

    expected_footprint = [[0.24, 0.21], [0.24, -0.21], [-0.24, -0.21], [-0.24, 0.21]]
    for costmap_name in ('local_costmap', 'global_costmap'):
        costmap = params[costmap_name][costmap_name]['ros__parameters']
        assert ast.literal_eval(costmap['footprint']) == expected_footprint
        assert costmap['footprint_padding'] == 0.02
        assert costmap['robot_base_frame'] == 'base_footprint'
        obstacle = costmap['obstacle_layer']['scan']
        assert obstacle['topic'] == 'scan'

    expected_plugins = {
        'nav2_costmap_2d::StaticLayer',
        'nav2_costmap_2d::ObstacleLayer',
        'nav2_costmap_2d::InflationLayer',
    }
    assert expected_plugins <= {
        params['global_costmap']['global_costmap']['ros__parameters'][name]['plugin']
        for name in ('static_layer', 'obstacle_layer', 'inflation_layer')
    }
    assert all(
        section['ros__parameters'].get('use_sim_time') is True
        for section in params.values()
        if 'ros__parameters' in section
    )


def test_wait_only_behavior_and_command_topics() -> None:
    """Only collision_monitor owns final cmd_vel and behavior output is isolated."""
    params = yaml.safe_load((PACKAGE / 'config' / 'nav2_params.yaml').read_text())
    behavior = params['behavior_server']['ros__parameters']
    assert behavior['behavior_plugins'] == ['wait']
    assert behavior['wait']['plugin'] == 'nav2_behaviors::Wait'
    collision = params['collision_monitor']['ros__parameters']
    assert collision['cmd_vel_in_topic'] == 'cmd_vel_smoothed'
    assert collision['cmd_vel_out_topic'] == 'cmd_vel'
    smoother = params['velocity_smoother']['ros__parameters']
    assert smoother['odom_topic'] == 'odom'

    launch_module = load_module('robotest_phase2_launch', PACKAGE / 'launch' / 'phase2.launch.py')
    assert launch_module.TF_REMAPS == ()
    assert ('cmd_vel', 'cmd_vel_nav') in launch_module.CONTROLLER_REMAPS
    assert ('cmd_vel', 'cmd_vel_nav') in launch_module.SMOOTHER_REMAPS
    assert ('cmd_vel', 'cmd_vel_behavior_unused') in launch_module.BEHAVIOR_REMAPS
    assert 'collision_monitor' in launch_module.LIFECYCLE_NODES


def test_behavior_tree_is_actuation_free() -> None:
    """The custom recovery tree contains no motion-recovery action."""
    root = ET.parse(PACKAGE / 'behavior_trees' / 'navigate_to_pose_actuation_free.xml').getroot()
    tags = {element.tag for element in root.iter()}
    assert 'FollowPath' in tags
    assert 'Wait' in tags
    assert 'ClearEntireCostmap' in tags
    assert not tags.intersection({'Spin', 'BackUp', 'DriveOnHeading', 'AssistedTeleop'})


def test_launch_description_is_fixed_noncomposed_and_discoverable() -> None:
    """The installed launch declares the public contract without starting a sim."""
    module = load_module('robotest_phase2_description', PACKAGE / 'launch' / 'phase2.launch.py')
    description = module.generate_launch_description()
    arguments = {
        entity.name for entity in description.entities if isinstance(entity, DeclareLaunchArgument)
    }
    assert arguments == {
        'namespace',
        'seed',
        'headless',
        'render_sensors',
        'rviz',
        'autostart',
        'map',
        'params_file',
        'navigation_start_delay_sec',
        'lifecycle_discovery_grace_sec',
        'lifecycle_service_timeout_sec',
        'lifecycle_response_timeout_sec',
        'lifecycle_startup_result_path',
        'log_level',
    }
    launch_text = (PACKAGE / 'launch' / 'phase2.launch.py').read_text()
    assert 'respawn=False' in launch_text
    assert 'ComposableNode' not in launch_text
    result = subprocess.run(
        ['ros2', 'launch', 'robotest_navigation', 'phase2.launch.py', '--show-args'],
        check=False,
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, result.stderr
    for argument in arguments:
        assert argument in result.stdout
