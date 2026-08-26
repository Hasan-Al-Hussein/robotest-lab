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

"""Canonical RoboTest Lab Phase 2 simulation and Nav2 bringup."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

DEFAULT_NAMESPACE = 'robotest'
DEFAULT_SIMULATOR_SEED = '42'
DEFAULT_NAVIGATION_START_DELAY_SEC = '7.0'
DEFAULT_LIFECYCLE_DISCOVERY_GRACE_SEC = '4.0'
DEFAULT_LIFECYCLE_SERVICE_TIMEOUT_SEC = '20.0'
DEFAULT_LIFECYCLE_RESPONSE_TIMEOUT_SEC = '60.0'

LIFECYCLE_NODES = (
    'map_server',
    'amcl',
    'planner_server',
    'controller_server',
    'behavior_server',
    'bt_navigator',
    'waypoint_follower',
    'velocity_smoother',
    'collision_monitor',
)

# The Phase 1 simulation publishes the canonical global /tf and /tf_static
# topics.  Leaving them unremapped keeps every Nav2 listener and AMCL
# broadcaster on that same tree; remapping to relative names would create an
# isolated /robotest/tf tree with no odom or robot-description transforms.
TF_REMAPS = ()
CONTROLLER_REMAPS = (('cmd_vel', 'cmd_vel_nav'),)
PLANNER_REMAPS = (('plan', 'navigation/plan'),)
BEHAVIOR_REMAPS = (('cmd_vel', 'cmd_vel_behavior_unused'),)
SMOOTHER_REMAPS = (
    ('cmd_vel', 'cmd_vel_nav'),
    ('cmd_vel_smoothed', 'cmd_vel_smoothed'),
)


def _nav_node(package, executable, namespace, params, log_level, remappings=TF_REMAPS):
    """Create one non-composed Nav2 process that fails the whole stack closed."""
    return Node(
        package=package,
        executable=executable,
        namespace=namespace,
        name=executable,
        output='screen',
        respawn=False,
        parameters=[params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=list(remappings),
        on_exit=Shutdown(reason=f'critical Nav2 process exited: {executable}'),
    )


def generate_launch_description():
    """Return the fixed Phase 2 simulation, localization, and navigation graph."""
    namespace = LaunchConfiguration('namespace')
    seed = LaunchConfiguration('seed')
    headless = LaunchConfiguration('headless')
    render_sensors = LaunchConfiguration('render_sensors')
    rviz = LaunchConfiguration('rviz')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    nav_delay = LaunchConfiguration('navigation_start_delay_sec')
    lifecycle_discovery_grace = LaunchConfiguration('lifecycle_discovery_grace_sec')
    lifecycle_service_timeout = LaunchConfiguration('lifecycle_service_timeout_sec')
    lifecycle_response_timeout = LaunchConfiguration('lifecycle_response_timeout_sec')
    lifecycle_startup_result_path = LaunchConfiguration('lifecycle_startup_result_path')
    log_level = LaunchConfiguration('log_level')
    use_sim_time = 'true'

    default_map = PathJoinSubstitution(
        [FindPackageShare('robotest_navigation'), 'maps', 'robotest_lab.yaml']
    )
    default_params = PathJoinSubstitution(
        [FindPackageShare('robotest_navigation'), 'config', 'nav2_params.yaml']
    )
    behavior_tree = PathJoinSubstitution(
        [
            FindPackageShare('robotest_navigation'),
            'behavior_trees',
            'navigate_to_pose_actuation_free.xml',
        ]
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('robotest_sim'), 'launch', 'sim.launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'seed': seed,
            'headless': headless,
            'render_sensors': render_sensors,
            'rviz': rviz,
            'spawn_delay_sec': '5.0',
            'spawn_x': '0.0',
            'spawn_y': '-3.5',
            'spawn_z': '0.12',
            'spawn_yaw': '0.0',
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        namespace=namespace,
        name='map_server',
        output='screen',
        respawn=False,
        parameters=[configured_params, {'yaml_filename': map_yaml}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=list(TF_REMAPS),
        on_exit=Shutdown(reason='critical Nav2 process exited: map_server'),
    )
    amcl = _nav_node('nav2_amcl', 'amcl', namespace, configured_params, log_level)
    planner_server = _nav_node(
        'nav2_planner',
        'planner_server',
        namespace,
        configured_params,
        log_level,
        PLANNER_REMAPS,
    )
    controller_server = _nav_node(
        'nav2_controller',
        'controller_server',
        namespace,
        configured_params,
        log_level,
        CONTROLLER_REMAPS,
    )
    behavior_server = _nav_node(
        'nav2_behaviors',
        'behavior_server',
        namespace,
        configured_params,
        log_level,
        BEHAVIOR_REMAPS,
    )
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        namespace=namespace,
        name='bt_navigator',
        output='screen',
        respawn=False,
        parameters=[
            configured_params,
            {'default_nav_to_pose_bt_xml': behavior_tree},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=list(TF_REMAPS),
        on_exit=Shutdown(reason='critical Nav2 process exited: bt_navigator'),
    )
    waypoint_follower = _nav_node(
        'nav2_waypoint_follower',
        'waypoint_follower',
        namespace,
        configured_params,
        log_level,
    )
    velocity_smoother = _nav_node(
        'nav2_velocity_smoother',
        'velocity_smoother',
        namespace,
        configured_params,
        log_level,
        SMOOTHER_REMAPS,
    )
    collision_monitor = _nav_node(
        'nav2_collision_monitor',
        'collision_monitor',
        namespace,
        configured_params,
        log_level,
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=namespace,
        name='lifecycle_manager_navigation',
        output='screen',
        respawn=False,
        parameters=[
            {
                'use_sim_time': True,
                # A separate bounded trigger starts Nav2 only after DDS discovery
                # has had an explicit wall-clock grace period.
                'autostart': False,
                'node_names': list(LIFECYCLE_NODES),
                'bond_timeout': 4.0,
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        on_exit=Shutdown(reason='critical Nav2 process exited: lifecycle_manager_navigation'),
    )
    lifecycle_startup_trigger = Node(
        package='robotest_navigation',
        executable='lifecycle_startup_trigger',
        namespace=namespace,
        name='lifecycle_startup_trigger',
        output='screen',
        respawn=False,
        condition=IfCondition(autostart),
        parameters=[{'use_sim_time': True}],
        arguments=[
            '--namespace',
            namespace,
            '--manager-node',
            'lifecycle_manager_navigation',
            '--discovery-grace-sec',
            lifecycle_discovery_grace,
            '--service-timeout-sec',
            lifecycle_service_timeout,
            '--response-timeout-sec',
            lifecycle_response_timeout,
            '--result-json',
            lifecycle_startup_result_path,
        ],
    )

    # Construct every unconfigured Nav2 process before the robot is spawned.
    # Starting these processes together is the largest one-time CPU burst in
    # the stack; keeping that burst ahead of contact production prevents it
    # from starving the fail-closed contact evidence path.  Only lifecycle
    # STARTUP remains delayed, so no Nav2 component consumes sensor data until
    # the robot and its bridges have had the original settling interval.
    navigation = GroupAction(
        actions=[
            map_server,
            amcl,
            planner_server,
            controller_server,
            behavior_server,
            bt_navigator,
            waypoint_follower,
            velocity_smoother,
            collision_monitor,
            lifecycle_manager,
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            DeclareLaunchArgument('namespace', default_value=DEFAULT_NAMESPACE),
            DeclareLaunchArgument(
                'seed',
                default_value=DEFAULT_SIMULATOR_SEED,
                description='Gazebo RNG seed forwarded to robotest_sim',
            ),
            DeclareLaunchArgument('headless', default_value='true'),
            DeclareLaunchArgument('render_sensors', default_value='true'),
            DeclareLaunchArgument('rviz', default_value='true'),
            DeclareLaunchArgument('autostart', default_value='true'),
            DeclareLaunchArgument('map', default_value=default_map),
            DeclareLaunchArgument('params_file', default_value=default_params),
            DeclareLaunchArgument(
                'navigation_start_delay_sec',
                default_value=DEFAULT_NAVIGATION_START_DELAY_SEC,
                description='Wall-clock delay before triggering Nav2 lifecycle STARTUP',
            ),
            DeclareLaunchArgument(
                'lifecycle_discovery_grace_sec',
                default_value=DEFAULT_LIFECYCLE_DISCOVERY_GRACE_SEC,
                description='Stable wall discovery grace before lifecycle STARTUP',
            ),
            DeclareLaunchArgument(
                'lifecycle_service_timeout_sec',
                default_value=DEFAULT_LIFECYCLE_SERVICE_TIMEOUT_SEC,
                description='Wall timeout for discovering the lifecycle manager service',
            ),
            DeclareLaunchArgument(
                'lifecycle_response_timeout_sec',
                default_value=DEFAULT_LIFECYCLE_RESPONSE_TIMEOUT_SEC,
                description='Wall timeout for the lifecycle STARTUP response',
            ),
            DeclareLaunchArgument(
                'lifecycle_startup_result_path',
                default_value='',
                description='Atomic lifecycle STARTUP result JSON path; empty disables output',
            ),
            DeclareLaunchArgument('log_level', default_value='info'),
            sim,
            navigation,
            TimerAction(period=nav_delay, actions=[lifecycle_startup_trigger]),
        ]
    )
