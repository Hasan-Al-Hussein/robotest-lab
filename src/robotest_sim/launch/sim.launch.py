# Copyright 2026 Hasan Ahmed
# SPDX-License-Identifier: Apache-2.0

"""Canonical CPU-safe RoboTest Lab simulation bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

DEFAULT_SIMULATOR_SEED = '42'
MIN_SIMULATOR_SEED = 0
MAX_SIMULATOR_SEED = 2**32 - 1


def _as_bool(value: str) -> bool:
    """Interpret a ROS launch boolean after substitutions are resolved."""
    return value.strip().lower() in {'1', 'true', 'yes', 'on'}


def _parse_simulator_seed(value: str) -> int:
    """Return a Gazebo-compatible unsigned 32-bit random seed."""
    try:
        seed = int(value.strip(), 10)
    except ValueError as error:
        raise RuntimeError('seed must be an integer in [0, 4294967295]') from error
    if seed < MIN_SIMULATOR_SEED or seed > MAX_SIMULATOR_SEED:
        raise RuntimeError('seed must be an integer in [0, 4294967295]')
    return seed


def _launch_runtime(context):
    """Build actions whose values require launch-context evaluation."""
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    if not namespace:
        raise RuntimeError('namespace must be a non-empty relative ROS namespace')

    headless = _as_bool(LaunchConfiguration('headless').perform(context))
    render_sensors = _as_bool(LaunchConfiguration('render_sensors').perform(context))
    simulator_seed = _parse_simulator_seed(LaunchConfiguration('seed').perform(context))
    spawn_delay = float(LaunchConfiguration('spawn_delay_sec').perform(context))
    if spawn_delay < 0.0 or spawn_delay > 30.0:
        raise RuntimeError('spawn_delay_sec must be in [0, 30]')

    sim_share = get_package_share_directory('robotest_sim')
    description_share = get_package_share_directory('robotest_description')
    faults_share = get_package_share_directory('robotest_faults')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(sim_share, 'worlds', 'robotest_lab.sdf')
    xacro_file = os.path.join(description_share, 'urdf', 'robotest.urdf.xacro')
    bridge_config = os.path.join(sim_share, 'config', 'bridge.yaml')
    rviz_config = os.path.join(sim_share, 'rviz', 'robotest.rviz')
    fault_launch = os.path.join(faults_share, 'launch', 'fault_proxy.launch.py')

    # Gazebo-side names are fixed by the Phase 0 contract. The ROS namespace
    # remains configurable, while the default resolves every endpoint under
    # /robotest as documented.
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_file,
                ' namespace:=/robotest',
                ' use_gazebo:=true',
                ' enable_ground_truth:=true',
            ]
        ),
        value_type=str,
    )

    # Keep the seed in the unconditional argument prefix so both GUI and every
    # headless rendering path use the same Gazebo random stream.
    gz_args = ['-r', '-v', '2', '--seed', str(simulator_seed)]
    if headless:
        gz_args.append('-s')
        if render_sensors:
            # The 360-ray gpu_lidar still needs server-side rendering.
            gz_args.append('--headless-rendering')
    gz_args.append(world_file)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ' '.join(gz_args),
            'on_exit_shutdown': 'true',
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
            }
        ],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        name='spawn_robotest',
        output='screen',
        arguments=[
            '-world',
            'robotest_lab',
            '-topic',
            f'/{namespace}/robot_description',
            '-name',
            'robotest',
            '-allow_renaming',
            'false',
            '-x',
            LaunchConfiguration('spawn_x'),
            '-y',
            LaunchConfiguration('spawn_y'),
            '-z',
            LaunchConfiguration('spawn_z'),
            '-Y',
            LaunchConfiguration('spawn_yaw'),
        ],
        parameters=[
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='parameter_bridge',
        output='screen',
        parameters=[
            {
                'config_file': bridge_config,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool
                ),
            },
        ],
    )

    fault_proxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(fault_launch),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
        ],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return [
        gazebo,
        state_publisher,
        bridge,
        fault_proxy,
        TimerAction(period=spawn_delay, actions=[spawn]),
        rviz,
    ]


def generate_launch_description():
    """Return the canonical Gazebo, spawn, proxy, bridge, and RViz chain."""
    plugin_dir = '/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins'
    current_plugins = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = plugin_dir
    if current_plugins:
        plugin_path = plugin_dir + os.pathsep + current_plugins

    return LaunchDescription(
        [
            SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                get_package_share_directory('robotest_description'),
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                get_package_share_directory('robotest_sim'),
            ),
            DeclareLaunchArgument('namespace', default_value='robotest'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument(
                'seed',
                default_value=DEFAULT_SIMULATOR_SEED,
                description='Gazebo RNG seed in the unsigned 32-bit range',
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='true',
                description='Run the Gazebo server without the Gazebo GUI',
            ),
            DeclareLaunchArgument(
                'render_sensors',
                default_value='true',
                description='Enable headless rendering required by gpu_lidar',
            ),
            DeclareLaunchArgument(
                'rviz',
                default_value='true',
                description='Launch the generated sensor visualization',
            ),
            DeclareLaunchArgument('spawn_delay_sec', default_value='5.0'),
            DeclareLaunchArgument('spawn_x', default_value='0.0'),
            DeclareLaunchArgument('spawn_y', default_value='-3.5'),
            DeclareLaunchArgument('spawn_z', default_value='0.12'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
            OpaqueFunction(function=_launch_runtime),
        ]
    )
