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

"""Launch the always-present RoboTest fault proxy."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the standalone pass-through proxy launch description."""
    default_params = os.path.join(
        get_package_share_directory('robotest_faults'),
        'config',
        'fault_proxy.yaml',
    )

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='robotest'),
            DeclareLaunchArgument('use_sim_time', default_value='true'),
            DeclareLaunchArgument('params_file', default_value=default_params),
            Node(
                package='robotest_faults',
                executable='fault_proxy_node',
                name='fault_proxy',
                namespace=namespace,
                output='screen',
                parameters=[
                    params_file,
                    {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)},
                ],
            ),
        ]
    )
