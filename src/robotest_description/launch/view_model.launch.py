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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    namespace = LaunchConfiguration('namespace')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = PathJoinSubstitution(
        [FindPackageShare('robotest_description'), 'urdf', 'robotest.urdf.xacro']
    )
    robot_description = ParameterValue(
        Command(
            [
                'xacro ',
                xacro_file,
                ' namespace:=/',
                namespace,
                ' use_gazebo:=false',
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='robotest',
                description='ROS namespace without a leading slash.',
            ),
            DeclareLaunchArgument(
                'use_joint_state_publisher',
                default_value='true',
                description='Publish zero wheel positions for model inspection.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use the simulation clock.',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=namespace,
                output='screen',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'use_sim_time': use_sim_time,
                    }
                ],
            ),
            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=namespace,
                output='screen',
                condition=IfCondition(use_joint_state_publisher),
                parameters=[{'use_sim_time': use_sim_time}],
            ),
        ]
    )
