#!/usr/bin/env python3
# Copyright 2019 Emerson Knapp
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
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def get_package_install_directory(package_name):
    return os.path.join(get_package_share_directory(package_name), '..')


def render_xacro(xacro_path: str) -> str:
    urdf_content = xacro.process_file(xacro_path)
    urdf_file = tempfile.NamedTemporaryFile(delete=False)
    rendered_urdf = urdf_content.toprettyxml(indent='  ')
    urdf_file.write(rendered_urdf.encode('utf-8'))
    return urdf_file.name


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('neato_description'),
        'urdf', 'neato_standalone.urdf.xacro')
    urdf_file = render_xacro(xacro_path)
    world = os.path.join(
        get_package_share_directory('neato_gazebo'),
        'worlds', 'neato_test.world')

    # extend the Gazebo model paths to find the models in the neato_description package
    model_path = ':'.join([
        get_package_install_directory('neato_description'),
    ])

    description_launch_path = os.path.join(
        get_package_share_directory('neato_description'),
        'launch',
        'description.launch.py'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('neato_description'),
        'rviz',
        'view.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            additional_env={
                'GAZEBO_MODEL_PATH': [model_path],
            },
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_neato',
            output='screen',
            arguments=['-file', urdf_file, '-entity', 'neato', '-spawn_service_timeout', '120'],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path)
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
