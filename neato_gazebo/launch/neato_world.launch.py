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
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from scripts.gazebo_ros_paths import GazeboRosPaths
import xacro


def render_xacro(xacro_path: str) -> str:
    urdf_content = xacro.process_file(xacro_path)
    urdf_file = tempfile.NamedTemporaryFile(delete=False)
    rendered_urdf = urdf_content.toprettyxml(indent='  ')
    urdf_file.write(rendered_urdf.encode('utf-8'))
    return urdf_file.name


def python_launchfile(pkg, filename):
    file_path = os.path.join(
        get_package_share_directory(pkg), 'launch', filename)
    return PythonLaunchDescriptionSource(file_path)


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('neato_description'),
        'urdf', 'neato_standalone.urdf.xacro')
    urdf_file = render_xacro(xacro_path)
    world = os.path.join(
        get_package_share_directory('neato_gazebo'),
        'worlds', 'neato_test.world')

    model_path, plugin_path, resource_path = GazeboRosPaths.get_paths()
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GAZEBO_MODEL_PATH']

    rviz_config_path = os.path.join(
        get_package_share_directory('neato_description'),
        'rviz',
        'view.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('viz', default_value='true'),
        IncludeLaunchDescription(
            python_launchfile('gazebo_ros', 'gazebo.launch.py'),
            launch_arguments={
                'world': world,
                'verbose': 'true',
            }.items(),
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
            python_launchfile('neato_description', 'description.launch.py'),
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
            condition=IfCondition(LaunchConfiguration('viz')),
        ),
    ])
