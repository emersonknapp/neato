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
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('neato_description'), 'urdf', 'neato_standalone.urdf.xacro')
    urdf_content = xacro.process_file(xacro_path)
    urdf_file = tempfile.NamedTemporaryFile(delete=False)
    rendered_urdf = urdf_content.toprettyxml(indent='  ')
    urdf_file.write(rendered_urdf.encode('utf-8'))

    description_launch_path = os.path.join(
        get_package_share_directory('neato_description'),
        'launch',
        'description.launch.py'
    )

    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=[
                'gazebo', '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            node_name='spawn_neato',
            output='screen',
            arguments=['-file', urdf_file.name, '-entity', 'neato'],
            # on_exit=launch.actions.Shutdown(),
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(description_launch_path)),
    ])
