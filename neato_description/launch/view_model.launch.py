import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('neato_description'), 'urdf', 'neato_standalone.urdf.xacro')
    urdf_content = xacro.process_file(xacro_path)
    urdf_file = tempfile.NamedTemporaryFile(delete=False)
    rendered_urdf = urdf_content.toprettyxml(indent='  ')
    urdf_file.write(rendered_urdf.encode('utf-8'))

    rviz_config_path = os.path.join(
        get_package_share_directory('neato_description'),
        'rviz',
        'view.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{'publish_frequency': 5.0}],
            arguments=[urdf_file.name],
        ),
        Node(
            package='joint_state_publisher',
            node_executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            arguments=[urdf_file.name],
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path, '--'],
        )
    ])
