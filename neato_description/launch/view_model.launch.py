import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_decription():
    # <arg name="urdf_file"
    #    default="$(find xacro)/xacro.py
    #        '$(find neato_description)/urdf/neato_standalone.urdf.xacro'" />
    # <param name="robot_description" command="$(arg urdf_file)" />
    xacro_path = os.path.join(
        get_package_share_directory('neato_description'), 'urdf', 'neato_standalone.urdf.xacro')
    urdf_content = xacro.process_file(xacro_path)
    urdf_temp_path = tempfile.mkstemp()
    with open(urdf_temp_path, 'w') as urdf_file:
        urdf_file.write(urdf_content.toprettyxml(indent='  '))

    # <node pkg="`robot_state_publisher" type="robot_state_publisher"
    #    name="robot_state_publisher" output="screen">
    #     <param name="publish_frequency" type="double" value="5.0" />
    # </node>
    robot_state_node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'publish_frequency': 5.0}],
        arguments=[urdf_temp_path],
    )

    # <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>
    joint_state_node = Node(
        package='joint_state_publisher',
        node_executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # <node name="rviz" pkg="rviz" type="rviz"
    # args="-d $(find neato_description)/rviz/model.rviz"/>
    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz',
        arguments=['-d', rviz_config_dir],
        output='screen',
    )

    return LaunchDescription([
        robot_state_node,
        joint_state_node,
        rviz_node,
    ])
