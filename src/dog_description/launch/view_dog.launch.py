from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('dog_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dog.urdf.xacro')

    robot_description = {
        'robot_description': Command(['xacro ', xacro_file])
    }

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    jsp = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_dog.rviz')],
        output='screen'
    )

    return LaunchDescription([jsp, rsp, rviz])
