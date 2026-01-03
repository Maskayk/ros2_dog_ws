import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
# ДОБАВЛЕН ИМПОРТ
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    desc_pkg_share = get_package_share_directory('dog_description')
    ctrl_pkg_share = get_package_share_directory('dog_controllers')
    
    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'dog.urdf.xacro')
    controller_config = os.path.join(ctrl_pkg_share, 'config', 'controllers.yaml')

    # ИСПРАВЛЕНИЕ: Оборачиваем Command в ParameterValue
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                urdf_file,
            ]
        ),
        value_type=str
    )
    
    # Передаем уже обернутое значение
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "dog",
            "-z", "0.5",
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_position_controller", 
            "--controller-manager", "/controller_manager",
            "--param-file", controller_config
        ],
        output="screen",
    )

    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_robot_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            gazebo,
            spawn_entity,
            delay_jsb_after_spawn,
            delay_robot_controller_after_jsb,
        ]
    )