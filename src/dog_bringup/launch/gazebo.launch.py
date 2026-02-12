import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bringup_pkg_share = get_package_share_directory('dog_bringup')
    desc_pkg_share = get_package_share_directory('dog_description')
    brain_pkg_share = get_package_share_directory('dog_brain')

    world_file = os.path.join(bringup_pkg_share, 'worlds', 'dog.world')
    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'dog.urdf.xacro')
    gait_params = os.path.join(brain_pkg_share, 'config', 'gait_params.yaml')

    # Начальные углы суставов (должны совпадать с URDF init_thigh/init_shin)
    init_thigh = "0.76"
    init_shin = "-1.47"

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
            world_file,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Спавн с -J флагами: задаём ФИЗИЧЕСКУЮ позу суставов при спавне
    # Без этого Gazebo создаёт модель с суставами в 0, робот падает
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "dog",
            "-z", "0.25",
            "-J", "front_left_thigh_joint", init_thigh,
            "-J", "front_left_shin_joint", init_shin,
            "-J", "front_right_thigh_joint", init_thigh,
            "-J", "front_right_shin_joint", init_shin,
            "-J", "rear_left_thigh_joint", init_thigh,
            "-J", "rear_left_shin_joint", init_shin,
            "-J", "rear_right_thigh_joint", init_thigh,
            "-J", "rear_right_shin_joint", init_shin,
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
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    trot_node = Node(
        package="dog_brain",
        executable="trot_node",
        name="trot_node",
        output="screen",
        parameters=[gait_params],
    )

    # Цепочка запуска: spawn -> JSB -> controller -> задержка 2с -> trot_node
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

    # Задержка перед запуском trot_node — даём роботу стабилизироваться
    delay_trot_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[TimerAction(period=2.0, actions=[trot_node])],
        )
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            gazebo,
            spawn_entity,
            delay_jsb_after_spawn,
            delay_robot_controller_after_jsb,
            delay_trot_after_controller,
        ]
    )
