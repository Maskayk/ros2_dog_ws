from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare("dog_description")
    urdf_file = PathJoinSubstitution([pkg_share, "urdf", "dog.urdf"])

    # robot_state_publisher
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": Command(["xacro ", urdf_file])}]
    )

    # ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_file]),
            "update_rate": 100
        }],
    )

    # спавн сущности (отложенный на 5 секунд после запуска Gazebo)
    spawn_dog = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=[
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-topic", "robot_description",
                "-entity", "dog"
            ],
            output="screen"
        )]
    )

    # Gazebo сервер
    gzserver = ExecuteProcess(
        cmd=["gazebo", "--verbose", "-s", "libgazebo_ros_factory.so"],
        output="screen"
    )

    # Gazebo клиент (GUI)
    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        env={"DISPLAY": os.environ.get("DISPLAY", ":0")}
    )

    return LaunchDescription([
        gzserver,
        gzclient,
        rsp_node,
        ros2_control_node,
        spawn_dog
    ])
