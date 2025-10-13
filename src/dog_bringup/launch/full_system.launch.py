# full_system.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    bringup_pkg = FindPackageShare('dog_bringup')
    gazebo_pkg = FindPackageShare('gazebo_ros')

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_pkg, '/launch/gazebo.launch.py'])
    )

    spawn_dog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            bringup_pkg, '/launch/spawn_dog.launch.py'
        ])
    )

    # ждём, пока gazebo_ros2_control создаст controller_manager
    joint_state_spawner = TimerAction(
        period=10.0,  # увеличить ожидание — Gazebo плагин поднимается медленно
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    leg_controller_spawner = TimerAction(
        period=12.0,  # ещё через пару секунд после предыдущего
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['leg_controller', '--controller-manager', '/controller_manager'],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_dog,
        joint_state_spawner,
        leg_controller_spawner
    ])
