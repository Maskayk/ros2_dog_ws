from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Путь к YAML
    config_file = os.path.join(
        get_package_share_directory('dog_controllers'),
        'config',
        'dog_leg_controllers.yaml'
    )

    # Запускаем controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'use_sim_time': True},
            config_file
        ],
        output='screen'
    )

    # Загружаем твой контроллер
    load_dog_leg = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['dog_leg_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([controller_manager, load_dog_leg])
