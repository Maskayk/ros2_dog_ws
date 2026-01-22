import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue 

def generate_launch_description():
    # Получаем пути к пакетам
    bringup_pkg_share = get_package_share_directory('dog_bringup')
    desc_pkg_share = get_package_share_directory('dog_description')
    ctrl_pkg_share = get_package_share_directory('dog_controllers')
    
    # Пути к файлам
    world_file = os.path.join(bringup_pkg_share, 'worlds', 'dog.world')
    urdf_file = os.path.join(desc_pkg_share, 'urdf', 'dog.urdf.xacro')
    controller_config = os.path.join(ctrl_pkg_share, 'config', 'controllers.yaml')

    # Генерируем описание робота из Xacro
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

    # Узел Robot State Publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Запуск Gazebo с нашим миром
    gazebo = ExecuteProcess(
        cmd=[
            "gazebo",
            "--verbose",
            world_file, # <-- ВОТ ЗДЕСЬ БЫЛА ОШИБКА, ТЕПЕРЬ ИСПРАВЛЕНО
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Спавн робота
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "dog",
            "-z", "0.25",
            # Мы используем initial_value в URDF, поэтому -J тут не нужны
        ],
        output="screen",
    )

    # Спавнер Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Спавнер основного контроллера
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_group_position_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Очередь запуска (Handlers)
    
    # 1. Сначала спавним робота -> потом запускаем broadcaster
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # 2. Когда broadcaster запустился -> запускаем контроллер
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