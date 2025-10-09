from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = FindPackageShare('dog_description')
    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'dog.urdf.xacro'])
    controllers_yaml = PathJoinSubstitution([pkg, 'config', 'dog_controllers.yaml'])

    # robot_description будет сформирован командой xacro при старте
    robot_description = Command(['xacro ', xacro_path])

    ld = LaunchDescription()

    # 1) (опционально) старт Gazebo с плагином ros factory
    ld.add_action(ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    ))

    # 2) robot_state_publisher (передаём robot_description через Command)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    ))

    # 3) ros2_control_node (передаём robot_description и файл контроллеров)
    ld.add_action(Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description}, controllers_yaml]
    ))

    # 4) Спавним контроллеры с небольшой задержкой (даём время ros2_control_node подняться)
    ld.add_action(TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        )]
    ))

    ld.add_action(TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['leg_position_controller'],
            output='screen'
        )]
    ))

    return ld
