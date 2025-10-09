# 1) Перейти в src и удалить старый пакет
cd ~/dog_ws/src || exit 1
rm -rf dog_description

# 2) Создать шаблон пакета (ament_cmake)
ros2 pkg create --build-type ament_cmake dog_description --dependencies xacro joint_state_publisher_gui gazebo_ros ros2_control

# 3) Создать папки
cd dog_description
mkdir -p urdf launch

# 4) Записать URDF/XACRO (dog.urdf.xacro)
cat > urdf/dog.urdf.xacro <<'EOF'
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="dog">

  <!-- параметры -->
  <xacro:property name="body_length" value="0.5"/>
  <xacro:property name="body_width" value="0.25"/>
  <xacro:property name="body_height" value="0.1"/>
  <xacro:property name="leg_length" value="0.2"/>
  <xacro:property name="leg_radius" value="0.02"/>
  <xacro:property name="leg_mass" value="0.2"/>

  <!-- тело -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${body_length} ${body_width} ${body_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- макрос для лапы -->
  <xacro:macro name="leg" params="name x y">

    <!-- hip -->
    <link name="${name}_hip">
      <visual>
        <geometry>
          <cylinder radius="${leg_radius}" length="0.05"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${leg_radius}" length="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${name}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${name}_hip"/>
      <origin xyz="${x} ${y} 0" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit effort="10.0" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>

    <!-- thigh -->
    <link name="${name}_thigh">
      <visual>
        <geometry>
          <cylinder radius="${leg_radius}" length="${leg_length}"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="0 0 0"/>
        <material name="blue">
          <color rgba="0 0 1 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${leg_radius}" length="${leg_length}"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0.01" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${name}_thigh_joint" type="revolute">
      <parent link="${name}_hip"/>
      <child link="${name}_thigh"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit effort="10.0" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>

    <!-- knee -->
    <link name="${name}_knee">
      <visual>
        <geometry>
          <cylinder radius="${leg_radius}" length="${leg_length}"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="0 0 0"/>
        <material name="red">
          <color rgba="1 0 0 1"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="${leg_radius}" length="${leg_length}"/>
        </geometry>
        <origin xyz="0 0 -${leg_length/2}" rpy="0 0 0"/>
      </collision>
      <inertial>
        <mass value="${leg_mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0.01" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${name}_knee_joint" type="revolute">
      <parent link="${name}_thigh"/>
      <child link="${name}_knee"/>
      <origin xyz="0 0 -${leg_length}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit effort="10.0" velocity="1.0" lower="-1.57" upper="1.57"/>
    </joint>

  </xacro:macro>

  <!-- Четыре лапы -->
  <xacro:leg name="front_left"  x="${ body_length/2}"  y="${ body_width/2}"/>
  <xacro:leg name="front_right" x="${ body_length/2}"  y="-${ body_width/2}"/>
  <xacro:leg name="rear_left"   x="-${ body_length/2}" y="${ body_width/2}"/>
  <xacro:leg name="rear_right"  x="-${ body_length/2}" y="-${ body_width/2}"/>

  <!-- ros2_control -->
  <ros2_control name="DogSystem" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>

    <xacro:macro name="joint_interfaces" params="joint">
      <joint name="${joint}">
        <command_interface name="position"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </xacro:macro>

    <xacro:joint_interfaces joint="front_left_hip_joint"/>
    <xacro:joint_interfaces joint="front_left_thigh_joint"/>
    <xacro:joint_interfaces joint="front_left_knee_joint"/>
    <xacro:joint_interfaces joint="front_right_hip_joint"/>
    <xacro:joint_interfaces joint="front_right_thigh_joint"/>
    <xacro:joint_interfaces joint="front_right_knee_joint"/>
    <xacro:joint_interfaces joint="rear_left_hip_joint"/>
    <xacro:joint_interfaces joint="rear_left_thigh_joint"/>
    <xacro:joint_interfaces joint="rear_left_knee_joint"/>
    <xacro:joint_interfaces joint="rear_right_hip_joint"/>
    <xacro:joint_interfaces joint="rear_right_thigh_joint"/>
    <xacro:joint_interfaces joint="rear_right_knee_joint"/>
  </ros2_control>

  <!-- Gazebo плагин -->
  <gazebo>
    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so"/>
  </gazebo>

</robot>
EOF

# 5) Записать корректный CMakeLists.txt (перезапишем существующий)
cat > CMakeLists.txt <<'EOF'
cmake_minimum_required(VERSION 3.5)
project(dog_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
EOF

# 6) Записать package.xml (перезапишем существующий)
cat > package.xml <<'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>dog_description</name>
  <version>0.0.1</version>
  <description>URDF/Xacro description of quadruped robot dog</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>ros2_control</exec_depend>
</package>
EOF

# 7) Создать надежный launch-файл (использует get_package_share_directory)
cat > launch/view_dog.launch.py <<'EOF'
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('dog_description')
    urdf_path = os.path.join(pkg_share, 'urdf', 'dog.urdf.xacro')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py', 'gui:=true'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'dog', '-file', urdf_path],
            output='screen'
        )
    ])
EOF

# 8) Вернуться в корень workspace, очистить и пересобрать (последовательно)
cd ~/dog_ws || exit 1
rm -rf build install log
colcon build --symlink-install --packages-select dog_description -v
# 9) Источник окружения
source install/setup.bash

# 10) Проверки: список пакетов и содержимое установленного share
echo "=== ros2 pkg list | grep dog_description ==="
ros2 pkg list | grep dog_description || true
echo "=== ls install/share/dog_description (if exists) ==="
ls -la install/share/dog_description || true
echo "=== ament prefix path ==="
echo $AMENT_PREFIX_PATH
