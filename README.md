```
dog_ws/
├── build/
├── install/
├── log/
└── src/
    ├── dog_description/                     # Геометрия, URDF, визуализация
    │   ├── urdf/
    │   │   ├── dog.urdf.xacro
    │   │   └── materials.xacro
    │   ├── meshes/
    │   │   ├── body.stl
    │   │   ├── leg_upper.stl
    │   │   └── leg_lower.stl
    │   ├── launch/
    │   │   └── display.launch.py
    │   └── config/
    │       └── joint_limits.yaml
    │
    ├── dog_bringup/                         # Главный пакет запуска симуляции
    │   ├── launch/
    │   │   ├── gazebo.launch.py             # Запуск Gazebo с моделью
    │   │   ├── rviz.launch.py               # RViz (опционально)
    │   │   └── full_system.launch.py        # Всё сразу (URDF + Gazebo + контроллеры)
    │   ├── config/
    │   │   └── controllers.yaml             # Подключение контроллеров ros2_control
    │   ├── worlds/
    │   │   └── empty.world
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    ├── dog_controllers/                     # Пользовательские контроллеры
    │   ├── include/dog_controllers/
    │   │   ├── dog_leg_controller.hpp
    │   │   ├── leg_kinematics.hpp
    │   │   ├── foot_trajectory.hpp
    │   │   └── gait_generator.hpp           # Ритмы ходьбы (трот, галоп и т.п.)
    │   ├── src/
    │   │   ├── dog_leg_controller.cpp
    │   │   ├── leg_kinematics.cpp
    │   │   ├── foot_trajectory.cpp
    │   │   ├── gait_generator.cpp
    │   │   └── plugin_registration.cpp      # Регистрация контроллера
    │   ├── config/
    │   │   └── dog_leg_controller.yaml
    │   ├── launch/
    │   │   └── dog_controllers.launch.py
    │   ├── dog_controllers_plugins.xml
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    ├── dog_hardware/                        # Эмуляция аппаратной части (gazebo + реальный драйвер)
    │   ├── include/dog_hardware/
    │   │   └── dog_hardware_interface.hpp
    │   ├── src/
    │   │   ├── dog_hardware_interface.cpp
    │   │   └── plugin_registration.cpp
    │   ├── dog_hardware_plugins.xml
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    ├── my_dog_node/                         # Пользовательский узел управления
    │   ├── src/
    │   │   └── my_dog_node.cpp
    │   ├── include/my_dog_node/
    │   │   └── my_dog_node.hpp
    │   ├── package.xml
    │   └── CMakeLists.txt
    │
    └── dog_teleop/                          # Управление с клавиатуры или джойстика
        ├── src/
        │   └── teleop_keyboard.cpp
        ├── launch/
        │   └── teleop.launch.py
        ├── package.xml
        └── CMakeLists.txt

```
