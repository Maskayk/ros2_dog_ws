dog_ws/
├── src/
│   ├── dog_description/               # пакет с URDF/XACRO, meshes, launch для RViz/Gazebo
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── urdf/
│   │   │   └── dog.urdf.xacro
│   │   ├── meshes/                     # optional: stl/dae
│   │   └── launch/
│   │       ├── display.launch.py       # RViz (robot_state_publisher + rviz)
│   │       └── gazebo.launch.py        # Gazebo + spawn + robot_state_publisher (xacro->urdf)
│   │
│   ├── dog_controllers/                # пакет с контроллером ros2_control (C++ plugin)
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   │   ├── dog_controllers_plugins.xml
│   │   ├── config/
│   │   │   └── dog_leg_controllers.yaml   # параметры контроллера, update_rate и т.п.
│   │   ├── include/
│   │   │   └── dog_controllers/
│   │   │       ├── dog_leg_controller.hpp   # объявление controller class
│   │   │       ├── leg_kinematics.hpp       # интерфейс 3DOF кинематики
│   │   │       └── foot_trajectory.hpp      # интерфейс генератора траектории
│   │   └── src/
│   │       ├── dog_leg_controller.cpp       # реализация controller_interface (update(), on_init...)
│   │       ├── leg_kinematics.cpp           # реализация IK/Jacobian (math-only, no ROS)
│   │       └── (опционально) foot_trajectory.cpp
│   │
│   ├── my_dog_node/                    # optional: вспомогательные nodes (high-level planner, teleop)
│   │   ├── package.xml
│   │   ├── CMakeLists.txt or setup.py   # зависимости (rclcpp/rclpy)
│   │   └── src/ or scripts/             # узлы управления (Python/C++)
│   │
│   └── other_packages/                 # вспомогательные пакеты (hw_interface, drivers)
│
├── build/
├── install/
├── log/
├── .gitignore
└── README.md
