dog_ws/
├── build/
├── install/
├── log/
├── src/
│   └── dog_controllers/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── dog_controllers_plugins.xml
│       │
│       ├── config/
│       │   └── dog_leg_controller.yaml
│       │
│       ├── include/
│       │   └── dog_controllers/
│       │       ├── dog_leg_controller.hpp      # Основной контроллер ног
│       │       ├── leg_kinematics.hpp          # Прямая и обратная кинематика
│       │       └── foot_trajectory.hpp         # Генерация траектории лап
│       │
│       ├── launch/
│       │   └── dog_controllers.launch.py       # Запуск контроллеров
│       │
│       └── src/
│           ├── dog_leg_controller.cpp          # Реализация контроллера
│           ├── leg_kinematics.cpp              # Реализация кинематики
│           └── main.cpp                        # Основная точка входа (опционально)
│
├── dog_description/                            # Описание модели робота
│   ├── urdf/
│   │   └── dog.urdf.xacro
│   ├── meshes/
│   │   ├── body.stl
│   │   ├── leg_upper.stl
│   │   └── leg_lower.stl
│   └── launch/
│       └── display.launch.py                   # Визуализация в RViz
│
└── my_dog_node/                                # Отдельный узел управления роботом
    ├── src/
    │   └── my_dog_node.cpp
    ├── include/
    │   └── my_dog_node/
    │       └── my_dog_node.hpp
    ├── package.xml
    └── CMakeLists.txt
