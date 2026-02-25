# ROS 2 Quadruped Robot — Dog Simulation

A modular ROS 2 / Gazebo Classic simulation of a 12-DOF quadruped robot with trot gait controller, analytical inverse kinematics, and IMU-based body stabilization.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)
![C++17](https://img.shields.io/badge/C%2B%2B-17-green)

---

## Features

- Pure C++ kinematics & gait library with zero ROS dependencies — portable to real hardware
- 12 joints: 4 legs x 3 DoF (hip\_roll, thigh\_pitch, shin\_pitch)
- Analytical IK/FK for each leg
- Trot gait with configurable period, duty factor, and step parameters
- Smooth startup ramp from spawn pose to standing (no torque spikes)
- IMU-based roll/pitch stabilization (optional)
- Runtime parameter tuning via `ros2 param set`
- SolidWorks STL meshes for visual model
- Ground truth odometry via `libgazebo_ros_p3d`

---

## Prerequisites

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| ros2\_control | Humble |
| gazebo\_ros2\_control | Humble |
| colcon | latest |

Install ROS 2 Humble and Gazebo Classic following the [official instructions](https://docs.ros.org/en/humble/Installation.html).

Install required ROS packages:

```bash
sudo apt install \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-broadcaster
```

---

## Getting Started

### Clone

```bash
mkdir -p ~/dog_ws/src
cd ~/dog_ws/src
git clone https://github.com/Maskayk/ros2_dog_ws.git .
cd ~/dog_ws
```

### Build

```bash
cd ~/dog_ws
colcon build --symlink-install
```

`--symlink-install` is required — it symlinks config files (YAML, launch scripts, meshes) so changes take effect without rebuilding.

### Source

```bash
source ~/dog_ws/install/setup.bash
```

Add to `~/.bashrc` to source automatically:

```bash
echo "source ~/dog_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running the Simulation

### Full simulation (Gazebo + controllers + trot gait)

```bash
ros2 launch dog_bringup gazebo.launch.py
```

Launch sequence:

```
robot_state_publisher
        |
      Gazebo
        |
   spawn_entity  (z=0.33)
        |
joint_state_broadcaster
        |
joint_group_position_controller  (position control @ 100 Hz)
        |
   [4 s delay — robot settles]
        |
    trot_node  (gait @ 50 Hz)
```

After launch the robot automatically:
1. Spawns near the ground (`z = 0.33`, legs nearly touching)
2. Position controller holds spawn joints (`thigh=0.0, shin=-0.1`)
3. `trot_node` starts after 4 s delay, ramps to standing pose (1.5 s smoothstep)
4. Holds standing pose until a `/cmd_vel` command is received

### URDF viewer (RViz)

```bash
ros2 launch dog_description view_dog.launch.py
```

---

## Control

### Velocity commands

```bash
# Move forward at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop (or just stop publishing — 0.5 s timeout auto-zeroes velocity)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Runtime parameter tuning

Parameters can be changed while the simulation is running:

```bash
ros2 param set /trot_node gait.period 0.6
ros2 param set /trot_node trajectory.step_height 0.06
ros2 param set /trot_node trajectory.step_amp_x 0.08
ros2 param set /trot_node stabilization.enabled true
```

All parameters are defined in [`src/dog_brain/config/gait_params.yaml`](src/dog_brain/config/gait_params.yaml).

---

## Architecture

### Data flow

```
/cmd_vel  ───────────────────────────────────────────┐
                                                      v
/imu/data ──> [body_controller]           [trot_node @ 50 Hz]
                                                      |
                                 /joint_group_position_controller/commands
                                                      |
                                                      v
                                  [gazebo_ros2_control position @ 100 Hz]
                                                      |
                                                      v
                                        [Gazebo ODE @ 1 kHz]
                                                      |
                                                      v
                                               /joint_states
```

### Package structure

```
dog_ws/src/
├── dog_description/              # Robot model
│   ├── urdf/
│   │   └── dog.urdf.xacro       # Robot description (xacro macro)
│   ├── meshes/
│   │   ├── leg11.STL             # Shin link mesh (SolidWorks)
│   │   └── leg22.STL             # Thigh link mesh (SolidWorks)
│   ├── config/
│   │   └── view_dog.rviz        # RViz config
│   └── launch/
│       └── view_dog.launch.py
│
├── dog_bringup/                  # Simulation launcher
│   ├── launch/
│   │   └── gazebo.launch.py     # Main launch file
│   ├── config/
│   │   └── controllers.yaml     # Controller type, joint list
│   └── worlds/
│       └── dog.world            # Gazebo world (ODE physics)
│
└── dog_brain/                    # Gait controller
    ├── include/dog_brain/
    │   ├── types.hpp             # RobotGeometry, LegJoints, FootPosition
    │   ├── leg_kinematics.hpp    # solveIK(), solveFK()
    │   ├── gait_generator.hpp    # Phase offsets — trot: {0, 0.5, 0.5, 0}
    │   ├── foot_trajectory.hpp   # Swing + stance trajectories
    │   └── body_controller.hpp   # IMU -> per-leg corrections
    ├── src/
    │   ├── leg_kinematics.cpp
    │   ├── gait_generator.cpp
    │   ├── foot_trajectory.cpp
    │   ├── body_controller.cpp
    │   └── trot_node.cpp         # ROS 2 node (thin wrapper)
    └── config/
        └── gait_params.yaml      # Runtime-tunable parameters
```

### `dog_brain` module breakdown

| Module | Responsibility |
|---|---|
| `types.hpp` | All shared data structures and geometry constants |
| `leg_kinematics` | Analytical IK and FK for a 3-DoF leg |
| `gait_generator` | Per-leg phase computation from time and gait params |
| `foot_trajectory` | Swing (smoothstep-X + polynomial-Z) and stance (linear push-back) |
| `body_controller` | IMU quaternion -> Euler -> per-leg Z/X corrections |
| `trot_node` | ROS subscriptions/publications, parameter loading, 50 Hz timer |

---

## Robot Geometry

| Parameter | Value |
|---|---|
| Hip link length | 0.06 m |
| Thigh link length | 0.144 m |
| Shin link length | 0.1525 m |
| Trunk length | 0.475 m |
| Trunk width | 0.18 m |
| Trunk height | 0.10 m |
| Trunk mass | 6.0 kg |
| Leg link mass | 0.5 kg (thigh, shin), 0.2 kg (hip) |
| Foot sphere radius | 0.03 m |
| Standing height (`z_nominal`) | -0.25 m |
| Total mass | ~10.8 kg |

**Joint order** (12-element arrays in `controllers.yaml` and `types.hpp`):
```
FL_hip, FL_thigh, FL_shin,
FR_hip, FR_thigh, FR_shin,
RL_hip, RL_thigh, RL_shin,
RR_hip, RR_thigh, RR_shin
```

---

## Gait Parameters

All parameters live in `src/dog_brain/config/gait_params.yaml` and can be tuned at runtime.

| Parameter | Default | Description |
|---|---|---|
| `gait.period` | 0.8 s | Duration of one full gait cycle |
| `gait.duty_factor` | 0.6 | Fraction of cycle in stance (60%) |
| `trajectory.z_nominal` | -0.25 m | Nominal foot height (standing depth) |
| `trajectory.x_standing` | 0.0 m | FK foot X offset (must be 0 to avoid sliding) |
| `trajectory.step_height` | 0.04 m | Max foot lift during swing |
| `trajectory.step_amp_x` | 0.06 m | Forward step amplitude scale |
| `trajectory.yaw_lever` | 0.08 m | Yaw-to-X lever arm |
| `startup.ramp_duration` | 1.5 s | Smoothstep ramp from spawn to standing pose |
| `startup.settle_time` | 0.5 s | Hold standing before accepting cmd\_vel |
| `cmd_vel_timeout` | 0.5 s | Zero velocity if no cmd\_vel received |

---

## trot\_node Startup Sequence

| Phase | Time | Action |
|---|---|---|
| Ramp | 0 - 1.5 s | Smoothstep interpolation from spawn pose (thigh=0, shin=-0.1) to standing |
| Settle | 1.5 - 2.0 s | Hold standing pose |
| Normal | 2.0 s+ | Accept /cmd\_vel; stand if stationary, walk if velocity above threshold |

---

## Updating STL Meshes from SolidWorks

1. Export each part as STL from SolidWorks (File -> Save As -> STL, units: millimeters)
2. Copy to `src/dog_description/meshes/`
3. In `dog.urdf.xacro`, reference with scale `0.001 0.001 0.001` (mm -> m):
   ```xml
   <mesh filename="package://dog_description/meshes/part.stl" scale="0.001 0.001 0.001"/>
   ```
4. Adjust `<origin xyz="..." rpy="..."/>` in the `<visual>` block to align with the joint frame
5. Keep `<collision>` geometry as simplified primitives (spheres/boxes) — never use STL for collision

> **Note on mirroring:** Right-side legs (FR, RR) are mirror images of left-side legs.
> Negative scale (e.g. `scale="-0.001 0.001 0.001"`) inverts normals in Gazebo Classic and
> causes rendering artifacts. Export a separate mirrored STL from SolidWorks for each mirrored part.

---

## Known Issues

| Issue | Status | Notes |
|---|---|---|
| Forward sliding ~40 mm/s when standing | Open | SetPosition mode in Gazebo Classic creates non-physical constraint forces with a forward bias on bent legs. PID torque control eliminates it but requires careful tuning to avoid instability at spawn. |
| Right-side mesh orientation | Open | FR/RR legs show wrong STL orientation — need mirrored STL from SolidWorks |

---

## Implementation Notes

### IK sign convention

Thigh joint axis is `xyz="0 1 0"`. Positive thigh angle moves the foot tip in the **-X direction** (backward in the world frame). The FK formula uses `foot.x = +L*sin(thigh)`, but this value is **physically backward**. For forward motion, `total_x` must be negated in the trajectory generator.

### xacro and `|` character

`xacro.process_file().toxml()` adds an autogenerated header containing `|` characters. This breaks `gazebo_ros2_control`'s in-process `rcl` argument parser. The launch file strips them:

```python
robot_description_str = xacro.process_file(urdf_file).toxml().replace('|', '')
```

All URDF comments must also be ASCII-only (no Cyrillic, no Unicode arrows).

### Spawn and initial\_value matching

`initial_value` in the URDF `<ros2_control>` block must match the physical joint position at spawn.
Gazebo Classic `spawn_entity.py` does not support `-J` flags, so joints land at their rest/limit positions.
Current spawn: `thigh=0.0` (rest), `shin=-0.1` (clamped to upper limit).
Mismatch causes large controller error at activation -> robot flips.

### implicitSpringDamper

All joints have `<implicitSpringDamper>true</implicitSpringDamper>` in their `<gazebo reference>` blocks. This requires `<dynamics damping="X"/>` on the joint to take effect — without it, Gazebo checks `damping==0` and skips. When active, it provides continuous viscous damping at every ODE substep, reducing micro-oscillation between controller updates.

### Foot contact

Only the foot sphere (radius 0.03 m at shin tip) has collision geometry. No collision on thigh or shin body — this prevents spurious contacts from leg links scraping the ground.

---

---

# ROS 2 Quadruped Robot — Dog Simulation (RU)

Модульная симуляция четвероногого робота (12 степеней свободы) в ROS 2 / Gazebo Classic с контроллером походки (рысь), аналитической обратной кинематикой и стабилизацией по IMU.

---

## Возможности

- Чистая C++ библиотека кинематики и походки без зависимостей от ROS — переносима на реальное железо
- 12 суставов: 4 ноги x 3 DoF (hip\_roll, thigh\_pitch, shin\_pitch)
- Аналитическая прямая и обратная кинематика
- Рысь (trot gait) с настраиваемым периодом, duty factor и параметрами шага
- Плавный разгон при старте: smoothstep от позы спавна до стойки (без рывков)
- Стабилизация по крену/тангажу через IMU (опционально)
- Настройка параметров в реальном времени через `ros2 param set`
- STL-меши из SolidWorks для визуализации
- Ground truth одометрия через `libgazebo_ros_p3d`

---

## Требования

| Зависимость | Версия |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| ros2\_control | Humble |
| gazebo\_ros2\_control | Humble |
| colcon | latest |

Установка ROS-пакетов:

```bash
sudo apt install \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-broadcaster
```

---

## Быстрый старт

### Клонирование

```bash
mkdir -p ~/dog_ws/src
cd ~/dog_ws/src
git clone https://github.com/Maskayk/ros2_dog_ws.git .
cd ~/dog_ws
```

### Сборка

```bash
cd ~/dog_ws
colcon build --symlink-install
```

`--symlink-install` — обязательный флаг: создаёт симлинки на конфиги (YAML, launch, meshes), чтобы изменения применялись без пересборки.

### Инициализация окружения

```bash
source ~/dog_ws/install/setup.bash
```

Для автоматической инициализации при каждом запуске терминала:

```bash
echo "source ~/dog_ws/install/setup.bash" >> ~/.bashrc
```

---

## Запуск симуляции

### Полная симуляция (Gazebo + контроллеры + походка)

```bash
ros2 launch dog_bringup gazebo.launch.py
```

Последовательность запуска:

```
robot_state_publisher
        |
      Gazebo
        |
   spawn_entity  (z=0.33)
        |
joint_state_broadcaster
        |
joint_group_position_controller  (позиционное управление @ 100 Гц)
        |
   [задержка 4 с — робот приземляется]
        |
    trot_node  (походка @ 50 Гц)
```

После запуска робот автоматически:
1. Спавнится у земли (`z = 0.33`, ноги почти касаются)
2. Позиционный контроллер удерживает суставы спавна (`thigh=0.0, shin=-0.1`)
3. Через 4 с запускается `trot_node`, плавно переводит в стойку (1.5 с smoothstep)
4. Стоит на месте до получения команды `/cmd_vel`

### Просмотр URDF (RViz)

```bash
ros2 launch dog_description view_dog.launch.py
```

---

## Управление

### Команды скорости

```bash
# Вперёд 0.3 м/с
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Поворот влево
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Стоп (или просто перестать публиковать — через 0.5 с скорость обнулится)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

### Настройка параметров в реальном времени

```bash
ros2 param set /trot_node gait.period 0.6
ros2 param set /trot_node trajectory.step_height 0.06
ros2 param set /trot_node trajectory.step_amp_x 0.08
ros2 param set /trot_node stabilization.enabled true
```

Все параметры описаны в [`src/dog_brain/config/gait_params.yaml`](src/dog_brain/config/gait_params.yaml).

---

## Архитектура

### Поток данных

```
/cmd_vel  ───────────────────────────────────────────┐
                                                      v
/imu/data ──> [body_controller]           [trot_node @ 50 Гц]
                                                      |
                                 /joint_group_position_controller/commands
                                                      |
                                                      v
                           [gazebo_ros2_control позиционное упр. @ 100 Гц]
                                                      |
                                                      v
                                        [Gazebo ODE @ 1 кГц]
                                                      |
                                                      v
                                               /joint_states
```

### Структура пакетов

```
dog_ws/src/
├── dog_description/              # Модель робота
│   ├── urdf/
│   │   └── dog.urdf.xacro       # Описание робота (xacro-макрос)
│   ├── meshes/
│   │   ├── leg11.STL             # Меш голени (SolidWorks)
│   │   └── leg22.STL             # Меш бедра (SolidWorks)
│   ├── config/
│   │   └── view_dog.rviz        # Конфиг RViz
│   └── launch/
│       └── view_dog.launch.py
│
├── dog_bringup/                  # Запуск симуляции
│   ├── launch/
│   │   └── gazebo.launch.py     # Главный launch-файл
│   ├── config/
│   │   └── controllers.yaml     # Тип контроллера, список суставов
│   └── worlds/
│       └── dog.world            # Мир Gazebo (физика ODE)
│
└── dog_brain/                    # Контроллер походки
    ├── include/dog_brain/
    │   ├── types.hpp             # RobotGeometry, LegJoints, FootPosition
    │   ├── leg_kinematics.hpp    # solveIK(), solveFK()
    │   ├── gait_generator.hpp    # Фазы ног — рысь: {0, 0.5, 0.5, 0}
    │   ├── foot_trajectory.hpp   # Траектории переноса и опоры
    │   └── body_controller.hpp   # IMU -> коррекции для каждой ноги
    ├── src/
    │   ├── leg_kinematics.cpp
    │   ├── gait_generator.cpp
    │   ├── foot_trajectory.cpp
    │   ├── body_controller.cpp
    │   └── trot_node.cpp         # ROS 2 нода (тонкая обёртка)
    └── config/
        └── gait_params.yaml      # Настраиваемые параметры
```

### Модули `dog_brain`

| Модуль | Назначение |
|---|---|
| `types.hpp` | Все общие структуры данных и константы геометрии |
| `leg_kinematics` | Аналитическая IK и FK для 3-DoF ноги |
| `gait_generator` | Вычисление фазы каждой ноги из времени и параметров походки |
| `foot_trajectory` | Перенос (smoothstep-X + полином-Z) и опора (линейный откат) |
| `body_controller` | Кватернион IMU -> углы Эйлера -> Z/X коррекции на ногу |
| `trot_node` | ROS-подписки/публикации, загрузка параметров, таймер 50 Гц |

---

## Геометрия робота

| Параметр | Значение |
|---|---|
| Длина звена бедра (hip) | 0.06 м |
| Длина звена бедренной кости (thigh) | 0.144 м |
| Длина звена голени (shin) | 0.1525 м |
| Длина корпуса (trunk) | 0.475 м |
| Ширина корпуса | 0.18 м |
| Высота корпуса | 0.10 м |
| Масса корпуса | 6.0 кг |
| Масса звеньев ног | 0.5 кг (бедро, голень), 0.2 кг (тазобедренное) |
| Радиус сферы стопы | 0.03 м |
| Высота стойки (`z_nominal`) | -0.25 м |
| Общая масса | ~10.8 кг |

**Порядок суставов** (массивы из 12 элементов в `controllers.yaml` и `types.hpp`):
```
FL_hip, FL_thigh, FL_shin,
FR_hip, FR_thigh, FR_shin,
RL_hip, RL_thigh, RL_shin,
RR_hip, RR_thigh, RR_shin
```

---

## Параметры походки

Все параметры находятся в `src/dog_brain/config/gait_params.yaml` и могут быть изменены во время работы.

| Параметр | По умолчанию | Описание |
|---|---|---|
| `gait.period` | 0.8 с | Длительность полного цикла походки |
| `gait.duty_factor` | 0.6 | Доля цикла в фазе опоры (60%) |
| `trajectory.z_nominal` | -0.25 м | Номинальная высота стопы (глубина стойки) |
| `trajectory.x_standing` | 0.0 м | Смещение стопы по X (должно быть 0 во избежание скольжения) |
| `trajectory.step_height` | 0.04 м | Максимальный подъём стопы при переносе |
| `trajectory.step_amp_x` | 0.06 м | Масштаб амплитуды шага вперёд |
| `trajectory.yaw_lever` | 0.08 м | Плечо для вычисления поворота |
| `startup.ramp_duration` | 1.5 с | Smoothstep-рамп от позы спавна до стойки |
| `startup.settle_time` | 0.5 с | Удержание стойки перед приёмом cmd\_vel |
| `cmd_vel_timeout` | 0.5 с | Обнуление скорости при отсутствии cmd\_vel |

---

## Последовательность запуска trot\_node

| Фаза | Время | Действие |
|---|---|---|
| Рамп | 0 - 1.5 с | Smoothstep-интерполяция от позы спавна (thigh=0, shin=-0.1) до стойки |
| Удержание | 1.5 - 2.0 с | Удержание позы стойки |
| Нормальная работа | 2.0 с+ | Приём /cmd\_vel; стоит при нулевой скорости, идёт при скорости выше порога |

---

## Обновление STL-мешей из SolidWorks

1. Экспортировать каждую деталь как STL из SolidWorks (File -> Save As -> STL, единицы: миллиметры)
2. Скопировать в `src/dog_description/meshes/`
3. В `dog.urdf.xacro` указать масштаб `0.001 0.001 0.001` (мм -> м):
   ```xml
   <mesh filename="package://dog_description/meshes/part.stl" scale="0.001 0.001 0.001"/>
   ```
4. Подогнать `<origin xyz="..." rpy="..."/>` в блоке `<visual>` для выравнивания с системой координат сустава
5. Для `<collision>` использовать упрощённые примитивы (сферы/боксы) — никогда не использовать STL

> **Зеркалирование:** Правые ноги (FR, RR) — зеркальные отражения левых.
> Отрицательный масштаб (напр. `scale="-0.001 0.001 0.001"`) инвертирует нормали в Gazebo Classic.
> Правильный подход — экспортировать отдельный зеркальный STL из SolidWorks.

---

## Известные проблемы

| Проблема | Статус | Описание |
|---|---|---|
| Скольжение вперёд ~40 мм/с при стойке | Открыта | Режим SetPosition в Gazebo Classic создаёт нефизичные силы на согнутых ногах. PID-управление по моментам устраняет эффект, но требует тщательной настройки. |
| Ориентация мешей правых ног | Открыта | FR/RR ноги показывают неправильную ориентацию STL — нужны зеркальные STL из SolidWorks |

---

## Технические детали

### Соглашение о знаках IK

Ось тазобедренного сустава (thigh) — `xyz="0 1 0"`. Положительный угол перемещает кончик стопы в направлении **-X** (назад в мировой СК). FK использует `foot.x = +L*sin(thigh)`, но это значение **физически направлено назад**. Для движения вперёд `total_x` должен быть отрицательным в генераторе траекторий.

### xacro и символ `|`

`xacro.process_file().toxml()` добавляет автогенерированный заголовок с символами `|`. Это ломает парсер аргументов `rcl` внутри `gazebo_ros2_control`. Launch-файл удаляет их:

```python
robot_description_str = xacro.process_file(urdf_file).toxml().replace('|', '')
```

Все комментарии в URDF должны быть только ASCII (без кириллицы и Unicode).

### Соответствие initial\_value и позы спавна

`initial_value` в блоке `<ros2_control>` URDF должен совпадать с физической позицией сустава при спавне.
`spawn_entity.py` в Gazebo Classic не поддерживает флаги `-J`, поэтому суставы принимают позиции покоя/лимитов.
Текущий спавн: `thigh=0.0` (покой), `shin=-0.1` (зажат верхним лимитом).
Рассогласование вызывает большую ошибку контроллера при активации -> робот переворачивается.

### implicitSpringDamper

На всех суставах включён `<implicitSpringDamper>true</implicitSpringDamper>` через `<gazebo reference>`. Для его работы необходим `<dynamics damping="X"/>` на суставе в URDF — без него Gazebo проверяет `damping==0` и пропускает. При активации обеспечивает вязкое демпфирование на каждом подшаге ODE, уменьшая микроосцилляции между обновлениями контроллера.

### Контакт стоп

Только сфера стопы (радиус 0.03 м на кончике голени) имеет геометрию столкновений. На бедре и теле голени нет collision — это предотвращает ложные контакты звеньев ног с землёй.
