# ROS 2 Quadruped Robot — Dog Simulation

A modular ROS 2 / Gazebo Classic simulation of a 12-DOF quadruped robot with trot gait controller, analytical inverse kinematics, and IMU-based body stabilization.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)
![C++17](https://img.shields.io/badge/C%2B%2B-17-green)

**Language / Язык:** [English](#english) | [Русский](#russian)

---

<a id="english"></a>

## English

### Features

- Pure C++ kinematics & gait library (`dog_brain_lib`) with zero ROS dependencies — portable to real hardware
- 12 joints: 4 legs x 3 DoF (hip\_roll, thigh\_pitch, shin\_pitch)
- Analytical IK/FK for each leg
- Trot gait with configurable period, duty factor, and step parameters
- Smooth startup: smoothstep ramp from spawn pose to standing (no torque spikes)
- IMU-based roll/pitch stabilization (optional)
- Runtime parameter tuning via `ros2 param set`
- SolidWorks STL meshes for visual model
- Ground truth odometry via `libgazebo_ros_p3d` (`/ground_truth/state`)

### Prerequisites

| Dependency | Version |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| ros2\_control | Humble |
| gazebo\_ros2\_control | Humble |
| colcon | latest |

```bash
sudo apt install \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-broadcaster
```

### Getting Started

```bash
# Clone
mkdir -p ~/dog_ws/src
cd ~/dog_ws/src
git clone https://github.com/Maskayk/ros2_dog_ws.git .
cd ~/dog_ws

# Build (--symlink-install required for config hot-reload)
colcon build --symlink-install

# Source
source ~/dog_ws/install/setup.bash
# (add to ~/.bashrc for auto-source)
```

### Running

```bash
# Full simulation (Gazebo + controllers + trot gait)
ros2 launch dog_bringup gazebo.launch.py

# URDF viewer (RViz only)
ros2 launch dog_description view_dog.launch.py
```

### Launch Sequence

```
robot_state_publisher
        |
      Gazebo (ODE @ 2 kHz, step=0.5 ms)
        |
   spawn_entity  (z=0.42, free-fall ~4 s)
        |
joint_state_broadcaster
        |
joint_group_position_controller  (SetPosition @ 200 Hz)
        |
   [4 s delay — settle on ground]
        |
    trot_node  (gait loop @ 50 Hz)
```

After launch:
1. Robot spawns at `z=0.42`, falls and settles (~4 s)
2. Position controller holds spawn joints (`thigh=0.0, shin=-0.1`) — zero initial error
3. `trot_node` starts, ramps to standing pose via 1.5 s smoothstep
4. Holds standing until `/cmd_vel` is received

### Control

```bash
# Forward 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop (or just stop publishing — 0.5 s timeout auto-zeroes)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

Runtime tuning:

```bash
ros2 param set /trot_node gait.period 0.6
ros2 param set /trot_node trajectory.step_height 0.06
ros2 param set /trot_node stabilization.enabled true
```

### Architecture

**Data flow:**
```
/cmd_vel ──────────────────────────────────────┐
                                                v
/imu/data ──> [body_controller]     [trot_node @ 50 Hz]
                                                |
                           /joint_group_position_controller/commands
                                                |
                                                v
                         [gazebo_ros2_control SetPosition @ 200 Hz]
                                                |
                                                v
                                  [Gazebo ODE @ 2 kHz, step=0.5 ms]
                                                |
                                                v
                                         /joint_states
```

**Package structure:**
```
dog_ws/src/
├── dog_description/          # Robot model (URDF/xacro, STL meshes)
├── dog_bringup/              # Launch, controller config, world
└── dog_brain/                # Gait controller (C++ lib + ROS node)
    ├── include/dog_brain/
    │   ├── types.hpp         # Geometry, LegJoints, FootPosition
    │   ├── leg_kinematics.hpp # solveIK(), solveFK()
    │   ├── gait_generator.hpp # Phase offsets — trot: {0, 0.5, 0.5, 0}
    │   ├── foot_trajectory.hpp # Swing + stance foot paths
    │   └── body_controller.hpp # IMU -> per-leg Z/X corrections
    └── src/trot_node.cpp     # ROS node: params, timer, publish
```

**Module roles:**

| Module | Role |
|---|---|
| `types.hpp` | Shared structs, geometry constants, leg sign arrays |
| `leg_kinematics` | Analytical IK/FK for 3-DoF leg |
| `gait_generator` | Per-leg phase from time + gait config |
| `foot_trajectory` | Swing (smoothstep-X, polynomial-Z), stance (linear push-back) |
| `body_controller` | IMU quaternion -> Euler -> per-leg Z/X corrections |
| `trot_node` | Thin ROS wrapper: subscriptions, parameter loading, 50 Hz timer |

### Robot Geometry

| Parameter | Value |
|---|---|
| Hip link | 0.06 m |
| Thigh link | 0.144 m |
| Shin link | 0.1525 m |
| Trunk (L x W x H) | 0.475 x 0.18 x 0.10 m |
| Trunk mass | 6.0 kg |
| Leg link mass | 0.5 kg (thigh, shin), 0.2 kg (hip) |
| Foot sphere | r = 0.03 m |
| Standing height (z\_nominal) | -0.25 m |
| Total mass | ~10.8 kg |

**Joint order** (12-element arrays):
```
FL_hip, FL_thigh, FL_shin,  FR_hip, FR_thigh, FR_shin,
RL_hip, RL_thigh, RL_shin,  RR_hip, RR_thigh, RR_shin
```

### Gait Parameters

All in `src/dog_brain/config/gait_params.yaml`, tunable at runtime.

| Parameter | Default | Description |
|---|---|---|
| `gait.period` | 0.8 s | Full gait cycle duration |
| `gait.duty_factor` | 0.6 | Stance fraction (60%) |
| `trajectory.z_nominal` | -0.25 m | Standing depth |
| `trajectory.x_standing` | 0.0 m | Foot X offset (keep 0) |
| `trajectory.step_height` | 0.04 m | Swing foot lift |
| `trajectory.step_amp_x` | 0.06 m | Forward step scale |
| `trajectory.yaw_lever` | 0.08 m | Yaw lever arm |
| `startup.ramp_duration` | 1.5 s | Spawn-to-standing ramp |
| `startup.settle_time` | 0.5 s | Hold before gait |
| `cmd_vel_timeout` | 0.5 s | Auto-zero on no cmd\_vel |

### trot\_node Startup

| Phase | Time | Action |
|---|---|---|
| Ramp | 0 - 1.5 s | Smoothstep from spawn (thigh=0, shin=-0.1) to standing |
| Settle | 1.5 - 2.0 s | Hold standing pose |
| Normal | 2.0 s+ | Accept /cmd\_vel; stand or walk |

### Updating STL Meshes

1. Export STL from SolidWorks (units: mm)
2. Place in `src/dog_description/meshes/`
3. Reference with `scale="0.001 0.001 0.001"` (mm to m)
4. Keep `<collision>` as simple primitives (never STL)
5. Mirror: export separate mirrored STL (negative scale breaks normals in Gazebo)

### Known Issues

| Issue | Notes |
|---|---|
| Forward drift ~20 mm/s standing | Gazebo Classic SetPosition artifact on bent legs. Reduced 2x by `max_step_size=0.0005`. Full fix requires effort-based (torque) control. |
| Right-side mesh orientation | FR/RR legs need mirrored STL from SolidWorks |

### Implementation Notes

**IK sign convention:** Thigh axis `xyz="0 1 0"`. Positive angle = foot moves -X (backward). FK: `foot.x = +L*sin(thigh)` but physically backward. For forward motion `total_x` must be negated.

**xacro `|` fix:** `xacro.process_file().toxml()` adds `|` chars that break `gazebo_ros2_control`. Launch strips them: `.replace('|', '')`. URDF comments must be ASCII-only.

**initial\_value matching:** `initial_value` in `<ros2_control>` must match physical spawn position. Current: `thigh=0.0, shin=-0.1` (shin clamped to upper limit). Mismatch = flip.

**Foot contact:** Only foot sphere (r=0.03 at shin tip) has collision. No collision on thigh/shin body.

---

<a id="russian"></a>

## Русский

### Возможности

- Чистая C++ библиотека кинематики и походки (`dog_brain_lib`) без зависимостей от ROS — переносима на реальное железо
- 12 суставов: 4 ноги x 3 DoF (hip\_roll, thigh\_pitch, shin\_pitch)
- Аналитическая IK/FK для каждой ноги
- Рысь (trot) с настраиваемым периодом, duty factor и параметрами шага
- Плавный старт: smoothstep-рамп от позы спавна до стойки (без рывков)
- Стабилизация крена/тангажа по IMU (опционально)
- Настройка параметров на лету через `ros2 param set`
- STL-меши из SolidWorks
- Ground truth одометрия через `libgazebo_ros_p3d` (топик `/ground_truth/state`)

### Требования

| Зависимость | Версия |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo Classic | 11 |
| ros2\_control | Humble |
| gazebo\_ros2\_control | Humble |
| colcon | latest |

```bash
sudo apt install \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-broadcaster
```

### Быстрый старт

```bash
# Клонирование
mkdir -p ~/dog_ws/src
cd ~/dog_ws/src
git clone https://github.com/Maskayk/ros2_dog_ws.git .
cd ~/dog_ws

# Сборка (--symlink-install для горячей подгрузки конфигов)
colcon build --symlink-install

# Инициализация окружения
source ~/dog_ws/install/setup.bash
# (добавить в ~/.bashrc для автозапуска)
```

### Запуск

```bash
# Полная симуляция (Gazebo + контроллеры + походка)
ros2 launch dog_bringup gazebo.launch.py

# Просмотр URDF (только RViz)
ros2 launch dog_description view_dog.launch.py
```

### Последовательность запуска

```
robot_state_publisher
        |
      Gazebo (ODE @ 2 кГц, шаг=0.5 мс)
        |
   spawn_entity  (z=0.42, свободное падение ~4 с)
        |
joint_state_broadcaster
        |
joint_group_position_controller  (SetPosition @ 200 Гц)
        |
   [задержка 4 с — приземление]
        |
    trot_node  (цикл походки @ 50 Гц)
```

После запуска:
1. Робот спавнится на `z=0.42`, падает и приземляется (~4 с)
2. Позиционный контроллер удерживает суставы спавна (`thigh=0.0, shin=-0.1`) — нулевая начальная ошибка
3. Запускается `trot_node`, плавно переводит в стойку за 1.5 с (smoothstep)
4. Стоит на месте до получения `/cmd_vel`

### Управление

```bash
# Вперёд 0.3 м/с
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Поворот влево
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Стоп (или перестать публиковать — через 0.5 с скорость обнулится)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

Настройка параметров на лету:

```bash
ros2 param set /trot_node gait.period 0.6
ros2 param set /trot_node trajectory.step_height 0.06
ros2 param set /trot_node stabilization.enabled true
```

### Архитектура

**Поток данных:**
```
/cmd_vel ──────────────────────────────────────┐
                                                v
/imu/data ──> [body_controller]     [trot_node @ 50 Гц]
                                                |
                           /joint_group_position_controller/commands
                                                |
                                                v
                        [gazebo_ros2_control SetPosition @ 200 Гц]
                                                |
                                                v
                                 [Gazebo ODE @ 2 кГц, шаг=0.5 мс]
                                                |
                                                v
                                         /joint_states
```

**Структура пакетов:**
```
dog_ws/src/
├── dog_description/          # Модель робота (URDF/xacro, STL-меши)
├── dog_bringup/              # Запуск, конфиг контроллеров, мир
└── dog_brain/                # Контроллер походки (C++ библиотека + ROS-нода)
    ├── include/dog_brain/
    │   ├── types.hpp         # Геометрия, LegJoints, FootPosition
    │   ├── leg_kinematics.hpp # solveIK(), solveFK()
    │   ├── gait_generator.hpp # Фазы — рысь: {0, 0.5, 0.5, 0}
    │   ├── foot_trajectory.hpp # Траектории переноса и опоры
    │   └── body_controller.hpp # IMU -> коррекции Z/X на ногу
    └── src/trot_node.cpp     # ROS-нода: параметры, таймер, публикация
```

**Роли модулей:**

| Модуль | Назначение |
|---|---|
| `types.hpp` | Общие структуры, константы геометрии, массивы знаков ног |
| `leg_kinematics` | Аналитическая IK/FK для 3-DoF ноги |
| `gait_generator` | Фаза каждой ноги из времени и параметров походки |
| `foot_trajectory` | Перенос (smoothstep-X, полином-Z), опора (линейный откат) |
| `body_controller` | Кватернион IMU -> Эйлер -> коррекции Z/X на ногу |
| `trot_node` | Тонкая ROS-обёртка: подписки, параметры, таймер 50 Гц |

### Геометрия робота

| Параметр | Значение |
|---|---|
| Звено тазобедренного (hip) | 0.06 м |
| Звено бедра (thigh) | 0.144 м |
| Звено голени (shin) | 0.1525 м |
| Корпус (Д x Ш x В) | 0.475 x 0.18 x 0.10 м |
| Масса корпуса | 6.0 кг |
| Масса звеньев ног | 0.5 кг (бедро, голень), 0.2 кг (hip) |
| Сфера стопы | r = 0.03 м |
| Высота стойки (z\_nominal) | -0.25 м |
| Общая масса | ~10.8 кг |

**Порядок суставов** (массивы из 12 элементов):
```
FL_hip, FL_thigh, FL_shin,  FR_hip, FR_thigh, FR_shin,
RL_hip, RL_thigh, RL_shin,  RR_hip, RR_thigh, RR_shin
```

### Параметры походки

Все в `src/dog_brain/config/gait_params.yaml`, изменяемые на лету.

| Параметр | По умолч. | Описание |
|---|---|---|
| `gait.period` | 0.8 с | Период полного цикла походки |
| `gait.duty_factor` | 0.6 | Доля фазы опоры (60%) |
| `trajectory.z_nominal` | -0.25 м | Глубина стойки |
| `trajectory.x_standing` | 0.0 м | Смещение стопы X (держать 0) |
| `trajectory.step_height` | 0.04 м | Подъём стопы при переносе |
| `trajectory.step_amp_x` | 0.06 м | Масштаб шага вперёд |
| `trajectory.yaw_lever` | 0.08 м | Плечо поворота |
| `startup.ramp_duration` | 1.5 с | Рамп спавн -> стойка |
| `startup.settle_time` | 0.5 с | Удержание перед походкой |
| `cmd_vel_timeout` | 0.5 с | Обнуление при отсутствии cmd\_vel |

### Запуск trot\_node

| Фаза | Время | Действие |
|---|---|---|
| Рамп | 0 - 1.5 с | Smoothstep от спавна (thigh=0, shin=-0.1) до стойки |
| Удержание | 1.5 - 2.0 с | Удержание позы стойки |
| Работа | 2.0 с+ | Приём /cmd\_vel; стоит или идёт |

### Обновление STL-мешей

1. Экспорт STL из SolidWorks (единицы: мм)
2. Поместить в `src/dog_description/meshes/`
3. Указать `scale="0.001 0.001 0.001"` (мм в м)
4. `<collision>` — только простые примитивы (никогда STL)
5. Зеркалирование: экспортировать отдельный зеркальный STL (отрицательный scale ломает нормали)

### Известные проблемы

| Проблема | Описание |
|---|---|
| Дрейф вперёд ~20 мм/с при стойке | Артефакт SetPosition в Gazebo Classic на согнутых ногах. Снижен в 2 раза `max_step_size=0.0005`. Полное решение требует управления по моментам (effort control). |
| Ориентация мешей правых ног | FR/RR ноги нуждаются в зеркальных STL из SolidWorks |

### Технические детали

**Знаки IK:** Ось thigh `xyz="0 1 0"`. Положительный угол = стопа идёт в -X (назад). FK: `foot.x = +L*sin(thigh)`, но физически это назад. Для движения вперёд `total_x` инвертируется.

**Фикс xacro `|`:** `xacro.process_file().toxml()` добавляет `|` в заголовок, что ломает `gazebo_ros2_control`. Launch-файл удаляет: `.replace('|', '')`. Комментарии в URDF — только ASCII.

**Соответствие initial\_value:** `initial_value` в `<ros2_control>` должен совпадать с физической позой спавна. Сейчас: `thigh=0.0, shin=-0.1` (shin зажат верхним лимитом). Несовпадение = переворот.

**Контакт стоп:** Только сфера стопы (r=0.03 на кончике голени) имеет коллизию. На бедре и теле голени коллизий нет — это предотвращает ложные контакты.
