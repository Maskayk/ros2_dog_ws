# ROS 2 Quadruped Robot — Dog Simulation

A modular ROS 2 / Gazebo Classic simulation of a 12-DOF quadruped robot with trot gait controller, analytical inverse kinematics, and IMU-based body stabilization.

![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)
![C++17](https://img.shields.io/badge/C%2B%2B-17-green)

---

## Features

- Pure C++ kinematics & gait library with zero ROS dependencies — portable to real hardware
- 12 joints: 4 legs × 3 DoF (hip\_roll, thigh\_pitch, shin\_pitch)
- Analytical IK/FK for each leg
- Trot gait with configurable period, duty factor, and step parameters
- Smooth startup ramp: no violent torque spikes at launch
- IMU-based roll/pitch stabilization (optional)
- Runtime parameter tuning via `ros2 param set`
- SolidWorks STL meshes for visual model

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
   spawn_entity  (z=0.35, robot falls and settles ~4 s)
        |
joint_state_broadcaster
        |
joint_group_position_controller  (PID p=80 d=2)
        |
   [4 s delay]
        |
    trot_node
```

After launch the robot automatically:
1. Falls to the ground (`z = 0.35`)
2. PID holds spawn joints (`thigh=0.0, shin=-0.5`) — no torque spike
3. `trot_node` starts → smooth ramp to standing pose (1.5 s)
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

# Stop
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
/cmd_vel  ──────────────────────────────────────────────┐
                                                         ▼
/imu/data ──► [body_controller]              [trot_node @ 50 Hz]
                                                         │
                                    /joint_group_position_controller/commands
                                                         │
                                                         ▼
                                     [gazebo_ros2_control PID @ 100 Hz]
                                                         │
                                                         ▼
                                           [Gazebo ODE @ 1 ms]
                                                         │
                                                         ▼
                                                  /joint_states
```

### Package structure

```
ros2_dog_ws/
├── src/
│   ├── dog_description/          # Robot model
│   │   ├── urdf/
│   │   │   └── dog.urdf.xacro    # Robot description (xacro)
│   │   ├── meshes/
│   │   │   ├── leg11.STL         # Shin link mesh (SolidWorks export)
│   │   │   └── leg22.STL         # Thigh link mesh (SolidWorks export)
│   │   ├── config/
│   │   │   └── view_dog.rviz     # RViz config
│   │   └── launch/
│   │       └── view_dog.launch.py
│   │
│   ├── dog_bringup/              # Simulation launcher
│   │   ├── launch/
│   │   │   └── gazebo.launch.py  # Main launch file
│   │   ├── config/
│   │   │   └── controllers.yaml  # PID gains, joint order
│   │   └── worlds/
│   │       └── dog.world         # Gazebo world
│   │
│   └── dog_brain/                # Gait controller
│       ├── include/dog_brain/
│       │   ├── types.hpp         # RobotGeometry, LegJoints, FootPosition, VelocityCommand
│       │   ├── leg_kinematics.hpp # solveIK(), solveFK()
│       │   ├── gait_generator.hpp # Phase offsets — trot: {0, 0.5, 0.5, 0}
│       │   ├── foot_trajectory.hpp # Swing: smoothstep-X + polynomial-Z
│       │   └── body_controller.hpp # IMU roll/pitch -> per-leg Z correction
│       ├── src/
│       │   ├── leg_kinematics.cpp
│       │   ├── gait_generator.cpp
│       │   ├── foot_trajectory.cpp
│       │   ├── body_controller.cpp
│       │   └── trot_node.cpp     # ROS 2 node (thin wrapper)
│       └── config/
│           └── gait_params.yaml  # Runtime-tunable parameters
│
├── CLAUDE.md                     # AI coding guidelines
└── README.md
```

### `dog_brain` module breakdown

| Module | Responsibility |
|---|---|
| `types.hpp` | All shared data structures and geometry constants |
| `leg_kinematics` | Analytical IK and FK for a 3-DoF leg |
| `gait_generator` | Per-leg phase computation from time and gait params |
| `foot_trajectory` | Swing (smoothstep-X + bell-Z) and stance (linear push-back) |
| `body_controller` | IMU quaternion → Euler → per-leg Z/X corrections |
| `trot_node` | ROS subscriptions/publications, parameter loading, timer |

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
| Foot sphere radius | 0.03 m |
| Standing height (`z_nominal`) | −0.25 m |

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
| `gait.duty_factor` | 0.6 | Fraction of cycle in stance (0.6 = 60% stance) |
| `trajectory.z_nominal` | −0.25 m | Nominal foot height (standing depth) |
| `trajectory.x_standing` | 0.0 m | FK foot X offset for CoM compensation |
| `trajectory.step_height` | 0.04 m | Max foot lift during swing |
| `trajectory.step_amp_x` | 0.06 m | Forward step amplitude scale |
| `trajectory.yaw_lever` | 0.08 m | Yaw-to-X lever arm |
| `startup.ramp_duration` | 1.5 s | Ramp from spawn pose to standing |
| `startup.settle_time` | 0.5 s | Hold standing before gait starts |

---

## Updating STL Meshes from SolidWorks

1. Export each part as STL from SolidWorks (File → Save As → STL, units: millimeters)
2. Copy to `src/dog_description/meshes/`
3. In `dog.urdf.xacro`, reference with scale `0.001 0.001 0.001` (mm → m):
   ```xml
   <mesh filename="package://dog_description/meshes/part.stl" scale="0.001 0.001 0.001"/>
   ```
4. Adjust `<origin xyz="..." rpy="..."/>` in the `<visual>` block to align with the joint frame
5. Keep `<collision>` geometry as simplified primitives (spheres/boxes) — never use STL for collision

> **Note on mirroring:** Right-side legs (FR, RR) are mirror images of left-side legs.
> Negative scale (e.g. `scale="-0.001 0.001 0.001"`) inverts normals in Gazebo Classic and
> causes rendering artifacts. The correct approach is to export a separate mirrored STL
> from SolidWorks for each mirrored part.

---

## Known Issues

| Issue | Status | Notes |
|---|---|---|
| Vibration when standing | Partial fix | z\_nominal=-0.25 reduces it; IMU stabilization can improve further |
| Right-side mesh orientation | Open | FR/RR legs show wrong STL orientation — need mirrored STL from SolidWorks |
| Robot slides when walking | Open | CoM offset causes pitch imbalance; IMU stabilization is the long-term fix |

---

## Implementation Notes

### IK sign convention

Thigh joint axis is `xyz="0 1 0"`. Positive thigh angle moves the foot tip in the **−X direction** (backward in the world frame). The FK formula therefore uses `foot.x = +L·sin(thigh)`, but this value is **physically backward** — so for forward motion, `total_x` must be negated.

### xacro and `|` character

`xacro.process_file().toxml()` adds an autogenerated header containing `|` characters. This breaks `gazebo_ros2_control`'s in-process `rcl` argument parser. The launch file strips them:

```python
robot_description_str = xacro.process_file(urdf_file).toxml().replace('|', '')
```

All URDF comments must also be ASCII-only (no Cyrillic, no Unicode arrows).

### Startup PID stability

`initial_value` in the URDF must match the physical joint position at spawn, otherwise the first PID tick produces a large error torque and the robot flips. Current values: `thigh=0.0, shin=-0.5`.
