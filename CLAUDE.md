# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS 2 quadruped robot ("robot dog") simulation workspace. 12-DOF robot (4 legs x 3 joints: hip_roll, thigh_pitch, shin_pitch) with modular trot gait controller, Gazebo Classic simulation, and ros2_control integration.

## Build & Run Commands

```bash
# Build entire workspace
cd /root/dog_ws && colcon build --symlink-install

# Build single package
colcon build --symlink-install --packages-select dog_brain

# Source workspace (required after build, before running)
source /root/dog_ws/install/setup.bash

# Launch full simulation (Gazebo + controllers + trot_node)
ros2 launch dog_bringup gazebo.launch.py

# Launch URDF viewer (RViz + joint_state_publisher_gui)
ros2 launch dog_description view_dog.launch.py

# Run trot controller separately (with params)
ros2 run dog_brain trot_node --ros-args --params-file install/dog_brain/share/dog_brain/config/gait_params.yaml

# Send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"

# Tune gait parameters at runtime
ros2 param set /trot_node trajectory.step_height 0.05
ros2 param set /trot_node gait.period 0.6

# Enable IMU stabilization at runtime
ros2 param set /trot_node stabilization.enabled true
ros2 param set /trot_node stabilization.kp_roll 0.3
```

## Architecture

### Packages (src/)

- **dog_description** — URDF/Xacro robot model, RViz config, joint limits. The xacro defines trunk + 4 legs via `dog_leg` macro with parameters `(prefix, side_x, side_y)`. Includes ros2_control hardware interface and IMU sensor plugin.
- **dog_bringup** — Gazebo launch, controller spawning, world file. Launch sequence: RSP → Gazebo → spawn → JSB → position_controller → trot_node.
- **dog_brain** — Modular gait controller. Split into a pure C++ library (`dog_brain_lib`) and a ROS node (`trot_node`).

### dog_brain Module Architecture

```
dog_brain/
├── include/dog_brain/
│   ├── types.hpp              # RobotGeometry, LegJoints, FootPosition, VelocityCommand, LEG_SIGN_X/Y
│   ├── leg_kinematics.hpp     # LegKinematics: solveIK(), solveFK()
│   ├── gait_generator.hpp     # GaitGenerator: phase offsets, swing/stance timing
│   ├── foot_trajectory.hpp    # FootTrajectory: smoothstep swing, linear stance
│   └── body_controller.hpp    # BodyController: IMU stabilization, quaternionToEuler()
├── src/
│   ├── leg_kinematics.cpp
│   ├── gait_generator.cpp
│   ├── foot_trajectory.cpp
│   ├── body_controller.cpp
│   └── trot_node.cpp          # ROS node orchestrator (thin: delegates all math to library)
└── config/
    └── gait_params.yaml       # All tunable parameters (gait.*, trajectory.*, stabilization.*, filter.*)
```

**Key design**: `dog_brain_lib` has zero ROS dependencies — pure C++ library. Only `trot_node` links against ROS. This enables unit testing without ROS infrastructure.

### Data Flow

```
/cmd_vel (Twist) → [trot_node @ 50Hz] → /joint_group_position_controller/commands (Float64MultiArray)
                         ↑                              ↓
                    /imu/data              [gazebo_ros2_control @ 100Hz, PID p=50 d=1]
                         ↑                              ↓
                         └──────────── [Gazebo ODE @ 2000Hz] ──→ /joint_states
```

**trot_node main loop**: `smoothVelocity() → gait.update(t) → trajectory.compute() → body_ctrl.corrections() → kinematics.solveIK() → publish`

### Joint Ordering Convention

All 12-element joint arrays follow this order (defined in controllers.yaml and types.hpp LegId enum):
`FL_hip, FL_thigh, FL_shin, FR_hip, FR_thigh, FR_shin, RL_hip, RL_thigh, RL_shin, RR_hip, RR_thigh, RR_shin`

### Robot Dimensions (single source: types.hpp RobotGeometry + dog.urdf.xacro)

| Parameter | types.hpp field | URDF property | Value |
|-----------|----------------|---------------|-------|
| Hip lateral offset | `l_hip` | `hip_l` | 0.06 m |
| Thigh length | `l_thigh` | `thigh_l` | 0.144 m |
| Shin length | `l_shin` | `shin_l` | 0.1525 m |
| Body length | `trunk_length` | `trunk_l` | 0.475 m |
| Body width | `trunk_width` | `trunk_w` | 0.18 m |

### Gait System

- **GaitGenerator**: Produces phase info per leg using configurable `phase_offsets` array. Trot={0, 0.5, 0.5, 0}. Easy to add Walk={0, 0.5, 0.75, 0.25}, Pace={0, 0.5, 0, 0.5}, Bound={0, 0, 0.5, 0.5}.
- **FootTrajectory**: Swing = smoothstep X (`3p²-2p³`) + polynomial Z lift (`16p²(1-p)²`). Stance = linear push-back.
- **BodyController**: Proportional IMU stabilization (roll/pitch → per-leg Z corrections). Disabled by default. Enable via `stabilization.enabled` parameter.

### Key Technical Details

- **IK solver** (leg_kinematics.cpp): Law of cosines for knee, atan2+acos for thigh. Hip roll = 0 (lateral walk reserved for future).
- **Initial joint pose**: thigh=0.76 rad, shin=-1.47 rad (URDF `initial_value`). Matches z_nominal=-0.22.
- **Spawn height**: 0.27m (foot ~5mm above ground). MUST match leg reach + foot radius.
- **Physics**: ODE solver, 100 iterations, 1ms timestep, foot friction mu=1.5, contact kp=1e5, kd=100.
- **CRITICAL**: z_nominal, URDF initial_value, and spawn height must be consistent. If you change z_nominal, recalculate IK for the new height and update URDF init_thigh/init_shin + spawn z.

## SolidWorks Mesh Integration

To replace primitive geometry with SolidWorks models:
1. Export each part as STL (File -> Save As -> STL), ensure coordinate origin matches joint frames
2. Place STL files in `src/dog_description/meshes/` (e.g., `body.stl`, `thigh.stl`, `shin.stl`, `hip.stl`)
3. In URDF xacro, replace `<geometry><box>` / `<geometry><cylinder>` with `<geometry><mesh filename="package://dog_description/meshes/part.stl" scale="0.001 0.001 0.001"/>` (scale 0.001 if exported in mm)
4. Keep collision geometry as simplified primitives for simulation performance
5. Recalculate inertia tensors using SolidWorks mass properties or MeshLab
