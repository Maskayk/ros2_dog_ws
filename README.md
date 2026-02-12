```
# 🐕 ROS 2 Quadruped Robot ("Dog") Simulation Workspace

A modular, production-ready ROS 2 simulation framework for a 12-DOF quadruped robot with trot gait controller, inverse kinematics, and IMU-based body stabilization.

**Key Features:**
- ✅ Pure C++ kinematics & gait library (ZERO ROS dependencies)
- ✅ Thin ROS 2 wrapper for real-time control
- ✅ 12 joints (4 legs × 3 DoF: hip_roll, thigh_pitch, shin_pitch)
- ✅ Gazebo Classic simulation with ODE physics
- ✅ ros2_control integration for hardware abstraction
- ✅ IMU stabilization with quaternion-based roll/pitch correction
- ✅ Runtime parameter tuning (gait timing, step height, stabilization gains)
- ✅ Multiple gait patterns (Trot, Walk, Pace, Bound ready)

---

## 📐 Architecture Overview

### Library Design Philosophy

**`dog_brain_lib`** — Pure C++ library with **ZERO ROS dependencies**
- All kinematics, gait planning, and trajectory computation
- Fully unit testable without ROS infrastructure
- Portable to real robot hardware
- Headers only in `include/dog_brain/` (types, interfaces)
- Implementation in `src/` (algorithms, math)

**`trot_node`** — Thin ROS 2 wrapper
- Subscribes to `/cmd_vel`, `/imu/data`, `/joint_states`
- Publishes to `/joint_group_position_controller/commands`
- **Delegates all math to `dog_brain_lib`** — no business logic in this node
- 50 Hz control loop
- Parameter server integration for runtime tuning

### Package Structure
ros2_dog_ws/
├── src/
│   ├── dog_description/
│   │   ├── urdf/
│   │   │   ├── dog.urdf.xacro         # Main robot URDF (xacro format)
│   │   │   └── dog_leg.xacro          # Leg macro (reused 4x with parameters)
│   │   ├── config/
│   │   │   └── rviz.rviz             # RViz visualization config
│   │   ├── launch/
│   │   │   └── view_dog.launch.py    # URDF viewer launch
│   │   └── package.xml
│   │
│   ├── dog_bringup/
│   │   ├── launch/
│   │   │   └── gazebo.launch.py      # Full simulation launcher
│   │   ├── worlds/
│   │   │   └── dog_world.world       # Gazebo world file
│   │   ├── config/
│   │   │   ├── controllers.yaml      # controller_manager config
│   │   │   └── ros2_control.yaml     # hardware interface config
│   │   └── package.xml
│   │
│   └── dog_brain/
│       ├── include/dog_brain/
│       │   ├── types.hpp             # Data structures & geometry constants
│       │   ├── leg_kinematics.hpp    # IK/FK algorithms
│       │   ├── gait_generator.hpp    # Gait phase offsets
│       │   ├── foot_trajectory.hpp   # Swing/stance trajectories
│       │   └── body_controller.hpp   # IMU stabilization
│       ├── src/
│       │   ├── leg_kinematics.cpp
│       │   ├── gait_generator.cpp
│       │   ├── foot_trajectory.cpp
│       │   ├── body_controller.cpp
│       │   ├── trot_node.cpp         # ROS 2 node (main entry point)
│       │   └── CMakeLists.txt
│       ├── test/
│       │   └── test_kinematics.cpp   # Unit tests
│       ├── config/
│       │   └── gait_params.yaml      # Tunable parameters
│       └── package.xml
│
├── install/                          # Build artifacts (generated)
├── build/                            # CMake build (generated)
├── CLAUDE.md                         # AI coding guidelines
└── README.md                         # This file 

```
