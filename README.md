# G1 MuJoCo ROS2 Simulation

ROS2 Humble simulation setup for the **Unitree G1 Edu U8** (29-DOF) humanoid robot using MuJoCo.

```
MuJoCo Python Simulator  <--DDS (loopback lo, domain 1)--> ROS2 Nodes
(unitree_sdk2py_bridge)                                     (unitree_hg msgs)
```

## Prerequisites

| Dependency | Version | Notes |
|---|---|---|
| ROS2 | Humble | Ubuntu 22.04 |
| MuJoCo | 3.x | `pip3 install mujoco` |
| unitree_mujoco | — | [github.com/unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) |
| unitree_ros2 | v0.3.0+ | [github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2) |
| unitree_sdk2_python | — | `pip3 install unitree-sdk2py` |
| pygame | — | `pip3 install pygame` |
| rmw_cyclonedds_cpp | Humble | `sudo apt install ros-humble-rmw-cyclonedds-cpp` |

## Repository Contents

```
g1-mujoco-ros2/
├── README.md
├── g1_sim/                        # ROS2 package — place in ros2_ws/src/
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/g1_sim/
│   │   └── motor_crc_hg.h         # CRC checksum for unitree_hg LowCmd
│   ├── src/
│   │   ├── stand_g1.cpp           # Stand demo node
│   │   └── motor_crc_hg.cpp
│   └── launch/
│       └── g1_sim.launch.py       # Launch file (sets DDS env vars)
└── mujoco_scene/
    └── scene_29dof.xml            # Modified G1 scene (better default camera)
```

## Setup

### 1. Initialise submodules

```bash
# Unitree repos are included as submodules — initialise them after cloning
git submodule update --init --recursive

# Unitree ROS2 messages (v0.3.0+) — still a separate clone into the workspace
cd ~/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2

# Build unitree_ros2 message packages
cd unitree_ros2/cyclonedds_ws
colcon build
```

### 2. Apply the modified MuJoCo scene (optional — better camera defaults)

```bash
cp mujoco_scene/scene_29dof.xml \
   ~/Projects/g1-mujoco-ros2/unitree_mujoco/unitree_robots/g1/scene_29dof.xml
```

### 3. Build the g1_sim package

```bash
cp -r g1_sim ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select g1_sim
```

## Running

### Terminal 1 — MuJoCo simulator

```bash
cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python

# Force X11 (required on Ubuntu 22.04 Wayland for interactive viewer)
DISPLAY=:1.0 env -u WAYLAND_DISPLAY python3 unitree_mujoco.py
```

### Terminal 2 — Stand demo

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch g1_sim g1_sim.launch.py
```

Watch for this message confirming DDS communication is working:
```
[stand_g1-1] State received. Starting stand-up sequence...
```

## Stand Demo Behaviour

| Phase | Duration | Action |
|---|---|---|
| 0 | 3 s | All joints → q=0 (upright) from actual current positions |
| 1 | 2 s | Hold standing |
| 2 | 2 s | Gentle knee bend |
| 3 | 2 s | Return to standing |
| — | hold | Maintain standing pose |

## MuJoCo Viewer Controls

| Action | Control |
|---|---|
| Zoom | Scroll wheel |
| Rotate | Left-click + drag |
| Pan | Right-click + drag |
| Switch camera | **Tab** |
| Pause / unpause | **Space** |

## DDS Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/lowcmd` | ROS2 → sim | `unitree_hg/LowCmd` | Motor commands |
| `/lowstate` | sim → ROS2 | `unitree_hg/LowState` | Joint states + IMU |
| `/sportmodestate` | sim → ROS2 | — | Base pose/velocity |

DDS runs on loopback (`lo`), domain ID 1. Change `INTERFACE` in `simulate_python/config.py` and `ROS_DOMAIN_ID` to `0` for real hardware.

## Known Issues

| Issue | Fix |
|---|---|
| MuJoCo viewer unclickable | Run with `DISPLAY=:1.0 env -u WAYLAND_DISPLAY python3 unitree_mujoco.py` |
| `Package 'g1_sim' not found` | Source `~/ros2_ws/install/setup.bash` first |
| Robot falls before demo starts | Launch stand demo within ~2 s of starting simulator |

## G1 29-DOF Joint Index

| Idx | Joint | Idx | Joint |
|---|---|---|---|
| 0–5 | Left leg (hip P/R/Y, knee, ankle P/R) | 15–21 | Left arm (shoulder P/R/Y, elbow, wrist R/P/Y) |
| 6–11 | Right leg | 22–28 | Right arm |
| 12–14 | Waist (yaw, roll, pitch) | | |
