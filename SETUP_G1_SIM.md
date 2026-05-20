# G1 Edu U8 — MuJoCo ROS2 Simulation Setup

## Overview

This documents the simulation pipeline for the **Unitree G1 Edu U8 (29-DOF)** humanoid robot using MuJoCo and ROS2 Humble.

```
Python MuJoCo Simulator  <--DDS (loopback)--> ROS2 Nodes (g1_sim)
(unitree_sdk2py_bridge)                        (unitree_hg messages)
       |
       +-- MuJoCo viewer (G1 29-DOF)
```

Communication goes over CycloneDDS on the loopback interface (`lo`), domain ID 1. This is the same protocol the real robot uses, so the same ROS2 nodes work on real hardware by changing the interface and domain ID.

---

## Where Everything Lives

### MuJoCo Simulator

| Path | Description |
|---|---|
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/` | Unitree MuJoCo simulator repo |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python/unitree_mujoco.py` | Python simulator entry point |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python/config.py` | Simulator config (robot, scene, DDS, joystick) |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python/unitree_sdk2py_bridge.py` | DDS bridge (lowcmd/lowstate) |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/unitree_robots/g1/scene_29dof.xml` | G1 29-DOF scene (modified for camera) |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | G1 29-DOF MJCF robot model |
| `~/Projects/g1-mujoco-ros2/unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md` | Joint index reference |

### ROS2 Workspace

| Path | Description |
|---|---|
| `~/Projects/ros2_ws/` | ROS2 Humble workspace root |
| `~/Projects/ros2_ws/src/g1_sim/` | **G1 simulation ROS2 package (created by us)** |
| `~/Projects/ros2_ws/src/unitree_ros2/` | Unitree ROS2 SDK + message definitions |
| `~/Projects/ros2_ws/src/unitree_ros2/cyclonedds_ws/` | Built unitree_hg / unitree_go / unitree_api packages |
| `~/Projects/ros2_ws/install/` | Compiled workspace (source before running) |

### g1_sim Package Files

| Path | Description |
|---|---|
| `src/g1_sim/src/stand_g1.cpp` | Stand demo node — reads lowstate, drives all joints to q=0 then demos knee bend |
| `src/g1_sim/src/motor_crc_hg.cpp` | CRC checksum for unitree_hg LowCmd |
| `src/g1_sim/include/g1_sim/motor_crc_hg.h` | CRC header |
| `src/g1_sim/scripts/joint_state_bridge.py` | Python bridge: /lowstate → /joint_states + world→pelvis TF |
| `src/g1_sim/launch/g1_sim.launch.py` | Launch file (sets DDS env vars, starts stand_g1) |
| `src/g1_sim/launch/g1_rviz.launch.py` | RViz bridge launch (bridge + robot_state_publisher + rviz2) |
| `src/g1_sim/rviz/g1_default.rviz` | RViz config (RobotModel + TF, fixed frame: world) |
| `src/g1_sim/package.xml` | ROS2 package descriptor |
| `src/g1_sim/CMakeLists.txt` | Build config |

### URDF

| Path | Description |
|---|---|
| `~/Projects/g1-mujoco-ros2/unitree_ros/robots/g1_description/g1_29dof.urdf` | G1 29-DOF URDF (from Unitree; root link: pelvis) |

### Dependencies

| Path | Description |
|---|---|
| `~/Projects/g1-mujoco-ros2/unitree_sdk2/` | Unitree C++ SDK (built) |
| `~/Projects/g1-mujoco-ros2/unitree_sdk2_python/` | Unitree Python SDK (installed) |
| `~/Projects/mujoco-3.2.2/` | MuJoCo 3.2.2 |

---

## Simulator Config (`simulate_python/config.py`)

```python
ROBOT = "g1"
ROBOT_SCENE = "../unitree_robots/g1/scene_29dof.xml"
DOMAIN_ID = 1       # simulation domain (real robot uses 0)
INTERFACE = "lo"    # loopback for simulation (real robot: e.g. enp3s0)
USE_JOYSTICK = 1
SIMULATE_DT = 0.005
VIEWER_DT = 0.02
```

---

## Running the Simulation

### Step 1 — Start the MuJoCo simulator

```bash
cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python
DISPLAY=:1.0 env -u WAYLAND_DISPLAY python3 unitree_mujoco.py
```

> The `DISPLAY` and `env -u WAYLAND_DISPLAY` flags force GLFW to use X11 (XWayland) instead of native Wayland, which is required for the interactive viewer to accept mouse/keyboard input on Ubuntu 22.04.

### Step 2 — Source the ROS2 workspace (in a new terminal)

```bash
source /opt/ros/humble/setup.bash
source ~/Projects/ros2_ws/install/setup.bash
```

> `ros2_ws/install/setup.bash` automatically chains the cyclonedds_ws so you don't need to source it separately.

Add to `~/.bashrc` if you want it loaded automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Projects/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 3a — Launch the stand demo (optional)

```bash
ros2 launch g1_sim g1_sim.launch.py
```

The launch file automatically sets:
- `ROS_DOMAIN_ID=1`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` (loopback interface)

Watch for this log line — it confirms DDS is communicating:
```
[stand_g1-1] State received. Starting stand-up sequence...
```

### Step 3b — Launch the RViz visualization bridge

In a separate terminal (with workspace sourced):

```bash
ros2 launch g1_sim g1_rviz.launch.py
```

This starts three nodes:
1. **joint_state_bridge** — subscribes to `/lowstate`, publishes `/joint_states` + `world→pelvis` TF
2. **robot_state_publisher** — reads G1 URDF, publishes full `/tf` tree from joint states
3. **rviz2** — opens the RViz window with the G1 robot model

Override the URDF path if needed:
```bash
ros2 launch g1_sim g1_rviz.launch.py \
  urdf_path:=~/Projects/g1-mujoco-ros2/unitree_ros/robots/g1_description/g1_29dof.urdf
```

**Expected RViz view:** G1 model standing upright in the `world` frame at z=0.787m (pelvis height). When `stand_g1` runs, the model moves in sync with the MuJoCo simulation.

---

## MuJoCo Viewer Controls

| Action | Control |
|---|---|
| Zoom in/out | Scroll wheel |
| Rotate | Left-click + drag |
| Pan | Right-click + drag |
| Switch camera | **Tab** (cycles to "default_view" named camera) |
| Pause / unpause | **Space** |

---

## G1 29-DOF Joint Index Reference

| Index | Joint | Index | Joint |
|---|---|---|---|
| 0 | L_HIP_PITCH | 15 | L_SHOULDER_PITCH |
| 1 | L_HIP_ROLL | 16 | L_SHOULDER_ROLL |
| 2 | L_HIP_YAW | 17 | L_SHOULDER_YAW |
| 3 | L_KNEE | 18 | L_ELBOW |
| 4 | L_ANKLE_PITCH | 19 | L_WRIST_ROLL |
| 5 | L_ANKLE_ROLL | 20 | L_WRIST_PITCH |
| 6 | R_HIP_PITCH | 21 | L_WRIST_YAW |
| 7 | R_HIP_ROLL | 22 | R_SHOULDER_PITCH |
| 8 | R_HIP_YAW | 23 | R_SHOULDER_ROLL |
| 9 | R_KNEE | 24 | R_SHOULDER_YAW |
| 10 | R_ANKLE_PITCH | 25 | R_ELBOW |
| 11 | R_ANKLE_ROLL | 26 | R_WRIST_ROLL |
| 12 | WAIST_YAW | 27 | R_WRIST_PITCH |
| 13 | WAIST_ROLL | 28 | R_WRIST_YAW |
| 14 | WAIST_PITCH | | |

Full reference: `~/Projects/g1-mujoco-ros2/unitree_mujoco/unitree_robots/g1/g1_joint_index_dds.md`

---

## Building the g1_sim Package

```bash
cd ~/Projects/ros2_ws
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select g1_sim
```

---

## DDS Topic Reference

| Topic | Direction | Message Type | Description |
|---|---|---|---|
| `/lowcmd` | ROS2 → Simulator | `unitree_hg/msg/LowCmd` | Motor position/torque commands |
| `/lowstate` | Simulator → ROS2 | `unitree_hg/msg/LowState` | Joint states + IMU |
| `/sportmodestate` | Simulator → ROS2 | `unitree_go/msg/SportModeState` | Base pose/velocity |
| `/wirelesscontroller` | Simulator → ROS2 | `unitree_go/msg/WirelessController` | Gamepad input |
| `/rt/secondary_imu` | Simulator → ROS2 | — | G1 torso IMU |

---

## Verifying the Pipeline

Run these checks after starting the simulator and RViz bridge:

```bash
# Check DDS data flows from simulator
ros2 topic echo /lowstate --qos-reliability best_effort --once

# Check bridge publishes joint states
ros2 topic echo /joint_states --once

# Check world→pelvis TF (should show z=0.787)
ros2 run tf2_ros tf2_echo world pelvis

# List all active topics
ros2 topic list
```

Expected topics: `/lowstate`, `/joint_states`, `/robot_description`, `/tf`, `/tf_static`

---

## Known Issues

| Issue | Fix |
|---|---|
| MuJoCo viewer unclickable (Wayland) | Run with `DISPLAY=:1.0 env -u WAYLAND_DISPLAY python3 unitree_mujoco.py` |
| `Package 'g1_sim' not found` | Source workspace: `source ~/Projects/ros2_ws/install/setup.bash` |
| Robot falls before demo starts | Launch the stand demo within ~2s of starting the simulator |
| `Failed to find a free participant index for domain 1` | Too many DDS processes ran and left stale state. Run: `pkill -f "ros2\|robot_state_publisher\|joint_state_bridge\|unitree_mujoco"` then restart |
| `ros2 topic echo /lowstate` shows Publisher: 1 but NO data | Missing env vars — ensure `ROS_DOMAIN_ID=1` and `CYCLONEDDS_URI` are set (the launch file handles this automatically) |
