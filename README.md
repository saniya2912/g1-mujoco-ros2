# G1 MuJoCo ROS2 Simulation

ROS2 Humble simulation for the **Unitree G1 Edu U8 (29-DOF)** humanoid robot using MuJoCo.

```
MuJoCo Python Simulator  <--DDS (loopback lo, domain 1)--> ROS2 Nodes (g1_sim)
(unitree_sdk2py_bridge)                                     (unitree_hg msgs)
        |
        +-- MuJoCo viewer (real-time physics)
        +-- RViz (robot_state_publisher + joint_state_bridge)
```

---

## Repository Layout

```
g1-mujoco-ros2/
├── g1_sim/                            # ROS2 package (symlinked into ros2_ws/src/)
│   ├── src/
│   │   ├── stand_g1.cpp               # Stand demo: drives all joints to q=0, then knee bend
│   │   └── motor_crc_hg.cpp           # CRC checksum for unitree_hg LowCmd
│   ├── include/g1_sim/motor_crc_hg.h
│   ├── scripts/
│   │   └── joint_state_bridge.py      # /lowstate → /joint_states + world→pelvis TF
│   ├── launch/
│   │   ├── g1_sim.launch.py           # Starts stand_g1 with correct DDS env vars
│   │   └── g1_rviz.launch.py          # Starts bridge + robot_state_publisher + RViz
│   └── rviz/g1_default.rviz           # RViz config (RobotModel enabled by default)
├── mujoco_scene/scene_29dof.xml       # Modified G1 scene (better default camera)
├── unitree_mujoco/                    # Git submodule
├── unitree_ros/                       # Git submodule (contains G1 URDF)
├── unitree_sdk2/                      # Git submodule (C++ SDK)
└── unitree_sdk2_python/               # Git submodule (Python SDK source)
```

---

## Prerequisites

| Dependency | Version | Install |
|---|---|---|
| Ubuntu | 22.04 | — |
| ROS2 | Humble | [docs.ros.org](https://docs.ros.org/en/humble/Installation.html) |
| MuJoCo | 3.x | `pip3 install mujoco` |
| unitree_sdk2py | latest | `pip3 install unitree-sdk2py` |
| pygame | any | `pip3 install pygame` |
| rmw_cyclonedds_cpp | Humble | `sudo apt install ros-humble-rmw-cyclonedds-cpp` |
| Unitree ROS2 msgs | latest | see setup step 3 |

---

## One-Time Setup

### 1. Clone and initialise submodules

```bash
git clone https://github.com/yourusername/g1-mujoco-ros2.git ~/Projects/g1-mujoco-ros2
cd ~/Projects/g1-mujoco-ros2
git submodule update --init --recursive
```

### 2. Install Python simulator dependencies

```bash
pip3 install mujoco unitree-sdk2py pygame
```

### 3. Build Unitree ROS2 message packages

The `unitree_hg` and `unitree_go` message types are not in the standard ROS2 package index and must be built from source.

```bash
mkdir -p ~/Projects/ros2_ws/src
cd ~/Projects/ros2_ws/src
git clone https://github.com/unitreerobotics/unitree_ros2

cd unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build
```

### 4. Symlink g1_sim into the workspace

Use a symlink (not a copy) so edits to `g1_sim/` take effect without re-copying:

```bash
ln -s ~/Projects/g1-mujoco-ros2/g1_sim ~/Projects/ros2_ws/src/g1_sim
```

### 5. Build the g1_sim package

```bash
cd ~/Projects/ros2_ws
source /opt/ros/humble/setup.bash
source src/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --packages-select g1_sim
```

> **Important:** if you edit C++ source files later, force a clean rebuild to avoid stale cached binaries:
> ```bash
> rm -rf build/g1_sim install/g1_sim && colcon build --packages-select g1_sim
> ```

### 6. Disable joystick in simulator config

`config.py` ships with `USE_JOYSTICK = 1`. Without a physical gamepad connected this causes
`sys.exit()` inside the simulation thread, silently freezing physics. Set it to `0`:

```python
# unitree_mujoco/simulate_python/config.py
USE_JOYSTICK = 0    # was 1 — set to 0 if no Xbox/Switch gamepad is connected
```

### 7. (Optional) Source workspace in every terminal automatically

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Projects/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Running

Open three terminals. Source `~/Projects/ros2_ws/install/setup.bash` in terminals 2 and 3 if you
did not add it to `~/.bashrc`.

### Terminal 1 — MuJoCo simulator

```bash
cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python

DISPLAY=:1.0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.* \
  env -u WAYLAND_DISPLAY python3 unitree_mujoco.py
```

Replace the `XAUTHORITY` glob with your actual file — find it with:
```bash
ls /run/user/1000/.mutter-Xwaylandauth.*
```

> **Why `DISPLAY` and `XAUTHORITY`?** MuJoCo's GLFW viewer needs X11. On Ubuntu 22.04
> (Wayland session), you must explicitly point it at the XWayland server and its auth cookie,
> otherwise `launch_passive` silently fails to open a window. See `whatwentwrong.md` for the
> full story.

Wait until the scene information table finishes printing, then proceed to terminal 2.

### Terminal 2 — Stand demo

```bash
source ~/Projects/ros2_ws/install/setup.bash
ros2 launch g1_sim g1_sim.launch.py
```

The launch file automatically sets `ROS_DOMAIN_ID=1`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
and `CYCLONEDDS_URI` (loopback interface, `MaxAutoParticipantIndex=50`).

Watch for:
```
[stand_g1-1] State received. Starting stand-up sequence...
[stand_g1-1] Demo complete. Holding standing pose.
```

### Terminal 3 — RViz visualisation

```bash
source ~/Projects/ros2_ws/install/setup.bash

DISPLAY=:1.0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.* \
  env -u WAYLAND_DISPLAY \
  ros2 launch g1_sim g1_rviz.launch.py
```

This starts three nodes:
- **joint_state_bridge** — `/lowstate` → `/joint_states` + static `world → pelvis` TF
- **robot_state_publisher** — publishes full `/tf` tree from the G1 URDF
- **rviz2** — opens with RobotModel already enabled (no checkbox clicking needed)

The G1 model should appear at standing height (`z = 0.787 m`) and mirror the MuJoCo simulation
while the stand demo runs.

---

## Stand Demo Sequence

| Phase | Duration | Motion |
|---|---|---|
| 1 | 3 s | Smooth ramp from actual start positions → all joints q = 0 (upright) |
| 2 | 2 s | Hold upright standing |
| 3 | 2 s | Gentle knee bend (knee = 0.6 rad, hip-pitch = 0.3 rad, ankle = −0.3 rad) |
| 4 | 2 s | Return to upright standing |
| — | hold | Maintain upright pose indefinitely |

PD gains: legs `kp=100, kd=2.5` · waist `kp=60, kd=1.5` · arms `kp=40, kd=1.0`

---

## Simulator Config Reference

`unitree_mujoco/simulate_python/config.py`:

| Key | Value | Notes |
|---|---|---|
| `ROBOT` | `"g1"` | Robot model |
| `ROBOT_SCENE` | `"../unitree_robots/g1/scene.xml"` | Path relative to simulate_python/ |
| `DOMAIN_ID` | `1` | DDS domain (real hardware uses 0) |
| `INTERFACE` | `"lo"` | Network interface (real hardware: e.g. `enp3s0`) |
| `USE_JOYSTICK` | **`0`** | Must be 0 unless a physical gamepad is connected |
| `SIMULATE_DT` | `0.005` | Physics timestep (200 Hz) |
| `VIEWER_DT` | `0.02` | Viewer sync rate (50 fps) |

---

## DDS Topics

| Topic | Direction | Type | Description |
|---|---|---|---|
| `/lowcmd` | ROS2 → sim | `unitree_hg/LowCmd` | Per-joint mode, q, dq, kp, kd, tau |
| `/lowstate` | sim → ROS2 | `unitree_hg/LowState` | Joint positions, velocities, torques, IMU |
| `/sportmodestate` | sim → ROS2 | `unitree_go/SportModeState` | Base pose and velocity |
| `/wirelesscontroller` | sim → ROS2 | `unitree_go/WirelessController` | Gamepad input passthrough |

All topics use CycloneDDS on loopback (`lo`), domain ID 1. The bridge maps each DDS topic
`rt/lowcmd` ↔ ROS2 `/lowcmd` automatically via `rmw_cyclonedds_cpp`.

---

## G1 29-DOF Joint Index

| Idx | Joint | Idx | Joint |
|---|---|---|---|
| 0 | left_hip_pitch | 15 | left_shoulder_pitch |
| 1 | left_hip_roll | 16 | left_shoulder_roll |
| 2 | left_hip_yaw | 17 | left_shoulder_yaw |
| 3 | left_knee | 18 | left_elbow |
| 4 | left_ankle_pitch | 19 | left_wrist_roll |
| 5 | left_ankle_roll | 20 | left_wrist_pitch |
| 6 | right_hip_pitch | 21 | left_wrist_yaw |
| 7 | right_hip_roll | 22 | right_shoulder_pitch |
| 8 | right_hip_yaw | 23 | right_shoulder_roll |
| 9 | right_knee | 24 | right_shoulder_yaw |
| 10 | right_ankle_pitch | 25 | right_elbow |
| 11 | right_ankle_roll | 26 | right_wrist_roll |
| 12 | waist_yaw | 27 | right_wrist_pitch |
| 13 | waist_roll | 28 | right_wrist_yaw |
| 14 | waist_pitch | | |

---

## MuJoCo Viewer Controls

| Action | Control |
|---|---|
| Zoom | Scroll wheel |
| Rotate | Left-click + drag |
| Pan | Right-click + drag |
| Switch camera | Tab |
| Pause / resume | Space |

---

## Inspecting the Node Graph

Use `rqt_graph` to visualise nodes, topics, publishers, and subscribers.
**Export the DDS variables before running** — inline assignment breaks the XML:

```bash
source ~/Projects/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="default"/></Interfaces></General><Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'

DISPLAY=:1.0 XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.BZIMP3 \
  env -u WAYLAND_DISPLAY rqt_graph
```

In the window set the dropdown to **Nodes/Topics (all)**. Expected graph:

```
[unitree_mujoco*] ──/lowstate──▶ [g1_stand_demo]
                               ──/lowstate──▶ [joint_state_bridge]
[g1_stand_demo]   ──/lowcmd───▶ [unitree_mujoco*]
[joint_state_bridge] ──/joint_states──▶ [robot_state_publisher]
                     ──/tf_static─────▶ [rviz2]
[robot_state_publisher] ──/tf──────────▶ [rviz2 / transform_listener_impl]
                        ──/tf_static───▶ [rviz2 / transform_listener_impl]
                        ──/robot_description──▶ [rviz2]
```

`*` The MuJoCo simulator is not a ROS2 node — it appears as an anonymous DDS
participant, not a named oval. `transform_listener_impl` is rviz2's internal
TF2 listener sub-node — it is part of rviz2, not a separate process. RViz does
not subscribe to `/joint_states` directly; `robot_state_publisher` converts
joint states to TF and rviz2 consumes only the TF output.

---

## Verifying the Pipeline

```bash
# Confirm lowstate is flowing from simulator (use best-effort QoS)
ros2 topic echo /lowstate --qos-reliability best_effort --once

# Confirm joint_state_bridge is republishing
ros2 topic echo /joint_states --once

# Confirm world→pelvis TF is published (z should be ~0.787)
ros2 run tf2_ros tf2_echo world pelvis

# See all active topics
ros2 topic list
```

Expected topics: `/lowstate`, `/joint_states`, `/robot_description`, `/tf`, `/tf_static`

---

## Known Issues

| Symptom | Cause | Fix |
|---|---|---|
| MuJoCo viewer opens but is unresponsive | Running under Wayland without X11 env vars | Add `DISPLAY=:1.0 XAUTHORITY=... env -u WAYLAND_DISPLAY` |
| Joints never move, `/lowstate` all zeros | `USE_JOYSTICK=1` with no gamepad connected kills the simulation thread silently | Set `USE_JOYSTICK = 0` in `config.py` |
| `Failed to find a free participant index for domain 1` | Too many CycloneDDS participants accumulated | Kill stale processes: `pkill -9 -f "unitree_mujoco\|stand_g1\|rviz2\|joint_state_bridge"` |
| RViz shows `Error retrieving file [meshes/head_link.STL]` | G1 URDF uses bare relative mesh paths with no `package://` prefix | Already fixed in `g1_rviz.launch.py` — mesh paths are rewritten to `file://` absolute URIs at launch time |
| `colcon build` finishes in < 5 s but binary has old behaviour | CMake relinked rather than recompiled due to cached object files | Force clean: `rm -rf build/g1_sim install/g1_sim && colcon build --packages-select g1_sim` |
| `Package 'g1_sim' not found` | Workspace not sourced | `source ~/Projects/ros2_ws/install/setup.bash` |

For a detailed explanation of every one of these issues and how they were diagnosed, see
[whatwentwrong.md](whatwentwrong.md).
