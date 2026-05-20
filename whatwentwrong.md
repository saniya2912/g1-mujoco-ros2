# What Went Wrong — Full Debug Log

This documents every bug, misdiagnosis, and fix encountered while setting up the
G1 MuJoCo ROS2 simulation pipeline from scratch. The goal is to explain not just
what broke but _why_ it broke and _how_ we figured it out, so the same class of
problem can be spotted immediately next time.

---

## Bug 1 — MuJoCo can't find the scene XML

### Symptom

```
ValueError: ParseXML: Error opening file '../unitree_robots/g1/scene.xml'
```

### What happened

`config.py` sets `ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene.xml"`.
That is a relative path. MuJoCo resolves it relative to the **current working
directory** of the Python process, not relative to the script file.

When the simulator is launched from a different directory — for example from
`~/Projects/g1-mujoco-ros2/` — the relative path `../unitree_robots/...` resolves
to the wrong place and the file is not found.

### Fix

Always `cd` into `simulate_python/` before running:

```bash
cd ~/Projects/g1-mujoco-ros2/unitree_mujoco/simulate_python
python3 unitree_mujoco.py
```

The launch command must do this explicitly because the shell's working directory
resets between commands in a non-interactive session.

---

## Bug 2 — DDS participant index exhaustion

### Symptom

```
Failed to find a free participant index for domain 1
```

Printed by CycloneDDS when a new process tries to join domain 1. Some processes
start, some silently fail to communicate.

### What happened

CycloneDDS assigns each participant (one per process) an integer index in a shared
memory segment. By default the maximum is around 8. Running the simulator, the
stand demo, RViz, the bridge, and multiple test scripts simultaneously — plus any
stale processes from earlier failed attempts — exhausted the default pool.

### Fix

Two changes together solved it:

**1. Raise the limit in `CYCLONEDDS_URI`** (added to both launch files):

```xml
<Discovery><MaxAutoParticipantIndex>50</MaxAutoParticipantIndex></Discovery>
```

**2. Kill stale processes** before restarting:

```bash
pkill -9 -f "unitree_mujoco\|stand_g1\|rviz2\|joint_state_bridge"
```

Stale participants hold their index slots even after the parent process is gone if
the shared memory segment wasn't cleaned up, so restarting without killing first
makes the problem worse.

---

## Bug 3 — RViz cannot load mesh files

### Symptom

```
[rviz2] Error retrieving file [meshes/head_link.STL]: Could not resolve host: meshes
```

Printed for every link mesh. The robot model appeared in RViz as a white skeleton
with no geometry.

### What happened

The G1 URDF (`g1_29dof.urdf`) references mesh files with bare relative paths:

```xml
<mesh filename="meshes/pelvis.STL"/>
```

There is no `package://` prefix and no `package.xml` in the `g1_description/`
directory, so `robot_state_publisher` cannot resolve the path through the ROS2
package system. RViz tries to treat `meshes` as a hostname in a URL and fails.

### Fix

Rewrite all mesh paths to absolute `file://` URIs at launch time inside
`g1_rviz.launch.py`, before the URDF string is passed to `robot_state_publisher`:

```python
mesh_dir = str(Path(urdf_path).parent / "meshes")
robot_description = robot_description.replace(
    'filename="meshes/',
    f'filename="file://{mesh_dir}/',
)
```

This converts `meshes/pelvis.STL` to `file:///home/saniya/.../g1_description/meshes/pelvis.STL`
which RViz can load without any package resolution.

---

## Bug 4 — Edited source code, rebuilt, binary still had old behaviour

### Symptom

After editing `stand_g1.cpp` (changed knee bend target from `0.7` to `0.6`) and
running `colcon build --packages-select g1_sim`, the build finished in under 4
seconds. The installed binary still behaved as if the old value was present.

### What happened

The build completed in 3.98 s, which is the time for linking only — no
recompilation happened. CMake checks modification timestamps on object files and
source files. Because the symlink in `ros2_ws/src/g1_sim` points to the source
tree, CMake may have had a stale dependency graph that didn't detect the change, or
object files from a previous build in `build/g1_sim/` were already newer than the
source.

We confirmed the binary had the old value by scanning for the floating-point
constant directly in the installed ELF binary:

```bash
python3 -c "
import struct, sys
data = open('/home/saniya/Projects/ros2_ws/install/g1_sim/lib/g1_sim/stand_g1','rb').read()
needle = struct.pack('<d', 0.7)    # old knee bend value
print('old 0.7 found:', needle in data)
needle2 = struct.pack('<d', 0.6)   # new value
print('new 0.6 found:', needle2 in data)
"
```

Output confirmed `0.7` was present and `0.6` was not — the binary was genuinely
not rebuilt.

### Fix

Force a clean rebuild by deleting CMake's cached build and install trees:

```bash
rm -rf build/g1_sim install/g1_sim
colcon build --packages-select g1_sim
```

The rebuild took 23.7 s (full recompile). Afterwards the binary scan found `0.6`
and not `0.7`.

**Lesson:** When behaviour doesn't match edited source, check build time. A 4-second
build that normally takes 20+ seconds is a sign only relinking happened. Delete the
build directory entirely.

---

## Bug 5 — MuJoCo viewer window never appeared (Wayland / XAUTHORITY)

### Symptom

The MuJoCo Python process started and printed its scene information table, but no
viewer window appeared. Subsequent code that checks `viewer.is_running()` would
have evaluated to `False` immediately, meaning the simulation loop never ran.

The log showed:

```
/home/saniya/.local/lib/python3.10/site-packages/glfw/__init__.py:914:
GLFWError: (65544) b'Wayland: Window position retrieval not supported'
```

### What happened

Ubuntu 22.04 uses a Wayland compositor (GNOME/Mutter) as the primary display
server. MuJoCo's viewer uses GLFW, which tries to create a window. On a Wayland
session without explicit direction, GLFW tries to use the Wayland backend but hits
incompatibilities with how the window position API works.

The fix is to use XWayland — the X11 compatibility layer that Mutter runs
alongside Wayland. GLFW supports X11 well. Two environment variables are required:

- `DISPLAY=:1.0` — tells GLFW which X display to use (XWayland is on `:1.0`)
- `XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.*` — provides the auth cookie
  so GLFW can connect to that X server

- `env -u WAYLAND_DISPLAY` — unsets `WAYLAND_DISPLAY` so GLFW doesn't prefer
  Wayland even when `DISPLAY` is set

Without `XAUTHORITY` specifically, GLFW can find the X display but cannot
authenticate, so `glfwInit()` fails silently. `mujoco.viewer.launch_passive()`
catches this and returns a viewer object where `is_running()` is `False` from
the start.

### Fix

```bash
DISPLAY=:1.0 \
XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.BZIMP3 \
env -u WAYLAND_DISPLAY \
python3 unitree_mujoco.py
```

Find the correct auth file with:
```bash
ls /run/user/1000/.mutter-Xwaylandauth.*
```

The filename changes across reboots (the suffix is a random token).

---

## Bug 6 — Multiple simulator instances publishing stale zeros

### Symptom

`/lowstate` was arriving with publisher count > 1. All motor positions were `0.0000`
even after the stand demo reportedly completed. Subscribing to the raw DDS topic
showed data arriving but always zero.

### What happened

Previous failed runs had left multiple `unitree_mujoco.py` processes running. Each
was publishing `/lowstate` at 500 Hz with the same DDS topic, so subscribers
received a mix of messages from several sources. The stale processes had `mj_step`
never called (see Bug 7), so their `sensordata` was always zero. Even when a new
correct process was started, the zero-publishing stale instances drowned it out.

### Fix

Kill all instances before starting a new one:

```bash
pkill -9 -f "unitree_mujoco.py"
```

Then verify only one remains:

```bash
ps aux | grep unitree_mujoco | grep -v grep
```

---

## Bug 7 — Physics never stepped, joints always read 0.000 (the main bug)

### Symptom

The stand demo printed:
```
State received. Starting stand-up sequence...
Demo complete. Holding standing pose.
```

But reading `/lowstate` showed all joint positions at `0.0000` before and after the
demo. RViz showed the robot model frozen in a T-pose. No movement was visible in
the MuJoCo viewer either, even though it was open.

### Investigation

We added debug prints at multiple layers to trace where things broke down.

**Step 1 — Confirm commands are sent.**
The stand demo publishes `/lowcmd` at 500 Hz. We confirmed this with `ros2 topic hz /lowcmd`.

**Step 2 — Confirm the bridge receives commands.**
Added a debug print inside `LowCmdHandler` in `unitree_sdk2py_bridge.py`:

```python
print(f"[bridge dbg] knee: kp={kp3:.1f} q_tgt={q3:.3f} q_act={s3:.3f} ctrl={c3:.3f}")
```

Output: `[bridge dbg #1] knee: kp=100.0 q_tgt=0.800 q_act=0.000 ctrl=80.000`

The bridge **was** receiving commands and computing `ctrl[3] = 80.0 Nm` for the
knee. So the DDS channel worked end to end. But `q_act` (read from `sensordata[3]`)
was `0.000`, always.

**Step 3 — Confirm `mj_step` is being called.**
Added prints in `SimulationThread` in `unitree_mujoco.py`:

```python
print(f"[sim] viewer.is_running() = {viewer.is_running()}")
# ... inside the loop:
print(f"[sim dbg #{_sim_step_count}] t={mj_data.time:.2f}s knee_q={mj_data.sensordata[3]:.4f}")
```

Neither print ever appeared in the log. The simulation thread was exiting before
reaching the `while viewer.is_running():` line.

**Step 4 — Find what kills the thread before the loop.**
Looking at `SimulationThread`:

```python
def SimulationThread():
    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)  # <-- here
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():   # never reached
        ...
```

With `USE_JOYSTICK = 1` in `config.py`, `SetupJoystick` runs. Inside:

```python
def SetupJoystick(self, device_id=0, js_type="xbox"):
    pygame.init()
    pygame.joystick.init()
    joystick_count = pygame.joystick.get_count()
    if joystick_count > 0:
        self.joystick = pygame.joystick.Joystick(device_id)
        self.joystick.init()
    else:
        print("No gamepad detected.")
        sys.exit()   # <-- kills the simulation thread entirely
```

No Xbox or Switch gamepad was connected. `sys.exit()` was called inside the
simulation thread. Python's `sys.exit()` raises `SystemExit`, which propagates up
through the call stack and terminates the thread.

**Why did everything else appear to work?**

The DDS bridge (`LowCmdHandler`, `PublishLowState`) runs in separate threads spawned
by the CycloneDDS runtime — not in `SimulationThread`. So those threads continued
running normally:

- `/lowstate` **was published** — but `sensordata` was always the initial zero
  because `mj_step` was never called
- `LowCmdHandler` **did run** and **did set** `mj_data.ctrl[3] = 80.0` — but
  without `mj_step`, the physics engine never applied those forces

The robot appeared to stand in the MuJoCo viewer because the viewer was open and
`viewer.sync()` ran in `PhysicsViewerThread` — but `mj_data.qpos` never changed
from its initial values (the pose set in the scene XML, which is already upright).

### Root cause summary

`config.py` defaults to `USE_JOYSTICK = 1`. Without a gamepad, `sys.exit()` in
`SetupJoystick` terminates the simulation thread before `mj_step` is ever called.
Physics is frozen at t=0. The DDS bridge threads keep running, publishing initial
(zero) sensor values indefinitely. Everything _looks_ like it's working but the
robot is a statue.

### Fix

```python
# unitree_mujoco/simulate_python/config.py
USE_JOYSTICK = 0   # changed from 1
```

After this change the simulation thread reaches `while viewer.is_running()` and
steps physics at 200 Hz. Joint positions in `/lowstate` immediately became non-zero
and matched the stand demo commands.

---

## Summary Table

| # | Symptom | Root cause | Fix |
|---|---|---|---|
| 1 | `Error opening file '../unitree_robots/...'` | Wrong working directory when launching | Always `cd simulate_python/` first |
| 2 | `Failed to find a free participant index` | Too many DDS participants; default cap too low | Kill stale processes; add `MaxAutoParticipantIndex=50` |
| 3 | RViz mesh errors `Could not resolve host: meshes` | G1 URDF uses bare relative mesh paths, no `package://` | Rewrite to `file://` absolute URIs in launch file |
| 4 | Edited C++ source, rebuilt, binary unchanged | CMake relinked only; object files not invalidated | `rm -rf build/g1_sim install/g1_sim` then rebuild |
| 5 | MuJoCo viewer silent (no window) | GLFW can't authenticate with XWayland without `XAUTHORITY` | Set `DISPLAY`, `XAUTHORITY`, unset `WAYLAND_DISPLAY` |
| 6 | `/lowstate` all zeros, multiple publishers | Stale zombie simulator processes publishing zero state | `pkill -9 -f "unitree_mujoco.py"` before restarting |
| 7 | Physics frozen, joints always 0 despite demo "completing" | `USE_JOYSTICK=1` + no gamepad → `sys.exit()` kills simulation thread before `mj_step` | Set `USE_JOYSTICK = 0` in `config.py` |

---

## Debugging Techniques That Were Useful

**Binary constant scan** — when a binary doesn't match edited source, scan for the
floating-point constant directly in the ELF:
```python
import struct
data = open('stand_g1', 'rb').read()
print(struct.pack('<d', 0.6) in data)
```

**Layered print tracing** — when end-to-end behaviour is wrong, add prints at each
stage (sender → bridge → physics → sensor readback) and find the first layer where
the print is absent. If `LowCmdHandler` prints but `mj_step` counter doesn't, the
simulation thread died between those two points.

**ps + log timestamp cross-reference** — check `ps aux` CPU usage. A
physics-stepping process uses 100–200% CPU on 2 threads. A frozen process idles
near 0%. If the MuJoCo process shows 0% after startup, the simulation thread exited.

**DDS topic introspection** — `ros2 topic info /lowstate` shows publisher and
subscriber counts. Multiple publishers on the same topic is a sign of zombie
processes. `ros2 topic hz` shows actual message rate.
