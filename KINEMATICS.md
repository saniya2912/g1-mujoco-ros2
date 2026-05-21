# G1 Forward / Inverse Kinematics Options

Reference for choosing an FK/IK approach for the Unitree G1 (29-DOF) in this
workspace. Covers what's already installed, what's wired up in the repos,
and what's a `pip install` away.

---

## Options at a glance

| Option | FK | IK | Installed? | Effort |
|---|---|---|---|---|
| **MuJoCo built-ins** (`mj_kinematics`, `mj_jacBody`, `mj_jacSite`) | ✅ direct | DLS / Jacobian-pseudoinverse in ~15 lines | ✅ `mujoco==3.8.1` | Lowest — no install. |
| **PyKDL** + `kdl_parser_py` | ✅ | ✅ Newton-Raphson / LMA chain solver | ✅ `python_orocos_kdl_vendor`, `kdl_parser` (ROS2 Humble pkgs) | Low. |
| **g1pilot OpenSoT solver** at `g1pilot/g1pilot/manipulation/opensot_solver.py` | ✅ via OpenSoT | ✅ whole-body QP: Cartesian + CoM + joint/velocity limits + collision avoidance | ❌ needs `pyopensot`, `xbot2_interface`, `pyopensot_collision` | High — OpenSoT is a C++ lib with non-trivial build. |
| **Pinocchio** | ✅ best-in-class for humanoid research | ✅ analytical + iterative | ❌ `pip install pin` or `apt install python3-pinocchio` | Low–medium. |
| **MoveIt2** | ✅ | ✅ via KDL / TRAC-IK / Bio-IK plugins | ❌ needs `ros-humble-moveit*` + a G1 MoveIt config (no one has shipped one yet) | High. |

**Repos that do *not* provide kinematics:** `unitree_sdk2`, `unitree_sdk2_python`,
`unitree_mujoco`, `unitree_rl_gym`. All comms / sim / RL — no FK/IK utilities.

---

## URDFs / MJCFs available in this workspace

Use any of these as input to KDL, Pinocchio, OpenSoT, or MoveIt:

| Path | Format | Notes |
|---|---|---|
| `unitree_ros/robots/g1_description/g1_29dof.urdf` | URDF | Canonical 29-DOF, used by our `g1_rviz.launch.py`. |
| `unitree_rl_gym/resources/robots/g1_description/g1_29dof.urdf` | URDF | Same robot, separate copy. |
| `unitree_rl_gym/resources/robots/g1_description/g1_12dof.urdf` | URDF | Legs-only (what the RL walking policy uses). |
| `unitree_rl_gym/resources/robots/g1_description/g1_23dof.urdf` | URDF | Locked waist + legs + arms, no wrists. |
| `unitree_rl_gym/resources/robots/g1_description/g1_29dof_with_hand.urdf` | URDF | Adds Dex3 hand joints. |
| `unitree_mujoco/unitree_robots/g1/g1_29dof.xml` | MJCF | What `unitree_mujoco.py` loads. |
| `unitree_rl_gym/resources/robots/g1_description/*.xml` | MJCF | Matching MuJoCo bodies for each URDF above. |
| `g1pilot/description_files/` | Mixed | Tuned for OpenSoT — used by g1pilot's solver if installed. |

---

## Recommendation by use case

| You want to… | Use |
|---|---|
| Compute where the hand is *right now* (FK) inside a MuJoCo Python script | MuJoCo built-ins (`mj_kinematics`, read `data.site_xpos` / `data.body_xpos`). |
| Drive end-effector to a Cartesian target from a ROS2 node, single chain | PyKDL — parse the URDF chain pelvis → left_wrist_yaw_link, call `ChainIkSolverPos_LMA`. |
| Whole-body posture (reach + balance + look-at simultaneously, with collision avoidance) | g1pilot's OpenSoT solver if you can install OpenSoT; otherwise Pinocchio + write your own QP. |
| Motion plan around obstacles (collision-free trajectories) | MoveIt2 — but you'll first need a G1 MoveIt config package (write SRDF, configure planners). |
| Research-grade humanoid kinematics + dynamics (CoM Jacobian, contact Jacobians, RNEA) | Pinocchio. |

---

## Quick-start snippets (the two zero-install options)

### MuJoCo: FK + Jacobian + damped-least-squares IK

```python
import mujoco, numpy as np

m = mujoco.MjModel.from_xml_path(
    ".../unitree_rl_gym/resources/robots/g1_description/g1_29dof.xml")
d = mujoco.MjData(m)

# Forward kinematics for the current d.qpos:
mujoco.mj_forward(m, d)

# Where is the left wrist body in world frame?
body_id = m.body("left_wrist_yaw_link").id
pos  = d.xpos[body_id].copy()        # 3-vec
quat = d.xquat[body_id].copy()       # wxyz

# Translational Jacobian (3 x nv) for that body:
jacp = np.zeros((3, m.nv))
jacr = np.zeros((3, m.nv))
mujoco.mj_jacBody(m, d, jacp, jacr, body_id)

# Damped-least-squares step toward target_pos:
target_pos = np.array([0.30, 0.20, 1.10])
err = target_pos - pos
damping = 0.05
dq = jacp.T @ np.linalg.solve(jacp @ jacp.T + damping**2 * np.eye(3), err)

# Apply step and re-evaluate:
d.qvel[:] = 0
d.qpos[7:] += dq[6:]   # skip freejoint (first 7 of qpos, 6 of qvel)
mujoco.mj_forward(m, d)
```

Iterate this loop until `‖err‖ < tol`. About 15 lines total. Works for any
chain (just change `body_id`).

### PyKDL: Chain FK + LMA IK from URDF

```python
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromFile

ok, tree = treeFromFile(".../unitree_ros/robots/g1_description/g1_29dof.urdf")
chain = tree.getChain("pelvis", "left_wrist_yaw_link")

fk = kdl.ChainFkSolverPos_recursive(chain)
ik = kdl.ChainIkSolverPos_LMA(chain)

# FK: joint angles -> Cartesian pose
q = kdl.JntArray(chain.getNrOfJoints())
# ... fill q with current arm joint values ...
pose = kdl.Frame()
fk.JntToCart(q, pose)
print(pose.p, pose.M)   # position, rotation

# IK: target pose -> joint angles (seed with current q)
target = kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(0.30, 0.20, 1.10))
q_out  = kdl.JntArray(chain.getNrOfJoints())
ik.CartToJnt(q, target, q_out)
```

The G1's left-arm chain `pelvis → left_wrist_yaw_link` has 7 joints — solvable
analytically, but LMA is fine. Right arm: swap `left_` → `right_`.

---

## Installing the harder options (if you decide to go there)

### Pinocchio (recommended next step beyond KDL)

```bash
# Easiest:
pip install pin

# Or system-wide via apt (more reliable for ROS2 integration):
sudo apt install python3-pinocchio
```

Then:
```python
import pinocchio as pin
model = pin.buildModelFromUrdf(".../g1_29dof.urdf",
                                pin.JointModelFreeFlyer())
data = model.createData()
```

### OpenSoT (for g1pilot's solver)

Follow the OpenSoT install guide — typically:
1. Build OpenSoT from source (`https://github.com/ADVRHumanoids/OpenSoT`)
2. Build the Python bindings (`pyopensot`)
3. Install `xbot2_interface` (`pip install xbot2_interface` or build from
   `https://github.com/ADVRHumanoids/xbot2_interface`)
4. Install `pyopensot_collision` for collision avoidance support

Then `g1pilot/g1pilot/manipulation/opensot_solver.py` becomes runnable.
Heavy install — only worth it if you actually want whole-body QP control.

### MoveIt2

```bash
sudo apt install ros-humble-moveit ros-humble-moveit-resources \
                 ros-humble-moveit-visual-tools
```

Then you'd need to author a G1 MoveIt config package (URDF + SRDF + planning
config + controllers config). No G1 MoveIt config exists in this workspace
or in the Unitree repos as of this writing — would need to be created.
