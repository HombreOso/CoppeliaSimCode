# CoppeliaSim UR5 IK — Practical Reference

> Covers every issue encountered when placing a UR5 tip at a desired pose
> `T_sd` (given in the robot base frame) using CoppeliaSim's built-in IK API.

---

## 1. Defining `T_sd` (desired tip pose in the base frame)

`T_sd` is a 4×4 homogeneous transformation matrix that expresses where you
want the tip to be **relative to the robot base**, not the world.

```python
import numpy as np

T_sd = np.array([
    [0.0,  1.0,  0.0, -0.5],
    [0.0,  0.0, -1.0,  0.1],
    [-1.0, 0.0,  0.0,  0.1],
    [0.0,  0.0,  0.0,  1.0],
])
```

The top-left 3×3 block is the rotation; the top-right 3×1 column is the
position **in the base frame (metres)**.

---

## 2. Setting the target — not the tip

**You cannot directly move the robot tip** by calling `setObjectMatrix` on it.
The tip's pose is determined by the joint angles. The correct workflow is:

1. Move the **`target`** dummy to `T_sd` relative to the UR5 base.
2. Run the IK solver so it finds joint angles that place the **`tip`** on the target.

```python
T_sd_flat = T_sd[:3, :].flatten().tolist()   # 12 floats, row-major
sim.setObjectMatrix(target, T_sd_flat, ur5)  # relative to ur5 base handle
```

**Critical arguments:**
| Argument | Value | Why |
|---|---|---|
| Object to move | `target` | Not `tip` — tip is rigidly attached to the chain |
| Reference frame | `ur5` handle | Interprets the matrix in the base frame, not world (`-1`) |

---

## 3. Building the IK environment

```python
ikEnv   = simIK.createEnvironment()
ikGroup = simIK.createGroup(ikEnv)

# REQUIRED: configure solver before solving — default is 1 iteration
simIK.setGroupCalculation(
    ikEnv, ikGroup,
    simIK.method_damped_least_squares,  # robust near singularities
    0.1,   # damping coefficient
    50,    # max iterations (default is 1 — produces ~1e-7 rad "solutions")
)

simIK.addElementFromScene(
    ikEnv, ikGroup, ur5, tip, target,
    simIK.constraint_pose  # constrain full 6-DOF pose
)
```

### Why `setGroupCalculation` is mandatory

The default IK group settings are:
- method: `pseudo_inverse`
- max iterations: **1**

One iteration from the home configuration toward a distant target produces
a joint update of ~1e-7 rad — effectively zero. Always call
`setGroupCalculation` to set a proper iteration count.

---

## 4. Calling `handleGroup` — correct Python signature

The Python API returns a **tuple**, not a dict:

```python
# Correct
result, flags, precision = simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})

# Wrong — causes: AttributeError: 'tuple' object has no attribute 'get'
res = simIK.handleGroup(...)
res.get('calcResult')   # FAILS
```

Interpreting the result:

```python
result_names = {
    simIK.result_not_performed: 'not_performed',
    simIK.result_success:       'success',
    simIK.result_fail:          'fail',
}
print(f"IK result: {result_names.get(result, result)}  precision={precision}")
# precision = [linear_error_m, angular_error_rad]
```

---

## 5. Collecting joints in base → tip order

Walk the scene tree **from tip upward**, collect joints, then reverse:

```python
joints = []
h = tip
while True:
    h = sim.getObjectParent(h)
    if h == -1 or h == ur5:
        break
    if sim.getObjectType(h) == sim.object_joint_type:
        joints.append(h)
joints.reverse()   # now ordered base → tip
```

---

## 6. Holding the pose under dynamics

`simIK.handleGroup(..., {'syncWorlds': True})` writes joint **positions** once,
but the physics engine's motor controllers then drive joints back toward their
last commanded **target position** (default: 0).

To hold the pose permanently:

### In `sysCall_init` — seed the motor targets after the first solve:

```python
result, flags, precision = simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
theta = [sim.getJointPosition(j) for j in joints]
for j, q in zip(joints, theta):
    sim.setJointTargetPosition(j, q)   # tell the motor to hold this angle
```

### In `sysCall_actuation` — re-solve every physics step:

```python
def sysCall_actuation():
    simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
    for j in joints:
        sim.setJointTargetPosition(j, sim.getJointPosition(j))
```

This is needed because dynamics can disturb the joints between steps.

### If you only want a one-time pose with no continuous IK:

Remove `sysCall_actuation` entirely. The `setJointTargetPosition` calls in
`sysCall_init` are sufficient to hold the pose, as long as the joint PID
gains are adequate.

---

## 7. Verifying the tip pose

The object info dialog in CoppeliaSim shows coordinates **relative to world**
by default, not the base frame. To verify that the tip is at `T_sd`:

**Option A — Change the dialog reference frame**
In the object properties dialog, set "Position is relative to" to the UR5 base.
The displayed values should then match the position column of `T_sd`.

**Option B — Print from script**

```python
import numpy as np
M = sim.getObjectMatrix(tip, ur5)    # 12 floats, tip relative to ur5 base
row = M + [0, 0, 0, 1]
T = np.array(row).reshape(4, 4)
print(T)
# Should match T_sd within IK precision (~1e-4 m, ~1e-3 rad)
```

**Option C — Convert manually (works when base has no rotation)**

If the base is axis-aligned with the world:

```
p_tip_world = p_base_world + p_tip_base
```

Example from this session:
- Base world position: `(-0.675, 0.400, 0.015)`
- Desired tip in base frame: `(-0.5, 0.1, 0.1)`
- Expected tip in world: `(-1.175, 0.500, 0.115)` ✓ (matched the dialog readout)

---

## 8. Controlling UR5 from an external Python script

Use the ZeroMQ remote API (not the legacy `import sim` bindings).

### Install

```bash
pip install coppeliasim-zmqremoteapi-client
```

### Correct import (version 2.x)

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
# NOT: from zmqRemoteApi import RemoteAPIClient  (old name, wrong in v2)
```

### Minimal script to set joint angles

```python
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def get_ur5_joints(sim):
    ur5 = sim.getObject('/UR5')
    tip = sim.getObject('/connection')
    joints = []
    h = tip
    while True:
        h = sim.getObjectParent(h)
        if h == -1 or h == ur5:
            break
        if sim.getObjectType(h) == sim.object_joint_type:
            joints.append(h)
    joints.reverse()
    return joints

client = RemoteAPIClient()
sim    = client.getObject('sim')

joints = get_ur5_joints(sim)
thetas = [...]   # 6 values in radians, base → tip order

for joint, theta in zip(joints, thetas):
    sim.setJointTargetPosition(joint, theta)
```

**Requirements:** CoppeliaSim must already be running with the ZeroMQ remote
API add-on active (enabled by default in recent versions).

---

## 9. Common errors and fixes

| Error | Cause | Fix |
|---|---|---|
| `ModuleNotFoundError: No module named 'sim'` | Legacy binding files not on path | Switch to ZeroMQ API |
| `ModuleNotFoundError: No module named 'zmqRemoteApi'` | Wrong module name for v2 package | Use `coppeliasim_zmqremoteapi_client` |
| `AttributeError: 'tuple' object has no attribute 'get'` | `handleGroup` returns a tuple, not a dict | Unpack: `result, flags, precision = simIK.handleGroup(...)` |
| Angles all ~1e-7 | Default IK group has `maxIterations=1` | Call `simIK.setGroupCalculation(..., 50)` |
| Robot snaps back to home after solve | Motors commanded to target=0 by dynamics | Call `setJointTargetPosition` after solve; re-solve in `sysCall_actuation` |
| Tip pose looks wrong in dialog | Dialog shows world frame, not base frame | Change reference frame in dialog, or add `p_base_world` manually |
| IK result is `fail` | Target is outside reach envelope, or too few iterations | Check reachability; increase `maxIterations`; reduce damping |
