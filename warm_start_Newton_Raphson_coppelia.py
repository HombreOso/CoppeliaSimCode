#python

# Paste this ENTIRE file (including the `#python` line above) into a
# CoppeliaSim Python script. Press Play to run sysCall_init().
#
# Minimal, hard-coded version. Moves the UR5 so the chosen tip reaches the
# pose T_sd using simIK.handleGroup (bounded, non-blocking).


BASE_ALIAS = 'UR5'
TIP_ALIAS = 'connection'       # 'connection' is the tip dummy of the UR5 model
TARGET_ALIAS = 'UR5_target'

# Desired end-effector pose in world frame (12 elements, row-major 3x4 matrix)
T_sd = [0, 0, -1, 1, 0, 0, 0, -1, 0, -0.5, 0.1, 0.1]

ik_env = -1
ik_group = -1


def _find_in_tree(root, alias):
    for h in sim.getObjectsInTree(root):
        if sim.getObjectAlias(h) == alias:
            return h
    return -1


def sysCall_init():
    global sim, simIK, ik_env, ik_group

    sim = require('sim')
    simIK = require('simIK')

    print('>>> warm_start_Newton_Raphson_coppelia.py is running <<<')

    base = sim.getObject('/' + BASE_ALIAS)
    tip = _find_in_tree(base, TIP_ALIAS)
    if tip == -1:
        raise RuntimeError(f'{TIP_ALIAS} not found under {BASE_ALIAS}')

    joints = sim.getObjectsInTree(base, sim.object_joint_type)
    print(f'base={sim.getObjectAlias(base)} tip={sim.getObjectAlias(tip)} joints={len(joints)}')

    try:
        target = sim.getObject('/' + TARGET_ALIAS)
    except Exception:
        target = sim.createDummy(0.03)
        sim.setObjectAlias(target, TARGET_ALIAS)
        print(f'created /{TARGET_ALIAS}')

    sim.setObjectMatrix(target, -1, T_sd)

    ik_env = simIK.createEnvironment()
    ik_group = simIK.createGroup(ik_env)
    simIK.setGroupCalculation(
        ik_env,
        ik_group,
        simIK.method_damped_least_squares,
        0.3,   # damping
        99,    # max iterations
    )
    simIK.addElementFromScene(
        ik_env, ik_group, base, tip, target, simIK.constraint_pose
    )


def sysCall_actuation():
    # One IK step per simulation step. Bounded and non-blocking.
    simIK.handleGroup(ik_env, ik_group, {'syncWorlds': True})


def sysCall_cleanup():
    global ik_env
    if ik_env != -1 and simIK is not None:
        simIK.eraseEnvironment(ik_env)
        ik_env = -1
