def sysCall_init():
    global sim, simIK, ikEnv, ikGroup, joints, target
    sim = require('sim')
    simIK = require('simIK')

    ur5    = sim.getObject('/UR5')
    tip    = sim.getObject('/connection')
    target = sim.getObject('/target')

    # Walk the kinematic chain from tip up to base, collecting joints
    joints = []
    h = tip
    while True:
        h = sim.getObjectParent(h)
        if h == -1 or h == ur5:
            break
        if sim.getObjectType(h) == sim.object_joint_type:
            joints.append(h)
    joints.reverse()  # base → tip order
    print(f"Found {len(joints)} joints")

    ikEnv   = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv, ikGroup, ur5, tip, target, simIK.constraint_pose)

    import numpy as np
    T_sd = np.array([
        [1.0, 0.0, 0.0, 0.4],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ])

    sim.setObjectMatrix(target, T_sd[:3, :].flatten().tolist(), -1)
    simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
    theta = [sim.getJointPosition(j) for j in joints]
    print("theta =", theta)

def sysCall_cleanup():
    if 'ikEnv' in globals():
        simIK.eraseEnvironment(ikEnv)