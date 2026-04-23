#python

import numpy as np

def sysCall_init():
    global sim, simIK, ikEnv, ikGroup, joints, target
    sim = require('sim')
    simIK = require('simIK')

    ur5        = sim.getObject('/UR5')
    tip        = sim.getObject('/connection')
    target     = sim.getObject('/target')
    joints     = [sim.getObject(f'/joint{i+1}') for i in range(6)]

    ikEnv   = simIK.createEnvironment()
    ikGroup = simIK.createGroup(ikEnv)
    simIK.addElementFromScene(ikEnv, ikGroup, ur5, tip, target, simIK.constraint_pose)

    # ---- Desired end-effector pose T_sd (world frame) ----
    T_sd = np.array([
        [1.0, 0.0, 0.0, 0.4],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.3],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Move target dummy to T_sd, solve IK, read joints
    sim.setObjectMatrix(target, T_sd[:3, :].flatten().tolist(), -1)
    simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
    theta = [sim.getJointPosition(j) for j in joints]
    print("theta =", theta)

def sysCall_cleanup():
    simIK.eraseEnvironment(ikEnv)