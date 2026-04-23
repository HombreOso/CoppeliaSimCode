#python slots

import numpy as np

def sysCall_init():
    sim = require('simIK')
    global sim_ik, sim_handle, ur5_base, ee_tip, ee_target, joints, ik_env, ik_group
    sim_ik = sim
    sim_handle = require('sim')

    # ---- Handles (edit these strings to match your scene) ----
    ur5_base  = sim_handle.getObject('/UR5_base_ID_to_be_edited')
    ee_tip    = sim_handle.getObject('/endeffector_ID_to_be_edited')       # tip dummy
    ee_target = sim_handle.getObject('/endeffectorTarget_ID_to_be_edited') # target dummy
    joints = [sim_handle.getObject(f'/UR5_joint{i+1}_ID_to_be_edited') for i in range(6)]

    # ---- Build IK environment ----
    ik_env = sim_ik.createEnvironment()
    ik_group = sim_ik.createGroup(ik_env)
    sim_ik.addElementFromScene(ik_env, ik_group, ur5_base, ee_tip, ee_target,
                               sim_ik.constraint_pose)

def sysCall_actuation():
    # ---- Desired pose T_sd (4x4 homogeneous matrix) ----
    T_sd = np.array([
        [ 0.0,  1.0,  0.0, -0.5],
        [ 0.0,  0.0, -1.0,  0.1],
        [-1.0,  0.0,  0.0,  0.1],
        [ 0.0,  0.0,  0.0,  1.0],
    ])

    # Convert to CoppeliaSim's 12-element matrix (row-major, top 3 rows)
    pose = T_sd[:3, :].flatten().tolist()

    # Move the target dummy to T_sd (expressed in world frame)
    sim_handle.setObjectMatrix(ee_target, -1, pose)

    # Solve IK
    sim_ik.handleGroup(ik_env, ik_group, {'syncWorlds': True})

    # Read the resulting joint values theta
    theta = [sim_handle.getJointPosition(j) for j in joints]
    print("theta =", theta)

def sysCall_cleanup():
    sim_ik.eraseEnvironment(ik_env)