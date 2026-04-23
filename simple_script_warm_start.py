def sysCall_init():
    # Declare globals so sysCall_cleanup (and any other callbacks) can access them
    global sim, simIK, ikEnv, ikGroup, joints, target
    # Load CoppeliaSim's main scripting API module
    sim = require('sim')
    # Load CoppeliaSim's inverse-kinematics API module
    simIK = require('simIK')

    # Handle of the UR5 robot model root in the scene
    ur5    = sim.getObject('/UR5')
    # Handle of the tip/end-effector dummy (the frame we want to control)
    tip    = sim.getObject('/connection')
    # Handle of the target dummy (the desired pose the IK solver will chase)
    target = sim.getObject('/target')

    # Walk the kinematic chain from tip up to base, collecting joints
    joints = []
    # Start traversal at the tip
    h = tip
    while True:
        # Move one level up in the scene graph
        h = sim.getObjectParent(h)
        # Stop if we ran off the top (-1) or reached the UR5 root
        if h == -1 or h == ur5:
            break
        # Keep only revolute/prismatic joint objects
        if sim.getObjectType(h) == sim.object_joint_type:
            joints.append(h)
    # Flip so the list is ordered from base joint (1) to tip joint (6)
    joints.reverse()
    # Sanity print; UR5 should show 6
    print(f"Found {len(joints)} joints")

    # Create an isolated IK environment (separate from the simulation scene)
    ikEnv   = simIK.createEnvironment()
    # Create an IK group inside that environment to hold IK elements
    ikGroup = simIK.createGroup(ikEnv)
    # Configure the solver: damped least squares is robust for UR5 near singularities.
    # Args: env, group, method, damping, maxIterations. Default is 1 iteration which
    # barely moves the joints, producing the ~1e-7 "solution" you saw.
    simIK.setGroupCalculation(
        ikEnv, ikGroup, simIK.method_damped_least_squares, 0.1, 50
    )
    # Mirror the UR5 chain (base→tip) into the IK env and set up a full-pose
    # constraint that drives `tip` onto `target`
    simIK.addElementFromScene(ikEnv, ikGroup, ur5, tip, target, simIK.constraint_pose)

    # NumPy is used to build the desired homogeneous transform
    import numpy as np
    # Desired tip pose T_sd, expressed in the UR5 base frame ({s} = space/base)
    T_sd = np.array([
        [0.0, 1.0, 0.0, -0.5],
        [0.0, 0.0, -1.0, 0.1],
        [-1.0, 0.0, 0.0, 0.1],
        [0.0, 0.0, 0.0, 1.0],
    ])

    # Flatten the top 3 rows (3x4 = 12 values, row-major) as CoppeliaSim expects
    T_sd_flat = T_sd[:3, :].flatten().tolist()
    # Set the TARGET's pose (not the tip's!) relative to the UR5 BASE (ur5 handle),
    # so T_sd is interpreted in the base frame exactly as defined above
    sim.setObjectMatrix(target, T_sd_flat, ur5)
    # Solve IK once here so we can print the initial solution for the chosen T_sd.
    # Python signature: result, flags, precision = simIK.handleGroup(env, group, options)
    result, flags, precision = simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
    # Map the numeric result code to a readable string for easier debugging
    result_names = {
        simIK.result_not_performed: 'not_performed',
        simIK.result_success:       'success',
        simIK.result_fail:          'fail',
    }
    # precision is [linear_error, angular_error] between tip and target after solving
    print(f"IK result: {result_names.get(result, result)}  precision={precision}")
    # Read the resulting joint angles (radians) in base→tip order
    theta = [sim.getJointPosition(j) for j in joints]
    # Print solution so the user can inspect it
    print("theta =", theta)
    # Seed the dynamic motor controllers with the IK solution so joints don't snap back
    for j, q in zip(joints, theta):
        sim.setJointTargetPosition(j, q)

def sysCall_actuation():
    # Re-solve IK each simulation step so the tip stays locked on target even under
    # dynamics. syncWorlds syncs current joint states in and writes solved states back.
    simIK.handleGroup(ikEnv, ikGroup, {'syncWorlds': True})
    # For dynamically simulated joints we must command target positions, not positions,
    # otherwise the motor PID drives them back to whatever target was last set (e.g. 0)
    for j in joints:
        sim.setJointTargetPosition(j, sim.getJointPosition(j))

def sysCall_cleanup():
    # Free the IK environment if it was successfully created during init
    if 'ikEnv' in globals():
        simIK.eraseEnvironment(ikEnv)



