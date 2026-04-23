#python
# This script sets the joint angles of the UR5 robot in CoppeliaSim via 
# the ZMQ remote API.


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


def main():
    client = RemoteAPIClient()
    sim = client.getObject('sim')

    print('Connected to CoppeliaSim')

    joints = get_ur5_joints(sim)
    print(f'Found {len(joints)} joints')

    # Desired UR5 joint angles in radians, ordered base -> tip.
    # Which were obtained by running the simple_script_warm_start.py script.
    thetas = [
        1.0249331404814992,
        0.5266493442780539,
        1.7266663629350327,
        0.8880957233026937,
        -0.5458554475372501,
        0.0001632317355335502,
    ]

    if len(joints) != len(thetas):
        raise RuntimeError(f'Expected {len(thetas)} joints, found {len(joints)}')

    for joint, theta in zip(joints, thetas):
        sim.setJointTargetPosition(joint, theta)

    print('Joint target positions set.')


if __name__ == '__main__':
    main()