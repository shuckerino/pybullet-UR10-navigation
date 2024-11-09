import time
from itertools import product

import numpy as np
import pybullet as p
import pybullet_data

import socket

def create_marker_point(x, y, z, size, colour=[0, 255, 0, 255]):
    """
    Create a marker point in the PyBullet simulation environment.

    Parameters:
    - x (float): x-coordinate of the marker point.
    - y (float): y-coordinate of the marker point.
    - z (float): z-coordinate of the marker point.
    - size (list): List of three floats representing the half extents of the marker point.
    - colour (list, optional): List of four integers representing the RGBA color of the marker point. Defaults to [0, 255, 0, 255].

    Returns:
    - shape_id (int): The unique identifier of the created marker point shape.

    """
    shape_id = p.createVisualShape(
        shapeType=p.GEOM_BOX,
        halfExtents=size,
        rgbaColor=colour,
    )

    # Create a multibody with the cube visual shape at the desired position
    # The baseMass=0 makes it static (i.e., it won't move or fall due to gravity)
    p.createMultiBody(
        baseMass=0,
        baseInertialFramePosition=[0, 0, 0],
        baseVisualShapeIndex=shape_id,
        basePosition=[x, y, z],
    )
    return shape_id


def load_robot():
    """
    Loads a UR10 robot model in the PyBullet physics simulation environment.

    Returns:
        robot_id (int): The unique identifier of the loaded robot model.
        joints (list): A list of dictionaries containing information about the robot's joints.
            Each dictionary contains the following keys:
                - jointID (int): The unique identifier of the joint.
                - jointName (str): The name of the joint.
                - jointType (str): The type of the joint (REVOLUTE, PRISMATIC, SPHERICAL, PLANAR, FIXED).
                - jointLowerLimit (float): The lower limit of the joint's range of motion.
                - jointUpperLimit (float): The upper limit of the joint's range of motion.
                - jointMaxForce (float): The maximum force that can be applied to the joint.
                - jointMaxVelocity (float): The maximum velocity that the joint can achieve.
    """
    robot_id = p.loadURDF(
        "urdf/ur10_robot.urdf",
        [0, 0, 0],
        p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=True,
    )
    joint_type = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
    joints = []

    for joint_id in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, joint_id)
        data = {
            "jointID": info[0],
            "jointName": info[1].decode("utf-8"),
            "jointType": joint_type[info[2]],
            "jointLowerLimit": info[8],
            "jointUpperLimit": info[9],
            "jointMaxForce": info[10],
            "jointMaxVelocity": info[11],
        }
        if data["jointType"] != "FIXED":
            joints.append(data)
    return robot_id, joints


def get_arm_position(robot_id):
    """
    Get the position of the arm's end effector.

    Parameters:
        robot_id (int): The ID of the robot in the simulation.

    Returns:
        numpy.ndarray: The position of the arm's end effector as a numpy array.
    """
    return np.array(p.getLinkState(robot_id, linkIndex=10, computeLinkVelocity=True)[0])


def euclidean_distance(a, b):
    """
    Calculate the Euclidean distance between two points.

    Parameters:
    a (numpy.ndarray): The first point.
    b (numpy.ndarray): The second point.

    Returns:
    float: The Euclidean distance between the two points.
    """
    return np.linalg.norm(a - b)


def create_tcp_connection(unity_ip : str, unity_port : int):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(f"Trying to connect to {unity_ip}:{unity_port}")
    client_socket.connect((unity_ip, unity_port))
    return client_socket


def send_joint_positions_to_unity(joint_angles : list[float], client_socket):
    """
    Sends the UR10's joint angles to Unity.
    Expects `joint_angles` as a list of x floats, with x being the number of joints the robot arm has.
    """
    # Construct message
    message_type = 3  # UpdatePose as per RobotControllerMessageType in Unity
    payload = ','.join(map(str, joint_angles)) + ',0,0,0,0,0,0,0'  # Extra zeros for brick data
    message = f"{message_type}\t{payload}"
    
    # Send message to Unity
    client_socket.sendall(message.encode('ascii'))

if __name__ == "__main__":
    
    unity_ip = "127.0.0.1"
    unity_port = 40000
    
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(
        cameraYaw=80,
        cameraPitch=0,
        cameraDistance=2,
        cameraTargetPosition=[2, 0, 1],
    )
    p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot_movement.mp4")

    robot_id, joints = load_robot()

    points = []
    for y, z in product(np.linspace(-0.5, 0.5, 7), np.linspace(0.25, 1.0, 6)):
        create_marker_point(0.5, y, z, [0.025, 0.025, 0.025])
        points.append([0.5, y, z])
    points.append([0, 0, 0.75])

    target_point_idx = [
        18,
        13,
        8,
        9,
        10,
        16,
        21,
        28,
        34,
        33,
        32,
        25,
        15,
        14,
        20,
        19,
        27,
        26,
        42,
    ]

    curr_point_idx = 0
    
    # Connect to unity
    client_socket = create_tcp_connection(unity_ip, unity_port)

    while curr_point_idx < len(target_point_idx):
        if curr_point_idx == len(target_point_idx) -1:
            curr_point_idx = 0
        if (
            euclidean_distance(
                get_arm_position(robot_id), points[target_point_idx[curr_point_idx]]
            )
            < 0.1
        ):
            create_marker_point(
                points[target_point_idx[curr_point_idx]][0],
                points[target_point_idx[curr_point_idx]][1],
                points[target_point_idx[curr_point_idx]][2],
                [0.05, 0.05, 0.05],
                [255, 0, 0, 255],
            )
            curr_point_idx += 1
            if curr_point_idx < len(target_point_idx):
                print("New target:", points[target_point_idx[curr_point_idx]])
            else:
                print("Finished!")
            continue

        joint_poses = p.calculateInverseKinematics(
            robot_id,
            10,  # 'gripper_finger_joint'
            points[target_point_idx[curr_point_idx]],
            p.getQuaternionFromEuler([0, 0, 0]),
            maxNumIterations=100,
            residualThreshold=0.01,
        )

        for joint_id in range(6):
            p.setJointMotorControl2(
                robot_id,
                joints[joint_id]["jointID"],
                p.POSITION_CONTROL,
                targetPosition=joint_poses[joint_id],
            )

        for joint_id in range(6, 12):
            value = -1
            minval, maxval = (
                joints[joint_id]["jointLowerLimit"],
                joints[joint_id]["jointUpperLimit"],
            )
            value = (value + 1) / 2 * (maxval - minval) + minval
            p.setJointMotorControl2(
                robot_id,
                joints[joint_id]["jointID"],
                p.POSITION_CONTROL,
                targetPosition=value,
            )

        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        p.stepSimulation()

    p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)
    time.sleep(3)
