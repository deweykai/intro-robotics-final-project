import numpy as np


def object_to_world(robot_pose, object_pose):
    """
    Convert position returned from camera recognition to world coordinates.
    """

    pose_x, pose_y, pose_theta = robot_pose
    obj_x, obj_y, obj_z = object_pose
    camera_offset = 0.08

    camera_x = np.cos(pose_theta) * camera_offset + pose_x
    camera_y = np.sin(pose_theta) * camera_offset + pose_y

    x = np.cos(pose_theta) * obj_x - \
        np.sin(pose_theta) * obj_y + camera_x
    y = np.sin(pose_theta) * obj_x + \
        np.cos(pose_theta) * obj_y + camera_y
    z = obj_z

    return np.array([x, y, z])
