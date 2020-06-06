from opensfm import types
import numpy as np


def mat_to_pose(mat):
    pose = types.Pose()
    pose.set_rotation_matrix(mat[0:3, 0:3])
    pose.translation = mat[0:3, 3]
    return pose


def pose_to_mat(pose):
    return np.vstack((pose.get_Rt(), np.array([0, 0, 0, 1])))


def in_image(point, width, height):
    if width > height:
        factor = height / width
        return point[0] >= -0.5 and point[0] <= 0.5 and \
            point[1] >= factor * -0.5 and point[1] <= factor * 0.5
    # height >= width
    factor = width / height
    return point[1] >= -0.5 and point[1] <= 0.5 and \
        point[0] >= factor * -0.5 and point[0] <= factor * 0.5

