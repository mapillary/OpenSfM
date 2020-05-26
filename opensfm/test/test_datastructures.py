import numpy as np
import scipy
import cv2
from opensfm import pymap
from opensfm import types
from opensfm import reconstruction
from opensfm import pysfm
from scipy.stats import special_ortho_group

def test_pose():
    pose = pymap.Pose()
    # Test default
    assert np.allclose(pose.get_cam_to_world(), np.eye(4), 1e-10)
    assert np.allclose(pose.get_world_to_cam(), np.eye(4), 1e-10)

    # Test setters and getters for translation
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)),np.array([0,0,0,1])))
    pose = pymap.Pose()
    pose.set_from_world_to_cam(T_cw)
    assert np.allclose(pose.get_world_to_cam(), T_cw)
    assert np.allclose(pose.get_cam_to_world(), np.linalg.inv(T_cw))
    assert np.allclose(pose.get_origin(), pose.get_t_cam_to_world())

    pose2 = pymap.Pose()
    pose2.set_from_world_to_cam(R_cw, t_cw)
    assert np.allclose(pose2.get_world_to_cam(), T_cw)
    assert np.allclose(pose2.get_cam_to_world(), np.linalg.inv(T_cw))
    assert np.allclose(pose2.get_origin(), pose.get_t_cam_to_world())

    # test the rotation matrix
    assert np.allclose(pose.get_rotation_matrix(), R_cw)
    assert np.allclose(pose.get_R_world_to_cam(), R_cw)
    assert np.allclose(pose.get_R_cam_to_world(), R_cw.transpose())
    
    pose = pymap.Pose()
    R_wc = special_ortho_group.rvs(3)
    t_wc = np.random.rand(3)
    T_wc = np.vstack((np.column_stack((R_wc, t_wc)),np.array([0,0,0,1])))
    pose.set_from_cam_to_world(T_wc)
    assert np.allclose(pose.get_cam_to_world(), T_wc)
    assert np.allclose(pose.get_world_to_cam(), np.linalg.inv(T_wc))
    assert np.allclose(pose.get_R_cam_to_world(), R_wc)
    assert np.allclose(pose.get_R_world_to_cam(), R_wc.transpose())
    assert np.allclose(pose.get_t_cam_to_world(), t_wc)
    assert np.allclose(pose.get_t_world_to_cam(), -R_wc.transpose().dot(t_wc))

    pose2 = pymap.Pose()
    pose2.set_from_cam_to_world(R_wc, t_wc)
    assert np.allclose(pose2.get_cam_to_world(), T_wc)
    assert np.allclose(pose2.get_world_to_cam(), np.linalg.inv(T_wc))
    assert np.allclose(pose2.get_origin(), pose.get_t_cam_to_world())

    pose.translation = t_cw
    assert np.allclose(pose.translation, t_cw)
    assert np.allclose(pose.get_t_world_to_cam(), t_cw)

def test_pose_minimal_representation():
    
    R_wc = special_ortho_group.rvs(3)
    t_wc = np.random.rand(3)
    pose = pymap.Pose()
    r_wc = cv2.Rodrigues(R_wc)[0]
    pose.set_from_cam_to_world(R_wc, t_wc)
    # print(pose.get_R_cam_to_world_min(), r_wc)
    assert np.allclose(pose.get_R_cam_to_world_min().reshape((3,1)), r_wc)
    pose.set_from_world_to_cam(R_wc, t_wc)
    assert np.allclose(pose.get_R_world_to_cam_min().reshape((3,1)), r_wc)

    
    R_wc = special_ortho_group.rvs(3)
    t_wc = np.random.rand(3)
    pose = pymap.Pose()
    r_wc = cv2.Rodrigues(R_wc)[0]
    pose.set_from_world_to_cam(r_wc, t_wc)
    assert np.allclose(R_wc, pose.get_R_cam_to_world())
    assert np.allclose(pose.get_R_cam_to_world_min().reshape((3,1)), r_wc)
    assert np.allclose(R_wc.transpose(), pose.get_R_world_to_cam())

    R_wc = special_ortho_group.rvs(3)
    pose = pymap.Pose()
    pose.set_rotation_matrix(R_wc)
    assert np.allclose(R_wc, pose.get_R_world_to_cam())
    r_wc = cv2.Rodrigues(R_wc)[0]
    assert np.allclose(r_wc, pose.get_R_world_to_cam_min().reshape((3,1)))
    
    R_wc = special_ortho_group.rvs(3)
    pose = pymap.Pose()
    r_wc = cv2.Rodrigues(R_wc)[0]
    pose.rotation = r_wc
    assert np.allclose(R_wc, pose.get_R_world_to_cam())
    assert np.allclose(r_wc, pose.get_R_world_to_cam_min().reshape((3,1)))
    r_cw = cv2.Rodrigues(R_wc.transpose())[0]
    assert np.allclose(r_cw, pose.get_R_cam_to_world_min().reshape((3,1)))

def test_shot_sfm():
    pass

def test_shot_slam():
    pass

def test_points():
    pass

def test_map():
    pass
    