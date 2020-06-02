import numpy as np
import scipy
from opensfm import pymap
from opensfm import pysfm
from opensfm import pygeometry
from opensfm import types
from opensfm import reconstruction

from scipy.stats import special_ortho_group
from scipy.spatial.transform import Rotation


def test_pose():
    pose = pymap.Pose()
    # Test default
    assert np.allclose(pose.get_cam_to_world(), np.eye(4), 1e-10)
    assert np.allclose(pose.get_world_to_cam(), np.eye(4), 1e-10)

    # Test setters and getters for translation
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
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
    T_wc = np.vstack((np.column_stack((R_wc, t_wc)), np.array([0, 0, 0, 1])))
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

    p1 = pymap.Pose()
    # Check identity pose
    p1.set_from_world_to_cam(np.array([0, 0, 0]), np.array([0, 0, 0]))
    assert np.allclose(p1.get_rotation_matrix(), np.eye(3))
    assert np.allclose(p1.get_R_world_to_cam_min(), np.zeros((1, 3)))
    assert np.allclose(p1.get_R_cam_to_world_min(), np.zeros((1, 3)))
    assert np.allclose(p1.get_cam_to_world(), np.eye(4))
    assert np.allclose(p1.get_world_to_cam(), np.eye(4))

    R_wc = special_ortho_group.rvs(3)
    t_wc = np.random.rand(3)
    pose = pymap.Pose()
    r_wc = Rotation.from_dcm(R_wc).as_rotvec()  # same as cv2.Rodrigues
    pose.set_from_cam_to_world(R_wc, t_wc)
    # print(pose.get_R_cam_to_world_min(), r_wc)
    assert np.allclose(pose.get_R_cam_to_world_min(), r_wc)
    pose.set_from_world_to_cam(R_wc, t_wc)
    assert np.allclose(pose.get_R_world_to_cam_min(), r_wc)

    R_wc = special_ortho_group.rvs(3)
    t_wc = np.random.rand(3)
    pose = pymap.Pose()
    r_wc = Rotation.from_dcm(R_wc).as_rotvec()  # same as cv2.Rodrigues
    pose.set_from_world_to_cam(r_wc, t_wc)
    assert np.allclose(R_wc, pose.get_R_world_to_cam())
    assert np.allclose(pose.get_R_world_to_cam_min(), r_wc)
    assert np.allclose(pose.get_R_cam_to_world_min(), -r_wc)
    assert np.allclose(R_wc.transpose(), pose.get_R_cam_to_world())

    R_wc = special_ortho_group.rvs(3)
    pose = pymap.Pose()
    pose.set_rotation_matrix(R_wc)
    assert np.allclose(R_wc, pose.get_R_world_to_cam())
    r_wc = Rotation.from_dcm(R_wc).as_rotvec()
    assert np.allclose(r_wc, pose.get_R_world_to_cam_min())

    R_wc = special_ortho_group.rvs(3)
    pose = pymap.Pose()
    r_wc = Rotation.from_dcm(R_wc).as_rotvec()
    pose.rotation = r_wc

    assert np.allclose(R_wc, pose.get_R_world_to_cam())
    assert np.allclose(r_wc, pose.get_R_world_to_cam_min())
    r_cw = Rotation.from_dcm(R_wc.transpose()).as_rotvec()
    assert np.allclose(r_cw, pose.get_R_cam_to_world_min())

    # Test again all the setters and getters!
    R_wc = special_ortho_group.rvs(3)
    r_wc = Rotation.from_dcm(R_wc).as_rotvec()
    t_wc = np.random.rand(3)
    pose = pymap.Pose()
    pose.set_from_cam_to_world(r_wc, t_wc)
    assert np.allclose(pose.get_R_cam_to_world(), R_wc)
    assert np.allclose(pose.get_R_world_to_cam(), R_wc.transpose())
    assert np.allclose(pose.get_R_cam_to_world_min(), r_wc)
    assert np.allclose(pose.get_t_cam_to_world(), t_wc)

    pose = pymap.Pose()
    pose.set_from_cam_to_world(R_wc, t_wc)
    assert np.allclose(pose.get_R_cam_to_world(), R_wc)
    assert np.allclose(pose.get_R_world_to_cam(), R_wc.transpose())
    assert np.allclose(pose.get_R_cam_to_world_min(), r_wc)
    assert np.allclose(pose.get_t_cam_to_world(), t_wc)

    pose = pymap.Pose()
    T_wc = np.vstack((np.column_stack((R_wc, t_wc)), np.array([0, 0, 0, 1])))
    pose.set_from_cam_to_world(T_wc)
    assert np.allclose(pose.get_R_cam_to_world(), R_wc)
    assert np.allclose(pose.get_R_world_to_cam(), R_wc.transpose())
    assert np.allclose(pose.get_R_cam_to_world_min(), r_wc)
    assert np.allclose(pose.get_t_cam_to_world(), t_wc)
    assert np.allclose(pose.get_cam_to_world(), T_wc)

    # Test again
    R_cw = special_ortho_group.rvs(3)
    r_cw = Rotation.from_dcm(R_cw).as_rotvec()
    t_cw = np.random.rand(3)
    pose = pymap.Pose()
    pose.set_from_world_to_cam(r_cw, t_cw)
    assert np.allclose(pose.get_R_cam_to_world(), R_cw.transpose())
    assert np.allclose(pose.get_R_world_to_cam(), R_cw)
    assert np.allclose(pose.get_R_cam_to_world_min(), -r_cw)
    assert np.allclose(pose.get_R_world_to_cam_min(), r_cw)
    assert np.allclose(pose.get_t_world_to_cam(), t_cw)

    pose = pymap.Pose()
    pose.set_from_cam_to_world(R_cw, t_cw)
    assert np.allclose(pose.get_R_cam_to_world(), R_cw)
    assert np.allclose(pose.get_R_world_to_cam(), R_cw.transpose())
    assert np.allclose(pose.get_R_cam_to_world_min(), r_cw)
    assert np.allclose(pose.get_R_world_to_cam_min(), -r_cw)
    assert np.allclose(pose.get_t_cam_to_world(), t_cw)

    pose = pymap.Pose()
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
    pose.set_from_cam_to_world(T_cw)
    assert np.allclose(pose.get_R_cam_to_world(), R_cw)
    assert np.allclose(pose.get_R_world_to_cam(), R_cw.transpose())
    assert np.allclose(pose.get_R_cam_to_world_min(), r_cw)
    assert np.allclose(pose.get_R_world_to_cam_min(), -r_cw)
    assert np.allclose(pose.get_t_cam_to_world(), t_cw)
    assert np.allclose(pose.get_cam_to_world(), T_cw)


def test_camera():
    m = pymap.Map()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    cam2 = pygeometry.Camera.create_perspective(1, 0, 0)
    cam2.id = "cam2"
    map_cam1 = m.create_camera(cam1)
    map_cam2 = m.create_camera(cam2)
    assert cam1.focal == map_cam1.focal
    assert cam2.focal == map_cam2.focal
    assert map_cam1.k1 == cam1.k1
    assert map_cam1.k2 == cam1.k2
    assert cam1.id == map_cam1.id and map_cam1.id == "cam1"
    assert cam2.id == map_cam2.id and map_cam2.id == "cam2"

    assert map_cam1 == m.get_camera("cam1")
    assert map_cam2 == m.get_camera("cam2")
    assert len(m.get_cameras()) == 2
    assert m.number_of_cameras() == 2
    cams = set([map_cam1, map_cam2])
    for cam in m.get_cameras():
        assert cam in cams


def test_shot_sfm():
    m = pymap.Map()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    map_cam1 = m.create_camera(cam1)
    cam1.id = "cam2"
    map_cam2 = m.create_camera(cam1)
    map_shot1 = m.create_shot("shot1", "cam1")
    map_shot2 = m.create_shot("shot2", "cam1")
    # check that already existing shot is returned
    assert map_shot1 == m.create_shot("shot1", "cam1")
    # check that already existing shot is returned even with different camera
    assert map_shot1 == m.create_shot("shot1", "cam2")
    # test getters
    assert map_shot1 == m.get_shot(
        "shot1") and map_shot2 == m.get_shot("shot2")
    assert m.get_shot("ab") == None
    assert m.number_of_cameras() == 2 and m.number_of_shots() == 2
    assert map_shot1.unique_id == 0 and map_shot2.unique_id == 1
    assert map_shot1.camera.id == "cam1" and map_shot1.get_camera_name() == "cam1"
    assert map_shot2.camera.id == "cam1" and map_shot2.get_camera_name() == "cam1"
    # test delete
    m.remove_shot("shot1")
    m.remove_shot("shot1")
    assert m.get_shot("shot1") == None
    assert m.number_of_shots() == 1
    map_shot3 = m.create_shot("shot1", "cam2")
    assert m.number_of_shots() == 2
    assert map_shot3 == m.get_shot("shot1")
    assert map_shot3.camera.id == "cam2" and map_shot3.get_camera_name() == "cam2"
    assert map_shot3.unique_id == 2
    m.remove_shot("shot1")
    m.remove_shot("shot2")
    assert m.number_of_shots() == 0

    for n in range(5):
        m.create_shot("shot" + str(n), "cam1")
    for n in range(5, 10):
        m.create_shot("shot" + str(n), "cam2")
    assert m.number_of_shots() == 10
    # try to create them again
    for n in range(5):
        m.create_shot("shot" + str(n), "cam1")
    for n in range(5, 10):
        m.create_shot("shot" + str(n), "cam2")
    assert m.number_of_shots() == 10
    # now remove all
    for n in range(5):
        m.remove_shot("shot" + str(n))
    for n in range(5, 10):
        m.remove_shot("shot" + str(n))
    assert m.number_of_shots() == 0


def test_shot_slam():
    pass


def test_points():
    m = pymap.Map()
    n_landmarks = 20
    for n in range(n_landmarks):
        pos = np.random.rand(3)
        lm = m.create_landmark(str(n), pos)
        assert lm == m.create_landmark(str(n), pos)
        assert np.allclose(pos, lm.get_global_pos())
        assert np.allclose(pos, lm.coordinates)
        assert lm.unique_id == n
        assert lm.id == str(n)
        assert m.has_landmark(str(n))

    assert m.number_of_landmarks() == n_landmarks
    for n in range(n_landmarks):
        m.remove_landmark(str(n))
    assert m.number_of_landmarks() == 0
    m2 = pymap.Map()
    for n in range(n_landmarks):
        pos = np.random.rand(3)
        lm = m2.create_landmark(str(n), pos)
        assert lm == m2.create_landmark(str(n), pos)
        assert np.allclose(pos, lm.get_global_pos())
        assert np.allclose(pos, lm.coordinates)
        assert lm.unique_id == n
        assert lm.id == str(n)
        assert lm == m2.get_landmark(str(n))
        color = (np.random.rand(3) * 255).astype(int)
        lm.color = color
        assert np.allclose(lm.color, color)
        assert lm.has_observations() == 0
        new_pos = np.random.rand(3)
        lm.coordinates = new_pos
        assert np.allclose(new_pos, lm.get_global_pos())
        assert np.allclose(new_pos, lm.coordinates)
        new_pos = np.random.rand(3)
        lm.set_global_pos(new_pos)
        assert np.allclose(new_pos, lm.get_global_pos())
        assert np.allclose(new_pos, lm.coordinates)
        # reprojection errors
        reproj_errors = dict(
            {"shot1": np.random.rand(2), "shot2": np.random.rand(2)})
        lm.reprojection_errors = reproj_errors
        errors = lm.reprojection_errors
        for k in reproj_errors.keys():
            assert np.allclose(errors[k], reproj_errors[k])

        for k in reproj_errors.keys():
            lm.remove_reprojection_error(k)
        errors = lm.reprojection_errors
        assert len(errors) == 0


def test_map():
    m = pymap.Map()
    n_cams = 2
    n_shots = 10
    n_landmarks = 1000
    # create the cameras
    for cam_id in range(n_cams):
        cam = pygeometry.Camera.create_perspective(0.5, 0, 0)
        cam.id = "cam" + str(cam_id)
        m.create_camera(cam)

    for shot_id in range(n_shots):
        m.create_shot(str(shot_id), "cam" +
                      str(int(np.random.rand(1) * 10 % n_cams)))

    for point_id in range(n_landmarks):
        m.create_landmark(str(point_id), np.random.rand(3))

    assert m.number_of_landmarks() == n_landmarks
    assert m.number_of_cameras() == n_cams
    assert m.number_of_shots() == n_shots

    n_total_obs = 0
    # Now establish random connections (observations) between shots and points
    for lm in m.get_all_landmarks().values():
        n_obs = 0
        for shot in m.get_all_shots().values():
            # create a new observation
            obs = pysfm.Observation(100, 200, 0.5, 255, 0, 0, int(lm.id))
            m.add_observation(shot, lm, obs)
            n_obs += 1
            n_total_obs += 1
            assert lm.is_observed_in_shot(shot)
        if n_obs > 0:
            assert lm.has_observations()
        else:
            assert not lm.has_observations()
        assert lm.number_of_observations() == n_obs

    for lm in m.get_all_landmarks().values():
        n_total_obs -= lm.number_of_observations()
    assert n_total_obs == 0

    # remove the observations for the first landmarks from all the shots
    for lm in m.get_all_landmarks().values():
        for shot_id in range(int(n_shots / 2)):
            m.remove_observation(str(shot_id), lm.id)
        assert lm.number_of_observations() == int(n_shots / 2)
    n_total_obs = 0
    for shot in m.get_all_shots().values():
        n_total_obs += shot.compute_num_valid_pts(1)
    assert n_total_obs == int((n_shots * n_landmarks) / 2)
    m.clear_observations_and_landmarks()
    n_total_obs = 0
    for shot in m.get_all_shots().values():
        n_total_obs += shot.compute_num_valid_pts(1)

    assert m.number_of_landmarks() == 0 and n_total_obs == 0
