import copy

import cv2
import numpy as np
from opensfm import pygeometry, pymap, types
# pyre-fixme[21]: Could not find name `special_ortho_group` in `scipy.stats`.
from scipy.stats import special_ortho_group


def test_reconstruction_class_initialization():

    # Instantiate Reconstruction
    reconstruction = types.Reconstruction()
    focal = 0.9722222222222222
    k1 = 0.006094395128698237
    k2 = -0.0004952058188617129
    # Instantiate camera instrinsics
    camera = pygeometry.Camera.create_perspective(focal, k1, k2)
    camera.id = "apple iphone 4s back camera 4.28mm f/2.4"
    camera.height = 2448
    camera.width = 3264
    reconstruction.add_camera(camera)

    # Instantiate GPS data
    metadata = pymap.ShotMeasurements()
    metadata.orientation.value = 1
    metadata.capture_time.value = 0.0
    metadata.gps_accuracy.value = 5.0
    metadata.gps_position.value = [
        1.0815875281451939,
        -0.96510451436708888,
        1.2042133903991235,
    ]
    metadata.accelerometer.value = [0.1, 0.9, 0.0]
    metadata.compass_angle.value = 270.0
    metadata.compass_accuracy.value = 15.0
    metadata.sequence_key.value = "a_sequence_key"

    # Instantiate shots
    pose0 = pygeometry.Pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    shot0 = reconstruction.create_shot("0", camera.id, pose0)
    shot0.metadata = metadata

    pose1 = pygeometry.Pose([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0])
    shot1 = reconstruction.create_shot("1", camera.id, pose1)
    shot1.metadata = metadata

    # TEST
    assert len(reconstruction.cameras) == 1
    assert len(reconstruction.shots) == 2
    assert len(reconstruction.points) == 0
    assert reconstruction.get_camera(camera.id) is not None

    assert reconstruction.get_shot(shot0.id) is not None
    assert reconstruction.get_shot(shot1.id) is not None


def test_is_panorama():
    """Test spherical projection--backprojection loop."""
    assert pygeometry.Camera.is_panorama("spherical")
    assert pygeometry.Camera.is_panorama("equirectangular")
    assert not pygeometry.Camera.is_panorama("fisheye")


def test_camera_deepcopy():
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam2 = copy.deepcopy(cam1)
    assert cam1.focal == cam2.focal
    cam2.focal = 0.7
    assert cam1.focal != cam2.focal
    cam3 = copy.deepcopy(cam2)
    assert cam3.focal == cam2.focal


def test_shot_measurement():
    m = pymap.ShotMeasurementInt()
    assert not m.has_value
    m.value = 4
    assert m.has_value
    assert m.value == 4


def _helper_pose_equal_to_T(pose, T_cw):
    assert np.allclose(pose.get_R_world_to_cam(), T_cw[0:3, 0:3])
    assert np.allclose(pose.get_t_world_to_cam(), T_cw[0:3, 3].reshape(3))
    assert np.allclose(pose.translation, T_cw[0:3, 3].reshape(3))
    # compute the min rotation
    r_cw = cv2.Rodrigues(T_cw[0:3, 0:3])[0].flatten()
    assert np.allclose(pose.rotation, r_cw)
    assert np.allclose(pose.get_R_world_to_cam_min(), r_cw)

    T_wc = np.linalg.inv(T_cw)
    assert np.allclose(pose.get_R_cam_to_world(), T_wc[0:3, 0:3])
    assert np.allclose(pose.get_t_cam_to_world(), T_wc[0:3, 3].reshape(3))
    assert np.allclose(pose.get_origin(), T_wc[0:3, 3].reshape(3))
    assert np.allclose(pose.get_R_cam_to_world_min(), -r_cw)
    assert np.allclose(pose.get_Rt(), T_cw[0:3, 0:4])


def _helper_poses_equal_py_cpp(py_pose, cpp_pose):
    assert np.allclose(py_pose.translation, cpp_pose.translation)
    assert np.allclose(py_pose.rotation, cpp_pose.rotation)
    assert np.allclose(py_pose.get_rotation_matrix(), cpp_pose.get_rotation_matrix())
    assert np.allclose(py_pose.get_origin(), cpp_pose.get_origin())


def _heper_poses_equal(pose1, pose2):
    assert np.allclose(pose1.translation, pose2.translation)
    assert np.allclose(pose1.rotation, pose2.rotation)
    assert np.allclose(pose1.get_rotation_matrix(), pose2.get_rotation_matrix())
    assert np.allclose(pose1.get_origin(), pose2.get_origin())
    assert np.allclose(pose1.get_R_cam_to_world(), pose2.get_R_cam_to_world())
    assert np.allclose(pose1.get_R_world_to_cam(), pose2.get_R_world_to_cam())
    assert np.allclose(pose1.get_t_cam_to_world(), pose2.get_t_cam_to_world())
    assert np.allclose(pose1.get_t_world_to_cam(), pose2.get_t_world_to_cam())
    assert np.allclose(pose1.get_world_to_cam(), pose2.get_world_to_cam())
    assert np.allclose(pose1.get_cam_to_world(), pose2.get_cam_to_world())
    assert np.allclose(pose1.get_Rt(), pose2.get_Rt())


def test_pose_setter():
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
    T_wc = np.linalg.inv(T_cw)
    r_cw = cv2.Rodrigues(R_cw)[0].flatten()
    r_wc = -r_cw

    # set world to cam
    p1 = pygeometry.Pose()
    p1.set_from_world_to_cam(T_cw)
    _helper_pose_equal_to_T(p1, T_cw)

    p2 = pygeometry.Pose()
    p2.set_from_world_to_cam(R_cw, t_cw)
    _helper_pose_equal_to_T(p2, T_cw)

    p3 = pygeometry.Pose()
    p3.set_from_world_to_cam(r_cw, t_cw)
    _helper_pose_equal_to_T(p3, T_cw)

    # set cam to world
    p4 = pygeometry.Pose()
    p4.set_from_cam_to_world(T_wc)
    _helper_pose_equal_to_T(p4, T_cw)

    p5 = pygeometry.Pose()
    p5.set_from_cam_to_world(T_wc[0:3, 0:3], T_wc[0:3, 3])
    _helper_pose_equal_to_T(p5, T_cw)

    p6 = pygeometry.Pose()
    p6.set_from_cam_to_world(r_wc, T_wc[0:3, 3])
    _helper_pose_equal_to_T(p6, T_cw)

    # set rotation, translation
    p7 = pygeometry.Pose()
    p7.rotation = r_cw
    p7.translation = t_cw
    _helper_pose_equal_to_T(p7, T_cw)

    p8 = pygeometry.Pose()
    p8.set_rotation_matrix(R_cw)
    p8.translation = t_cw
    _helper_pose_equal_to_T(p7, T_cw)


def test_pose_transform():
    pt = np.random.rand(3)
    pts = np.random.rand(10, 3)
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
    T_wc = np.linalg.inv(T_cw)
    p = pygeometry.Pose(R_cw, t_cw)
    p_inv = pygeometry.Pose(T_wc[0:3, 0:3], T_wc[0:3, 3])
    # Test via transform and inverse transform
    assert np.allclose(p_inv.transform_many(p.transform_many(pts)), pts)
    assert np.allclose(p_inv.transform(p.transform(pt)), pt)
    assert np.allclose(p.transform(p.transform_inverse(pt)), pt)
    assert np.allclose(p.transform_many(p.transform_inverse_many(pts)), pts)


def test_pose_init():
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
    pose = pygeometry.Pose(R_cw, t_cw)
    _helper_pose_equal_to_T(pose, T_cw)

    r_cw = cv2.Rodrigues(T_cw[0:3, 0:3])[0].flatten()
    pose2 = pygeometry.Pose(r_cw, t_cw)
    _helper_pose_equal_to_T(pose2, T_cw)
    _heper_poses_equal(pose, pose2)

    # Test default init
    pose3 = pygeometry.Pose()
    _helper_pose_equal_to_T(pose3, np.eye(4))
    pose4 = pygeometry.Pose(T_cw[0:3, 0:3])
    _helper_pose_equal_to_T(
        pose4,
        np.vstack(
            (
                np.column_stack((T_cw[0:3, 0:3], np.zeros((3, 1)))),
                np.array([0, 0, 0, 1]),
            )
        ),
    )
    pose5 = pygeometry.Pose(r_cw)
    _helper_pose_equal_to_T(
        pose5,
        np.vstack(
            (
                np.column_stack((T_cw[0:3, 0:3], np.zeros((3, 1)))),
                np.array([0, 0, 0, 1]),
            )
        ),
    )


def test_pose_inverse():
    R_cw = special_ortho_group.rvs(3)
    t_cw = np.random.rand(3)
    T_cw = np.vstack((np.column_stack((R_cw, t_cw)), np.array([0, 0, 0, 1])))
    T_wc = np.linalg.inv(T_cw)
    pose = pygeometry.Pose(T_cw[0:3, 0:3], T_cw[0:3, 3])
    pose_inv = pose.inverse()
    pose_inv2 = pygeometry.Pose(T_wc[0:3, 0:3], T_wc[0:3, 3])
    _heper_poses_equal(pose_inv, pose_inv2)


def test_pixel_to_normalized_conversion():
    cam = pygeometry.Camera.create_perspective(1, 0, 0)
    width, height = 400, 150
    cam.width, cam.height = width, height
    px_coord = np.array([50, 300])
    norm_coord_comp = cam.pixel_to_normalized_coordinates(px_coord)
    norm_coord_static = pygeometry.Camera.pixel_to_normalized_coordinates_common(
        px_coord, width, height
    )
    norm_coord_gt = px_coord - np.array([(width - 1.0) / 2.0, (height - 1.0) / 2.0])
    norm_coord_gt /= max(width, height)
    assert np.allclose(norm_coord_comp, norm_coord_gt)
    assert np.allclose(norm_coord_static, norm_coord_gt)

    px_coord_comp1 = cam.normalized_to_pixel_coordinates(norm_coord_comp)
    px_coord_comp2 = pygeometry.Camera.normalized_to_pixel_coordinates_common(
        norm_coord_comp, width, height
    )
    assert np.allclose(px_coord, px_coord_comp1)
    assert np.allclose(px_coord, px_coord_comp2)
