import numpy as np
import copy
import random
import pytest
from opensfm import pymap
from opensfm import pysfm
from opensfm import pygeometry
from opensfm import types


def _create_reconstruction(n_cameras=0,
                           n_shots_cam={},
                           n_pano_shots_cam={},
                           n_points=0,
                           dist_to_shots=False,
                           dist_to_pano_shots=False):
    """ Creates a reconstruction with n_cameras random cameras and
        shots, where n_shots_cam is a dictionary, containing the 
        camera_id and the number of shots.

        Example:
        shot_cams = {"0": 50, "1": 30}
        _create_reconstruction(2, shot_cams)

        Will create a reconstruction with two cameras and 80 shots,
        50 are associated with cam "0" and 30 with cam "1".

        n_points_in_shots is the number of points to create.
        If dist_to_shots, then observations are created and randomly
        distributed to all shots. We pick with the repeat option, thus
        if we have three shots the distribution could be
        something like: [1,2,2], [0,1,2]. We avoid things like [3,3,3]
        """
    rec = types.Reconstruction()
    if n_cameras > 0:
        for i in range(n_cameras):
            # cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
            focal, k1, k2 = np.random.rand(3)
            cam = pygeometry.Camera.create_perspective(focal, k1, k2)
            cam.id = str(i)
            rec.add_camera(cam)

        shot_id = 0
        for cam_id, n_shots in n_shots_cam.items():
            for _ in range(n_shots):
                rec.create_shot(str(shot_id), cam_id)
                shot_id += 1

        shot_id = 0
        for cam_id, n_shots in n_pano_shots_cam.items():
            for _ in range(n_shots):
                rec.create_pano_shot(str(shot_id), cam_id)
                shot_id += 1

    if n_points > 0:
        for i in range(n_points):
            rec.create_point(str(i), np.random.rand(3))

        if dist_to_shots:
            n_shots = len(rec.shots)
            for pt in rec.points.values():
                choice = set(np.random.choice(n_shots, n_shots))
                if len(choice) > 1:
                    for ch in choice:
                        # create a new observation
                        obs = pysfm.Observation(100, 200, 0.5, 255, 0, 0, int(pt.id))
                        shot = rec.shots[str(ch)]
                        rec.add_observation(shot, pt, obs)
        # TODO: If required, we have to do the same for pano shots
    return rec


"""
Camera Tests
"""


def _helper_compare_persp_cameras(cam1, cam2):
    assert cam1.focal == cam2.focal
    assert cam1.id == cam2.id
    assert cam1.k1 == cam2.k1
    assert cam1.k2 == cam2.k2


def test_create_cameras():
    n_cameras = 100
    rec = types.Reconstruction()

    for cam_id in range(0, n_cameras):
        focal, k1, k2 = np.random.rand(3)
        cam = pygeometry.Camera.create_perspective(focal, k1, k2)
        cam.id = str(cam_id)
        # create the camera within the reconstruction
        map_cam = rec.add_camera(cam)
        _helper_compare_persp_cameras(cam, map_cam)
        # Check that the cameras are different
        assert cam is not map_cam
        # Check the getters
        assert map_cam is rec.get_camera(str(cam_id))
        assert map_cam is rec.cameras[str(cam_id)]

    assert rec.map.number_of_cameras() == n_cameras
    assert len(rec.cameras) == n_cameras


def test_camera_iterators():
    n_cameras = 100
    rec = _create_reconstruction(n_cameras)

    # Key iterator
    visited_cams = set()
    for cam_id in rec.cameras:
        visited_cams.add(cam_id)
    assert len(visited_cams) == n_cameras

    for idx in range(0, n_cameras):
        assert str(idx) in visited_cams

    # value iterator
    visited_cams = set()
    for cam in rec.cameras.values():
        visited_cams.add(cam.id)
        focal = np.random.rand(1)
        cam.focal = focal
        assert rec.cameras[cam.id].focal == focal
        assert cam is rec.cameras[cam.id]

    assert len(visited_cams) == n_cameras

    # item iterator
    for idx in range(0, n_cameras):
        assert str(idx) in visited_cams

    for cam_id, cam in rec.cameras.items():
        assert cam_id == cam.id
        focal = np.random.rand(1)
        cam.focal = focal
        assert rec.cameras[cam.id].focal == focal
        assert cam is rec.cameras[cam.id]


def _check_common_cam_properties(cam1, cam2):
    assert cam1.id == cam2.id
    assert cam1.width == cam2.width
    assert cam1.height == cam2.height
    assert cam1.projection_type == cam2.projection_type


def test_brown_camera():
    rec = types.Reconstruction()
    focal_x = 0.6
    focal_y = 0.7
    c_x = 0.1
    c_y = -0.05
    k1 = -0.1
    k2 = 0.01
    p1 = 0.001
    p2 = 0.002
    k3 = 0.01
    cam_cpp = pygeometry.Camera.create_brown(
        focal_x, focal_y / focal_x,
        [c_x, c_y],
        [k1, k2, k3, p1, p2])
    cam_cpp.width = 800
    cam_cpp.height = 600
    cam_cpp.id = "cam"
    c = rec.add_camera(cam_cpp)
    _check_common_cam_properties(cam_cpp, c)

    # The specific parameters
    assert cam_cpp.k1 == c.k1 and cam_cpp.k2 == c.k2 and cam_cpp.k3 == c.k3
    assert cam_cpp.p2 == c.p2 and cam_cpp.p1 == c.p1
    assert np.allclose(cam_cpp.principal_point, c.principal_point)
    assert np.allclose(cam_cpp.distortion, c.distortion)
    assert cam_cpp.focal == c.focal
    assert cam_cpp.aspect_ratio == c.aspect_ratio


def test_fisheye_camera():
    rec = types.Reconstruction()
    focal = 0.6
    k1 = -0.1
    k2 = 0.01
    cam_cpp = pygeometry.Camera.create_fisheye(focal, k1, k2)
    cam_cpp.width = 800
    cam_cpp.height = 600
    cam_cpp.id = "cam"
    c = rec.add_camera(cam_cpp)
    _check_common_cam_properties(cam_cpp, c)

    # The specific parameters
    assert cam_cpp.k1 == c.k1 and cam_cpp.k2 == c.k2
    assert np.allclose(cam_cpp.distortion, c.distortion)
    assert cam_cpp.focal == c.focal


def test_dual_camera():
    rec = types.Reconstruction()
    focal = 0.6
    k1 = -0.1
    k2 = 0.01
    transition = 0.5
    cam_cpp = pygeometry.Camera.create_dual(transition, focal, k1, k2)
    cam_cpp.width = 800
    cam_cpp.height = 600
    cam_cpp.id = "cam"
    c = rec.add_camera(cam_cpp)
    _check_common_cam_properties(cam_cpp, c)

    # The specific parameters
    assert cam_cpp.k1 == c.k1 and cam_cpp.k2 == c.k2
    assert np.allclose(cam_cpp.distortion, c.distortion)
    assert cam_cpp.focal == c.focal
    assert cam_cpp.transition == c.transition


def test_perspective_camera():
    rec = types.Reconstruction()
    focal = 0.6
    k1 = -0.1
    k2 = 0.01
    cam_cpp = pygeometry.Camera.create_perspective(focal, k1, k2)
    cam_cpp.width = 800
    cam_cpp.height = 600
    cam_cpp.id = "cam"
    c = rec.add_camera(cam_cpp)
    _check_common_cam_properties(cam_cpp, c)

    # The specific parameters
    assert cam_cpp.k1 == c.k1 and cam_cpp.k2 == c.k2
    assert np.allclose(cam_cpp.distortion, c.distortion)
    assert cam_cpp.focal == c.focal


def test_spherical_camera():
    rec = types.Reconstruction()
    cam_cpp = pygeometry.Camera.create_spherical()
    cam_cpp.width = 800
    cam_cpp.height = 600
    cam_cpp.id = "cam"
    c = rec.add_camera(cam_cpp)
    _check_common_cam_properties(cam_cpp, c)


# Test Metadata


def _helper_metadata_equal(m1, m2):
    # Asserts if both are correct and if the exist
    assert m1.capture_time.has_value == m2.capture_time.has_value
    if m1.capture_time.has_value:
        assert m1.capture_time.value == m2.capture_time.value

    assert m1.gps_position.has_value == m2.gps_position.has_value
    if m1.gps_position.has_value:
        assert np.allclose(m1.gps_position.value, m2.gps_position.value)

    assert m1.gps_accuracy.has_value == m2.gps_accuracy.has_value
    if m1.gps_accuracy.has_value:
        assert m1.gps_accuracy.value == m2.gps_accuracy.value

    assert m1.compass_accuracy.has_value == m2.compass_accuracy.has_value
    if m1.compass_accuracy.has_value:
        assert m1.compass_accuracy.value == m2.compass_accuracy.value

    assert m1.compass_angle.has_value == m2.compass_angle.has_value
    if m1.compass_angle.has_value:
        assert m1.compass_angle.value == m2.compass_angle.value

    assert m1.accelerometer.has_value == m2.accelerometer.has_value
    if m1.accelerometer.has_value:
        assert np.allclose(m1.accelerometer.value, m2.accelerometer.value)

    assert m1.orientation.has_value == m2.orientation.has_value
    if m1.orientation.has_value:
        assert m1.orientation.value == m2.orientation.value

    assert m1.sequence_key.has_value == m2.sequence_key.has_value
    if m1.sequence_key.has_value:
        assert m1.sequence_key.value == m2.sequence_key.value


def _help_measurement_test(measurement, attr, val):
    # Test metadata's has_value properties
    assert getattr(measurement, attr).has_value is False
    getattr(measurement, attr).value = val
    if np.shape(val) == ():  # just a value
        assert getattr(measurement, attr).value == val
    else:
        assert np.allclose(getattr(measurement, attr).value, val)
    # Test metadata's has_value properties!
    assert getattr(measurement, attr).has_value is True
    # Test reset
    getattr(measurement, attr).reset()
    assert getattr(measurement, attr).has_value is False


def test_shot_measurement_setter_and_getter():
    m1 = pymap.ShotMeasurements()
    # Test basic functionality
    _help_measurement_test(m1, 'capture_time', np.random.rand(1))
    _help_measurement_test(m1, 'gps_position', np.random.rand(3))
    _help_measurement_test(m1, 'gps_accuracy', np.random.rand(1))
    _help_measurement_test(m1, 'compass_accuracy', np.random.rand(1))
    _help_measurement_test(m1, 'compass_angle', np.random.rand(1))
    _help_measurement_test(m1, 'accelerometer', np.random.rand(3))
    _help_measurement_test(m1, 'orientation', random.randint(0, 100))
    _help_measurement_test(m1, 'sequence_key', "key_test")


def _helper_populate_metadata(m):
    m.capture_time.value = np.random.rand(1)
    m.gps_position.value = np.random.rand(3)
    m.gps_accuracy.value = np.random.rand(1)
    m.compass_accuracy.value = np.random.rand(1)
    m.compass_angle.value = np.random.rand(1)
    m.accelerometer.value = np.random.rand(3)
    m.orientation.value = random.randint(0, 100)
    m.sequence_key.value = "sequence_key"


def test_shot_measurement_set():
    m1 = pymap.ShotMeasurements()
    _helper_populate_metadata(m1)
    m2 = pymap.ShotMeasurements()
    # Test setting metadata with other metadata
    m2.set(m1)
    # Check that m2 has the same values as m1
    _helper_metadata_equal(m1, m2)
    m3 = pymap.ShotMeasurements()
    m1.set(m3)
    # Now m1 should be completely reset
    _helper_metadata_equal(m1, m3)


def test_shot_add():
    rec = _create_reconstruction(2)
    shot1 = rec.create_shot("shot0", "0")
    assert shot1.id == "shot0"
    assert shot1.camera.id == "0"
    n_shots = 10
    # Test repeated add and with (non)-existing cameras
    for i in range(n_shots):
        assert shot1 == rec.create_shot("shot0", "0")
        assert shot1 == rec.create_shot("shot0", "1")
    assert len(rec.shots) == 1

    # add new shots
    for i in range(1, n_shots):
        shot1 = rec.create_shot("shot" + str(i), "0")
        assert shot1.unique_id == i
    assert len(rec.shots) == n_shots
    assert rec.map.number_of_shots() == n_shots


def test_shot_delete():
    n_shots = 10
    rec = _create_reconstruction(1, {"0": n_shots})
    # delete non-existing shot
    rec.remove_shot("abcde")
    assert len(rec.shots) == n_shots
    del_shots = np.random.choice(n_shots, int(n_shots / 2), replace=False)
    for i in del_shots:
        rec.remove_shot(str(i))
    assert len(rec.shots) == n_shots - len(del_shots)


def test_shot_get():
    rec = _create_reconstruction(1)
    shot_id = "shot0"
    shot1 = rec.create_shot(shot_id, "0")
    assert shot1 is rec.shots[shot_id]
    assert shot1 is rec.get_shot(shot_id)


def test_pano_shot_get():
    rec = _create_reconstruction(1)
    shot_id = "shot0"
    shot1 = rec.create_pano_shot(shot_id, "0")
    assert shot1 is rec.pano_shots[shot_id]
    assert shot1 is rec.get_pano_shot(shot_id)


def test_pano_shot_add():
    rec = _create_reconstruction(2)
    shot1 = rec.create_pano_shot("shot0", "0")
    assert shot1.id == "shot0"
    assert shot1.camera.id == "0"
    n_shots = 10
    # Test repeated add and with (non)-existing cameras
    for i in range(n_shots):
        assert shot1 == rec.create_pano_shot("shot0", "0")
        assert shot1 == rec.create_pano_shot("shot0", "1")
    assert len(rec.pano_shots) == 1

    # add new shots
    for i in range(1, n_shots):
        shot1 = rec.create_pano_shot("shot" + str(i), "0")
        assert shot1.unique_id == i
    assert len(rec.pano_shots) == n_shots
    assert rec.map.number_of_pano_shots() == n_shots


def test_pano_shot_delete():
    n_shots = 10
    rec = _create_reconstruction(1, n_pano_shots_cam={"0": n_shots})
    # delete non-existing shot
    rec.remove_pano_shot("abcde")
    assert len(rec.pano_shots) == n_shots
    del_shots = np.random.choice(n_shots, int(n_shots / 2), replace=False)
    for i in del_shots:
        rec.remove_pano_shot(str(i))
    assert len(rec.pano_shots) == n_shots - len(del_shots)


def test_shot_attributes():
    rec = _create_reconstruction(1, {"0": 2})
    map_shot1 = rec.shots["0"]
    map_shot2 = rec.shots["1"]
    # test attributes
    map_shot1.merge_cc = 10
    assert map_shot1.merge_cc == 10
    map_shot1.covariance = np.diag([1, 2, 3])
    assert np.allclose(map_shot1.covariance, np.diag([1, 2, 3]))
    map_shot2.covariance = map_shot1.covariance
    # Check that they are different objects
    assert map_shot2.covariance is not map_shot1.covariance
    assert np.allclose(map_shot2.covariance, np.diag([1, 2, 3]))
    map_shot2.covariance = np.diag([2, 2, 2])
    assert np.allclose(map_shot2.covariance, np.diag([1, 2, 3])) is False
    map_shot1.merge_cc = 100
    assert map_shot1.merge_cc == 100


def test_shot_add_remove_add():
    n_shots = 10
    rec = _create_reconstruction(1, {"0": n_shots})
    rec.remove_shot("0")
    assert len(rec.shots) == n_shots - 1
    rec.create_shot("0", "0")
    assert len(rec.shots) == n_shots


def test_pano_shot_add_remove_add():
    n_shots = 10
    rec = _create_reconstruction(1, n_pano_shots_cam={"0": n_shots})
    rec.remove_pano_shot("0")
    assert len(rec.pano_shots) == n_shots - 1
    rec.create_pano_shot("0", "0")
    assert len(rec.pano_shots) == n_shots


def _helper_shots_equal(shot1, shot2):
    assert shot1.id == shot2.id
    assert shot1.merge_cc == shot2.merge_cc
    assert shot1.camera.id == shot2.camera.id
    assert np.allclose(shot1.covariance, shot2.covariance)
    _helper_metadata_equal(shot1.metadata, shot2.metadata)


def test_add_shot_from_shot():
    n_shots = 5
    rec = _create_reconstruction(1, n_shots_cam={"0": n_shots})
    shot1 = rec.shots["0"]
    _helper_populate_metadata(shot1.metadata)
    rec_new = types.Reconstruction()
    rec_new.add_shot(rec.shots["0"])
    rec_new.add_shot(rec.shots["1"])
    assert len(rec_new.shots) == 2
    rec_new.add_shot(rec.shots["1"])
    assert len(rec_new.shots) == 2
    for k in rec_new.shots.keys():
        _helper_shots_equal(rec.shots[k], rec_new.shots[k])


def test_shot_metadata_assign():
    rec = _create_reconstruction(1, n_shots_cam={"0": 2})
    shot1 = rec.shots["0"]
    shot2 = rec.shots["1"]
    _helper_populate_metadata(shot1.metadata)
    assert shot1.metadata is not shot2.metadata
    shot2.metadata = shot1.metadata
    assert shot1.metadata is not shot2.metadata
    _helper_metadata_equal(shot1.metadata, shot2.metadata)


def test_add_pano_shot_from_pano_shot():
    n_shots = 5
    rec = _create_reconstruction(1, n_pano_shots_cam={"0": n_shots})
    shot1 = rec.pano_shots["0"]
    _helper_populate_metadata(shot1.metadata)
    rec_new = types.Reconstruction()
    rec_new.add_pano_shot(rec.pano_shots["0"])
    rec_new.add_pano_shot(rec.pano_shots["1"])
    assert len(rec_new.pano_shots) == 2
    rec_new.add_shot(rec.pano_shots["1"])
    assert len(rec_new.pano_shots) == 2
    for k in rec_new.pano_shots.keys():
        _helper_shots_equal(rec.pano_shots[k], rec_new.pano_shots[k])


def test_single_point_add():
    rec = types.Reconstruction()
    pt = rec.create_point("0")
    assert pt == rec.points["0"] and pt == rec.get_point("0")
    
    assert len(rec.points) == 1 and rec.map.number_of_landmarks() == 1
    assert pt.id == "0"
    assert pt.unique_id == 0
    assert np.allclose(pt.coordinates, np.zeros(3))
    coord = np.random.rand(3)
    pt.coordinates = coord
    assert np.allclose(pt.coordinates, coord)
    assert np.allclose(pt.get_global_pos(), coord)
    assert np.allclose(pt.color, [255, 0, 0])
    color = np.random.randint((255, 255, 255))
    pt.color = color
    assert np.allclose(pt.color, color)


def test_point_add_from_point():
    rec = types.Reconstruction()
    rec2 = types.Reconstruction()
    coord2 = np.random.rand(3)
    pt2 = rec2.create_point("1", coord2)
    pt2_1 = rec.add_point(pt2)
    assert len(rec.points) == 1 and rec.map.number_of_landmarks() == 1
    assert pt2 is not pt2_1
    assert "1" == pt2_1.id
    assert pt2_1.unique_id == 0
    assert pt2_1 == rec.points["1"]
    assert np.allclose(pt2_1.coordinates, coord2)


def test_point_reproj_errors():
    rec = _create_reconstruction(n_points=1)
    pt = rec.points["0"]
    reproj_errors = dict(
        {"shot1": np.random.rand(2), "shot2": np.random.rand(2)})
    pt.reprojection_errors = reproj_errors
    errors = pt.reprojection_errors
    for k in reproj_errors.keys():
        assert np.allclose(errors[k], reproj_errors[k])

    for k in reproj_errors.keys():
        pt.remove_reprojection_error(k)
    errors = pt.reprojection_errors
    assert len(errors) == 0


def test_point_delete():
    n_points = 100
    rec = _create_reconstruction(n_points=n_points)
    # Delete non-existing point
    rec.remove_point("abcdef")

    assert len(rec.points) == n_points
    del_list = list(rec.points.keys())
    # Delete all points
    for k in del_list:
        rec.remove_point(k)
    assert len(rec.points) == 0

    # Try another deletion method
    rec = _create_reconstruction(n_points=n_points)
    assert len(rec.points) == n_points
    rec.points = {}
    assert len(rec.points) == 0


def test_single_observation():
    rec = _create_reconstruction(1, n_shots_cam={"0": 1}, n_points=1)
    # Create one camera, one shot and one point
    # create a new observation
    obs = pysfm.Observation(100, 200, 0.5, 255, 0, 0, 100)
    rec.add_observation("0", "0", obs)
    shot = rec.shots["0"]
    pt = rec.points["0"]
    assert pt.has_observations()
    observations = pt.get_observations()
    assert len(observations) == 1
    assert pt.number_of_observations() == 1
    obs = shot.get_landmark_observation(pt)
    assert obs is not None
    rec.remove_observation(shot.id, pt.id)
    assert pt.has_observations() is False
    observations = pt.get_observations()
    assert len(observations) == 0
    assert pt.number_of_observations() == 0


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


def test_camera_deepcopy():
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam2 = copy.deepcopy(cam1)
    assert cam1.focal == cam2.focal
    cam2.focal = 0.7
    assert cam1.focal != cam2.focal
    cam3 = copy.deepcopy(cam2)
    assert cam3.focal == cam2.focal


def test_observation_shot_removal():
    """Remove a shot and all its corresponding observations"""
    rec = _create_reconstruction(n_cameras=2,
                                 n_shots_cam={"0": 1, "1": 1},
                                 n_points=200, dist_to_shots=True)
    rec.remove_shot("0")
    for p in rec.points.values():
        assert len(p.get_observations()) <= 1
    rec.remove_shot("1")
    for p in rec.points.values():
        assert len(p.get_observations()) == 0


def test_observation_point_removal():
    """Remove a point and all its corresponding observations"""
    rec = _create_reconstruction(n_cameras=2,
                                 n_shots_cam={"0": 50, "1": 40},
                                 n_pano_shots_cam={"0": 20, "1": 30},
                                 n_points=200, dist_to_shots=True)
    pt_list = list(rec.points.keys())
    for pt_id in pt_list:
        p = rec.points[pt_id]
        obs = p.get_observations()
        n_obs = p.number_of_observations()
        assert len(obs) == n_obs
        shots = []
        for shot in obs:
            shots.append((shot, shot.compute_num_valid_pts()))
        rec.remove_point(pt_id)
        for shot, n_shots in shots:
            # Check that the point really was deleted
            assert n_shots - 1 == shot.compute_num_valid_pts(1)


def test_rec_deepcopy():
    rec = _create_reconstruction(n_cameras=2,
                                 n_shots_cam={"0": 50, "1": 40},
                                 n_pano_shots_cam={"0": 20, "1": 30},
                                 n_points=200, dist_to_shots=True)
    for shot in rec.shots.values():
        _helper_populate_metadata(shot.metadata)
    for shot in rec.pano_shots.values():
        _helper_populate_metadata(shot.metadata)

    rec2 = copy.deepcopy(rec, {"copy_observations": True})
    assert len(rec2.cameras) == 2
    assert len(rec2.shots) == 90
    assert len(rec2.pano_shots) == 50
    assert len(rec2.points) == 200
    for k in rec.cameras:
        cam, cam_cpy = rec.cameras[k], rec2.cameras[k]
        assert cam != cam_cpy
        _helper_compare_persp_cameras(cam, cam_cpy)

    # Check shots
    for shot_id in rec2.shots.keys():
        shot1, shot2 = rec.shots[shot_id], rec2.shots[shot_id]
        assert shot1 is not shot2
        _helper_shots_equal(shot1, shot2)

    # Check pano shots
    for shot_id in rec2.pano_shots.keys():
        shot1, shot2 = rec.pano_shots[shot_id], rec2.pano_shots[shot_id]
        assert shot1 is not shot2
        _helper_shots_equal(shot1, shot2)

    # Check points
    for pt_id in rec2.points:
        pt, pt_cpy = rec.points[pt_id], rec2.points[pt_id]
        assert pt != pt_cpy
        assert pt.id == pt_cpy.id
        assert np.allclose(pt.coordinates, pt_cpy.coordinates)
        assert np.allclose(pt.color, pt_cpy.color)
        obs = pt.get_observations()
        obs_cpy = pt_cpy.get_observations()
        assert len(obs) == len(obs_cpy)
        # Now check the observations
        for shot, obs_id in obs.items():
            obs1 = shot.get_observation(obs_id)
            shot_cpy = rec2.shots[shot.id]
            obs_cpy = shot_cpy.get_observation(obs_id)
            assert obs1 is not obs_cpy


def test_gcp():
    gcp = []
    for i in range(0, 10):
        p = pymap.GroundControlPoint()
        p.id = 'p' + str(i)
        o1 = pymap.GroundControlPointObservation()
        o1.shot_id = 'p1'
        o2 = pymap.GroundControlPointObservation()
        o2.shot_id = 'p2'
        obs = [o1, o2]
        p.observations = obs
        gcp.append(p)
        assert(p.observations[0].shot_id == "p1")
        assert(p.observations[1].shot_id == 'p2')
        p.add_observation(o2)
        p.add_observation(o2)
        assert len(p.observations) == 4
    for pt in gcp:
        assert(pt.observations[0].shot_id == "p1")
        assert(pt.observations[1].shot_id == 'p2')
