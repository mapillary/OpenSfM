import numpy as np
import copy
import random
from opensfm import pymap
from opensfm import pysfm
from opensfm import pygeometry
from opensfm import types


def test_camera():
    rec = types.Reconstruction()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    cam2 = pygeometry.Camera.create_perspective(1, 0, 0)
    cam2.id = "cam2"
    map_cam1 = rec.add_camera(cam1)
    map_cam2 = rec.add_camera(cam2)
    assert cam1.focal == map_cam1.focal
    assert cam2.focal == map_cam2.focal
    assert map_cam1.k1 == cam1.k1
    assert map_cam1.k2 == cam1.k2
    assert cam1.id == map_cam1.id and map_cam1.id == "cam1"
    assert cam2.id == map_cam2.id and map_cam2.id == "cam2"

    assert map_cam1 == rec.get_camera("cam1")
    assert map_cam2 == rec.get_camera("cam2")
    assert rec.map.number_of_cameras() == 2
    assert len(rec.cameras) == 2
    cams = set([map_cam1, map_cam2])
    for id, cam in rec.cameras.items():
        assert cam in cams
    for id, cam in rec.cameras.items():
        cam.focal = 0.5
        assert cam.focal == rec.cameras[cam.id].focal

    for cam in rec.cameras.values():
        assert cam in cams

    # TODO ADD TEST to set camera parameters from the iterators
    for cam in rec.cameras.values():
        cam.focal = 0.5
        assert cam.focal == rec.cameras[cam.id].focal


def test_shot_sfm():
    m = pymap.Map()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    map_cam1 = m.create_camera(cam1)
    cam1.id = "cam2"
    map_cam2 = m.create_camera(cam1)
    map_shot1 = m.create_shot("shot1", "cam1")
    map_shot2 = m.create_shot("shot2", "cam1")

    # test attributes
    map_shot1.merge_cc = 0
    assert map_shot1.merge_cc == 0
    map_shot1.covariance = np.diag([1, 2, 3])
    assert np.allclose(map_shot1.covariance, np.diag([1, 2, 3]))
    map_shot2.covariance = map_shot1.covariance
    assert np.allclose(map_shot2.covariance, np.diag([1, 2, 3]))
    map_shot2.covariance = np.diag([2, 2, 2])
    assert np.allclose(map_shot2.covariance, np.diag([1, 2, 3])) is False
    map_shot1.merge_cc = 100
    assert map_shot1.merge_cc == 100

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



def test_pano_shot_sfm():
    m = pymap.Map()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    map_cam1 = m.create_camera(cam1)
    cam1.id = "cam2"
    map_cam2 = m.create_camera(cam1)
    map_shot1 = m.create_pano_shot("shot1", "cam1")
    map_shot2 = m.create_pano_shot("shot2", "cam1")
    assert m.number_of_pano_shots() == 2
    # check that already existing shot is returned
    assert map_shot1 == m.create_pano_shot("shot1", "cam1")
    # check that already existing shot is returned even with different camera
    assert map_shot1 == m.create_pano_shot("shot1", "cam2")
    # test getters
    assert map_shot1 == m.get_pano_shot("shot1")
    assert map_shot2 == m.get_pano_shot("shot2")
    assert m.get_pano_shot("ab") is None
    assert m.number_of_cameras() == 2 and m.number_of_pano_shots() == 2
    assert map_shot1.unique_id == 0 and map_shot2.unique_id == 1
    assert map_shot1.camera.id == "cam1"
    assert map_shot1.get_camera_name() == "cam1"
    assert map_shot2.camera.id == "cam1"
    assert map_shot2.get_camera_name() == "cam1"
    # test delete
    m.remove_pano_shot("shot1")
    m.remove_pano_shot("shot1")
    assert m.get_pano_shot("shot1") is None
    assert m.number_of_pano_shots() == 1
    map_shot3 = m.create_pano_shot("shot1", "cam2")
    assert m.number_of_pano_shots() == 2
    assert map_shot3 == m.get_pano_shot("shot1")
    assert map_shot3.camera.id == "cam2"
    assert map_shot3.get_camera_name() == "cam2"
    assert map_shot3.unique_id == 2
    m.remove_pano_shot("shot1")
    m.remove_pano_shot("shot2")
    assert m.number_of_pano_shots() == 0

    for n in range(5):
        m.create_pano_shot("shot" + str(n), "cam1")
    for n in range(5, 10):
        m.create_pano_shot("shot" + str(n), "cam2")
    assert m.number_of_pano_shots() == 10
    # try to create them again
    for n in range(5):
        m.create_pano_shot("shot" + str(n), "cam1")
    for n in range(5, 10):
        m.create_pano_shot("shot" + str(n), "cam2")
    assert m.number_of_pano_shots() == 10
    # now remove all
    for n in range(5):
        m.remove_pano_shot("shot" + str(n))
    for n in range(5, 10):
        m.remove_pano_shot("shot" + str(n))
    assert m.number_of_pano_shots() == 0


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


def test_metadata():
    m = pymap.ShotMeasurements()
    # Test basic functionality
    _help_measurement_test(m, 'capture_time', 126)
    _help_measurement_test(m, 'gps_position', np.array([1, 2, 3]))
    _help_measurement_test(m, 'gps_accuracy', 34)
    _help_measurement_test(m, 'compass_accuracy', 89)
    _help_measurement_test(m, 'compass_angle', 100)
    _help_measurement_test(m, 'accelerometer', np.array([4, 5, 6]))
    _help_measurement_test(m, 'orientation', 5)
    _help_measurement_test(m, 'sequence_key', "key_test")

    # Test setting metadata with other metadata
    m2 = pymap.ShotMeasurements()
    m2.capture_time.value = 8910
    m2.gps_accuracy.value = 5
    m1 = pymap.ShotMeasurements()
    m1.set(m2)
    assert m1.capture_time.value == m2.capture_time.value
    m1.capture_time.value = 518
    # Test if two different objects
    assert m1.capture_time != m2.capture_time
    assert m1.capture_time.value != m2.capture_time.value
    assert m1.gps_accuracy.value == m2.gps_accuracy.value


def test_metadata_with_shot():
    # Set up the shots
    rec = types.Reconstruction()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    cam2 = pygeometry.Camera.create_perspective(1, 0, 0)
    cam2.id = "cam2"
    rec.add_camera(cam1)
    rec.add_camera(cam2)
    shot1 = rec.create_shot('1', "cam1")
    shot2 = rec.create_shot('2', "cam2")
    shot1.metadata.capture_time.value = 10
    assert shot1.metadata.capture_time.value == 10
    # Test assignment
    shot2.metadata = shot1.metadata
    assert shot1.metadata.capture_time.has_value
    assert shot2.metadata.capture_time.has_value
    assert shot1.metadata.capture_time.value == shot2.metadata.capture_time.value
    shot1.metadata.capture_time.value = 518

    # Test if two different objects
    assert shot1.metadata.capture_time != shot2.metadata.capture_time
    assert shot1.metadata.capture_time.value != shot2.metadata.capture_time.value
    assert shot1.metadata.capture_time.value == 518
    assert shot2.metadata.capture_time.value == 10


def test_cam_iterator():
    rec = types.Reconstruction()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    cam2 = pygeometry.Camera.create_perspective(1, 0, 0)
    cam2.id = "cam2"
    rec.add_camera(cam1)
    rec.add_camera(cam2)

    for cam in rec.cameras.values():
        foc = random.random()
        cam.focal = foc
        assert(cam.focal == rec.cameras[cam.id].focal)
        assert(cam.focal == foc)
        assert foc == rec.cameras[cam.id].focal

    for k, cam in rec.cameras.items():
        foc = random.random()
        cam.focal = foc
        assert(cam.focal == rec.cameras[cam.id].focal)
        assert(cam.focal == foc)
        assert foc == rec.cameras[cam.id].focal

    for k in rec.cameras.keys():
        foc = random.random()
        cam = rec.cameras[k]
        cam.focal = foc
        assert(cam.focal == rec.cameras[k].focal)
        assert(cam.focal == foc)
        assert foc == rec.cameras[k].focal


def test_copy():
    pose = pygeometry.Pose()
    p2 = copy.copy(pose)
    p3 = copy.deepcopy(pose)

    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam2 = copy.deepcopy(cam1)
    assert cam1.focal == cam2.focal
    cam2.focal = 0.7
    assert cam1.focal != cam2.focal
    cam3 = copy.deepcopy(cam2)
    assert cam3.focal == cam2.focal


def test_add_shot():
    rec = types.Reconstruction()
    cam = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam.id = "cam1"
    rec.add_camera(cam)
    shot1 = rec.create_shot("shot1", cam.id)
    shot2 = rec.create_shot("shot2", cam.id)

    rec_copy = types.Reconstruction()
    shot1_cpy = rec_copy.add_shot(shot1)

    assert shot1 != shot1_cpy
    assert shot1.id == shot1_cpy.id

    dir(shot1.metadata)
    meta = shot1.metadata
    meta_cpy = shot1_cpy.metadata
    for a in dir(meta):
        if not callable(getattr(meta, a))\
                and not a.startswith("__"):
            assert getattr(meta, a).has_value is False
            assert getattr(meta_cpy, a).has_value is False

    # Do something with shot2
    shot2.scale = 10
    shot2.merge_cc = 123
    cov = np.random.rand(3, 3)
    shot2.covariance = cov

    # Set the metadata
    meta2 = shot2.metadata
    meta2.capture_time.value = 15
    meta2.gps_position.value = np.array([1, 2, 3])
    meta2.gps_accuracy.value = 0.3
    meta2.compass_accuracy.value = 0.7
    meta2.compass_angle.value = 0.15
    meta2.accelerometer.value = np.array([5, 6, 7])
    meta2.orientation.value = 1
    meta2.sequence_key.value = "skey2"

    shot2_cpy = rec_copy.add_shot(shot2)
    assert shot2_cpy.scale == 10
    assert shot2_cpy.merge_cc == 123
    assert np.allclose(shot2_cpy.covariance, cov)
    assert shot2 != shot2_cpy
    assert shot2.id == shot2_cpy.id

    meta2_cpy = shot2_cpy.metadata
    assert shot2.metadata != shot2_cpy.metadata
    # Now assert the metadata
    assert meta2_cpy.capture_time.value == 15
    assert np.allclose(meta2_cpy.gps_position.value, np.array([1, 2, 3]))
    assert meta2_cpy.gps_accuracy.value == 0.3
    assert meta2_cpy.compass_accuracy.value == 0.7
    assert meta2_cpy.compass_angle.value == 0.15
    assert np.allclose(meta2_cpy.accelerometer.value, np.array([5, 6, 7]))
    assert meta2_cpy.orientation.value == 1
    assert meta2_cpy.sequence_key.value == "skey2"


def test_add_pano_shot():
    rec = types.Reconstruction()
    cam = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam.id = "cam1"
    rec.add_camera(cam)
    shot1 = rec.create_pano_shot("shot1", cam.id)
    shot2 = rec.create_pano_shot("shot2", cam.id)

    rec_copy = types.Reconstruction()
    shot1_cpy = rec_copy.add_pano_shot(shot1)

    assert shot1 != shot1_cpy
    assert shot1.id == shot1_cpy.id

    dir(shot1.metadata)
    meta = shot1.metadata
    meta_cpy = shot1_cpy.metadata
    for a in dir(meta):
        if not callable(getattr(meta, a))\
                and not a.startswith("__"):
            assert getattr(meta, a).has_value is False
            assert getattr(meta_cpy, a).has_value is False

    # Do something with shot2
    shot2.scale = 10
    shot2.merge_cc = 123
    cov = np.random.rand(3, 3)
    shot2.covariance = cov

    # Set the metadata
    meta2 = shot2.metadata
    meta2.capture_time.value = 15
    meta2.gps_position.value = np.array([1, 2, 3])
    meta2.gps_accuracy.value = 0.3
    meta2.compass_accuracy.value = 0.7
    meta2.compass_angle.value = 0.15
    meta2.accelerometer.value = np.array([5, 6, 7])
    meta2.orientation.value = 1
    meta2.sequence_key.value = "skey2"

    shot2_cpy = rec_copy.add_pano_shot(shot2)
    assert shot2_cpy.scale == 10
    assert shot2_cpy.merge_cc == 123
    assert np.allclose(shot2_cpy.covariance, cov)
    assert shot2 != shot2_cpy
    assert shot2.id == shot2_cpy.id

    meta2_cpy = shot2_cpy.metadata
    assert shot2.metadata != shot2_cpy.metadata
    # Now assert the metadata
    assert meta2_cpy.capture_time.value == 15
    assert np.allclose(meta2_cpy.gps_position.value, np.array([1, 2, 3]))
    assert meta2_cpy.gps_accuracy.value == 0.3
    assert meta2_cpy.compass_accuracy.value == 0.7
    assert meta2_cpy.compass_angle.value == 0.15
    assert np.allclose(meta2_cpy.accelerometer.value, np.array([5, 6, 7]))
    assert meta2_cpy.orientation.value == 1
    assert meta2_cpy.sequence_key.value == "skey2"


def __helper_metadata_cmp(m1, m2):
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


def test_rec_deepcopy():
    rec = types.Reconstruction()
    cam1 = pygeometry.Camera.create_perspective(0.5, 0, 0)
    cam1.id = "cam1"
    cam2 = pygeometry.Camera.create_perspective(1, 0, 0)
    cam2.id = "cam2"
    rec.add_camera(cam1)
    rec.add_camera(cam2)

    # Add a few shots
    shot1 = rec.create_shot(
        "s1", cam1.id, pygeometry.Pose(np.eye(3), [1, 2, 3]))
    shot2 = rec.create_shot(
        "s2", cam2.id, pygeometry.Pose(np.eye(3), [4, 5, 6]))

    # Add points
    n_points = 10
    for pid in range(n_points):
        pt = rec.create_point("pt" + str(pid), np.random.rand(3))
        obs = pysfm.Observation(100, 200, 0.5, 255, 0, 0, int(pid))
        rec.add_observation(shot1, pt, obs)
        rec.add_observation(shot2, pt, obs)
        pt.color = np.random.rand(3)

    rec_cpy = copy.deepcopy(rec)
    assert len(rec.cameras) == len(rec_cpy.cameras)
    for k in rec.cameras:
        cam = rec.cameras[k]
        cam_cpy = rec_cpy.cameras[k]
        assert cam != cam_cpy
        assert cam.id == cam_cpy.id
        assert cam.focal == cam_cpy.focal

    assert len(rec.shots) == len(rec_cpy.shots)
    for shot_id in rec.shots:
        shot = rec.shots[shot_id]
        shot_cpy = rec_cpy.shots[shot_id]
        assert shot != shot_cpy
        assert shot.id == shot_cpy.id
        __helper_metadata_cmp(shot.metadata, shot_cpy.metadata)
        # compare the poses
        assert np.allclose(shot.pose.rotation, shot_cpy.pose.rotation)
        assert np.allclose(shot.pose.translation, shot_cpy.pose.translation)

    for ptid in rec.points:
        pt = rec.points[ptid]
        pt_cpy = rec_cpy.points[ptid]
        assert pt != pt_cpy
        assert pt.id == pt_cpy.id
        assert np.allclose(pt.coordinates, pt_cpy.coordinates)
        if pt.color is None:
            assert pt_cpy.color is None
        else:
            assert np.allclose(pt.color, pt_cpy.color)
