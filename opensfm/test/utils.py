# Test utils for python
import numpy as np
from opensfm import pymap, pygeometry, types


def assert_cameras_equal(cam1: pygeometry.Camera, cam2: pygeometry.Camera) -> None:
    assert np.allclose(cam1.get_parameters_values(), cam2.get_parameters_values())
    assert cam1.projection_type == cam2.projection_type
    assert cam1.width == cam2.width
    assert cam1.height == cam2.height
    assert cam1.id == cam2.id


def assert_metadata_equal(
    m1: pymap.ShotMeasurements, m2: pymap.ShotMeasurements
) -> None:
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


def assert_landmarks_equal(lm1: pymap.Landmark, lm2: pymap.Landmark) -> None:
    assert lm1.id == lm2.id
    assert np.allclose(lm1.coordinates, lm2.coordinates, 1e-5)


def assert_shots_equal(shot1: pymap.Shot, shot2: pymap.Shot) -> None:
    assert shot1.id == shot2.id
    assert np.allclose(shot1.pose.get_Rt(), shot2.pose.get_Rt(), 1e-5)
    assert shot1.merge_cc == shot2.merge_cc
    assert shot1.camera.id == shot2.camera.id
    assert np.allclose(shot1.covariance, shot2.covariance)
    assert_metadata_equal(shot1.metadata, shot2.metadata)


def assert_maps_equal(map1: pymap.Map, map2: pymap.Map) -> None:
    # compare number of shots, landmarks
    assert len(map1.get_shots()) == len(map2.get_shots())
    assert len(map1.get_landmarks()) == len(map2.get_landmarks())

    # compare shots
    map2_shots = map2.get_shots()
    for shot_id, shot1 in map2_shots.items():
        assert shot_id in map2_shots
        assert_shots_equal(shot1, map2_shots.get(shot_id))

    # compare landmarks
    map2_lms = map2.get_landmarks()
    for lm_id, lm1 in map2_lms.items():
        assert lm_id in map2_lms
        assert_landmarks_equal(lm1, map2_lms.get(lm_id))

    map2_cams = map2.get_cameras()
    for cam_id, cam1 in map2_cams.items():
        assert cam_id in map2_cams
        assert_cameras_equal(cam1, map2_cams.get(cam_id))


def assert_reconstructions_equal(
    rec1: types.Reconstruction, rec2: types.Reconstruction
) -> None:
    assert_maps_equal(rec1.map, rec2.map)
