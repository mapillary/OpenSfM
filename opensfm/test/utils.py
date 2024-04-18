# Test utils for python

# pyre-unsafe
import numpy as np
from opensfm import pygeo, pygeometry, pymap


def assert_cameras_equal(cam1: pygeometry.Camera, cam2: pygeometry.Camera) -> None:
    assert np.allclose(cam1.get_parameters_values(), cam2.get_parameters_values())
    assert cam1.projection_type == cam2.projection_type
    assert cam1.width == cam2.width
    assert cam1.height == cam2.height
    assert cam1.id == cam2.id


def assert_metadata_equal(
    m1: pymap.ShotMeasurements,
    m2: pymap.ShotMeasurements,
    test_mapillary_specific: bool = True,
) -> None:
    assert m1.capture_time.has_value == m2.capture_time.has_value
    if m1.capture_time.has_value:
        assert np.allclose(m1.capture_time.value, m2.capture_time.value, 1e-1)

    assert m1.gps_position.has_value == m2.gps_position.has_value
    if m1.gps_position.has_value:
        assert np.allclose(m1.gps_position.value, m2.gps_position.value)

    assert m1.gps_accuracy.has_value == m2.gps_accuracy.has_value
    if m1.gps_accuracy.has_value:
        assert m1.gps_accuracy.value == m2.gps_accuracy.value

    assert m1.orientation.has_value == m2.orientation.has_value
    if m1.orientation.has_value:
        assert m1.orientation.value == m2.orientation.value

    if test_mapillary_specific:
        assert m1.gravity_down.has_value == m2.gravity_down.has_value
        if m1.gravity_down.has_value:
            assert np.allclose(m1.gravity_down.value, m2.gravity_down.value)

        assert m1.compass_accuracy.has_value == m2.compass_accuracy.has_value
        if m1.compass_accuracy.has_value:
            assert m1.compass_accuracy.value == m2.compass_accuracy.value

        assert m1.compass_angle.has_value == m2.compass_angle.has_value
        if m1.compass_angle.has_value:
            assert m1.compass_angle.value == m2.compass_angle.value

        assert m1.sequence_key.has_value == m2.sequence_key.has_value
        if m1.sequence_key.has_value:
            assert m1.sequence_key.value == m2.sequence_key.value


def assert_landmarks_equal(lm1: pymap.Landmark, lm2: pymap.Landmark) -> None:
    assert lm1.id == lm2.id
    assert np.allclose(lm1.coordinates, lm2.coordinates, 1e-5)


def assert_shots_equal(
    shot1: pymap.Shot, shot2: pymap.Shot, test_mapillary_specific: bool = True
) -> None:
    assert shot1.id == shot2.id
    assert np.allclose(shot1.pose.get_Rt(), shot2.pose.get_Rt(), 1e-5)
    assert shot1.merge_cc == shot2.merge_cc
    assert np.allclose(shot1.covariance, shot2.covariance)
    assert_metadata_equal(shot1.metadata, shot2.metadata, test_mapillary_specific)


def assert_bias_equal(
    bias1: pygeometry.Similarity, bias2: pygeometry.Similarity
) -> None:
    assert bias1.scale == bias2.scale
    assert np.allclose(bias1.rotation, bias2.rotation, 1e-5)
    assert np.allclose(bias1.translation, bias2.translation, 1e-5)


def assert_topo_equal(
    topo1: pygeo.TopocentricConverter, topo2: pygeo.TopocentricConverter
) -> None:
    eps = 1e-8
    assert np.allclose(topo1.lat, topo2.lat, eps)
    assert np.allclose(topo1.lon, topo2.lon, eps)
    assert np.allclose(topo1.alt, topo2.alt, eps)


def assert_maps_equal(
    map1: pymap.Map,
    map2: pymap.Map,
    test_cameras_ids: bool = True,
    test_mapillary_specific: bool = True,
) -> None:
    # In some scenarios (MDC), camera IDs are lost during conversion
    if test_cameras_ids:
        # Cameras are different objects of same value
        for k in map1.get_cameras():
            cam, cam_cpy = map1.get_cameras()[k], map2.get_cameras()[k]
            assert cam != cam_cpy
            assert_cameras_equal(cam, cam_cpy)

        # Biases are different objects of same value
        for b in map1.get_biases():
            bias, bias_cpy = map1.get_biases()[b], map2.get_biases()[b]
            assert bias != bias_cpy
            assert_bias_equal(bias, bias_cpy)

    # Shots are different objects of same value
    for shot_id in map1.get_shots():
        shot1, shot2 = map1.get_shots()[shot_id], map2.get_shots()[shot_id]
        assert shot1 is not shot2
        assert_shots_equal(shot1, shot2, test_mapillary_specific)

    # Pano shots are different objects of same value
    for shot_id in map1.get_pano_shots():
        shot1, shot2 = map1.get_pano_shots()[shot_id], map2.get_pano_shots()[shot_id]
        assert shot1 is not shot2
        assert_shots_equal(shot1, shot2, test_mapillary_specific)

    # Points are different objects of same value
    for pt_id in map1.get_landmarks():
        pt, pt_cpy = map1.get_landmarks()[pt_id], map2.get_landmarks()[pt_id]
        assert pt != pt_cpy
        assert pt.id == pt_cpy.id
        assert np.allclose(pt.coordinates, pt_cpy.coordinates)
        assert np.allclose(pt.color, pt_cpy.color)
        obs = pt.get_observations()
        obs_cpy = pt_cpy.get_observations()
        assert len(obs) == len(obs_cpy)

        # Observations are different objects of same value
        for shot, obs_id in obs.items():
            obs1 = shot.get_observation(obs_id)
            shot_cpy = map2.get_shots()[shot.id]
            obs_cpy = shot_cpy.get_observation(obs_id)
            assert obs1 is not obs_cpy

    # Topocentric reference are different objects of same value
    toporef, toporef_cpy = map1.get_reference(), map2.get_reference()
    assert toporef != toporef_cpy
    assert_topo_equal(toporef, toporef_cpy)
