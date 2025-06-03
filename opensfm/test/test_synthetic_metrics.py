# pyre-strict
import numpy as np
from opensfm.synthetic_data import synthetic_metrics, synthetic_scene


def test_change_geo_reference(
    scene_synthetic: synthetic_scene.SyntheticInputData,
) -> None:
    original = scene_synthetic.reconstruction
    lat = original.reference.lat + 0.001  # about 111 m
    lon = original.reference.lon + 0.002
    alt = original.reference.alt + 2.34

    aligned = synthetic_metrics.change_geo_reference(original, lat, lon, alt)

    for shot_id in original.shots:
        original_position = original.shots[shot_id].pose.get_origin()
        aligned_position = aligned.shots[shot_id].pose.get_origin()
        aligned_lla = aligned.reference.to_lla(*aligned_position)
        aligned_in_original = original.reference.to_topocentric(*aligned_lla)
        assert np.allclose(original_position, aligned_in_original, atol=0.01)

        assert original.shots[shot_id].metadata.gps_position.has_value
        assert aligned.shots[shot_id].metadata.gps_position.has_value
        original_gps_prior = original.shots[shot_id].metadata.gps_position.value
        aligned_gps_prior = aligned.shots[shot_id].metadata.gps_position.value
        aligned_gps_lla = aligned.reference.to_lla(*aligned_gps_prior)
        aligned_gps_in_original = original.reference.to_topocentric(*aligned_gps_lla)
        assert np.allclose(original_gps_prior, aligned_gps_in_original, atol=0.01)

    for point_id in original.points:
        original_position = original.points[point_id].coordinates
        aligned_position = aligned.points[point_id].coordinates
        aligned_lla = aligned.reference.to_lla(*aligned_position)
        aligned_in_original = original.reference.to_topocentric(*aligned_lla)
        assert np.allclose(original_position, aligned_in_original, atol=0.01)
