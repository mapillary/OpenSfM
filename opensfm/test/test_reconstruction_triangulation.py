# pyre-unsafe
from opensfm import reconstruction
from opensfm.synthetic_data import synthetic_dataset, synthetic_scene


def test_reconstruction_triangulation(
    scene_synthetic_triangulation: synthetic_scene.SyntheticInputData,
) -> None:
    reference = scene_synthetic_triangulation.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic_triangulation.exifs,
        scene_synthetic_triangulation.features,
        scene_synthetic_triangulation.tracks_manager,
        scene_synthetic_triangulation.gcps,
    )

    dataset.config["bundle_compensate_gps_bias"] = True
    dataset.config["bundle_use_gcp"] = True
    _, reconstructed_scene = reconstruction.triangulation_reconstruction(
        dataset, scene_synthetic_triangulation.tracks_manager
    )
    errors = synthetic_scene.compare(
        reference,
        scene_synthetic_triangulation.gcps,
        reconstructed_scene[0],
    )

    assert reconstructed_scene[0].reference.lat == 47.0
    assert reconstructed_scene[0].reference.lon == 6.0

    assert errors["ratio_cameras"] == 1.0
    assert 0.7 < errors["ratio_points"] < 1.0

    assert 0 < errors["aligned_position_rmse"] < 0.030
    assert 0 < errors["aligned_rotation_rmse"] < 0.002
    assert 0 < errors["aligned_points_rmse"] < 0.1

    # Sanity check that GPS error is similar to the generated gps_noise
    assert 0.01 < errors["absolute_gps_rmse"] < 0.1

    # Sanity check that GCP error is similar to the generated gcp_noise
    assert 0.01 < errors["absolute_gcp_rmse_horizontal"] < 0.05
    assert 0.005 < errors["absolute_gcp_rmse_vertical"] < 0.04

    # Check that the GPS bias (only translation) is recovered
    translation = reconstructed_scene[0].biases["1"].translation
    assert 9.9 < translation[0] < 10.11
    assert 99.9 < translation[2] < 100.11
