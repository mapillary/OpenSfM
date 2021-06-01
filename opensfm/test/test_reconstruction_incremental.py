from opensfm import reconstruction
from opensfm.synthetic_data import synthetic_dataset, synthetic_scene


def test_reconstruction_incremental(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    _, reconstructed_scene = reconstruction.incremental_reconstruction(
        dataset, scene_synthetic.tracks_manager
    )
    errors = synthetic_scene.compare(reference, reconstructed_scene[0])

    assert reconstructed_scene[0].reference.lat == 47.0
    assert reconstructed_scene[0].reference.lon == 6.0

    assert errors["ratio_cameras"] == 1.0
    assert 0.7 < errors["ratio_points"] < 1.0

    assert 0 < errors["aligned_position_rmse"] < 0.02
    assert 0 < errors["aligned_rotation_rmse"] < 0.001
    assert 0 < errors["aligned_points_rmse"] < 0.1

    # Sanity check that GPS error is similar to the generated gps_noise
    assert 4.0 < errors["absolute_gps_rmse"] < 7.0


def test_reconstruction_incremental_rig(
    scene_synthetic_rig: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic_rig.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic_rig.exifs,
        scene_synthetic_rig.features,
        scene_synthetic_rig.tracks_manager,
    )

    dataset.config["align_method"] = "orientation_prior"
    _, reconstructed_scene = reconstruction.incremental_reconstruction(
        dataset, scene_synthetic_rig.tracks_manager
    )
    errors = synthetic_scene.compare(reference, reconstructed_scene[0])

    assert reconstructed_scene[0].reference.lat == 47.0
    assert reconstructed_scene[0].reference.lon == 6.0

    assert errors["ratio_cameras"] == 1.0
    assert 0.7 < errors["ratio_points"] < 1.0

    assert 0 < errors["aligned_position_rmse"] < 0.005
    assert 0 < errors["aligned_rotation_rmse"] < 0.001
    assert 0 < errors["aligned_points_rmse"] < 0.05

    assert 0 < errors["absolute_gps_rmse"] < 0.15
