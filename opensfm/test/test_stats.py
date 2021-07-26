from opensfm import stats, types
from opensfm.synthetic_data import synthetic_dataset, synthetic_scene


def test_processing_statistics_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    processing_statistics = stats.processing_statistics(dataset, [reference])

    assert list(processing_statistics.keys()) == ["steps_times", "date", "area"]
    assert processing_statistics["steps_times"] == {
        "Feature Extraction": -1,
        "Features Matching": -1,
        "Tracks Merging": -1,
        "Reconstruction": -1,
        "Total Time": 0,
    }
    assert processing_statistics["date"] == "unknown"
    assert 3500 < processing_statistics["area"] < 3600


def test_processing_statistics_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    processing_statistics = stats.processing_statistics(dataset, [null_scene])

    assert list(processing_statistics.keys()) == ["steps_times", "date", "area"]
    assert processing_statistics["steps_times"] == {
        "Feature Extraction": -1,
        "Features Matching": -1,
        "Tracks Merging": -1,
        "Reconstruction": -1,
        "Total Time": 0,
    }
    assert processing_statistics["date"] == "unknown"
    assert processing_statistics["area"] == -1


def test_features_statistics_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    features_statistics = stats.features_statistics(
        dataset, scene_synthetic.tracks_manager, [reference]
    )
    assert list(features_statistics.keys()) == [
        "detected_features",
        "reconstructed_features",
    ]
    assert (
        features_statistics["detected_features"]
        == features_statistics["reconstructed_features"]
    )
    assert features_statistics["reconstructed_features"] == {
        "min": 303,
        "max": 1065,
        "mean": 841,
        "median": 884,
    }


def test_features_statistics_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    features_statistics = stats.features_statistics(
        dataset, scene_synthetic.tracks_manager, [null_scene]
    )

    assert list(features_statistics.keys()) == [
        "detected_features",
        "reconstructed_features",
    ]
    assert (
        features_statistics["detected_features"]
        == features_statistics["reconstructed_features"]
    )
    assert features_statistics["reconstructed_features"] == {
        "min": -1,
        "max": -1,
        "mean": -1,
        "median": -1,
    }


def test_reconstruction_statistics_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    reconstruction_statistics = stats.reconstruction_statistics(
        dataset, scene_synthetic.tracks_manager, [reference]
    )

    assert reconstruction_statistics["components"] == 1
    assert not reconstruction_statistics["has_gps"]
    assert not reconstruction_statistics["has_gcp"]
    assert 4900 < reconstruction_statistics["initial_points_count"] < 5000
    assert reconstruction_statistics["initial_shots_count"] == 20
    assert 4900 < reconstruction_statistics["reconstructed_points_count"] < 5000
    assert reconstruction_statistics["reconstructed_shots_count"] == 20
    assert 16800 < reconstruction_statistics["observations_count"] < 16900
    assert 3.3 < reconstruction_statistics["average_track_length"] < 3.4
    assert 3.4 < reconstruction_statistics["average_track_length_over_two"] < 3.5
    assert len(reconstruction_statistics["histogram_track_length"]) == 5
    assert 0.15 < reconstruction_statistics["reprojection_error_normalized"] < 0.16
    assert 1.25 < reconstruction_statistics["reprojection_error_pixels"] < 1.28
    assert len(reconstruction_statistics["reprojection_histogram_normalized"][0]) == 30
    assert len(reconstruction_statistics["reprojection_histogram_normalized"][1]) == 31
    assert len(reconstruction_statistics["reprojection_histogram_pixels"][0]) == 30
    assert len(reconstruction_statistics["reprojection_histogram_pixels"][1]) == 31


def test_reconstruction_statistics_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    reconstruction_statistics = stats.reconstruction_statistics(
        dataset, scene_synthetic.tracks_manager, [null_scene]
    )

    assert reconstruction_statistics["components"] == 1
    assert not reconstruction_statistics["has_gps"]
    assert not reconstruction_statistics["has_gcp"]
    assert 4900 < reconstruction_statistics["initial_points_count"] < 5000
    assert reconstruction_statistics["initial_shots_count"] == 0
    assert reconstruction_statistics["reconstructed_points_count"] == 0
    assert reconstruction_statistics["reconstructed_shots_count"] == 0
    assert reconstruction_statistics["observations_count"] == 0
    assert reconstruction_statistics["average_track_length"] == -1
    assert reconstruction_statistics["average_track_length_over_two"] == -1
    assert len(reconstruction_statistics["histogram_track_length"]) == 0
    assert reconstruction_statistics["reprojection_error_normalized"] == -1.0
    assert reconstruction_statistics["reprojection_error_pixels"] == -1.0
    assert len(reconstruction_statistics["reprojection_histogram_normalized"][0]) == 0
    assert len(reconstruction_statistics["reprojection_histogram_normalized"][1]) == 0
    assert len(reconstruction_statistics["reprojection_histogram_pixels"][0]) == 0
    assert len(reconstruction_statistics["reprojection_histogram_pixels"][1]) == 0


def test_cameras_statistics_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    cameras_statistics = stats.cameras_statistics(dataset, [reference])
    assert cameras_statistics == {
        "1": {
            "initial_values": {"k1": -0.1, "k2": 0.01, "focal": 0.7},
            "optimized_values": {"k1": -0.1, "k2": 0.01, "focal": 0.7},
            "bias": {
                "rotation": [-0.0, -0.0, -0.0],
                "scale": 1.0,
                "translation": [0.0, 0.0, 0.0],
            },
        }
    }


def test_cameras_statistics_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    cameras_statistics = stats.cameras_statistics(dataset, [null_scene])
    assert cameras_statistics == {}


def test_rig_statistics_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    rig_statistics = stats.rig_statistics(dataset, [reference])
    assert rig_statistics == {}


def test_rig_statistics_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    cameras_statistics = stats.rig_statistics(dataset, [null_scene])
    assert cameras_statistics == {}


def test_gps_errors_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    gps_errors = stats.gps_errors([reference])
    assert gps_errors == {}


def test_gps_errors_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    gps_errors = stats.gps_errors([null_scene])
    assert gps_errors == {}


def test_gcp_errors_normal(
    scene_synthetic: synthetic_scene.SyntheticInputData,
):
    reference = scene_synthetic.reconstruction
    dataset = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    gcp_errors = stats.gcp_errors(dataset, [reference])
    assert gcp_errors == {}


def test_gcp_errors_null(
    scene_synthetic: synthetic_scene.SyntheticInputData,
    null_scene: types.Reconstruction,
):
    dataset = synthetic_dataset.SyntheticDataSet(
        null_scene,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    gcp_errors = stats.gcp_errors(dataset, [null_scene])
    assert gcp_errors == {}
