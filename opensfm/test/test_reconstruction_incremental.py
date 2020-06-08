import pytest
import numpy as np

from opensfm import reconstruction
from opensfm.synthetic_data import synthetic_dataset
from opensfm.synthetic_data import synthetic_scene


def test_reconstruction_incremental(scene_synthetic):
    reference = scene_synthetic[0].get_reconstruction()
    dataset = synthetic_dataset.SyntheticDataSet(reference,
                                                 scene_synthetic[1],
                                                 scene_synthetic[2],
                                                 scene_synthetic[3],
                                                 scene_synthetic[4],
                                                 scene_synthetic[5])

    _, reconstructed_scene = reconstruction.\
        incremental_reconstruction(dataset, scene_synthetic[5])
    errors = synthetic_scene.compare(reference, reconstructed_scene[0])

    assert errors['ratio_cameras'] == 1.0
    assert 0.7 < errors['ratio_points'] < 1.0

    assert 0 < errors['aligned_position_rmse'] < 0.02
    assert 0 < errors['aligned_rotation_rmse'] < 0.001
    assert 0 < errors['aligned_points_rmse'] < 0.1

    # Sanity check that GPS error is similar to the generated gps_noise
    assert 4.0 < errors['absolute_gps_rmse'] < 7.0
