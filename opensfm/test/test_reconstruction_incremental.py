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

    assert errors['ratio_cameras'] >= 0.95          # Keeps jumping last resection between 9 and 14 inliers with Python3
    assert 0.920 < errors['ratio_points'] < 0.950

    assert 0.002 < errors['rotation_average'] < 0.095
    assert 0.0002 < errors['rotation_std'] < 0.0022

    # below, (av+std) should be in order of ||gps_noise||^2
    assert 1.5 < errors['position_average'] < 3
    assert 1.1 < errors['position_std'] < 4
    assert 8.0 < errors['gps_std'] < 10.0
    assert errors['gps_average'] < 3e-3
