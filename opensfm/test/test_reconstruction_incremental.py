import pytest
import numpy as np

from opensfm import reconstruction
from opensfm import io

from opensfm.synthetic_data import synthetic_dataset
from opensfm.synthetic_data import synthetic_scene
from opensfm.synthetic_data import synthetic_examples


@pytest.fixture(scope='module')
def scene():
    np.random.seed(42)
    return synthetic_examples.synthetic_ellipse_scene()


def test_reconstruction_incremental(scene):
    reference = scene.get_reconstruction()
    maximum_depth = 40
    projection_noise = 1.0
    gps_noise = 5.0

    exifs = scene.get_scene_exifs(gps_noise)
    features, desc, colors, graph = scene.get_tracks_data(maximum_depth,
                                                          projection_noise)
    dataset = synthetic_dataset.SyntheticDataSet(reference, exifs, features,
                                                 desc, colors, graph)

    _, reconstructed_scene = reconstruction.\
        incremental_reconstruction(dataset, graph)
    errors = scene.compare(reconstructed_scene[0])

    assert errors['ratio_cameras'] == 1.0
    assert 0.940 < errors['ratio_points'] < 0.950

    assert 0.002 < errors['rotation_average'] < 0.09
    assert 0.0002 < errors['rotation_std'] < 0.0012

    # below, (av+std) should be in order of ||gps_noise||^2
    assert 1.5 < errors['position_average'] < 3
    assert 1.1 < errors['position_std'] < 4
    assert 8.0 < errors['gps_std'] < 10.0
    assert np.allclose(errors['gps_average'], 0.0)
