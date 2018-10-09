import pytest
import numpy as np

from opensfm import reconstruction
from opensfm import io

from opensfm.synthetic_data import synthetic_dataset
from opensfm.synthetic_data import synthetic_scene


@pytest.fixture(scope='module')
def scene():
    np.random.seed(42)

    scene_length = 60
    points_count = 5000
    generator = synthetic_scene.get_scene_generator('ellipse', scene_length)
    scene = synthetic_scene.SyntheticScene(generator)
    scene.add_street(points_count, 7, 7).perturb_floor([0, 0, 0.1]).\
        perturb_walls([0.2, 0.2, 0.01])

    camera_height = 1.5
    camera_interval = 3
    position_perturbation = [0.2, 0.2, 0.01]
    rotation_perturbation = 0.3
    camera = synthetic_scene.get_camera('perspective', '1', 0.9, -0.1, 0.01)
    scene.add_camera_sequence(camera, 0, scene_length,
                              camera_height, camera_interval,
                              position_perturbation,
                              rotation_perturbation)
    return scene


def test_reconstruction_incremental(scene):
    reference = scene.get_reconstruction()
    maximum_depth = 40
    projection_noise = 1.0
    gps_noise = 5.0

    exifs = scene.get_scene_exifs(gps_noise)
    features, colors, graph = scene.get_tracks_data(maximum_depth,
                                                    projection_noise)
    dataset = synthetic_dataset.SyntheticDataSet(reference, exifs, features,
                                                 colors, graph)

    _, reconstructed_scene = reconstruction.\
        incremental_reconstruction(dataset, graph)
    errors = scene.compare(reconstructed_scene[0])

    assert errors['ratio_cameras'] == 1.0
    assert 0.940 < errors['ratio_points'] < 0.950

    assert 0.02 < errors['rotation_average'] < 0.06
    assert 0.0007 < errors['rotation_std'] < 0.0012

    # below, (av+std) should be in order of ||gps_noise||^2
    assert 1.5 < errors['position_average'] < 2.5
    assert 1.1 < errors['position_std'] < 2.6
    assert 8.0 < errors['gps_std'] < 10.0
    assert np.allclose(errors['gps_average'], 0.0)
