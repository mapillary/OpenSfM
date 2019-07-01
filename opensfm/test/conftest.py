from distutils.version import LooseVersion

import pytest
import numpy as np

from opensfm.synthetic_data import synthetic_examples


def pytest_configure(config):
    use_legacy_numpy_printoptions()


def use_legacy_numpy_printoptions():
    """Ensure numpy use legacy print formant."""
    if LooseVersion(np.__version__).version[:2] > [1, 13]:
        np.set_printoptions(legacy='1.13')


@pytest.fixture(scope='module')
def scene_synthetic():
    np.random.seed(42)
    data = synthetic_examples.synthetic_ellipse_scene()

    maximum_depth = 40
    projection_noise = 1.0
    gps_noise = 5.0

    exifs = data.get_scene_exifs(gps_noise)
    features, desc, colors, graph = data.get_tracks_data(maximum_depth,
                                                         projection_noise)
    return data, exifs, features, desc, colors, graph
