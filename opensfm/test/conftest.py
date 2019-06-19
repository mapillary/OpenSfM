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
def scene():
    np.random.seed(42)
    return synthetic_examples.synthetic_ellipse_scene()
