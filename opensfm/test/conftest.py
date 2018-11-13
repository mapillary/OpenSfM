from distutils.version import LooseVersion

import numpy


def pytest_configure(config):
    use_legacy_numpy_printoptions()


def use_legacy_numpy_printoptions():
    """Ensure numpy use legacy print formant."""
    if LooseVersion(numpy.__version__).version[:2] > [1, 13]:
        numpy.set_printoptions(legacy='1.13')
