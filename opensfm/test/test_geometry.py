import numpy as np
from opensfm import geometry


def test_rotaction_from_ptr_v2():
    ptr = 0.1, 0.2, 0.3
    rotation = geometry.rotation_from_ptr(*ptr)
    assert np.allclose(ptr, geometry.ptr_from_rotation(rotation))


def test_rotaction_from_ptr_v2():
    ptr = 0.1, 0.2, 0.3
    rotation = geometry.rotation_from_ptr_v2(*ptr)
    assert np.allclose(ptr, geometry.ptr_from_rotation_v2(rotation))


def test_rotation_from_ptr_compatibility():
    """Check the two implementations yield the same rotation."""
    ptr = 0.1, 0.2, 0.3
    assert np.allclose(
        geometry.rotation_from_ptr(*ptr), geometry.rotation_from_ptr_v2(*ptr)
    )
