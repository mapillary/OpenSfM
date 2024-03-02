# pyre-unsafe
import numpy as np
from opensfm import geometry


def test_rotation_from_ptr() -> None:
    ptr = 0.1, 0.2, 0.3
    rotation = geometry.rotation_from_ptr(*ptr)
    assert np.allclose(ptr, geometry.ptr_from_rotation(rotation))


def test_rotation_from_ptr_v2() -> None:
    ptr = 0.1, 0.2, 0.3
    rotation = geometry.rotation_from_ptr_v2(*ptr)
    assert np.allclose(ptr, geometry.ptr_from_rotation_v2(rotation))


def test_rotation_from_ptr_compatibility() -> None:
    """Check the two implementations yield the same rotation."""
    ptr = 0.1, 0.2, 0.3
    assert np.allclose(
        geometry.rotation_from_ptr(*ptr), geometry.rotation_from_ptr_v2(*ptr)
    )


def test_rotation_from_opk() -> None:
    ptr = 0.1, 0.2, 0.3
    rotation = geometry.rotation_from_opk(*ptr)
    assert np.allclose(ptr, geometry.opk_from_rotation(rotation))
