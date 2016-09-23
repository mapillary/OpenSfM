import numpy as np

from opensfm import dense


def test_angle_between_points(tmpdir):
    origin = [0, 0, 0]
    p1 = [1, 0, 0]
    p2 = [0, 1, 0]

    res = dense.angle_between_points(origin, p1, p2)

    assert np.allclose(res, np.pi / 2)

    origin = [10, 15, 20]
    p1 = [10, 16, 20]
    p2 = [10, 16, 21]

    res = dense.angle_between_points(origin, p1, p2)

    assert np.allclose(res, np.pi / 4)
