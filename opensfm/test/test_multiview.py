import numpy as np

from opensfm import multiview
from opensfm import transformations as tf


def normalized(x):
    return x / np.linalg.norm(x)


def test_motion_from_plane_homography():
    R = tf.random_rotation_matrix()[:3, :3]
    t = normalized(2 * np.random.rand(3) - 1)
    n = normalized(2 * np.random.rand(3) - 1)
    d = 2 * np.random.rand() - 1
    scale = 2 * np.random.rand() - 1
    H = scale * (d * R - np.outer(t, n))

    motions = multiview.motion_from_plane_homography(H)

    goodness = []
    for Re, te, ne, de in motions:
        scalee = np.linalg.norm(te)
        good_R = np.allclose(R, Re)
        good_t = np.allclose(normalized(te), t)
        sign_n = np.sign(np.dot(ne, n))
        good_n = np.allclose(sign_n * ne, n)
        good_d = np.allclose(sign_n * de / scalee, d)
        goodness.append(good_R and good_t and good_n and good_d)

    assert any(goodness)
