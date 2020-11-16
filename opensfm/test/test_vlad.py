import numpy as np
import pytest
from opensfm import vlad


def test_vlad_distances_order():
    im = "im1"
    other_ims = ["im2", "im3"]

    histograms = {
        "im1": np.array([1, 0, 0]),
        "im2": np.array([0, 1, 0]),
        "im3": np.array([1, 1, 0]) / np.linalg.norm([1, 1, 0]),
    }

    im_res, distance_res, other_res = vlad.vlad_distances(im, other_ims, histograms)

    assert im_res == im
    assert len(distance_res) == len(other_ims)
    assert other_res == other_ims

    order_res = np.argsort(distance_res)
    assert other_ims[order_res[0]] == "im3"
    assert other_ims[order_res[1]] == "im2"


def test_signed_square_root_normalize():
    v = [1, 0.01]
    res = vlad.signed_square_root_normalize(v)

    assert pytest.approx(np.linalg.norm(res), 1e-6) == 1
    assert pytest.approx(v[0] / v[1], 1e-6) == 10 * res[0] / res[1]


def test_unnormalized_vlad():
    features = np.array([[0, 1.1]])
    centers = np.array(
        [
            [1, 0],
            [0, 1],
        ]
    )

    res = vlad.unnormalized_vlad(features, centers)

    assert res[0] == res[1] == res[2] == 0
    assert pytest.approx(res[3], 1e-6) == 0.1
