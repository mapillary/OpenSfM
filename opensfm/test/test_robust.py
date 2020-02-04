import pytest
import numpy as np

from opensfm import csfm
from opensfm import multiview
from opensfm.synthetic_data import synthetic_examples


def line_data():
    a, b = 2, 3
    samples = 100
    x = np.linspace(0, 100, samples)
    return a, b, x, samples


def add_outliers(ratio_outliers, x, min, max):
    for index in np.random.permutation(len(x))[:int(ratio_outliers*len(x))]:
        shape = x[index].shape
        noise = np.random.uniform(min, max, size=shape)
        sign = [1 if x > 0 else -1 for x in np.random.randint(2, size=shape)]
        x[index] += sign*noise


def test_uniform_line_ransac():
    a, b, x, samples = line_data()

    scale = 2.0
    y = a*x + b + np.random.rand(x.shape[0])*scale

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, scale, csfm.RansacType.RANSAC)

    assert result.score == samples
    assert len(result.inliers_indices) == samples


def test_outliers_line_ransac():
    a, b, x, samples = line_data()

    scale = 2.0
    y = a*x + b + np.random.rand(x.shape[0])*scale

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, scale, outliers_max)

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, scale, csfm.RansacType.RANSAC)

    inliers_count = (1-ratio_outliers)*samples
    assert result.score == inliers_count
    assert len(result.inliers_indices) == inliers_count


def test_normal_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier*sigma, csfm.RansacType.MSAC)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=5)


def test_outliers_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, multiplier*sigma, multiplier*outliers_max)

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier*sigma, csfm.RansacType.MSAC)

    inliers_count = (1-ratio_outliers)*samples
    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), inliers_count,
                      rtol=(1 - confidence), atol=5)


def test_normal_line_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier, csfm.RansacType.LMedS)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=8)


def test_outliers_line_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, multiplier*sigma, multiplier*outliers_max)

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier, csfm.RansacType.LMedS)

    inliers_count = (1-ratio_outliers)*samples
    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), inliers_count,
                      rtol=(1 - confidence), atol=8)


def test_uniform_essential_ransac(one_pair_and_its_E):
    f1, f2, E = one_pair_and_its_E
    points = np.concatenate((f1, f2), axis=1)

    scale = 1e-2
    points += np.random.rand(*points.shape)*scale

    result = csfm.ransac_essential(points[:, 0:3], points[:, 3:6], scale, csfm.RansacType.RANSAC)

    assert len(result.inliers_indices) == len(f1) == len(f2)


def test_outliers_essential_ransac(one_pair_and_its_E):
    f1, f2, E = one_pair_and_its_E
    points = np.concatenate((f1, f2), axis=1)

    scale = 1e-3
    points += np.random.rand(*points.shape)*scale

    ratio_outliers = 0.4
    add_outliers(ratio_outliers, points, 0.1, 1.0)

    result = csfm.ransac_essential(points[:, 0:3], points[:, 3:6], scale, csfm.RansacType.RANSAC)

    tolerance = 0.04    # some outliers might have been moved along the epipolar
    inliers_count = (1 - ratio_outliers) * len(points)
    assert np.isclose(len(result.inliers_indices),
                      inliers_count, rtol=tolerance)
