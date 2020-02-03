import numpy as np

from opensfm import csfm
from opensfm.synthetic_data import synthetic_examples


def line_data():
    a, b = 2, 3
    samples = 100
    x = np.linspace(0, 100, samples)
    return a, b, x, samples


def add_outliers(ratio_outliers, x, min, max):
    for index in np.random.permutation(len(x))[:int(ratio_outliers*len(x))]:
        if np.random.rand() < 0.5:
            x[index] -= np.random.uniform(min, max)
        else:
            x[index] += np.random.uniform(min, max)


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


def test_normal_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier*sigma, csfm.RansacType.MSAC)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=5)


def test_outliers_msac():
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


def test_normal_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier, csfm.RansacType.LMedS)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=8)


def test_outliers_LMedS():
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


def test_zero_essential_ransac():
    np.random.seed(42)
    data = synthetic_examples.synthetic_small_line_scene()

    scale = 0.0
    features, _, _, _ = data.get_tracks_data(40, scale)

    shots = sorted(list(features.values()), key=lambda x: -len(x))
    f1 = shots[0][:,0:2]
    f2 = shots[1][:,0:2]
    result = csfm.ransac_relative_pose(f1, f2, scale, csfm.RansacType.RANSAC)
    a = result.inliers_indices

