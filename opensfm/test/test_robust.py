import numpy as np

from opensfm import csfm


def line_data():
    a, b = 2, 3
    samples = 100
    x = np.linspace(0, 100, samples)
    return a, b, x, samples


def test_robust_line_ransac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.rand(x.shape[0])*sigma

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, sigma, csfm.RansacType.RANSAC)

    assert result.score == samples
    assert len(result.inliers_indices) == samples


def test_robust_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier, csfm.RansacType.MSAC)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=5)


def test_robust_line_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()
    result = csfm.ransac_line(data, multiplier, csfm.RansacType.LMedS)

    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), samples,
                      rtol=(1 - confidence), atol=8)

