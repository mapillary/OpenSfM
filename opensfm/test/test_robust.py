import pytest
import numpy as np

from opensfm import pyrobust
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
        if len(shape) == 0:
            sign = 1 if np.random.randint(2) > 0 else -1
        else:
            sign = [1 if r > 0 else -1 for r in np.random.randint(2, size=shape)]
        x[int(index)] += sign*noise


def test_uniform_line_ransac():
    a, b, x, samples = line_data()

    scale = 2.0
    y = a*x + b + np.random.rand(x.shape[0])*scale

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, scale, params, pyrobust.RansacType.RANSAC)

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

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, scale, params, pyrobust.RansacType.RANSAC)

    inliers_count = (1-ratio_outliers)*samples
    assert result.score == inliers_count
    assert len(result.inliers_indices) == inliers_count


def test_normal_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a*x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, multiplier*sigma, params, pyrobust.RansacType.MSAC)

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

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, multiplier*sigma, params, pyrobust.RansacType.MSAC)

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

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, multiplier, params, pyrobust.RansacType.LMedS)

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

    params = pyrobust.RobustEstimatorParams()

    # can't be used with LMedS as an over-estimated sigma will make it stop early
    params.use_iteration_reduction = False

    result = pyrobust.ransac_line(data, multiplier, params, pyrobust.RansacType.LMedS)

    inliers_count = (1-ratio_outliers)*samples
    confidence = 0.95   # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(len(result.inliers_indices), inliers_count,
                      rtol=(1 - confidence), atol=8)


def test_uniform_essential_ransac(one_pair_and_its_E):
    f1, f2, E, _ = one_pair_and_its_E
    points = np.concatenate((f1, f2), axis=1)

    scale = 1e-2
    points += np.random.rand(*points.shape)*scale

    f1, f2 = points[:, 0:3],  points[:, 3:6]
    f1 /= np.linalg.norm(f1, axis=1)[:, None]
    f2 /= np.linalg.norm(f2, axis=1)[:, None]

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_essential(f1, f2, scale, params, pyrobust.RansacType.RANSAC)

    assert len(result.inliers_indices) == len(f1) == len(f2)


def test_outliers_essential_ransac(one_pair_and_its_E):
    f1, f2, E, _ = one_pair_and_its_E
    points = np.concatenate((f1, f2), axis=1)

    scale = 1e-3
    points += np.random.rand(*points.shape)*scale

    ratio_outliers = 0.4
    add_outliers(ratio_outliers, points, 0.1, 0.4)

    f1, f2 = points[:, 0:3],  points[:, 3:6]
    f1 /= np.linalg.norm(f1, axis=1)[:, None]
    f2 /= np.linalg.norm(f2, axis=1)[:, None]

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_essential(f1, f2, scale, params, pyrobust.RansacType.RANSAC)

    tolerance = 0.04    # some outliers might have been moved along the epipolar
    inliers_count = (1 - ratio_outliers) * len(points)
    assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)

    # sometimes, the negative of E is the good one
    correct_found = 0
    for sign in [-1, 1]:
        correct_found += np.linalg.norm(E-sign*result.lo_model, ord='fro') < 5e-2
    assert correct_found == 1


def test_outliers_relative_pose_ransac(one_pair_and_its_E):
    f1, f2, _, pose = one_pair_and_its_E
    points = np.concatenate((f1, f2), axis=1)

    scale = 1e-3
    points += np.random.rand(*points.shape)*scale

    ratio_outliers = 0.1
    add_outliers(ratio_outliers, points, 0.1, 1.0)

    f1, f2 = points[:, 0:3],  points[:, 3:6]
    f1 /= np.linalg.norm(f1, axis=1)[:, None]
    f2 /= np.linalg.norm(f2, axis=1)[:, None]

    params = pyrobust.RobustEstimatorParams()
    params.iterations = 1000
    result = pyrobust.ransac_relative_pose(f1, f2, scale, params, pyrobust.RansacType.RANSAC)

    pose.translation /= np.linalg.norm(pose.translation)
    expected = pose.get_Rt()

    tolerance = 0.04    # some outliers might have been moved along the epipolar
    inliers_count = (1 - ratio_outliers) * len(points)
    assert np.isclose(len(result.inliers_indices),
                      inliers_count, rtol=tolerance)

    assert np.linalg.norm(expected-result.lo_model, ord='fro')  < 8e-2
