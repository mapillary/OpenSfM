import copy

import numpy as np
from opensfm import pyrobust


def line_data():
    a, b = 2, 3
    samples = 100
    x = np.linspace(0, 100, samples)
    return a, b, x, samples


def add_outliers(ratio_outliers, x, min, max):
    for index in np.random.permutation(len(x))[: int(ratio_outliers * len(x))]:
        shape = x[index].shape
        noise = np.random.uniform(min, max, size=shape)
        if len(shape) == 0:
            sign = 1 if np.random.randint(2) > 0 else -1
        else:
            sign = [1 if r > 0 else -1 for r in np.random.randint(2, size=shape)]
        x[int(index)] += sign * noise


def test_uniform_line_ransac():
    a, b, x, samples = line_data()

    scale = 2.0
    y = a * x + b + np.random.rand(x.shape[0]) * scale

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, scale, params, pyrobust.RansacType.RANSAC)

    assert result.score == samples
    assert len(result.inliers_indices) == samples


def test_outliers_line_ransac():
    a, b, x, samples = line_data()

    scale = 2.0
    y = a * x + b + np.random.rand(x.shape[0]) * scale

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, scale, outliers_max)

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, scale, params, pyrobust.RansacType.RANSAC)

    inliers_count = (1 - ratio_outliers) * samples
    assert result.score == inliers_count
    assert len(result.inliers_indices) == inliers_count


def test_normal_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a * x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(
        data, multiplier * sigma, params, pyrobust.RansacType.MSAC
    )

    confidence = 0.95  # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(
        len(result.inliers_indices), samples, rtol=(1 - confidence), atol=8
    )


def test_outliers_line_msac():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a * x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, multiplier * sigma, multiplier * outliers_max)

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(
        data, multiplier * sigma, params, pyrobust.RansacType.MSAC
    )

    inliers_count = (1 - ratio_outliers) * samples
    confidence = 0.95  # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(
        len(result.inliers_indices), inliers_count, rtol=(1 - confidence), atol=5
    )


def test_normal_line_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a * x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()
    result = pyrobust.ransac_line(data, multiplier, params, pyrobust.RansacType.LMedS)

    confidence = 0.95  # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(
        len(result.inliers_indices), samples, rtol=(1 - confidence), atol=11
    )


def test_outliers_line_LMedS():
    a, b, x, samples = line_data()

    sigma = 2.0
    y = a * x + b + np.random.normal(scale=sigma, size=x.shape[0])

    multiplier = 1.96

    ratio_outliers = 0.4
    outliers_max = 5.0
    add_outliers(ratio_outliers, x, multiplier * sigma, multiplier * outliers_max)

    data = np.array([x, y]).transpose()

    params = pyrobust.RobustEstimatorParams()

    # can't be used with LMedS as an over-estimated sigma will make it stop early
    params.use_iteration_reduction = False

    result = pyrobust.ransac_line(data, multiplier, params, pyrobust.RansacType.LMedS)

    inliers_count = (1 - ratio_outliers) * samples
    confidence = 0.95  # 1.96*MAD -> 95% rejecting inliers
    assert np.isclose(
        len(result.inliers_indices), inliers_count, rtol=(1 - confidence), atol=8
    )


def test_uniform_essential_ransac(pairs_and_their_E):
    for f1, f2, _, _ in pairs_and_their_E:
        points = np.concatenate((f1, f2), axis=1)

        scale = 1e-2
        points += np.random.rand(*points.shape) * scale

        f1, f2 = points[:, 0:3], points[:, 3:6]
        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 /= np.linalg.norm(f2, axis=1)[:, None]

        scale_eps_ratio = 5e-1
        params = pyrobust.RobustEstimatorParams()
        params.use_iteration_reduction = False
        result = pyrobust.ransac_essential(
            f1, f2, scale * (1.0 + scale_eps_ratio), params, pyrobust.RansacType.RANSAC
        )

        assert len(result.inliers_indices) == len(f1) == len(f2)


def test_outliers_essential_ransac(pairs_and_their_E):
    for f1, f2, _, _ in pairs_and_their_E:
        points = np.concatenate((f1, f2), axis=1)

        scale = 1e-3
        points += np.random.rand(*points.shape) * scale

        ratio_outliers = 0.3
        add_outliers(ratio_outliers, points, 0.1, 0.4)

        f1, f2 = points[:, 0:3], points[:, 3:6]
        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 /= np.linalg.norm(f2, axis=1)[:, None]

        scale_eps_ratio = 0.5
        params = pyrobust.RobustEstimatorParams()
        result = pyrobust.ransac_essential(
            f1, f2, scale * (1.0 + scale_eps_ratio), params, pyrobust.RansacType.RANSAC
        )

        tolerance = 0.12  # some outliers might have been moved along the epipolar
        inliers_count = (1 - ratio_outliers) * len(points)
        assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)


def test_outliers_relative_pose_ransac(pairs_and_their_E):
    for f1, f2, _, pose in pairs_and_their_E:
        points = np.concatenate((f1, f2), axis=1)

        scale = 1e-3
        points += np.random.rand(*points.shape) * scale

        ratio_outliers = 0.3
        add_outliers(ratio_outliers, points, 0.1, 1.0)

        f1, f2 = points[:, 0:3], points[:, 3:6]
        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 /= np.linalg.norm(f2, axis=1)[:, None]

        scale_eps_ratio = 1e-1
        params = pyrobust.RobustEstimatorParams()
        params.iterations = 1000
        result = pyrobust.ransac_relative_pose(
            f1, f2, scale * (1.0 + scale_eps_ratio), params, pyrobust.RansacType.RANSAC
        )

        expected = pose.get_world_to_cam()[:3]
        expected[:, 3] /= np.linalg.norm(expected[:, 3])

        tolerance = 0.12
        inliers_count = (1 - ratio_outliers) * len(points)
        assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)

    assert np.linalg.norm(expected - result.lo_model, ord="fro") < 16e-2


def test_outliers_relative_rotation_ransac(pairs_and_their_E):
    for f1, _, _, _ in pairs_and_their_E:

        vec_x = np.random.rand(3)
        vec_x /= np.linalg.norm(vec_x)
        vec_y = np.array([-vec_x[1], vec_x[0], 0.0])
        vec_y /= np.linalg.norm(vec_y)
        vec_z = np.cross(vec_x, vec_y)

        rotation = np.array([vec_x, vec_y, vec_z])

        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 = [rotation.dot(x) for x in f1]

        points = np.concatenate((f1, f2), axis=1)

        scale = 1e-3
        points += np.random.rand(*points.shape) * scale

        ratio_outliers = 0.3
        add_outliers(ratio_outliers, points, 0.1, 1.0)

        f1, f2 = points[:, 0:3], points[:, 3:6]
        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 /= np.linalg.norm(f2, axis=1)[:, None]

        params = pyrobust.RobustEstimatorParams()
        params.iterations = 1000

        result = pyrobust.ransac_relative_rotation(
            f1, f2, np.sqrt(3 * scale * scale), params, pyrobust.RansacType.RANSAC
        )

        tolerance = 0.04
        inliers_count = (1 - ratio_outliers) * len(points)
        assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)

        assert np.linalg.norm(rotation - result.lo_model, ord="fro") < 8e-2


def test_outliers_absolute_pose_ransac(shots_and_their_points):
    for pose, bearings, points in shots_and_their_points:
        scale = 1e-3
        bearings = copy.deepcopy(bearings)
        bearings += np.random.rand(*bearings.shape) * scale

        ratio_outliers = 0.3
        add_outliers(ratio_outliers, bearings, 0.1, 1.0)
        bearings /= np.linalg.norm(bearings, axis=1)[:, None]

        params = pyrobust.RobustEstimatorParams()
        params.iterations = 1000
        result = pyrobust.ransac_absolute_pose(
            bearings, points, scale, params, pyrobust.RansacType.RANSAC
        )

        expected = pose.get_world_to_cam()[:3]

        tolerance = 0.05
        inliers_count = (1 - ratio_outliers) * len(points)
        assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)

        assert np.linalg.norm(expected - result.lo_model, ord="fro") < 8e-2


def test_outliers_absolute_pose_known_rotation_ransac(shots_and_their_points):
    for pose, bearings, points in shots_and_their_points:
        scale = 1e-3
        bearings = copy.deepcopy(bearings)
        bearings += np.random.rand(*bearings.shape) * scale

        ratio_outliers = 0.3
        add_outliers(ratio_outliers, bearings, 0.1, 1.0)
        bearings /= np.linalg.norm(bearings, axis=1)[:, None]

        R = pose.get_rotation_matrix()
        p_rotated = np.array([R.dot(p) for p in points])

        params = pyrobust.RobustEstimatorParams()
        params.iterations = 1000
        result = pyrobust.ransac_absolute_pose_known_rotation(
            bearings, p_rotated, scale, params, pyrobust.RansacType.RANSAC
        )

        tolerance = 0.05
        inliers_count = (1 - ratio_outliers) * len(points)
        assert np.isclose(len(result.inliers_indices), inliers_count, rtol=tolerance)

        assert np.linalg.norm(pose.translation - result.lo_model) < 8e-2
