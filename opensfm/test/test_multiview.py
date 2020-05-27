import copy
import random
import numpy as np

from opensfm import multiview
from opensfm import pygeometry
from opensfm import transformations as tf
from opensfm.synthetic_data import synthetic_examples


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


def test_essential_five_points(pairs_and_their_E):
    exact_found = 0
    for f1, f2, E, _ in pairs_and_their_E:

        result = pygeometry.essential_five_points(f1[0:5, :], f2[0:5, :])

        # run over the N solutions, looking for the exact one
        for E_found in result:
            if E_found[0, 0]*E[0, 0] < 0.:
                E_found *= -1.0

            good_det = np.linalg.det(E_found) < 1e-10
            is_exact = np.linalg.norm(E-E_found, ord='fro') < 1e-6
            exact_found += (good_det and is_exact)

    exacts = len(pairs_and_their_E)-1
    assert exact_found >= exacts


def test_absolute_pose_three_points(shots_and_their_points):
    exact_found = 0
    for pose, bearings, points in shots_and_their_points:
        result = pygeometry.absolute_pose_three_points(bearings, points)

        expected = pose.get_Rt()
        for Rt in result:
            exact_found += (np.linalg.norm(expected-Rt, ord='fro') < 1e-6)

    exacts = len(shots_and_their_points)-2
    assert exact_found >= exacts


def test_absolute_pose_n_points(shots_and_their_points):
    for pose, bearings, points in shots_and_their_points:
        result = pygeometry.absolute_pose_n_points(bearings, points)

        expected = pose.get_Rt()
        assert np.linalg.norm(expected-result, ord='fro') < 1e-5


def test_absolute_pose_n_points_known_rotation(shots_and_their_points):
    for pose, bearings, points in shots_and_their_points:
        R = pose.get_rotation_matrix()
        p_rotated = np.array([R.dot(p) for p in points])
        result = pygeometry.absolute_pose_n_points_known_rotation(bearings, p_rotated)

        assert np.linalg.norm(pose.translation-result) < 1e-6


def test_essential_n_points(pairs_and_their_E):
    for f1, f2, E, _ in pairs_and_their_E:

        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 /= np.linalg.norm(f2, axis=1)[:, None]

        result = pygeometry.essential_n_points(f1, f2)
        E_found = result[0]

        if E_found[0, 0]*E[0, 0] < 0.:
            E_found *= -1.0

        assert np.linalg.det(E_found) < 1e-10
        assert np.linalg.norm(E-E_found, ord='fro') < 1e-6


def test_relative_pose_from_essential(pairs_and_their_E):
    for f1, f2, E, pose in pairs_and_their_E:

        result = pygeometry.relative_pose_from_essential(E, f1, f2)

        pose = copy.deepcopy(pose)
        pose.translation /= np.linalg.norm(pose.translation)

        expected = pose.get_Rt()
        assert np.allclose(expected, result, rtol=1e-10)


def test_relative_rotation(pairs_and_their_E):
    for f1, _, _, _ in pairs_and_their_E:

        vec_x = np.random.rand(3)
        vec_x /= np.linalg.norm(vec_x)
        vec_y = np.array([-vec_x[1], vec_x[0], 0.])
        vec_y /= np.linalg.norm(vec_y)
        vec_z = np.cross(vec_x, vec_y)

        rotation = np.array([vec_x, vec_y, vec_z])

        f1 /= np.linalg.norm(f1, axis=1)[:, None]
        f2 = [rotation.dot(x) for x in f1]

        result = pygeometry.relative_rotation_n_points(f1, f2)

        assert np.allclose(rotation, result, rtol=1e-10)


def test_relative_pose_refinement(pairs_and_their_E):
    exact_found = 0
    for f1, f2, _, pose in pairs_and_their_E:
        pose = copy.deepcopy(pose)
        pose.translation /= np.linalg.norm(pose.translation)

        noisy_pose = copy.deepcopy(pose)
        noisy_pose.translation += np.random.rand(3)*1e-1
        noisy_pose.rotation += np.random.rand(3)*1e-2
        result = pygeometry.relative_pose_refinement(noisy_pose.get_Rt(), f1, f2, 1000)

        expected = pose.get_Rt()
        exact_found += np.linalg.norm(expected-result) < 1.8e-1

    exacts = len(pairs_and_their_E)-1
    assert exact_found >= exacts
