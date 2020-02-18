import copy
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


def test_essential_five_points(one_pair_and_its_E):
    f1, f2, E, _ = one_pair_and_its_E

    result = pygeometry.essential_five_points(f1[0:5, :], f2[0:5, :])

    # run over the N solutions, looking for the exact one
    for E_found in result:
        if E_found[0, 0]*E[0, 0] < 0.:
            E_found *= -1.0

        good_det = np.linalg.det(E_found) < 1e-10
        is_exact = np.linalg.norm(E-E_found, ord='fro') < 1e-6
        if good_det and is_exact:
            exact_found = True

    assert exact_found


def test_essential_n_points(one_pair_and_its_E):
    f1, f2, E, _ = one_pair_and_its_E

    f1 /= np.linalg.norm(f1, axis=1)[:, None]
    f2 /= np.linalg.norm(f2, axis=1)[:, None]

    result = pygeometry.essential_n_points(f1, f2)
    E_found = result[0]

    if E_found[0, 0]*E[0, 0] < 0.:
        E_found *= -1.0

    assert np.linalg.det(E_found) < 1e-10
    assert np.linalg.norm(E-E_found, ord='fro') < 1e-6


def test_relative_pose_from_essential(one_pair_and_its_E):
    f1, f2, E, pose = one_pair_and_its_E

    result = pygeometry.relative_pose_from_essential(E, f1, f2)

    pose.translation /= np.linalg.norm(pose.translation)
    expected = pose.get_Rt()
    assert np.allclose(expected, result, rtol=1e-10)


def test_relative_pose_refinement(one_pair_and_its_E):
    f1, f2, _, pose = one_pair_and_its_E
    pose.translation /= np.linalg.norm(pose.translation)

    noisy_pose = copy.deepcopy(pose)
    noisy_pose.translation += np.random.rand(3)*1e-2
    noisy_pose.rotation += np.random.rand(3)*1e-2
    result = pygeometry.relative_pose_refinement(noisy_pose.get_Rt(), f1, f2, 1000)

    expected = pose.get_Rt()
    assert np.linalg.norm(expected-result) < 5e-2
