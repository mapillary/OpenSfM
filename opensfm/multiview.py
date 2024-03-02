# pyre-unsafe
import math
import random
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
from opensfm import pygeometry, pymap, pyrobust, transformations as tf


def nullspace(A: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Compute the null space of A.

    Return the smallest singular value and the corresponding vector.
    """
    u, s, vh = np.linalg.svd(A)
    return s[-1], vh[-1]


def homogeneous(x: np.ndarray) -> np.ndarray:
    """Add a column of ones to x."""
    s = x.shape[:-1] + (1,)
    return np.hstack((x, np.ones(s)))


def homogeneous_vec(x: np.ndarray) -> np.ndarray:
    """Add a column of zeros to x."""
    s = x.shape[:-1] + (1,)
    return np.hstack((x, np.zeros(s)))


def euclidean(x: np.ndarray) -> np.ndarray:
    """Divide by last column and drop it."""
    return x[..., :-1] / x[..., -1:]


def cross_product_matrix(x: np.ndarray) -> np.ndarray:
    """Return the matrix representation of x's cross product"""
    return np.array([[0, -x[2], x[1]], [x[2], 0, -x[0]], [-x[1], x[0], 0]])


def P_from_KRt(K: np.ndarray, R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """P = K[R|t]."""
    P = np.empty((3, 4))
    P[:, :3] = np.dot(K, R)
    P[:, 3] = np.dot(K, t)
    return P


def KRt_from_P(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Factorize the camera matrix into K,R,t as P = K[R|t].

    >>> K = np.array([[1, 2, 3],
    ...               [0, 4, 5],
    ...               [0, 0, 1]])
    >>> R = np.array([[ 0.57313786, -0.60900664,  0.54829181],
    ...               [ 0.74034884,  0.6716445 , -0.02787928],
    ...               [-0.35127851,  0.42190588,  0.83582225]])
    >>> t = np.array([1, 2, 3])
    >>> P = P_from_KRt(K, R, t)
    >>> KK, RR, tt = KRt_from_P(P)
    >>> np.allclose(K, KK)
    True
    >>> np.allclose(R, RR)
    True
    >>> np.allclose(t, tt)
    True
    """
    K, R = rq(P[:, :3])

    T = np.diag(np.sign(np.diag(K)))  # ensure K has positive diagonal
    K = np.dot(K, T)
    R = np.dot(T, R)
    t = np.linalg.solve(K, P[:, 3])
    if np.linalg.det(R) < 0:  # ensure det(R) = 1
        R = -R
        t = -t
    K /= K[2, 2]  # normalise K

    return K, R, t


def rq(A: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Decompose a matrix into a triangular times rotation.
    (from PCV)

    >>> Q = np.array([[ 0.57313786, -0.60900664,  0.54829181],
    ...               [ 0.74034884,  0.6716445 , -0.02787928],
    ...               [-0.35127851,  0.42190588,  0.83582225]])
    >>> R = np.array([[1, 2, 3],
    ...               [0, 4, 5],
    ...               [0, 0, 1]])
    >>> r, q = rq(R.dot(Q))
    >>> np.allclose(r.dot(q), R.dot(Q))
    True
    >>> np.allclose(abs(np.linalg.det(q)), 1.0)
    True
    >>> np.allclose(r[1,0], 0) and np.allclose(r[2,0], 0) and np.allclose(r[2,1], 0)
    True
    """
    Q, R = np.linalg.qr(np.flipud(A).T)
    R = np.flipud(R.T)
    Q = Q.T
    return R[:, ::-1], Q[::-1, :]


def vector_angle(u: np.ndarray, v: np.ndarray) -> float:
    """Angle between two vectors.

    >>> u = [ 0.99500417, -0.33333333, -0.09983342]
    >>> v = [ -0.99500417, +0.33333333, +0.09983342]
    >>> vector_angle(u, u)
    0.0
    >>> np.isclose(vector_angle(u, v), np.pi)
    True
    """
    cos = np.dot(u, v) / math.sqrt(np.dot(u, u) * np.dot(v, v))
    cos = np.clip(cos, -1, 1)
    return math.acos(cos)


def decompose_similarity_transform(
    T: np.ndarray,
) -> Tuple[float, np.ndarray, np.ndarray]:
    """Decompose the similarity transform to scale, rotation and translation"""
    m, n = T.shape[0:2]
    assert m == n
    A, b = T[: (m - 1), : (m - 1)], T[: (m - 1), (m - 1)]
    s = np.linalg.det(A) ** (1.0 / (m - 1))
    return s, A / s, b


def ransac_max_iterations(
    kernel: Any, inliers: np.ndarray, failure_probability: float
) -> float:
    if len(inliers) >= kernel.num_samples():
        return 0
    inlier_ratio = float(len(inliers)) / kernel.num_samples()
    n = kernel.required_samples
    return math.log(failure_probability) / math.log(1.0 - inlier_ratio**n)


TRansacSolution = Tuple[np.ndarray, np.ndarray, float]


def ransac(kernel: Any, threshold: float) -> TRansacSolution:
    """Robustly fit a model to data.

    >>> x = np.array([1., 2., 3.])
    >>> y = np.array([2., 4., 7.])
    >>> kernel = TestLinearKernel(x, y)
    >>> model, inliers, error = ransac(kernel, 0.1)
    >>> np.allclose(model, 2.0)
    True
    >>> inliers
    array([0, 1])
    >>> np.allclose(error, 0.1)
    True
    """
    max_iterations = 1000
    best_error = float("inf")
    best_model = None
    best_inliers = []
    i = 0
    while i < max_iterations:
        try:
            samples = kernel.sampling()
        except AttributeError:
            samples = random.sample(
                range(kernel.num_samples()), kernel.required_samples
            )
        models = kernel.fit(samples)
        for model in models:
            errors = kernel.evaluate(model)
            inliers = np.flatnonzero(np.fabs(errors) < threshold)
            error = np.fabs(errors).clip(0, threshold).sum()
            if len(inliers) and error < best_error:
                best_error = error
                best_model = model
                best_inliers = inliers
                max_iterations = min(
                    max_iterations, ransac_max_iterations(kernel, best_inliers, 0.01)
                )
        i += 1
    return best_model, best_inliers, best_error


class TestLinearKernel:
    """A kernel for the model y = a * x.

    >>> x = np.array([1., 2., 3.])
    >>> y = np.array([2., 4., 7.])
    >>> kernel = TestLinearKernel(x, y)
    >>> models = kernel.fit([0])
    >>> models
    [2.0]
    >>> errors = kernel.evaluate(models[0])
    >>> np.allclose(errors, [0., 0., 1.])
    True
    """

    required_samples = 1

    def __init__(self, x: np.ndarray, y: np.ndarray) -> None:
        self.x: np.ndarray = x
        self.y: np.ndarray = y

    def num_samples(self) -> int:
        return len(self.x)

    def fit(self, samples: np.ndarray) -> List[float]:
        x = self.x[samples[0]]
        y = self.y[samples[0]]
        return [y / x]

    def evaluate(self, model: np.ndarray) -> np.ndarray:
        return self.y - model * self.x


class PlaneKernel:
    """
    A kernel for estimating plane from on-plane points and vectors
    """

    def __init__(
        self, points, vectors, verticals, point_threshold=1.0, vector_threshold=5.0
    ) -> None:
        self.points = points
        self.vectors = vectors
        self.verticals = verticals
        self.required_samples = 3
        self.point_threshold = point_threshold
        self.vector_threshold = vector_threshold

    def num_samples(self) -> int:
        return len(self.points)

    def sampling(self) -> Dict[str, Any]:
        samples = {}
        if len(self.vectors) > 0:
            samples["points"] = self.points[
                random.sample(range(len(self.points)), 2), :
            ]
            samples["vectors"] = [
                self.vectors[i] for i in random.sample(range(len(self.vectors)), 1)
            ]
        else:
            samples["points"] = self.points[
                :, random.sample(range(len(self.points)), 3)
            ]
            samples["vectors"] = None
        return samples

    def fit(self, samples: Dict[str, np.ndarray]) -> List[np.ndarray]:
        model = fit_plane(samples["points"], samples["vectors"], self.verticals)
        return [model]

    def evaluate(self, model) -> np.ndarray:
        # only evaluate on points
        normal = model[0:3]
        normal_norm = np.linalg.norm(normal) + 1e-10
        point_error = np.abs(model.T.dot(homogeneous(self.points).T)) / normal_norm
        vectors = np.array(self.vectors)
        vector_norm = np.sum(vectors * vectors, axis=1)
        vectors = (vectors.T / vector_norm).T
        vector_error = abs(
            np.rad2deg(abs(np.arccos(vectors.dot(normal) / normal_norm))) - 90
        )
        vector_error[vector_error < self.vector_threshold] = 0.0
        vector_error[vector_error >= self.vector_threshold] = self.point_threshold + 0.1
        point_error[point_error < self.point_threshold] = 0.0
        point_error[point_error >= self.point_threshold] = self.point_threshold + 0.1
        errors = np.hstack((point_error, vector_error))
        return errors


def fit_plane_ransac(
    points: np.ndarray,
    vectors: np.ndarray,
    verticals: np.ndarray,
    point_threshold: float = 1.2,
    vector_threshold: float = 5.0,
) -> TRansacSolution:
    vectors = np.array([v / math.pi * 180.0 for v in vectors])
    kernel = PlaneKernel(
        points - points.mean(axis=0),
        vectors,
        verticals,
        point_threshold,
        vector_threshold,
    )
    p, inliers, error = ransac(kernel, point_threshold)
    num_point = points.shape[0]
    points_inliers = points[inliers[inliers < num_point], :]
    vectors_inliers = np.array(
        [vectors[i - num_point] for i in inliers[inliers >= num_point]]
    )
    p = fit_plane(
        points_inliers - points_inliers.mean(axis=0), vectors_inliers, verticals
    )
    return p, inliers, error


def fit_plane(
    points: np.ndarray, vectors: Optional[np.ndarray], verticals: Optional[np.ndarray]
) -> np.ndarray:
    """Estimate a plane from on-plane points and vectors.

    >>> x = [[0,0,0], [1,0,0], [0,1,0]]
    >>> p = fit_plane(x, None, None)
    >>> np.allclose(p, [0,0,1,0]) or np.allclose(p, [0,0,-1,0])
    True
    >>> x = [[0,0,0], [0,1,0]]
    >>> v = [[1,0,0]]
    >>> p = fit_plane(x, v, None)
    >>> np.allclose(p, [0,0,1,0]) or np.allclose(p, [0,0,-1,0])
    True
    >>> vert = [[0,0,1]]
    >>> p = fit_plane(x, v, vert)
    >>> np.allclose(p, [0,0,1,0])
    True
    """
    # (x 1) p = 0
    # (v 0) p = 0
    points = np.array(points)
    s = 1.0 / max(1e-8, points.std())  # Normalize the scale to improve conditioning.
    x = homogeneous(s * points)
    if vectors is not None and len(vectors) > 0:
        v = homogeneous_vec(s * np.array(vectors))
        A = np.vstack((x, v))
    else:
        A = x
    evalues, evectors = np.linalg.eig(A.T.dot(A))
    smallest_evalue_idx = min(enumerate(evalues), key=lambda x: x[1])[0]
    p = evectors[:, smallest_evalue_idx]

    if np.allclose(p[:3], [0, 0, 0]):
        return np.array([0.0, 0.0, 1.0, 0])

    # Use verticals to decide the sign of p
    if verticals is not None and len(verticals) > 0:
        d = 0
        for vertical in verticals:
            d += p[:3].dot(vertical)
        p *= np.sign(d)
    return p


def plane_horizontalling_rotation(p: np.ndarray) -> Optional[np.ndarray]:
    """Compute a rotation that brings p to z=0

    >>> p = [1.0, 2.0, 3.0]
    >>> R = plane_horizontalling_rotation(p)
    >>> np.allclose(R.dot(p), [0, 0, np.linalg.norm(p)])
    True

    >>> p = [0, 0, 1.0]
    >>> R = plane_horizontalling_rotation(p)
    >>> np.allclose(R.dot(p), [0, 0, np.linalg.norm(p)])
    True

    >>> p = [0, 0, -1.0]
    >>> R = plane_horizontalling_rotation(p)
    >>> np.allclose(R.dot(p), [0, 0, np.linalg.norm(p)])
    True

    >>> p = [1e-14, 1e-14, -1.0]
    >>> R = plane_horizontalling_rotation(p)
    >>> np.allclose(R.dot(p), [0, 0, np.linalg.norm(p)])
    True
    """
    v0 = p[:3]
    v1 = np.array([0.0, 0.0, 1.0])
    angle = tf.angle_between_vectors(v0, v1)
    axis = tf.vector_product(v0, v1)
    if np.linalg.norm(axis) > 0:
        return tf.rotation_matrix(angle, axis)[:3, :3]
    elif angle < 1.0:
        return np.eye(3)
    elif angle > 3.0:
        return np.diag([1, -1, -1])
    return None


def fit_similarity_transform(
    p1: np.ndarray, p2: np.ndarray, max_iterations: int = 1000, threshold: float = 1
) -> Tuple[np.ndarray, np.ndarray]:
    """Fit a similarity transform T such as p2 = T . p1 between two points sets p1 and p2"""
    # TODO (Yubin): adapt to RANSAC class

    num_points, dim = p1.shape[0:2]

    assert p1.shape[0] == p2.shape[0]

    best_inliers = []
    best_T = np.array((3, 4))
    for _ in range(max_iterations):
        rnd = np.random.permutation(num_points)
        rnd = rnd[0:dim]

        T = tf.affine_matrix_from_points(p1[rnd, :].T, p2[rnd, :].T, shear=False)
        p1h = homogeneous(p1)
        p2h = homogeneous(p2)

        errors = np.sqrt(np.sum((p2h.T - np.dot(T, p1h.T)) ** 2, axis=0))
        inliers = np.argwhere(errors < threshold)[:, 0]
        if len(inliers) >= len(best_inliers):
            best_T = T.copy()
            best_inliers = np.argwhere(errors < threshold)[:, 0]

    # Estimate similarity transform with inliers
    if len(best_inliers) > dim + 3:
        best_T = tf.affine_matrix_from_points(
            p1[best_inliers, :].T, p2[best_inliers, :].T, shear=False
        )
        errors = np.sqrt(np.sum((p2h.T - np.dot(best_T, p1h.T)) ** 2, axis=0))
        best_inliers = np.argwhere(errors < threshold)[:, 0]

    return best_T, best_inliers


def K_from_camera(camera: Dict[str, Any]) -> np.ndarray:
    f = float(camera["focal"])
    return np.array([[f, 0.0, 0.0], [0.0, f, 0.0], [0.0, 0.0, 1.0]])


def focal_from_homography(H: np.ndarray) -> np.ndarray:
    """Solve for w = H w H^t, with w = diag(a, a, b)

    >>> K = np.diag([0.8, 0.8, 1])
    >>> R = cv2.Rodrigues(np.array([0.3, 0, 0]))[0]
    >>> H = K.dot(R).dot(np.linalg.inv(K))
    >>> f = focal_from_homography(3 * H)
    >>> np.allclose(f, 0.8)
    True
    """
    H = H / np.linalg.det(H) ** (1.0 / 3.0)
    A = np.array(
        [
            [H[0, 0] * H[0, 0] + H[0, 1] * H[0, 1] - 1, H[0, 2] * H[0, 2]],
            [H[0, 0] * H[1, 0] + H[0, 1] * H[1, 1], H[0, 2] * H[1, 2]],
            [H[0, 0] * H[2, 0] + H[0, 1] * H[2, 1], H[0, 2] * H[2, 2]],
            [H[1, 0] * H[1, 0] + H[1, 1] * H[1, 1] - 1, H[1, 2] * H[1, 2]],
            [H[1, 0] * H[2, 0] + H[1, 1] * H[2, 1], H[1, 2] * H[2, 2]],
            [H[2, 0] * H[2, 0] + H[2, 1] * H[2, 1], H[2, 2] * H[2, 2] - 1],
        ]
    )
    _, (a, b) = nullspace(A)
    focal = np.sqrt(a / b)
    return focal


def R_from_homography(
    H: np.ndarray, f1: np.ndarray, f2: np.ndarray
) -> Optional[np.ndarray]:
    K1 = np.diag([f1, f1, 1])
    K2 = np.diag([f2, f2, 1])
    K2inv = np.linalg.inv(K2)
    R = K2inv.dot(H).dot(K1)
    R = project_to_rotation_matrix(R)
    return R


def project_to_rotation_matrix(A: np.ndarray) -> Optional[np.ndarray]:
    try:
        u, d, vt = np.linalg.svd(A)
    except np.linalg.linalg.LinAlgError:
        return None
    return u.dot(vt)


def camera_up_vector(rotation_matrix: np.ndarray) -> np.ndarray:
    """Unit vector pointing to zenit in camera coords.

    :param rotation: camera pose rotation
    """
    return rotation_matrix[:, 2]


def camera_compass_angle(rotation_matrix: np.ndarray) -> float:
    """Compass angle of a camera

    Angle between world's Y axis and camera's Z axis projected
    onto the XY world plane.

    :param rotation: camera pose rotation
    """
    z = rotation_matrix[2, :]  # Camera's Z axis in world coordinates
    angle = np.arctan2(z[0], z[1])
    return np.degrees(angle)


def rotation_matrix_from_up_vector_and_compass(
    up_vector: List[float], compass_angle: float
) -> np.ndarray:
    """Camera rotation given up_vector and compass.

    >>> d = [1, 2, 3]
    >>> angle = -123
    >>> R = rotation_matrix_from_up_vector_and_compass(d, angle)
    >>> np.allclose(np.linalg.det(R), 1.0)
    True
    >>> up = camera_up_vector(R)
    >>> np.allclose(d / np.linalg.norm(d), up)
    True
    >>> np.allclose(camera_compass_angle(R), angle)
    True

    >>> d = [0, 0, 1]
    >>> angle = 123
    >>> R = rotation_matrix_from_up_vector_and_compass(d, angle)
    >>> np.allclose(np.linalg.det(R), 1.0)
    True
    >>> up = camera_up_vector(R)
    >>> np.allclose(d / np.linalg.norm(d), up)
    True
    """
    r3 = np.array(up_vector) / np.linalg.norm(up_vector)
    ez = np.array([0.0, 0.0, 1.0])
    r2 = ez - np.dot(ez, r3) * r3
    r2n = np.linalg.norm(r2)
    if r2n > 1e-8:
        r2 /= r2n
        r1 = np.cross(r2, r3)
    else:  # We are looking to nadir or zenith
        r1 = np.array([1.0, 0.0, 0.0])
        r2 = np.cross(r3, r1)

    compass_rotation = cv2.Rodrigues(np.radians([0.0, 0.0, compass_angle]))[0]
    return np.column_stack([r1, r2, r3]).dot(compass_rotation)


def motion_from_plane_homography(
    H: np.ndarray,
) -> Optional[List[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]]:
    """Compute candidate camera motions from a plane-induced homography.

    Returns up to 8 motions.
    The homography is assumed to be in normalized camera coordinates.

    Uses the method of [Faugueras and Lustman 1988]

    [Faugueras and Lustman 1988] Faugeras, Olivier, and F. Lustman.
    “Motion and Structure from Motion in a Piecewise Planar Environment.”
    Report. INRIA, June 1988. https://hal.inria.fr/inria-00075698/document
    """

    try:
        u, l, vh = np.linalg.svd(H)
    except ValueError:
        return None

    d1, d2, d3 = l
    s = np.linalg.det(u) * np.linalg.det(vh)

    # Skip the cases where some singular values are nearly equal
    if d1 / d2 < 1.0001 or d2 / d3 < 1.0001:
        return None

    abs_x1 = np.sqrt((d1**2 - d2**2) / (d1**2 - d3**2))
    abs_x3 = np.sqrt((d2**2 - d3**2) / (d1**2 - d3**2))
    possible_x1_x3 = [
        (abs_x1, abs_x3),
        (abs_x1, -abs_x3),
        (-abs_x1, abs_x3),
        (-abs_x1, -abs_x3),
    ]
    solutions = []

    # Compute solutions for the two cases (1) case d' > 0 and
    # (2) case d' < 0

    for x1, x3 in possible_x1_x3:
        # compute sin and cos for d'>0 (theta, _n) and d'<0 (phi, _n) cases
        sin_term = x1 * x3 / d2
        sin_theta = (d1 - d3) * sin_term
        sin_phi = (d1 + d3) * sin_term

        d1_x3_2 = (d1 * x3**2)
        d3_x1_2 = (d3 * x1**2)
        cos_theta = (d3_x1_2 + d1_x3_2) / d2
        cos_phi = (d3_x1_2 - d1_x3_2) / d2

        # define rotation matrices for both cases
        Rp_p = np.array(
            [[cos_theta, 0, -sin_theta], [0, 1, 0], [sin_theta, 0, cos_theta]]
        )  # case d' > 0
        Rp_n = np.array(
            [[cos_phi, 0, sin_phi], [0, -1, 0], [sin_phi, 0, -cos_phi]]
        )  # case d' < 0

        # compute transformations
        np_ = np.array([x1, 0, x3])

        tp_p = (d1 - d3) * np.array([x1, 0, -x3])  # case d' > 0
        tp_n = (d1 + d3) * np_  # case d' < 0

        R_p = s * np.dot(np.dot(u, Rp_p), vh)  # case d' > 0
        R_n = s * np.dot(np.dot(u, Rp_n), vh)  # case d' < 0
        t_p = np.dot(u, tp_p)  # case d' > 0
        t_n = np.dot(u, tp_n)  # case d' < 0
        n = -np.dot(vh.T, np_)
        d = s * d2

        solutions.append((R_p, t_p, n, d))  # case d' > 0
        solutions.append((R_n, t_n, n, -d))  # case d' < 0

    return solutions


def absolute_pose_known_rotation_ransac(
    bs: np.ndarray,
    Xs: np.ndarray,
    threshold: float,
    iterations: int,
    probability: float,
) -> np.ndarray:
    params = pyrobust.RobustEstimatorParams()
    params.iterations = iterations
    result = pyrobust.ransac_absolute_pose_known_rotation(
        bs, Xs, threshold, params, pyrobust.RansacType.RANSAC
    )

    t = -result.lo_model.copy()
    R = np.identity(3)
    return np.concatenate((R, [[t[0]], [t[1]], [t[2]]]), axis=1)


def absolute_pose_ransac(
    bs: np.ndarray,
    Xs: np.ndarray,
    threshold: float,
    iterations: int,
    probability: float,
) -> np.ndarray:
    params = pyrobust.RobustEstimatorParams()
    params.iterations = iterations
    result = pyrobust.ransac_absolute_pose(
        bs, Xs, threshold, params, pyrobust.RansacType.RANSAC
    )

    Rt = result.lo_model.copy()
    R, t = Rt[:3, :3].copy(), Rt[:, 3].copy()
    Rt[:3, :3] = R.T
    Rt[:, 3] = -R.T.dot(t)
    return Rt


def relative_pose_ransac(
    b1: np.ndarray,
    b2: np.ndarray,
    threshold: float,
    iterations: int,
    probability: float,
) -> np.ndarray:
    params = pyrobust.RobustEstimatorParams()
    params.iterations = iterations
    result = pyrobust.ransac_relative_pose(
        b1, b2, threshold, params, pyrobust.RansacType.RANSAC
    )

    Rt = result.lo_model.copy()
    R, t = Rt[:3, :3].copy(), Rt[:, 3].copy()
    Rt[:3, :3] = R.T
    Rt[:, 3] = -R.T.dot(t)
    return Rt


def relative_pose_ransac_rotation_only(
    b1: np.ndarray,
    b2: np.ndarray,
    threshold: float,
    iterations: int,
    probability: float,
) -> np.ndarray:
    params = pyrobust.RobustEstimatorParams()
    params.iterations = iterations
    result = pyrobust.ransac_relative_rotation(
        b1, b2, threshold, params, pyrobust.RansacType.RANSAC
    )
    return result.lo_model.T


def relative_pose_optimize_nonlinear(
    b1: np.ndarray, b2: np.ndarray, t: np.ndarray, R: np.ndarray, iterations: int
) -> np.ndarray:
    Rt = np.zeros((3, 4))
    Rt[:3, :3] = R.T
    Rt[:, 3] = -R.T.dot(t)
    Rt_refined = pygeometry.relative_pose_refinement(Rt, b1, b2, iterations)

    R, t = Rt_refined[:3, :3].copy(), Rt_refined[:, 3].copy()
    Rt[:3, :3] = R.T
    Rt[:, 3] = -R.T.dot(t)
    return Rt


def triangulate_gcp(
    point: pymap.GroundControlPoint,
    shots: Dict[str, pymap.Shot],
    reproj_threshold: float = 0.02,
    min_ray_angle_degrees: float = 1.0,
) -> Optional[np.ndarray]:
    """Compute the reconstructed position of a GCP from observations."""

    os, bs, ids = [], [], []
    for observation in point.observations:
        shot_id = observation.shot_id
        if shot_id in shots:
            shot = shots[shot_id]
            os.append(shot.pose.get_origin())
            x = observation.projection
            b = shot.camera.pixel_bearing(np.array(x))
            r = shot.pose.get_rotation_matrix().T
            bs.append(r.dot(b))
            ids.append(shot_id)

    if len(os) >= 2:
        thresholds = len(os) * [reproj_threshold]
        valid_triangulation, X = pygeometry.triangulate_bearings_midpoint(
            np.asarray(os),
            np.asarray(bs),
            thresholds,
            np.radians(min_ray_angle_degrees),
            np.radians(180.0 - min_ray_angle_degrees),
        )
        if valid_triangulation:
            return X
    return None
