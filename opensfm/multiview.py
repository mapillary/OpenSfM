# -*- coding: utf-8 -*-

from subprocess import call, Popen, PIPE
import numpy as np
from numpy.linalg import qr
import random
import math
import cv2
from opensfm import transformations as tf
from opensfm import csfm

def nullspace(A):
    '''Compute the null space of A.

    Return the smallest sigular value and the corresponding vector.
    '''
    u, s, vh = np.linalg.svd(A)
    return s[-1], vh[-1]


def homogeneous(x):
    '''Add a column of ones to x.
    '''
    s = x.shape[:-1] + (1,)
    return np.hstack((x, np.ones(s)))

def homogeneous_vec(x):
    '''Add a column of zeros to x.
    '''
    s = x.shape[:-1] + (1,)
    return np.hstack((x, np.zeros(s)))

def euclidean(x):
    '''Divide by last column and drop it.
    '''
    return x[..., :-1] / x[..., -1:]


def P_from_KRt(K, R, t):
    '''P = K[R|t].
    '''
    P = np.empty((3, 4))
    P[:, :3] = np.dot(K, R)
    P[:, 3] = np.dot(K, t)
    return P

def KRt_from_P(P):
    '''Factorize the camera matrix into K,R,t as P = K[R|t].

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
    '''
    K, R = rq(P[:, :3])

    T = np.diag(np.sign(np.diag(K))) # ensure K has positive diagonal
    K = np.dot(K, T)
    R = np.dot(T, R)
    t = np.linalg.solve(K, P[:,3])
    if np.linalg.det(R) < 0:         # ensure det(R) = 1
        R = -R
        t = -t
    K /= K[2, 2]                     # normalise K

    return K, R, t


def rq(A):
    '''Decompose a matrix into a triangular times rotation.
    (from PCV)

    >>> Q = np.array([[ 0.57313786, -0.60900664,  0.54829181],
    ...               [ 0.74034884,  0.6716445 , -0.02787928],
    ...               [-0.35127851,  0.42190588,  0.83582225]])
    >>> R = np.array([[1, 2, 3],
    ...               [0, 4, 5],
    ...               [0, 0, 1]])
    >>> r, q = rq(R.dot(Q))
    >>> np.allclose(r, R)
    True
    >>> np.allclose(q, Q)
    True
    '''
    Q, R = qr(np.flipud(A).T)
    R = np.flipud(R.T)
    Q = Q.T
    return R[:,::-1], Q[::-1,:]


def pixel_direction(KR, x):
    '''Find a unit vector v such that x = KR v.

    >>> v = np.array([2, 3, 1])
    >>> v = v / np.linalg.norm(v)
    >>> K = [[6, 1, 3], [0, 5, 3], [0, 0, 1]]
    >>> x = np.dot(K, v)[:2]
    >>> estimated_v = pixel_direction(K, x)
    >>> np.allclose(estimated_v, v)
    True
    '''
    v = np.linalg.solve(KR, [x[0], x[1], 1])
    return v / np.linalg.norm(v)

def vector_angle(u, v):
    '''
    >>> u = [ 0.99500417 -0.33333333 -0.09983342]
    >>> v = [ 0.99500417 -0.33333333 -0.09983342]
    >>> vector_angle(u, v)
    0.0
    '''
    cos = np.dot(u, v) / math.sqrt(np.dot(u,u) * np.dot(v,v))
    if cos >= 1.0: return 0.0
    else: return math.acos(cos)

def decompose_similarity_transform(T):
    ''' Decompose the similarity transform to scale, rotation and translation
    '''
    m, n = T.shape[0:2]
    assert(m==n)
    A, b = T[:(m-1),:(m-1)], T[:(m-1),(m-1)]
    s = np.linalg.det(A)**(1./(m-1))
    A /= s
    return s, A, b

def triangulate(Ps, xs):
    '''
    >>> xs = [np.array([ 1, 1]),
    ...       np.array([-1, 1])]
    >>> Ps = [np.array([[1, 0, 0, 0],
    ...                 [0, 1, 0, 0],
    ...                 [0, 0, 1, 0]]),
    ...       np.array([[1, 0, 0,-1],
    ...                 [0, 1, 0, 0],
    ...                 [0, 0, 1, 0]])]
    >>> triangulate(Ps, xs)
    array([ 0.5,  0.5,  0.5])
    '''
    # HZ 12.2 pag.312
    A = np.zeros((2 * len(Ps), 4))
    for i, (P, x) in enumerate(zip(Ps, xs)):
        A[2 * i    ] = x[0] * P[2] - P[0]
        A[2 * i + 1] = x[1] * P[2] - P[1]
    s, X = nullspace(A)
    return euclidean(X)


def ransac_max_iterations(kernel, inliers, failure_probability):
    if len(inliers) >= kernel.num_samples():
        return 0
    inlier_ratio = float(len(inliers)) / kernel.num_samples()
    n = kernel.required_samples
    return math.log(failure_probability) / math.log(1.0 - inlier_ratio**n)


def ransac(kernel, threshold):
    '''Robustly fit a model to data.

    >>> x = np.array([1., 2., 3.])
    >>> y = np.array([2., 4., 7.])
    >>> kernel = TestLinearKernel(x, y)
    >>> ransac(kernel, 0.1)
    (2.0, array([0, 1]), 0.10000000000000001)
    '''
    max_iterations = 1000
    best_error = float('inf')
    best_model = None
    best_inliers = []
    i = 0
    while i < max_iterations:
        try:
            samples = kernel.sampling()
        except AttributeError:
            samples = random.sample(xrange(kernel.num_samples()),
                                kernel.required_samples)
        models = kernel.fit(samples)
        for model in models:
            errors = kernel.evaluate(model)
            inliers = np.flatnonzero(np.fabs(errors) < threshold)
            error = np.fabs(errors).clip(0, threshold).sum()
            if len(inliers) and error < best_error:
                best_error = error
                best_model = model
                best_inliers = inliers
                max_iterations = min(max_iterations,
                    ransac_max_iterations(kernel, best_inliers, 0.01))
        i += 1
    return best_model, best_inliers, best_error


class TestLinearKernel:
    '''A kernel for the model y = a * x.

    >>> x = np.array([1., 2., 3.])
    >>> y = np.array([2., 4., 7.])
    >>> kernel = TestLinearKernel(x, y)
    >>> models = kernel.fit([0])
    >>> models
    [2.0]
    >>> kernel.evaluate(models[0])
    array([ 0.,  0.,  1.])
    '''
    required_samples = 1

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def num_samples(self):
        return len(self.x)

    def fit(self, samples):
        x = self.x[samples[0]]
        y = self.y[samples[0]]
        return [y / x]

    def evaluate(self, model):
        return self.y - model * self.x

class PlaneKernel:
    '''
    A kernel for estimating plane from on-plane points and vectors
    '''

    def __init__(self, points, vectors, verticals, point_threshold=1.0, vector_threshold=5.0):
        self.points = points
        self.vectors = vectors
        self.verticals = verticals
        self.required_samples = 3
        self.point_threshold = point_threshold
        self.vector_threshold = vector_threshold

    def num_samples(self):
        return len(self.points)

    def sampling(self):
        samples = {}
        if len(self.vectors)>0:
            samples['points'] = self.points[random.sample(xrange(len(self.points)), 2),:]
            samples['vectors'] = [self.vectors[i] for i in random.sample(xrange(len(self.vectors)), 1)]
        else:
            samples['points'] = self.points[:,random.sample(xrange(len(self.points)), 3)]
            samples['vectors'] = None
        return samples

    def fit(self, samples):
        model = fit_plane(samples['points'], samples['vectors'], self.verticals)
        return [model]

    def evaluate(self, model):
        # only evaluate on points
        normal = model[0:3]
        normal_norm = np.linalg.norm(normal)+1e-10
        point_error = np.abs(model.T.dot(homogeneous(self.points).T))/normal_norm
        vectors = np.array(self.vectors)
        vector_norm = np.sum(vectors*vectors, axis=1)
        vectors = (vectors.T / vector_norm).T
        vector_error = abs(np.rad2deg(abs(np.arccos(vectors.dot(normal)/normal_norm)))-90)
        vector_error[vector_error<self.vector_threshold] = 0.0
        vector_error[vector_error>=self.vector_threshold] = self.point_threshold+0.1
        point_error[point_error<self.point_threshold] = 0.0
        point_error[point_error>=self.point_threshold] = self.point_threshold+0.1
        errors = np.hstack((point_error, vector_error))
        return errors

def fit_plane_ransac(points, vectors, verticals, point_threshold=1.2, vector_threshold=5.0):
    vectors = [v/math.pi*180.0 for v in vectors]
    kernel = PlaneKernel(points - points.mean(axis=0), vectors, verticals, point_threshold, vector_threshold)
    p, inliers, error = ransac(kernel, point_threshold)
    num_point = points.shape[0]
    points_inliers = points[inliers[inliers<num_point],:]
    vectors_inliers = [vectors[i-num_point] for i in inliers[inliers>=num_point]]
    p = fit_plane(points_inliers - points_inliers.mean(axis=0), vectors_inliers, verticals)
    return p, inliers, error

def fit_plane(points, vectors, verticals):
    '''Estimate a plane fron on-plane points and vectors.

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
    '''
    # (x 1) p = 0
    # (v 0) p = 0
    points = np.array(points)
    s = 1. / max(1e-8, points.std())           # Normalize the scale to improve conditioning.
    x = homogeneous(s * points)
    if vectors:
        v = homogeneous_vec(s * np.array(vectors))
        A = np.vstack((x, v))
    else:
        A = x
    _, p = nullspace(A)
    p[3] /= s

    if np.allclose(p[:3], [0,0,0]):
        return np.array([0.0, 0.0, 1.0, 0])

    # Use verticals to decide the sign of p
    if verticals:
        d = 0
        for vertical in verticals:
            d += p[:3].dot(vertical)
        p *= np.sign(d)
    return p


def plane_horizontalling_rotation(p):
    '''Compute a rotation that brings p to z=0

    >>> p = [1.,2.,3.]
    >>> R = plane_horizontalling_rotation(p)
    >>> np.allclose(R.dot(p), [0,0,np.linalg.norm(p)])
    True
    '''
    v0 = p[:3]
    v1 = [0,0,1.0]
    angle = tf.angle_between_vectors(v0, v1)
    if angle > 0:
        return tf.rotation_matrix(angle,
                                  tf.vector_product(v0, v1)
                                  )[:3,:3]
    else:
        return np.eye(3)


def fit_similarity_transform(p1, p2, max_iterations=1000, threshold=1):
    ''' Fit a similarity transform between two points sets
    '''
    # TODO (Yubin): adapt to RANSAC class

    num_points, dim = p1.shape[0:2]

    assert(p1.shape[0]==p2.shape[0])

    best_inliers= 0

    for i in xrange(max_iterations):

        rnd = np.random.permutation(num_points)
        rnd = rnd[0:dim]

        T = tf.affine_matrix_from_points(p1[rnd,:].T, p2[rnd,:].T, shear=False)
        p1h = homogeneous(p1)
        p2h = homogeneous(p2)

        errors = np.sqrt(np.sum( ( p2h.T - np.dot(T, p1h.T) ) ** 2 , axis=0 ) )

        inliers = np.argwhere(errors < threshold)[:,0]

        num_inliers = len(inliers)

        if num_inliers >= best_inliers:
            best_inliers = num_inliers
            best_T = T.copy()
            inliers = np.argwhere(errors < threshold)[:,0]

    # Estimate similarity transform with inliers
    if len(inliers)>dim+3:
        best_T = tf.affine_matrix_from_points(p1[inliers,:].T, p2[inliers,:].T, shear=False)

    return best_T, inliers


def K_from_camera(camera):
    f = float(camera['focal'])
    return np.array([[f, 0., 0.],
                     [0., f, 0.],
                     [0., 0., 1.]])

def undistort_points(camera, points):
    ''' Undistort image points (2 x N array) with radial distortion
    '''

    num_point = points.shape[1]

    points = points.T.reshape((num_point, 1, 2)).astype(np.float32)

    distortion = np.array([camera['k1'], camera['k2'], 0., 0.])

    K = K_from_camera(camera)
    points_undistort = cv2.undistortPoints(points, K_from_camera(camera), distortion)
    points_undistort = points_undistort.reshape((num_point, 2))
    points_undistort = np.dot(K[0:2,:], homogeneous(points_undistort).T )

    return points_undistort

def two_view_reconstruction_run_csfm(p1, p2, f1, f2, threshold):
    return csfm.two_view_reconstruction(p1, p2, f1, f2, threshold)

def two_view_reconstruction_run_bundle(ba):
    return ba.run()

def two_view_reconstruction(p1, p2, f1, f2, threshold, bundle=True):
    assert len(p1) == len(p2)
    assert len(p1) >= 5
    npoints = len(p1)

    # Run 5-point algorithm.
    res = two_view_reconstruction_run_csfm(p1, p2, f1, f2, threshold)

    num_iter = 10
    if not bundle:
        num_iter = 1

    if res:
        r, t, cov, inliers = res
        print cov
        R = cv2.Rodrigues(r)[0]

        K1 = np.array([[f1, 0, 0], [0, f1, 0], [0, 0, 1.]])
        K2 = np.array([[f2, 0, 0], [0, f2, 0], [0, 0, 1.]])

        old_inliers = np.zeros(npoints)
        errors = [0, 0, 0, 0]
        for it in range(num_iter):
            # Triangulate Features.
            P1 = P_from_KRt(K1, np.eye(3), np.zeros(3))
            P2 = P_from_KRt(K2, R, t)
            Ps = [P1, P2]
            Xs = []
            inliers = []
            for x1, x2 in zip(p1, p2):
                e, X = csfm.triangulate(Ps, [x1, x2], threshold, -1.0)
                errors[e] += 1
                if X is not None:
                    Xs.append(X)
                    inliers.append(True)
                else:
                    Xs.append([0,0,0])
                    inliers.append(False)
            inliers = np.array(inliers)
            Xs = np.array(Xs)

            inlier_changes = np.count_nonzero(inliers - old_inliers)
            old_inliers = inliers.copy()

            if inlier_changes < npoints * 0.05:
                break
            if bundle:
                # Refine R, t
                ba = csfm.BundleAdjuster()
                ba.add_camera('c1', f1, 0, 0, f1, True)
                ba.add_camera('c2', f2, 0, 0, f2, True)
                ba.add_shot('s1', 'c1', 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, True)
                r = cv2.Rodrigues(R)[0].ravel()
                ba.add_shot('s2', 'c2', r[0], r[1], r[2], t[0], t[1], t[2], 0, 0, 0, 1, False)
                for i in xrange(npoints):
                    if inliers[i]:
                        X = Xs[i]
                        ba.add_point(str(i), X[0], X[1], X[2], False)
                        ba.add_observation('s1', str(i), p1[i][0], p1[i][1])
                        ba.add_observation('s2', str(i), p2[i][0], p2[i][1])

                ba.set_loss_function('TruncatedLoss', threshold);
                two_view_reconstruction_run_bundle(ba)
                s = ba.get_shot('s2')
                R = cv2.Rodrigues((s.rx, s.ry, s.rz))[0]
                t = np.array((s.tx, s.ty, s.tz))
                t /= np.linalg.norm(t)
    else:
        return None

    return R, t, Xs, np.nonzero(inliers)[0]


def focal_from_homography(H):
    '''
    Solve for w = H w H^t, with w = diag(a, a, b)
    >>> K = np.diag([0.8, 0.8, 1])
    >>> R = cv2.Rodrigues(np.array([0.3, 0, 0]))[0]
    >>> H = K.dot(R).dot(np.linalg.inv(K))
    >>> f = focal_from_homography(3 * H)
    >>> np.allclose(f, 0.8)
    True
    '''
    H = H / np.linalg.det(H)**(1.0 / 3.0)
    A = np.array([
        [H[0,0] * H[0,0] + H[0,1] * H[0,1] - 1, H[0,2] * H[0,2]    ],
        [H[0,0] * H[1,0] + H[0,1] * H[1,1]    , H[0,2] * H[1,2]    ],
        [H[0,0] * H[2,0] + H[0,1] * H[2,1]    , H[0,2] * H[2,2]    ],
        [H[1,0] * H[1,0] + H[1,1] * H[1,1] - 1, H[1,2] * H[1,2]    ],
        [H[1,0] * H[2,0] + H[1,1] * H[2,1]    , H[1,2] * H[2,2]    ],
        [H[2,0] * H[2,0] + H[2,1] * H[2,1]    , H[2,2] * H[2,2] - 1],
    ])
    _, (a,b) = nullspace(A)
    if a <= 0 or b <= 0:
        return 1.0
    focal = np.sqrt(a / b)
    return focal

def R_from_homography(H, f1, f2):
    K1 = np.diag([f1, f1, 1])
    K2 = np.diag([f2, f2, 1])
    K2inv = np.linalg.inv(K2)
    R = K2inv.dot(H).dot(K1)
    R = project_to_rotation_matrix(R)
    return R

def count_focal_homography_inliers(f1, f2, H, p1, p2, threshold=0.02):
    R = R_from_homography(f1, f2, H)
    if R is None:
        return 0
    H = K1.dot(R).dot(K2inv)
    return count_homography_inliers(H, p1, p2, threshold)

def count_homography_inliers(H, p1, p2, threshold=0.02):
    p2map = euclidean(H.dot(homogeneous(p1).T).T)
    d = p2 - p2map
    return np.sum((d * d).sum(axis=1) < threshold**2)

def project_to_rotation_matrix(A):
    try:
        u, d, vt = np.linalg.svd(A)
    except np.linalg.linalg.LinAlgError:
        return None
    return u.dot(vt)
