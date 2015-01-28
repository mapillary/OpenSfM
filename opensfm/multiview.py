# -*- coding: utf-8 -*-

from subprocess import call, Popen, PIPE
import numpy as np
from numpy.linalg import qr
import random
import math
import cv2
from opensfm import transformations as tf
from opensfm import context

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
    max_iterations = 10000
    best_error = float('inf')
    best_model = None
    best_inliers = None
    i = 0
    while i < max_iterations:
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


def two_view_reconstruction(p1, p2, d1, d2, config):
    '''Computes a two view reconstruction from a set of matches.
    '''
    s = ''
    for l in np.hstack((p1, p2)):
        s += ' '.join(str(i) for i in l) + '\n'

    params = [context.TWO_VIEW_RECONSTRUCTION,
              '-threshold', str(config.get('five_point_algo_threshold', 0.006)),
              '-focal1', d1['focal_ratio'],
              '-focal2', d2['focal_ratio']]
    params = map(str, params)

    p = Popen(params, stdout=PIPE, stdin=PIPE, stderr=PIPE)
    res = p.communicate(input=s)[0]
    if not res:
        return None, None, None, None
    res = res.split(None, 9 + 3)
    Rt_res = map(float, res[:-1])
    inliers_res = res[-1]
    R = np.array(Rt_res[:9]).reshape(3,3)
    t = np.array(Rt_res[9:])

    inliers = []
    Xs = []
    for line in inliers_res.splitlines():
        words = line.split()
        inliers.append(int(words[0]))
        Xs.append(map(float, words[1:]))

    return R, t, inliers, Xs


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
    s = 1. / points.std()           # Normalize the scale to improve conditioning.
    x = homogeneous(s * points)
    if vectors:
        v = homogeneous_vec(s * np.array(vectors))
        A = np.vstack((x, v))
    else:
        A = x
    _, p = nullspace(A)
    p[3] /= s

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
    return tf.rotation_matrix(tf.angle_between_vectors(v0, v1),
                              tf.vector_product(v0, v1)
                              )[:3,:3]


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
