from numpy.linalg import qr
import numpy as np
import random
import math
import transformations as tf

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
    return math.acos(min(1, vector_cos(u, v)))

def vector_cos(u, v):
    return np.dot(u, v) / np.linalg.norm(u) / np.linalg.norm(v)


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
