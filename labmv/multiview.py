from scipy.linalg import qr
import numpy as np
import random
import math


def null_space(A):
    '''Compute the null space of A.

    Return the smallest sigular value and the corresponding vector.
    '''
    u, s, vh = np.linalg.svd(A)
    return s[-1], vh[-1]


def is_proportional(x, y, tol=1e-8):
    '''Check x is proportional to y (i.e. x = a * y for some a).

    >>> x = np.array([3., 4.])
    >>> y = np.array([6., 8.])
    >>> is_proportional(x, y)
    True
    >>> x = np.array([3., 4.])
    >>> y = np.array([-6., -8.])
    >>> is_proportional(x, y)
    True
    >>> x = np.array([3., 4.])
    >>> y = np.array([3., 5.])
    >>> is_proportional(x, y)
    False
    '''
    xn = x / abs(x).sum()
    yn = y / abs(y).sum()
    d1 = xn - yn
    d2 = xn + yn
    return abs(d1).sum() < tol or abs(d2).sum() < tol


def homogeneous(x):
    '''Add a column of ones to x.
    '''
    return np.hstack((x, np.ones((len(x), 1))))


def euclidean(x):
    '''Divide by last column and drop it.
    '''
    return x[:, :-1] / x[:, -1:]


def P_from_KRt(K, R, t):
    '''P = K[R|t].
    '''
    P = np.empty((3, 4))
    P[:, :3] = K.dot(R)
    P[:, 3] = K.dot(t)
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

    T = np.diag(np.sign(np.diag(K)))
    if np.linalg.det(np.dot(T, R)) < 0:
        T[1, 1] *= -1

    K = np.dot(K, T)
    R = np.dot(T, R)
    t = np.linalg.solve(K, P[:,3])
    K /= K[2, 2]
    
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


def resection_linear(x, X):
    '''Find P such that x = PX.

    >>> P = np.random.rand(3, 4)
    >>> X = np.random.rand(10, 4)
    >>> x = np.dot(P, X.T).T
    >>> P_estimated = resection_linear(x, X)
    >>> is_proportional(P, P_estimated)
    True
    '''
    n = len(x)
    D = np.zeros((2 * n, 12))
    for i in range(n):
        D[2 * i    , 4:8 ] = -x[i, 2] * X[i]
        D[2 * i    , 8:12] =  x[i, 1] * X[i]
        D[2 * i + 1, 0:4 ] =  x[i, 2] * X[i]
        D[2 * i + 1, 8:12] = -x[i, 0] * X[i]
    s, p = null_space(D)
    return p.reshape((3,4))


class ResectionLinearKernel:
    '''A kernel for the resection problem x = P * X
    >>> P = np.random.rand(3, 4)
    >>> X = np.random.rand(10, 4)
    >>> x = np.dot(P, X.T).T
    >>> kernel = ResectionLinearKernel(x, X)
    >>> P_estimated = kernel.fit(range(6))[0]
    >>> is_proportional(P, P_estimated)
    True
    >>> errors = kernel.evaluate(P_estimated)
    >>> (errors > 10e-8).any()
    False
    '''
    required_samples = 6

    def __init__(self, x, X):
        self.x = x if x.shape[1] == 3 else homogeneous(x)
        self.X = X if X.shape[1] == 4 else homogeneous(X)

    def num_samples(self):
        return len(self.x)

    def fit(self, samples):
        x = self.x[samples]
        X = self.X[samples]
        return [resection_linear(x, X)]

    def evaluate(self, model):
        x_estimated = model.dot(self.X.T).T
        d = euclidean(self.x) - euclidean(x_estimated)
        return (d * d).sum(axis=1)


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
    max_iterations = 1000 # TODO(pau) compute from num_inliers.
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
            if error < best_error:
                best_error = error
                best_model = model
                best_inliers = inliers
                max_iterations = ransac_max_iterations(kernel, best_inliers, 0.01)
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

