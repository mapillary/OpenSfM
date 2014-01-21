from scipy.linalg import qr
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


def Preconditioner(x):
    '''Return an isotropic preconditioner.

    HZ 4.4.4 pag.109: Point conditioning (isotropic)
    
    >>> x = np.array([[0, 0],
    ...               [1, 1],
    ...               [0, 1],
    ...               [1, 0]])
    >>> T = Preconditioner(x)
    >>> y = euclidean(T.dot(homogeneous(x).T).T)
    >>> np.allclose(y.mean(axis=0), 0)
    True
    >>> np.allclose(y.var(), 2)
    True
    '''
    d = x.shape[1]
    mean = x.mean(axis=0)
    variance = x.var()
    scale = math.sqrt(2.0 / variance);

    T = np.diag([scale] * d + [1])
    T[:-1,-1] = - scale * mean
    return T


def pixel_direction(K, x):
    '''Find a vector such that x = K v and v[2] = 1.

    >>> v = [2, 3, 1]
    >>> K = [[6, 1, 3], [0, 5, 3], [0, 0, 1]]
    >>> x = np.dot(K, v)[:2]
    >>> estimated_v = pixel_direction(K, x)
    >>> np.allclose(estimated_v, v)
    True
    '''
    v = np.linalg.solve(K, [x[0], x[1], 1])
    return v / v[2]

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


def rotation_matrix(angleaxis):
    angle = np.linalg.norm(angleaxis)
    if angle < 1e-8:
        return np.eye(3)
    return tf.rotation_matrix(angle, angleaxis)[:-1, :-1]


def rotate(angleaxis, point):
    R = rotation_matrix(angleaxis)
    return R.dot(np.array(point))


def AbsoluteOrientation3points(X, Xp):
    '''The absolute orientation algorithm recovers the transformation between
    3 3D points, X and Xp such that:

              Xp = R * X + t

    The recovery of the absolute orientation is implemented after:
    Horn, "Closed-form solution of absolute orientation using
    orthonormal matrices" section 5.A

    >>> Xs = [[ 1, 0, 0], [ 0, 1, 0], [ -1, 0, 0]]
    >>> R = rotation_matrix([0.0,0.2,0.0])
    >>> t = [0, 0, 0]
    >>> Xps = [np.dot(R, X) + t for X in Xs]
    >>> estimated_R , estimated_t = AbsoluteOrientation3points(Xs, Xps)
    >>> np.allclose(estimated_R, R)
    True
    >>> np.allclose(estimated_t, t)
    True
    '''
    X = np.array(X)
    Xp = np.array(Xp)
    Xmean = X.mean(axis=0)
    Xpmean = Xp.mean(axis=0)
    Xn = X - Xmean
    Xpn = Xp - Xpmean

    nl = np.cross(Xn[2] - Xn[0], Xn[1] - Xn[0])
    nr = np.cross(Xpn[2] - Xpn[0], Xpn[1] - Xpn[0])
    nl = nl / np.linalg.norm(nl)
    nr = nr / np.linalg.norm(nr)

    # Rotation to match planes
    a = np.cross(nl, nr)
    norm_a = np.linalg.norm(a)
    # TODO(pau): compute also cos(phi) and phi from atan2
    phi = math.asin(norm_a)
    if norm_a > 1e-8:
        angleaxis1 = phi * a / norm_a
    else:
        angleaxis1 = [0,0,0]
    Xrot = [rotate(angleaxis1, i) for i in Xn]

    alpha = [vector_angle(i, j) for i, j in zip(Xpn, Xrot)]

    C = sum(i.dot(j) for i, j in zip(Xpn, Xrot))
    S = sum(np.cross(i, j) for i, j in zip(Xpn, Xrot)).dot(nr)
    # TODO(pau): check the sign of atan. It may not be always negative.
    theta = -math.atan2(S, C)
    angleaxis2 = theta * nr

    R1 = rotation_matrix(angleaxis1)
    R2 = rotation_matrix(angleaxis2)
    R = R2.dot(R1)

    # Compute the final translation.
    t = Xpmean - R.dot(Xmean)

    # print 'phi', phi
    # print 'a', a
    # print 'angleaxis1', angleaxis1
    # print 'Xrot', Xrot
    # print 'alpha', alpha
    # print 'theta', theta
    # print 'X', X
    # print 'C', C
    # print 't', t

    return R, t





# def AbsoluteOrientation(X, Xp):
#     '''The absolute orientation algorithm recovers the transformation between
#     a set of 3D points, X and Xp such that:

#               Xp = R * X + t

#     The recovery of the absolute orientation is implemented after:
#     Horn, Hilden, "Closed-form solution of absolute orientation using
#     orthonormal matrices"

#     Copied from libmv/multiview/euclidean_resection.cc

#     >>> Xs = [[ 1, 0, 0], [ 0, 1, 0], [ -1, 0, 0], [0,-1,0], [0,0,-1], [0,0,1]]
#     >>> R = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
#     >>> t = [0, 0, 0]
#     >>> Xps = [np.dot(R, X) + t for X in Xs]
#     >>> Xps
#     >>> estimated_R, estimated_t = AbsoluteOrientation(Xs, Xps)
#     >>> estimated_R, estimated_t
#     >>> np.allclose(estimated_R, R)
#     True
#     >>> np.allclose(estimated_t, t)
#     True
#     '''
#     X = np.array(X)
#     Xp = np.array(Xp)
#     C = X.mean(axis=0)
#     Cp = Xp.mean(axis=0)
#     Xn = X - C
#     Xpn = Xp - Cp

#     # Construct the N matrix (pg. 635).
#     Sxx = Xn[0].dot(Xpn[0])
#     Syy = Xn[1].dot(Xpn[1])
#     Szz = Xn[2].dot(Xpn[2])
#     Sxy = Xn[0].dot(Xpn[1])
#     Syx = Xn[1].dot(Xpn[0])
#     Sxz = Xn[0].dot(Xpn[2])
#     Szx = Xn[2].dot(Xpn[0])
#     Syz = Xn[1].dot(Xpn[2])
#     Szy = Xn[2].dot(Xpn[1])

#     N = np.array([
#          [Sxx + Syy + Szz, Syz - Szy,        Szx - Sxz,        Sxy - Syx],
#          [Syz - Szy,       Sxx - Syy - Szz,  Sxy + Syx,        Szx + Sxz],
#          [Szx - Sxz,       Sxy + Syx,       -Sxx + Syy - Szz,  Syz + Szy],
#          [Sxy - Syx,       Szx + Sxz,        Syz + Szy,       -Sxx - Syy + Szz]
#     ])

#     # Find the unit quaternion q that maximizes qNq. It is the eigenvector
#     # corresponding to the lagest eigenvalue.
#     w, v = np.linalg.eig(N)
#     a, b, c, d = v[:, w.argmax()]

#     # Retrieve the 3x3 rotation matrix from unit quaternion.
#     R = np.array([[a*a + b*b - c*c - d*d, 2*b*c - 2*a*d, 2*b*d + 2*a*c],
#                   [2*b*c + 2*a*d, a*a - b*b + c*c - d*d, 2*c*d - 2*a*b],
#                   [2*b*d - 2*a*c, 2*c*d + 2*a*b, a*a - b*b - c*c + d*d]])

#     # Fix the handedness of the R matrix.
#     if np.linalg.det(R) < 0:
#         R[2] *= -1

#     # Compute the final translation.
#     t = Cp - R.dot(C)

#     return R, t


def resection_p3p(xs, Xs, K):
    '''Find R and t such that x = K[R t]X

    See Apendix A of  Ransac paper (Fischler 1981).

    >>> Xs = np.array([[-1, 0, 1], [ 1, 0, 1], [ 0, 1, 1]])
    >>> K = [[6, 1, 3], [0, 5, 3], [0, 0, 1]]
    >>> R = rotation_matrix([0.8, 0.3, 0.1])
    >>> t = np.array([1, 0, 0])
    >>> P = P_from_KRt(K, R, t)
    >>> xs = [euclidean(np.dot(P, homogeneous(X))) for X in Xs]
    >>> Rts = resection_p3p(xs, Xs, K)
    >>> found = False
    >>> for estimated_R, estimated_t in Rts:
    ...     if np.allclose(estimated_R, R, atol=1e-6) and np.allclose(estimated_t, t, atol=1e-6):
    ...         found = True
    >>> found
    True
    '''
    Xs = np.array(Xs)
    Rab = np.linalg.norm(Xs[0] - Xs[1])
    Rbc = np.linalg.norm(Xs[1] - Xs[2])
    Rac = np.linalg.norm(Xs[0] - Xs[2])

    if Rab < 1e-6 or Rbc < 1e-6 or Rac < 1e-6:
        return []

    va = pixel_direction(K, xs[0])
    vb = pixel_direction(K, xs[1])
    vc = pixel_direction(K, xs[2])
    cosab = vector_cos(va, vb)
    cosbc = vector_cos(vb, vc)
    cosac = vector_cos(va, vc)

    K1 = (Rbc**2) / (Rac**2)
    K2 = (Rbc**2) / (Rab**2)

    G4 = (K1 * K2 - K1 - K2)**2 - 4 * K1 * K2 * (cosbc**2)
    G3 = (4 * (K1 * K2 - K1 - K2) * K2 * (1 - K1) * cosab
        + 4 * K1 * cosbc * ((K1 * K2 + K2 - K1) * cosac 
            + 2 * K2 * cosab * cosbc))
    G2 = ((2 * K2 * (1 - K1) * cosab)**2
        + 2 * (K1 * K2 + K1 - K2) * (K1 * K2 - K1 - K2)
        + 4 * K1 * ((K1 - K2) * (cosbc**2) + (1 - K2) * K1 * (cosac**2)
        - 2 * K2 * (1 + K1) * cosab * cosac * cosbc))
    G1 = (4 * (K1 * K2 + K1 - K2) * K2 * (1 - K1) * cosab
        + 4 * K1 * ((K1 * K2 - K1 + K2) * cosac * cosbc
        + 2 * K1 * K2 * cosab * (cosac**2)))
    G0 = (K1 * K2 + K1 - K2)**2 - 4 * (K1**2) * K2 * (cosac**2)

    roots = np.roots([G4, G3, G2, G1, G0])
    xs = [x.real for x in roots if x.imag < 1e-7]

    # print 'roots', roots
    # print 'xs', xs

    abcs = []
    for x in xs:
        if x > 0:
            a = Rab / math.sqrt((x**2) - 2 * x * cosab + 1)
            b = a * x
            for sign in [-1, 1]:
                D = cosac**2 + ((Rac**2) - (a**2)) / (a**2)
                if D >= 0:
                    y = cosac + sign * math.sqrt(D)
                    c = y * a
                    abcs.append((a,b,c))
    #print abcs

    solutions = []
    for a, b, c in abcs:
        Xa = a * va / np.linalg.norm(va)
        Xb = b * vb / np.linalg.norm(vb)
        Xc = c * vc / np.linalg.norm(vc)
        #print 'Xs', np.array(Xs)
        #print 'Xa', np.array([Xa, Xb, Xc])
        R, t = AbsoluteOrientation3points(Xs, [Xa, Xb, Xc])
        solutions.append((R, t))
    return solutions


class ResectionP3PKernel:
    '''A kernel for the resection problem x = K(R t) * X with known K

    >>> K = [[6, 1, 3], [0, 5, 3], [0, 0, 1]]
    >>> R = rotation_matrix([0.8, 0.3, 0.1])
    >>> t = np.array([1, 2, 3])
    >>> P = P_from_KRt(K, R, t)
    >>> X = np.random.rand(10, 3)
    >>> x = euclidean(np.dot(P, homogeneous(X).T).T)
    >>> kernel = ResectionP3PKernel(x, X, K)
    >>> Rts = kernel.fit(range(3))
    >>> found = False
    >>> for estimated_R, estimated_t in Rts:
    ...     if np.allclose(estimated_R, R, atol=1e-6) and np.allclose(estimated_t, t, atol=1e-6):
    ...         found = True
    ...         break
    >>> found
    True
    >>> kernel.evaluate((R, t))
    '''
    required_samples = 3

    def __init__(self, x, X, K):
        self.x = x if x.shape[1] == 2 else euclidean(x)
        self.X = X if X.shape[1] == 3 else euclidean(X)
        self.K = K

    def num_samples(self):
        return len(self.x)

    def fit(self, samples):
        x = self.x[samples]
        X = self.X[samples]
        return resection_p3p(x, X, self.K)

    def evaluate(self, model):
        R, t = model
        P = P_from_KRt(self.K, R, t)
        x_estimated = np.dot(P, homogeneous(self.X).T).T
        d = self.x - euclidean(x_estimated)
        return (d * d).sum(axis=1)


def resection_linear(x, X):
    '''Find P such that x = PX.

    >>> P = np.random.rand(3, 4)
    >>> X = np.random.rand(10, 4)
    >>> x = np.dot(P, X.T).T
    >>> P_estimated = resection_linear(x, X)
    >>> is_proportional(P, P_estimated)
    True
    '''
    T = Preconditioner(euclidean(x))
    U = Preconditioner(euclidean(X))
    x = T.dot(x.T).T
    X = U.dot(X.T).T
    n = len(x)
    A = np.zeros((2 * n, 12))
    for i in range(n):
        A[2 * i    , 4:8 ] = -x[i, 2] * X[i]
        A[2 * i    , 8:12] =  x[i, 1] * X[i]
        A[2 * i + 1, 0:4 ] =  x[i, 2] * X[i]
        A[2 * i + 1, 8:12] = -x[i, 0] * X[i]
    s, p = nullspace(A)
    P = p.reshape((3,4))
    return np.linalg.inv(T).dot(P.dot(U))


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

