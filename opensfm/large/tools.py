import cv2
import numpy as np

from opensfm import context


def kmeans(samples, nclusters, criteria, attempts, flags):
    if context.OPENCV3:
        return cv2.kmeans(samples, nclusters, None, criteria, attempts, flags)
    else:
        return cv2.kmeans(samples, nclusters, criteria, attempts, flags)


def scale_matrix(covariance):
    try:
        L = np.linalg.cholesky(covariance)
    except Exception as e:
        logger.error('Could not compute Cholesky of covariance matrix {}'.format(covariance))
        d = np.diag(np.diag(covariance).clip(1e-8, None))
        L = np.linalg.cholesky(d)

    return np.linalg.inv(L)