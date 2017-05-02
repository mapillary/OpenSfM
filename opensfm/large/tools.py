import cv2
import logging
import numpy as np
import scipy.spatial as spatial

from opensfm import context
from opensfm import csfm
from opensfm import geo

logger = logging.getLogger(__name__)


def kmeans(samples, nclusters, max_iter=100, attempts=20):
    criteria = (cv2.TERM_CRITERIA_MAX_ITER, max_iter, 1.0)
    flags = cv2.KMEANS_PP_CENTERS

    if context.OPENCV3:
        return cv2.kmeans(samples, nclusters, None, criteria, attempts, flags)
    else:
        return cv2.kmeans(samples, nclusters, criteria, attempts, flags)


def add_cluster_neighbors(positions, labels, centers, max_distance):
    reference = np.mean(positions, 0)

    topocentrics = []
    for position in positions:
        x, y, z = geo.topocentric_from_lla(
            position[0],
            position[1],
            0,
            reference[0],
            reference[1],
            0)

        topocentrics.append([x, y])

    topocentrics = np.array(topocentrics)
    topo_tree = spatial.cKDTree(topocentrics)

    clusters = []
    for label in np.arange(centers.shape[0]):
        cluster_indices = np.where(labels.ravel() == label)[0]

        neighbors = []
        for i in cluster_indices:
            neighbors.extend(
                topo_tree.query_ball_point(topocentrics[i], max_distance))

        cluster = list(np.union1d(cluster_indices, neighbors))
        clusters.append(cluster)

    return clusters


def scale_matrix(covariance):
    try:
        L = np.linalg.cholesky(covariance)
    except Exception as e:
        logger.error(
            'Could not compute Cholesky of covariance matrix {}'
                .format(covariance))

        d = np.diag(np.diag(covariance).clip(1e-8, None))
        L = np.linalg.cholesky(d)

    return np.linalg.inv(L)


def invert_similarity(s, A, b):
    s_inv = 1 / s
    A_inv = A.T
    b_inv = -s_inv * A_inv.dot(b)

    return s_inv, A_inv, b_inv


def align_reconstructions(reconstruction_shots):
    def encode_reconstruction_name(key):
        return str(key[0]) + "_index" + str(key[1])

    ra = csfm.ReconstructionAlignment()
    added_shots = set()
    for key in reconstruction_shots:
        shots = reconstruction_shots[key]
        rec_name = encode_reconstruction_name(key)
        ra.add_reconstruction(rec_name, 0, 0, 0, 0, 0, 0, 1, False)
        for shot_id in shots:
            shot = shots[shot_id]
            shot_name = str(shot_id)

            R = shot.pose.rotation
            t = shot.pose.translation

            if shot_id not in added_shots:
                ra.add_shot(shot_name, R[0], R[1], R[2], t[0], t[1], t[2], False)

                gps = shot.metadata.gps_position
                gps_sd = shot.metadata.gps_dop
                ra.add_absolute_position_constraint(
                    shot_name, gps[0], gps[1], gps[2], gps_sd)

                added_shots.add(shot_id)

            covariance = np.diag([1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2])
            sm = scale_matrix(covariance)
            rmc = csfm.RARelativeMotionConstraint(
                rec_name, shot_name, R[0], R[1], R[2], t[0], t[1], t[2])

            for i in range(6):
                for j in range(6):
                    rmc.set_scale_matrix(i, j, sm[i, j])

            ra.add_relative_motion_constraint(rmc)

    ra.run()

    transformations = {}
    for key in reconstruction_shots:
        rec_name = encode_reconstruction_name(key)
        r = ra.get_reconstruction(rec_name)
        s = r.scale
        A = cv2.Rodrigues(np.array([r.rx, r.ry, r.rz]))[0]
        b = np.array([r.tx, r.ty, r.tz])
        transformations[key] = invert_similarity(s, A, b)

    return transformations
