import cv2
import itertools
import logging
import networkx as nx
import numpy as np
import scipy.spatial as spatial

from collections import namedtuple
from networkx.algorithms import bipartite

from opensfm import align
from opensfm import context
from opensfm import csfm
from opensfm import dataset
from opensfm import geo

logger = logging.getLogger(__name__)


PartialReconstruction = namedtuple("PartialReconstruction", ["submodel_path", "index"])


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


def connected_reconstructions(reconstruction_shots):
    g = nx.Graph()
    for r in reconstruction_shots:
        g.add_node(r, bipartite=0)
        for shot_id in reconstruction_shots[r]:
            g.add_node(shot_id, bipartite=1)
            g.add_edge(r, shot_id)

    p = bipartite.projected_graph(g, reconstruction_shots.keys())

    return p.edges()


def corresponding_tracks(tracks1, tracks2):
    features1 = {tracks1[t1]["feature_id"]: t1 for t1 in tracks1}
    corresponding_tracks = []
    for t2 in tracks2:
        feature_id = tracks2[t2]["feature_id"]
        if feature_id in features1:
            corresponding_tracks.append((features1[feature_id], t2))

    return corresponding_tracks


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


def encode_reconstruction_name(key):
    return str(key.submodel_path) + "_index" + str(key.index)


def add_camera_constraints(ra, reconstruction_shots):
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


def add_point_constraints(ra, reconstruction_shots):
    connections = connected_reconstructions(reconstruction_shots)
    for connection in connections:
        d1 = dataset.DataSet(connection[0].submodel_path)
        d2 = dataset.DataSet(connection[1].submodel_path)

        r1 = d1.load_reconstruction()[connection[0].index]
        r2 = d2.load_reconstruction()[connection[1].index]

        common_ims = set(r1.shots.keys()).intersection(r2.shots.keys())

        g1 = d1.load_tracks_graph()
        g2 = d2.load_tracks_graph()

        rec_name1 = encode_reconstruction_name(connection[0])
        rec_name2 = encode_reconstruction_name(connection[1])

        common_tracks = set()
        for im in common_ims:
            for t1, t2 in corresponding_tracks(g1[im], g2[im]):
                if t1 in r1.points and t2 in r2.points:
                    common_tracks.add((t1, t2))

        for t1, t2 in common_tracks:
            c1 = r1.points[t1].coordinates
            c2 = r2.points[t2].coordinates

            ra.add_common_point_constraint(
                rec_name1, c1[0], c1[1], c1[2],
                rec_name2, c2[0], c2[1], c2[2],
                1e-1)


def load_reconstruction_shots(meta_data):
    reconstruction_shots = {}
    for submodel_path in meta_data.get_submodel_paths():
        data = dataset.DataSet(submodel_path)
        if not data.reconstruction_exists():
            continue

        reconstruction = data.load_reconstruction()
        for index, partial_reconstruction in enumerate(reconstruction):
            key = PartialReconstruction(submodel_path, index)
            reconstruction_shots[key] = partial_reconstruction.shots

    return reconstruction_shots


def align_reconstructions(reconstruction_shots):
    ra = csfm.ReconstructionAlignment()

    add_camera_constraints(ra, reconstruction_shots)
    add_point_constraints(ra, reconstruction_shots)

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


def apply_transformations(transformations):
    submodels = itertools.groupby(transformations.keys(), lambda key: key.submodel_path)
    for submodel_path, keys in submodels:
        data = dataset.DataSet(submodel_path)
        if not data.reconstruction_exists():
            continue

        reconstruction = data.load_reconstruction()
        for key in keys:
            partial_reconstruction = reconstruction[key.index]
            s, A, b = transformations[key]
            align.apply_similarity(partial_reconstruction, s, A, b)

        data.save_reconstruction(reconstruction, 'reconstruction.aligned.json')
