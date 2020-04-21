import numpy as np
import networkx as nx

from opensfm import reconstruction
from opensfm import multiview
from opensfm import config
from opensfm import types
from opensfm.synthetic_data import synthetic_scene


def test_corresponding_tracks():
    t1 = {1: {"feature_id": 1}}
    t2 = {1: {"feature_id": 2}}

    correspondences = reconstruction.corresponding_tracks(t1, t2)
    assert len(correspondences) == 0

    t1 = {1: {"feature_id": 3}}
    t2 = {2: {"feature_id": 3}}

    correspondences = reconstruction.corresponding_tracks(t1, t2)
    assert len(correspondences) == 1
    assert correspondences[0] == (1, 2)

    t1 = {1: {"feature_id": 3}, 2: {"feature_id": 4}}
    t2 = {1: {"feature_id": 4}, 2: {"feature_id": 5}}

    correspondences = reconstruction.corresponding_tracks(t1, t2)
    assert len(correspondences) == 1
    assert correspondences[0] == (2, 1)

    t1 = {1: {"feature_id": 5}, 2: {"feature_id": 6}}
    t2 = {3: {"feature_id": 5}, 4: {"feature_id": 6}}

    correspondences = reconstruction.corresponding_tracks(t1, t2)
    correspondences.sort(key=lambda c: c[0] + c[1])
    assert len(correspondences) == 2
    assert correspondences[0] == (1, 3)
    assert correspondences[1] == (2, 4)


def copy_cluster_points(cluster, tracks, points, noise):
    for shot in cluster.shots:
        for point in tracks[shot]:
            base = points[point]
            copy = types.Point()
            copy.id = base.id
            copy.coordinates = base.coordinates+np.random.rand()*noise
            cluster.add_point(copy)
    return cluster


def split_synthetic_reconstruction(scene, tracks,
                                   cluster_size,
                                   point_noise):
    cluster1 = types.Reconstruction()
    cluster2 = types.Reconstruction()
    cluster1.cameras = scene.cameras
    cluster2.cameras = scene.cameras
    for (i, shot) in zip(range(len(scene.shots)),
                         scene.shots.values()):
        if(i >= cluster_size):
            cluster2.add_shot(shot)
        if(i <= cluster_size):
            cluster1.add_shot(shot)

    cluster1 = copy_cluster_points(
        cluster1, tracks, scene.points,
        point_noise)
    cluster2 = copy_cluster_points(
        cluster2, tracks, scene.points,
        point_noise)
    return cluster1, cluster2


def move_and_scale_cluster(cluster):
    scale = np.random.rand(1)
    translation = np.random.rand(3)
    for point in cluster.points.values():
        point.coordinates = scale*point.coordinates + translation
    return cluster, translation, scale


def test_absolute_pose_generalized_shot(scene_synthetic_cube):
    """Whole reconstruction resection (generalized pose) on a toy
    reconstruction with 0.01 meter point noise and zero outliers."""
    noise = 0.01
    parameters = config.default_config()
    scene, tracks = scene_synthetic_cube
    cluster1, cluster2 = split_synthetic_reconstruction(
        scene, tracks, 3, noise)
    cluster2, translation, scale = move_and_scale_cluster(cluster2)

    status, T, inliers = reconstruction.\
        resect_reconstruction(cluster1, cluster2,
                              tracks, tracks,
                              2*noise,
                              parameters['resection_min_inliers'])

    assert status is True
    s, A, b = multiview.decompose_similarity_transform(T)
    np.testing.assert_almost_equal(scale, s, 2)
    np.testing.assert_almost_equal(np.eye(3), A, 2)
    np.testing.assert_almost_equal(translation, b, 2)
