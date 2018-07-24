import data_generation
import numpy as np

from opensfm import reconstruction
from opensfm import multiview
from opensfm import config
from opensfm import types


def synthetic_reconstruction():
    cube_dataset = data_generation.CubeDataset(10, 100, 0.001, 0.3)
    synthetic_reconstruction = types.Reconstruction()
    for shot in cube_dataset.shots.values():
        synthetic_reconstruction.add_shot(shot)
    for camera in cube_dataset.cameras.values():
        synthetic_reconstruction.add_camera(camera)
    for point_id, point in cube_dataset.points.iteritems():
        point_type = types.Point()
        point_type.coordinates = point
        point_type.id = point_id
        synthetic_reconstruction.add_point(point_type)
    return synthetic_reconstruction, cube_dataset.tracks


def copy_cluster_points(cluster, tracks, points, noise):
    for shot in cluster.shots:
        for point in tracks[shot]:
            base = points[point]
            copy = types.Point()
            copy.id = base.id
            copy.coordinates = base.coordinates+np.random.rand()*noise
            cluster.add_point(copy)
    return cluster


def split_synthetic_reconstruction(synthetic_reconstruction,
                                   synthetic_tracks,
                                   cluster_size,
                                   point_noise):
    cluster1 = types.Reconstruction()
    cluster2 = types.Reconstruction()
    cluster1.cameras = synthetic_reconstruction.cameras
    cluster2.cameras = synthetic_reconstruction.cameras
    for (i, shot) in zip(range(len(synthetic_reconstruction.shots)),
                         synthetic_reconstruction.shots.values()):
        if(i >= cluster_size):
            cluster2.add_shot(shot)
        else:
            cluster1.add_shot(shot)

    cluster1 = copy_cluster_points(
        cluster1, synthetic_tracks, synthetic_reconstruction.points,
        point_noise)
    cluster2 = copy_cluster_points(
        cluster2, synthetic_tracks, synthetic_reconstruction.points,
        point_noise)
    return cluster1, cluster2


def move_and_scale_cluster(cluster):
    scale = np.random.rand(1)
    translation = np.random.rand(3)
    for point in cluster.points.values():
        point.coordinates = scale*point.coordinates + translation
    return cluster, translation, scale


def test_absolute_pose_single_shot():
    """ Single-camera resection on a toy reconstruction with """
    """ 1/1000 pixel noise and zero outliers """
    parameters = config.default_config()
    synthetic_data, synthetic_tracks = synthetic_reconstruction()

    shot_id = 'shot1'
    camera_id = 'camera1'
    metadata = types.ShotMetadata()
    camera = synthetic_data.cameras[camera_id]

    shot_before = synthetic_data.shots[shot_id]
    status, report = reconstruction.resect(synthetic_tracks, synthetic_data,
                                           shot_id, camera, metadata,
                                           parameters['resection_threshold'],
                                           parameters['resection_min_inliers'])
    shot_after = synthetic_data.shots[shot_id]

    assert status is True
    assert report['num_inliers'] is len(synthetic_data.points)
    np.testing.assert_almost_equal(
        shot_before.pose.rotation, shot_after.pose.rotation, 1)
    np.testing.assert_almost_equal(
        shot_before.pose.translation, shot_after.pose.translation, 1)


def test_absolute_pose_generalized_shot():
    """ Whole reconstruction resection (generalized pose) on a toy """
    """ reconstruction with 1/1000 pixel noise and zero outliers """
    parameters = config.default_config()
    scene, tracks = synthetic_reconstruction()
    cluster1, cluster2 = split_synthetic_reconstruction(scene, tracks, 3, 0.01)
    cluster2, translation, scale = move_and_scale_cluster(cluster2)

    status, report = reconstruction.\
        resect_reconstruction(cluster1, cluster2,
                              tracks, tracks,
                              parameters['resection_threshold'],
                              parameters['resection_min_inliers'])

    assert status is True
    s, A, b = multiview.decompose_similarity_transform(report[0])
    np.testing.assert_almost_equal(scale, s, 2)
    np.testing.assert_almost_equal(np.eye(3), A, 2)
    np.testing.assert_almost_equal(translation, b, 2)
