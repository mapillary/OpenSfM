import data_generation
import numpy as np

from opensfm import reconstruction
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
    status, report = reconstruction.resect(synthetic_tracks, synthetic_data, shot_id, camera, metadata,
                          parameters['resection_threshold'], parameters['resection_min_inliers'])
    shot_after = synthetic_data.shots[shot_id]

    assert status is True
    assert report['num_inliers'] is len(synthetic_data.points)
    np.testing.assert_almost_equal(
        shot_before.pose.rotation, shot_after.pose.rotation, 1)
    np.testing.assert_almost_equal(
        shot_before.pose.translation, shot_after.pose.translation, 1)
