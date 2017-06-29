import logging
from multiprocessing import Pool

import cv2
import numpy as np

from opensfm import dataset
from opensfm import features
from opensfm import transformations as tf
from opensfm import types

logger = logging.getLogger(__name__)


class Command:
    name = 'undistort'
    help = "Save radially undistorted images"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()
        graph = data.load_tracks_graph()

        if reconstructions:
            self.undistort_images(graph, reconstructions[0], data)

        data.save_undistorted_tracks_graph(graph)

    def undistort_images(self, graph, reconstruction, data):
        urec = types.Reconstruction()
        urec.points = reconstruction.points

        logger.debug('Undistorting the reconstruction')
        undistorted_shots = {}
        for shot in reconstruction.shots.values():
            if shot.camera.projection_type == 'perspective':
                urec.add_camera(shot.camera)
                urec.add_shot(shot)
                undistorted_shots[shot.id] = [shot]
            elif shot.camera.projection_type == 'fisheye':
                shot.camera = perspective_camera_from_fisheye(shot.camera)
                urec.add_camera(shot.camera)
                urec.add_shot(shot)
                undistorted_shots[shot.id] = [shot]
            elif shot.camera.projection_type in ['equirectangular', 'spherical']:
                subshot_width = int(data.config['depthmap_resolution'])
                subshots = perspective_views_of_a_panorama(shot, subshot_width)
                for subshot in subshots:
                    urec.add_camera(subshot.camera)
                    urec.add_shot(subshot)
                    add_subshot_tracks(graph, shot, subshot)
                undistorted_shots[shot.id] = subshots
        data.save_undistorted_reconstruction([urec])

        arguments = []
        for shot in reconstruction.shots.values():
            arguments.append((shot, undistorted_shots[shot.id], data))

        processes = data.config['processes']
        if processes == 1:
            for arg in arguments:
                undistort_image(arg)
        else:
            p = Pool(processes)
            p.map(undistort_image, arguments)


def undistort_image(arguments):
    shot, undistorted_shots, data = arguments
    logger.debug('Undistorting image {}'.format(shot.id))

    if shot.camera.projection_type == 'perspective':
        image = data.image_as_array(shot.id)
        undistorted = undistort_perspective_image(image, shot.camera)
        data.save_undistorted_image(shot.id, undistorted)
    elif shot.camera.projection_type == 'fisheye':
        image = data.image_as_array(shot.id)
        undistorted = undistort_fisheye_image(image, shot.camera)
        data.save_undistorted_image(shot.id, undistorted)
    elif shot.camera.projection_type in ['equirectangular', 'spherical']:
        original = data.image_as_array(shot.id)
        subshot_width = int(data.config['depthmap_resolution'])
        width = 4 * subshot_width
        height = width / 2
        image = cv2.resize(original, (width, height), interpolation=cv2.INTER_AREA)
        for subshot in undistorted_shots:
            undistorted = render_perspective_view_of_a_panorama(
                image, shot, subshot)
            data.save_undistorted_image(subshot.id, undistorted)


def undistort_perspective_image(image, camera):
    """Remove radial distortion from a perspective image."""
    height, width = image.shape[:2]
    K = camera.get_K_in_pixel_coordinates(width, height)
    distortion = np.array([camera.k1, camera.k2, 0, 0])
    return cv2.undistort(image, K, distortion)


def undistort_fisheye_image(image, camera):
    """Remove radial distortion from a perspective image."""
    height, width = image.shape[:2]
    K = camera.get_K_in_pixel_coordinates(width, height)
    distortion = np.array([camera.k1, camera.k2, 0, 0])
    return cv2.fisheye.undistortImage(image, K, distortion, K)


def perspective_camera_from_fisheye(fisheye):
    """Create a perspective camera from a fisheye."""
    camera = types.PerspectiveCamera()
    camera.id = fisheye.id
    camera.width = fisheye.width
    camera.height = fisheye.height
    camera.focal = fisheye.focal
    camera.focal_prior = fisheye.focal_prior
    camera.k1 = camera.k1_prior = camera.k2 = camera.k2_prior = 0.0
    return camera


def perspective_views_of_a_panorama(spherical_shot, width):
    """Create 6 perspective views of a panorama."""
    camera = types.PerspectiveCamera()
    camera.id = 'perspective_panorama_camera'
    camera.width = width
    camera.height = width
    camera.focal = 0.5
    camera.focal_prior = camera.focal
    camera.k1 = camera.k1_prior = camera.k2 = camera.k2_prior = 0.0

    names = ['front', 'left', 'back', 'right', 'top', 'bottom']
    rotations = [
        tf.rotation_matrix(-0 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-1 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-2 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-3 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-np.pi / 2, (1, 0, 0)),
        tf.rotation_matrix(+np.pi / 2, (1, 0, 0)),
    ]
    shots = []
    for name, rotation in zip(names, rotations):
        shot = types.Shot()
        shot.id = '{}_perspective_view_{}'.format(spherical_shot.id, name)
        shot.camera = camera
        R = np.dot(rotation[:3, :3], spherical_shot.pose.get_rotation_matrix())
        o = spherical_shot.pose.get_origin()
        shot.pose = types.Pose()
        shot.pose.set_rotation_matrix(R)
        shot.pose.set_origin(o)
        shots.append(shot)
    return shots


def render_perspective_view_of_a_panorama(image, panoshot, perspectiveshot):
    """Render a perspective view of a panorama."""
    # Get destination pixel coordinates
    dst_shape = (perspectiveshot.camera.height, perspectiveshot.camera.width)
    dst_y, dst_x = np.indices(dst_shape).astype(np.float32)
    dst_pixels_denormalized = np.column_stack([dst_x.ravel(), dst_y.ravel()])

    dst_pixels = features.normalized_image_coordinates(
        dst_pixels_denormalized,
        perspectiveshot.camera.width,
        perspectiveshot.camera.height)

    # Convert to bearing
    dst_bearings = perspectiveshot.camera.pixel_bearings(dst_pixels)

    # Rotate to panorama reference frame
    rotation = np.dot(panoshot.pose.get_rotation_matrix(),
                      perspectiveshot.pose.get_rotation_matrix().T)
    rotated_bearings = np.dot(dst_bearings, rotation.T)

    # Project to panorama pixels
    src_x, src_y = panoshot.camera.project((rotated_bearings[:, 0],
                                            rotated_bearings[:, 1],
                                            rotated_bearings[:, 2]))
    src_pixels = np.column_stack([src_x.ravel(), src_y.ravel()])

    src_pixels_denormalized = features.denormalized_image_coordinates(
        src_pixels, image.shape[1], image.shape[0])

    # Sample color
    colors = cv2.remap(image,
                       src_pixels_denormalized[:, 0].astype(np.float32),
                       src_pixels_denormalized[:, 1].astype(np.float32),
                       cv2.INTER_LINEAR)
    colors.shape = dst_shape + (-1,)
    return colors


def add_subshot_tracks(graph, panoshot, perspectiveshot):
    """Add edges between subshots and visible tracks."""
    if panoshot.id not in graph:
        return
    graph.add_node(perspectiveshot.id, bipartite=0)
    for track in graph[panoshot.id]:
        edge = graph[panoshot.id][track]
        feature = edge['feature']
        bearing = panoshot.camera.pixel_bearing(feature)
        rotation = np.dot(perspectiveshot.pose.get_rotation_matrix(),
                          panoshot.pose.get_rotation_matrix().T)

        rotated_bearing = np.dot(bearing, rotation.T)
        if rotated_bearing[2] <= 0:
            continue

        perspective_feature = perspectiveshot.camera.project(rotated_bearing)
        if (perspective_feature[0] < -0.5 or
                perspective_feature[0] > 0.5 or
                perspective_feature[1] < -0.5 or
                perspective_feature[1] > 0.5):
            continue

        graph.add_edge(perspectiveshot.id,
                       track,
                       feature=perspective_feature,
                       feature_id=edge['feature_id'],
                       feature_color=edge['feature_color'])
