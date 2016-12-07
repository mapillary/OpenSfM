import logging

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
        data.save_undistorted_tracks_graph(graph)

        if reconstructions:
            self.undistort_images(reconstructions[0], data)

    def undistort_images(self, reconstruction, data):
        urec = types.Reconstruction()
        urec.points = reconstruction.points

        for shot in reconstruction.shots.values():
            if shot.camera.projection_type == 'perspective':
                urec.add_camera(shot.camera)
                urec.add_shot(shot)

                image = data.image_as_array(shot.id)
                undistorted = undistort_image(image, shot)
                data.save_undistorted_image(shot.id, undistorted)
            elif shot.camera.projection_type in ['equirectangular', 'spherical']:
                image = data.image_as_array(shot.id)
                shots = perspective_views_of_a_panorama(shot)
                for subshot in shots:
                    urec.add_camera(subshot.camera)
                    urec.add_shot(subshot)
                    undistorted = render_perspective_view_of_a_panorama(
                        image, shot, subshot)
                    data.save_undistorted_image(subshot.id, undistorted)

        data.save_undistorted_reconstruction([urec])


def undistort_image(image, shot):
    """Remove radial distortion from a perspective image."""
    camera = shot.camera
    height, width = image.shape[:2]
    K = camera.get_K_in_pixel_coordinates(width, height)
    distortion = np.array([camera.k1, camera.k2, 0, 0])
    return cv2.undistort(image, K, distortion)


def perspective_views_of_a_panorama(spherical_shot):
    """Create 6 perspective views of a panorama."""
    camera = types.PerspectiveCamera()
    camera.id = 'perspective_panorama_camera'
    camera.width = 640
    camera.height = 640
    camera.focal = 0.5
    camera.focal_prior = camera.focal
    camera.k1 = 0.0
    camera.k1_prior = camera.k1
    camera.k2 = 0.0
    camera.k2_prior = camera.k2

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
        src_pixels,
        panoshot.camera.width,
        panoshot.camera.height)

    # Sample color
    colors = image[src_pixels_denormalized[:, 1].astype(int),
                   src_pixels_denormalized[:, 0].astype(int)]
    colors.shape = dst_shape + (-1,)

    if False:
        import matplotlib.pyplot as plt
        plt.imshow(image)
        plt.show()
        plt.imshow(colors)
        plt.show()

    return colors
