import logging

import cv2
import numpy as np

from opensfm import dataset
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

        data.save_undistorted_reconstruction([reconstruction])


def undistort_image(image, shot):
    """Remove radial distortion from a perspective image."""
    camera = shot.camera
    height, width = image.shape[:2]
    K = camera.get_K_in_pixel_coordinates(width, height)
    distortion = np.array([camera.k1, camera.k2, 0, 0])
    return cv2.undistort(image, K, distortion)
