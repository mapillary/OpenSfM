import logging
import os

import cv2
import numpy as np

from opensfm import csfm
from opensfm import dataset

logger = logging.getLogger(__name__)


class Command:
    name = 'export_openmvs'
    help = "Export reconstruction to openMVS format"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()
        graph = data.load_tracks_graph()

        if reconstructions:
            self.undistort_images(reconstructions[0], data)
            self.export(reconstructions[0], graph, data)

    def export(self, reconstruction, graph, data):
        exporter = csfm.OpenMVSExporter()
        for camera in reconstruction.cameras.values():
            if camera.projection_type == 'perspective':
                w, h = camera.width, camera.height
                K = np.array([
                    [camera.focal, 0, (w - 1.0) / 2 / max(w, h)],
                    [0, camera.focal, (h - 1.0) / 2 / max(w, h)],
                    [0, 0, 1],
                ])
                exporter.add_camera(str(camera.id), K)

        for shot in reconstruction.shots.values():
            if shot.camera.projection_type == 'perspective':
                image_path = data._undistorted_image_file(shot.id)
                exporter.add_shot(
                    str(os.path.abspath(image_path)),
                    str(shot.id),
                    str(shot.camera.id),
                    shot.pose.get_rotation_matrix(),
                    shot.pose.get_origin())

        for point in reconstruction.points.values():
            shots = graph[point.id].keys()
            coordinates = np.array(point.coordinates, dtype=np.float64)
            exporter.add_point(coordinates, shots)

        exporter.export(data.data_path + '/openmvs_scene.mvs')

    def undistort_images(self, reconstruction, data):
        for shot in reconstruction.shots.values():
            if shot.camera.projection_type == 'perspective':
                image = data.image_as_array(shot.id)
                undistorted = undistort_image(image, shot)
                data.save_undistorted_image(shot.id, undistorted)


def undistort_image(image, shot):
    """Remove radial distortion from a perspective image."""
    camera = shot.camera
    height, width = image.shape[:2]
    K = opencv_calibration_matrix(width, height, camera.focal)
    distortion = np.array([camera.k1, camera.k2, 0, 0])
    return cv2.undistort(image, K, distortion)


def opencv_calibration_matrix(width, height, focal):
    """Calibration matrix as used by OpenCV and PMVS.

    Duplicated with bin.export_openmvs.opencv_calibration_matrix
    """
    f = focal * max(width, height)
    return np.matrix([[f, 0, 0.5 * (width - 1)],
                      [0, f, 0.5 * (height - 1)],
                      [0, 0, 1.0]])
