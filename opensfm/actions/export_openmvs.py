import os

import numpy as np

from opensfm import dataset
from opensfm import pydense
from opensfm import io


def run_dataset(data):
    """ Export reconstruction to OpenMVS format. """

    udata = dataset.UndistortedDataSet(data, 'undistorted')
    reconstructions = udata.load_undistorted_reconstruction()
    tracks_manager = udata.load_undistorted_tracks_manager()

    if reconstructions:
        export(reconstructions[0], tracks_manager, udata, data)

def export(reconstruction, tracks_manager, udata, data):
    exporter = pydense.OpenMVSExporter()
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
            image_path = udata._undistorted_image_file(shot.id)
            exporter.add_shot(
                str(os.path.abspath(image_path)),
                str(shot.id),
                str(shot.camera.id),
                shot.pose.get_rotation_matrix(),
                shot.pose.get_origin())

    for point in reconstruction.points.values():
        shots = list(tracks_manager.get_track_observations(point.id))

        coordinates = np.array(point.coordinates, dtype=np.float64)
        exporter.add_point(coordinates, shots)

    io.mkdir_p(udata.data_path + '/openmvs')
    exporter.export(udata.data_path + '/openmvs/scene.mvs')
