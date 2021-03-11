import os

from opensfm import dataset, undistort
from opensfm.dataset import DataSet


def run_dataset(data: DataSet, reconstruction, reconstruction_index, tracks, output):
    """Export reconstruction to NVM_V3 format from VisualSfM

    Args:
        reconstruction: reconstruction to undistort
        reconstruction_index: index of the reconstruction component to undistort
        tracks: tracks graph of the reconstruction
        output: undistorted

    """
    undistorted_data_path = os.path.join(data.data_path, output)
    udata = dataset.UndistortedDataSet(data, undistorted_data_path)
    reconstructions = data.load_reconstruction(reconstruction)
    if data.tracks_exists(tracks):
        tracks_manager = data.load_tracks_manager(tracks)
    else:
        tracks_manager = None

    if reconstructions:
        r = reconstructions[reconstruction_index]
        undistort.undistort_reconstruction_and_images(tracks_manager, r, data, udata)
