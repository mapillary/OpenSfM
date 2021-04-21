import os

from opensfm import io
from opensfm.dataset import DataSet


def run_dataset(data: DataSet, list_path, bundle_path, undistorted):
    """Export reconstruction to bundler format.

    Args:
        list_path: txt list of images to export
        bundle_path : output path
        undistorted : export undistorted reconstruction

    """

    udata = data.undistorted_dataset()

    default_path = os.path.join(data.data_path, "bundler")
    list_file_path = list_path if list_path else default_path
    bundle_file_path = bundle_path if bundle_path else default_path

    if undistorted:
        reconstructions = udata.load_undistorted_reconstruction()
        track_manager = udata.load_undistorted_tracks_manager()
        images = reconstructions[0].shots.keys()
    else:
        reconstructions = data.load_reconstruction()
        track_manager = data.load_tracks_manager()
        images = data.images()

    io.export_bundler(
        images, reconstructions, track_manager, bundle_file_path, list_file_path
    )
