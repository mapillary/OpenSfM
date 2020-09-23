import os

from opensfm import dataset
from opensfm import io


def run_dataset(data, list_path, bundle_path, undistorted):
    """ Export reconstruction to bundler format.

    Args:
        list_path: txt list of images to export
        bundle_path : output path
        undistorted : export undistorted reconstruction

    """

    udata = dataset.UndistortedDataSet(data, 'undistorted')

    default_path = os.path.join(data.data_path, 'bundler')
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

    io.export_bundler(images, reconstructions, track_manager,
                        bundle_file_path, list_file_path)
