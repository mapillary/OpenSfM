import os

from opensfm import dataset, undistort
from opensfm.dataset import DataSet


def run_dataset(
    data: DataSet,
    reconstruction: str,
    reconstruction_index: int,
    tracks: str,
    output: str,
    skip_images: bool = False,
) -> None:
    """Export reconstruction to NVM_V3 format from VisualSfM

    Args:
        reconstruction: reconstruction to undistort
        reconstruction_index: index of the reconstruction component to undistort
        tracks: tracks graph of the reconstruction
        output: undistorted
        skip_images: do not undistort images
    """
    undistorted_data_path = os.path.join(data.data_path, output)
    udata = dataset.UndistortedDataSet(
        data, undistorted_data_path, io_handler=data.io_handler
    )
    reconstructions = data.load_reconstruction(reconstruction)
    if data.tracks_exists(tracks):
        tracks_manager = data.load_tracks_manager(tracks)
    else:
        tracks_manager = None

    if reconstructions:
        r = reconstructions[reconstruction_index]
        undistort.undistort_reconstruction_with_images(
            tracks_manager, r, data, udata, skip_images
        )
