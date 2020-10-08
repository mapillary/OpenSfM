from opensfm import dataset
from opensfm import dense


def run_dataset(data, subfolder, interactive):
    """ Compute depthmap on a dataset with has SfM ran already.

    Args:
        subfolder: dataset's subfolder where to store results
        interactive : display plot of computed depthmaps

    """

    udataset = dataset.UndistortedDataSet(data, subfolder)
    data.config['interactive'] = interactive
    reconstructions = udataset.load_undistorted_reconstruction()
    tracks_manager = udataset.load_undistorted_tracks_manager()
    dense.compute_depthmaps(udataset, tracks_manager, reconstructions[0])
