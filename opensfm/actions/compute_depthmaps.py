import os

from opensfm import dataset
from opensfm import dense
from opensfm.dataset import DataSet


def run_dataset(data: DataSet, subfolder, interactive):
    """Compute depthmap on a dataset with has SfM ran already.

    Args:
        subfolder: dataset's subfolder where to store results
        interactive : display plot of computed depthmaps

    """

    udata_path = os.path.join(data.data_path, subfolder)
    udataset = dataset.UndistortedDataSet(data, udata_path, io_handler=data.io_handler)
    data.config["interactive"] = interactive
    reconstructions = udataset.load_undistorted_reconstruction()
    tracks_manager = udataset.load_undistorted_tracks_manager()
    dense.compute_depthmaps(udataset, tracks_manager, reconstructions[0])
