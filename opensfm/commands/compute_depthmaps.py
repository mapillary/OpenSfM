import logging

from opensfm import dataset
from opensfm import dense
from . import command


logger = logging.getLogger(__name__)


class Command(command.CommandBase):
    def __init__(self):
        super(Command, self).__init__()
        self.name = "compute_depthmaps"
        self.help = "Compute depthmap"

        self.args["--subfolder"] = {
            "help": "Undistorted subfolder where to load and store data",
            "default": 'undistorted',
        }
        self.args["--interactive"] = {
            "help": "Plot results as they are being computed'",
            "action": "store_true",
        }

    def run_dataset(self, options, data):
        udataset = dataset.UndistortedDataSet(data, options.subfolder)
        data.config['interactive'] = options.interactive
        reconstructions = udataset.load_undistorted_reconstruction()
        tracks_manager = udataset.load_undistorted_tracks_manager()
        dense.compute_depthmaps(udataset, tracks_manager, reconstructions[0])
