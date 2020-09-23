import logging
import os

from opensfm import dataset
from opensfm import io
from . import command


logger = logging.getLogger(__name__)


class Command(command.CommandBase):
    def __init__(self):
        super(Command, self).__init__()
        self.name = "export_bundler"
        self.help = "Export reconstruction to bundler format"

        self.args["--list_path"] = {
            "help": "Path to the list.txt file",
        }
        self.args["--bundle_path"] = {
            "help": "Path to the bundle.out file'",
        }
        self.args["--undistorted"] = {
            "help": "Export the undistorted reconstruction'",
            "action": "store_true",
        }

    def run_dataset(self, options, data):
        udata = dataset.UndistortedDataSet(data, 'undistorted')

        default_path = os.path.join(data.data_path, 'bundler')
        list_file_path = options.list_path if options.list_path else default_path
        bundle_file_path = options.bundle_path if options.bundle_path else default_path

        if options.undistorted:
            reconstructions = udata.load_undistorted_reconstruction()
            track_manager = udata.load_undistorted_tracks_manager()
            images = reconstructions[0].shots.keys()
        else:
            reconstructions = data.load_reconstruction()
            track_manager = data.load_tracks_manager()
            images = data.images()

        io.export_bundler(images, reconstructions, track_manager,
                          bundle_file_path, list_file_path)
