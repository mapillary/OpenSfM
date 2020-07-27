import logging

from opensfm import dataset
from opensfm import dense

logger = logging.getLogger(__name__)


class Command:
    name = 'compute_depthmaps'
    help = "Compute depthmap"

    def add_arguments(self, parser):
        parser.add_argument(
            'dataset',
            help='dataset to process',
        )
        parser.add_argument(
            '--subfolder',
            help='undistorted subfolder where to load and store data',
            default='undistorted'
        )
        parser.add_argument(
            '--interactive',
            help='plot results as they are being computed',
            action='store_true',
        )

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        udata = dataset.UndistortedDataSet(data, args.subfolder)
        data.config['interactive'] = args.interactive
        reconstructions = udata.load_undistorted_reconstruction()
        tracks_manager = udata.load_undistorted_tracks_manager()

        dense.compute_depthmaps(udata, tracks_manager, reconstructions[0])
