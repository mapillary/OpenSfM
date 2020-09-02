import logging

from opensfm import dataset
from opensfm import stats

logger = logging.getLogger(__name__)


class Command:
    name = 'export_statistics'
    help = "Compute statistics and export them in the stats folder"

    def add_arguments(self, parser):
        parser.add_argument(
            'dataset',
            help='dataset to process',
        )
        parser.add_argument(
            '--interactive',
            help='plot results as they are being computed',
            action='store_true',
        )

    def run(self, args):
        data = dataset.DataSet(args.dataset)

        reconstructions = data.load_reconstruction()
        tracks_manager = data.load_tracks_manager()

        overall_stats = stats.compute_overall_statistics(reconstructions[0])
        grids = stats.compute_residual_grids(tracks_manager, reconstructions[0])