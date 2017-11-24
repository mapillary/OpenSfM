import logging

from opensfm import dataset

logger = logging.getLogger(__name__)


class Command:
    name = 'export_ply'
    help = "Export reconstruction to PLY format"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('--no-cameras',
                            action='store_true',
                            default=False,
                            help='Do not save camera positions')
        parser.add_argument('--no-points',
                            action='store_true',
                            default=False,
                            help='Do not save points')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()
        no_cameras = args.no_cameras
        no_points = args.no_points

        if reconstructions:
            data.save_ply(reconstructions[0], None, no_cameras, no_points)
