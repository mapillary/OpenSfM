import logging

from opensfm import dataset

logger = logging.getLogger(__name__)


class Command:
    name = 'export_ply'
    help = "Export reconstruction to PLY format"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()

        if reconstructions:
            data.save_ply(reconstructions[0])
