import logging

from opensfm import dataset
from opensfm import dense

logger = logging.getLogger(__name__)


class Command:
    name = 'compute_depthmaps'
    help = "Compute depthmap"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        reconstructions = data.load_reconstruction()

        for reconstruction in reconstructions:
            for shot_id in reconstruction.shots:
                dense.compute_depthmap(data, reconstruction, shot_id)
