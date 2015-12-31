import logging
import time

from opensfm import dataset
from opensfm import reconstruction

logger = logging.getLogger(__name__)


class Command:
    name = 'reconstruct'
    help = "Compute the reconstruction"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        start = time.time()
        data = dataset.DataSet(args.dataset)
        reconstruction.incremental_reconstruction(data)
        end = time.time()
        with open(data.profile_log(), 'a') as fout:
            fout.write('reconstruct: {0}\n'.format(end - start))
