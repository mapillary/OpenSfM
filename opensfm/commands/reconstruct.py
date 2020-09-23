import logging
import time

from opensfm import io
from opensfm import reconstruction
from . import command


logger = logging.getLogger(__name__)


class Command(command.CommandBase):
    def __init__(self):
        super(Command, self).__init__()
        self.name = "reconstruct"
        self.help = "Compute the reconstruction"

    def run_dataset(self, options, data):
        start = time.time()
        tracks_manager = data.load_tracks_manager()
        report, reconstructions = reconstruction.\
            incremental_reconstruction(data, tracks_manager)
        end = time.time()
        with open(data.profile_log(), 'a') as fout:
            fout.write('reconstruct: {0}\n'.format(end - start))
        data.save_reconstruction(reconstructions)
        data.save_report(io.json_dumps(report), 'reconstruction.json')
