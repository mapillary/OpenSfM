import logging
from timeit import default_timer as timer

from opensfm import io
from opensfm import matching
from . import command


logger = logging.getLogger(__name__)


class Command(command.CommandBase):
    def __init__(self):
        super(Command, self).__init__()
        self.name = "match_features"
        self.help = "Match features between image pairs"

    def run_dataset(self, options, data):
        images = data.images()

        start = timer()
        pairs_matches, preport = matching.match_images(data, images, images)
        matching.save_matches(data, images, pairs_matches)
        end = timer()

        with open(data.profile_log(), 'a') as fout:
            fout.write('match_features: {0}\n'.format(end - start))
        self.write_report(data, preport, list(pairs_matches.keys()), end - start)

    def write_report(self, data, preport, pairs, wall_time):
        report = {
            "wall_time": wall_time,
            "num_pairs": len(pairs),
            "pairs": pairs,
        }
        report.update(preport)
        data.save_report(io.json_dumps(report), 'matches.json')
