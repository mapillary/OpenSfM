import logging
from timeit import default_timer as timer

import numpy as np

from opensfm import dataset
from opensfm import io
from opensfm import log
from opensfm import matching
from opensfm import pairs_selection
from opensfm.context import parallel_map


logger = logging.getLogger(__name__)


class Command:
    name = 'match_features'
    help = 'Match features between image pairs'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        images = data.images()
        exifs = {im: data.load_exif(im) for im in images}
        pairs, preport = pairs_selection.match_candidates_from_metadata(images, images, exifs, data)

        num_pairs = sum(len(c) for c in pairs.values())
        logger.info('Matching {} image pairs'.format(num_pairs))

        ctx = Context()
        ctx.data = data
        ctx.cameras = ctx.data.load_camera_models()
        ctx.exifs = exifs
        args = list(match_arguments(pairs, ctx))

        start = timer()
        processes = ctx.data.config['processes']
        parallel_map(match, args, processes)
        end = timer()

        with open(ctx.data.profile_log(), 'a') as fout:
            fout.write('match_features: {0}\n'.format(end - start))
        self.write_report(data, preport, pairs, end - start)

    def write_report(self, data, preport, pairs, wall_time):
        pair_list = []
        for im1, others in pairs.items():
            for im2 in others:
                pair_list.append((im1, im2))

        report = {
            "wall_time": wall_time,
            "num_pairs": len(pair_list),
            "pairs": pair_list,
        }
        report.update(preport)
        data.save_report(io.json_dumps(report), 'matches.json')


class Context:
    pass


def match_arguments(pairs, ctx):
    for i, (im, candidates) in enumerate(pairs.items()):
        yield im, candidates, i, len(pairs), ctx


def match(args):
    """Compute all matches for a single image"""
    log.setup()

    im1, candidates, i, n, ctx = args
    logger.info('Matching {}  -  {} / {}'.format(im1, i + 1, n))

    t = timer()
    im1_matches = {}
    for im2 in candidates:
        p1, f1, _ = ctx.data.load_features(im1)
        p2, f2, _ = ctx.data.load_features(im2)
        w1 = ctx.data.load_words(im1)
        w2 = ctx.data.load_words(im2)
        camera1 = ctx.cameras[ctx.exifs[im1]['camera']]
        camera2 = ctx.cameras[ctx.exifs[im2]['camera']]
        im1_matches[im2] = matching.match(im1, im2, camera1, camera2,
                                          p1, p2, f1, f2, w1, w2,
                                          None, None, ctx.data)
    ctx.data.save_matches(im1, im1_matches)
