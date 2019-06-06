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
        pairs, preport = pairs_selection.match_candidates_from_metadata(images, exifs, data)

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

    config = ctx.data.config
    matcher_type = config['matcher_type']
    robust_matching_min_match = config['robust_matching_min_match']

    im1_matches = {}

    for im2 in candidates:
        # symmetric matching
        t = timer()
        p1, f1, c1 = ctx.data.load_features(im1)
        p2, f2, c2 = ctx.data.load_features(im2)

        if matcher_type == 'WORDS':
            w1 = ctx.data.load_words(im1)
            w2 = ctx.data.load_words(im2)
            matches = matching.match_words_symmetric(f1, w1, f2, w2, config)
        elif matcher_type == 'FLANN':
            i1 = ctx.data.load_feature_index(im1, f1)
            i2 = ctx.data.load_feature_index(im2, f2)
            matches = matching.match_flann_symmetric(f1, i1, f2, i2, config)
        elif matcher_type == 'BRUTEFORCE':
            matches = matching.match_brute_force_symmetric(f1, f2, config)
        else:
            raise ValueError("Invalid matcher_type: {}".format(matcher_type))

        logger.debug('{} - {} has {} candidate matches'.format(
            im1, im2, len(matches)))
        if len(matches) < robust_matching_min_match:
            im1_matches[im2] = []
            continue

        # robust matching
        t_robust_matching = timer()
        camera1 = ctx.cameras[ctx.exifs[im1]['camera']]
        camera2 = ctx.cameras[ctx.exifs[im2]['camera']]

        rmatches = matching.robust_match(p1, p2, camera1, camera2, matches,
                                         config)

        if len(rmatches) < robust_matching_min_match:
            im1_matches[im2] = []
            continue
        im1_matches[im2] = rmatches
        logger.debug('Robust matching time : {0}s'.format(
            timer() - t_robust_matching))

        logger.debug("Full matching {0} / {1}, time: {2}s".format(
            len(rmatches), len(matches), timer() - t))
    ctx.data.save_matches(im1, im1_matches)
