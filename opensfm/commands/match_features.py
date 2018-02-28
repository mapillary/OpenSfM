import logging
from itertools import combinations
from timeit import default_timer as timer

import numpy as np
import scipy.spatial as spatial

from opensfm import dataset
from opensfm import geo
from opensfm import io
from opensfm import log
from opensfm import matching
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
        pairs, preport = match_candidates_from_metadata(images, exifs, data)

        num_pairs = sum(len(c) for c in pairs.values())
        logger.info('Matching {} image pairs'.format(num_pairs))

        ctx = Context()
        ctx.data = data
        ctx.cameras = ctx.data.load_camera_models()
        ctx.exifs = exifs
        ctx.p_pre, ctx.f_pre = load_preemptive_features(data)
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


def load_preemptive_features(data):
    p, f = {}, {}
    if data.config['preemptive_threshold'] > 0:
        logger.debug('Loading preemptive data')
        for image in data.images():
            try:
                p[image], f[image] = \
                    data.load_preemtive_features(image)
            except IOError:
                p, f, c = data.load_features(image)
                p[image], f[image] = p, f
            preemptive_max = min(data.config['preemptive_max'],
                                 p[image].shape[0])
            p[image] = p[image][:preemptive_max, :]
            f[image] = f[image][:preemptive_max, :]
    return p, f


def has_gps_info(exif):
    return (exif and
            'gps' in exif and
            'latitude' in exif['gps'] and
            'longitude' in exif['gps'])


def match_candidates_by_distance(images, exifs, reference, max_neighbors, max_distance):
    """Find candidate matching pairs by GPS distance."""
    if max_neighbors <= 0 and max_distance <= 0:
        return set()
    max_neighbors = max_neighbors or 99999999
    max_distance = max_distance or 99999999.
    k = min(len(images), max_neighbors + 1)

    points = np.zeros((len(images), 3))
    for i, image in enumerate(images):
        gps = exifs[image]['gps']
        alt = gps.get('altitude', 2.0)
        points[i] = geo.topocentric_from_lla(
            gps['latitude'], gps['longitude'], alt,
            reference['latitude'], reference['longitude'], reference['altitude'])

    tree = spatial.cKDTree(points)

    pairs = set()
    for i, image in enumerate(images):
        distances, neighbors = tree.query(
            points[i], k=k, distance_upper_bound=max_distance)
        for j in neighbors:
            if i != j and j < len(images):
                pairs.add(tuple(sorted((images[i], images[j]))))
    return pairs


def match_candidates_by_time(images, exifs, max_neighbors):
    """Find candidate matching pairs by time difference."""
    if max_neighbors <= 0:
        return set()
    k = min(len(images), max_neighbors + 1)

    times = np.zeros((len(images), 1))
    for i, image in enumerate(images):
        times[i] = exifs[image]['capture_time']

    tree = spatial.cKDTree(times)

    pairs = set()
    for i, image in enumerate(images):
        distances, neighbors = tree.query(times[i], k=k)
        for j in neighbors:
            if i != j and j < len(images):
                pairs.add(tuple(sorted((images[i], images[j]))))
    return pairs


def match_candidates_by_order(images, exifs, max_neighbors):
    """Find candidate matching pairs by sequence order."""
    if max_neighbors <= 0:
        return set()
    n = (max_neighbors + 1) // 2

    pairs = set()
    for i, image in enumerate(images):
        a = max(0, i - n)
        b = min(len(images), i + n)
        for j in range(a, b):
            if i != j:
                pairs.add(tuple(sorted((images[i], images[j]))))
    return pairs


def match_candidates_from_metadata(images, exifs, data):
    """Compute candidate matching pairs"""
    max_distance = data.config['matching_gps_distance']
    gps_neighbors = data.config['matching_gps_neighbors']
    time_neighbors = data.config['matching_time_neighbors']
    order_neighbors = data.config['matching_order_neighbors']

    if not data.reference_lla_exists():
        data.invent_reference_lla()
    reference = data.load_reference_lla()

    if not all(map(has_gps_info, exifs.values())):
        if gps_neighbors != 0:
            logger.warn("Not all images have GPS info. "
                        "Disabling matching_gps_neighbors.")
        gps_neighbors = 0
        max_distance = 0

    images.sort()

    if max_distance == gps_neighbors == time_neighbors == order_neighbors == 0:
        # All pair selection strategies deactivated so we match all pairs
        d = set()
        t = set()
        o = set()
        pairs = combinations(images, 2)
    else:
        d = match_candidates_by_distance(images, exifs, reference,
                                         gps_neighbors, max_distance)
        t = match_candidates_by_time(images, exifs, time_neighbors)
        o = match_candidates_by_order(images, exifs, order_neighbors)
        pairs = d | t | o

    res = {im: [] for im in images}
    for im1, im2 in pairs:
        res[im1].append(im2)

    report = {
        "num_pairs_distance": len(d),
        "num_pairs_time": len(t),
        "num_pairs_order": len(o)
    }
    return res, report


def match_arguments(pairs, ctx):
    for i, (im, candidates) in enumerate(pairs.items()):
        yield im, candidates, i, len(pairs), ctx


def match(args):
    """Compute all matches for a single image"""
    log.setup()

    im1, candidates, i, n, ctx = args
    logger.info('Matching {}  -  {} / {}'.format(im1, i + 1, n))

    config = ctx.data.config
    robust_matching_min_match = config['robust_matching_min_match']
    preemptive_threshold = config['preemptive_threshold']
    lowes_ratio = config['lowes_ratio']
    preemptive_lowes_ratio = config['preemptive_lowes_ratio']

    im1_matches = {}

    for im2 in candidates:
        # preemptive matching
        if preemptive_threshold > 0:
            t = timer()
            config['lowes_ratio'] = preemptive_lowes_ratio
            matches_pre = matching.match_lowe_bf(
                ctx.f_pre[im1], ctx.f_pre[im2], config)
            config['lowes_ratio'] = lowes_ratio
            logger.debug("Preemptive matching {0}, time: {1}s".format(
                len(matches_pre), timer() - t))
            if len(matches_pre) < preemptive_threshold:
                logger.debug(
                    "Discarding based of preemptive matches {0} < {1}".format(
                        len(matches_pre), preemptive_threshold))
                continue

        # symmetric matching
        t = timer()
        p1, f1, c1 = ctx.data.load_features(im1)
        p2, f2, c2 = ctx.data.load_features(im2)

        if config['matcher_type'] == 'FLANN':
            i1 = ctx.data.load_feature_index(im1, f1)
            i2 = ctx.data.load_feature_index(im2, f2)
        else:
            i1 = None
            i2 = None

        matches = matching.match_symmetric(f1, i1, f2, i2, config)
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
