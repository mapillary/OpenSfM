import logging
from itertools import combinations
from collections import defaultdict
import numpy as np

import scipy.spatial as spatial

from opensfm import bow
from opensfm.context import parallel_map


logger = logging.getLogger(__name__)


def has_gps_info(exif):
    return (exif and
            'gps' in exif and
            'latitude' in exif['gps'] and
            'longitude' in exif['gps'])


def match_candidates_by_distance(images_ref, images_cand, exifs, reference,
                                 max_neighbors, max_distance):
    """Find candidate matching pairs by GPS distance.

    The GPS altitude is ignored because we want images of the same position
    at different altitudes to be matched together.  Otherwise, for drone
    datasets, flights at different altitudes do not get matched.
    """
    if max_neighbors <= 0 and max_distance <= 0:
        return set()
    max_neighbors = max_neighbors or 99999999
    max_distance = max_distance or 99999999.
    k = min(len(images_cand), max_neighbors + 1)

    points = np.zeros((len(images_cand), 3))
    for i, image in enumerate(images_cand):
        gps = exifs[image]['gps']
        points[i] = reference.to_topocentric(
            gps['latitude'], gps['longitude'], 0)

    tree = spatial.cKDTree(points)

    pairs = set()
    for image_ref in images_ref:
        gps = exifs[image]['gps']
        point = reference.to_topocentric(
            gps['latitude'], gps['longitude'], 0)
        distances, neighbors = tree.query(
            point, k=k, distance_upper_bound=max_distance)

        for j in neighbors:
            image_cand = images_cand[j]
            if image_cand != image_ref and j < len(images_cand):
                pairs.add(tuple(sorted((image_ref, image_cand))))
    return pairs


def match_candidates_with_bow(data, images_ref, images_cand,
                              exifs, reference, max_neighbors,
                              max_gps_distance, max_gps_neighbors,
                              enforce_other_cameras):
    """Find candidate matching pairs using BoW-based distance.

    If max_gps_distance > 0, then we use first restrain a set of
    candidates using max_gps_neighbors neighbors selected using
    GPS distance.

    If enforce_other_cameras is True, we keep max_neighbors images
    with same cameras AND  max_neighbors images from any other different
    camera.
    """
    if max_neighbors <= 0:
        return set()

    preempted_cand = {im: images_cand for im in images_ref}
    if max_gps_distance > 0 or max_gps_neighbors > 0:
        gps_pairs = match_candidates_by_distance(images_ref, images_cand,
                                                 exifs, reference,
                                                 max_gps_neighbors,
                                                 max_gps_distance)
        preempted_cand = defaultdict(list)
        for p in gps_pairs:
            preempted_cand[p[0]].append(p[1])

    need_load = set(preempted_cand.keys())
    for v in preempted_cand.values():
        preempted_cand.update(v)

    ctx = DummyContext()
    ctx.words = {im: _keep_first_word(data.load_words(im)) for im in preempted_cand}
    ctx.bows = bow.load_bows(data.config)
    ctx.exifs = exifs
    ctx.masks = {} # {feature_loader.load_masks(data, im) for im in preempted_cand}
    args = list(match_bow_arguments(preempted_cand, ctx))

    start = timer()
    processes = data.config['processes']
    results = parallel_map(match_bow_unwrap_args, args, processes)
    end = timer()

    pairs = set()
    for im, order, other in results:
        if enforce_other_cameras:
            pairs.union(_pairs_from_neighbors(im, exifs, order, other, max_neighbors))
        else:
            for i in order[:max_neighbors]:
                pairs.add(tuple(sorted((im, other[i]))))
    return pairs


def match_bow_arguments(candidates, ctx):
    for im, cands in candidates.items():
        yield (im, cands, ctx.words, ctx.masks, ctx.bows)


def match_bow_unwrap_args(args):
    args = image, other_images, words, masks, bows
    return bow_distances(image, other_images, words, masks, bows)


class DummyContext:
    pass


def match_candidates_by_time(images_ref, images_cand, exifs, max_neighbors):
    """Find candidate matching pairs by time difference."""
    if max_neighbors <= 0:
        return set()
    k = min(len(images_cand), max_neighbors + 1)

    times = np.zeros((len(images_cand), 1))
    for i, image in enumerate(images_cand):
        times[i] = exifs[image]['capture_time']

    tree = spatial.cKDTree(times)

    pairs = set()
    for image_ref in images_ref:
        time = exifs[image_ref]['capture_time']
        distances, neighbors = tree.query(time, k=k)
        for j in neighbors:
            image_cand = images_cand[j]
            if image_ref != image_cand and j < len(images):
                pairs.add(tuple(sorted((image_ref, image_cand))))
    return pairs


def match_candidates_by_order(images_ref, images_cand, max_neighbors):
    """Find candidate matching pairs by sequence order."""
    if max_neighbors <= 0:
        return set()
    n = (max_neighbors + 1) // 2

    pairs = set()
    for i, image_ref in enumerate(images_ref):
        a = max(0, i - n)
        b = min(len(images_cand), i + n)
        for j in range(a, b):
            image_cand = images_cand[j]
            if image_ref != image_cand:
                pairs.add(tuple(sorted((image_ref, images_cand))))
    return pairs


def match_candidates_from_metadata(images_ref, images_cand, exifs, data):
    """Compute candidate matching pairs between between images_ref and images_cand"""
    max_distance = data.config['matching_gps_distance']
    gps_neighbors = data.config['matching_gps_neighbors']
    time_neighbors = data.config['matching_time_neighbors']
    order_neighbors = data.config['matching_order_neighbors']
    bow_neighbors = data.config['matching_bow_neighbors']
    bow_gps_distance = data.config['matching_bow_gps_distance']
    bow_gps_neighbors = data.config['matching_bow_gps_neighbors']
    bow_other_cameras = data.config['matching_bow_other_cameras']

    if not data.reference_lla_exists():
        data.invent_reference_lla()
    reference = data.load_reference()

    if not all(map(has_gps_info, exifs.values())):
        if gps_neighbors != 0:
            logger.warn("Not all images have GPS info. "
                        "Disabling matching_gps_neighbors.")
        gps_neighbors = 0
        max_distance = 0

    images_ref.sort()

    if max_distance == gps_neighbors == time_neighbors == order_neighbors == bow_neighbors == 0:
        # All pair selection strategies deactivated so we match all pairs
        d = set()
        t = set()
        o = set()
        b = set()
        pairs = combinations(images, 2)
    else:
        d = match_candidates_by_distance(images_ref, images_cand, exifs, reference,
                                         gps_neighbors, max_distance)
        t = match_candidates_by_time(images_ref, images_cand, exifs, time_neighbors)
        o = match_candidates_by_order(images_ref, images_cand, order_neighbors)
        b = match_candidates_with_bow(data, images_ref, images_cand,
                                      exifs, reference, bow_neighbors,
                                      bow_gps_distance, bow_gps_neighbors,
                                      bow_other_cameras)
        pairs = d | t | o | b

    res = {im: [] for im in images_ref}
    for im1, im2 in pairs:
        res[im1].append(im2)

    report = {
        "num_pairs_distance": len(d),
        "num_pairs_time": len(t),
        "num_pairs_order": len(o)
        "num_pairs_bow": len(b)
    }
    return res, report


def bow_distances(image, other_images, words, masks, bows):
    """ Compute BoW-based distance (L1 on histogram of words)
        between an image and other images.

        Can use optionaly masks for discarding some features
    """
    # params
    min_num_feature = 8

    if words[image] is None:
        logger.error("Could not load words for image {}".format(image))
        return []

    filtered_words = words[image][masks[image]] if masks else words[image]
    if len(filtered_words) <= min_num_feature:
        logger.warning("Too few filtered features in image {}: {}".format(
            image, len(filtered_words)))
        return []

    # Compute distance
    distances = []
    other = []
    h = bows.histogram(filtered_words[:, 0])
    for im2 in other_images:
        if im2 != image and words[im2] is not None:
            im2_words = words[im2][masks[im2]] if masks else words[im2]
            if len(im2_words) > min_num_feature:
                h2 = bows.histogram(im2_words[:, 0])
                distances.append(np.fabs(h - h2).sum())
                other.append(im2)
            else:
                logger.warning(
                    "Too few features in matching image {}: {}".format(
                        im2, len(words[im2])))
    return image, np.argsort(distances), other


def _keep_first_word(words):
    """Keep only the first word of each feature.

    This is useful to free memory when only the first word is going to
    be used.  It copies the array so that the original words array can
    be freed.
    """
    if words is None or len(words) == 0:
        return words
    return words[:, :1].copy()


def _pairs_from_neighbors(image, exifs, order, other, max_neighbors):
    """Construct matching pairs given closest ordered neighbors.

    Pairs will of form (image, im2), im2 being the closest max_neighbors
    given by (order, other) having the same cameras OR the closest max_neighbors
    having from any other camera.
    """
    same_camera, other_cameras = [], []
    for i in order:
        im2 = other[i]
        if exifs[im2]['camera'] == exifs[image]['camera']:
            if len(same_camera) < max_neighbors:
                same_camera.append(im2)
        else:
            if len(other_cameras) < max_neighbors:
                other_cameras.append(im2)
        if len(same_camera) + len(other_cameras) >= 2 * max_neighbors:
            break

    pairs = set()
    for im2 in same_camera+other_cameras:
        pairs.add(tuple(sorted((image, im2))))
    return pairs
