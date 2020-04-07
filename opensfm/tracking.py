import logging
import sys

import numpy as np

from collections import defaultdict
from itertools import combinations
from six import iteritems

from opensfm.unionfind import UnionFind
from opensfm import pysfm


logger = logging.getLogger(__name__)


def load_features(dataset, images):
    logging.info('reading features')
    features = {}
    colors = {}
    for im in images:
        p, f, c = dataset.load_features(im)
        features[im] = p[:, :3]
        colors[im] = c
    return features, colors


def load_matches(dataset, images):
    matches = {}
    for im1 in images:
        try:
            im1_matches = dataset.load_matches(im1)
        except IOError:
            continue
        for im2 in im1_matches:
            if im2 in images:
                matches[im1, im2] = im1_matches[im2]
    return matches


def create_tracks_manager(features, colors, matches, config):
    """Link matches into tracks."""
    logger.debug('Merging features onto tracks')
    uf = UnionFind()
    for im1, im2 in matches:
        for f1, f2 in matches[im1, im2]:
            uf.union((im1, f1), (im2, f2))

    sets = {}
    for i in uf:
        p = uf[i]
        if p in sets:
            sets[p].append(i)
        else:
            sets[p] = [i]

    min_length = config['min_track_length']
    tracks = [t for t in sets.values() if _good_track(t, min_length)]
    logger.debug('Good tracks: {}'.format(len(tracks)))

    tracks_manager = pysfm.TracksManager()
    for track_id, track in enumerate(tracks):
        track_tmp = {}
        for image, featureid in track:
            if image not in features:
                continue
            x, y, s = features[image][featureid]
            r, g, b = colors[image][featureid]
            track_tmp[image] = pysfm.Keypoint(x, y, s, int(r), int(g), int(b), featureid)
        tracks_manager.add_track(str(track_id), track_tmp)
    return tracks_manager


def tracks_and_images(tracks_manager):
    """List of tracks and images in the tracks_manager."""
    return tracks_manager.get_track_ids(), tracks_manager.get_shot_ids()


def common_tracks(graph, im1, im2):
    """List of tracks observed in both images.

    Args:
        graph: tracks graph
        im1: name of the first image
        im2: name of the second image

    Returns:
        tuple: tracks, feature from first image, feature from second image
    """
    t1, t2 = graph.get_observations_of_shot(im1),\
        graph.get_observations_of_shot(im2)
    tracks, p1, p2 = [], [], []
    for track in t1:
        if track in t2:
            p1.append(t1[track]['feature'])
            p2.append(t2[track]['feature'])
            tracks.append(track)
    p1 = np.array(p1)
    p2 = np.array(p2)
    return tracks, p1, p2


def all_common_tracks(tracks_manager, include_features=True, min_common=50):
    """List of tracks observed by each image pair.

    Args:
        tracks_manager: tracks manager
        include_features: whether to include the features from the images
        min_common: the minimum number of tracks the two images need to have
            in common

    Returns:
        tuple: im1, im2 -> tuple: tracks, features from first image, features
        from second image
    """
    common_tracks = {}
    for(im1, im2), tuples in tracks_manager.get_all_common_observations_all_pairs().items():
        if len(tuples) < min_common:
            continue

        if include_features:
            common_tracks[im1, im2] = ([v for v, _, _ in tuples],
                                       np.array([p.point for _, p, _ in tuples]),
                                       np.array([p.point for _, _, p in tuples]))
        else:
            common_tracks[im1, im2] = [v for v, _, _ in tuples]
    return common_tracks


def _good_track(track, min_length):
    if len(track) < min_length:
        return False
    images = [f[0] for f in track]
    if len(images) != len(set(images)):
        return False
    return True
