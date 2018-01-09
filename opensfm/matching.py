import numpy as np
import cv2
import pyopengv
import networkx as nx
import logging
from collections import defaultdict
from itertools import combinations

from opensfm import context
from opensfm.unionfind import UnionFind


logger = logging.getLogger(__name__)


# pairwise matches
def match_lowe(index, f2, config):
    search_params = dict(checks=config['flann_checks'])
    results, dists = index.knnSearch(f2, 2, params=search_params)
    squared_ratio = config['lowes_ratio']**2  # Flann returns squared L2 distances
    good = dists[:, 0] < squared_ratio * dists[:, 1]
    matches = zip(results[good, 0], good.nonzero()[0])
    return np.array(matches, dtype=int)


def match_symmetric(fi, indexi, fj, indexj, config):
    if config['matcher_type'] == 'FLANN':
        matches_ij = [(a, b) for a, b in match_lowe(indexi, fj, config)]
        matches_ji = [(b, a) for a, b in match_lowe(indexj, fi, config)]
    else:
        matches_ij = [(a, b) for a, b in match_lowe_bf(fi, fj, config)]
        matches_ji = [(b, a) for a, b in match_lowe_bf(fj, fi, config)]

    matches = set(matches_ij).intersection(set(matches_ji))
    return np.array(list(matches), dtype=int)


def convert_matches_to_vector(matches):
    '''Convert Dmatch object to matrix form
    '''
    matches_vector = np.zeros((len(matches), 2), dtype=np.int)
    k = 0
    for mm in matches:
        matches_vector[k, 0] = mm.queryIdx
        matches_vector[k, 1] = mm.trainIdx
        k = k+1
    return matches_vector


def match_lowe_bf(f1, f2, config):
    '''Bruteforce feature matching
    '''
    assert(f1.dtype.type == f2.dtype.type)
    if (f1.dtype.type == np.uint8):
        matcher_type = 'BruteForce-Hamming'
    else:
        matcher_type = 'BruteForce'
    matcher = cv2.DescriptorMatcher_create(matcher_type)
    matches = matcher.knnMatch(f1, f2, k=2)

    ratio = config['lowes_ratio']
    good_matches = []
    for match in matches:
        if match and len(match) == 2:
            m, n = match
            if m.distance < ratio * n.distance:
                good_matches.append(m)
    good_matches = convert_matches_to_vector(good_matches)
    return np.array(good_matches, dtype=int)


def robust_match_fundamental(p1, p2, matches, config):
    '''Computes robust matches by estimating the Fundamental matrix via RANSAC.
    '''
    if len(matches) < 8:
        return np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()

    FM_RANSAC = cv2.FM_RANSAC if context.OPENCV3 else cv2.cv.CV_FM_RANSAC
    F, mask = cv2.findFundamentalMat(p1, p2, FM_RANSAC, config['robust_matching_threshold'], 0.9999)
    inliers = mask.ravel().nonzero()

    if F[2, 2] == 0.0:
        return []

    return matches[inliers]


def compute_inliers_bearings(b1, b2, T):
    R = T[:, :3]
    t = T[:, 3]
    p = pyopengv.triangulation_triangulate(b1, b2, t, R)

    br1 = p.copy()
    br1 /= np.linalg.norm(br1, axis=1)[:, np.newaxis]

    br2 = R.T.dot((p - t).T).T
    br2 /= np.linalg.norm(br2, axis=1)[:, np.newaxis]

    ok1 = np.linalg.norm(br1 - b1, axis=1) < 0.01   # TODO(pau): compute angular error and use proper threshold
    ok2 = np.linalg.norm(br2 - b2, axis=1) < 0.01
    return ok1 * ok2


def robust_match_calibrated(p1, p2, camera1, camera2, matches, config):
    '''Computes robust matches by estimating the Essential matrix via RANSAC.
    '''

    if len(matches) < 8:
        return np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()
    b1 = camera1.pixel_bearings(p1)
    b2 = camera2.pixel_bearings(p2)

    threshold = config['robust_matching_threshold']
    T = pyopengv.relative_pose_ransac(b1, b2, "STEWENIUS", 1 - np.cos(threshold), 1000)

    inliers = compute_inliers_bearings(b1, b2, T)

    return matches[inliers]


def robust_match(p1, p2, camera1, camera2, matches, config):
    if (camera1.projection_type == 'perspective'
            and camera1.k1 == 0.0
            and camera2.projection_type == 'perspective'
            and camera2.k1 == 0.0):
        return robust_match_fundamental(p1, p2, matches, config)
    else:
        return robust_match_calibrated(p1, p2, camera1, camera2, matches, config)


def good_track(track, min_length):
    if len(track) < min_length:
        return False
    images = [f[0] for f in track]
    if len(images) != len(set(images)):
        return False
    return True


def create_tracks_graph(features, colors, matches, config):
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

    tracks = [t for t in sets.values() if good_track(t, config['min_track_length'])]
    logger.debug('Good tracks: {}'.format(len(tracks)))

    tracks_graph = nx.Graph()
    for track_id, track in enumerate(tracks):
        for image, featureid in track:
            if image not in features:
                continue
            x, y = features[image][featureid]
            r, g, b = colors[image][featureid]
            tracks_graph.add_node(image, bipartite=0)
            tracks_graph.add_node(str(track_id), bipartite=1)
            tracks_graph.add_edge(image,
                                  str(track_id),
                                  feature=(x, y),
                                  feature_id=featureid,
                                  feature_color=(float(r), float(g), float(b)))

    return tracks_graph


def tracks_and_images(graph):
    """List of tracks and images in the graph."""
    tracks, images = [], []
    for n in graph.nodes(data=True):
        if n[1]['bipartite'] == 0:
            images.append(n[0])
        else:
            tracks.append(n[0])
    return tracks, images


def common_tracks(g, im1, im2):
    """
    Return the list of tracks observed in both images
    :param g: Graph structure (networkx) as returned by :method:`DataSet.tracks_graph`
    :param im1: Image name, with extension (i.e. 123.jpg)
    :param im2: Image name, with extension (i.e. 123.jpg)
    :return: tuple: track, feature from first image, feature from second image
    """
    t1, t2 = g[im1], g[im2]
    tracks, p1, p2 = [], [], []
    for track in t1:
        if track in t2:
            p1.append(t1[track]['feature'])
            p2.append(t2[track]['feature'])
            tracks.append(track)
    p1 = np.array(p1)
    p2 = np.array(p2)
    return tracks, p1, p2


def all_common_tracks(graph, tracks, include_features=True, min_common=50):
    """List of tracks observed by each image pair.

    :param graph: Graph structure (networkx) as returned by :method:`DataSet.tracks_graph`
    :param tracks: list of track identifiers
    :param include_features: whether to include the features from the images
    :param min_common: the minimum number of tracks the two images need to have in common
    :return: tuple: im1, im2 -> tuple: tracks, features from first image, features from second image
    """
    track_dict = defaultdict(list)
    for track in tracks:
        track_images = sorted(graph[track].keys())
        for im1, im2 in combinations(track_images, 2):
            track_dict[im1, im2].append(track)

    common_tracks = {}
    for k, v in track_dict.iteritems():
        if len(v) < min_common:
            continue
        im1, im2 = k
        if include_features:
            p1 = np.array([graph[im1][tr]['feature'] for tr in v])
            p2 = np.array([graph[im2][tr]['feature'] for tr in v])
            common_tracks[im1, im2] = (v, p1, p2)
        else:
            common_tracks[im1, im2] = v
    return common_tracks
