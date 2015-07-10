import numpy as np
import json
import cv2
import networkx as nx
import logging
from opensfm.unionfind import UnionFind


logger = logging.getLogger(__name__)


# pairwise matches
def match_lowe(index, f2, config):
    search_params = dict(checks=config.get('flann_checks', 200))
    results, dists = index.knnSearch(f2, 2, params=search_params)
    squared_ratio = config.get('lowes_ratio', 0.6)**2  # Flann returns squared L2 distances
    good = dists[:, 0] < squared_ratio * dists[:, 1]
    matches = zip(results[good, 0], good.nonzero()[0])
    return np.array(matches, dtype=int)


def match_symmetric(fi, indexi, fj, indexj, config):
    if config.get('matcher_type', 'FLANN') == 'FLANN':
        matches_ij = [(a,b) for a,b in match_lowe(indexi, fj, config)]
        matches_ji = [(b,a) for a,b in match_lowe(indexj, fi, config)]
    else:
        matches_ij = [(a,b) for a,b in match_lowe_bf(fi, fj, config)]
        matches_ji = [(b,a) for a,b in match_lowe_bf(fj, fi, config)]

    matches = set(matches_ij).intersection(set(matches_ji))
    return np.array(list(matches), dtype=int)


def convert_matches_to_vector(matches):
    '''Convert Dmatch object to matrix form
    '''
    matches_vector = np.zeros((len(matches),2),dtype=np.int)
    k = 0
    for mm in matches:
        matches_vector[k,0] = mm.queryIdx
        matches_vector[k,1] = mm.trainIdx
        k = k+1
    return matches_vector


def match_lowe_bf(f1, f2, config):
    '''Bruteforce feature matching
    '''
    assert(f1.dtype.type==f2.dtype.type)
    if (f1.dtype.type == np.uint8):
        matcher_type = 'BruteForce-Hamming'
    else:
        matcher_type = 'BruteForce'
    matcher = cv2.DescriptorMatcher_create(matcher_type)
    matches = matcher.knnMatch(f1, f2, k=2)

    ratio = config.get('lowes_ratio', 0.6)
    good_matches = []
    for match in matches:
        if match and len(match) == 2:
            m, n = match
            if m.distance < ratio * n.distance:
                good_matches.append(m)
    good_matches = convert_matches_to_vector(good_matches)
    return np.array(good_matches, dtype=int)


def robust_match(p1, p2, matches, config):
    '''Computes robust matches by estimating the Fundamental matrix via RANSAC.
    '''
    if len(matches) < 8:
        return np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()

    F, mask = cv2.findFundamentalMat(p1, p2, cv2.cv.CV_FM_RANSAC, config.get('robust_matching_threshold', 0.006), 0.9999)
    inliers = mask.ravel().nonzero()

    if F[2,2] == 0.0:
        return []

    return matches[inliers]


def good_track(track, min_length):
    if len(track) < min_length:
        return False
    images = [f[0] for f in track]
    if len(images) != len(set(images)):
        return False
    return True


def create_tracks_graph(features, colors, matches, config):
    logging.debug('Merging features onto tracks')
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

    tracks = [t for t in sets.values() if good_track(t, config.get('min_track_length', 2))]
    logging.debug('Good tracks: {}'.format(len(tracks)))

    tracks_graph = nx.Graph()
    for track_id, track in enumerate(tracks):
        for image_feature in track:
            image = image_feature[0]
            featureid = image_feature[1]
            x, y = features[image][featureid]
            r, g, b = colors[image][featureid]
            tracks_graph.add_node(image, bipartite=0)
            tracks_graph.add_node(str(track_id), bipartite=1)
            tracks_graph.add_edge(image, str(track_id), feature=(x,y), feature_id=featureid, feature_color=(float(r),float(g),float(b)))

    return tracks_graph

