from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import numpy as np
from six import iteritems

from opensfm import config
from opensfm import matching
from opensfm import pairs_selection
from opensfm import bow
from opensfm import pyfeatures
from opensfm.synthetic_data import synthetic_dataset
from opensfm.test import data_generation


def compute_words(features, bag_of_words, num_words, bow_matcher_type):
    closest_words = bag_of_words.map_to_words(
        features, num_words, bow_matcher_type)
    if closest_words is None:
        return np.array([], dtype=np.int32)
    else:
        return closest_words.astype(np.int32)


def example_features(nfeatures, config):
    words, frequencies = bow.load_bow_words_and_frequencies(config)
    bag_of_words = bow.BagOfWords(words, frequencies)

    # features 1
    f1 = np.random.normal(size=(nfeatures, 128)).astype(np.float32)
    f1 /= np.linalg.norm(f1)
    w1 = compute_words(f1, bag_of_words, config['bow_words_to_match'],
                       'FLANN')

    # features 2
    f2 = f1 + np.random.normal(size=f1.shape).astype(np.float32) / 500.0
    f2 /= np.linalg.norm(f2)
    w2 = compute_words(f2, bag_of_words, config['bow_words_to_match'],
                       'FLANN')

    return [f1, f2], [w1, w2]


def test_example_features():
    nfeatures = 1000

    features, words = example_features(nfeatures, config.default_config())
    assert len(features[0]) == nfeatures
    assert len(words[0]) == nfeatures


def test_match_using_words():
    configuration = config.default_config()
    nfeatures = 1000

    features, words = example_features(nfeatures, configuration)
    matches = pyfeatures.match_using_words(features[0], words[0],
                                           features[1], words[1][:, 0],
                                           configuration['lowes_ratio'],
                                           configuration['bow_num_checks'])
    assert len(matches) == nfeatures
    for i, j in matches:
        assert i == j

        
def test_unfilter_matches():
    matches = np.array([])
    m1 = np.array([], dtype=bool)
    m2 = np.array([], dtype=bool)
    res = matching.unfilter_matches(matches, m1, m2)
    assert len(res) == 0

    matches = np.array([[0, 2], [2, 1]])
    i1 = np.array([False, False, False, True, False, True, False, True, False])
    i2 = np.array([False, False, False, False, True, False, True, False, True])
    res = matching.unfilter_matches(matches, i1, i2)
    assert len(res) == 2
    assert res[0][0] == 3
    assert res[0][1] == 8
    assert res[1][0] == 7
    assert res[1][1] == 6


def test_match_images(scene_synthetic):
    reference = scene_synthetic[0].get_reconstruction()
    synthetic = synthetic_dataset.SyntheticDataSet(reference,
                                                   scene_synthetic[1],
                                                   scene_synthetic[2],
                                                   scene_synthetic[3],
                                                   scene_synthetic[4],
                                                   scene_synthetic[5])

    synthetic.matches_exists = lambda im: False
    synthetic.save_matches = lambda im, m: False

    synthetic.config['matching_gps_neighbors'] = 0
    synthetic.config['matching_gps_distance'] = 0
    synthetic.config['matching_time_neighbors'] = 2

    images = sorted(synthetic.images())
    pairs, _ = matching.match_images(synthetic, images, images)
    matching.save_matches(synthetic, images, pairs)

    for i in range(len(images) - 1):
        pair = images[i], images[i + 1]
        matches = pairs.get(pair)
        if matches is None or len(matches) == 1:
            matches = pairs.get(pair[::-1])
        assert len(matches) > 25


def test_ordered_pairs():
    neighbors = [
        [1, 3],
        [1, 2],
        [2, 5],
        [3, 2],
        [4, 5],
        ]
    images = [1, 2, 3]
    pairs = pairs_selection.ordered_pairs(neighbors, images)
    assert set(pairs) == {(1, 2), (1, 3), (2, 5), (3, 2)}
