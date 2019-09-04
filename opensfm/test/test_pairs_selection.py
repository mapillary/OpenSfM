from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import numpy as np
import os.path
import pytest

from six import iteritems

from opensfm import commands
from opensfm import dataset
from opensfm import pairs_selection
from opensfm import feature_loader

from opensfm.test import data_generation


NEIGHBORS = 6


class Args():
    def __init__(self, dataset):
        self.dataset = dataset


@pytest.fixture(scope="module", autouse=True)
def clear_cache():
    """
    Clear feature loader cache to avoid using cached
    masks etc from berlin dataset which has the same
    naming convention for images.
    """
    feature_loader.instance.clear_cache()
    yield
    feature_loader.instance.clear_cache()


@pytest.fixture(scope="module", autouse=True)
def lund_path(tmpdir_factory):
    """
    Precompute exif and features to avoid doing
    it for every test which is time consuming.
    """
    path = str(tmpdir_factory.mktemp('lund'))

    os.symlink(os.path.abspath('data/lund/images'),
               os.path.join(path, 'images'))

    # Use words matcher type to support the bow retrieval test
    data_generation.save_config({'matcher_type': 'WORDS'}, path)

    args = Args(path)
    commands.extract_metadata.Command().run(args)
    commands.detect_features.Command().run(args)

    return path


def test_vlad_distances_order():
    im = 'im1'
    other_ims = ['im2', 'im3']

    histograms = {
        'im1': np.array([1, 0, 0]),
        'im2': np.array([0, 1, 0]),
        'im3': np.array([1, 1, 0]) / np.linalg.norm([1, 1, 0]),
    }

    im_res, order_res, other_res = pairs_selection.vlad_distances(
        im, other_ims, histograms)

    assert im_res == im
    assert len(order_res) == len(other_ims)
    assert other_res == other_ims

    assert other_ims[order_res[0]] == 'im3'
    assert other_ims[order_res[1]] == 'im2'


def test_signed_square_root_normalize():
    v = [1, 0.01]
    res = pairs_selection.signed_square_root_normalize(v)

    assert pytest.approx(np.linalg.norm(res), 1e-6) == 1
    assert pytest.approx(v[0] / v[1], 1e-6) == 10 * res[0] / res[1]


def test_unnormalized_vlad():
    features = np.array([[0, 1.1]])
    centers = np.array([
        [1, 0],
        [0, 1],
    ])

    res = pairs_selection.unnormalized_vlad(features, centers)

    assert res[0] == res[1] == res[2] == 0
    assert pytest.approx(res[3], 1e-6) == 0.1


def match_candidates_from_metadata(data, neighbors=NEIGHBORS, assert_count=NEIGHBORS):
    assert neighbors >= assert_count

    args = Args(data.data_path)
    commands.extract_metadata.Command().run(args)
    commands.detect_features.Command().run(args)

    ims = sorted(data.images())
    ims_ref = ims[:1]
    ims_cand = ims[1:]

    exifs = { im: data.load_exif(im) for im in ims }

    pairs, _ = pairs_selection.match_candidates_from_metadata(
        ims_ref, ims_cand, exifs, data)

    matches = [p[1] for p in pairs]
    names = ['{}.jpg'.format(str(i).zfill(2)) for i in range(2, 2+neighbors)]
    count = 0
    for name in names:
        if name in matches:
            count += 1

    assert count >= assert_count


def create_match_candidates_config(**kwargs):
    config = {
        'matcher_type': 'BRUTEFORCE',
        'matching_gps_distance': 0,
        'matching_gps_neighbors': 0,
        'matching_time_neighbors': 0,
        'matching_order_neighbors': 0,
        'matching_bow_neighbors': 0,
        'matching_vlad_neighbors': 0,
    }

    for key, value in kwargs.items():
        config[key] = value

    return config


def test_match_candidates_from_metadata_vlad(lund_path):
    config = create_match_candidates_config(matching_vlad_neighbors=NEIGHBORS)
    data_generation.save_config(config, lund_path)
    data = dataset.DataSet(lund_path)
    match_candidates_from_metadata(data, assert_count=5)


def test_match_candidates_from_metadata_bow(lund_path):
    config = create_match_candidates_config(
        matching_bow_neighbors=NEIGHBORS,
        matcher_type='WORDS')
    data_generation.save_config(config, lund_path)
    data = dataset.DataSet(lund_path)
    match_candidates_from_metadata(data, assert_count=5)


def test_match_candidates_from_metadata_gps(lund_path):
    config = create_match_candidates_config(matching_gps_neighbors=NEIGHBORS)
    data_generation.save_config(config, lund_path)
    data = dataset.DataSet(lund_path)
    match_candidates_from_metadata(data)


def test_match_candidates_from_metadata_time(lund_path):
    config = create_match_candidates_config(matching_time_neighbors=NEIGHBORS)
    data_generation.save_config(config, lund_path)
    data = dataset.DataSet(lund_path)
    match_candidates_from_metadata(data)
