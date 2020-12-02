import os.path

import pytest
from opensfm import commands, dataset, feature_loader, pairs_selection
from opensfm.test import data_generation


NEIGHBORS = 6


class Args:
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
    src = os.path.join(data_generation.DATA_PATH, "lund", "images")
    path = str(tmpdir_factory.mktemp("lund"))
    os.symlink(src, os.path.join(path, "images"))

    # Use words matcher type to support the bow retrieval test
    data_generation.save_config({"matcher_type": "WORDS"}, path)

    args = Args(path)
    data = dataset.DataSet(path)
    commands.extract_metadata.Command().run(data, args)
    commands.detect_features.Command().run(data, args)

    return path


def match_candidates_from_metadata(data, neighbors=NEIGHBORS, assert_count=NEIGHBORS):
    assert neighbors >= assert_count

    ims = sorted(data.images())
    ims_ref = ims[:1]
    ims_cand = ims[1:]

    exifs = {im: data.load_exif(im) for im in ims}

    pairs, _ = pairs_selection.match_candidates_from_metadata(
        ims_ref,
        ims_cand,
        exifs,
        data,
        {},
    )

    matches = [p[1] for p in pairs]
    names = ["{}.jpg".format(str(i).zfill(2)) for i in range(2, 2 + neighbors)]
    count = 0
    for name in names:
        if name in matches:
            count += 1

    assert count >= assert_count


def create_match_candidates_config(**kwargs):
    config = {
        "matcher_type": "BRUTEFORCE",
        "matching_gps_distance": 0,
        "matching_gps_neighbors": 0,
        "matching_time_neighbors": 0,
        "matching_order_neighbors": 0,
        "matching_bow_neighbors": 0,
        "matching_vlad_neighbors": 0,
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
        matching_bow_neighbors=NEIGHBORS, matcher_type="WORDS"
    )
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
