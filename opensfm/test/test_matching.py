# pyre-unsafe
from typing import Any, Dict, List, Set, Tuple

import numpy as np
from opensfm import bow
from opensfm import config
from opensfm import matching
from opensfm import pairs_selection
from opensfm import pyfeatures
from opensfm.synthetic_data import synthetic_dataset


def compute_words(features: np.ndarray, bag_of_words, num_words, bow_matcher_type) -> np.ndarray:
    closest_words = bag_of_words.map_to_words(features, num_words, bow_matcher_type)
    if closest_words is None:
        return np.array([], dtype=np.int32)
    else:
        return closest_words.astype(np.int32)


def example_features(
    nfeatures: int, config: Dict[str, Any]
) -> Tuple[List[np.ndarray], List[np.ndarray]]:
    words, frequencies = bow.load_bow_words_and_frequencies(config)
    bag_of_words = bow.BagOfWords(words, frequencies)

    # features 1
    f1 = np.random.normal(size=(nfeatures, 128)).astype(np.float32)
    f1 /= np.linalg.norm(f1)
    w1 = compute_words(f1, bag_of_words, config["bow_words_to_match"], "FLANN")

    # features 2
    f2 = f1 + np.random.normal(size=f1.shape).astype(np.float32) / 500.0
    f2 /= np.linalg.norm(f2)
    w2 = compute_words(f2, bag_of_words, config["bow_words_to_match"], "FLANN")

    return [f1, f2], [w1, w2]


def test_example_features() -> None:
    nfeatures = 1000

    features, words = example_features(nfeatures, config.default_config())
    assert len(features[0]) == nfeatures
    assert len(words[0]) == nfeatures


def test_match_using_words() -> None:
    configuration = config.default_config()
    nfeatures = 1000

    features, words = example_features(nfeatures, configuration)
    matches = pyfeatures.match_using_words(
        features[0],
        words[0],
        features[1],
        words[1][:, 0],
        configuration["lowes_ratio"],
        configuration["bow_num_checks"],
    )
    assert len(matches) == nfeatures
    for i, j in matches:
        assert i == j


def test_unfilter_matches() -> None:
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


def test_match_images(scene_synthetic) -> None:
    reference = scene_synthetic.reconstruction
    synthetic = synthetic_dataset.SyntheticDataSet(
        reference,
        scene_synthetic.exifs,
        scene_synthetic.features,
        scene_synthetic.tracks_manager,
    )

    # pyre-fixme[8]: Attribute has type
    #  `BoundMethod[typing.Callable(SyntheticDataSet.matches_exists)[[Named(self,
    #  SyntheticDataSet), Named(image, str)], bool], SyntheticDataSet]`; used as `(im:
    #  Any) -> bool`.
    synthetic.matches_exists = lambda im: False
    # pyre-fixme[8]: Attribute has type
    #  `BoundMethod[typing.Callable(DataSet.save_matches)[[Named(self, DataSet),
    #  Named(image, str), Named(matches, Dict[str, ndarray])], None],
    #  SyntheticDataSet]`; used as `(im: Any, m: Any) -> bool`.
    synthetic.save_matches = lambda im, m: False

    override = {}
    override["matching_gps_neighbors"] = 0
    override["matching_gps_distance"] = 0
    override["matching_time_neighbors"] = 2

    images = sorted(synthetic.images())
    pairs, _ = matching.match_images(synthetic, override, images, images)
    matching.save_matches(synthetic, images, pairs)

    for i in range(len(images) - 1):
        pair = images[i], images[i + 1]
        matches = pairs.get(pair)
        if matches is None or len(matches) == 1:
            matches = pairs.get(pair[::-1])
        assert matches is not None
        assert len(matches) > 25


def test_ordered_pairs() -> None:
    neighbors: Set[Tuple[str, str]] = {
        ("1", "3"),
        ("1", "2"),
        ("2", "5"),
        ("3", "2"),
        ("4", "5"),
    }
    images = ["1", "2", "3"]
    pairs = pairs_selection.ordered_pairs(neighbors, images)

    assert {tuple(sorted(p)) for p in pairs} == {
        ("1", "2"),
        ("1", "3"),
        ("2", "5"),
        ("2", "3"),
    }


def test_triangulation_inliers(pairs_and_their_E) -> None:
    for f1, f2, _, pose in pairs_and_their_E:
        Rt = pose.get_cam_to_world()[:3]

        count_outliers = np.random.randint(0, len(f1) / 10)
        f1[:count_outliers, :] += np.random.uniform(0, 1e-1, size=(count_outliers, 3))

        inliers = matching.compute_inliers_bearings(f1, f2, Rt[:, :3], Rt[:, 3])
        assert sum(inliers) >= len(f1) - count_outliers
