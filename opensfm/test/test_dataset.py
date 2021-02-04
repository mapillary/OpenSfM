import numpy as np
from opensfm.test import data_generation


def test_dataset_load_features_sift(tmpdir):
    data = data_generation.create_berlin_test_folder(tmpdir)

    assert len(data.images()) == 3

    data.config["feature_type"] = "SIFT"

    image = data.images()[0]
    points = np.random.random((3, 4))
    descriptors = np.random.random((128, 4))
    colors = np.random.random((3, 4))
    segmentations = np.random.random((3, 4))
    instances = np.random.random((3, 4))
    data.save_features(image, points, descriptors, colors, segmentations, instances)

    p, d, c, s = data.load_features(image)

    assert np.allclose(p, points)
    assert np.allclose(d, descriptors)
    assert np.allclose(c, colors)
    assert np.allclose(s["segmentations"], segmentations)
    assert np.allclose(s["instances"], instances)
