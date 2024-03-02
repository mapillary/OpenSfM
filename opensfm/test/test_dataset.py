# pyre-unsafe
import numpy as np
from opensfm import features
from opensfm.test import data_generation


def test_dataset_load_features_sift(tmpdir) -> None:
    data = data_generation.create_berlin_test_folder(tmpdir)

    assert len(data.images()) == 3

    data.config["feature_type"] = "SIFT"

    image = data.images()[0]
    points = np.random.random((3, 4))
    descriptors = np.random.random((128, 4))
    colors = np.random.random((3, 4))
    segmentations = np.random.randint(low=0, high=255, size=(3, 4))
    instances = np.random.randint(low=0, high=255, size=(3, 4))

    semantic_data = features.SemanticData(
        segmentations, instances, data.segmentation_labels()
    )
    before = features.FeaturesData(points, descriptors, colors, semantic_data)
    data.save_features(image, before)
    after = data.load_features(image)
    assert after
    assert np.allclose(points, after.points)
    assert np.allclose(descriptors, after.descriptors)
    assert np.allclose(colors, after.colors)
    semantic = after.semantic
    assert semantic
    assert np.allclose(
        segmentations,
        semantic.segmentation,
    )
    assert np.allclose(instances, semantic.instances)
