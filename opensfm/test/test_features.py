import io
from typing import Any

import numpy as np
import pytest
from opensfm import config as osfm_config, features


def _config(**overrides: Any) -> dict[str, Any]:
    config = osfm_config.default_config()
    config.update(overrides)
    return config


def _features_data(with_semantic: bool) -> features.FeaturesData:
    points = np.array(
        [[0.1, 0.2, 0.01, 0.0], [0.3, -0.1, 0.02, 1.0], [-0.2, 0.4, 0.03, 2.0]]
    )
    descriptors = np.arange(12, dtype=np.float32).reshape(3, 4)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]])
    semantic = None
    if with_semantic:
        semantic = features.SemanticData(
            segmentation=np.array([1, 2, 3]),
            instances=np.array([10, 20, 30]),
            labels=[{"name": "a"}, {"name": "b"}],
        )
    return features.FeaturesData(points, descriptors, colors, semantic)


def _textured_image(size: int = 200) -> np.ndarray:
    rng = np.random.default_rng(0)
    image = np.zeros((size, size), dtype=np.uint8)
    # Random filled squares give plenty of corners for every detector.
    for _ in range(60):
        x, y = rng.integers(0, size - 20, size=2)
        w = int(rng.integers(5, 20))
        image[y : y + w, x : x + w] = rng.integers(50, 255)
    return image


def test_semantic_data_mask_filters_segmentation_and_instances() -> None:
    semantic = features.SemanticData(
        np.array([1, 2, 3]), np.array([4, 5, 6]), [{"id": 1}]
    )
    masked = semantic.mask(np.array([True, False, True]))
    assert masked.segmentation.tolist() == [1, 3]
    instances = masked.instances
    assert instances is not None
    assert instances.tolist() == [4, 6]
    assert masked.labels == [{"id": 1}]
    assert masked.has_instances()


def test_semantic_data_mask_without_instances() -> None:
    semantic = features.SemanticData(np.array([1, 2, 3]), None, [])
    masked = semantic.mask(np.array([0, 2]))
    assert masked.segmentation.tolist() == [1, 3]
    assert masked.instances is None
    assert not masked.has_instances()


def test_semantic_data_mask_invalid_mask_raises() -> None:
    semantic = features.SemanticData(np.array([1, 2, 3]), None, [])
    with pytest.raises(IndexError):
        semantic.mask(np.array([7]))


def test_features_data_accessors() -> None:
    with_semantic = _features_data(with_semantic=True)
    segmentation = with_semantic.get_segmentation()
    assert segmentation is not None
    assert segmentation.tolist() == [1, 2, 3]
    assert with_semantic.has_instances()

    without_semantic = _features_data(with_semantic=False)
    assert without_semantic.get_segmentation() is None
    assert not without_semantic.has_instances()


def test_features_data_mask_applies_to_all_fields() -> None:
    data = _features_data(with_semantic=True)
    data.depths = np.array([1.0, 2.0, 3.0])
    masked = data.mask(np.array([False, True, True]))
    assert np.allclose(masked.points, data.points[1:])
    descriptors = masked.descriptors
    assert descriptors is not None
    assert np.allclose(descriptors, np.arange(4, 12).reshape(2, 4))
    assert masked.colors.tolist() == [[0, 255, 0], [0, 0, 255]]
    depths = masked.depths
    assert depths is not None
    assert depths.tolist() == [2.0, 3.0]
    semantic = masked.semantic
    assert semantic is not None
    assert semantic.segmentation.tolist() == [2, 3]


def test_features_data_mask_without_optional_fields() -> None:
    data = _features_data(with_semantic=False)
    data.descriptors = None
    masked = data.mask(np.array([True, False, False]))
    assert masked.descriptors is None
    assert masked.semantic is None
    assert masked.depths is None
    assert len(masked.points) == 1


@pytest.mark.parametrize("with_semantic", [False, True])
def test_features_data_save_load_roundtrip(with_semantic: bool) -> None:
    config = _config(feature_type="SIFT")
    data = _features_data(with_semantic)
    buffer = io.BytesIO()
    data.save(buffer, config)
    buffer.seek(0)

    loaded = features.FeaturesData.from_file(buffer, config)
    assert np.allclose(loaded.points, data.points)
    descriptors = loaded.descriptors
    assert descriptors is not None
    assert descriptors.dtype == np.float32
    assert np.allclose(descriptors, np.arange(12).reshape(3, 4))
    assert np.allclose(loaded.colors, data.colors)
    if with_semantic:
        semantic = loaded.semantic
        assert semantic is not None
        assert semantic.segmentation.tolist() == [1, 2, 3]
        instances = semantic.instances
        assert instances is not None
        assert instances.tolist() == [10, 20, 30]
        assert len(semantic.labels) == 2
    else:
        assert loaded.semantic is None


@pytest.mark.parametrize(
    "overrides",
    [
        {"feature_type": "ORB"},
        {"feature_type": "AKAZE", "akaze_descriptor": "MLDB"},
        {"feature_type": "HAHOG", "hahog_normalize_to_uchar": True},
    ],
)
def test_features_data_save_uses_uint8_for_binary_descriptors(
    overrides: dict[str, Any],
) -> None:
    buffer = io.BytesIO()
    _features_data(with_semantic=False).save(buffer, _config(**overrides))
    buffer.seek(0)
    raw = np.load(buffer)
    assert raw["descriptors"].dtype == np.uint8
    assert raw["OPENSFM_FEATURES_VERSION"] == features.FeaturesData.FEATURES_VERSION


def test_features_data_save_without_descriptors_raises() -> None:
    data = _features_data(with_semantic=False)
    data.descriptors = None
    with pytest.raises(RuntimeError, match="No descriptors"):
        data.save(io.BytesIO(), _config())


def _npz(**arrays: Any) -> io.BytesIO:
    buffer = io.BytesIO()
    np.savez(buffer, **arrays)
    buffer.seek(0)
    return buffer


def test_from_file_v0_overrides_scale_with_reprojection_sd() -> None:
    config = _config(
        feature_type="HAHOG", hahog_normalize_to_uchar=True, reprojection_error_sd=0.5
    )
    buffer = _npz(
        points=np.array([[0.1, 0.2, 9.0, 0.0]]),
        descriptors=np.array([[1, 2]], dtype=np.uint8),
        colors=np.array([[1, 2, 3]], dtype=np.uint8),
    )
    loaded = features.FeaturesData.from_file(buffer, config)
    assert loaded.points[0, 2] == 0.5
    descriptors = loaded.descriptors
    assert descriptors is not None
    assert descriptors.dtype == np.float32
    assert loaded.colors.dtype == float
    assert loaded.semantic is None


def test_from_file_v1_keeps_scale() -> None:
    buffer = _npz(
        points=np.array([[0.1, 0.2, 9.0, 0.0]]),
        descriptors=np.array([[1.0, 2.0]], dtype=np.float32),
        colors=np.array([[1, 2, 3]]),
        OPENSFM_FEATURES_VERSION=1,
    )
    loaded = features.FeaturesData.from_file(buffer, _config(feature_type="SIFT"))
    assert loaded.points[0, 2] == 9.0
    assert loaded.semantic is None


def test_from_file_v2_with_segmentation() -> None:
    buffer = _npz(
        points=np.array([[0.1, 0.2, 0.3, 0.0]]),
        descriptors=np.array([[1.0, 2.0]], dtype=np.float32),
        colors=np.array([[1, 2, 3]]),
        segmentations=np.array([5]),
        instances=np.array([7]),
        segmentation_labels=np.array(["road"]),
        OPENSFM_FEATURES_VERSION=2,
    )
    loaded = features.FeaturesData.from_file(buffer, _config(feature_type="SIFT"))
    semantic = loaded.semantic
    assert semantic is not None
    assert semantic.segmentation.tolist() == [5]
    instances = semantic.instances
    assert instances is not None
    assert instances.tolist() == [7]
    assert list(semantic.labels) == ["road"]


def test_from_file_v2_with_pickled_arrays_drops_semantic() -> None:
    # Version 2 files stored object arrays, which cannot be loaded safely.
    buffer = io.BytesIO()
    np.savez(
        buffer,
        points=np.array([[0.1, 0.2, 0.3, 0.0]]),
        descriptors=np.array([[1.0, 2.0]], dtype=np.float32),
        colors=np.array([[1, 2, 3]]),
        segmentations=np.array([None], dtype=object),
        instances=np.array([None], dtype=object),
        segmentation_labels=np.array([{"a": 1}], dtype=object),
        OPENSFM_FEATURES_VERSION=2,
    )
    buffer.seek(0)
    loaded = features.FeaturesData.from_file(buffer, _config(feature_type="SIFT"))
    assert loaded.semantic is None
    assert loaded.points.shape == (1, 4)


def test_from_file_v3_with_only_instances() -> None:
    buffer = _npz(
        points=np.array([[0.1, 0.2, 0.3, 0.0]]),
        descriptors=np.array([[1, 2]], dtype=np.uint8),
        colors=np.array([[1, 2, 3]]),
        segmentations=np.array([]),
        instances=np.array([3]),
        segmentation_labels=np.array([]),
        OPENSFM_FEATURES_VERSION=3,
    )
    config = _config(feature_type="HAHOG", hahog_normalize_to_uchar=True)
    loaded = features.FeaturesData.from_file(buffer, config)
    semantic = loaded.semantic
    assert semantic is not None
    assert semantic.segmentation is None
    assert semantic.has_instances()
    descriptors = loaded.descriptors
    assert descriptors is not None
    assert descriptors.dtype == np.float32


def test_resized_image() -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    assert features.resized_image(image, 50).shape == (25, 50, 3)
    assert features.resized_image(image, 400) is image
    assert features.resized_image(image, -1) is image


def test_root_feature() -> None:
    desc = np.array([[1.0, 3.0], [4.0, 0.0]])
    result = features.root_feature(desc.copy())
    assert np.allclose(result, [[0.5, np.sqrt(0.75)], [1.0, 0.0]])
    # Hellinger mapping produces unit L2 norm rows.
    assert np.allclose(np.linalg.norm(result, axis=1), 1.0)

    l2 = features.root_feature(np.array([[3.0, 4.0]]), l2_normalization=True)
    assert np.allclose(l2, np.sqrt(np.array([[3.0, 4.0]]) / 7.0))


@pytest.mark.parametrize("partial", [False, True])
def test_root_feature_surf(partial: bool) -> None:
    rng = np.random.default_rng(1)
    desc = rng.random((2, 64)) - 0.5
    original = desc.copy()
    result = features.root_feature_surf(desc.copy(), partial=partial)
    abs_sum = np.sum(np.abs(original), axis=1, keepdims=True)
    expected = np.sign(original) * np.sqrt(np.abs(original) / abs_sum)
    changed = (
        np.array([i % 4 in (2, 3) for i in range(64)]) if partial else np.ones(64, bool)
    )
    assert np.allclose(result[:, changed], expected[:, changed])
    assert np.allclose(result[:, ~changed], original[:, ~changed])


def test_root_feature_surf_l2_normalization() -> None:
    desc = np.ones((1, 64))
    result = features.root_feature_surf(desc.copy(), l2_normalization=True)
    assert np.allclose(result, np.sqrt(1.0 / 64))


def test_root_feature_surf_ignores_non_64_dim() -> None:
    desc = np.ones((2, 32))
    assert np.array_equal(features.root_feature_surf(desc.copy()), desc)


def test_normalized_and_denormalized_coordinates_roundtrip() -> None:
    pixels = np.array([[0.0, 0.0], [199.0, 99.0], [99.5, 49.5]])
    normalized = features.normalized_image_coordinates(pixels, 200, 100)
    assert np.allclose(normalized[0], [(0.5 - 100) / 200, (0.5 - 50) / 200])
    assert np.allclose(normalized[2], [0.0, 0.0])
    back = features.denormalized_image_coordinates(normalized, 200, 100)
    assert np.allclose(back, pixels)


def test_normalize_features_scales_size() -> None:
    points = np.array([[99.5, 49.5, 20.0, 0.3]])
    desc = np.zeros((1, 2))
    colors = np.zeros((1, 3))
    p, d, c = features.normalize_features(points, desc, colors, 200, 100)
    assert np.allclose(p, [[0.0, 0.0, 0.1, 0.3]])
    assert d is desc and c is colors


def test_in_mask() -> None:
    mask = np.zeros((10, 10), dtype=np.uint8)
    mask[:5, :5] = 1
    # Image is twice the mask resolution.
    assert features._in_mask(np.array([2.0, 2.0]), 20, 20, mask)
    assert not features._in_mask(np.array([15.0, 15.0]), 20, 20, mask)


@pytest.mark.parametrize("feature_type", ["SIFT", "ORB", "AKAZE", "HAHOG"])
def test_extract_features(feature_type: str) -> None:
    config = _config(
        feature_type=feature_type,
        feature_process_size=-1,
        feature_min_frames=50,
        feature_root=True,
    )
    image = np.stack([_textured_image()] * 3, axis=2)
    points, desc, colors = features.extract_features(image, config, False)
    assert len(points) > 0
    assert len(points) == len(desc) == len(colors)
    # Normalized coordinates lie within [-0.5, 0.5].
    assert np.all(np.abs(points[:, :2]) <= 0.5 + 1e-6)
    assert colors.shape[1] == 3


def test_extract_features_grayscale_repeats_colors() -> None:
    config = _config(feature_type="ORB", feature_process_size=-1)
    points, _, colors = features.extract_features(_textured_image(), config, False)
    assert len(points) > 0
    assert colors.shape == (len(points), 3)
    assert np.all(colors[:, 0] == colors[:, 1])
    assert np.all(colors[:, 1] == colors[:, 2])


def test_extract_features_hahog_normalizes_to_uchar() -> None:
    config = _config(
        feature_type="HAHOG",
        feature_process_size=-1,
        hahog_normalize_to_uchar=True,
        feature_root=False,
    )
    _, desc, _ = features.extract_features(_textured_image(), config, False)
    assert desc.min() >= 0 and desc.max() <= 255
    assert np.allclose(desc, desc.round())


def test_extract_features_panorama_uses_panorama_size() -> None:
    config = _config(
        feature_type="ORB",
        feature_process_size=-1,
        feature_process_size_panorama=100,
    )
    points, _, _ = features.extract_features(_textured_image(), config, True)
    assert len(points) > 0


def test_extract_features_unknown_type() -> None:
    config = _config(feature_type="FOO", feature_process_size=-1)
    with pytest.raises(ValueError, match="Unknown feature type"):
        features.extract_features(_textured_image(), config, False)


def test_akaze_descriptor_type_falls_back_to_msurf() -> None:
    assert (
        features.akaze_descriptor_type("MLDB")
        == features.pyfeatures.AkazeDescriptorType.MLDB
    )
    assert (
        features.akaze_descriptor_type("NOT_A_TYPE")
        == features.pyfeatures.AkazeDescriptorType.MSURF
    )


@pytest.mark.parametrize("algorithm", ["KMEANS", "KDTREE"])
def test_build_flann_index_float_descriptors(algorithm: str) -> None:
    rng = np.random.default_rng(2)
    descriptors = rng.random((50, 8)).astype(np.float32)
    index = features.build_flann_index(descriptors, _config(flann_algorithm=algorithm))
    _, dists = index.knnSearch(descriptors[:3], 1, params={})
    assert np.allclose(dists, 0.0, atol=1e-5)


def test_build_flann_index_uint8_descriptors() -> None:
    rng = np.random.default_rng(3)
    descriptors = rng.integers(0, 255, size=(50, 32)).astype(np.uint8)
    index = features.build_flann_index(descriptors, _config())
    assert index is not None


def test_build_flann_index_errors() -> None:
    with pytest.raises(ValueError, match="Unknown flann algorithm"):
        features.build_flann_index(
            np.zeros((5, 4), dtype=np.float32), _config(flann_algorithm="FOO")
        )
    with pytest.raises(ValueError, match="FLANN isn't supported"):
        features.build_flann_index(np.zeros((5, 4), dtype=np.float64), _config())
