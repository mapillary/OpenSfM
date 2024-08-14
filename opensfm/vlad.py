# pyre-unsafe
from functools import lru_cache
from typing import List, Tuple, Iterable, Dict, Optional

import numpy as np
from opensfm import pyfeatures, feature_loader, bow
from opensfm.dataset_base import DataSetBase


def unnormalized_vlad(
    features: np.ndarray, centers: np.ndarray
) -> Optional[np.ndarray]:
    """Compute unnormalized VLAD histograms from a set of
    features in relation to centers.

    Returns the unnormalized VLAD vector.
    """
    correct_dims = centers.shape[1] == features.shape[1]
    correct_type = centers.dtype == features.dtype
    if not correct_dims or not correct_type:
        return None
    return pyfeatures.compute_vlad_descriptor(features, centers)


def signed_square_root_normalize(v: np.ndarray) -> np.ndarray:
    """Compute Signed Square Root (SSR) normalization on
    a vector.

    Returns the SSR normalized vector.
    """
    v = np.sign(v) * np.sqrt(np.abs(v))
    v /= np.linalg.norm(v)
    return v


def vlad_distances(
    image: str, other_images: Iterable[str], histograms: Dict[str, np.ndarray]
) -> Tuple[str, List[float], List[str]]:
    """Compute VLAD-based distance (L2 on VLAD-histogram)
    between an image and other images.

    Returns the image, the order of the other images,
    and the other images.
    """
    distances, others = pyfeatures.compute_vlad_distances(
        histograms, image, set(other_images)
    )
    return image, distances, others


class VladCache:
    def clear_cache(self) -> None:
        self.load_words.cache_clear()
        self.vlad_histogram.cache_clear()

    @lru_cache(1)
    def load_words(self, data: DataSetBase) -> np.ndarray:
        words, _ = bow.load_vlad_words_and_frequencies(data.config)
        return words

    @lru_cache(1000)
    def vlad_histogram(self, data: DataSetBase, image: str) -> Optional[np.ndarray]:
        words = self.load_words(data)
        features_data = feature_loader.instance.load_all_data(
            data, image, masked=True, segmentation_in_descriptor=False
        )
        if features_data is None:
            return None
        descriptors = features_data.descriptors
        if descriptors is None:
            return None
        vlad = unnormalized_vlad(descriptors, words)
        if vlad is None:
            return None
        vlad = signed_square_root_normalize(vlad)
        return vlad


instance = VladCache()
