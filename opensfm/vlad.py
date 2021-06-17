from functools import lru_cache
from typing import List, Tuple, Iterable, Dict, Optional

import numpy as np
from opensfm import bow
from opensfm import feature_loader
from opensfm.dataset import DataSetBase


def unnormalized_vlad(features: np.ndarray, centers: np.ndarray) -> np.ndarray:
    """Compute unnormalized VLAD histograms from a set of
    features in relation to centers.

    Returns the unnormalized VLAD vector.
    """
    vlad = np.zeros(centers.shape, dtype=np.float32)
    for f in features:
        i = np.argmin(np.linalg.norm(f - centers, axis=1))
        vlad[i, :] += f - centers[i]
    return vlad.flatten()


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
    if image not in histograms:
        return image, [], []

    distances = []
    other = []
    h = histograms[image]
    for im2 in other_images:
        if im2 != image and im2 in histograms:
            h2 = histograms[im2]
            distances.append(np.linalg.norm(h - h2))
            other.append(im2)
    return image, distances, other


class VladCache(object):
    @lru_cache(1)
    def load_words(self, data: DataSetBase) -> np.ndarray:
        words, _ = bow.load_vlad_words_and_frequencies(data.config)
        return words

    @lru_cache(1000)
    def vlad_histogram(self, data: DataSetBase, image: str) -> Optional[np.ndarray]:
        words = self.load_words(data)
        features_data = feature_loader.instance.load_all_data(data, image, masked=True)
        if features_data is None:
            return None
        descriptors = features_data.descriptors
        if descriptors is None:
            return None
        vlad = unnormalized_vlad(descriptors, words)
        vlad = signed_square_root_normalize(vlad)
        return vlad


instance = VladCache()
