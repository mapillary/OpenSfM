# pyre-strict
from functools import lru_cache
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
from numpy.typing import NDArray
from opensfm import bow, feature_loader, pyfeatures
from opensfm.dataset_base import DataSetBase


def unnormalized_vlad(features: NDArray, centers: NDArray) -> Optional[NDArray]:
    """Compute unnormalized VLAD histograms from a set of
    features in relation to centers.

    Returns the unnormalized VLAD vector.
    """
    correct_dims = centers.shape[1] == features.shape[1]
    correct_type = centers.dtype == features.dtype
    if not correct_dims or not correct_type:
        return None
    return pyfeatures.compute_vlad_descriptor(features, centers)


def signed_square_root_normalize(v: NDArray) -> NDArray:
    """Compute Signed Square Root (SSR) normalization on
    a vector.

    Returns the SSR normalized vector.
    """
    v = np.sign(v) * np.sqrt(np.abs(v))
    v /= np.linalg.norm(v)
    return v


def vlad_distances(
    image: str, other_images: Iterable[str], histograms: Dict[str, NDArray]
) -> Tuple[str, List[float], List[str]]:
    """Compute VLAD-based distance (L2 on VLAD-histogram)
    between an image and other images.

    Returns the image, the order of the other images,
    and the other images.
    """

    # avoid passing a gigantic VLAD dictionary in case of preemption : copy instead
    ratio_copy = 0.5
    need_copy = len(other_images) < ratio_copy*len(histograms)
    candidates = {k: histograms[k] for k in other_images + [image]} if need_copy else histograms

    distances, others = pyfeatures.compute_vlad_distances(
        candidates, image, set(other_images)
    )
    return image, distances, others


class VladCache:
    def clear_cache(self) -> None:
        self.load_words.cache_clear()
        self.vlad_histogram.cache_clear()

    @lru_cache(1)
    def load_words(self, data: DataSetBase) -> NDArray:
        words, _ = bow.load_vlad_words_and_frequencies(data.config)
        return words

    @lru_cache(1000)
    def vlad_histogram(self, data: DataSetBase, image: str) -> Optional[NDArray]:
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
