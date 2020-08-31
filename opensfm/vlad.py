from functools import lru_cache

import numpy as np

from opensfm import bow
from opensfm import feature_loader


def unnormalized_vlad(features, centers):
    """ Compute unnormalized VLAD histograms from a set of
        features in relation to centers.

        Returns the unnormalized VLAD vector.
    """
    vlad = np.zeros(centers.shape, dtype=np.float32)
    for f in features:
        i = np.argmin(np.linalg.norm(f-centers, axis=1))
        vlad[i, :] += f-centers[i]
    vlad = np.ndarray.flatten(vlad)
    return vlad


def signed_square_root_normalize(v):
    """ Compute Signed Square Root (SSR) normalization on
        a vector.

        Returns the SSR normalized vector.
    """
    v = np.sign(v) * np.sqrt(np.abs(v))
    v /= np.linalg.norm(v)
    return v


def vlad_distances(image, other_images, histograms):
    """ Compute VLAD-based distance (L2 on VLAD-histogram)
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
    def load_words(self, data):
        words, _ = bow.load_vlad_words_and_frequencies(data.config)
        return words

    @lru_cache(1000)
    def vlad_histogram(self, data, image):
        words = self.load_words(data)
        _, features, _ = feature_loader.instance.load_points_features_colors(
            data, image, masked=True)
        if features is None:
            return None
        vlad = unnormalized_vlad(features, words)
        vlad = signed_square_root_normalize(vlad)
        return vlad


instance = VladCache()
