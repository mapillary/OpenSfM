import logging
from functools import lru_cache

import numpy as np

from opensfm import features as ft


logger = logging.getLogger(__name__)


class FeatureLoader(object):
    def clear_cache(self):
        self.load_mask.cache_clear()
        self.load_points_colors.cache_clear()
        self._load_points_features_colors_unmasked.cache_clear()
        self._load_points_features_colors_masked.cache_clear()
        self.load_features_index.cache_clear()
        self.load_words.cache_clear

    @lru_cache(1000)
    def load_mask(self, data, image):
        points, _, _ = self._load_points_features_colors_unmasked(data, image)
        if points is None:
            return None
        return data.load_features_mask(image, points[:, :2])

    @lru_cache(1000)
    def load_points_colors(self, data, image):
        points, _, colors = self._load_features_nocache(data, image)
        return points, colors

    def load_points_features_colors(self, data, image, masked):
        if masked:
            return self._load_points_features_colors_masked(data, image)
        else:
            return self._load_points_features_colors_unmasked(data, image)

    @lru_cache(20)
    def _load_points_features_colors_unmasked(self, data, image):
        points, features, colors = self._load_features_nocache(data, image)
        return points, features, colors

    @lru_cache(200)
    def _load_points_features_colors_masked(self, data, image):
        points, features, colors = self._load_points_features_colors_unmasked(data, image)
        mask = self.load_mask(data, image)
        if mask is not None:
            points = points[mask]
            features = features[mask]
            colors = colors[mask]
        return points, features, colors

    @lru_cache(200)
    def load_features_index(self, data, image, masked):
        _, features, _ = self.load_points_features_colors(data, image, masked)
        return features, ft.build_flann_index(features, data.config)

    @lru_cache(200)
    def load_words(self, data, image, masked):
        words = data.load_words(image)
        if masked:
            mask = self.load_mask(data, image)
            if mask is not None:
                words = words[mask]
        return words

    def _load_features_nocache(self, data, image):
        points, features, colors = data.load_features(image)
        if points is None:
            logger.error('Could not load features for image {}'.format(image))
        else:
            points = np.array(points[:, :3], dtype=float)
        return points, features, colors
