from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import logging

import numpy as np
from repoze.lru import LRUCache

from opensfm import features as ft


logger = logging.getLogger(__name__)


class FeatureLoader(object):
    def __init__(self):
        self.points_cache = LRUCache(1000)
        self.colors_cache = LRUCache(1000)
        self.features_cache = LRUCache(200)
        self.words_cache = LRUCache(200)
        self.masks_cache = LRUCache(1000)
        self.index_cache = LRUCache(200)

    def clear_cache(self):
        self.points_cache.clear()
        self.colors_cache.clear()
        self.features_cache.clear()
        self.words_cache.clear()
        self.masks_cache.clear()

    def load_points_colors(self, data, image):
        points = self.points_cache.get(image)
        colors = self.colors_cache.get(image)
        if points is None or colors is None:
            points, _, colors = self._load_features_nocache(data, image)
            self.points_cache.put(image, points)
            self.colors_cache.put(image, colors)
        return points, colors

    def load_masks(self, data, image):
        points, _ = self.load_points_colors(data, image)
        masks = self.masks_cache.get(image)
        if masks is None:
            masks = data.load_features_mask(image, points[:, :2])
            self.masks_cache.put(image, masks)
        return masks

    def load_features_index(self, data, image, features):
        index = self.index_cache.get(image)
        current_features = self.load_points_features_colors(data, image)
        use_load = len(current_features) == len(features) and index is None
        use_rebuild = len(current_features) != len(features)
        if use_load:
            index = data.load_feature_index(image, features)
        if use_rebuild:
            index = ft.build_flann_index(features, data.config)
        if use_load or use_rebuild:
            self.index_cache.put(image, index)
        return index

    def load_points_features_colors(self, data, image):
        points = self.points_cache.get(image)
        features = self.features_cache.get(image)
        colors = self.colors_cache.get(image)
        if points is None or features is None or colors is None:
            points, features, colors = self._load_features_nocache(data, image)
            self.points_cache.put(image, points)
            self.features_cache.put(image, features)
            self.colors_cache.put(image, colors)
        return points, features, colors

    def load_words(self, data, image):
        words = self.words_cache.get(image)
        if words is None:
            words = data.load_words(image)
            self.words_cache.put(image, words)
        return words

    def _load_features_nocache(self, data, image):
        points, features, colors = data.load_features(image)
        if points is None:
            logger.error('Could not load features for image {}'.format(image))
        else:
            points = np.array(points[:, :3], dtype=float)
        return points, features, colors