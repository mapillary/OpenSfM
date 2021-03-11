import logging
from functools import lru_cache

import numpy as np
from opensfm import features as ft
from opensfm.dataset import DataSetBase


logger = logging.getLogger(__name__)


class FeatureLoader(object):
    def clear_cache(self):
        self.load_mask.cache_clear()
        self.load_points_colors_segmentations_instances.cache_clear()
        self._load_all_data_unmasked.cache_clear()
        self._load_all_data_masked.cache_clear()
        self.load_features_index.cache_clear()
        self.load_words.cache_clear

    @lru_cache(1000)
    def load_mask(self, data: DataSetBase, image):
        points, _, _, segmentations, _ = self._load_all_data_unmasked(data, image)
        if data.config["features_bake_segmentation"] and segmentations is not None:
            ignore_values = set(data.segmentation_ignore_values(image))
            return [
                False if segmentations[i] in ignore_values else True
                for i in range(len(segmentations))
            ]
        else:
            if points is None:
                return None
            return data.load_features_mask(image, points[:, :2])

    @lru_cache(1000)
    def load_points_colors_segmentations_instances(self, data: DataSetBase, image):
        points, _, colors, segmentation_data = self._load_features_nocache(data, image)
        return (
            points,
            colors,
            segmentation_data["segmentations"] if segmentation_data else None,
            segmentation_data["instances"] if segmentation_data else None,
        )

    def load_all_data(self, data: DataSetBase, image, masked):
        if masked:
            return self._load_all_data_masked(data, image)
        else:
            return self._load_all_data_unmasked(data, image)

    @lru_cache(20)
    def _load_all_data_unmasked(self, data: DataSetBase, image):
        points, features, colors, segmentation_data = self._load_features_nocache(
            data, image
        )
        return (
            points,
            features,
            colors,
            segmentation_data["segmentations"] if segmentation_data else None,
            segmentation_data["instances"] if segmentation_data else None,
        )

    @lru_cache(200)
    def _load_all_data_masked(self, data: DataSetBase, image):
        (
            points,
            features,
            colors,
            segmentations,
            instances,
        ) = self._load_all_data_unmasked(data, image)
        mask = self.load_mask(data, image)
        if mask is not None:
            points = points[mask]
            features = features[mask]
            colors = colors[mask]
            if segmentations is not None:
                segmentations = segmentations[mask]
            if instances is not None:
                instances = instances[mask]
        return points, features, colors, segmentations, instances

    @lru_cache(200)
    def load_features_index(self, data: DataSetBase, image, masked):
        _, features, _, _, _ = self.load_all_data(data, image, masked)
        return features, ft.build_flann_index(features, data.config)

    @lru_cache(200)
    def load_words(self, data: DataSetBase, image, masked):
        words = data.load_words(image)
        if masked:
            mask = self.load_mask(data, image)
            if mask is not None:
                words = words[mask]
        return words

    def _load_features_nocache(self, data: DataSetBase, image):
        points, features, colors, segmentation_data = data.load_features(image)
        if points is None:
            logger.error("Could not load features for image {}".format(image))
        else:
            points = np.array(points[:, :3], dtype=float)
        return points, features, colors, segmentation_data
