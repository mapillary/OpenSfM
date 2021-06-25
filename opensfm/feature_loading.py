import logging
from functools import lru_cache
from typing import Optional, Tuple, Any

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
    def load_mask(self, data: DataSetBase, image: str) -> Optional[np.ndarray]:
        all_features_data = self._load_all_data_unmasked(data, image)
        if not all_features_data:
            return None
        if (
            data.config["features_bake_segmentation"]
            and all_features_data.semantic is not None
        ):
            # pyre-fixme [16]: `Optional` has no attribute `segmentation`
            segmentations = all_features_data.semantic.segmentation
            ignore_values = set(data.segmentation_ignore_values(image))
            return np.array(
                [
                    False if segmentations[i] in ignore_values else True
                    for i in range(len(segmentations))
                ],
                dtype=bool
            )
        else:
            return data.load_features_mask(image, all_features_data.points[:, :2])

    @lru_cache(1000)
    def load_points_colors_segmentations_instances(
        self, data: DataSetBase, image: str
    ) -> Optional[ft.FeaturesData]:
        all_features_data = self._load_features_nocache(data, image)
        if not all_features_data:
            return None
        return ft.FeaturesData(
            all_features_data.points,
            None,
            all_features_data.colors,
            all_features_data.semantic,
        )

    def load_all_data(
        self, data: DataSetBase, image: str, masked: bool
    ) -> Optional[ft.FeaturesData]:
        if masked:
            return self._load_all_data_masked(data, image)
        else:
            return self._load_all_data_unmasked(data, image)

    @lru_cache(20)
    def _load_all_data_unmasked(
        self, data: DataSetBase, image: str
    ) -> Optional[ft.FeaturesData]:
        return self._load_features_nocache(data, image)

    @lru_cache(200)
    def _load_all_data_masked(
        self, data: DataSetBase, image: str
    ) -> Optional[ft.FeaturesData]:
        features_data = self._load_all_data_unmasked(data, image)
        if not features_data:
            return features_data
        mask = self.load_mask(data, image)
        if mask is not None:
            return features_data.mask(mask)
        return features_data

    @lru_cache(200)
    def load_features_index(
        self, data: DataSetBase, image: str, masked: bool
    ) -> Optional[Tuple[ft.FeaturesData, Any]]:
        features_data = self.load_all_data(data, image, masked)
        if not features_data:
            return None
        return features_data, ft.build_flann_index(
            # pyre-fixme [6]: Expected `np.ndarray`
            features_data.descriptors,
            data.config,
        )

    @lru_cache(200)
    def load_words(self, data: DataSetBase, image: str, masked: bool) -> np.ndarray:
        words = data.load_words(image)
        if masked:
            mask = self.load_mask(data, image)
            if mask is not None:
                words = words[mask]
        return words

    def _load_features_nocache(
        self, data: DataSetBase, image: str
    ) -> Optional[ft.FeaturesData]:
        features_data = data.load_features(image)
        if features_data is None:
            logger.error("Could not load features for image {}".format(image))
            return None
        else:
            features_data.points = np.array(features_data.points[:, :3], dtype=float)
        return features_data
