# pyre-unsafe
import logging
from functools import lru_cache
from typing import Optional, Tuple, Any

import numpy as np
from opensfm import pygeometry, features as ft, masking
from opensfm.dataset_base import DataSetBase


logger: logging.Logger = logging.getLogger(__name__)


SEGMENTATION_IN_DESCRIPTOR_MULT = (
    35  # determined experimentally for HAHOG UCHAR type descriptors
)


class FeatureLoader:
    def clear_cache(self) -> None:
        self.load_mask.cache_clear()
        self.load_points_colors_segmentations_instances.cache_clear()
        self._load_all_data_unmasked.cache_clear()
        self._load_all_data_masked.cache_clear()
        self.load_features_index.cache_clear()
        self.load_words.cache_clear()

    @lru_cache(1000)
    def load_mask(self, data: DataSetBase, image: str) -> Optional[np.ndarray]:
        all_features_data = self._load_all_data_unmasked(data, image)
        if not all_features_data:
            return None
        if (
            data.config["features_bake_segmentation"]
            and all_features_data.semantic is not None
        ):
            # feature mask for baked segmentation
            segmentations = all_features_data.semantic.segmentation
            ignore_values = set(data.segmentation_ignore_values(image))
            smask = np.array(
                [
                    False if segmentations[i] in ignore_values else True
                    for i in range(len(segmentations))
                ],
                dtype=bool,
            )

            # combine with 'classic' mask if any
            mask_image = data.load_mask(image)
            if mask_image is not None:
                mask = masking.load_features_mask(
                    data, image, all_features_data.points[:, :2], mask_image
                )
                smask &= mask

            n_removed = np.sum(smask == 0)
            logger.debug(
                "Masking {} / {} ({:.2f}) features for {}".format(
                    n_removed, len(smask), n_removed / len(smask), image
                )
            )

            return smask

        else:
            return masking.load_features_mask(
                data, image, all_features_data.points[:, :2]
            )

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

    @lru_cache(2000)
    def load_bearings(
        self,
        data: DataSetBase,
        image: str,
        masked: bool,
        camera: pygeometry.Camera,
    ) -> Optional[np.ndarray]:
        if masked:
            features_data = self._load_all_data_masked(data, image)
        else:
            features_data = self._load_all_data_unmasked(data, image)
        if not features_data:
            return None
        keypoints_2d = np.array(features_data.points[:, :2], dtype=float)
        bearings_3d = camera.pixel_bearing_many(keypoints_2d)
        return bearings_3d

    def load_all_data(
        self,
        data: DataSetBase,
        image: str,
        masked: bool,
        segmentation_in_descriptor: bool,
    ) -> Optional[ft.FeaturesData]:
        if masked:
            features_data = self._load_all_data_masked(data, image)
        else:
            features_data = self._load_all_data_unmasked(data, image)
        if not features_data:
            return None
        if segmentation_in_descriptor:
            return self._add_segmentation_in_descriptor(data, features_data)
        else:
            return features_data

    def _add_segmentation_in_descriptor(
        self, data: DataSetBase, features: ft.FeaturesData
    ) -> ft.FeaturesData:
        if (
            not data.config["hahog_normalize_to_uchar"]
            or data.config["feature_type"] != "HAHOG"
        ):
            raise RuntimeError(
                "Semantic segmentation in descriptor only supported for HAHOG UCHAR descriptors"
            )

        segmentation = features.get_segmentation()
        if segmentation is None:
            return features

        desc_augmented = np.concatenate(
            (
                features.descriptors,
                (np.array([segmentation]).T).astype(np.float32),
            ),
            axis=1,
        )
        desc_augmented[:, -1] *= SEGMENTATION_IN_DESCRIPTOR_MULT

        return ft.FeaturesData(
            features.points,
            desc_augmented,
            features.colors,
            features.semantic,
        )

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
        self,
        data: DataSetBase,
        image: str,
        masked: bool,
        segmentation_in_descriptor: bool,
    ) -> Optional[Tuple[ft.FeaturesData, Any]]:
        features_data = self.load_all_data(
            data, image, masked, segmentation_in_descriptor
        )
        if not features_data:
            return None
        descriptors = features_data.descriptors
        if descriptors is None:
            return None
        return features_data, ft.build_flann_index(
            descriptors,
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
