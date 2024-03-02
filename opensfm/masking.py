# pyre-unsafe
import logging
from typing import List, Tuple, Optional

import cv2
import numpy as np
from opensfm import upright
from opensfm.dataset_base import DataSetBase

logger: logging.Logger = logging.getLogger(__name__)


def mask_from_segmentation(
    segmentation: np.ndarray, ignore_values: List[int]
) -> np.ndarray:
    """Binary mask that is 0 for pixels with segmentation value to ignore."""
    mask = np.ones(segmentation.shape, dtype=np.uint8)
    for value in ignore_values:
        mask &= segmentation != value
    return mask


def combine_masks(
    mask1: Optional[np.ndarray], mask2: Optional[np.ndarray]
) -> Optional[np.ndarray]:
    """Combine two masks as mask1 AND mask2.

    Ignore any missing mask argument.
    """
    if mask1 is None:
        if mask2 is None:
            return None
        else:
            return mask2
    else:
        if mask2 is None:
            return mask1
        else:
            mask1, mask2 = _resize_masks_to_match(mask1, mask2)
            return mask1 & mask2


def _resize_masks_to_match(
    im1: np.ndarray,
    im2: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    h, w = max(im1.shape, im2.shape)
    if im1.shape != (h, w):
        im1 = cv2.resize(im1, (w, h), interpolation=cv2.INTER_NEAREST)
    if im2.shape != (h, w):
        im2 = cv2.resize(im2, (w, h), interpolation=cv2.INTER_NEAREST)
    return im1, im2


def load_features_mask(
    data: DataSetBase,
    image: str,
    points: np.ndarray,
    mask_image: Optional[np.ndarray] = None,
) -> np.ndarray:
    """Load a feature-wise mask.

    This is a binary array true for features that lie inside the
    combined mask.
    The array is all true when there's no mask.
    """
    if points is None or len(points) == 0:
        return np.array([], dtype=bool)

    if mask_image is None:
        mask_image = _load_combined_mask(data, image)
    if mask_image is None:
        logger.debug("No segmentation for {}, no features masked.".format(image))
        return np.ones((points.shape[0],), dtype=bool)

    exif = data.load_exif(image)
    width = exif["width"]
    height = exif["height"]
    orientation = exif["orientation"]

    new_height, new_width = mask_image.shape
    ps = upright.opensfm_to_upright(
        points[:, :2],
        width,
        height,
        orientation,
        new_width=new_width,
        new_height=new_height,
    ).astype(int)
    mask = mask_image[ps[:, 1], ps[:, 0]]

    n_removed = np.sum(mask == 0)
    logger.debug(
        "Masking {} / {} ({:.2f}) features for {}".format(
            n_removed, len(mask), n_removed / len(mask), image
        )
    )

    return np.array(mask, dtype=bool)


def _load_segmentation_mask(data: DataSetBase, image: str) -> Optional[np.ndarray]:
    """Build a mask from segmentation ignore values.

    The mask is non-zero only for pixels with segmentation
    labels not in segmentation_ignore_values.
    """
    ignore_values = data.segmentation_ignore_values(image)
    if not ignore_values:
        return None

    segmentation = data.load_segmentation(image)
    if segmentation is None:
        return None

    return mask_from_segmentation(segmentation, ignore_values)


def _load_combined_mask(data: DataSetBase, image: str) -> Optional[np.ndarray]:
    """Combine binary mask with segmentation mask.

    Return a mask that is non-zero only where the binary
    mask and the segmentation mask are non-zero.
    """
    mask = data.load_mask(image)
    smask = _load_segmentation_mask(data, image)
    return combine_masks(mask, smask)
