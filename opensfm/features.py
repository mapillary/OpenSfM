"""Tools to extract features."""

import logging
import sys
import time

import cv2
import numpy as np
from opensfm import context, pyfeatures


logger = logging.getLogger(__name__)


def resized_image(image, max_size):
    """Resize image to feature_process_size."""
    h, w, _ = image.shape
    size = max(w, h)
    if 0 < max_size < size:
        dsize = w * max_size // size, h * max_size // size
        return cv2.resize(image, dsize=dsize, interpolation=cv2.INTER_AREA)
    else:
        return image


def root_feature(desc, l2_normalization=False):
    if l2_normalization:
        s2 = np.linalg.norm(desc, axis=1)
        desc = (desc.T / s2).T
    s = np.sum(desc, 1)
    desc = np.sqrt(desc.T / s).T
    return desc


def root_feature_surf(desc, l2_normalization=False, partial=False):
    """
    Experimental square root mapping of surf-like feature, only work for 64-dim surf now
    """
    if desc.shape[1] == 64:
        if l2_normalization:
            s2 = np.linalg.norm(desc, axis=1)
            desc = (desc.T / s2).T
        if partial:
            ii = np.array([i for i in range(64) if (i % 4 == 2 or i % 4 == 3)])
        else:
            ii = np.arange(64)
        desc_sub = np.abs(desc[:, ii])
        desc_sub_sign = np.sign(desc[:, ii])
        # s_sub = np.sum(desc_sub, 1)  # This partial normalization gives slightly better results for AKAZE surf
        s_sub = np.sum(np.abs(desc), 1)
        desc_sub = np.sqrt(desc_sub.T / s_sub).T
        desc[:, ii] = desc_sub * desc_sub_sign
    return desc


def normalized_image_coordinates(pixel_coords, width, height):
    size = max(width, height)
    p = np.empty((len(pixel_coords), 2))
    p[:, 0] = (pixel_coords[:, 0] + 0.5 - width / 2.0) / size
    p[:, 1] = (pixel_coords[:, 1] + 0.5 - height / 2.0) / size
    return p


def denormalized_image_coordinates(norm_coords, width, height):
    size = max(width, height)
    p = np.empty((len(norm_coords), 2))
    p[:, 0] = norm_coords[:, 0] * size - 0.5 + width / 2.0
    p[:, 1] = norm_coords[:, 1] * size - 0.5 + height / 2.0
    return p


def normalize_features(points, desc, colors, width, height):
    """Normalize feature coordinates and size."""
    points[:, :2] = normalized_image_coordinates(points[:, :2], width, height)
    points[:, 2:3] /= max(width, height)
    return points, desc, colors


def _in_mask(point, width, height, mask):
    """Check if a point is inside a binary mask."""
    u = mask.shape[1] * (point[0] + 0.5) / width
    v = mask.shape[0] * (point[1] + 0.5) / height
    return mask[int(v), int(u)] != 0


def extract_features_sift(image, config, features_count):
    sift_edge_threshold = config["sift_edge_threshold"]
    sift_peak_threshold = float(config["sift_peak_threshold"])
    # SIFT support is in cv2 main from version 4.4.0
    if context.OPENCV44 or context.OPENCV5:
        # OpenCV versions concerned /** 3.4.11, >= 4.4.0 **/  ==> Sift became free since March 2020
        detector = cv2.SIFT_create(
            edgeThreshold=sift_edge_threshold, contrastThreshold=sift_peak_threshold
        )
        descriptor = detector
    elif context.OPENCV3 or context.OPENCV4:
        try:
            # OpenCV versions concerned /** 3.2.x, 3.3.x, 3.4.0, 3.4.1, 3.4.2, 3.4.10, 4.3.0, 4.4.0 **/
            detector = cv2.xfeatures2d.SIFT_create(
                edgeThreshold=sift_edge_threshold, contrastThreshold=sift_peak_threshold
            )
        except AttributeError as ae:
            # OpenCV versions concerned /** 3.4.3, 3.4.4, 3.4.5, 3.4.6, 3.4.7, 3.4.8, 3.4.9, 4.0.x, 4.1.x, 4.2.x **/
            if "no attribute 'xfeatures2d'" in str(ae):
                logger.error(
                    "OpenCV Contrib modules are required to extract SIFT features"
                )
            raise
        descriptor = detector
    else:
        detector = cv2.FeatureDetector_create("SIFT")
        descriptor = cv2.DescriptorExtractor_create("SIFT")
        detector.setDouble("edgeThreshold", sift_edge_threshold)
    while True:
        logger.debug("Computing sift with threshold {0}".format(sift_peak_threshold))
        t = time.time()
        # SIFT support is in cv2 main from version 4.4.0
        if context.OPENCV44 or context.OPENCV5:
            detector = cv2.SIFT_create(
                edgeThreshold=sift_edge_threshold, contrastThreshold=sift_peak_threshold
            )
        elif context.OPENCV3:
            detector = cv2.xfeatures2d.SIFT_create(
                edgeThreshold=sift_edge_threshold, contrastThreshold=sift_peak_threshold
            )
        else:
            detector.setDouble("contrastThreshold", sift_peak_threshold)
        points = detector.detect(image)
        logger.debug("Found {0} points in {1}s".format(len(points), time.time() - t))
        if len(points) < features_count and sift_peak_threshold > 0.0001:
            sift_peak_threshold = (sift_peak_threshold * 2) / 3
            logger.debug("reducing threshold")
        else:
            logger.debug("done")
            break
    points, desc = descriptor.compute(image, points)
    if config["feature_root"]:
        desc = root_feature(desc)
    points = np.array([(i.pt[0], i.pt[1], i.size, i.angle) for i in points])
    return points, desc


def extract_features_surf(image, config, features_count):
    surf_hessian_threshold = config["surf_hessian_threshold"]
    if context.OPENCV3:
        try:
            detector = cv2.xfeatures2d.SURF_create()
        except AttributeError as ae:
            if "no attribute 'xfeatures2d'" in str(ae):
                logger.error(
                    "OpenCV Contrib modules are required to extract SURF features"
                )
            raise
        descriptor = detector
        detector.setHessianThreshold(surf_hessian_threshold)
        detector.setNOctaves(config["surf_n_octaves"])
        detector.setNOctaveLayers(config["surf_n_octavelayers"])
        detector.setUpright(config["surf_upright"])
    else:
        detector = cv2.FeatureDetector_create("SURF")
        descriptor = cv2.DescriptorExtractor_create("SURF")
        detector.setDouble("hessianThreshold", surf_hessian_threshold)
        detector.setDouble("nOctaves", config["surf_n_octaves"])
        detector.setDouble("nOctaveLayers", config["surf_n_octavelayers"])
        detector.setInt("upright", config["surf_upright"])

    while True:
        logger.debug("Computing surf with threshold {0}".format(surf_hessian_threshold))
        t = time.time()
        if context.OPENCV3:
            detector.setHessianThreshold(surf_hessian_threshold)
        else:
            detector.setDouble(
                "hessianThreshold", surf_hessian_threshold
            )  # default: 0.04
        points = detector.detect(image)
        logger.debug("Found {0} points in {1}s".format(len(points), time.time() - t))
        if len(points) < features_count and surf_hessian_threshold > 0.0001:
            surf_hessian_threshold = (surf_hessian_threshold * 2) / 3
            logger.debug("reducing threshold")
        else:
            logger.debug("done")
            break

    points, desc = descriptor.compute(image, points)
    if config["feature_root"]:
        desc = root_feature_surf(desc, partial=True)
    points = np.array([(i.pt[0], i.pt[1], i.size, i.angle) for i in points])
    return points, desc


def akaze_descriptor_type(name):
    d = pyfeatures.AkazeDescriptorType.__dict__
    if name in d:
        return d[name]
    else:
        logger.debug("Wrong akaze descriptor type")
        return d["MSURF"]


def extract_features_akaze(image, config, features_count):
    options = pyfeatures.AKAZEOptions()
    options.omax = config["akaze_omax"]
    akaze_descriptor_name = config["akaze_descriptor"]
    options.descriptor = akaze_descriptor_type(akaze_descriptor_name)
    options.descriptor_size = config["akaze_descriptor_size"]
    options.descriptor_channels = config["akaze_descriptor_channels"]
    options.dthreshold = config["akaze_dthreshold"]
    options.kcontrast_percentile = config["akaze_kcontrast_percentile"]
    options.use_isotropic_diffusion = config["akaze_use_isotropic_diffusion"]
    options.target_num_features = features_count
    options.use_adaptive_suppression = config["feature_use_adaptive_suppression"]

    logger.debug("Computing AKAZE with threshold {0}".format(options.dthreshold))
    t = time.time()
    points, desc = pyfeatures.akaze(image, options)
    logger.debug("Found {0} points in {1}s".format(len(points), time.time() - t))

    if config["feature_root"]:
        if akaze_descriptor_name in ["SURF_UPRIGHT", "MSURF_UPRIGHT"]:
            desc = root_feature_surf(desc, partial=True)
        elif akaze_descriptor_name in ["SURF", "MSURF"]:
            desc = root_feature_surf(desc, partial=False)
    points = points.astype(float)
    return points, desc


def extract_features_hahog(image, config, features_count):
    t = time.time()
    points, desc = pyfeatures.hahog(
        image.astype(np.float32) / 255,  # VlFeat expects pixel values between 0, 1
        peak_threshold=config["hahog_peak_threshold"],
        edge_threshold=config["hahog_edge_threshold"],
        target_num_features=features_count,
        use_adaptive_suppression=config["feature_use_adaptive_suppression"],
    )

    if config["feature_root"]:
        desc = np.sqrt(desc)
        uchar_scaling = 362  # x * 512 < 256  =>  sqrt(x) * 362 < 256
    else:
        uchar_scaling = 512

    if config["hahog_normalize_to_uchar"]:
        desc = (uchar_scaling * desc).clip(0, 255).round()

    logger.debug("Found {0} points in {1}s".format(len(points), time.time() - t))
    return points, desc


def extract_features_orb(image, config, features_count):
    if context.OPENCV3:
        detector = cv2.ORB_create(nfeatures=features_count)
        descriptor = detector
    else:
        detector = cv2.FeatureDetector_create("ORB")
        descriptor = cv2.DescriptorExtractor_create("ORB")
        detector.setDouble("nFeatures", features_count)

    logger.debug("Computing ORB")
    t = time.time()
    points = detector.detect(image)

    points, desc = descriptor.compute(image, points)
    points = np.array([(i.pt[0], i.pt[1], i.size, i.angle) for i in points])

    logger.debug("Found {0} points in {1}s".format(len(points), time.time() - t))
    return points, desc


def extract_features(image, config, is_panorama):
    """Detect features in a color or gray-scale image.

    The type of feature detected is determined by the ``feature_type``
    config option.

    The coordinates of the detected points are returned in normalized
    image coordinates.

    Parameters:
        - image: a color image with shape (h, w, 3) or
                 gray-scale image with (h, w) or (h, w, 1)
        - config: the configuration structure
        - is_panorama : if True, alternate settings are used for feature count and extraction size.

    Returns:
        tuple:
        - points: ``x``, ``y``, ``size`` and ``angle`` for each feature
        - descriptors: the descriptor of each feature
        - colors: the color of the center of each feature
    """
    extraction_size = (
        config["feature_process_size_panorama"]
        if is_panorama
        else config["feature_process_size"]
    )
    features_count = (
        config["feature_min_frames_panorama"]
        if is_panorama
        else config["feature_min_frames"]
    )

    assert len(image.shape) == 3 or len(image.shape) == 2

    if len(image.shape) == 2:  # convert (h, w) to (h, w, 1)
        image = np.expand_dims(image, axis=2)

    image = resized_image(image, extraction_size)

    # convert color to gray-scale if necessary
    if image.shape[2] == 3:
        image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    else:
        image_gray = image

    feature_type = config["feature_type"].upper()
    if feature_type == "SIFT":
        points, desc = extract_features_sift(image_gray, config, features_count)
    elif feature_type == "SURF":
        points, desc = extract_features_surf(image_gray, config, features_count)
    elif feature_type == "AKAZE":
        points, desc = extract_features_akaze(image_gray, config, features_count)
    elif feature_type == "HAHOG":
        points, desc = extract_features_hahog(image_gray, config, features_count)
    elif feature_type == "ORB":
        points, desc = extract_features_orb(image_gray, config, features_count)
    else:
        raise ValueError(
            "Unknown feature type " "(must be SURF, SIFT, AKAZE, HAHOG or ORB)"
        )

    xs = points[:, 0].round().astype(int)
    ys = points[:, 1].round().astype(int)
    colors = image[ys, xs]
    if image.shape[2] == 1:
        colors = np.repeat(colors, 3).reshape((-1, 3))

    return normalize_features(points, desc, colors, image.shape[1], image.shape[0])


def build_flann_index(features, config):
    # FLANN_INDEX_LINEAR = 0
    FLANN_INDEX_KDTREE = 1
    FLANN_INDEX_KMEANS = 2
    # FLANN_INDEX_COMPOSITE = 3
    # FLANN_INDEX_KDTREE_SINGLE = 4
    # FLANN_INDEX_HIERARCHICAL = 5
    FLANN_INDEX_LSH = 6

    if features.dtype.type is np.float32:
        algorithm_type = config["flann_algorithm"].upper()
        if algorithm_type == "KMEANS":
            FLANN_INDEX_METHOD = FLANN_INDEX_KMEANS
        elif algorithm_type == "KDTREE":
            FLANN_INDEX_METHOD = FLANN_INDEX_KDTREE
        else:
            raise ValueError("Unknown flann algorithm type " "must be KMEANS, KDTREE")
    else:
        FLANN_INDEX_METHOD = FLANN_INDEX_LSH

    flann_params = {
        "algorithm": FLANN_INDEX_METHOD,
        "branching": config["flann_branching"],
        "iterations": config["flann_iterations"],
        "tree": config["flann_tree"],
    }

    return context.flann_Index(features, flann_params)


FEATURES_VERSION = 2
FEATURES_HEADER = "OPENSFM_FEATURES_VERSION"


def load_features(filepath, config):
    """ Load features from filename """
    s = np.load(filepath, allow_pickle=True)
    version = _features_file_version(s)
    return getattr(sys.modules[__name__], "_load_features_v%d" % version)(s, config)


def _features_file_version(obj):
    """ Retrieve features file version. Return 0 if none """
    if FEATURES_HEADER in obj:
        return obj[FEATURES_HEADER]
    else:
        return 0


def _load_features_v0(s, config):
    """Base version of features file

    Scale (desc[2]) set to reprojection_error_sd by default (legacy behaviour)
    """
    feature_type = config["feature_type"]
    if feature_type == "HAHOG" and config["hahog_normalize_to_uchar"]:
        descriptors = s["descriptors"].astype(np.float32)
    else:
        descriptors = s["descriptors"]
    points = s["points"]
    points[:, 2:3] = config["reprojection_error_sd"]
    return points, descriptors, s["colors"].astype(float), None


def _load_features_v1(s, config):
    """Version 1 of features file

    Scale is not properly set higher in the pipeline, default is gone.
    """
    feature_type = config["feature_type"]
    if feature_type == "HAHOG" and config["hahog_normalize_to_uchar"]:
        descriptors = s["descriptors"].astype(np.float32)
    else:
        descriptors = s["descriptors"]
    return s["points"], descriptors, s["colors"].astype(float), None


def _load_features_v2(s, config):
    """Version 2 of features file

    Added segmentation and segmentation labels.
    """
    feature_type = config["feature_type"]
    if feature_type == "HAHOG" and config["hahog_normalize_to_uchar"]:
        descriptors = s["descriptors"].astype(np.float32)
    else:
        descriptors = s["descriptors"]
    has_segmentation = s["segmentations"].any()
    has_instances = s["instances"].any()
    return (
        s["points"],
        descriptors,
        s["colors"].astype(float),
        {
            "segmentations": s["segmentations"] if has_segmentation else None,
            "instances": s["instances"] if has_instances else None,
            "segmentation_labels": s["segmentation_labels"],
        },
    )


def save_features(
    filepath,
    points,
    desc,
    colors,
    segmentations,
    instances,
    segmentation_labels,
    config,
):
    feature_type = config["feature_type"]
    if (
        (
            feature_type == "AKAZE"
            and config["akaze_descriptor"] in ["MLDB_UPRIGHT", "MLDB"]
        )
        or (feature_type == "HAHOG" and config["hahog_normalize_to_uchar"])
        or (feature_type == "ORB")
    ):
        feature_data_type = np.uint8
    else:
        feature_data_type = np.float32
    np.savez_compressed(
        filepath,
        points=points.astype(np.float32),
        descriptors=desc.astype(feature_data_type),
        colors=colors,
        segmentations=segmentations,
        instances=instances,
        segmentation_labels=segmentation_labels,
        OPENSFM_FEATURES_VERSION=FEATURES_VERSION,
        allow_pickle=True,
    )
