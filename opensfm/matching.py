# pyre-unsafe
import logging
from timeit import default_timer as timer
from typing import Any, Dict, Generator, List, Optional, Sized, Tuple

import cv2
import numpy as np
from opensfm import (
    context,
    feature_loader,
    log,
    multiview,
    pairs_selection,
    pyfeatures,
    pygeometry,
)
from opensfm.dataset_base import DataSetBase


logger: logging.Logger = logging.getLogger(__name__)


def clear_cache() -> None:
    feature_loader.instance.clear_cache()


def match_images(
    data: DataSetBase,
    config_override: Dict[str, Any],
    ref_images: List[str],
    cand_images: List[str],
) -> Tuple[Dict[Tuple[str, str], List[Tuple[int, int]]], Dict[str, Any]]:
    """Perform pair matchings between two sets of images.

    It will do matching for each pair (i, j), i being in
    ref_images and j in cand_images, taking assumption that
    matching(i, j) == matching(j ,i). This does not hold for
    non-symmetric matching options like WORDS. Data will be
    stored in i matching only.
    """

    # Get EXIFs data
    all_images = list(set(ref_images + cand_images))
    exifs = {im: data.load_exif(im) for im in all_images}

    # Generate pairs for matching
    pairs, preport = pairs_selection.match_candidates_from_metadata(
        ref_images,
        cand_images,
        exifs,
        data,
        config_override,
    )

    # Match them !
    return (
        match_images_with_pairs(data, config_override, exifs, pairs),
        preport,
    )


def match_images_with_pairs(
    data: DataSetBase,
    config_override: Dict[str, Any],
    exifs: Dict[str, Any],
    pairs: List[Tuple[str, str]],
    poses: Optional[Dict[str, pygeometry.Pose]] = None,
) -> Dict[Tuple[str, str], List[Tuple[int, int]]]:
    """Perform pair matchings given pairs."""
    cameras = data.load_camera_models()
    args = list(match_arguments(pairs, data, config_override, cameras, exifs, poses))

    # Perform all pair matchings in parallel
    start = timer()
    logger.info("Matching {} image pairs".format(len(pairs)))
    processes = config_override.get("processes", data.config["processes"])
    mem_per_process = 512
    jobs_per_process = 2
    processes = context.processes_that_fit_in_memory(processes, mem_per_process)
    logger.info("Computing pair matching with %d processes" % processes)
    matches = context.parallel_map(match_unwrap_args, args, processes, jobs_per_process)
    logger.info(
        "Matched {} pairs {} in {} seconds ({} seconds/pair).".format(
            len(pairs),
            log_projection_types(pairs, exifs, cameras),
            timer() - start,
            (timer() - start) / len(pairs) if pairs else 0,
        )
    )

    # Index results per pair
    resulting_pairs = {}
    for im1, im2, m in matches:
        resulting_pairs[im1, im2] = m
    return resulting_pairs


def log_projection_types(
    pairs: List[Tuple[str, str]],
    exifs: Dict[str, Any],
    cameras: Dict[str, pygeometry.Camera],
) -> str:
    if not pairs:
        return ""

    projection_type_pairs = {}
    for im1, im2 in pairs:
        pt1 = cameras[exifs[im1]["camera"]].projection_type
        pt2 = cameras[exifs[im2]["camera"]].projection_type

        if pt1 not in projection_type_pairs:
            projection_type_pairs[pt1] = {}

        if pt2 not in projection_type_pairs[pt1]:
            projection_type_pairs[pt1][pt2] = []

        projection_type_pairs[pt1][pt2].append((im1, im2))

    output = "("
    for pt1 in projection_type_pairs:
        for pt2 in projection_type_pairs[pt1]:
            output += "{}-{}: {}, ".format(
                pt1, pt2, len(projection_type_pairs[pt1][pt2])
            )

    return output[:-2] + ")"


def save_matches(
    data: DataSetBase,
    images_ref: List[str],
    matched_pairs: Dict[Tuple[str, str], List[Tuple[int, int]]],
) -> None:
    """Given pairwise matches (image 1, image 2) - > matches,
    save them such as only {image E images_ref} will store the matches.
    """
    images_ref_set = set(images_ref)
    matches_per_im1 = {im: {} for im in images_ref}
    for (im1, im2), m in matched_pairs.items():
        if im1 in images_ref_set:
            matches_per_im1[im1][im2] = m
        elif im2 in images_ref_set:
            matches_per_im1[im2][im1] = m
        else:
            raise RuntimeError(
                "Couldn't save matches for {}. No image found in images_ref.".format(
                    (im1, im2)
                )
            )

    for im1, im1_matches in matches_per_im1.items():
        data.save_matches(im1, im1_matches)


def match_arguments(
    pairs: List[Tuple[str, str]],
    data: DataSetBase,
    config_override: Dict[str, Any],
    cameras: Dict[str, pygeometry.Camera],
    exifs: Dict[str, pygeometry.Camera],
    poses: Optional[Dict[str, pygeometry.Pose]],
) -> Generator[
    Tuple[
        str,
        str,
        Dict[str, pygeometry.Camera],
        Dict[str, pygeometry.Camera],
        DataSetBase,
        Dict[str, Any],
        Optional[Dict[str, pygeometry.Pose]],
    ],
    None,
    None,
]:
    """Generate arguments for parallel processing of pair matching"""
    for im1, im2 in pairs:
        yield im1, im2, cameras, exifs, data, config_override, poses


def match_unwrap_args(
    args: Tuple[
        str,
        str,
        Dict[str, pygeometry.Camera],
        Dict[str, Any],
        DataSetBase,
        Dict[str, Any],
        Optional[Dict[str, pygeometry.Pose]],
    ]
) -> Tuple[str, str, np.ndarray]:
    """Wrapper for parallel processing of pair matching.

    Compute all pair matchings of a given image and save them.
    """
    log.setup()
    im1 = args[0]
    im2 = args[1]
    cameras = args[2]
    exifs = args[3]
    data: DataSetBase = args[4]
    config_override = args[5]
    poses = args[6]
    if poses:
        pose1 = poses[im1]
        pose2 = poses[im2]
        pose = pose2.relative_to(pose1)
    else:
        pose = None
    camera1 = cameras[exifs[im1]["camera"]]
    camera2 = cameras[exifs[im2]["camera"]]
    matches = match(im1, im2, camera1, camera2, data, config_override, pose)
    return im1, im2, matches


def match_descriptors(
    im1: str,
    im2: str,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    data: DataSetBase,
    config_override: Dict[str, Any],
) -> np.ndarray:
    """Perform descriptor matching for a pair of images."""
    # Override parameters
    overriden_config = data.config.copy()
    overriden_config.update(config_override)

    # Run descriptor matching
    time_start = timer()
    _, _, matches, matcher_type = _match_descriptors_impl(
        im1, im2, camera1, camera2, data, overriden_config
    )
    time_2d_matching = timer() - time_start

    # From indexes in filtered sets, to indexes in original sets of features
    matches_unfiltered = []
    m1 = feature_loader.instance.load_mask(data, im1)
    m2 = feature_loader.instance.load_mask(data, im2)
    if m1 is not None and m2 is not None:
        matches_unfiltered = unfilter_matches(matches, m1, m2)

    symmetric = "symmetric" if overriden_config["symmetric_matching"] else "one-way"
    logger.debug(
        "Matching {} and {}.  Matcher: {} ({}) "
        "T-desc: {:1.3f} Matches: {}".format(
            im1,
            im2,
            matcher_type,
            symmetric,
            time_2d_matching,
            len(matches_unfiltered),
        )
    )
    return np.array(matches_unfiltered, dtype=int)


def _match_descriptors_guided_impl(
    im1: str,
    im2: str,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    relative_pose: pygeometry.Pose,
    data: DataSetBase,
    overriden_config: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """Perform descriptor guided matching for a pair of images, using their relative pose. It also apply static objects removal."""
    guided_matcher_override = "BRUTEFORCE"
    matcher_type = overriden_config["matcher_type"].upper()
    symmetric_matching = overriden_config["symmetric_matching"]
    if matcher_type in ["WORDS", "FLANN"] or symmetric_matching:
        logger.warning(
            f"{matcher_type} and/or symmetric isn't supported for guided matching, switching to asymmetric {guided_matcher_override}"
        )
        matcher_type = guided_matcher_override

    # Will apply mask to features if any
    dummy = np.array([])
    segmentation_in_descriptor = overriden_config["matching_use_segmentation"]
    features_data1 = feature_loader.instance.load_all_data(
        data,
        im1,
        masked=True,
        segmentation_in_descriptor=segmentation_in_descriptor,
    )
    features_data2 = feature_loader.instance.load_all_data(
        data, im2, masked=True, segmentation_in_descriptor=segmentation_in_descriptor
    )
    bearings1 = feature_loader.instance.load_bearings(
        data, im1, masked=True, camera=camera1
    )
    bearings2 = feature_loader.instance.load_bearings(
        data, im2, masked=True, camera=camera2
    )

    if (
        features_data1 is None
        or bearings1 is None
        or len(features_data1.points) < 2
        or features_data2 is None
        or bearings2 is None
        or len(features_data2.points) < 2
    ):
        return dummy, dummy, dummy, matcher_type

    d1 = features_data1.descriptors
    d2 = features_data2.descriptors
    if d1 is None or d2 is None:
        return dummy, dummy, dummy, matcher_type

    epipolar_mask = compute_inliers_bearing_epipolar(
        bearings1,
        bearings2,
        relative_pose,
        overriden_config["guided_matching_threshold"],
    )
    matches = match_brute_force_symmetric(d1, d2, overriden_config, epipolar_mask)

    # Adhoc filters
    if overriden_config["matching_use_filters"]:
        matches = apply_adhoc_filters(
            data,
            matches,
            im1,
            camera1,
            features_data1.points,
            im2,
            camera2,
            features_data2.points,
        )
    return (
        features_data1.points,
        features_data2.points,
        np.array(matches, dtype=int),
        matcher_type,
    )


def _match_descriptors_impl(
    im1: str,
    im2: str,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    data: DataSetBase,
    overriden_config: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, str]:
    """Perform descriptor matching for a pair of images. It also apply static objects removal."""
    dummy = np.array([])
    matcher_type = overriden_config["matcher_type"].upper()
    dummy_ret = dummy, dummy, dummy, matcher_type

    # Will apply mask to features if any
    dummy = np.array([])
    segmentation_in_descriptor = overriden_config["matching_use_segmentation"]
    features_data1 = feature_loader.instance.load_all_data(
        data, im1, masked=True, segmentation_in_descriptor=segmentation_in_descriptor
    )
    features_data2 = feature_loader.instance.load_all_data(
        data, im2, masked=True, segmentation_in_descriptor=segmentation_in_descriptor
    )
    if (
        features_data1 is None
        or len(features_data1.points) < 2
        or features_data2 is None
        or len(features_data2.points) < 2
    ):
        return dummy_ret

    d1 = features_data1.descriptors
    d2 = features_data2.descriptors
    if d1 is None or d2 is None:
        return dummy_ret

    symmetric_matching = overriden_config["symmetric_matching"]
    if matcher_type == "WORDS":
        words1 = feature_loader.instance.load_words(data, im1, masked=True)
        words2 = feature_loader.instance.load_words(data, im2, masked=True)
        if words1 is None or words2 is None:
            return dummy_ret

        if symmetric_matching:
            matches = match_words_symmetric(
                d1,
                words1,
                d2,
                words2,
                overriden_config,
            )
        else:
            matches = match_words(
                d1,
                words1,
                d2,
                words2,
                overriden_config,
            )

    elif matcher_type == "FLANN":
        f1 = feature_loader.instance.load_features_index(
            data,
            im1,
            masked=True,
            segmentation_in_descriptor=segmentation_in_descriptor,
        )
        if not f1:
            return dummy_ret
        feat_data_index1, index1 = f1
        if symmetric_matching:
            f2 = feature_loader.instance.load_features_index(
                data,
                im2,
                masked=True,
                segmentation_in_descriptor=segmentation_in_descriptor,
            )
            if not f2:
                return dummy_ret
            feat_data_index2, index2 = f2

            descriptors1 = feat_data_index1.descriptors
            descriptors2 = feat_data_index2.descriptors
            if descriptors1 is None or descriptors2 is None:
                return dummy_ret

            matches = match_flann_symmetric(
                descriptors1,
                index1,
                descriptors2,
                index2,
                overriden_config,
            )
        else:
            matches = match_flann(index1, d2, overriden_config)
    elif matcher_type == "BRUTEFORCE":
        if symmetric_matching:
            matches = match_brute_force_symmetric(d1, d2, overriden_config)
        else:
            matches = match_brute_force(d1, d2, overriden_config)
    else:
        raise ValueError("Invalid matcher_type: {}".format(matcher_type))

    # Adhoc filters
    if overriden_config["matching_use_filters"]:
        matches = apply_adhoc_filters(
            data,
            list(matches),
            im1,
            camera1,
            features_data1.points,
            im2,
            camera2,
            features_data2.points,
        )
    return (
        features_data1.points,
        features_data2.points,
        np.array(matches, dtype=int),
        matcher_type,
    )


def match_robust(
    im1: str,
    im2: str,
    matches: Sized,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    data: DataSetBase,
    config_override: Dict[str, Any],
    input_is_masked: bool = True,
) -> np.ndarray:
    """Perform robust geometry matching on a set of matched descriptors indexes."""
    # Override parameters
    overriden_config = data.config.copy()
    overriden_config.update(config_override)

    # Will apply mask to features if any
    segmentation_in_descriptor = overriden_config[
        "matching_use_segmentation"
    ]  # unused but keep using the same cache
    features_data1 = feature_loader.instance.load_all_data(
        data,
        im1,
        masked=input_is_masked,
        segmentation_in_descriptor=segmentation_in_descriptor,
    )
    features_data2 = feature_loader.instance.load_all_data(
        data,
        im2,
        masked=input_is_masked,
        segmentation_in_descriptor=segmentation_in_descriptor,
    )
    if (
        features_data1 is None
        or len(features_data1.points) < 2
        or features_data2 is None
        or len(features_data2.points) < 2
    ):
        return np.array([])

    # Run robust matching
    np_matches = np.array(matches, dtype=int)
    t = timer()
    rmatches = _match_robust_impl(
        im1,
        im2,
        features_data1.points,
        features_data2.points,
        np_matches,
        camera1,
        camera2,
        data,
        overriden_config,
    )
    time_robust_matching = timer() - t

    # From indexes in filtered sets, to indexes in original sets of features
    rmatches_unfiltered = []
    m1 = feature_loader.instance.load_mask(data, im1)
    m2 = feature_loader.instance.load_mask(data, im2)
    if m1 is not None and m2 is not None and input_is_masked:
        rmatches_unfiltered = unfilter_matches(rmatches, m1, m2)
    else:
        rmatches_unfiltered = rmatches

    robust_matching_min_match = overriden_config["robust_matching_min_match"]
    logger.debug(
        "Matching {} and {}. T-robust: {:1.3f} "
        "Matches: {} Robust: {} Success: {}".format(
            im1,
            im2,
            time_robust_matching,
            len(matches),
            len(rmatches_unfiltered),
            len(rmatches_unfiltered) >= robust_matching_min_match,
        )
    )

    if len(rmatches_unfiltered) < robust_matching_min_match:
        return np.array([])
    return np.array(rmatches_unfiltered, dtype=int)


def _match_robust_impl(
    im1: str,
    im2: str,
    p1: np.ndarray,
    p2: np.ndarray,
    matches: np.ndarray,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    data: DataSetBase,
    overriden_config: Dict[str, Any],
) -> np.ndarray:
    """Perform robust geometry matching on a set of matched descriptors indexes."""
    # robust matching
    rmatches = robust_match(p1, p2, camera1, camera2, matches, overriden_config)
    rmatches = np.array([[a, b] for a, b in rmatches])
    return rmatches


def match(
    im1: str,
    im2: str,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    data: DataSetBase,
    config_override: Dict[str, Any],
    guided_matching_pose: Optional[pygeometry.Pose],
) -> np.ndarray:
    """Perform full matching (descriptor+robust, optionally guided) for a pair of images."""
    # Override parameters
    overriden_config = data.config.copy()
    overriden_config.update(config_override)

    # Run descriptor matching
    time_start = timer()
    if guided_matching_pose:
        p1, p2, matches, matcher_type = _match_descriptors_guided_impl(
            im1, im2, camera1, camera2, guided_matching_pose, data, overriden_config
        )
    else:
        p1, p2, matches, matcher_type = _match_descriptors_impl(
            im1, im2, camera1, camera2, data, overriden_config
        )
    time_2d_matching = timer() - time_start

    symmetric = "symmetric" if overriden_config["symmetric_matching"] else "one-way"
    robust_matching_min_match = overriden_config["robust_matching_min_match"]
    if len(matches) < robust_matching_min_match:
        logger.debug(
            "Matching {} and {}.  Matcher: {} ({}) T-desc: {:1.3f} "
            "Matches: FAILED".format(
                im1, im2, matcher_type, symmetric, time_2d_matching
            )
        )
        return np.array([])

    # Run robust matching (non guided case only)
    t = timer()
    rmatches = _match_robust_impl(
        im1, im2, p1, p2, matches, camera1, camera2, data, overriden_config
    )
    time_robust_matching = timer() - t

    # From indexes in filtered sets, to indexes in original sets of features
    m1 = feature_loader.instance.load_mask(data, im1)
    m2 = feature_loader.instance.load_mask(data, im2)
    if m1 is not None and m2 is not None:
        rmatches = unfilter_matches(rmatches, m1, m2)

    time_total = timer() - time_start

    logger.debug(
        "Matching {} and {}.  Matcher: {} ({}) "
        "T-desc: {:1.3f} T-robust: {:1.3f} T-total: {:1.3f} "
        "Matches: {} Robust: {} Success: {}".format(
            im1,
            im2,
            matcher_type,
            symmetric,
            time_2d_matching,
            time_robust_matching,
            time_total,
            len(matches),
            len(rmatches),
            len(rmatches) >= robust_matching_min_match,
        )
    )

    if len(rmatches) < robust_matching_min_match:
        return np.array([])
    return np.array(rmatches, dtype=int)


def match_words(
    f1: np.ndarray,
    words1: np.ndarray,
    f2: np.ndarray,
    words2: np.ndarray,
    config: Dict[str, Any],
) -> np.ndarray:
    """Match using words and apply Lowe's ratio filter.

    Args:
        f1: feature descriptors of the first image
        w1: the nth closest words for each feature in the first image
        f2: feature descriptors of the second image
        w2: the nth closest words for each feature in the second image
        config: config parameters
    """
    ratio = config["lowes_ratio"]
    num_checks = config["bow_num_checks"]
    return pyfeatures.match_using_words(f1, words1, f2, words2[:, 0], ratio, num_checks)


def match_words_symmetric(
    f1: np.ndarray,
    words1: np.ndarray,
    f2: np.ndarray,
    words2: np.ndarray,
    config: Dict[str, Any],
) -> List[Tuple[int, int]]:
    """Match using words in both directions and keep consistent matches.

    Args:
        f1: feature descriptors of the first image
        w1: the nth closest words for each feature in the first image
        f2: feature descriptors of the second image
        w2: the nth closest words for each feature in the second image
        config: config parameters
    """
    matches_ij = match_words(f1, words1, f2, words2, config)
    matches_ji = match_words(f2, words2, f1, words1, config)
    matches_ij = [(a, b) for a, b in matches_ij]
    matches_ji = [(b, a) for a, b in matches_ji]

    return list(set(matches_ij).intersection(set(matches_ji)))


def match_flann(
    index: Any, f2: np.ndarray, config: Dict[str, Any]
) -> List[Tuple[int, int]]:
    """Match using FLANN and apply Lowe's ratio filter.

    Args:
        index: flann index if the first image
        f2: feature descriptors of the second image
        config: config parameters
    """
    search_params = dict(checks=config["flann_checks"])
    results, dists = index.knnSearch(f2, 2, params=search_params)
    squared_ratio = config["lowes_ratio"] ** 2  # Flann returns squared L2 distances
    good = dists[:, 0] < squared_ratio * dists[:, 1]
    return list(zip(results[good, 0], good.nonzero()[0]))


def match_flann_symmetric(
    fi: np.ndarray, indexi: Any, fj: np.ndarray, indexj: Any, config: Dict[str, Any]
) -> List[Tuple[int, int]]:
    """Match using FLANN in both directions and keep consistent matches.

    Args:
        fi: feature descriptors of the first image
        indexi: flann index if the first image
        fj: feature descriptors of the second image
        indexj: flann index of the second image
        config: config parameters
        maskij: optional boolean mask of len(i descriptors) x len(j descriptors)
    """
    matches_ij = [(a, b) for a, b in match_flann(indexi, fj, config)]
    matches_ji = [(b, a) for a, b in match_flann(indexj, fi, config)]

    return list(set(matches_ij).intersection(set(matches_ji)))


def match_brute_force(
    f1: np.ndarray,
    f2: np.ndarray,
    config: Dict[str, Any],
    maskij: Optional[np.ndarray] = None,
) -> List[Tuple[int, int]]:
    """Brute force matching and Lowe's ratio filtering.

    Args:
        f1: feature descriptors of the first image
        f2: feature descriptors of the second image
        config: config parameters
        maskij: optional boolean mask of len(i descriptors) x len(j descriptors)
    """
    assert f1.dtype.type == f2.dtype.type
    if f1.dtype.type == np.uint8:
        matcher_type = "BruteForce-Hamming"
    else:
        matcher_type = "BruteForce"
    matcher = cv2.DescriptorMatcher_create(matcher_type)
    matcher.add([f2])
    if maskij is not None:
        matches = matcher.knnMatch(f1, k=2, masks=np.array([maskij]).astype(np.uint8))
    else:
        matches = matcher.knnMatch(f1, k=2)

    ratio = config["lowes_ratio"]
    good_matches = []
    for match in matches:
        if match and len(match) == 2:
            m, n = match
            if m.distance < ratio * n.distance:
                good_matches.append(m)
    return _convert_matches_to_vector(good_matches)


def _convert_matches_to_vector(matches: List[Any]) -> List[Tuple[int, int]]:
    """Convert Dmatch object to matrix form."""
    return [(mm.queryIdx, mm.trainIdx) for mm in matches]


def match_brute_force_symmetric(
    fi: np.ndarray,
    fj: np.ndarray,
    config: Dict[str, Any],
    maskij: Optional[np.ndarray] = None,
) -> List[Tuple[int, int]]:
    """Match with brute force in both directions and keep consistent matches.

    Args:
        fi: feature descriptors of the first image
        fj: feature descriptors of the second image
        config: config parameters
        maskij: optional boolean mask of len(i descriptors) x len(j descriptors)
    """
    matches_ij = [(a, b) for a, b in match_brute_force(fi, fj, config, maskij)]
    maskijT = maskij.T if maskij is not None else None
    matches_ji = [(b, a) for a, b in match_brute_force(fj, fi, config, maskijT)]

    return list(set(matches_ij).intersection(set(matches_ji)))


def robust_match_fundamental(
    p1: np.ndarray,
    p2: np.ndarray,
    matches: np.ndarray,
    config: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray]:
    """Filter matches by estimating the Fundamental matrix via RANSAC."""
    if len(matches) < 8:
        return np.array([]), np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()

    FM_RANSAC = cv2.FM_RANSAC if context.OPENCV3 else cv2.cv.CV_FM_RANSAC
    threshold = config["robust_matching_threshold"]
    F, mask = cv2.findFundamentalMat(p1, p2, FM_RANSAC, threshold, 0.9999)
    inliers = mask.ravel().nonzero()

    if F is None or F[2, 2] == 0.0:
        return F, np.array([])

    return F, matches[inliers]


def compute_inliers_bearings(
    b1: np.ndarray,
    b2: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    threshold: float = 0.01,
) -> List[bool]:
    """Compute points that can be triangulated.

    Args:
        b1, b2: Bearings in the two images.
        R, t: Rotation and translation from the second image to the first.
              That is the convention and the opposite of many
              functions in this module.
        threshold: max reprojection error in radians.
    Returns:
        array: Array of boolean indicating inliers/outliers
    """
    p = pygeometry.triangulate_two_bearings_midpoint_many(b1, b2, R, t)

    good_idx = [i for i in range(len(p)) if p[i][0]]
    points = np.array([p[i][1] for i in range(len(p)) if p[i][0]])

    inliers = [False] * len(b1)
    if len(points) < 1:
        return inliers

    br1 = points.copy()
    br1 /= np.linalg.norm(br1, axis=1)[:, np.newaxis]
    br2 = R.T.dot((points - t).T).T
    br2 /= np.linalg.norm(br2, axis=1)[:, np.newaxis]

    ok1 = np.linalg.norm(br1 - b1[good_idx], axis=1) < threshold
    ok2 = np.linalg.norm(br2 - b2[good_idx], axis=1) < threshold
    is_ok = ok1 * ok2

    for i, ok in enumerate(is_ok):
        inliers[good_idx[i]] = ok
    return inliers


def compute_inliers_bearing_epipolar(
    b1: np.ndarray, b2: np.ndarray, pose: pygeometry.Pose, threshold: float
) -> np.ndarray:
    """Compute mask of epipolarly consistent bearings, given two lists of bearings

    Args:
        b1, b2: Bearings in the two images. Expected to be normalized.
        pose: Pose of the second image wrt. the first one (relative pose)
        threshold: max reprojection error in radians.
    Returns:
        array: Matrix of boolean indicating inliers/outliers
    """
    symmetric_angle_error = pygeometry.epipolar_angle_two_bearings_many(
        b1.astype(np.float32),
        b2.astype(np.float32),
        pose.get_R_cam_to_world(),
        pose.get_origin(),
    )
    mask = symmetric_angle_error < threshold
    return mask


def robust_match_calibrated(
    p1: np.ndarray,
    p2: np.ndarray,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    matches: np.ndarray,
    config: Dict[str, Any],
) -> np.ndarray:
    """Filter matches by estimating the Essential matrix via RANSAC."""

    if len(matches) < 8:
        return np.array([])

    p1 = p1[matches[:, 0]][:, :2].copy()
    p2 = p2[matches[:, 1]][:, :2].copy()
    b1 = camera1.pixel_bearing_many(p1)
    b2 = camera2.pixel_bearing_many(p2)

    threshold = config["robust_matching_calib_threshold"]
    T = multiview.relative_pose_ransac(b1, b2, threshold, 1000, 0.999)

    for relax in [4, 2, 1]:
        inliers = compute_inliers_bearings(b1, b2, T[:, :3], T[:, 3], relax * threshold)
        if np.sum(inliers) < 8:
            return np.array([])
        iterations = config["five_point_refine_match_iterations"]
        T = multiview.relative_pose_optimize_nonlinear(
            b1[inliers], b2[inliers], T[:3, 3], T[:3, :3], iterations
        )

    inliers = compute_inliers_bearings(b1, b2, T[:, :3], T[:, 3], threshold)

    return matches[inliers]


def robust_match(
    p1: np.ndarray,
    p2: np.ndarray,
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
    matches: np.ndarray,
    config: Dict[str, Any],
) -> np.ndarray:
    """Filter matches by fitting a geometric model.

    If cameras are perspective without distortion, then the Fundamental
    matrix is used.  Otherwise, we use the Essential matrix.
    """
    if (
        camera1.projection_type in ["perspective", "brown"]
        and camera1.k1 == 0.0
        and camera1.k2 == 0.0
        and camera2.projection_type in ["perspective", "brown"]
        and camera2.k1 == 0.0
        and camera2.k2 == 0.0
    ):
        return robust_match_fundamental(p1, p2, matches, config)[1]
    else:
        return robust_match_calibrated(p1, p2, camera1, camera2, matches, config)


def unfilter_matches(matches, m1, m2) -> np.ndarray:
    """Given matches and masking arrays, get matches with un-masked indexes."""
    i1 = np.flatnonzero(m1)
    i2 = np.flatnonzero(m2)
    return np.array([(i1[match[0]], i2[match[1]]) for match in matches])


def apply_adhoc_filters(
    data: DataSetBase,
    matches: List[Tuple[int, int]],
    im1: str,
    camera1: pygeometry.Camera,
    p1: np.ndarray,
    im2: str,
    camera2: pygeometry.Camera,
    p2: np.ndarray,
) -> List[Tuple[int, int]]:
    """Apply a set of filters functions defined further below
    for removing static data in images.

    """
    matches = _non_static_matches(p1, p2, matches)
    matches = _not_on_pano_poles_matches(p1, p2, matches, camera1, camera2)
    matches = _not_on_vermont_watermark(p1, p2, matches, im1, im2, data)
    matches = _not_on_blackvue_watermark(p1, p2, matches, im1, im2, data)
    return matches


def _non_static_matches(
    p1: np.ndarray, p2: np.ndarray, matches: List[Tuple[int, int]]
) -> List[Tuple[int, int]]:
    """Remove matches with same position in both images.

    That should remove matches on that are likely belong to rig occluders,
    watermarks or dust, but not discard entirely static images.
    """
    threshold = 0.001
    res = []
    for match in matches:
        d = p1[match[0]] - p2[match[1]]
        if d[0] ** 2 + d[1] ** 2 >= threshold**2:
            res.append(match)

    static_ratio_threshold = 0.85
    static_ratio_removed = 1 - len(res) / max(len(matches), 1)
    if static_ratio_removed > static_ratio_threshold:
        return matches
    else:
        return res


def _not_on_pano_poles_matches(
    p1: np.ndarray,
    p2: np.ndarray,
    matches: List[Tuple[int, int]],
    camera1: pygeometry.Camera,
    camera2: pygeometry.Camera,
) -> List[Tuple[int, int]]:
    """Remove matches for features that are too high or to low on a pano.

    That should remove matches on the sky and and carhood part of panoramas
    """
    min_lat = -0.125
    max_lat = 0.125
    is_pano1 = pygeometry.Camera.is_panorama(camera1.projection_type)
    is_pano2 = pygeometry.Camera.is_panorama(camera2.projection_type)
    if is_pano1 or is_pano2:
        res = []
        for match in matches:
            if (not is_pano1 or min_lat < p1[match[0]][1] < max_lat) and (
                not is_pano2 or min_lat < p2[match[1]][1] < max_lat
            ):
                res.append(match)
        return res
    else:
        return matches


def _not_on_vermont_watermark(
    p1: np.ndarray,
    p2: np.ndarray,
    matches: List[Tuple[int, int]],
    im1: str,
    im2: str,
    data: DataSetBase,
) -> List[Tuple[int, int]]:
    """Filter Vermont images watermark."""
    meta1 = data.load_exif(im1)
    meta2 = data.load_exif(im2)

    if meta1["make"] == "VTrans_Camera" and meta1["model"] == "VTrans_Camera":
        matches = [m for m in matches if _vermont_valid_mask(p1[m[0]])]
    if meta2["make"] == "VTrans_Camera" and meta2["model"] == "VTrans_Camera":
        matches = [m for m in matches if _vermont_valid_mask(p2[m[1]])]
    return matches


def _vermont_valid_mask(p: np.ndarray) -> bool:
    """Check if pixel inside the valid region.

    Pixel coord Y should be larger than 50.
    In normalized coordinates y > (50 - h / 2) / w
    """
    return p[1] > -0.255


def _not_on_blackvue_watermark(
    p1: np.ndarray,
    p2: np.ndarray,
    matches: List[Tuple[int, int]],
    im1: str,
    im2: str,
    data: DataSetBase,
) -> List[Tuple[int, int]]:
    """Filter Blackvue's watermark."""
    meta1 = data.load_exif(im1)
    meta2 = data.load_exif(im2)

    if meta1["make"].lower() == "blackvue":
        matches = [m for m in matches if _blackvue_valid_mask(p1[m[0]])]
    if meta2["make"].lower() == "blackvue":
        matches = [m for m in matches if _blackvue_valid_mask(p2[m[1]])]
    return matches


def _blackvue_valid_mask(p: np.ndarray) -> bool:
    """Check if pixel inside the valid region.

    Pixel coord Y should be smaller than h - 70.
    In normalized coordinates y < (h - 70 - h / 2) / w,
    with h = 2160 and w = 3840
    """
    return p[1] < 0.263
