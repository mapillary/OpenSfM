import logging
import typing as t

import cv2
import numpy as np
from opensfm import io
from opensfm import log
from opensfm import pydense
from opensfm import pymap
from opensfm import pysfm
from opensfm import tracking
from opensfm import types
from opensfm.context import parallel_map
from opensfm.dataset import UndistortedDataSet

logger = logging.getLogger(__name__)


def compute_depthmaps(
    data: UndistortedDataSet,
    graph: pymap.TracksManager,
    reconstruction: types.Reconstruction,
):
    """Compute and refine depthmaps for all shots.

    Args:
        data: an UndistortedDataset
        graph: the tracks graph
        reconstruction: the undistorted reconstruction
    """
    logger.info("Computing neighbors")
    config = data.config
    processes = config["processes"]
    num_neighbors = config["depthmap_num_neighbors"]

    neighbors = {}
    common_tracks = common_tracks_double_dict(graph)
    for shot in reconstruction.shots.values():
        neighbors[shot.id] = find_neighboring_images(
            shot, common_tracks, reconstruction, num_neighbors
        )

    arguments = []
    for shot in reconstruction.shots.values():
        if len(neighbors[shot.id]) <= 1:
            continue
        mind, maxd = compute_depth_range(graph, reconstruction, shot, config)
        arguments.append((data, neighbors[shot.id], mind, maxd, shot))
    parallel_map(compute_depthmap_catched, arguments, processes)

    arguments = []
    for shot in reconstruction.shots.values():
        if len(neighbors[shot.id]) <= 1:
            continue
        arguments.append((data, neighbors[shot.id], shot))
    parallel_map(clean_depthmap_catched, arguments, processes)

    arguments = []
    for shot in reconstruction.shots.values():
        if len(neighbors[shot.id]) <= 1:
            continue
        arguments.append((data, neighbors[shot.id], shot))
    parallel_map(prune_depthmap_catched, arguments, processes)

    point_cloud = merge_depthmaps(data, reconstruction)
    data.save_point_cloud(*point_cloud, filename="merged.ply")


def compute_depthmap_catched(arguments):
    try:
        compute_depthmap(arguments)
    except Exception as e:
        logger.error("Exception on child. Arguments: {}".format(arguments))
        logger.exception(e)


def clean_depthmap_catched(arguments):
    try:
        clean_depthmap(arguments)
    except Exception as e:
        logger.error("Exception on child. Arguments: {}".format(arguments))
        logger.exception(e)


def prune_depthmap_catched(arguments):
    try:
        prune_depthmap(arguments)
    except Exception as e:
        logger.error("Exception on child. Arguments: {}".format(arguments))
        logger.exception(e)


def compute_depthmap(arguments):
    """Compute depthmap for a single shot."""
    log.setup()

    data: UndistortedDataSet = arguments[0]
    neighbors = arguments[1]
    min_depth = arguments[2]
    max_depth = arguments[3]
    shot = arguments[4]

    method = data.config["depthmap_method"]

    if data.raw_depthmap_exists(shot.id):
        logger.info("Using precomputed raw depthmap {}".format(shot.id))
        return
    logger.info("Computing depthmap for image {0} with {1}".format(shot.id, method))

    de = pydense.DepthmapEstimator()
    de.set_depth_range(min_depth, max_depth, 100)
    de.set_patchmatch_iterations(data.config["depthmap_patchmatch_iterations"])
    de.set_patch_size(data.config["depthmap_patch_size"])
    de.set_min_patch_sd(data.config["depthmap_min_patch_sd"])
    add_views_to_depth_estimator(data, neighbors, de)

    if method == "BRUTE_FORCE":
        depth, plane, score, nghbr = de.compute_brute_force()
    elif method == "PATCH_MATCH":
        depth, plane, score, nghbr = de.compute_patch_match()
    elif method == "PATCH_MATCH_SAMPLE":
        depth, plane, score, nghbr = de.compute_patch_match_sample()
    else:
        raise ValueError(
            "Unknown depthmap method type "
            "(must be BRUTE_FORCE, PATCH_MATCH or PATCH_MATCH_SAMPLE)"
        )

    good_score = score > data.config["depthmap_min_correlation_score"]
    depth = depth * (depth < max_depth) * good_score

    # Save and display results
    neighbor_ids = [i.id for i in neighbors[1:]]
    data.save_raw_depthmap(shot.id, depth, plane, score, nghbr, neighbor_ids)

    if data.config["depthmap_save_debug_files"]:
        image = data.load_undistorted_image(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with io.open_wt(data.depthmap_file(shot.id, "raw.npz.ply")) as fout:
            fout.write(ply)

    if data.config.get("interactive"):
        import matplotlib.pyplot as plt

        plt.figure()
        plt.suptitle("Shot: " + shot.id + ", neighbors: " + ", ".join(neighbor_ids))
        plt.subplot(2, 3, 1)
        plt.imshow(image)
        plt.subplot(2, 3, 2)
        plt.imshow(color_plane_normals(plane))
        plt.subplot(2, 3, 3)
        plt.imshow(depth)
        plt.colorbar()
        plt.subplot(2, 3, 4)
        plt.imshow(score)
        plt.colorbar()
        plt.subplot(2, 3, 5)
        plt.imshow(nghbr)
        plt.colorbar()
        plt.show()


def clean_depthmap(arguments):
    """Clean depthmap by checking consistency with neighbors."""
    log.setup()

    data: UndistortedDataSet = arguments[0]
    neighbors = arguments[1]
    shot = arguments[2]

    if data.clean_depthmap_exists(shot.id):
        logger.info("Using precomputed clean depthmap {}".format(shot.id))
        return
    logger.info("Cleaning depthmap for image {}".format(shot.id))

    dc = pydense.DepthmapCleaner()
    dc.set_same_depth_threshold(data.config["depthmap_same_depth_threshold"])
    dc.set_min_consistent_views(data.config["depthmap_min_consistent_views"])
    add_views_to_depth_cleaner(data, neighbors, dc)
    depth = dc.clean()

    # Save and display results
    raw_depth, raw_plane, raw_score, raw_nghbr, nghbrs = data.load_raw_depthmap(shot.id)
    data.save_clean_depthmap(shot.id, depth, raw_plane, raw_score)

    if data.config["depthmap_save_debug_files"]:
        image = data.load_undistorted_image(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with io.open_wt(data.depthmap_file(shot.id, "clean.npz.ply")) as fout:
            fout.write(ply)

    if data.config.get("interactive"):
        import matplotlib.pyplot as plt

        plt.figure()
        plt.suptitle("Shot: " + shot.id)
        plt.subplot(2, 2, 1)
        plt.imshow(raw_depth)
        plt.colorbar()
        plt.subplot(2, 2, 2)
        plt.imshow(depth)
        plt.colorbar()
        plt.show()


def prune_depthmap(arguments):
    """Prune depthmap to remove redundant points."""
    log.setup()

    data: UndistortedDataSet = arguments[0]
    neighbors = arguments[1]
    shot = arguments[2]

    if data.pruned_depthmap_exists(shot.id):
        logger.info("Using precomputed pruned depthmap {}".format(shot.id))
        return
    logger.info("Pruning depthmap for image {}".format(shot.id))

    dp = pydense.DepthmapPruner()
    dp.set_same_depth_threshold(data.config["depthmap_same_depth_threshold"])
    add_views_to_depth_pruner(data, neighbors, dp)
    points, normals, colors, labels = dp.prune()

    # Save and display results
    data.save_pruned_depthmap(shot.id, points, normals, colors, labels)

    if data.config["depthmap_save_debug_files"]:
        data.save_point_cloud(points, normals, colors, labels, "pruned.npz.ply")


def aggregate_depthmaps(shot_ids, depthmap_provider):
    """Aggregate depthmaps by concatenation."""

    points = []
    normals = []
    colors = []
    labels = []
    for shot_id in shot_ids:
        p, n, c, l = depthmap_provider(shot_id)
        points.append(p)
        normals.append(n)
        colors.append(c)
        labels.append(l)

    return (
        np.concatenate(points),
        np.concatenate(normals),
        np.concatenate(colors),
        np.concatenate(labels),
    )


def merge_depthmaps(
    data: UndistortedDataSet, reconstruction: types.Reconstruction
) -> t.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Merge depthmaps into a single point cloud."""
    shot_ids = [s for s in reconstruction.shots if data.pruned_depthmap_exists(s)]

    def depthmap_provider(shot_id):
        return data.load_pruned_depthmap(shot_id)

    return merge_depthmaps_from_provider(shot_ids, depthmap_provider)


def merge_depthmaps_from_provider(
    shot_ids: t.Iterable[str], depthmap_provider: t.Callable
) -> t.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Merge depthmaps into a single point cloud."""
    logger.info("Merging depthmaps")

    if not shot_ids:
        logger.warning("Depthmaps contain no points.  Try using more images.")
        return np.array([]), np.array([]), np.array([]), np.array([])

    return aggregate_depthmaps(shot_ids, depthmap_provider)


def add_views_to_depth_estimator(data: UndistortedDataSet, neighbors, de):
    """Add neighboring views to the DepthmapEstimator."""
    num_neighbors = data.config["depthmap_num_matching_views"]
    for shot in neighbors[: num_neighbors + 1]:
        assert shot.camera.projection_type == "perspective"
        color_image = data.load_undistorted_image(shot.id)
        mask = load_combined_mask(data, shot)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        original_height, original_width = gray_image.shape
        width = min(original_width, int(data.config["depthmap_resolution"]))
        height = width * original_height // original_width
        image = scale_down_image(gray_image, width, height)
        mask = scale_down_image(mask, width, height, cv2.INTER_NEAREST)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        de.add_view(K, R, t, image, mask)


def add_views_to_depth_cleaner(data: UndistortedDataSet, neighbors, dc):
    for shot in neighbors:
        if not data.raw_depthmap_exists(shot.id):
            continue
        depth, plane, score, nghbr, nghbrs = data.load_raw_depthmap(shot.id)
        height, width = depth.shape
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dc.add_view(K, R, t, depth)


def load_combined_mask(data: UndistortedDataSet, shot):
    """Load the undistorted mask.

    If no mask exists return an array of ones.
    """
    mask = data.load_undistorted_combined_mask(shot.id)
    if mask is None:
        size = int(shot.camera.height), int(shot.camera.width)
        return np.ones(size, dtype=np.uint8)
    else:
        return mask


def load_segmentation_labels(data: UndistortedDataSet, shot):
    """Load the undistorted segmentation labels.

    If no segmentation exists return an array of zeros.
    """
    if data.undistorted_segmentation_exists(shot.id):
        return data.load_undistorted_segmentation(shot.id)
    else:
        size = shot.camera.height, shot.camera.width
        return np.zeros(size, dtype=np.uint8)


def add_views_to_depth_pruner(data: UndistortedDataSet, neighbors, dp):
    for shot in neighbors:
        if not data.clean_depthmap_exists(shot.id):
            continue
        depth, plane, score = data.load_clean_depthmap(shot.id)
        height, width = depth.shape
        color_image = data.load_undistorted_image(shot.id)
        labels = load_segmentation_labels(data, shot)
        height, width = depth.shape
        image = scale_down_image(color_image, width, height)
        labels = scale_down_image(labels, width, height, cv2.INTER_NEAREST)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dp.add_view(K, R, t, depth, plane, image, labels)


def compute_depth_range(tracks_manager, reconstruction, shot, config):
    """Compute min and max depth based on reconstruction points."""
    depths = []
    for track in tracks_manager.get_shot_observations(shot.id):
        if track in reconstruction.points:
            p = reconstruction.points[track].coordinates
            z = shot.pose.transform(p)[2]
            depths.append(z)
    min_depth = np.percentile(depths, 10) * 0.9
    max_depth = np.percentile(depths, 90) * 1.1

    config_min_depth = config["depthmap_min_depth"]
    config_max_depth = config["depthmap_max_depth"]

    return config_min_depth or min_depth, config_max_depth or max_depth


def common_tracks_double_dict(
    tracks_manager: pymap.TracksManager,
) -> t.Dict[str, t.Dict[str, t.List[str]]]:
    """List of track ids observed by each image pair.

    Return a dict, ``res``, such that ``res[im1][im2]`` is the list of
    common tracks between ``im1`` and ``im2``.
    """
    common_tracks_per_pair = tracking.all_common_tracks_without_features(tracks_manager)
    res = {image: {} for image in tracks_manager.get_shot_ids()}
    for (im1, im2), v in common_tracks_per_pair.items():
        res[im1][im2] = v
        res[im2][im1] = v
    return res


def find_neighboring_images(
    shot: pymap.Shot,
    common_tracks: t.Dict[str, t.Dict[str, t.List[str]]],
    reconstruction: types.Reconstruction,
    num_neighbors: int,
):
    """Find neighboring images based on common tracks."""
    theta_min = np.pi / 60
    theta_max = np.pi / 6
    ns = []
    C1 = shot.pose.get_origin()
    for other_id, tracks in common_tracks.get(shot.id, {}).items():
        if other_id not in reconstruction.shots:
            continue
        other = reconstruction.shots[other_id]
        score = 0
        C2 = other.pose.get_origin()
        for track in tracks:
            if track in reconstruction.points:
                p = reconstruction.points[track].coordinates
                theta = angle_between_points(p, C1, C2)
                if theta > theta_min and theta < theta_max:
                    score += 1
        if score > 20:
            ns.append((other, score))

    ns.sort(key=lambda ns: ns[1], reverse=True)
    return [shot] + [n for n, s in ns[:num_neighbors]]


def angle_between_points(origin, p1, p2):
    a0 = p1[0] - origin[0]
    a1 = p1[1] - origin[1]
    a2 = p1[2] - origin[2]
    b0 = p2[0] - origin[0]
    b1 = p2[1] - origin[1]
    b2 = p2[2] - origin[2]
    dot = a0 * b0 + a1 * b1 + a2 * b2
    la = a0 * a0 + a1 * a1 + a2 * a2
    lb = b0 * b0 + b1 * b1 + b2 * b2
    return np.arccos(dot / np.sqrt(la * lb))


def distance_between_shots(shot, other):
    o1 = shot.pose.get_origin()
    o2 = other.pose.get_origin()
    d = o2 - o1
    return np.sqrt(np.sum(d ** 2))


def scale_down_image(image, width, height, interpolation=cv2.INTER_AREA):
    width = min(width, image.shape[1])
    height = min(height, image.shape[0])
    return cv2.resize(image, (width, height), interpolation=interpolation)


def depthmap_to_ply(shot, depth, image):
    """Export depthmap points as a PLY string"""
    height, width = depth.shape
    K = shot.camera.get_K_in_pixel_coordinates(width, height)
    R = shot.pose.get_rotation_matrix()
    t = shot.pose.translation
    y, x = np.mgrid[:height, :width]
    v = np.vstack((x.ravel(), y.ravel(), np.ones(width * height)))
    camera_coords = depth.reshape((1, -1)) * np.linalg.inv(K).dot(v)
    points = R.T.dot(camera_coords - t.reshape(3, 1))

    vertices = []
    for p, c, d in zip(points.T, image.reshape(-1, 3), depth.reshape(-1, 1)):
        if d != 0:  # ignore points with zero depth
            s = "{} {} {} {} {} {}".format(p[0], p[1], p[2], c[0], c[1], c[2])
            vertices.append(s)

    return io.points_to_ply_string(vertices)


def color_plane_normals(plane):
    norm = np.linalg.norm(plane, axis=2)
    normal = plane / norm[..., np.newaxis]
    normal[..., 1] *= -1  # Reverse Y because it points down
    normal[..., 2] *= -1  # Reverse Z because standard colormap does so
    return ((normal + 1) * 128).astype(np.uint8)
