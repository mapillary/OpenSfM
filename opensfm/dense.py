
import logging

import cv2
import numpy as np

from opensfm import csfm
from opensfm import io
from opensfm import log
from opensfm import matching
from opensfm.context import parallel_map


logger = logging.getLogger(__name__)


def compute_depthmaps(data, graph, reconstruction):
    """Compute and refine depthmaps for all shots."""
    logger.info('Computing neighbors')
    processes = data.config.get('processes', 1)
    num_neighbors = data.config['depthmap_num_neighbors']
    tracks, _ = matching.tracks_and_images(graph)
    common_tracks = matching.all_common_tracks(graph, tracks, include_features=False)

    neighbors = {}
    for shot in reconstruction.shots.values():
        neighbors[shot.id] = find_neighboring_images(
            shot, common_tracks, reconstruction, num_neighbors)

    arguments = []
    for shot in reconstruction.shots.values():
        if len(neighbors[shot.id]) <= 1:
            continue
        min_depth, max_depth = compute_depth_range(graph, reconstruction, shot)
        arguments.append((data, neighbors[shot.id], min_depth, max_depth, shot))
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

    merge_depthmaps(data, graph, reconstruction, neighbors)


def compute_depthmap_catched(arguments):
    try:
        compute_depthmap(arguments)
    except Exception as e:
        logger.error('Exception on child. Arguments: {}'.format(arguments))
        logger.exception(e)


def clean_depthmap_catched(arguments):
    try:
        clean_depthmap(arguments)
    except Exception as e:
        logger.error('Exception on child. Arguments: {}'.format(arguments))
        logger.exception(e)


def prune_depthmap_catched(arguments):
    try:
        prune_depthmap(arguments)
    except Exception as e:
        logger.error('Exception on child. Arguments: {}'.format(arguments))
        logger.exception(e)


def compute_depthmap(arguments):
    """Compute depthmap for a single shot."""
    log.setup()

    data, neighbors, min_depth, max_depth, shot = arguments
    method = data.config['depthmap_method']

    if data.raw_depthmap_exists(shot.id):
        logger.info("Using precomputed raw depthmap {}".format(shot.id))
        return
    logger.info("Computing depthmap for image {0} with {1}".format(shot.id, method))

    de = csfm.DepthmapEstimator()
    de.set_depth_range(min_depth, max_depth, 100)
    de.set_patchmatch_iterations(data.config['depthmap_patchmatch_iterations'])
    de.set_min_patch_sd(data.config['depthmap_min_patch_sd'])
    add_views_to_depth_estimator(data, neighbors, de)

    if (method == 'BRUTE_FORCE'):
        depth, plane, score, nghbr = de.compute_brute_force()
    elif (method == 'PATCH_MATCH'):
        depth, plane, score, nghbr = de.compute_patch_match()
    elif (method == 'PATCH_MATCH_SAMPLE'):
        depth, plane, score, nghbr = de.compute_patch_match_sample()
    else:
        raise ValueError(
            'Unknown depthmap method type '
            '(must be BRUTE_FORCE, PATCH_MATCH or PATCH_MATCH_SAMPLE)')

    good_score = score > data.config['depthmap_min_correlation_score']
    depth = depth * (depth < max_depth) * good_score

    # Save and display results
    neighbor_ids = [i.id for i in neighbors[1:]]
    data.save_raw_depthmap(shot.id, depth, plane, score, nghbr, neighbor_ids)

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with io.open_wt(data._depthmap_file(shot.id, 'raw.npz.ply')) as fout:
            fout.write(ply)

    if data.config.get('interactive'):
        import matplotlib.pyplot as plt
        plt.figure()
        plt.suptitle("Shot: " + shot.id + ", neighbors: " + ', '.join(neighbor_ids))
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

    data, neighbors, shot = arguments

    if data.clean_depthmap_exists(shot.id):
        logger.info("Using precomputed clean depthmap {}".format(shot.id))
        return
    logger.info("Cleaning depthmap for image {}".format(shot.id))

    dc = csfm.DepthmapCleaner()
    dc.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    dc.set_min_consistent_views(data.config['depthmap_min_consistent_views'])
    add_views_to_depth_cleaner(data, neighbors, dc)
    depth = dc.clean()

    # Save and display results
    raw_depth, raw_plane, raw_score, raw_nghbr, nghbrs = data.load_raw_depthmap(shot.id)
    data.save_clean_depthmap(shot.id, depth, raw_plane, raw_score)

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with io.open_wt(data._depthmap_file(shot.id, 'clean.npz.ply')) as fout:
            fout.write(ply)

    if data.config.get('interactive'):
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

    data, neighbors, shot = arguments

    if data.pruned_depthmap_exists(shot.id):
        logger.info("Using precomputed pruned depthmap {}".format(shot.id))
        return
    logger.info("Pruning depthmap for image {}".format(shot.id))

    dp = csfm.DepthmapPruner()
    dp.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    add_views_to_depth_pruner(data, neighbors, dp)
    points, normals, colors = dp.prune()

    # Save and display results
    data.save_pruned_depthmap(shot.id, points, normals, colors)

    if data.config['depthmap_save_debug_files']:
        ply = point_cloud_to_ply(points, normals, colors)
        with io.open_wt(data._depthmap_file(shot.id, 'pruned.npz.ply')) as fout:
            fout.write(ply)


def merge_depthmaps(data, graph, reconstruction, neighbors):
    """Merge depthmaps into a single point cloud."""
    logger.info("Merging depthmaps")

    shot_ids = [s for s in neighbors if data.pruned_depthmap_exists(s)]
    points = []
    normals = []
    colors = []
    for shot_id in shot_ids:
        p, n, c = data.load_pruned_depthmap(shot_id)
        points.append(p)
        normals.append(n)
        colors.append(c)

    points = np.concatenate(points)
    normals = np.concatenate(normals)
    colors = np.concatenate(colors)

    ply = point_cloud_to_ply(points, normals, colors)
    with io.open_wt(data._depthmap_path() + '/merged.ply') as fout:
        fout.write(ply)


def add_views_to_depth_estimator(data, neighbors, de):
    """Add neighboring views to the DepthmapEstimator."""
    num_neighbors = data.config['depthmap_num_matching_views']
    for shot in neighbors[:num_neighbors + 1]:
        assert shot.camera.projection_type == 'perspective'
        color_image = data.undistorted_image_as_array(shot.id)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        original_height, original_width = gray_image.shape
        width = int(data.config['depthmap_resolution'])
        height = width * original_height // original_width
        image = scale_down_image(gray_image, width, height)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        de.add_view(K, R, t, image)


def add_views_to_depth_cleaner(data, neighbors, dc):
    for shot in neighbors:
        if not data.raw_depthmap_exists(shot.id):
            continue
        depth, plane, score, nghbr, nghbrs = data.load_raw_depthmap(shot.id)
        height, width = depth.shape
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dc.add_view(K, R, t, depth)


def add_views_to_depth_pruner(data, neighbors, dp):
    for shot in neighbors:
        if not data.raw_depthmap_exists(shot.id):
            continue
        depth, plane, score = data.load_clean_depthmap(shot.id)
        height, width = depth.shape
        color_image = data.undistorted_image_as_array(shot.id)
        height, width = depth.shape
        image = scale_down_image(color_image, width, height)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dp.add_view(K, R, t, depth, plane, image)


def compute_depth_range(graph, reconstruction, shot):
    """Compute min and max depth based on reconstruction points."""
    depths = []
    for track in graph[shot.id]:
        if track in reconstruction.points:
            p = reconstruction.points[track].coordinates
            z = shot.pose.transform(p)[2]
            depths.append(z)
    min_depth = np.percentile(depths, 10)
    max_depth = np.percentile(depths, 90)
    return min_depth * 0.9, max_depth * 1.1


def find_neighboring_images(shot, common_tracks, reconstruction, num_neighbors=5):
    """Find neighboring images based on common tracks."""
    theta_min = np.pi / 60
    theta_max = np.pi / 6
    ns = []
    C1 = shot.pose.get_origin()
    others = (s for s in reconstruction.shots.values() if s.id != shot.id)
    for other in others:
        score = 0
        C2 = other.pose.get_origin()
        tracks = common_tracks.get(tuple(sorted([shot.id, other.id])), [])
        for track in tracks:
            if track in reconstruction.points:
                p = reconstruction.points[track].coordinates
                theta = angle_between_points(p, C1, C2)
                if theta > theta_min and theta < theta_max:
                    score += 1

        if (score > 20):
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
    l = o2 - o1
    return np.sqrt(np.sum(l**2))


def scale_down_image(image, width, height):
    width = min(width, image.shape[1])
    height = min(height, image.shape[0])
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


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
    for p, c in zip(points.T, image.reshape(-1, 3)):
        s = u"{} {} {} {} {} {}".format(p[0], p[1], p[2], c[0], c[1], c[2])
        vertices.append(s)

    header = [
        u"ply",
        u"format ascii 1.0",
        u"element vertex {}".format(len(vertices)),
        u"property float x",
        u"property float y",
        u"property float z",
        u"property uchar diffuse_red",
        u"property uchar diffuse_green",
        u"property uchar diffuse_blue",
        u"end_header",
    ]

    return '\n'.join(header + vertices + [''])


def point_cloud_to_ply(points, normals, colors):
    """Export depthmap points as a PLY string"""
    vertices = []
    for p, n, c in zip(points, normals, colors):
        s = u"{:.4f} {:.4f} {:.4f} {:.3f} {:.3f} {:.3f} {} {} {}".format(
            p[0], p[1], p[2], n[0], n[1], n[2], int(c[0]), int(c[1]), int(c[2]))
        vertices.append(s)

    header = [
        u"ply",
        u"format ascii 1.0",
        u"element vertex {}".format(len(vertices)),
        u"property float x",
        u"property float y",
        u"property float z",
        u"property float nx",
        u"property float ny",
        u"property float nz",
        u"property uchar diffuse_red",
        u"property uchar diffuse_green",
        u"property uchar diffuse_blue",
        u"end_header",
    ]

    return '\n'.join(header + vertices + [''])


def color_plane_normals(plane):
    l = np.linalg.norm(plane, axis=2)
    normal = plane / l[..., np.newaxis]
    normal[..., 1] *= -1  # Reverse Y because it points down
    normal[..., 2] *= -1  # Reverse Z because standard colormap does so
    return ((normal + 1) * 128).astype(np.uint8)
