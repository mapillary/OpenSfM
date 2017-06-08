
import logging
from multiprocessing import Pool

import cv2
import numpy as np

from opensfm import csfm
from opensfm import matching


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
        arguments.append((data, reconstruction, neighbors, min_depth, max_depth, shot))
    parallel_run(compute_depthmap, arguments, processes)

    arguments = []
    for shot in reconstruction.shots.values():
        if len(neighbors[shot.id]) <= 1:
            continue
        arguments.append((data, reconstruction, neighbors, shot))
    parallel_run(clean_depthmap, arguments, processes)

    merge_depthmaps(data, graph, reconstruction, neighbors)


def parallel_run(function, arguments, num_processes):
    """Run function for all arguments using multiple processes."""
    num_processes = min(num_processes, len(arguments))
    if num_processes == 1:
        return [function(arg) for arg in arguments]
    else:
        p = Pool(num_processes)
        return p.map(function, arguments)


def compute_depthmap(arguments):
    """Compute depthmap for a single shot."""
    data, reconstruction, neighbors, min_depth, max_depth, shot = arguments
    method = data.config['depthmap_method']

    if data.raw_depthmap_exists(shot.id):
        logger.info("Using precomputed raw depthmap {}".format(shot.id))
        return
    logger.info("Computing depthmap for image {0} with {1}".format(shot.id, method))

    de = csfm.DepthmapEstimator()
    de.set_depth_range(min_depth, max_depth, 100)
    de.set_patchmatch_iterations(data.config['depthmap_patchmatch_iterations'])
    de.set_min_patch_sd(data.config['depthmap_min_patch_sd'])
    add_views_to_depth_estimator(data, reconstruction, neighbors[shot.id], de)

    if (method == 'BRUTE_FORCE'):
        depth, plane, score, nghbr = de.compute_brute_force()
    elif (method == 'PATCH_MATCH'):
        depth, plane, score, nghbr = de.compute_patch_match()
    elif (method == 'PATCH_MATCH_SAMPLE'):
        depth, plane, score, nghbr = de.compute_patch_match_sample()
    else:
        raise ValueError('Unknown depthmap method type ' \
            '(must be BRUTE_FORCE, PATCH_MATCH or PATCH_MATCH_SAMPLE)')

    good_score = score > data.config['depthmap_min_correlation_score']
    depth = depth * (depth < max_depth) * good_score

    # Save and display results
    data.save_raw_depthmap(shot.id, depth, plane, score, nghbr, neighbors[shot.id][1:])

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with open(data._depthmap_file(shot.id, 'raw.npz.ply'), 'w') as fout:
            fout.write(ply)

    if data.config.get('interactive'):
        import matplotlib.pyplot as plt
        plt.figure()
        plt.suptitle("Shot: " + shot.id + ", neighbors: " + ', '.join(neighbors[shot.id][1:]))
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
    data, reconstruction, neighbors, shot = arguments

    if data.clean_depthmap_exists(shot.id):
        logger.info("Using precomputed clean depthmap {}".format(shot.id))
        return
    logger.info("Cleaning depthmap for image {}".format(shot.id))

    dc = csfm.DepthmapCleaner()
    dc.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    dc.set_min_consistent_views(data.config['depthmap_min_consistent_views'])
    add_views_to_depth_cleaner(data, reconstruction, neighbors[shot.id], dc)
    depth = dc.clean()

    # Save and display results
    raw_depth, raw_plane, raw_score, raw_nghbr, nghbrs = data.load_raw_depthmap(shot.id)
    data.save_clean_depthmap(shot.id, depth, raw_plane, raw_score)

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with open(data._depthmap_file(shot.id, 'clean.npz.ply'), 'w') as fout:
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


def merge_depthmaps(data, graph, reconstruction, neighbors):
    """Merge depthmaps into a single point cloud."""
    logger.info("Merging depthmaps")

    # Load clean depthmaps.
    depths, planes = {}, {}
    for sid in neighbors:
        if data.clean_depthmap_exists(sid):
            depths[sid], planes[sid], _ = data.load_clean_depthmap(sid)

    # Set up merger.
    dm = csfm.DepthmapMerger()
    dm.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    shot_ids = depths.keys()
    indices = {k: i for i, k in enumerate(shot_ids)}
    for shot_id in shot_ids:
        depth = depths[shot_id]
        shot = reconstruction.shots[shot_id]
        neighbors_indices = [indices[n] for n in neighbors[shot.id] if n in indices]
        color_image = data.undistorted_image_as_array(shot.id)
        height, width = depth.shape
        image = scale_down_image(color_image, width, height)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dm.add_view(K, R, t, depth, planes[shot.id], image, neighbors_indices)

    # Merge.
    points, normals, colors = dm.merge()
    ply = point_cloud_to_ply(points, normals, colors)
    with open(data._depthmap_path() + '/merged.ply', 'w') as fout:
        fout.write(ply)


def add_views_to_depth_estimator(data, reconstruction, neighbors, de):
    """Add neighboring views to the DepthmapEstimator."""
    num_neighbors = data.config['depthmap_num_matching_views']
    for neighbor in neighbors[:num_neighbors + 1]:
        shot = reconstruction.shots[neighbor]
        assert shot.camera.projection_type == 'perspective'
        color_image = data.undistorted_image_as_array(shot.id)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        original_height, original_width = gray_image.shape
        width = int(data.config['depthmap_resolution'])
        height = width * original_height / original_width
        image = scale_down_image(gray_image, width, height)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        de.add_view(K, R, t, image)


def add_views_to_depth_cleaner(data, reconstruction, neighbors, dc):
    for neighbor in neighbors:
        shot = reconstruction.shots[neighbor]
        if not data.raw_depthmap_exists(shot.id):
            continue
        depth, plane, score, nghbr, nghbrs = data.load_raw_depthmap(shot.id)
        height, width = depth.shape
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dc.add_view(K, R, t, depth)


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
                p = np.array(reconstruction.points[track].coordinates)
                theta = angle_between_points(p, C1, C2)
                if theta > theta_min and theta < theta_max:
                    score += 1

        if (score > 20):
            ns.append((other, score))

    ns.sort(key=lambda ns: ns[1], reverse=True)
    return [shot.id] + [n.id for n, s in ns[:num_neighbors]]


def angle_between_points(origin, p1, p2):
    a = np.asarray(p1) - origin
    b = np.asarray(p2) - origin
    return np.arccos(a.dot(b) / (np.linalg.norm(a) * np.linalg.norm(b)))


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
        s = "{} {} {} {} {} {}".format(p[0], p[1], p[2], c[0], c[1], c[2])
        vertices.append(s)

    header = [
        "ply",
        "format ascii 1.0",
        "element vertex {}".format(len(vertices)),
        "property float x",
        "property float y",
        "property float z",
        "property uchar diffuse_red",
        "property uchar diffuse_green",
        "property uchar diffuse_blue",
        "end_header",
    ]

    return '\n'.join(header + vertices + [''])


def point_cloud_to_ply(points, normals, colors):
    """Export depthmap points as a PLY string"""
    vertices = []
    for p, n, c in zip(points, normals, colors):
        s = "{:.4f} {:.4f} {:.4f} {:.3f} {:.3f} {:.3f} {} {} {}".format(
            p[0], p[1], p[2], n[0], n[1], n[2], int(c[0]), int(c[1]), int(c[2]))
        vertices.append(s)

    header = [
        "ply",
        "format ascii 1.0",
        "element vertex {}".format(len(vertices)),
        "property float x",
        "property float y",
        "property float z",
        "property float nx",
        "property float ny",
        "property float nz",
        "property uchar diffuse_red",
        "property uchar diffuse_green",
        "property uchar diffuse_blue",
        "end_header",
    ]

    return '\n'.join(header + vertices + [''])


def color_plane_normals(plane):
    l = np.linalg.norm(plane, axis=2)
    normal = plane / l[..., np.newaxis]
    normal[..., 1] *= -1  # Reverse Y because it points down
    normal[..., 2] *= -1  # Reverse Z because standard colormap does so
    return ((normal + 1) * 128).astype(np.uint8)
