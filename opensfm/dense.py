
from functools import partial
import logging
from multiprocessing import Pool

import cv2
import numpy as np

from opensfm import csfm
from opensfm import matching


logger = logging.getLogger(__name__)


def compute_depthmaps(data, graph, reconstruction):
    """Compute and refine depthmaps for all shots."""
    pool = Pool(data.config.get('processes', 1))

    logger.info('Computing neighbors')
    neighbors = {}
    num_neighbors = data.config['depthmap_num_neighbors']

    neighboring_images = pool.map(partial(find_neighboring_images, graph, reconstruction, num_neighbors), reconstruction.shots.values())
    for (shot_id, neighboring_image) in zip(map(lambda shot: shot.id, shots), neighboring_images):
        neighbors[shot_id] = neighboring_image

    depths = {}
    planes = {}
    scores = {}

    shots = filter(lambda shot: len(neighbors[shot.id]) > 1, reconstruction.shots.values())
    depthmaps = pool.map(partial(compute_depthmap, data, graph, reconstruction, neighbors), shots)

    for (shot_id, (depth, plane, score)) in zip(map(lambda shot: shot.id, shots), depthmaps):
        depths[shot_id] = depth
        planes[shot_id] = plane
        scores[shot_id] = score

    clean_depths = {}

    shots = filter(lambda shot: shot.id in depths, reconstruction.shots.values())
    clean_depthmaps = pool.map(partial(clean_depthmap, data, graph, reconstruction, neighbors, depths, planes, scores), shots)

    for (shot_id, cleaned_depthmap) in zip(map(lambda shot: shot.id, shots), clean_depthmaps):
        clean_depths[shot_id] = cleaned_depthmap

    merge_depthmaps(data, graph, reconstruction, neighbors,
                    clean_depths, planes)


def compute_depthmap(data, graph, reconstruction, neighbors, shot):
    """Compute depthmap for a single shot."""
    logger.info("Computing depthmap for image {}".format(shot.id))

    if data.raw_depthmap_exists(shot.id):
        return data.load_raw_depthmap(shot.id)

    min_depth, max_depth = compute_depth_range(graph, reconstruction, shot)

    de = csfm.DepthmapEstimator()
    de.set_depth_range(min_depth, max_depth, 100)
    de.set_patchmatch_iterations(data.config['depthmap_patchmatch_iterations'])
    add_views_to_depth_estimator(data, reconstruction, neighbors[shot.id], de)
    depth, plane, score = de.compute_patch_match()
    good_score = score > data.config['depthmap_min_correlation_score']
    depth = depth * (depth < max_depth) * good_score

    # Save and display results
    data.save_raw_depthmap(shot.id, depth, plane, score)

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with open(data._depthmap_file(shot.id, 'raw.npz.ply'), 'w') as fout:
            fout.write(ply)

    if data.config.get('interactive'):
        import matplotlib.pyplot as plt
        plt.subplot(2, 2, 1)
        plt.imshow(image)
        plt.subplot(2, 2, 2)
        plt.imshow(color_plane_normals(plane))
        plt.subplot(2, 2, 3)
        plt.imshow(depth)
        plt.colorbar()
        plt.subplot(2, 2, 4)
        plt.imshow(score)
        plt.colorbar()
        plt.show()

    return depth, plane, score


def clean_depthmap(data, graph, reconstruction, neighbors, depths,
                   planes, scores, shot):
    logger.info("Cleaning depthmap for image {}".format(shot.id))

    if data.clean_depthmap_exists(shot.id):
        return data.load_clean_depthmap(shot.id)[0]

    dc = csfm.DepthmapCleaner()
    dc.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    dc.set_min_consistent_views(data.config['depthmap_min_consistent_views'])
    add_views_to_depth_cleaner(reconstruction, depths, neighbors[shot.id], dc)
    depth = dc.clean()

    # Save and display results
    data.save_clean_depthmap(shot.id, depth, planes[shot.id], scores[shot.id])

    if data.config['depthmap_save_debug_files']:
        image = data.undistorted_image_as_array(shot.id)
        image = scale_down_image(image, depth.shape[1], depth.shape[0])
        ply = depthmap_to_ply(shot, depth, image)
        with open(data._depthmap_file(shot.id, 'clean.npz.ply'), 'w') as fout:
            fout.write(ply)

    if data.config.get('interactive'):
        import matplotlib.pyplot as plt
        plt.subplot(2, 2, 1)
        plt.imshow(depths[shot.id])
        plt.colorbar()
        plt.subplot(2, 2, 2)
        plt.imshow(depth)
        plt.colorbar()
        plt.show()

    return depth


def merge_depthmaps(data, graph, reconstruction, neighbors,
                    clean_depths, planes):
    dm = csfm.DepthmapMerger()
    dm.set_same_depth_threshold(data.config['depthmap_same_depth_threshold'])
    shot_ids = clean_depths.keys()
    indices = {k: i for i, k in enumerate(shot_ids)}
    for shot_id in shot_ids:
        depth = clean_depths[shot_id]
        shot = reconstruction.shots[shot_id]
        neighbors_indices = [indices[n] for n in neighbors[shot.id]]
        color_image = data.undistorted_image_as_array(shot.id)
        height, width = depth.shape
        image = scale_down_image(color_image, width, height)
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        dm.add_view(K, R, t, depth, planes[shot.id], image, neighbors_indices)
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


def add_views_to_depth_cleaner(reconstruction, depths, neighbors, dc):
    for neighbor in neighbors:
        shot = reconstruction.shots[neighbor]
        depth = depths[neighbor]
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


def find_neighboring_images(graph, reconstruction, shot, num_neighbors=5):
    """Find neighbouring images based on common tracks."""
    theta_min = np.pi / 60
    theta_max = np.pi / 6
    ns = []
    C1 = shot.pose.get_origin()
    others = (s for s in reconstruction.shots.values() if s.id != shot.id)
    for other in others:
        score = 0
        C2 = other.pose.get_origin()
        tracks, p1s, p2s = matching.common_tracks(graph, shot.id, other.id)
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

    return '\n'.join(header + vertices)


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

    return '\n'.join(header + vertices)


def color_plane_normals(plane):
    l = np.linalg.norm(plane, axis=2)
    normal = plane / l[..., np.newaxis]
    normal[..., 1] *= -1  # Reverse Y because it points down
    normal[..., 2] *= -1  # Reverse Z because standard colormap does so
    return ((normal + 1) * 128).astype(np.uint8)
