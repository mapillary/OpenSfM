
import cv2
import numpy as np

from opensfm import csfm


def compute_depthmaps(data, graph, reconstruction):
    """Compute and refine depthmaps for all shots."""
    depths = {}
    planes = {}
    scores = {}
    for shot in reconstruction.shots.values():
        depth, plane, score = compute_depthmap(data, graph, reconstruction, shot)
        depths[shot.id] = depth
        planes[shot.id] = plane
        scores[shot.id] = score

    clean_depths = {}
    for shot in reconstruction.shots.values():
        clean_depths[shot.id] = clean_depthmap(
            data, graph, reconstruction, shot, depths, planes, scores)

    merge_depthmaps(data, reconstruction, clean_depths, planes)


def compute_depthmap(data, graph, reconstruction, shot):
    """Compute depthmap for a single shot."""
    if data.raw_depthmap_exists(shot.id):
        return data.load_raw_depthmap(shot.id)

    neighbors = find_neighboring_images(shot, reconstruction)
    min_depth, max_depth = compute_depth_range(graph, reconstruction, shot)

    de = csfm.DepthmapEstimator()
    add_views_to_depth_estimator(data, reconstruction, neighbors, de)
    de.set_depth_range(min_depth, max_depth, 100)
    # depth, plane, score = de.compute_brute_force()
    depth, plane, score = de.compute_patch_match()
    depth = depth * (depth < max_depth) * (score > 0.5)

    # Save and display results
    data.save_raw_depthmap(shot.id, depth, plane, score)
    image = data.undistorted_image_as_array(shot.id)
    image = scale_down_image(image, depth.shape[1], depth.shape[0])
    ply = depthmap_to_ply(shot, depth, image)
    with open(data._depthmap_file(shot.id, 'raw.npz.ply'), 'w') as fout:
        fout.write(ply)

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


def clean_depthmap(data, graph, reconstruction, shot, depths, planes, scores):
    if data.clean_depthmap_exists(shot.id):
        return data.load_clean_depthmap(shot.id)[0]

    neighbors = find_neighboring_images(shot, reconstruction, num_neighbors=5)
    dc = csfm.DepthmapCleaner()
    add_views_to_depth_cleaner(reconstruction, depths, neighbors, dc)
    depth = dc.clean()

    # Save and display results
    data.save_clean_depthmap(shot.id, depth, planes[shot.id], scores[shot.id])
    image = data.undistorted_image_as_array(shot.id)
    image = scale_down_image(image, depth.shape[1], depth.shape[0])
    ply = depthmap_to_ply(shot, depth, image)
    with open(data._depthmap_file(shot.id, 'clean.npz.ply'), 'w') as fout:
        fout.write(ply)

    import matplotlib.pyplot as plt
    plt.subplot(2, 2, 1)
    plt.imshow(depths[shot.id])
    plt.colorbar()
    plt.subplot(2, 2, 2)
    plt.imshow(depth)
    plt.colorbar()
    plt.show(block=True)
    return depth


def merge_depthmaps(data, reconstruction, clean_depths, planes):
    dm = csfm.DepthmapMerger()
    shot_ids = clean_depths.keys()
    indices = {k: i for i, k in enumerate(shot_ids)}
    for shot_id in shot_ids:
        depth = clean_depths[shot_id]
        shot = reconstruction.shots[shot_id]
        neighbors = find_neighboring_images(shot, reconstruction, num_neighbors=5)
        neighbors_indices = [indices[n] for n in neighbors]
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
    for neighbor in neighbors:
        shot = reconstruction.shots[neighbor]
        assert shot.camera.projection_type == 'perspective'
        color_image = data.undistorted_image_as_array(shot.id)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        original_height, original_width = gray_image.shape
        width = 640
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


def find_neighboring_images(shot, reconstruction, num_neighbors=5):
    """Find closest images."""
    others = reconstruction.shots.values()
    distances = [distance_between_shots(shot, other) for other in others]
    pairs = sorted(zip(distances, others))
    return [s.id for d, s in pairs[:num_neighbors]]


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
    from opensfm import features
    vertices = []
    height, width = depth.shape
    for i in range(height):
        for j in range(width):
            pixel = features.normalized_image_coordinates(
                np.array([[j, i]]), width, height)[0]
            p = shot.back_project(pixel, depth[i, j])
            c = image[i, j]
            s = "{} {} {} {} {} {}".format(
                p[0], p[1], p[2], c[0], c[1], c[2])
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
        s = "{} {} {} {} {} {} {} {} {}".format(
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
