
import cv2
import numpy as np

from opensfm import csfm


def compute_depthmap(data, graph, reconstruction, shot_id):
    """Compute depthmap for a single shot."""
    shot = reconstruction.shots[shot_id]
    neighbors = find_neighboring_images(shot, reconstruction)
    min_depth, max_depth = compute_depth_range(graph, reconstruction, shot)

    de = csfm.DepthmapEstimator()
    add_views_to_depth_estimator(data, reconstruction, neighbors, de)
    de.set_depth_range(min_depth, max_depth, 100)
    depth, score = de.compute_patch_match()

    # Save and display results
    data.save_depthmap(shot_id, depth)
    image = data.image_as_array(shot.id)
    image = cv2.resize(image, (depth.shape[1], depth.shape[0]))
    ply = depthmap_to_ply(shot, depth, image)
    with open(data._depthmap_ply_file(shot_id), 'w') as fout:
        fout.write(ply)

    import matplotlib.pyplot as plt
    plt.subplot(1, 3, 1)
    plt.imshow(data.image_as_array(shot_id))
    plt.colorbar()
    plt.subplot(1, 3, 2)
    plt.imshow(depth)
    plt.colorbar()
    plt.subplot(1, 3, 3)
    plt.imshow(score)
    plt.colorbar()
    plt.show()


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
        image = cv2.resize(gray_image, (width, height))
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        de.add_view(K, R, t, image)


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
