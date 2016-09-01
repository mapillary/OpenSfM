import cv2
import numpy as np

from opensfm import csfm


def compute_depthmap(data, graph, reconstruction, shot_id):
    neighbors = find_neighboring_images(shot_id, reconstruction)
    print shot_id, neighbors
    de = csfm.DepthmapEstimator()
    images = {}
    for sid in neighbors:
        shot = reconstruction.shots[sid]
        assert shot.camera.projection_type == 'perspective'
        color_image = data.undistorted_image_as_array(shot_id)
        gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
        original_height, original_width = gray_image.shape
        width = 160
        height = width * original_height / original_width
        images[sid] = cv2.resize(gray_image, (width, height))
        K = shot.camera.get_K_in_pixel_coordinates(width, height)
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        de.add_view(K, R, t, images[sid])
    shot = reconstruction.shots[shot_id]
    min_depth, max_depth = compute_depth_range(graph, reconstruction, shot)
    de.set_depth_range(min_depth, max_depth, 100)
    depth, score = de.compute()

    import matplotlib.pyplot as plt
    plt.subplot(1, 3, 1)
    plt.imshow(images[shot_id])
    plt.colorbar()
    plt.subplot(1, 3, 2)
    plt.imshow(depth)
    plt.colorbar()
    plt.subplot(1, 3, 3)
    plt.imshow(score)
    plt.colorbar()
    plt.show()


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
    return min_depth, max_depth


def find_neighboring_images(shot_id, reconstruction, num_neighbors=5):
    """Find closest images."""
    shot = reconstruction.shots[shot_id]
    others = reconstruction.shots.values()
    distances = [distance_between_shots(shot, other) for other in others]
    pairs = sorted(zip(distances, others))
    return [s.id for d, s in pairs[:num_neighbors]]


def distance_between_shots(shot, other):
    o1 = shot.pose.get_origin()
    o2 = other.pose.get_origin()
    l = o2 - o1
    return np.sqrt(np.sum(l**2))
