"""Tools to align a reconstruction to GPS and GCP data."""

import numpy as np

from opensfm import multiview
from opensfm import transformations as tf


def apply_similarity(reconstruction, s, A, b):
    """Apply a similarity (y = s A x + b) to a reconstruction.

    :param reconstruction: The reconstruction to transform.
    :param s: The scale (a scalar)
    :param A: The rotation matrix (3x3)
    :param b: The translation vector (3)
    """
    # Align points.
    for point in reconstruction.points.values():
        Xp = s * A.dot(point.coordinates) + b
        point.coordinates = Xp.tolist()

    # Align cameras.
    for shot in reconstruction.shots.values():
        R = shot.pose.get_rotation_matrix()
        t = np.array(shot.pose.translation)
        Rp = R.dot(A.T)
        tp = -Rp.dot(b) + s * t
        shot.pose.set_rotation_matrix(Rp)
        shot.pose.translation = list(tp)


def align_reconstruction_naive_similarity(reconstruction):
    if len(reconstruction.shots) < 3:
        return
    # Compute similarity Xp = s A X + b
    X, Xp = [], []
    for shot in reconstruction.shots.values():
        X.append(shot.pose.get_origin())
        Xp.append(shot.metadata.gps_position)
    X = np.array(X)
    Xp = np.array(Xp)
    T = tf.superimposition_matrix(X.T, Xp.T, scale=True)

    A, b = T[:3, :3], T[:3, 3]
    s = np.linalg.det(A)**(1. / 3)
    A /= s
    return s, A, b


def get_horizontal_and_vertical_directions(R, orientation):
    """Get orientation vectors from camera rotation matrix and orientation tag.

    Return a 3D vectors pointing to the positive XYZ directions of the image.
    X points to the right, Y to the bottom, Z to the front.
    """
    # See http://sylvana.net/jpegcrop/exif_orientation.html
    if orientation == 1:
        return R[0, :], R[1, :], R[2, :]
    if orientation == 2:
        return -R[0, :], R[1, :], -R[2, :]
    if orientation == 3:
        return -R[0, :], -R[1, :], R[2, :]
    if orientation == 4:
        return R[0, :], -R[1, :], R[2, :]
    if orientation == 5:
        return R[1, :], R[0, :], -R[2, :]
    if orientation == 6:
        return -R[1, :], R[0, :], R[2, :]
    if orientation == 7:
        return -R[1, :], -R[0, :], -R[2, :]
    if orientation == 8:
        return R[1, :], -R[0, :], R[2, :]
    print 'ERROR unknown orientation {0}. Using 1 instead'.format(orientation)
    return R[0, :], R[1, :], R[2, :]


def align_reconstruction(reconstruction, config):
    s, A, b = align_reconstruction_similarity(reconstruction, config)
    apply_similarity(reconstruction, s, A, b)


def align_reconstruction_similarity(reconstruction, config):
    align_method = config.get('align_method', 'orientation_prior')
    if align_method == 'orientation_prior':
        return align_reconstruction_orientation_prior_similarity(
            reconstruction, config)
    elif align_method == 'naive':
        return align_reconstruction_naive_similarity(reconstruction)


def align_reconstruction_orientation_prior_similarity(reconstruction, config):
    X, Xp = [], []
    orientation_type = config.get('align_orientation_prior', 'horizontal')
    onplane, verticals = [], []
    for shot in reconstruction.shots.values():
        X.append(shot.pose.get_origin())
        Xp.append(shot.metadata.gps_position)
        R = shot.pose.get_rotation_matrix()
        x, y, z = get_horizontal_and_vertical_directions(
            R, shot.metadata.orientation)
        if orientation_type == 'no_roll':
            onplane.append(x)
            verticals.append(-y)
        elif orientation_type == 'horizontal':
            onplane.append(x)
            onplane.append(z)
            verticals.append(-y)
        elif orientation_type == 'vertical':
            onplane.append(x)
            onplane.append(y)
            verticals.append(-z)

    X = np.array(X)
    Xp = np.array(Xp)

    # Estimate ground plane.
    p = multiview.fit_plane(X - X.mean(axis=0), onplane, verticals)
    Rplane = multiview.plane_horizontalling_rotation(p)
    X = Rplane.dot(X.T).T

    # Estimate 2d similarity to align to GPS
    if (len(X) < 2 or
            X.std(axis=0).max() < 1e-8 or     # All points are the same.
            Xp.std(axis=0).max() < 0.01):      # All GPS points are the same.
        # Set the arbitrary scale proportional to the number of cameras.
        s = len(X) / max(1e-8, X.std(axis=0).max())
        A = Rplane
        b = Xp.mean(axis=0) - X.mean(axis=0)
    else:
        T = tf.affine_matrix_from_points(X.T[:2], Xp.T[:2], shear=False)
        s = np.linalg.det(T[:2, :2])**0.5
        A = np.eye(3)
        A[:2, :2] = T[:2, :2] / s
        A = A.dot(Rplane)
        b = np.array([
            T[0, 2],
            T[1, 2],
            Xp[:, 2].mean() - s * X[:, 2].mean()  # vertical alignment
        ])
    return s, A, b
