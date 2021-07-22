"""Tools to align a reconstruction to GPS and GCP data."""

import logging
import math
from collections import defaultdict

import cv2
import numpy as np
from opensfm import multiview, transformations as tf, pygeometry, types


logger = logging.getLogger(__name__)


def align_reconstruction(
    reconstruction, gcp, config, use_gps=True, bias_override=False
):
    """Align a reconstruction with GPS and GCP data."""
    use_scale = len(reconstruction.rig_instances) < 1
    if bias_override and config["bundle_compensate_gps_bias"]:
        return set_gps_bias(reconstruction, config, gcp, use_scale)
    else:
        res = compute_reconstruction_similarity(
            reconstruction, gcp, config, use_gps, use_scale
        )
        if res:
            s, A, b = res
            apply_similarity(reconstruction, s, A, b)
        return res


def apply_similarity_pose(pose, s, A, b):
    """Apply a similarity (y = s A x + b) to an object having a 'pose' member."""
    R = pose.get_rotation_matrix()
    t = np.array(pose.translation)
    Rp = R.dot(A.T)
    tp = -Rp.dot(b) + s * t
    pose.set_rotation_matrix(Rp)
    pose.translation = list(tp)


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
        if shot.is_in_rig():
            continue
        apply_similarity_pose(shot.pose, s, A, b)

    # Align rig instances
    for rig_instance in reconstruction.rig_instances.values():
        apply_similarity_pose(rig_instance.pose, s, A, b)


def compute_reconstruction_similarity(reconstruction, gcp, config, use_gps, use_scale):
    """Compute similarity so that the reconstruction is aligned with GPS and GCP data.

    Config parameter `align_method` can be used to choose the alignment method.
    Accepted values are
     - navie: does a direct 3D-3D fit
     - orientation_prior: assumes a particular camera orientation
    """
    align_method = config["align_method"]
    if align_method == "auto":
        align_method = detect_alignment_constraints(
            config,
            reconstruction,
            gcp,
        )
    if align_method == "orientation_prior":
        res = compute_orientation_prior_similarity(
            reconstruction, config, gcp, use_gps, use_scale
        )
    elif align_method == "naive":
        res = compute_naive_similarity(config, reconstruction, gcp, use_gps, use_scale)

    if not res:
        return None

    s, A, b = res
    if (s == 0) or np.isnan(A).any() or np.isnan(b).any():
        logger.warning(
            "Computation of alignment similarity (%s) is degenerate." % align_method
        )
        return None
    return res


def alignment_constraints(config, reconstruction, gcp, use_gps):
    """Gather alignment constraints to be used by checking bundle_use_gcp and bundle_use_gps."""

    X, Xp = [], []

    # Get Ground Control Point correspondences
    if gcp and config["bundle_use_gcp"]:
        triangulated, measured = triangulate_all_gcp(reconstruction, gcp)
        X.extend(triangulated)
        Xp.extend(measured)

    # Get camera center correspondences
    if use_gps and config["bundle_use_gps"]:
        for shot in reconstruction.shots.values():
            if shot.metadata.gps_position.has_value:
                X.append(shot.pose.get_origin())
                Xp.append(shot.metadata.gps_position.value)

    return X, Xp


def detect_alignment_constraints(config, reconstruction, gcp, use_gps):
    """Automatically pick the best alignment method, depending
    if alignment data such as GPS/GCP is aligned on a single-line or not.

    """

    X, Xp = alignment_constraints(config, reconstruction, gcp, use_gps)
    if len(X) < 3:
        return "orientation_prior"

    X = np.array(X)
    X = X - np.average(X, axis=0)
    evalues, _ = np.linalg.eig(X.T.dot(X))

    evalues = np.array(sorted(evalues))
    ratio_1st_2nd = math.fabs(evalues[2] / evalues[1])

    epsilon_abs = 1e-10
    epsilon_ratio = 5e3
    is_line = sum(evalues < epsilon_abs) > 1 or ratio_1st_2nd > epsilon_ratio
    if is_line:
        logger.warning(
            "Shots and/or GCPs are aligned on a single-line. Using %s prior",
            config["align_orientation_prior"],
        )
        return "orientation_prior"
    else:
        logger.info(
            "Shots and/or GCPs are well-conditioned. Using naive 3D-3D alignment."
        )
        return "naive"


def compute_naive_similarity(config, reconstruction, gcp, use_gps, use_scale):
    """Compute similarity with GPS and GCP data using direct 3D-3D matches."""
    X, Xp = alignment_constraints(config, reconstruction, gcp, use_gps)

    if len(X) == 0:
        return None

    # Translation-only case, either :
    #  - a single value
    #  - identical values
    same_values = np.linalg.norm(np.std(Xp, axis=0)) < 1e-10
    single_value = len(X) == 1
    if single_value:
        logger.warning("Only 1 constraints. Using translation-only alignment.")
    if same_values:
        logger.warning(
            "GPS/GCP data seems to have identical values. Using translation-only alignment."
        )
    if same_values or single_value:
        t = np.array(Xp[0]) - np.array(X[0])
        return 1.0, np.identity(3), t

    # Will be up to some unknown rotation
    if len(X) == 2:
        logger.warning("Only 2 constraints. Will be up to some unknown rotation.")
        X.append(X[1])
        Xp.append(Xp[1])

    # Compute similarity Xp = s A X + b
    X = np.array(X)
    Xp = np.array(Xp)
    T = tf.superimposition_matrix(X.T, Xp.T, scale=use_scale)

    A, b = T[:3, :3], T[:3, 3]
    s = np.linalg.det(A) ** (1.0 / 3)
    A /= s
    return s, A, b


def compute_orientation_prior_similarity(
    reconstruction, config, gcp, use_gps, use_scale
):
    """Compute similarity with GPS data assuming particular a camera orientation.

    In some cases, using 3D-3D matches directly fails to find proper
    orientation of the world.  That happends mainly when all cameras lie
    close to a straigh line.

    In such cases, we can impose a particular orientation of the cameras
    to improve the orientation of the alignment.  The config parameter
    `align_orientation_prior` can be used to specify such orientation.
    Accepted values are:
     - no_roll: assumes horizon is horizontal on the images
     - horizontal: assumes cameras are looking towards the horizon
     - vertical: assumes cameras are looking down towards the ground
    """
    X, Xp = alignment_constraints(config, reconstruction, gcp, use_gps)
    X = np.array(X)
    Xp = np.array(Xp)

    if len(X) < 1:
        return None

    p = estimate_ground_plane(reconstruction, config)
    Rplane = multiview.plane_horizontalling_rotation(p)
    X = Rplane.dot(X.T).T

    # Estimate 2d similarity to align to GPS
    two_shots = len(X) == 2
    single_shot = len(X) < 2
    same_shots = (
        X.std(axis=0).max() < 1e-8
        or Xp.std(axis=0).max() < 0.01  # All points are the same.
    )  # All GPS points are the same.
    if single_shot or same_shots:
        s = 1.0
        A = Rplane
        b = Xp.mean(axis=0) - X.mean(axis=0)

        # Clamp shots pair scale to 1km, so the
        # optimizer can still catch-up acceptable error
        max_scale = 1000
        current_scale = np.linalg.norm(b)
        if two_shots and current_scale > max_scale:
            b = max_scale * b / current_scale
            s = max_scale / current_scale
    else:
        T = tf.affine_matrix_from_points(
            X.T[:2], Xp.T[:2], shear=False, scale=use_scale
        )
        s = np.linalg.det(T[:2, :2]) ** 0.5
        A = np.eye(3)
        A[:2, :2] = T[:2, :2] / s
        A = A.dot(Rplane)
        b = np.array(
            [
                T[0, 2],
                T[1, 2],
                Xp[:, 2].mean() - s * X[:, 2].mean(),  # vertical alignment
            ]
        )
    return s, A, b


def set_gps_bias(reconstruction, config, gcp, use_scale):
    """Compute and set the bias transform of the GPS coordinate system wrt. to the GCP one."""

    # Compute similarity ('gps_bias') that brings the reconstruction on the GCPs ONLY
    gps_bias = compute_reconstruction_similarity(
        reconstruction, gcp, config, False, use_scale
    )
    if not gps_bias:
        logger.warning("Cannot align on GCPs only, GPS bias won't be compensated.")
        return

    # Align the reconstruction on GCPs ONLY
    s, A, b = gps_bias
    A_angle_axis = cv2.Rodrigues(A)[0].flatten()
    logger.info(
        f"Applying global bias with scale {s:.5f} / translation {b} / rotation {A_angle_axis}"
    )
    apply_similarity(reconstruction, s, A, b)

    # Compute per camera similarity between the GCP and the shots positions
    per_camera_shots = defaultdict(list)
    for s in reconstruction.shots.values():
        per_camera_shots[s.camera.id].append(s.id)

    per_camera_transform = {}
    for camera_id, shots_id in per_camera_shots.items():

        # As we re-use 'compute_reconstruction_similarity', we need to construct a 'Reconstruction'
        subrec = types.Reconstruction()
        subrec.add_camera(reconstruction.cameras[camera_id])
        for shot_id in shots_id:
            subrec.add_shot(reconstruction.shots[shot_id])
        per_camera_transform[camera_id] = compute_reconstruction_similarity(
            subrec, {}, config, True, use_scale
        )

    if any([True for x in per_camera_transform.values() if not x]):
        logger.warning("Cannot compensate some shots, GPS bias won't be compensated.")
    else:
        for camera_id, transform in per_camera_transform.items():
            s, A, b = transform
            A_angle_axis = cv2.Rodrigues(A)[0].flatten()
            s, A_angle_axis, b = 1.0 / s, -A_angle_axis, -A.T.dot(b) / s
            logger.info(
                f"Camera {camera_id} bias : scale {s:.5f} / translation {b} / rotation {A_angle_axis}"
            )
            camera_bias = pygeometry.Similarity(A_angle_axis, b, s)
            reconstruction.set_bias(camera_id, camera_bias)

    return gps_bias


def estimate_ground_plane(reconstruction, config):
    """Estimate ground plane orientation.

    It assumes cameras are all at a similar height and uses the
    align_orientation_prior option to enforce cameras to look
    horizontally or vertically.
    """
    orientation_type = config["align_orientation_prior"]
    onplane, verticals = [], []
    for shot in reconstruction.shots.values():
        R = shot.pose.get_rotation_matrix()
        x, y, z = get_horizontal_and_vertical_directions(
            R, shot.metadata.orientation.value
        )
        if orientation_type == "no_roll":
            onplane.append(x)
            verticals.append(-y)
        elif orientation_type == "horizontal":
            onplane.append(x)
            onplane.append(z)
            verticals.append(-y)
        elif orientation_type == "vertical":
            onplane.append(x)
            onplane.append(y)
            verticals.append(-z)

    ground_points = []
    for shot in reconstruction.shots.values():
        ground_points.append(shot.pose.get_origin())
    ground_points = np.array(ground_points)
    ground_points -= ground_points.mean(axis=0)

    plane = multiview.fit_plane(ground_points, onplane, verticals)
    return plane


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
    logger.error("unknown orientation {0}. Using 1 instead".format(orientation))
    return R[0, :], R[1, :], R[2, :]


def triangulate_all_gcp(reconstruction, gcp):
    """Group and triangulate Ground Control Points seen in 2+ images."""
    triangulated, measured = [], []
    for point in gcp:
        x = multiview.triangulate_gcp(
            point,
            reconstruction.shots,
            reproj_threshold=0.004,
            min_ray_angle_degrees=2.0,
        )
        if x is not None:
            triangulated.append(x)
            measured.append(point.coordinates.value)
    return triangulated, measured
