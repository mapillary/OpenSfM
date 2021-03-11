import logging

import cv2
import numpy as np
from opensfm import features
from opensfm import log
from opensfm import pygeometry
from opensfm import pysfm
from opensfm import transformations as tf
from opensfm import types
from opensfm.context import parallel_map
from opensfm.dataset import DataSetBase, UndistortedDataSet

logger = logging.getLogger(__name__)


def undistort_reconstruction(
    tracks_manager, reconstruction, data: DataSetBase, udata: UndistortedDataSet
):
    image_format = data.config["undistorted_image_format"]
    urec = types.Reconstruction()
    urec.points = reconstruction.points
    utracks_manager = pysfm.TracksManager()
    logger.debug("Undistorting the reconstruction")
    undistorted_shots = {}
    for shot in reconstruction.shots.values():
        if shot.camera.projection_type == "perspective":
            camera = perspective_camera_from_perspective(shot.camera)
            urec.add_camera(camera)
            subshots = [
                get_shot_with_different_camera(urec, shot, camera, image_format)
            ]
        elif shot.camera.projection_type == "brown":
            camera = perspective_camera_from_brown(shot.camera)
            urec.add_camera(camera)
            subshots = [
                get_shot_with_different_camera(urec, shot, camera, image_format)
            ]
        elif shot.camera.projection_type in ["fisheye", "fisheye_opencv"]:
            camera = perspective_camera_from_fisheye(shot.camera)
            urec.add_camera(camera)
            subshots = [
                get_shot_with_different_camera(urec, shot, camera, image_format)
            ]
        elif pygeometry.Camera.is_panorama(shot.camera.projection_type):
            subshot_width = int(data.config["depthmap_resolution"])
            subshots = perspective_views_of_a_panorama(
                shot, subshot_width, urec, image_format
            )

        for subshot in subshots:
            if tracks_manager:
                add_subshot_tracks(tracks_manager, utracks_manager, shot, subshot)
        undistorted_shots[shot.id] = subshots

    udata.save_undistorted_reconstruction([urec])
    if tracks_manager:
        udata.save_undistorted_tracks_manager(utracks_manager)

    udata.save_undistorted_shot_ids(
        {
            shot_id: [ushot.id for ushot in ushots]
            for shot_id, ushots in undistorted_shots.items()
        }
    )

    return undistorted_shots


def undistort_reconstruction_and_images(
    tracks_manager, reconstruction, data: DataSetBase, udata: UndistortedDataSet
):
    undistorted_shots = undistort_reconstruction(
        tracks_manager, reconstruction, data, udata
    )

    arguments = []
    for shot in reconstruction.shots.values():
        arguments.append((shot, undistorted_shots[shot.id], data, udata))

    processes = data.config["processes"]
    parallel_map(undistort_image_and_masks, arguments, processes)


def undistort_image_and_masks(arguments):
    shot, undistorted_shots, data, udata = arguments
    log.setup()
    logger.debug("Undistorting image {}".format(shot.id))
    max_size = data.config["undistorted_image_max_size"]

    # Undistort image
    image = data.load_image(shot.id, unchanged=True, anydepth=True)
    if image is not None:
        undistorted = undistort_image(
            shot, undistorted_shots, image, cv2.INTER_AREA, max_size
        )
        for k, v in undistorted.items():
            udata.save_undistorted_image(k, v)

    # Undistort mask
    mask = data.load_mask(shot.id)
    if mask is not None:
        undistorted = undistort_image(
            shot, undistorted_shots, mask, cv2.INTER_NEAREST, max_size
        )
        for k, v in undistorted.items():
            udata.save_undistorted_mask(k, v)

    # Undistort segmentation
    segmentation = data.load_segmentation(shot.id)
    if segmentation is not None:
        undistorted = undistort_image(
            shot, undistorted_shots, segmentation, cv2.INTER_NEAREST, max_size
        )
        for k, v in undistorted.items():
            udata.save_undistorted_segmentation(k, v)

    # Undistort detections
    detection = data.load_detection(shot.id)
    if detection is not None:
        undistorted = undistort_image(
            shot, undistorted_shots, detection, cv2.INTER_NEAREST, max_size
        )
        for k, v in undistorted.items():
            udata.save_undistorted_detection(k, v)


def undistort_image(shot, undistorted_shots, original, interpolation, max_size):
    """Undistort an image into a set of undistorted ones.

    Args:
        shot: the distorted shot
        undistorted_shots: the set of undistorted shots covering the
            distorted shot field of view. That is 1 for most camera
            types and 6 for spherical cameras.
        original: the original distorted image array.
        interpolation: the opencv interpolation flag to use.
        max_size: maximum size of the undistorted image.
    """
    if original is None:
        return

    projection_type = shot.camera.projection_type
    if projection_type in ["perspective", "brown", "fisheye", "fisheye_opencv"]:
        [undistorted_shot] = undistorted_shots
        new_camera = undistorted_shot.camera
        height, width = original.shape[:2]
        map1, map2 = pygeometry.compute_camera_mapping(
            shot.camera, new_camera, width, height
        )
        undistorted = cv2.remap(original, map1, map2, interpolation)
        return {undistorted_shot.id: scale_image(undistorted, max_size)}
    elif pygeometry.Camera.is_panorama(projection_type):
        subshot_width = undistorted_shots[0].camera.width
        width = 4 * subshot_width
        height = width // 2
        image = cv2.resize(original, (width, height), interpolation=interpolation)
        mint = cv2.INTER_LINEAR if interpolation == cv2.INTER_AREA else interpolation
        res = {}
        for undistorted_shot in undistorted_shots:
            undistorted = render_perspective_view_of_a_panorama(
                image, shot, undistorted_shot, mint
            )
            res[undistorted_shot.id] = scale_image(undistorted, max_size)
        return res
    else:
        raise NotImplementedError(
            "Undistort not implemented for projection type: {}".format(
                shot.camera.projection_type
            )
        )


def scale_image(image, max_size):
    """Scale an image not to exceed max_size."""
    height, width = image.shape[:2]
    factor = max_size / float(max(height, width))
    if factor >= 1:
        return image
    width = int(round(width * factor))
    height = int(round(height * factor))
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_NEAREST)


def add_image_format_extension(shot_id, image_format):
    if shot_id.endswith(f".{image_format}"):
        return shot_id
    else:
        return f"{shot_id}.{image_format}"


def get_shot_with_different_camera(urec, shot, camera, image_format):
    new_shot_id = add_image_format_extension(shot.id, image_format)
    new_shot = urec.create_shot(new_shot_id, shot.camera.id, shot.pose)
    new_shot.metadata = shot.metadata
    return new_shot


def perspective_camera_from_perspective(distorted):
    """Create an undistorted camera from a distorted."""
    camera = pygeometry.Camera.create_perspective(distorted.focal, 0.0, 0.0)
    camera.id = distorted.id
    camera.width = distorted.width
    camera.height = distorted.height
    return camera


def perspective_camera_from_brown(brown):
    """Create a perspective camera from a Brown camera."""
    camera = pygeometry.Camera.create_perspective(
        brown.focal * (1 + brown.aspect_ratio) / 2.0, 0.0, 0.0
    )
    camera.id = brown.id
    camera.width = brown.width
    camera.height = brown.height
    return camera


def perspective_camera_from_fisheye(fisheye):
    """Create a perspective camera from a fisheye."""
    camera = pygeometry.Camera.create_perspective(fisheye.focal, 0.0, 0.0)
    camera.id = fisheye.id
    camera.width = fisheye.width
    camera.height = fisheye.height
    return camera


def perspective_camera_from_fisheye_opencv(fisheye_opencv):
    """Create a perspective camera from a fisheye extended."""
    camera = pygeometry.Camera.create_perspective(
        fisheye_opencv.focal * (1 + fisheye_opencv.aspect_ratio) / 2.0, 0.0, 0.0
    )
    camera.id = fisheye_opencv.id
    camera.width = fisheye_opencv.width
    camera.height = fisheye_opencv.height
    return camera


def perspective_camera_from_fisheye62(fisheye62):
    """Create a perspective camera from a fisheye extended."""
    camera = pygeometry.Camera.create_perspective(
        fisheye62.focal * (1 + fisheye62.aspect_ratio) / 2.0, 0.0, 0.0
    )
    camera.id = fisheye62.id
    camera.width = fisheye62.width
    camera.height = fisheye62.height
    return camera


def perspective_views_of_a_panorama(
    spherical_shot, width, reconstruction, image_format
):
    """Create 6 perspective views of a panorama."""
    camera = pygeometry.Camera.create_perspective(0.5, 0.0, 0.0)
    camera.id = "perspective_panorama_camera"
    camera.width = width
    camera.height = width
    reconstruction.add_camera(camera)

    names = ["front", "left", "back", "right", "top", "bottom"]
    rotations = [
        tf.rotation_matrix(-0 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-1 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-2 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-3 * np.pi / 2, (0, 1, 0)),
        tf.rotation_matrix(-np.pi / 2, (1, 0, 0)),
        tf.rotation_matrix(+np.pi / 2, (1, 0, 0)),
    ]
    shots = []
    for name, rotation in zip(names, rotations):
        R = np.dot(rotation[:3, :3], spherical_shot.pose.get_rotation_matrix())
        o = spherical_shot.pose.get_origin()
        pose = pygeometry.Pose()
        pose.set_rotation_matrix(R)
        pose.set_origin(o)
        shot_id = add_image_format_extension(
            f"{spherical_shot.id}_perspective_view_{name}", image_format
        )
        shots.append(
            reconstruction.create_shot(
                shot_id,
                camera.id,
                pose,
            )
        )
    return shots


def render_perspective_view_of_a_panorama(
    image,
    panoshot,
    perspectiveshot,
    interpolation=cv2.INTER_LINEAR,
    borderMode=cv2.BORDER_WRAP,
):
    """Render a perspective view of a panorama."""
    # Get destination pixel coordinates
    dst_shape = (perspectiveshot.camera.height, perspectiveshot.camera.width)
    dst_y, dst_x = np.indices(dst_shape).astype(np.float32)
    dst_pixels_denormalized = np.column_stack([dst_x.ravel(), dst_y.ravel()])

    dst_pixels = features.normalized_image_coordinates(
        dst_pixels_denormalized,
        perspectiveshot.camera.width,
        perspectiveshot.camera.height,
    )

    # Convert to bearing
    dst_bearings = perspectiveshot.camera.pixel_bearing_many(dst_pixels)

    # Rotate to panorama reference frame
    rotation = np.dot(
        panoshot.pose.get_rotation_matrix(),
        perspectiveshot.pose.get_rotation_matrix().T,
    )
    rotated_bearings = np.dot(dst_bearings, rotation.T)

    # Project to panorama pixels
    src_pixels = panoshot.camera.project_many(rotated_bearings)
    src_pixels_denormalized = features.denormalized_image_coordinates(
        src_pixels, image.shape[1], image.shape[0]
    )

    src_pixels_denormalized.shape = dst_shape + (2,)

    # Sample color
    x = src_pixels_denormalized[..., 0].astype(np.float32)
    y = src_pixels_denormalized[..., 1].astype(np.float32)
    colors = cv2.remap(image, x, y, interpolation, borderMode=borderMode)

    return colors


def add_subshot_tracks(tracks_manager, utracks_manager, shot, subshot):
    """Add shot tracks to the undistorted tracks_manager."""
    if shot.id not in tracks_manager.get_shot_ids():
        return

    if pygeometry.Camera.is_panorama(shot.camera.projection_type):
        add_pano_subshot_tracks(tracks_manager, utracks_manager, shot, subshot)
    else:
        for track_id, obs in tracks_manager.get_shot_observations(shot.id).items():
            utracks_manager.add_observation(subshot.id, track_id, obs)


def add_pano_subshot_tracks(tracks_manager, utracks_manager, panoshot, perspectiveshot):
    """Add edges between subshots and visible tracks."""
    for track_id, obs in tracks_manager.get_shot_observations(panoshot.id).items():
        bearing = panoshot.camera.pixel_bearing(obs.point)
        rotation = np.dot(
            perspectiveshot.pose.get_rotation_matrix(),
            panoshot.pose.get_rotation_matrix().T,
        )

        rotated_bearing = np.dot(bearing, rotation.T)
        if rotated_bearing[2] <= 0:
            continue

        perspective_feature = perspectiveshot.camera.project(rotated_bearing)
        if (
            perspective_feature[0] < -0.5
            or perspective_feature[0] > 0.5
            or perspective_feature[1] < -0.5
            or perspective_feature[1] > 0.5
        ):
            continue

        obs.point = perspective_feature
        utracks_manager.add_observation(perspectiveshot.id, track_id, obs)
