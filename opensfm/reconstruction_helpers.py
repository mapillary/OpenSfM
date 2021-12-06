import logging
import math
from typing import Dict, Any, Iterable

import numpy as np
from opensfm import (
    multiview,
    pygeometry,
    pymap,
    geometry,
    types,
    exif as oexif,
    rig,
)
from opensfm.dataset_base import DataSetBase


logger = logging.getLogger(__name__)


def guess_acceleration_from_orientation_tag(orientation):
    """Guess upward vector in camera coordinates given the orientation tag.

    Assumes camera is looking towards the horizon and horizon is horizontal
    on the image when taking in to account the orientation tag.
    """
    # See http://sylvana.net/jpegcrop/exif_orientation.html
    if orientation == 1:
        return [0, -1, 0]
    if orientation == 2:
        return [0, -1, 0]
    if orientation == 3:
        return [0, 1, 0]
    if orientation == 4:
        return [0, 1, 0]
    if orientation == 5:
        return [-1, 0, 0]
    if orientation == 6:
        return [-1, 0, 0]
    if orientation == 7:
        return [1, 0, 0]
    if orientation == 8:
        return [1, 0, 0]
    logger.error("Unknown orientation tag: {}".format(orientation))


def orientation_from_acceleration_in_image_axis(x, y, z):
    """Return the orientation tag corresponding to an acceleration"""
    if y <= -(np.fabs(x)):
        return 1
    elif y >= np.fabs(x):
        return 3
    elif x <= -(np.fabs(y)):
        return 6
    elif x >= np.fabs(y):
        return 8


def transform_acceleration_from_phone_to_image_axis(x, y, z, orientation):
    """Compute acceleration in image axis.

    Orientation tag is used to ensure that the resulting acceleration points
    downwards in the image.  This validation is not needed if the orientation
    tag is the one from the original picture.  It is only required when
    the image has been rotated with respect to the original and the orientation
    tag modified accordingly.
    """
    assert orientation in [1, 3, 6, 8]

    # Orientation in image axis assuming image has not been transformed
    length = np.sqrt(x * x + y * y + z * z)
    if length < 3:  # Assume IOS device since gravity is 1 in there
        ix, iy, iz = y, -x, z
    else:  # Assume Android device since gravity is 9.8 in there
        ix, iy, iz = -y, -x, -z

    for _ in range(4):
        if orientation == orientation_from_acceleration_in_image_axis(ix, iy, iz):
            break
        else:
            ix, iy = -iy, ix

    return [ix, iy, iz]


def shot_acceleration_in_image_axis(shot):
    """Get or guess shot's acceleration."""
    orientation = shot.metadata.orientation.value
    if not 1 <= orientation <= 8:
        logger.error(
            "Unknown orientation tag {} for image {}".format(orientation, shot.id)
        )
        orientation = 1

    if shot.metadata.accelerometer.has_value:
        x, y, z = shot.metadata.accelerometer.value
        if x != 0 or y != 0 or z != 0:
            return transform_acceleration_from_phone_to_image_axis(x, y, z, orientation)
    return guess_acceleration_from_orientation_tag(orientation)


def rotation_from_shot_metadata(shot):
    rotation = rotation_from_angles(shot)
    if rotation is None:
        rotation = rotation_from_orientation_compass(shot)
    return rotation


def rotation_from_orientation_compass(shot):
    up_vector = shot_acceleration_in_image_axis(shot)
    if shot.metadata.compass_angle.has_value:
        angle = shot.metadata.compass_angle.value
    else:
        angle = 0.0
    return multiview.rotation_matrix_from_up_vector_and_compass(up_vector, angle)


def rotation_from_angles(shot):
    if not shot.metadata.opk_angles.has_value:
        return None
    opk_degrees = shot.metadata.opk_angles.value
    opk_rad = map(math.radians, opk_degrees)
    return geometry.rotation_from_opk(*opk_rad)


def reconstruction_from_metadata(data: DataSetBase, images: Iterable[str]):
    """Initialize a reconstruction by using EXIF data for constructing shot poses and cameras."""
    data.init_reference()
    rig_assignments = rig.rig_assignments_per_image(data.load_rig_assignments())

    reconstruction = types.Reconstruction()
    reconstruction.reference = data.load_reference()
    reconstruction.cameras = data.load_camera_models()
    for image in images:
        camera_id = data.load_exif(image)["camera"]

        if image in rig_assignments:
            rig_instance_id, rig_camera_id, _ = rig_assignments[image]
        else:
            rig_instance_id = image
            rig_camera_id = camera_id

        reconstruction.add_rig_camera(pymap.RigCamera(pygeometry.Pose(), rig_camera_id))
        reconstruction.add_rig_instance(pymap.RigInstance(rig_instance_id))
        shot = reconstruction.create_shot(
            shot_id=image,
            camera_id=camera_id,
            rig_camera_id=rig_camera_id,
            rig_instance_id=rig_instance_id,
        )

        shot.metadata = get_image_metadata(data, image)

        if not shot.metadata.gps_position.has_value:
            reconstruction.remove_shot(image)
            continue
        gps_pos = shot.metadata.gps_position.value

        shot.pose.set_rotation_matrix(rotation_from_shot_metadata(shot))
        shot.pose.set_origin(gps_pos)
        shot.scale = 1.0
    return reconstruction


def exif_to_metadata(
    exif: Dict[str, Any], use_altitude: bool, reference: types.TopocentricConverter
) -> pymap.ShotMeasurements:
    """Construct a metadata object from raw EXIF tags (as a dict)."""
    metadata = pymap.ShotMeasurements()

    gps = exif.get("gps")
    if gps and "latitude" in gps and "longitude" in gps:
        lat, lon = gps["latitude"], gps["longitude"]
        if use_altitude:
            alt = min([oexif.maximum_altitude, gps.get("altitude", 2.0)])
        else:
            alt = 2.0  # Arbitrary value used to align the reconstruction
        x, y, z = reference.to_topocentric(lat, lon, alt)
        metadata.gps_position.value = np.array([x, y, z])
        metadata.gps_accuracy.value = gps.get("dop", 15.0)
        if metadata.gps_accuracy.value == 0.0:
            metadata.gps_accuracy.value = 15.0
    else:
        metadata.gps_position.value = np.array([0.0, 0.0, 0.0])
        metadata.gps_accuracy.value = 999999.0

    opk = exif.get("opk")
    if opk and "omega" in opk and "phi" in opk and "kappa" in opk:
        omega, phi, kappa = opk["omega"], opk["phi"], opk["kappa"]
        metadata.opk_angles.value = np.array([omega, phi, kappa])
        metadata.opk_accuracy.value = opk.get("accuracy", 1.0)

    metadata.orientation.value = exif.get("orientation", 1)

    if "accelerometer" in exif:
        metadata.accelerometer.value = exif["accelerometer"]

    if "compass" in exif:
        metadata.compass_angle.value = exif["compass"]["angle"]
        if "accuracy" in exif["compass"]:
            metadata.compass_accuracy.value = exif["compass"]["accuracy"]

    if "capture_time" in exif:
        metadata.capture_time.value = exif["capture_time"]

    if "skey" in exif:
        metadata.sequence_key.value = exif["skey"]

    return metadata


def get_image_metadata(data: DataSetBase, image: str) -> pymap.ShotMeasurements:
    """Get image metadata as a ShotMetadata object."""
    exif = data.load_exif(image)
    reference = data.load_reference()
    return exif_to_metadata(exif, data.config["use_altitude_tag"], reference)
