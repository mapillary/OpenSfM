import logging

import numpy as np
from opensfm import rig, reconstruction as orec, pygeometry, types
from opensfm.dataset import DataSet, DataSetBase


logger = logging.getLogger(__name__)


def run_dataset(data: DataSet, method, definition, output_debug):
    """Given a dataset that contains rigs, construct rig data files.

    Args:
        data: dataset object
        method : `auto` will run `reconstruct` process and try to detect rig pattern (TODO)
                 `camera` will create instances based on the camera model name
                 'pattern` will create instances based on a REGEX pattern (see below)
        definition : JSON dict (one for each RigCamera) with values as :
                    - (.*) for `pattern` method where the part outside
                        of parenthesis defines a RigCamera instance
                    - a camera model ID for the `camera` method
        output_debug : output a debug JSON reconstruction `rig_instances.json` with rig instances
    """
    rig.create_rigs_with_pattern(data, definition)
    if output_debug:
        reconstructions = _reconstruction_from_rigs_and_assignments(data)
        data.save_reconstruction(reconstructions, "rig_instances.json")


def _reconstruction_from_rigs_and_assignments(data: DataSetBase):
    assignments = data.load_rig_assignments()
    rig_cameras = data.load_rig_cameras()

    if not data.reference_lla_exists():
        data.invent_reference_lla()

    base_rotation = np.zeros(3)

    reconstruction = types.Reconstruction()
    reconstruction.cameras = data.load_camera_models()
    for instance in assignments:
        for image, camera_id in instance:
            rig_camera = rig_cameras[camera_id]
            rig_pose = pygeometry.Pose(base_rotation)
            rig_pose.set_origin(orec.get_image_metadata(data, image).gps_position.value)
            d = data.load_exif(image)
            shot = reconstruction.create_shot(image, d["camera"])
            shot.pose = rig_camera.pose.compose(rig_pose)
            shot.metadata = orec.get_image_metadata(data, image)
    return [reconstruction]
