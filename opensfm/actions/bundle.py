# pyre-unsafe
import opensfm.reconstruction as orec
from opensfm.dataset_base import DataSetBase
from typing import Optional


def run_dataset(dataset: DataSetBase, input: Optional[str], output: Optional[str]) -> None:
    """Bundle a reconstructions.

    Args:
        input: input reconstruction JSON in the dataset
        output: input reconstruction JSON in the dataset

    """

    reconstructions = dataset.load_reconstruction(input)
    camera_priors = dataset.load_camera_models()
    rig_cameras_priors = dataset.load_rig_cameras()
    tracks_manager = dataset.load_tracks_manager()

    # load the tracks manager and add its observations to the reconstruction
    # go through all the points and add their shots
    for reconstruction in reconstructions:
        reconstruction.add_correspondences_from_tracks_manager(tracks_manager)
        gcp = dataset.load_ground_control_points()
        orec.bundle(
            reconstruction, camera_priors, rig_cameras_priors, gcp, dataset.config
        )
    dataset.save_reconstruction(reconstructions, output)
