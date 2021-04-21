import opensfm.reconstruction as orec
from opensfm.dataset import DataSetBase


def run_dataset(dataset: DataSetBase, input, output):
    """Bundle a reconstructions.

    Args:
        input: input reconstruction JSON in the dataset
        output: input reconstruction JSON in the dataset

    """

    reconstructions = dataset.load_reconstruction(input)
    camera_priors = dataset.load_camera_models()
    rig_cameras_priors = dataset.load_rig_cameras()
    gcp = dataset.load_ground_control_points()
    tracks_manager = dataset.load_tracks_manager()

    # load the tracks manager and add its observations to the reconstruction
    # go through all the points and add their shots
    for reconstruction in reconstructions:
        reconstruction.add_correspondences_from_tracks_manager(tracks_manager)
        orec.bundle(
            reconstruction, camera_priors, rig_cameras_priors, gcp, dataset.config
        )
    dataset.save_reconstruction(reconstructions, output)
