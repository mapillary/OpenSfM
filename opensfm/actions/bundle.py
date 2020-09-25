import opensfm.reconstruction as orec


def run_dataset(dataset, input, output):
    """ Bundle a reconstructions.

    Args:
        input: input reconstruction JSON in the dataset
        output: input reconstruction JSON in the dataset

    """

    reconstructions = dataset.load_reconstruction(input)
    camera_priors = dataset.load_camera_models()
    gcp = dataset.load_ground_control_points()

    for reconstruction in reconstructions:
        orec.bundle(reconstruction, camera_priors, gcp, dataset.config)
    dataset.save_reconstruction(reconstructions, output)
