import time

import opensfm.reconstruction as orec


def run_dataset(dataset, input, output):
    """ Bundle a reconstructions.

    Args:
        input: input reconstruction JSON in the dataset
        output: input reconstruction JSON in the dataset

    """

    start = time.time()
    reconstructions = dataset.load_reconstruction(input)
    camera_priors = dataset.load_camera_models()
    gcp = dataset.load_ground_control_points()

    for reconstruction in reconstructions:
        orec.bundle(reconstruction, camera_priors, gcp, dataset.config)

    end = time.time()
    with open(dataset.profile_log(), 'a') as fout:
        fout.write('bundle: {0}\n'.format(end - start))
    dataset.save_reconstruction(reconstructions, output)
