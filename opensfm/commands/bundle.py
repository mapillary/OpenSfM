import logging
import time

import opensfm.reconstruction as orec
from . import command


logger = logging.getLogger(__name__)


class Command(command.CommandBase):
    def __init__(self):
        super(Command, self).__init__()
        self.name = "bundle"
        self.help = "Bundle a reconstruction"

        self.args["--input"] = {
            "help": "File name of the reconstruction to bundle",
        }
        self.args["--output"] = {
            "help": "File name where to store the bundled reconstruction'",
        }

    def run_dataset(self, options, dataset):
        start = time.time()
        reconstructions = dataset.load_reconstruction(options.input)
        camera_priors = dataset.load_camera_models()
        gcp = dataset.load_ground_control_points()

        for reconstruction in reconstructions:
            orec.bundle(reconstruction, camera_priors, gcp, dataset.config)

        end = time.time()
        with open(dataset.profile_log(), 'a') as fout:
            fout.write('bundle: {0}\n'.format(end - start))
        dataset.save_reconstruction(reconstructions, options.output)
