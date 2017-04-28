import itertools
import numpy as np
import cv2

import create_submodels as metadataset
from opensfm import align
from opensfm import dataset
from opensfm import csfm
from opensfm import types


class Command:
    name = 'merge_submodels'
    help = 'Merge submodels by aligning the reconstructions'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        meta_data = metadataset.MetaDataSet(args.dataset)

        reconstruction_shots = self._load_reconstruction_shots(meta_data)
        transformations = self._merge_reconstructions(reconstruction_shots)
        self._apply_transformations(transformations)

    def _load_reconstruction_shots(self, meta_data):
        reconstruction_shots = {}
        for submodel_path in meta_data.get_submodel_paths():
            data = dataset.DataSet(submodel_path)
            reconstruction = data.load_reconstruction()

            for index, partial_reconstruction in enumerate(reconstruction):
                reconstruction_shots[(submodel_path, index)] = partial_reconstruction.shots

        return reconstruction_shots

    def _merge_reconstructions(self, reconstruction_shots):
        ra = csfm.ReconstructionAlignment()
        added_shots = set()
        for key in reconstruction_shots:
            shots = reconstruction_shots[key]
            reconstruction_name = self._encode_reconstruction_name(key)
            ra.add_reconstruction(reconstruction_name, 0, 0, 0, 0, 0, 0, 1, False)
            for shot_id in shots:
                shot = shots[shot_id]
                shot_name = str(shot_id)

                R = shot.pose.rotation
                t = shot.pose.translation

                if shot_id not in added_shots:
                    ra.add_shot(shot_name, R[0], R[1], R[2], t[0], t[1], t[2], False)

                    gps = shot.metadata.gps_position
                    gps_sd = shot.metadata.gps_dop
                    ra.add_absolute_position_constraint(shot_name, gps[0], gps[1], gps[2], gps_sd)

                    added_shots.add(shot_id)

                covariance = np.diag([1e-5, 1e-5, 1e-5, 1e-2, 1e-2, 1e-2])
                sm = scale_matrix(covariance)
                rmc = csfm.RARelativeMotionConstraint(
                    reconstruction_name, shot_name, R[0], R[1], R[2], t[0], t[1], t[2])

                for i in range(6):
                    for j in range(6):
                        rmc.set_scale_matrix(i, j, sm[i, j])

                ra.add_relative_motion_constraint(rmc)

        ra.run()

        transformations = {}
        for key in reconstruction_shots:
            reconstruction_name = self._encode_reconstruction_name(key)
            r = ra.get_reconstruction(reconstruction_name)
            s = 1 / r.scale
            A = cv2.Rodrigues(np.array([r.rx, r.ry, r.rz]))[0].T
            b = -s * A.dot(np.array([r.tx, r.ty, r.tz]))
            transformations[key] = (s, A, b)

        return transformations

    def _apply_transformations(self, transformations):
        submodels = itertools.groupby(transformations.keys(), lambda key: key[0])
        for submodel_path, keys in submodels:
            data = dataset.DataSet(submodel_path)
            reconstruction = data.load_reconstruction()
            for key in keys:
                partial_reconstruction = reconstruction[key[1]]
                s, A, b = transformations[key]
                align.apply_similarity(partial_reconstruction, s, A, b)

            data.save_reconstruction(reconstruction, 'reconstruction.aligned.json')

    def _encode_reconstruction_name(self, key):
        return str(key[0]) + "_index" + str(key[1])


def scale_matrix(covariance):
    try:
        L = np.linalg.cholesky(covariance)
    except Exception as e:
        logger.error('Could not compute Cholesky of covariance matrix {}'.format(covariance))
        d = np.diag(np.diag(covariance).clip(1e-8, None))
        L = np.linalg.cholesky(d)

    return np.linalg.inv(L)
