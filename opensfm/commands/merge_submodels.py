import itertools
import numpy as np

import create_submodels as metadataset
from opensfm import align
from opensfm import dataset


class Command:
    name = 'merge_submodels'
    help = 'Merge submodels by aligning the reconstructions'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        meta_data = metadataset.MetaDataSet(args.dataset)

        reconstruction_shots = self.load_reconstruction_shots(meta_data)
        transformations = self.merge_reconstructions(reconstruction_shots)
        self.apply_transformations(transformations)

    def load_reconstruction_shots(self, meta_data):
        reconstruction_shots = {}
        for submodel_path in meta_data.get_submodel_paths():
            data = dataset.DataSet(submodel_path)
            reconstruction = data.load_reconstruction()

            for index, partial_reconstruction in enumerate(reconstruction):
                reconstruction_shots[(submodel_path, index)] = partial_reconstruction.shots

        return reconstruction_shots

    def merge_reconstructions(self, reconstruction_shots):
        transformations = {}
        for key in reconstruction_shots:
            transformations[key] = (1, np.eye(3), np.zeros(3))

        return transformations

    def apply_transformations(self, transformations):
        submodels = itertools.groupby(transformations.keys(), lambda key: key[0])
        for submodel_path, keys in submodels:
            data = dataset.DataSet(submodel_path)
            reconstruction = data.load_reconstruction()
            for key in keys:
                partial_reconstruction = reconstruction[key[1]]
                s, A, b = transformations[key]
                align.apply_similarity(partial_reconstruction, s, A, b)

            data.save_reconstruction(reconstruction, 'reconstruction.aligned.json')
