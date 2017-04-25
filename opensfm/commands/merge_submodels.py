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

        reconstructions = self.load_reconstructions(meta_data)
        transformations = self.merge_reconstructions(reconstructions)
        reconstructions = self.apply_transformations(reconstructions, transformations)
        self.save_aligned_reconstructions(reconstructions, meta_data)

    def load_reconstructions(self, meta_data):
        reconstructions = {}
        for submodel_path in meta_data.get_submodel_paths():
            data = dataset.DataSet(submodel_path)
            reconstruction = data.load_reconstruction()

            for index, partial_reconstruction in enumerate(reconstruction):
                reconstructions[(submodel_path, index)] = partial_reconstruction

        return reconstructions

    def merge_reconstructions(self, reconstructions):
        transformations = {}
        for key in reconstructions:
            transformations[key] = (1, np.eye(3), np.zeros(3))

        return transformations

    def apply_transformations(self, reconstructions, transformations):
        aligned_reconstructions = {}
        for key, reconstruction in reconstructions.iteritems():
            s, A, b = transformations[key]
            align.apply_similarity(reconstruction, s, A, b)
            aligned_reconstructions[key] = reconstruction

        return aligned_reconstructions

    def save_aligned_reconstructions(self, aligned_reconstructions, meta_data):
        reconstruction_groups = itertools.groupby(aligned_reconstructions.keys(), lambda k: k[0])
        for submodel_path, submodel_keys in reconstruction_groups:
            data = dataset.DataSet(submodel_path)
            reconstruction = [aligned_reconstructions[key] for key in submodel_keys]
            data.save_reconstruction(reconstruction, 'reconstruction.aligned.json')
