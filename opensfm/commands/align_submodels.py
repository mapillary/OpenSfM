import itertools

from opensfm import align
from opensfm import dataset
from opensfm.large import metadataset
from opensfm.large import tools


class Command:
    name = 'align_submodels'
    help = 'Align submodel reconstructions'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        meta_data = metadataset.MetaDataSet(args.dataset)

        reconstruction_shots = self._load_reconstruction_shots(meta_data)
        transformations = tools.align_reconstructions(reconstruction_shots)
        self._apply_transformations(transformations)

    def _load_reconstruction_shots(self, meta_data):
        reconstruction_shots = {}
        for submodel_path in meta_data.get_submodel_paths():
            data = dataset.DataSet(submodel_path)
            if not data.reconstruction_exists():
                continue

            reconstruction = data.load_reconstruction()

            for index, partial_reconstruction in enumerate(reconstruction):
                reconstruction_shots[(submodel_path, index)] = partial_reconstruction.shots

        return reconstruction_shots

    def _apply_transformations(self, transformations):
        submodels = itertools.groupby(transformations.keys(), lambda key: key[0])
        for submodel_path, keys in submodels:
            data = dataset.DataSet(submodel_path)
            if not data.reconstruction_exists():
                continue

            reconstruction = data.load_reconstruction()
            for key in keys:
                partial_reconstruction = reconstruction[key[1]]
                s, A, b = transformations[key]
                align.apply_similarity(partial_reconstruction, s, A, b)

            data.save_reconstruction(reconstruction, 'reconstruction.aligned.json')
