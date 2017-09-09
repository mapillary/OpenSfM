from opensfm import metadataset
from opensfm import tools


class Command:
    name = 'align_submodels'
    help = 'Align submodel reconstructions'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        meta_data = metadataset.MetaDataSet(args.dataset)
        reconstruction_shots = tools.load_reconstruction_shots(meta_data)
        transformations = tools.align_reconstructions(reconstruction_shots)
        tools.apply_transformations(transformations)
