# pyre-strict
from opensfm.dataset import DataSet
from opensfm.large import metadataset, tools


def run_dataset(data: DataSet) -> None:
    """Align submodel reconstructions for of MetaDataSet."""

    meta_data = metadataset.MetaDataSet(data.data_path)
    reconstruction_shots = tools.load_reconstruction_shots(meta_data)
    transformations = tools.align_reconstructions(
        reconstruction_shots, tools.partial_reconstruction_name, True
    )
    tools.apply_transformations(transformations)
