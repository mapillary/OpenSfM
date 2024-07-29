# pyre-unsafe
from opensfm import io
from opensfm import reconstruction
from opensfm.dataset_base import DataSetBase


def run_dataset(data: DataSetBase, input: str, output: str) -> None:
    """Reconstruct the from a prior reconstruction."""

    tracks_manager = data.load_tracks_manager()
    rec_prior = data.load_reconstruction(input)
    if len(rec_prior) > 0:
        report, rec = reconstruction.reconstruct_from_prior(
            data, tracks_manager, rec_prior[0]
        )
        data.save_reconstruction([rec], output)
        data.save_report(io.json_dumps(report), "reconstruction.json")
