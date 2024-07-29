# pyre-unsafe
from opensfm import io
from opensfm import reconstruction
from opensfm.dataset_base import DataSetBase


def run_dataset(data: DataSetBase, algorithm: reconstruction.ReconstructionAlgorithm) -> None:
    """Compute the SfM reconstruction."""

    tracks_manager = data.load_tracks_manager()

    if algorithm == reconstruction.ReconstructionAlgorithm.INCREMENTAL:
        report, reconstructions = reconstruction.incremental_reconstruction(
            data, tracks_manager
        )
    elif algorithm == reconstruction.ReconstructionAlgorithm.TRIANGULATION:
        report, reconstructions = reconstruction.triangulation_reconstruction(
            data, tracks_manager
        )
    else:
        raise RuntimeError(f"Unsupported algorithm for reconstruction {algorithm}")

    data.save_reconstruction(reconstructions)
    data.save_report(io.json_dumps(report), "reconstruction.json")
