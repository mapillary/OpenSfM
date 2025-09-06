# pyre-strict
from timeit import default_timer as timer

from opensfm import io, pymap, tracking
from opensfm.dataset_base import DataSetBase


def run_dataset(data: DataSetBase) -> None:
    """Link matches pair-wise matches into tracks."""

    start = timer()
    features, colors, segmentations, instances, depths = tracking.load_features(
        data, data.images()
    )
    features_end = timer()
    matches = lambda: tracking.load_matches(data, data.images())
    matches_end = timer()

    tracks_manager = tracking.create_tracks_manager_from_matches_iter(
        features,
        colors,
        segmentations,
        instances,
        matches,
        data.config["min_track_length"],
        depths,
        data.config["depth_is_radial"],
        data.config["depth_std_deviation_m_default"],
    )
    tracks_end = timer()
    data.save_tracks_manager(tracks_manager)
    write_report(
        data,
        tracks_manager,
        features_end - start,
        matches_end - features_end,
        tracks_end - matches_end,
    )


def write_report(
    data: DataSetBase,
    tracks_manager: pymap.TracksManager,
    features_time: float,
    matches_time: float,
    tracks_time: float,
) -> None:
    view_graph = [
        (k[0], k[1], v) for k, v in tracks_manager.get_all_pairs_connectivity().items()
    ]

    report = {
        "wall_times": {
            "load_features": features_time,
            "load_matches": matches_time,
            "compute_tracks": tracks_time,
        },
        "wall_time": features_time + matches_time + tracks_time,
        "num_images": tracks_manager.num_shots(),
        "num_tracks": tracks_manager.num_tracks(),
        "view_graph": view_graph,
    }
    data.save_report(io.json_dumps(report), "tracks.json")
