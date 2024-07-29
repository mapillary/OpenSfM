# pyre-unsafe
from timeit import default_timer as timer

from opensfm import io
from opensfm import matching
from opensfm.dataset_base import DataSetBase


def run_dataset(data: DataSetBase) -> None:
    """Match features between image pairs."""

    images = data.images()

    start = timer()
    pairs_matches, preport = matching.match_images(data, {}, images, images)
    matching.save_matches(data, images, pairs_matches)
    matching.clear_cache()
    end = timer()
    write_report(data, preport, list(pairs_matches.keys()), end - start)


def write_report(data: DataSetBase, preport, pairs, wall_time) -> None:
    report = {
        "wall_time": wall_time,
        "num_pairs": len(pairs),
        "pairs": pairs,
    }
    report.update(preport)
    data.save_report(io.json_dumps(report), "matches.json")
