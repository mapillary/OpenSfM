# pyre-unsafe
import logging
from timeit import default_timer as timer

from opensfm import features_processing, io
from opensfm.dataset_base import DataSetBase


logger: logging.Logger = logging.getLogger(__name__)


def run_dataset(data: DataSetBase) -> None:
    """Compute features for all images."""

    start = timer()
    features_processing.run_features_processing(data, data.images(), False)
    end = timer()
    write_report(data, end - start)


def write_report(data: DataSetBase, wall_time: float) -> None:
    image_reports = []
    for image in data.images():
        try:
            txt = data.load_report("features/{}.json".format(image))
            image_reports.append(io.json_loads(txt))
        except IOError:
            logger.warning("No feature report image {}".format(image))

    report = {"wall_time": wall_time, "image_reports": image_reports}
    data.save_report(io.json_dumps(report), "features.json")
