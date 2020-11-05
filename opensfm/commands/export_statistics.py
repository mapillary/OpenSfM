import logging
import os

from opensfm import io
from opensfm import dataset
from opensfm import stats

logger = logging.getLogger(__name__)


class Command:
    name = "export_statistics"
    help = "Compute statistics and export them in the stats folder"

    def add_arguments(self, parser):
        parser.add_argument(
            "dataset",
            help="dataset to process",
        )
        parser.add_argument(
            "--interactive",
            help="plot results as they are being computed",
            action="store_true",
        )

    def run(self, args):
        data = dataset.DataSet(args.dataset)

        reconstructions = data.load_reconstruction()
        tracks_manager = data.load_tracks_manager()

        output_path = os.path.join(data.data_path, "stats")
        io.mkdir_p(output_path)

        stats_dict = {}
        overall_stats = []
        for rec in reconstructions:
            overall_stats.append(stats.compute_overall_statistics(rec))
        stats_dict["overall_stats"] = overall_stats

        stats.save_residual_grids(data, tracks_manager, reconstructions, output_path)
        stats.save_heat_map(data, tracks_manager, reconstructions, output_path)

        with io.open_wt(os.path.join(output_path, "stats.json")) as fout:
            io.json_dump(stats_dict, fout)
