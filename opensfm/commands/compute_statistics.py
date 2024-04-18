# pyre-unsafe
from . import command
import argparse
from opensfm.dataset import DataSet
from opensfm.actions import compute_statistics


class Command(command.CommandBase):
    name = "compute_statistics"
    help = "Compute statistics and save them in the stats folder"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        compute_statistics.run_dataset(dataset, args.diagram_max_points)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--diagram_max_points",
            default=-1,
            type=int,
            help="Cap the number of points (by index-based decimation) for computing heatmap / topview diagrams. Default: %(default)s (use all available points)",
        )
