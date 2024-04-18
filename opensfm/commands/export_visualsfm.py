# pyre-unsafe
from opensfm.actions import export_visualsfm

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "export_visualsfm"
    help = "Export reconstruction to NVM_V3 format from VisualSfM"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        export_visualsfm.run_dataset(dataset, args.points, args.image_list)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--points", action="store_true", help="export points")
        parser.add_argument(
            "--image_list",
            type=str,
            help="Export only the shots included in this file (path to .txt file)",
        )
