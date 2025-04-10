# pyre-unsafe
import argparse

from opensfm.actions import export_openmvs
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "export_openmvs"
    help = "Export reconstruction to openMVS format"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        export_openmvs.run_dataset(dataset, args.image_list)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--image_list",
            type=str,
            help="Export only the shots included in this file (path to .txt file)",
        )
