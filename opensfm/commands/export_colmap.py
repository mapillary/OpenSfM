# pyre-strict
import argparse

from opensfm.actions import export_colmap
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "export_colmap"
    help = "Export reconstruction to colmap format"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        export_colmap.run_dataset(dataset, args.binary)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--binary", help="export using binary format", action="store_true"
        )
