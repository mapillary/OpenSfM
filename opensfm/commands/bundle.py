# pyre-unsafe
from opensfm.actions import bundle

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "bundle"
    help = "Bundle a reconstruction"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        bundle.run_dataset(dataset, args.input, args.output)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument("--input", help="file name of the reconstruction to bundle")
        parser.add_argument(
            "--output", help="file name where to store the bundled reconstruction"
        )
