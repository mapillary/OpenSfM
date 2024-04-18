# pyre-unsafe
from opensfm.actions import create_tracks

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "create_tracks"
    help = "Link matches pair-wise matches into tracks"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        create_tracks.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
