# pyre-unsafe
from opensfm.actions import match_features

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "match_features"
    help = "Match features between image pairs"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        match_features.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
