# pyre-strict
import argparse

from opensfm.actions import match_features
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "match_features"
    help = "Match features between image pairs"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        match_features.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
