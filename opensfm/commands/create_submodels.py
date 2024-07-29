# pyre-unsafe
from opensfm.actions import create_submodels

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "create_submodels"
    help = "Split the dataset into smaller submodels"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        create_submodels.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
