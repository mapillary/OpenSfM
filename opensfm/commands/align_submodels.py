# pyre-unsafe
import argparse

from opensfm.actions import align_submodels
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "align_submodels"
    help = "Align submodel reconstructions"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        align_submodels.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
