# pyre-unsafe
import argparse

from opensfm import reconstruction
from opensfm.actions import reconstruct
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "reconstruct"
    help = "Compute the reconstruction"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        reconstruct.run_dataset(dataset, args.algorithm)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "--algorithm",
            help="SfM algorithm to use to run reconstrution",
            type=str,
            choices=[k.value for k in reconstruction.ReconstructionAlgorithm],
            default=reconstruction.ReconstructionAlgorithm.INCREMENTAL.value,
        )
