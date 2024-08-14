# pyre-unsafe
from timeit import default_timer as timer
import argparse
from opensfm.dataset import DataSet

class CommandBase:
    """ Base class for executable commands."""

    name = "Undefined command"
    help = "Undefined command help"

    def run(self, data, args: argparse.Namespace) -> None:
        start = timer()
        self.run_impl(data, args)
        end = timer()
        data.append_to_profile_log(f"{type(self).name}: {end - start}\n")

    def add_arguments(self, parser: argparse.ArgumentParser) -> None:
        parser.add_argument("dataset", help="dataset to process")
        self.add_arguments_impl(parser)

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        raise NotImplementedError("Command " + self.name + " not implemented")

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        raise NotImplementedError("Command " + self.name + " not implemented")
