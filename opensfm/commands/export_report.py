# pyre-unsafe
from . import command
import argparse
from opensfm.dataset import DataSet
from opensfm.actions import export_report


class Command(command.CommandBase):
    name = "export_report"
    help = "Export a nice report based on previously generated statistics"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        export_report.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
