# pyre-strict
import argparse

from opensfm.actions import export_report
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "export_report"
    help = "Export a nice report based on previously generated statistics"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        export_report.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
