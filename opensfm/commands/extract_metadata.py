# pyre-strict
import argparse

from opensfm.actions import extract_metadata
from opensfm.dataset import DataSet

from . import command


class Command(command.CommandBase):
    name = "extract_metadata"
    help = "Extract metadata from images' EXIF tag"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        extract_metadata.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
