# pyre-unsafe
from opensfm.actions import extract_metadata

from . import command
import argparse
from opensfm.dataset import DataSet


class Command(command.CommandBase):
    name = "extract_metadata"
    help = "Extract metadata from images' EXIF tag"

    def run_impl(self, dataset: DataSet, args: argparse.Namespace) -> None:
        extract_metadata.run_dataset(dataset)

    def add_arguments_impl(self, parser: argparse.ArgumentParser) -> None:
        pass
