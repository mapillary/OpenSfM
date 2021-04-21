import sys
from os.path import abspath, join, dirname

sys.path.insert(0, abspath(join(dirname(__file__), "..")))

from opensfm import commands
from opensfm.dataset import DataSet


def create_default_dataset(dataset_path, dataset_type):
    return DataSet(dataset_path)


if __name__ == "__main__":
    commands.command_runner(
        commands.opensfm_commands, create_default_dataset, dataset_choices=["opensfm"]
    )
