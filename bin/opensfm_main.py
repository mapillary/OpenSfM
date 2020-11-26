import sys
from os.path import abspath, join, dirname

sys.path.insert(0, abspath(join(dirname(__file__), "..")))

from opensfm import commands
from opensfm.dataset import DataSet


def default_dataset_type(dataset_path):
    return DataSet(dataset_path)


if __name__ == "__main__":
    commands.command_runner(commands.opensfm_commands, default_dataset_type)
