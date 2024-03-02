# pyre-unsafe
import sys
from os.path import abspath, dirname, join

sys.path.insert(0, abspath(join(dirname(__file__), "..")))

import contextlib
from typing import Generator

from opensfm import commands
from opensfm.dataset import DataSet


@contextlib.contextmanager
def create_default_dataset_context(
    dataset_path: str, dataset_type: str = ""
) -> Generator[DataSet, None, None]:
    dataset = DataSet(dataset_path)
    try:
        yield dataset
    finally:
        dataset.clean_up()


def main() -> None:
    commands.command_runner(
        commands.opensfm_commands,
        create_default_dataset_context,
        dataset_choices=["opensfm"],
    )


if __name__ == "__main__":
    main()  # pragma: no cover
