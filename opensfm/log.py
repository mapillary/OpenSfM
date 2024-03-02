# pyre-unsafe
import logging
import os
from typing import Optional


def setup() -> None:
    logging.basicConfig(
        format="%(asctime)s %(levelname)s: %(message)s", level=logging.DEBUG
    )


def memory_available() -> Optional[int]:
    """Available memory in MB.

    Only works on linux and returns None otherwise.
    """
    lines = os.popen("free -t -m").readlines()
    if not lines:
        return None
    available_mem = int(lines[1].split()[6])
    return available_mem
