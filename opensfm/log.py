import logging
import os
import resource
import sys
from typing import Optional


def setup():
    logging.basicConfig(
        format="%(asctime)s %(levelname)s: %(message)s", level=logging.DEBUG
    )


def memory_usage() -> float:
    if sys.platform == "darwin":
        rusage_denom = 1024.0 * 1024.0
    else:
        rusage_denom = 1024.0
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / rusage_denom


def memory_available() -> Optional[int]:
    """Available memory in MB.

    Only works on linux and returns None otherwise.
    """
    lines = os.popen("free -t -m").readlines()
    if not lines:
        return None
    available_mem = int(lines[1].split()[6])
    return available_mem
