# pyre-unsafe
import logging
import os

try:
    import resource
except ModuleNotFoundError:
    pass  # Windows
import ctypes
import sys
from typing import Optional

import cv2
from joblib import Parallel, delayed, parallel_backend


logger: logging.Logger = logging.getLogger(__name__)


abspath = os.path.dirname(os.path.realpath(__file__))
SENSOR_DATA = os.path.join(abspath, "data", "sensor_data.json")
CAMERA_CALIBRATION = os.path.join(abspath, "data", "camera_calibration.yaml")
BOW_PATH = os.path.join(abspath, "data", "bow")


# Handle different OpenCV versions
OPENCV5: bool = int(cv2.__version__.split(".")[0]) >= 5
OPENCV4: bool = int(cv2.__version__.split(".")[0]) >= 4
OPENCV44: bool = (
    int(cv2.__version__.split(".")[0]) == 4 and int(cv2.__version__.split(".")[1]) >= 4
)
OPENCV3: bool = int(cv2.__version__.split(".")[0]) >= 3

if hasattr(cv2, "flann_Index"):
    flann_Index = cv2.flann_Index
elif hasattr(cv2, "flann") and hasattr(cv2.flann, "Index"):
    flann_Index = cv2.flann.Index
else:
    logger.warning("Unable to find flann Index")
    flann_Index = None


# Parallel processes
def parallel_map(func, args, num_proc: int, max_batch_size: int = 1):
    """Run function for all arguments using multiple processes."""
    # De-activate/Restore any inner OpenCV threading
    threads_used = cv2.getNumThreads()
    cv2.setNumThreads(0)

    num_proc = min(num_proc, len(args))
    if num_proc <= 1:
        res = list(map(func, args))
    else:
        with parallel_backend("threading", n_jobs=num_proc):
            batch_size = max(1, int(len(args) / (num_proc * 2)))
            batch_size = (
                min(batch_size, max_batch_size) if max_batch_size else batch_size
            )
            res = Parallel(batch_size=batch_size)(delayed(func)(arg) for arg in args)

    cv2.setNumThreads(threads_used)
    return res


# Memory usage

if sys.platform == "win32":

    class MEMORYSTATUSEX(ctypes.Structure):
        _fields_ = [
            ("dwLength", ctypes.c_ulong),
            ("dwMemoryLoad", ctypes.c_ulong),
            ("ullTotalPhys", ctypes.c_ulonglong),
            ("ullAvailPhys", ctypes.c_ulonglong),
            ("ullTotalPageFile", ctypes.c_ulonglong),
            ("ullAvailPageFile", ctypes.c_ulonglong),
            ("ullTotalVirtual", ctypes.c_ulonglong),
            ("ullAvailVirtual", ctypes.c_ulonglong),
            ("sullAvailExtendedVirtual", ctypes.c_ulonglong),
        ]

        def __init__(self) -> None:
            # have to initialize this to the size of MEMORYSTATUSEX
            self.dwLength = ctypes.sizeof(self)
            super(MEMORYSTATUSEX, self).__init__()

    def memory_available() -> Optional[int]:
        """Available memory in MB.

        Only works on Windows
        """
        stat = MEMORYSTATUSEX()
        ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(stat))
        return stat.ullAvailPhys / 1024 / 1024

    def current_memory_usage() -> int:
        stat = MEMORYSTATUSEX()
        ctypes.windll.kernel32.GlobalMemoryStatusEx(ctypes.byref(stat))
        return (stat.ullTotalPhys - stat.ullAvailPhys) / 1024


else:
    if sys.platform == "darwin":
        rusage_unit = 1
    else:
        rusage_unit = 1024

    def memory_available() -> Optional[int]:
        """Available memory in MB.

        Only works on linux and returns None otherwise.
        """
        with os.popen("free -t -m") as fp:
            lines = fp.readlines()
        if not lines:
            return None
        available_mem = int(lines[1].split()[6])
        return available_mem

    def current_memory_usage() -> int:
        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * rusage_unit


def processes_that_fit_in_memory(desired: int, per_process: int) -> int:
    """Amount of parallel BoW process that fit in memory."""
    available_mem = memory_available()
    if available_mem is not None:
        fittable = max(1, int(available_mem / per_process))
        return min(desired, fittable)
    else:
        return desired
