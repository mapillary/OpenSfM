# -*- coding: utf-8 -*-
import logging
import os
import resource
import sys

import cv2
from joblib import Parallel, parallel_backend, delayed

from opensfm import log


logger = logging.getLogger(__name__)


abspath = os.path.abspath(os.path.dirname(__file__))
SENSOR = os.path.join(abspath, 'data', 'sensor_data.json')
BOW_PATH = os.path.join(abspath, 'data', 'bow')


# Handle different OpenCV versions
OPENCV3 = int(cv2.__version__.split('.')[0]) >= 3

if hasattr(cv2, 'flann_Index'):
    flann_Index = cv2.flann_Index
elif hasattr(cv2, 'flann') and hasattr(cv2.flann, 'Index'):
    flann_Index = cv2.flann.Index
else:
    logger.warning('Unable to find flann Index')
    flann_Index = None


# Parallel processes
def parallel_map(func, args, num_proc):
    """Run function for all arguments using multiple processes."""
    num_proc = min(num_proc, len(args))
    if num_proc <= 1:
        return list(map(func, args))
    else:
        with parallel_backend('loky', n_jobs=num_proc):
            return Parallel()(delayed(func)(arg) for arg in args)


# Memory usage
if sys.platform == 'darwin':
    rusage_unit = 1
else:
    rusage_unit = 1024


def processes_that_fit_in_memory(desired):
    """Amount of parallel matching process that fit in memory."""
    per_process_mem = 1.6 * 1024
    available_mem = log.memory_available()
    if available_mem is not None:
        fittable = max(1, int(available_mem / per_process_mem))
        return min(desired, fittable)
    else:
        return desired


def current_memory_usage():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * rusage_unit
