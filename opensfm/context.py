# -*- coding: utf-8 -*-
import logging
import os
import resource
import sys

import cv2
from loky import get_reusable_executor


logger = logging.getLogger(__name__)


abspath = os.path.abspath(os.path.dirname(__file__))
SENSOR = os.path.join(abspath, 'data/sensor_data.json')


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
        with get_reusable_executor(max_workers=num_proc, timeout=None) as e:
            return list(e.map(func, args))


# Memory usage
if sys.platform == 'darwin':
    rusage_unit = 1
else:
    rusage_unit = 1024


def current_memory_usage():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * rusage_unit
