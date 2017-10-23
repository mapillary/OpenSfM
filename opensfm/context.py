# -*- coding: utf-8 -*-
import os
import cv2
from loky import get_reusable_executor


abspath = os.path.abspath(os.path.dirname(__file__))
SENSOR = os.path.join(abspath,  'data/sensor_data.json')

OPENCV3 = int(cv2.__version__.split('.')[0]) >= 3


def parallel_map(func, args, num_proc):
    """Run function for all arguments using multiple processes."""
    num_proc = min(num_proc, len(args))
    if num_proc <= 1:
        return map(func, args)
    else:
        with get_reusable_executor(max_workers=num_proc, timeout=None) as e:
            return list(e.map(func, args))
