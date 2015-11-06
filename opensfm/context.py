# -*- coding: utf-8 -*-
import os
import cv2

abspath = os.path.abspath(os.path.dirname(__file__))
SENSOR = os.path.join(abspath,  'data/sensor_data.json')

OPENCV3 = int(cv2.__version__.split('.')[0]) >= 3
