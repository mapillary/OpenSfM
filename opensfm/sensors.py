# -*- coding: utf-8 -*-

import string
import json

from . import context

with open(context.SENSOR,'rb') as f:
    sensor_data = json.loads(f.read())

# Convert model types to lower cases for easier query
sensor_data = dict(list(zip(list(map(str.lower,list(sensor_data.keys()))),list(sensor_data.values()))))
