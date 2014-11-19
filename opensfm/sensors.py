# -*- coding: utf-8 -*-

import string
import json

import context

with open(context.SENSOR,'rb') as f:
    sensor_data = json.loads(f.read())

# Convert model types to lower cases for easier query
sensor_data = dict(zip(map(string.lower,sensor_data.keys()),sensor_data.values()))
