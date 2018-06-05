import json

from opensfm import context
from opensfm import io

with io.open_rt(context.SENSOR) as f:
    sensor_data = io.json_load(f)

# Convert model types to lower cases for easier query
keys = [k.lower() for k in sensor_data.keys()]
values = sensor_data.values()
sensor_data = dict(zip(keys, values))
