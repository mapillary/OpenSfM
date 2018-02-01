import json

from opensfm import context

with open(context.SENSOR, 'rb') as f:
    sensor_data = json.loads(f.read())

# Convert model types to lower cases for easier query
keys = [k.lower() for k in sensor_data.keys()]
values = sensor_data.values()
sensor_data = dict(zip(keys, values))
