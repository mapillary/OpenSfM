# pyre-unsafe
from functools import lru_cache
from typing import Any, Dict, List
import yaml
from opensfm import context
from opensfm import io


@lru_cache(1)
def sensor_data() -> Dict[str, Any]:
    with io.open_rt(context.SENSOR_DATA) as f:
        data = io.json_load(f)

    # Convert model types to lower cases for easier query
    return {k.lower(): v for k, v in data.items()}


@lru_cache(1)
def camera_calibration()-> List[Dict[str, Any]]:
    with io.open_rt(context.CAMERA_CALIBRATION) as f:
        data = yaml.safe_load(f)
    return data
