import os

import opensfm.dataset
import yaml
from opensfm import io


try:
    from libfb.py import parutil

    DATA_PATH = parutil.get_dir_path("mapillary/opensfm/opensfm/test/data")
except ImportError:
    DATA_PATH = os.path.abspath("data")


def create_berlin_test_folder(tmpdir):
    src = os.path.join(DATA_PATH, "berlin")
    dst = str(tmpdir.mkdir("berlin"))
    files = ["images", "masks", "config.yaml", "ground_control_points.json"]
    for filename in files:
        os.symlink(os.path.join(src, filename), os.path.join(dst, filename))
    return opensfm.dataset.DataSet(dst)


def save_config(config, path):
    with io.open_wt(os.path.join(path, "config.yaml")) as fout:
        yaml.safe_dump(config, fout, default_flow_style=False)
