import os

import networkx as nx
import numpy as np
from six import iteritems
import yaml

from opensfm import io
from opensfm import types
import opensfm.dataset


def create_berlin_test_folder(tmpdir):
    path = str(tmpdir.mkdir('berlin'))
    os.symlink(os.path.abspath('data/berlin/images'),
               os.path.join(path, 'images'))
    os.symlink(os.path.abspath('data/berlin/masks'),
               os.path.join(path, 'masks'))
    os.symlink(os.path.abspath('data/berlin/config.yaml'),
               os.path.join(path, 'config.yaml'))
    os.symlink(os.path.abspath('data/berlin/ground_control_points.json'),
               os.path.join(path, 'ground_control_points.json'))
    return opensfm.dataset.DataSet(path)


def save_config(config, path):
    with io.open_wt(os.path.join(path, 'config.yaml')) as fout:
        yaml.safe_dump(config, fout, default_flow_style=False)
