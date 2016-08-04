import json
import os.path

import numpy as np

from opensfm import io

filename = os.path.join(os.path.dirname(__file__),
                        'reconstruction_berlin.json')


def test_reconstructions_from_json():
    with open(filename) as fin:
        obj = json.loads(fin.read())

    reconstructions = io.reconstructions_from_json(obj)

    assert len(reconstructions) == 1
    assert len(reconstructions[0].cameras) == 1
    assert len(reconstructions[0].shots) == 3
    assert len(reconstructions[0].points) == 1588


def test_reconstruction_to_ply():
    with open(filename) as fin:
        obj = json.loads(fin.read())
    reconstructions = io.reconstructions_from_json(obj)
    ply = io.reconstruction_to_ply(reconstructions[0])
    assert len(ply.splitlines()) > len(reconstructions[0].points)


def test_parse_projection():
    proj = io._parse_projection('WGS84')
    assert proj is None

    proj = io._parse_projection('WGS84 UTM 31N')
    easting, northing = 431760, 4582313.7
    lat, lon = 41.38946, 2.18378
    plon, plat = proj(easting, northing, inverse=True)
    assert np.allclose((lat, lon), (plat, plon))
