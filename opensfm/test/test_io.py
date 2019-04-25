from __future__ import unicode_literals

import json
import os.path
from io import StringIO

import numpy as np

from opensfm import geo
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


def test_read_ground_control_points_list():
    text = """WGS84
13.400740745 52.519134104 12.0792090446 2335.0 1416.7 01.jpg
13.400740745 52.519134104 12.0792090446 2639.1 938.0 02.jpg
13.400502446 52.519251158 16.7021233002 766.0 1133.1 01.jpg
    """
    fp = StringIO(text)
    reference = geo.TopocentricConverter(52.51913, 13.4007, 0)
    images = ['01.jpg', '02.jpg']
    exif = {i: {'width': 3000, 'height': 2000} for i in images}

    points = io.read_ground_control_points_list(fp, reference, exif)
    assert len(points) == 2

    a, b = (len(point.observations) for point in points)
    assert min(a, b) == 1
    assert max(a, b) == 2
