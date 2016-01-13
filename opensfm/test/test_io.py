import json
import os.path

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
