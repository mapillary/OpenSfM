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


if __name__ == "__main__":
    test_reconstructions_from_json()
