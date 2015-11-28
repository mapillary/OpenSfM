import os.path, sys

from opensfm import io

filename = os.path.join(os.path.dirname(__file__), 'reconstruction_berlin.json')

def test_import_opensfm():

    reconstructions = io.import_opensfm(filename)

    assert len(reconstructions) == 1
    assert len(reconstructions[0].cameras) == 1
    assert len(reconstructions[0].shots) == 3
    assert len(reconstructions[0].points) == 1588

if __name__ == "__main__":
    test_import_opensfm()
