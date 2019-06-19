from io import StringIO

from opensfm import tracking
from opensfm.test import data_generation


def test_tracks_io():
    d = data_generation.CubeDataset(2, 100, 0.0, 0.3)

    output = StringIO()

    tracks_before = d.tracks
    tracking.save_tracks_graph(output, tracks_before)
    output.seek(0)
    tracks_after = tracking.load_tracks_graph(output)

    assert len(tracks_before.nodes()) == len(tracks_after.nodes())
    assert len(tracks_before.edges()) == len(tracks_after.edges())
