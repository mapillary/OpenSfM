from io import StringIO

import opensfm.tracking
import data_generation


def test_tracks_io():
    d = data_generation.CubeDataset(2, 100, 0.0, 0.3)

    output = StringIO()

    tracks_before = d.tracks
    opensfm.tracking.save_tracks_graph(output, tracks_before)
    output.seek(0)
    tracks_after = opensfm.tracking.load_tracks_graph(output)

    assert len(tracks_before.nodes()) == len(tracks_after.nodes())
    assert len(tracks_before.edges()) == len(tracks_after.edges())
