from io import StringIO

from opensfm import tracking
from opensfm.synthetic_data import synthetic_scene


def test_tracks_io():
    data = synthetic_scene.SyntheticCubeScene(2, 100, 0.0)
    _, _, _, graph = data.get_tracks_data(40, 0.0)

    output = StringIO()

    tracks_before = graph
    tracking.save_tracks_graph(output, tracks_before)
    output.seek(0)
    tracks_after = tracking.load_tracks_graph(output)

    assert len(tracks_before.nodes()) == len(tracks_after.nodes())
    assert len(tracks_before.edges()) == len(tracks_after.edges())
