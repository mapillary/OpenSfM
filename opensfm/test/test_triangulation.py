import numpy as np
import networkx as nx

from opensfm import io
from opensfm import csfm
from opensfm import reconstruction


def test_track_triangulator_equirectangular():
    """Test triangulating tracks of spherical images."""
    graph = nx.Graph()
    graph.add_node('im1', bipartite=0)
    graph.add_node('im2', bipartite=0)
    graph.add_node('1', bipartite=1)
    graph.add_edge('im1', '1', feature=(0, 0), feature_scale=1.0, feature_id=0, feature_color=0)
    graph.add_edge('im2', '1', feature=(-0.1, 0), feature_scale=1.0, feature_id=1, feature_color=0)

    rec = io.reconstruction_from_json({
        "cameras": {
            "theta": {
                "projection_type": "equirectangular",
                "width": 800,
                "height": 400,
            }
        },

        "shots": {
            'im1': {
                "camera": "theta",
                "rotation": [0.0, 0.0, 0.0],
                "translation": [0.0, 0.0, 0.0],
            },
            'im2': {
                "camera": "theta",
                "rotation": [0, 0, 0.0],
                "translation": [-1, 0, 0.0],
            },
        },

        "points": {
        },
    })

    graph_inliers = nx.Graph()
    triangulator = reconstruction.TrackTriangulator(graph, graph_inliers, rec)
    triangulator.triangulate('1', 0.01, 2.0)
    assert '1' in rec.points
    p = rec.points['1'].coordinates
    assert np.allclose(p, [0, 0, 1.3763819204711])
    assert len(graph_inliers.edges()) == 2


def unit_vector(x):
    return np.array(x) / np.linalg.norm(x)


def test_triangulate_bearings_midpoint():
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    res, X = csfm.triangulate_bearings_midpoint(
        [o1, o2], [b1, b2], 2 * [max_reprojection], min_ray_angle)

    assert np.allclose(X, [0, 0, 1.0])
    assert res == 0
