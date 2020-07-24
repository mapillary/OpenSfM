import numpy as np
import networkx as nx

from opensfm import io
from opensfm import pygeometry
from opensfm import reconstruction
from opensfm import pysfm


def test_track_triangulator_equirectangular():
    """Test triangulating tracks of spherical images."""
    tracks_manager = pysfm.TracksManager()
    tracks_manager.add_observation('im1', '1', pysfm.Observation(0, 0, 1.0, 0, 0, 0, 0))
    tracks_manager.add_observation('im2', '1', pysfm.Observation(-0.1, 0, 1.0, 0, 0, 0, 1))

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
    triangulator = reconstruction.TrackTriangulator(tracks_manager, graph_inliers, rec)
    triangulator.triangulate('1', 0.01, 2.0)
    assert '1' in rec.points
    p = rec.points['1'].coordinates
    assert np.allclose(p, [0, 0, 1.3763819204711])
    assert len(graph_inliers.edges()) == 2


def unit_vector(x):
    return np.array(x) / np.linalg.norm(x)


def test_triangulate_bearings_dlt():
    rt1 = np.append(np.identity(3), [[0], [0], [0]], axis=1)
    rt2 = np.append(np.identity(3), [[-1], [0], [0]], axis=1)
    b1 = unit_vector([0.0, 0, 1])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    res, X = pygeometry.triangulate_bearings_dlt(
        [rt1, rt2], [b1, b2], max_reprojection, min_ray_angle)
    assert np.allclose(X, [0, 0, 1.0])
    assert res is True


def test_triangulate_bearings_midpoint():
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    res, X = pygeometry.triangulate_bearings_midpoint(
        [o1, o2], [b1, b2], 2 * [max_reprojection], min_ray_angle)
    assert np.allclose(X, [0, 0, 1.0])
    assert res is True


def test_triangulate_two_bearings_midpoint():
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    X = pygeometry.triangulate_two_bearings_midpoint([o1, o2], [b1, b2])
    assert np.allclose(X, [0, 0, 1.0])
