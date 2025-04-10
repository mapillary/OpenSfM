# pyre-unsafe
import numpy as np
from opensfm import io, pygeometry, pymap, reconstruction


def test_track_triangulator_spherical() -> None:
    """Test triangulating tracks of spherical images."""
    tracks_manager = pymap.TracksManager()
    tracks_manager.add_observation("im1", "1", pymap.Observation(0, 0, 1.0, 0, 0, 0, 0))
    tracks_manager.add_observation(
        "im2", "1", pymap.Observation(-0.1, 0, 1.0, 0, 0, 0, 1)
    )

    rec = io.reconstruction_from_json(
        {
            "cameras": {
                "theta": {
                    "projection_type": "spherical",
                    "width": 800,
                    "height": 400,
                }
            },
            "shots": {
                "im1": {
                    "camera": "theta",
                    "rotation": [0.0, 0.0, 0.0],
                    "translation": [0.0, 0.0, 0.0],
                },
                "im2": {
                    "camera": "theta",
                    "rotation": [0.0, 0.0, 0.0],
                    "translation": [-1.0, 0.0, 0.0],
                },
            },
            "points": {},
        }
    )

    triangulator = reconstruction.TrackTriangulator(
        rec, reconstruction.TrackHandlerTrackManager(tracks_manager, rec)
    )
    triangulator.triangulate(
        "1",
        reproj_threshold=0.01,
        min_ray_angle_degrees=2.0,
        min_depth=0.001,
        iterations=10,
    )
    assert "1" in rec.points
    p = rec.points["1"].coordinates
    assert np.allclose(p, [0, 0, 1.3763819204711])
    assert len(rec.points["1"].get_observations()) == 2


def test_track_triangulator_coincident_camera_origins() -> None:
    """Test triangulating tracks when two cameras have the same origin.

    Triangulation should fail and no points should be added to the reconstruction.
    """
    tracks_manager = pymap.TracksManager()
    tracks_manager.add_observation("im1", "1", pymap.Observation(0, 0, 1.0, 0, 0, 0, 0))
    tracks_manager.add_observation(
        "im2", "1", pymap.Observation(-0.1, 0, 1.0, 0, 0, 0, 1)
    )

    rec = io.reconstruction_from_json(
        {
            "cameras": {
                "theta": {
                    "projection_type": "spherical",
                    "width": 800,
                    "height": 400,
                }
            },
            "shots": {
                "im1": {
                    "camera": "theta",
                    "rotation": [0.0, 0.0, 0.0],
                    "translation": [0.0, 0.0, 0.0],
                },
                "im2": {
                    "camera": "theta",
                    "rotation": [0.0, 0.0, 0.0],
                    "translation": [0.0, 0.0, 0.0],
                },
            },
            "points": {},
        }
    )

    triangulator = reconstruction.TrackTriangulator(
        rec, reconstruction.TrackHandlerTrackManager(tracks_manager, rec)
    )
    triangulator.triangulate(
        "1",
        reproj_threshold=0.01,
        min_ray_angle_degrees=2.0,
        min_depth=0.0001,
        iterations=10,
    )
    assert not rec.points


def unit_vector(x: object) -> np.ndarray:
    # pyre-fixme[6]: For 1st argument expected `Union[_SupportsArray[dtype[typing.Any...
    return np.array(x) / np.linalg.norm(x)


def test_triangulate_bearings_dlt() -> None:
    rt1 = np.append(np.identity(3), [[0], [0], [0]], axis=1)
    rt2 = np.append(np.identity(3), [[-1], [0], [0]], axis=1)
    b1 = unit_vector([0.0, 0, 1])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    min_depth = 0.001
    res, X = pygeometry.triangulate_bearings_dlt(
        [rt1, rt2], np.asarray([b1, b2]), max_reprojection, min_ray_angle, min_depth
    )
    assert np.allclose(X, [0, 0, 1.0])
    assert res is True


def test_triangulate_bearings_dlt_coincident_camera_origins() -> None:
    rt1 = np.append(np.identity(3), [[0], [0], [0]], axis=1)
    rt2 = np.append(np.identity(3), [[0], [0], [0]], axis=1)  # same origin
    b1 = unit_vector([0.0, 0, 1])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    min_depth = 0.001
    res, X = pygeometry.triangulate_bearings_dlt(
        [rt1, rt2], np.asarray([b1, b2]), max_reprojection, min_ray_angle, min_depth
    )
    assert res is False


def test_triangulate_bearings_midpoint() -> None:
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    min_depth = 0.001
    valid_triangulation, X = pygeometry.triangulate_bearings_midpoint(
        np.asarray([o1, o2]),
        np.asarray([b1, b2]),
        2 * [max_reprojection],
        min_ray_angle,
        min_depth,
    )
    assert np.allclose(X, [0, 0, 1.0])
    assert valid_triangulation is True


def test_triangulate_bearings_midpoint_coincident_camera_origins() -> None:
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([0.0, 0, 0])  # same origin
    b2 = unit_vector([-1.0, 0, 1])
    max_reprojection = 0.01
    min_ray_angle = np.radians(2.0)
    min_depth = 0.001
    valid_triangulation, X = pygeometry.triangulate_bearings_midpoint(
        np.asarray([o1, o2]),
        np.asarray([b1, b2]),
        2 * [max_reprojection],
        min_ray_angle,
        min_depth,
    )
    assert valid_triangulation is False


def test_triangulate_two_bearings_midpoint() -> None:
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])
    b2 = unit_vector([-1.0, 0, 1])
    ok, X = pygeometry.triangulate_two_bearings_midpoint(
        np.asarray([o1, o2]), np.asarray([b1, b2])
    )
    assert ok is True
    assert np.allclose(X, [0, 0, 1.0])


def test_triangulate_two_bearings_midpoint_failed() -> None:
    o1 = np.array([0.0, 0, 0])
    b1 = unit_vector([0.0, 0, 1])
    o2 = np.array([1.0, 0, 0])

    # almost parallel. 1e-5 will make it triangulate again.
    b2 = b1 + np.array([-1e-10, 0, 0])

    ok, X = pygeometry.triangulate_two_bearings_midpoint(
        np.asarray([o1, o2]), np.asarray([b1, b2])
    )
    assert ok is False
