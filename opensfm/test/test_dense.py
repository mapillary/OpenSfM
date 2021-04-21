import numpy as np
from opensfm import dense
from opensfm import pygeometry
from opensfm import types


def test_angle_between_points():
    origin = [0, 0, 0]
    p1 = [1, 0, 0]
    p2 = [0, 1, 0]

    res = dense.angle_between_points(origin, p1, p2)

    assert np.allclose(res, np.pi / 2)

    origin = [10, 15, 20]
    p1 = [10, 16, 20]
    p2 = [10, 16, 21]

    res = dense.angle_between_points(origin, p1, p2)

    assert np.allclose(res, np.pi / 4)


def test_depthmap_to_ply():
    height, width = 2, 3

    camera = pygeometry.Camera.create_perspective(0.8, 0.0, 0.0)
    camera.id = "cam1"
    camera.height = height
    camera.width = width
    r = types.Reconstruction()
    r.add_camera(camera)
    shot = r.create_shot(
        "shot1", camera.id, pygeometry.Pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    )

    image = np.zeros((height, width, 3))
    depth = np.ones((height, width))

    ply = dense.depthmap_to_ply(shot, depth, image)
    assert len(ply.splitlines()) == 16
