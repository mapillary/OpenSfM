import numpy as np

from opensfm import context
from opensfm import types
from opensfm import pygeometry

"""
Trying to imitate the following structure

reconstruction = {
    "cameras": {
        "theta": {
            "projection_type": "equirectangular"
        }
    },

    "shots" : {
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

    "points" : {
    },
}
"""


def test_reconstruction_class_initialization():

    # Instantiate Reconstruction
    reconstruction = types.Reconstruction()

    # Instantiate camera instrinsics
    camera = types.PerspectiveCamera()
    camera.id = 'apple iphone 4s back camera 4.28mm f/2.4'
    camera.focal = 0.9722222222222222
    camera.k1 = 0.006094395128698237
    camera.k2 = -0.0004952058188617129
    camera.height = 2448
    camera.width = 3264

    # Instantiate GPS data
    metadata = types.ShotMetadata()
    metadata.orientation = 1
    metadata.capture_time = 0.0
    metadata.gps_dop = 5.0
    metadata.gps_position = [1.0815875281451939,
                             -0.96510451436708888,
                             1.2042133903991235]

    # Instantiate shots
    pose0 = types.Pose([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    shot0 = types.Shot()
    shot0.id = 0
    shot0.camera = camera
    shot0.pose = pose0
    shot0.metadata = metadata

    pose1 = types.Pose([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0])

    shot1 = types.Shot()
    shot1.id = 1
    shot1.camera = camera
    shot1.pose = pose1
    shot1.metadata = metadata

    # Add info to current reconstruction
    reconstruction.add_camera(camera)
    reconstruction.add_shot(shot0)
    reconstruction.add_shot(shot1)

    # TEST
    assert len(reconstruction.cameras) == 1
    assert len(reconstruction.shots) == 2
    assert len(reconstruction.points) == 0
    assert reconstruction.get_camera(camera.id) == camera
    assert reconstruction.get_camera(1) is None
    assert reconstruction.get_shot(shot0.id) == shot0
    assert reconstruction.get_shot(shot1.id) == shot1
    assert reconstruction.get_shot(2) is None


def test_perspective_camera_projection():
    """Test perspectiive projection--backprojection loop."""
    for camera in _get_perspective_camera():
        pixel = [0.1, 0.2]
        bearing = camera.pixel_bearing(pixel)
        projected = camera.project(bearing)
        assert np.allclose(pixel, projected)


def test_fisheye_camera_projection():
    """Test fisheye projection--backprojection loop."""
    if not context.OPENCV3:
        return
    for camera in _get_fisheye_camera():
        pixel = [0.1, 0.2]
        bearing = camera.pixel_bearing(pixel)
        projected = camera.project(bearing)
        assert np.allclose(pixel, projected)


def test_dual_camera_projection():
    """Test dual projection--backprojection loop."""
    if not context.OPENCV3:
        return
    for camera in _get_dual_camera():
        pixel = [0.1, 0.2]
        bearing = camera.pixel_bearing(pixel)
        projected = camera.project(bearing)
        assert np.allclose(pixel, projected)


def test_spherical_camera_projection():
    """Test spherical projection--backprojection loop."""
    for camera in _get_spherical_camera():
        pixel = [0.1, 0.2]
        bearing = camera.pixel_bearing(pixel)
        projected = camera.project(bearing)
        assert np.allclose(pixel, projected)


def test_pose_properties():
    """Test pose constructor, getters and setters."""
    p = types.Pose([1, 2, 3], [4, 5, 6])
    assert np.allclose(p.rotation, [1, 2, 3])
    assert type(p.rotation) == np.ndarray
    assert p.rotation.dtype == float
    assert np.allclose(p.translation, [4, 5, 6])
    assert type(p.translation) == np.ndarray
    assert p.translation.dtype == float


def test_pose_inverse():
    p = types.Pose([1, 2, 3], [4, 5, 6])
    inverse = p.inverse()
    identity = p.compose(inverse)
    assert np.allclose(identity.rotation, [0, 0, 0])
    assert np.allclose(identity.translation, [0, 0, 0])


def test_shot_project_back_project():
    pixels = np.array([[0.1, 0.2], [-0.1, 0.2]], dtype=float)
    depths = np.array([1, 2], dtype=float)
    pose = types.Pose([1, 2, 3], [4, 5, 6])
    cameras = [
        _get_perspective_camera(),
        _get_brown_perspective_camera(),
        _get_spherical_camera(),
    ]
    if context.OPENCV3:
        cameras.append(_get_fisheye_camera())

    shot = types.Shot()
    shot.pose = pose
    for pair in cameras:
        for cam in pair:
            shot.camera = cam
            bp_single = [shot.back_project(p,d) for p,d in zip(pixels, depths)]
            bp_many = shot.back_project_many(pixels, depths)
            assert np.allclose(bp_single, bp_many), cam.projection_type

            px_single = [shot.project(p) for p in bp_single]
            px_many = shot.project_many(bp_many)

            assert np.allclose(pixels, px_single), cam.projection_type
            assert np.allclose(pixels, px_many), cam.projection_type


def test_single_vs_many():
    points = np.array([[1, 2, 3], [4, 5, 6]], dtype=float)
    pixels = np.array([[0.1, 0.2], [0.3, 0.4]], dtype=float)
    depths = np.array([1, 2], dtype=float)

    pose = types.Pose([1, 2, 3], [4, 5, 6])
    t_single = [pose.transform(p) for p in points]
    t_many = pose.transform_many(points)
    assert np.allclose(t_single, t_many)

    t_single = [pose.transform_inverse(p) for p in points]
    t_many = pose.transform_inverse_many(points)
    assert np.allclose(t_single, t_many)

    cameras = [
        _get_perspective_camera(),
        _get_brown_perspective_camera(),
        _get_spherical_camera(),
    ]
    if context.OPENCV3:
        cameras.append(_get_fisheye_camera())

    for camera, camera_cpp in cameras:
        p = camera.project_many(points)
        p_cpp = camera_cpp.project_many(points)
        assert np.allclose(p, p_cpp)

        b = camera.pixel_bearing_many(pixels)
        b_cpp = camera_cpp.pixel_bearing_many(pixels)
        assert np.allclose(b, b_cpp)

        if hasattr(camera, 'back_project'):
            q_single = [camera.back_project(p, d)
                        for p, d in zip(pixels, depths)]
            q_many = camera.back_project_many(pixels, depths)
            assert np.allclose(q_single, q_many)


def _get_perspective_camera():
    camera = types.PerspectiveCamera()
    camera.width = 800
    camera.height = 600
    camera.focal = 0.6
    camera.k1 = -0.1
    camera.k2 = 0.01
    camera_cpp = pygeometry.Camera.create_perspective(
        camera.focal, camera.k1, camera.k2)
    return camera, camera_cpp


def _get_brown_perspective_camera():
    camera = types.BrownPerspectiveCamera()
    camera.width = 800
    camera.height = 600
    camera.focal_x = 0.6
    camera.focal_y = 0.7
    camera.c_x = 0.1
    camera.c_y = -0.05
    camera.k1 = -0.1
    camera.k2 = 0.01
    camera.p1 = 0.001
    camera.p2 = 0.002
    camera.k3 = 0.01
    camera_cpp = pygeometry.Camera.create_brown(
        camera.focal_x, camera.focal_y / camera.focal_x,
        [camera.c_x, camera.c_y],
        [camera.k1, camera.k2, camera.k3, camera.p1, camera.p2])
    return camera, camera_cpp


def _get_fisheye_camera():
    camera = types.FisheyeCamera()
    camera.width = 800
    camera.height = 600
    camera.focal = 0.6
    camera.k1 = -0.1
    camera.k2 = 0.01
    camera_cpp = pygeometry.Camera.create_fisheye(
        camera.focal, camera.k1, camera.k2)
    return camera, camera_cpp


def _get_dual_camera():
    camera = types.DualCamera()
    camera.width = 800
    camera.height = 600
    camera.focal = 0.3
    camera.k1 = -0.1
    camera.k2 = 0.01
    camera.transition = 0.5
    camera_cpp = pygeometry.Camera.create_dual(
        camera.transition, camera.focal, camera.k1, camera.k2)
    return camera, camera_cpp


def _get_spherical_camera():
    camera = types.SphericalCamera()
    camera.width = 800
    camera.height = 600
    camera_cpp = pygeometry.Camera.create_spherical()
    return camera, camera_cpp
