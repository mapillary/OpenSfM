# pyre-unsafe
from typing import Optional

import opensfm.synthetic_data.synthetic_scene as ss
from opensfm import geo


def synthetic_circle_scene(
    reference: Optional[geo.TopocentricConverter] = None,
) -> ss.SyntheticStreetScene:
    scene_length = 60
    points_count = 5000
    generator = ss.get_scene_generator("circle", scene_length)
    scene = ss.SyntheticStreetScene(generator, reference)
    scene.add_street(points_count, 7, 7).perturb_floor([0, 0, 0.1]).perturb_walls(
        [0.2, 0.2, 0.01]
    )

    make_regular_scene(scene_length, scene)
    return scene


def synthetic_cube_scene() -> ss.SyntheticCubeScene:
    return ss.SyntheticCubeScene(10, 1000, 0.001)


def synthetic_rig_scene(
    reference: Optional[geo.TopocentricConverter] = None,
) -> ss.SyntheticStreetScene:
    scene_length = 20
    points_count = 5000
    generator = ss.get_scene_generator("line", scene_length)
    scene = ss.SyntheticStreetScene(generator, reference)
    scene.add_street(points_count, 15, 12).perturb_floor([0, 0, 0.1]).perturb_walls(
        [0.2, 0.2, 0.01]
    )
    make_4_cameras_rig_scene(scene_length, scene)
    return scene


def make_4_cameras_rig_scene(
    scene_length: float, scene: ss.SyntheticStreetScene
) -> None:
    camera_height = 2
    camera_interval = 3
    position_perturbation = [0.2, 0.2, 0.01]
    rotation_perturbation = 0.3

    relative_positions = [[0, 0, 0.2], [0, 0, -0.2], [-0.2, 0, 0], [0.2, 0, 0]]
    relative_rotations = [
        [0.0, 0.0, 0.0],
        [0.0, 3.1415927, 0.0],
        [0.0, 1.5707963, 0.0],
        [0.0, -1.5707963, 0.0],
    ]
    camera = ss.get_camera("perspective", "1", 0.7, -0.1, 0.01)
    cameras = [
        camera,
        camera,
        camera,
        camera,
    ]

    scene.add_rig_camera_sequence(
        cameras,
        relative_positions,
        relative_rotations,
        scene_length,
        camera_height,
        camera_interval,
        position_perturbation,
        rotation_perturbation,
    )


def make_regular_scene(scene_length: float, scene: ss.SyntheticStreetScene) -> None:
    camera_height = 1.5
    camera_interval = 3
    position_perturbation = [0.2, 0.2, 0.01]
    rotation_perturbation = 0.2
    camera1 = ss.get_camera("perspective", "1", 0.7, -0.1, 0.01)
    scene.add_camera_sequence(
        camera1,
        scene_length,
        camera_height,
        camera_interval,
        position_perturbation,
        rotation_perturbation,
        None,
    )
