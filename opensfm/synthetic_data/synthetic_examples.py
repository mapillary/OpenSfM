from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import opensfm.synthetic_data.synthetic_scene as ss


def synthetic_circle_scene():
    scene_length = 60
    points_count = 5000
    generator = ss.get_scene_generator('circle', scene_length)
    scene = ss.SyntheticStreetScene(generator)
    scene.add_street(points_count, 7, 7).perturb_floor([0, 0, 0.1]).\
        perturb_walls([0.2, 0.2, 0.01])

    camera_height = 1.5
    camera_interval = 3
    position_perturbation = [0.2, 0.2, 0.01]
    rotation_perturbation = 0.2
    camera = ss.get_camera('perspective', '1', 0.9, -0.1, 0.01)
    scene.add_camera_sequence(camera, 0, scene_length,
                              camera_height, camera_interval,
                              position_perturbation,
                              rotation_perturbation)
    return scene


def synthetic_cube_scene():
    return ss.SyntheticCubeScene(10, 1000, 0.001)


def synthetic_small_line_scene():
    scene_length = 15
    points_count = 5000
    generator = ss.get_scene_generator('line', scene_length)
    scene = ss.SyntheticStreetScene(generator)
    scene.add_street(points_count, 7, 7).perturb_floor([0, 0, 0.1]).\
        perturb_walls([0.2, 0.2, 0.01])

    camera_height = 1.5
    camera_interval = 3
    position_perturbation = [0.2, 0.2, 0.01]
    rotation_perturbation = 0.3
    camera = ss.get_camera('perspective', '1', 0.9, -0.1, 0.01)
    scene.add_camera_sequence(camera, 0, scene_length,
                              camera_height, camera_interval,
                              position_perturbation,
                              rotation_perturbation)
    return scene
