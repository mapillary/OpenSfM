import numpy as np
import functools
import cv2
import synthetic_metrics as metrics

from opensfm import types

from synthetic_generator import ellipse_generator
from synthetic_generator import line_generator
from synthetic_generator import weird_curve

from synthetic_generator import generate_street
from synthetic_generator import generate_cameras

from synthetic_generator import samples_generator_random_count
from synthetic_generator import samples_generator_interval

from synthetic_generator import perturb_points
from synthetic_generator import perturb_rotations

from synthetic_generator import generate_track_data
from synthetic_generator import create_reconstruction


def get_camera(type, id, focal, k1, k2):
    camera = None
    if type == 'perspective':
        camera = types.PerspectiveCamera()
    if type == 'fisheye':
        camera = types.FisheyeCamera()
    camera.id = id
    camera.focal = focal
    camera.k1 = k1
    camera.k2 = k2
    camera.focal_prior = camera.focal
    camera.k1_prior = camera.k1
    camera.k2_prior = camera.k2
    camera.height = 1600
    camera.width = 2000
    return camera


def get_scene_generator(type, length):
    generator = None
    if type == 'ellipse':
        ellipse_ratio = 4
        generator = functools.partial(ellipse_generator, length,
                                      length/ellipse_ratio)
    if type == 'line':
        generator = functools.partial(line_generator, length)
    if type == 'curve':
        generator = functools.partial(weird_curve, length)
    return generator


class SyntheticScene(object):
    def __init__(self, generator):
        self.generator = generator
        self.wall_points = None
        self.floor_points = None
        self.width = None
        self.shot_positions = []
        self.shot_rotations = []
        self.cameras = []

    def add_street(self, points_count, height, width):
        self.wall_points, self.floor_points = generate_street(
            samples_generator_random_count(
                points_count/3), self.generator,
            height, width)
        self.width = width
        return self

    def perturb_walls(self, walls_pertubation):
        perturb_points(self.wall_points, walls_pertubation)
        return self

    def perturb_floor(self, floor_pertubation):
        perturb_points(self.floor_points, floor_pertubation)
        return self

    def add_camera_sequence(self, camera, start, length, height, interval,
                            position_noise=None, rotation_noise=None,
                            gps_noise=None):
        default_noise_interval = 0.25*interval
        positions, rotations = generate_cameras(
            samples_generator_interval(start, length, interval,
                                       default_noise_interval),
            self.generator, height)
        perturb_points(positions, position_noise)
        perturb_rotations(rotations, rotation_noise)
        self.shot_rotations.append(rotations)
        self.shot_positions.append(positions)
        self.cameras.append(camera)
        return self

    def get_reconstruction(self):
        floor_color = [120, 90, 10]
        wall_color = [10, 90, 130]
        return create_reconstruction(
            [self.floor_points, self.wall_points],
            [floor_color, wall_color],
            self.cameras, self.shot_positions,
            self.shot_rotations)

    def get_tracks_data(self, maximum_depth, noise):
        return generate_track_data(self.get_reconstruction(), maximum_depth)

    def compare(self, reconstruction):
        reference = self.get_reconstruction()
        position = metrics.position_errors(reference, reconstruction)
        rotation = metrics.rotation_errors(reference, reconstruction)
        points = metrics.points_errors(reference, reconstruction)
        completeness = metrics.completeness_errors(reference, reconstruction)
        return {'position_average': np.average(position),
                'position_std': np.std(position),
                'rotation_average': np.average(rotation),
                'rotation_std': np.std(rotation),
                'points_average': np.average(points),
                'points_std': np.std(points),
                'ratio_cameras': completeness[0],
                'ratio_points': completeness[1]}



