from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import numpy as np
import networkx as nx
import functools
import math

from opensfm import types
from opensfm import pygeometry

import opensfm.synthetic_data.synthetic_metrics as sm
import opensfm.synthetic_data.synthetic_generator as sg


def get_camera(type, id, focal, k1, k2):
    camera = None
    if type == 'perspective':
        camera = pygeometry.Camera.create_perspective(focal, k1, k2)
    if type == 'fisheye':
        camera = pygeometry.Camera.create_fisheye(focal, k1, k2)
    camera.id = id

    camera.height = 1600
    camera.width = 2000
    return camera


def get_scene_generator(type, length):
    generator = None
    if type == 'circle':
        generator = functools.partial(sg.ellipse_generator, length, length)
    if type == 'ellipse':
        ellipse_ratio = 2
        generator = functools.partial(sg.ellipse_generator, length,
                                      length / ellipse_ratio)
    if type == 'line':
        generator = functools.partial(sg.line_generator, length)
    if type == 'curve':
        generator = functools.partial(sg.weird_curve, length)
    return generator


def normalized(x):
    return x / np.linalg.norm(x)


def camera_pose(position, lookat, up):
    '''
    Pose from position and look at direction

    >>> position = [1.0, 2.0, 3.0]
    >>> lookat = [0., 10.0, 2.0]
    >>> up = [0.0, 0.0, 1.0]
    >>> pose = camera_pose(position, lookat, up)
    >>> np.allclose(pose.get_origin(), position)
    True
    >>> d = normalized(pose.transform(lookat))
    >>> np.allclose(d, [0, 0, 1])
    True
    '''
    ez = normalized(np.array(lookat) - np.array(position))
    ex = normalized(np.cross(ez, up))
    ey = normalized(np.cross(ez, ex))
    pose = types.Pose()
    pose.set_rotation_matrix([ex, ey, ez])
    pose.set_origin(position)
    return pose

class SyntheticScene(object):
    def get_reconstruction(self, rotation_noise=0.0,
                           position_noise=0.0,
                           camera_noise=0.0):
        raise NotImplementedError()

    def get_scene_exifs(self, gps_noise):
        raise NotImplementedError()

    def get_tracks_data(self, maximum_depth, noise):
        raise NotImplementedError()


class SyntheticCubeScene(SyntheticScene):
    """ Scene consisting in of cameras looking at point in a cube. """
    def __init__(self, num_cameras, num_points, noise):
        self.cameras = {}
        for i in range(num_cameras):
            camera = camera = pygeometry.Camera.create_perspective(0.9, -0.1, 0.01)
            camera.id = 'camera%04d' % i
            camera.height = 600
            camera.width = 800
            self.cameras[camera.id] = camera

        self.shots = {}
        r = 2.0
        for i in range(num_cameras):
            phi = np.random.rand()*math.pi
            theta = np.random.rand()*2.0*math.pi
            x = r*np.sin(theta)*np.cos(phi)
            y = r*np.sin(theta)*np.sin(phi)
            z = r*np.cos(theta)
            position = [x, y, z]

            alpha = np.random.rand()
            lookat = [0.0, 0, 0]
            up = [alpha * 0.2, alpha * 0.2, 1.0]

            shot = types.Shot()
            shot.id = 'shot%04d' % i
            shot.camera = self.cameras['camera%04d' % i]
            shot.pose = camera_pose(position, lookat, up)
            self.shots[shot.id] = shot

        points = np.random.rand(num_points, 3)-[0.5, 0.5, 0.5]
        self.points = {}
        for i, p in enumerate(points):
            pt = types.Point()
            pt.id = 'point' + str(i)
            pt.coordinates = p
            pt.color = [100, 100, 20]
            self.points[pt.id] = pt

    def get_reconstruction(self, rotation_noise=0.0,
                           position_noise=0.0,
                           camera_noise=0.0):
        reconstruction = types.Reconstruction()
        reconstruction.shots = self.shots
        reconstruction.points = self.points
        reconstruction.cameras = self.cameras
        return reconstruction

    def get_tracks_data(self, maximum_depth, noise):
        return sg.generate_track_data(self.get_reconstruction(),
                                      maximum_depth, noise)


class SyntheticStreetScene(SyntheticScene):
    """ Scene consisting in a virtual street extruded along some
        parametric shape (line, ellipse), with camera placed along
        the shape.
    """
    def __init__(self, generator):
        self.generator = generator
        self.wall_points = None
        self.floor_points = None
        self.width = None
        self.shot_positions = []
        self.shot_rotations = []
        self.cameras = []

    def add_street(self, points_count, height, width):
        self.wall_points, self.floor_points = sg.generate_street(
            sg.samples_generator_random_count(
                int(points_count // 3)), self.generator,
            height, width)
        self.width = width
        return self

    def perturb_walls(self, walls_pertubation):
        sg.perturb_points(self.wall_points, walls_pertubation)
        return self

    def perturb_floor(self, floor_pertubation):
        sg.perturb_points(self.floor_points, floor_pertubation)
        return self

    def add_camera_sequence(self, camera, start, length, height, interval,
                            position_noise=None, rotation_noise=None,
                            gps_noise=None):
        default_noise_interval = 0.25*interval
        positions, rotations = sg.generate_cameras(
            sg.samples_generator_interval(start, length, interval,
                                          default_noise_interval),
            self.generator, height)
        sg.perturb_points(positions, position_noise)
        sg.perturb_rotations(rotations, rotation_noise)
        self.shot_rotations.append(rotations)
        self.shot_positions.append(positions)
        self.cameras.append(camera)
        return self

    def get_reconstruction(self, rotation_noise=0.0,
                           position_noise=0.0,
                           camera_noise=0.0):
        floor_color = [120, 90, 10]
        wall_color = [10, 90, 130]

        positions = self.shot_positions
        if position_noise != 0.0:
            for p in positions:
                sg.perturb_points(p, position_noise)
        rotations = self.shot_rotations
        if position_noise != 0.0:
            for r in rotations:
                sg.perturb_rotations(r, rotation_noise)
        cameras = self.cameras
        if camera_noise != 0.0:
            for c in cameras:
                c.focal *= (1+camera_noise)

        return sg.create_reconstruction(
            [self.floor_points, self.wall_points],
            [floor_color, wall_color],
            cameras, positions, rotations)

    def get_scene_exifs(self, gps_noise):
        return sg.generate_exifs(self.get_reconstruction(),
                                 gps_noise)

    def get_tracks_data(self, maximum_depth, noise):
        return sg.generate_track_data(self.get_reconstruction(),
                                      maximum_depth, noise)


def compare(reference, reconstruction):
    completeness = sm.completeness_errors(reference, reconstruction)

    absolute_position = sm.position_errors(reference, reconstruction)
    absolute_rotation = sm.rotation_errors(reference, reconstruction)
    absolute_points = sm.points_errors(reference, reconstruction)
    absolute_gps = sm.gps_errors(reconstruction)

    aligned = sm.aligned_to_reference(reference, reconstruction)
    aligned_position = sm.position_errors(reference, aligned)
    aligned_rotation = sm.rotation_errors(reference, aligned)
    aligned_points = sm.points_errors(reference, aligned)
    aligned_gps = sm.gps_errors(aligned)

    return {
        'ratio_cameras': completeness[0],
        'ratio_points': completeness[1],

        'absolute_position_rmse': sm.rmse(absolute_position),
        'absolute_rotation_rmse': sm.rmse(absolute_rotation),
        'absolute_points_rmse': sm.rmse(absolute_points),
        'absolute_gps_rmse': sm.rmse(absolute_gps),

        'aligned_position_rmse': sm.rmse(aligned_position),
        'aligned_rotation_rmse': sm.rmse(aligned_rotation),
        'aligned_points_rmse': sm.rmse(aligned_points),
        'aligned_gps_rmse': sm.rmse(aligned_gps),
    }
