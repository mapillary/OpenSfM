"""Basic types for building a reconstruction."""

import numpy as np
from opensfm import pymap
from opensfm import pygeometry
import cv2
import math


class ShotMesh(object):
    """Triangular mesh of points visible in a shot

    Attributes:
        vertices: (list of vectors) mesh vertices
        faces: (list of triplets) triangles' topology
    """

    def __init__(self):
        self.vertices = None
        self.faces = None


class GroundControlPoint(object):
    """A ground control point with its observations.

    Attributes:
        lla: latitue, longitude and altitude
        coordinates: x, y, z coordinates in topocentric reference frame
        has_altitude: true if z coordinate is known
        observations: list of observations of the point on images
    """

    def __init__(self):
        self.id = None
        self.lla = None
        self.coordinates = None
        self.has_altitude = None
        self.observations = []


class GroundControlPointObservation(object):
    """A ground control point observation.

    Attributes:
        shot_id: the shot where the point is observed
        projection: 2d coordinates of the observation
    """

    def __init__(self):
        self.shot_id = None
        self.projection = None


class Reconstruction(object):
    """Defines the reconstructed scene.

    Attributes:
      cameras (Dict(Camera)): List of cameras.
      shots (Dict(Shot)): List of reconstructed shots.
      points (Dict(Point)): List of reconstructed points.
      reference (TopocentricConverter): Topocentric reference converter.
    """

    def __init__(self):
        """Defaut constructor"""
        self.map = pymap.Map()

    def get_cameras(self):
        return pymap.CameraView(self.map)

    def set_cameras(self, value):
        for cam in value.values():
            self.map.create_camera(cam)

    cameras = property(get_cameras, set_cameras)

    def get_shots(self):
        return pymap.ShotView(self.map)

    def set_shots(self, value):
        for shot in value.values():
            self.add_shot(shot)

    shots = property(get_shots, set_shots)

    def get_points(self):
        return pymap.LandmarkView(self.map)

    def set_points(self, value):
        self.map.clear_observations_and_landmarks()
        for point in value.values():
            self.add_point(point)

    points = property(get_points, set_points)

    def get_reference(self):
        return self.map.get_reference()

    def set_reference(self, value):
        self.map.set_reference(value.lat, value.lon, value.alt)

    reference = property(get_reference, set_reference)

    def add_camera(self, camera):
        """Add a camera in the list

        :param camera: The camera.
        """
        self.map.create_camera(camera)

    def get_camera(self, id):
        """Return a camera by id.

        :return: If exists returns the camera, otherwise None.
        """
        return self.cameras.get(id)

    def create_shot(self, shot_id, camera_id, pose=pygeometry.Pose()):
        return self.map.create_shot(shot_id, camera_id, pose)

    def add_shot(self, shot):
        """Add a shot in the list

        :param shot: The shot.
        """

        pose = pygeometry.Pose()
        if shot.pose is not None:
            pose.set_from_world_to_cam(
                shot.pose.rotation, shot.pose.translation)
        map_shot = self.map.create_shot(shot.id, shot.camera.id, pose)
        if shot.metadata is not None:
            try:  # Ugly handling of both pymap.Shot and types.Shot
                map_shot.metadata = shot.metadata
            except TypeError:
                shot.metadata.add_to_map_shot(map_shot)

    def get_shot(self, id):
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.shots.get(id)

    def create_point(self, point_id, coord = [0, 0, 0]):
        return self.map.create_landmark(point_id, coord) 

    def add_point(self, point):
        """Add a point in the list

        :param point: The point.
        """
        if point.coordinates is None:
            new_pt = self.map.create_landmark(point.id, [0, 0, 0])
        else:
            new_pt = self.map.create_landmark(point.id, point.coordinates)
        if point.color is not None:
            new_pt.color = point.color

    def get_point(self, id):
        """Return a point by id.

        :return: If exists returns the point, otherwise None.
        """
        return self.points.get(id)

    def add_observation(self, shot_id, lm_id, observation):
        """ Adds an observation between a shot and a landmark
        :param shot_id: The id of the shot
        :param lm_id: The id of the landmark
        :param observation: The observation
        """
        self.map.add_observation(shot_id, lm_id, observation)
