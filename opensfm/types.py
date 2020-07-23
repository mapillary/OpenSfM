"""Basic types for building a reconstruction."""

import numpy as np
from opensfm import pymap
from opensfm import pygeometry
import cv2
import math
import copy

class ShotMesh(object):
    """Triangular mesh of points visible in a shot

    Attributes:
        vertices: (list of vectors) mesh vertices
        faces: (list of triplets) triangles' topology
    """

    def __init__(self):
        self.vertices = None
        self.faces = None

class Camera(object):
    """Abstract camera class.
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

    def get_pano_shots(self):
        return pymap.PanoShotView(self.map)

    def set_pano_shots(self, value):
        for shot in value.values():
            self.add_pano_shot(shot)

    pano_shots = property(get_pano_shots, set_pano_shots)

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
        return self.map.create_camera(camera)

    def get_camera(self, id):
        """Return a camera by id.

        :return: If exists returns the camera, otherwise None.
        """
        return self.cameras.get(id)

    # Shot
    def create_shot(self, shot_id, camera_id, pose=pygeometry.Pose()):
        return self.map.create_shot(shot_id, camera_id, pose)

    def add_shot(self, shot):
        """Creates a copy of the passed shot
            in the current reconstruction"""

        if shot.camera.id not in self.cameras:
            self.add_camera(shot.camera)
        return self.map.add_shot(shot)

    def get_shot(self, id):
        """Return a shot by id.
        :return: If exists returns the shot, otherwise None.
        """
        return self.shots.get(id)

    # PanoShot
    def create_pano_shot(self, shot_id, camera_id, pose=pygeometry.Pose()):
        return self.map.create_pano_shot(shot_id, camera_id, pose)

    def add_pano_shot(self, pshot):
        if pshot.camera.id not in self.cameras:
            self.add_camera(pshot.camera)
        return self.map.add_pano_shot(pshot)

    def get_pano_shot(self, id):
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.pano_shots.get(id)

    def create_point(self, point_id, coord=[0, 0, 0]):
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
        return new_pt

    def get_point(self, id):
        """Return a point by id.

        :return: If exists returns the point, otherwise None.
        """
        return self.points.get(id)

    def remove_point(self, id):
        self.map.remove_landmark(id)

    def add_observation(self, shot_id, lm_id, observation):
        """ Adds an observation between a shot and a landmark
        :param shot_id: The id of the shot
        :param lm_id: The id of the landmark
        :param observation: The observation
        """
        self.map.add_observation(shot_id, lm_id, observation)

    def __deepcopy__(self, d):
        # create new reconstruction
        rec_cpy = Reconstruction()
        copy_observations = False
        # Check if we also need the observations
        if "copy_observations" in d:
            copy_observations = d["copy_observations"]

        # Copy the cameras
        rec_cpy.cameras = self.cameras

        # Copy the shots
        for shot in self.shots.values():
            rec_cpy.add_shot(shot)

        # Copy the points
        for point in self.points.values():
            rec_cpy.add_point(point)
            if copy_observations:
                for shot, obs_id in shot.get_observations():
                    obs = shot.get_observation(obs_id)
                    rec_cpy.add_observation(shot.id, point.id, obs)

        return rec_cpy
