"""Basic types for building a reconstruction."""
from opensfm import pygeometry
from opensfm import pymap
from opensfm.geo import TopocentricConverter


class ShotMesh(object):
    """Triangular mesh of points visible in a shot

    Attributes:
        vertices: (list of vectors) mesh vertices
        faces: (list of triplets) triangles' topology
    """

    def __init__(self):
        self.vertices = None
        self.faces = None


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
        self.camera_view = pymap.CameraView(self.map)
        self.rig_models_view = pymap.RigModelView(self.map)
        self.rig_instances_view = pymap.RigInstanceView(self.map)
        self.shot_view = pymap.ShotView(self.map)
        self.pano_shot_view = pymap.PanoShotView(self.map)
        self.landmark_view = pymap.LandmarkView(self.map)

    def get_cameras(self):
        return self.camera_view

    def set_cameras(self, value):
        for cam in value.values():
            self.map.create_camera(cam)

    cameras = property(get_cameras, set_cameras)

    def get_rig_models(self):
        return self.rig_models_view

    def set_rig_models(self, values):
        for rig_model in values.values():
            self.map.create_rig_model(rig_model)

    rig_models = property(get_rig_models, set_rig_models)

    def get_rig_instances(self):
        return self.rig_instances_view

    def set_rig_instances(self, values):
        for rig_instance in values:
            self.add_rig_instance(rig_instance)

    rig_instances = property(get_rig_instances, set_rig_instances)

    def get_shots(self):
        return self.shot_view

    def set_shots(self, value):
        for shot in value.values():
            self.add_shot(shot)

    shots = property(get_shots, set_shots)

    def get_pano_shots(self):
        return self.pano_shot_view

    def set_pano_shots(self, value):
        for shot in value.values():
            self.add_pano_shot(shot)

    pano_shots = property(get_pano_shots, set_pano_shots)

    def get_points(self):
        return self.landmark_view

    def set_points(self, value):
        self.map.clear_observations_and_landmarks()
        for point in value.values():
            self.add_point(point)

    def remove_point(self, point_id):
        self.map.remove_landmark(point_id)

    points = property(get_points, set_points)

    def get_reference(self):
        ref = self.map.get_reference()
        return TopocentricConverter(ref.lat, ref.lon, ref.alt)

    def set_reference(self, value):
        self.map.set_reference(value.lat, value.lon, value.alt)

    reference = property(get_reference, set_reference)

    # Cameras
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

    # Rigs
    def add_rig_model(self, rig_model):
        """Add a rig_model in the list

        :param rig_model: The rig_model.
        """
        return self.map.create_rig_model(rig_model)

    def add_rig_instance(self, rig_instance):
        """Creates a copy of the passed rig instance
        in the current reconstruction"""

        if rig_instance.rig_model.id not in self.rig_models:
            self.map.create_rig_model(rig_instance.rig_model)
        in_any_instance = any(
            (set(rig_instance.shots) & set(ri.shots))
            for ri in self.rig_instances.values()
        )
        if in_any_instance:
            raise RuntimeError("Shots already exist in another instance")

        if rig_instance.id not in self.rig_instances:
            self.map.create_rig_instance(
                rig_instance.rig_model, rig_instance.id, rig_instance.camera_ids
            )
        return self.map.update_rig_instance(rig_instance)

    # Shot
    def create_shot(self, shot_id, camera_id, pose=None):
        if pose is None:
            pose = pygeometry.Pose()
        return self.map.create_shot(shot_id, camera_id, pose)

    def add_shot(self, shot):
        """Creates a copy of the passed shot
        in the current reconstruction"""

        if shot.camera.id not in self.cameras:
            self.add_camera(shot.camera)
        if shot.id not in self.shots:
            self.create_shot(shot.id, shot.camera.id, shot.pose)
        return self.map.update_shot(shot)

    def get_shot(self, id):
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.shots.get(id)

    def remove_shot(self, shot_id):
        self.map.remove_shot(shot_id)

    # PanoShot
    def create_pano_shot(self, shot_id, camera_id, pose=None):
        if pose is None:
            pose = pygeometry.Pose()
        return self.map.create_pano_shot(shot_id, camera_id, pose)

    def add_pano_shot(self, pshot):
        if pshot.camera.id not in self.cameras:
            self.add_camera(pshot.camera)
        if pshot.id not in self.pano_shots:
            self.create_pano_shot(pshot.id, pshot.camera.id, pshot.pose)
        return self.map.update_pano_shot(pshot)

    def get_pano_shot(self, id):
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.pano_shots.get(id)

    def remove_pano_shot(self, shot_id):
        self.map.remove_pano_shot(shot_id)

    def create_point(self, point_id, coord=(0, 0, 0)):
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

    def add_observation(self, shot_id, lm_id, observation):
        """Adds an observation between a shot and a landmark
        :param shot_id: The id of the shot
        :param lm_id: The id of the landmark
        :param observation: The observation
        """
        self.map.add_observation(shot_id, lm_id, observation)

    def remove_observation(self, shot_id, lm_id):
        self.map.remove_observation(shot_id, lm_id)

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

        # Copy the pano shots
        for shot in self.pano_shots.values():
            rec_cpy.add_pano_shot(shot)

        # Copy the points
        for point in self.points.values():
            rec_cpy.add_point(point)
            if copy_observations:
                for shot, obs_id in point.get_observations().items():
                    obs = shot.get_observation(obs_id)
                    rec_cpy.add_observation(shot.id, point.id, obs)

        return rec_cpy

    def add_correspondences_from_tracks_manager(self, tracks_manager):
        for track_id in self.points.keys():
            track_obs = tracks_manager.get_track_observations(track_id)
            for shot_id in track_obs.keys():
                if shot_id in self.shots:
                    observation = tracks_manager.get_observation(shot_id, track_id)
                    self.add_observation(shot_id, track_id, observation)
