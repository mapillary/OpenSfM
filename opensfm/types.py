# pyre-unsafe
"""Basic types for building a reconstruction."""
from typing import Dict, Optional

import numpy as np
from opensfm import pygeometry, pymap
from opensfm.geo import TopocentricConverter


PANOSHOT_RIG_PREFIX = "panoshot_"


class ShotMesh:
    """Triangular mesh of points visible in a shot

    Attributes:
        vertices: (list of vectors) mesh vertices
        faces: (list of triplets) triangles' topology
    """

    def __init__(self):
        self.vertices = None
        self.faces = None


class Reconstruction:
    """Defines the reconstructed scene.

    Attributes:
      cameras (Dict(Camera)): List of cameras.
      shots (Dict(Shot)): List of reconstructed shots.
      points (Dict(Point)): List of reconstructed points.
      reference (TopocentricConverter): Topocentric reference converter.
    """

    def __init__(self) -> None:
        """Defaut constructor"""
        self._setup_from_map(pymap.Map())

    def _setup_from_map(self, map_obj: pymap.Map):
        self.map = map_obj
        self.camera_view = pymap.CameraView(self.map)
        self.bias_view = pymap.BiasView(self.map)
        self.rig_cameras_view = pymap.RigCameraView(self.map)
        self.rig_instances_view = pymap.RigInstanceView(self.map)
        self.shot_view = pymap.ShotView(self.map)
        self.pano_shot_view = pymap.PanoShotView(self.map)
        self.landmark_view = pymap.LandmarkView(self.map)

    def __repr__(self):
        return (
            "<Reconstruction"
            f" cameras={len(self.cameras)}"
            f" shots={len(self.shots)}"
            f" points={len(self.points)}"
            f" rig_cameras={len(self.rig_cameras)}"
            f" rig_instances={len(self.rig_instances)}"
            ">"
        )

    def get_cameras(self) -> pymap.CameraView:
        return self.camera_view

    def set_cameras(self, value: Dict[str, pygeometry.Camera]) -> None:
        for cam in value.values():
            self.map.create_camera(cam)

    cameras = property(get_cameras, set_cameras)

    def get_biases(self) -> pymap.BiasView:
        return self.bias_view

    def set_biases(self, value: Dict[str, pygeometry.Similarity]) -> None:
        for cam_id, bias in value.items():
            self.map.set_bias(cam_id, bias)

    def set_bias(self, cam_id: str, bias: pygeometry.Similarity) -> None:
        self.map.set_bias(cam_id, bias)

    biases = property(get_biases, set_biases)

    def get_rig_cameras(self) -> pymap.RigCameraView:
        return self.rig_cameras_view

    def set_rig_cameras(self, values: Dict[str, pymap.RigCamera]) -> None:
        for rig_camera in values.values():
            self.map.create_rig_camera(rig_camera)

    rig_cameras = property(get_rig_cameras, set_rig_cameras)

    def get_rig_instances(self) -> pymap.RigInstanceView:
        return self.rig_instances_view

    def set_rig_instances(self, values: Dict[str, pymap.RigInstance]) -> None:
        for rig_instance in values.values():
            self.add_rig_instance(rig_instance)

    def remove_rig_instance(self, rig_instance_id: str) -> None:
        self.map.remove_rig_instance(rig_instance_id)

    rig_instances = property(get_rig_instances, set_rig_instances)

    def get_shots(self) -> pymap.ShotView:
        return self.shot_view

    def set_shots(self, value: Dict[str, pymap.Shot]) -> None:
        for shot in value.values():
            self.add_shot(shot)

    shots = property(get_shots, set_shots)

    def get_pano_shots(self) -> pymap.PanoShotView:
        return self.pano_shot_view

    def set_pano_shots(self, value: Dict[str, pymap.Shot]) -> None:
        for shot in value.values():
            self.add_pano_shot(shot)

    pano_shots = property(get_pano_shots, set_pano_shots)

    def get_points(self) -> pymap.LandmarkView:
        return self.landmark_view

    def set_points(self, value: Dict[str, pymap.Landmark]) -> None:
        self.map.clear_observations_and_landmarks()
        for point in value.values():
            self.add_point(point)

    def remove_point(self, point_id: str) -> None:
        self.map.remove_landmark(point_id)

    points = property(get_points, set_points)

    def get_reference(self) -> TopocentricConverter:
        ref = self.map.get_reference()
        return TopocentricConverter(ref.lat, ref.lon, ref.alt)

    def set_reference(self, value: TopocentricConverter) -> None:
        self.map.set_reference(value.lat, value.lon, value.alt)

    reference = property(get_reference, set_reference)

    # Cameras
    def add_camera(self, camera: pygeometry.Camera) -> pygeometry.Camera:
        """Add a camera in the list

        :param camera: The camera.
        """
        if camera.id not in self.cameras:
            return self.map.create_camera(camera)
        else:
            return self.get_camera(camera.id)

    def get_camera(self, id: str) -> pygeometry.Camera:
        """Return a camera by id.

        :return: If exists returns the camera, otherwise None.
        """
        return self.cameras.get(id)

    # Rigs
    def add_rig_camera(self, rig_camera: pymap.RigCamera) -> pymap.RigCamera:
        """Add a rig camera in the list

        :param rig_camera: The rig camera.
        """
        if rig_camera.id not in self.rig_cameras:
            return self.map.create_rig_camera(rig_camera)
        else:
            return self.rig_cameras.get(rig_camera.id)

    def add_rig_instance(self, rig_instance: pymap.RigInstance) -> pymap.RigInstance:
        """Creates a copy of the passed rig instance
        in the current reconstruction"""

        for camera in rig_instance.rig_cameras.values():
            if camera.id not in self.rig_cameras:
                self.map.create_rig_camera(camera)
        in_any_instance = any(
            (set(rig_instance.shots) & set(ri.shots))
            for ri in self.rig_instances.values()
        )
        if in_any_instance:
            raise RuntimeError("Shots already exist in another instance")

        if rig_instance.id not in self.rig_instances:
            self.map.create_rig_instance(rig_instance.id)
        return self.map.update_rig_instance(rig_instance)

    # Shot
    def create_shot(
        self,
        shot_id: str,
        camera_id: str,
        pose: Optional[pygeometry.Pose] = None,
        rig_camera_id: Optional[str] = None,
        rig_instance_id: Optional[str] = None,
    ) -> pymap.Shot:
        passed_rig_camera_id = rig_camera_id if rig_camera_id else camera_id
        passed_rig_instance_id = rig_instance_id if rig_instance_id else shot_id

        if (not rig_camera_id) and (camera_id not in self.rig_cameras):
            self.add_rig_camera(pymap.RigCamera(pygeometry.Pose(), camera_id))
        if passed_rig_camera_id not in self.rig_cameras:
            raise RuntimeError(
                f"Rig Camera {passed_rig_camera_id} doesn't exist in reconstruction"
            )

        if (not rig_instance_id) and (shot_id not in self.rig_instances):
            self.add_rig_instance(pymap.RigInstance(shot_id))
        if passed_rig_instance_id not in self.rig_instances:
            raise RuntimeError(
                f"Rig Instance {passed_rig_instance_id} doesn't exist in reconstruction"
            )

        if pose is None:
            created_shot = self.map.create_shot(
                shot_id, camera_id, passed_rig_camera_id, passed_rig_instance_id
            )
        else:
            created_shot = self.map.create_shot(
                shot_id, camera_id, passed_rig_camera_id, passed_rig_instance_id, pose
            )

        return created_shot

    def add_shot(self, shot: pymap.Shot) -> pymap.Shot:
        """Creates a copy of the passed shot in the current reconstruction
        If the shot belong to a Rig, we recursively copy the entire rig
        instance, so rigs stay consistents.
        """

        if shot.camera.id not in self.cameras:
            self.add_camera(shot.camera)
        if shot.rig_instance_id not in self.rig_instances:
            self.map.create_rig_instance(shot.rig_instance_id)
        if shot.rig_camera_id not in self.rig_cameras:
            self.map.create_rig_camera(shot.rig_camera)
        if shot.id not in self.shots:
            self.map.create_shot(
                shot.id,
                shot.camera.id,
                shot.rig_camera_id,
                shot.rig_instance_id,
                shot.pose,
            )
        ret = self.map.update_shot(shot)
        return ret

    def get_shot(self, id: str) -> pymap.Shot:
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.shots.get(id)

    def remove_shot(self, shot_id: str) -> None:
        self.map.remove_shot(shot_id)

    # PanoShot
    def create_pano_shot(
        self, shot_id: str, camera_id: str, pose: Optional[pygeometry.Pose] = None
    ) -> pymap.Shot:
        if pose is None:
            pose = pygeometry.Pose()

        rig_camera_id = f"{PANOSHOT_RIG_PREFIX}{camera_id}"
        if rig_camera_id not in self.rig_cameras:
            self.add_rig_camera(pymap.RigCamera(pygeometry.Pose(), rig_camera_id))
        rig_instance_id = f"{PANOSHOT_RIG_PREFIX}{shot_id}"
        if rig_instance_id not in self.rig_instances:
            self.add_rig_instance(pymap.RigInstance(rig_instance_id))

        created_shot = self.map.create_pano_shot(
            shot_id, camera_id, rig_camera_id, rig_instance_id, pose
        )

        return created_shot

    def add_pano_shot(self, pshot: pymap.Shot) -> pymap.Shot:
        if pshot.camera.id not in self.cameras:
            self.add_camera(pshot.camera)
        if pshot.id not in self.pano_shots:
            self.create_pano_shot(pshot.id, pshot.camera.id, pshot.pose)
        return self.map.update_pano_shot(pshot)

    def get_pano_shot(self, id: str) -> pymap.Shot:
        """Return a shot by id.

        :return: If exists returns the shot, otherwise None.
        """
        return self.pano_shots.get(id)

    def remove_pano_shot(self, shot_id: str) -> None:
        self.map.remove_pano_shot(shot_id)

    def create_point(
        self, point_id: str, coord: Optional[np.ndarray] = None
    ) -> pymap.Landmark:
        if coord is None:
            return self.map.create_landmark(point_id, np.array([0, 0, 0]))
        return self.map.create_landmark(point_id, coord)

    def add_point(self, point: pymap.Landmark) -> pymap.Landmark:
        """Add a point in the list

        :param point: The point.
        """
        if point.coordinates is None:
            new_pt = self.map.create_landmark(point.id, np.array([0, 0, 0]))
        else:
            new_pt = self.map.create_landmark(point.id, point.coordinates)
        if point.color is not None:
            new_pt.color = point.color
        return new_pt

    def get_point(self, id: str) -> pymap.Landmark:
        """Return a point by id.

        :return: If exists returns the point, otherwise None.
        """
        return self.points.get(id)

    def add_observation(
        self, shot_id: str, lm_id: str, observation: pymap.Observation
    ) -> None:
        """Adds an observation between a shot and a landmark
        :param shot_id: The id of the shot
        :param lm_id: The id of the landmark
        :param observation: The observation
        """
        self.map.add_observation(shot_id, lm_id, observation)

    def remove_observation(self, shot_id: str, lm_id: str) -> None:
        self.map.remove_observation(shot_id, lm_id)

    def __deepcopy__(self, d):
        rec_cpy = Reconstruction()

        copy_observations = False
        # Check if we also need the observations
        if "copy_observations" in d:
            copy_observations = d["copy_observations"]

        map_copy = pymap.Map.deep_copy(self.map, copy_observations)
        rec_cpy._setup_from_map(map_copy)

        return rec_cpy

    def add_correspondences_from_tracks_manager(
        self, tracks_manager: pymap.TracksManager
    ) -> None:
        for track_id in tracks_manager.get_track_ids():
            if track_id not in self.points:
                continue
            track_obs = tracks_manager.get_track_observations(track_id)
            for shot_id in track_obs.keys():
                if shot_id in self.shots:
                    observation = tracks_manager.get_observation(shot_id, track_id)
                    self.add_observation(shot_id, track_id, observation)
