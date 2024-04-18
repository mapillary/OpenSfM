# pyre-unsafe
import json
import logging
import os
import shutil
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Union, Dict, Any, Iterable, List, IO, Tuple, TextIO, Optional

import cv2
import numpy as np
import pyproj
from numpy import ndarray
from opensfm import context, features, geo, pygeometry, pymap, types
from PIL import Image

logger: logging.Logger = logging.getLogger(__name__)


def camera_from_json(key: str, obj: Dict[str, Any]) -> pygeometry.Camera:
    """
    Read camera from a json object
    """
    camera = None
    pt = obj.get("projection_type", "perspective")
    if pt == "perspective":
        camera = pygeometry.Camera.create_perspective(
            obj["focal"], obj.get("k1", 0.0), obj.get("k2", 0.0)
        )
    elif pt == "brown":
        camera = pygeometry.Camera.create_brown(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            np.array(
                [
                    obj.get("k1", 0.0),
                    obj.get("k2", 0.0),
                    obj.get("k3", 0.0),
                    obj.get("p1", 0.0),
                    obj.get("p2", 0.0),
                ]
            ),
        )
    elif pt == "fisheye":
        camera = pygeometry.Camera.create_fisheye(
            obj["focal"], obj.get("k1", 0.0), obj.get("k2", 0.0)
        )
    elif pt == "fisheye_opencv":
        camera = pygeometry.Camera.create_fisheye_opencv(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            np.array(
                [
                    obj.get("k1", 0.0),
                    obj.get("k2", 0.0),
                    obj.get("k3", 0.0),
                    obj.get("k4", 0.0),
                ]
            ),
        )
    elif pt == "fisheye62":
        camera = pygeometry.Camera.create_fisheye62(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            np.array(
                [
                    obj.get("k1", 0.0),
                    obj.get("k2", 0.0),
                    obj.get("k3", 0.0),
                    obj.get("k4", 0.0),
                    obj.get("k5", 0.0),
                    obj.get("k6", 0.0),
                    obj.get("p1", 0.0),
                    obj.get("p2", 0.0),
                ]
            ),
        )
    elif pt == "fisheye624":
        camera = pygeometry.Camera.create_fisheye624(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            np.array(
                [
                    obj.get("k1", 0.0),
                    obj.get("k2", 0.0),
                    obj.get("k3", 0.0),
                    obj.get("k4", 0.0),
                    obj.get("k5", 0.0),
                    obj.get("k6", 0.0),
                    obj.get("p1", 0.0),
                    obj.get("p2", 0.0),
                    obj.get("s0", 0.0),
                    obj.get("s1", 0.0),
                    obj.get("s2", 0.0),
                    obj.get("s3", 0.0),
                ]
            ),
        )
    elif pt == "radial":
        camera = pygeometry.Camera.create_radial(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            np.array(
                [
                    obj.get("k1", 0.0),
                    obj.get("k2", 0.0),
                ]
            ),
        )
    elif pt == "simple_radial":
        camera = pygeometry.Camera.create_simple_radial(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            np.array([obj.get("c_x", 0.0), obj.get("c_y", 0.0)]),
            obj.get("k1", 0.0),
        )
    elif pt == "dual":
        camera = pygeometry.Camera.create_dual(
            obj.get("transition", 0.5),
            obj["focal"],
            obj.get("k1", 0.0),
            obj.get("k2", 0.0),
        )
    elif pygeometry.Camera.is_panorama(pt):
        camera = pygeometry.Camera.create_spherical()
    else:
        raise NotImplementedError
    camera.id = key
    camera.width = int(obj.get("width", 0))
    camera.height = int(obj.get("height", 0))
    return camera


def pose_from_json(obj: Dict[str, Any]) -> pygeometry.Pose:
    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    if "translation" in obj:
        pose.translation = obj["translation"]
    return pose


def bias_from_json(obj: Dict[str, Any]) -> pygeometry.Similarity:
    return pygeometry.Similarity(obj["rotation"], obj["translation"], obj["scale"])


def assign_shot_attributes(obj: Dict[str, Any], shot: pymap.Shot) -> None:
    shot.metadata = json_to_pymap_metadata(obj)
    if "scale" in obj:
        shot.scale = obj["scale"]
    if "covariance" in obj:
        shot.covariance = np.array(obj["covariance"])
    if "merge_cc" in obj:
        shot.merge_cc = obj["merge_cc"]
    if "vertices" in obj and "faces" in obj:
        shot.mesh.vertices = obj["vertices"]
        shot.mesh.faces = obj["faces"]


def shot_in_reconstruction_from_json(
    reconstruction: types.Reconstruction,
    key: str,
    obj: Dict[str, Any],
    rig_instance_id: Optional[str] = None,
    rig_camera_id: Optional[str] = None,
    is_pano_shot: bool = False,
) -> pymap.Shot:
    """
    Read shot from a json object and append it to a reconstruction
    """
    pose = pose_from_json(obj)

    if is_pano_shot:
        shot = reconstruction.create_pano_shot(key, obj["camera"], pose)
    else:
        shot = reconstruction.create_shot(
            key, obj["camera"], pose, rig_camera_id, rig_instance_id
        )
    assign_shot_attributes(obj, shot)
    return shot


def single_shot_from_json(
    key: str, obj: Dict[str, Any], camera: pygeometry.Camera
) -> pymap.Shot:
    """
    Read shot from a json object
    """
    pose = pose_from_json(obj)
    shot = pymap.Shot(key, camera, pose)
    assign_shot_attributes(obj, shot)
    return shot


def point_from_json(
    reconstruction: types.Reconstruction, key: str, obj: Dict[str, Any]
) -> pymap.Landmark:
    """
    Read a point from a json object
    """
    point = reconstruction.create_point(key, obj["coordinates"])
    point.color = obj["color"]
    return point


def rig_camera_from_json(key: str, obj: Dict[str, Any]) -> pymap.RigCamera:
    """
    Read a rig cameras from a json object
    """
    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    pose.translation = obj["translation"]
    rig_camera = pymap.RigCamera(pose, key)
    return rig_camera


def rig_cameras_from_json(obj: Dict[str, Any]) -> Dict[str, pymap.RigCamera]:
    """
    Read rig cameras from a json object
    """
    rig_cameras = {}
    for key, value in obj.items():
        rig_cameras[key] = rig_camera_from_json(key, value)
    return rig_cameras


def rig_instance_from_json(
    reconstruction: types.Reconstruction, instance_id: str, obj: Dict[str, Any]
) -> None:
    """
    Read any rig instance from a json shot object
    """
    reconstruction.add_rig_instance(pymap.RigInstance(instance_id))

    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    pose.translation = obj["translation"]
    reconstruction.rig_instances[instance_id].pose = pose


def rig_instance_camera_per_shot(obj: Dict[str, Any]) -> Dict[str, Tuple[str, str]]:
    """
    Given JSON root data, return (rig_instance_id, rig_camera_id) per shot.
    """
    panoshots = set(obj["pano_shots"].keys()) if "pano_shots" in obj else {}
    rig_shots = {}
    if "rig_instances" in obj:
        rig_shots = {
            s_key: (i_key, c_key)
            for i_key, ri in obj["rig_instances"].items()
            for s_key, c_key in ri["rig_camera_ids"].items()
            if s_key not in panoshots
        }
    return rig_shots


def reconstruction_from_json(obj: Dict[str, Any]) -> types.Reconstruction:
    """
    Read a reconstruction from a json object
    """
    reconstruction = types.Reconstruction()

    # Extract cameras
    for key, value in obj["cameras"].items():
        camera = camera_from_json(key, value)
        reconstruction.add_camera(camera)

    # Extract camera biases
    if "biases" in obj:
        for key, value in obj["biases"].items():
            transform = bias_from_json(value)
            reconstruction.set_bias(key, transform)

    # Extract rig models
    if "rig_cameras" in obj:
        for key, value in obj["rig_cameras"].items():
            reconstruction.add_rig_camera(rig_camera_from_json(key, value))

    # Extract rig instances from shots
    if "rig_instances" in obj:
        for key, value in obj["rig_instances"].items():
            rig_instance_from_json(reconstruction, key, value)

    # Extract shots
    rig_shots = rig_instance_camera_per_shot(obj)
    for key, value in obj["shots"].items():
        shot_in_reconstruction_from_json(
            reconstruction,
            key,
            value,
            rig_camera_id=rig_shots[key][1] if key in rig_shots else None,
            rig_instance_id=rig_shots[key][0] if key in rig_shots else None,
            is_pano_shot=False,
        )

    # Extract points
    if "points" in obj:
        for key, value in obj["points"].items():
            point_from_json(reconstruction, key, value)

    # Extract pano_shots
    if "pano_shots" in obj:
        for key, value in obj["pano_shots"].items():
            shot_in_reconstruction_from_json(
                reconstruction, key, value, is_pano_shot=True
            )

    # Extract reference topocentric frame
    if "reference_lla" in obj:
        lla = obj["reference_lla"]
        reconstruction.reference = geo.TopocentricConverter(
            lla["latitude"], lla["longitude"], lla["altitude"]
        )

    return reconstruction


def reconstructions_from_json(obj: List[Dict[str, Any]]) -> List[types.Reconstruction]:
    """
    Read all reconstructions from a json object
    """
    return [reconstruction_from_json(i) for i in obj]


def cameras_from_json(obj: Dict[str, Any]) -> Dict[str, pygeometry.Camera]:
    """
    Read cameras from a json object
    """
    cameras = {}
    for key, value in obj.items():
        cameras[key] = camera_from_json(key, value)
    return cameras


def camera_to_json(camera) -> Dict[str, Any]:
    """
    Write camera to a json object
    """
    if camera.projection_type == "perspective":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal": camera.focal,
            "k1": camera.k1,
            "k2": camera.k2,
        }
    elif camera.projection_type == "brown":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
            "k2": camera.k2,
            "p1": camera.p1,
            "p2": camera.p2,
            "k3": camera.k3,
        }
    elif camera.projection_type == "fisheye":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal": camera.focal,
            "k1": camera.k1,
            "k2": camera.k2,
        }
    elif camera.projection_type == "fisheye_opencv":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
            "k2": camera.k2,
            "k3": camera.k3,
            "k4": camera.k4,
        }
    elif camera.projection_type == "fisheye62":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
            "k2": camera.k2,
            "k3": camera.k3,
            "k4": camera.k4,
            "k5": camera.k5,
            "k6": camera.k6,
            "p1": camera.p1,
            "p2": camera.p2,
        }
    elif camera.projection_type == "fisheye624":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
            "k2": camera.k2,
            "k3": camera.k3,
            "k4": camera.k4,
            "k5": camera.k5,
            "k6": camera.k6,
            "p1": camera.p1,
            "p2": camera.p2,
            "s0": camera.s0,
            "s1": camera.s1,
            "s2": camera.s2,
            "s3": camera.s3,
        }
    elif camera.projection_type == "simple_radial":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
        }
    elif camera.projection_type == "radial":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal_x": camera.focal,
            "focal_y": camera.focal * camera.aspect_ratio,
            "c_x": camera.principal_point[0],
            "c_y": camera.principal_point[1],
            "k1": camera.k1,
            "k2": camera.k2,
        }
    elif camera.projection_type == "dual":
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
            "focal": camera.focal,
            "k1": camera.k1,
            "k2": camera.k2,
            "transition": camera.transition,
        }
    elif pygeometry.Camera.is_panorama(camera.projection_type):
        return {
            "projection_type": camera.projection_type,
            "width": camera.width,
            "height": camera.height,
        }
    else:
        raise NotImplementedError


def shot_to_json(shot: pymap.Shot) -> Dict[str, Any]:
    """
    Write shot to a json object
    """
    obj: Dict[str, Any] = {
        "rotation": list(shot.pose.rotation),
        "translation": list(shot.pose.translation),
        "camera": shot.camera.id,
    }

    if shot.metadata is not None:
        obj.update(pymap_metadata_to_json(shot.metadata))
    if shot.mesh is not None:
        obj["vertices"] = [list(vertice) for vertice in shot.mesh.vertices]
        obj["faces"] = [list(face) for face in shot.mesh.faces]
    if hasattr(shot, "scale"):
        obj["scale"] = shot.scale
    if hasattr(shot, "covariance"):
        obj["covariance"] = shot.covariance.tolist()
    if hasattr(shot, "merge_cc"):
        obj["merge_cc"] = shot.merge_cc
    return obj


def rig_instance_to_json(rig_instance: pymap.RigInstance) -> Dict[str, Any]:
    """
    Write a rig instance to a json object
    """
    return {
        "translation": list(rig_instance.pose.translation),
        "rotation": list(rig_instance.pose.rotation),
        "rig_camera_ids": rig_instance.rig_camera_ids,
    }


def rig_camera_to_json(rig_camera: pymap.RigCamera) -> Dict[str, Any]:
    """
    Write a rig camera to a json object
    """
    obj = {
        "rotation": list(rig_camera.pose.rotation),
        "translation": list(rig_camera.pose.translation),
    }
    return obj


def pymap_metadata_to_json(metadata: pymap.ShotMeasurements) -> Dict[str, Any]:
    obj = {}
    if metadata.orientation.has_value:
        obj["orientation"] = metadata.orientation.value
    if metadata.capture_time.has_value:
        obj["capture_time"] = metadata.capture_time.value
    if metadata.gps_accuracy.has_value:
        obj["gps_dop"] = metadata.gps_accuracy.value
    if metadata.gps_position.has_value:
        obj["gps_position"] = list(metadata.gps_position.value)
    if metadata.gravity_down.has_value:
        obj["gravity_down"] = list(metadata.gravity_down.value)
    if metadata.compass_angle.has_value and metadata.compass_accuracy.has_value:
        obj["compass"] = {
            "angle": metadata.compass_angle.value,
            "accuracy": metadata.compass_accuracy.value,
        }
    else:
        if metadata.compass_angle.has_value:
            obj["compass"] = {"angle": metadata.compass_angle.value}
        elif metadata.compass_accuracy.has_value:
            obj["compass"] = {"accuracy": metadata.compass_accuracy.value}
    if metadata.sequence_key.has_value:
        obj["skey"] = metadata.sequence_key.value
    return obj


def json_to_pymap_metadata(obj: Dict[str, Any]) -> pymap.ShotMeasurements:
    metadata = pymap.ShotMeasurements()
    if obj.get("orientation") is not None:
        metadata.orientation.value = obj.get("orientation")
    if obj.get("capture_time") is not None:
        metadata.capture_time.value = obj.get("capture_time")
    if obj.get("gps_dop") is not None:
        metadata.gps_accuracy.value = obj.get("gps_dop")
    if obj.get("gps_position") is not None:
        metadata.gps_position.value = obj.get("gps_position")
    if obj.get("skey") is not None:
        metadata.sequence_key.value = obj.get("skey")
    if obj.get("gravity_down") is not None:
        metadata.gravity_down.value = obj.get("gravity_down")
    if obj.get("compass") is not None:
        compass = obj.get("compass")
        if "angle" in compass:
            metadata.compass_angle.value = compass["angle"]
        if "accuracy" in compass:
            metadata.compass_accuracy.value = compass["accuracy"]
    return metadata


def point_to_json(point: pymap.Landmark) -> Dict[str, Any]:
    """
    Write a point to a json object
    """
    return {
        "color": list(point.color.astype(float)),
        "coordinates": list(point.coordinates),
    }


def reconstruction_to_json(reconstruction: types.Reconstruction) -> Dict[str, Any]:
    """
    Write a reconstruction to a json object
    """
    obj = {"cameras": {}, "shots": {}, "points": {}, "biases": {}}

    # Extract cameras
    for camera in reconstruction.cameras.values():
        obj["cameras"][camera.id] = camera_to_json(camera)

    # Extract cameras biases
    for camera_id, bias in reconstruction.biases.items():
        obj["biases"][camera_id] = bias_to_json(bias)

    # Extract rig models
    if len(reconstruction.rig_cameras):
        obj["rig_cameras"] = {}
    for rig_camera in reconstruction.rig_cameras.values():
        obj["rig_cameras"][rig_camera.id] = rig_camera_to_json(rig_camera)
    if len(reconstruction.rig_instances):
        obj["rig_instances"] = {}
    for rig_instance in reconstruction.rig_instances.values():
        obj["rig_instances"][rig_instance.id] = rig_instance_to_json(rig_instance)

    # Extract shots
    for shot in reconstruction.shots.values():
        obj["shots"][shot.id] = shot_to_json(shot)

    # Extract points
    for point in reconstruction.points.values():
        obj["points"][point.id] = point_to_json(point)

    # Extract pano_shots
    if hasattr(reconstruction, "pano_shots"):
        if len(reconstruction.pano_shots) > 0:
            obj["pano_shots"] = {}
            for shot in reconstruction.pano_shots.values():
                obj["pano_shots"][shot.id] = shot_to_json(shot)

    # Extract reference topocentric frame
    if reconstruction.reference:
        ref = reconstruction.reference
        obj["reference_lla"] = {
            "latitude": ref.lat,
            "longitude": ref.lon,
            "altitude": ref.alt,
        }

    return obj


def reconstructions_to_json(
    reconstructions: Iterable[types.Reconstruction],
) -> List[Dict[str, Any]]:
    """
    Write all reconstructions to a json object
    """
    return [reconstruction_to_json(i) for i in reconstructions]


def cameras_to_json(cameras: Dict[str, pygeometry.Camera]) -> Dict[str, Dict[str, Any]]:
    """
    Write cameras to a json object
    """
    obj = {}
    for camera in cameras.values():
        obj[camera.id] = camera_to_json(camera)
    return obj


def bias_to_json(bias: pygeometry.Similarity) -> Dict[str, Any]:
    return {
        "rotation": list(bias.rotation),
        "translation": list(bias.translation),
        "scale": bias.scale,
    }


def rig_cameras_to_json(
    rig_cameras: Dict[str, pymap.RigCamera]
) -> Dict[str, Dict[str, Any]]:
    """
    Write rig cameras to a json object
    """
    obj = {}
    for rig_camera in rig_cameras.values():
        obj[rig_camera.id] = rig_camera_to_json(rig_camera)
    return obj


def camera_from_vector(
    camera_id: str,
    width: int,
    height: int,
    projection_type: str,
    parameters: List[float],
) -> pygeometry.Camera:
    """Build a camera from a serialized vector of parameters."""
    if projection_type == "perspective":
        focal, k1, k2 = parameters
        camera = pygeometry.Camera.create_perspective(focal, k1, k2)
    elif projection_type == "brown":
        fx, fy, cx, cy, k1, k2, p1, p2, k3 = parameters
        camera = pygeometry.Camera.create_brown(
            fx, fy / fx, np.array([cx, cy]), np.array([k1, k2, k3, p1, p2])
        )
    elif projection_type == "fisheye":
        focal, k1, k2 = parameters
        camera = pygeometry.Camera.create_fisheye(focal, k1, k2)
    elif projection_type == "fisheye_opencv":
        fx, fy, cx, cy, k1, k2, k3, k4 = parameters
        camera = pygeometry.Camera.create_fisheye_opencv(
            fx, fy / fx, np.array([cx, cy]), np.array([k1, k2, k3, k4])
        )
    elif projection_type == "fisheye62":
        fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2 = parameters
        camera = pygeometry.Camera.create_fisheye62(
            fx, fy / fx, np.array([cx, cy]), np.array([k1, k2, k3, k4, k5, k6, p1, p2])
        )
    elif projection_type == "fisheye624":
        fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2, s0, s1, s2, s3 = parameters
        camera = pygeometry.Camera.create_fisheye624(
            fx,
            fy / fx,
            np.array([cx, cy]),
            np.array([k1, k2, k3, k4, k5, k6, p1, p2, s0, s1, s2, s3]),
        )
    elif projection_type == "radial":
        fx, fy, cx, cy, k1, k2 = parameters
        camera = pygeometry.Camera.create_radial(
            fx, fy / fx, np.array([cx, cy]), np.array([k1, k2])
        )
    elif projection_type == "simple_radial":
        fx, fy, cx, cy, k1 = parameters
        camera = pygeometry.Camera.create_simple_radial(
            fx, fy / fx, np.array([cx, cy]), k1
        )
    elif projection_type == "dual":
        focal, k1, k2, transition = parameters
        camera = pygeometry.Camera.create_dual(transition, focal, k1, k2)
    elif pygeometry.Camera.is_panorama(projection_type):
        camera = pygeometry.Camera.create_spherical()
    else:
        raise NotImplementedError
    camera.id = camera_id
    camera.width = width
    camera.height = height
    return camera


def camera_to_vector(camera: pygeometry.Camera) -> List[float]:
    """Serialize camera parameters to a vector of floats."""
    if camera.projection_type == "perspective":
        parameters = [camera.focal, camera.k1, camera.k2]
    elif camera.projection_type == "brown":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
            camera.k2,
            camera.p1,
            camera.p2,
            camera.k3,
        ]
    elif camera.projection_type == "fisheye":
        parameters = [camera.focal, camera.k1, camera.k2]
    elif camera.projection_type == "fisheye_opencv":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
            camera.k2,
            camera.k3,
            camera.k4,
        ]
    elif camera.projection_type == "fisheye62":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
            camera.k2,
            camera.k3,
            camera.k4,
            camera.k5,
            camera.k6,
            camera.p1,
            camera.p2,
        ]
    elif camera.projection_type == "fisheye624":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
            camera.k2,
            camera.k3,
            camera.k4,
            camera.k5,
            camera.k6,
            camera.p1,
            camera.p2,
            camera.s0,
            camera.s1,
            camera.s2,
            camera.s3,
        ]
    elif camera.projection_type == "radial":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
            camera.k2,
        ]
    elif camera.projection_type == "simple_radial":
        parameters = [
            camera.focal,
            camera.focal * camera.aspect_ratio,
            camera.principal_point[0],
            camera.principal_point[1],
            camera.k1,
        ]
    elif camera.projection_type == "dual":
        parameters = [
            camera.focal,
            camera.k1,
            camera.k2,
            camera.transition,
        ]
    elif pygeometry.Camera.is_panorama(camera.projection_type):
        parameters = []
    else:
        raise NotImplementedError

    return parameters


def _read_gcp_list_lines(
    lines: Iterable[str],
    projection,
    exifs: Dict[str, Dict[str, Any]],
) -> List[pymap.GroundControlPoint]:
    points = {}
    for line in lines:
        words = line.split(None, 5)
        easting, northing, alt, pixel_x, pixel_y = map(float, words[:5])
        key = (easting, northing, alt)

        shot_tokens = words[5].split(None)
        shot_id = shot_tokens[0]
        if shot_id not in exifs:
            continue

        if key in points:
            point = points[key]
        else:
            # Convert 3D coordinates
            if np.isnan(alt):
                alt = 0
                has_altitude = False
            else:
                has_altitude = True
            if projection is not None:
                lat, lon = projection.transform(easting, northing)
            else:
                lon, lat = easting, northing

            point = pymap.GroundControlPoint()
            point.id = "unnamed-%d" % len(points)
            point.lla = {"latitude": lat, "longitude": lon, "altitude": alt}
            point.has_altitude = has_altitude

            points[key] = point

        # Convert 2D coordinates
        d = exifs[shot_id]
        coordinates = features.normalized_image_coordinates(
            np.array([[pixel_x, pixel_y]]), d["width"], d["height"]
        )[0]

        o = pymap.GroundControlPointObservation()
        o.shot_id = shot_id
        o.projection = coordinates
        point.add_observation(o)

    return list(points.values())


def _parse_utm_projection_string(line: str) -> str:
    """Convert strings like 'WGS84 UTM 32N' to a proj4 definition."""
    words = line.lower().split()
    assert len(words) == 3
    zone = line.split()[2].upper()
    if zone[-1] == "N":
        zone_number = int(zone[:-1])
        zone_hemisphere = "north"
    elif zone[-1] == "S":
        zone_number = int(zone[:-1])
        zone_hemisphere = "south"
    else:
        zone_number = int(zone)
        zone_hemisphere = "north"
    s = "+proj=utm +zone={} +{} +ellps=WGS84 +datum=WGS84 +units=m +no_defs"
    return s.format(zone_number, zone_hemisphere)


def _parse_projection(line: str) -> Optional[pyproj.Transformer]:
    """Build a proj4 from the GCP format line."""
    crs_4326 = pyproj.CRS.from_epsg(4326)
    if line.strip() == "WGS84":
        return None
    elif line.upper().startswith("WGS84 UTM"):
        return pyproj.Transformer.from_proj(
            pyproj.CRS(_parse_utm_projection_string(line)), crs_4326
        )
    elif "+proj" in line:
        return pyproj.Transformer.from_proj(pyproj.CRS(line), crs_4326)
    elif line.upper().startswith("EPSG:"):
        return pyproj.Transformer.from_proj(
            pyproj.CRS.from_epsg(int(line.split(":")[1])), crs_4326
        )
    else:
        raise ValueError("Un-supported geo system definition: {}".format(line))


def _valid_gcp_line(line: str) -> bool:
    stripped = line.strip()
    return stripped != "" and stripped[0] != "#"


def read_gcp_list(fileobj, exif: Dict[str, Any]) -> List[pymap.GroundControlPoint]:
    """Read a ground control points from a gcp_list.txt file.

    It requires the points to be in the WGS84 lat, lon, alt format.
    If reference is None, topocentric data won't be initialized.
    """
    all_lines = fileobj.readlines()
    lines = iter(filter(_valid_gcp_line, all_lines))
    projection = _parse_projection(next(lines))
    points = _read_gcp_list_lines(lines, projection, exif)
    return points


def read_ground_control_points(fileobj: IO) -> List[pymap.GroundControlPoint]:
    """Read ground control points from json file"""
    obj = json_load(fileobj)

    points = []
    for point_dict in obj["points"]:
        point = pymap.GroundControlPoint()
        point.id = point_dict["id"]
        lla = point_dict.get("position")
        if lla:
            point.lla = lla
            point.has_altitude = "altitude" in point.lla

        observations = []
        observing_images = set()
        for o_dict in point_dict["observations"]:
            o = pymap.GroundControlPointObservation()
            o.shot_id = o_dict["shot_id"]
            if o.shot_id in observing_images:
                logger.warning(
                    "GCP {} has multiple observations in image {}".format(
                        point.id, o.shot_id
                    )
                )
            observing_images.add(o.shot_id)
            if "projection" in o_dict:
                o.projection = np.array(o_dict["projection"])
            observations.append(o)
        point.observations = observations
        points.append(point)
    return points


def write_ground_control_points(
    gcp: List[pymap.GroundControlPoint],
    fileobj: IO,
) -> None:
    """Write ground control points to json file."""
    obj = {"points": []}

    for point in gcp:
        point_obj = {}
        point_obj["id"] = point.id
        if point.lla:
            point_obj["position"] = {
                "latitude": point.lla["latitude"],
                "longitude": point.lla["longitude"],
            }
            if point.has_altitude:
                point_obj["position"]["altitude"] = point.lla["altitude"]

        point_obj["observations"] = []
        for observation in point.observations:
            point_obj["observations"].append(
                {
                    "shot_id": observation.shot_id,
                    "projection": tuple(observation.projection),
                }
            )

        obj["points"].append(point_obj)

    json_dump(obj, fileobj)


def json_dump_kwargs(minify: bool = False) -> Dict[str, Any]:
    if minify:
        indent, separators = None, (",", ":")
    else:
        indent, separators = 4, None
    return {"indent": indent, "ensure_ascii": False, "separators": separators}


def json_dump(data, fout: IO[str], minify: bool = False) -> None:
    kwargs = json_dump_kwargs(minify)
    return json.dump(data, fout, **kwargs)


def json_dumps(data, minify: bool = False) -> str:
    kwargs = json_dump_kwargs(minify)
    return json.dumps(data, **kwargs)

def json_load(fp: Union[IO[str], IO[bytes]]) -> Any:
    return json.load(fp)


def json_loads(text: Union[str, bytes]) -> Any:
    return json.loads(text)


# PLY


def ply_header(
    count_vertices: int, with_normals: bool = False, point_num_views: bool = False
) -> List[str]:
    if with_normals:
        header = [
            "ply",
            "format ascii 1.0",
            "element vertex {}".format(count_vertices),
            "property float x",
            "property float y",
            "property float z",
            "property float nx",
            "property float ny",
            "property float nz",
            "property uchar red",      
            "property uchar green",     
            "property uchar blue",      
        ]
    else:
        header = [
            "ply",
            "format ascii 1.0",
            "element vertex {}".format(count_vertices),
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",    
            "property uchar green",   
            "property uchar blue", 
        ]

    if point_num_views:
        header += ["property uchar views"]

    header += ["end_header"]

    return header


def points_to_ply_string(vertices: List[str], point_num_views: bool = False) -> str:
    header = ply_header(len(vertices), point_num_views=point_num_views)
    return "\n".join(header + vertices + [""])


def reconstruction_to_ply(
    reconstruction: types.Reconstruction,
    tracks_manager: Optional[pymap.TracksManager] = None,
    no_cameras: bool = False,
    no_points: bool = False,
    point_num_views: bool = False,
) -> str:
    """Export reconstruction points as a PLY string."""
    vertices = []

    if not no_points:
        for point in reconstruction.points.values():
            p, c = point.coordinates, point.color
            s = "{} {} {} {} {} {}".format(
                p[0], p[1], p[2], int(c[0]), int(c[1]), int(c[2])
            )

            if point_num_views and tracks_manager:
                obs_count = point.number_of_observations()
                if obs_count == 0:
                    obs_count = len(tracks_manager.get_track_observations(point.id))
                s += " {}".format(obs_count)

            vertices.append(s)

    if not no_cameras:
        for shot in reconstruction.shots.values():
            o = shot.pose.get_origin()
            R = shot.pose.get_rotation_matrix()
            for axis in range(3):
                c = 255 * np.eye(3)[axis]
                for depth in np.linspace(0, 2, 10):
                    p = o + depth * R[axis]
                    s = "{} {} {} {} {} {}".format(
                        p[0], p[1], p[2], int(c[0]), int(c[1]), int(c[2])
                    )
                    if point_num_views:
                        s += " 0"
                    vertices.append(s)
    return points_to_ply_string(vertices, point_num_views)


def point_cloud_from_ply(
    fp: TextIO,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Load point cloud from a PLY file."""
    all_lines = fp.read().splitlines()
    start = all_lines.index("end_header") + 1
    lines = all_lines[start:]
    n = len(lines)

    points = np.zeros((n, 3), dtype=np.float32)
    normals = np.zeros((n, 3), dtype=np.float32)
    colors = np.zeros((n, 3), dtype=np.uint8)
    labels = np.zeros((n,), dtype=np.uint8)

    for i, row in enumerate(lines):
        words = row.split()
        label = int(words[9])
        points[i] = list(map(float, words[0:3]))
        normals[i] = list(map(float, words[3:6]))
        colors[i] = list(map(int, words[6:9]))
        labels[i] = label

    return points, normals, colors, labels


def point_cloud_to_ply(
    points: np.ndarray,
    normals: np.ndarray,
    colors: np.ndarray,
    labels: np.ndarray,
    fp: TextIO,
) -> None:
    fp.write("ply\n")
    fp.write("format ascii 1.0\n")
    fp.write("element vertex {}\n".format(len(points)))
    fp.write("property float x\n")
    fp.write("property float y\n")
    fp.write("property float z\n")
    fp.write("property float nx\n")
    fp.write("property float ny\n")
    fp.write("property float nz\n")
    fp.write("property uchar red\n")
    fp.write("property uchar green\n")
    fp.write("property uchar blue\n")
    fp.write("property uchar class\n")
    fp.write("end_header\n")

    template = "{:.4f} {:.4f} {:.4f} {:.3f} {:.3f} {:.3f} {} {} {} {}\n"
    for i in range(len(points)):
        p, n, c, l = points[i], normals[i], colors[i], labels[i]
        fp.write(
            template.format(
                p[0],
                p[1],
                p[2],
                n[0],
                n[1],
                n[2],
                int(c[0]),
                int(c[1]),
                int(c[2]),
                int(l),
            )
        )


# Filesystem interaction methods
def mkdir_p(path: str) -> None:
    """Make a directory including parent directories."""
    os.makedirs(path, exist_ok=True)


def open_wt(path: str) -> IO[Any]:
    """Open a file in text mode for writing utf-8."""
    return open(path, "w", encoding="utf-8")


def open_rt(path: str) -> IO[Any]:
    """Open a file in text mode for reading utf-8."""
    return open(path, "r", encoding="utf-8")


def imread(
    path: str, grayscale: bool = False, unchanged: bool = False, anydepth: bool = False
) -> ndarray:
    with open(path, "rb") as fb:
        return imread_from_fileobject(fb, grayscale, unchanged, anydepth)


def imread_from_fileobject(
    fb, grayscale: bool = False, unchanged: bool = False, anydepth: bool = False
) -> np.ndarray:
    """Load image as an array ignoring EXIF orientation."""
    if context.OPENCV3:
        if grayscale:
            flags = cv2.IMREAD_GRAYSCALE
        elif unchanged:
            flags = cv2.IMREAD_UNCHANGED
        else:
            flags = cv2.IMREAD_COLOR

        try:
            flags |= cv2.IMREAD_IGNORE_ORIENTATION
        except AttributeError:
            logger.warning(
                "OpenCV version {} does not support loading images without "
                "rotating them according to EXIF. Please upgrade OpenCV to "
                "version 3.2 or newer.".format(cv2.__version__)
            )

        if anydepth:
            flags |= cv2.IMREAD_ANYDEPTH
    else:
        if grayscale:
            flags = cv2.CV_LOAD_IMAGE_GRAYSCALE
        elif unchanged:
            flags = cv2.CV_LOAD_IMAGE_UNCHANGED
        else:
            flags = cv2.CV_LOAD_IMAGE_COLOR

        if anydepth:
            flags |= cv2.CV_LOAD_IMAGE_ANYDEPTH

    im_buffer = np.asarray(bytearray(fb.read()), dtype=np.uint8)
    image = cv2.imdecode(im_buffer, flags)

    if image is None:
        raise IOError("Unable to load image")

    if len(image.shape) == 3:
        image[:, :, :3] = image[:, :, [2, 1, 0]]  # Turn BGR to RGB (or BGRA to RGBA)
    return image

    @classmethod
    def imwrite(cls, path: str, image: np.ndarray) -> None:
        with cls.open(path, "wb") as fwb:
            imwrite(fwb, image, path)


def imwrite(path: str, image: np.ndarray) -> None:
    with open(path, "wb") as fwb:
        return imwrite_from_fileobject(fwb, image, path)


def imwrite_from_fileobject(fwb, image: np.ndarray, ext: str) -> None:
    """Write an image to a file object"""
    if len(image.shape) == 3:
        image[:, :, :3] = image[:, :, [2, 1, 0]]  # Turn RGB to BGR (or RGBA to BGRA)
    _, im_buffer = cv2.imencode(ext, image)
    fwb.write(im_buffer)


def image_size_from_fileobject(
    fb: Union[IO[bytes], bytes, Path, str, TextIO]
) -> Tuple[int, int]:
    """Height and width of an image."""
    if isinstance(fb, TextIO):
        image = imread(fb.name)
        return image.shape[:2]
    else:
        with Image.open(fb) as img:
            width, height = img.size
            return height, width


def image_size(path: str) -> Tuple[int, int]:
    """Height and width of an image."""
    with open(path, "rb") as fb:
        return image_size_from_fileobject(fb)


# IO Filesystem
class IoFilesystemBase(ABC):
    @classmethod
    @abstractmethod
    def exists(cls, path: str):
        pass

    @classmethod
    def ls(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def isfile(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def isdir(cls, path: str):
        pass

    @classmethod
    def rm_if_exist(cls, filename: str):
        pass

    @classmethod
    def symlink(cls, src_path: str, dst_path: str, **kwargs):
        pass

    @classmethod
    @abstractmethod
    def open(cls, *args, **kwargs) -> IO[Any]:
        pass

    @classmethod
    @abstractmethod
    def open_wt(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def open_rt(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def mkdir_p(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def imwrite(cls, path: str, image):
        pass

    @classmethod
    @abstractmethod
    def imread(cls, path: str, grayscale=False, unchanged=False, anydepth=False):
        pass

    @classmethod
    @abstractmethod
    def image_size(cls, path: str):
        pass

    @classmethod
    @abstractmethod
    def timestamp(cls, path: str):
        pass


class IoFilesystemDefault(IoFilesystemBase):
    def __init__(self) -> None:
        self.type = "default"

    @classmethod
    def exists(cls, path: str) -> str:
        # pyre-fixme[7]: Expected `str` but got `bool`.
        return os.path.exists(path)

    @classmethod
    def ls(cls, path: str) -> List[str]:
        return os.listdir(path)

    @classmethod
    def isfile(cls, path: str) -> str:
        # pyre-fixme[7]: Expected `str` but got `bool`.
        return os.path.isfile(path)

    @classmethod
    def isdir(cls, path: str) -> str:
        # pyre-fixme[7]: Expected `str` but got `bool`.
        return os.path.isdir(path)

    @classmethod
    def rm_if_exist(cls, filename: str) -> None:
        if os.path.islink(filename):
            os.unlink(filename)
        if os.path.exists(filename):
            if os.path.isdir(filename):
                shutil.rmtree(filename)
            else:
                os.remove(filename)

    @classmethod
    def symlink(cls, src_path: str, dst_path: str, **kwargs):
        os.symlink(src_path, dst_path, **kwargs)

    @classmethod
    def open(cls, *args, **kwargs) -> IO[Any]:
        return open(*args, **kwargs)

    @classmethod
    def open_wt(cls, path: str):
        return cls.open(path, "w", encoding="utf-8")

    @classmethod
    def open_rt(cls, path: str):
        return cls.open(path, "r", encoding="utf-8")

    @classmethod
    def mkdir_p(cls, path: str):
        return os.makedirs(path, exist_ok=True)

    @classmethod
    def imread(
        cls,
        path: str,
        grayscale: bool = False,
        unchanged: bool = False,
        anydepth: bool = False,
    ):
        with cls.open(path, "rb") as fb:
            return imread_from_fileobject(fb, grayscale, unchanged, anydepth)

    @classmethod
    def imwrite(cls, path: str, image) -> None:
        with cls.open(path, "wb") as fwb:
            imwrite_from_fileobject(fwb, image, path)

    @classmethod
    def image_size(cls, path: str) -> Tuple[int, int]:
        with cls.open(path, "rb") as fb:
            return image_size_from_fileobject(fb)

    @classmethod
    def timestamp(cls, path: str) -> str:
        # pyre-fixme[7]: Expected `str` but got `float`.
        return os.path.getmtime(path)
