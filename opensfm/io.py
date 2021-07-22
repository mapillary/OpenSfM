import json
import logging
import os
import shutil
import typing as t
from abc import ABC, abstractmethod

import cv2
import numpy as np
import pyproj
from opensfm import context, features, geo, pygeometry, pymap, types
from PIL import Image

logger = logging.getLogger(__name__)


def camera_from_json(key, obj):
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
            [obj.get("c_x", 0.0), obj.get("c_y", 0.0)],
            [
                obj.get("k1", 0.0),
                obj.get("k2", 0.0),
                obj.get("k3", 0.0),
                obj.get("p1", 0.0),
                obj.get("p2", 0.0),
            ],
        )
    elif pt == "fisheye":
        camera = pygeometry.Camera.create_fisheye(
            obj["focal"], obj.get("k1", 0.0), obj.get("k2", 0.0)
        )
    elif pt == "fisheye_opencv":
        camera = pygeometry.Camera.create_fisheye_opencv(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            [obj.get("c_x", 0.0), obj.get("c_y", 0.0)],
            [
                obj.get("k1", 0.0),
                obj.get("k2", 0.0),
                obj.get("k3", 0.0),
                obj.get("k4", 0.0),
            ],
        )
    elif pt == "fisheye62":
        camera = pygeometry.Camera.create_fisheye62(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            [obj.get("c_x", 0.0), obj.get("c_y", 0.0)],
            [
                obj.get("k1", 0.0),
                obj.get("k2", 0.0),
                obj.get("k3", 0.0),
                obj.get("k4", 0.0),
                obj.get("k5", 0.0),
                obj.get("k6", 0.0),
                obj.get("p1", 0.0),
                obj.get("p2", 0.0),
            ],
        )
    elif pt == "radial":
        camera = pygeometry.Camera.create_radial(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            [obj.get("c_x", 0.0), obj.get("c_y", 0.0)],
            [
                obj.get("k1", 0.0),
                obj.get("k2", 0.0),
            ],
        )
    elif pt == "simple_radial":
        camera = pygeometry.Camera.create_simple_radial(
            obj["focal_x"],
            obj["focal_y"] / obj["focal_x"],
            [obj.get("c_x", 0.0), obj.get("c_y", 0.0)],
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


def pose_from_json(obj):
    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    if "translation" in obj:
        pose.translation = obj["translation"]
    return pose


def bias_from_json(obj):
    return pygeometry.Similarity(obj["rotation"], obj["translation"], obj["scale"])


def assign_shot_attributes(obj, shot):
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


def shot_in_reconstruction_from_json(reconstruction, key, obj, is_pano_shot=False):
    """
    Read shot from a json object and append it to a reconstruction
    """
    pose = pose_from_json(obj)

    if is_pano_shot:
        shot = reconstruction.create_pano_shot(key, obj["camera"], pose)
    else:
        shot = reconstruction.create_shot(key, obj["camera"], pose)
    assign_shot_attributes(obj, shot)
    return shot


def single_shot_from_json(key, obj, camera):
    """
    Read shot from a json object
    """
    pose = pose_from_json(obj)
    shot = pymap.Shot(key, camera, pose)
    assign_shot_attributes(obj, shot)
    return shot


def point_from_json(reconstruction, key, obj):
    """
    Read a point from a json object
    """
    point = reconstruction.create_point(key, obj["coordinates"])
    point.color = obj["color"]
    return point


def rig_camera_from_json(key, obj):
    """
    Read a rig cameras from a json object
    """
    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    pose.translation = obj["translation"]
    rig_camera = pymap.RigCamera(pose, key)
    return rig_camera


def rig_cameras_from_json(obj):
    """
    Read rig cameras from a json object
    """
    rig_cameras = {}
    for key, value in obj.items():
        rig_cameras[key] = rig_camera_from_json(key, value)
    return rig_cameras


def rig_instance_from_json(reconstruction, key, obj):
    """
    Read any rig instance from a json shot object
    """
    instance_id = int(key)
    reconstruction.add_rig_instance(pymap.RigInstance(instance_id))

    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    pose.translation = obj["translation"]
    reconstruction.rig_instances[instance_id].pose = pose

    for shot_id, rig_camera_id in obj["rig_camera_ids"].items():
        reconstruction.rig_instances[instance_id].add_shot(
            reconstruction.rig_cameras[rig_camera_id], reconstruction.shots[shot_id]
        )


def reconstruction_from_json(obj: t.Dict[str, t.Any]):
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

    # Extract shots
    for key, value in obj["shots"].items():
        shot_in_reconstruction_from_json(reconstruction, key, value)

    # Extract rig instances from shots
    if "rig_instances" in obj:
        for key, value in obj["rig_instances"].items():
            rig_instance_from_json(reconstruction, key, value)

    # Extract points
    if "points" in obj:
        for key, value in obj["points"].items():
            point_from_json(reconstruction, key, value)

    # Extract pano_shots
    if "pano_shots" in obj:
        for key, value in obj["pano_shots"].items():
            is_pano_shot = True
            shot_in_reconstruction_from_json(reconstruction, key, value, is_pano_shot)

    # Extract main and unit shots
    if "main_shot" in obj:
        # pyre-fixme[16]: `types.Reconstruction` has no attribute `main_shot`
        reconstruction.main_shot = obj["main_shot"]
    if "unit_shot" in obj:
        # pyre-fixme[16]: `types.Reconstruction` has no attribute `unit_shot`
        reconstruction.unit_shot = obj["unit_shot"]

    # Extract reference topocentric frame
    if "reference_lla" in obj:
        lla = obj["reference_lla"]
        reconstruction.reference = geo.TopocentricConverter(
            lla["latitude"], lla["longitude"], lla["altitude"]
        )

    return reconstruction


def reconstructions_from_json(obj):
    """
    Read all reconstructions from a json object
    """
    return [reconstruction_from_json(i) for i in obj]


def cameras_from_json(obj):
    """
    Read cameras from a json object
    """
    cameras = {}
    for key, value in obj.items():
        cameras[key] = camera_from_json(key, value)
    return cameras


def camera_to_json(camera):
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


def shot_to_json(shot):
    """
    Write shot to a json object
    """
    obj = {
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


def rig_instance_to_json(rig_instance):
    """
    Write a rig instance to a json object
    """
    return {
        "translation": list(rig_instance.pose.translation),
        "rotation": list(rig_instance.pose.rotation),
        "rig_camera_ids": rig_instance.camera_ids,
    }


def rig_camera_to_json(rig_camera):
    """
    Write a rig camera to a json object
    """
    obj = {
        "rotation": list(rig_camera.pose.rotation),
        "translation": list(rig_camera.pose.translation),
    }
    return obj


def pymap_metadata_to_json(metadata):
    obj = {}
    if metadata.orientation.has_value:
        obj["orientation"] = metadata.orientation.value
    if metadata.capture_time.has_value:
        obj["capture_time"] = metadata.capture_time.value
    if metadata.gps_accuracy.has_value:
        obj["gps_dop"] = metadata.gps_accuracy.value
    if metadata.gps_position.has_value:
        obj["gps_position"] = list(metadata.gps_position.value)
    if metadata.accelerometer.has_value:
        obj["accelerometer"] = list(metadata.accelerometer.value)
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


def json_to_pymap_metadata(obj):
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
    if obj.get("accelerometer") is not None:
        metadata.accelerometer.value = obj.get("accelerometer")
    if obj.get("compass") is not None:
        compass = obj.get("compass")
        if "angle" in compass:
            metadata.compass_angle.value = compass["angle"]
        if "accuracy" in compass:
            metadata.compass_accuracy.value = compass["accuracy"]
    return metadata


def point_to_json(point):
    """
    Write a point to a json object
    """
    return {
        "color": list(point.color.astype(float)),
        "coordinates": list(point.coordinates),
    }


def reconstruction_to_json(reconstruction) -> t.Dict[str, t.Any]:
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

    # Extract main and unit shots
    if hasattr(reconstruction, "main_shot"):
        obj["main_shot"] = reconstruction.main_shot
    if hasattr(reconstruction, "unit_shot"):
        obj["unit_shot"] = reconstruction.unit_shot

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
    reconstructions: t.Iterable[types.Reconstruction],
) -> t.List[t.Dict[str, t.Any]]:
    """
    Write all reconstructions to a json object
    """
    return [reconstruction_to_json(i) for i in reconstructions]


def cameras_to_json(cameras):
    """
    Write cameras to a json object
    """
    obj = {}
    for camera in cameras.values():
        obj[camera.id] = camera_to_json(camera)
    return obj


def bias_to_json(bias):
    return {
        "rotation": list(bias.rotation),
        "translation": list(bias.translation),
        "scale": bias.scale,
    }


def rig_cameras_to_json(rig_cameras):
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
    parameters: t.List[float],
) -> pygeometry.Camera:
    """Build a camera from a serialized vector of parameters."""
    if projection_type == "perspective":
        focal, k1, k2 = parameters
        camera = pygeometry.Camera.create_perspective(focal, k1, k2)
    elif projection_type == "brown":
        fx, fy, cx, cy, k1, k2, p1, p2, k3 = parameters
        camera = pygeometry.Camera.create_brown(
            fx, fy / fx, [cx, cy], [k1, k2, k3, p1, p2]
        )
    elif projection_type == "fisheye":
        focal, k1, k2 = parameters
        camera = pygeometry.Camera.create_fisheye(focal, k1, k2)
    elif projection_type == "fisheye_opencv":
        fx, fy, cx, cy, k1, k2, k3, k4 = parameters
        camera = pygeometry.Camera.create_fisheye_opencv(
            fx, fy / fx, [cx, cy], [k1, k2, k3, k4]
        )
    elif projection_type == "fisheye62":
        fx, fy, cx, cy, k1, k2, k3, k4, k5, k6, p1, p2 = parameters
        camera = pygeometry.Camera.create_fisheye62(
            fx, fy / fx, [cx, cy], [k1, k2, k3, k4, k5, k6, p1, p2]
        )
    elif projection_type == "radial":
        fx, fy, cx, cy, k1, k2 = parameters
        camera = pygeometry.Camera.create_radial(fx, fy / fx, [cx, cy], [k1, k2])
    elif projection_type == "simple_radial":
        fx, fy, cx, cy, k1 = parameters
        camera = pygeometry.Camera.create_simple_radial(fx, fy / fx, [cx, cy], k1)
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


def camera_to_vector(camera: pygeometry.Camera) -> t.List[float]:
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


def _read_gcp_list_lines(lines, projection, reference, exif):
    points = {}
    for line in lines:
        words = line.split(None, 5)
        easting, northing, alt, pixel_x, pixel_y = map(float, words[:5])
        shot_id = words[5].strip()
        key = (easting, northing, alt)

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
                lon, lat = projection(easting, northing, inverse=True)
            else:
                lon, lat = easting, northing

            point = pymap.GroundControlPoint()
            point.id = "unnamed-%d" % len(points)
            point.lla = {"latitude": lat, "longitude": lon, "altitude": alt}
            point.has_altitude = has_altitude

            if reference:
                x, y, z = reference.to_topocentric(lat, lon, alt)
                point.coordinates.value = np.array([x, y, z])
            else:
                point.coordinates.reset()

            points[key] = point

        # Convert 2D coordinates
        d = exif[shot_id]
        coordinates = features.normalized_image_coordinates(
            np.array([[pixel_x, pixel_y]]), d["width"], d["height"]
        )[0]

        o = pymap.GroundControlPointObservation()
        o.shot_id = shot_id
        o.projection = coordinates
        point.add_observation(o)

    return list(points.values())


def _parse_utm_projection_string(line):
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


def _parse_projection(line):
    """Build a proj4 from the GCP format line."""
    if line.strip() == "WGS84":
        return None
    elif line.upper().startswith("WGS84 UTM"):
        return pyproj.Proj(_parse_utm_projection_string(line))
    elif "+proj" in line:
        return pyproj.Proj(line)
    else:
        raise ValueError("Un-supported geo system definition: {}".format(line))


def _valid_gcp_line(line):
    stripped = line.strip()
    return stripped and stripped[0] != "#"


def read_gcp_list(fileobj, reference, exif):
    """Read a ground control points from a gcp_list.txt file.

    It requires the points to be in the WGS84 lat, lon, alt format.
    If reference is None, topocentric data won't be initialized.
    """
    all_lines = fileobj.readlines()
    lines = iter(filter(_valid_gcp_line, all_lines))
    projection = _parse_projection(next(lines))
    points = _read_gcp_list_lines(lines, projection, reference, exif)
    return points


def read_ground_control_points(fileobj, reference):
    """Read ground control points from json file.

    Returns list of types.GroundControlPoint.
    """
    obj = json_load(fileobj)

    points = []
    for point_dict in obj["points"]:
        point = pymap.GroundControlPoint()
        point.id = point_dict["id"]
        lla = point_dict.get("position")
        if lla:
            point.lla = lla
            point.has_altitude = "altitude" in point.lla
            if reference:
                point.coordinates.value = reference.to_topocentric(
                    point.lla["latitude"],
                    point.lla["longitude"],
                    point.lla.get("altitude", 0),
                )
            else:
                point.coordinates.reset()

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


def write_ground_control_points(gcp, fileobj, reference):
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
        elif point.coordinates.has_value:
            lat, lon, alt = reference.to_lla(*point.coordinates.value)
            point_obj["position"] = {
                "latitude": lat,
                "longitude": lon,
            }
            if point.has_altitude:
                point_obj["position"]["altitude"] = alt

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


def json_dump_kwargs(minify=False):
    if minify:
        indent, separators = None, (",", ":")
    else:
        indent, separators = 4, None
    return {"indent": indent, "ensure_ascii": False, "separators": separators}


def json_dump(data, fout, minify=False):
    kwargs = json_dump_kwargs(minify)
    return json.dump(data, fout, **kwargs)


def json_dumps(data, minify=False):
    kwargs = json_dump_kwargs(minify)
    return json.dumps(data, **kwargs)


def json_load(fp):
    return json.load(fp)


def json_loads(text):
    return json.loads(text)


# PLY


def ply_header(count_vertices, with_normals=False, point_num_views=False):
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
            "property uchar diffuse_red",
            "property uchar diffuse_green",
            "property uchar diffuse_blue",
        ]
    else:
        header = [
            "ply",
            "format ascii 1.0",
            "element vertex {}".format(count_vertices),
            "property float x",
            "property float y",
            "property float z",
            "property uchar diffuse_red",
            "property uchar diffuse_green",
            "property uchar diffuse_blue",
        ]

    if point_num_views:
        header += ["property uchar views"]

    header += ["end_header"]

    return header


def points_to_ply_string(vertices, point_num_views=False):
    header = ply_header(len(vertices), point_num_views=point_num_views)
    return "\n".join(header + vertices + [""])


def reconstruction_to_ply(
    reconstruction,
    tracks_manager=None,
    no_cameras=False,
    no_points=False,
    point_num_views=False,
):
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
    fp: t.TextIO,
) -> t.Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
    fp: t.TextIO,
) -> None:
    """Export depthmap points as a PLY string"""
    lines = _point_cloud_to_ply_lines(points, normals, colors, labels)
    fp.writelines(lines)


def _point_cloud_to_ply_lines(
    points: np.ndarray,
    normals: np.ndarray,
    colors: np.ndarray,
    labels: np.ndarray,
):
    yield "ply\n"
    yield "format ascii 1.0\n"
    yield "element vertex {}\n".format(len(points))
    yield "property float x\n"
    yield "property float y\n"
    yield "property float z\n"
    yield "property float nx\n"
    yield "property float ny\n"
    yield "property float nz\n"
    yield "property uchar diffuse_red\n"
    yield "property uchar diffuse_green\n"
    yield "property uchar diffuse_blue\n"
    yield "property uchar class\n"
    yield "end_header\n"

    template = "{:.4f} {:.4f} {:.4f} {:.3f} {:.3f} {:.3f} {} {} {} {}\n"
    for i in range(len(points)):
        p, n, c, l = points[i], normals[i], colors[i], labels[i]
        yield template.format(
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


# Filesystem interaction methods
def mkdir_p(path):
    """Make a directory including parent directories."""
    return os.makedirs(path, exist_ok=True)


def open_wt(path):
    """Open a file in text mode for writing utf-8."""
    return open(path, "w", encoding="utf-8")


def open_rt(path):
    """Open a file in text mode for reading utf-8."""
    return open(path, "r", encoding="utf-8")


def imread(path, grayscale=False, unchanged=False, anydepth=False):
    with open(path, "rb") as fb:
        return imread_from_fileobject(fb, grayscale, unchanged, anydepth)


def imread_from_fileobject(fb, grayscale=False, unchanged=False, anydepth=False):
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
    def imwrite(cls, path, image):
        with cls.open(path, "wb") as fwb:
            imwrite(fwb, image, path)


def imwrite(path, image: np.ndarray):
    with open(path, "wb") as fwb:
        return imwrite_from_fileobject(fwb, image, path)


def imwrite_from_fileobject(fwb, image: np.ndarray, ext: str):
    """Write an image to a file object"""
    if len(image.shape) == 3:
        image[:, :, :3] = image[:, :, [2, 1, 0]]  # Turn RGB to BGR (or RGBA to BGRA)
    _, im_buffer = cv2.imencode(ext, image)
    fwb.write(im_buffer)


def image_size_from_fileobject(fb):
    """Height and width of an image."""
    try:
        with Image.open(fb) as img:
            width, height = img.size
            return height, width
    except Exception:
        # Slower fallback
        image = imread(fb)
        return image.shape[:2]


def image_size(path):
    """Height and width of an image."""
    with open(path, "rb") as fb:
        return image_size_from_fileobject(fb)


# IO Filesystem
class IoFilesystemBase(ABC):
    @classmethod
    @abstractmethod
    def exists(cls, path):
        pass

    @classmethod
    def ls(cls, path):
        pass

    @classmethod
    @abstractmethod
    def isfile(cls, path):
        pass

    @classmethod
    @abstractmethod
    def isdir(cls, path):
        pass

    @classmethod
    def rm_if_exist(cls, filename):
        pass

    @classmethod
    def symlink(cls, src_path, dst_path, **kwargs):
        pass

    @classmethod
    @abstractmethod
    def open(cls, *args, **kwargs):
        pass

    @classmethod
    @abstractmethod
    def open_wt(cls, path):
        pass

    @classmethod
    @abstractmethod
    def open_rt(cls, path):
        pass

    @classmethod
    @abstractmethod
    def mkdir_p(cls, path):
        pass

    @classmethod
    @abstractmethod
    def imwrite(cls, filename, image):
        pass

    @classmethod
    @abstractmethod
    def imread(cls, path, grayscale=False, unchanged=False, anydepth=False):
        pass

    @classmethod
    @abstractmethod
    def image_size(cls, filename):
        pass

    @classmethod
    @abstractmethod
    def timestamp(cls, path):
        pass


class IoFilesystemDefault(IoFilesystemBase):
    def __init__(self):
        self.type = "default"

    @classmethod
    def exists(cls, path):
        return os.path.exists(path)

    @classmethod
    def ls(cls, path):
        return os.listdir(path)

    @classmethod
    def isfile(cls, path):
        return os.path.isfile(path)

    @classmethod
    def isdir(cls, path):
        return os.path.isdir(path)

    @classmethod
    def rm_if_exist(cls, filename):
        if os.path.islink(filename):
            os.unlink(filename)
        if os.path.exists(filename):
            if os.path.isdir(filename):
                shutil.rmtree(filename)
            else:
                os.remove(filename)

    @classmethod
    def symlink(cls, src_path, dst_path, **kwargs):
        os.symlink(src_path, dst_path, **kwargs)

    @classmethod
    def open(cls, *args, **kwargs):
        return open(*args, **kwargs)

    @classmethod
    def open_wt(cls, path):
        return cls.open(path, "w", encoding="utf-8")

    @classmethod
    def open_rt(cls, path):
        return cls.open(path, "r", encoding="utf-8")

    @classmethod
    def mkdir_p(cls, path):
        return os.makedirs(path, exist_ok=True)

    @classmethod
    def imread(cls, path, grayscale=False, unchanged=False, anydepth=False):
        with cls.open(path, "rb") as fb:
            return imread_from_fileobject(fb, grayscale, unchanged, anydepth)

    @classmethod
    def imwrite(cls, path, image):
        with cls.open(path, "wb") as fwb:
            imwrite_from_fileobject(fwb, image, path)

    @classmethod
    def image_size(cls, path):
        with cls.open(path, "rb") as fb:
            return image_size_from_fileobject(fb)

    @classmethod
    def timestamp(cls, path):
        return os.path.getmtime(path)
