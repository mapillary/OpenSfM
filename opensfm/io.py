import json
import logging
import os
from typing import List

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


def shot_from_json(reconstruction, key, obj, is_pano_shot=False):
    """
    Read shot from a json object
    """
    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    if "translation" in obj:
        pose.translation = obj["translation"]

    if is_pano_shot:
        shot = reconstruction.create_pano_shot(key, obj["camera"], pose)
    else:
        shot = reconstruction.create_shot(key, obj["camera"], pose)
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

    return shot


def point_from_json(reconstruction, key, obj):
    """
    Read a point from a json object
    """
    point = reconstruction.create_point(key, obj["coordinates"])
    point.color = obj["color"]
    return point


def rig_model_from_json(key, obj):
    """
    Read a rig model from a json object
    """
    rig_model = pymap.RigModel(key)
    for key, rig_camera in obj["rig_cameras"].items():
        rig_model.add_rig_camera(rig_camera_from_json(key, rig_camera))
    return rig_model


def rig_models_from_json(obj):
    """
    Read rig models from a json object
    """
    rig_models = {}
    for key, value in obj.items():
        rig_models[key] = rig_model_from_json(key, value)
    return rig_models


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
    rig_model_id = obj["rig_model_id"]
    reconstruction.add_rig_instance(
        pymap.RigInstance(reconstruction.rig_models[rig_model_id], instance_id)
    )

    pose = pygeometry.Pose()
    pose.rotation = obj["rotation"]
    pose.translation = obj["translation"]
    reconstruction.rig_instances[instance_id].pose = pose

    for shot_id, rig_camera_id in obj["rig_camera_ids"].items():
        reconstruction.rig_instances[instance_id].add_shot(
            rig_camera_id, reconstruction.shots[shot_id]
        )


def reconstruction_from_json(obj):
    """
    Read a reconstruction from a json object
    """
    reconstruction = types.Reconstruction()

    # Extract cameras
    for key, value in obj["cameras"].items():
        camera = camera_from_json(key, value)
        reconstruction.add_camera(camera)

    # Extract rig models
    if "rig_models" in obj:
        for key, value in obj["rig_models"].items():
            reconstruction.add_rig_model(rig_model_from_json(key, value))

    # Extract shots
    for key, value in obj["shots"].items():
        shot_from_json(reconstruction, key, value)

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
            shot_from_json(reconstruction, key, value, is_pano_shot)

    # Extract main and unit shots
    if "main_shot" in obj:
        reconstruction.main_shot = obj["main_shot"]
    if "unit_shot" in obj:
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
        "rig_model_id": rig_instance.rig_model.id,
        "translation": list(rig_instance.pose.translation),
        "rotation": list(rig_instance.pose.rotation),
        "rig_camera_ids": rig_instance.camera_ids,
    }


def rig_model_to_json(rig_model):
    """
    Write a rig model to a json object
    """
    json_rig_cameras = {}
    for rig_camera in rig_model.get_rig_cameras().values():
        json_rig_cameras[rig_camera.id] = rig_camera_to_json(rig_camera)
    return {"rig_cameras": json_rig_cameras}


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


def reconstruction_to_json(reconstruction):
    """
    Write a reconstruction to a json object
    """
    obj = {"cameras": {}, "shots": {}, "points": {}}

    # Extract cameras
    for camera in reconstruction.cameras.values():
        obj["cameras"][camera.id] = camera_to_json(camera)

    # Extract rig models
    if len(reconstruction.rig_models):
        obj["rig_models"] = {}
    for rig_model in reconstruction.rig_models.values():
        obj["rig_models"][rig_model.id] = rig_model_to_json(rig_model)
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


def reconstructions_to_json(reconstructions):
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


def rig_models_to_json(rig_models):
    """
    Write rig models to a json object
    """
    obj = {}
    for rig_model in rig_models.values():
        obj[rig_model.id] = rig_model_to_json(rig_model)
    return obj


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
    parameters: List[float],
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


def mkdir_p(path):
    """Make a directory including parent directories."""
    return os.makedirs(path, exist_ok=True)


def open_wt(path):
    """Open a file in text mode for writing utf-8."""
    return open(path, "w", encoding="utf-8")


def open_rt(path):
    """Open a file in text mode for reading utf-8."""
    return open(path, "r", encoding="utf-8")


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


def imread(filename, grayscale=False, unchanged=False, anydepth=False):
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

    image = cv2.imread(filename, flags)

    if image is None:
        raise IOError("Unable to load image {}".format(filename))

    if len(image.shape) == 3:
        image[:, :, :3] = image[:, :, [2, 1, 0]]  # Turn BGR to RGB (or BGRA to RGBA)
    return image


def imwrite(filename, image):
    """Write an image to a file"""
    if len(image.shape) == 3:
        image[:, :, :3] = image[:, :, [2, 1, 0]]  # Turn RGB to BGR (or RGBA to BGRA)
    cv2.imwrite(filename, image)


def image_size(filename):
    """Height and width of an image."""
    try:
        with Image.open(filename) as img:
            width, height = img.size
            return height, width
    except Exception:
        # Slower fallback
        image = imread(filename)
        return image.shape[:2]


# Bundler


def export_bundler(
    image_list, reconstructions, track_manager, bundle_file_path, list_file_path
):
    """
    Generate a reconstruction file that is consistent with Bundler's format
    """

    mkdir_p(bundle_file_path)
    mkdir_p(list_file_path)

    for j, reconstruction in enumerate(reconstructions):
        lines = []
        lines.append("# Bundle file v0.3")
        points = reconstruction.points
        shots = reconstruction.shots
        num_point = len(points)
        num_shot = len(image_list)
        lines.append(" ".join(map(str, [num_shot, num_point])))
        shots_order = {key: i for i, key in enumerate(image_list)}

        # cameras
        for shot_id in image_list:
            if shot_id in shots:
                shot = shots[shot_id]
                camera = shot.camera
                if shot.camera.projection_type == "brown":
                    # Will aproximate Brown model, not optimal
                    focal_normalized = camera.focal_x
                else:
                    focal_normalized = camera.focal
                scale = max(camera.width, camera.height)
                focal = focal_normalized * scale
                k1 = camera.k1
                k2 = camera.k2
                R = shot.pose.get_rotation_matrix()
                t = np.array(shot.pose.translation)
                R[1], R[2] = -R[1], -R[2]  # Reverse y and z
                t[1], t[2] = -t[1], -t[2]
                lines.append(" ".join(map(str, [focal, k1, k2])))
                for i in range(3):
                    lines.append(" ".join(map(str, R[i])))
                t = " ".join(map(str, t))
                lines.append(t)
            else:
                for _ in range(5):
                    lines.append("0 0 0")

        # tracks
        for point in points.values():
            coord = point.coordinates
            color = list(map(int, point.color))
            view_list = track_manager.get_track_observations(point.id)
            lines.append(" ".join(map(str, coord)))
            lines.append(" ".join(map(str, color)))
            view_line = []
            for shot_key, obs in view_list.items():
                if shot_key in shots.keys():
                    v = obs.point
                    shot_index = shots_order[shot_key]
                    camera = shots[shot_key].camera
                    scale = max(camera.width, camera.height)
                    x = v[0] * scale
                    y = -v[1] * scale
                    view_line.append(" ".join(map(str, [shot_index, obs.id, x, y])))

            lines.append(str(len(view_line)) + " " + " ".join(view_line))

        bundle_file = os.path.join(
            bundle_file_path, "bundle_r" + str(j).zfill(3) + ".out"
        )
        with open_wt(bundle_file) as fout:
            fout.writelines("\n".join(lines) + "\n")

        list_file = os.path.join(list_file_path, "list_r" + str(j).zfill(3) + ".out")
        with open_wt(list_file) as fout:
            fout.writelines("\n".join(map(str, image_list)))


def import_bundler(
    data_path, bundle_file, list_file, track_file, reconstruction_file=None
):
    """
    Reconstruction and tracks graph from Bundler's output
    """

    # Init OpenSfM working folder.
    mkdir_p(data_path)

    # Copy image list.
    list_dir = os.path.dirname(list_file)
    with open_rt(list_file) as fin:
        lines = fin.read().splitlines()
    ordered_shots = []
    image_list = []
    for line in lines:
        image_path = os.path.join(list_dir, line.split()[0])
        rel_to_data = os.path.relpath(image_path, data_path)
        image_list.append(rel_to_data)
        ordered_shots.append(os.path.basename(image_path))
    with open_wt(os.path.join(data_path, "image_list.txt")) as fout:
        fout.write("\n".join(image_list) + "\n")

    # Check for bundle_file
    if not bundle_file or not os.path.isfile(bundle_file):
        return None

    with open_rt(bundle_file) as fin:
        lines = fin.readlines()
    offset = 1 if "#" in lines[0] else 0

    # header
    num_shot, num_point = map(int, lines[offset].split(" "))
    offset += 1

    # initialization
    reconstruction = types.Reconstruction()

    # cameras
    for i in range(num_shot):
        # Creating a model for each shot.
        shot_key = ordered_shots[i]
        focal, k1, k2 = map(float, lines[offset].rstrip("\n").split(" "))

        if focal > 0:
            im = imread(os.path.join(data_path, image_list[i]))
            height, width = im.shape[0:2]
            camera = pygeometry.Camera.create_perspective(
                focal / max(width, height), k1, k2
            )
            camera.id = "camera_" + str(i)
            camera.width = width
            camera.height = height
            reconstruction.add_camera(camera)

            # Shots
            rline = []
            for k in range(3):
                rline += lines[offset + 1 + k].rstrip("\n").split(" ")
            R = " ".join(rline)
            t = lines[offset + 4].rstrip("\n").split(" ")
            R = np.array(list(map(float, R.split()))).reshape(3, 3)
            t = np.array(list(map(float, t)))
            R[1], R[2] = -R[1], -R[2]  # Reverse y and z
            t[1], t[2] = -t[1], -t[2]
            pose = pygeometry.Pose()
            pose.set_rotation_matrix(R)
            pose.translation = t

            reconstruction.create_shot(shot_key, camera.id, pose)
        else:
            logger.warning("ignoring failed image {}".format(shot_key))
        offset += 5

    # tracks
    track_lines = []
    for i in range(num_point):
        coordinates = lines[offset].rstrip("\n").split(" ")
        color = lines[offset + 1].rstrip("\n").split(" ")
        point = reconstruction.create_point(i, list(map(float, coordinates)))
        point.color = list(map(int, color))

        view_line = lines[offset + 2].rstrip("\n").split(" ")

        num_view, view_list = int(view_line[0]), view_line[1:]

        for k in range(num_view):
            shot_key = ordered_shots[int(view_list[4 * k])]
            if shot_key in reconstruction.shots:
                camera = reconstruction.shots[shot_key].camera
                scale = max(camera.width, camera.height)
                v = "{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}".format(
                    shot_key,
                    i,
                    view_list[4 * k + 1],
                    float(view_list[4 * k + 2]) / scale,
                    -float(view_list[4 * k + 3]) / scale,
                    point.color[0],
                    point.color[1],
                    point.color[2],
                )
                track_lines.append(v)
        offset += 3

    # save track file
    with open_wt(track_file) as fout:
        fout.writelines("\n".join(track_lines))

    # save reconstruction
    if reconstruction_file is not None:
        with open_wt(reconstruction_file) as fout:
            obj = reconstructions_to_json([reconstruction])
            json_dump(obj, fout)
    return reconstruction


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


def ply_to_points(filename):
    points, normals, colors = [], [], []
    with open(filename, "r") as fin:
        line = fin.readline()
        while "end_header" not in line:
            line = fin.readline()
        line = fin.readline()
        while line != "":
            line = fin.readline()
            tokens = line.rstrip().split(" ")
            if len(tokens) == 6 or len(tokens) == 7:  # XYZ and RGB(A)
                x, y, z, r, g, b = tokens[0:6]
                nx, ny, nz = 0, 0, 0
            elif len(tokens) > 7:  # XYZ + Normal + RGB
                x, y, z = tokens[0:3]
                nx, ny, nz = tokens[3:6]
                r, g, b = tokens[6:9]
            else:
                break
            points.append([float(x), float(y), float(z)])
            normals.append([float(nx), float(ny), float(nz)])
            colors.append([int(r), int(g), int(b)])
    return np.array(points), np.array(normals), np.array(colors)


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
