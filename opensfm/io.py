import errno
import json
import logging
import os

import cv2
import numpy as np
import pyproj

from opensfm import features
from opensfm import geo
from opensfm import types
from opensfm import context


logger = logging.getLogger(__name__)


def camera_from_json(key, obj):
    """
    Read camera from a json object
    """
    pt = obj.get('projection_type', 'perspective')
    if pt == 'perspective':
        camera = types.PerspectiveCamera()
        camera.id = key
        camera.width = obj.get('width', 0)
        camera.height = obj.get('height', 0)
        camera.focal = obj['focal']
        camera.k1 = obj.get('k1', 0.0)
        camera.k2 = obj.get('k2', 0.0)
        camera.focal_prior = obj.get('focal_prior', camera.focal)
        camera.k1_prior = obj.get('k1_prior', camera.k1)
        camera.k2_prior = obj.get('k2_prior', camera.k2)
        return camera
    if pt == 'brown':
        camera = types.BrownPerspectiveCamera()
        camera.id = key
        camera.width = obj.get('width', 0)
        camera.height = obj.get('height', 0)
        camera.focal_x = obj['focal_x']
        camera.focal_y = obj['focal_y']
        camera.c_x = obj.get('c_x', 0.0)
        camera.c_y = obj.get('c_y', 0.0)
        camera.k1 = obj.get('k1', 0.0)
        camera.k2 = obj.get('k2', 0.0)
        camera.p1 = obj.get('p1', 0.0)
        camera.p2 = obj.get('p2', 0.0)
        camera.k3 = obj.get('k3', 0.0)
        camera.focal_x_prior = obj.get('focal_x_prior', camera.focal_x)
        camera.focal_y_prior = obj.get('focal_y_prior', camera.focal_y)
        camera.c_x_prior = obj.get('c_x_prior', camera.c_x)
        camera.c_y_prior = obj.get('c_y_prior', camera.c_y)
        camera.k1_prior = obj.get('k1_prior', camera.k1)
        camera.k2_prior = obj.get('k2_prior', camera.k2)
        camera.p1_prior = obj.get('p1_prior', camera.k1)
        camera.p2_prior = obj.get('p2_prior', camera.k2)
        camera.k3_prior = obj.get('k3_prior', camera.k1)
        return camera
    elif pt == 'fisheye':
        camera = types.FisheyeCamera()
        camera.id = key
        camera.width = obj.get('width', 0)
        camera.height = obj.get('height', 0)
        camera.focal = obj['focal']
        camera.k1 = obj.get('k1', 0.0)
        camera.k2 = obj.get('k2', 0.0)
        camera.focal_prior = obj.get('focal_prior', camera.focal)
        camera.k1_prior = obj.get('k1_prior', camera.k1)
        camera.k2_prior = obj.get('k2_prior', camera.k2)
        return camera
    elif pt in ['equirectangular', 'spherical']:
        camera = types.SphericalCamera()
        camera.id = key
        camera.width = obj['width']
        camera.height = obj['height']
        return camera
    else:
        raise NotImplementedError


def shot_from_json(key, obj, cameras):
    """
    Read shot from a json object
    """
    pose = types.Pose()
    pose.rotation = obj["rotation"]
    if "translation" in obj:
        pose.translation = obj["translation"]

    metadata = types.ShotMetadata()
    metadata.orientation = obj.get("orientation")
    metadata.capture_time = obj.get("capture_time")
    metadata.gps_dop = obj.get("gps_dop")
    metadata.gps_position = obj.get("gps_position")

    shot = types.Shot()
    shot.id = key
    shot.metadata = metadata
    shot.pose = pose
    shot.camera = cameras.get(obj["camera"])

    if 'scale' in obj:
        shot.scale = obj['scale']
    if 'covariance' in obj:
        shot.covariance = np.array(obj['covariance'])
    if 'merge_cc' in obj:
        shot.merge_cc = obj['merge_cc']
    if 'vertices' in obj and 'faces' in obj:
        shot.mesh = types.ShotMesh()
        shot.mesh.vertices = obj['vertices']
        shot.mesh.faces = obj['faces']

    return shot


def point_from_json(key, obj):
    """
    Read a point from a json object
    """
    point = types.Point()
    point.id = key
    point.color = obj["color"]
    point.coordinates = obj["coordinates"]
    if "reprojection_error" in obj:
        point.reprojection_error = obj["reprojection_error"]
    return point


def reconstruction_from_json(obj):
    """
    Read a reconstruction from a json object
    """
    reconstruction = types.Reconstruction()

    # Extract cameras
    for key, value in obj['cameras'].iteritems():
        camera = camera_from_json(key, value)
        reconstruction.add_camera(camera)

    # Extract shots
    for key, value in obj['shots'].iteritems():
        shot = shot_from_json(key, value, reconstruction.cameras)
        reconstruction.add_shot(shot)

    # Extract points
    if 'points' in obj:
        for key, value in obj['points'].iteritems():
            point = point_from_json(key, value)
            reconstruction.add_point(point)

    # Extract pano_shots
    if 'pano_shots' in obj:
        reconstruction.pano_shots = {}
        for key, value in obj['pano_shots'].iteritems():
            shot = shot_from_json(key, value, reconstruction.cameras)
            reconstruction.pano_shots[shot.id] = shot

    # Extract main and unit shots
    if 'main_shot' in obj:
        reconstruction.main_shot = obj['main_shot']
    if 'unit_shot' in obj:
        reconstruction.unit_shot = obj['unit_shot']

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
    for key, value in obj.iteritems():
        cameras[key] = camera_from_json(key, value)
    return cameras


def camera_to_json(camera):
    """
    Write camera to a json object
    """
    if camera.projection_type == 'perspective':
        return {
            'projection_type': camera.projection_type,
            'width': camera.width,
            'height': camera.height,
            'focal': camera.focal,
            'k1': camera.k1,
            'k2': camera.k2,
            'focal_prior': camera.focal_prior,
            'k1_prior': camera.k1_prior,
            'k2_prior': camera.k2_prior
        }
    elif camera.projection_type == 'brown':
        return {
            'projection_type': camera.projection_type,
            'width': camera.width,
            'height': camera.height,
            'focal_x': camera.focal_x,
            'focal_y': camera.focal_y,
            'c_x': camera.c_x,
            'c_y': camera.c_y,
            'k1': camera.k1,
            'k2': camera.k2,
            'p1': camera.p1,
            'p2': camera.p2,
            'k3': camera.k3,
            'focal_x_prior': camera.focal_x_prior,
            'focal_y_prior': camera.focal_y_prior,
            'c_x_prior': camera.c_x_prior,
            'c_y_prior': camera.c_y_prior,
            'k1_prior': camera.k1_prior,
            'k2_prior': camera.k2_prior,
            'p1_prior': camera.p1_prior,
            'p2_prior': camera.p2_prior,
            'k3_prior': camera.k3_prior
        }
    elif camera.projection_type == 'fisheye':
        return {
            'projection_type': camera.projection_type,
            'width': camera.width,
            'height': camera.height,
            'focal': camera.focal,
            'k1': camera.k1,
            'k2': camera.k2,
            'focal_prior': camera.focal_prior,
            'k1_prior': camera.k1_prior,
            'k2_prior': camera.k2_prior
        }
    elif camera.projection_type in ['equirectangular', 'spherical']:
        return {
            'projection_type': camera.projection_type,
            'width': camera.width,
            'height': camera.height
        }
    else:
        raise NotImplementedError


def shot_to_json(shot):
    """
    Write shot to a json object
    """
    obj = {
        'rotation': list(shot.pose.rotation),
        'translation': list(shot.pose.translation),
        'camera': shot.camera.id
    }
    if shot.metadata is not None:
        if shot.metadata.orientation is not None:
            obj['orientation'] = shot.metadata.orientation
        if shot.metadata.capture_time is not None:
            obj['capture_time'] = shot.metadata.capture_time
        if shot.metadata.gps_dop is not None:
            obj['gps_dop'] = shot.metadata.gps_dop
        if shot.metadata.gps_position is not None:
            obj['gps_position'] = shot.metadata.gps_position
        if shot.metadata.accelerometer is not None:
            obj['accelerometer'] = shot.metadata.accelerometer
        if shot.metadata.compass is not None:
            obj['compass'] = shot.metadata.compass
        if shot.metadata.skey is not None:
            obj['skey'] = shot.metadata.skey
    if shot.mesh is not None:
        obj['vertices'] = shot.mesh.vertices
        obj['faces'] = shot.mesh.faces
    if hasattr(shot, 'scale'):
        obj['scale'] = shot.scale
    if hasattr(shot, 'covariance'):
        obj['covariance'] = shot.covariance.tolist()
    if hasattr(shot, 'merge_cc'):
        obj['merge_cc'] = shot.merge_cc
    return obj


def point_to_json(point):
    """
    Write a point to a json object
    """
    return {
        'color': list(point.color),
        'coordinates': list(point.coordinates),
        'reprojection_error': point.reprojection_error
    }


def reconstruction_to_json(reconstruction):
    """
    Write a reconstruction to a json object
    """
    obj = {
        "cameras": {},
        "shots": {},
        "points": {}
    }

    # Extract cameras
    for camera in reconstruction.cameras.values():
        obj['cameras'][camera.id] = camera_to_json(camera)

    # Extract shots
    for shot in reconstruction.shots.values():
        obj['shots'][shot.id] = shot_to_json(shot)

    # Extract points
    for point in reconstruction.points.values():
        obj['points'][point.id] = point_to_json(point)

    # Extract pano_shots
    if hasattr(reconstruction, 'pano_shots'):
        obj['pano_shots'] = {}
        for shot in reconstruction.pano_shots.values():
            obj['pano_shots'][shot.id] = shot_to_json(shot)

    # Extract main and unit shots
    if hasattr(reconstruction, 'main_shot'):
        obj['main_shot'] = reconstruction.main_shot
    if hasattr(reconstruction, 'unit_shot'):
        obj['unit_shot'] = reconstruction.unit_shot

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


def _read_ground_control_points_list_line(line, projection, reference_lla, exif):
    words = line.split()
    easting, northing, alt, pixel_x, pixel_y = map(float, words[:5])
    shot_id = words[5]

    # Convert 3D coordinates
    if projection is not None:
        lon, lat = projection(easting, northing, inverse=True)
    else:
        lon, lat = easting, northing
    x, y, z = geo.topocentric_from_lla(
        lat, lon, alt,
        reference_lla['latitude'],
        reference_lla['longitude'],
        reference_lla['altitude'])

    # Convert 2D coordinates
    d = exif[shot_id]
    coordinates = features.normalized_image_coordinates(
        np.array([[pixel_x, pixel_y]]), d['width'], d['height'])[0]

    o = types.GroundControlPointObservation()
    o.lla = np.array([lat, lon, alt])
    o.coordinates = np.array([x, y, z])
    o.shot_id = shot_id
    o.shot_coordinates = coordinates
    return o


def _parse_utm_projection_string(line):
    """Convert strings like 'WGS84 UTM 32N' to a proj4 definition."""
    words = line.lower().split()
    assert len(words) == 3
    zone = line.split()[2].upper()
    if zone[-1] == 'N':
        zone_number = int(zone[:-1])
        zone_hemisphere = 'north'
    elif zone[-1] == 'S':
        zone_number = int(zone['-1'])
        zone_hemisphere = 'south'
    else:
        zone_number = int(zone)
        zone_hemisphere = 'north'
    s = '+proj=utm +zone={} +{} +ellps=WGS84 +datum=WGS84 +units=m +no_defs'
    return s.format(zone_number, zone_hemisphere)


def _parse_projection(line):
    """Build a proj4 from the GCP format line."""
    if line.strip() == 'WGS84':
        return None
    elif line.upper().startswith('WGS84 UTM'):
        return pyproj.Proj(_parse_utm_projection_string(line))
    elif '+proj' in line:
        return pyproj.Proj(line)
    else:
        raise ValueError("Un-supported geo system definition: {}".format(line))


def read_ground_control_points_list(fileobj, reference_lla, exif):
    """Read a ground control point list file.

    It requires the points to be in the WGS84 lat, lon, alt format.
    """
    lines = fileobj.readlines()
    projection = _parse_projection(lines[0])
    points = [_read_ground_control_points_list_line(line, projection, reference_lla, exif)
              for line in lines[1:]]
    return points


def mkdir_p(path):
    '''Make a directory including parent directories.
    '''
    try:
        os.makedirs(path)
    except os.error as exc:
        if exc.errno != errno.EEXIST or not os.path.isdir(path):
            raise


def json_dump_kwargs(minify=False, codec='utf-8'):
    if minify:
        indent, separators = None, (',', ':')
    else:
        indent, separators = 4, None
    return dict(indent=indent, ensure_ascii=False,
                encoding=codec, separators=separators)


def json_dump(data, fout, minify=False, codec='utf-8'):
    kwargs = json_dump_kwargs(minify, codec)
    return json.dump(data, fout, **kwargs)


def json_dumps(data, minify=False, codec='utf-8'):
    kwargs = json_dump_kwargs(minify, codec)
    return json.dumps(data, **kwargs)


def json_loads(text, codec='utf-8'):
    return json.loads(text.decode(codec))


def imread(filename):
    """Load image as an RGB array ignoring EXIF orientation."""
    if context.OPENCV3:
        flags = cv2.IMREAD_COLOR
        try:
            flags |= cv2.IMREAD_IGNORE_ORIENTATION
        except AttributeError:
            logger.warning(
                "OpenCV version {} does not support loading images without "
                "rotating them according to EXIF. Please upgrade OpenCV to "
                "version 3.2 or newer.".format(cv2.__version__))
    else:
        flags = cv2.CV_LOAD_IMAGE_COLOR
    bgr = cv2.imread(filename, flags)
    return bgr[:, :, ::-1]  # Turn BGR to RGB


# Bundler

def export_bundler(image_list, reconstructions, track_graph, bundle_file_path,
                   list_file_path):
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
        lines.append(' '.join(map(str, [num_shot, num_point])))
        shots_order = {key: i for i, key in enumerate(image_list)}

        # cameras
        for shot_id in image_list:
            if shot_id in shots:
                shot = shots[shot_id]
                camera = shot.camera
                scale = max(camera.width, camera.height)
                focal = camera.focal * scale
                k1 = camera.k1
                k2 = camera.k2
                R = shot.pose.get_rotation_matrix()
                t = np.array(shot.pose.translation)
                R[1], R[2] = -R[1], -R[2]  # Reverse y and z
                t[1], t[2] = -t[1], -t[2]
                lines.append(' '.join(map(str, [focal, k1, k2])))
                for i in xrange(3):
                    lines.append(' '.join(list(map(str, R[i]))))
                t = ' '.join(map(str, t))
                lines.append(t)
            else:
                for i in range(5):
                    lines.append("0 0 0")

        # tracks
        for point_id, point in points.iteritems():
            coord = point.coordinates
            color = map(int, point.color)
            view_list = track_graph[point_id]
            lines.append(' '.join(map(str, coord)))
            lines.append(' '.join(map(str, color)))
            view_line = []
            for shot_key, view in view_list.iteritems():
                if shot_key in shots.keys():
                    v = view['feature']
                    shot_index = shots_order[shot_key]
                    camera = shots[shot_key].camera
                    scale = max(camera.width, camera.height)
                    x = v[0] * scale
                    y = -v[1] * scale
                    view_line.append(' '.join(
                        map(str, [shot_index, view['feature_id'], x, y])))

            lines.append(str(len(view_line)) + ' ' + ' '.join(view_line))

        bundle_file = os.path.join(bundle_file_path,
                                   'bundle_r' + str(j).zfill(3) + '.out')
        with open(bundle_file, 'wb') as fout:
            fout.writelines('\n'.join(lines) + '\n')

        list_file = os.path.join(list_file_path,
                                 'list_r' + str(j).zfill(3) + '.out')
        with open(list_file, 'wb') as fout:
            fout.writelines('\n'.join(map(str, image_list)))


def import_bundler(data_path, bundle_file, list_file, track_file,
                   reconstruction_file=None):
    """
    Reconstruction and tracks graph from Bundler's output
    """

    # Init OpenSfM working folder.
    mkdir_p(data_path)

    # Copy image list.
    list_dir = os.path.dirname(list_file)
    with open(list_file, 'rb') as fin:
        lines = fin.read().splitlines()
    ordered_shots = []
    image_list = []
    for line in lines:
        image_path = os.path.join(list_dir, line.split()[0])
        rel_to_data = os.path.relpath(image_path, data_path)
        image_list.append(rel_to_data)
        ordered_shots.append(os.path.basename(image_path))
    with open(os.path.join(data_path, 'image_list.txt'), 'w') as fout:
        fout.write('\n'.join(image_list) + '\n')

    # Check for bundle_file
    if not bundle_file or not os.path.isfile(bundle_file):
        return None

    with open(bundle_file, 'rb') as fin:
        lines = fin.readlines()
    offset = 1 if '#' in lines[0] else 0

    # header
    num_shot, num_point = map(int, lines[offset].split(' '))
    offset += 1

    # initialization
    reconstruction = types.Reconstruction()

    # cameras
    for i in xrange(num_shot):
        # Creating a model for each shot.
        shot_key = ordered_shots[i]
        focal, k1, k2 = map(float, lines[offset].rstrip('\n').split(' '))

        if focal > 0:
            im = imread(os.path.join(data_path, image_list[i]))
            height, width = im.shape[0:2]
            camera = types.PerspectiveCamera()
            camera.id = 'camera_' + str(i)
            camera.width = width
            camera.height = height
            camera.focal = focal / max(width, height)
            camera.k1 = k1
            camera.k2 = k2
            reconstruction.add_camera(camera)

            # Shots
            rline = []
            for k in xrange(3):
                rline += lines[offset + 1 + k].rstrip('\n').split(' ')
            R = ' '.join(rline)
            t = lines[offset + 4].rstrip('\n').split(' ')
            R = np.array(map(float, R.split())).reshape(3, 3)
            t = np.array(map(float, t))
            R[1], R[2] = -R[1], -R[2]  # Reverse y and z
            t[1], t[2] = -t[1], -t[2]

            shot = types.Shot()
            shot.id = shot_key
            shot.camera = camera
            shot.pose = types.Pose()
            shot.pose.set_rotation_matrix(R)
            shot.pose.translation = t
            reconstruction.add_shot(shot)
        else:
            print 'ignore failed image', shot_key
        offset += 5

    # tracks
    track_lines = []
    for i in xrange(num_point):
        coordinates = lines[offset].rstrip('\n').split(' ')
        color = lines[offset + 1].rstrip('\n').split(' ')
        point = types.Point()
        point.id = i
        point.coordinates = map(float, coordinates)
        point.color = map(int, color)
        reconstruction.add_point(point)

        view_line = lines[offset + 2].rstrip('\n').split(' ')

        num_view, view_list = int(view_line[0]), view_line[1:]

        for k in xrange(num_view):
            shot_key = ordered_shots[int(view_list[4 * k])]
            if shot_key in reconstruction.shots:
                camera = reconstruction.shots[shot_key].camera
                scale = max(camera.width, camera.height)
                v = '{}\t{}\t{}\t{}\t{}\t{}\t{}\t{}'.format(
                    shot_key,
                    i,
                    view_list[4 * k + 1],
                    float(view_list[4 * k + 2]) / scale,
                    -float(view_list[4 * k + 3]) / scale,
                    point.color[0],
                    point.color[1],
                    point.color[2]
                )
                track_lines.append(v)
        offset += 3

    # save track file
    with open(track_file, 'wb') as fout:
        fout.writelines('\n'.join(track_lines))

    # save reconstruction
    if reconstruction_file is not None:
        with open(reconstruction_file, 'wb') as fout:
            obj = reconstructions_to_json([reconstruction])
            json_dump(obj, fout)
    return reconstruction


# PLY

def reconstruction_to_ply(reconstruction, no_cameras=False, no_points=False):
    """Export reconstruction points as a PLY string."""
    vertices = []

    if not no_points:
        for point in reconstruction.points.values():
            p, c = point.coordinates, point.color
            s = "{} {} {} {} {} {}".format(
                p[0], p[1], p[2], int(c[0]), int(c[1]), int(c[2]))
            vertices.append(s)

    if not no_cameras:
        for shot in reconstruction.shots.values():
            o = shot.pose.get_origin()
            R = shot.pose.get_rotation_matrix()
            for axis in range(3):
                c = 255 * np.eye(3)[axis]
                for depth in np.linspace(0, 1, 10):
                    p = o + depth * R[axis]
                    s = "{} {} {} {} {} {}".format(
                        p[0], p[1], p[2], int(c[0]), int(c[1]), int(c[2]))
                    vertices.append(s)

    header = [
        "ply",
        "format ascii 1.0",
        "element vertex {}".format(len(vertices)),
        "property float x",
        "property float y",
        "property float z",
        "property uchar diffuse_red",
        "property uchar diffuse_green",
        "property uchar diffuse_blue",
        "end_header",
    ]

    return '\n'.join(header + vertices + [''])
