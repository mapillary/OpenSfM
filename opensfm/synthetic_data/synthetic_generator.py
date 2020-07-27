import numpy as np
from collections import defaultdict
import math
import copy
import cv2

from opensfm import geo
from opensfm import types
from opensfm import reconstruction as rc
from opensfm import pysfm


def derivative(func, x):
    eps = 1e-10
    d = (func(x+eps)-func(x))/eps
    d /= np.linalg.norm(d)
    return d


def samples_generator_random_count(count):
    return np.random.rand(count)


def samples_generator_interval(start, length, interval, interval_noise):
    samples = np.linspace(start/length, 1, num=int(length/interval))
    samples += np.random.normal(0.0,
                                float(interval_noise)/float(length),
                                samples.shape)
    return samples


def generate_samples_and_local_frame(samples, shape):
    points = []
    tangents = []
    for i in samples:
        point = shape(i)
        points += [point]
        ex = derivative(shape, i)
        ez = np.array([ex[1], -ex[0]])
        tangents += [np.array([ez, ex])]
    return np.array(points), np.array(tangents)


def generate_samples_shifted(samples, shape, shift):
    plane_points = []
    for i in samples:
        point = shape(i)
        tangent = derivative(shape, i)
        tangent = np.array([-tangent[1], tangent[0]])
        point += tangent*(shift/2)
        plane_points += [point]
    return np.array(plane_points)


def generate_z_plane(samples, shape, thickness):
    plane_points = []
    for i in samples:
        point = shape(i)
        tangent = derivative(shape, i)
        tangent = np.array([-tangent[1], tangent[0]])
        shift = tangent*((np.random.rand()-0.5)*thickness)
        point += shift
        plane_points += [point]
    plane_points = np.array(plane_points)
    return np.insert(plane_points, 2, values=0, axis=1)


def generate_xy_planes(samples, shape, z_size, y_size):
    xy1 = generate_samples_shifted(samples, shape, y_size)
    xy2 = generate_samples_shifted(samples, shape, -y_size)
    xy1 = np.insert(xy1, 2, values=np.random.rand(
        xy1.shape[0])*z_size, axis=1)
    xy2 = np.insert(xy2, 2, values=np.random.rand(
        xy2.shape[0])*z_size, axis=1)
    return np.concatenate((xy1, xy2), axis=0)


def generate_street(samples, shape, height, width):
    walls = generate_xy_planes(samples, shape, height, width)
    floor = generate_z_plane(samples, shape, width)
    return walls, floor


def generate_cameras(samples, shape, height):
    positions, rotations = generate_samples_and_local_frame(samples, shape)
    positions = np.insert(positions, 2, values=height, axis=1)
    rotations = np.insert(rotations, 2, values=0, axis=2)
    rotations = np.insert(rotations, 1, values=np.array([0, 0, -1]), axis=1)
    return positions, rotations


def line_generator(length, point):
    x = point*length
    return np.transpose(np.array([x, 0]))


def weird_curve(length, point):
    y = math.cos(point+0.1*point**2+2.3*point**4 +
                 0.8*point**7+0.1*point**9)
    return length*np.transpose(np.array([point, y]))


def ellipse_generator(x_size, y_size, point):
    y = np.sin(point*2*np.pi)*y_size/2
    x = np.cos(point*2*np.pi)*x_size/2
    return np.transpose(np.array([x, y]))


def perturb_points(points, sigmas):
    eps = 1e-10
    for point in points:
        sigmas = np.array([max(s, eps) for s in sigmas])
        point += np.random.normal(0.0, sigmas,
                                  point.shape)


def generate_exifs(reconstruction, gps_noise, speed_ms=10):
    """Generate fake exif metadata from the reconstruction."""
    previous_pose = None
    previous_time = 0
    exifs = {}
    reference = geo.TopocentricConverter(0, 0, 0)
    for shot_name in sorted(reconstruction.shots.keys()):
        shot = reconstruction.shots[shot_name]
        exif = {}
        exif['width'] = shot.camera.width
        exif['height'] = shot.camera.height
        exif['focal_ratio'] = shot.camera.focal
        exif['camera'] = str(shot.camera.id)
        exif['make'] = str(shot.camera.id)

        pose = shot.pose.get_origin()

        if previous_pose is not None:
            previous_time += np.linalg.norm(pose-previous_pose)*speed_ms
        previous_pose = pose
        exif['capture_time'] = previous_time

        perturb_points([pose], [gps_noise, gps_noise, gps_noise])

        shot_copy = copy.deepcopy(shot)
        shot_copy.pose.set_origin(pose)
        lat, lon, alt, comp = rc.shot_lla_and_compass(shot_copy, reference)

        exif['gps'] = {}
        exif['gps']['latitude'] = lat
        exif['gps']['longitude'] = lon
        exif['gps']['altitude'] = alt
        exif['gps']['dop'] = gps_noise
        exif['compass'] = {'angle': comp}
        exifs[shot_name] = exif
    return exifs


def perturb_rotations(rotations, angle_sigma):
    for i in range(len(rotations)):
        rotation = rotations[i]
        rodrigues = cv2.Rodrigues(rotation)[0].ravel()
        angle = np.linalg.norm(rodrigues)
        angle_pertubed = angle + np.random.normal(0.0, angle_sigma)
        rodrigues *= (float(angle_pertubed)/float(angle))
        rotations[i] = cv2.Rodrigues(rodrigues)[0]


def add_shots_to_reconstruction(positions, rotations,
                                camera, reconstruction):
    shift = len(reconstruction.shots)
    for i, item in enumerate(zip(positions, rotations)):
        shot = types.Shot()
        shot.id = 'shot%04d' % (shift + i)
        shot.camera = camera
        shot.pose = types.Pose()
        shot.pose.set_rotation_matrix(item[1])
        shot.pose.set_origin(item[0])
        reconstruction.add_shot(shot)
    reconstruction.add_camera(camera)


def add_points_to_reconstruction(points, color, reconstruction):
    shift = len(reconstruction.points)
    for i in range(points.shape[0]):
        point = types.Point()
        point.id = str(shift+i)
        point.coordinates = points[i, :]
        point.color = color
        reconstruction.add_point(point)


def create_reconstruction(points, colors,
                          cameras, positions,
                          rotations):
    reconstruction = types.Reconstruction()
    for item in zip(points, colors):
        add_points_to_reconstruction(item[0], item[1], reconstruction)
    for item in zip(positions, rotations, cameras):
        add_shots_to_reconstruction(item[0], item[1], item[2], reconstruction)
    return reconstruction


def generate_track_data(reconstruction, maximum_depth, noise):
    tracks_manager = pysfm.TracksManager()

    feature_data_type = np.float32
    desc_size = 128
    non_zeroes = 5
    track_descriptors = {}
    for track_index in reconstruction.points:
        descriptor = np.zeros(desc_size)
        for i in range(non_zeroes):
            index = np.random.randint(0, desc_size)
            descriptor[index] = np.random.random()*255
        track_descriptors[track_index] = descriptor.round().\
            astype(feature_data_type)

    colors = {}
    features = {}
    descriptors = {}
    default_scale = 0.004
    for shot_index, shot in reconstruction.shots.items():
        # need to have these as we lost track of keys
        all_keys = list(reconstruction.points.keys())
        all_values = list(reconstruction.points.values())

        # temporary work on numpy array
        all_coordinates = [p.coordinates for p in all_values]
        projections = shot.project_many(np.array(all_coordinates))
        projections_inside = []
        descriptors_inside = []
        colors_inside = []
        for i, projection in enumerate(projections):
            if not _is_inside_camera(projection, shot.camera):
                continue
            original_key = all_keys[i]
            original_point = all_values[i]
            if not _is_in_front(original_point, shot):
                continue
            if not _check_depth(original_point, shot, maximum_depth):
                continue

            # add perturbation
            perturbation = float(noise)/float(max(shot.camera.width,
                                                  shot.camera.height))
            perturb_points([projection], np.array([perturbation, perturbation]))

            projections_inside.append(np.hstack((projection, [default_scale])))
            descriptors_inside.append(track_descriptors[original_key])
            colors_inside.append(original_point.color)
            obs = pysfm.Observation(
                projection[0], projection[1], default_scale,
                original_point.color[0], original_point.color[1],
                original_point.color[2], len(projections_inside) - 1)
            tracks_manager.add_observation(str(shot_index), str(original_key), obs)
        features[shot_index] = np.array(projections_inside)
        colors[shot_index] = np.array(colors_inside)
        descriptors[shot_index] = np.array(descriptors_inside)

    return features, descriptors, colors, tracks_manager


def _check_depth(point, shot, maximum_depth):
    return shot.pose.transform(point.coordinates)[2] < maximum_depth


def _is_in_front(point, shot):
    return np.dot((point.coordinates - shot.pose.get_origin()),
                  shot.pose.get_rotation_matrix()[2]) > 0


def _is_inside_camera(projection, camera):
    if camera.width > camera.height:
        return (-0.5 < projection[0] < 0.5) and \
            (-float(camera.height)/float(2*camera.width) < projection[1]
                < float(camera.height)/float(2*camera.width))
    else:
        return (-0.5 < projection[1] < 0.5) and \
            (-float(camera.width)/float(2*camera.height) < projection[0]
                < float(camera.width)/float(2*camera.height))
