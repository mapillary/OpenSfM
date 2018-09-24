import numpy as np
import functools
import math

from opensfm import types
from opensfm import io


def derivative(func, x):
    eps = 1e-10
    d = (func(x+eps)-func(x))/eps
    d /= np.linalg.norm(d)
    return d


def samples_generator_random_count(count):
    return np.random.rand(count)


def samples_generator_interval(length, interval):
    return np.linspace(0, 1, num=length/interval)


def generate_samples_and_local_frame(samples, shape):
    points = []
    tangents = []
    for i in samples:
        point = shape(i)
        points += [point]
        ez = derivative(shape, i)
        ex = np.array([-ez[1], ez[0]])
        tangents += [np.array([ex, ez])]
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
        shift = tangent*((np.random.rand()-0.5)*thickness/2)
        point += shift
        plane_points += [point]
    plane_points = np.array(plane_points)
    return np.insert(plane_points, 2, values=0, axis=1)


def generate_xy_planes(samples, shape, z_size, y_size):
    xy1 = generate_samples_shifted(samples, shape, y_size/2)
    xy2 = generate_samples_shifted(samples, shape, -y_size/2)
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
    rotations = np.insert(rotations, 1, values=np.array([0, 0, 1]), axis=1)
    return positions, rotations


def line_generator(length, point):
    x = point*length
    return np.transpose(np.array([x, 0]))


def ellipse_generator(x_size, y_size, point):
    y = np.sin(point*2*np.pi)*y_size/2
    x = np.cos(point*2*np.pi)*x_size/2
    return np.transpose(np.array([x, y]))


def sample_camera():
    camera = types.PerspectiveCamera()
    camera.id = 'camera'
    camera.focal = 0.9
    camera.k1 = -0.1
    camera.k2 = 0.01
    camera.height = 600
    camera.width = 800
    return camera


def add_shots_to_reconstruction(positions, rotations, camera, reconstruction):
    shift = len(reconstruction.shots)
    for i, item in enumerate(zip(positions, rotations)):
        shot = types.Shot()
        shot.id = 'shot' + str(shift+i)
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
        point.id = shift+i
        point.coordinates = points[i, :]
        point.color = color
        reconstruction.add_point(point)


def create_reconstruction(points, colors,
                          cameras, positions, rotations):
    reconstruction = types.Reconstruction()
    for item in zip(points, colors):
        add_points_to_reconstruction(item[0], item[1], reconstruction)
    for item in zip(positions, rotations, cameras):
        add_shots_to_reconstruction(item[0], item[1], item[2], reconstruction)
    return reconstruction


def generate_corridor_data():
    width = 30
    height = 18
    length = 500
    ellipse_ratio = 4
    points_count = 100000

    # generator = functools.partial(ellipse_generator, length,
    #                               length/ellipse_ratio)
    generator = functools.partial(line_generator, length)

    wall_points, floor_points = generate_street(
        samples_generator_random_count(
            points_count), generator,
        height, width)
    positions, rotations = generate_cameras(samples_generator_interval(length, 3),
                                            generator, 1.5)

    reconstruction = create_reconstruction(
        [floor_points, wall_points],
        [[120, 90, 10], [10, 90, 130]],
        [sample_camera()], [positions], [rotations])
    ply_file = io.reconstruction_to_ply(reconstruction)
    with open("/home/yann/corridor.ply", "w") as ply_output:
        ply_output.write(ply_file)


generate_corridor_data()
