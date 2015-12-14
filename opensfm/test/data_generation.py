import networkx as nx
import numpy as np
import cv2

from opensfm import types


def normalized(x):
    return x / np.linalg.norm(x)


def camera_pose(position, lookat, up):
    '''
    Pose from position and look at direction

    >>> position = [1.0, 2.0, 3.0]
    >>> lookat = [0., 10.0, 2.0]
    >>> up = [0.0, 0.0, 1.0]
    >>> R, t = camera_pose(position, lookat, up)
    >>> o = -np.dot(R.T, t)
    >>> np.allclose(o, position)
    True
    >>> d = normalized(np.dot(R, lookat) + t)
    >>> np.allclose(d, [0, 0, 1])
    True
    '''
    ez = normalized(np.array(lookat) - np.array(position))
    ex = normalized(np.cross(ez, up))
    ey = normalized(np.cross(ez, ex))
    R = np.array([ex, ey, ez])
    t = -R.dot(position)
    return R, t


def add_noise(noise_level, clean_point):
    noisy_point = clean_point + np.random.uniform(-noise_level, noise_level, 3)
    return normalized(noisy_point)


class CubeDataset:
    '''
    Dataset of cameras looking at point in a cube

    >>> d = CubeDataset(3, 10, 0.1, 0.3)
    '''
    def __init__(self, num_cameras, num_points, noise, outlier_fraction):
        self.cameras = {}
        for i in range(num_cameras):
            camera = types.PerspectiveCamera()
            camera.id = str(i)
            camera.focal = 0.9
            camera.k1 = -0.1
            camera.k2 = 0.01
            camera.height = 600
            camera.width = 800
            self.cameras[camera.id] = camera

        self.shots = {}
        for i in range(num_cameras):
            alpha = float(i) / (num_cameras - 1)
            position = [alpha, -5.0, 0.5]
            lookat = [1.0 - alpha, alpha, alpha]
            up = [alpha * 0.2, alpha * 0.2, 1.0]
            R, t = camera_pose(position, lookat, up)

            pose = types.Pose()
            pose.rotation = cv2.Rodrigues(R)[0].ravel()
            pose.translation = t

            shot = types.Shot()
            shot.id = str(i)
            shot.camera = self.cameras[shot.id]
            shot.pose = pose
            self.shots[shot.id] = shot

        points = np.random.rand(num_points, 3)
        self.points = {str(i): p for p in points}

        g = nx.Graph()
        for shot_id, shot in self.shots.iteritems():
            for point_id, point in self.points.iteritems():
                feature = shot.project(point)
                g.add_node(shot_id, bipartite=0)
                g.add_node(point_id, bipartite=1)
                g.add_edge(shot_id, point_id, feature=feature,
                           feature_id=point_id, feature_color=(0, 0, 0))
        self.tracks = g
