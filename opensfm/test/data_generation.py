import os

import networkx as nx
import numpy as np
from six import iteritems

from opensfm import types
import opensfm.dataset


def normalized(x):
    return x / np.linalg.norm(x)


def camera_pose(position, lookat, up):
    '''
    Pose from position and look at direction

    >>> position = [1.0, 2.0, 3.0]
    >>> lookat = [0., 10.0, 2.0]
    >>> up = [0.0, 0.0, 1.0]
    >>> pose = camera_pose(position, lookat, up)
    >>> np.allclose(pose.get_origin(), position)
    True
    >>> d = normalized(pose.transform(lookat))
    >>> np.allclose(d, [0, 0, 1])
    True
    '''
    ez = normalized(np.array(lookat) - np.array(position))
    ex = normalized(np.cross(ez, up))
    ey = normalized(np.cross(ez, ex))
    pose = types.Pose()
    pose.set_rotation_matrix([ex, ey, ez])
    pose.set_origin(position)
    return pose


class CubeDataset:
    '''
    Dataset of cameras looking at point in a cube

    >>> d = CubeDataset(3, 10, 0.1, 0.3)
    >>> len(d.cameras)
    3
    >>> len(d.shots)
    3
    >>> len(d.points)
    10
    '''
    def __init__(self, num_cameras, num_points, noise, outlier_fraction):
        self.cameras = {}
        for i in range(num_cameras):
            camera = types.PerspectiveCamera()
            camera.id = 'camera' + str(i)
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

            shot = types.Shot()
            shot.id = 'shot' + str(i)
            shot.camera = self.cameras['camera' + str(i)]
            shot.pose = camera_pose(position, lookat, up)
            self.shots[shot.id] = shot

        points = np.random.rand(num_points, 3)
        self.points = {'point' + str(i): p for i, p in enumerate(points)}

        g = nx.Graph()
        for shot_id, shot in iteritems(self.shots):
            for point_id, point in iteritems(self.points):
                feature = shot.project(point)
                g.add_node(shot_id, bipartite=0)
                g.add_node(point_id, bipartite=1)
                g.add_edge(shot_id, point_id, feature=feature,
                           feature_id=point_id, feature_color=(0, 0, 0))
        self.tracks = g


def create_berlin_test_folder(tmpdir):
    path = str(tmpdir.mkdir('berlin'))
    os.symlink(os.path.abspath('data/berlin/images'),
               os.path.join(path, 'images'))
    return opensfm.dataset.DataSet(path)
