import logging
import matplotlib.pyplot as plt
import numpy as np
import cv2

from timeit import default_timer as timer
from collections import defaultdict
from opensfm import features
from opensfm import pygeometry
from opensfm import pyslam
logger = logging.getLogger(__name__)

disable_debug = True


class AvgTimings(object):
    def __init__(self):
        self.times = defaultdict(float)
        self.n_mean = defaultdict(int)

    def addTimes(self, timings):
        for (_, (k, v, _)) in timings.items():
            self.times[k] += v
            self.n_mean[k] += 1

    def printAvgTimings(self):
        for (k, v) in self.n_mean.items():
            print("{} with {} runs: {}s".format(k, v, self.times[k]/v))


avg_timings = AvgTimings()


class Chronometer:
    def __init__(self):
        self.start()

    def start(self):
        t = timer()
        lap = ('start', 0, t)
        self.laps = [lap]
        self.laps_dict = {'start': lap}

    def lap(self, key):
        t = timer()
        dt = t - self.laps[-1][2]
        lap = (key, dt, t)
        self.laps.append(lap)
        self.laps_dict[key] = lap

    def lap_time(self, key):
        return self.laps_dict[key][1]

    def lap_times(self):
        return [(k, dt) for k, dt, t in self.laps[1:]]

    def total_time(self):
        return self.laps[-1][2] - self.laps[0][2]


def check_shot_for_double_entries(shot):
    added_lms = {}
    for lm, idx in shot.get_valid_landmarks_and_indices():
        if lm in added_lms:
            print("double!!!", lm.id, idx, added_lms[lm])
            exit(0)
        else:
            added_lms[lm] = idx


def visualize_graph(graph, frame1: str, frame2: str, data, do_show=True):
    if disable_debug:
        return
    print("visualize_graph: ", frame1, frame2)
    lms = graph[frame1]
    pts2D_1 = []
    pts2D_2 = []
    for lm_id in lms:
        obs2 = \
            graph.get_edge_data(str(frame2), str(lm_id))
        if obs2 is not None:
            obs1 = \
                graph.get_edge_data(str(frame1), str(lm_id))
            pts2D_1.append(obs1['feature'])
            pts2D_2.append(obs2['feature'])
    if len(pts2D_1) == 0:
        return
    im1 = data.load_image(frame1)
    im2 = data.load_image(frame2)
    h1, w1, c = im1.shape
    fig, ax = plt.subplots(1)

    obs_d1 = features.\
        denormalized_image_coordinates(np.asarray(pts2D_1), w1, h1)
    obs_d2 = features.\
        denormalized_image_coordinates(np.asarray(pts2D_2), w1, h1)
    print("len(obs_d1): ", len(obs_d1), "len(obs_d2): ", len(obs_d2))
    im = np.hstack((im1, im2))
    ax.imshow(im)
    ax.scatter(obs_d1[:, 0], obs_d1[:, 1], c=[[0, 1, 0]])
    ax.scatter(w1+obs_d2[:, 0], obs_d2[:, 1], c=[[0, 1, 0]])
    ax.set_title(frame1 + "<->" + frame2)

    if do_show:
        plt.show()


def reproject_landmarks(points3D, observations, T_world_to_cam,
                        im, camera, title="", obs_normalized=True, do_show=True):
    """Draw observations and reprojects observations into image"""
    if disable_debug:
        return
    if points3D is None:  # or observations is None:
        return
    if len(points3D) == 0:  # or len(observations) == 0:
        return
    pose_world_to_cam = pygeometry.Pose()
    pose_world_to_cam.set_rotation_matrix(T_world_to_cam[0:3, 0:3])
    pose_world_to_cam.translation = T_world_to_cam[0:3, 3]
    legend = ['reproj']
    camera_point = pose_world_to_cam.transform_many(points3D)
    points2D = camera.project_many(camera_point)
    fig, ax = plt.subplots(1)
    if len(im.shape) == 3:
        h1, w1, c = im.shape
    else:
        h1, w1 = im.shape
    pt = features.denormalized_image_coordinates(points2D, w1, h1)
    ax.imshow(im)
    ax.scatter(pt[:, 0], pt[:, 1], c=[[1, 0, 0]])
    if observations is not None:
        if obs_normalized:
            obs = features.denormalized_image_coordinates(observations, w1, h1)
        else:
            obs = observations
        ax.scatter(obs[:, 0], obs[:, 1], c=[[0, 1, 0]])
        legend.append('observation')
    ax.set_title(title)
    ax.legend(legend)
    if do_show:
        plt.show()


def visualize_matches_pts(pts1, pts2, matches, im1, im2, is_normalized= True, do_show=True, title = ""):
    if disable_debug:
        return
    if matches is None:
        matches = np.column_stack((np.arange(len(pts1)), np.arange(len(pts1))))
    if len(matches) == 0:
        return
    if len(im1.shape) == 3:
        h1, w1, c = im1.shape
    else:
        h1, w1 = im1.shape

    pts1 = np.asarray(pts1)
    pts2 = np.asarray(pts2)
    fig, ax = plt.subplots(1)
    im = np.hstack((im1, im2))
    if is_normalized:
        obs_d1 = features.\
            denormalized_image_coordinates(pts1[matches[:, 0]], w1, h1)
        obs_d2 = features.\
            denormalized_image_coordinates(pts2[matches[:, 1]], w1, h1)
    else:
        obs_d1, obs_d2 = pts1[matches[:, 0]], pts2[matches[:, 1]]
    ax.imshow(im)
    skip = 1
    ax.scatter(obs_d1[:, 0], obs_d1[:, 1], c=[[0, 1, 0]])
    ax.scatter(w1+obs_d2[:, 0], obs_d2[:, 1], c=[[0, 1, 0]])
    # for i1, i2 in matches:
    #     ax.text(w1+obs_d2[i2, 0], obs_d2[i2, 1], str(i2))
    for a, b in zip(obs_d1[::skip, :], obs_d2[::skip, :]):
        ax.plot([a[0], b[0] + w1], [a[1], b[1]])
    ax.set_title(title)
    if do_show:
        plt.show()


def visualize_tracked_lms(points2D, shot, data, is_normalized=False):
    # if disable_debug:
        # return
    im1 = data.load_image(shot.id)
    h1, w1, c = im1.shape
    im1 = cv2.cvtColor(im1, cv2.COLOR_RGB2BGR)
    if is_normalized:
        p1d = np.array(features.denormalized_image_coordinates(
            points2D, w1, h1), dtype=int)
        for x, y in p1d:
            cv2.drawMarker(im1, (x, y), (255, 0, 0),
                           markerType=cv2.MARKER_SQUARE, markerSize=10)
    else:
        for x, y,_ in points2D:
            cv2.drawMarker(im1, (x, y), (255, 0, 0),
                           markerType=cv2.MARKER_SQUARE, markerSize=10)
    cv2.imwrite("./debug/track_" + shot.id, im1)


def visualize_lms_shot(shot, im, title="reproj", show=True):
    if disable_debug is False:
        pose = shot.get_pose()
        lms = shot.get_valid_landmarks()
        points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(shot)
        points3D = np.zeros((len(lms), 3), dtype=np.float)
        for idx, lm in enumerate(lms):
            points3D[idx, :] = lm.get_global_pos()
        # camera = shot.camera()
        reproject_landmarks(points3D, points2D,
                            pose.get_world_to_cam(),
                            im,
                            shot.camera, title="reproj",
                            obs_normalized=True, do_show=shot)
