# -*- coding: utf-8 -*-

from collections import defaultdict
from itertools import combinations
import os
from subprocess import call, Popen, PIPE
import tempfile
import datetime
from itertools import combinations

import numpy as np
import cv2
import json
import time
import math
import networkx as nx
from networkx.algorithms import bipartite

from opensfm import transformations as tf
from opensfm import dataset
from opensfm import features
from opensfm import multiview
from opensfm import geo
from opensfm import csfm


def bundle(graph, reconstruction, config, fix_cameras=False):
    '''Bundle adjust a reconstruction.
    '''

    start = time.time()
    ba = csfm.BundleAdjuster()
    for k, v in reconstruction['cameras'].items():
        ba.add_camera(str(k), v['focal'], v['k1'], v['k2'],
            v['focal_prior'], fix_cameras)

    for k, v in reconstruction['shots'].items():
        r = v['rotation']
        t = v['translation']
        g = v['gps_position']
        ba.add_shot(
            str(k), str(v['camera']),
            r[0], r[1], r[2],
            t[0], t[1], t[2],
            g[0], g[1], g[2],
            v['gps_dop'], False
        )

    for k, v in reconstruction['points'].items():
        x = v['coordinates']
        ba.add_point(str(k), x[0], x[1], x[2], False)

    for shot in reconstruction['shots']:
        for track in graph[shot]:
            if track in reconstruction['points']:
                ba.add_observation(str(shot), str(track), *graph[shot][track]['feature'])

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(config.get('exif_focal_sd', 0.01),
                                        config.get('radial_distorsion_k1_sd', 0.01),
                                        config.get('radial_distorsion_k2_sd', 0.01))

    setup = time.time()

    ba.set_num_threads(config['processes'])
    ba.run()
    print ba.brief_report()

    run = time.time()

    for k, v in reconstruction['cameras'].items():
        c = ba.get_camera(str(k))
        v['focal'] = c.focal
        v['k1'] = c.k1
        v['k2'] = c.k2

    for k, v in reconstruction['shots'].items():
        s = ba.get_shot(str(k))
        v['rotation'] = [s.rx, s.ry, s.rz]
        v['translation'] = [s.tx, s.ty, s.tz]

    for k, v in reconstruction['points'].items():
        p = ba.get_point(str(k))
        v['coordinates'] = [p.x, p.y, p.z]
        v['reprojection_error'] = p.reprojection_error

    teardown = time.time()

    print 'setup/run/teardown {0}/{1}/{2}'.format(setup - start, run - setup, teardown - run)


def bundle_single_view(graph, reconstruction, shot_id, config):
    '''Bundle adjust a single camera
    '''
    ba = csfm.BundleAdjuster()
    shot = reconstruction['shots'][shot_id]
    camera_id = shot['camera']
    camera = reconstruction['cameras'][camera_id]

    ba.add_camera(str(camera_id), camera['focal'], camera['k1'], camera['k2'],
            camera['focal_prior'], True)

    r = shot['rotation']
    t = shot['translation']
    g = shot['gps_position']
    ba.add_shot(
        str(shot_id), str(camera_id),
        r[0], r[1], r[2],
        t[0], t[1], t[2],
        g[0], g[1], g[2],
        shot['gps_dop'], False
    )

    for track_id in graph[shot_id]:
        if track_id in reconstruction['points']:
            track = reconstruction['points'][track_id]
            x = track['coordinates']
            ba.add_point(str(track_id), x[0], x[1], x[2], True)
            ba.add_observation(str(shot_id), str(track_id), *graph[shot_id][track_id]['feature'])

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(config.get('exif_focal_sd', 0.01),
                                        config.get('radial_distorsion_k1_sd', 0.01),
                                        config.get('radial_distorsion_k2_sd', 0.01))

    ba.set_num_threads(config['processes'])
    ba.run()

    s = ba.get_shot(str(shot_id))
    shot['rotation'] = [s.rx, s.ry, s.rz]
    shot['translation'] = [s.tx, s.ty, s.tz]


def pairwise_reconstructability(common_tracks, homography_inliers):
    outliers = common_tracks - homography_inliers
    outlier_ratio = float(outliers) / common_tracks
    if outlier_ratio > 0.3:
        return common_tracks
    else:
        return 0

def compute_image_pairs(graph, image_graph, config):
    '''All matched image pairs sorted by reconstructability.
    '''
    pairs = []
    score = []
    for im1, im2, d in image_graph.edges(data=True):
        tracks, p1, p2 = dataset.common_tracks(graph, im1, im2)
        if len(tracks) >= 50:
            H, inliers = cv2.findHomography(p1, p2, cv2.RANSAC, config.get('homography_threshold', 0.004))
            r = pairwise_reconstructability(len(tracks), inliers.sum())
            if r > 0:
                pairs.append((im1,im2))
                score.append(r)
    order = np.argsort(-np.array(score))
    return [pairs[o] for o in order]


def add_gps_position(data, shot, image):
    exif = data.load_exif(image)
    reflla = data.load_reference_lla()
    if 'gps' in exif and 'latitude' in exif['gps'] and 'longitude' in exif['gps']:
        lat = exif['gps']['latitude']
        lon = exif['gps']['longitude']
        if data.config.get('use_altitude_tag', False):
            alt = exif['gps'].get('altitude', 2.0)
        else:
            alt = 2.0 # Arbitrary constant value that will be used to align the reconstruction
        x, y, z = geo.topocentric_from_lla(lat, lon, alt,
            reflla['latitude'], reflla['longitude'], reflla['altitude'])
        shot['gps_position'] = [x, y, z]
        shot['gps_dop'] = exif['gps'].get('dop', 15.0)
    else:
        shot['gps_position'] = [0.0, 0.0, 0.0]
        shot['gps_dop'] = 999999.0

    shot['orientation'] = exif.get('orientation', 1)

    if 'accelerometer' in exif:
        shot['accelerometer'] = exif['accelerometer']

    if 'compass' in exif:
        shot['compass'] = exif['compass']

    if 'capture_time' in exif:
        shot['capture_time'] = exif['capture_time']

    if 'skey' in exif:
        shot['skey'] = exif['skey']


def bootstrap_reconstruction(data, graph, im1, im2):
    '''Starts a reconstruction using two shots.
    '''
    print 'Initial reconstruction with', im1, 'and', im2
    d1 = data.load_exif(im1)
    d2 = data.load_exif(im2)
    cameras = data.load_camera_models()

    tracks, p1, p2 = dataset.common_tracks(graph, im1, im2)
    print 'Number of common tracks', len(tracks)

    f1 = d1['focal_prior']
    f2 = d2['focal_prior']
    threshold = data.config.get('five_point_algo_threshold', 0.006)
    ret = csfm.two_view_reconstruction(p1, p2, f1, f2, threshold)
    if ret is not None:
        R, t, cov, inliers = ret
    else:
        return None
    if len(inliers):
        print 'Number of inliers', len(inliers)
        reconstruction = {
            "cameras": cameras,

            "shots" : {
                im1: {
                    "camera": str(d1['camera']),
                    "rotation": [0.0, 0.0, 0.0],
                    "translation": [0.0, 0.0, 0.0],
                },
                im2: {
                    "camera": str(d2['camera']),
                    "rotation": list(R),
                    "translation": list(t),
                },
            },

            "points" : {
            },
        }
        add_gps_position(data, reconstruction['shots'][im1], im1)
        add_gps_position(data, reconstruction['shots'][im2], im2)
        triangulate_shot_features(
                    graph, reconstruction, im1,
                    data.config.get('triangulation_threshold', 0.004),
                    data.config.get('triangulation_min_ray_angle', 2.0))
        print 'Number of reconstructed 3D points :{}'.format(len(reconstruction['points']))
        if len(reconstruction['points']) > data.config.get('five_point_algo_min_inliers', 50):
            print 'Found initialize good pair', im1 , 'and', im2
            return reconstruction

    print 'Pair', im1, ':', im2, 'fails'
    return None



def reconstructed_points_for_images(graph, reconstruction, images):
    res = []
    for image in images:
        if image not in reconstruction['shots']:
            common_tracks = 0
            for track in graph[image]:
                if track in reconstruction['points']:
                    common_tracks += 1
            res.append((image, common_tracks))
    return sorted(res, key=lambda x: -x[1])


def rotate(angleaxis, point):
    R = cv2.Rodrigues(np.array(angleaxis, dtype=float))[0]
    return R.dot(np.array(point))

def camera_coordinates(camera, shot, point):
    p = rotate(shot['rotation'], point)
    p += shot['translation']
    return p

def back_project(camera, shot, pixel, depth):
    K = multiview.K_from_camera(camera)
    R = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    t = shot['translation']
    A = K.dot(R)
    b = depth * np.array([pixel[0], pixel[1], 1]) - K.dot(t)
    return np.linalg.solve(A, b)


def reproject(camera, shot, point):
    ''' Reproject 3D point onto image plane given a camera
    '''
    p = rotate(shot['rotation'], point['coordinates'])
    p += shot['translation']
    xp = p[0] / p[2]
    yp = p[1] / p[2]

    l1 = camera.get('k1', 0.0)
    l2 = camera.get('k2', 0.0)
    r2 = xp * xp + yp * yp
    distortion = 1.0 + r2  * (l1 + l2  * r2)

    x_reproject = camera['focal'] * distortion * xp
    y_reproject = camera['focal'] * distortion * yp

    return np.array([x_reproject, y_reproject])

def single_reprojection_error(camera, shot, point, observation):
    ''' Reprojection error of a single points
    '''

    point_reprojected = reproject(camera, shot, point)

    err = point_reprojected - observation

    return np.linalg.norm(err)


def reprojection_error(graph, reconstruction):
    errors = []
    for shot_id in reconstruction['shots']:
        for track_id in graph[shot_id]:
            if track_id in reconstruction['points']:
                observation = graph[shot_id][track_id]['feature']
                shot = reconstruction['shots'][shot_id]
                camera = reconstruction['cameras'][shot['camera']]
                point = reconstruction['points'][track_id]
                errors.append(single_reprojection_error(camera, shot, point, observation))
    return np.median(errors)


def reprojection_error_track(track, graph, reconstruction):
    errors = []
    error = 999999999.
    if track in reconstruction['points']:
        for shot_id in graph[track]:
            observation = graph[shot_id][track]['feature']
            if shot_id in reconstruction['shots']:
                shot = reconstruction['shots'][shot_id]
                camera = reconstruction['cameras'][shot['camera']]
                point = reconstruction['points'][track]
                errors.append(single_reprojection_error(camera, shot, point, observation))
        if errors:
            error = np.max(errors)
    else:
        error = 999999999.

    return error


def resect(data, graph, reconstruction, shot_id):
    '''Add a shot to the reconstruction.
    '''
    xs = []
    Xs = []
    for track in graph[shot_id]:
        if track in reconstruction['points']:
            xs.append(graph[shot_id][track]['feature'])
            Xs.append(reconstruction['points'][track]['coordinates'])
    x = np.array(xs)
    X = np.array(Xs)
    if len(x) < 5:
        return False
    exif = data.load_exif(shot_id)
    camera_model = exif['camera']
    K = multiview.K_from_camera(reconstruction['cameras'][camera_model])
    dist = np.array([0,0,0,0.])

    # Prior on focal length
    R, t, inliers = cv2.solvePnPRansac(X.astype(np.float32), x.astype(np.float32), K, dist,
        reprojectionError=data.config.get('resection_threshold', 0.004))

    if inliers is None:
        print 'Resection', shot_id, 'no inliers'
        return False
    print 'Resection', shot_id, 'inliers:', len(inliers), '/', len(x)
    if len(inliers) >= data.config.get('resection_min_inliers', 15):
        reconstruction['shots'][shot_id] = {
            "camera": camera_model,
            "rotation": list(R.flat),
            "translation": list(t.flat),
        }
        add_gps_position(data, reconstruction['shots'][shot_id], shot_id)
        bundle_single_view(graph, reconstruction, shot_id, data.config)
        return True
    else:
        return False


def Rt_from_shot(shot):
    Rt = np.empty((3, 4))
    Rt[:,:3] = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    Rt[:, 3] = shot['translation']
    return Rt

def projection_matrix(camera, shot):
    K = multiview.K_from_camera(camera)
    Rt = Rt_from_shot(shot)
    return np.dot(K, Rt)


def triangulate_track(track, graph, reconstruction, P_by_id, UNUSED1, UNUSED2, reproj_threshold, min_ray_angle_degrees=2.0):
    ''' Triangulate a track
    '''
    min_ray_angle = np.radians(min_ray_angle_degrees)
    Ps, KR1s, xs, vs = [], [], [], []

    for shot in graph[track]:
        if shot in reconstruction['shots']:
            if shot not in P_by_id:
                s = reconstruction['shots'][shot]
                c = reconstruction['cameras'][s['camera']]
                P = projection_matrix(c, s)
                P_by_id[shot] = P
            Ps.append(P_by_id[shot])
            x = graph[track][shot]['feature']
            xs.append(np.array(x))

    if len(Ps) >= 2:
        e, X = csfm.triangulate(Ps, xs, reproj_threshold, min_ray_angle)
        if X is not None:
            reconstruction['points'][track] = {
                "coordinates": list(X),
            }


def triangulate_shot_features(graph, reconstruction, shot_id, reproj_threshold, min_ray_angle):
    '''Reconstruct as many tracks seen in shot_id as possible.
    '''
    P_by_id = {}
    KR1_by_id = {}
    Kinv_by_id = {}

    for track in graph[shot_id]:
        if track not in reconstruction['points']:
            triangulate_track(track, graph, reconstruction, P_by_id, KR1_by_id, Kinv_by_id, reproj_threshold, min_ray_angle)


def retriangulate(graph, reconstruction, image_graph, config):
    '''Re-triangulate 3D points
    '''
    P_by_id = {}
    KR1_by_id = {}
    Kinv_by_id = {}
    shots = reconstruction['shots']
    points = reconstruction['points']
    points_added = 0
    tracks_added = []
    points_before = len(points)
    for im1, im2, d in image_graph.edges(data=True):
        if (im1 in shots) and (im2 in shots):
            tracks, p1, p2 = dataset.common_tracks(graph, im1, im2)
            # find already reconstructed tracks
            diff = np.setdiff1d(tracks, points.keys())
            reconstruct_ratio = 1 - len(diff)/float(len(tracks))
            if reconstruct_ratio < 0.3:
                for track in diff:
                    if track not in tracks_added:
                        triangulate_track(track, graph, reconstruction, P_by_id, KR1_by_id, Kinv_by_id, reproj_threshold=0.006)
                        points_added += 1
                        tracks_added.append(track)

    # bundle adjustment
    bundle(graph, reconstruction, config)

    # filter points with large reprojection errors
    track_to_delete = []
    for track in tracks_added:
        error = reprojection_error_track(track, graph, reconstruction)
        if error > config.get('triangulation_threshold', 0.004):
            track_to_delete.append(track)
    print 'Add {0} points after retriangulation.'.format(len(reconstruction['points']) - points_before)
    for t in track_to_delete:
        if t in reconstruction['points']:
            del reconstruction['points'][t]

    # bundle adjustment
    bundle(graph, reconstruction, config)


def retriangulate_all(graph, reconstruction, image_graph, config):
    '''
    Retrianguate all points
    '''
    triangulation_threshold = config.get('retriangulation_threshold', 0.004)
    min_ray_angle = config.get('triangulation_min_ray_angle', 2.0)
    P_by_id = {}
    KR1_by_id = {}
    Kinv_by_id = {}
    tracks, images = tracks_and_images(graph)
    for track in tracks:
        triangulate_track(track, graph, reconstruction, P_by_id, KR1_by_id, Kinv_by_id, triangulation_threshold, min_ray_angle)
    # bundle adjustment
    bundle(graph, reconstruction, config)


def remove_outliers(graph, reconstruction, config):
    threshold = config.get('bundle_outlier_threshold', 0.008)
    if threshold > 0:
        outliers = []
        for track in reconstruction['points']:
            error = reconstruction['points'][track]['reprojection_error']
            if error > threshold:
                outliers.append(track)
        for track in outliers:
            del reconstruction['points'][track]
        print 'Remove {0} outliers'.format(len(outliers))


def optical_center(shot):
    R = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    t = shot['translation']
    return -R.T.dot(t)


def viewing_direction(shot):
    """ Calculates the viewing direction for a shot.

    :param shot: The shot.
    :return: The viewing direction.
    """
    R = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    t = np.array([0, 0, 1])
    return R.T.dot(t)


def apply_similarity(reconstruction, s, A, b):
    """Apply a similarity (y = s A x + t) to a reconstruction.

    :param reconstruction: The reconstruction to transform.
    :param s: The scale (a scalar)
    :param A: The rotation matrix (3x3)
    :param b: The translation vector (3)
    """
    # Align points.
    for point in reconstruction['points'].values():
        Xp = s * A.dot(point['coordinates']) + b
        point['coordinates'] = list(Xp)

    # Align cameras.
    for shot in reconstruction['shots'].values():
        R = cv2.Rodrigues(np.array(shot['rotation']))[0]
        t = np.array(shot['translation'])
        Rp = R.dot(A.T)
        tp = -Rp.dot(b) + s * t
        shot['rotation'] = list(cv2.Rodrigues(Rp)[0].flat)
        shot['translation'] = list(tp)


def align_reconstruction_naive_similarity(reconstruction):
    if len(reconstruction['shots']) < 3: return
    # Compute similarity Xp = s A X + b
    X, Xp = [], []
    for shot in reconstruction['shots'].values():
        X.append(optical_center(shot))
        Xp.append(shot['gps_position'])
    X = np.array(X)
    Xp = np.array(Xp)
    T = tf.superimposition_matrix(X.T, Xp.T, scale=True)

    A, b = T[:3,:3], T[:3,3]
    s = np.linalg.det(A)**(1./3)
    A /= s
    return s, A, b


def get_horitzontal_and_vertical_directions(R, orientation):
    '''Get orientation vectors from camera rotation matrix and orientation tag.

    Return a 3D vectors pointing to the positive XYZ directions of the image.
    X points to the right, Y to the bottom, Z to the front.
    '''
    # See http://sylvana.net/jpegcrop/exif_orientation.html
    if orientation == 1:
        return  R[0, :],  R[1, :],  R[2, :]
    if orientation == 2:
        return -R[0, :],  R[1, :], -R[2, :]
    if orientation == 3:
        return -R[0, :], -R[1, :],  R[2, :]
    if orientation == 4:
        return  R[0, :], -R[1, :],  R[2, :]
    if orientation == 5:
        return  R[1, :],  R[0, :], -R[2, :]
    if orientation == 6:
        return  R[1, :], -R[0, :],  R[2, :]
    if orientation == 7:
        return -R[1, :], -R[0, :], -R[2, :]
    if orientation == 8:
        return -R[1, :],  R[0, :],  R[2, :]
    print 'ERROR unknown orientation {0} - Assuming orientation 1 instead.'.format(orientation)
    return  R[0, :],  R[1, :],  R[2, :]


def align_reconstruction(reconstruction, config):
    s, A, b = align_reconstruction_similarity(reconstruction, config)
    apply_similarity(reconstruction, s, A, b)

def align_reconstruction_similarity(reconstruction, config):
    align_method = config.get('align_method', 'orientation_prior')
    if align_method == 'orientation_prior':
        return align_reconstruction_orientation_prior_similarity(reconstruction, config)
    elif align_method == 'naive':
        return align_reconstruction_naive_similarity(reconstruction)


def align_reconstruction_orientation_prior_similarity(reconstruction, config):
    X, Xp = [], []
    orientation_type = config.get('align_orientation_prior', 'horizontal')
    onplane, verticals = [], []
    for shot in reconstruction['shots'].values():
        X.append(optical_center(shot))
        Xp.append(shot['gps_position'])
        R = cv2.Rodrigues(np.array(shot['rotation']))[0]
        x, y, z = get_horitzontal_and_vertical_directions(R, shot.get('orientation'))
        if orientation_type == 'no_roll':
            onplane.append(x)
            verticals.append(-y)
        elif orientation_type == 'horizontal':
            onplane.append(x)
            onplane.append(z)
            verticals.append(-y)
        elif orientation_type == 'vertical':
            onplane.append(x)
            onplane.append(y)
            verticals.append(-z)

    X = np.array(X)
    Xp = np.array(Xp)

    # Estimate ground plane.
    p = multiview.fit_plane(X - X.mean(axis=0), onplane, verticals)
    Rplane = multiview.plane_horizontalling_rotation(p)
    X = Rplane.dot(X.T).T

    # Estimate 2d similarity to align to GPS
    if (len(X) < 2 or
           X.std(axis=0).max() < 1e-8 or     # All points are the same.
           Xp.std(axis=0).max() < 0.01):      # All GPS points are the same.
        s = len(X) / max(1e-8, X.std(axis=0).max())           # Set the arbitrary scale proportional to the number of cameras.
        A = Rplane
        b = Xp.mean(axis=0) - X.mean(axis=0)
    else:
        T = tf.affine_matrix_from_points(X.T[:2], Xp.T[:2], shear=False)
        s = np.linalg.det(T[:2,:2])**(1./2)
        A = np.eye(3)
        A[:2,:2] = T[:2,:2] / s
        A = A.dot(Rplane)
        b = np.array([T[0,2],
                      T[1,2],
                      Xp[:,2].mean() - s * X[:,2].mean()])  # vertical alignment
    return s, A, b


def register_reconstruction_with_gps(reconstruction, reference):
    """
    register reconstrution with gps positions and compass angles
    """
    shots = reconstruction['shots']
    for shot_id, shot in shots.iteritems():
        gps = {}
        topo = optical_center(shot)
        lat, lon, alt = geo.lla_from_topocentric(topo[0], topo[1], topo[2],
                                reference['latitude'], reference['longitude'], reference['altitude'])

        # find direction
        shot['translation'][2] -= 1
        topo2 = optical_center(shot)
        dz = topo2 - topo
        angle = np.rad2deg(np.arctan2(dz[0], dz[1]))
        angle = (angle+360) % 360
        reconstruction['shots'][shot_id]['gps'] = {
                                            'lon': lon,
                                            'lat': lat,
                                            'altitude': alt,
                                            'direction': angle
                                      }


def merge_two_reconstructions(r1, r2, config, threshold=1):
    ''' Merge two reconstructions with common tracks
    '''
    t1, t2 = r1['points'], r2['points']
    common_tracks = list(set(t1) & set(t2))

    # print 'Number of common tracks between two reconstructions: {0}'.format(len(common_tracks))
    if len(common_tracks) > 6:

        # Estimate similarity transform
        p1 = np.array([t1[t]['coordinates'] for t in common_tracks])
        p2 = np.array([t2[t]['coordinates'] for t in common_tracks])

        T, inliers = multiview.fit_similarity_transform(p1, p2, max_iterations=1000, threshold=threshold)

        if len(inliers) >= 10:
            s, A, b = multiview.decompose_similarity_transform(T)
            r1p = r1
            apply_similarity(r1p, s, A, b)
            r = r2
            r['shots'].update(r1p['shots'])
            r['points'].update(r1p['points'])
            align_reconstruction(r, config)
            return [r]
        else:
            return [r1, r2]
    else:
        return [r1, r2]


def merge_reconstructions(reconstructions, config):
    ''' Greedily merge reconstructions with common tracks
    '''
    num_reconstruction = len(reconstructions)
    ids_reconstructions = np.arange(num_reconstruction)
    remaining_reconstruction = ids_reconstructions
    reconstructions_merged = []
    num_merge = 0
    pairs = []

    for (i, j) in combinations(ids_reconstructions, 2):
        if (i in remaining_reconstruction) and (j in remaining_reconstruction):
            r = merge_two_reconstructions(reconstructions[i], reconstructions[j], config)
            if len(r) == 1:
                remaining_reconstruction = list(set(remaining_reconstruction) - set([i, j]))
                for k in remaining_reconstruction:
                    rr = merge_two_reconstructions(r[0], reconstructions[k], config)
                    if len(r) == 2:
                        break
                    else:
                        r = rr
                        remaining_reconstruction = list(set(remaining_reconstruction) - set([k]))
                reconstructions_merged.append(r[0])
                num_merge += 1


    for k in remaining_reconstruction:
        reconstructions_merged.append(reconstructions[k])

    print 'Merged {0} reconstructions.'.format(num_merge)

    return reconstructions_merged


def paint_reconstruction(data, graph, reconstruction):
    for track in reconstruction['points']:
        reconstruction['points'][track]['color'] = graph[track].values()[0]['feature_color']

def paint_reconstruction_constant(data, graph, reconstruction):
    for track in reconstruction['points']:
        reconstruction['points'][track]['color'] = [200, 180, 255]


def grow_reconstruction(data, graph, reconstruction, images, image_graph):
    bundle_interval = data.config.get('bundle_interval', 0)
    bundle_new_points_ratio = data.config.get('bundle_new_points_ratio', 1.2)
    retriangulation = data.config.get('retriangulation', False)
    retriangulation_ratio = data.config.get('retriangulation_ratio', 1.25)

    bundle(graph, reconstruction, data.config)
    align_reconstruction(reconstruction, data.config)

    num_points_last_bundle = len(reconstruction['points'])
    num_shots_last_bundle = len(reconstruction['shots'])
    num_points_last_retriangulation = len(reconstruction['points'])
    num_shots_reconstructed = len(reconstruction['shots'])

    while True:
        if data.config.get('save_partial_reconstructions', False):
            paint_reconstruction(data, graph, reconstruction)
            data.save_reconstruction(reconstruction, 'reconstruction.{}.json'.format(
                datetime.datetime.now().isoformat().replace(':', '_')))

        common_tracks = reconstructed_points_for_images(graph, reconstruction, images)
        if not common_tracks:
            break

        for image, num_tracks in common_tracks:
            if resect(data, graph, reconstruction, image):
                print '-------------------------------------------------------'
                print 'Adding {0} to the reconstruction'.format(image)
                images.remove(image)

                triangulate_shot_features(
                                graph, reconstruction, image,
                                data.config.get('triangulation_threshold', 0.004),
                                data.config.get('triangulation_min_ray_angle', 2.0))

                if (len(reconstruction['points']) >= num_points_last_bundle * bundle_new_points_ratio
                    or len(reconstruction['shots']) >= num_shots_last_bundle + bundle_interval):
                    bundle(graph, reconstruction, data.config)
                    remove_outliers(graph, reconstruction, data.config)
                    align_reconstruction(reconstruction, data.config)
                    num_points_last_bundle = len(reconstruction['points'])
                    num_shots_last_bundle = len(reconstruction['shots'])


                num_points = len(reconstruction['points'])
                if retriangulation and num_points > num_points_last_retriangulation * retriangulation_ratio:
                    print 'Re-triangulating'
                    retriangulate_all(graph, reconstruction, image_graph, data.config)
                    num_points_last_retriangulation = len(reconstruction['points'])
                    print '  Reprojection Error:', reprojection_error(graph, reconstruction)

                break
        else:
            print 'Some images can not be added'
            break


    bundle(graph, reconstruction, data.config)
    align_reconstruction(reconstruction, data.config)

    print 'Reprojection Error:', reprojection_error(graph, reconstruction)
    print 'Painting the reconstruction from {0} cameras'.format(len(reconstruction['shots']))
    paint_reconstruction(data, graph, reconstruction)
    print 'Done.'
    return reconstruction


def nonfisheye_cameras(data, images):
    fisheye = [
        "gopro hero3+ black edition",
        "gopro hero2",
    ]
    res = []
    for image in images:
        exif = data.load_exif(image)
        if exif['camera'] not in fisheye and 1 <= exif['orientation'] <= 8:
            res.append(image)
    return res

def tracks_and_images(graph):
    tracks, images = [], []
    for n in graph.nodes(data=True):
        if n[1]['bipartite'] == 0:
            images.append(n[0])
        else:
            tracks.append(n[0])
    return tracks, images

def incremental_reconstruction(data):
    data.invent_reference_lla()
    graph = data.load_tracks_graph()
    tracks, images = tracks_and_images(graph)
    remaining_images = set(nonfisheye_cameras(data, images))
    print 'images', len(images)
    print 'nonfisheye images', len(remaining_images)
    image_graph = bipartite.weighted_projected_graph(graph, images)
    reconstructions = []
    pairs = compute_image_pairs(graph, image_graph, data.config)
    for im1, im2 in pairs:
        if im1 in remaining_images and im2 in remaining_images:
            reconstruction = bootstrap_reconstruction(data, graph, im1, im2)
            if reconstruction:
                remaining_images.remove(im1)
                remaining_images.remove(im2)
                reconstruction = grow_reconstruction(data, graph, reconstruction, remaining_images, image_graph)
                reconstructions.append(reconstruction)
                reconstructions = sorted(reconstructions, key=lambda x: -len(x['shots']))
                data.save_reconstruction(reconstructions)

    for k, r in enumerate(reconstructions):
        print 'Reconstruction', k, ':', len(r['shots']), 'images', ',', len(r['points']),'points'

    print len(reconstructions), 'partial reconstructions in total.'
