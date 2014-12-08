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
import networkx as nx
from networkx.algorithms import bipartite

from opensfm import context
from opensfm import transformations as tf
from opensfm import dataset
from opensfm import features
from opensfm import multiview
from opensfm import geo


def bundle(tracks_file, reconstruction, config):
    '''Extracts features of image and save them
    '''
    f = tempfile.NamedTemporaryFile(delete=False)
    f.close()
    source = f.name
    f = tempfile.NamedTemporaryFile(delete=False)
    f.close()
    dest = f.name

    with open(source, 'w') as fout:
        fout.write(json.dumps(reconstruction))

    call([context.BUNDLE,
        '--tracks', tracks_file,
        '--input', source,
        '--output', dest,
        '--loss_function', config.get('loss_function', 'TruncatedLoss'),
        '--loss_function_threshold', str(config.get('loss_function_threshold', 3.0)),
        '--exif_focal_sd', str(config.get('exif_focal_sd', 999)),
        ])

    with open(dest) as fin:
        result = json.load(fin)

    os.remove(source)
    os.remove(dest)
    return result



def pairwise_reconstructability(common_tracks, homography_inliers):
    outliers = common_tracks - homography_inliers
    outlier_ratio = float(outliers) / common_tracks
    if outlier_ratio > 0.3:
        return common_tracks
    else:
        return 0

def compute_image_pairs(graph, image_graph):
    '''All matched image pairs sorted by reconstructability.
    '''
    pairs = []
    score = []
    for im1, im2, d in image_graph.edges(data=True):
        tracks, p1, p2 = dataset.common_tracks(graph, im1, im2)
        if len(tracks) >= 100:
            H, inliers = cv2.findHomography(p1, p2, cv2.RANSAC, 8)
            r = pairwise_reconstructability(len(tracks), inliers.sum())
            if r > 0:
                pairs.append((im1,im2))
                score.append(r)
    order = np.argsort(-np.array(score))
    return [pairs[o] for o in order]


def add_gps_position(data, reconstruction, image):
    exif = data.exif_data(image)
    reflla = data.reference_lla()
    if 'gps' in exif:
        lat = exif['gps']['latitude']
        lon = exif['gps']['longitude']
        alt = 2.0 #exif['gps'].get('altitude', 0)
        x, y, z = geo.topocentric_from_lla(lat, lon, alt, *reflla)
        reconstruction['shots'][image]['gps_position'] = [x, y, z]
        reconstruction['shots'][image]['gps_dop'] = exif['gps'].get('dop', 15.0)
    else:
        reconstruction['shots'][image]['gps_position'] = [0.0, 0.0, 0.0]
        reconstruction['shots'][image]['gps_dop'] = 999999.0

    reconstruction['shots'][image]['exif_orientation'] = exif.get('orientation', 1)


def bootstrap_reconstruction(data, graph, im1, im2):
    '''Starts a reconstruction using two shots.
    '''
    print 'Initial reconstruction with', im1, 'and', im2
    d1 = data.exif_data(im1)
    d2 = data.exif_data(im2)
    cameras = data.camera_model_data()

    tracks, p1, p2 = dataset.common_tracks(graph, im1, im2)
    print 'Number of common tracks', len(tracks)

    R, t, inliers, Xs = multiview.two_view_reconstruction(p1, p2, d1, d2, data.config)
    if inliers:
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
                    "rotation": list(cv2.Rodrigues(R)[0].flat),
                    "translation": list(t),
                },
            },

            "points" : {
            },
        }
        add_gps_position(data, reconstruction, im1)
        add_gps_position(data, reconstruction, im2)
        triangulate_shot_features(
                    graph, reconstruction, im1,
                    data.config.get('triangulation_threshold', 3.0),
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

    x_reproject = camera['focal'] * distortion * xp + camera['width'] / 2
    y_reproject = camera['focal'] * distortion * yp + camera['height'] / 2

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
    exif = data.exif_data(shot_id)
    camera_model = exif['camera']
    K = K_from_camera(reconstruction['cameras'][camera_model])
    dist = np.array([0,0,0,0.])

    # Prior on focal length
    R, t, inliers = cv2.solvePnPRansac(X.astype(np.float32), x.astype(np.float32), K, dist,
        reprojectionError=data.config.get('resection_threshold', 8.0))

    if inliers is None:
        print 'Resection no inliers'
        return False
    print 'Resection inliers:', len(inliers), '/', len(x)
    if len(inliers) >= data.config.get('resection_min_inliers', 15):
        reconstruction['shots'][shot_id] = {
            "camera": camera_model,
            "rotation": list(R.flat),
            "translation": list(t.flat),
        }
        add_gps_position(data, reconstruction, shot_id)
        return True
    else:
        return False

def K_from_camera(camera):
    f = float(camera['focal'])
    w = camera['width']
    h = camera['height']
    return np.array([[f, 0., w / 2],
                     [0., f, h / 2],
                     [0., 0., 1.]])

def Rt_from_shot(shot):
    Rt = np.empty((3, 4))
    Rt[:,:3] = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    Rt[:, 3] = shot['translation']
    return Rt

def projection_matrix(camera, shot):
    K = K_from_camera(camera)
    Rt = Rt_from_shot(shot)
    return np.dot(K, Rt)

def angle_between_rays(KR11, x1, KR12, x2):
    v1 = KR11.dot([x1[0], x1[1], 1])
    v2 = KR12.dot([x2[0], x2[1], 1])
    return multiview.vector_angle(v1, v2)


def triangulate_track(track, graph, reconstruction, P_by_id, KR1_by_id, Kinv_by_id, reproj_threshold, min_ray_angle=2.0):
    ''' Triangulate a track
    '''
    Ps, Ps_initial, KR1_initial, Kinv_initial = [], [], [], []
    xs, xs_initial = [], []

    for shot in graph[track]:
        if shot in reconstruction['shots']:
            if shot not in P_by_id:
                s = reconstruction['shots'][shot]
                c = reconstruction['cameras'][s['camera']]
                P = projection_matrix(c, s)
                P_by_id[shot] = P
                KR1_by_id[shot] = np.linalg.inv(P[:,:3])
                Kinv_by_id[shot] = np.linalg.inv(K_from_camera(c))
            Ps_initial.append(P_by_id[shot])
            xs_initial.append(graph[track][shot]['feature'])
            KR1_initial.append(KR1_by_id[shot])
            Kinv_initial.append(Kinv_by_id[shot])
    valid_set = []
    if len(Ps_initial) >= 2:
        max_angle = 0
        for i, j in combinations(range(len(Ps_initial)), 2):
            angle = angle_between_rays(
                    KR1_initial[i], xs_initial[i], KR1_initial[j], xs_initial[j])
            if 1:
                if i not in valid_set:
                    valid_set.append(i)
                if j not in valid_set:
                    valid_set.append(j)
            max_angle = max(angle, max_angle)
        if max_angle > np.radians(min_ray_angle):
            for k in valid_set:
                Ps.append(np.dot(Kinv_initial[k], Ps_initial[k] ))
                xx = np.dot(Kinv_initial[k][:2,:], multiview.homogeneous(np.array(xs_initial[k])))
                xs.append(xx[0:2])
            X = multiview.triangulate(Ps, xs)
            error = 0
            Xh = multiview.homogeneous(X)
            for P, x in zip(Ps, xs):
                xx, yy, zz = P.dot(Xh)
                if zz <= 0:
                    error = 999999999.0
                reprojected_x = np.array([xx / zz, yy / zz])
                error = max(error, (reprojected_x - x).max())
            if error < reproj_threshold:
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


def retriangulate(track_file, graph, reconstruction, image_graph, config):
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
                        triangulate_track(track, graph, reconstruction, P_by_id, KR1_by_id, Kinv_by_id, reproj_threshold=8.0)
                        points_added += 1
                        tracks_added.append(track)

    # bundle adjustment
    reconstruction = bundle(track_file, reconstruction, config)

    # filter points with large reprojection errors
    track_to_delete = []
    for track in tracks_added:
        error = reprojection_error_track(track, graph, reconstruction)
        if error > 3.:
            track_to_delete.append(track)
    print 'Add {0} points after retriangulation.'.format(len(reconstruction['points']) - points_before)
    for t in track_to_delete:
        if t in reconstruction['points']:
            del reconstruction['points'][t]

    # bundle adjustment
    reconstruction = bundle(track_file, reconstruction, config)


def optical_center(shot):
    R = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    t = shot['translation']
    return -R.T.dot(t)


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


def align_reconstruction_naive(reconstruction):
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

    apply_similarity(reconstruction, s, A, b)

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
    print 'ERROR unknown orientation', orientation


def align_reconstruction(reconstruction):
    X, Xp = [], []
    onplane, verticals = [], []
    for shot in reconstruction['shots'].values():
        X.append(optical_center(shot))
        Xp.append(shot['gps_position'])
        R = cv2.Rodrigues(np.array(shot['rotation']))[0]
        x, y, z = get_horitzontal_and_vertical_directions(R, shot['exif_orientation'])
        onplane.append(x)
        onplane.append(z)
        verticals.append(-y)
    X = np.array(X)
    Xp = np.array(Xp)

    # Estimate ground plane.
    p = multiview.fit_plane(X - X.mean(axis=0), onplane, verticals)
    Rplane = multiview.plane_horizontalling_rotation(p)
    X = Rplane.dot(X.T).T

    # Estimate 2d similarity to align to GPS
    if len(X) < 2 or Xp.std(axis=0).max() < 0.01:  # All GPS points are the same.
        s = 1
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

    apply_similarity(reconstruction, s, A, b)


def merge_two_reconstructions(r1, r2, threshold=1):
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
            align_reconstruction(r)
            return [r]
        else:
            return [r1, r2]
    else:
        return [r1, r2]


def merge_reconstructions(reconstructions):
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
            r = merge_two_reconstructions(reconstructions[i], reconstructions[j])
            if len(r) == 1:
                remaining_reconstruction = list(set(remaining_reconstruction) - set([i, j]))
                for k in remaining_reconstruction:
                    rr = merge_two_reconstructions(r[0], reconstructions[k])
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
    to_paint = defaultdict(list)
    to_paint_track = defaultdict(list)
    for track in reconstruction['points']:
        for shot in graph[track]:
            to_paint[shot].append(graph[track][shot]['feature'])
            to_paint_track[shot].append(track)

    track_colors = {track: np.zeros(3) for track in reconstruction['points']}
    track_sum = {track: 0 for track in reconstruction['points']}

    for shot in to_paint:
        features = np.array(to_paint[shot]).astype(int)
        tracks = to_paint_track[shot]
        im = data.image_as_array(shot)
        colors = im[features[:,1], features[:,0]]
        for track, color in zip(tracks, colors):
            track_colors[track] += color
            track_sum[track] += 1

    for track in reconstruction['points']:
        c = track_colors[track] / track_sum[track]
        reconstruction['points'][track]['color'] = list(c)

def paint_reconstruction_constant(data, graph, reconstruction):
    for track in reconstruction['points']:
        reconstruction['points'][track]['color'] = [200, 180, 255]


def grow_reconstruction(data, graph, reconstruction, images, image_graph):
    bundle_interval = data.config.get('bundle_interval', 1)
    retriangulation = data.config.get('retriangulation', False)
    retriangulation_ratio = data.config.get('retriangulation_ratio', 1.25)

    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)
    # align_reconstruction(reconstruction)

    prev_num_points = len(reconstruction['points'])

    num_shots_reconstructed = len(reconstruction['shots'])

    while True:
        if data.config.get('save_partial_reconstructions', False):
            paint_reconstruction_constant(data, graph, reconstruction)
            fname = data.reconstruction_file() + datetime.datetime.now().isoformat().replace(':', '_')
            with open(fname, 'w') as fout:
                fout.write(json.dumps(reconstruction))


        common_tracks = reconstructed_points_for_images(graph, reconstruction, images)
        if not common_tracks:
            break

        for image, num_tracks in common_tracks:

            if resect(data, graph, reconstruction, image):
                print '-------------------------------------------------------'
                print 'Adding {0} to the reconstruction'.format(image)
                images.remove(image)

                if len(reconstruction['shots']) % bundle_interval == 0:
                    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)

                triangulate_shot_features(
                                graph, reconstruction, image,
                                data.config.get('triangulation_threshold', 3.0),
                                data.config.get('triangulation_min_ray_angle', 2.0))

                if len(reconstruction['shots']) % bundle_interval == 0:
                    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)

                # align_reconstruction(reconstruction)

                num_points = len(reconstruction['points'])
                if retriangulation and num_points > prev_num_points * retriangulation_ratio:
                    print 'Re-triangulating'
                    retriangulate(data.tracks_file(), graph, reconstruction, image_graph, data.config)
                    prev_num_points = len(reconstruction['points'])
                    print '  Reprojection Error:', reprojection_error(graph, reconstruction)

                if data.config.get('bundle_outlier_threshold',3.5) > 0:
                    track_outlier = []
                    for track in reconstruction['points']:
                        error = reprojection_error_track(track, graph, reconstruction)
                        if error > data.config.get('bundle_outlier_threshold',3.5):
                            track_outlier.append(track)
                    for track in track_outlier:
                        del reconstruction['points'][track]
                    print 'Remove {0} outliers'.format(len(track_outlier))

                break

        else:
            print 'Some images can not be added'
            break


    align_reconstruction(reconstruction)

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
        exif = data.exif_data(image)
        if exif['camera'] not in fisheye:
            res.append(image)
    return res


def incremental_reconstruction(data):
    data.invent_reference_lla()
    graph = data.tracks_graph()
    tracks, images = bipartite.sets(graph)
    remaining_images = set(nonfisheye_cameras(data, images))
    remaining_images = images
    print 'images', len(images)
    print 'nonfisheye images', len(remaining_images)
    image_graph = bipartite.weighted_projected_graph(graph, images)
    reconstructions = []
    pairs = compute_image_pairs(graph, image_graph)
    for im1, im2 in pairs:
        if im1 in remaining_images and im2 in remaining_images:
            reconstruction = bootstrap_reconstruction(data, graph, im1, im2)
            if reconstruction:
                remaining_images.remove(im1)
                remaining_images.remove(im2)
                reconstruction = grow_reconstruction(data, graph, reconstruction, remaining_images, image_graph)
                reconstructions.append(reconstruction)
                reconstructions = sorted(reconstructions, key=lambda x: -len(x['shots']))
                with open(data.reconstruction_file(), 'w') as fout:
                    fout.write(json.dumps(reconstructions))

    for k, r in enumerate(reconstructions):
        print 'Reconstruction', k, ':', len(r['shots']), 'images', ',', len(r['points']),'points'

    print len(reconstructions), 'partial reconstructions in total.'
