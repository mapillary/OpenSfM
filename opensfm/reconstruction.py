# -*- coding: utf-8 -*-

from collections import defaultdict
from itertools import combinations
from subprocess import call, Popen, PIPE
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
    source = "/tmp/bundle_source.json"
    dest = "/tmp/bundle_dest.json"

    # print 'Focal before bundle', reconstruction['cameras']['main_camera']['focal']
    with open(source, 'w') as fout:
        fout.write(json.dumps(reconstruction, indent=4))

    call([context.BUNDLE,
        '--exif_focal_sd', str(config.get('exif_focal_sd', 999)),
        '--tracks', tracks_file,
        '--input', source,
        '--output', dest])

    with open(dest) as fin:
        result = json.load(fin)
        # print 'Focal after bundle', result['cameras']['main_camera']['focal']
        return result


def pairwise_reconstructability(common_tracks, homography_inliers):
    outliers = common_tracks - homography_inliers
    outlier_ratio = float(outliers) / common_tracks
    if outlier_ratio > 0.3:
        return common_tracks
    else:
        return 0

def compute_image_pairs(graph):
    '''All matched image pairs sorted by reconstructability.
    '''
    track_nodes, image_nodes = bipartite.sets(graph)
    image_graph = bipartite.weighted_projected_graph(graph, image_nodes)
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
    d = data.exif_data(image)
    reflla = data.reference_lla()
    if 'gps' in d:
        lat = d['gps']['latitude']
        lon = d['gps']['longitude']
        alt = 2.0 #d['gps'].get('altitude', 0)
        x, y, z = geo.topocentric_from_lla(lat, lon, alt, *reflla)
        reconstruction['shots'][image]['gps_position'] = [x, y, z]
        reconstruction['shots'][image]['gps_dop'] = d['gps'].get('dop', 15.0)
    else:
        reconstruction['shots'][image]['gps_position'] = [0.0, 0.0, 0.0]
        reconstruction['shots'][image]['gps_dop'] = 999999.0


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
        triangulate_shot_features(graph, reconstruction, im1)
        print 'Number of reconstructed 3D points :{}'.format(len(reconstruction['points']))
        if len(reconstruction['points']) > 50:
            print 'Found initialize good pair', im1 , 'and', im2
            return reconstruction

    print 'Pair', im1, ':', im2, 'fails'
    return None



def reconstructed_points_for_images(graph, reconstruction, shots=None):
    tracks, images = bipartite.sets(graph)
    if shots == None:
        shots = images
    res = []
    for image in images:
        if image not in reconstruction['shots'] and image in shots:
            common_tracks = 0
            for track in graph[image]:
                if track in reconstruction['points']:
                    common_tracks += 1
            res.append((image, common_tracks))
    return sorted(res, key=lambda x: -x[1])


def rotate(angleaxis, point):
    R = cv2.Rodrigues(np.array(angleaxis, dtype=float))[0]
    return R.dot(np.array(point))


def single_reprojection_error(camera, shot, point, observation):
    p = rotate(shot['rotation'], point['coordinates'])
    p += shot['translation']
    xp = p[0] / p[2]
    yp = p[1] / p[2]

    l1 = camera.get('k1', 0.0)
    l2 = camera.get('k2', 0.0)
    r2 = xp * xp + yp * yp;
    distortion = 1.0 + r2  * (l1 + l2  * r2)

    predicted_x = camera['focal'] * distortion * xp + camera['width'] / 2
    predicted_y = camera['focal'] * distortion * yp + camera['height'] / 2

    rx = predicted_x - observation[0]
    ry = predicted_y - observation[1]
    return np.sqrt(rx * rx + ry * ry)


def reprojection_error(graph, reconstruction):
    tracks, shots = bipartite.sets(graph)
    errors = []
    for shot_id in shots:
        if shot_id in reconstruction['shots']:
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


def resect(data, graph, reconstruction, shot_id, min_inliers=20):
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
    if K[0,0]>0:
        # Prior on focal length
        R, t, inliers = cv2.solvePnPRansac(X.astype(np.float32), x.astype(np.float32), K, dist)
    else:
        pass
        # # Estimate focal length and pose
        # p3d = X.astype(np.float32)
        # p2d = x.astype(np.float32)
        # p2d -= np.array(K[[0,1],2])
        # R, t, f = multiview.p3pf(p2d, p3d)
        # K[0,0], K[1,1] = f, f
        # R, t, inliers = cv2.solvePnPRansac(X.astype(np.float32), x.astype(np.float32), K, dist)
    if inliers is None:
        print 'Resection no inliers'
        return False
    print 'Resection inliers:', len(inliers), '/', len(x)
    if len(inliers) >= min_inliers:
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

def angle_between_rays(P1, x1, P2, x2):
    v1 = multiview.pixel_direction(P1[:,:3], x1)
    v2 = multiview.pixel_direction(P2[:,:3], x2)
    return multiview.vector_angle(v1, v2)


def triangulate_track(track, graph, reconstruction, reproj_threshold=3):
    ''' Triangulate a track
    '''
    P_by_id = {}
    Ps = []
    xs = []
    for shot in graph[track]:
        if shot in reconstruction['shots']:
            if shot not in P_by_id:
                s = reconstruction['shots'][shot]
                c = reconstruction['cameras'][s['camera']]
                P_by_id[shot] = projection_matrix(c, s)
            Ps.append(P_by_id[shot])
            xs.append(graph[track][shot]['feature'])
    if len(Ps) >= 2:
        max_angle = 0
        for i, j in combinations(range(len(Ps)), 2):
            angle = angle_between_rays(Ps[i], xs[i], Ps[j], xs[j])
            max_angle = max(angle, max_angle)

        if max_angle > np.radians(2.5):
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

def triangulate_shot_features(graph, reconstruction, shot_id, reproj_threshold=3):
    '''Reconstruct as many tracks seen in shot_id as possible.
    '''
    for track in graph[shot_id]:
        if track not in reconstruction['points']:
            triangulate_track(track, graph, reconstruction,reproj_threshold=reproj_threshold)


def retriangulate(track_file, graph, reconstruction, config):
    '''Re-triangulate 3D points
    '''
    track_nodes, image_nodes = bipartite.sets(graph)
    for track in track_nodes:
        # if track not in reconstruction['points']:
        triangulate_track(track, graph, reconstruction, reproj_threshold=5)

    # bundle adjustment
    reconstruction = bundle(track_file, reconstruction, config)

    # filter points with large reprojection errors
    track_to_delete = []
    for track in reconstruction['points']:
        error = reprojection_error_track(track, graph, reconstruction)
        if error > 3.:
            track_to_delete.append(track)
    print 'Removing {0} points after retriangulation.'.format(len(track_to_delete))
    for t in track_to_delete:
        del reconstruction['points'][t]

def optical_center(shot):
    R = cv2.Rodrigues(np.array(shot['rotation'], dtype=float))[0]
    t = shot['translation']
    return -R.T.dot(t)


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

def align_reconstruction(reconstruction):
    # Compute similarity Xp = s A X + b
    X, Xp = [], []
    vx, vy = [], []
    for shot in reconstruction['shots'].values():
        X.append(optical_center(shot))
        Xp.append(shot['gps_position'])
        R = cv2.Rodrigues(np.array(shot['rotation']))[0]
        # TODO(pau): make this dependent of EXIF orientation tag.
        vx.append(R.T[:,0])   # Direction of cameras' X axis.
        vy.append(-R.T[:,1])  # Direction of cameras' Y axis.
    X = np.array(X)
    Xp = np.array(Xp)

    # Estimate ground plane.
    p = multiview.fit_plane(X, vx, vy)
    Rplane = multiview.plane_horizontalling_rotation(p)

    # Estimate 2d similarity to align to GPS
    X = Rplane.dot(X.T).T
    if Xp.std(axis=0).max() < 0.01: # All GPS points are the same
        Xp[0,0] += 1.0 # Shift one point arbitrarly to avoid degeneracy
    T = tf.affine_matrix_from_points(X.T[:2], Xp.T[:2], shear=False) # 2D transform

    s = np.linalg.det(T[:2,:2])**(1./2)
    A = np.eye(3)
    if s < 1e-10:
        s = 1
    else:
        A[:2,:2] = T[:2,:2] / s
    A = A.dot(Rplane)
    b = np.array([T[0,2],
                  T[1,2],
                  Xp[:,2].mean() - s * X[:,2].mean()])  # vertical alignment

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


def grow_reconstruction(data, graph, reconstruction, images):
    bundle_interval = data.config.get('bundle_interval', 1)
    print 'Aligning'
    align_reconstruction(reconstruction)

    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)
    retriangulation = data.config.get('retriangulation', False)
    retriangulation_ratio = data.config.get('retriangulation_ratio', 1.25)

    prev_num_points = len(reconstruction['points'])

    while True:
        if False:
            paint_reconstruction_constant(data, graph, reconstruction)
            fname = data.reconstruction_file().replace('json', '%04d.json' % len(reconstruction['shots']))
            with open(fname, 'w') as fout:
                fout.write(json.dumps(reconstruction, indent=4))

        common_tracks = reconstructed_points_for_images(graph, reconstruction, images)
        if not common_tracks:
            break
        for image, num_tracks in common_tracks:
            if resect(data, graph, reconstruction, image, min_inliers=15):
                print '-------------------------------------------------------'
                print 'Adding {0} to the reconstruction'.format(image)
                images.remove(image)

                if len(reconstruction['shots']) % bundle_interval == 0:
                    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)

                triangulate_shot_features(graph, reconstruction, image, reproj_threshold=5.0)

                if len(reconstruction['shots']) % bundle_interval == 0:
                    reconstruction = bundle(data.tracks_file(), reconstruction, data.config)

                print 'Reprojection Error:', reprojection_error(graph, reconstruction)

                num_points = len(reconstruction['points'])
                if retriangulation and num_points > prev_num_points * retriangulation_ratio:
                    print 'Re-triangulating'
                    retriangulate(data.tracks_file(), graph, reconstruction, data.config)
                    prev_num_points = len(reconstruction['points'])
                    print '  Reprojection Error:', reprojection_error(graph, reconstruction)

                break
        else:
            print 'Some images can not be added'
            break

    print 'Aligning'
    align_reconstruction(reconstruction)
    print 'Painting the reconstruction from {0} cameras'.format(len(reconstruction['shots']))
    paint_reconstruction(data, graph, reconstruction)
    print 'Done.'
    return reconstruction

def incremental_reconstruction(data):
    data.invent_reference_lla()
    graph = data.tracks_graph()
    tracks, images = bipartite.sets(graph)
    remaining_images = set(images)

    reconstructions = []
    pairs = compute_image_pairs(graph)
    for im1, im2 in pairs:
        if im1 in remaining_images and im2 in remaining_images:
            reconstruction = bootstrap_reconstruction(data, graph, im1, im2)
            if reconstruction:
                remaining_images.remove(im1)
                remaining_images.remove(im2)
                reconstruction = grow_reconstruction(data, graph, reconstruction, remaining_images)
                reconstructions.append(reconstruction)
                with open(data.reconstruction_file(), 'w') as fout:
                    fout.write(json.dumps(reconstructions, indent=4))

    for k, r in enumerate(reconstructions):
        print 'Reconstruction', k, ':', len(r['shots']), 'images', ',', len(r['points']),'points'

    print len(reconstructions), 'partial reconstructions in total.'
