# -*- coding: utf-8 -*-

from itertools import combinations
import datetime

import numpy as np
import cv2
import pyopengv
import time
from networkx.algorithms import bipartite

from opensfm import csfm
from opensfm import geo
from opensfm import matching
from opensfm import multiview
from opensfm import transformations as tf
from opensfm import types


def bundle(graph, reconstruction, config, fix_cameras=False):
    '''Bundle adjust a reconstruction.
    '''

    start = time.time()
    ba = csfm.BundleAdjuster()
    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            ba.add_perspective_camera(
                str(camera.id), camera.focal, camera.k1, camera.k2,
                camera.focal_prior, camera.k1_prior, camera.k2_prior, fix_cameras)

        elif camera.projection_type in ['equirectangular', 'spherical']:
            ba.add_equirectangular_camera(str(camera.id))

    for shot in reconstruction.shots.values():
        r = shot.pose.rotation
        t = shot.pose.translation
        g = shot.metadata.gps_position
        ba.add_shot(
            str(shot.id), str(shot.camera.id),
            r[0], r[1], r[2],
            t[0], t[1], t[2],
            g[0], g[1], g[2],
            shot.metadata.gps_dop, False
        )

    for point in reconstruction.points.values():
        x = point.coordinates
        ba.add_point(str(point.id), x[0], x[1], x[2], False)

    for shot_id in reconstruction.shots:
        if shot_id in graph:
            for track in graph[shot_id]:
                if track in reconstruction.points:
                    ba.add_observation(str(shot_id), str(track), *graph[shot_id][track]['feature'])

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(config.get('exif_focal_sd', 0.01),
                                        config.get('radial_distorsion_k1_sd', 0.01),
                                        config.get('radial_distorsion_k2_sd', 0.01))

    setup = time.time()

    ba.set_num_threads(config['processes'])
    ba.run()

    run = time.time()
    print ba.brief_report()

    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            c = ba.get_perspective_camera(str(camera.id))
            camera.focal = c.focal
            camera.k1 = c.k1
            camera.k2 = c.k2

    for shot in reconstruction.shots.values():
        s = ba.get_shot(str(shot.id))
        shot.pose.rotation = [s.rx, s.ry, s.rz]
        shot.pose.translation = [s.tx, s.ty, s.tz]

    for point in reconstruction.points.values():
        p = ba.get_point(str(point.id))
        point.coordinates = [p.x, p.y, p.z]
        point.reprojection_error = p.reprojection_error

    teardown = time.time()

    print 'setup/run/teardown {0}/{1}/{2}'.format(setup - start, run - setup, teardown - run)


def bundle_single_view(graph, reconstruction, shot_id, config):
    '''Bundle adjust a single camera
    '''
    ba = csfm.BundleAdjuster()
    shot = reconstruction.shots[shot_id]
    camera = shot.camera

    if camera.projection_type == 'perspective':
        ba.add_perspective_camera(
            str(camera.id), camera.focal, camera.k1, camera.k2,
            camera.focal_prior, camera.k1_prior, camera.k2_prior, True)
    elif camera.projection_type in ['equirectangular', 'spherical']:
        ba.add_equirectangular_camera(str(camera.id))

    r = shot.pose.rotation
    t = shot.pose.translation
    g = shot.metadata.gps_position
    ba.add_shot(
        str(shot_id), str(camera.id),
        r[0], r[1], r[2],
        t[0], t[1], t[2],
        g[0], g[1], g[2],
        shot.metadata.gps_dop, False
    )

    for track_id in graph[shot_id]:
        if track_id in reconstruction.points:
            track = reconstruction.points[track_id]
            x = track.coordinates
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
    shot.pose.rotation = [s.rx, s.ry, s.rz]
    shot.pose.translation = [s.tx, s.ty, s.tz]


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
        tracks, p1, p2 = matching.common_tracks(graph, im1, im2)
        if len(tracks) >= 50:
            H, inliers = cv2.findHomography(p1, p2, cv2.RANSAC, config.get('homography_threshold', 0.004))
            r = pairwise_reconstructability(len(tracks), inliers.sum())
            if r > 0:
                pairs.append((im1,im2))
                score.append(r)
    order = np.argsort(-np.array(score))
    return [pairs[o] for o in order]


def get_image_metadata(data, image):
    metadata = types.ShotMetadata()
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
        metadata.gps_position = [x, y, z]
        metadata.gps_dop = exif['gps'].get('dop', 15.0)
    else:
        metadata.gps_position = [0.0, 0.0, 0.0]
        metadata.gps_dop = 999999.0

    metadata.orientation = exif.get('orientation', 1)

    if 'accelerometer' in exif:
        metadata.accelerometer = exif['accelerometer']

    if 'compass' in exif:
        metadata.compass = exif['compass']

    if 'capture_time' in exif:
        metadata.capture_time = exif['capture_time']

    if 'skey' in exif:
        metadata.skey = exif['skey']

    return metadata


def _two_view_reconstruction_inliers(b1, b2, R, t, threshold):
    p = pyopengv.triangulation_triangulate(b1, b2, t, R)

    br1 = p.copy()
    br1 /= np.linalg.norm(br1, axis=1)[:, np.newaxis]

    br2 = R.T.dot((p - t).T).T
    br2 /= np.linalg.norm(br2, axis=1)[:, np.newaxis]

    ok1 = np.linalg.norm(br1 - b1, axis=1) < threshold
    ok2 = np.linalg.norm(br2 - b2, axis=1) < threshold
    return np.nonzero(ok1 * ok2)[0]


def run_relative_pose_ransac(b1, b2, method, threshold, iterations):
    return pyopengv.relative_pose_ransac(b1, b2, method, threshold, iterations)


def run_relative_pose_optimize_nonlinear(b1, b2, t, R):
    return pyopengv.relative_pose_optimize_nonlinear(b1, b2, t, R)


def two_view_reconstruction(p1, p2, camera1, camera2, threshold):
    b1 = camera1.pixel_bearings(p1)
    b2 = camera2.pixel_bearings(p2)

    # Note on threshold:
    # See opengv doc on thresholds here: http://laurentkneip.github.io/opengv/page_how_to_use.html
    # Here we arbitrarily assume that the threshold is given for a camera of focal length 1
    # Also arctan(threshold) \approx threshold since threshold is small
    T = run_relative_pose_ransac(b1, b2, "STEWENIUS", 1 - np.cos(threshold), 1000)
    R = T[:, :3]
    t = T[:, 3]
    inliers = _two_view_reconstruction_inliers(b1, b2, R, t, threshold)

    T = run_relative_pose_optimize_nonlinear(b1[inliers], b2[inliers], t, R)
    R = T[:, :3]
    t = T[:, 3]
    inliers = _two_view_reconstruction_inliers(b1, b2, R, t, threshold)

    return cv2.Rodrigues(R.T)[0].ravel(), -R.T.dot(t), inliers


def _two_view_reconstruction_rotation_only_inliers(b1, b2, R, threshold):
    br2 = R.dot(b2.T).T
    ok = np.linalg.norm(br2 - b1, axis=1) < threshold
    return np.nonzero(ok)[0]


def two_view_reconstruction_rotation_only(p1, p2, camera1, camera2, threshold):
    b1 = camera1.pixel_bearings(p1)
    b2 = camera2.pixel_bearings(p2)
    t = np.zeros(3)

    R = pyopengv.relative_pose_ransac_rotation_only(b1, b2, 1 - np.cos(threshold), 1000)
    inliers = _two_view_reconstruction_rotation_only_inliers(b1, b2, R, threshold)

    # R = pyopengv.relative_pose_rotation_only(b1[inliers], b2[inliers])
    # inliers = _two_view_reconstruction_rotation_only_inliers(b1, b2, R, threshold)

    return cv2.Rodrigues(R.T)[0].ravel(), inliers


def bootstrap_reconstruction(data, graph, im1, im2):
    '''Starts a reconstruction using two shots.
    '''
    print 'Initial reconstruction with', im1, 'and', im2
    d1 = data.load_exif(im1)
    d2 = data.load_exif(im2)
    cameras = data.load_camera_models()
    camera1 = cameras[d1['camera']]
    camera2 = cameras[d2['camera']]

    tracks, p1, p2 = matching.common_tracks(graph, im1, im2)
    print 'Number of common tracks', len(tracks)

    threshold = data.config.get('five_point_algo_threshold', 0.006)
    R, t, inliers = two_view_reconstruction(p1, p2, camera1, camera2, threshold)
    if len(inliers) > 5:
        print 'Number of inliers', len(inliers)
        reconstruction = types.Reconstruction()
        reconstruction.cameras = cameras

        shot1 = types.Shot()
        shot1.id = im1
        shot1.camera = cameras[str(d1['camera'])]
        shot1.pose = types.Pose()
        shot1.metadata = get_image_metadata(data, im1)
        reconstruction.add_shot(shot1)

        shot2 = types.Shot()
        shot2.id = im2
        shot2.camera = cameras[str(d2['camera'])]
        shot2.pose = types.Pose()
        shot2.pose.rotation = R
        shot2.pose.translation = t
        shot2.metadata = get_image_metadata(data, im2)
        reconstruction.add_shot(shot2)

        triangulate_shot_features(
                    graph, reconstruction, im1,
                    data.config.get('triangulation_threshold', 0.004),
                    data.config.get('triangulation_min_ray_angle', 2.0))
        print 'Number of reconstructed 3D points :{}'.format(len(reconstruction.points))
        if len(reconstruction.points) > data.config.get('five_point_algo_min_inliers', 50):
            print 'Found initialize good pair', im1 , 'and', im2
            bundle_single_view(graph, reconstruction, im2, data.config)
            retriangulate(graph, reconstruction, data.config)
            bundle_single_view(graph, reconstruction, im2, data.config)
            return reconstruction

    print 'Pair', im1, ':', im2, 'fails'
    return None


def reconstructed_points_for_images(graph, reconstruction, images):
    res = []
    for image in images:
        if image not in reconstruction.shots:
            common_tracks = 0
            for track in graph[image]:
                if track in reconstruction.points:
                    common_tracks += 1
            res.append((image, common_tracks))
    return sorted(res, key=lambda x: -x[1])


def single_reprojection_error(shot, point, observation):
    ''' Reprojection error of a single points
    '''
    point_reprojected = shot.project(point.coordinates)
    err = point_reprojected - observation
    return np.linalg.norm(err)


def reprojection_error(graph, reconstruction):
    errors = []
    for shot_id in reconstruction.shots:
        for track_id in graph[shot_id]:
            if track_id in reconstruction.points:
                observation = graph[shot_id][track_id]['feature']
                shot = reconstruction.shots[shot_id]
                point = reconstruction.points[track_id]
                errors.append(single_reprojection_error(shot, point, observation))
    return np.median(errors)


def resect(data, graph, reconstruction, shot_id):
    '''Add a shot to the reconstruction.
    '''
    exif = data.load_exif(shot_id)
    camera = reconstruction.cameras[exif['camera']]

    bs = []
    Xs = []
    for track in graph[shot_id]:
        if track in reconstruction.points:
            x = graph[track][shot_id]['feature']
            b = camera.pixel_bearing(x)
            bs.append(b)
            Xs.append(reconstruction.points[track].coordinates)
    bs = np.array(bs)
    Xs = np.array(Xs)
    if len(bs) < 5:
        return False

    threshold = data.config.get('resection_threshold', 0.004)
    T = pyopengv.absolute_pose_ransac(bs, Xs, "KNEIP", 1 - np.cos(threshold), 1000)

    R = T[:, :3]
    t = T[:, 3]

    reprojected_bs = R.T.dot((Xs - t).T).T
    reprojected_bs /= np.linalg.norm(reprojected_bs, axis=1)[:, np.newaxis]

    inliers = np.linalg.norm(reprojected_bs - bs, axis=1) < threshold
    ninliers = sum(inliers)

    print 'Resection', shot_id, 'inliers:', ninliers, '/', len(bs)
    if ninliers >= data.config.get('resection_min_inliers', 15):
        R = T[:, :3].T
        t = -R.dot(T[:, 3])
        shot = types.Shot()
        shot.id = shot_id
        shot.camera = camera
        shot.pose = types.Pose()
        shot.pose.set_rotation_matrix(R)
        shot.pose.translation = t
        shot.metadata = get_image_metadata(data, shot_id)
        reconstruction.add_shot(shot)
        bundle_single_view(graph, reconstruction, shot_id, data.config)
        return True
    else:
        return False


def triangulate_track(track, graph, reconstruction, Rt_by_id, reproj_threshold, min_ray_angle_degrees):
    min_ray_angle = np.radians(min_ray_angle_degrees)
    Rts, bs = [], []

    for shot_id in graph[track]:
        if shot_id in reconstruction.shots:
            shot = reconstruction.shots[shot_id]
            if shot_id not in Rt_by_id:
                Rt_by_id[shot_id] = shot.pose.get_Rt()
            Rts.append(Rt_by_id[shot_id])
            x = graph[track][shot_id]['feature']
            b = shot.camera.pixel_bearing(np.array(x))
            bs.append(b)

    if len(Rts) >= 2:
        e, X = csfm.triangulate_bearings(Rts, bs, reproj_threshold, min_ray_angle)
        if X is not None:
            point = types.Point()
            point.id = track
            point.coordinates = X.tolist()
            reconstruction.add_point(point)


def triangulate_shot_features(graph, reconstruction, shot_id, reproj_threshold, min_ray_angle):
    '''Reconstruct as many tracks seen in shot_id as possible.
    '''
    Rt_by_id = {}

    for track in graph[shot_id]:
        if track not in reconstruction.points:
            triangulate_track(track, graph, reconstruction, Rt_by_id, reproj_threshold, min_ray_angle)


def retriangulate(graph, reconstruction, config):
    '''Retrianguate all points
    '''
    threshold = config.get('triangulation_threshold', 0.004)
    min_ray_angle = config.get('triangulation_min_ray_angle', 2.0)
    Rt_by_id = {}
    tracks, images = tracks_and_images(graph)
    for track in tracks:
        triangulate_track(track, graph, reconstruction, Rt_by_id, threshold, min_ray_angle)


def remove_outliers(graph, reconstruction, config):
    threshold = config.get('bundle_outlier_threshold', 0.008)
    if threshold > 0:
        outliers = []
        for track in reconstruction.points:
            error = reconstruction.points[track].reprojection_error
            if error > threshold:
                outliers.append(track)
        for track in outliers:
            del reconstruction.points[track]
        print 'Remove {0} outliers'.format(len(outliers))


def apply_similarity(reconstruction, s, A, b):
    """Apply a similarity (y = s A x + b) to a reconstruction.

    :param reconstruction: The reconstruction to transform.
    :param s: The scale (a scalar)
    :param A: The rotation matrix (3x3)
    :param b: The translation vector (3)
    """
    # Align points.
    for point in reconstruction.points.values():
        Xp = s * A.dot(point.coordinates) + b
        point.coordinates = Xp.tolist()

    # Align cameras.
    for shot in reconstruction.shots.values():
        R = shot.pose.get_rotation_matrix()
        t = np.array(shot.pose.translation)
        Rp = R.dot(A.T)
        tp = -Rp.dot(b) + s * t
        shot.pose.set_rotation_matrix(Rp)
        shot.pose.translation = list(tp)


def align_reconstruction_naive_similarity(reconstruction):
    if len(reconstruction.shots) < 3:
        return
    # Compute similarity Xp = s A X + b
    X, Xp = [], []
    for shot in reconstruction.shots.values():
        X.append(shot.pose.get_origin())
        Xp.append(shot.metadata.gps_position)
    X = np.array(X)
    Xp = np.array(Xp)
    T = tf.superimposition_matrix(X.T, Xp.T, scale=True)

    A, b = T[:3, :3], T[:3, 3]
    s = np.linalg.det(A)**(1. / 3)
    A /= s
    return s, A, b


def get_horizontal_and_vertical_directions(R, orientation):
    '''Get orientation vectors from camera rotation matrix and orientation tag.

    Return a 3D vectors pointing to the positive XYZ directions of the image.
    X points to the right, Y to the bottom, Z to the front.
    '''
    # See http://sylvana.net/jpegcrop/exif_orientation.html
    if orientation == 1:
        return R[0, :], R[1, :], R[2, :]
    if orientation == 2:
        return -R[0, :], R[1, :], -R[2, :]
    if orientation == 3:
        return -R[0, :], -R[1, :], R[2, :]
    if orientation == 4:
        return R[0, :], -R[1, :], R[2, :]
    if orientation == 5:
        return R[1, :], R[0, :], -R[2, :]
    if orientation == 6:
        return -R[1, :], R[0, :], R[2, :]
    if orientation == 7:
        return -R[1, :], -R[0, :], -R[2, :]
    if orientation == 8:
        return R[1, :], -R[0, :], R[2, :]
    print 'ERROR unknown orientation {0} - Assuming orientation 1 instead.'.format(orientation)
    return R[0, :], R[1, :], R[2, :]


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
    for shot in reconstruction.shots.values():
        X.append(shot.pose.get_origin())
        Xp.append(shot.metadata.gps_position)
        R = shot.pose.get_rotation_matrix()
        x, y, z = get_horizontal_and_vertical_directions(R, shot.metadata.orientation)
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


def shot_lla_and_compass(shot, reference):
    """
    Lat, lon, alt and compass of the reconstructed shot position
    """
    topo = shot.pose.get_origin()
    lat, lon, alt = geo.lla_from_topocentric(
        topo[0], topo[1], topo[2],
        reference['latitude'], reference['longitude'], reference['altitude'])

    dz = shot.viewing_direction()
    angle = np.rad2deg(np.arctan2(dz[0], dz[1]))
    angle = (angle + 360) % 360
    return lat, lon, alt, angle


def merge_two_reconstructions(r1, r2, config, threshold=1):
    ''' Merge two reconstructions with common tracks
    '''
    t1, t2 = r1.points, r2.points
    common_tracks = list(set(t1) & set(t2))

    # print 'Number of common tracks between two reconstructions: {0}'.format(len(common_tracks))
    if len(common_tracks) > 6:

        # Estimate similarity transform
        p1 = np.array([t1[t].coordinates for t in common_tracks])
        p2 = np.array([t2[t].coordinates for t in common_tracks])

        T, inliers = multiview.fit_similarity_transform(p1, p2, max_iterations=1000, threshold=threshold)

        if len(inliers) >= 10:
            s, A, b = multiview.decompose_similarity_transform(T)
            r1p = r1
            apply_similarity(r1p, s, A, b)
            r = r2
            r.shots.update(r1p.shots)
            r.points.update(r1p.points)
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
    for k, point in reconstruction.points.iteritems():
        point.color = graph[k].values()[0]['feature_color']


def grow_reconstruction(data, graph, reconstruction, images):
    bundle_interval = data.config.get('bundle_interval', 0)
    bundle_new_points_ratio = data.config.get('bundle_new_points_ratio', 1.2)
    retriangulation = data.config.get('retriangulation', False)
    retriangulation_ratio = data.config.get('retriangulation_ratio', 1.25)

    bundle(graph, reconstruction, data.config)
    align_reconstruction(reconstruction, data.config)

    num_points_last_bundle = len(reconstruction.points)
    num_shots_last_bundle = len(reconstruction.shots)
    num_points_last_retriangulation = len(reconstruction.points)
    num_shots_reconstructed = len(reconstruction.shots)

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

                if (len(reconstruction.points) >= num_points_last_bundle * bundle_new_points_ratio
                        or len(reconstruction.shots) >= num_shots_last_bundle + bundle_interval):
                    bundle(graph, reconstruction, data.config)
                    remove_outliers(graph, reconstruction, data.config)
                    align_reconstruction(reconstruction, data.config)
                    num_points_last_bundle = len(reconstruction.points)
                    num_shots_last_bundle = len(reconstruction.shots)

                num_points = len(reconstruction.points)
                if retriangulation and num_points > num_points_last_retriangulation * retriangulation_ratio:
                    print 'Re-triangulating'
                    retriangulate(graph, reconstruction, data.config)
                    bundle(graph, reconstruction, data.config)
                    num_points_last_retriangulation = len(reconstruction.points)
                    print '  Reprojection Error:', reprojection_error(graph, reconstruction)

                break
        else:
            print 'Some images can not be added'
            break

    bundle(graph, reconstruction, data.config)
    align_reconstruction(reconstruction, data.config)

    print 'Reprojection Error:', reprojection_error(graph, reconstruction)
    print 'Painting the reconstruction from {0} cameras'.format(len(reconstruction.shots))
    paint_reconstruction(data, graph, reconstruction)
    print 'Done.'
    return reconstruction


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
    remaining_images = set(images)
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
                reconstruction = grow_reconstruction(data, graph, reconstruction, remaining_images)
                reconstructions.append(reconstruction)
                reconstructions = sorted(reconstructions, key=lambda x: -len(x.shots))
                data.save_reconstruction(reconstructions)

    for k, r in enumerate(reconstructions):
        print 'Reconstruction', k, ':', len(r.shots), 'images', ',', len(r.points),'points'

    print len(reconstructions), 'partial reconstructions in total.'
