# -*- coding: utf-8 -*-
"""Incremental reconstruction pipeline"""

import datetime
import logging
from itertools import combinations

import numpy as np
import cv2
import pyopengv
import time
from multiprocessing import Pool

from opensfm import align
from opensfm import csfm
from opensfm import geo
from opensfm import matching
from opensfm import multiview
from opensfm import types


logger = logging.getLogger(__name__)


def bundle(graph, reconstruction, gcp, config):
    """Bundle adjust a reconstruction."""
    fix_cameras = not config['optimize_camera_parameters']

    start = time.time()
    ba = csfm.BundleAdjuster()
    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            ba.add_perspective_camera(
                str(camera.id), camera.focal, camera.k1, camera.k2,
                camera.focal_prior, camera.k1_prior, camera.k2_prior,
                fix_cameras)
        elif camera.projection_type == 'fisheye':
            ba.add_fisheye_camera(
                str(camera.id), camera.focal, camera.k1, camera.k2,
                camera.focal_prior, camera.k1_prior, camera.k2_prior,
                fix_cameras)
        elif camera.projection_type in ['equirectangular', 'spherical']:
            ba.add_equirectangular_camera(str(camera.id))

    for shot in reconstruction.shots.values():
        r = shot.pose.rotation
        t = shot.pose.translation
        ba.add_shot(
            str(shot.id), str(shot.camera.id),
            r[0], r[1], r[2],
            t[0], t[1], t[2],
            False
        )

    for point in reconstruction.points.values():
        x = point.coordinates
        ba.add_point(str(point.id), x[0], x[1], x[2], False)

    for shot_id in reconstruction.shots:
        if shot_id in graph:
            for track in graph[shot_id]:
                if track in reconstruction.points:
                    ba.add_observation(str(shot_id), str(track),
                                       *graph[shot_id][track]['feature'])

    if config['bundle_use_gps']:
        for shot in reconstruction.shots.values():
            g = shot.metadata.gps_position
            ba.add_position_prior(str(shot.id), g[0], g[1], g[2],
                                  shot.metadata.gps_dop)

    if config['bundle_use_gcp'] and gcp:
        for observation in gcp:
            if observation.shot_id in reconstruction.shots:
                ba.add_ground_control_point_observation(
                    str(observation.shot_id),
                    observation.coordinates[0],
                    observation.coordinates[1],
                    observation.coordinates[2],
                    observation.shot_coordinates[0],
                    observation.shot_coordinates[1])

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(
        config.get('exif_focal_sd', 0.01),
        config.get('radial_distorsion_k1_sd', 0.01),
        config.get('radial_distorsion_k2_sd', 0.01))

    setup = time.time()

    ba.set_num_threads(config['processes'])
    ba.run()

    run = time.time()
    logger.debug(ba.brief_report())

    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            c = ba.get_perspective_camera(str(camera.id))
            camera.focal = c.focal
            camera.k1 = c.k1
            camera.k2 = c.k2
        elif camera.projection_type == 'fisheye':
            c = ba.get_fisheye_camera(str(camera.id))
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

    logger.debug('Bundle setup/run/teardown {0}/{1}/{2}'.format(
        setup - start, run - setup, teardown - run))


def bundle_single_view(graph, reconstruction, shot_id, config):
    """Bundle adjust a single camera."""
    ba = csfm.BundleAdjuster()
    shot = reconstruction.shots[shot_id]
    camera = shot.camera

    if camera.projection_type == 'perspective':
        ba.add_perspective_camera(
            str(camera.id), camera.focal, camera.k1, camera.k2,
            camera.focal_prior, camera.k1_prior, camera.k2_prior, True)
    elif camera.projection_type == 'fisheye':
        ba.add_fisheye_camera(
            str(camera.id), camera.focal, camera.k1, camera.k2,
            camera.focal_prior, camera.k1_prior, camera.k2_prior, True)
    elif camera.projection_type in ['equirectangular', 'spherical']:
        ba.add_equirectangular_camera(str(camera.id))

    r = shot.pose.rotation
    t = shot.pose.translation
    ba.add_shot(
        str(shot.id), str(camera.id),
        r[0], r[1], r[2],
        t[0], t[1], t[2],
        False
    )

    for track_id in graph[shot_id]:
        if track_id in reconstruction.points:
            track = reconstruction.points[track_id]
            x = track.coordinates
            ba.add_point(str(track_id), x[0], x[1], x[2], True)
            ba.add_observation(str(shot_id), str(track_id),
                               *graph[shot_id][track_id]['feature'])

    if config['bundle_use_gps']:
        g = shot.metadata.gps_position
        ba.add_position_prior(str(shot.id), g[0], g[1], g[2],
                              shot.metadata.gps_dop)

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(
        config.get('exif_focal_sd', 0.01),
        config.get('radial_distorsion_k1_sd', 0.01),
        config.get('radial_distorsion_k2_sd', 0.01))

    ba.set_num_threads(config['processes'])
    ba.run()

    s = ba.get_shot(str(shot_id))
    shot.pose.rotation = [s.rx, s.ry, s.rz]
    shot.pose.translation = [s.tx, s.ty, s.tz]


def bundle_local(graph, reconstruction, gcp, central_shot_id, config):
    """Bundle adjust the local neighborhood of a shot."""
    start = time.time()

    interior, boundary = shot_neighborhood(
        graph, reconstruction, central_shot_id, config['local_bundle_radius'])

    logger.debug('Local bundle sets: interior {}  boundary {}  other {}'.format(
          len(interior),
          len(boundary),
          len(reconstruction.shots) - len(interior) - len(boundary)))

    point_ids = set()
    for shot_id in interior:
        if shot_id in graph:
            for track in graph[shot_id]:
                if track in reconstruction.points:
                    point_ids.add(track)

    ba = csfm.BundleAdjuster()
    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            ba.add_perspective_camera(
                str(camera.id), camera.focal, camera.k1, camera.k2,
                camera.focal_prior, camera.k1_prior, camera.k2_prior,
                True)

        elif camera.projection_type in ['equirectangular', 'spherical']:
            ba.add_equirectangular_camera(str(camera.id))

    for shot_id in interior | boundary:
        shot = reconstruction.shots[shot_id]
        r = shot.pose.rotation
        t = shot.pose.translation
        ba.add_shot(
            str(shot.id), str(shot.camera.id),
            r[0], r[1], r[2],
            t[0], t[1], t[2],
            shot.id in boundary
        )

    for point_id in point_ids:
        point = reconstruction.points[point_id]
        x = point.coordinates
        ba.add_point(str(point.id), x[0], x[1], x[2], False)

    for shot_id in interior | boundary:
        if shot_id in graph:
            for track in graph[shot_id]:
                if track in reconstruction.points:
                    ba.add_observation(str(shot_id), str(track),
                                       *graph[shot_id][track]['feature'])

    if config['bundle_use_gps']:
        for shot_id in interior:
            shot = reconstruction.shots[shot_id]
            g = shot.metadata.gps_position
            ba.add_position_prior(str(shot.id), g[0], g[1], g[2],
                                  shot.metadata.gps_dop)

    if config['bundle_use_gcp'] and gcp:
        for observation in gcp:
            if observation.shot_id in interior:
                ba.add_ground_control_point_observation(
                    observation.shot_id,
                    observation.coordinates[0],
                    observation.coordinates[1],
                    observation.coordinates[2],
                    observation.shot_coordinates[0],
                    observation.shot_coordinates[1])

    ba.set_loss_function(config.get('loss_function', 'SoftLOneLoss'),
                         config.get('loss_function_threshold', 1))
    ba.set_reprojection_error_sd(config.get('reprojection_error_sd', 0.004))
    ba.set_internal_parameters_prior_sd(
        config.get('exif_focal_sd', 0.01),
        config.get('radial_distorsion_k1_sd', 0.01),
        config.get('radial_distorsion_k2_sd', 0.01))

    setup = time.time()

    ba.set_num_threads(config['processes'])
    ba.run()

    run = time.time()
    logger.debug(ba.brief_report())

    for camera in reconstruction.cameras.values():
        if camera.projection_type == 'perspective':
            c = ba.get_perspective_camera(str(camera.id))
            camera.focal = c.focal
            camera.k1 = c.k1
            camera.k2 = c.k2

    for shot_id in interior:
        shot = reconstruction.shots[shot_id]
        s = ba.get_shot(str(shot.id))
        shot.pose.rotation = [s.rx, s.ry, s.rz]
        shot.pose.translation = [s.tx, s.ty, s.tz]

    for point in point_ids:
        point = reconstruction.points[point]
        p = ba.get_point(str(point.id))
        point.coordinates = [p.x, p.y, p.z]
        point.reprojection_error = p.reprojection_error

    teardown = time.time()

    logger.debug('Local bundle setup/run/teardown {0}/{1}/{2}'.format(
        setup - start, run - setup, teardown - run))


def shot_neighborhood(graph, reconstruction, central_shot_id, radius):
    """Reconstructed shots near a given shot.

    Returns:
        a tuple with interior and boundary:
        - interior: the list of shots at distance smaller than radius
        - boundary: shots at distance radius

    Central shot is at distance 0.  Shots at distance n + 1 share at least
    one point with shots at distance n.
    """
    interior = set()
    boundary = set([central_shot_id])
    for distance in range(radius):
        new_boundary = set()
        for shot_id in boundary:
            neighbors = shot_direct_neighbors(graph, reconstruction, shot_id)
            for neighbor in neighbors:
                if neighbor not in boundary and neighbor not in interior:
                    new_boundary.add(neighbor)
        interior.update(boundary)
        boundary = new_boundary
    return interior, boundary


def shot_direct_neighbors(graph, reconstruction, shot_id):
    """Reconstructed shots sharing reconstructed points with a given shot."""
    neighbors = set()
    for track_id in graph[shot_id]:
        if track_id in reconstruction.points:
            for neighbor in graph[track_id]:
                if neighbor in reconstruction.shots:
                    neighbors.add(neighbor)
    return neighbors


def pairwise_reconstructability(common_tracks, homography_inliers):
    """Likeliness of an image pair giving a good initial reconstruction."""
    outliers = common_tracks - homography_inliers
    outlier_ratio = float(outliers) / common_tracks
    if outlier_ratio > 0.3:
        return common_tracks
    else:
        return 0


def compute_image_pairs(track_dict, config):
    """All matched image pairs sorted by reconstructability."""
    args = _pair_reconstructability_arguments(track_dict, config)
    processes = config.get('processes', 1)
    if processes == 1:
        result = map(_compute_pair_reconstructability, args)
    else:
        p = Pool(processes)
        result = p.map(_compute_pair_reconstructability, args)
    pairs = [(im1, im2) for im1, im2, r in result if r > 0]
    score = [r for im1, im2, r in result if r > 0]
    order = np.argsort(-np.array(score))
    return [pairs[o] for o in order]


def _pair_reconstructability_arguments(track_dict, config):
    threshold = config.get('homography_threshold', 0.004)
    return [(threshold, im1, im2, p1, p2)
            for (im1, im2), (tracks, p1, p2) in track_dict.items()]


def _compute_pair_reconstructability(args):
    threshold, im1, im2, p1, p2 = args
    H, inliers = cv2.findHomography(p1, p2, cv2.RANSAC, threshold)
    r = pairwise_reconstructability(len(p1), inliers.sum())
    return (im1, im2, r)


def get_image_metadata(data, image):
    """Get image metadata as a ShotMetadata object."""
    metadata = types.ShotMetadata()
    exif = data.load_exif(image)
    reflla = data.load_reference_lla()
    if ('gps' in exif and
            'latitude' in exif['gps'] and
            'longitude' in exif['gps']):
        lat = exif['gps']['latitude']
        lon = exif['gps']['longitude']
        if data.config.get('use_altitude_tag', False):
            alt = exif['gps'].get('altitude', 2.0)
        else:
            alt = 2.0  # Arbitrary value used to align the reconstruction
        x, y, z = geo.topocentric_from_lla(
            lat, lon, alt,
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
    """Reconstruct two views from point correspondences.

    Args:
        p1, p2: lists points in the images
        camera1, camera2: Camera models
        threshold: reprojection error threshold

    Returns:
        rotation, translation and inlier list
    """
    b1 = camera1.pixel_bearings(p1)
    b2 = camera2.pixel_bearings(p2)

    # Note on threshold:
    # See opengv doc on thresholds here:
    #   http://laurentkneip.github.io/opengv/page_how_to_use.html
    # Here we arbitrarily assume that the threshold is given for a camera of
    # focal length 1.  Also, arctan(threshold) \approx threshold since
    # threshold is small
    T = run_relative_pose_ransac(
        b1, b2, "STEWENIUS", 1 - np.cos(threshold), 1000)
    R = T[:, :3]
    t = T[:, 3]
    inliers = _two_view_reconstruction_inliers(b1, b2, R, t, threshold)

    T = run_relative_pose_optimize_nonlinear(b1[inliers], b2[inliers], t, R)
    R = T[:, :3]
    t = T[:, 3]
    inliers = _two_view_reconstruction_inliers(b1, b2, R, t, threshold)

    return cv2.Rodrigues(R.T)[0].ravel(), -R.T.dot(t), inliers


def _two_view_rotation_inliers(b1, b2, R, threshold):
    br2 = R.dot(b2.T).T
    ok = np.linalg.norm(br2 - b1, axis=1) < threshold
    return np.nonzero(ok)[0]


def two_view_reconstruction_rotation_only(p1, p2, camera1, camera2, threshold):
    """Find rotation between two views from point correspondences.

    Args:
        p1, p2: lists points in the images
        camera1, camera2: Camera models
        threshold: reprojection error threshold

    Returns:
        rotation and inlier list
    """
    b1 = camera1.pixel_bearings(p1)
    b2 = camera2.pixel_bearings(p2)

    R = pyopengv.relative_pose_ransac_rotation_only(
        b1, b2, 1 - np.cos(threshold), 1000)
    inliers = _two_view_rotation_inliers(b1, b2, R, threshold)

    return cv2.Rodrigues(R.T)[0].ravel(), inliers


def bootstrap_reconstruction(data, graph, im1, im2, p1, p2):
    """Start a reconstruction using two shots."""
    logger.info("Starting reconstruction with {} and {}".format(im1, im2))
    d1 = data.load_exif(im1)
    d2 = data.load_exif(im2)
    cameras = data.load_camera_models()
    camera1 = cameras[d1['camera']]
    camera2 = cameras[d2['camera']]

    logger.info("Common tracks: {}".format(len(p1)))

    thresh = data.config.get('five_point_algo_threshold', 0.006)
    min_inliers = data.config.get('five_point_algo_min_inliers', 50)
    R, t, inliers = two_view_reconstruction(p1, p2, camera1, camera2, thresh)
    if len(inliers) > 5:
        logger.info("Two-view reconstruction inliers {}".format(len(inliers)))
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
        shot2.pose = types.Pose(R, t)
        shot2.metadata = get_image_metadata(data, im2)
        reconstruction.add_shot(shot2)

        triangulate_shot_features(
            graph, reconstruction, im1,
            data.config.get('triangulation_threshold', 0.004),
            data.config.get('triangulation_min_ray_angle', 2.0))
        logger.info("Triangulated: {}".format(len(reconstruction.points)))
        if len(reconstruction.points) > min_inliers:
            bundle_single_view(graph, reconstruction, im2, data.config)
            retriangulate(graph, reconstruction, data.config)
            bundle_single_view(graph, reconstruction, im2, data.config)
            return reconstruction

    logger.info("Starting reconstruction with {} and {} failed")


def reconstructed_points_for_images(graph, reconstruction, images):
    """Number of reconstructed points visible on each image.

    Returns:
        A list of (image, num_point) pairs sorted by decreasing number
        of points.
    """
    res = []
    for image in images:
        if image not in reconstruction.shots:
            common_tracks = 0
            for track in graph[image]:
                if track in reconstruction.points:
                    common_tracks += 1
            res.append((image, common_tracks))
    return sorted(res, key=lambda x: -x[1])


def resect(data, graph, reconstruction, shot_id):
    """Try resecting and adding a shot to the reconstruction.

    Return:
        True on success.
    """
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
    T = pyopengv.absolute_pose_ransac(
        bs, Xs, "KNEIP", 1 - np.cos(threshold), 1000)

    R = T[:, :3]
    t = T[:, 3]

    reprojected_bs = R.T.dot((Xs - t).T).T
    reprojected_bs /= np.linalg.norm(reprojected_bs, axis=1)[:, np.newaxis]

    inliers = np.linalg.norm(reprojected_bs - bs, axis=1) < threshold
    ninliers = sum(inliers)

    logger.info("{} resection inliers: {} / {}".format(
        shot_id, ninliers, len(bs)))
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


class TrackTriangulator:
    """Triangulate tracks in a reconstruction.

    Caches shot origin and rotation matrix
    """

    def __init__(self, graph, reconstruction):
        """Build a triangulator for a specific reconstruction."""
        self.graph = graph
        self.reconstruction = reconstruction
        self.origins = {}
        self.rotation_inverses = {}
        self.Rts = {}

    def triangulate(self, track, reproj_threshold, min_ray_angle_degrees):
        """Triangulate track and add point to reconstruction."""
        os, bs = [], []
        for shot_id in self.graph[track]:
            if shot_id in self.reconstruction.shots:
                shot = self.reconstruction.shots[shot_id]
                os.append(self._shot_origin(shot))
                x = self.graph[track][shot_id]['feature']
                b = shot.camera.pixel_bearing(np.array(x))
                r = self._shot_rotation_inverse(shot)
                bs.append(r.dot(b))

        if len(os) >= 2:
            e, X = csfm.triangulate_bearings_midpoint(
                os, bs, reproj_threshold, np.radians(min_ray_angle_degrees))
            if X is not None:
                point = types.Point()
                point.id = track
                point.coordinates = X.tolist()
                self.reconstruction.add_point(point)

    def triangulate_dlt(self, track, reproj_threshold, min_ray_angle_degrees):
        """Triangulate track using DLT and add point to reconstruction."""
        Rts, bs = [], []
        for shot_id in self.graph[track]:
            if shot_id in self.reconstruction.shots:
                shot = self.reconstruction.shots[shot_id]
                Rts.append(self._shot_Rt(shot))
                x = self.graph[track][shot_id]['feature']
                b = shot.camera.pixel_bearing(np.array(x))
                bs.append(b)

        if len(Rts) >= 2:
            e, X = csfm.triangulate_bearings_dlt(
                Rts, bs, reproj_threshold, np.radians(min_ray_angle_degrees))
            if X is not None:
                point = types.Point()
                point.id = track
                point.coordinates = X.tolist()
                self.reconstruction.add_point(point)

    def _shot_origin(self, shot):
        if shot.id in self.origins:
            return self.origins[shot.id]
        else:
            o = shot.pose.get_origin()
            self.origins[shot.id] = o
            return o

    def _shot_rotation_inverse(self, shot):
        if shot.id in self.rotation_inverses:
            return self.rotation_inverses[shot.id]
        else:
            r = shot.pose.get_rotation_matrix().T
            self.rotation_inverses[shot.id] = r
            return r

    def _shot_Rt(self, shot):
        if shot.id in self.Rts:
            return self.Rts[shot.id]
        else:
            r = shot.pose.get_Rt()
            self.Rts[shot.id] = r
            return r


def triangulate_shot_features(graph, reconstruction, shot_id, reproj_threshold,
                              min_ray_angle):
    """Reconstruct as many tracks seen in shot_id as possible."""
    triangulator = TrackTriangulator(graph, reconstruction)

    for track in graph[shot_id]:
        if track not in reconstruction.points:
            triangulator.triangulate(track, reproj_threshold, min_ray_angle)


def retriangulate(graph, reconstruction, config):
    """Retrianguate all points"""
    threshold = config.get('triangulation_threshold', 0.004)
    min_ray_angle = config.get('triangulation_min_ray_angle', 2.0)
    triangulator = TrackTriangulator(graph, reconstruction)
    tracks, images = matching.tracks_and_images(graph)
    for track in tracks:
        triangulator.triangulate(track, threshold, min_ray_angle)


def remove_outliers(graph, reconstruction, config):
    """Remove points with large reprojection error."""
    threshold = config.get('bundle_outlier_threshold', 0.008)
    if threshold > 0:
        outliers = []
        for track in reconstruction.points:
            error = reconstruction.points[track].reprojection_error
            if error > threshold:
                outliers.append(track)
        for track in outliers:
            del reconstruction.points[track]
        logger.info("Removed outliers: {}".format(len(outliers)))


def shot_lla_and_compass(shot, reference):
    """Lat, lon, alt and compass of the reconstructed shot position."""
    topo = shot.pose.get_origin()
    lat, lon, alt = geo.lla_from_topocentric(
        topo[0], topo[1], topo[2],
        reference['latitude'], reference['longitude'], reference['altitude'])

    dz = shot.viewing_direction()
    angle = np.rad2deg(np.arctan2(dz[0], dz[1]))
    angle = (angle + 360) % 360
    return lat, lon, alt, angle


def merge_two_reconstructions(r1, r2, config, threshold=1):
    """Merge two reconstructions with common tracks."""
    t1, t2 = r1.points, r2.points
    common_tracks = list(set(t1) & set(t2))

    if len(common_tracks) > 6:

        # Estimate similarity transform
        p1 = np.array([t1[t].coordinates for t in common_tracks])
        p2 = np.array([t2[t].coordinates for t in common_tracks])

        T, inliers = multiview.fit_similarity_transform(
            p1, p2, max_iterations=1000, threshold=threshold)

        if len(inliers) >= 10:
            s, A, b = multiview.decompose_similarity_transform(T)
            r1p = r1
            align.apply_similarity(r1p, s, A, b)
            r = r2
            r.shots.update(r1p.shots)
            r.points.update(r1p.points)
            align.align_reconstruction(r, None, config)
            return [r]
        else:
            return [r1, r2]
    else:
        return [r1, r2]


def merge_reconstructions(reconstructions, config):
    """Greedily merge reconstructions with common tracks."""
    num_reconstruction = len(reconstructions)
    ids_reconstructions = np.arange(num_reconstruction)
    remaining_reconstruction = ids_reconstructions
    reconstructions_merged = []
    num_merge = 0

    for (i, j) in combinations(ids_reconstructions, 2):
        if (i in remaining_reconstruction) and (j in remaining_reconstruction):
            r = merge_two_reconstructions(
                reconstructions[i], reconstructions[j], config)
            if len(r) == 1:
                remaining_reconstruction = list(set(
                    remaining_reconstruction) - set([i, j]))
                for k in remaining_reconstruction:
                    rr = merge_two_reconstructions(r[0], reconstructions[k],
                                                   config)
                    if len(r) == 2:
                        break
                    else:
                        r = rr
                        remaining_reconstruction = list(set(
                            remaining_reconstruction) - set([k]))
                reconstructions_merged.append(r[0])
                num_merge += 1

    for k in remaining_reconstruction:
        reconstructions_merged.append(reconstructions[k])

    logger.info("Merged {0} reconstructions".format(num_merge))

    return reconstructions_merged


def paint_reconstruction(data, graph, reconstruction):
    """Set the color of the points from the color of the tracks."""
    for k, point in reconstruction.points.iteritems():
        point.color = graph[k].values()[0]['feature_color']


class ShouldBundle:
    """Helper to keep track of when to run bundle."""

    def __init__(self, data, reconstruction):
        self.interval = data.config.get('bundle_interval', 0)
        self.new_points_ratio = data.config.get('bundle_new_points_ratio', 1.2)
        self.done(reconstruction)

    def should(self, reconstruction):
        max_points = self.num_points_last * self.new_points_ratio
        max_shots = self.num_shots_last + self.interval
        return (len(reconstruction.points) >= max_points or
                len(reconstruction.shots) >= max_shots)

    def done(self, reconstruction):
        self.num_points_last = len(reconstruction.points)
        self.num_shots_last = len(reconstruction.shots)


class ShouldRetriangulate:
    """Helper to keep track of when to re-triangulate."""

    def __init__(self, data, reconstruction):
        self.active = data.config.get('retriangulation', False)
        self.ratio = data.config.get('retriangulation_ratio', 1.25)
        self.done(reconstruction)

    def should(self, reconstruction):
        max_points = self.num_points_last * self.ratio
        return self.active and len(reconstruction.points) > max_points

    def done(self, reconstruction):
        self.num_points_last = len(reconstruction.points)


def grow_reconstruction(data, graph, reconstruction, images, gcp):
    """Incrementally add shots to an initial reconstruction."""
    bundle(graph, reconstruction, None, data.config)
    align.align_reconstruction(reconstruction, gcp, data.config)

    should_bundle = ShouldBundle(data, reconstruction)
    should_retriangulate = ShouldRetriangulate(data, reconstruction)

    while True:
        if data.config.get('save_partial_reconstructions', False):
            paint_reconstruction(data, graph, reconstruction)
            data.save_reconstruction(
                [reconstruction], 'reconstruction.{}.json'.format(
                    datetime.datetime.now().isoformat().replace(':', '_')))

        common_tracks = reconstructed_points_for_images(graph, reconstruction,
                                                        images)
        if not common_tracks:
            break

        logger.info("-------------------------------------------------------")
        for image, num_tracks in common_tracks:
            if resect(data, graph, reconstruction, image):
                logger.info("Adding {0} to the reconstruction".format(image))
                images.remove(image)

                triangulate_shot_features(
                    graph, reconstruction, image,
                    data.config.get('triangulation_threshold', 0.004),
                    data.config.get('triangulation_min_ray_angle', 2.0))

                if should_bundle.should(reconstruction):
                    bundle(graph, reconstruction, None, data.config)
                    remove_outliers(graph, reconstruction, data.config)
                    align.align_reconstruction(reconstruction, gcp,
                                               data.config)
                    should_bundle.done(reconstruction)
                else:
                    if data.config['local_bundle_radius'] > 0:
                        bundle_local(graph, reconstruction, None, image, data.config)

                if should_retriangulate.should(reconstruction):
                    logger.info("Re-triangulating")
                    retriangulate(graph, reconstruction, data.config)
                    bundle(graph, reconstruction, None, data.config)
                    should_retriangulate.done(reconstruction)
                break
        else:
            logger.info("Some images can not be added")
            break

    logger.info("-------------------------------------------------------")

    bundle(graph, reconstruction, gcp, data.config)
    align.align_reconstruction(reconstruction, gcp, data.config)
    paint_reconstruction(data, graph, reconstruction)
    return reconstruction


def incremental_reconstruction(data):
    """Run the entire incremental reconstruction pipeline."""
    logger.info("Starting incremental reconstruction")
    if not data.reference_lla_exists():
        data.invent_reference_lla()

    graph = data.load_tracks_graph()
    tracks, images = matching.tracks_and_images(graph)
    remaining_images = set(images)
    gcp = None
    if data.ground_control_points_exist():
        gcp = data.load_ground_control_points()
    common_tracks = matching.all_common_tracks(graph, tracks)
    reconstructions = []
    pairs = compute_image_pairs(common_tracks, data.config)
    for im1, im2 in pairs:
        if im1 in remaining_images and im2 in remaining_images:
            tracks, p1, p2 = common_tracks[im1, im2]
            reconstruction = bootstrap_reconstruction(data, graph, im1, im2, p1, p2)
            if reconstruction:
                remaining_images.remove(im1)
                remaining_images.remove(im2)
                reconstruction = grow_reconstruction(
                    data, graph, reconstruction, remaining_images, gcp)
                reconstructions.append(reconstruction)
                reconstructions = sorted(reconstructions,
                                         key=lambda x: -len(x.shots))
                data.save_reconstruction(reconstructions)

    for k, r in enumerate(reconstructions):
        logger.info("Reconstruction {}: {} images, {} points".format(
            k, len(r.shots), len(r.points)))
    logger.info("{} partial reconstructions in total.".format(
        len(reconstructions)))
