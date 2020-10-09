import argparse
import json
import os

import numpy as np
import logging

import opensfm.reconstruction as orec
from opensfm import align
from opensfm import dataset
from opensfm import log
from opensfm import multiview
from opensfm import pygeometry
from opensfm import transformations as tf
from opensfm import types

logger = logging.getLogger(__name__)


def merge_reconstructions(reconstructions, tracks_manager):
    """Put all reconstruction points and shots in a single one.

    No alignment is performed.  Just merging the lists of points and
    shots as they are.

    Point(track) names have to be unique in the merged reconstruction.
    We add a prefix according to the source reconstruction index.
    """
    merged = types.Reconstruction()

    for ix_r, reconstruction in enumerate(reconstructions):
        for camera in reconstruction.cameras.values():
            merged.add_camera(camera)

        for point in reconstruction.points.values():
            new_point = merged.create_point(f"R{ix_r}_{point.id}", point.coordinates)
            new_point.color = point.color

        for shot in reconstruction.shots.values():
            merged.add_shot(shot)
            obsdict = tracks_manager.get_shot_observations(shot.id)
            for track_id, obs in obsdict.items():
                merged_track_id = f"R{ix_r}_{track_id}"
                if merged_track_id in merged.points:
                    merged.add_observation(shot.id, merged_track_id, obs)

    return merged


def resplit_reconstruction(merged, original_reconstructions):
    """Resplit reconstructions that were previously merged.

    The original_reconstructions are used to decide which point and shot
    goes on which of the split reconstructions.
    """
    split = []

    for ix_r, original in enumerate(original_reconstructions):
        r = types.Reconstruction()
        for shot_id in original.shots:
            r.add_shot(merged.shots[shot_id])
        for point_id in original.points:
            merged_point = merged.points[f"R{ix_r}_{point_id}"]
            new_point = r.create_point(point_id, merged_point.coordinates)
            new_point.color = merged_point.color
        for camera_id in original.cameras:
            r.add_camera(merged.cameras[camera_id])
        split.append(r)

    return split


def triangulate_gcps(gcps, reconstruction):
    coords = []
    for gcp in gcps:
        res = orec.triangulate_gcp(gcp, reconstruction.shots)
        coords.append(res)
    return coords


def reproject_gcps(gcps, reconstruction):
    output = {}
    for gcp in gcps:
        point = orec.triangulate_gcp(gcp, reconstruction.shots)
        if point is None:
            point = np.nan
        output[gcp.id] = {}
        for observation in gcp.observations:
            if observation.shot_id not in reconstruction.shots:
                continue
            shot = reconstruction.shots[observation.shot_id]
            reproj = shot.project(point)
            error = np.linalg.norm(reproj - observation.projection)
            output[gcp.id][observation.shot_id] = {'error': error,
                                                   'reprojection': [reproj[0], reproj[1]]}
    return output


def get_all_reprojection_errors(gcp_reprojections):
    output = []
    for gcp_id in gcp_reprojections:
        for shot_id in gcp_reprojections[gcp_id]:
            e = gcp_reprojections[gcp_id][shot_id]['error']
            output.append((gcp_id, shot_id, e))
    return sorted(output, key=lambda t:-t[2])


def get_number_of_wrong_annotations_per_gcp(gcp_reprojections, wrong_threshold):
    output = {}
    for gcp_id, reprojections in gcp_reprojections.items():
        errors = [reprojections[shot_id]['error'] for shot_id in reprojections]
        output[gcp_id] = sum(e>wrong_threshold for e in errors)
    return output


def compute_gcp_std(gcp_errors):
    """Compute the standard deviation of reprojection errors of all gcps."""
    all_errors = []
    for gcp_id in gcp_errors:
        errors = [e['error'] for e in gcp_errors[gcp_id].values()]
        logger.info(f"gcp {gcp_id} mean reprojection error = {np.mean(errors)}")
        all_errors.extend(errors)
    return np.sqrt(np.mean(np.array(all_errors)**2))


def find_alignment(points0, points1):
    """Compute similarity transform between point sets.

    Returns (s, A, b) such that ``points0 = s * A * points1 + b``
    """
    v0, v1 = [], []
    for p0, p1 in zip(points0, points1):
        if p0 is not None and p1 is not None:
            v0.append(p0)
            v1.append(p1)
    v0 = np.array(v0).T
    v1 = np.array(v1).T
    M = tf.affine_matrix_from_points(v0, v1, shear=False)
    s = np.linalg.det(M[:3, :3])**(1. / 3.)
    A = M[:3, :3] / s
    b = M[:3, 3]
    return s, A, b


def add_gcp_to_bundle(ba, gcp, gcp_std, shots):
    """Add Ground Control Points constraints to the bundle problem."""
    for point in gcp:
        point_id = 'gcp-' + point.id

        coordinates = orec.triangulate_gcp(point, shots)
        if coordinates is None:
            if point.coordinates is not None:
                coordinates = point.coordinates
            else:
                logger.warning("Cannot initialize GCP '{}'."
                               "  Ignoring it".format(point.id))
                continue

        ba.add_point(point_id, coordinates, False)

        for observation in point.observations:
            if observation.shot_id in shots:
                ba.add_point_projection_observation(
                    observation.shot_id,
                    point_id,
                    observation.projection[0],
                    observation.projection[1],
                    gcp_std)


def bundle_with_fixed_images(reconstruction, camera_priors, gcp, gcp_std,
                             fixed_images, config):
    """Bundle adjust a reconstruction while keeping some images fixed."""
    fix_cameras = True

    chrono = orec.Chronometer()
    ba = orec.pybundle.BundleAdjuster()

    for camera in reconstruction.cameras.values():
        camera_prior = camera_priors[camera.id]
        ba.add_camera(camera.id, camera, camera_prior, fix_cameras)

    for shot in reconstruction.shots.values():
        r = shot.pose.rotation
        t = shot.pose.translation
        ba.add_shot(shot.id, shot.camera.id, r, t, shot.id in fixed_images)

    for point in reconstruction.points.values():
        ba.add_point(point.id, point.coordinates, False)
        x, y, z = point.coordinates
        ba.add_point_position_prior(point.id, x, y, z, 100.0)

    for shot_id in reconstruction.shots:
        shot = reconstruction.get_shot(shot_id)
        for point in shot.get_valid_landmarks():
            obs = shot.get_landmark_observation(point)
            ba.add_point_projection_observation(
                shot.id, point.id, obs.point[0], obs.point[1], obs.scale)

    add_gcp_to_bundle(ba, gcp, gcp_std, reconstruction.shots)

    ba.set_point_projection_loss_function(config['loss_function'],
                                          config['loss_function_threshold'])
    ba.set_internal_parameters_prior_sd(
        config['exif_focal_sd'],
        config['principal_point_sd'],
        config['radial_distortion_k1_sd'],
        config['radial_distortion_k2_sd'],
        config['tangential_distortion_p1_sd'],
        config['tangential_distortion_p2_sd'],
        config['radial_distortion_k3_sd'],
        config['radial_distortion_k4_sd'])
    ba.set_num_threads(config['processes'])
    ba.set_max_num_iterations(config['bundle_max_iterations'])
    ba.set_linear_solver_type("SPARSE_SCHUR")
    ba.set_compute_covariances(True)

    chrono.lap('setup')
    ba.run()
    chrono.lap('run')

    if not ba.get_covariance_estimation_valid():
        logger.warning('Could not compute covariance')

    for camera in reconstruction.cameras.values():
        orec._get_camera_from_bundle(ba, camera)

    for shot in reconstruction.shots.values():
        s = ba.get_shot(shot.id)
        shot.pose.rotation = [s.r[0], s.r[1], s.r[2]]
        shot.pose.translation = [s.t[0], s.t[1], s.t[2]]
        shot.covariance = np.array([[s.get_covariance_inv_param(i, j)
                                     for j in range(6)] for i in range(6)])

    for point in reconstruction.points.values():
        p = ba.get_point(point.id)
        point.coordinates = [p.p[0], p.p[1], p.p[2]]
        point.reprojection_errors = p.reprojection_errors

    chrono.lap('teardown')

    logger.info(ba.full_report())
    logger.info(ba.brief_report())
    report = {
        'wall_times': dict(chrono.lap_times()),
        'brief_report': ba.brief_report(),
    }
    return report


def decompose_covariance(covariance):
    """Decompose covariance into a rotation and diagonal scaling."""
    u, s, vh = np.linalg.svd(covariance)
    return u, np.sqrt(s)


def resect_annotated_single_images(reconstruction, gcps, camera_models, data):
    """Resect images that do not belong to reconstruction but have enough GCPs annotated.

    Returns:
        A reconstruction with all the resected images.
    """
    not_in_rec = set()
    for gcp in gcps:
        for obs in gcp.observations:
            im = obs.shot_id
            if im not in reconstruction.shots:
                not_in_rec.add(im)

    resected = types.Reconstruction()
    for im in not_in_rec:
        exif = data.load_exif(im)
        camera = camera_models[exif["camera"]]
        resect_image(im, camera, gcps, reconstruction, data, resected)

    print(f"Resected: {len(resected.shots)} shots and {len(resected.cameras)} cameras")
    return resected


def _gcp_image_observation(gcp, image):
    for obs in gcp.observations:
        if image == obs.shot_id:
            return obs
    return None


def resect_image(im, camera, gcps, reconstruction, data, dst_reconstruction=None):
    """
    Resect an image into a reconstruction based only on GCPs annotations.
    Pass another reconstruction to dst_reconstruction
    if you want the resected points to be added there instead

    Returns:
        The resected shot.
    """
    threshold = 0.01
    min_inliers = 3

    bs, Xs = [], []
    for gcp in gcps:
        obs = _gcp_image_observation(gcp, im)
        if not obs:
            continue
        gcp_3d_coords = orec.triangulate_gcp(gcp, reconstruction.shots)
        if gcp_3d_coords is None:
            continue

        b = camera.pixel_bearing(obs.projection)
        bs.append(b)
        Xs.append(gcp_3d_coords)
    bs = np.array(bs)
    Xs = np.array(Xs)

    if len(bs) < min_inliers:
        print(f"Not enough annotations to resect image {im}")
        return None

    T = multiview.absolute_pose_ransac(bs, Xs, threshold, 1000, 0.999)
    R = T[:, :3]
    t = T[:, 3]

    reprojected_bs = R.T.dot((Xs - t).T).T
    reprojected_bs /= np.linalg.norm(reprojected_bs, axis=1)[:, np.newaxis]

    inliers = np.linalg.norm(reprojected_bs - bs, axis=1) < threshold
    ninliers = int(sum(inliers))

    logger.info(f"{im} resection inliers: {ninliers} / {len(bs)}")

    if dst_reconstruction is None:
        dst_reconstruction = reconstruction

    if ninliers >= min_inliers:
        R = T[:, :3].T
        t = -R.dot(T[:, 3])
        dst_reconstruction.add_camera(camera)
        shot = dst_reconstruction.create_shot(im, camera.id, pygeometry.Pose(R,t))
        shot.metadata = orec.get_image_metadata(data, im)
        return shot
    else:
        print(f"Not enough inliers to resect image {im}")
        return None


def parse_args():
    parser = argparse.ArgumentParser(description='Merge reconstructions and run BA with GCPs')
    parser.add_argument(
        'dataset',
        help='dataset to process',
    )
    parser.add_argument(
        '--mode',
        default="3d_to_3d",
        help="If '3d_to_2d': the largest reconstruction is used as a fixed reference: "
             "Images not belonging to it are resected into it using the ground control points. "
             "If '3d_to_3d': The two largest reconstructions are aligned to each other using the ground control points."
    )
    parser.add_argument(
        '--std-threshold',
        default=0.3,
        help='positional threshold (m) above which we consider an image to be badly localized',
    )
    parser.add_argument(
        '--px-threshold',
        default=0.016, # a little bit over 10 pixels at VGA resolution
        help='threshold in normalized pixels above which we consider a GCP annotation to be wrong',
    )
    args = parser.parse_args()
    assert args.mode in ("3d_to_3d", "3d_to_2d")
    return args


def main():
    args = parse_args()
    path = args.dataset
    data = dataset.DataSet(path)
    for fn in (
        'reconstruction.json',
        'ground_control_points.json',
        'tracks.csv'
    ):
        if not (os.path.exists(os.path.join(path, fn))):
            logger.error(f"Missing file: {fn}")
            return

    camera_models = data.load_camera_models()
    tracks_manager = data.load_tracks_manager()

    all_reconstructions = data.load_reconstruction()

    if len(all_reconstructions) > 2:
        logger.warning(f"There are more than two reconstructions in {path}")

    reconstructions = all_reconstructions[:2]
    gcps = data.load_ground_control_points()

    if args.mode == "3d_to_3d":
        coords0 = triangulate_gcps(gcps, reconstructions[0])
        coords1 = triangulate_gcps(gcps, reconstructions[1])
        s, A, b = find_alignment(coords1, coords0)
        align.apply_similarity(reconstructions[1], s, A, b)
    else:
        base = reconstructions[0]
        resected = resect_annotated_single_images(base, gcps, camera_models, data)
        for shot in resected.shots.values():
            shot.metadata.gps_accuracy.value = 1E12
            shot.metadata.gps_position.value = shot.pose.get_origin()
        reconstructions = [base, resected]
    data.save_reconstruction(reconstructions, 'reconstruction_gcp_rigid.json')

    merged = merge_reconstructions(reconstructions, tracks_manager)
    data.save_reconstruction([merged], 'reconstruction_merged.json')

    data.config['bundle_max_iterations'] = 200
    data.config['bundle_use_gcp'] = True
    orec.bundle(merged, camera_models, gcp=gcps, config=data.config)
    # rigid rotation to put images on the ground
    orec.align_reconstruction(merged, None, data.config)
    data.save_reconstruction([merged], 'reconstruction_gcp_ba.json')

    gcp_reprojections = reproject_gcps(gcps, merged)
    reprojection_errors = get_all_reprojection_errors(gcp_reprojections)
    err_values = [t[2] for t in reprojection_errors]
    mean_reprojection_error = np.mean(err_values)
    max_reprojection_error = np.max(err_values)
    median_reprojection_error = np.median(err_values)

    with open(data.data_path + '/gcp_reprojections.json', 'w') as f:
        json.dump(gcp_reprojections, f, indent=4, sort_keys=True)

    gcp_std = compute_gcp_std(gcp_reprojections)
    logger.info(f"GCP reprojection error STD: {gcp_std}")

    resplit = resplit_reconstruction(merged, reconstructions)
    data.save_reconstruction(resplit, 'reconstruction_gcp_ba_resplit.json')

    all_shots_std = []
    # We run bundle by fixing one reconstruction.
    # If we have two reconstructions, we do this twice, fixing each one.
    _rec_ixs = [(0,1), (1,0)] if args.mode == "3d_to_3d" else [(0,1)]
    for rec_ixs in _rec_ixs:
        fixed_images = set(reconstructions[rec_ixs[0]].shots.keys())
        bundle_with_fixed_images(merged, camera_models, gcp=gcps, gcp_std=gcp_std,
                                 fixed_images=fixed_images, config=data.config)

        logger.info(f"STD in the position of shots in rec {rec_ixs[1]} w.r.t rec {rec_ixs[0]}")
        for shot in merged.shots.values():
            if shot.id in reconstructions[rec_ixs[1]].shots:
                u, std = decompose_covariance(shot.covariance[3:, 3:])
                all_shots_std.append((shot.id, np.linalg.norm(std)))
                logger.info(f"{shot.id} position std: {np.linalg.norm(std)}")

    # If the STD of all shots is the same, replace by nan
    std_values = [x[1] for x in all_shots_std]
    n_bad_std = sum(std > args.std_threshold for std in std_values)
    if np.allclose(std_values, std_values[0]):
        all_shots_std = [(x[0], np.nan) for x in all_shots_std]
        n_bad_std = len(std_values)

    # Average positional STD
    average_shot_std = np.mean([t[1] for t in all_shots_std])
    median_shot_std = np.median([t[1] for t in all_shots_std])

    # Save the shot STD to a file
    with open(data.data_path + '/shots_std.csv', 'w') as f:
        s = sorted(all_shots_std, key=lambda t:-t[-1])
        for t in s:
            line = "{}, {}".format(*t)
            f.write(line + '\n')


    n_bad_gcp_annotations = int(sum(t[2] > args.px_threshold for t in reprojection_errors))
    metrics = {
        'n_reconstructions': len(all_reconstructions),
        'median_shot_std' : median_shot_std,
        'max_shot_std': s[0][1],
        'max_reprojection_error': max_reprojection_error,
        'median_reprojection_error': median_reprojection_error,
        'n_gcp': len(gcps),
        'n_bad_gcp_annotations': n_bad_gcp_annotations,
        'n_bad_position_std': int(n_bad_std),
    }

    logger.info(metrics)
    p_metrics = data.data_path + '/run_ba_metrics.json'
    logger.info(f"Saved metrics to {p_metrics}")
    with open(p_metrics, 'w') as f:
        json.dump(metrics, f, indent=4, sort_keys=True)

    logger.info("========================================")
    logger.info("=============== Summary ================")
    logger.info("========================================")
    if n_bad_std == 0 and n_bad_gcp_annotations == 0:
        logger.info(
            f"No issues. All gcp reprojections are under {args.px_threshold}"
            f" and all frames are localized within {args.std_threshold}m"
        )
    else:
        if np.isnan(all_shots_std[0][1]):
            logger.info(
                f"Could not get positional uncertainty. It could be because: a) there are not enough GCPs."
                " b) they are badly distributed in 3D. c) there are some wrong annotations"
            )
        else:
            logger.info(f"{n_bad_std} badly localized images (error>{args.std_threshold}). Use the frame list on each view to find these")
        logger.info(f"{n_bad_gcp_annotations} annotations with large reprojection error. Press Q to jump to the worst, correct it and repeat. Here are the top 5:")

        stats_bad_reprojections = get_number_of_wrong_annotations_per_gcp(gcp_reprojections, args.px_threshold)
        gcps_sorted = sorted(stats_bad_reprojections, key=lambda k:-stats_bad_reprojections[k])[:5]
        for ix, gcp_id in enumerate(gcps_sorted):
            logger.info(f"#{ix+1}: GCP: {gcp_id} with {stats_bad_reprojections[gcp_id]} bad annotations")


if __name__ == "__main__":
    log.setup()
    exit(main())
