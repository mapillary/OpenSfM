import argparse
import json

import numpy as np
import logging

import opensfm.reconstruction as orec
from opensfm import align
from opensfm import dataset
from opensfm import log
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
            merged.create_shot(shot.id, shot.camera.id, shot.pose)
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
            shot = reconstruction.shots[observation.shot_id]
            reproj = shot.project(point)
            error = np.linalg.norm(reproj - observation.projection)
            output[gcp.id][observation.shot_id] = {'error': error,
                                                   'reprojection': [reproj[0], reproj[1]]}
    return output


def get_mean_max_reprojection_errors(gcp_reprojections):
    max_gcp_reprojection_error = 0
    mean_gcp_reprojection_error = []
    for gcp_id in gcp_reprojections:
        for shot_id in gcp_reprojections[gcp_id]:
            e = gcp_reprojections[gcp_id][shot_id]['error']
            mean_gcp_reprojection_error.append(e)
            if e > max_gcp_reprojection_error:
                max_gcp_reprojection_error = e
    mean_gcp_reprojection_error = np.mean(mean_gcp_reprojection_error)
    median_gcp_reprojection_error = np.median(mean_gcp_reprojection_error)
    return mean_gcp_reprojection_error, max_gcp_reprojection_error, median_gcp_reprojection_error


def compute_gcp_std(gcp_errors):
    """Compute the standard deviation of reprojection errors of all gcps."""
    all_errors = []
    for gcp_id in gcp_errors:
        errors = [e['error'] for e in gcp_errors[gcp_id].values()]
        print(f"gcp {gcp_id} mean reprojection error = {np.mean(errors)}")
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
    fix_cameras = False

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

    print(ba.full_report())
    print(ba.brief_report())
    report = {
        'wall_times': dict(chrono.lap_times()),
        'brief_report': ba.brief_report(),
    }
    return report


def decompose_covariance(covariance):
    """Decompose covariance into a rotation and diagonal scaling."""
    u, s, vh = np.linalg.svd(covariance)
    return u, np.sqrt(s)


def parse_args():
    parser = argparse.ArgumentParser(description='Merge reconstructions and run BA with GCPs')
    parser.add_argument(
        'dataset',
        help='dataset to process',
    )
    return parser.parse_args()


def main():
    args = parse_args()
    path = args.dataset
    data = dataset.DataSet(path)

    camera_models = data.load_camera_models()
    tracks_manager = data.load_tracks_manager()

    reconstructions = [data.load_reconstruction()[index] for index in (0, 1)]
    gcps = data.load_ground_control_points()

    coords0 = triangulate_gcps(gcps, reconstructions[0])
    coords1 = triangulate_gcps(gcps, reconstructions[1])

    s, A, b = find_alignment(coords1, coords0)
    align.apply_similarity(reconstructions[1], s, A, b)

    data.save_reconstruction(reconstructions, 'reconstruction_gcp_rigid.json')

    merged = merge_reconstructions(reconstructions, tracks_manager)
    data.save_reconstruction([merged], 'reconstruction_merged.json')

    data.config['bundle_max_iterations'] = 200
    orec.bundle(merged, camera_models, gcp=gcps, config=data.config)
    # rigid rotation to put images on the ground
    orec.align_reconstruction(merged, None, data.config)
    data.save_reconstruction([merged], 'reconstruction_gcp_ba.json')

    gcp_reprojections = reproject_gcps(gcps, merged)
    mean_reprojection_error, max_reprojection_error, median_reprojection_error = get_mean_max_reprojection_errors(gcp_reprojections)
    with open(data.data_path + '/gcp_reprojections.json', 'w') as f:
        json.dump(gcp_reprojections, f, indent=4, sort_keys=True)

    gcp_std = compute_gcp_std(gcp_reprojections)
    print("GCP reprojection error STD:", gcp_std)

    resplit = resplit_reconstruction(merged, reconstructions)
    data.save_reconstruction(resplit, 'reconstruction_gcp_ba_resplit.json')

    all_shots_std = []
    for rec_ixs in ((0,1), (1,0)):
        fixed_images = set(reconstructions[rec_ixs[0]].shots.keys())
        bundle_with_fixed_images(merged, camera_models, gcp=gcps, gcp_std=gcp_std,
                                 fixed_images=fixed_images, config=data.config)

        print("STD in the position of shots in rec {} w.r.t rec {}".format(rec_ixs[1], rec_ixs[0]))
        for shot in merged.shots.values():
            if shot.id in reconstructions[rec_ixs[1]].shots:
                u, std = decompose_covariance(shot.covariance[3:, 3:])
                all_shots_std.append((shot.id, np.linalg.norm(std)))
                print(shot.id, 'position std:', np.linalg.norm(std))
    # Average positional STD
    average_shot_std = np.mean([t[1] for t in all_shots_std])
    median_shot_std = np.median([t[1] for t in all_shots_std])

    # Save the shot STD to a file
    with open(data.data_path + '/shots_std.csv', 'w') as f:
        s = sorted(all_shots_std, key=lambda t:-t[-1])
        for t in s:
            line = "{}, {}".format(*t)
            f.write(line + '\n')

    print("=============== Key metrics ================")

    print("Position STD")
    print(f"   max: {s[0][1]} ({s[0][0]})")
    #print(f"  mean: {average_shot_std}")
    print(f"median: {median_shot_std}")

    print("Reprojection errors [in px / max(w,h)]")
    print(f"   max: {max_reprojection_error}")
    #print(f"  mean: {mean_reprojection_error}")
    print(f"median: {median_reprojection_error}")


if __name__ == "__main__":
    log.setup()
    exit(main())
