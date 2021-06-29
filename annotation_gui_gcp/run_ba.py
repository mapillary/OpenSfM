import argparse
import json
import logging
import os
import sys
from collections import defaultdict

import numpy as np
import opensfm.reconstruction as orec
from opensfm import dataset, log, multiview, pygeometry
from opensfm import transformations as tf
from opensfm import types
from opensfm.align import apply_similarity

logger = logging.getLogger(__name__)


def merge_reconstructions(reconstructions, tracks_manager):
    """Put all reconstruction points and shots in a single one.

    No alignment is performed.  Just merging the lists of points and
    shots as they are.

    Point(track) names have to be unique in the merged reconstruction.
    We add a prefix according to the source reconstruction index.
    """
    merged = types.Reconstruction()
    merged.set_reference(reconstructions[0].reference)

    for ix_r, reconstruction in enumerate(reconstructions):
        assert merged.reference == reconstruction.reference

        for camera in reconstruction.cameras.values():
            merged.add_camera(camera)

        for point in reconstruction.points.values():
            new_point = merged.create_point(f"R{ix_r}_{point.id}", point.coordinates)
            new_point.color = point.color

        for shot in reconstruction.shots.values():
            merged.add_shot(shot)
            try:
                obsdict = tracks_manager.get_shot_observations(shot.id)
            except RuntimeError:
                logger.warning(f"Shot id {shot.id} missing from tracks_manager!")
                continue
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
        r.set_reference(merged.reference)
        for shot_id in original.shots:
            r.add_shot(merged.shots[shot_id])
        for point_id in original.points:
            merged_id = f"R{ix_r}_{point_id}"
            if merged_id not in merged.points:
                continue
            merged_point = merged.points[merged_id]
            new_point = r.create_point(point_id, merged_point.coordinates)
            new_point.color = merged_point.color
        for camera_id in original.cameras:
            r.add_camera(merged.cameras[camera_id])
        split.append(r)

    return split


def gcp_geopositional_error(gcps, reconstruction):
    coords_reconstruction = triangulate_gcps(gcps, reconstruction)
    out = {}
    for ix, gcp in enumerate(gcps):
        expected = gcp.coordinates.value if gcp.coordinates.has_value else None
        triangulated = (
            coords_reconstruction[ix] if coords_reconstruction[ix] is not None else None
        )

        if expected is not None and triangulated is not None:
            error = np.linalg.norm(expected - triangulated)
            out[gcp.id] = {
                "expected_xyz": list(expected),
                "triangulated_xyz": list(triangulated),
                "expected_lla": reconstruction.reference.to_lla(*expected),
                "triangulated_lla": reconstruction.reference.to_lla(*triangulated),
                "error": float(error),
            }

            # Compute the metric error, ignoring altitude
            lat, lon, _alt = out[gcp.id]["expected_lla"]
            expected_xy = reconstruction.reference.to_topocentric(lat, lon, 0)
            lat, lon, _alt = out[gcp.id]["triangulated_lla"]
            triangulated_xy = reconstruction.reference.to_topocentric(lat, lon, 0)
            out[gcp.id]["error_planar"] = np.linalg.norm(
                np.array(expected_xy) - np.array(triangulated_xy)
            )
        else:
            out[gcp.id] = {"error": np.nan, "error_planar": np.nan}

    return out


def triangulate_gcps(gcps, reconstruction):
    coords = []
    for gcp in gcps:
        res = multiview.triangulate_gcp(
            gcp,
            reconstruction.shots,
            reproj_threshold=1,
            min_ray_angle_degrees=0.1,
        )
        coords.append(res)
    return coords


def reproject_gcps(gcps, reconstruction, reproj_threshold):
    output = {}
    for gcp in gcps:
        point = multiview.triangulate_gcp(
            gcp,
            reconstruction.shots,
            reproj_threshold=reproj_threshold,
            min_ray_angle_degrees=0.1,
        )
        output[gcp.id] = {}
        n_obs = len(gcp.observations)
        if point is None:
            logger.info(f"Could not triangulate {gcp.id} with {n_obs} annotations")
            continue
        for observation in gcp.observations:
            lat, lon, alt = reconstruction.reference.to_lla(*point)
            output[gcp.id][observation.shot_id] = {"lla": [lat, lon, alt], "error": 0}
            if observation.shot_id not in reconstruction.shots:
                continue
            shot = reconstruction.shots[observation.shot_id]
            reproj = shot.project(point)
            error = np.linalg.norm(reproj - observation.projection)
            output[gcp.id][observation.shot_id].update(
                {
                    "error": error,
                    "reprojection": [reproj[0], reproj[1]],
                }
            )
    return output


def get_sorted_reprojection_errors(gcp_reprojections):
    output = []
    for gcp_id in gcp_reprojections:
        for shot_id in gcp_reprojections[gcp_id]:
            e = gcp_reprojections[gcp_id][shot_id]["error"]
            output.append((gcp_id, shot_id, e))
    return sorted(output, key=lambda t: -t[2])


def get_number_of_wrong_annotations_per_gcp(gcp_reprojections, wrong_threshold):
    output = {}
    for gcp_id, reprojections in gcp_reprojections.items():
        errors = [reprojections[shot_id]["error"] for shot_id in reprojections]
        output[gcp_id] = sum(e > wrong_threshold for e in errors)
    return output


def compute_gcp_std(gcp_errors):
    """Compute the standard deviation of reprojection errors of all gcps."""
    all_errors = []
    for gcp_id in gcp_errors:
        errors = [e["error"] for e in gcp_errors[gcp_id].values()]
        logger.info(f"gcp {gcp_id} mean reprojection error = {np.mean(errors)}")
        all_errors.extend(errors)

    all_errors = [e for e in all_errors if not np.isnan(e)]
    return np.sqrt(np.mean(np.array(all_errors) ** 2))


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
    s = np.linalg.det(M[:3, :3]) ** (1.0 / 3.0)
    A = M[:3, :3] / s
    b = M[:3, 3]
    return s, A, b


def add_gcp_to_bundle(ba, gcp, gcp_std, shots):
    """Add Ground Control Points constraints to the bundle problem."""
    for point in gcp:
        point_id = "gcp-" + point.id

        coordinates = multiview.triangulate_gcp(
            point,
            shots,
            reproj_threshold=1,
            min_ray_angle_degrees=0.1,
        )
        if coordinates is None:
            if point.coordinates.has_value:
                logger.warning(
                    f"Could not triangulate GCP '{point.id}'."
                    f"Using {point.coordinates.value} (derived from lat,lon)"
                )
                coordinates = point.coordinates.value
            else:
                logger.warning(
                    "Cannot initialize GCP '{}'." "  Ignoring it".format(point.id)
                )
                continue

        ba.add_point(point_id, coordinates, False)

        for observation in point.observations:
            if observation.shot_id in shots:
                ba.add_point_projection_observation(
                    observation.shot_id,
                    point_id,
                    observation.projection,
                    gcp_std,
                )


def bundle_with_fixed_images(
    reconstruction, camera_priors, gcp, gcp_std, fixed_images, config
):
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
            ba.add_point_projection_observation(shot.id, point.id, obs.point, obs.scale)

    add_gcp_to_bundle(ba, gcp, gcp_std, reconstruction.shots)

    ba.set_point_projection_loss_function(
        config["loss_function"], config["loss_function_threshold"]
    )
    ba.set_internal_parameters_prior_sd(
        config["exif_focal_sd"],
        config["principal_point_sd"],
        config["radial_distortion_k1_sd"],
        config["radial_distortion_k2_sd"],
        config["tangential_distortion_p1_sd"],
        config["tangential_distortion_p2_sd"],
        config["radial_distortion_k3_sd"],
        config["radial_distortion_k4_sd"],
    )
    ba.set_num_threads(config["processes"])
    ba.set_max_num_iterations(config["bundle_max_iterations"])
    ba.set_linear_solver_type("SPARSE_SCHUR")
    ba.set_compute_covariances(True)

    chrono.lap("setup")
    ba.run()
    chrono.lap("run")

    cov_valid = ba.get_covariance_estimation_valid()
    if not cov_valid:
        logger.warning("Could not compute covariance")

    for camera in reconstruction.cameras.values():
        orec._get_camera_from_bundle(ba, camera)

    for shot in reconstruction.shots.values():
        s = ba.get_shot(shot.id)
        shot.pose.rotation = [s.r[0], s.r[1], s.r[2]]
        shot.pose.translation = [s.t[0], s.t[1], s.t[2]]
        shot.covariance = s.get_covariance_inv_param()

    for point in reconstruction.points.values():
        p = ba.get_point(point.id)
        point.coordinates = [p.p[0], p.p[1], p.p[2]]
        point.reprojection_errors = p.reprojection_errors

    chrono.lap("teardown")

    logger.info(ba.full_report())
    logger.info(ba.brief_report())
    return cov_valid


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
            if im not in reconstruction.shots and im in data.images():
                not_in_rec.add(im)

    resected = types.Reconstruction()
    resected.reference = reconstruction.reference
    for im in not_in_rec:
        exif = data.load_exif(im)
        camera = camera_models[exif["camera"]]
        resect_image(im, camera, gcps, reconstruction, data, resected)

    logger.info(
        f"Resected: {len(resected.shots)} shots and {len(resected.cameras)} cameras"
    )
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
        gcp_3d_coords = multiview.triangulate_gcp(
            gcp,
            reconstruction.shots,
            reproj_threshold=1,
            min_ray_angle_degrees=0.1,
        )
        if gcp_3d_coords is None:
            continue

        b = camera.pixel_bearing(obs.projection)
        bs.append(b)
        Xs.append(gcp_3d_coords)
    bs = np.array(bs)
    Xs = np.array(Xs)

    if len(bs) < min_inliers:
        logger.info(f"Not enough annotations to resect image {im}")
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
        shot = dst_reconstruction.create_shot(im, camera.id, pygeometry.Pose(R, t))
        shot.metadata = orec.get_image_metadata(data, im)
        return shot
    else:
        logger.info(f"Not enough inliers to resect image {im}")
        return None


def parse_args():
    parser = argparse.ArgumentParser(
        description="Merge reconstructions and run BA with GCPs"
    )
    parser.add_argument(
        "path_dataset",
        help="dataset to process",
    )
    parser.add_argument(
        "--rec_a",
        default=0,
        type=int,
        help="Index of reconstruction A."
        "\nIf rec_b is set to None, the rec_a is used as a fixed reference:"
        " all annotated images not belonging to it are resected into it using the GCPs."
        "\nIf reconstruction B is set to a number, the pair of reconstructions (A,B)"
        " will be aligned to each other using the ground control points.",
    )
    parser.add_argument(
        "--rec_b",
        default=1,
        type=int,
        help="Index of reconstruction B. Read the help for rec_a",
    )
    parser.add_argument(
        "--std-threshold",
        default=0.3,
        help="Positional threshold (m) to classify images as well-localized",
    )
    parser.add_argument(
        "--px-threshold",
        default=0.016,  # a little bit over 10 pixels at VGA resolution
        help="threshold in normalized pixels to classify a GCP annotation as correct",
        type=float,
    )
    parser.add_argument(
        "--rigid",
        action="store_true",
        help="Skip BA entirely",
    )
    parser.add_argument(
        "--covariance",
        action="store_true",
        help="Run BA with fixed images to obtain pose covariances",
    )
    args = parser.parse_args()

    if args.covariance:
        assert not args.rigid, "rigid and covariance are mutually exclusive"

    return args


def align_3d_annotations_to_reconstruction(
    gcps, gcps_3d, model_id, reconstruction, gcps_this_model
):
    logger.info(f"{model_id} has {len(gcps)} gcps")
    if len(gcps) < 3:
        logger.info(f"{model_id} has {len(gcps)} gcps, not aligning")
        return

    coords_triangulated_gcps = triangulate_gcps(gcps_this_model, reconstruction)
    n_triangulated = sum(x is not None for x in coords_triangulated_gcps)
    if n_triangulated < 3:
        logger.info(f"{model_id} has {n_triangulated} gcps, not aligning")
        return

    coords_annotated_gcps = []
    for gcp in gcps:
        # Find the corresponding 3D-annotated gcp coordinates, add in the same order
        for annotated_point_3D, gcp_id in gcps_3d[model_id]:
            if gcp_id == gcp.id:
                coords_annotated_gcps.append(annotated_point_3D)
                break

    # Move / "reproject" the triangulated GCPs to the reference frame of the CAD model for display/interaction
    gcp_reprojections = {}
    try:
        s, A, b = find_alignment(coords_triangulated_gcps, coords_annotated_gcps)
        for gcp, coords in zip(gcps, coords_triangulated_gcps):
            gcp_reprojections[gcp.id] = (
                (s * A.dot(coords) + b).tolist() if coords else None
            )
    except ValueError:
        logger.warning(f"Could not align reconstruction with {model_id}")

    return gcp_reprojections, {"s": s.tolist(), "A": A.tolist(), "b": b.tolist()}


def align_external_3d_models_to_reconstruction(data, gcps, reconstruction, ix_rec):
    """
    Triangulates all the control points in common with 3D models (those that have at least one 3D annotation)
    and saves them in the coordinate space of the 3D models so that they can be visualized in the annotation tool.

    gcps only contains the 2D observations
    We re-load the gcps file here manually to also get the 3D observations (hacky but will do for now)
    """

    # Manually load GCP file to get the 3D annotations
    # Dict[model_filename, List[Tuple[3d_point, gcp_id]]

    # Create a mapping from filename to gcp ids (for those gcps that have 3D annotations)
    map_filename_to_gcp_ids = defaultdict(set)
    gcps_3d = load_3d_annotations_from_gcp_file(data)
    for shot_id, observations in gcps_3d.items():
        for _coords, gcp_id in observations:
            map_filename_to_gcp_ids[shot_id].add(gcp_id)

    gcps_per_source_file = defaultdict(list)
    # Group the 2D annotations by the 3D model that they're also annotated in
    for gcp in gcps:
        for (
            shot_id,
            gcp_ids_annotated_in_3d_for_this_file,
        ) in map_filename_to_gcp_ids.items():
            if gcp.id in gcp_ids_annotated_in_3d_for_this_file:
                gcps_per_source_file[shot_id].append(gcp)

    # We now operate independently on each 3D model
    for model_id, gcps_this_model in gcps_per_source_file.items():
        gcp_reprojections, alignment = align_3d_annotations_to_reconstruction(
            gcps, gcps_3d, model_id, reconstruction, gcps_this_model
        )
        p_out = f"{data.data_path}/gcp_reprojections_3D_{ix_rec}x{model_id}.json"
        logger.info(f"Saving reprojected 3D points to {p_out}")
        with open(p_out, "w") as f:
            json.dump(gcp_reprojections, f, indent=4, sort_keys=True)
        p_out = f"{data.data_path}/affine_alignment_{ix_rec}x{model_id}.json"
        logger.info(f"Saving affine parameters to {p_out}")
        with open(p_out, "w") as f:
            json.dump(alignment, f, indent=4, sort_keys=True)


def load_3d_annotations_from_gcp_file(data):
    with open(data._ground_control_points_file()) as f:
        raw_gcps = json.load(f)
    dict_gcps = defaultdict(list)
    for gcp in raw_gcps["points"]:
        for obs in gcp["observations"]:
            if "point" in obs:
                dict_gcps[obs["shot_id"]].append((obs["point"], gcp["id"]))
    return dict_gcps


def fix_3d_annotations_in_gcp_file(data):
    # Temporary hack: correct the GCP file so that 3D observations are called 'point' and not 'projection'
    with open(data._ground_control_points_file()) as f:
        raw_gcps = json.load(f)
    for gcp in raw_gcps["points"]:
        for obs in gcp["observations"]:
            if "projection" not in obs:
                continue
            if len(obs["projection"]) == 3:
                obs["point"] = obs["projection"]
                del obs["projection"]

    with open(data._ground_control_points_file(), "w") as f:
        json.dump(raw_gcps, f, indent=4, sort_keys=True)


def align(
    path: str,
    rec_a: int,
    rec_b: int,
    rigid: bool,
    covariance: bool,
    px_threshold: float,
    std_threshold: float,
):
    data = dataset.DataSet(path)
    for fn in ("reconstruction.json", "ground_control_points.json", "tracks.csv"):
        if not (os.path.exists(os.path.join(path, fn))):
            logger.error(f"Missing file: {fn}")
            return

    camera_models = data.load_camera_models()
    tracks_manager = data.load_tracks_manager()
    fix_3d_annotations_in_gcp_file(data)
    gcps = data.load_ground_control_points()

    fn_resplit = f"reconstruction_gcp_ba_resplit_{rec_a}x{rec_b}.json"
    fn_rigid = f"reconstruction_gcp_rigid_{rec_a}x{rec_b}.json"

    reconstructions = data.load_reconstruction()
    if len(reconstructions) > 1:
        if rec_b is not None:  # reconstruction - to - reconstruction alignment
            if rigid and os.path.exists(data._reconstruction_file(fn_resplit)):
                reconstructions = data.load_reconstruction(fn_resplit)
            else:
                reconstructions = data.load_reconstruction()
                reconstructions = [reconstructions[rec_a], reconstructions[rec_b]]
            coords0 = triangulate_gcps(gcps, reconstructions[0])
            coords1 = triangulate_gcps(gcps, reconstructions[1])
            n_valid_0 = sum(c is not None for c in coords0)
            logger.debug(f"Triangulated {n_valid_0}/{len(gcps)} gcps for rec #{rec_a}")
            n_valid_1 = sum(c is not None for c in coords1)
            logger.debug(f"Triangulated {n_valid_1}/{len(gcps)} gcps for rec #{rec_b}")
            try:
                s, A, b = find_alignment(coords1, coords0)
                apply_similarity(reconstructions[1], s, A, b)
            except ValueError:
                logger.warning(f"Could not rigidly align rec #{rec_b} to rec #{rec_a}")
                return
            logger.info(f"Rigidly aligned rec #{rec_b} to rec #{rec_a}")
        else:  # Image - to - reconstruction annotation
            reconstructions = data.load_reconstruction()
            base = reconstructions[rec_a]
            resected = resect_annotated_single_images(base, gcps, camera_models, data)
            reconstructions = [base, resected]
    else:
        logger.debug(
            "Only one reconstruction in reconstruction.json. Will only to 3d-3d alignment if any"
        )
        align_external_3d_models_to_reconstruction(
            data, gcps, reconstructions[0], rec_a
        )
        return

    logger.debug(f"Aligning annotations, if any, to rec #{rec_a}")
    align_external_3d_models_to_reconstruction(data, gcps, reconstructions[0], rec_a)

    # Set the GPS constraint of the moved/resected shots to the manually-aligned position
    for shot in reconstructions[1].shots.values():
        shot.metadata.gps_position.value = shot.pose.get_origin()

    data.save_reconstruction(reconstructions, fn_rigid)

    logger.info("Merging reconstructions")
    merged = merge_reconstructions(reconstructions, tracks_manager)

    # data.save_reconstruction(
    #     [merged], f"reconstruction_merged_{rec_a}x{rec_b}.json"
    # )

    # Scale the GPS DOP with the number of shots to ensure GCPs are used to align
    for shot in merged.shots.values():
        shot.metadata.gps_accuracy.value = 0.5 * len(merged.shots)

    gcp_alignment = {"after_rigid": gcp_geopositional_error(gcps, merged)}
    logger.info(
        "GCP errors after rigid alignment:\n"
        + "\n".join(
            "[{}]: {:.2f} m / {:.2f} m (planar)".format(
                k, v["error"], v["error_planar"]
            )
            for k, v in gcp_alignment["after_rigid"].items()
        )
    )
    if not rigid:
        data.config["bundle_max_iterations"] = 200
        data.config["bundle_use_gcp"] = True

        logger.info("Running BA on merged reconstructions")
        # orec.align_reconstruction(merged, None, data.config)
        orec.bundle(merged, camera_models, {}, gcp=gcps, config=data.config)
        data.save_reconstruction(
            [merged], f"reconstruction_gcp_ba_{rec_a}x{rec_b}.json"
        )

        gcp_alignment["after_bundle"] = gcp_geopositional_error(gcps, merged)
        logger.info(
            "GCP errors after bundle:\n"
            + "\n".join(
                "[{}]: {:.2f} m / {:.2f} m (planar)".format(
                    k, v["error"], v["error_planar"]
                )
                for k, v in gcp_alignment["after_bundle"].items()
            )
        )
        with open(f"{data.data_path}/gcp_alignment_{rec_a}x{rec_b}.json", "w") as f:
            json.dump(gcp_alignment, f, indent=4, sort_keys=True)

    # Reproject GCPs with a very loose threshold so that we get a point every time
    # These reprojections are only used for feedback in any case
    gcp_reprojections = reproject_gcps(gcps, merged, reproj_threshold=10)
    reprojection_errors = get_sorted_reprojection_errors(gcp_reprojections)
    err_values = [t[2] for t in reprojection_errors]
    max_reprojection_error = np.max(err_values)
    median_reprojection_error = np.median(err_values)

    with open(f"{data.data_path}/gcp_reprojections_{rec_a}x{rec_b}.json", "w") as f:
        json.dump(gcp_reprojections, f, indent=4, sort_keys=True)

    n_bad_gcp_annotations = int(sum(t[2] > px_threshold for t in reprojection_errors))
    if n_bad_gcp_annotations > 0:
        logger.info(f"{n_bad_gcp_annotations} large reprojection errors:")
        for t in reprojection_errors:
            if t[2] > px_threshold:
                logger.info(t)

    gcp_std = compute_gcp_std(gcp_reprojections)
    logger.info(f"GCP reprojection error STD: {gcp_std}")

    resplit = resplit_reconstruction(merged, reconstructions)
    data.save_reconstruction(resplit, fn_resplit)
    if covariance:

        # Re-triangulate to remove badly conditioned points
        n_points = len(merged.points)

        logger.info("Re-triangulating...")
        backup = data.config["triangulation_min_ray_angle"]
        data.config["triangulation_min_ray_angle"] = 2.0
        orec.retriangulate(tracks_manager, merged, data.config)
        orec.paint_reconstruction(data, tracks_manager, merged)
        data.config["triangulation_min_ray_angle"] = backup
        logger.info(
            f"Re-triangulated. Removed {n_points - len(merged.points)}."
            f" Kept {int(100*len(merged.points)/n_points)}%"
        )
        data.save_reconstruction(
            [merged],
            f"reconstruction_gcp_ba_retriangulated_{rec_a}x{rec_b}.json",
        )

        all_shots_std = []
        # We run bundle by fixing one reconstruction.
        # If we have two reconstructions, we do this twice, fixing each one.
        _rec_ixs = [(0, 1), (1, 0)] if rec_b is not None else [(0, 1)]
        for rec_ixs in _rec_ixs:
            logger.info(f"Running BA with fixed images. Fixing rec #{rec_ixs[0]}")
            fixed_images = set(reconstructions[rec_ixs[0]].shots.keys())
            covariance_estimation_valid = bundle_with_fixed_images(
                merged,
                camera_models,
                gcp=gcps,
                gcp_std=gcp_std,
                fixed_images=fixed_images,
                config=data.config,
            )
            if not covariance_estimation_valid:
                logger.info(
                    f"Could not get positional uncertainty for pair {rec_ixs} It could be because:"
                    "\na) there are not enough GCPs."
                    "\nb) they are badly distributed in 3D."
                    "\nc) there are some wrong annotations"
                )
                shots_std_this_pair = [
                    (shot, np.nan) for shot in reconstructions[rec_ixs[1]].shots
                ]
            else:
                shots_std_this_pair = []
                for shot in merged.shots.values():
                    if shot.id in reconstructions[rec_ixs[1]].shots:
                        u, std_v = decompose_covariance(shot.covariance[3:, 3:])
                        std = np.linalg.norm(std_v)
                        shots_std_this_pair.append((shot.id, std))
                        logger.debug(f"{shot.id} std: {std}")

            all_shots_std.extend(shots_std_this_pair)

        std_values = [x[1] for x in all_shots_std]
        n_nan_std = sum(np.isnan(std) for std in std_values)
        n_good_std = sum(
            std <= std_threshold for std in std_values if not np.isnan(std)
        )
        n_bad_std = len(std_values) - n_good_std - n_nan_std

        # Average positional STD
        median_shot_std = np.median(std_values)

        # Save the shot STD to a file
        with open(f"{data.data_path}/shots_std_{rec_a}x{rec_b}.csv", "w") as f:
            s = sorted(all_shots_std, key=lambda t: -t[-1])
            for t in s:
                line = "{}, {}".format(*t)
                f.write(line + "\n")

            max_shot_std = s[0][1]
    else:
        n_nan_std = -1
        n_bad_std = -1
        n_good_std = -1
        median_shot_std = -1
        max_shot_std = -1

    metrics = {
        "n_reconstructions": len(data.load_reconstruction()),
        "median_shot_std": median_shot_std,
        "max_shot_std": max_shot_std,
        "max_reprojection_error": max_reprojection_error,
        "median_reprojection_error": median_reprojection_error,
        "n_gcp": len(gcps),
        "n_bad_gcp_annotations": n_bad_gcp_annotations,
        "n_bad_position_std": int(n_bad_std),
        "n_good_position_std": int(n_good_std),
        "n_nan_position_std": int(n_nan_std),
        "rec_a": rec_a,
        "rec_b": rec_b,
    }

    logger.info(metrics)
    p_metrics = f"{data.data_path}/run_ba_metrics_{rec_a}x{rec_b}.json"
    with open(p_metrics, "w") as f:
        json.dump(metrics, f, indent=4, sort_keys=True)
    logger.info(f"Saved metrics to {p_metrics}")

    logger.info("========================================")
    logger.info("=============== Summary ================")
    logger.info("========================================")
    if n_bad_std == 0 and n_bad_gcp_annotations == 0:
        logger.info(
            f"No issues. All gcp reprojections are under {px_threshold}"
            f" and all frames are localized within {std_threshold}m"
        )
    if n_bad_std != 0 or n_bad_gcp_annotations != 0:
        if rigid:
            logger.info("Positional uncertainty was not calculated. (--rigid was set).")
        elif not covariance:
            logger.info(
                "Positional uncertainty was not calculated (--covariance not set)."
            )
        else:
            logger.info(
                # pyre-fixme[61]: `std_values` may not be initialized here.
                f"{n_nan_std}/{len(std_values)} images with unknown error."
                # pyre-fixme[61]: `std_values` may not be initialized here.
                f"\n{n_good_std}/{len(std_values)} well-localized images."
                # pyre-fixme[61]: `std_values` may not be initialized here.
                f"\n{n_bad_std}/{len(std_values)} badly localized images."
            )

        if n_bad_gcp_annotations > 0:
            logger.info(
                f"{n_bad_gcp_annotations} annotations with large reprojection error."
                " Worst offenders:"
            )

            stats_bad_reprojections = get_number_of_wrong_annotations_per_gcp(
                gcp_reprojections, px_threshold
            )
            gcps_sorted = sorted(
                stats_bad_reprojections, key=lambda k: -stats_bad_reprojections[k]
            )
            for ix, gcp_id in enumerate(gcps_sorted[:5]):
                n = stats_bad_reprojections[gcp_id]
                if n > 0:
                    logger.info(f"#{ix+1} - {gcp_id}: {n} bad annotations")
        else:
            logger.info("No annotations with large reprojection errors")


if __name__ == "__main__":
    log.setup()
    args = parse_args()
    sys.exit(
        align(
            args.path_dataset,
            args.rec_a,
            args.rec_b,
            args.rigid,
            args.covariance,
            args.px_threshold,
            args.std_threshold,
        )
    )
