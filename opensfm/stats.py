import datetime
import math
import os
import random
import statistics
from collections import defaultdict
from functools import lru_cache

import matplotlib as mpl
import matplotlib.cm as cm
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np
from opensfm import io, multiview, feature_loader, pymap
from opensfm.dataset import DataSet, DataSetBase

RESIDUAL_PIXEL_CUTOFF = 4


def _norm2d(point):
    return math.sqrt(point[0] * point[0] + point[1] * point[1])


def _length_histogram(tracks_manager, points):
    hist = defaultdict(int)
    for point in points.values():
        obs_count = point.number_of_observations()
        if not obs_count:
            obs_count = len(tracks_manager.get_track_observations(point.id))
        hist[obs_count] += 1
    return list(hist.keys()), list(hist.values())


def _gps_errors(reconstruction):
    errors = []
    for shot in reconstruction.shots.values():
        if shot.metadata.gps_position.has_value:
            bias = reconstruction.biases[shot.camera.id]
            gps = shot.metadata.gps_position.value
            unbiased_gps = bias.scale * bias.transform(gps)
            optical_center = shot.pose.get_origin()
            errors.append(np.array(optical_center - unbiased_gps))
    return errors


def _gps_gcp_errors_stats(errors):
    if not errors:
        return {}

    stats = {}
    squared = np.multiply(errors, errors)
    m_squared = np.mean(squared, 0)
    mean = np.mean(errors, 0)
    std_dev = np.std(errors, 0)
    average = np.average(np.linalg.norm(errors, axis=1))

    stats["mean"] = {"x": mean[0], "y": mean[1], "z": mean[2]}
    stats["std"] = {"x": std_dev[0], "y": std_dev[1], "z": std_dev[2]}
    stats["error"] = {
        "x": math.sqrt(m_squared[0]),
        "y": math.sqrt(m_squared[1]),
        "z": math.sqrt(m_squared[2]),
    }
    stats["average_error"] = average
    return stats


def gps_errors(reconstructions):
    all_errors = []
    for rec in reconstructions:
        all_errors += _gps_errors(rec)
    return _gps_gcp_errors_stats(all_errors)


def gcp_errors(data: DataSetBase, reconstructions):
    all_errors = []

    gcp = data.load_ground_control_points()
    if not gcp:
        return {}

    all_errors = []
    for gcp in gcp:
        if not gcp.coordinates.has_value:
            continue

        for rec in reconstructions:
            triangulated = multiview.triangulate_gcp(gcp, rec.shots, 1.0, 0.1)
            if triangulated is None:
                continue
            else:
                break

        # pyre-fixme[61]: `triangulated` may not be initialized here.
        if triangulated is None:
            continue
        all_errors.append(triangulated - gcp.coordinates.value)

    return _gps_gcp_errors_stats(all_errors)


def _compute_errors(reconstructions, tracks_manager):
    @lru_cache(10)
    def _compute_errors_cached(index, error_type):
        return reconstructions[index].map.compute_reprojection_errors(
            tracks_manager,
            error_type,
        )

    return _compute_errors_cached


def _get_valid_observations(reconstructions, tracks_manager):
    @lru_cache(10)
    def _get_valid_observations_cached(index):
        return reconstructions[index].map.get_valid_observations(tracks_manager)

    return _get_valid_observations_cached


def _projection_error(tracks_manager, reconstructions):
    all_errors_normalized, all_errors_pixels, all_errors_angular = [], [], []
    average_error_normalized, average_error_pixels, average_error_angular = 0, 0, 0
    for i in range(len(reconstructions)):
        errors_normalized = _compute_errors(reconstructions, tracks_manager)(
            i, pymap.ErrorType.Normalized
        )
        errors_unnormalized = _compute_errors(reconstructions, tracks_manager)(
            i, pymap.ErrorType.Pixel
        )
        errors_angular = _compute_errors(reconstructions, tracks_manager)(
            i, pymap.ErrorType.Angular
        )

        for shot_id, shot_errors_normalized in errors_normalized.items():
            shot = reconstructions[i].get_shot(shot_id)
            normalizer = max(shot.camera.width, shot.camera.height)

            for error_normalized, error_unnormalized, error_angular in zip(
                shot_errors_normalized.values(),
                errors_unnormalized[shot_id].values(),
                errors_angular[shot_id].values(),
            ):
                norm_pixels = _norm2d(error_unnormalized * normalizer)
                norm_normalized = _norm2d(error_normalized)
                norm_angle = error_angular[0]
                if norm_pixels > RESIDUAL_PIXEL_CUTOFF or math.isnan(norm_angle):
                    continue
                average_error_normalized += norm_normalized
                average_error_pixels += norm_pixels
                average_error_angular += norm_angle
                all_errors_normalized.append(norm_normalized)
                all_errors_pixels.append(norm_pixels)
                all_errors_angular.append(norm_angle)

    error_count = len(all_errors_normalized)
    if error_count == 0:
        return (-1.0, -1.0, -1.0, ([], []), ([], []), ([], []))

    bins = 30
    return (
        average_error_normalized / error_count,
        average_error_pixels / error_count,
        average_error_angular / error_count,
        np.histogram(all_errors_normalized, bins),
        np.histogram(all_errors_pixels, bins),
        np.histogram(all_errors_angular, bins),
    )


def reconstruction_statistics(data: DataSetBase, tracks_manager, reconstructions):
    stats = {}

    stats["components"] = len(reconstructions)
    gps_count = 0
    for rec in reconstructions:
        for shot in rec.shots.values():
            gps_count += shot.metadata.gps_position.has_value
    stats["has_gps"] = gps_count > 2
    stats["has_gcp"] = True if data.load_ground_control_points() else False

    stats["initial_points_count"] = tracks_manager.num_tracks()
    stats["initial_shots_count"] = len(data.images())

    stats["reconstructed_points_count"] = 0
    stats["reconstructed_shots_count"] = 0
    stats["observations_count"] = 0
    hist_agg = defaultdict(int)

    for rec in reconstructions:
        if len(rec.points) > 0:
            stats["reconstructed_points_count"] += len(rec.points)
        stats["reconstructed_shots_count"] += len(rec.shots)

        # get tracks length distrbution for current reconstruction
        hist, values = _length_histogram(tracks_manager, rec.points)

        # update aggregrated histogram
        for length, count_tracks in zip(hist, values):
            hist_agg[length] += count_tracks

    # observations total and average tracks lengths
    hist_agg = sorted(hist_agg.items(), key=lambda x: x[0])
    lengths, counts = np.array([int(x[0]) for x in hist_agg]), np.array(
        [x[1] for x in hist_agg]
    )

    points_count = stats["reconstructed_points_count"]
    points_count_over_two = sum(counts[1:])
    stats["observations_count"] = int(sum(lengths * counts))
    stats["average_track_length"] = (
        (stats["observations_count"] / points_count) if points_count > 0 else -1
    )
    stats["average_track_length_over_two"] = (
        (int(sum(lengths[1:] * counts[1:])) / points_count_over_two)
        if points_count_over_two > 0
        else -1
    )
    stats["histogram_track_length"] = {k: v for k, v in hist_agg}

    (
        avg_normalized,
        avg_pixels,
        avg_angular,
        (hist_normalized, bins_normalized),
        (hist_pixels, bins_pixels),
        (hist_angular, bins_angular),
    ) = _projection_error(tracks_manager, reconstructions)
    stats["reprojection_error_normalized"] = avg_normalized
    stats["reprojection_error_pixels"] = avg_pixels
    stats["reprojection_error_angular"] = avg_angular
    stats["reprojection_histogram_normalized"] = (
        list(map(float, hist_normalized)),
        list(map(float, bins_normalized)),
    )
    stats["reprojection_histogram_pixels"] = (
        list(map(float, hist_pixels)),
        list(map(float, bins_pixels)),
    )
    stats["reprojection_histogram_angular"] = (
        list(map(float, hist_angular)),
        list(map(float, bins_angular)),
    )

    return stats


def processing_statistics(data: DataSet, reconstructions):
    steps = {
        "Feature Extraction": "features.json",
        "Features Matching": "matches.json",
        "Tracks Merging": "tracks.json",
        "Reconstruction": "reconstruction.json",
    }

    steps_times = {}
    for step_name, report_file in steps.items():
        file_path = os.path.join(data.data_path, "reports", report_file)
        if os.path.exists(file_path):
            with io.open_rt(file_path) as fin:
                obj = io.json_load(fin)
        else:
            obj = {}
        if "wall_time" in obj:
            steps_times[step_name] = obj["wall_time"]
        elif "wall_times" in obj:
            steps_times[step_name] = sum(obj["wall_times"].values())
        else:
            steps_times[step_name] = -1

    stats = {}
    stats["steps_times"] = steps_times
    stats["steps_times"]["Total Time"] = sum(
        filter(lambda x: x >= 0, steps_times.values())
    )

    try:
        stats["date"] = datetime.datetime.fromtimestamp(
            data.io_handler.timestamp(data._reconstruction_file(None))
        ).strftime("%d/%m/%Y at %H:%M:%S")
    except FileNotFoundError:
        stats["date"] = "unknown"

    default_max = 1e30
    min_x, min_y, max_x, max_y = default_max, default_max, 0, 0
    for rec in reconstructions:
        for shot in rec.shots.values():
            o = shot.pose.get_origin()
            min_x = min(min_x, o[0])
            min_y = min(min_y, o[1])
            max_x = max(max_x, o[0])
            max_y = max(max_y, o[1])
    stats["area"] = (max_x - min_x) * (max_y - min_y) if min_x != default_max else -1
    return stats


def features_statistics(data: DataSetBase, tracks_manager, reconstructions):
    stats = {}
    detected = []
    images = {s for r in reconstructions for s in r.shots}
    for im in images:
        features_data = feature_loader.instance.load_all_data(data, im, False)
        if not features_data:
            continue
        detected.append(len(features_data.points))
    if len(detected) > 0:
        stats["detected_features"] = {
            "min": min(detected),
            "max": max(detected),
            "mean": int(np.mean(detected)),
            "median": int(np.median(detected)),
        }
    else:
        stats["detected_features"] = {"min": -1, "max": -1, "mean": -1, "median": -1}

    per_shots = defaultdict(int)
    for rec in reconstructions:
        all_points_keys = set(rec.points.keys())
        for shot_id in rec.shots:
            if shot_id not in tracks_manager.get_shot_ids():
                continue
            for point_id in tracks_manager.get_shot_observations(shot_id):
                if point_id not in all_points_keys:
                    continue
                per_shots[shot_id] += 1
    per_shots = list(per_shots.values())

    stats["reconstructed_features"] = {
        "min": int(min(per_shots)) if len(per_shots) > 0 else -1,
        "max": int(max(per_shots)) if len(per_shots) > 0 else -1,
        "mean": int(np.mean(per_shots)) if len(per_shots) > 0 else -1,
        "median": int(np.median(per_shots)) if len(per_shots) > 0 else -1,
    }
    return stats


def _cameras_statistics(camera_model):
    camera_stats = {}
    for param_type, param_value in camera_model.get_parameters_map().items():
        camera_stats[str(param_type).split(".")[1]] = param_value
    return camera_stats


def cameras_statistics(data: DataSetBase, reconstructions):
    stats = {}
    permutation = np.argsort([-len(r.shots) for r in reconstructions])
    for camera_id, camera_model in data.load_camera_models().items():
        stats[camera_id] = {"initial_values": _cameras_statistics(camera_model)}

    for idx in permutation:
        rec = reconstructions[idx]
        for camera in rec.cameras.values():
            if "optimized_values" in stats[camera.id]:
                continue
            stats[camera.id]["optimized_values"] = _cameras_statistics(camera)
            stats[camera.id]["bias"] = io.bias_to_json(rec.biases[camera.id])

    for camera_id in data.load_camera_models():
        if "optimized_values" not in stats[camera_id]:
            del stats[camera_id]

    return stats


def rig_statistics(data: DataSetBase, reconstructions):
    stats = {}
    permutation = np.argsort([-len(r.shots) for r in reconstructions])
    for rig_camera_id, rig_camera in data.load_rig_cameras().items():
        stats[rig_camera_id] = {
            "initial_values": {
                "rotation": list(rig_camera.pose.rotation),
                "translation": list(rig_camera.pose.translation),
            }
        }

    for idx in permutation:
        rec = reconstructions[idx]
        for rig_camera in rec.rig_cameras.values():
            if "optimized_values" in stats[rig_camera.id]:
                continue
            stats[rig_camera.id]["optimized_values"] = {
                "rotation": list(rig_camera.pose.rotation),
                "translation": list(rig_camera.pose.translation),
            }

    for rig_camera_id in data.load_rig_cameras():
        if "optimized_values" not in stats[rig_camera_id]:
            del stats[rig_camera_id]

    return stats


def compute_all_statistics(data: DataSet, tracks_manager, reconstructions):
    stats = {}

    stats["processing_statistics"] = processing_statistics(data, reconstructions)
    stats["features_statistics"] = features_statistics(
        data, tracks_manager, reconstructions
    )
    stats["reconstruction_statistics"] = reconstruction_statistics(
        data, tracks_manager, reconstructions
    )
    stats["camera_errors"] = cameras_statistics(data, reconstructions)
    stats["rig_errors"] = rig_statistics(data, reconstructions)
    stats["gps_errors"] = gps_errors(reconstructions)
    stats["gcp_errors"] = gcp_errors(data, reconstructions)

    return stats


def _grid_buckets(camera):
    buckets = 40
    if camera.projection_type == "spherical":
        return 2 * buckets, buckets
    else:
        return buckets, buckets


def _heatmap_buckets(camera):
    buckets = 500
    if camera.projection_type == "spherical":
        return 2 * buckets, buckets
    else:
        return buckets, int(buckets / camera.width * camera.height)


def _get_gaussian_kernel(radius, ratio):
    std_dev = radius / ratio
    half_kernel = list(range(1, radius + 1))
    kernel = np.array(half_kernel + [radius + 1] + list(reversed(half_kernel)))
    kernel = np.exp(np.outer(kernel.T, kernel) / (2 * std_dev * std_dev))
    return kernel / sum(np.ndarray.flatten(kernel))


def save_matchgraph(
    data: DataSetBase,
    tracks_manager,
    reconstructions,
    output_path,
    io_handler,
):
    all_shots = []
    all_points = []
    shot_component = {}
    for i, rec in enumerate(reconstructions):
        all_points += rec.points
        all_shots += rec.shots
        for shot in rec.shots:
            shot_component[shot] = i

    connectivity = tracks_manager.get_all_pairs_connectivity(all_shots, all_points)
    all_values = connectivity.values()
    lowest = np.percentile(list(all_values), 5)
    highest = np.percentile(list(all_values), 95)

    plt.clf()
    cmap = cm.get_cmap("viridis")
    for (node1, node2), edge in sorted(connectivity.items(), key=lambda x: x[1]):
        if edge < 2 * data.config["resection_min_inliers"]:
            continue
        comp1 = shot_component[node1]
        comp2 = shot_component[node2]
        if comp1 != comp2:
            continue
        o1 = reconstructions[comp1].shots[node1].pose.get_origin()
        o2 = reconstructions[comp2].shots[node2].pose.get_origin()
        c = max(0, min(1.0, 1 - (edge - lowest) / (highest - lowest)))
        plt.plot([o1[0], o2[0]], [o1[1], o2[1]], linestyle="-", color=cmap(c))

    for i, rec in enumerate(reconstructions):
        for shot in rec.shots.values():
            o = shot.pose.get_origin()
            c = i / len(reconstructions)
            plt.plot(o[0], o[1], linestyle="", marker="o", color=cmap(c))

    plt.xticks([])
    plt.yticks([])
    ax = plt.gca()
    for b in ["top", "bottom", "left", "right"]:
        ax.spines[b].set_visible(False)

    norm = colors.Normalize(vmin=lowest, vmax=highest)
    sm = cm.ScalarMappable(norm=norm, cmap=cmap.reversed())
    sm.set_array([])
    plt.colorbar(
        sm,
        orientation="horizontal",
        label="Number of matches between images",
        pad=0.0,
    )

    with io_handler.open(os.path.join(output_path, "matchgraph.png"), "wb") as fwb:
        plt.savefig(
            fwb,
            dpi=300,
            bbox_inches="tight",
        )


def save_residual_histogram(
    stats,
    output_path,
    io_handler,
):
    backup = dict(mpl.rcParams)
    fig, axs = plt.subplots(1, 3, tight_layout=True, figsize=(15, 3))

    h_norm, b_norm = stats["reconstruction_statistics"][
        "reprojection_histogram_normalized"
    ]
    n, _, p_norm = axs[0].hist(b_norm[:-1], b_norm, weights=h_norm)
    n = n.astype("int")
    for i in range(len(p_norm)):
        p_norm[i].set_facecolor(plt.cm.viridis(n[i] / max(n)))

    h_pixel, b_pixel = stats["reconstruction_statistics"][
        "reprojection_histogram_pixels"
    ]
    n, _, p_pixel = axs[1].hist(b_pixel[:-1], b_pixel, weights=h_pixel)
    n = n.astype("int")
    for i in range(len(p_pixel)):
        p_pixel[i].set_facecolor(plt.cm.viridis(n[i] / max(n)))

    h_angular, b_angular = stats["reconstruction_statistics"][
        "reprojection_histogram_angular"
    ]
    n, _, p_angular, = axs[
        2
    ].hist(b_angular[:-1], b_angular, weights=h_angular)
    n = n.astype("int")
    for i in range(len(p_angular)):
        p_angular[i].set_facecolor(plt.cm.viridis(n[i] / max(n)))

    axs[0].set_title("Normalized Residual")
    axs[1].set_title("Pixel Residual")
    axs[2].set_title("Angular Residual")

    with io_handler.open(
        os.path.join(output_path, "residual_histogram.png"), "wb"
    ) as fwb:
        plt.savefig(
            fwb,
            dpi=300,
            bbox_inches="tight",
        )
    mpl.rcParams = backup


def save_topview(
    data: DataSetBase, tracks_manager, reconstructions, output_path, io_handler
):
    points = []
    colors = []
    for rec in reconstructions:
        for point in rec.points.values():
            track = tracks_manager.get_track_observations(point.id)
            if len(track) < 2:
                continue
            coords = point.coordinates
            points.append(coords)

            r, g, b = [], [], []
            for obs in track.values():
                r.append(obs.color[0])
                g.append(obs.color[1])
                b.append(obs.color[2])
            colors.append(
                (statistics.median(r), statistics.median(g), statistics.median(b))
            )

    all_x = []
    all_y = []
    for rec in reconstructions:
        for shot in rec.shots.values():
            o = shot.pose.get_origin()
            all_x.append(o[0])
            all_y.append(o[1])
            if not shot.metadata.gps_position.has_value:
                continue
            gps = shot.metadata.gps_position.value
            all_x.append(gps[0])
            all_y.append(gps[1])

    # compute camera's XY bounding box
    low_x, high_x = np.min(all_x), np.max(all_x)
    low_y, high_y = np.min(all_y), np.max(all_y)

    # get its size
    size_x = high_x - low_x
    size_y = high_y - low_y

    # expand bounding box by some margin
    margin = 0.05
    low_x -= size_x * margin
    high_x += size_y * margin
    low_y -= size_x * margin
    high_y += size_y * margin

    # update size
    size_x = high_x - low_x
    size_y = high_y - low_y

    im_size_x = 2000
    im_size_y = int(im_size_x * size_y / size_x)
    topview = np.zeros((im_size_y, im_size_x, 3))

    # splat points using gaussian + max-pool
    splatting = 15
    size = 2 * splatting + 1
    kernel = _get_gaussian_kernel(splatting, 2)
    kernel /= kernel[splatting, splatting]
    for point, color in zip(points, colors):
        x, y = int((point[0] - low_x) / size_x * im_size_x), int(
            (point[1] - low_y) / size_y * im_size_y
        )
        if not ((0 < x < (im_size_x - 1)) and (0 < y < (im_size_y - 1))):
            continue

        k_low_x, k_low_y = -min(x - splatting, 0), -min(y - splatting, 0)
        k_high_x, k_high_y = (
            size - max(x + splatting - (im_size_x - 2), 0),
            size - max(y + splatting - (im_size_y - 2), 0),
        )
        h_low_x, h_low_y = max(x - splatting, 0), max(y - splatting, 0)
        h_high_x, h_high_y = min(x + splatting + 1, im_size_x - 1), min(
            y + splatting + 1, im_size_y - 1
        )

        for i in range(3):
            current = topview[h_low_y:h_high_y, h_low_x:h_high_x, i]
            splat = kernel[k_low_y:k_high_y, k_low_x:k_high_x]
            topview[h_low_y:h_high_y, h_low_x:h_high_x, i] = np.maximum(
                splat * (color[i] / 255.0), current
            )

    plt.clf()
    plt.imshow(topview)

    # display computed camera's XY
    linewidth = 1
    markersize = 4
    for rec in reconstructions:
        sorted_shots = sorted(
            rec.shots.values(), key=lambda x: x.metadata.capture_time.value
        )
        c_camera = cm.get_cmap("cool")(0 / len(reconstructions))
        c_gps = cm.get_cmap("autumn")(0 / len(reconstructions))
        for j, shot in enumerate(sorted_shots):
            o = shot.pose.get_origin()
            x, y = int((o[0] - low_x) / size_x * im_size_x), int(
                (o[1] - low_y) / size_y * im_size_y
            )
            plt.plot(
                x,
                y,
                linestyle="",
                marker="o",
                color=c_camera,
                markersize=markersize,
                linewidth=1,
            )

            # also display camera path using capture time
            if j < len(sorted_shots) - 1:
                n = sorted_shots[j + 1].pose.get_origin()
                nx, ny = int((n[0] - low_x) / size_x * im_size_x), int(
                    (n[1] - low_y) / size_y * im_size_y
                )
                plt.plot(
                    [x, nx], [y, ny], linestyle="-", color=c_camera, linewidth=linewidth
                )

            # display GPS error
            if not shot.metadata.gps_position.has_value:
                continue
            gps = shot.metadata.gps_position.value
            gps_x, gps_y = int((gps[0] - low_x) / size_x * im_size_x), int(
                (gps[1] - low_y) / size_y * im_size_y
            )
            plt.plot(
                gps_x,
                gps_y,
                linestyle="",
                marker="v",
                color=c_gps,
                markersize=markersize,
                linewidth=1,
            )
            plt.plot(
                [x, gps_x], [y, gps_y], linestyle="-", color=c_gps, linewidth=linewidth
            )

    plt.xticks(
        [0, im_size_x / 2, im_size_x],
        [0, f"{int(size_x / 2):.0f}", f"{size_x:.0f} meters"],
        fontsize="small",
    )
    plt.yticks(
        [im_size_y, im_size_y / 2, 0],
        [0, f"{int(size_y / 2):.0f}", f"{size_y:.0f} meters"],
        fontsize="small",
    )
    with io_handler.open(os.path.join(output_path, "topview.png"), "wb") as fwb:
        plt.savefig(
            fwb,
            dpi=300,
            bbox_inches="tight",
        )


def save_heatmap(
    data: DataSetBase, tracks_manager, reconstructions, output_path, io_handler
):
    all_projections = {}

    splatting = 15
    size = 2 * splatting + 1
    kernel = _get_gaussian_kernel(splatting, 2)

    all_cameras = {}
    for rec in reconstructions:
        for camera in rec.cameras.values():
            all_projections[camera.id] = []
            all_cameras[camera.id] = camera

    for i in range(len(reconstructions)):
        valid_observations = _get_valid_observations(reconstructions, tracks_manager)(i)
        for shot_id, observations in valid_observations.items():
            shot = reconstructions[i].get_shot(shot_id)
            w = shot.camera.width
            h = shot.camera.height
            center = [w / 2.0, h / 2.0]
            normalizer = max(shot.camera.width, shot.camera.height)

            buckets_x, buckets_y = _heatmap_buckets(shot.camera)
            w_bucket = buckets_x / w
            h_bucket = buckets_y / h

            shots_projections = []
            for observation in observations.values():
                bucket = observation.point * normalizer + center
                x = max([0, min([int(bucket[0] * w_bucket), buckets_x - 1])])
                y = max([0, min([int(bucket[1] * h_bucket), buckets_y - 1])])
                shots_projections.append((x, y))
            all_projections[shot.camera.id] += shots_projections

    for camera_id, projections in all_projections.items():
        buckets_x, buckets_y = _heatmap_buckets(rec.cameras[camera_id])
        camera_heatmap = np.zeros((buckets_y, buckets_x))
        for x, y in projections:
            k_low_x, k_low_y = -min(x - splatting, 0), -min(y - splatting, 0)
            k_high_x, k_high_y = (
                size - max(x + splatting - (buckets_x - 2), 0),
                size - max(y + splatting - (buckets_y - 2), 0),
            )
            h_low_x, h_low_y = max(x - splatting, 0), max(y - splatting, 0)
            h_high_x, h_high_y = min(x + splatting + 1, buckets_x - 1), min(
                y + splatting + 1, buckets_y - 1
            )
            camera_heatmap[h_low_y:h_high_y, h_low_x:h_high_x] += kernel[
                k_low_y:k_high_y, k_low_x:k_high_x
            ]

        highest = np.max(camera_heatmap)
        lowest = np.min(camera_heatmap)

        plt.clf()
        plt.imshow((camera_heatmap - lowest) / (highest - lowest) * 255)

        plt.title(
            f"Detected features heatmap for camera {camera_id}",
            fontsize="x-small",
        )

        camera = all_cameras[camera_id]
        w = camera.width
        h = camera.height

        plt.xticks(
            [0, buckets_x / 2, buckets_x],
            [0, int(w / 2), w],
            fontsize="x-small",
        )
        plt.yticks(
            [buckets_y, buckets_y / 2, 0],
            [0, int(h / 2), h],
            fontsize="x-small",
        )

    with io_handler.open(
        os.path.join(
            output_path, "heatmap_" + str(camera_id.replace("/", "_")) + ".png"
        ),
        "wb",
    ) as fwb:

        plt.savefig(
            fwb,
            dpi=300,
            bbox_inches="tight",
        )


def save_residual_grids(
    data: DataSetBase,
    tracks_manager,
    reconstructions,
    output_path,
    io_handler,
):
    all_errors = {}

    scaling = 4
    for rec in reconstructions:
        for camera_id in rec.cameras:
            all_errors[camera_id] = []

    for i in range(len(reconstructions)):
        valid_observations = _get_valid_observations(reconstructions, tracks_manager)(i)
        errors_scaled = _compute_errors(reconstructions, tracks_manager)(
            i, pymap.ErrorType.Normalized
        )
        errors_unscaled = _compute_errors(reconstructions, tracks_manager)(
            i, pymap.ErrorType.Pixel
        )

        for shot_id, shot_errors in errors_scaled.items():
            shot = reconstructions[i].get_shot(shot_id)
            w = shot.camera.width
            h = shot.camera.height
            center = [w / 2.0, h / 2.0]
            normalizer = max(shot.camera.width, shot.camera.height)

            buckets_x, buckets_y = _grid_buckets(shot.camera)
            w_bucket = buckets_x / w
            h_bucket = buckets_y / h

            shots_errors = []
            for error_scaled, error_unscaled, observation in zip(
                shot_errors.values(),
                errors_unscaled[shot_id].values(),
                valid_observations[shot_id].values(),
            ):
                if _norm2d(error_unscaled * normalizer) > RESIDUAL_PIXEL_CUTOFF:
                    continue

                bucket = observation.point * normalizer + center
                x = max([0, min([int(bucket[0] * w_bucket), buckets_x - 1])])
                y = max([0, min([int(bucket[1] * h_bucket), buckets_y - 1])])

                shots_errors.append((x, y, error_scaled))
            all_errors[shot.camera.id] += shots_errors

    for camera_id, errors in all_errors.items():
        if not errors:
            continue
        buckets_x, buckets_y = _grid_buckets(rec.cameras[camera_id])
        camera_array_res = np.zeros((buckets_y, buckets_x, 2))
        camera_array_count = np.full((buckets_y, buckets_x, 1), 1)
        for x, y, e in errors:
            camera_array_res[y, x] += e
            camera_array_count[y, x, 0] += 1
        camera_array_res = np.divide(camera_array_res, camera_array_count)

        # pyre-fixme[61]: `shot_id` may not be initialized here.
        shot = rec.get_shot(shot_id)
        w = shot.camera.width
        h = shot.camera.height
        normalizer = max(w, h)

        clamp = 0.1
        res_colors = np.linalg.norm(camera_array_res[:, :, :2], axis=2)
        lowest = np.percentile(res_colors, 0)
        highest = np.percentile(res_colors, 100 * (1 - clamp))
        np.clip(res_colors, lowest, highest, res_colors)
        res_colors /= highest - lowest

        plt.clf()
        plt.figure(figsize=(12, 10))
        Q = plt.quiver(
            camera_array_res[:, :, 0] * scaling,
            camera_array_res[:, :, 1] * scaling,
            res_colors,
            units="xy",
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.1,
            cmap="viridis_r",
        )

        scale = highest - lowest
        plt.quiverkey(
            Q,
            X=0.1,
            Y=1.04,
            U=scale * scaling,
            label=f"Residual grid scale : {scale:.2f}",
            labelpos="E",
        )
        plt.title(
            "                      ",
            fontsize="large",
        )

        norm = colors.Normalize(vmin=lowest, vmax=highest)
        cmap = cm.get_cmap("viridis_r")
        sm = cm.ScalarMappable(norm=norm, cmap=cmap)
        sm.set_array([])
        plt.colorbar(
            mappable=sm,
            orientation="horizontal",
            label="Residual Norm",
            pad=0.08,
            aspect=40,
        )

        plt.xticks(
            [0, buckets_x / 2, buckets_x], [0, int(w / 2), w], fontsize="x-small"
        )
        plt.yticks(
            [0, buckets_y / 2, buckets_y], [0, int(h / 2), h], fontsize="x-small"
        )

        with io_handler.open(
            os.path.join(
                output_path, "residuals_" + str(camera_id.replace("/", "_")) + ".png"
            ),
            "wb",
        ) as fwb:
            plt.savefig(
                fwb,
                dpi=300,
                bbox_inches="tight",
            )


def decimate_points(reconstructions, max_num_points):
    """
    Destructively decimate the points in a reconstruction
    if they exceed max_num_points by removing points
    at random
    """
    for rec in reconstructions:
        if len(rec.points) > max_num_points:
            all_points = rec.points
            random_ids = list(all_points.keys())
            random.shuffle(random_ids)
            random_ids = set(random_ids[: len(all_points) - max_num_points])

            for point_id in random_ids:
                rec.remove_point(point_id)
