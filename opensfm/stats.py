import os
import math
from functools import lru_cache
import numpy as np

from collections import defaultdict

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as colors


def _length_histogram(tracks_manager, points):
    hist = defaultdict(int)
    for point in points.values():
        obs_count = point.number_of_observations()
        if not obs_count:
            obs_count = len(tracks_manager.get_track_observations(point.id))
        hist[obs_count] += 1
    return np.array(list(hist.keys())), np.array(list(hist.values()))


def _gps_errors(reconstruction):
    errors = []
    for shot in reconstruction.shots.values():
        if shot.metadata.gps_position.has_value:
            errors.append(
                np.array(shot.metadata.gps_position.value - shot.pose.get_origin())
            )
    return errors


def gps_errors(reconstructions):
    all_errors = []
    for rec in reconstructions:
        all_errors += _gps_errors(rec)
    squared = np.multiply(all_errors, all_errors)
    mean = np.mean(all_errors, 0)
    std_dev = np.std(all_errors, 0)
    average = np.average(np.linalg.norm(all_errors, axis=1))

    stats = {}
    if all_errors:
        stats["mean"] = {"x": mean[0], "y": mean[1], "z": mean[2]}
        stats["std"] = {"x": std_dev[0], "y": std_dev[1], "z": std_dev[2]}
        stats["error"] = {
            "x": math.sqrt(np.mean(squared, 0)[0]),
            "y": math.sqrt(np.mean(squared, 0)[1]),
            "z": math.sqrt(np.mean(squared, 0)[2]),
        }
        stats["average_error"] = average
    return stats


def reconstruction_statistics(data, tracks_manager, reconstructions):
    stats = {}

    stats["initial_points_count"] = tracks_manager.num_tracks()
    stats["initial_shots_count"] = len(data.images())

    stats["reconstructed_points_count"] = 0
    stats["reconstructed_shots_count"] = 0
    stats["observations_count"] = 0
    obs_notwo, count_notwo = 0, 0
    for rec in reconstructions:
        if len(rec.points) > 0:
            stats["reconstructed_points_count"] += len(rec.points)
        stats["reconstructed_shots_count"] += len(rec.shots)

        hist, values = _length_histogram(tracks_manager, rec.points)
        stats["observations_count"] += int(sum(hist * values))
        obs_notwo += int(sum(hist[1:] * values[1:]))
        count_notwo += sum(values[1:])
    stats["average_track_length"] = (
        stats["observations_count"] / stats["reconstructed_points_count"]
    )
    stats["average_track_length_over_two"] = obs_notwo / count_notwo

    length_histogram = {}
    for length, count_tracks in zip(hist, values):
        length_histogram[int(length)] = int(count_tracks)
    stats["histogram_track_length"] = length_histogram

    return stats


@lru_cache(1)
def _load_matches(data):
    matches = {}
    images = data.images()
    for im1 in images:
        try:
            im1_matches = data.load_matches(im1)
        except IOError:
            continue
        for im2 in im1_matches:
            if im2 in images:
                matches[im1, im2] = len(im1_matches[im2])
    return matches


def features_statistics(data, tracks_manager, reconstructions):
    stats = {}
    detected = [len(data.load_features(im)[0]) for im in data.images()]
    stats["detected_features"] = {
        "min": min(detected),
        "max": max(detected),
        "mean": int(np.mean(detected)),
        "median": int(np.median(detected)),
    }

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
        "min": int(min(per_shots)),
        "max": int(max(per_shots)),
        "mean": int(np.mean(per_shots)),
        "median": int(np.median(per_shots)),
    }
    return stats


def matching_statistics(data):
    return {}


def _cameras_statistics(camera_model):
    camera_stats = {}
    for param_type, param_value in camera_model.get_parameters_map().items():
        camera_stats[str(param_type).split(".")[1]] = param_value
    return camera_stats


def cameras_statistics(data, reconstructions):
    stats = {}
    permutation = np.argsort([-len(r.shots) for r in reconstructions])
    for camera_id, camera_model in data.load_camera_models().items():
        stats[camera_id] = {"initial_values": _cameras_statistics(camera_model)}

    for idx in permutation:
        rec = reconstructions[idx]
        for camera in rec.cameras.values():
            if "optimized_values" in stats[camera.id]:
                continue
            stats[camera_id]["optimized_values"] = _cameras_statistics(camera)
    return stats


def compute_all_statistics(data, tracks_manager, reconstructions):
    stats = {}

    stats["features_statistics"] = features_statistics(
        data, tracks_manager, reconstructions
    )
    stats["matching_statistics"] = matching_statistics(data)
    stats["reconstruction_statistics"] = reconstruction_statistics(
        data, tracks_manager, reconstructions
    )
    stats["camera_errors"] = cameras_statistics(data, reconstructions)
    stats["gps_errors"] = gps_errors(reconstructions)

    return stats


def _grid_buckets(camera):
    buckets = 60
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


def _get_gaussian_kernel(radius):
    std_dev = radius / 2
    half_kernel = list(range(1, radius + 1))
    kernel = np.array(half_kernel + [radius + 1] + list(reversed(half_kernel)))
    kernel = np.exp(np.outer(kernel.T, kernel) / (2 * std_dev * std_dev))
    return kernel / sum(np.ndarray.flatten(kernel))


def save_matchgraph(data, tracks_manager, reconstructions, output_path):
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
    norm = colors.Normalize(vmin=lowest, vmax=highest)
    plt.colorbar(
        cm.ScalarMappable(norm=norm, cmap=cmap.reversed()),
        orientation="horizontal",
        label="Number of matches between images",
    )
    plt.savefig(
        os.path.join(output_path, "matchgraph.png"),
        dpi=300,
        bbox_inches="tight",
    )


def save_heatmap(data, tracks_manager, reconstructions, output_path):
    all_projections = {}

    scaling = 1e4
    splatting = 15
    size = 2 * splatting + 1
    kernel = _get_gaussian_kernel(splatting)

    for rec in reconstructions:
        for camera_id in rec.cameras:
            all_projections[camera_id] = []

    for rec in reconstructions:
        all_points = rec.points
        all_points_keys = set(all_points.keys())
        for shot_id in rec.shots:
            shot = rec.get_shot(shot_id)
            w = shot.camera.width
            h = shot.camera.height
            center = [w / 2.0, h / 2.0]
            normalizer = max(w, h)

            buckets_x, buckets_y = _heatmap_buckets(shot.camera)
            w_bucket = buckets_x / w
            h_bucket = buckets_y / h
            if shot_id not in tracks_manager.get_shot_ids():
                continue
            for point_id, obs in tracks_manager.get_shot_observations(shot_id).items():
                if point_id not in all_points_keys:
                    continue

                bucket = obs.point * normalizer + center
                x = max([0, min([int(bucket[0] * w_bucket), buckets_x - 1])])
                y = max([0, min([int(bucket[1] * h_bucket), buckets_y - 1])])
                all_projections[shot.camera.id].append((x, y))

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

        plt.xticks(
            [0, buckets_x / 2, buckets_x], [0, int(w / 2), w], fontsize="x-small"
        )
        plt.yticks(
            [buckets_y, buckets_y / 2, 0], [0, int(h / 2), h], fontsize="x-small"
        )

        plt.savefig(
            os.path.join(output_path, "heatmap_" + str(camera_id) + ".png"),
            dpi=300,
            bbox_inches="tight",
        )


def save_residual_grids(data, tracks_manager, reconstructions, output_path):
    all_errors = {}

    scaling = 1e4
    cutoff = data.config["bundle_outlier_fixed_threshold"]

    for rec in reconstructions:
        for camera_id in rec.cameras:
            all_errors[camera_id] = []

    for rec in reconstructions:
        all_points = rec.points
        all_points_keys = set(all_points.keys())
        for shot_id in rec.shots:
            shot = rec.get_shot(shot_id)
            w = shot.camera.width
            h = shot.camera.height
            center = [w / 2.0, h / 2.0]
            normalizer = max(w, h)

            buckets_x, buckets_y = _grid_buckets(shot.camera)
            w_bucket = buckets_x / w
            h_bucket = buckets_y / h
            if shot_id not in tracks_manager.get_shot_ids():
                continue
            for point_id, obs in tracks_manager.get_shot_observations(shot_id).items():
                if point_id not in all_points_keys:
                    continue
                proj = shot.project(all_points[point_id].coordinates)
                error = obs.point - proj
                if np.linalg.norm(error) > cutoff:
                    continue

                bucket = obs.point * normalizer + center
                x = max([0, min([int(bucket[0] * w_bucket), buckets_x - 1])])
                y = max([0, min([int(bucket[1] * h_bucket), buckets_y - 1])])

                all_errors[shot.camera.id].append(((x, y), error))

    for camera_id, errors in all_errors.items():
        buckets_x, buckets_y = _grid_buckets(rec.cameras[camera_id])
        camera_array_res = np.zeros((buckets_y, buckets_x, 2))
        camera_array_count = np.full((buckets_y, buckets_x, 1), 1)
        for (x, y), e in errors:
            camera_array_res[y, x] += e
            camera_array_count[y, x, 0] += 1
        camera_array_res = np.divide(camera_array_res, camera_array_count)

        shot = rec.get_shot(shot_id)
        w = shot.camera.width
        h = shot.camera.height
        normalizer = max(w, h)

        plt.clf()
        colors = np.linalg.norm(camera_array_res[:, :, :1], axis=2)
        Q = plt.quiver(
            camera_array_res[:, :, 0] * scaling,
            camera_array_res[:, :, 1] * scaling,
            colors,
            units="xy",
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.1,
            cmap="viridis",
        )

        plt.quiverkey(Q, 0.9, 1.2, 1, "toto", labelpos="E", coordinates="figure")
        plt.title(
            f"Reprojection error for camera {camera_id}\n\nAverage error : {np.average(np.ndarray.flatten(colors))*normalizer:.3f} pixels",
            fontsize="x-small",
        )

        plt.xticks(
            [0, buckets_x / 2, buckets_x], [0, int(w / 2), w], fontsize="x-small"
        )
        plt.yticks(
            [0, buckets_y / 2, buckets_y], [0, int(h / 2), h], fontsize="x-small"
        )
        plt.savefig(
            os.path.join(output_path, "residuals_" + str(camera_id) + ".png"),
            dpi=300,
            bbox_inches="tight",
        )
