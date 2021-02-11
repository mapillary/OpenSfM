import datetime
import math
import os
import statistics
from collections import defaultdict
from functools import lru_cache

import matplotlib.cm as cm
import matplotlib.colors as colors
import matplotlib.pyplot as plt
import numpy as np
from opensfm import io, multiview


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
            errors.append(
                np.array(shot.metadata.gps_position.value - shot.pose.get_origin())
            )
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


def gcp_errors(data, reconstructions):
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

        if triangulated is None:
            continue
        all_errors.append(triangulated - gcp.coordinates.value)

    return _gps_gcp_errors_stats(all_errors)


def _projection_error(data, tracks_manager, reconstructions):
    average_error, error_count = 0, 0
    for rec in reconstructions:
        all_points = rec.points
        all_points_keys = set(all_points.keys())
        for shot_id in rec.shots:
            shot = rec.get_shot(shot_id)
            normalizer = max(shot.camera.width, shot.camera.height)
            if shot_id not in tracks_manager.get_shot_ids():
                continue
            for point_id, obs in tracks_manager.get_shot_observations(shot_id).items():
                if point_id not in all_points_keys:
                    continue
                proj = shot.project(all_points[point_id].coordinates)
                error = np.linalg.norm(obs.point - proj)
                if error > data.config["bundle_outlier_fixed_threshold"]:
                    continue
                average_error += error * normalizer
                error_count += 1

    if error_count == 0:
        return -1.0
    return average_error / error_count


def reconstruction_statistics(data, tracks_manager, reconstructions):
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
    stats["observations_count"] = int(sum(lengths * counts))
    stats["average_track_length"] = (
        stats["observations_count"] / stats["reconstructed_points_count"]
    )
    stats["average_track_length_over_two"] = int(sum(lengths[1:] * counts[1:])) / sum(
        counts[1:]
    )
    stats["histogram_track_length"] = {k: v for k, v in hist_agg}

    stats["reprojection_error"] = _projection_error(
        data, tracks_manager, reconstructions
    )

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


def processing_statistics(data, reconstructions):
    steps = {
        "Feature Extraction": "features.json",
        "Features Matching": "matches.json",
        "Tracks Merging": "tracks.json",
        "Reconstruction": "reconstruction.json",
    }

    steps_times = {}
    for step_name, report_file in steps.items():
        file_path = os.path.join(data.data_path, "reports", report_file)
        with io.open_rt(file_path) as fin:
            obj = io.json_load(fin)
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

    stats["date"] = datetime.datetime.fromtimestamp(
        os.path.getmtime(data._reconstruction_file(None))
    ).strftime("%d/%m/%Y at %H:%M:%S")

    min_x, min_y, max_x, max_y = 1e30, 1e30, 0, 0
    for rec in reconstructions:
        for shot in rec.shots.values():
            o = shot.pose.get_origin()
            min_x = min(min_x, o[0])
            min_y = min(min_y, o[1])
            max_x = max(max_x, o[0])
            max_y = max(max_y, o[1])
    stats["area"] = (max_x - min_x) * (max_y - min_y)
    return stats


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
            stats[camera.id]["optimized_values"] = _cameras_statistics(camera)

    for camera_id in data.load_camera_models():
        if "optimized_values" not in stats[camera_id]:
            del stats[camera_id]

    return stats


def compute_all_statistics(data, tracks_manager, reconstructions):
    stats = {}

    stats["processing_statistics"] = processing_statistics(data, reconstructions)
    stats["features_statistics"] = features_statistics(
        data, tracks_manager, reconstructions
    )
    stats["matching_statistics"] = matching_statistics(data)
    stats["reconstruction_statistics"] = reconstruction_statistics(
        data, tracks_manager, reconstructions
    )
    stats["camera_errors"] = cameras_statistics(data, reconstructions)
    stats["gps_errors"] = gps_errors(reconstructions)
    stats["gcp_errors"] = gcp_errors(data, reconstructions)

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


def _get_gaussian_kernel(radius, ratio):
    std_dev = radius / ratio
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
    ax = plt.gca()
    for b in ["top", "bottom", "left", "right"]:
        ax.spines[b].set_visible(False)

    norm = colors.Normalize(vmin=lowest, vmax=highest)
    plt.colorbar(
        cm.ScalarMappable(norm=norm, cmap=cmap.reversed()),
        orientation="horizontal",
        label="Number of matches between images",
        pad=0.0,
    )
    plt.savefig(
        os.path.join(output_path, "matchgraph.png"),
        dpi=300,
        bbox_inches="tight",
    )


def save_topview(data, tracks_manager, reconstructions, output_path):
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

    plt.savefig(
        os.path.join(output_path, "topview.png"),
        dpi=300,
        bbox_inches="tight",
    )


def save_heatmap(data, tracks_manager, reconstructions, output_path):
    all_projections = {}

    splatting = 15
    size = 2 * splatting + 1
    kernel = _get_gaussian_kernel(splatting, 2)

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
            os.path.join(
                output_path, "heatmap_" + str(camera_id.replace("/", "_")) + ".png"
            ),
            dpi=300,
            bbox_inches="tight",
        )


def save_residual_grids(data, tracks_manager, reconstructions, output_path):
    all_errors = {}

    scaling = 2
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

                all_errors[shot.camera.id].append(((x, y), error * normalizer))

    for camera_id, errors in all_errors.items():
        if not errors:
            continue
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

        clamp = 0.1
        res_colors = np.linalg.norm(camera_array_res[:, :, :1], axis=2)
        lowest = np.percentile(res_colors, 100 * clamp)
        highest = np.percentile(res_colors, 100 * (1 - clamp))
        np.clip(res_colors, lowest, highest, res_colors)
        res_colors /= highest - lowest

        plt.clf()
        Q = plt.quiver(
            camera_array_res[:, :, 0] * scaling,
            camera_array_res[:, :, 1] * scaling,
            res_colors,
            units="xy",
            angles="xy",
            scale_units="xy",
            scale=1,
            width=0.1,
            cmap="viridis",
        )

        scale = 1
        plt.quiverkey(
            Q,
            X=0.5,
            Y=1.04,
            U=scale * scaling,
            label=f"Residual grid scale : {scale} pixels",
            labelpos="E",
        )
        plt.title(
            "                      ",
            fontsize="large",
        )

        norm = colors.Normalize(vmin=lowest * scaling, vmax=highest * scaling)
        cmap = cm.get_cmap("viridis")
        plt.colorbar(
            cm.ScalarMappable(norm=norm, cmap=cmap),
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
        plt.savefig(
            os.path.join(
                output_path, "residuals_" + str(camera_id.replace("/", "_")) + ".png"
            ),
            dpi=300,
            bbox_inches="tight",
        )
