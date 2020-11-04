import os
import numpy as np

from collections import defaultdict

import matplotlib.pyplot as plt


def _length_histogram(points):
    hist = defaultdict(int)
    for point in points.values():
        hist[point.number_of_observations()] += 1
    return np.array(list(hist.keys())), np.array(list(hist.values()))


def _gps_errors(reconstruction):
    errors = []
    for shot in reconstruction.shots.values():
        if shot.metadata.gps_position.has_value:
            errors.append(
                np.array(shot.metadata.gps_position.value - shot.pose.get_origin())
            )
    return np.array(errors)


def compute_overall_statistics(reconstruction):
    stats = {}
    stats["points_count"] = len(reconstruction.points)
    stats["cameras_count"] = len(reconstruction.shots)

    hist, values = _length_histogram(reconstruction.points)
    stats["observations_count"] = int(sum(hist * values))
    if len(reconstruction.points) > 0:
        stats["average_track_length"] = float(stats["observations_count"]) / len(
            reconstruction.points
        )
    else:
        stats["average_track_length"] = -1
    tracks_notwo = sum(
        [
            1 if p.number_of_observations() > 2 else 0
            for p in reconstruction.points.values()
        ]
    )

    if tracks_notwo > 0:
        stats["average_track_length_notwo"] = (
            float(sum(hist[1:] * values[1:])) / tracks_notwo
        )
    else:
        stats["average_track_length_notwo"] = -1

    gps_errors = _gps_errors(reconstruction)
    mean = np.mean(gps_errors, 0)
    std_dev = np.std(gps_errors, 0)
    average = np.average(np.linalg.norm(gps_errors, axis=1))

    gps_errors_dict = {}
    if gps_errors.any():
        gps_errors_dict["mean"] = {"x": mean[0], "y": mean[1], "z": mean[2]}
        gps_errors_dict["std"] = {"x": std_dev[0], "y": std_dev[1], "z": std_dev[2]}
        gps_errors_dict["error"] = {
            "x": np.abs(mean[0]) + std_dev[0],
            "y": np.abs(mean[1]) + std_dev[1],
            "z": np.abs(mean[2]) + std_dev[2],
        }
    gps_errors_dict["average_error"] = average
    stats["gps_errors"] = gps_errors_dict

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


def save_heat_map(data, tracks_manager, reconstructions, output_path):
    all_projections = {}

    scaling = 1e4
    splatting = 3

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
                x = max(
                    [
                        splatting,
                        min([int(bucket[0] * w_bucket), buckets_x - 1 - splatting]),
                    ]
                )
                y = max(
                    [
                        splatting,
                        min([int(bucket[1] * h_bucket), buckets_y - 1 - splatting]),
                    ]
                )
                for i in range(-splatting, splatting):
                    for j in range(-splatting, splatting):
                        all_projections[shot.camera.id].append((x + i, y + j))

    for camera_id, projections in all_projections.items():
        buckets_x, buckets_y = _heatmap_buckets(rec.cameras[camera_id])
        camera_heatmap = np.zeros((buckets_y, buckets_x))
        for x, y in projections:
            camera_heatmap[y, x] += 1

        highest = np.max(camera_heatmap)
        lowest = np.min(camera_heatmap)
        plt.imshow((camera_heatmap - lowest) / (highest - lowest) * 255)
        plt.show()


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
