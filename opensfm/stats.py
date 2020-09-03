import os
import numpy as np

from collections import defaultdict

import matplotlib.pyplot as plt


def _length_histogram(points):
    hist = defaultdict(int)
    for point in points.values():
        hist[point.number_of_observations()] += 1
    return np.array(list(hist.keys())), np.array(list(hist.values()))


def compute_overall_statistics(reconstruction):
    stats = {}
    stats['points_count'] = len(reconstruction.points)
    stats['cameras_count'] = len(reconstruction.shots)

    hist, values = _length_histogram(reconstruction.points)
    stats['observations_count'] = int(sum(hist * values))
    if len(reconstruction.points) > 0:
        stats['average_track_length'] = float(stats['observations_count'])/len(reconstruction.points)
    else:
        stats['average_track_length'] = -1
    tracks_notwo = sum([1 if p.number_of_observations() > 2 else 0 for p in reconstruction.points.values()])

    if tracks_notwo > 0:
        stats['average_track_length_notwo'] = float(sum(hist[1:]*values[1:]))/tracks_notwo
    else:
        stats['average_track_length_notwo'] = -1
    return stats


def _grid_buckets(camera):
    buckets = 40
    if camera.projection_type == 'spherical':
        return 2*buckets, buckets
    else:
        return buckets, buckets


def save_residual_grids(tracks_manager, reconstructions, output_path):
    all_errors = {}

    scaling = 2e4
    cutoff = 0.0005

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
            center = [w/2.0, h/2.0]
            normalizer = max(w, h)

            buckets_x, buckets_y = _grid_buckets(shot.camera)
            w_bucket = buckets_x/w
            h_bucket = buckets_y/h
            if shot_id not in tracks_manager.get_shot_ids():
                continue
            for point_id, obs in tracks_manager.get_shot_observations(shot_id).items():
                if point_id not in all_points_keys:
                    continue
                proj = shot.project(all_points[point_id].coordinates)
                error = obs.point-proj
                if np.linalg.norm(error) > cutoff:
                    continue

                bucket = (obs.point*normalizer + center)
                x = max([0, min([int(bucket[0]*w_bucket), buckets_x-1])])
                y = max([0, min([int(bucket[1]*h_bucket), buckets_y-1])])

                all_errors[shot.camera.id].append(((x, y), error*scaling))

    for camera_id, errors in all_errors.items():
        buckets_x, buckets_y = _grid_buckets(rec.cameras[camera_id])
        camera_array_res = np.zeros((buckets_y, buckets_x, 2))
        camera_array_count = np.full((buckets_y, buckets_x, 1), 1)
        for (x, y), e in errors:
            camera_array_res[y, x] += e
            camera_array_count[y, x, 0] += 1
        camera_array_res = np.divide(camera_array_res, camera_array_count)
        plt.quiver(camera_array_res[:, :, 0], camera_array_res[:, :, 1],
                   units='xy', angles='xy', scale_units='xy', scale=1, width=0.1)
        plt.grid(True)
        plt.savefig(os.path.join(output_path, 'residuals_' + str(camera_id) + '.png'), dpi=300, bbox_inches='tight')
