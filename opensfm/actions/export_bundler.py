import os

import numpy as np
from opensfm import io
from opensfm.dataset import DataSet


def run_dataset(data: DataSet, list_path, bundle_path, undistorted):
    """Export reconstruction to bundler format.

    Args:
        list_path: txt list of images to export
        bundle_path : output path
        undistorted : export undistorted reconstruction

    """

    udata = data.undistorted_dataset()

    default_path = os.path.join(data.data_path, "bundler")
    list_file_path = list_path if list_path else default_path
    bundle_file_path = bundle_path if bundle_path else default_path

    if undistorted:
        reconstructions = udata.load_undistorted_reconstruction()
        track_manager = udata.load_undistorted_tracks_manager()
        images = reconstructions[0].shots.keys()
    else:
        reconstructions = data.load_reconstruction()
        track_manager = data.load_tracks_manager()
        images = data.images()

    export_bundler(
        images, reconstructions, track_manager, bundle_file_path, list_file_path
    )


def export_bundler(
    image_list, reconstructions, track_manager, bundle_file_path, list_file_path
):
    """
    Generate a reconstruction file that is consistent with Bundler's format
    """

    io.mkdir_p(bundle_file_path)
    io.mkdir_p(list_file_path)

    for j, reconstruction in enumerate(reconstructions):
        lines = []
        lines.append("# Bundle file v0.3")
        points = reconstruction.points
        shots = reconstruction.shots
        num_point = len(points)
        num_shot = len(image_list)
        lines.append(" ".join(map(str, [num_shot, num_point])))
        shots_order = {key: i for i, key in enumerate(image_list)}

        # cameras
        for shot_id in image_list:
            if shot_id in shots:
                shot = shots[shot_id]
                camera = shot.camera
                if shot.camera.projection_type == "brown":
                    # Will aproximate Brown model, not optimal
                    focal_normalized = camera.focal_x
                else:
                    focal_normalized = camera.focal
                scale = max(camera.width, camera.height)
                focal = focal_normalized * scale
                k1 = camera.k1
                k2 = camera.k2
                R = shot.pose.get_rotation_matrix()
                t = np.array(shot.pose.translation)
                R[1], R[2] = -R[1], -R[2]  # Reverse y and z
                t[1], t[2] = -t[1], -t[2]
                lines.append(" ".join(map(str, [focal, k1, k2])))
                for i in range(3):
                    lines.append(" ".join(map(str, R[i])))
                t = " ".join(map(str, t))
                lines.append(t)
            else:
                for _ in range(5):
                    lines.append("0 0 0")

        # tracks
        for point in points.values():
            coord = point.coordinates
            color = list(map(int, point.color))
            view_list = track_manager.get_track_observations(point.id)
            lines.append(" ".join(map(str, coord)))
            lines.append(" ".join(map(str, color)))
            view_line = []
            for shot_key, obs in view_list.items():
                if shot_key in shots.keys():
                    v = obs.point
                    shot_index = shots_order[shot_key]
                    camera = shots[shot_key].camera
                    scale = max(camera.width, camera.height)
                    x = v[0] * scale
                    y = -v[1] * scale
                    view_line.append(" ".join(map(str, [shot_index, obs.id, x, y])))

            lines.append(str(len(view_line)) + " " + " ".join(view_line))

        bundle_file = os.path.join(
            bundle_file_path, "bundle_r" + str(j).zfill(3) + ".out"
        )
        with io.open_wt(bundle_file) as fout:
            fout.writelines("\n".join(lines) + "\n")

        list_file = os.path.join(list_file_path, "list_r" + str(j).zfill(3) + ".out")
        with io.open_wt(list_file) as fout:
            fout.writelines("\n".join(map(str, image_list)))
