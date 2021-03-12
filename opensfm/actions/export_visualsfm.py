import logging
import os
import sys

from opensfm import io
from opensfm import transformations as tf
from opensfm.dataset import DataSet, UndistortedDataSet


logger = logging.getLogger(__name__)


def run_dataset(data: DataSet, points, image_list):
    udata = data.undistorted_dataset()

    validate_image_names(data, udata)

    reconstructions = udata.load_undistorted_reconstruction()
    tracks_manager = udata.load_undistorted_tracks_manager()

    export_only = None
    if image_list:
        export_only = {}
        with open(image_list, "r") as f:
            for image in f:
                export_only[image.strip()] = True

    if reconstructions:
        export(reconstructions[0], tracks_manager, udata, points, export_only)


def export(
    reconstruction, tracks_manager, udata: UndistortedDataSet, with_points, export_only
):
    lines = ["NVM_V3", "", str(len(reconstruction.shots))]
    shot_size_cache = {}
    shot_index = {}
    i = 0
    skipped_shots = 0

    for shot in reconstruction.shots.values():
        if export_only is not None and shot.id not in export_only:
            skipped_shots += 1
            continue

        q = tf.quaternion_from_matrix(shot.pose.get_rotation_matrix())
        o = shot.pose.get_origin()

        shot_size_cache[shot.id] = udata.undistorted_image_size(shot.id)
        shot_index[shot.id] = i
        i += 1
        if shot.camera.projection_type == "brown":
            # Will approximate Brown model, not optimal
            focal_normalized = (shot.camera.focal_x + shot.camera.focal_y) / 2.0
        else:
            focal_normalized = shot.camera.focal

        words = [
            image_path(shot.id, udata),
            focal_normalized * max(shot_size_cache[shot.id]),
            q[0],
            q[1],
            q[2],
            q[3],
            o[0],
            o[1],
            o[2],
            "0",
            "0",
        ]
        lines.append(" ".join(map(str, words)))

    # Adjust shots count
    lines[2] = str(int(lines[2]) - skipped_shots)

    if with_points:
        skipped_points = 0
        lines.append("")
        points = reconstruction.points
        lines.append(str(len(points)))
        points_count_index = len(lines) - 1

        for point_id, point in points.items():
            shots = reconstruction.shots
            coord = point.coordinates
            color = list(map(int, point.color))

            view_line = []
            for shot_key, obs in tracks_manager.get_track_observations(
                point_id
            ).items():
                if export_only is not None and shot_key not in export_only:
                    continue

                if shot_key in shots.keys():
                    v = obs.point
                    x = (0.5 + v[0]) * shot_size_cache[shot_key][1]
                    y = (0.5 + v[1]) * shot_size_cache[shot_key][0]
                    view_line.append(
                        " ".join(map(str, [shot_index[shot_key], obs.id, x, y]))
                    )

            if len(view_line) > 1:
                lines.append(
                    " ".join(map(str, coord))
                    + " "
                    + " ".join(map(str, color))
                    + " "
                    + str(len(view_line))
                    + " "
                    + " ".join(view_line)
                )
            else:
                skipped_points += 1

        # Adjust points count
        lines[points_count_index] = str(int(lines[points_count_index]) - skipped_points)
    else:
        lines += ["0", ""]

    lines += ["0", "", "0"]

    with io.open_wt(udata.data_path + "/reconstruction.nvm") as fout:
        fout.write("\n".join(lines))


def image_path(image, udata: UndistortedDataSet):
    """Path to the undistorted image relative to the dataset path."""
    path = udata._undistorted_image_file(image)
    return os.path.relpath(path, udata.data_path)


def validate_image_names(data: DataSet, udata: UndistortedDataSet):
    """Check that image files do not have spaces."""
    for image in data.images():
        filename = image_path(image, udata)
        if " " in filename:
            logger.error(
                'Image name "{}" contains spaces.  '
                "This is not supported by the NVM format.  "
                "Please, rename it before running OpenSfM.".format(filename)
            )
            sys.exit(1)
