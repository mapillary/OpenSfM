#!/usr/bin/env python3

# Snippets to read from the colmap database taken from:
# https://github.com/colmap/colmap/blob/ad7bd93f1a27af7533121aa043a167fe1490688c /
# scripts/python/export_to_bundler.py
# scripts/python/read_write_model.py
# License is that derived from those files.

import argparse
import logging
import math
import os
import sqlite3
from collections import defaultdict
from pathlib import Path
from struct import unpack

import cv2
import matplotlib.pyplot as pl
import numpy as np
import opensfm.actions.undistort as osfm_u
from matplotlib import cm
from mpl_toolkits.axes_grid1 import make_axes_locatable
from opensfm import dataset, features, pygeometry, pymap, types

EXPORT_DIR_NAME = "opensfm_export"
logger = logging.getLogger(__name__)

camera_models = {
    0: ("SIMPLE_PINHOLE", 3),
    1: ("PINHOLE", 4),
    2: ("SIMPLE_RADIAL", 4),
    3: ("RADIAL", 5),
    4: ("OPENCV", 8),
    5: ("OPENCV_FISHEYE", 8),
    6: ("FULL_OPENCV", 12),
    7: ("FOV", 5),
    8: ("SIMPLE_RADIAL_FISHEYE", 4),
    9: ("RADIAL_FISHEYE", 5),
    10: ("THIN_PRISM_FISHEYE", 12),
}


def compute_and_save_undistorted_reconstruction(
    reconstruction, tracks_manager, data, udata
):
    image_format = data.config["undistorted_image_format"]
    urec = types.Reconstruction()
    utracks_manager = pymap.TracksManager()
    undistorted_shots = []
    for shot in reconstruction.shots.values():
        if shot.camera.projection_type == "perspective":
            ucamera = osfm_u.perspective_camera_from_perspective(shot.camera)
        elif shot.camera.projection_type == "brown":
            ucamera = osfm_u.perspective_camera_from_brown(shot.camera)
        elif shot.camera.projection_type == "fisheye":
            ucamera = osfm_u.perspective_camera_from_fisheye(shot.camera)
        else:
            raise ValueError
        urec.add_camera(ucamera)
        ushot = osfm_u.get_shot_with_different_camera(urec, shot, image_format)
        if tracks_manager:
            osfm_u.add_subshot_tracks(tracks_manager, utracks_manager, shot, ushot)
        undistorted_shots.append(ushot)

        image = data.load_image(shot.id, unchanged=True, anydepth=True)
        if image is not None:
            max_size = data.config["undistorted_image_max_size"]
            undistorted = osfm_u.undistort_image(
                shot, undistorted_shots, image, cv2.INTER_AREA, max_size
            )
            for k, v in undistorted.items():
                udata.save_undistorted_image(k, v)

    udata.save_undistorted_reconstruction([urec])
    if tracks_manager:
        udata.save_undistorted_tracks_manager(utracks_manager)

    return urec


def small_colorbar(ax, mappable=None):
    divider = make_axes_locatable(ax)
    cax = divider.append_axes("right", size="5%", pad=0.05)
    pl.colorbar(cax=cax, mappable=mappable)


def depth_colormap(d, cmap=None, invalid_val=0, invalid_color=(0.5, 0.5, 0.5)):
    """
    Colormaps and sets 0 (invalid) values to zero_color
    """
    sm = cm.ScalarMappable(cmap=cm.get_cmap(cmap))
    sm.set_array(d)
    rgb = sm.to_rgba(d)[:, :, :3]
    rgb[d == invalid_val] = invalid_color
    return rgb, sm


def import_cameras_images(db, data):
    cursor = db.cursor()
    cursor.execute(
        "SELECT camera_id, model, width, height, prior_focal_length, params FROM "
        "cameras;"
    )
    cameras = {}
    for row in cursor:
        camera_id, camera_model_id, width, height, prior_focal, params = row
        params = np.fromstring(params, dtype=np.double)
        cam = cam_from_colmap_params(
            camera_model_id, width, height, params, prior_focal
        )
        cam.id = str(camera_id)
        cameras[camera_id] = cam

    data.save_camera_models(cameras)

    images_map = {}
    cursor.execute("SELECT image_id, camera_id, name FROM images;")
    for row in cursor:
        image_id, camera_id, filename = int(row[0]), int(row[1]), row[2]
        images_map[image_id] = (filename, camera_id)
        cam = cameras[camera_id]

        focal_ratio = cam.focal_x if cam.projection_type == "brown" else cam.focal
        exif_data = {
            "make": "unknown",
            "model": "unknown",
            "width": cam.width,
            "height": cam.height,
            "projection_type": cam.projection_type,
            "focal_ratio": focal_ratio,
            "orientation": 1,
            "camera": "{}".format(camera_id),
            "skey": "TheSequence",
            "capture_time": 0.0,
            "gps": {},
        }
        data.save_exif(filename, exif_data)

    cursor.close()
    return cameras, images_map


def pair_id_to_image_ids(pair_id):
    image_id2 = pair_id % 2147483647
    image_id1 = (pair_id - image_id2) // 2147483647
    return image_id1, image_id2


def get_scale_orientation_from_affine(arr):
    # (x, y, a_11, a_12, a_21, a_22)
    a11 = arr[:, 2]
    a12 = arr[:, 3]
    a21 = arr[:, 4]
    a22 = arr[:, 5]
    scale_x = np.sqrt(a11 * a11 + a21 * a21)
    scale_y = np.sqrt(a12 * a12 + a22 * a22)
    orientation = np.arctan2(a21, a11)
    # shear = np.arctan2(-a12, a22) - orientation
    scale = (scale_x + scale_y) / 2
    return scale, orientation


def import_features(db, data, image_map, camera_map):
    cursor = db.cursor()
    cursor.execute("SELECT image_id, rows, cols, data FROM keypoints;")
    keypoints = {}
    colors = {}
    for row in cursor:
        image_id, n_rows, n_cols, arr = row
        filename, camera_id = image_map[image_id]
        cam = camera_map[camera_id]

        arr = np.fromstring(arr, dtype=np.float32).reshape((n_rows, n_cols))

        rgb = data.load_image(filename).astype(np.float32)
        xc = np.clip(arr[:, 1].astype(int), 0, rgb.shape[0] - 1)
        yc = np.clip(arr[:, 0].astype(int), 0, rgb.shape[1] - 1)
        colors[image_id] = rgb[xc, yc, :]

        arr[:, :2] = features.normalized_image_coordinates(
            arr[:, :2], cam.width, cam.height
        )
        if n_cols == 4:
            x, y, s, o = arr[:, 0], arr[:, 1], arr[:, 2], arr[:, 3]
        elif n_cols == 6:
            x, y = arr[:, 0], arr[:, 1]
            s, o = get_scale_orientation_from_affine(arr)
        elif n_cols == 2:
            x, y = arr[:, 0], arr[:, 1]
            s = np.zeros_like(x)
            o = np.zeros_like(x)
        else:
            raise ValueError
        s = s / max(cam.width, cam.height)
        keypoints[image_id] = np.vstack((x, y, s, o)).T

    cursor.execute("SELECT image_id, rows, cols, data FROM descriptors;")
    for row in cursor:
        image_id, n_rows, n_cols, arr = row
        filename, _ = image_map[image_id]
        descriptors = np.fromstring(arr, dtype=np.uint8).reshape((n_rows, n_cols))
        kp = keypoints[image_id]
        features_data = features.FeaturesData(kp, descriptors, colors[image_id], None)
        data.save_features(filename, features_data)

    cursor.close()
    return keypoints


def import_matches(db, data, image_map):
    cursor = db.cursor()
    min_matches = 1
    cursor.execute(
        "SELECT pair_id, data FROM two_view_geometries WHERE rows>=?;", (min_matches,)
    )

    matches_per_im1 = {m[0]: {} for m in image_map.values()}

    for row in cursor:
        pair_id = row[0]
        inlier_matches = np.fromstring(row[1], dtype=np.uint32).reshape(-1, 2)
        image_id1, image_id2 = pair_id_to_image_ids(pair_id)
        image_name1 = image_map[image_id1][0]
        image_name2 = image_map[image_id2][0]
        matches_per_im1[image_name1][image_name2] = inlier_matches

    for image_name1, matches in matches_per_im1.items():
        data.save_matches(image_name1, matches)

    cursor.close()


def import_cameras_reconstruction(path_cameras, rec):
    """
    Imports cameras from a COLMAP reconstruction cameras.bin file
    """
    logger.info("Importing cameras from {}".format(path_cameras))
    with open(path_cameras, "rb") as f:
        n_cameras = unpack("<Q", f.read(8))[0]
        for _ in range(n_cameras):
            camera_id = unpack("<i", f.read(4))[0]
            camera_model_id = unpack("<i", f.read(4))[0]
            width = unpack("<Q", f.read(8))[0]
            height = unpack("<Q", f.read(8))[0]
            params = []
            n_params = camera_models[camera_model_id][1]
            for _ in range(n_params):
                params.append(unpack("<d", f.read(8))[0])
            cam = cam_from_colmap_params(camera_model_id, width, height, params)
            cam.id = str(camera_id)
            rec.add_camera(cam)


def cam_from_colmap_params(camera_model_id, width, height, params, prior_focal=1):
    """
    Helper function to map from colmap parameters to an OpenSfM camera
    """
    mapping = {1: "pinhole", 3: "perspective", 9: "fisheye"}
    if camera_model_id not in mapping.keys():
        raise ValueError("Not supported: " + camera_models[camera_model_id][0])
    projection_type = mapping[camera_model_id]
    normalizer = max(width, height)
    focal = params[0] / normalizer if prior_focal else 0.85
    if projection_type == "perspective":
        cam = pygeometry.Camera.create_perspective(focal, params[3], params[4])
    elif projection_type == "pinhole":
        cam = pygeometry.Camera.create_perspective(focal, 0, 0)
    else:  # projection_type == 'fisheye'
        cam = pygeometry.Camera.create_fisheye(focal, params[3], 0)
    cam.width = width
    cam.height = height
    return cam


def import_points_reconstruction(path_points, rec):
    logger.info("Importing points from {}".format(path_points))
    with open(path_points, "rb") as f:
        n_points = unpack("<Q", f.read(8))[0]
        for _ in range(n_points):
            pid = unpack("<Q", f.read(8))[0]
            x = unpack("<d", f.read(8))[0]
            y = unpack("<d", f.read(8))[0]
            z = unpack("<d", f.read(8))[0]
            r = unpack("<B", f.read(1))[0]
            g = unpack("<B", f.read(1))[0]
            b = unpack("<B", f.read(1))[0]
            _ = unpack("<d", f.read(8))[0]  # error
            track_len = unpack("<Q", f.read(8))[0]
            # Ignore track info
            f.seek(8 * track_len, 1)
            p = rec.create_point(str(pid), (x, y, z))
            p.color = (r, g, b)


def read_colmap_ply(path_ply):
    """
    Reads the ply output from COLMAP.
    This is not a generic ply binary reader but a quick hack to read only this file
    """
    logger.info("Reading fused pointcloud {}".format(path_ply))
    header_should_be = [
        "ply\n",
        "format binary_little_endian 1.0\n",
        "element vertex\n",
        "property float x\n",
        "property float y\n",
        "property float z\n",
        "property float nx\n",
        "property float ny\n",
        "property float nz\n",
        "property uchar red\n",
        "property uchar green\n",
        "property uchar blue\n",
        "end_header\n",
    ]
    properties = [
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("nx", "<f4"),
        ("ny", "<f4"),
        ("nz", "<f4"),
        ("red", "<u1"),
        ("green", "<u1"),
        ("blue", "<u1"),
    ]

    n_vertices = 0
    with open(path_ply, "rb") as f:
        header = []
        for line in f:
            line = line.decode()
            if line.startswith("element vertex"):
                n_vertices = int(line.strip().split()[-1])
                line = "element vertex\n"
            header.append(line)
            if line == header_should_be[-1]:
                break

        assert header == header_should_be
        data = np.fromfile(f, dtype=properties, count=n_vertices)

    points, normals, colors = [], [], []
    for row in data:
        points.append(np.array([row[0], row[1], row[2]]))
        normals.append(np.array([row[3], row[4], row[5]]))
        colors.append(np.array([row[6], row[7], row[8]]))
    return np.array(points), np.array(normals), np.array(colors)


def import_images_reconstruction(path_images, keypoints, rec):
    """
    Read images.bin, building shots and tracks graph
    """
    logger.info("Importing images from {}".format(path_images))
    tracks_manager = pymap.TracksManager()
    image_ix_to_shot_id = {}
    with open(path_images, "rb") as f:
        n_ims = unpack("<Q", f.read(8))[0]
        for image_ix in range(n_ims):
            image_id = unpack("<I", f.read(4))[0]
            q0 = unpack("<d", f.read(8))[0]
            q1 = unpack("<d", f.read(8))[0]
            q2 = unpack("<d", f.read(8))[0]
            q3 = unpack("<d", f.read(8))[0]
            t0 = unpack("<d", f.read(8))[0]
            t1 = unpack("<d", f.read(8))[0]
            t2 = unpack("<d", f.read(8))[0]
            camera_id = unpack("<I", f.read(4))[0]
            filename = ""
            while True:
                c = f.read(1).decode()
                if c == "\0":
                    break
                filename += c
            q = np.array([q0, q1, q2, q3])
            q /= np.linalg.norm(q)
            t = np.array([t0, t1, t2])

            pose = pygeometry.Pose(rotation=quaternion_to_angle_axis(q), translation=t)
            shot = rec.create_shot(filename, str(camera_id), pose)
            image_ix_to_shot_id[image_ix] = shot.id

            n_points_2d = unpack("<Q", f.read(8))[0]
            for point2d_ix in range(n_points_2d):
                x = unpack("<d", f.read(8))[0]
                y = unpack("<d", f.read(8))[0]
                point3d_id = unpack("<Q", f.read(8))[0]
                if point3d_id != np.iinfo(np.uint64).max:
                    kp = keypoints[image_id][point2d_ix]
                    r, g, b = rec.points[str(point3d_id)].color
                    obs = pymap.Observation(
                        x,
                        y,
                        kp[2],
                        int(r),
                        int(g),
                        int(b),
                        point2d_ix,
                    )
                    tracks_manager.add_observation(shot.id, str(point3d_id), obs)

    return tracks_manager, image_ix_to_shot_id


def read_vis(path_vis, image_ix_to_shot_id):
    logger.info("Reading visibility file {}".format(path_vis))
    points_seen = defaultdict(list)
    with open(path_vis, "rb") as f:
        n_points = unpack("<Q", f.read(8))[0]
        for point_ix in range(n_points):
            n_images = unpack("<I", f.read(4))[0]
            for _ in range(n_images):
                image_ix = unpack("<I", f.read(4))[0]
                shot_id = image_ix_to_shot_id[image_ix]
                points_seen[shot_id].append(point_ix)
    for ixs in points_seen.values():
        assert len(ixs) == len(set(ixs))
    return points_seen


def import_depthmaps_from_fused_pointcloud(udata, urec, image_ix_to_shot_id, path_ply):
    """
    Imports the depthmaps by reprojecting the fused pointcloud
    """
    # Read ply
    points, normals, colors = read_colmap_ply(path_ply)

    # Read visibility file
    points_seen = read_vis(path_ply.with_suffix(".ply.vis"), image_ix_to_shot_id)

    # Project to shots and save as depthmaps
    max_size = udata.config["depthmap_resolution"]
    for shot_id, points_seen_ixs in points_seen.items():
        logger.info("Projecting shot {}".format(shot_id))
        project_pointcloud_save_depth(
            udata, urec, points[points_seen_ixs], shot_id, max_size
        )


def project_pointcloud_save_depth(udata, urec, points, shot_id, max_sz):
    # Project points to the undistorted image
    shot = urec.shots[shot_id]
    w, h = shot.camera.width, shot.camera.height
    large = max(w, h)
    if large > max_sz:
        ar = w / h
        if w > h:
            w = max_sz
            h = int(w / ar)
        else:
            h = max_sz
            w = int(ar * h)

    points_2d = shot.project_many(points)

    pixel_coords = features.denormalized_image_coordinates(points_2d, w, h).astype(int)
    # Filter out points that fall out of the image
    # <<< aren't we supposed to have points that are visible from this image only??!?!
    mask = np.ones(pixel_coords.shape[0], dtype=bool)
    mask[pixel_coords[:, 0] < 0] = 0
    mask[pixel_coords[:, 1] < 0] = 0
    mask[pixel_coords[:, 0] >= w] = 0
    mask[pixel_coords[:, 1] >= h] = 0
    pixel_coords = pixel_coords[mask]

    # Compute the depth
    distances = np.linalg.norm(points - shot.pose.get_origin(), axis=1)
    viewing_angles = np.arctan2(np.linalg.norm(points_2d, axis=1), shot.camera.focal)
    depths = distances * np.cos(viewing_angles)
    depths[depths > udata.config["depthmap_max_depth"]] = 0

    # Create depth image
    depth_image = np.zeros([h, w])
    depth_image[pixel_coords[:, 1], pixel_coords[:, 0]] = depths[mask]

    # Save numpy
    filepath = Path(udata.depthmap_file(shot_id, "clean.npz"))
    filepath.parent.mkdir(exist_ok=True, parents=True)
    np.savez_compressed(
        filepath, depth=depth_image, plane=np.zeros(1), score=np.zeros(1)
    )

    # Save jpg for visualization
    import matplotlib.pyplot as plt

    fig = plt.figure()
    rgb, sm = depth_colormap(depth_image)
    plt.imshow(rgb)
    small_colorbar(plt.gca(), mappable=sm)
    filepath = Path(udata.data_path) / "plot_depthmaps" / "{}.png".format(shot_id)
    filepath.parent.mkdir(exist_ok=True, parents=True)
    plt.savefig(filepath, dpi=300)
    plt.close(fig)


def quaternion_to_angle_axis(quaternion):
    if quaternion[0] > 1:
        quaternion = quaternion / np.linalg.norm(quaternion)
    qw, qx, qy, qz = quaternion
    s = max(0.001, math.sqrt(1 - qw * qw))
    x = qx / s
    y = qy / s
    z = qz / s
    angle = 2 * math.acos(qw)
    return [angle * x, angle * y, angle * z]


def main():
    parser = argparse.ArgumentParser(
        description="Convert COLMAP database to OpenSfM dataset"
    )
    parser.add_argument("database", help="path to the database to be processed")
    parser.add_argument("images", help="path to the images")
    args = parser.parse_args()
    logger.info(f"Converting {args.database} to COLMAP format")

    p_db = Path(args.database)
    assert p_db.is_file()

    export_folder = p_db.parent / EXPORT_DIR_NAME
    export_folder.mkdir(exist_ok=True)
    images_path = export_folder / "images"
    if not images_path.exists():
        os.symlink(os.path.abspath(args.images), images_path, target_is_directory=True)

    # Copy the config if this is an colmap export of an opensfm export
    if (
        p_db.parent.name == "colmap_export"
        and not (export_folder / "config.yaml").exists()
    ):
        os.symlink(p_db.parent.parent / "config.yaml", export_folder / "config.yaml")

    data = dataset.DataSet(export_folder)
    db = sqlite3.connect(p_db.as_posix())
    camera_map, image_map = import_cameras_images(db, data)

    # Create image_list.txt
    with open(export_folder / "image_list.txt", "w") as f:
        for _, (filename, _) in image_map.items():
            f.write("images/" + filename + "\n")
    data.load_image_list()

    keypoints = import_features(db, data, image_map, camera_map)
    import_matches(db, data, image_map)

    rec_cameras = p_db.parent / "cameras.bin"
    rec_points = p_db.parent / "points3D.bin"
    rec_images = p_db.parent / "images.bin"
    if rec_cameras.exists() and rec_images.exists() and rec_points.exists():
        reconstruction = types.Reconstruction()
        import_cameras_reconstruction(rec_cameras, reconstruction)
        import_points_reconstruction(rec_points, reconstruction)
        tracks_manager, _ = import_images_reconstruction(
            rec_images, keypoints, reconstruction
        )

        data.save_reconstruction([reconstruction])
        data.save_tracks_manager(tracks_manager)

        # Save undistorted reconstruction as well
        udata = dataset.UndistortedDataSet(data, io_handler=data.io_handler)
        urec = compute_and_save_undistorted_reconstruction(
            reconstruction, tracks_manager, data, udata
        )

        # Project colmap's fused pointcloud to save depths in opensfm format
        path_ply = p_db.parent / "dense/fused.ply"
        if path_ply.is_file():
            rec_cameras = p_db.parent / "dense/sparse/cameras.bin"
            rec_images = p_db.parent / "dense/sparse/images.bin"
            rec_points = p_db.parent / "points3D.bin"
            reconstruction = types.Reconstruction()
            import_cameras_reconstruction(rec_cameras, reconstruction)
            import_points_reconstruction(rec_points, reconstruction)
            _, image_ix_to_shot_id = import_images_reconstruction(
                rec_images, keypoints, reconstruction
            )
            logger.info(f"Projecting {path_ply} to depth images")
            import_depthmaps_from_fused_pointcloud(
                udata, urec, image_ix_to_shot_id, path_ply
            )
        else:
            logger.info(
                "Not importing dense reconstruction: Didn't find {}".format(path_ply)
            )

    else:
        logger.info(
            "Didn't find some of the reconstruction files at {}".format(p_db.parent)
        )

    db.close()


if __name__ == "__main__":
    main()
