# Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#
#     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
#       its contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Johannes L. Schoenberger (jsch at inf.ethz.ch)

# This script is based on an original implementation by True Price.

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import logging
import math
import os
import sqlite3
import sys
from struct import pack

import numpy as np

from opensfm import dataset
from opensfm import features
from opensfm import io
from opensfm import matching

logger = logging.getLogger(__name__)


class Command:
    name = 'export_colmap'
    help = "Export reconstruction to colmap format"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('--binary', help='export using binary format', action='store_true')

    def run(self, args):
        data = dataset.DataSet(args.dataset)

        export_folder = os.path.join(data.data_path, 'colmap_export')
        io.mkdir_p(export_folder)

        database_path = os.path.join(export_folder, 'colmap_database.db')
        images_path = os.path.join(data.data_path, 'images')

        if os.path.exists(database_path):
            os.remove(database_path)
        db = COLMAPDatabase.connect(database_path)
        db.create_tables()

        images_map, camera_map = export_cameras(data, db)
        features_map = export_features(data, db, images_map)
        export_matches(data, db, features_map, images_map)

        if data.reconstruction_exists():
            export_ini_file(export_folder, database_path, images_path)
            export_cameras_reconstruction(data, export_folder, camera_map, args.binary)
            points_map = export_points_reconstruction(data, export_folder, images_map, args.binary)
            export_images_reconstruction(data, export_folder, camera_map, images_map,
                                         features_map, points_map, args.binary)
        db.commit()
        db.close()


IS_PYTHON3 = sys.version_info[0] >= 3

MAX_IMAGE_ID = 2**31 - 1

CREATE_CAMERAS_TABLE = """CREATE TABLE IF NOT EXISTS cameras (
    camera_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    model INTEGER NOT NULL,
    width INTEGER NOT NULL,
    height INTEGER NOT NULL,
    params BLOB,
    prior_focal_length INTEGER NOT NULL)"""

CREATE_DESCRIPTORS_TABLE = """CREATE TABLE IF NOT EXISTS descriptors (
    image_id INTEGER PRIMARY KEY NOT NULL,
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    data BLOB,
    FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE)"""

CREATE_IMAGES_TABLE = """CREATE TABLE IF NOT EXISTS images (
    image_id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
    name TEXT NOT NULL UNIQUE,
    camera_id INTEGER NOT NULL,
    prior_qw REAL,
    prior_qx REAL,
    prior_qy REAL,
    prior_qz REAL,
    prior_tx REAL,
    prior_ty REAL,
    prior_tz REAL,
    CONSTRAINT image_id_check CHECK(image_id >= 0 and image_id < {}),
    FOREIGN KEY(camera_id) REFERENCES cameras(camera_id))
""".format(MAX_IMAGE_ID)

CREATE_TWO_VIEW_GEOMETRIES_TABLE = """
CREATE TABLE IF NOT EXISTS two_view_geometries (
    pair_id INTEGER PRIMARY KEY NOT NULL,
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    data BLOB,
    config INTEGER NOT NULL,
    F BLOB,
    E BLOB,
    H BLOB)
"""

CREATE_KEYPOINTS_TABLE = """CREATE TABLE IF NOT EXISTS keypoints (
    image_id INTEGER PRIMARY KEY NOT NULL,
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    data BLOB,
    FOREIGN KEY(image_id) REFERENCES images(image_id) ON DELETE CASCADE)
"""

CREATE_MATCHES_TABLE = """CREATE TABLE IF NOT EXISTS matches (
    pair_id INTEGER PRIMARY KEY NOT NULL,
    rows INTEGER NOT NULL,
    cols INTEGER NOT NULL,
    data BLOB)"""

CREATE_NAME_INDEX = \
    "CREATE UNIQUE INDEX IF NOT EXISTS index_name ON images(name)"

CREATE_ALL = "; ".join([
    CREATE_CAMERAS_TABLE,
    CREATE_IMAGES_TABLE,
    CREATE_KEYPOINTS_TABLE,
    CREATE_DESCRIPTORS_TABLE,
    CREATE_MATCHES_TABLE,
    CREATE_TWO_VIEW_GEOMETRIES_TABLE,
    CREATE_NAME_INDEX
])


def image_ids_to_pair_id(image_id1, image_id2):
    if image_id1 > image_id2:
        image_id1, image_id2 = image_id2, image_id1
    return image_id1 * MAX_IMAGE_ID + image_id2


def pair_id_to_image_ids(pair_id):
    image_id2 = pair_id % MAX_IMAGE_ID
    image_id1 = (pair_id - image_id2) // MAX_IMAGE_ID
    return image_id1, image_id2


def array_to_blob(array):
    if IS_PYTHON3:
        return array.tostring()
    else:
        return np.getbuffer(array)


def blob_to_array(blob, dtype, shape=(-1,)):
    if IS_PYTHON3:
        return np.fromstring(blob, dtype=dtype).reshape(*shape)
    else:
        return np.frombuffer(blob, dtype=dtype).reshape(*shape)


class COLMAPDatabase(sqlite3.Connection):

    @staticmethod
    def connect(database_path):
        return sqlite3.connect(database_path, factory=COLMAPDatabase)

    def __init__(self, *args, **kwargs):
        super(COLMAPDatabase, self).__init__(*args, **kwargs)

        self.create_tables = lambda: self.executescript(CREATE_ALL)
        self.create_cameras_table = \
            lambda: self.executescript(CREATE_CAMERAS_TABLE)
        self.create_descriptors_table = \
            lambda: self.executescript(CREATE_DESCRIPTORS_TABLE)
        self.create_images_table = \
            lambda: self.executescript(CREATE_IMAGES_TABLE)
        self.create_two_view_geometries_table = \
            lambda: self.executescript(CREATE_TWO_VIEW_GEOMETRIES_TABLE)
        self.create_keypoints_table = \
            lambda: self.executescript(CREATE_KEYPOINTS_TABLE)
        self.create_matches_table = \
            lambda: self.executescript(CREATE_MATCHES_TABLE)
        self.create_name_index = lambda: self.executescript(CREATE_NAME_INDEX)

    def add_camera(self, model, width, height, params,
                   prior_focal_length=False, camera_id=None):
        params = np.asarray(params, np.float64)
        cursor = self.execute(
            "INSERT INTO cameras VALUES (?, ?, ?, ?, ?, ?)",
            (camera_id, model, width, height, array_to_blob(params),
             prior_focal_length))
        return cursor.lastrowid

    def add_image(self, name, camera_id,
                  prior_q=np.zeros(4), prior_t=np.zeros(3), image_id=None):
        cursor = self.execute(
            "INSERT INTO images VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (image_id, name, camera_id, prior_q[0], prior_q[1], prior_q[2],
             prior_q[3], prior_t[0], prior_t[1], prior_t[2]))
        return cursor.lastrowid

    def add_keypoints(self, image_id, keypoints):
        assert(len(keypoints.shape) == 2)
        assert(keypoints.shape[1] in [2, 4, 6])

        keypoints = np.asarray(keypoints, np.float32)
        self.execute(
            "INSERT INTO keypoints VALUES (?, ?, ?, ?)",
            (image_id,) + keypoints.shape + (array_to_blob(keypoints),))

    def add_descriptors(self, image_id, descriptors):
        descriptors = np.ascontiguousarray(descriptors, np.uint8)
        self.execute(
            "INSERT INTO descriptors VALUES (?, ?, ?, ?)",
            (image_id,) + descriptors.shape + (array_to_blob(descriptors),))

    def add_matches(self, image_id1, image_id2, matches):
        assert(len(matches.shape) == 2)
        assert(matches.shape[1] == 2)

        if image_id1 > image_id2:
            matches = matches[:, ::-1]

        pair_id = image_ids_to_pair_id(image_id1, image_id2)
        matches = np.asarray(matches, np.uint32)
        self.execute(
            "INSERT INTO matches VALUES (?, ?, ?, ?)",
            (pair_id,) + matches.shape + (array_to_blob(matches),))

    def add_two_view_geometry(self, image_id1, image_id2, matches,
                              F=np.eye(3), E=np.eye(3), H=np.eye(3), config=2):
        assert(len(matches.shape) == 2)
        assert(matches.shape[1] == 2)

        if image_id1 > image_id2:
            matches = matches[:, ::-1]

        pair_id = image_ids_to_pair_id(image_id1, image_id2)
        matches = np.asarray(matches, np.uint32)
        F = np.asarray(F, dtype=np.float64)
        E = np.asarray(E, dtype=np.float64)
        H = np.asarray(H, dtype=np.float64)
        self.execute(
            "INSERT INTO two_view_geometries VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            (pair_id,) + matches.shape + (array_to_blob(matches), config,
                                          array_to_blob(F), array_to_blob(E), array_to_blob(H)))


COLMAP_TYPES_MAP = {'brown': 'FULL_OPENCV', 'perspective': 'RADIAL', 'fisheye': 'RADIAL_FISHEYE'}
COLMAP_ID_MAP = {'brown': 6, 'perspective': 3, 'fisheye': 9}


def export_cameras(data, db):
    camera_map = {}
    for camera_model, camera in data.load_camera_models().items():
        if data.camera_models_overrides_exists():
            overrides = data.load_camera_models_overrides()
            if camera_model in overrides:
                camera = overrides[camera_model]

        normalizer = max(camera.width, camera.height)

        if camera.projection_type == 'perspective':
            parameters = np.array((camera.focal*normalizer,
                                   camera.width/2, camera.height/2,
                                   camera.k1, camera.k2))
        if camera.projection_type == 'brown':
            parameters = np.array((camera.focal_x*normalizer,
                                   camera.focal_y*normalizer,
                                   (camera.width-1)*0.5 + camera.c_x*normalizer,
                                   (camera.height-1)*0.5 + camera.c_y*normalizer,
                                   camera.k1, camera.k2,
                                   camera.p1, camera.p2,
                                   camera.k3, 0.0, 0.0, 0.0))
        if camera.projection_type == 'fisheye':
            parameters = np.array((camera.focal*normalizer,
                                   camera.width/2, camera.height/2,
                                   camera.k1, camera.k2))
        camera_id = db.add_camera(COLMAP_ID_MAP[camera.projection_type],
                                  camera.width, camera.height,
                                  parameters)
        camera_map[camera_model] = camera_id

    images_map = {}
    for image in data.images():
        camera_model = data.load_exif(image)['camera']
        image_id = db.add_image(image, camera_map[camera_model])
        images_map[image] = image_id

    return images_map, camera_map


def export_features(data, db, images_map):
    features_map = {}
    for image in data.images():
        width = data.load_exif(image)['width']
        height = data.load_exif(image)['height']
        feat, _, _ = data.load_features(image)
        feat = features.denormalized_image_coordinates(feat, width, height)
        features_map[image] = feat
        db.add_keypoints(images_map[image], feat)
    return features_map


def export_matches(data, db, features_map, images_map):
    matches_per_pair = {}
    for image1 in data.images():
        matches = data.load_matches(image1)
        for image2, image_matches in matches.items():
            pair_key = (min(image1, image2), max(image1, image2))
            pair_matches = matches_per_pair.setdefault(pair_key, {})
            for match in image_matches:
                if image1 < image2:
                    pair_matches.update({(match[0], match[1]): True})
                else:
                    pair_matches.update({(match[1], match[0]): True})

    data.config['robust_matching_threshold'] = 8
    for pair, matches in matches_per_pair.items():
        matches_numpy = np.array([np.array([m[0], m[1]]) for m in matches])
        if len(matches_numpy) < 10:
            continue
        F, inliers = matching.robust_match_fundamental(features_map[pair[0]],
                                                       features_map[pair[1]],
                                                       matches_numpy, data.config)
        if len(inliers) > 10:
            db.add_two_view_geometry(images_map[pair[0]], images_map[pair[1]],
                                     inliers, F=F)
            db.add_matches(images_map[pair[0]], images_map[pair[1]], inliers)


def export_cameras_reconstruction(data, path, camera_map, binary=False):
    reconstructions = data.load_reconstruction()
    cameras = {}
    for reconstruction in reconstructions:
        for camera_id, camera in reconstruction.cameras.items():
            cameras[camera_id] = camera

    if binary:
        fout = open(os.path.join(path, 'cameras.bin'), 'wb')
        fout.write(pack('<Q', len(cameras)))
    else:
        fout = io.open_wt(os.path.join(path, 'cameras.txt'))

    for camera_id, camera in cameras.items():
        w = camera.width
        h = camera.height
        normalizer = max(w, h)
        colmap_id = camera_map[camera_id]
        colmap_type = COLMAP_TYPES_MAP[camera.projection_type]
        if camera.projection_type == 'perspective':
            f = camera.focal*normalizer
            k1 = camera.k1
            k2 = camera.k2
            if binary:
                fout.write(pack('<2i', colmap_id, COLMAP_ID_MAP[camera.projection_type]))
                fout.write(pack('<2Q', w, h))
                fout.write(pack('<5d', f, w*0.5, h*0.5, k1, k2))
            else:
                fout.write('%d %s %d %d %f %f %f %f %f\n' %
                           (colmap_id, colmap_type, w, h, f, w*0.5, h*0.5, k1, k2))
        elif camera.projection_type == 'brown':
            f_x = camera.focal_x*normalizer
            f_y = camera.focal_y*normalizer
            c_x = (w-1)*0.5 + normalizer*camera.c_x
            c_y = (h-1)*0.5 + normalizer*camera.c_y
            k1 = camera.k1
            k2 = camera.k2
            k3 = camera.k3
            p1 = camera.p1
            p2 = camera.p2
            if binary:
                fout.write(pack('<2i', colmap_id, COLMAP_ID_MAP[camera.projection_type]))
                fout.write(pack('<2Q', w, h))
                fout.write(pack('<12d', f_x, f_y, c_x, c_y, k1, k2, p1, p2, k3, 0.0, 0.0, 0.0))
            else:
                fout.write('%d %s %d %d %f %f %f %f %f %f %f %f %f %f %f %f\n' %
                           (colmap_id, colmap_type, w, h, f_x, f_y, c_x, c_y,
                           k1, k2, p1, p2, k3,
                           0.0, 0.0, 0.0))
        elif camera.projection_type == 'fisheye':
            f = camera.focal*normalizer
            k1 = camera.k1
            k2 = camera.k2
            if binary:
                fout.write(pack('<2i', colmap_id, COLMAP_ID_MAP[camera.projection_type]))
                fout.write(pack('<2Q', w, h))
                fout.write(pack('<5d', f, w*0.5, h*0.5, k1, k2))
            else:
                fout.write('%d %s %d %d %f %f %f %f %f\n' %
                           (colmap_id, colmap_type, w, h, f, w*0.5, h*0.5, k1, k2))
    fout.close()


def export_images_reconstruction(data, path, camera_map, images_map,
                                 features_map, points_map, binary=False):
    reconstructions = data.load_reconstruction()
    tracks_manager = data.load_tracks_manager()

    if binary:
        fout = open(os.path.join(path, 'images.bin'), 'wb')
        n_ims = 0
        for reconstruction in reconstructions:
            n_ims += len(reconstruction.shots)
        fout.write(pack('<Q', n_ims))
    else:
        fout = io.open_wt(os.path.join(path, 'images.txt'))

    for reconstruction in reconstructions:

        for shot_id, shot in reconstruction.shots.items():
            colmap_camera_id = camera_map[shot.camera.id]
            colmap_shot_id = images_map[shot_id]

            t = shot.pose.translation
            q = angle_axis_to_quaternion(shot.pose.rotation)

            if binary:
                fout.write(pack('<I', colmap_shot_id))
                fout.write(pack('<7d', *(list(q) + list(t))))
                fout.write(pack('<I', colmap_camera_id))
                for char in shot_id:
                    fout.write(pack('<c', char.encode("utf-8")))
                fout.write(pack('<c', b"\x00"))
            format_line = '%d %f %f %f %f %f %f %f %d %s\n'
            format_tuple = [colmap_shot_id,
                            q[0], q[1], q[2], q[3],
                            t[0], t[1], t[2],
                            colmap_camera_id, shot_id]

            point_per_feat = {obs.id: k for k, obs in tracks_manager.get_shot_observations(shot_id).items()}

            points_tuple = []
            for feature_id in range(len(features_map[shot_id])):
                colmap_point_id = -1
                if feature_id in point_per_feat:
                    point_id = point_per_feat[feature_id]
                    if point_id in points_map:
                        colmap_point_id = points_map[point_id]

                if colmap_point_id != -1:
                    x, y = features_map[shot_id][feature_id]
                    format_line += '%f %f %d '
                    points_tuple += [x, y, colmap_point_id]
            format_line += '\n'

            if binary:
                fout.write(pack('<Q', len(points_tuple) // 3))
                for i in range(0, len(points_tuple), 3):
                    x, y, colmap_point_id = points_tuple[i:i + 3]
                    fout.write(pack('<2d', x, y))
                    fout.write(pack('<Q', colmap_point_id))
            else:
                fout.write(format_line % tuple(format_tuple + points_tuple))
    fout.close()


def export_points_reconstruction(data, path, images_map, binary=False):
    reconstructions = data.load_reconstruction()
    tracks_manager = data.load_tracks_manager()

    points_map = {}

    if binary:
        fout = open(os.path.join(path, 'points3D.bin'), 'wb')
        n_points = 0
        for reconstruction in reconstructions:
            n_points += len(reconstruction.points)
        fout.write(pack('<Q', n_points))
    else:
        fout = io.open_wt(os.path.join(path, 'points3D.txt'))

    i = 0
    for reconstruction in reconstructions:
        for point in reconstruction.points.values():
            c = point.coordinates
            cl = point.color
            format_line = '%d %f %f %f %d %d %d %f '
            format_tuple = [int(i), c[0], c[1], c[2],
                            int(cl[0]), int(cl[1]), int(cl[2]), 0.0]

            if binary:
                fout.write(pack('<Q', int(i)))
                fout.write(pack('<3d', c[0], c[1], c[2])) # Position
                fout.write(pack('<3B', *[int(i) for i in cl])) # Color
                fout.write(pack('<d', 0.)) # Error

            track_tuple = []
            for image, obs in tracks_manager.get_track_observations(point.id).items():
                if image not in reconstruction.shots:
                    continue
                format_line += '%d %d '
                track_tuple += [images_map[image], obs.id]
            format_line += '\n'

            if binary:
                fout.write(pack('<Q', len(track_tuple) // 2)) # Track length
                for el in track_tuple:
                    fout.write(pack('<i', el)) # Track
            else:
                fout.write(format_line % tuple(format_tuple + track_tuple))
            points_map[point.id] = i
            i += 1
    fout.close()
    return points_map


def angle_axis_to_quaternion(angle_axis):
    angle = np.linalg.norm(angle_axis)

    x = angle_axis[0]/angle
    y = angle_axis[1]/angle
    z = angle_axis[2]/angle

    qw = math.cos(angle/2.0)
    qx = x * math.sqrt(1-qw*qw)
    qy = y * math.sqrt(1-qw*qw)
    qz = z * math.sqrt(1-qw*qw)

    return [qw, qx, qy, qz]


def export_ini_file(path, db_path, images_path):
    with io.open_wt(os.path.join(path, 'project.ini')) as fout:
        fout.write('log_to_stderr=false\nlog_level=2\n')
        fout.write('database_path=%s\n' % db_path)
        fout.write('image_path=%s\n' % images_path)
