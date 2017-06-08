# -*- coding: utf-8 -*-

import os
import json
import errno
import pickle
import gzip
import numpy as np
import networkx as nx
import cv2

from opensfm import io
from opensfm import config
from opensfm import context


class DataSet:
    """
    Dataset representing directory with images, extracted , feature descriptors (SURF, SIFT), etc.

    Methods to retrieve *base directory* for data file(s) have suffix ``_path``, methods to retrieve path of specified
    data file have suffix ``_file``.
    """
    def __init__(self, data_path):
        """
        Create dataset instance. Empty directories (for EXIF, matches, etc) will be created if they don't exist
        already.

        :param data_path: Path to directory containing dataset
        """
        self.data_path = data_path

        self._load_config()

        # Load list of images.
        image_list_file = os.path.join(self.data_path, 'image_list.txt')
        if os.path.isfile(image_list_file):
            with open(image_list_file) as fin:
                lines = fin.read().splitlines()
            self.set_image_list(lines)
        else:
            self.set_image_path(os.path.join(self.data_path, 'images'))

        # Load list of masks if they exist.
        mask_list_file = os.path.join(self.data_path, 'mask_list.txt')
        if os.path.isfile(mask_list_file):
            with open(mask_list_file) as fin:
                lines = fin.read().splitlines()
            self.set_mask_list(lines)
        else:
            self.set_mask_path(os.path.join(self.data_path, 'masks'))

    def _load_config(self):
        config_file = os.path.join(self.data_path, 'config.yaml')
        self.config = config.load_config(config_file)

    def images(self):
        """Return list of file names of all images in this dataset"""
        return self.image_list

    def __image_file(self, image):
        """
        Return path of image with given name
        :param image: Image file name (**with extension**)
        """
        return self.image_files[image]

    def load_image(self, image):
        return open(self.__image_file(image), 'rb')

    def image_as_array(self, image):
        """Return image pixels as 3-dimensional numpy array (R G B order)"""
        return io.imread(self.__image_file(image))

    def _undistorted_image_path(self):
        return os.path.join(self.data_path, 'undistorted')

    def _undistorted_image_file(self, image):
        """Path of undistorted version of an image."""
        return os.path.join(self._undistorted_image_path(), image + '.jpg')

    def undistorted_image_as_array(self, image):
        """Undistorted image pixels as 3-dimensional numpy array (R G B order)"""
        return io.imread(self._undistorted_image_file(image))

    def save_undistorted_image(self, image, array):
        io.mkdir_p(self._undistorted_image_path())
        cv2.imwrite(self._undistorted_image_file(image), array[:, :, ::-1])

    def masks(self):
        """Return list of file names of all masks in this dataset"""
        return self.mask_list

    def mask_as_array(self, image):
        """Given an image, returns the associated mask as an array if it exists, otherwise returns None"""
        mask_name = image + '.png'
        if mask_name in self.masks():
            mask_path = self.mask_files[mask_name]
            mask = cv2.imread(mask_path)
            if len(mask.shape) == 3:
                mask = mask.max(axis=2)
        else:
            mask = None
        return mask

    def _depthmap_path(self):
        return os.path.join(self.data_path, 'depthmaps')

    def _depthmap_file(self, image, suffix):
        """Path to the depthmap file"""
        return os.path.join(self._depthmap_path(), image + '.' + suffix)

    def raw_depthmap_exists(self, image):
        return os.path.isfile(self._depthmap_file(image, 'raw.npz'))

    def save_raw_depthmap(self, image, depth, plane, score, nghbr, nghbrs):
        io.mkdir_p(self._depthmap_path())
        filepath = self._depthmap_file(image, 'raw.npz')
        np.savez_compressed(filepath, depth=depth, plane=plane, score=score, nghbr=nghbr, nghbrs=nghbrs)

    def load_raw_depthmap(self, image):
        o = np.load(self._depthmap_file(image, 'raw.npz'))
        return o['depth'], o['plane'], o['score'], o['nghbr'], o['nghbrs']

    def clean_depthmap_exists(self, image):
        return os.path.isfile(self._depthmap_file(image, 'clean.npz'))

    def save_clean_depthmap(self, image, depth, plane, score):
        io.mkdir_p(self._depthmap_path())
        filepath = self._depthmap_file(image, 'clean.npz')
        np.savez_compressed(filepath, depth=depth, plane=plane, score=score)

    def load_clean_depthmap(self, image):
        o = np.load(self._depthmap_file(image, 'clean.npz'))
        return o['depth'], o['plane'], o['score']

    @staticmethod
    def __is_image_file(filename):
        return filename.split('.')[-1].lower() in {'jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif'}

    def set_image_path(self, path):
        """Set image path and find all images in there"""
        self.image_list = []
        self.image_files = {}
        if os.path.exists(path):
            for name in os.listdir(path):
                if self.__is_image_file(name):
                    self.image_list.append(name)
                    self.image_files[name] = os.path.join(path, name)

    def set_image_list(self, image_list):
            self.image_list = []
            self.image_files = {}
            for line in image_list:
                path = os.path.join(self.data_path, line)
                name = os.path.basename(path)
                self.image_list.append(name)
                self.image_files[name] = path

    @staticmethod
    def __is_mask_file(filename):
        return DataSet.__is_image_file(filename)

    def set_mask_path(self, path):
        """Set mask path and find all masks in there"""
        self.mask_list = []
        self.mask_files = {}
        if os.path.exists(path):
            for name in os.listdir(path):
                if self.__is_mask_file(name):
                    self.mask_list.append(name)
                    self.mask_files[name] = os.path.join(path, name)

    def set_mask_list(self, mask_list):
            self.mask_list = []
            self.mask_files = {}
            for line in mask_list:
                path = os.path.join(self.data_path, line)
                name = os.path.basename(path)
                self.mask_list.append(name)
                self.mask_files[name] = path

    def __exif_path(self):
        """Return path of extracted exif directory"""
        return os.path.join(self.data_path, 'exif')

    def __exif_file(self, image):
        """
        Return path of exif information for given image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.__exif_path(), image + '.exif')

    def load_exif(self, image):
        """
        Return extracted exif information, as dictionary, usually with fields:

        ================  =====  ===================================
        Field             Type   Description
        ================  =====  ===================================
        width             int    Width of image, in pixels
        height            int    Height of image, in pixels
        focal_prior       float  Focal length (real) / sensor width
        ================  =====  ===================================

        :param image: Image name, with extension (i.e. 123.jpg)
        """
        with open(self.__exif_file(image), 'rb') as fin:
            return json.load(fin)

    def save_exif(self, image, data):
        io.mkdir_p(self.__exif_path())
        with open(self.__exif_file(image), 'wb') as fout:
            io.json_dump(data, fout)

    def feature_type(self):
        """Return the type of local features (e.g. AKAZE, SURF, SIFT)
        """
        feature_name = self.config.get('feature_type', 'sift').lower()
        if self.config.get('feature_root', False): feature_name = 'root_' + feature_name
        return feature_name

    def __feature_path(self):
        """Return path of feature descriptors and FLANN indices directory"""
        return os.path.join(self.data_path, "features")

    def __feature_file(self, image):
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.__feature_path(), image + '.npz')

    def __save_features(self, filepath, image, points, descriptors, colors=None):
        io.mkdir_p(self.__feature_path())
        feature_type = self.config.get('feature_type')
        if ((feature_type == 'AKAZE' and self.config.get('akaze_descriptor') in ['MLDB_UPRIGHT', 'MLDB']) or
            (feature_type == 'HAHOG' and self.config.get('hahog_normalize_to_uchar', False))):
            feature_data_type = np.uint8
        else:
            feature_data_type = np.float32
        np.savez_compressed(filepath,
                 points=points.astype(np.float32),
                 descriptors=descriptors.astype(feature_data_type),
                 colors=colors)

    def features_exist(self, image):
        return os.path.isfile(self.__feature_file(image))

    def load_features(self, image):
        feature_type = self.config.get('feature_type')
        s = np.load(self.__feature_file(image))
        if feature_type == 'HAHOG' and self.config.get('hahog_normalize_to_uchar', False):
            descriptors = s['descriptors'].astype(np.float32)
        else:
            descriptors = s['descriptors']
        return s['points'], descriptors, s['colors'].astype(float)

    def save_features(self, image, points, descriptors, colors):
        self.__save_features(self.__feature_file(image), image, points, descriptors, colors)

    def feature_index_exists(self, image):
        return os.path.isfile(self.__feature_index_file(image))

    def __feature_index_file(self, image):
        """
        Return path of FLANN index file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.__feature_path(), image + '.flann')

    def load_feature_index(self, image, features):
        index = cv2.flann.Index() if context.OPENCV3 else cv2.flann_Index()
        index.load(features, self.__feature_index_file(image))
        return index

    def save_feature_index(self, image, index):
        index.save(self.__feature_index_file(image))

    def __preemptive_features_file(self, image):
        """
        Return path of preemptive feature file (a short list of the full feature file)
        for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.__feature_path(), image + '_preemptive' + '.npz')

    def load_preemtive_features(self, image):
        s = np.load(self.__preemptive_features_file(image))
        return s['points'], s['descriptors']

    def save_preemptive_features(self, image, points, descriptors):
        self.__save_features(self.__preemptive_features_file(image), image, points, descriptors)

    def __matches_path(self):
        """Return path of matches directory"""
        return os.path.join(self.data_path, 'matches')

    def __matches_file(self, image):
        """File for matches for an image"""
        return os.path.join(self.__matches_path(), '{}_matches.pkl.gz'.format(image))

    def matches_exists(self, image):
        return os.path.isfile(self.__matches_file(image))

    def load_matches(self, image):
        with gzip.open(self.__matches_file(image), 'rb') as fin:
            matches = pickle.load(fin)
        return matches

    def save_matches(self, image, matches):
        io.mkdir_p(self.__matches_path())
        with gzip.open(self.__matches_file(image), 'wb') as fout:
            pickle.dump(matches, fout)

    def find_matches(self, im1, im2):
        if self.matches_exists(im1):
            im1_matches = self.load_matches(im1)
            if im2 in im1_matches:
                return im1_matches[im2]
        if self.matches_exists(im2):
            im2_matches = self.load_matches(im2)
            if im1 in im2_matches:
                if len(im2_matches[im1]):
                    return im2_matches[im1][:, [1, 0]]
        return []

    def __tracks_graph_file(self, filename=None):
        """Return path of tracks file"""
        return os.path.join(self.data_path, filename or 'tracks.csv')

    def load_tracks_graph(self, filename=None):
        """Return graph (networkx data structure) of tracks"""
        with open(self.__tracks_graph_file(filename)) as fin:
            return load_tracks_graph(fin)

    def save_tracks_graph(self, graph, filename=None):
        with open(self.__tracks_graph_file(filename), 'w') as fout:
            save_tracks_graph(fout, graph)

    def load_undistorted_tracks_graph(self):
        return self.load_tracks_graph('undistorted_tracks.csv')

    def save_undistorted_tracks_graph(self, graph):
        return self.save_tracks_graph(graph, 'undistorted_tracks.csv')

    def __reconstruction_file(self, filename):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, filename or 'reconstruction.json')

    def load_reconstruction(self, filename=None):
        with open(self.__reconstruction_file(filename)) as fin:
            reconstructions = io.reconstructions_from_json(json.load(fin))
        return reconstructions

    def save_reconstruction(self, reconstruction, filename=None, minify=False):
        with open(self.__reconstruction_file(filename), 'w') as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify)

    def load_undistorted_reconstruction(self):
        return self.load_reconstruction(
            filename='undistorted_reconstruction.json')

    def save_undistorted_reconstruction(self, reconstruction):
        return self.save_reconstruction(
            reconstruction, filename='undistorted_reconstruction.json')

    def __reference_lla_path(self):
        return os.path.join(self.data_path, 'reference_lla.json')

    def invent_reference_lla(self, images=None):
        lat, lon, alt = 0.0, 0.0, 0.0
        wlat, wlon, walt = 0.0, 0.0, 0.0
        if images is None: images = self.images()
        for image in images:
            d = self.load_exif(image)
            if 'gps' in d and 'latitude' in d['gps'] and 'longitude' in d['gps']:
                w = 1.0 / d['gps'].get('dop', 15)
                lat += w * d['gps']['latitude']
                lon += w * d['gps']['longitude']
                wlat += w
                wlon += w
                if 'altitude' in d['gps']:
                    alt += w * d['gps']['altitude']
                    walt += w
        if wlat: lat /= wlat
        if wlon: lon /= wlon
        if walt: alt /= walt
        reference = {'latitude': lat, 'longitude': lon, 'altitude': 0}  # Set altitude manually.
        self.save_reference_lla(reference)
        return reference

    def save_reference_lla(self, reference):
        with open(self.__reference_lla_path(), 'w') as fout:
            json.dump(reference, fout)

    def load_reference_lla(self):
        with open(self.__reference_lla_path(), 'r') as fin:
            return json.load(fin)

    def __camera_models_file(self):
        """Return path of camera model file"""
        return os.path.join(self.data_path, 'camera_models.json')

    def load_camera_models(self):
        """Return camera models data"""
        with open(self.__camera_models_file(), 'r') as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def save_camera_models(self, camera_models):
        """Save camera models data"""
        with open(self.__camera_models_file(), 'w') as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def __camera_models_overrides_file(self):
        """Return path of camera model overrides file"""
        return os.path.join(self.data_path, 'camera_models_overrides.json')

    def camera_models_overrides_exists(self):
        return os.path.isfile(self.__camera_models_overrides_file())

    def load_camera_models_overrides(self):
        """Return camera models overrides data"""
        with open(self.__camera_models_overrides_file(), 'r') as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def profile_log(self):
        "Filename where to write timings."
        return os.path.join(self.data_path, 'profile.log')

    def __navigation_graph_file(self):
        "Return the path of the navigation graph."
        return os.path.join(self.data_path, 'navigation_graph.json')

    def save_navigation_graph(self, navigation_graphs):
        with open(self.__navigation_graph_file(), 'w') as fout:
            io.json_dump(navigation_graphs, fout)

    def __ply_file(self):
        return os.path.join(self.data_path, 'reconstruction.ply')

    def save_ply(self, reconstruction):
        """Save a reconstruction in PLY format"""
        ply = io.reconstruction_to_ply(reconstruction)
        with open(self.__ply_file(), 'w') as fout:
            fout.write(ply)

    def __ground_control_points_file(self):
        return os.path.join(self.data_path, 'gcp_list.txt')

    def ground_control_points_exist(self):
        return os.path.isfile(self.__ground_control_points_file())

    def load_ground_control_points(self):
        """Load ground control points.

        It uses reference_lla to convert the coordinates
        to topocentric reference frame.
        """
        exif = {image: self.load_exif(image) for image in self.images()}

        with open(self.__ground_control_points_file()) as fin:
            return io.read_ground_control_points_list(
                fin, self.load_reference_lla(), exif)


def load_tracks_graph(fileobj):
    g = nx.Graph()
    for line in fileobj:
        image, track, observation, x, y, R, G, B = line.split('\t')
        g.add_node(image, bipartite=0)
        g.add_node(track, bipartite=1)
        g.add_edge(
            image, track,
            feature=(float(x), float(y)),
            feature_id=int(observation),
            feature_color=(float(R), float(G), float(B)))
    return g


def save_tracks_graph(fileobj, graph):
    for node, data in graph.nodes(data=True):
        if data['bipartite'] == 0:
            image = node
            for track, data in graph[image].items():
                x, y = data['feature']
                fid = data['feature_id']
                r, g, b = data['feature_color']
                fileobj.write('%s\t%s\t%d\t%g\t%g\t%g\t%g\t%g\n' % (
                    str(image), str(track), fid, x, y, r, g, b))
