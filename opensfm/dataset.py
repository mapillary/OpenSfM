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

        # Load configuration.
        config_file = os.path.join(self.data_path, 'config.yaml')
        self.config = config.load_config(config_file)

        # Load list of images.
        image_list_file = os.path.join(self.data_path, 'image_list.txt')
        if os.path.isfile(image_list_file):
            with open(image_list_file) as fin:
                lines = fin.read().splitlines()
            self.set_image_list(lines)
        else:
            self.set_image_path(os.path.join(self.data_path, 'images'))

        # Create output folders.
        for p in [self.__exif_path(),
                  self.__feature_path(),
                  self.__matches_path()]:
            io.mkdir_p(p)

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
        return open(self.__image_file(image))

    def image_as_array(self, image):
        """Return image pixels as 3-dimensional numpy array (R G B order)"""
        return cv2.imread(self.__image_file(image), cv2.CV_LOAD_IMAGE_COLOR)[:,:,::-1]  # Turn BGR to RGB

    @staticmethod
    def __is_image_file(filename):
        return filename.split('.')[-1].lower() in {'jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif'}

    def set_image_path(self, path):
        """Set image path and find the all images in there"""
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
        with open(self.__exif_file(image), 'r') as fin:
            return json.load(fin)

    def save_exif(self, image, data):
        with open(self.__exif_file(image), 'w') as fout:
            fout.write(io.json_dumps(data))

    def feature_type(self):
        """Return the type of local features (e.g. AKAZE, SURF, SIFT)
        """
        feature_name = self.config.get('feature_type', 'sift').lower()
        if self.config.get('feature_root', False): feature_name = 'root_' + feature_name
        return feature_name

    def descriptor_type(self):
        """Return the type of the descriptor (if exists)
        """
        if self.feature_type() == 'akaze':
            return self.config.get('akaze_descriptor', '')
        else:
            return ''

    def __feature_path(self):
        """Return path of feature descriptors and FLANN indices directory"""
        __feature_path = self.feature_type()
        if len(self.descriptor_type()) > 0:
            __feature_path += '_' + self.descriptor_type()
        return os.path.join(self.data_path, __feature_path)

    def __feature_file(self, image):
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.__feature_path(), image + '.' + self.feature_type() + '.npz')

    def __save_features(self, filepath, image, points, descriptors, colors=None):
        feature_type = self.config.get('feature_type')
        if ((feature_type == 'AKAZE' and self.config.get('akaze_descriptor') in ['MLDB_UPRIGHT', 'MLDB']) or
            (feature_type == 'HAHOG' and self.config.get('hahog_normalize_to_uchar', False)) or
            (feature_type == 'SIFT')):
            feature_data_type = np.uint8
        else:
            feature_data_type = np.float32
        np.savez(filepath,
                 points=points.astype(np.float32),
                 descriptors=descriptors.astype(feature_data_type),
                 colors=colors)

    def features_exist(self, image):
        return os.path.isfile(self.__feature_file(image))

    def load_features(self, image):
        feature_type = self.config.get('feature_type')
        s = np.load(self.__feature_file(image))
        if (feature_type == 'HAHOG' and self.config.get('hahog_normalize_to_uchar', False) or
            feature_type == 'SIFT'):
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
        return os.path.join(self.__feature_path(), image + '.' + self.feature_type() + '.flann')

    def load_feature_index(self, image, features):
        index = cv2.flann_Index()
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
        return os.path.join(self.__feature_path(), image + '_preemptive.' + self.feature_type() + '.npz')

    def load_preemtive_features(self, image):
        s = np.load(self.__preemptive_features_file(image))
        return s['points'], s['descriptors']

    def save_preemptive_features(self, image, points, descriptors):
        self.__save_features(self.__preemptive_features_file(image), image, points, descriptors)


    def matcher_type(self):
        """Return the type of matcher
        """
        matcher_type = self.config.get('matcher_type', 'BruteForce')
        if 'BruteForce' in matcher_type:
            if self.feature_type() == 'akaze' and (self.config.get('akaze_descriptor', 5) >= 4):
                 matcher_type = 'BruteForce-Hamming'
            self.config['matcher_type'] = matcher_type
        return matcher_type # BruteForce, BruteForce-L1, BruteForce-Hamming


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

    def __tracks_graph_file(self):
        """Return path of tracks file"""
        return os.path.join(self.data_path, 'tracks.csv')

    def load_tracks_graph_as_list(self):
        """Return tranks graph as a list of edges"""
        track_list = []
        images = self.images()
        image_inv = {}
        for i, im in enumerate(images):
            image_inv[im] = int(i)
        with open(self.__tracks_graph_file()) as fin:
            for line in fin:
                image, track_id, observation, x, y = line.split('\t')
                if int(track_id) >= len(track_list):
                    track_list.append([])
                track_list[int(track_id)].append([image_inv[image], int(observation)])
        return track_list

    def load_tracks_graph(self):
        """Return graph (networkx data structure) of tracks"""
        with open(self.__tracks_graph_file()) as fin:
            return load_tracks_graph(fin)

    def save_tracks_graph(self, graph):
        with open(self.__tracks_graph_file(), 'w') as fout:
            save_tracks_graph(fout, graph)

    def __reconstruction_file(self, filename):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, filename or 'reconstruction.json')

    def load_reconstruction(self, filename=None):
        with open(self.__reconstruction_file(filename)) as fin:
            reconstructions = json.load(fin)
        return reconstructions

    def save_reconstruction(self, reconstruction, filename=None, indent=4):
        with open(self.__reconstruction_file(filename), 'w') as fout:
            fout.write(io.json_dumps(reconstruction))

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
            return json.load(fin)

    def save_camera_models(self, camera_models):
        """Save camera models data"""
        with open(self.__camera_models_file(), 'w') as fout:
            fout.write(io.json_dumps(camera_models))


    def __epipolar_path(self):
        return os.path.join(self.data_path, 'epipolar_geometries')

    def __epipolar_file(self, im1, im2):
        return os.path.join(self.__epipolar_path(), '%s_%s_epipolar.npz' % (im1, im2))

    def save_epipolar(self, im1, im2, R, t, X=[], inliers=[]):
        io.mkdir_p(self.__epipolar_path())
        np.savez(self.__epipolar_file(im1, im2), R=R, t=t, X=X, inliers=inliers)

    def load_epipolar(self, im1, im2):
        try:
            s = np.load(self.__epipolar_file(im1, im2))
        except IOError:
            return None, None, None, None
        return s['R'], s['t'], s['X'], s['inliers']

    def profile_log(self):
        "Filename where to write timings."
        return os.path.join(self.data_path, 'profile.log')

    def __navigation_graph_file(self):
        "Return the path of the navigation graph."
        return os.path.join(self.data_path, 'navigation_graph.json')

    def save_navigation_graph(self, navigation_graphs):
        with open(self.__navigation_graph_file(), 'w') as fout:
            fout.write(io.json_dumps(navigation_graphs))



def load_tracks_graph(fileobj):
    g = nx.Graph()
    for line in fileobj:
        image, track, observation, x, y, R, G, B = line.split('\t')
        g.add_node(image, bipartite=0)
        g.add_node(track, bipartite=1)
        g.add_edge(image, track,
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
                fileobj.write('%s\t%s\t%d\t%g\t%g\t%g\t%g\t%g\n' % (str(image), str(track), fid, x, y, r, g, b))


def common_tracks(g, im1, im2):
    """
    Return the list of tracks observed in both images
    :param g: Graph structure (networkx) as returned by :method:`DataSet.tracks_graph`
    :param im1: Image name, with extension (i.e. 123.jpg)
    :param im2: Image name, with extension (i.e. 123.jpg)
    :return: tuple: track, feature from first image, feature from second image
    """
    # TODO: return value is unclear
    # TODO: maybe make as method of DataSet?
    t1, t2 = g[im1], g[im2]
    tracks, p1, p2 = [], [], []
    for track in t1:
        if track in t2:
            p1.append(t1[track]['feature'])
            p2.append(t2[track]['feature'])
            tracks.append(track)
    p1 = np.array(p1)
    p2 = np.array(p2)
    return tracks, p1, p2
