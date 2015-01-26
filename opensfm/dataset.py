# -*- coding: utf-8 -*-

import os
import json
import errno
import numpy as np
import networkx as nx
import yaml
import cv2

import io

class DataSet:
    """
    Dataset representing directory with images, extracted , feature descriptors (SURF, SIFT), etc.

    Methods to retrieve *base directory* for data file(s) have suffix ``_path``, methods to retrieve path of specified
    data file have suffix ``_file``.
    """
    def __init__(self, data_path):
        """
        Create dataset instance. Empty directories (for EXIF, robust matches, etc) will be created if they don't exist
        already.

        :param data_path: Path to directory containing dataset
        """
        self.data_path = data_path

        # Load configuration.
        config_file = os.path.join(self.data_path, 'config.yaml')
        if os.path.isfile(config_file):
            with open(config_file) as fin:
                self.config = yaml.load(fin)
        else:
            self.config = {}

        # Load list of images.
        image_list_file = os.path.join(self.data_path, 'image_list.txt')
        if os.path.isfile(image_list_file):
            with open(image_list_file) as fin:
                lines = fin.read().splitlines()
            self.set_image_list(lines)
        else:
            self.set_image_path(os.path.join(self.data_path, 'images'))

        # Create output folders.
        for p in [self.exif_path(),
                  self.feature_path(),
                  self.robust_matches_path()]:
            io.mkdir_p(p)

    def images(self):
        """Return list of file names of all images in this dataset"""
        return self.image_list

    def image_file(self, image):
        """
        Return path of image with given name
        :param image: Image file name (**with extension**)
        """
        return self.image_files[image]

    def image_as_array(self, image):
        """Return image pixels as 3-dimensional OpenCV matrix (R G B order)"""
        return cv2.imread(self.image_file(image))[:,:,::-1]  # Turn BGR to RGB

    @staticmethod
    def _is_image_file(filename):
        return filename.split('.')[-1].lower() in {'jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif'}

    def set_image_path(self, path):
        """Set image path and find the all images in there"""
        self.image_list = []
        self.image_files = {}
        for name in os.listdir(path):
            if self._is_image_file(name):
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

    def exif_path(self):
        """Return path of extracted exif directory"""
        return os.path.join(self.data_path, 'exif')

    def exif_file(self, image):
        """
        Return path of exif information for given image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.exif_path(), image + '.exif')

    def exif_data(self, image):
        """
        Return extracted exif information, as dictionary, usually with fields:

        ================  =====  ===================================
        Field             Type   Description
        ================  =====  ===================================
        width             int    Width of image, in pixels
        height            int    Height of image, in pixels
        focal_ratio       float  Focal length (real) / sensor width
        focal_35mm_equiv  float  Focal length in 35 mm equivalent
        ================  =====  ===================================

        :param image: Image name, with extension (i.e. 123.jpg)
        """
        with open(self.exif_file(image), 'r') as fin:
            return json.load(fin)

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
            descriptor_type = self.config.get('akaze_descriptor', 5)
            if descriptor_type == 0:
                descriptor_type_str = 'surf_upright'
            elif descriptor_type == 1:
                descriptor_type_str = 'surf'
            elif descriptor_type == 2:
                descriptor_type_str = 'msurf_upright'
            elif descriptor_type == 3:
                descriptor_type_str = 'msurf'
            elif descriptor_type == 4:
                descriptor_type_str = 'mldb_upright'
            elif descriptor_type == 5:
                descriptor_type_str = 'mldb'
            return descriptor_type_str
        else:
            return ''

    def feature_path(self):
        """Return path of feature descriptors and FLANN indices directory"""
        feature_path = self.feature_type()
        if len(self.descriptor_type()) > 0:
            feature_path += '_' + self.descriptor_type()
        return os.path.join(self.data_path, feature_path)

    def feature_file(self, image):
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.feature_path(), image + '.' + self.feature_type() + '.npz')

    def feature_index_file(self, image):
        """
        Return path of FLANN index file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.feature_path(), image + '.' + self.feature_type() + '.flann')

    def preemptive_feature_file(self, image):
        """
        Return path of preemptive feature file (a short list of the full feature file)
        for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.feature_path(), image + '_preemptive.' + self.feature_type() + '.npz')

    def matches_path(self):
        """Return path of matches directory"""
        return os.path.join(self.data_path, 'matches')

    def matches_file(self, image1, image2):
        """
        Return path of matches file for pair of specified images
        :param image1: Image name, with extension (i.e. 123.jpg)
        :param image2: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.matches_path(), '%s_%s_matches.csv' % (image1, image2))

    def matcher_type(self):
        """Return the type of matcher
        """
        matcher_type = self.config.get('matcher_type', 'BruteForce')
        if 'BruteForce' in matcher_type:
            if self.feature_type() == 'akaze' and (self.config.get('akaze_descriptor', 5) >= 4):
                 matcher_type = 'BruteForce-Hamming'
            self.config['matcher_type'] = matcher_type
        return matcher_type # BruteForce, BruteForce-L1, BruteForce-Hamming

    def robust_matches_path(self):
        """Return path of robust matches directory"""
        return os.path.join(self.data_path, 'robust_matches')

    def robust_matches_file(self, image1, image2):
        """
        Return path of *robust* matches file for pair of specified images
        :param image1: Image name, with extension (i.e. 123.jpg)
        :param image2: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.robust_matches_path(), '%s_%s_matches.csv' % (image1, image2))

    def tracks_file(self):
        """Return path of tracks file"""
        return os.path.join(self.data_path, 'tracks.csv')

    def tracks_graph(self, images=None):
        """Return graph (networkx data structure) of tracks"""
        with open(self.tracks_file()) as fin:
            g = nx.Graph()
            for line in fin:
                image, track, observation, x, y = line.split('\t')
                g.add_node(image, bipartite=0)
                g.add_node(track, bipartite=1)
                g.add_edge(image, track, feature=(float(x), float(y)), feature_id=int(observation))
            return g

    def reconstruction_file(self):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, 'reconstruction.json')

    def invent_reference_lla(self, images=None):
        lat, lon, alt = 0.0, 0.0, 0.0
        wlat, wlon, walt = 0.0, 0.0, 0.0
        if images is None: images = self.images()
        for image in images:
            d = self.exif_data(image)
            if 'gps' in d:
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
        self.set_reference_lla(lat, lon, 0) # Set altitude manually.
        return {'latitude': lat, 'longitude': lon, 'altitude': alt}

    def reference_lla_path(self):
        return os.path.join(self.data_path, 'reference_lla.json')

    def set_reference_lla(self, lat, lon, alt):
        with open(self.reference_lla_path(), 'w') as fout:
            json.dump({'latitude': lat,
                       'longitude': lon,
                       'altitude': alt}, fout)

    def reference_lla(self):
        with open(self.reference_lla_path(), 'r') as fin:
            d = json.load(fin)
            return d['latitude'], d['longitude'], d['altitude']

    def camera_model_data(self):
        """
        Return camera model data
        """
        with open(self.camera_model_file(), 'r') as fin:
            return json.load(fin)

    def camera_model_file(self):
        """Return path of camera model file"""
        return os.path.join(self.data_path, 'camera_models.json')

    def profile_log(self):
        "Filename where to write timings."
        return os.path.join(self.data_path, 'profile.log')


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
