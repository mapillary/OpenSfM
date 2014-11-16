# -*- coding: utf-8 -*-

import os
import json
import errno
import numpy as np
import networkx as nx
import yaml
import cv2


class DataSet:
    """
    Dataset representing directory with images, extracted EXIF, SIFT descriptors, etc.

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

        with open(os.path.join(self.data_path, 'config.yaml')) as fin:
            self.config = yaml.load(fin)

        for p in [self.exif_path(),
                  self.sift_path(),
                  self.robust_matches_path()]:
            try:
                os.makedirs(p)
            except os.error as exc:
                if exc.errno == errno.EEXIST and os.path.isdir(p):
                    pass
                else:
                    raise

    def is_image_file(self, file):
        return file.split('.')[-1].lower() in ['jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif']

    def images(self):
        """Return list of file paths of all images in this dataset"""
        return [i for i in os.listdir(self.image_path()) if self.is_image_file(i)]

    def image_path(self):
        """Return path of images directory"""
        return os.path.join(self.data_path, 'images')

    def image_file(self, image):
        """
        Return path of image with given name
        :param image: Image file name (**with extension**)
        """
        return os.path.join(self.image_path(), image)

    def image_as_array(self, image):
        """Return image pixels as 3-dimensional OpenCV matrix (R G B order)"""
        return cv2.imread(self.image_file(image))[:,:,::-1]  # Turn BGR to RGB

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
        return None

    def sift_path(self):
        """Return path of SIFT descriptors and FLANN indices directory"""
        return os.path.join(self.data_path, 'sift')

    def sift_file(self, image):
        """
        Return path of SIFT file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.sift_path(), image + '.sift')

    def sift_index_file(self, image):
        """
        Return path of FLANN index file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self.sift_path(), image + '.flann')

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

    def tracks_graph(self):
        """Return graph (networkx data structure) of tracks"""
        with open(self.tracks_file()) as fin:
            g = nx.Graph()
            for line in fin:
                image, track, observation, x, y = line.split('\t')
                g.add_node(image, bipartite=0)
                g.add_node(track, bipartite=1)
                g.add_edge(image, track, feature=(float(x), float(y)))
            return g
        return None

    def reconstruction_file(self):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, 'reconstruction.json')


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


