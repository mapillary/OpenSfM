#!/usr/bin/env python

import os
import json
import numpy as np
from PIL import Image
import networkx as nx


class DataSet:

    def __init__(self, data_path):
        self.data_path = data_path

        for p in [self.exif_path(),
                  self.sift_path(),
                  self.matches_path(),
                  self.robust_matches_path()]:
            try:
                os.makedirs(p)
            except:
                pass

    def is_image_file(self, file):
        return file.split('.')[-1].lower() in ['jpg', 'jpeg', 'png', 'tif', 'tiff', 'pgm', 'pnm', 'gif']

    def images(self):
        return [i for i in os.listdir(self.image_path()) if self.is_image_file(i)]

    def image_path(self):
        return os.path.join(self.data_path, 'images')

    def image_file(self, image):
        return os.path.join(self.image_path(), image)

    def image_as_array(self, image):
        return np.array(Image.open(self.image_file(image)))

    def exif_path(self):
        return os.path.join(self.data_path, 'exif')

    def exif_file(self, image):
        return os.path.join(self.exif_path(), image + '.exif')

    def exif_data(self, image):
        with open(self.exif_file(image), 'r') as fin:
            return json.load(fin)
        return None

    def sift_path(self):
        return os.path.join(self.data_path, 'sift')

    def sift_file(self, image):
        return os.path.join(self.sift_path(), image + '.sift')

    def matches_path(self):
        return os.path.join(self.data_path, 'matches')

    def matches_file(self, image1, image2):
        return os.path.join(self.matches_path(), '%s_%s_matches.csv' % (image1, image2))

    def robust_matches_path(self):
        return os.path.join(self.data_path, 'robust_matches')

    def robust_matches_file(self, image1, image2):
        return os.path.join(self.robust_matches_path(), '%s_%s_matches.csv' % (image1, image2))

    def tracks_file(self):
        return os.path.join(self.data_path, 'tracks.csv')

    def tracks_graph(self):
        with open(self.tracks_file()) as fin:
            g = nx.Graph()
            for line in fin:
                image, track, observation, x, y = line.split()
                g.add_node(image, bipartite=0)
                g.add_node(track, bipartite=1)
                g.add_edge(image, track, feature=(float(x), float(y)))
            return g
        return None

    def reconstruction_file(self):
        return os.path.join(self.data_path, 'reconstruction.json')


