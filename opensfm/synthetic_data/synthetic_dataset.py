import logging
import os
import os.path

from opensfm.dataset import DataSet


logger = logging.getLogger(__name__)


class SyntheticDataSet(DataSet, object):

    def __init__(self, reconstruction, features=None,
                 colors=None, matches=None, graph=None):
        super(SyntheticDataSet, self).__init__('')
        self.reconstruction = reconstruction
        self.features = features
        self.colors = colors
        self.matches = matches
        self.graph = graph
        self.image_list = reconstruction.shots.keys()
        self.reference_lla = {'latitude': 0, 'longitude': 0, 'altitude': 0}

    def images(self):
        return self.image_list

    def load_camera_models(self):
        return self.reconstruction.cameras

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
        exif = {}
        exif['width'] = self.reconstruction.shots[image].camera.width
        exif['height'] = self.reconstruction.shots[image].camera.height
        exif['focal_prior'] = self.reconstruction.shots[image].camera.focal_prior
        exif['camera'] = str(self.reconstruction.shots[image].camera.id)
        return exif

    def exif_exists(self, image):
        return True

    def features_exist(self, image):
        if self.features is None or self.colors is None:
            return False
        return True

    def load_features(self, image):
        return self.features[image], self.colors[image]

    def save_features(self, image, points, descriptors, colors):
        pass

    def matches_exists(self, image):
        if self.matches is None:
            return False
        return True

    def load_matches(self, image):
        if self.matches is not None:
            return self.matches[image]

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

    def load_tracks_graph(self, filename=None):
        return self.graph

    def save_tracks_graph(self, graph, filename=None):
        pass

    def invent_reference_lla(self, images=None):
        return self.reference_lla

    def load_reference_lla(self):
        return self.reference_lla

    def reference_lla_exists(self):
        return True
