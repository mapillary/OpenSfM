import logging
import os
import os.path
import numpy as np

from opensfm.dataset import DataSet
from opensfm import matching
from opensfm import geo


logger = logging.getLogger(__name__)


class SyntheticDataSet(DataSet, object):

    def __init__(self, reconstruction, features=None,
                 colors=None, graph=None):
        super(SyntheticDataSet, self).__init__('/home/yann/')
        self.reconstruction = reconstruction
        self.features = features
        self.colors = colors
        self.graph = graph
        self.image_list = reconstruction.shots.keys()
        self.reference_lla = {'latitude': 0, 'longitude': 0, 'altitude': 0}
        self.matches = None

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
        shot = self.reconstruction.shots[image]
        exif['width'] = shot.camera.width
        exif['height'] = shot.camera.height
        exif['focal_prior'] = shot.camera.focal_prior
        exif['camera'] = str(shot.camera.id)

        pose = shot.pose.get_origin()
        lat, lon, alt = geo.lla_from_topocentric(pose[0], pose[1], pose[2],
                                                 0, 0, 0)
        exif['gps'] = {}
        exif['gps']['latitude'] = lat
        exif['gps']['longitude'] = lon
        exif['gps']['altitude'] = alt
        return exif

    def exif_exists(self, image):
        return True

    def features_exist(self, image):
        if self.features is None or self.colors is None:
            return False
        return True

    def load_features(self, image):
        return self.features[image], {}, self.colors[image]

    def save_features(self, image, points, descriptors, colors):
        pass

    def matches_exists(self, image):
        self._check_and_create_matches()
        if self.matches is None:
            return False
        return True

    def load_matches(self, image):
        self._check_and_create_matches()
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

    def _check_and_create_matches(self):
        if self.matches is None:
            self.matches = self._construct_matches()

    def _construct_matches(self):
        matches = {}
        for im1 in self.images():
            for im2 in self.images():
                if im1 == im2:
                    continue
                image_matches = matches.setdefault(im1, {})
                tracks = matching.common_tracks(self.graph,
                                                im1, im2)[0]
                if len(tracks) > 10:
                    pair_matches = np.array(
                        [np.array([self.graph[t][im1]['feature_id'],
                                   self.graph[t][im2]['feature_id']])
                         for t in tracks])
                    image_matches[im2] = pair_matches
        return matches

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
