from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import logging
import numpy as np

from opensfm.dataset import DataSet
from opensfm import tracking


logger = logging.getLogger(__name__)


class SyntheticDataSet(DataSet):

    def __init__(self, reconstruction, exifs, features=None,
                 descriptors=None, colors=None, tracks_manager=None):
        super(SyntheticDataSet, self).__init__('')
        self.reconstruction = reconstruction
        self.exifs = exifs
        self.features = features
        self.descriptors = descriptors
        self.colors = colors
        self.tracks_manager = tracks_manager
        self.image_list = list(reconstruction.shots.keys())
        self.reference_lla = {'latitude': 0, 'longitude': 0, 'altitude': 0}
        self.matches = None
        self.config['use_altitude_tag'] = True
        self.config['align_method'] = 'naive'

    def images(self):
        return self.image_list

    def load_camera_models(self):
        return self.reconstruction.cameras

    def load_exif(self, image):
        return self.exifs[image]

    def exif_exists(self, image):
        return True

    def features_exist(self, image):
        if self.features is None or self.colors is None:
            return False
        return True

    def load_words(self, image):
        n_closest = 50
        return [image] * n_closest

    def load_features(self, image):
        return self.features[image], self.descriptors[image], self.colors[image]

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
                tracks = tracking.common_tracks(self.tracks_manager,
                                                im1, im2)[0]
                if len(tracks) > 10:
                    pair_matches = []
                    for t in tracks:
                        observations = self.tracks_manager.get_track_observations(t)
                        pair_matches.append(np.array([observations[im1].id,
                                                      observations[im2].id]))
                    image_matches[im2] = np.array(pair_matches)
        return matches

    def load_tracks_manager(self, filename=None):
        return self.tracks_manager

    def save_tracks_manager(self, manager, filename=None):
        pass

    def invent_reference_lla(self, images=None):
        return self.reference_lla

    def load_reference_lla(self):
        return self.reference_lla

    def reference_lla_exists(self):
        return True
