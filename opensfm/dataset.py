import cv2 as cv
import os
from os.path import abspath, join, dirname
import copy
import logging
import argparse
import time
import sys
import numpy as np
import errno
import io
import json
import gzip
import pickle
import six
import networkx as nx

from opensfm import extract_metadata
from opensfm import detect_features
from opensfm import match_features
from opensfm import create_tracks
from opensfm import reconstruct
from opensfm import mesh_data
from opensfm import undistort
from opensfm import undistorted_dataset
from opensfm import compute_depthmaps

from opensfm import dataset
from opensfm import log
from opensfm import io
from opensfm import exif
from opensfm import types
from opensfm import config
from opensfm import features
from opensfm import tracking

from PIL import Image

logger = logging.getLogger(__name__)
logging.getLogger("Starting Webcam!!").setLevel(logging.WARNING)

class DataSet(object):
    """Accessors to the main input and output data.

    Data include input images, masks, and segmentation as well
    temporary data such as features and matches and the final
    reconstructions.

    All data should be stored inside a variable to use RAM.
    Not in the folder.

    It is possible to store data remotely or in different formats
    by subclassing this class and overloading its methods.
    """
    def __init__(self, data_path, image_list):
        self.data_path=data_path
        self.image_list=image_list
        self._load_config()
        self._load_mask_list()
        self.meta_data_d={}
        self.camera_models={}
        self.feature_of_images={}
        self.feature_report={}
        self.match_of_images={}
        self.track_graph_of_images=nx.Graph()
        self.reconstructions=[]
        self.reconstructions_as_json={}
        #self.undistorted_data
        self.udata_image={}
        self.udata_reconstruction=[]
        self.udata_track_graph=nx.Graph()
        self.raw_depthmap={}
        self.raw_ply={}
        self.clean_depthmap={}
        self.clean_ply={}
        self.pruned_depthmap={}
        self.pruned_ply={}


    def _load_config(self):
        config_file = os.path.join(self.data_path, 'config.yaml')
        self.config = config.load_config(config_file)
    def show(self, image):
        cv.imshow('test',image)
        cv.waitKey(0)
        cv.destroyAllWindows()
    
    def images(self):
        """List of all images in the dataset""" 
        return self.image_list.keys()

    def _image_file(self, image):
        """Path to the image file."""
        return self.image_list[image]

    def image_size(self,image):
        return image.shape[0], image.shape[1]

    def _exif_file(self, image):
        """
        Return path of exif information for given image
        :param image: Image name, with extension (i.e. 123.jpg)
        """      
        return os.path.join(self._exif_path(), image + '.exif')
    def _exif_path(self):
        """Return path of extracted exif directory"""
        return os.path.join(self.data_path, 'exif')

    def save_exif(self, image, data):
        try:
            os.makedirs(self._exif_path())
        except os.error as exc:
            if exc.errno != errno.EEXIST or not os.path.isdir(self._exif_path()):
                raise
        with io.open_wt(self._exif_file(image)) as fout:
            io.json_dump(data, fout)

    def load_exif(self, image):
        #image='1.jpg'
        return self.meta_data_d

    def _camera_models_file(self):
        """Return path of camera model file"""
        return os.path.join(self.data_path, 'camera_models.json')

    def load_camera_models(self):
       # """Return camera models data"""
        with io.open_rt(self._camera_models_file()) as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)
        #"""Load pre-extracted image exif metadata."""
        

    def save_camera_models(self, camera_models):
        """Save camera models data"""
        with io.open_wt(self._camera_models_file()) as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def _camera_models_file(self):
        """Return path of camera model file"""
        return os.path.join(self.data_path, 'camera_models.json')

    def _load_mask_list(self):
        """Load mask list from mask_list.txt or list masks/ folder."""
        mask_list_file = os.path.join(self.data_path, 'mask_list.txt')
        if os.path.isfile(mask_list_file):
            with io.open_rt(mask_list_file) as fin:
                lines = fin.read().splitlines()
            self._set_mask_list(lines)
        else:
            self._set_mask_path(os.path.join(self.data_path, 'masks'))
    
    def _set_mask_path(self, path):
        """Set mask path and find all masks in there"""
        self.mask_files = {}
        for image in self.images():
            filepath = os.path.join(path, image + '.png')
            if os.path.isfile(filepath):
                self.mask_files[image] = filepath            

    def features_exist(self, image):
        return os.path.isfile(self._feature_file(image)) or\
            os.path.isfile(self._feature_file_legacy(image))

    def feature_type(self):
        """Return the type of local features (e.g. AKAZE, SURF, SIFT)"""
        feature_name = self.config['feature_type'].lower()
        if self.config['feature_root']:
            feature_name = 'root_' + feature_name
        return feature_name
    
    def load_image(self, image, unchanged=False, anydepth=False):
        """Load image pixels as numpy array.

        The array is 3D, indexed by y-coord, x-coord, channel.
        The channels are in RGB order.
        """
        a=io.imread(self._image_file(image), unchanged=unchanged, anydepth=anydepth)
        return a

    def load_mask(self, image):
        """Load image mask if it exists, otherwise return None."""
        if image in self.mask_files: ## 여기 안들어감 
            mask_path = self.mask_files[image]
            mask = io.imread(mask_path, grayscale=True)
           
            if mask is None:
                raise IOError("Unable to load mask for image {} "
                              "from file {}".format(image, mask_path))
        else:
            mask = None
        return mask

    def load_segmentation_mask(self, image):
        """Build a mask from segmentation ignore values.

        The mask is non-zero only for pixels with segmentation
        labels not in segmentation_ignore_values.
        """
        ignore_values = self.segmentation_ignore_values(image)
        if not ignore_values:   
            return None
        segmentation = self.load_segmentation(image)
        if segmentation is None:
            return None

        return self._mask_from_segmentation(segmentation, ignore_values)

    def _combine_masks(self, mask, smask):
        if mask is None:
            if smask is None:
                return None
            else:
                return smask
        else:
            if smask is None:
                return mask
            else:
                return mask & smask

    def segmentation_ignore_values(self, image):
        """List of label values to ignore.

        Pixels with this labels values will be masked out and won't be
        processed when extracting features or computing depthmaps.
        """
        return self.config.get('segmentation_ignore_values', [])

    def load_combined_mask(self, image):
        """Combine binary mask with segmentation mask.
        Return a mask that is non-zero only where the binary
        mask and the segmentation mask are non-zero.
        """
        mask = self.load_mask(image) # 필요없다 None값
        smask = self.load_segmentation_mask(image) # 필요없다 None 값 
        return self._combine_masks(mask, smask)
    
    def load_features_mask(self, image, points):
        """Load a feature-wise mask.

        This is a binary array true for features that lie inside the
        combined mask.
        The array is all true when there's no mask.
        """
        if points is None or len(points) == 0:
            return np.array([], dtype=bool)

        mask_image = self.load_combined_mask(image)
        
        if mask_image is None:
            logger.debug('No segmentation for {}, no features masked.'.format(image))
            return np.ones((points.shape[0],), dtype=bool)
        #detect features명령어에선 여기서 종료함 **********

        exif = self.load_exif(image)
        width = exif["width"]
        height = exif["height"]
        orientation = exif["orientation"]

        new_height, new_width = mask_image.shape
        ps = upright.opensfm_to_upright(
            points[:, :2], width, height, orientation,
            new_width=new_width, new_height=new_height).astype(int)
        mask = mask_image[ps[:, 1], ps[:, 0]]

        n_removed = np.sum(mask == 0)
        logger.debug('Masking {} / {} ({:.2f}) features for {}'.format(
            n_removed, len(mask), n_removed / len(mask), image))

        return np.array(mask, dtype=bool)
    
    def _feature_file(self, image):
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._feature_path(), image + '.features.npz')
    
    def _feature_path(self):
        """Return path of feature descriptors and FLANN indices directory"""
        return os.path.join(self.data_path, "features")

    def _save_features(self, filepath, points, descriptors, colors=None):
        io.mkdir_p(self._feature_path())
        features.save_features(filepath, points, descriptors, colors, self.config)

    def save_features(self, image, points, descriptors, colors):
        self._save_features(self._feature_file(image), points, descriptors, colors)

    def save_total_features(self,features):
        self.feature_of_images=features

    def save_report_of_features(self,image,report):
        self.feature_report.update({image:report})
    
    def _matches_path(self):
        """Return path of matches directory"""
        return os.path.join(self.data_path, 'matches')
    
    def _matches_file(self, image):
    #"""File for matches for an image"""
        return os.path.join(self._matches_path(), '{}_matches.pkl.gz'.format(image))

    def save_matches(self, image, matches):
        io.mkdir_p(self._matches_path())
        with gzip.open(self._matches_file(image), 'wb') as fout:
            pickle.dump(matches, fout)

    def load_matches(self, image):
        return self.match_of_images[image]

    def save_total_matches(self,image,matches):
        self.match_of_images.update({image:matches})


    def load_features(self, image):
        return features.load_features(self._feature_file(image), self.config)

    def _tracks_graph_file(self, filename=None):
        """Return path of tracks file"""
        return os.path.join(self.data_path, filename or 'tracks.csv')

    def save_tracks_graph(self, graph, filename=None):
        with io.open_wt(self._tracks_graph_file(filename)) as fout:
            tracking.save_tracks_graph(fout, graph) 

    def save_total_tracks_graph(self,graph,filename=None):
        self.track_graph_of_images=graph

    def invent_reference_lla(self, images=None):
        #lat, lon, alt = 0.0, 0.0, 0.0
        reference = {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0}  # Set altitude manually.
        #self.save_reference_lla(reference)
        return reference

    def save_reference_lla(self, reference):
        with io.open_wt(self._reference_lla_path()) as fout:
            io.json_dump(reference, fout)

    def _reference_lla_path(self):
        return os.path.join(self.data_path, 'reference_lla.json')

    def reference_lla_exists(self):
        return os.path.isfile(self._reference_lla_path())

    def _reconstruction_file(self, filename):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, filename or 'reconstruction.json')

    def save_reconstruction(self, reconstruction, filename=None, minify=False):
        with io.open_wt(self._reconstruction_file(filename)) as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify)

    def save_reconstruction_to_json(self, reconstruction):
        self.reconstructions_as_json.update({'cameras':reconstruction.cameras})
        self.reconstructions_as_json.update({'shots':reconstruction.shots})
        self.reconstructions_as_json.update({'points':reconstruction.points})


    def save_undistorted_reconstruction(self, reconstruction):
        self.udata_reconstruction=reconstruction

    def save_undistorted_tracks_graph(self,graph):
        self.udata_track_graph=graph

    def save_udata(self,udata):
        self.undistorted_data=udata

    def save_raw_depthmap(self, image, depthmap):
        self.raw_depthmap.update({image:depthmap})

    def save_raw_ply(self, image, ply):
        self.raw_ply.update({image:ply})

    def save_clean_depthmap(self, image, depthmap):
        self.clean_depthmap.update({image:depthmap})

    def save_clean_ply(self, image, ply):
        self.clean_ply.update({image:ply})

    def save_pruned_depthmap(self, image, depthmap):
        self.pruned_depthmap.update({image:depthmap})

    def save_ply_line(self, image, ply):
        self.pruned_ply.update({image:ply})


    def load_raw_depthmap(self, image):
        o = self.raw_depthmap[image]
        return o['depth'], o['plane'], o['score'], o['nghbr'], o['nghbrs']

    def load_clean_depthmap(self, image):
        o = self.clean_depthmap[image]
        return o['depth'], o['plane'], o['score']

    def load_pruned_depthmap(self, image):
        o = self.pruned_depthmap[image]
        if 'detections' not in o:
            return o['points'], o['normals'], o['colors'], o['labels'], np.zeros(o['labels'].shape)
        else:
            return o['points'], o['normals'], o['colors'], o['labels'], o['detections']

    def _report_path(self):
        return os.path.join(self.data_path, 'reports')

    def save_report(self, report_str, path):
        """Save report string to a file."""
        filepath = os.path.join(self._report_path(), path)
        io.mkdir_p(os.path.dirname(filepath))
        with io.open_wt(filepath) as fout:
            return fout.write(report_str)
    
    def load_report(self, path):
        """Load a report file as a string."""
        with io.open_rt(os.path.join(self._report_path(), path)) as fin:
            return fin.read()

    def profile_log(self):
        #"Filename where to write timings."
        return os.path.join(self.data_path, 'profile.log')

    def _depthmap_path(self):
        #self.depthmap_path = os.path.join(self.data_path, 'undistorted')
        return os.path.join(self.data_path)