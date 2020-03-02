
import os
import json
import logging
import pickle
import gzip

import numpy as np
import six

from opensfm import io
from opensfm import config
from opensfm import geo
from opensfm import tracking
from opensfm import features
from opensfm import upright


logger = logging.getLogger(__name__)


class UndistortedDataSet(object):
    """Accessors to the undistorted data of a dataset.

    Data include undistorted images, masks, and segmentation as well
    the undistorted reconstruction, tracks graph and computed depth maps.

    All data is stored inside a single folder which should be a subfolder
    of the base, distorted dataset.
    """
    def __init__(self, base_dataset, undistorted_subfolder):
        """Init dataset associated to a folder."""
        self.base = base_dataset
        self.config = self.base.config
        self.subfolder = undistorted_subfolder
        self.data_path = os.path.join(self.base.data_path, self.subfolder)
        print(self.subfolder)
        print(self.base)
        print(self.data_path)
        

    def _undistorted_image_path(self):
        return os.path.join(self.data_path, 'images')

    def _undistorted_image_file(self, image):
        """Path of undistorted version of an image."""
        image_format = self.config['undistorted_image_format']
        filename = image + '.' + image_format
        return os.path.join(self._undistorted_image_path(), filename)

    def load_undistorted_image(self, image):
        """Load undistorted image pixels as a numpy array."""
        return io.imread(self._undistorted_image_file(image))

    def save_undistorted_image(self, image, array):
        """Save undistorted image pixels."""
        io.mkdir_p(self._undistorted_image_path())
        io.imwrite(self._undistorted_image_file(image), array)

    def undistorted_image_size(self, image):
        """Height and width of the undistorted image."""
        return io.image_size(self._undistorted_image_file(image))

    def _undistorted_mask_path(self):
        return os.path.join(self.data_path, 'masks')

    def _undistorted_mask_file(self, image):
        """Path of undistorted version of a mask."""
        return os.path.join(self._undistorted_mask_path(), image + '.png')

    def undistorted_mask_exists(self, image):
        """Check if the undistorted mask file exists."""
        return os.path.isfile(self._undistorted_mask_file(image))

    def load_undistorted_mask(self, image):
        """Load undistorted mask pixels as a numpy array."""
        return io.imread(self._undistorted_mask_file(image), grayscale=True)

    def save_undistorted_mask(self, image, array):
        """Save the undistorted image mask."""
        io.mkdir_p(self._undistorted_mask_path())
        io.imwrite(self._undistorted_mask_file(image), array)

    def _undistorted_detection_path(self):
        return os.path.join(self.data_path, 'detections')

    def _undistorted_detection_file(self, image):
        """Path of undistorted version of a detection."""
        return os.path.join(self._undistorted_detection_path(), image + '.png')

    def undistorted_detection_exists(self, image):
        """Check if the undistorted detection file exists."""
        return os.path.isfile(self._undistorted_detection_file(image))

    def load_undistorted_detection(self, image):
        """Load an undistorted image detection."""
        return io.imread(self._undistorted_detection_file(image),
                         grayscale=True)

    def save_undistorted_detection(self, image, array):
        """Save the undistorted image detection."""
        io.mkdir_p(self._undistorted_detection_path())
        io.imwrite(self._undistorted_detection_file(image), array)

    def _undistorted_segmentation_path(self):
        return os.path.join(self.data_path, 'segmentations')

    def _undistorted_segmentation_file(self, image):
        """Path of undistorted version of a segmentation."""
        return os.path.join(self._undistorted_segmentation_path(), image + '.png')

    def undistorted_segmentation_exists(self, image):
        """Check if the undistorted segmentation file exists."""
        return os.path.isfile(self._undistorted_segmentation_file(image))

    def load_undistorted_segmentation(self, image):
        """Load an undistorted image segmentation."""
        return io.imread(self._undistorted_segmentation_file(image),
                         grayscale=True)

    def save_undistorted_segmentation(self, image, array):
        """Save the undistorted image segmentation."""
        io.mkdir_p(self._undistorted_segmentation_path())
        io.imwrite(self._undistorted_segmentation_file(image), array)

    def load_undistorted_segmentation_mask(self, image):
        """Build a mask from the undistorted segmentation.

        The mask is non-zero only for pixels with segmentation
        labels not in segmentation_ignore_values.
        """
        ignore_values = self.base.segmentation_ignore_values(image)
        if not ignore_values:
            return None

        segmentation = self.load_undistorted_segmentation(image)
        if segmentation is None:
            return None

        return self.base._mask_from_segmentation(segmentation, ignore_values)

    def load_undistorted_combined_mask(self, image):
        """Combine undistorted binary mask with segmentation mask.

        Return a mask that is non-zero only where the binary
        mask and the segmentation mask are non-zero.
        """
        mask = None
        if self.undistorted_mask_exists(image):
            mask = self.load_undistorted_mask(image)
        smask = None
        if self.undistorted_segmentation_exists(image):
            smask = self.load_undistorted_segmentation_mask(image)
        return self.base._combine_masks(mask, smask)

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

    def pruned_depthmap_exists(self, image):
        return os.path.isfile(self._depthmap_file(image, 'pruned.npz'))

    def save_pruned_depthmap(self, image, points, normals, colors, labels, detections):
        io.mkdir_p(self._depthmap_path())
        filepath = self._depthmap_file(image, 'pruned.npz')
        np.savez_compressed(filepath,
                            points=points, normals=normals,
                            colors=colors, labels=labels,
                            detections=detections)

    def load_pruned_depthmap(self, image):
        o = np.load(self._depthmap_file(image, 'pruned.npz'))
        if 'detections' not in o:
            return o['points'], o['normals'], o['colors'], o['labels'], np.zeros(o['labels'].shape)
        else:
            return o['points'], o['normals'], o['colors'], o['labels'], o['detections']

    def load_undistorted_tracks_graph(self):
        return self.base.load_tracks_graph(os.path.join(self.subfolder, 'tracks.csv'))

    def save_undistorted_tracks_graph(self, graph):
        return self.base.save_tracks_graph(graph, os.path.join(self.subfolder, 'tracks.csv'))

    def load_undistorted_reconstruction(self):
        return self.base.load_reconstruction(
            filename=os.path.join(self.subfolder, 'reconstruction.json'))

    def save_undistorted_reconstruction(self, reconstruction):
        io.mkdir_p(self.data_path)
        return self.base.save_reconstruction(
            reconstruction, filename=os.path.join(self.subfolder, 'reconstruction.json'))
