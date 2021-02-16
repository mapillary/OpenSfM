import gzip
import json
import logging
import os
import pickle

import cv2
import numpy as np
from opensfm import config
from opensfm import features
from opensfm import geo
from opensfm import io
from opensfm import pysfm
from opensfm import upright

logger = logging.getLogger(__name__)


class DataSet(object):
    """Accessors to the main input and output data.

    Data include input images, masks, and segmentation as well
    temporary data such as features and matches and the final
    reconstructions.

    All data is stored inside a single folder with a specific subfolder
    structure.

    It is possible to store data remotely or in different formats
    by subclassing this class and overloading its methods.
    """

    def __init__(self, data_path):
        """Init dataset associated to a folder."""
        self.data_path = data_path
        self.load_config()
        self.load_image_list()
        self.load_mask_list()

    def _config_file(self):
        return os.path.join(self.data_path, "config.yaml")

    def load_config(self):
        self.config = config.load_config(self._config_file())

    def _image_list_file(self):
        return os.path.join(self.data_path, "image_list.txt")

    def load_image_list(self):
        """Load image list from image_list.txt or list images/ folder."""
        image_list_file = self._image_list_file()
        if os.path.isfile(image_list_file):
            with io.open_rt(image_list_file) as fin:
                lines = fin.read().splitlines()
            self._set_image_list(lines)
        else:
            self._set_image_path(os.path.join(self.data_path, "images"))

    def images(self):
        """List of file names of all images in the dataset."""
        return self.image_list

    def _image_file(self, image):
        """Path to the image file."""
        return self.image_files[image]

    def open_image_file(self, image):
        """Open image file and return file object."""
        return open(self._image_file(image), "rb")

    def load_image(self, image, unchanged=False, anydepth=False, grayscale=False):
        """Load image pixels as numpy array.

        The array is 3D, indexed by y-coord, x-coord, channel.
        The channels are in RGB order.
        """
        return io.imread(
            self._image_file(image),
            unchanged=unchanged,
            anydepth=anydepth,
            grayscale=grayscale,
        )

    def image_size(self, image):
        """Height and width of the image."""
        return io.image_size(self._image_file(image))

    def load_mask_list(self):
        """Load mask list from mask_list.txt or list masks/ folder."""
        mask_list_file = os.path.join(self.data_path, "mask_list.txt")
        if os.path.isfile(mask_list_file):
            with io.open_rt(mask_list_file) as fin:
                lines = fin.read().splitlines()
            self._set_mask_list(lines)
        else:
            self._set_mask_path(os.path.join(self.data_path, "masks"))

    def load_mask(self, image):
        """Load image mask if it exists, otherwise return None."""
        if image in self.mask_files:
            mask_path = self.mask_files[image]
            mask = io.imread(mask_path, grayscale=True)
            if mask is None:
                raise IOError(
                    "Unable to load mask for image {} "
                    "from file {}".format(image, mask_path)
                )
        else:
            mask = None
        return mask

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
            logger.debug("No segmentation for {}, no features masked.".format(image))
            return np.ones((points.shape[0],), dtype=bool)

        exif = self.load_exif(image)
        width = exif["width"]
        height = exif["height"]
        orientation = exif["orientation"]

        new_height, new_width = mask_image.shape
        ps = upright.opensfm_to_upright(
            points[:, :2],
            width,
            height,
            orientation,
            new_width=new_width,
            new_height=new_height,
        ).astype(int)
        mask = mask_image[ps[:, 1], ps[:, 0]]

        n_removed = np.sum(mask == 0)
        logger.debug(
            "Masking {} / {} ({:.2f}) features for {}".format(
                n_removed, len(mask), n_removed / len(mask), image
            )
        )

        return np.array(mask, dtype=bool)

    def _detection_path(self):
        return os.path.join(self.data_path, "detections")

    def _detection_file(self, image):
        return os.path.join(self._detection_path(), image + ".png")

    def load_detection(self, image):
        """Load image detection if it exists, otherwise return None."""
        detection_file = self._detection_file(image)
        if os.path.isfile(detection_file):
            detection = io.imread(detection_file, grayscale=True)
        else:
            detection = None
        return detection

    def _instances_path(self):
        return os.path.join(self.data_path, "instances")

    def _instances_file(self, image):
        return os.path.join(self._instances_path(), image + ".png")

    def load_instances(self, image):
        """Load image instances file if it exists, otherwise return None."""
        instances_file = self._instances_file(image)
        if os.path.isfile(instances_file):
            instances = io.imread(instances_file, grayscale=True)
        else:
            instances = None
        return instances

    def _segmentation_path(self):
        return os.path.join(self.data_path, "segmentations")

    def _segmentation_file(self, image):
        return os.path.join(self._segmentation_path(), image + ".png")

    def segmentation_labels(self):
        return []

    def load_segmentation(self, image):
        """Load image segmentation if it exists, otherwise return None."""
        segmentation_file = self._segmentation_file(image)
        if os.path.isfile(segmentation_file):
            segmentation = io.imread(segmentation_file, grayscale=True)
        else:
            segmentation = None
        return segmentation

    def segmentation_ignore_values(self, image):
        """List of label values to ignore.

        Pixels with this labels values will be masked out and won't be
        processed when extracting features or computing depthmaps.
        """
        return self.config.get("segmentation_ignore_values", [])

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

    def _mask_from_segmentation(self, segmentation, ignore_values):
        mask = np.ones(segmentation.shape, dtype=np.uint8)
        for value in ignore_values:
            mask &= segmentation != value
        return mask

    def load_combined_mask(self, image):
        """Combine binary mask with segmentation mask.

        Return a mask that is non-zero only where the binary
        mask and the segmentation mask are non-zero.
        """
        mask = self.load_mask(image)
        smask = self.load_segmentation_mask(image)
        return self._combine_masks(mask, smask)

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
                mask, smask = self._resize_masks_to_match(mask, smask)
                return mask & smask

    def _resize_masks_to_match(self, im1, im2):
        h, w = max(im1.shape, im2.shape)
        if im1.shape != (h, w):
            im1 = cv2.resize(im1, (w, h), interpolation=cv2.INTER_NEAREST)
        if im2.shape != (h, w):
            im2 = cv2.resize(im2, (w, h), interpolation=cv2.INTER_NEAREST)
        return im1, im2

    def _is_image_file(self, filename):
        extensions = {"jpg", "jpeg", "png", "tif", "tiff", "pgm", "pnm", "gif"}
        return filename.split(".")[-1].lower() in extensions

    def _set_image_path(self, path):
        """Set image path and find all images in there"""
        self.image_list = []
        self.image_files = {}
        if os.path.exists(path):
            for name in os.listdir(path):
                if self._is_image_file(name):
                    self.image_list.append(name)
                    self.image_files[name] = os.path.join(path, name)

    def _set_image_list(self, image_list):
        self.image_list = []
        self.image_files = {}
        for line in image_list:
            path = os.path.join(self.data_path, line)
            name = os.path.basename(path)
            self.image_list.append(name)
            self.image_files[name] = path

    def _set_mask_path(self, path):
        """Set mask path and find all masks in there"""
        self.mask_files = {}
        for image in self.images():
            filepath = os.path.join(path, image + ".png")
            if os.path.isfile(filepath):
                self.mask_files[image] = filepath

    def _set_mask_list(self, mask_list_lines):
        self.mask_files = {}
        for line in mask_list_lines:
            image, relpath = line.split(None, 1)
            path = os.path.join(self.data_path, relpath.strip())
            self.mask_files[image.strip()] = path

    def _exif_path(self):
        """Return path of extracted exif directory"""
        return os.path.join(self.data_path, "exif")

    def _exif_file(self, image):
        """
        Return path of exif information for given image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._exif_path(), image + ".exif")

    def load_exif(self, image):
        """Load pre-extracted image exif metadata."""
        with io.open_rt(self._exif_file(image)) as fin:
            return json.load(fin)

    def save_exif(self, image, data):
        io.mkdir_p(self._exif_path())
        with io.open_wt(self._exif_file(image)) as fout:
            io.json_dump(data, fout)

    def exif_exists(self, image):
        return os.path.isfile(self._exif_file(image))

    def feature_type(self):
        """Return the type of local features (e.g. AKAZE, SURF, SIFT)"""
        feature_name = self.config["feature_type"].lower()
        if self.config["feature_root"]:
            feature_name = "root_" + feature_name
        return feature_name

    def _feature_path(self):
        """Return path of feature descriptors and FLANN indices directory"""
        return os.path.join(self.data_path, "features")

    def _feature_file(self, image):
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._feature_path(), image + ".features.npz")

    def _feature_file_legacy(self, image):
        """
        Return path of a legacy feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._feature_path(), image + ".npz")

    def _save_features(
        self,
        filepath,
        points,
        descriptors,
        colors=None,
        segmentations=None,
        instances=None,
    ):
        io.mkdir_p(self._feature_path())
        features.save_features(
            filepath,
            points,
            descriptors,
            colors,
            segmentations,
            instances,
            self.segmentation_labels(),
            self.config,
        )

    def features_exist(self, image):
        return os.path.isfile(self._feature_file(image)) or os.path.isfile(
            self._feature_file_legacy(image)
        )

    def load_features(self, image):
        if os.path.isfile(self._feature_file_legacy(image)):
            return features.load_features(self._feature_file_legacy(image), self.config)
        return features.load_features(self._feature_file(image), self.config)

    def save_features(
        self, image, points, descriptors, colors, segmentations, instances
    ):
        self._save_features(
            self._feature_file(image),
            points,
            descriptors,
            colors,
            segmentations,
            instances,
        )

    def _words_file(self, image):
        return os.path.join(self._feature_path(), image + ".words.npz")

    def words_exist(self, image):
        return os.path.isfile(self._words_file(image))

    def load_words(self, image):
        s = np.load(self._words_file(image))
        return s["words"].astype(np.int32)

    def save_words(self, image, words):
        np.savez_compressed(self._words_file(image), words=words.astype(np.uint16))

    def _matches_path(self):
        """Return path of matches directory"""
        return os.path.join(self.data_path, "matches")

    def _matches_file(self, image):
        """File for matches for an image"""
        return os.path.join(self._matches_path(), "{}_matches.pkl.gz".format(image))

    def matches_exists(self, image):
        return os.path.isfile(self._matches_file(image))

    def load_matches(self, image):
        with gzip.open(self._matches_file(image), "rb") as fin:
            matches = pickle.load(fin)
        return matches

    def save_matches(self, image, matches):
        io.mkdir_p(self._matches_path())
        with gzip.open(self._matches_file(image), "wb") as fout:
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

    def _tracks_manager_file(self, filename=None):
        """Return path of tracks file"""
        return os.path.join(self.data_path, filename or "tracks.csv")

    def load_tracks_manager(self, filename=None):
        """Return the tracks manager"""
        return pysfm.TracksManager.instanciate_from_file(
            self._tracks_manager_file(filename)
        )

    def tracks_exists(self, filename=None):
        return os.path.isfile(self._tracks_manager_file(filename))

    def save_tracks_manager(self, tracks_manager, filename=None):
        tracks_manager.write_to_file(self._tracks_manager_file(filename))

    def _reconstruction_file(self, filename):
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, filename or "reconstruction.json")

    def reconstruction_exists(self, filename=None):
        return os.path.isfile(self._reconstruction_file(filename))

    def load_reconstruction(self, filename=None):
        with io.open_rt(self._reconstruction_file(filename)) as fin:
            reconstructions = io.reconstructions_from_json(io.json_load(fin))
        return reconstructions

    def save_reconstruction(self, reconstruction, filename=None, minify=False):
        with io.open_wt(self._reconstruction_file(filename)) as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify)

    def _reference_lla_path(self):
        return os.path.join(self.data_path, "reference_lla.json")

    def invent_reference_lla(self, images=None):
        lat, lon, alt = 0.0, 0.0, 0.0
        wlat, wlon, walt = 0.0, 0.0, 0.0
        if images is None:
            images = self.images()
        for image in images:
            d = self.load_exif(image)
            if "gps" in d and "latitude" in d["gps"] and "longitude" in d["gps"]:
                w = 1.0 / max(0.01, d["gps"].get("dop", 15))
                lat += w * d["gps"]["latitude"]
                lon += w * d["gps"]["longitude"]
                wlat += w
                wlon += w
                if "altitude" in d["gps"]:
                    alt += w * d["gps"]["altitude"]
                    walt += w

        if not wlat and not wlon:
            for gcp in self.load_ground_control_points_impl(None):
                lat += gcp.lla["latitude"]
                lon += gcp.lla["longitude"]
                wlat += 1
                wlon += 1

                if gcp.has_altitude:
                    alt += gcp.lla["altitude"]
                    walt += 1

        if wlat:
            lat /= wlat
        if wlon:
            lon /= wlon
        if walt:
            alt /= walt
        reference = {
            "latitude": lat,
            "longitude": lon,
            "altitude": 0,
        }  # Set altitude manually.
        self.save_reference_lla(reference)
        return reference

    def save_reference_lla(self, reference):
        with io.open_wt(self._reference_lla_path()) as fout:
            io.json_dump(reference, fout)

    def load_reference_lla(self):
        with io.open_rt(self._reference_lla_path()) as fin:
            return io.json_load(fin)

    def load_reference(self):
        """Load reference as a topocentric converter."""
        lla = self.load_reference_lla()
        return geo.TopocentricConverter(
            lla["latitude"], lla["longitude"], lla["altitude"]
        )

    def reference_lla_exists(self):
        return os.path.isfile(self._reference_lla_path())

    def _camera_models_file(self):
        """Return path of camera model file"""
        return os.path.join(self.data_path, "camera_models.json")

    def load_camera_models(self):
        """Return camera models data"""
        with io.open_rt(self._camera_models_file()) as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def save_camera_models(self, camera_models):
        """Save camera models data"""
        with io.open_wt(self._camera_models_file()) as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def _camera_models_overrides_file(self):
        """Path to the camera model overrides file."""
        return os.path.join(self.data_path, "camera_models_overrides.json")

    def camera_models_overrides_exists(self):
        """Check if camera overrides file exists."""
        return os.path.isfile(self._camera_models_overrides_file())

    def load_camera_models_overrides(self):
        """Load camera models overrides data."""
        with io.open_rt(self._camera_models_overrides_file()) as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def save_camera_models_overrides(self, camera_models):
        """Save camera models overrides data"""
        with io.open_wt(self._camera_models_overrides_file()) as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def _exif_overrides_file(self):
        """Path to the EXIF overrides file."""
        return os.path.join(self.data_path, "exif_overrides.json")

    def exif_overrides_exists(self):
        """Check if EXIF overrides file exists."""
        return os.path.isfile(self._exif_overrides_file())

    def load_exif_overrides(self):
        """Load EXIF overrides data."""
        with io.open_rt(self._exif_overrides_file()) as fin:
            return json.load(fin)

    def _rig_models_file(self):
        """Return path of rig models file"""
        return os.path.join(self.data_path, "rig_models.json")

    def load_rig_models(self):
        """Return rig models data"""
        with io.open_rt(self._rig_models_file()) as fin:
            return json.load(fin)

    def save_rig_models(self, rig_models):
        """Save rig models data"""
        with io.open_wt(self._rig_models_file()) as fout:
            io.json_dump(rig_models, fout)

    def _rig_assignments_file(self):
        """Return path of rig assignments file"""
        return os.path.join(self.data_path, "rig_assignments.json")

    def load_rig_assignments(self):
        """Return rig assignments  data"""
        with io.open_rt(self._rig_assignments_file()) as fin:
            return json.load(fin)

    def save_rig_assignments(self, rig_assignments):
        """Save rig assignments  data"""
        with io.open_wt(self._rig_assignments_file()) as fout:
            io.json_dump(rig_assignments, fout)

    def profile_log(self):
        "Filename where to write timings."
        return os.path.join(self.data_path, "profile.log")

    def _report_path(self):
        return os.path.join(self.data_path, "reports")

    def load_report(self, path):
        """Load a report file as a string."""
        with io.open_rt(os.path.join(self._report_path(), path)) as fin:
            return fin.read()

    def save_report(self, report_str, path):
        """Save report string to a file."""
        filepath = os.path.join(self._report_path(), path)
        io.mkdir_p(os.path.dirname(filepath))
        with io.open_wt(filepath) as fout:
            return fout.write(report_str)

    def _navigation_graph_file(self):
        "Return the path of the navigation graph."
        return os.path.join(self.data_path, "navigation_graph.json")

    def save_navigation_graph(self, navigation_graphs):
        with io.open_wt(self._navigation_graph_file()) as fout:
            io.json_dump(navigation_graphs, fout)

    def _ply_file(self, filename):
        return os.path.join(self.data_path, filename or "reconstruction.ply")

    def save_ply(
        self, reconstruction, tracks_manager, filename=None, no_cameras=False, no_points=False, point_num_views=False
    ):
        """Save a reconstruction in PLY format."""
        ply = io.reconstruction_to_ply(reconstruction, tracks_manager, no_cameras, no_points, point_num_views)
        with io.open_wt(self._ply_file(filename)) as fout:
            fout.write(ply)

    def _ground_control_points_file(self):
        return os.path.join(self.data_path, "ground_control_points.json")

    def _gcp_list_file(self):
        return os.path.join(self.data_path, "gcp_list.txt")

    def load_ground_control_points(self):
        """Load ground control points.

        It uses reference_lla to convert the coordinates
        to topocentric reference frame.
        """

        reference = self.load_reference()
        return self.load_ground_control_points_impl(reference)

    def load_ground_control_points_impl(self, reference):
        """Load ground control points.

        It might use reference to convert the coordinates
        to topocentric reference frame.
        If reference is None, it won't initialize topocentric data,
        thus allowing loading raw data only.
        """
        exif = {image: self.load_exif(image) for image in self.images()}

        gcp = []
        if os.path.isfile(self._gcp_list_file()):
            with io.open_rt(self._gcp_list_file()) as fin:
                gcp = io.read_gcp_list(fin, reference, exif)

        pcs = []
        if os.path.isfile(self._ground_control_points_file()):
            with io.open_rt(self._ground_control_points_file()) as fin:
                pcs = io.read_ground_control_points(fin, reference)

        return gcp + pcs

    def image_as_array(self, image):
        logger.warning("image_as_array() is deprecated. Use load_image() instead.")
        return self.load_image(image)

    def mask_as_array(self, image):
        logger.warning("mask_as_array() is deprecated. Use load_mask() instead.")
        return self.load_mask(image)

    def subset(self, name, images_subset):
        """ Create a subset of this dataset by symlinking input data. """
        subset_dataset_path = os.path.join(self.data_path, name)
        io.mkdir_p(subset_dataset_path)
        io.mkdir_p(os.path.join(subset_dataset_path, "images"))
        io.mkdir_p(os.path.join(subset_dataset_path, "segmentations"))
        subset_dataset = DataSet(subset_dataset_path)

        files = []
        for method in [
            "_camera_models_file",
            "_config_file",
            "_camera_models_overrides_file",
            "_exif_overrides_file",
            "_image_list_file",
        ]:
            files.append(
                (
                    getattr(self, method)(),
                    getattr(subset_dataset, method)(),
                )
            )
        for image in images_subset:
            files.append(
                (
                    self._image_file(image),
                    os.path.join(subset_dataset_path, "images", image),
                )
            )
            files.append(
                (
                    self._segmentation_file(image),
                    os.path.join(subset_dataset_path, "segmentations", image + ".png"),
                )
            )

        for src, dst in files:
            if not os.path.exists(src):
                continue

            if os.path.islink(dst):
                os.unlink(dst)
            if os.path.exists(dst):
                os.remove(dst)

            os.symlink(src, dst)

        return DataSet(subset_dataset_path)


class UndistortedDataSet(object):
    """Accessors to the undistorted data of a dataset.

    Data include undistorted images, masks, and segmentation as well
    the undistorted reconstruction, tracks graph and computed depth maps.

    All data is stored inside the single folder ``undistorted_data_path``.
    By default, this path is set to the ``undistorted`` subfolder.
    """

    def __init__(self, base_dataset, undistorted_data_path=None):
        """Init dataset associated to a folder."""
        self.base = base_dataset
        self.config = self.base.config
        if undistorted_data_path:
            self.data_path = undistorted_data_path
        else:
            self.data_path = os.path.join(self.base.data_path, "undistorted")

    def load_undistorted_shot_ids(self):
        filename = os.path.join(self.data_path, "undistorted_shot_ids.json")
        with io.open_rt(filename) as fin:
            return io.json_load(fin)

    def save_undistorted_shot_ids(self, ushot_dict):
        filename = os.path.join(self.data_path, "undistorted_shot_ids.json")
        io.mkdir_p(self.data_path)
        with io.open_wt(filename) as fout:
            io.json_dump(ushot_dict, fout, minify=False)

    def _undistorted_image_path(self):
        return os.path.join(self.data_path, "images")

    def _undistorted_image_file(self, image):
        """Path of undistorted version of an image."""
        return os.path.join(self._undistorted_image_path(), image)

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
        return os.path.join(self.data_path, "masks")

    def _undistorted_mask_file(self, image):
        """Path of undistorted version of a mask."""
        return os.path.join(self._undistorted_mask_path(), image + ".png")

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
        return os.path.join(self.data_path, "detections")

    def _undistorted_detection_file(self, image):
        """Path of undistorted version of a detection."""
        return os.path.join(self._undistorted_detection_path(), image + ".png")

    def undistorted_detection_exists(self, image):
        """Check if the undistorted detection file exists."""
        return os.path.isfile(self._undistorted_detection_file(image))

    def load_undistorted_detection(self, image):
        """Load an undistorted image detection."""
        return io.imread(self._undistorted_detection_file(image), grayscale=True)

    def save_undistorted_detection(self, image, array):
        """Save the undistorted image detection."""
        io.mkdir_p(self._undistorted_detection_path())
        io.imwrite(self._undistorted_detection_file(image), array)

    def _undistorted_segmentation_path(self):
        return os.path.join(self.data_path, "segmentations")

    def _undistorted_segmentation_file(self, image):
        """Path of undistorted version of a segmentation."""
        return os.path.join(self._undistorted_segmentation_path(), image + ".png")

    def undistorted_segmentation_exists(self, image):
        """Check if the undistorted segmentation file exists."""
        return os.path.isfile(self._undistorted_segmentation_file(image))

    def load_undistorted_segmentation(self, image):
        """Load an undistorted image segmentation."""
        return io.imread(self._undistorted_segmentation_file(image), grayscale=True)

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
        return os.path.join(self.data_path, "depthmaps")

    def depthmap_file(self, image, suffix):
        """Path to the depthmap file"""
        return os.path.join(self._depthmap_path(), image + "." + suffix)

    def point_cloud_file(self, filename="merged.ply"):
        return os.path.join(self._depthmap_path(), filename)

    def raw_depthmap_exists(self, image):
        return os.path.isfile(self.depthmap_file(image, "raw.npz"))

    def save_raw_depthmap(self, image, depth, plane, score, nghbr, nghbrs):
        io.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "raw.npz")
        np.savez_compressed(
            filepath, depth=depth, plane=plane, score=score, nghbr=nghbr, nghbrs=nghbrs
        )

    def load_raw_depthmap(self, image):
        o = np.load(self.depthmap_file(image, "raw.npz"))
        return o["depth"], o["plane"], o["score"], o["nghbr"], o["nghbrs"]

    def clean_depthmap_exists(self, image):
        return os.path.isfile(self.depthmap_file(image, "clean.npz"))

    def save_clean_depthmap(self, image, depth, plane, score):
        io.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "clean.npz")
        np.savez_compressed(filepath, depth=depth, plane=plane, score=score)

    def load_clean_depthmap(self, image):
        o = np.load(self.depthmap_file(image, "clean.npz"))
        return o["depth"], o["plane"], o["score"]

    def pruned_depthmap_exists(self, image):
        return os.path.isfile(self.depthmap_file(image, "pruned.npz"))

    def save_pruned_depthmap(self, image, points, normals, colors, labels, detections):
        io.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "pruned.npz")
        np.savez_compressed(
            filepath,
            points=points,
            normals=normals,
            colors=colors,
            labels=labels,
            detections=detections,
        )

    def load_pruned_depthmap(self, image):
        o = np.load(self.depthmap_file(image, "pruned.npz"))
        if "detections" not in o:
            return (
                o["points"],
                o["normals"],
                o["colors"],
                o["labels"],
                np.zeros(o["labels"].shape),
            )
        else:
            return o["points"], o["normals"], o["colors"], o["labels"], o["detections"]

    def load_undistorted_tracks_manager(self):
        filename = os.path.join(self.data_path, "tracks.csv")
        return pysfm.TracksManager.instanciate_from_file(filename)

    def save_undistorted_tracks_manager(self, tracks_manager):
        filename = os.path.join(self.data_path, "tracks.csv")
        return tracks_manager.write_to_file(filename)

    def load_undistorted_reconstruction(self):
        filename = os.path.join(self.data_path, "reconstruction.json")
        with io.open_rt(filename) as fin:
            return io.reconstructions_from_json(io.json_load(fin))

    def save_undistorted_reconstruction(self, reconstruction):
        filename = os.path.join(self.data_path, "reconstruction.json")
        io.mkdir_p(self.data_path)
        with io.open_wt(filename) as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify=True)
