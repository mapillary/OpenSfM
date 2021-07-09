import gzip
import json
import logging
import os
import pickle
from abc import ABC, abstractmethod
from io import BytesIO
from typing import Dict, List, Tuple, Optional, IO, Any

import cv2
import numpy as np
from opensfm import config, features, geo, io, upright, pygeometry, types, pymap

logger = logging.getLogger(__name__)


class DataSetBase(ABC):
    """Base for dataset classes providing i/o access to persistent data.

    It is possible to store data remotely or in different formats
    by subclassing this class and overloading its methods.
    """

    @property
    @abstractmethod
    def io_handler(self) -> io.IoFilesystemBase:
        pass

    @property
    @abstractmethod
    def config(self) -> Dict[str, Any]:
        pass

    @abstractmethod
    def images(self) -> List[str]:
        pass

    @abstractmethod
    def open_image_file(self, image: str) -> IO[Any]:
        pass

    @abstractmethod
    def load_image(
        self,
        image: str,
        unchanged: bool = False,
        anydepth: bool = False,
        grayscale: bool = False,
    ) -> np.ndarray:
        pass

    @abstractmethod
    def image_size(self, image: str) -> Tuple[int, int]:
        pass

    @abstractmethod
    def load_mask(self, image: str) -> Optional[np.ndarray]:
        pass

    @abstractmethod
    def load_features_mask(self, image: str, points: np.ndarray) -> np.ndarray:
        pass

    @abstractmethod
    def load_instances(self, image: str) -> Optional[np.ndarray]:
        pass

    @abstractmethod
    def segmentation_labels(self) -> List[Any]:
        pass

    @abstractmethod
    def load_segmentation(self, image: str) -> Optional[np.ndarray]:
        pass

    @abstractmethod
    def segmentation_ignore_values(self, image: str) -> List[int]:
        pass

    @abstractmethod
    def mask_from_segmentation(
        self, segmentation: np.ndarray, ignore_values: List[int]
    ) -> np.ndarray:
        pass

    @abstractmethod
    def combine_masks(
        self, mask: Optional[np.ndarray], smask: Optional[np.ndarray]
    ) -> Optional[np.ndarray]:
        pass

    @abstractmethod
    def load_exif(self, image: str) -> Dict[str, Any]:
        pass

    @abstractmethod
    def save_exif(self, image: str, data: Dict[str, Any]) -> None:
        pass

    @abstractmethod
    def exif_exists(self, image: str) -> bool:
        pass

    @abstractmethod
    def feature_type(self) -> str:
        pass

    @abstractmethod
    def features_exist(self, image: str) -> bool:
        pass

    @abstractmethod
    def load_features(self, image: str) -> Optional[features.FeaturesData]:
        pass

    @abstractmethod
    def save_features(self, image: str, features_data: features.FeaturesData) -> None:
        pass

    @abstractmethod
    def words_exist(self, image: str) -> bool:
        pass

    @abstractmethod
    def load_words(self, image: str) -> np.ndarray:
        pass

    @abstractmethod
    def save_words(self, image: str, words: np.ndarray) -> None:
        pass

    @abstractmethod
    def matches_exists(self, image: str) -> bool:
        pass

    @abstractmethod
    def load_matches(self, image: str) -> Dict[str, np.ndarray]:
        pass

    @abstractmethod
    def save_matches(self, image: str, matches: Dict[str, np.ndarray]) -> None:
        pass

    @abstractmethod
    def load_tracks_manager(
        self, filename: Optional[str] = None
    ) -> pymap.TracksManager:
        pass

    @abstractmethod
    def save_tracks_manager(
        self, tracks_manager: pymap.TracksManager, filename: Optional[str] = None
    ) -> None:
        pass

    @abstractmethod
    def load_reconstruction(
        self, filename: Optional[str] = None
    ) -> List[types.Reconstruction]:
        pass

    @abstractmethod
    def save_reconstruction(
        self,
        reconstruction: List[types.Reconstruction],
        filename: Optional[str] = None,
        minify=False,
    ) -> None:
        pass

    @abstractmethod
    def invent_reference_lla(
        self, images: Optional[List[str]] = None
    ) -> Dict[str, float]:
        pass

    @abstractmethod
    def load_reference(self) -> geo.TopocentricConverter:
        pass

    @abstractmethod
    def reference_lla_exists(self) -> bool:
        pass

    @abstractmethod
    def load_camera_models(self) -> Dict[str, pygeometry.Camera]:
        pass

    @abstractmethod
    def save_camera_models(self, camera_models: Dict[str, pygeometry.Camera]) -> None:
        pass

    @abstractmethod
    def camera_models_overrides_exists(self) -> bool:
        pass

    @abstractmethod
    def load_camera_models_overrides(self) -> Dict[str, pygeometry.Camera]:
        pass

    @abstractmethod
    def save_camera_models_overrides(
        self, camera_models: Dict[str, pygeometry.Camera]
    ) -> None:
        pass

    @abstractmethod
    def exif_overrides_exists(self) -> bool:
        pass

    @abstractmethod
    def load_exif_overrides(self) -> Dict[str, Any]:
        pass

    @abstractmethod
    def load_rig_cameras(
        self,
    ) -> Dict[str, pymap.RigCamera]:
        pass

    @abstractmethod
    def save_rig_cameras(self, rig_cameras: Dict[str, pymap.RigCamera]) -> None:
        pass

    @abstractmethod
    def load_rig_assignments(self) -> List[List[Tuple[str, str]]]:
        pass

    @abstractmethod
    def load_rig_assignments_per_image(
        self,
    ) -> Dict[str, Tuple[int, str, List[str]]]:
        pass

    @abstractmethod
    def save_rig_assignments(self, rig_assignments: List[List[Tuple[str, str]]]):
        pass

    # TODO(pau): switch this to save_profile_log
    @abstractmethod
    def profile_log(self) -> str:
        pass

    @abstractmethod
    def load_report(self, path: str) -> str:
        pass

    @abstractmethod
    def save_report(self, report_str: str, path: str) -> None:
        pass

    @abstractmethod
    def load_ground_control_points(
        self,
    ) -> List[pymap.GroundControlPoint]:
        pass


class DataSet(DataSetBase):
    """Accessors to the main input and output data.

    Data include input images, masks, and segmentation as well
    temporary data such as features and matches and the final
    reconstructions.

    All data is stored inside a single folder with a specific subfolder
    structure.

    It is possible to store data remotely or in different formats
    by subclassing this class and overloading its methods.
    """

    io_handler: io.IoFilesystemBase = io.IoFilesystemDefault()
    config = None
    image_files: Dict[str, str] = {}
    mask_files: Dict[str, str] = {}
    image_list: List[str] = []

    def __init__(self, data_path: str, io_handler=io.IoFilesystemDefault) -> None:
        """Init dataset associated to a folder."""
        self.io_handler = io_handler
        self.data_path = data_path
        self.load_config()
        self.load_image_list()
        self.load_mask_list()

    def _config_file(self) -> str:
        return os.path.join(self.data_path, "config.yaml")

    def load_config(self) -> None:
        config_file_path = self._config_file()
        if self.io_handler.isfile(config_file_path):
            with self.io_handler.open(config_file_path) as f:
                self.config = config.load_config_from_fileobject(f)
        else:
            self.config = config.default_config()

    def _image_list_file(self) -> str:
        return os.path.join(self.data_path, "image_list.txt")

    def load_image_list(self) -> None:
        """Load image list from image_list.txt or list images/ folder."""
        image_list_file = self._image_list_file()
        if self.io_handler.isfile(image_list_file):
            with self.io_handler.open_rt(image_list_file) as fin:
                lines = fin.read().splitlines()
            self._set_image_list(lines)
        else:
            self._set_image_path(os.path.join(self.data_path, "images"))

    def images(self) -> List[str]:
        """List of file names of all images in the dataset."""
        return self.image_list

    def _image_file(self, image: str) -> str:
        """Path to the image file."""
        return self.image_files[image]

    def open_image_file(self, image: str) -> IO[Any]:
        """Open image file and return file object."""
        return self.io_handler.open(self._image_file(image), "rb")

    def load_image(
        self,
        image: str,
        unchanged: bool = False,
        anydepth: bool = False,
        grayscale: bool = False,
    ) -> np.ndarray:
        """Load image pixels as numpy array.

        The array is 3D, indexed by y-coord, x-coord, channel.
        The channels are in RGB order.
        """
        return self.io_handler.imread(
            self._image_file(image),
            unchanged=unchanged,
            anydepth=anydepth,
            grayscale=grayscale,
        )

    def image_size(self, image: str) -> Tuple[int, int]:
        """Height and width of the image."""
        return self.io_handler.image_size(self._image_file(image))

    def load_mask_list(self) -> None:
        """Load mask list from mask_list.txt or list masks/ folder."""
        mask_list_file = os.path.join(self.data_path, "mask_list.txt")
        if self.io_handler.isfile(mask_list_file):
            with self.io_handler.open_rt(mask_list_file) as fin:
                lines = fin.read().splitlines()
            self._set_mask_list(lines)
        else:
            self._set_mask_path(os.path.join(self.data_path, "masks"))

    def load_mask(self, image: str) -> Optional[np.ndarray]:
        """Load image mask if it exists, otherwise return None."""
        if image in self.mask_files:
            mask_path = self.mask_files[image]
            mask = self.io_handler.imread(mask_path, grayscale=True)
            if mask is None:
                raise IOError(
                    "Unable to load mask for image {} "
                    "from file {}".format(image, mask_path)
                )
        else:
            mask = None
        return mask

    def load_features_mask(self, image: str, points: np.ndarray) -> np.ndarray:
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

    def _instances_path(self) -> str:
        return os.path.join(self.data_path, "instances")

    def _instances_file(self, image: str) -> str:
        return os.path.join(self._instances_path(), image + ".png")

    def load_instances(self, image: str) -> Optional[np.ndarray]:
        """Load image instances file if it exists, otherwise return None."""
        instances_file = self._instances_file(image)
        if self.io_handler.isfile(instances_file):
            instances = self.io_handler.imread(instances_file, grayscale=True)
        else:
            instances = None
        return instances

    def _segmentation_path(self) -> str:
        return os.path.join(self.data_path, "segmentations")

    def _segmentation_file(self, image: str) -> str:
        return os.path.join(self._segmentation_path(), image + ".png")

    def segmentation_labels(self) -> List[Any]:
        return []

    def load_segmentation(self, image: str) -> Optional[np.ndarray]:
        """Load image segmentation if it exists, otherwise return None."""
        segmentation_file = self._segmentation_file(image)
        if self.io_handler.isfile(segmentation_file):
            segmentation = self.io_handler.imread(segmentation_file, grayscale=True)
        else:
            segmentation = None
        return segmentation

    def segmentation_ignore_values(self, image: str) -> List[int]:
        """List of label values to ignore.

        Pixels with this labels values will be masked out and won't be
        processed when extracting features or computing depthmaps.
        """
        return self.config.get("segmentation_ignore_values", [])

    def load_segmentation_mask(self, image: str) -> Optional[np.ndarray]:
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

        return self.mask_from_segmentation(segmentation, ignore_values)

    def mask_from_segmentation(
        self, segmentation: np.ndarray, ignore_values: List[int]
    ) -> np.ndarray:
        mask = np.ones(segmentation.shape, dtype=np.uint8)
        for value in ignore_values:
            mask &= segmentation != value
        return mask

    def load_combined_mask(self, image: str) -> Optional[np.ndarray]:
        """Combine binary mask with segmentation mask.

        Return a mask that is non-zero only where the binary
        mask and the segmentation mask are non-zero.
        """
        mask = self.load_mask(image)
        smask = self.load_segmentation_mask(image)
        return self.combine_masks(mask, smask)

    def combine_masks(
        self, mask: Optional[np.ndarray], smask: Optional[np.ndarray]
    ) -> Optional[np.ndarray]:
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

    def _resize_masks_to_match(
        self,
        im1: np.ndarray,
        im2: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        h, w = max(im1.shape, im2.shape)
        if im1.shape != (h, w):
            im1 = cv2.resize(im1, (w, h), interpolation=cv2.INTER_NEAREST)
        if im2.shape != (h, w):
            im2 = cv2.resize(im2, (w, h), interpolation=cv2.INTER_NEAREST)
        return im1, im2

    def _is_image_file(self, filename: str) -> bool:
        extensions = {"jpg", "jpeg", "png", "tif", "tiff", "pgm", "pnm", "gif"}
        return filename.split(".")[-1].lower() in extensions

    def _set_image_path(self, path: str) -> None:
        """Set image path and find all images in there"""
        self.image_list = []
        self.image_files = {}
        if self.io_handler.exists(path):
            for name in self.io_handler.ls(path):
                if self._is_image_file(name):
                    self.image_list.append(name)
                    self.image_files[name] = os.path.join(path, name)

    def _set_image_list(self, image_list: List[str]) -> None:
        self.image_list = []
        self.image_files = {}
        for line in image_list:
            path = os.path.join(self.data_path, line)
            name = os.path.basename(path)
            self.image_list.append(name)
            self.image_files[name] = path

    def _set_mask_path(self, path: str) -> None:
        """Set mask path and find all masks in there"""
        self.mask_files = {}
        if self.io_handler.isdir(path):
            files = set(self.io_handler.ls(path))
            for image in self.images():
                mask = image + ".png"
                if mask in files:
                    self.mask_files[image] = os.path.join(path, mask)

    def _set_mask_list(self, mask_list_lines: List[str]) -> None:
        self.mask_files = {}
        for line in mask_list_lines:
            image, relpath = line.split(None, 1)
            path = os.path.join(self.data_path, relpath.strip())
            self.mask_files[image.strip()] = path

    def _exif_path(self) -> str:
        """Return path of extracted exif directory"""
        return os.path.join(self.data_path, "exif")

    def _exif_file(self, image: str) -> str:
        """
        Return path of exif information for given image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._exif_path(), image + ".exif")

    def load_exif(self, image: str) -> Dict[str, Any]:
        """Load pre-extracted image exif metadata."""
        with self.io_handler.open_rt(self._exif_file(image)) as fin:
            return json.load(fin)

    def save_exif(self, image: str, data: Dict[str, Any]) -> None:
        self.io_handler.mkdir_p(self._exif_path())
        with self.io_handler.open_wt(self._exif_file(image)) as fout:
            io.json_dump(data, fout)

    def exif_exists(self, image: str) -> bool:
        return self.io_handler.isfile(self._exif_file(image))

    def feature_type(self) -> str:
        """Return the type of local features (e.g. AKAZE, SURF, SIFT)"""
        feature_name = self.config["feature_type"].lower()
        if self.config["feature_root"]:
            feature_name = "root_" + feature_name
        return feature_name

    def _feature_path(self) -> str:
        """Return path of feature descriptors and FLANN indices directory"""
        return os.path.join(self.data_path, "features")

    def _feature_file(self, image: str) -> str:
        """
        Return path of feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._feature_path(), image + ".features.npz")

    def _feature_file_legacy(self, image: str) -> str:
        """
        Return path of a legacy feature file for specified image
        :param image: Image name, with extension (i.e. 123.jpg)
        """
        return os.path.join(self._feature_path(), image + ".npz")

    def _save_features(
        self, filepath: str, features_data: features.FeaturesData
    ) -> None:
        self.io_handler.mkdir_p(self._feature_path())
        with self.io_handler.open(filepath, "wb") as fwb:
            features_data.save(fwb, self.config)

    def features_exist(self, image: str) -> bool:
        return self.io_handler.isfile(
            self._feature_file(image)
        ) or self.io_handler.isfile(self._feature_file_legacy(image))

    def load_features(self, image: str) -> Optional[features.FeaturesData]:
        features_filepath = (
            self._feature_file_legacy(image)
            if self.io_handler.isfile(self._feature_file_legacy(image))
            else self._feature_file(image)
        )
        with self.io_handler.open(features_filepath, "rb") as f:
            return features.FeaturesData.from_file(f, self.config)

    def save_features(self, image: str, features_data: features.FeaturesData) -> None:
        self._save_features(self._feature_file(image), features_data)

    def _words_file(self, image: str) -> str:
        return os.path.join(self._feature_path(), image + ".words.npz")

    def words_exist(self, image: str) -> bool:
        return self.io_handler.isfile(self._words_file(image))

    def load_words(self, image: str) -> np.ndarray:
        with self.io_handler.open(self._words_file(image), "rb") as f:
            s = np.load(f)
            return s["words"].astype(np.int32)

    def save_words(self, image: str, words: np.ndarray) -> None:
        with self.io_handler.open(self._words_file(image), "wb") as f:
            np.savez_compressed(f, words=words.astype(np.uint16))

    def _matches_path(self) -> str:
        """Return path of matches directory"""
        return os.path.join(self.data_path, "matches")

    def _matches_file(self, image: str) -> str:
        """File for matches for an image"""
        return os.path.join(self._matches_path(), "{}_matches.pkl.gz".format(image))

    def matches_exists(self, image: str) -> bool:
        return self.io_handler.isfile(self._matches_file(image))

    def load_matches(self, image: str) -> Dict[str, np.ndarray]:
        with self.io_handler.open(self._matches_file(image), "rb") as fin:
            matches = pickle.load(BytesIO(gzip.decompress(fin.read())))
        return matches

    def save_matches(self, image: str, matches: Dict[str, np.ndarray]) -> None:
        self.io_handler.mkdir_p(self._matches_path())

        with BytesIO() as buffer:
            with gzip.GzipFile(fileobj=buffer, mode="w") as fzip:
                # pyre-fixme[6]: Expected `IO[bytes]` for 2nd param but got `GzipFile`.
                pickle.dump(matches, fzip)
            with self.io_handler.open(self._matches_file(image), "wb") as fw:
                fw.write(buffer.getvalue())

    def find_matches(self, im1: str, im2: str) -> np.ndarray:
        if self.matches_exists(im1):
            im1_matches = self.load_matches(im1)
            if im2 in im1_matches:
                return im1_matches[im2]
        if self.matches_exists(im2):
            im2_matches = self.load_matches(im2)
            if im1 in im2_matches:
                if len(im2_matches[im1]):
                    return im2_matches[im1][:, [1, 0]]
        return np.array([])

    def _tracks_manager_file(self, filename: Optional[str] = None) -> str:
        """Return path of tracks file"""
        return os.path.join(self.data_path, filename or "tracks.csv")

    def load_tracks_manager(
        self, filename: Optional[str] = None
    ) -> pymap.TracksManager:
        """Return the tracks manager"""
        with self.io_handler.open(self._tracks_manager_file(filename), "r") as f:
            return pymap.TracksManager.instanciate_from_string(f.read())

    def tracks_exists(self, filename: Optional[str] = None) -> bool:
        return self.io_handler.isfile(self._tracks_manager_file(filename))

    def save_tracks_manager(
        self, tracks_manager: pymap.TracksManager, filename: Optional[str] = None
    ) -> None:
        with self.io_handler.open(self._tracks_manager_file(filename), "w") as fw:
            fw.write(tracks_manager.as_string())

    def _reconstruction_file(self, filename: Optional[str]) -> str:
        """Return path of reconstruction file"""
        return os.path.join(self.data_path, filename or "reconstruction.json")

    def reconstruction_exists(self, filename: Optional[str] = None) -> bool:
        return self.io_handler.isfile(self._reconstruction_file(filename))

    def load_reconstruction(
        self, filename: Optional[str] = None
    ) -> List[types.Reconstruction]:
        with self.io_handler.open_rt(self._reconstruction_file(filename)) as fin:
            reconstructions = io.reconstructions_from_json(io.json_load(fin))
        return reconstructions

    def save_reconstruction(
        self,
        reconstruction: List[types.Reconstruction],
        filename: Optional[str] = None,
        minify=False,
    ) -> None:
        with self.io_handler.open_wt(self._reconstruction_file(filename)) as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify)

    def _reference_lla_path(self) -> str:
        return os.path.join(self.data_path, "reference_lla.json")

    def invent_reference_lla(
        self, images: Optional[List[str]] = None
    ) -> Dict[str, float]:
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

    def save_reference_lla(self, reference: Dict[str, float]) -> None:
        with self.io_handler.open_wt(self._reference_lla_path()) as fout:
            io.json_dump(reference, fout)

    def load_reference_lla(self) -> Dict[str, float]:
        with self.io_handler.open_rt(self._reference_lla_path()) as fin:
            return io.json_load(fin)

    def load_reference(self) -> geo.TopocentricConverter:
        """Load reference as a topocentric converter."""
        lla = self.load_reference_lla()
        return geo.TopocentricConverter(
            lla["latitude"], lla["longitude"], lla["altitude"]
        )

    def reference_lla_exists(self) -> bool:
        return self.io_handler.isfile(self._reference_lla_path())

    def _camera_models_file(self) -> str:
        """Return path of camera model file"""
        return os.path.join(self.data_path, "camera_models.json")

    def load_camera_models(self) -> Dict[str, pygeometry.Camera]:
        """Return camera models data"""
        with self.io_handler.open_rt(self._camera_models_file()) as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def save_camera_models(self, camera_models: Dict[str, pygeometry.Camera]) -> None:
        """Save camera models data"""
        with self.io_handler.open_wt(self._camera_models_file()) as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def _camera_models_overrides_file(self) -> str:
        """Path to the camera model overrides file."""
        return os.path.join(self.data_path, "camera_models_overrides.json")

    def camera_models_overrides_exists(self) -> bool:
        """Check if camera overrides file exists."""
        return self.io_handler.isfile(self._camera_models_overrides_file())

    def load_camera_models_overrides(self) -> Dict[str, pygeometry.Camera]:
        """Load camera models overrides data."""
        with self.io_handler.open_rt(self._camera_models_overrides_file()) as fin:
            obj = json.load(fin)
            return io.cameras_from_json(obj)

    def save_camera_models_overrides(
        self, camera_models: Dict[str, pygeometry.Camera]
    ) -> None:
        """Save camera models overrides data"""
        with self.io_handler.open_wt(self._camera_models_overrides_file()) as fout:
            obj = io.cameras_to_json(camera_models)
            io.json_dump(obj, fout)

    def _exif_overrides_file(self) -> str:
        """Path to the EXIF overrides file."""
        return os.path.join(self.data_path, "exif_overrides.json")

    def exif_overrides_exists(self) -> bool:
        """Check if EXIF overrides file exists."""
        return self.io_handler.isfile(self._exif_overrides_file())

    def load_exif_overrides(self) -> Dict[str, Any]:
        """Load EXIF overrides data."""
        with self.io_handler.open_rt(self._exif_overrides_file()) as fin:
            return json.load(fin)

    def _rig_cameras_file(self) -> str:
        """Return path of rig models file"""
        return os.path.join(self.data_path, "rig_cameras.json")

    def load_rig_cameras(self) -> Dict[str, pymap.RigCamera]:
        """Return rig models data"""
        if not self.io_handler.exists(self._rig_cameras_file()):
            return {}
        with self.io_handler.open_rt(self._rig_cameras_file()) as fin:
            return io.rig_cameras_from_json(json.load(fin))

    def save_rig_cameras(self, rig_cameras: Dict[str, pymap.RigCamera]) -> None:
        """Save rig models data"""
        with self.io_handler.open_wt(self._rig_cameras_file()) as fout:
            io.json_dump(io.rig_cameras_to_json(rig_cameras), fout)

    def _rig_assignments_file(self) -> str:
        """Return path of rig assignments file"""
        return os.path.join(self.data_path, "rig_assignments.json")

    def load_rig_assignments(self) -> List[List[Tuple[str, str]]]:
        """Return rig assignments  data"""
        if not self.io_handler.exists(self._rig_assignments_file()):
            return []
        with self.io_handler.open_rt(self._rig_assignments_file()) as fin:
            return json.load(fin)

    def load_rig_assignments_per_image(
        self,
    ) -> Dict[str, Tuple[int, str, List[str]]]:
        """Return rig assignments  data"""
        raw_assignments = self.load_rig_assignments()
        assignments_per_image = {}
        for instance_id, instance in enumerate(raw_assignments):
            instance_shots = [s[0] for s in instance]
            for (shot_id, rig_camera_id) in instance:
                assignments_per_image[shot_id] = (
                    instance_id,
                    rig_camera_id,
                    instance_shots,
                )
        return assignments_per_image

    def save_rig_assignments(self, rig_assignments: List[List[Tuple[str, str]]]):
        """Save rig assignments  data"""
        with self.io_handler.open_wt(self._rig_assignments_file()) as fout:
            io.json_dump(rig_assignments, fout)

    def profile_log(self) -> str:
        "Filename where to write timings."
        return os.path.join(self.data_path, "profile.log")

    def _report_path(self) -> str:
        return os.path.join(self.data_path, "reports")

    def load_report(self, path: str) -> str:
        """Load a report file as a string."""
        with self.io_handler.open_rt(os.path.join(self._report_path(), path)) as fin:
            return fin.read()

    def save_report(self, report_str: str, path: str) -> None:
        """Save report string to a file."""
        filepath = os.path.join(self._report_path(), path)
        self.io_handler.mkdir_p(os.path.dirname(filepath))
        with self.io_handler.open_wt(filepath) as fout:
            return fout.write(report_str)

    def _ply_file(self, filename: Optional[str]):
        return os.path.join(self.data_path, filename or "reconstruction.ply")

    def save_ply(
        self,
        reconstruction: types.Reconstruction,
        tracks_manager: pymap.TracksManager,
        filename: Optional[str] = None,
        no_cameras: bool = False,
        no_points: bool = False,
        point_num_views: bool = False,
    ):
        """Save a reconstruction in PLY format."""
        ply = io.reconstruction_to_ply(
            reconstruction, tracks_manager, no_cameras, no_points, point_num_views
        )
        with self.io_handler.open_wt(self._ply_file(filename)) as fout:
            fout.write(ply)

    def _ground_control_points_file(self) -> str:
        return os.path.join(self.data_path, "ground_control_points.json")

    def _gcp_list_file(self) -> str:
        return os.path.join(self.data_path, "gcp_list.txt")

    def load_ground_control_points(self) -> List[pymap.GroundControlPoint]:
        """Load ground control points.

        It uses reference_lla to convert the coordinates
        to topocentric reference frame.
        """

        reference = self.load_reference()
        return self.load_ground_control_points_impl(reference)

    def load_ground_control_points_impl(
        self, reference: Optional[geo.TopocentricConverter]
    ) -> List[pymap.GroundControlPoint]:
        """Load ground control points.

        It might use reference to convert the coordinates
        to topocentric reference frame.
        If reference is None, it won't initialize topocentric data,
        thus allowing loading raw data only.
        """
        exif = {image: self.load_exif(image) for image in self.images()}

        gcp = []
        if self.io_handler.isfile(self._gcp_list_file()):
            with self.io_handler.open_rt(self._gcp_list_file()) as fin:
                gcp = io.read_gcp_list(fin, reference, exif)

        pcs = []
        if self.io_handler.isfile(self._ground_control_points_file()):
            with self.io_handler.open_rt(self._ground_control_points_file()) as fin:
                pcs = io.read_ground_control_points(fin, reference)

        return gcp + pcs

    def image_as_array(self, image: str) -> np.ndarray:
        logger.warning("image_as_array() is deprecated. Use load_image() instead.")
        return self.load_image(image)

    def mask_as_array(self, image: str) -> Optional[np.ndarray]:
        logger.warning("mask_as_array() is deprecated. Use load_mask() instead.")
        return self.load_mask(image)

    def subset(self, name: str, images_subset: List[str]) -> "DataSet":
        """Create a subset of this dataset by symlinking input data."""
        subset_dataset_path = os.path.join(self.data_path, name)
        self.io_handler.mkdir_p(subset_dataset_path)
        self.io_handler.mkdir_p(os.path.join(subset_dataset_path, "images"))
        self.io_handler.mkdir_p(os.path.join(subset_dataset_path, "segmentations"))
        subset_dataset = DataSet(subset_dataset_path, self.io_handler)

        files = []
        for method in [
            "_camera_models_file",
            "_config_file",
            "_camera_models_overrides_file",
            "_exif_overrides_file",
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
            if not self.io_handler.exists(src):
                continue
            self.io_handler.rm_if_exist(dst)
            self.io_handler.symlink(src, dst)

        return DataSet(subset_dataset_path, self.io_handler)

    def undistorted_dataset(self) -> "UndistortedDataSet":
        return UndistortedDataSet(
            self, os.path.join(self.data_path, "undistorted"), self.io_handler
        )


class UndistortedDataSet(object):
    """Accessors to the undistorted data of a dataset.

    Data include undistorted images, masks, and segmentation as well
    the undistorted reconstruction, tracks graph and computed depth maps.

    All data is stored inside the single folder ``undistorted_data_path``.
    By default, this path is set to the ``undistorted`` subfolder.
    """

    base: DataSetBase
    config: Dict[str, Any] = {}
    data_path: str

    def __init__(
        self,
        base_dataset: DataSetBase,
        undistorted_data_path: str,
        io_handler=io.IoFilesystemDefault,
    ):
        """Init dataset associated to a folder."""
        self.base = base_dataset
        self.config = self.base.config
        self.data_path = undistorted_data_path
        self.io_handler = io_handler

    def load_undistorted_shot_ids(self) -> Dict[str, List[str]]:
        filename = os.path.join(self.data_path, "undistorted_shot_ids.json")
        with self.io_handler.open_rt(filename) as fin:
            return io.json_load(fin)

    def save_undistorted_shot_ids(self, ushot_dict: Dict[str, List[str]]):
        filename = os.path.join(self.data_path, "undistorted_shot_ids.json")
        self.io_handler.mkdir_p(self.data_path)
        with self.io_handler.open_wt(filename) as fout:
            io.json_dump(ushot_dict, fout, minify=False)

    def _undistorted_image_path(self) -> str:
        return os.path.join(self.data_path, "images")

    def _undistorted_image_file(self, image: str) -> str:
        """Path of undistorted version of an image."""
        return os.path.join(self._undistorted_image_path(), image)

    def load_undistorted_image(self, image: str) -> np.ndarray:
        """Load undistorted image pixels as a numpy array."""
        return self.io_handler.imread(self._undistorted_image_file(image))

    def save_undistorted_image(self, image: str, array: np.ndarray) -> None:
        """Save undistorted image pixels."""
        self.io_handler.mkdir_p(self._undistorted_image_path())
        self.io_handler.imwrite(self._undistorted_image_file(image), array)

    def undistorted_image_size(self, image: str) -> Tuple[int, int]:
        """Height and width of the undistorted image."""
        return self.io_handler.image_size(self._undistorted_image_file(image))

    def _undistorted_mask_path(self) -> str:
        return os.path.join(self.data_path, "masks")

    def _undistorted_mask_file(self, image: str) -> str:
        """Path of undistorted version of a mask."""
        return os.path.join(self._undistorted_mask_path(), image + ".png")

    def undistorted_mask_exists(self, image: str) -> bool:
        """Check if the undistorted mask file exists."""
        return self.io_handler.isfile(self._undistorted_mask_file(image))

    def load_undistorted_mask(self, image: str) -> np.ndarray:
        """Load undistorted mask pixels as a numpy array."""
        return self.io_handler.imread(
            self._undistorted_mask_file(image), grayscale=True
        )

    def save_undistorted_mask(self, image: str, array: np.ndarray):
        """Save the undistorted image mask."""
        self.io_handler.mkdir_p(self._undistorted_mask_path())
        self.io_handler.imwrite(self._undistorted_mask_file(image), array)

    def _undistorted_segmentation_path(self) -> str:
        return os.path.join(self.data_path, "segmentations")

    def _undistorted_segmentation_file(self, image: str) -> str:
        """Path of undistorted version of a segmentation."""
        return os.path.join(self._undistorted_segmentation_path(), image + ".png")

    def undistorted_segmentation_exists(self, image: str) -> bool:
        """Check if the undistorted segmentation file exists."""
        return self.io_handler.isfile(self._undistorted_segmentation_file(image))

    def load_undistorted_segmentation(self, image: str) -> np.ndarray:
        """Load an undistorted image segmentation."""
        return self.io_handler.imread(
            self._undistorted_segmentation_file(image), grayscale=True
        )

    def save_undistorted_segmentation(self, image: str, array: np.ndarray) -> None:
        """Save the undistorted image segmentation."""
        self.io_handler.mkdir_p(self._undistorted_segmentation_path())
        self.io_handler.imwrite(self._undistorted_segmentation_file(image), array)

    def load_undistorted_segmentation_mask(self, image: str) -> Optional[np.ndarray]:
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

        return self.base.mask_from_segmentation(segmentation, ignore_values)

    def load_undistorted_combined_mask(self, image: str) -> Optional[np.ndarray]:
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
        return self.base.combine_masks(mask, smask)

    def _depthmap_path(self) -> str:
        return os.path.join(self.data_path, "depthmaps")

    def depthmap_file(self, image: str, suffix: str) -> str:
        """Path to the depthmap file"""
        return os.path.join(self._depthmap_path(), image + "." + suffix)

    def point_cloud_file(self, filename: str = "merged.ply") -> str:
        return os.path.join(self._depthmap_path(), filename)

    def load_point_cloud(
        self, filename: str = "merged.ply"
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self.io_handler.open(self.point_cloud_file(filename), "r") as fp:
            return io.point_cloud_from_ply(fp)

    def save_point_cloud(
        self,
        points: np.ndarray,
        normals: np.ndarray,
        colors: np.ndarray,
        labels: np.ndarray,
        filename: str = "merged.ply",
    ) -> None:
        self.io_handler.mkdir_p(self._depthmap_path())
        with self.io_handler.open(self.point_cloud_file(filename), "w") as fp:
            io.point_cloud_to_ply(points, normals, colors, labels, fp)

    def raw_depthmap_exists(self, image: str) -> bool:
        return self.io_handler.isfile(self.depthmap_file(image, "raw.npz"))

    def save_raw_depthmap(
        self,
        image: str,
        depth: np.ndarray,
        plane: np.ndarray,
        score: np.ndarray,
        nghbr: np.ndarray,
        nghbrs: np.ndarray,
    ) -> None:
        self.io_handler.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "raw.npz")
        with self.io_handler.open(filepath, "wb") as f:
            np.savez_compressed(
                f, depth=depth, plane=plane, score=score, nghbr=nghbr, nghbrs=nghbrs
            )

    def load_raw_depthmap(
        self, image: str
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self.io_handler.open(self.depthmap_file(image, "raw.npz"), "rb") as f:
            o = np.load(f)
            return o["depth"], o["plane"], o["score"], o["nghbr"], o["nghbrs"]

    def clean_depthmap_exists(self, image: str) -> bool:
        return self.io_handler.isfile(self.depthmap_file(image, "clean.npz"))

    def save_clean_depthmap(
        self, image: str, depth: np.ndarray, plane: np.ndarray, score: np.ndarray
    ):
        self.io_handler.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "clean.npz")
        with self.io_handler.open(filepath, "wb") as f:
            np.savez_compressed(f, depth=depth, plane=plane, score=score)

    def load_clean_depthmap(
        self, image: str
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        with self.io_handler.open(self.depthmap_file(image, "clean.npz"), "rb") as f:
            o = np.load(f)
            return o["depth"], o["plane"], o["score"]

    def pruned_depthmap_exists(self, image: str) -> bool:
        return self.io_handler.isfile(self.depthmap_file(image, "pruned.npz"))

    def save_pruned_depthmap(
        self,
        image: str,
        points: np.ndarray,
        normals: np.ndarray,
        colors: np.ndarray,
        labels: np.ndarray,
    ) -> None:
        self.io_handler.mkdir_p(self._depthmap_path())
        filepath = self.depthmap_file(image, "pruned.npz")
        with self.io_handler.open(filepath, "wb") as f:
            np.savez_compressed(
                f,
                points=points,
                normals=normals,
                colors=colors,
                labels=labels,
            )

    def load_pruned_depthmap(
        self, image: str
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        with self.io_handler.open(self.depthmap_file(image, "pruned.npz"), "rb") as f:
            o = np.load(f)
            return (
                o["points"],
                o["normals"],
                o["colors"],
                o["labels"],
            )

    def load_undistorted_tracks_manager(self) -> pymap.TracksManager:
        filename = os.path.join(self.data_path, "tracks.csv")
        with self.io_handler.open(filename, "r") as f:
            return pymap.TracksManager.instanciate_from_string(f.read())

    def save_undistorted_tracks_manager(
        self, tracks_manager: pymap.TracksManager
    ) -> None:
        filename = os.path.join(self.data_path, "tracks.csv")
        with self.io_handler.open(filename, "w") as fw:
            fw.write(tracks_manager.as_string())

    def load_undistorted_reconstruction(self) -> List[types.Reconstruction]:
        filename = os.path.join(self.data_path, "reconstruction.json")
        with self.io_handler.open_rt(filename) as fin:
            return io.reconstructions_from_json(io.json_load(fin))

    def save_undistorted_reconstruction(
        self, reconstruction: List[types.Reconstruction]
    ) -> None:
        filename = os.path.join(self.data_path, "reconstruction.json")
        self.io_handler.mkdir_p(self.data_path)
        with self.io_handler.open_wt(filename) as fout:
            io.json_dump(io.reconstructions_to_json(reconstruction), fout, minify=True)
