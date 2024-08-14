# pyre-unsafe
import logging
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Optional, IO, Any

import numpy as np
from opensfm import (
    features,
    geo,
    io,
    pygeometry,
    types,
    pymap,
)

logger: logging.Logger = logging.getLogger(__name__)


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
    def undistorted_segmentation_ignore_values(self, image: str) -> List[int]:
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
    def init_reference(self, images: Optional[List[str]] = None) -> None:
        pass

    @abstractmethod
    def reference_exists(self) -> bool:
        pass

    @abstractmethod
    def load_reference(self) -> geo.TopocentricConverter:
        pass

    @abstractmethod
    def save_reference(self, reference: geo.TopocentricConverter) -> None:
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
    def load_rig_assignments(self) -> Dict[str, List[Tuple[str, str]]]:
        pass

    @abstractmethod
    def save_rig_assignments(
        self, rig_assignments: Dict[str, List[Tuple[str, str]]]
    ) -> None:
        pass

    @abstractmethod
    def append_to_profile_log(self, content: str) -> None:
        pass

    @abstractmethod
    def load_report(self, path: str) -> str:
        pass

    @abstractmethod
    def save_report(self, report_str: str, path: str) -> None:
        pass

    @abstractmethod
    def load_ground_control_points(self) -> List[pymap.GroundControlPoint]:
        pass

    @abstractmethod
    def save_ground_control_points(
        self, points: List[pymap.GroundControlPoint]
    ) -> None:
        pass

    def clean_up(self) -> None:
        pass
