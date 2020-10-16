from typing import Tuple

import numpy as np
import rasterio.warp
from opensfm import features

import GUI
from orthophoto_manager import OrthoPhotoManager
from view import View


class OrthoPhotoView(View):
    def __init__(self, main_ui: GUI.Gui, path: str, is_geo_reference: bool = False):
        """[summary]

        Args:
            main_ui (GUI.Gui)
            path (str): path containing geotiffs
        """
        self.name = path.split("/")[-1]
        self.image_manager = OrthoPhotoManager(path, 100.0)
        self.image_keys = self.image_manager.image_keys
        self.zoom_window_size_px = 500
        self.current_image = self.image_keys[0]
        self.is_geo_reference = is_geo_reference

        lon, lat = -122.33500709776536, 47.61825766649998
        self.center_lat, self.center_lon = lat, lon
        self.group_name = "Images covering lat:{:.4f}, lon:{:.4f}".format(lat, lon)

        self.size = 50  # TODO add widget for zoom level
        super(OrthoPhotoView, self).__init__(main_ui, self.name)

    def get_image(self, new_image):
        image, image_window, geot = self.image_manager.read_image_around_latlon(
            new_image, self.center_lat, self.center_lon, self.size
        )
        self.image_window = image_window
        self.geot = geot
        return image

    def get_candidate_images(self):
        return self.image_manager.get_candidate_images(self.center_lat, self.center_lon)

    def pixel_to_latlon(self, x: float, y: float):
        """
        From pixels (in the viewing window) to latlon
        """
        if not self.is_geo_reference:
            return None

        # Pixel to whatever crs the image is in
        x, y = self.geot.xy(y, x)
        # And then to WSG84 (lat/lon)
        lons, lats = rasterio.warp.transform(self.geot.crs, "EPSG:4326", [x], [y])
        return lats[0], lons[0]

    def gcp_to_pixel_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from normalized coordinates (in the whole geotiff) to
        pixels (in the viewing window)
        """
        h, w = self.image_manager.get_image_size(self.current_image)
        px = features.denormalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        x = px[0] - self.image_window.col_off
        y = px[1] - self.image_window.row_off
        return [x, y]

    def pixel_to_gcp_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from pixels (in the viewing window) to normalized coordinates
        (in the whole geotiff)
        """
        x += self.image_window.col_off
        y += self.image_window.row_off
        h, w = self.image_manager.get_image_size(self.current_image)
        coords = features.normalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        return coords.tolist()
