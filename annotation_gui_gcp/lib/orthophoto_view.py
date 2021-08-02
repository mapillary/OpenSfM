import typing as t

import numpy as np
import rasterio.warp
import rasterio.windows
from annotation_gui_gcp.lib.orthophoto_manager import OrthoPhotoManager
from annotation_gui_gcp.lib.view import View
from opensfm import features
from rasterio.io import DatasetReader


class OrthoPhotoView(View):
    def __init__(
        self,
        main_ui,
        path: str,
        init_lat: float,
        init_lon: float,
        is_the_geo_reference: bool = False,
    ):
        """[summary]

        Args:
            main_ui (GUI.Gui)
            path (str): path containing geotiffs
        """
        self.image_manager = OrthoPhotoManager(path, 100.0)
        self.images_in_list = self.image_manager.image_keys
        self.zoom_window_size_px = 300
        self.is_the_geo_reference = is_the_geo_reference
        self.geot: t.Optional[DatasetReader] = None
        self.image_window: t.Optional[rasterio.windows.Window] = None

        self.size = 50  # TODO add widget for zoom level

        super(OrthoPhotoView, self).__init__(main_ui, False)
        self.refocus(init_lat, init_lon)
        self.populate_image_list()
        if self.images_in_list:
            self.bring_new_image(self.images_in_list[0])
        self.set_title()

    def get_image(self, new_image: str):
        (
            crop,
            self.image_window,
            self.geot,
        ) = self.image_manager.read_image_around_latlon(
            new_image, self.center_lat, self.center_lon, self.size
        )
        return crop

    def get_candidate_images(self):
        return self.image_manager.get_candidate_images(
            self.center_lat, self.center_lon, self.size
        )

    def latlon_to_pixel(self, lat: float, lon: float) -> t.Tuple[float, float]:
        """
        From latlon to pixels (in the viewing window)
        """
        geot = self.geot
        image_window = self.image_window
        assert geot
        assert image_window
        # From WSG84 (lat/lon) to the image crs
        # pyre-fixme[16]: Module `DatasetReader` has no attribute `crs`.
        x, y = rasterio.warp.transform("EPSG:4326", geot.crs, [lon], [lat])

        # Image crs to pixel
        ys, xs = geot.index(x, y)
        x, y = xs[0], ys[0]

        # Offset by the viewing window
        x -= image_window.col_off
        y -= image_window.row_off

        return x, y

    def pixel_to_latlon(self, x: float, y: float):
        """
        From pixels (in the viewing window) to latlon
        """
        if not self.is_the_geo_reference:
            return None

        geot = self.geot
        image_window = self.image_window
        assert geot
        assert image_window

        # Offset by the viewing window
        x += image_window.col_off
        y += image_window.row_off

        # Pixel to whatever crs the image is in
        x, y = geot.xy(y, x)
        # And then to WSG84 (lat/lon)
        # pyre-fixme[16]: Module `DatasetReader` has no attribute `crs`.
        lons, lats = rasterio.warp.transform(geot.crs, "EPSG:4326", [x], [y])
        return lats[0], lons[0]

    def gcp_to_pixel_coordinates(self, x: float, y: float) -> t.Tuple[float, float]:
        """
        Transforms from normalized coordinates (in the whole geotiff) to
        pixels (in the viewing window)
        """
        image_window = self.image_window
        assert image_window
        h, w = self.image_manager.get_image_size(self.current_image)
        px = features.denormalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        x = px[0] - image_window.col_off
        y = px[1] - image_window.row_off
        # pyre-fixme[7]: Expected `t.Tuple[float, float]` but got `List[typing.Any]`.
        return [x, y]

    def pixel_to_gcp_coordinates(self, x: float, y: float) -> t.Tuple[float, float]:
        """
        Transforms from pixels (in the viewing window) to normalized coordinates
        (in the whole geotiff)
        """
        image_window = self.image_window
        assert image_window
        x += image_window.col_off
        y += image_window.row_off
        h, w = self.image_manager.get_image_size(self.current_image)
        coords = features.normalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        return coords.tolist()

    def refocus(self, lat, lon):
        self.center_lat = lat
        self.center_lon = lon
        self.populate_image_list()
        if self.images_in_list:
            if self.current_image not in self.images_in_list:
                self.bring_new_image(self.images_in_list[0])
            else:
                self.bring_new_image(self.current_image)
        self.set_title()

    def bring_new_image(self, new_image):
        super(OrthoPhotoView, self).bring_new_image(new_image, force=True)
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        artists = self.ax.plot(np.mean(xlim), np.mean(ylim), "rx")
        self.plt_artists.extend(artists)
        self.canvas.draw_idle()

    def set_title(self):
        lat, lon = self.center_lat, self.center_lon
        if self.images_in_list:
            t = "Images covering lat:{:.4f}, lon:{:.4f}".format(lat, lon)
            shot = self.current_image
            seq_ix = self.images_in_list.index(shot)
            title = f"{t} [{seq_ix+1}/{len(self.images_in_list)}]: {shot}"
        else:
            title = f"No orthophotos around {lat}, {lon}"
            self.current_image = None
            self.ax.clear()
            self.ax.axis("off")
            self.canvas.draw_idle()

        self.window.title(title)
