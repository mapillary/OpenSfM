from pathlib import Path

import rasterio
import rasterio.features
import rasterio.plot
import rasterio.warp
import rasterio.windows
from rasterio.plot import reshape_as_image


class OrthoPhotoManager:
    def __init__(self, path: str, size: float):
        self.path = Path(path)
        self.size = size
        self.image_keys = [p.name for p in self.path.iterdir() if p.suffix == ".tif"]

    def check_latlon_covered(self, img: str, lat: float, lon: float):
        t = rasterio.open(self.path / img)
        xs, ys = rasterio.warp.transform("EPSG:4326", t.crs, [lon], [lat], zs=None)
        row_center, col_center = t.index(xs[0], ys[0])
        window = rasterio.windows.Window(row_center, col_center, 1, 1)
        mask = t.read_masks(1, window=window, boundless=True)
        return mask.item != 0

    def read_image_around_latlon(self, img: str, lat: float, lon: float, size: float):
        t = rasterio.open(self.path / img)

        # From latlon to the coordinate reference system of the image in m
        xs, ys = rasterio.warp.transform("EPSG:4326", t.crs, [lon], [lat], zs=None)

        # Create the corners of the viewing window in pixels
        top, right = t.index(xs[0] + size / 2, ys[0] + size / 2)
        bottom, left = t.index(xs[0] - size / 2, ys[0] - size / 2)

        window = rasterio.windows.Window(left, top, right - left, bottom - top)
        self.current_window = window

        # TODO downsample image if the zoom level is too low / the image too large
        tw = reshape_as_image(t.read(window=window, boundless=True))
        return tw, window, t

    def get_candidate_images(self, lat: float, lon: float):
        # Returns only the images that cover this lat/lon
        # TODO actually filter around latlon, update list
        return [k for k in self.image_keys if self.check_latlon_covered(k, lat, lon)]

    def get_image(self, img, lat: float, lon: float, size: float):
        return self.read_image_around_latlon(img, lat, lon, size)

    def get_image_size(self, img):
        return self.current_window.height, self.current_window.width
