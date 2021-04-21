from pathlib import Path

import rasterio
import rasterio.features
import rasterio.plot
import rasterio.warp
import rasterio.windows
from rasterio.plot import reshape_as_image
from rasterio.io import DatasetReader


class OrthoPhotoManager:
    def __init__(self, path: str, size: float):
        self.path = Path(path)
        self.size = size
        self.image_keys = [p.name for p in self.path.iterdir() if p.suffix == ".tif"]
        self.candidate_cache = {}

    def window_around_latlon(
        self, t: DatasetReader, lat: float, lon: float, size: float
    ):
        # From latlon to the coordinate reference system of the image in m
        # pyre-fixme[16]: `DatasetReader` has no attribute `crs`.
        xs, ys = rasterio.warp.transform("EPSG:4326", t.crs, [lon], [lat], zs=None)

        # Create the corners of the viewing window in pixels
        top, right = t.index(xs[0] + size / 2, ys[0] + size / 2)
        bottom, left = t.index(xs[0] - size / 2, ys[0] - size / 2)
        # pyre-fixme[19]: Expected 0 positional arguments.
        window = rasterio.windows.Window(left, top, right - left, bottom - top)
        return window

    def check_latlon_covered(self, img: str, lat: float, lon: float, size: float):
        t = rasterio.open(self.path / img)
        window = self.window_around_latlon(t, lat, lon, size)
        mask = t.read_masks(1, window=window, boundless=True)
        return mask.sum() > 0

    def load_latlons(self):
        # We don't have a 'canonical' lat/lon for orthophotos
        return {}

    def read_image_around_latlon(self, img: str, lat: float, lon: float, size: float):
        t = rasterio.open(self.path / img)

        window = self.window_around_latlon(t, lat, lon, size)

        # TODO downsample image if the zoom level is too low / the image too large
        tw = reshape_as_image(t.read(window=window, boundless=True))
        return tw, window, t

    def get_candidate_images(self, lat: float, lon: float, size: float):
        # lat, lon -> nearmap file id + nearmap pixel coordinates
        if (lat, lon, size) not in self.candidate_cache:
            self.candidate_cache[(lat, lon, size)] = [
                k
                for k in self.image_keys
                if self.check_latlon_covered(k, lat, lon, size)
            ]
        return self.candidate_cache[(lat, lon, size)]

    def get_image(self, img, lat: float, lon: float, size: float):
        return self.read_image_around_latlon(img, lat, lon, size)

    def get_image_size(self, img):
        t = rasterio.open(self.path / img)
        return t.height, t.width
