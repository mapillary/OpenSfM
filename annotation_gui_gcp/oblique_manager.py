from opensfm.features import denormalized_image_coordinates
from opensfm.dataset import DataSet
from rtree import index
from PIL import Image
from opensfm import dataset
from matplotlib.image import _rgb_to_rgba
import numpy as np
import sys
from pathlib import Path
import multiprocessing
import os

import numpy as np
from matplotlib.image import _rgb_to_rgba
from opensfm import dataset
from PIL import Image
from rtree import index

from opensfm.dataset import DataSet
from opensfm.features import denormalized_image_coordinates

IMAGE_MAX_SIZE = 1000


def load_image(in_tuple, win=int(IMAGE_MAX_SIZE/2)):
    '''
    Load an image around a pixel location

    Inputs
    ------
    in_tuple : tuple
         (path, px, py)
         path: str, px: int, py:int
    '''

    # package for pool
    path, px, py = in_tuple
    rgb = Image.open(path)

    y1 = np.max([py-win, 0])
    y2 = np.min([py+win, rgb.height])
    x1 = np.max([px-win, 0])
    x2 = np.min([px+win, rgb.width])

    # use this to mark feature point?
    # will need to back out original px, py after click
    pt_x = np.min([px, win])
    pt_y = np.min([py, win])

    if win is not None:
        rgb = rgb.crop((x1, y1, x2, y2))

    # Matplotlib will transform to rgba when plotting
    return _rgb_to_rgba(np.asarray(rgb))


def world_points(ds: DataSet):
    # this is taken from export_reconstruction_points.py
    # not in public opensfm?

    output = {}

    tracks_manager = ds.load_tracks_manager()
    for reconstruction in ds.load_reconstruction():
        print(reconstruction.reference)
        for point in reconstruction.points.values():
            x, y, z = point.coordinates
            lat, lon, alt = reconstruction.reference.to_lla(x, y, z)
            images = []
            for shot_id, obs in tracks_manager.get_track_observations(point.id).items():
                if shot_id in reconstruction.shots:
                    shot = reconstruction.shots[shot_id]

                    image_id = shot_id
                    x_px, y_px = obs.point
                    x_px, y_px = denormalized_image_coordinates(
                        np.array([[x_px, y_px]]
                                 ), shot.camera.width, shot.camera.height
                    )[0]
                    images.append(
                        {"image_id": image_id, "x_px": x_px, "y_px": y_px})
            point_id = f"{point.id}"
            output[point_id] = {
                "location": {"lat": lat, "lon": lon, "alt": alt},
                "images": images,
            }
    return output


def get_distance(lat1, lon1, alt1, lat2, lon2, alt2):
    ecef_str = 'epsg:4978'
    ll_str = 'epsg:4326'
    ecef_trans = Transformer.from_proj(Proj(ll_str), Proj(ecef_str))
    x1, y1, z1 = ecef_trans.transform(lat1, lon1, alt1)
    x2, y2, z2 = ecef_trans.transform(lat2, lon2, alt2)
    distance = np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
    return distance


class ObliqueManager:
    def __init__(self, path: str, preload_images=True):
        self.path = Path(path)
        self.image_cache = {}
        self.preload_bol = preload_images
        self.build_rtree_index()

    def image_path(self, image_name):
        return f"{self.path}/images/{image_name}"

    def get_image(self, image_name):
        if image_name not in self.image_cache:
            path = self.image_path(image_name)
            self.image_cache[image_name] = load_image(path)
        return self.image_cache[image_name]

    def load_latlons(self):
        # 'canonical' latlon not as useful for obliques
        return {}

    def get_candidate_images(self, lat: float, lon: float):
        """
        Given a lat lon alt, find prospective oblique images
        TODO: add alt as arg
        """
        cross_buf = 1.0e-4  # what is this?

        aerial_matches = list(self.aerial_idx.intersection(
            (lon - cross_buf, lat - cross_buf,
             lon + cross_buf, lat + cross_buf), objects=True))

        self.aerial_matches = [x.object['images'] for x in aerial_matches]
        print(f"Found {len(self.aerial_matches)} aerial images")
        # TODO: sort by distance between lat, lon, alt of ground and aerial
        # and limit to nearest

        if self.preload_bol:
            self.preload_images()

        return

    def build_rtree_index(self):
        buf = 1.0e-13  # what is this?

        ds = DataSet(self.path)
        data = world_points(ds)
        aerial_keypoints = []
        aerial_idx = index.Index(properties=index.Property())
        for i, (key, val) in enumerate(data.items()):
            images = val['images']
            ims = []
            for im in images:
                xpx = int(np.round(im['x_px']))
                ypx = int(np.round(im['y_px']))
                imn = {'x_px_int': xpx,
                       'y_px_int': ypx,
                       'image_name': f"{im['image_id']}_{xpx}_{ypx}"}
                ims.append(dict(im, **imn))
            lat = val['location']['lat']
            lon = val['location']['lon']
            alt = val['location']['alt']
            pt = {'key': key, 'lat': lat, 'lon': lon,
                  'alt': alt, 'images': ims}
            aerial_keypoints.append(pt)
            aerial_idx.insert(i, (lon-buf, lat-buf, lon+buf, lat+buf), obj=pt)

        self.aerial_idx = aerial_idx

    def preload_images(self):
        n_cpu = multiprocessing.cpu_count()
        print(f"Preloading images with {n_cpu} processes")
        paths = []
        image_names = []
        for matches in self.aerial_matches:
            for match in matches:
                image_names.append(match['image_name'])
                paths.append(
                    (self.image_path(match['image_id']), match['x_px_int'], (match['y_px_int'])))
        pool = multiprocessing.Pool(processes=n_cpu)
        images = pool.map(load_image, paths)
        for image_name, im in zip(image_names, images):
            self.image_cache[image_name] = im

    def get_image_size(self, image_name):
        return self.get_image(image_name).shape[:2]


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    # show cached images
    path = sys.argv[1]
    # https://stackoverflow.com/questions/45720153/python-multiprocessing-error-attributeerror-module-main-has-no-attribute
    __spec__ = None
    image_manager = ObliqueManager(path)
    image_manager.get_candidate_images(47.614, -122.34677)
    for i, im in image_manager.image_cache.items():
        plt.figure()
        plt.imshow(im)
        plt.show()
