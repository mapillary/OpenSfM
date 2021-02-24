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
import pandas as pd
from matplotlib.image import _rgb_to_rgba
from opensfm import dataset
from PIL import Image
from rtree import index

from opensfm.dataset import DataSet
from export_reconstruction_points import world_points

IMAGE_MAX_SIZE = 2000
MIN_OBLIQUE_ANGLE = 10
MAX_OBLIQUE_ANGLE = 55
OBLIQUE_DATA_FILE = 'Photos.csv'


def get_distance(lat1, lon1, alt1, lat2, lon2, alt2):
    ecef_str = 'epsg:4978'
    ll_str = 'epsg:4326'
    ecef_trans = Transformer.from_proj(Proj(ll_str), Proj(ecef_str))
    x1, y1, z1 = ecef_trans.transform(lat1, lon1, alt1)
    x2, y2, z2 = ecef_trans.transform(lat2, lon2, alt2)
    distance = np.sqrt((x1-x2)**2+(y1-y2)**2+(z1-z2)**2)
    return distance


def is_oblique(omega, phi):
    """
    Given omega, phi, kappa in degrees, return the angle that describes the
    obliqueness of the image (where 0 is ortho.)
    """
    return (np.abs(omega) < MAX_OBLIQUE_ANGLE) and (np.abs(phi) < MAX_OBLIQUE_ANGLE) \
        and (np.abs(omega) > MIN_OBLIQUE_ANGLE or np.abs(phi > MIN_OBLIQUE_ANGLE))


def oblique_rotation_angle(omega, phi, kappa):
    """
    Given omega, phi, kappa in degrees, return the rotation angle needed
    to reorient the image
    """
    if kappa < 0 and np.abs(omega) > MIN_OBLIQUE_ANGLE and omega < 0:
        return kappa + 180
    elif kappa < 0 and np.abs(phi) > MIN_OBLIQUE_ANGLE and phi < 0:
        return kappa + 90
    elif kappa > 0 and np.abs(phi) > MIN_OBLIQUE_ANGLE and phi > 0:
        return kappa - 90
    elif kappa > 0 and np.abs(omega) > MIN_OBLIQUE_ANGLE and omega > 0:
        return kappa
    # don't reorient images that are ortho
    else:
        return 0


def coords_in_rotated_image(point, theta, image_shape=(5120, 5120)):
    """
        point: (x, y) coordinates of point in original image
        theta: rotation angle in degrees
        x_len, y_len: size of original image in pixels
    """
    x, y = point

    x_len, y_len = image_shape
    x_centered, y_centered = x - x_len/2, y - y_len/2
    theta_rad = np.pi * theta / 180

    x_centered_rotated = x_centered*np.cos(theta_rad) + y_centered*np.sin(theta_rad)
    y_centered_rotated = y_centered*np.cos(theta_rad) - x_centered*np.sin(theta_rad)
    x_len_rotated = np.abs(x_len*np.cos(theta_rad)) + np.abs(y_len*np.sin(theta_rad))
    y_len_rotated = np.abs(y_len*np.cos(theta_rad)) + np.abs(x_len*np.sin(theta_rad))

    x_rotated = x_len_rotated/2 + x_centered_rotated
    y_rotated = y_len_rotated/2 + y_centered_rotated
    return int(np.round(x_rotated)), int(np.round(y_rotated))


def invert_coords_from_rotated_image(point, theta, original_image_shape=(5120, 5120)):
    x_rot, y_rot = point
    x_len, y_len = original_image_shape

    theta_rad = np.pi * theta / 180

    x_len_rot = np.abs(x_len*np.cos(theta_rad)) + np.abs(y_len*np.sin(theta_rad))
    y_len_rot = np.abs(y_len*np.cos(theta_rad)) + np.abs(x_len*np.sin(theta_rad))
    x_rot_centered, y_rot_centered = x_rot - x_len_rot/2, y_rot - y_len_rot/2

    x_centered = x_rot_centered*np.cos(theta_rad) - y_rot_centered*np.sin(theta_rad)
    y_centered = y_rot_centered*np.cos(theta_rad) + x_rot_centered*np.sin(theta_rad)
    x = x_centered + x_len/2
    y = y_centered + y_len/2

    return int(np.round(x)), int(np.round(y))


class ObliqueManager:
    def __init__(self, path: str, preload_images=True):
        self.path = Path(path)
        self.rtree_path=f'{self.path}/rtree_index'
        self.image_cache = {}
        self.image_coord = {}
        self.image_rotations = {}
        self.candidate_images = []
        self.preload_bol = preload_images
        self.ds = DataSet(self.path)
        self.get_rtree_index()
        self.image_data_path = f"{self.path}/{OBLIQUE_DATA_FILE}"
        self.image_metadata = pd.read_csv(self.image_data_path, usecols=['PhotoName', 'OmegaDeg', 'PhiDeg', 'KappaDeg'])
        self.image_metadata.set_index('PhotoName', inplace=True)

    def image_path(self, image_name):
        return f"{self.path}/images/{image_name}"

    def get_image(self, image_name):
        # image_name has the convention {root_name}_{px}_{py}
        # the image retrieved has been cropped
        if image_name not in self.image_cache:
            path = self.image_path(image_name)
            px = image_name.split('_')[-2]
            py = image_name.split('_')[-1]
            self.image_cache[image_name] = self.load_image((path, px, py))

        img_rgb = self.image_cache[image_name]

        return img_rgb

    def load_latlons(self):
        # 'canonical' latlon not as useful for obliques
        return {}

    def get_candidates(self, lat: float, lon: float, filter_obliques=True):
        """
        Given a lat lon alt, find prospective oblique images
        TODO: add alt as arg, make 3d
        """
        if lat is None or lon is None:
            return []

        aerial_match = self.aerial_idx.nearest(
            (lon, lat), objects=True)

        self.aerial_matches = [x.object['images'] for x in aerial_match][0]
        self.image_names = [x['image_name']
                            for x in self.aerial_matches ]
        print(f"Found {len(self.aerial_matches)} aerial images")
        if self.preload_bol:
            self.preload_images()


        return self.image_names

    def get_rtree_index(self):

        if os.path.exists(f'{self.rtree_path}.dat'):
            self.aerial_idx = index.Index(self.rtree_path)
        else:
            self.build_rtree_index()
            self.aerial_idx = index.Index(self.rtree_path)

    def build_rtree_index(self):
        print("building oblique SfM rtree...")

        data = world_points(self.ds)
        aerial_keypoints = []
        p = index.Property()
        p.dimension = 2
        aerial_idx = index.Index(self.rtree_path, properties=p)
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
            aerial_idx.insert(i, (lon, lat), obj=pt)

        aerial_idx.close()

    def preload_images(self):
        n_cpu = multiprocessing.cpu_count()
        print(f"Preloading images with {n_cpu} processes")
        paths = []
        image_names = []
        for match in self.aerial_matches:
            image_names.append(match['image_name'])
            paths.append(
                (self.image_path(match['image_id']), match['x_px_int'], (match['y_px_int'])))
        pool = multiprocessing.Pool(processes=n_cpu)
        images = pool.map(self.load_image, paths)
        for image_name, im, path in zip(image_names, images, paths):
            self.image_cache[image_name] = im
            self.image_coord[image_name] = (path[1:])

    def get_image_size(self, image_name):
        return self.get_image(image_name).shape[:2]

    def get_offsets(self, image_name, rotate=True):
        px, py = self.image_coord[image_name]
        height, width = self.get_image_size(image_name)

        if rotate:
            theta = self.get_rotation_angle(image_name)
            px, py = coords_in_rotated_image((px, py), theta)

        win = int(IMAGE_MAX_SIZE/2)
        y1 = np.max([py-win, 0])
        x1 = np.max([px-win, 0])
        return x1, y1

    def get_nearest_feature(self, image_name, x, y):
        return None

    def get_rotation_angle(self, image_name):
        """
        This gets the rotation angle in degrees for reorienting aerial images.

        The image name can be either for the original image ({root}) or
        the cropped version, e.g. with {root}_{px}_{py}

        """
        root_name = image_name.split('_')[0] + '_' + image_name.split('_')[1]
        omega, phi, kappa = self.image_metadata.loc[root_name]
        theta = oblique_rotation_angle(omega, phi, kappa)
        return theta


    def load_image(self, in_tuple, win=int(IMAGE_MAX_SIZE/2), rotate=True):
        '''
        Load an image around a pixel location. The input px and py are the
        coordinates of the feature in the original image

        Inputs
        ------
        in_tuple : tuple
            (path, px, py)
            path: str, px: int, py:int
        '''

        # package for pool
        path, px, py = in_tuple
        rgb = Image.open(path)
        width, height = rgb.size

        if rotate:
            theta = self.get_rotation_angle(os.path.basename(path))
            rgb = rgb.rotate(theta, resample=Image.BICUBIC, expand=True)
            px, py = coords_in_rotated_image((px, py), theta, (height, width))

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
