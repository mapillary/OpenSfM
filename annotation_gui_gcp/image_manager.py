import multiprocessing

import numpy as np
from matplotlib.image import _rgb_to_rgba
from opensfm import dataset
from PIL import Image
from rtree import index

from opensfm.dataset import DataSet
from opensfm import features
from export_reconstruction_points import world_points

IMAGE_MAX_SIZE = 1000


def load_image(path):
    rgb = Image.open(path)

    # Reduce to some reasonable maximum size
    scale = max(rgb.size) / IMAGE_MAX_SIZE
    if scale > 1:
        new_w = int(round(rgb.size[0] / scale))
        new_h = int(round(rgb.size[1] / scale))
        rgb = rgb.resize((new_w, new_h), resample=Image.BILINEAR)
    # Matplotlib will transform to rgba when plotting
    return _rgb_to_rgba(np.asarray(rgb))


class ImageManager:
    def __init__(self, seqs, path, preload_images=True):
        self.seqs = seqs
        self.path = path
        self.image_cache = {}

        if preload_images:
            self.preload_images()

    def image_path(self, image_name):
        return f"{self.path}/images/{image_name}"

    def get_image(self, image_name):
        if image_name not in self.image_cache:
            path = self.image_path(image_name)
            self.image_cache[image_name] = load_image(path)
        return self.image_cache[image_name]

    def load_latlons(self):
        data = dataset.DataSet(self.path)
        latlons = {}
        for keys in self.seqs.values():
            for k in keys:
                exif = data.load_exif(k)
                if "l" in exif:
                    latlons[k] = exif["l"]
                elif "gps" in exif:
                    latlons[k] = {
                        "lat": exif["gps"]["latitude"],
                        "lon": exif["gps"]["longitude"],
                    }
                elif "cl" in exif:
                    latlons[k] = exif["cl"]
        return latlons

    def preload_images(self):
        n_cpu = multiprocessing.cpu_count()
        print(f"Preloading images with {n_cpu} processes")
        paths = []
        image_names = []
        for keys in self.seqs.values():
            for k in keys:
                image_names.append(k)
                paths.append(self.image_path(k))
        pool = multiprocessing.Pool(processes=n_cpu)
        images = pool.map(load_image, paths)
        for image_name, im in zip(image_names, images):
            self.image_cache[image_name] = im

    def get_image_size(self, image_name):
        return self.get_image(image_name).shape[:2]

    def get_raw_image_size(self, image_name):
        path = self.image_path(image_name)
        rgb = Image.open(path)
        return rgb.size

    def load_features(self, image_name):
        print(f'building pixel rtree for features in {image_name}...')
        ds = DataSet(self.path)
        data = world_points(ds)
        self.image_keypoints = {}
        for i, (key, val) in enumerate(data.items()):
            images = val['images']
            lat = val['location']['lat']
            lon = val['location']['lon']
            alt = val['location']['alt']
            for im in images:
                if im['image_id'] == image_name:
                    h, w = self.get_raw_image_size(image_name)
                    x_px, y_px = features.normalized_image_coordinates(
                        np.array([[im['x_px'], im['y_px']]]), w, h)[0]
                    pt = {'x_px': x_px,
                          'y_px': y_px,
                          'lat': lat,
                          'lon': lon,
                          'alt': alt}
                    if im['image_id'] not in self.image_keypoints:
                        self.image_keypoints[im['image_id']] = index.Index(
                            properties=index.Property())
                        self.image_keypoints[im['image_id']].insert(
                            0, (x_px, y_px), obj=pt)
                    else:
                        idx = self.image_keypoints[im['image_id']].get_size()+1
                        self.image_keypoints[im['image_id']].insert(
                            idx, (x_px, y_px), obj=pt)

    def get_nearest_feature(self, image_name, x, y):
        # arguably could just find the min distance here, but rtree makes repeated queires faster?
        if 'image_keypoints' not in dir(self) or image_name not in self.image_keypoints:
            self.load_features(image_name)

        ground_match = self.image_keypoints[image_name].nearest(
            (x, y), 1, objects=True)
        objs = [rt.object for rt in ground_match][0]
        xx = objs['x_px']
        yy = objs['y_px']

        dis = np.sqrt((x-xx)**2+(y-yy)**2)
        print(f'feature {dis} fraction of image away')
        dispixels = 0.1 #add as arg?
        if dis > dispixels:
            print(f'No features within {dispixels} fraction of the image')
            return (None, None)
        latlon = (objs['lat'], objs['lon'])
        print(f"nearest feature at ({latlon[0]},{latlon[1]})")
        return latlon
