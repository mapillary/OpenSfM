import multiprocessing
from functools import partial

import numpy as np
from matplotlib.image import _rgb_to_rgba
from opensfm import dataset
from PIL import Image


def load_image(path, max_image_size):
    rgb = Image.open(path)

    # Reduce to some reasonable maximum size
    scale = max(rgb.size) / max_image_size
    if scale > 1:
        new_w = int(round(rgb.size[0] / scale))
        new_h = int(round(rgb.size[1] / scale))
        rgb = rgb.resize((new_w, new_h), resample=Image.BILINEAR)

    # Support grayscale images
    if rgb.mode == "L":
        rgb = rgb.convert("RGB")

    # Matplotlib will transform to rgba when plotting
    return _rgb_to_rgba(np.asarray(rgb))


class ImageManager:
    def __init__(self, seqs, path, preload_images=True, max_image_size=1000):
        self.seqs = seqs
        self.path = path
        self.max_image_size = max_image_size
        self.image_cache = {}

        if preload_images:
            self.preload_images()

    def image_path(self, image_name):
        return f"{self.path}/images/{image_name}"

    def get_image(self, image_name):
        if image_name not in self.image_cache:
            path = self.image_path(image_name)
            self.image_cache[image_name] = load_image(path, self.max_image_size)
        return self.image_cache[image_name]

    def load_latlons(self):
        data = dataset.DataSet(self.path)
        latlons = {}
        for keys in self.seqs.values():
            for k in keys:
                if not data.exif_exists(k):
                    continue
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

        f_pool = partial(load_image, max_image_size=self.max_image_size)
        pool = multiprocessing.Pool(processes=n_cpu)
        images = pool.map(f_pool, paths)
        for image_name, im in zip(image_names, images):
            self.image_cache[image_name] = im

    def get_image_size(self, image_name):
        return self.get_image(image_name).shape[:2]
