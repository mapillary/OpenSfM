
from PIL import Image
import time

from opensfm import features
import numpy as np
import os
from collections import OrderedDict
import json
from matplotlib.image import _rgb_to_rgba


class Database:
    def __init__(self, seqs, path, preload_images=True):
        self.points = OrderedDict()
        self.seqs = seqs
        self.path = path
        self.image_cache = {}

        if preload_images:
            print("Preloading images")
            for keys in self.seqs.values():
                for k in keys:
                    self.get_image(k)

        p_gcp_errors = self.path + '/gcp_reprojections.json'
        if os.path.exists(p_gcp_errors):
            self.load_gcp_reprojections(p_gcp_errors)
        else:
            self.gcp_reprojections = {}

    def load_gcp_reprojections(self, filename):
        with open(filename, 'r') as f:
            d = json.load(f)

        for gcp_id in d:
            for shot_id in d[gcp_id]:
                h, w = self.get_image_size(shot_id)
                reproj = d[gcp_id][shot_id]['reprojection']
                reproj = features.denormalized_image_coordinates(np.array([reproj]), w, h)[0]
                d[gcp_id][shot_id]['reprojection'] = reproj

        self.gcp_reprojections = d

    def go_to_image_path(self):
        img_folder = self.path + "/images/"
        return img_folder

    def init_points(self, points):
        for point in points:
            point_id, observations = point['id'], point['observations']
            for observation in observations:
                h, w = self.get_image_size(observation["shot_id"])
                observation["projection"] = features.denormalized_image_coordinates(
                    np.array([observation["projection"]]), w, h)[0]
            self.points[point_id] = observations

    def get_image(self, img_name, max_size=1000):
        if img_name not in self.image_cache:
            rgb = Image.open(self.go_to_image_path() + img_name)

            # Reduce to some reasonable maximum size
            scale = max(rgb.size) / max_size
            if scale > 1:
                new_w = int(round(rgb.size[0] / scale))
                new_h = int(round(rgb.size[1] / scale))
                rgb = rgb.resize((new_w, new_h), resample=Image.BILINEAR)

            # Matplotlib will transform to rgba when plotting
            self.image_cache[img_name] = _rgb_to_rgba(np.asarray(rgb))
        return self.image_cache[img_name]

    def get_image_size(self, img_name):
        img = self.get_image(img_name)
        return img.shape[:2]

    def get_visible_points_coords(self, main_image):
        visible_points_coords = OrderedDict()
        for point_id, observations in self.points.items():
            for observation in observations:
                if observation["shot_id"] == main_image:
                    visible_points_coords[point_id] = observation["projection"]

        return visible_points_coords

    def point_exists(self, point_id):
        return point_id in self.points

    def point_sees(self, point, image):
        for obs in self.points[point]:
            if image == obs['shot_id']:
                return True
        return False

    def add_point(self, point_id):
        if self.point_exists(point_id):
            print('ERROR: trying to add an existing point')
            return
        self.points[point_id] = []

    def add_point_observation(self, point_id, shot_id, projection):
        if not self.point_exists(point_id):
            print('ERROR: trying to modify a non-existing point')
            return
        self.points[point_id].append({
            "shot_id": shot_id,
            "projection": projection,
        })

    def write_to_file(self, filename):
        data = {"points": []}
        for point_id, observations in self.points.items():
            point = {"id": point_id, "observations": []}
            for observation in observations:
                h, w = self.get_image_size(observation["shot_id"])
                scaled_projection = features.normalized_image_coordinates(
                    np.array([observation["projection"]]), w, h)[0].tolist()
                point["observations"].append({
                    "shot_id": observation["shot_id"],
                    "projection": scaled_projection,
                })
            data["points"].append(point)

        with open(filename, 'wt') as fp:
            json.dump(data, fp, indent=4, sort_keys=True)

    def compute_gcp_errors(self):
        error_avg = {}
        worst_gcp_error = 0
        worst_gcp = None
        shot_worst_gcp = None
        for gcp_id in self.points:
            error_avg[gcp_id] = 0
        for gcp_id in self.gcp_reprojections:
            if gcp_id not in self.points:
                continue
            for shot_id in self.gcp_reprojections[gcp_id]:
                err = self.gcp_reprojections[gcp_id][shot_id]['error']
                error_avg[gcp_id] += err
                if err > worst_gcp_error:
                    worst_gcp_error = err
                    shot_worst_gcp = shot_id
                    worst_gcp = gcp_id
            error_avg[gcp_id] /= len(self.gcp_reprojections[gcp_id])

        return worst_gcp, shot_worst_gcp, worst_gcp_error, error_avg

    def shot_with_max_gcp_error(self, image_keys, gcp):
        # Return they key with most reprojection error for this GCP
        annotated_images = set(self.gcp_reprojections[gcp]).intersection(set(image_keys))
        errors = {k: self.gcp_reprojections[gcp][k]['error'] for k in annotated_images}
        return max(errors, key=lambda k: errors[k])

    def remove_gcp(self, point_id):
        if self.point_exists(point_id):
            del self.points[point_id]

    def remove_point_observation(self, point_id, shot_id):
        if not self.point_exists(point_id):
            print('ERROR: trying to modify a non-existing point')
            return
        self.points[point_id] = [obs for obs in self.points[point_id] if obs["shot_id"] != shot_id]
        if point_id in self.gcp_reprojections:
            if shot_id in self.gcp_reprojections[point_id]:
                self.gcp_reprojections[point_id][shot_id]['error'] = 0

    def bring_adjacent_image(self, image, skey, offset):
        self.get_image_index(image, skey)
        next_idx = self.get_image_index + offset
        return self.bring_image(skey, next_idx)

    def get_image_index(self, image, skey):
        seq = self.seqs[skey]
        return seq.index(image)

    def bring_image(self, skey, idx):
        seq = self.seqs[skey]
        return seq[max(0, min(idx, len(seq) - 1))]
