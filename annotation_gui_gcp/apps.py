from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

import numpy as np
import string
import random
from opensfm import types
import json
import opensfm.reconstruction as orec
from pprint import pprint
from opensfm import features
# from metrics import compute_distance
from mytk import Listbox


class ref:
    def __init__(self, obj): self.obj = obj

    def get(self):    return self.obj

    def set(self, obj):      self.obj = obj


distinct_colors = ['#46f0f0', '#f032e6', '#bcf60c', '#fabebe', '#008080',
                   '#9a6324', '#fffac8', '#800000', '#aaffc3', '#808000',
                   '#3cb44b', '#ffe119', '#4363d8', '#f58231', '#911eb4',
                   '#000075', '#808080', '#ffffff', '#000000']


class Imageset:

    def __init__(self):
        self.prev = None
        self.img = None
        self.next = None


def id_generator(size=6, chars=string.ascii_uppercase + string.digits):
    return ''.join(random.choice(chars) for _ in range(size))


def normalize_img_coords(pixel_coords, img_size):
    norm_points = []
    for point in pixel_coords:
        point = (point[0] / img_size[1] - 0.5, point[1] / img_size[0] - 0.5)
        norm_points.append(point)
    return norm_points


def denormalize_img_coords(norm_coords, img_size):
    denorm_points = []
    for point in norm_coords:
        point = ((point[0] + 0.5) * img_size[1], (point[1] + 0.5) * img_size[0])
        denorm_points.append(point)
    return denorm_points


def pix_coords(x, image):
    return features.denormalized_image_coordinates(
        np.array([[x[0], x[1]]]), image.shape[1], image.shape[0])[0]


def convert_tuple_cords_to_list(matched_points):
    list_form = [[] for i in range(2)]
    for point, observations in matched_points.items():
        list_form[0].append(observations[0])
        list_form[1].append(observations[1])

    return np.transpose(list_form)


def point_exists(reconstruction, point_id):
    for point in reconstruction.points.values():
        if point_id == point.id:
            return True
    return False


class CustomListbox(Listbox):
    def __contains__(self, str):
        return str in self.get(0, "end")


def merge_two_reconstructions(reconstructions):
    """"" MERGING POINTS, SHOTS AND CAMERAS"""

    combined_reconstruction = types.Reconstruction()
    for point in reconstructions[0].points.values():
        combined_reconstruction.add_point(point)
    for point2 in reconstructions[1].points.values():
        if not point_exists(reconstructions[0], point2.id):
            combined_reconstruction.add_point(point2)
    for recons in reconstructions:
        for shot in recons.shots.values():
            combined_reconstruction.add_shot(shot)

    for camera in reconstructions[0].cameras.values():
        combined_reconstruction.add_camera(camera)

    return [combined_reconstruction]


def save_gcp(gcp_test, gcp_train, path):
    for idx, gcp_set in enumerate([gcp_train, gcp_test]):
        data = {'points': []}
        for gcp in gcp_set:
            data['points'].append(
                {'id': gcp.id,

                 'observations': [
                     {"shot_id": gcp.observations[0].shot_id,
                      "projection": list(gcp.observations[0].projection),  # in normalized image coordinates

                      },
                     {
                         "shot_id": gcp.observations[1].shot_id,
                         "projection": list(gcp.observations[1].projection),  # in normalized image coordinates

                     }
                 ]
                 }
            )
        set_type = '_test' if idx else '_train'
        with open(path + '/ground_control_points' + set_type + '.json', 'w') as outfile:
            json.dump(data, outfile)


def read_gcp_file(file_path):
    file = open(file_path)
    data = json.load(file)
    return data['points']
