from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

from opensfm import dataset
import numpy as np


def calc_epipol_line(point_coord, img_pair, path, main_image_idx):
    data = dataset.DataSet(path)
    reconstruction = data.load_reconstruction()
    if not reconstruction[0].get_shot(img_pair[main_image_idx]):
        main_shot = reconstruction[1].get_shot(img_pair[main_image_idx])
        pair_shot = reconstruction[0].get_shot(img_pair[not main_image_idx])
    else :
        main_shot = reconstruction[0].get_shot(img_pair[main_image_idx])
        pair_shot = reconstruction[0].get_shot(img_pair[not main_image_idx])

# CHANGE COMING FROM RECONS 1
    line_pts = [[] for i in range(2)]
    for depth in np.arange(-100, 100, 5):
        point3d = main_shot.back_project(point_coord, depth)
        reprojection = pair_shot.project(point3d)
        line_pts[0].append(reprojection[0])
        line_pts[1].append(reprojection[1])

    return np.transpose(line_pts)
