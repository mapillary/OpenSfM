import os

import numpy as np

from opensfm import dataset
from opensfm import io
from opensfm.dense import depthmap_to_ply, scale_down_image


def run_dataset(data, no_cameras, no_points, depthmaps):
    """ Export reconstruction to PLY format

    Args:
        no_cameras: do not save camera positions
        no_points: do not save points
        depthmaps: export per-image depthmaps as pointclouds

    """

    reconstructions = data.load_reconstruction()
    no_cameras = no_cameras
    no_points = no_points

    if reconstructions:
        data.save_ply(reconstructions[0], None, no_cameras, no_points)

    if depthmaps and reconstructions:
        udata = dataset.UndistortedDataSet(data, 'undistorted')
        for id, shot in reconstructions[0].shots.items():
            rgb = udata.load_undistorted_image(id)
            for t in ('clean', 'raw'):
                path_depth = udata._depthmap_file(id, t + '.npz')
                if not os.path.exists(path_depth):
                    continue
                depth = np.load(path_depth)['depth']
                rgb = scale_down_image(rgb, depth.shape[1], depth.shape[0])
                ply = depthmap_to_ply(shot, depth, rgb)
                with io.open_wt(udata._depthmap_file(id, t + '.ply')) as fout:
                    fout.write(ply)
