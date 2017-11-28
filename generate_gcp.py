#!/usr/bin/env python
import argparse

import matplotlib.pyplot as plt
import numpy as np

from opensfm import dataset
from opensfm import features
from opensfm import geo

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generate GCP by sampling points from the reconstruction')
    parser.add_argument(
        'dataset',
        help='path to the dataset to be processed')
    parser.add_argument(
        '--num_points',
        default=3,
        type=int,
        help='number of points to generate')
    args = parser.parse_args()

    data = dataset.DataSet(args.dataset)
    reference = data.load_reference_lla()
    reconstruction = data.load_reconstruction()[0]

    print 'WGS84'
    for i in range(args.num_points):
        point = np.random.choice(reconstruction.points.values())

        for shot in reconstruction.shots.values():
            pixel = shot.project(point.coordinates)
            if np.fabs(pixel).max() < 0.5:

                lla = geo.lla_from_topocentric(
                    point.coordinates[0],
                    point.coordinates[1],
                    point.coordinates[2],
                    reference['latitude'],
                    reference['longitude'],
                    reference['altitude'])

                x, y = features.denormalized_image_coordinates(
                    pixel.reshape(1, 2), shot.camera.width, shot.camera.height)[0]

                print "{} {} {} {} {} {}".format(
                    lla[0], lla[1], lla[2],
                    x, y, shot.id)
