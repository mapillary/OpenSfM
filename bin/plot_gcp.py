"""Plot image crops around GCPs.
"""

import argparse
import logging

import numpy as np
import matplotlib.pyplot as plt

import opensfm.reconstruction as orec
from opensfm import features
from opensfm import io
from opensfm import dataset

logger = logging.getLogger(__name__)


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__)
    parser.add_argument(
        'dataset',
        help='dataset',
    )
    return parser.parse_args()


def pix_coords(x, image):
    return features.denormalized_image_coordinates(
        np.array([[x[0], x[1]]]), image.shape[1], image.shape[0])[0]


def gcp_to_ply(gcps, reconstruction):
    """Export GCP position as a PLY string."""
    vertices = []

    for gcp in gcps:
        if gcp.coordinates.has_value:
            p = gcp.coordinates
        else:
            p = orec.triangulate_gcp(gcp, reconstruction.shots)

        if p is None:
            logger.warning("Could not compute the 3D position of GCP '{}'"
                           .format(gcp.id))
            continue

        c = 255, 0, 0
        s = "{} {} {} {} {} {}".format(
            p[0], p[1], p[2], int(c[0]), int(c[1]), int(c[2]))
        vertices.append(s)

    header = [
        "ply",
        "format ascii 1.0",
        "element vertex {}".format(len(vertices)),
        "property float x",
        "property float y",
        "property float z",
        "property uchar diffuse_red",
        "property uchar diffuse_green",
        "property uchar diffuse_blue",
        "end_header",
    ]

    return '\n'.join(header + vertices + [''])


def main():
    args = parse_args()
    logging.basicConfig(
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        level=logging.DEBUG)

    data = dataset.DataSet(args.dataset)
    reconstruction = data.load_reconstruction()[0]
    gcps = data.load_ground_control_points()

    with io.open_wt(data.data_path + '/gcp.ply') as fout:
        fout.write(gcp_to_ply(gcps, reconstruction))

    for gcp in gcps:
        plt.suptitle("GCP '{}'".format(gcp.id))

        if gcp.coordinates.has_value:
            coordinates = gcp.coordinates
        else:
            coordinates = orec.triangulate_gcp(gcp, reconstruction.shots)

        if coordinates is None:
            logger.warning("Could not compute the 3D position of GCP '{}'"
                           .format(gcp.id))
            continue

        for i, observation in enumerate(gcp.observations):
            image = data.load_image(observation.shot_id)
            shot = reconstruction.shots[observation.shot_id]

            reprojected = shot.project(coordinates)
            annotated = observation.projection
            rpixel = pix_coords(reprojected, image)
            apixel = pix_coords(annotated, image)

            n = (len(gcp.observations) + 3) / 4
            ax = plt.subplot(n, min(len(gcp.observations), 4), i + 1)
            plt.imshow(image)
            ax.title.set_text("{}".format(observation.shot_id))
            plt.scatter(rpixel[0], rpixel[1])
            plt.scatter(apixel[0], apixel[1])
        plt.show()


if __name__ == '__main__':
    main()
