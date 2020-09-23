import os
import logging

import numpy as np
import pyproj

from opensfm import dataset
from opensfm import io


logger = logging.getLogger(__name__)


def run_dataset(data, proj, transformation, image_positions,
                reconstruction, dense, output):
    """ Export reconstructions in geographic coordinates

    Args:
        proj: PROJ.4 projection string
        transformation : print cooordinate transformation matrix'
        image_positions : export image positions
        reconstruction : export reconstruction.json
        dense : export dense point cloud (depthmaps/merged.ply)
        output : path of the output file relative to the dataset

    """

    if not (transformation
            or image_positions
            or reconstruction
            or dense):
        logger.info('Nothing to do. At least on of the options: ')
        logger.info(' --transformation, --image-positions, --reconstruction, --dense')

    reference = data.load_reference()

    projection = pyproj.Proj(proj)
    transformation = _get_transformation(reference, projection)

    if transformation:
        output = output or 'geocoords_transformation.txt'
        output_path = os.path.join(data.data_path, output)
        _write_transformation(transformation, output_path)

    if image_positions:
        reconstructions = data.load_reconstruction()
        output = output or 'image_geocoords.tsv'
        output_path = os.path.join(data.data_path, output)
        _transform_image_positions(reconstructions, transformation,
                                        output_path)

    if reconstruction:
        reconstructions = data.load_reconstruction()
        for r in reconstructions:
            _transform_reconstruction(r, transformation)
        output = output or 'reconstruction.geocoords.json'
        data.save_reconstruction(reconstructions, output)

    if dense:
        output = output or 'undistorted/depthmaps/merged.geocoords.ply'
        output_path = os.path.join(data.data_path, output)
        udata = dataset.UndistortedDataSet(data, 'undistorted')
        _transform_dense_point_cloud(udata, transformation, output_path)

def _get_transformation(reference, projection):
    """Get the linear transform from reconstruction coords to geocoords."""
    p = [[1, 0, 0],
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 0]]
    q = [_transform(point, reference, projection) for point in p]

    transformation = np.array([
        [q[0][0] - q[3][0], q[1][0] - q[3][0], q[2][0] - q[3][0], q[3][0]],
        [q[0][1] - q[3][1], q[1][1] - q[3][1], q[2][1] - q[3][1], q[3][1]],
        [q[0][2] - q[3][2], q[1][2] - q[3][2], q[2][2] - q[3][2], q[3][2]],
        [0, 0, 0, 1]
    ])
    return transformation

def _write_transformation(transformation, filename):
    """Write the 4x4 matrix transformation to a text file."""
    with io.open_wt(filename) as fout:
        for row in transformation:
            fout.write(u' '.join(map(str, row)))
            fout.write(u'\n')

def _transform(point, reference, projection):
    """Transform on point from local coords to a proj4 projection."""
    lat, lon, altitude = reference.to_lla(point[0], point[1], point[2])
    easting, northing = projection(lon, lat)
    return [easting, northing, altitude]

def _transform_image_positions(reconstructions, transformation, output):
    A, b = transformation[:3, :3], transformation[:3, 3]

    rows = ['Image\tX\tY\tZ']
    for r in reconstructions:
        for shot in r.shots.values():
            o = shot.pose.get_origin()
            to = np.dot(A, o) + b
            row = [shot.id, to[0], to[1], to[2]]
            rows.append('\t'.join(map(str, row)))

    text = '\n'.join(rows + [''])
    with open(output, 'w') as fout:
        fout.write(text)

def _transform_reconstruction(reconstruction, transformation):
    """Apply a transformation to a reconstruction in-place."""
    A, b = transformation[:3, :3], transformation[:3, 3]
    A1 = np.linalg.inv(A)
    b1 = -np.dot(A1, b)

    for shot in reconstruction.shots.values():
        R = shot.pose.get_rotation_matrix()
        t = shot.pose.translation
        shot.pose.set_rotation_matrix(np.dot(R, A1))
        shot.pose.translation = list(np.dot(R, b1) + t)

    for point in reconstruction.points.values():
        point.coordinates = list(np.dot(A, point.coordinates) + b)

def _transform_dense_point_cloud(udata, transformation, output_path):
    """Apply a transformation to the merged point cloud."""
    A, b = transformation[:3, :3], transformation[:3, 3]
    input_path = os.path.join(udata._depthmap_path(), 'merged.ply')
    with io.open_rt(input_path) as fin:
        with io.open_wt(output_path) as fout:
            for i, line in enumerate(fin):
                if i < 13:
                    fout.write(line)
                else:
                    x, y, z, nx, ny, nz, red, green, blue = line.split()
                    x, y, z = np.dot(A, map(float, [x, y, z])) + b
                    nx, ny, nz = np.dot(A, map(float, [nx, ny, nz]))
                    fout.write(
                        "{} {} {} {} {} {} {} {} {}\n".format(
                            x, y, z, nx, ny, nz, red, green, blue))
