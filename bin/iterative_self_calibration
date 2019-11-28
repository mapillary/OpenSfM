#!/usr/bin/env python3

"""Auto-calibrate a camera by reconstructing a dataset iteratively."""

import argparse
import os
import subprocess
import shutil
import sys
from os.path import join as pj

from opensfm import io
from opensfm import dataset


python = sys.executable
this_file_path = os.path.abspath(os.path.dirname(__file__))


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__,
    )
    parser.add_argument(
        'dataset',
        help='dataset to calibrate',
    )
    parser.add_argument(
        '--workdir',
        help="directory where to put intermediate results",
    )
    parser.add_argument(
        '--iterations',
        help="number of iterations",
        type=int,
        default=10,
    )
    parser.add_argument(
        '--remove-previous',
        help="remove previous calibration workdir if existing",
        action='store_true'
    )
    parser.add_argument(
        '--opensfm-path',
        help='path to OpenSfM library',
        default=pj(this_file_path, '..'),
    )
    return parser.parse_args()


def main():
    args = parse_args()

    workdir = args.workdir or os.path.join(args.dataset, 'calibration')
    if args.remove_previous:
        remove_previous_workdir(workdir)

    camera_models = None
    prev_path = args.dataset
    for i in range(args.iterations):
        print("Stating iteration {}".format(i))
        current_path = pj(workdir, 'iteration_{}'.format(i))
        init_dataset(prev_path, current_path, camera_models)
        run_dataset(current_path, args.opensfm_path)
        camera_models = get_optimized_cameras(current_path)
        prev_path = current_path
    current_path = pj(workdir, 'result')
    init_dataset(prev_path, current_path, camera_models)


def remove_previous_workdir(path):
    if os.path.isdir(path):
        shutil.rmtree(path)


def init_dataset(src, dst, camera_models):
    """Init a dataset using data from a previous iteration."""
    filenames = [
        'config.yaml',
        'image_list.txt',
        'images',
        'masks',
        'segmentations',
        'features',
        'exif',
    ]

    io.mkdir_p(dst)

    if camera_models:
        data = dataset.DataSet(dst)
        data.save_camera_models_overrides(camera_models)
    else:
        filenames.append('camera_models_overrides.json')

    for filename in filenames:
        src_path = pj(src, filename)
        if os.path.isfile(src_path) or os.path.isdir(src_path):
            dst_path = pj(dst, filename)

            if os.path.islink(dst_path):
                os.unlink(dst_path)

            os.symlink(os.path.relpath(src_path, dst), dst_path)


def run_dataset(path, opensfm_path):
    cmd = pj(opensfm_path, 'bin', 'opensfm')

    subprocess.call([python, cmd, 'extract_metadata', path])
    subprocess.call([python, cmd, 'detect_features', path])
    subprocess.call([python, cmd, 'match_features', path])
    subprocess.call([python, cmd, 'create_tracks', path])
    subprocess.call([python, cmd, 'reconstruct', path])


def get_optimized_cameras(path):
    data = dataset.DataSet(path)
    reconstruction = data.load_reconstruction()[0]
    return reconstruction.cameras


if __name__ == '__main__':
    main()
