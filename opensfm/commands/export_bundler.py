import logging
import os

import numpy as np

from opensfm import pydense
from opensfm import dataset
from opensfm import io

logger = logging.getLogger(__name__)


class Command:
    name = 'export_bundler'
    help = "Export reconstruction to bundler format"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('--list_path', help='path to the list.txt file')
        parser.add_argument('--bundle_path', help='path to the bundle.out file')
        parser.add_argument('--undistorted',
                            action='store_true',
                            help='export the undistorted reconstruction')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        udata = dataset.UndistortedDataSet(data, 'undistorted')

        default_path = os.path.join(data.data_path, 'bundler')
        list_file_path = args.list_path if args.list_path else default_path
        bundle_file_path = args.bundle_path if args.bundle_path else default_path

        if args.undistorted:
            reconstructions = udata.load_undistorted_reconstruction()
            track_manager = udata.load_undistorted_tracks_manager()
            images = reconstructions[0].shots.keys()
        else:
            reconstructions = data.load_reconstruction()
            track_manager = data.load_tracks_manager()
            images = data.images()

        io.export_bundler(images, reconstructions, track_manager,
                          bundle_file_path, list_file_path)
