from __future__ import unicode_literals

import logging
import os
import sys
import cv2

import numpy as np

from opensfm import dataset
from opensfm import transformations as tf
from opensfm import io
from opensfm import types
from opensfm import tracking
from opensfm import features
from six import iteritems

logger = logging.getLogger(__name__)


class Command:
    name = 'export_pmvs'
    help = "Export reconstruction to PMVS"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('--points',
                            action='store_true',
                            help='export points')
        parser.add_argument('--image_list',
                            type=str,
                            help='Export only the shots included in this file (path to .txt file)')
        parser.add_argument('--output', help='output pmvs directory')
        parser.add_argument('--undistorted',
                            action='store_true',
                            help='export the undistorted reconstruction')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        udata = dataset.UndistortedDataSet(data, 'undistorted')

        base_output_path = args.output if args.output else os.path.join(data.data_path, 'pmvs')
        io.mkdir_p(base_output_path)
        logger.info("Converting dataset [%s] to PMVS dir [%s]" % (
            data.data_path, base_output_path))

        if args.undistorted:
            reconstructions = udata.load_undistorted_reconstruction()
        else:
            reconstructions = data.load_reconstruction()

        # load tracks for vis.dat
        try:
            if args.undistorted:
                tracks_manager = udata.load_undistorted_tracks_manager()
            else:
                tracks_manager = data.load_tracks_manager()
            image_graph = tracking.as_weighted_graph(tracks_manager)
        except IOError:
            image_graph = None

        export_only = None
        if args.image_list:
            export_only = {}
            with open(args.image_list, 'r') as f:
                for image in f:
                    export_only[image.strip()] = True

        for h, reconstruction in enumerate(reconstructions):
            self.export(reconstruction, h,
                        image_graph, tracks_manager,
                        base_output_path, data,
                        args.undistorted, udata,
                        args.points, export_only)

    def export(self, reconstruction, index,
               image_graph, tracks_manager,
               base_output_path, data,
               undistorted, udata,
               with_points, export_only):
        logger.info("Reconstruction %d" % index)
        output_path = os.path.join(base_output_path, "recon%d" % index)
        io.mkdir_p(output_path)
        io.mkdir_p(os.path.join(output_path, "visualize"))
        io.mkdir_p(os.path.join(output_path, "txt"))
        io.mkdir_p(os.path.join(output_path, "models"))

        shot_index = {image: i for i, image in enumerate(reconstruction.shots)}

        fvis = open(os.path.join(output_path, "vis.dat"), "w")
        fvis.write("VISDATA\n")
        fvis.write("%d\n" % len(shot_index))

        for image, i in shot_index.items():
            shot = reconstruction.shots[image]
            base = "%08d" % i
            logger.info("Image: %s %s" % (image, base))

            # vis.dat for this image
            if image_graph:
                adj_indices = []
                for adj_image in image_graph[image]:
                    weight = image_graph[image][adj_image]["weight"]
                    if weight > 0 and adj_image in shot_index:
                        adj_indices.append(shot_index[adj_image])

                num_covisible = len(adj_indices)
                fvis.write("%d " % i)
                fvis.write("%d " % num_covisible)
                for ai in adj_indices:
                    fvis.write("%d " % ai)
                fvis.write("\n")

            # radially undistort the original image
            camera = shot.camera
            if undistorted:
                undistorted_image = udata.load_undistorted_image(image)
            else:
                original_image = data.load_image(image)[:, :, ::-1]
                original_h, original_w = original_image.shape[:2]
                K = camera.get_K_in_pixel_coordinates(original_w, original_h)
                distortion = np.array([camera.k1, camera.k2, 0, 0])
                undistorted_image = cv2.undistort(original_image, K, distortion)

            # resize and save the undistorted to visualize/%08d.jpg
            resized_image = features.resized_image(undistorted_image, data.config)
            new_image_path = os.path.join(output_path, "visualize", base + ".jpg")
            cv2.imwrite(new_image_path, resized_image)

            # write camera projection matrix to txt/%08d.txt
            resized_h, resized_w = resized_image.shape[:2]
            resized_K = camera.get_K_in_pixel_coordinates(resized_w, resized_h)
            P = resized_K.dot(shot.pose.get_Rt())

            new_txt = os.path.join(output_path, "txt", base + ".txt")
            with open(new_txt, "wb") as f:
                np.savetxt(f, P, str('%f'), header="CONTOUR")

        fvis.close()

        # options.txt
        with open(os.path.join(output_path, "pmvs_options.txt"), "w") as f:
            f.write("level 1\n")
            f.write("csize 2\n")
            f.write("threshold 0.7\n")
            f.write("wsize 7\n")
            f.write("minImageNum 3\n")
            f.write("CPU 8\n")
            f.write("setEdge 0\n")
            f.write("useBound 0\n")
            f.write("useVisData {}\n".format(int(image_graph is not None)))
            f.write("sequence -1\n")
            f.write("timages -1 0 %d\n" % len(shot_index))
            f.write("oimages 0\n")
