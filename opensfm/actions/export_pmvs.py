import logging
import os

import cv2
import numpy as np
from opensfm import features
from opensfm import io
from opensfm import tracking
from opensfm.dataset import DataSet, UndistortedDataSet


logger = logging.getLogger(__name__)


def run_dataset(data: DataSet, points, image_list, output, undistorted):
    """Export reconstruction to PLY format

    Args:
        points: export points
        image_list: export only the shots included in this file (path to .txt file)
        output: output pmvs directory
        undistorted: export the undistorted reconstruction

    """

    udata = data.undistorted_dataset()

    base_output_path = output if output else os.path.join(data.data_path, "pmvs")
    io.mkdir_p(base_output_path)
    logger.info(
        "Converting dataset [%s] to PMVS dir [%s]" % (data.data_path, base_output_path)
    )

    if undistorted:
        reconstructions = udata.load_undistorted_reconstruction()
    else:
        reconstructions = data.load_reconstruction()

    # load tracks for vis.dat
    try:
        if undistorted:
            tracks_manager = udata.load_undistorted_tracks_manager()
        else:
            tracks_manager = data.load_tracks_manager()
        image_graph = tracking.as_weighted_graph(tracks_manager)
    except IOError:
        image_graph = None

    export_only = None
    if image_list:
        export_only = {}
        with open(image_list, "r") as f:
            for image in f:
                export_only[image.strip()] = True

    for h, reconstruction in enumerate(reconstructions):
        export(
            reconstruction,
            h,
            image_graph,
            # pyre-fixme[61]: `tracks_manager` may not be initialized here.
            tracks_manager,
            base_output_path,
            data,
            undistorted,
            udata,
            points,
            export_only,
        )


def export(
    reconstruction,
    index,
    image_graph,
    tracks_manager,
    base_output_path,
    data: DataSet,
    undistorted,
    udata: UndistortedDataSet,
    with_points,
    export_only,
):
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
        resized_image = features.resized_image(
            undistorted_image, data.config["feature_process_size"]
        )
        new_image_path = os.path.join(output_path, "visualize", base + ".jpg")
        cv2.imwrite(new_image_path, resized_image)

        # write camera projection matrix to txt/%08d.txt
        resized_h, resized_w = resized_image.shape[:2]
        resized_K = camera.get_K_in_pixel_coordinates(resized_w, resized_h)
        P = resized_K.dot(shot.pose.get_world_to_cam()[:3])

        new_txt = os.path.join(output_path, "txt", base + ".txt")
        with open(new_txt, "wb") as f:
            np.savetxt(f, P, str("%f"), header="CONTOUR")

    fvis.close()

    # txt
    with open(os.path.join(output_path, "pmvs_txt"), "w") as f:
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
