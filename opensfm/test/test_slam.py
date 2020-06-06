from opensfm import dataset
import numpy as np
import cv2
import argparse
def test_camera_functions():
    parser = argparse.ArgumentParser()
    parser.add_argument('dataset', help='dataset to process')
    args = parser.parse_args()
    args.dataset = "/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05"
    data = dataset.DataSet(args.dataset)
    cam = next(iter(data.load_camera_models().values()))
    corner_pts = np.array([[0, 0],  # left top
                        [cam.width, 0],  # right top
                        [0, cam.height],  # left bottom
                        [cam.width, cam.height]])  # right bottom
    my_corners = []
    cv_corners = []
    # create opencv stuff
    dist = np.float32(cam.distortion)
    K = np.float32(cam.get_K_in_pixel_coordinates())
    for pt in corner_pts:
        my_corners.append(cam.undistort_image_coordinate(pt))
    cv_corners = cv2.undistortPoints(np.float32(corner_pts), K, dist, P=K)
    assert np.allclose(cv_corners.flatten(), np.array(my_corners).flatten())


test_camera_functions()
