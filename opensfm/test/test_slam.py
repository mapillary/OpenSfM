from opensfm import dataset
from opensfm import pymap
from opensfm import pyslam
import numpy as np
import cv2


def test_camera_functions():
    data = dataset.DataSet("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05")
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


def test_matching():
    data = dataset.DataSet("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05")
    cam = next(iter(data.load_camera_models().values()))
    rec = pymap.Map()
    camera = rec.create_camera(cam)
    shot1: pymap.Shot = rec.create_shot("1", camera.id)
    shot2: pymap.Shot = rec.create_shot("2", camera.id)
    im1 = cv2.imread("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05/images/000001.png", cv2.IMREAD_GRAYSCALE)

    feat_pyr_levels = 8
    feat_scale = 1.2
    feat_max_number = 4000
    feat_fast_min_th = 7
    feat_fast_ini_th = 20
    extractor = pyslam.\
        OrbExtractor(feat_max_number, feat_scale,
                     feat_pyr_levels, feat_fast_ini_th, feat_fast_min_th)

    extractor.extract_to_shot(shot1, im1, np.array([]))
    extractor.extract_to_shot(shot2, im1, np.array([]))
    kpts1 = shot1.get_keypoints()
    kpts2 = shot2.get_keypoints()
    assert len(kpts1) == len(kpts2)
    for (kp1, kp2) in zip(kpts1, kpts2):
        assert kp1.id == kp2.id
        assert np.allclose(kp1.color, kp2.color)
        assert np.allclose(kp1.point, kp2.point)
        assert kp1.scale == kp2.scale
        assert kp1.octave == kp2.octave

    # Matching
    grid_n_cols = 64
    grid_n_rows = 48
    corner_pts = np.array([[0, 0],  # left top
                           [camera.width, 0],  # right top
                           [0, camera.height],  # left bottom
                           [camera.width, camera.height]])  # right bottom

    # corners = self.camera.undistort_many(corner_pts).reshape((4, 2))
    corners = camera.undistort_many(corner_pts)
    bounds = np.array([np.min((corners[0, 0], corners[2, 0])),
                       np.max((corners[1, 0], corners[3, 0])),
                       np.min((corners[0, 1], corners[2, 1])),
                       np.max((corners[1, 1], corners[3, 1]))])

    inv_cell_w = grid_n_cols / (bounds[1] - bounds[0])
    inv_cell_h = grid_n_rows / (bounds[3] - bounds[2])
    grid_params =\
        pyslam.\
        GridParameters(grid_n_cols,
                       grid_n_rows,
                       bounds[0], bounds[2], bounds[1], bounds[3],
                       inv_cell_w, inv_cell_h)

    matcher = pyslam.GuidedMatcher(grid_params,
                                   feat_scale,
                                   feat_pyr_levels)

    shot1.undistort_and_compute_bearings()
    matcher.distribute_undist_keypts_to_grid(shot1)
    shot2.undistort_and_compute_bearings()
    matcher.distribute_undist_keypts_to_grid(shot2)
    prev_pts = np.array([obs.point for obs in shot2.slam_data.undist_keypts])[:, 0:2]
    margin = 100
    # Add noise from [-1 to 1] * margin
    prev_pts += (np.random.rand(prev_pts.shape[0],
                                prev_pts.shape[1]) - 0.5) * 2 * margin

    matches = matcher.match_shot_to_shot(shot1, shot2, prev_pts, margin)
    matches = np.array(matches)

    assert len(matches) == len(kpts1)
    assert len(matches) == len(kpts2)
