from opensfm import dataset
from opensfm import pymap
from opensfm import pyslam
from opensfm import reconstruction
from opensfm import pysfm
from opensfm import types
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
    margin = 100
    prev_pts = np.array([obs.point for obs in shot2.slam_data.undist_keypts])[:, 0:2]
    # no noise
    matches = matcher.match_shot_to_shot(shot1, shot2, prev_pts, margin)
    assert len(matches) == len(kpts1) and len(matches) == len(kpts2)
    prev_pts = np.array([obs.point for obs in shot1.slam_data.undist_keypts])[:, 0:2]
    matches = matcher.match_shot_to_shot(shot2, shot1, prev_pts, margin)
    assert len(matches) == len(kpts1) and len(matches) == len(kpts2)
    for i in range(10):
        prev_pts = np.array([obs.point for obs in shot2.slam_data.undist_keypts])[:, 0:2]
        # Add noise from [-1 to 1] * margin
        prev_pts += (np.random.rand(prev_pts.shape[0],
                                    prev_pts.shape[1]) - 0.5) * 2 * margin

        matches = matcher.match_shot_to_shot(shot1, shot2, prev_pts, margin)
        assert len(matches) == len(kpts1) and len(matches) == len(kpts2)


def test_init_and_matching():
    # For now choose two KITTI images
    data = dataset.DataSet("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05")
    cam = next(iter(data.load_camera_models().values()))
    rec = pymap.Map()
    camera = rec.create_camera(cam)
    shot1: pymap.Shot = rec.create_shot("000000.png", camera.id)
    shot2: pymap.Shot = rec.create_shot("000003.png", camera.id)
    im1 = cv2.imread("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05/images/000000.png", cv2.IMREAD_GRAYSCALE)
    im2 = cv2.imread("/home/fschenk/software/mapillary_repos/mapillary_sfm_evaluation/sfm_evaluation_workspace/kitti_05/images/000003.png", cv2.IMREAD_GRAYSCALE)

    feat_pyr_levels = 8
    feat_scale = 1.2
    feat_max_number = 4000
    feat_fast_min_th = 7
    feat_fast_ini_th = 20
    extractor = pyslam.\
        OrbExtractor(feat_max_number, feat_scale,
                     feat_pyr_levels, feat_fast_ini_th, feat_fast_min_th)

    extractor.extract_to_shot(shot1, im1, np.array([]))
    extractor.extract_to_shot(shot2, im2, np.array([]))

    # Matching
    grid_n_cols = 64
    grid_n_rows = 48
    corner_pts = np.array([[0, 0],  # left top
                           [camera.width, 0],  # right top
                           [0, camera.height],  # left bottom
                           [camera.width, camera.height]])  # right bottom

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
    # Normalize the original detections
    shot1.normalize_keypts()
    shot2.normalize_keypts()

    # (1) Run standard initialization 
    #   (1-1) Exhaustively matches over all features
    prev_pts = np.array([obs.point for obs in shot2.slam_data.undist_keypts])[:, 0:2]
    margin = 600
    matches = matcher.match_shot_to_shot(shot1, shot2, prev_pts, margin)
    matches = np.array(matches)
    print(matches.shape)
    #   (1-2) Test reconstructabilitys
    f1_points = pyslam.SlamUtilities.keypts_from_shot(shot1)
    f2_points = pyslam.SlamUtilities.keypts_from_shot(shot2)
    five_point_algo_threshold = 0.004
    threshold = 4 * five_point_algo_threshold
    args = []
    # shot1.id
    # shot2.id
    norm_p1 = f1_points[matches[:, 0], 0:2]
    norm_p2 = f2_points[matches[:, 1], 0:2]
    args.append((shot1.id, im2, norm_p1, norm_p2,
                 camera, camera, threshold))
    i1, i2, r = reconstruction._compute_pair_reconstructability(args[0])
    if r == 0:
        return False, None
    print("Reconstruct")
    #   (1-3) Set up tracks graph
    tracks_graph = pysfm.TracksManager()
    for (track_id, (f1_id, f2_id)) in enumerate(matches):
        tracks_graph.add_observation(
            shot1.id, str(track_id), shot1.get_observation(f1_id))
        tracks_graph.add_observation(
            shot2.id, str(track_id), shot2.get_observation(f2_id))
    #   (1-4) bootstrap_reconstruction on two frames
    rec_report = {}
    rec_init, rec_report['bootstrap'] = \
        reconstruction.bootstrap_reconstruction(data, tracks_graph, data.load_camera_models(),
                                                shot1.id, shot2.id, norm_p1, norm_p2)
    success = reconstruction is not None
    if success:
        print("Created init rec from {}<->{} with {} points from {} matches"
                        .format(shot1.id, shot2.id, len(rec_init.points), len(matches)))

    print("bef scale: ", rec_init.shots[shot2.id].pose.translation)
    median_depth = rec_init.shots[shot2.id].compute_median_depth(False)
    scale = 1.0 / median_depth
    rec_init.shots[shot2.id].scale_pose(scale)
    print("aft scale: ", rec_init.shots[shot2.id].pose.translation)

    # (2) Create a new reconstruction for comparison
    rec2 = types.Reconstruction()
    rec2.cameras = data.load_camera_models()
    # camera = rec2.create_camera(cam)
    shot1_c: pymap.Shot = rec2.create_shot("000000.png", camera.id)
    shot2_c: pymap.Shot = rec2.create_shot("000003.png", camera.id)
    extractor.extract_to_shot(shot1_c, im1, np.array([]))
    extractor.extract_to_shot(shot2_c, im2, np.array([]))
    shot1_c.undistort_and_compute_bearings()
    matcher.distribute_undist_keypts_to_grid(shot1_c)
    shot2_c.undistort_and_compute_bearings()
    matcher.distribute_undist_keypts_to_grid(shot2_c)
    # Normalize the original detections
    shot1_c.normalize_keypts()
    shot2_c.normalize_keypts()


    # (2) Initialize the second frame with the pose
    shot2_c.pose = rec_init.shots[shot2_c.id].pose
    # (3) Use the pose and try the epipolar matcher
    shot2_cam_center = shot2_c.pose.get_origin()
    shot2_R = shot2_c.pose.get_R_world_to_cam()
    shot2_t = shot2_c.pose.get_t_world_to_cam()
    #   (3-1) Compare matches to exhaustive matching
    shot1_pose = shot1_c.pose # should be I
    baseline_vec = shot1_pose.get_origin() - shot2_cam_center
    baseline_dist = np.linalg.norm(baseline_vec)
    median_depth_in_old = shot1.compute_median_depth(True)
    #min_d, max_d = pyslam.SlamUtilities.compute_min_max_depth(shot2)
    min_d = -10
    max_d = 10
    # check the baseline
    if baseline_dist >= 0.02 * median_depth_in_old:
        # Compute essential matrix!
        E_old_to_new = pyslam.SlamUtilities.\
            create_E_21(shot2_R, shot2_t,
                        shot1_pose.get_R_world_to_cam(),
                        shot1_pose.get_t_world_to_cam())
        matchesE = matcher.\
            match_for_triangulation_epipolar(
                shot2_c, shot1_c, E_old_to_new, min_d, max_d, True, 10)
        print("matchesE: ", len(matchesE), 10)
        matchesE = matcher.\
            match_for_triangulation_epipolar(
                shot2_c, shot1_c, E_old_to_new, min_d, max_d, True, 5)
        print("matchesE: ", len(matchesE), 5)
        #   (3-2) Compare triangulation to bootstrap_reconstruction
        print("matches: ", len(matches))
        if (len(matchesE) == 0):
            return

        # create the graph with the new tracks manager
        tracks_graph = pysfm.TracksManager()
        track_id = 0
        for (f1_id, f2_id) in matchesE:
            obs1 = shot2_c.get_observation(f1_id)
            tracks_graph.add_observation(shot2_c.id, str(track_id), obs1)
            obs2 = shot1_c.get_observation(f2_id)
            tracks_graph.add_observation(shot1_c.id, str(track_id), obs2)
            track_id += 1
 
        reconstruction.triangulate_shot_features(tracks_graph,
                                                 rec2,
                                                 shot2_c.id,
                                                 data.config)
        
        print("Init pts: ", len(rec_init.points), " pts: ", len(rec2.points))

    print("matchesE: ", len(matchesE))

# test_matching()
test_init_and_matching()
