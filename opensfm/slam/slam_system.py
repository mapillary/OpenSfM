import numpy as np
import slam_config
from opensfm import pyslam
from opensfm import pymap
from opensfm import dataset
from opensfm import reconstruction
from opensfm import types
from slam_initializer import SlamInitializer
from slam_mapper import SlamMapper
from slam_tracker import SlamTracker

import logging
logger = logging.getLogger(__name__)


class SlamSystem(object):

    def __init__(self, args):
        self.data = dataset.DataSet(args.dataset)
        self.config = self.data.config
        self.config_slam = slam_config.default_config()
        self.reconstruction = types.Reconstruction()
        self.map = self.reconstruction.map
        ref = self.data.load_reference()
        self.map.set_reference(ref.lat, ref.lon, ref.alt)
        # Create the camera model
        c = next(iter(self.data.load_camera_models().items()))
        self.camera = self.map.create_camera(c[1])

        self.extractor = pyslam.OrbExtractor(
            self.config_slam['feat_max_number'],
            self.config_slam['feat_scale'],
            self.config_slam['feat_pyr_levels'],
            self.config_slam['feat_fast_ini_th'],
            self.config_slam['feat_fast_min_th']
        )

        corner_pts = np.array([[0, 0],  # left top
                               [self.camera.width, 0],  # right top
                               [0, self.camera.height],  # left bottom
                               [self.camera.width, self.camera.height]])  # right bottom

        corners = self.camera.undistort_many(corner_pts)
        bounds = np.array([np.min((corners[0, 0], corners[2, 0])),
                           np.max((corners[1, 0], corners[3, 0])),
                           np.min((corners[0, 1], corners[2, 1])),
                           np.max((corners[1, 1], corners[3, 1]))])
        inv_cell_w = self.config_slam['grid_n_cols'] / (bounds[1] - bounds[0])
        inv_cell_h = self.config_slam['grid_n_rows'] / (bounds[3] - bounds[2])
        self.grid_params =\
            pyslam.\
            GridParameters(self.config_slam['grid_n_cols'],
                           self.config_slam['grid_n_rows'],
                           bounds[0], bounds[2], bounds[1], bounds[3],
                           inv_cell_w, inv_cell_h)
        self.matcher = pyslam.GuidedMatcher(self.grid_params,
                                            self.config_slam['feat_scale'],
                                            self.config_slam['feat_pyr_levels'])
        self.slam_mapper = SlamMapper(
            self.data, self.config_slam, self.camera, self.reconstruction, self.extractor, self.matcher)
        self.slam_init =\
            SlamInitializer(self.data, self.camera, self.matcher, self.map)
        self.slam_tracker = SlamTracker(self.matcher)
        self.system_initialized = False

    def process_frame(self, im_name, gray_scale_img):
        curr_shot: pymap.Shot = self.reconstruction.create_shot(
            im_name, self.camera.id)
        metadata = reconstruction.get_image_metadata(self.data, im_name)
        self.reconstruction.set_shot_metadata(curr_shot, metadata)
        self.extractor.extract_to_shot(curr_shot, gray_scale_img, np.array([]))
        curr_shot.undistort_and_compute_bearings()
        self.matcher.distribute_undist_keypts_to_grid(curr_shot)

        # Normalize the original detections
        curr_shot.normalize_keypts()
        if not self.system_initialized:
            self.system_initialized = self.init_slam_system(curr_shot)
            return self.system_initialized

        # Tracking
        pose = self.track_frame(curr_shot)
        if pose is not None:
            curr_shot.set_pose(pose)

        # Check if new KF needed
        self.slam_mapper.update_with_last_frame(curr_shot)
        self.slam_mapper.num_tracked_lms = self.slam_tracker.num_tracked_lms
        if self.slam_mapper.new_keyframe_is_needed(curr_shot):
            self.slam_mapper.insert_new_keyframe(curr_shot)
        return pose is not None

    def init_slam_system(self, curr_shot: pymap.Shot):
        """Find the initial depth estimates for the slam map"""
        if not self.system_initialized:
            success, rec_init = self.slam_init.initialize(curr_shot)
            self.system_initialized = success

            if self.system_initialized:
                self.slam_mapper.create_init_map(rec_init,
                                                 self.slam_init.init_shot,
                                                 curr_shot)
                self.slam_mapper.velocity = np.eye(4)
            if self.system_initialized:
                logger.info("Initialized with {} ".format(curr_shot.id))
                return True

    def track_frame(self, curr_shot: pymap.Shot):
        """ Tracks a frame
        """
        data = self.data
        logger.info("Tracking: {}, {}".format(curr_shot.id, curr_shot.id))
        return self.slam_tracker.track(self.slam_mapper, curr_shot,
                                       self.config, self.camera, data)
