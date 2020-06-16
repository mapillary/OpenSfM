import numpy as np
from opensfm import pymap
from opensfm import pyslam
from opensfm import types
from opensfm import reconstruction
from opensfm import pybundle
from opensfm import pysfm
# from opensfm import dataset
# import slam_utils
# import slam_debug
import logging
# import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)

from collections import defaultdict


class SlamMapper(object):

    def __init__(self, data, config_slam, camera, slam_map, extractor, matcher):
        self.data = data
        self.camera = camera
        self.config = data.config
        self.config_slam = config_slam
        self.reconstruction = slam_map
        self.map = slam_map.map
        self.keyframes = []
        # self.kf_ids = {}
        self.n_keyframes = 0
        self.n_frames = 0
        self.curr_kf = None
        self.last_shot = None
        self.pre_last = None
        self.extractor = extractor
        self.num_tracked_lms = 0
        self.num_tracked_lms_thr = 15
        self.lms_ratio_thr = 0.9
        self.fresh_landmarks = set()
        self.guided_matcher = matcher
        self.curr_kf_id = 0
        self.frame_id_to_kf_id = {}  # converts the frame id to kf id

    def add_keyframe(self, kf):
        """Adds a keyframe to the map graph
        and the covisibility graph
        """
        logger.debug("Adding new keyframe # {}, {}".format(kf.id, kf.unique_id))
        # self.kf_ids[self.n_keyframes] = kf.id
        self.keyframes.append(kf)
        self.n_keyframes += 1
        self.curr_kf = kf
        # helper variables for unique KF id
        self.frame_id_to_kf_id[kf.id] = self.curr_kf_id
        self.curr_kf_id += 1  # If no KF deletion then equal to n_keyframes

    def update_with_last_frame(self, shot: pymap.Shot):
        """Updates the last frame and the related variables in slam mapper
        """
        if self.n_frames > 0:  # we alread have frames
            self.velocity = shot.get_pose().get_world_to_cam().dot(
                self.last_shot.get_pose().get_cam_to_world())
            self.pre_last = self.last_shot
        self.n_frames += 1
        self.last_shot = shot

    def create_init_map(self, rec_init,
                        init_shot: pymap.Shot, curr_shot: pymap.Shot):
        """
        Now, take the init reconstruction
        create the full reconstruction!
        The reason, why we cannot simply switch is that the
        shots with their keypoints, etc. are in "main" map
        """

        # We need the following steps
        # (1) Update the poses
        kf1 = init_shot
        kf2 = curr_shot
        kf1.set_pose(rec_init.shots[kf1.id].pose)
        kf2.set_pose(rec_init.shots[kf2.id].pose)
        # (2) Add observations and landmarks to the reconstruction
        self.add_keyframe(kf1)
        self.add_keyframe(kf2)
        self.update_with_last_frame(kf1)
        self.update_with_last_frame(kf2)
        for lm_init in rec_init.points.values():
            lm = self.map.create_landmark(lm_init.id, lm_init.coordinates)
            lm.set_ref_shot(kf1)
            lm.color = lm_init.color
            f1_id = lm_init.get_observation_id_in_shot(kf1)
            f2_id = lm_init.get_observation_id_in_shot(kf2)
            # connect landmark -> kf
            self.map.add_observation(kf1, lm, f1_id)
            self.map.add_observation(kf2, lm, f2_id)
            pyslam.SlamUtilities.compute_descriptor(lm)
            pyslam.SlamUtilities.compute_normal_and_depth(
                lm, self.extractor.get_scale_levels())
        self.map.set_landmark_unique_id(
            rec_init.map.current_landmark_unique_id())

        median_depth = kf1.compute_median_depth(False)
        min_num_triangulated = 100
        if kf2.compute_num_valid_pts(1) < min_num_triangulated\
                and median_depth < 0:
            logger.info("Something wrong in the initialization")
        else:
            scale = 1.0 / median_depth
            kf2.scale_pose(scale)
            kf2.scale_landmarks(scale)

    def new_keyframe_is_needed(self, shot: pymap.Shot):
        num_keyfrms = len(self.keyframes)
        min_obs_thr = 3 if (3 <= num_keyfrms) else 2
        last_kf: pymap.Shot = self.keyframes[-1]
        
        num_reliable_lms = last_kf.compute_num_valid_pts(min_obs_thr)
        max_num_frms_ = 10  # the fps
        min_num_frms_ = 2
        frm_id_of_last_keyfrm_ = self.curr_kf.unique_id
        # Condition A1: Add keyframes if max_num_frames_ or more have passed
        # since the last keyframe insertion
        cond_a1 = (frm_id_of_last_keyfrm_ + max_num_frms_ <= shot.unique_id)
        # Condition A2: Add keyframe if min_num_frames_ or more has passed
        # and mapping module is in standby state
        cond_a2 = (frm_id_of_last_keyfrm_ + min_num_frms_ <= shot.unique_id)
        # cond_a2 = False
        # Condition A3: Add a key frame if the viewpoint has moved from the
        # previous key frame
        cond_a3 = self.num_tracked_lms < (num_reliable_lms * 0.25)

        # Condition B: (Requirement for adding keyframes)
        # Add a keyframe if 3D points are observed above the threshold and
        # the percentage of 3D points is below a certain percentage
        cond_b = (self.num_tracked_lms_thr <= self.num_tracked_lms) and \
                 (self.num_tracked_lms < num_reliable_lms * self.lms_ratio_thr)

        # Do not add KF if enough points are observed
        if not cond_b:
            return False

        # Do not add if none of A is satisfied
        if not cond_a1 and not cond_a2 and not cond_a3:
            return False
        logger.debug("Adding new key frame {}".format(shot.id))
        return True

    def remove_redundant_kfs(self):
        # TODO: Implement
        pass

    def remove_redundant_lms(self):
        # TODO: Implement in C++
        observed_ratio_th = 0.3
        num_reliable_kfs = 2
        num_obs_thr = 2
        unclear_lms = []
        n_removed = 0
        # think about making a self.fresh_landmarks set!
        for idx, lm_id in enumerate(self.fresh_landmarks):
            lm = self.reconstruction.points[lm_id]
            if lm is None:
                continue
            if (lm.slam_data.get_observed_ratio() < observed_ratio_th):
                self.map.remove_landmark(lm)
                n_removed += 1
            elif (num_reliable_kfs + self.frame_id_to_kf_id[lm.get_ref_shot().id]) < self.curr_kf_id\
                    and lm.number_of_observations() <= num_obs_thr:
                self.map.remove_landmark(lm)
                n_removed += 1
            elif num_reliable_kfs + 1 + self.frame_id_to_kf_id[lm.get_ref_shot().id] < self.curr_kf_id:
                # valid, do nothing
                pass
            else:  # not clear
                unclear_lms.append(lm_id)
        self.fresh_landmarks = set(unclear_lms)

    def insert_new_keyframe(self, shot: pymap.Shot):
        # Create new Keyframe
        self.add_keyframe(shot)
        # Now, we have to add the lms as observations
        lm_idc = shot.get_valid_landmarks_and_indices()
        scale_levels = self.extractor.get_scale_levels()
        # already matched
        matched = {}
        for lm, idx in lm_idc:
            # If observed correctly, check it!
            # Triggers only for replaced landmarks
            if lm.is_observed_in_shot(shot):
                self.fresh_landmarks.add(lm.id)
            else:
                matched[lm.id] = idx
                # add observation it
                self.map.add_observation(shot, lm, idx)
                # compute geometry
                pyslam.SlamUtilities.compute_descriptor(lm)
                pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_levels)
        # Update connection
        shot.slam_data.update_graph_node()

        self.remove_redundant_lms()
        self.create_new_landmarks()
        self.update_new_keyframe(shot)
        if self.n_keyframes % self.config_slam["run_local_ba_every_nth"] == 0:
            self.local_bundle_adjustment(shot)

        if self.n_keyframes % 20 == 0:
            self.save_reconstruction("rec" + str(self.n_keyframes) + ".json")

        self.remove_redundant_kfs()

    def update_new_keyframe(self, shot: pymap.Shot):
        """ update new keyframe
        detect and resolve the duplication of the landmarks observed
        in the current frame
        """      
        # again, check the last 10 frames
        fuse_shots = pyslam.SlamUtilities.\
            get_second_order_covisibility_for_shot(shot, 20, 5)
        fuse_margin = 3
        # Get a set of covisible KFs and try to fuse the visible landmarks in the current
        # shot with the old ones.
        # If a visible landmark in the current frame is not visible in a fuse_shot
        # but can be matched to a feature, there are two cases:
        # - No landmark matched to this particular feature -> add observation
        # - Feature already part of landmark -> fuse the landmarks (replace one by the other)
        pyslam.SlamUtilities.fuse_duplicated_landmarks(
            shot, fuse_shots, self.guided_matcher, fuse_margin, self.map)
        landmarks = shot.get_valid_landmarks()
        scale_factors = self.extractor.get_scale_levels()
        for lm in landmarks:
            pyslam.SlamUtilities.compute_descriptor(lm)
            pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_factors)
        shot.slam_data.update_graph_node()

    def create_new_landmarks(self):
        """Creates a new landmarks with using the newly added KF
        """
        new_kf = self.keyframes[-1]
        kf_pose: pymap.Pose = new_kf.get_pose()
        new_cam_center = kf_pose.get_origin()
        new_R = kf_pose.get_R_world_to_cam()
        new_t = kf_pose.get_t_world_to_cam()
        # TODO: replace "local" keyframes by that
        # cov_kfs = new_kf.get_graph_node().get_top_n_covisibilities(2 * num_covisibilities)
        local_keyframes = self.keyframes[-5:-1]

        min_d, max_d = pyslam.SlamUtilities.compute_min_max_depth(new_kf)
        for old_kf in local_keyframes:
            old_kf_pose = old_kf.pose
            baseline_vec = old_kf_pose.get_origin() - new_cam_center
            baseline_dist = np.linalg.norm(baseline_vec)
            median_depth_in_old = old_kf.compute_median_depth(True)
            # check the baseline
            if baseline_dist >= 0.02 * median_depth_in_old:
                # Compute essential matrix!
                E_old_to_new = pyslam.SlamUtilities.\
                    create_E_21(new_R, new_t,
                                old_kf_pose.get_R_world_to_cam(),
                                old_kf_pose.get_t_world_to_cam())
                matches = self.guided_matcher.\
                    match_for_triangulation_epipolar(
                        new_kf, old_kf, E_old_to_new, min_d, max_d, False, 10)
                self.triangulate_from_two_kfs(new_kf, old_kf, matches)

    def triangulate_from_two_kfs(self, new_kf, old_kf, matches):
        if (len(matches) == 0):
            return
        new_kf_name = new_kf.id
        old_kf_name = old_kf.id

        # create the graph with the new tracks manager
        tracks_graph = pysfm.TracksManager()
        track_id = self.map.current_landmark_unique_id()
        for (f1_id, f2_id) in matches:
            obs1 = new_kf.get_observation(f1_id)
            tracks_graph.add_observation(new_kf_name, str(track_id), obs1)
            obs2 = old_kf.get_observation(f2_id)
            tracks_graph.add_observation(old_kf_name, str(track_id), obs2)
            track_id += 1

        np_before = len(self.reconstruction.points)
        reconstruction.triangulate_shot_features(tracks_graph,
                                                 self.reconstruction,
                                                 new_kf_name,
                                                 self.data.config)
        np_after = len(self.reconstruction.points)
        logger.debug("Successfully triangulated {} new points between {} and {}.".
                     format(np_after - np_before, new_kf_name, old_kf_name))

        kf2 = old_kf
        scale_factors = self.extractor.get_scale_levels()
        tracks = tracks_graph.get_track_ids()
        # Avoids repeated ids
        unique_lms = self.map.current_landmark_unique_id() + len(tracks)
        for track in tracks:
            lm = self.reconstruction.points[track]
            if lm is not None:
                lm.set_ref_shot(kf2)
                pyslam.SlamUtilities.compute_descriptor(lm)
                pyslam.SlamUtilities.compute_normal_and_depth(
                    lm, scale_factors)
                self.fresh_landmarks.add(lm.id)
        self.map.set_landmark_unique_id(unique_lms)

    def save_reconstruction(self, file=""):
        reconstruction.paint_reconstruction(
            self.data, None, self.reconstruction)
        self.data.save_reconstruction([self.reconstruction], filename=file)

    def local_bundle_adjustment(self, shot: pymap.Shot):
        """ TODO: Build optimization problem directly from C++"""
        if self.n_keyframes <= 2:
            return
        ba = pybundle.BundleAdjuster()
        # Find "earliest" KF seen by the current map! 
        # Add new landmarks to optimize
        # local_keyframes = self.c_keyframes[-n_kfs_optimize: -1]

        # (1), find all the kfs that see landmarks in the current frame and let's
        #       call them local keyframes
        # (2) find all the landmarks seen in local keyframes
        # (3) find all the keyframes containing the landmarks but set the ones
        #     not in local keyframes constant 
        # correct local keyframes of the current keyframe
        kf_added = {}
        cam = self.camera
        ba.add_camera(cam.id, cam, cam, True)
        cam_id = cam.id
        local_kfs_idx = pyslam.SlamUtilities.compute_local_keyframes(shot)
        local_kfs = []
        kfs_dict_constant = {}
        # Get the local keyframes
        for kf_id in local_kfs_idx:
            kf = self.map.get_shot(kf_id)
            kf_pose = kf.get_pose()
            local_kfs.append(kf)
            # add them directly to BA problem
            ba.add_shot(kf_id, cam_id, kf_pose.get_R_world_to_cam_min(),
                        kf_pose.get_t_world_to_cam(), kf_id == 0)
            kf_added[kf_id] = True
            kfs_dict_constant[kf_id] = True if kf_id == 0 else False

        # Get the landmarks from the keyframes
        # From the local keyframes, get the landmarks
        # and the non-local keyframes
        lm_kf_added = set()
        lm_added = set()
        lm_ids = set()
        for kf_id in local_kfs_idx:
            kf = self.map.get_shot(kf_id)
            lms = kf.get_valid_landmarks()
            points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(kf)
            # print("points2D: ", points2D)
            for (lm, pt2D) in zip(lms, points2D):
                lm_id = lm.id
                ba.add_point(lm_id, lm.get_global_pos(), False)
                ba.add_point_projection_observation(kf_id, lm_id,
                                                    pt2D[0], pt2D[1], pt2D[2])
                lm_kf_added.add((lm_id, kf_id))
                lm_added.add(lm)
                lm_ids.add(lm_id)

        assert(len(lm_ids) == len(lm_added))
        # Go through the added landmarks and add the keyframes
        # that are not in local keyframes
        # Now, get all the keyframes that are not in local keyframes
        # from the landmarks and fix their poses
        for lm in lm_added:
            kf_idx_list = lm.get_observations()
            for kf, idx in kf_idx_list.items():
                kf_id = kf.id
                lm_id = lm.id
                if (kf_id, lm_id) in lm_kf_added:
                    continue
                lm_kf_added.add((kf_id, lm_id))
                if kf_added.get(kf_id) is None:
                    # add the kf
                    ba.add_shot(kf_id, cam_id,
                                kf.pose.get_R_world_to_cam_min(),
                                kf.pose.get_t_world_to_cam(), True)
                    kf_added[kf_id] = True
                    kfs_dict_constant[kf_id] = True
                pt = kf.get_obs_by_idx(idx)
                ba.add_point_projection_observation(kf_id, lm_id,
                                                    pt[0], pt[1], pt[2])

        config = self.config
        # Assume observations N x 3 (x,y,s)
        ba.add_absolute_up_vector(next(iter(local_kfs_idx)), [0, 0, -1], 1e-3)
        ba.set_point_projection_loss_function(config['loss_function'],
                                              config['loss_function_threshold'])
        ba.set_internal_parameters_prior_sd(
            config['exif_focal_sd'],
            config['principal_point_sd'],
            config['radial_distorsion_k1_sd'],
            config['radial_distorsion_k2_sd'],
            config['radial_distorsion_p1_sd'],
            config['radial_distorsion_p2_sd'],
            config['radial_distorsion_k3_sd'])
        ba.set_num_threads(config['processes'])
        ba.set_max_num_iterations(50)
        ba.set_linear_solver_type("SPARSE_SCHUR")
        ba.run()

        # TODO: check outliers!
        logger.debug("Local BA  {}".format(ba.brief_report()))

        th = 0.006
        # Here, we should update the all landmarks!
        for lm in lm_added:
            # Check for outliers and discard them
            point = ba.get_point(lm.id)
            # print(point.reprojection_errors)
            for error in point.reprojection_errors.values():
                if np.linalg.norm(error) > th:
                    # pts_outside += 1
                    # TODO: REMOVE DEBUG
                    # visualize the wrong landmarks
                    # for shot, feat_id in lm.get_observations().items():
                    #     fig, ax = plt.subplots(1)
                    #     ax.imshow(self.data.load_image(shot.id))
                    #     ax.set_title(shot.id)
                    #     pt1 = shot.get_obs_by_idx(feat_id).reshape((1,3))
                        
                    #     pt_denorm = features.denormalized_image_coordinates(pt1, self.camera.width, self.camera.height)
                    #     ax.scatter(pt_denorm[0, 0], pt_denorm[0, 1],
                    #                c=[[0, 1, 0]])
                    # plt.show()
                    self.reconstruction.remove_point(lm.id)
                    break
            else:
                lm.set_global_pos(ba.get_point(lm.id).p)

        # Update keyframes
        for kf_id, constant in kfs_dict_constant.items():
            if not constant:
                kf = self.map.get_shot(kf_id)
                ba_shot = ba.get_shot(kf_id)
                kf.pose.rotation = ba_shot.r
                kf.pose.translation = ba_shot.t

    def create_reconstruction(self):
        # now we create the reconstruction
        # add only gray points
        
        all_kfs = self.map.get_all_keyframes()
        all_landmarks = self.mapmap.get_all_landmarks()
        # reconstruction = self.reconstruction
        # add all kfs to reconstruction
        rec = types.Reconstruction()
        rec.reference = self.data.load_reference()
        rec.cameras = self.data.load_camera_models() 

        for kf in all_kfs:
            shot1 = types.Shot()
            shot1.id = kf.im_name
            shot1.camera = rec.cameras[self.camera[0]]

            T_cw = kf.get_Tcw()
            pose = types.Pose()
            pose.set_rotation_matrix(T_cw[0:3, 0:3])
            pose.translation = T_cw[0:3, 3]
            shot1.pose = pose
            shot1.metadata = reconstruction.\
                get_image_metadata(self.data, kf.im_name)
            rec.add_shot(shot1)

        for lm in all_landmarks:
            point = types.Point()
            point.id = lm.lm_id
            point.color = [255, 0, 0]
            pos_w = lm.get_pos_in_world()
            point.coordinates = pos_w.tolist()
            rec.add_point(point)

        self.reconstruction = rec
