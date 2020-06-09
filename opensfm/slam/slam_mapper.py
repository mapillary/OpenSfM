import numpy as np
import networkx as nx
from opensfm import pymap
from opensfm import pyslam
from opensfm import features
from opensfm import types
from opensfm import reconstruction
from opensfm import pybundle
from opensfm import pysfm
import slam_utils
import slam_debug
import logging
import cv2
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
        self.frame_id_to_kf_id = {} # converts the frame id to kf id

    def add_keyframe(self, kf):
        """Adds a keyframe to the map graph
        and the covisibility graph
        """
        logger.debug("Adding new keyframe # {}, {}".format(kf.id, kf.unique_id))
        self.n_keyframes += 1
        self.keyframes.append(kf)
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
            print("Updating velocity: T_{},W * T_W,{}".format(shot.id, self.last_shot.id))
            # self.velocity = frame.world_pose.compose(self.last_frame.world_pose.inverse())
            self.pre_last = self.last_shot
        self.n_frames += 1
        self.last_shot = shot

    def create_init_map(self, rec_init,
                        init_shot: pymap.Shot, curr_shot: pymap.Shot):
        """The graph contains the KFs/shots and landmarks.
        Edges are connections between keyframes and landmarks and
        basically "observations"
        """
        # Now, take the init reconstruction
        # create the full reconstruction!
        # The reason, why we cannot simply switch is that the 
        # shots with their keypoints, etc. are in "main" map

        # We need the following steps
        # Update the poses
        kf1 = init_shot
        kf2 = curr_shot
        kf1.set_pose(rec_init.shots[kf1.id].pose)
        kf2.set_pose(rec_init.shots[kf2.id].pose)
        # Add the observations
        # Add to data and covisibility
        self.add_keyframe(kf1)
        self.add_keyframe(kf2)
        self.update_with_last_frame(kf1)
        self.update_with_last_frame(kf2)
        for lm_init in rec_init.points.values():
            lm = self.map.create_landmark(lm_init.id, lm_init.coordinates)
            lm.set_ref_shot(kf1)
            f1_id = lm_init.get_observation_id_in_shot(kf1)
            f2_id = lm_init.get_observation_id_in_shot(kf2)
            # connect landmark -> kf
            self.map.add_observation(kf1, lm, f1_id)
            self.map.add_observation(kf2, lm, f2_id)
            pyslam.SlamUtilities.compute_descriptor(lm)
            pyslam.SlamUtilities.compute_normal_and_depth(
                lm, self.extractor.get_scale_levels())
        print("create_init_map: len(local_landmarks): ",
              self.map.number_of_landmarks())
        self.map.set_landmark_unique_id(
            rec_init.map.current_landmark_unique_id())
        print("unique_id: ", self.map.current_landmark_unique_id(),
              ", ", rec_init.map.current_landmark_unique_id())

        # slam_debug.disable_debug = False                
        # slam_debug.reproject_landmarks(np.asarray(points3D), pyslam.SlamUtilities.keypts_from_shot(kf1),
        #                         kf1.pose.get_world_to_cam(), self.data.load_image(kf1.id), self.camera[1], title="init_1", obs_normalized=True, do_show=False)

        # slam_debug.reproject_landmarks(np.asarray(points3D), pyslam.SlamUtilities.keypts_from_shot(kf2),
        #                         T2, self.data.load_image(kf2.name), self.camera[1], title="init_2", obs_normalized=False, do_show=False)
        # slam_debug.reproject_landmarks(np.asarray(points3D), pyslam.SlamUtilities.keypts_from_shot(kf2),
        #                         np.linalg.inv(T2), self.data.load_image(kf2.name), self.camera[1], title="init_inv_2", obs_normalized=False, do_show=False)

        # T3 = np.vstack((np.column_stack([T2[0:3, 0:3], -np.linalg.inv(T2[0:3, 0:3]).dot(T2[0:3,3])]),
        #                 np.array([0, 0, 0, 1])))
        # slam_debug.reproject_landmarks(np.asarray(points3D), pyslam.SlamUtilities.keypts_from_shot(kf2),
        #                         T3, self.data.load_image(kf2.name), self.camera[1], title="init_3", obs_normalized=False, do_show=True)

        # Change that according to cam model
        median_depth = kf1.compute_median_depth(False)
        min_num_triangulated = 100
        # print("curr_kf.world_pose: ", curr_kf.world_pose.get_Rt)
        print("Tcw bef scale: ", kf2.get_pose().get_world_to_cam())
        if kf2.compute_num_valid_pts(1) < min_num_triangulated and median_depth < 0:
            logger.info("Something wrong in the initialization")
        else:
            scale = 1.0 / median_depth
            kf2.scale_pose(scale)
            kf2.scale_landmarks(scale)
            # self.slam_map.scale_map(kf1, kf2, 1.0 / median_depth)
        # curr_frame.world_pose = slam_utils.mat_to_pose(kf2.get_Tcw())
        print("Tcw aft scale: ", kf2.get_pose().get_world_to_cam())
        # curr_frame.world_pose = curr_kf.world_pose
        print("Finally finished scale")

    def new_keyframe_is_needed(self, shot: pymap.Shot):
        num_keyfrms = len(self.keyframes)
        min_obs_thr = 3 if (3 <= num_keyfrms) else 2
        last_kf: pymap.Shot = self.keyframes[-1]
        
        num_reliable_lms = last_kf.compute_num_valid_pts(min_obs_thr)
        max_num_frms_ = 10  # the fps
        min_num_frms_ = 2
        # if frame.frame_id > 15 and frame.frame_id % 3:
        #     return True
        frm_id_of_last_keyfrm_ = self.curr_kf.unique_id
        print("curr_kf: ", self.curr_kf.id, num_keyfrms)
        print("frame.frame_id: ", shot.id, frm_id_of_last_keyfrm_)
        # frame.id
        # ## mapping: Whether is processing
        # #const bool mapper_is_idle = mapper_->get_keyframe_acceptability();
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

        print("self.num_tracked_lms_thr {} self.num_tracked_lms {}\n \
               num_reliable_lms {} * self.lms_ratio_th={}".
              format(self.num_tracked_lms_thr, self.num_tracked_lms,
                     num_reliable_lms, num_reliable_lms * self.lms_ratio_thr))
        # Condition B: (Requirement for adding keyframes)
        # Add a keyframe if 3D points are observed above the threshold and
        # the percentage of 3D points is below a certain percentage
        cond_b = (self.num_tracked_lms_thr <= self.num_tracked_lms) and \
                 (self.num_tracked_lms < num_reliable_lms * self.lms_ratio_thr)

        print("cond_a1: {}, cond_a2: {}, cond_a3: {}, cond_b: {}"
              .format(cond_a1, cond_a2, cond_a3, cond_b))

        # Do not add if B is not satisfied
        if not cond_b:
            print("not cond_b -> no kf")
            return False

        # Do not add if none of A is satisfied
        if not cond_a1 and not cond_a2 and not cond_a3:
            print("not cond_a1 and not cond_a2 and not cond_a3 -> no kf")
            return False
        print("NEW KF", shot.id)
        return True

    def remove_redundant_kfs(self):
        pass

    def remove_redundant_lms(self):
        return
        observed_ratio_th = 0.3
        num_reliable_kfs = 2
        num_obs_thr = 2
        unclear_lms = []
        # think about making a self.fresh_landmarks set!
        for lm in self.fresh_landmarks:
            if (lm.slam_data.get_observed_ratio() < observed_ratio_th):
                self.map.remove_landmark(lm)
            elif (num_reliable_kfs + self.frame_id_to_kf_id[lm.get_ref_shot().id]) < self.curr_kf_id\
                    and lm.number_of_observations() <= num_obs_thr:
                self.map.remove_landmark(lm)
            elif num_reliable_kfs + 1 + lm.kf_id < self.curr_kf_id:
                # valid
                pass
            else:  # not clear
                unclear_lms.append(lm)
        print("Removed {} out of {} redundant landmarks"
              .format((len(self.fresh_landmarks) - len(unclear_lms)),
                      len(self.fresh_landmarks)))
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
                self.fresh_landmarks.add(lm)
                if lm.id in matched:
                    print("Already in there!!", matched[lm.id], " now: ", idx)
            else:
                if lm.id in matched:
                    print("Already in there!!")
                matched[lm.id] = idx
                # add observation it
                self.map.add_observation(shot, lm, idx)
                # compute geometry
                pyslam.SlamUtilities.compute_descriptor(lm)
                pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_levels)
        # TODO: REMOVE double check
        slam_debug.check_shot_for_double_entries(shot)
        # TODO: REMOVE double check

        # Update connection
        shot.slam_data.update_graph_node()

        # self.slam_map_cleaner.update_lms_after_kf_insert(new_kf.ckf)
        self.remove_redundant_lms()

        # self.slam_map_cleaner.remove_redundant_lms(new_kf.kf_id)
        print("create_new_landmarks_before")
        chrono = reconstruction.Chronometer()
        self.create_new_landmarks()
        chrono.lap("create_landmarks")
        print("create_new_landmarks_after")
        self.update_new_keyframe(shot)
        print("aft update_new_keyframe")
        chrono.lap("update_keyframe")
        if self.n_keyframes % self.config_slam["run_local_ba_every_nth"] == 0:
            print("bef local_bundle_adjustment")
            self.local_bundle_adjustment(shot)
        chrono.lap("local_bundle_adjustment")
        slam_debug.avg_timings.addTimes(chrono.laps_dict)

        if self.n_keyframes % 50 == 0:
            chrono.start()
            self.create_reconstruction()
            self.save_reconstruction(shot.id + "aft")
            chrono.lap("create+save rec")
            slam_debug.avg_timings.addTimes(chrono.laps_dict)
        chrono.start()
        # n_kf_removed = self.remove_redundant_kfs()
        # n_kf_removed = self.slam_map_cleaner.remove_redundant_kfs(new_kf.ckf, self.c_keyframes[0].kf_id)
        self.remove_redundant_kfs()
        n_kf_removed = 0
        print("Removed {} keyframes ".format(n_kf_removed))
        if (n_kf_removed > 0):
            print("Finally removed frames")
        chrono.lap("remove_redundant_kfs")
        slam_debug.avg_timings.addTimes(chrono.laps_dict)

    def update_new_keyframe(self, shot: pymap.Shot):
        """ update new keyframe
        detect and resolve the duplication of the landmarks observed in the current frame
        """      
        # again, check the last 10 frames
        fuse_shots = pyslam.SlamUtilities.get_second_order_covisibility_for_shot(shot, 20, 5)
        print("update_new_keyframe fuse")
        for fuse_shot in fuse_shots:
            print("get_second_order_covisibilities_for_kf kf: ", fuse_shot.id)
        # slam_debug.visualize_tracked_lms(self.curr_kf.ckf.get_valid_kpts(), frame, data)
        # im = self.data.load_image(self.curr_kf.ckf.im_name)
        # slam_debug.disable_debug = False
        # slam_debug.draw_obs_in_image_no_norm(self.curr_kf.ckf.get_valid_keypts(), im, title="bef fuse", do_show=False)
        # self.slam_map_cleaner.\
        #     fuse_landmark_duplication(self.curr_kf.ckf, list(fuse_kfs))
        fuse_margin = 3
        pyslam.SlamUtilities.fuse_duplicated_landmarks(
            shot, fuse_shots, self.guided_matcher, fuse_margin, self.map)
        # slam_debug.draw_obs_in_image_no_norm(self.curr_kf.ckf.get_valid_keypts(), im, title="aft fuse", do_show=True)
        # slam_debug.disable_debug = False
        print("update_new_keyframe fuse done")
        # cslam.SlamUtilities.update_new_keyframe(self.curr_kf.ckf)
        # update all the visible landmarks
        landmarks = shot.get_valid_landmarks()
        scale_factors = self.extractor.get_scale_levels()
        print("computing_descriptors")
        for lm in landmarks:
            pyslam.SlamUtilities.compute_descriptor(lm)
            pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_factors)
        # self.curr_kf.ckf.get_graph_node().update_connections()
        print("update_graph_node")
        shot.slam_data.update_graph_node()
        print("update_new_keyframe done")

    def create_new_landmarks(self):
        """Creates a new landmarks with using the newly added KF
        """
        # new_kf = self.c_keyframes[-1]
        new_kf = self.keyframes[-1]
        # new_im = self.data.load_image(new_kf.name)
        kf_pose: pymap.Pose = new_kf.get_pose()
        new_cam_center = kf_pose.get_origin()
        new_Tcw = kf_pose.get_world_to_cam()
        new_R = new_Tcw[0:3, 0:3]
        new_t = new_Tcw[0:3, 3]
        # Again, just take the last 10 frames
        # but not the current one!
        # num_covisibilities = 10
        # TODO: replace "local" keyframes by that
        # cov_kfs = new_kf.get_graph_node().get_top_n_covisibilities(2 * num_covisibilities)
        local_keyframes = self.keyframes[-5:-1]
        print("local_kf: ", len(local_keyframes))

        # TODO! check new_kf pose
        # for (old_kf, old_kf_py) in zip(local_keyframes, py_kfs):
        n_baseline_reject = 0
        chrono = slam_debug.Chronometer()
        min_d, max_d = pyslam.SlamUtilities.compute_min_max_depth(new_kf)
        # min_d *= 0.5
        # max_d *= 2
        for old_kf in local_keyframes:
            old_kf_pose = old_kf.pose
            baseline_vec = old_kf_pose.get_origin() - new_cam_center
            baseline_dist = np.linalg.norm(baseline_vec)
            median_depth_in_old = old_kf.compute_median_depth(True)
            # check the baseline
            if baseline_dist >= 0.02 * median_depth_in_old:
                # Compute essential matrix!
                old_R = old_kf_pose.get_R_world_to_cam()
                old_t = old_kf_pose.get_t_world_to_cam()

                chrono.start()
                E_old_to_new = pyslam.SlamUtilities.\
                    create_E_21(new_R, new_t, old_R, old_t)
                chrono.lap("compute E")
                matches = self.guided_matcher.\
                    match_for_triangulation_epipolar(
                        new_kf, old_kf, E_old_to_new, min_d, max_d, False, 10)
                chrono.lap("match_for_triangulation_line_10")
                self.triangulate_from_two_kfs(new_kf, old_kf, matches)
                chrono.lap("triangulate_from_two_kfs")
                slam_debug.avg_timings.addTimes(chrono.laps_dict)
            else:
                n_baseline_reject += 1

        print("n_baseline_reject: ", n_baseline_reject)

    def triangulate_from_two_kfs(self, new_kf: pymap.Shot, old_kf: pymap.Shot, matches):
        new_kf_name = new_kf.id
        old_kf_name = old_kf.id

        norm_p1 = pyslam.SlamUtilities.keypts_from_shot(new_kf)
        # norm_p1, _, _ = features.\
        #     normalize_features(pts1, None, None,
        #                        self.camera[1].width, self.camera[1].height)

        norm_p2 = pyslam.SlamUtilities.keypts_from_shot(old_kf)
        # norm_p2, _, _ = features.\
        #     normalize_features(pts2, None, None,
        #                        self.camera[1].width, self.camera[1].height)

        # new_pose: pymap.Pose = new_kf.get_pose()
        # old_pose: pymap.Pose = old_kf.get_pose()

        f_processed = defaultdict(int)

        # create the graph with the new tracks manager
        tracks_graph = pysfm.TracksManager()
        track_id = self.map.current_landmark_unique_id()
        # for (track_id, (f1_id, f2_id)) in enumerate(matches):
        for (f1_id, f2_id) in matches:
            f_processed[f1_id] += 1
            if f_processed[f1_id] > 1:
                print("double add!!")
                exit()
            x, y, s = norm_p1[f1_id, 0:3]
            r, g, b = [255, 0, 0]
            obs1 = pysfm.Observation(x, y, s, int(r), int(g), int(b), f1_id)
            tracks_graph.add_observation(new_kf_name, str(track_id), obs1)

            x, y, s = norm_p2[f2_id, 0:3]
            r, g, b = [255, 0, 0]
            obs2 = pysfm.Observation(x, y, s, int(r), int(g), int(b), f2_id)
            tracks_graph.add_observation(old_kf_name, str(track_id), obs2)
            track_id += 1
        # cameras = self.data.load_camera_models()
        # camera = next(iter(cameras.values()))
        # rec_tri = types.Reconstruction()
        # rec_tri.reference = self.data.load_reference()
        # rec_tri.cameras = cameras
        # pose1 = slam_utils.mat_to_pose(new_pose.get_world_to_cam())
        # shot1 = types.Shot()
        # shot1.id = new_kf.id
        # shot1.camera = camera
        # shot1.pose = pose1
        # shot1.metadata = reconstruction.get_image_metadata(
        #     self.data, new_kf.id)
        # rec_tri.add_shot(shot1)

        # pose2 = slam_utils.mat_to_pose(old_pose.get_world_to_cam())
        # shot2 = types.Shot()
        # shot2.id = old_kf.id
        # shot2.camera = camera
        # shot2.pose = pose2
        # shot2.metadata = reconstruction.get_image_metadata(
        #     self.data, old_kf.id)
        # rec_tri.add_shot(shot2)
        # graph_inliers = pysfm.TracksManager()
        # graph_inliers = nx.Graph()
        # TODO: instead of rec_tri, use reconstruction
        np_before = len(self.reconstruction.points)
        reconstruction.triangulate_shot_features(tracks_graph,
                                                 self.reconstruction,
                                                 new_kf_name,
                                                 self.data.config)
        # reconstruction.triangulate_shot_features(tracks_graph, graph_inliers,
        #                                          rec_tri, new_kf_name,
        #                                          self.data.config)
        np_after = len(self.reconstruction.points)
        print("Successfully triangulated {} out of {} points.".
              format(np_after - np_before, np_after))
        # TODO: Remove debug stuff
        # new_pose = new_kf.get_pose()
        # old_pose = old_kf.get_pose()
        # points = self.reconstruction.points
        # points3D = np.zeros((len(points), 3))
        # for idx, pt3D in enumerate(self.reconstruction.points.values()):
        #     points3D[idx, :] = pt3D.coordinates

        # slam_debug.reproject_landmarks(points3D, None,
        #                                new_pose.get_world_to_cam(),
        #                                self.data.load_image(new_kf_name),
        #                                self.camera, do_show=False)
        # slam_debug.reproject_landmarks(points3D, None,
        #                                old_pose.get_world_to_cam(),
        #                                self.data.load_image(old_kf_name),
        #                                self.camera, do_show=True)
        # TODO: Remove debug stuff

        # kf1 = new_kf
        print("names: ", new_kf_name, old_kf_name)
        print("landmark: ", self.map.current_landmark_unique_id())

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
                self.fresh_landmarks.add(lm)
        self.map.set_landmark_unique_id(unique_lms)
        # Add to graph -> or better just create clm
        # for _, gi_lm_id in graph_inliers.edges(new_kf_name):
        #     # TODO: Write something like create_landmark
        #     pos_w = rec_tri.points[gi_lm_id].coordinates
        #     next_id = self.map.next_unique_landmark_id()
        #     lm = self.map.create_landmark(next_id, pos_w)
        #     lm.set_ref_shot(kf2)
        #     e1 = graph_inliers.get_edge_data(new_kf_name, gi_lm_id)
        #     e2 = graph_inliers.get_edge_data(old_kf_name, gi_lm_id)
        #     f1_id = e1['feature_id']
        #     f2_id = e2['feature_id']
        #     self.map.add_observation(kf1, lm, f1_id)
        #     self.map.add_observation(kf2, lm, f2_id)
        #     pyslam.SlamUtilities.compute_descriptor(lm)
        #     pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_factors)
        #     self.fresh_landmarks.add(lm)
    
    def triangulate_from_two_kfs_old(self, new_kf: pymap.Shot, old_kf: pymap.Shot, matches):
        # TODO: try without tracks graph
        frame1 = new_kf.name
        frame2 = old_kf.name
        # create the graph
        tracks_graph = nx.Graph()
        tracks_graph.add_node(str(frame1), bipartite=0)
        tracks_graph.add_node(str(frame2), bipartite=0)
        f_processed = defaultdict(int)
        pts1 = pyslam.SlamUtilities.keypts_from_shot(new_kf)
        p1, _, _ = features.\
            normalize_features(pts1, None, None,
                               self.camera[1].width, self.camera[1].height)
        
        pts2 = pyslam.SlamUtilities.keypts_from_shot(old_kf)
        p2, _, _ = features.\
            normalize_features(pts2, None, None,
                               self.camera[1].width, self.camera[1].height)

        new_pose: pymap.Pose = new_kf.get_pose()
        old_pose: pymap.Pose = old_kf.get_pose()

        for (track_id, (f1_id, f2_id)) in enumerate(matches):
            # this checks whether the current kf was matched
            # to one of the landmarks.
            # if f2 is already in a lm
            f_processed[f1_id] += 1
            if f_processed[f1_id] > 1:
                print("double add!!")
                exit()
            x, y, s = p2[f2_id, 0:3]
            r, g, b = [0, 0, 0] #c2[f2_id, :]
            tracks_graph.add_node(str(track_id), bipartite=1)
            tracks_graph.add_edge(str(frame2),
                                  str(track_id),
                                  feature=(float(x), float(y)),
                                  feature_scale=float(s),
                                  feature_id=int(f2_id),
                                  feature_color=(float(r), float(g), float(b)))

            x, y, s = p1[f1_id, 0:3]
            r, g, b = [0, 0, 0]
            tracks_graph.add_edge(str(frame1),
                                  str(track_id),
                                  feature=(float(x), float(y)),
                                  feature_scale=float(s),
                                  feature_id=int(f1_id),
                                  feature_color=(float(r), float(g), float(b)))
        # chrono.lap("track_graph")
        cameras = self.data.load_camera_models()
        camera = next(iter(cameras.values()))
        rec_tri = types.Reconstruction()
        rec_tri.reference = self.data.load_reference()
        rec_tri.cameras = cameras
        pose1 = slam_utils.mat_to_pose(new_pose.get_world_to_cam())
        shot1 = types.Shot()
        shot1.id = frame1
        shot1.camera = camera
        shot1.pose = pose1
        shot1.metadata = reconstruction.get_image_metadata(self.data, frame1)
        rec_tri.add_shot(shot1)

        pose2 = slam_utils.mat_to_pose(old_pose.get_world_to_cam())
        shot2 = types.Shot()
        shot2.id = frame2
        shot2.camera = camera
        shot2.pose = pose2
        shot2.metadata = reconstruction.get_image_metadata(self.data, frame2)
        rec_tri.add_shot(shot2)

        graph_inliers = nx.Graph()
        # chrono.lap("ba setup")
        np_before = len(rec_tri.points)
        reconstruction.triangulate_shot_features(tracks_graph, graph_inliers,
                                                 rec_tri, frame1,
                                                 self.data.config)
        np_after = len(rec_tri.points)
        print("Successfully triangulated {} out of {} points.".
              format(np_after, np_before))
        points = rec_tri.points
        points3D = np.zeros((len(points), 3))
        for idx, pt3D in enumerate(points.values()):
            points3D[idx, :] = pt3D.coordinates

        slam_debug.reproject_landmarks(points3D, None, slam_utils.mat_to_pose(new_pose.get_world_to_cam()), self.data.load_image(new_kf.name), self.camera[1], do_show=False)
        slam_debug.reproject_landmarks(points3D, None, slam_utils.mat_to_pose(old_pose.get_world_to_cam()), self.data.load_image(old_kf.name), self.camera[1], do_show=True)
        kf1 = new_kf
        kf2 = old_kf
        scale_factors = self.extractor.get_scale_levels()
        # Add to graph -> or better just create clm
        for _, gi_lm_id in graph_inliers.edges(frame1):
            # TODO: Write something like create_landmark
            pos_w = rec_tri.points[gi_lm_id].coordinates
            next_id = self.map.next_unique_landmark_id()
            lm = self.map.create_landmark(next_id, pos_w)
            lm.set_ref_shot(kf2)
            e1 = graph_inliers.get_edge_data(frame1, gi_lm_id)
            e2 = graph_inliers.get_edge_data(frame2, gi_lm_id)
            f1_id = e1['feature_id']
            f2_id = e2['feature_id']
            self.map.add_observation(kf1, lm, f1_id)
            self.map.add_observation(kf2, lm, f2_id)
            pyslam.SlamUtilities.compute_descriptor(lm)
            pyslam.SlamUtilities.compute_normal_and_depth(lm, scale_factors)
            self.fresh_landmarks.add(lm)
    
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
        print("correct local keyframes of the current keyframe")
        kf_added = {}
        cam = self.camera
        reconstruction._add_camera_to_bundle(ba, cam, cam, constant=True)
        cam_id = cam.id
        local_kfs_idx = pyslam.SlamUtilities.compute_local_keyframes(shot)
        local_kfs = []
        kfs_dict_constant = {}
        # Get the local keyframes
        for kf_id in local_kfs_idx:
            # kf = self.c_keyframes[kf_id]
            kf = self.map.get_shot(kf_id)
            kf_pose = kf.get_pose()
            local_kfs.append(kf)
            # add them directly to BA problem
            # T_cw = kf_pose.get_world_to_cam()
            # R_cw = cv2.Rodrigues(T_cw[0:3, 0:3])[0]
            # t_cw = T_cw[0:3, 3]
            # kf_pose.get_R_world_to_cam_min()
            # kf_pose.get_t_world_to_cam()
            ba.add_shot(kf_id, cam_id, kf_pose.get_R_world_to_cam_min(),
                        kf_pose.get_t_world_to_cam(), kf_id == 0)
            kf_added[kf_id] = True
            kfs_dict_constant[kf_id] = True if kf_id == 0 else False
        print("Get the landmarks from the keyframes")

        # Get the landmarks from the keyframes
        # From the local keyframes, get the landmarks
        # and the non-local keyframes
        lm_kf_added = set()
        lm_added = set()
        for kf_id in local_kfs_idx:
            # kf = self.c_keyframes[kf_id]
            kf = self.map.get_shot(kf_id)
            lms = kf.get_valid_landmarks()
            points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(kf)
            # points2D, _, _ = features.\
            #     normalize_features(kpts, None, None,
            #                        cam.width, cam.height)

            for (lm, pt2D) in zip(lms, points2D):
                lm_id = lm.id
                ba.add_point(lm_id, lm.get_global_pos(), False)
                ba.add_point_projection_observation(kf_id, lm_id,
                                                    pt2D[0], pt2D[1], pt2D[2])
                lm_kf_added.add((lm_id, kf_id))
                lm_added.add(lm)

        print("test something")
        # test something
        kf = self.keyframes[-1]
        lms = kf.get_valid_landmarks()
        kf_pose = kf.get_pose()
        points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(kf)
        points3D = np.zeros((len(lms), 3), dtype=np.float)
        for idx, lm in enumerate(lms):
            points3D[idx, :] = lm.get_global_pos()
        slam_debug.disable_debug = False
        slam_debug.reproject_landmarks(points3D, points2D,
                                       kf_pose.get_world_to_cam(),
                                       self.data.load_image(kf.id),
                                       self.camera, title="repro",
                                       do_show=True, obs_normalized=True)
        slam_debug.disable_debug = True
        # End test something
        print("Go through the added landmarks and add the keyframes")

        # Go through the added landmarks and add the keyframes
        # that are not in local keyframes
        # Now, get all the keyframes that are not in local keyframes
        # from the landmarks and fix their poses
        for lm in lm_added:
            # print("Getting observation ", lm.id)
            # print("Has observation ", lm.has_observations(), "#", lm.number_of_observations())
            kf_idx_list = lm.get_observations()
            # print("Processing ", lm.id)
            for kf, idx in kf_idx_list.items():
                # print("Processing ", kf.id, " with ", lm.id)

                kf_id = kf.id
                lm_id = lm.id
                if (kf_id, lm_id) in lm_kf_added:
                    continue
                lm_kf_added.add((kf_id, lm_id))
                if kf_added.get(kf_id) is None:
                    # kf_pose = kf.get_pose().get_world_to_cam()
                    # add the kf
                    T_cw = kf.get_pose().get_world_to_cam()
                    R_cw = cv2.Rodrigues(T_cw[0:3, 0:3])[0]
                    t_cw = T_cw[0:3, 3]
                    kf_pose.get_R_world_to_cam_min()
                    kf_pose.get_t_world_to_cam()
                    ba.add_shot(kf_id, cam_id, R_cw, t_cw, True)
                    kf_added[kf_id] = True
                    kfs_dict_constant[kf_id] = True
                # add reprojections
                # print("Getting obs: ", idx)
                pt = kf.get_obs_by_idx(idx)
                # print("Got obs: ", idx, pt)
                # pt2D, _, _ = features.normalize_features(pt.reshape((1, 3)), None, None, cam.width, cam.height)
                # pt2D = pt2D.reshape((3, 1))
                ba.add_point_projection_observation(kf_id, lm_id, pt[0], pt[1], pt[2])
        print("Set up BA")

        config = self.config
        # Assume observations N x 3 (x,y,s)
        ba.add_absolute_up_vector(next(iter(local_kfs_idx)), [0, 0, -1], 1e-3)
        # ba.add_absolute_up_vector(local_kfs_idx[0], [0, 0, -1], 1e-3)
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
        print("ba.full_report(): ", ba.full_report())

        # Update landmarks
        lms = shot.get_valid_landmarks()
        for lm in lms:
            pos_w = ba.get_point(lm.id).p
            lm.set_global_pos(pos_w)

        # DEBUG
        points3D = np.zeros((len(lms), 3))
        for idx, lm in enumerate(lms):
            point = ba.get_point(lm.id)
            # print("point.reprojection_errors: ", point.reprojection_errors)
            pos_w = point.p
            n_th = 0
            th = 0.006
            for (k, v) in point.reprojection_errors.items():
                print(np.linalg.norm(v))
                if np.linalg.norm(v) > th:
                    # print("remove lm_id: ", lm_idx, " kf_id", k)
                    # remove outlier observations
                    # self.graph.remove_edge(lm_id, k)
                    # k -> kf_id
                    n_th += 1
            points3D[idx, :] = pos_w
        print("Found: ", n_th, " outliers!")
        slam_debug.disable_debug = False
        slam_debug.reproject_landmarks(points3D, None, shot.pose.get_world_to_cam(),
                                       self.data.load_image(
                                           shot.id), self.camera,
                                       do_show=False, title="bef", obs_normalized=True)
        ba_shot = ba.get_shot(shot.id)
        # pose = types.Pose(ba_shot.r, ba_shot.t)
        pose = pymap.Pose()
        pose.set_from_world_to_cam(ba_shot.r, ba_shot.t)
        slam_debug.reproject_landmarks(points3D, None, pose.get_world_to_cam(),
                                       self.data.load_image(
                                           shot.id), self.camera,
                                       do_show=True, title="aft", obs_normalized=True)
        slam_debug.disable_debug = True
        # DEBUG END

        # Update keyframes
        for kf_id, constant in kfs_dict_constant.items():
            if not constant:
                kf = self.map.get_shot(kf_id)
                ba_shot = ba.get_shot(kf_id)
                pose = types.Pose(ba_shot.r, ba_shot.t)
                # new_pose = pymap.Pose()
                # new_pose.set_from_world_to_cam(np.vstack((pose.get_Rt(), np.array([0, 0, 0, 1]))))
                # kf.set_pose(new_pose)
                kf.pose.rotation = ba_shot.r
                kf.pose.translation = ba_shot.t

    def create_reconstruction(self):
        # now we create the reconstruction
        # add only gray points
        all_kfs = self.slam_map.get_all_keyframes()
        all_landmarks = self.slam_map.get_all_landmarks()
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
