from opensfm import pyslam
from opensfm import pymap
from opensfm import pybundle
from opensfm import pygeometry
from opensfm import reconstruction
from slam_mapper import SlamMapper

import slam_debug
import slam_utils
import logging
import numpy as np
logger = logging.getLogger(__name__)

class SlamTracker(object):
    def __init__(self, guided_matcher):
        self.guided_matcher = guided_matcher
        self.scale_factors = None
        self.num_tracked_lms = 0

    def track(self, slam_mapper: SlamMapper, curr_shot: pymap.Shot, config, camera,
              data):
        """Tracks the current frame with respect to the reconstruction

        (1) estimates the initial pose with respect to the landmarks seen in the last shot

        (2) then estimates a refined pose with all the local landmarks
        """

        # (1) Estimate initial pose with respect to last shot
        pose_tracking = self.track_motion(slam_mapper, curr_shot,
                                          camera, config, data)
        if pose_tracking is None:
            return None
        curr_shot.set_pose(pose_tracking)
        
        # (2) Compute the refined pose with local landmarks
        pyslam.SlamUtilities.match_shot_to_local_lms(curr_shot, self.guided_matcher)
        lms = curr_shot.get_valid_landmarks()
        points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(curr_shot)
        valid_ids = curr_shot.get_valid_landmarks_indices()
        points3D = np.zeros((len(lms), 3), dtype=np.float)
        for i, lm in enumerate(lms):
            points3D[i, :] = lm.get_global_pos()

        pose, valid_pts = self.\
            bundle_tracking(points3D, points2D,
                            curr_shot, camera, data.config, data)
        # pose = self.\
        #     bundle_tracking(points3D, points2D,
        #                     curr_shot, camera, data.config, data)
        # point_ids = [lm.id for lm in lms]
        # n_outliers = reconstruction.remove_outliers(slam_mapper.reconstruction, config, point_ids)
        n_tracked = 0

        for idx, is_valid in enumerate(valid_pts):
            if not is_valid:
                curr_shot.remove_observation(valid_ids[idx])
            else:
                n_tracked += 1
        assert(curr_shot.compute_num_valid_pts(1) == np.sum(valid_pts)) # TODO: Remove debug stuff
        print("Discarded {} landmarks".format(
            len(valid_pts) - n_tracked))
        self.num_tracked_lms = n_tracked
        logger.info("Tracked {} landmarks in {}!".format(
            n_tracked, curr_shot.id))
        return pose

    def track_motion(self, slam_mapper: SlamMapper, curr_shot: pymap.Shot,
                     camera, config, data):
        """Estimate 6 DOF world pose of frame
        Reproject the landmarks seen in the last frame
        to frame and estimate the relative 6 DOF motion between
        the two by minimizing the reprojection error.
        """
        # TODO: Make an actual update on the closest frames in the map
        # For now, simply take the last 10 keyframes

        margin = 20
        last_shot = slam_mapper.last_shot
        T_init = slam_mapper.velocity.dot(
            last_shot.get_pose().get_world_to_cam())
        if slam_mapper.just_initialized:
            margin = 40  # to make sure we start with many matches            
        curr_shot.pose.set_from_world_to_cam(T_init)
        # Match landmarks seen in last frame to current one
        n_matches = self.guided_matcher.\
            assign_shot_landmarks_to_kpts_new(slam_mapper.last_shot,
                                              curr_shot, margin)
        logger.info("n_matches after matching {}".format(n_matches))
        if n_matches < 10:  # not enough matches found, increase margin
            n_matches = self.guided_matcher.\
                assign_shot_landmarks_to_kpts(
                    slam_mapper.last_shot, curr_shot, 2 * margin)
            if n_matches < 10:
                logger.error("Tracking lost!!, Implement robust matching!")
        lms = curr_shot.get_valid_landmarks()
        points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(curr_shot)
        valid_ids = curr_shot.get_valid_landmarks_indices()

        points3D = np.zeros((len(lms), 3), dtype=np.float)
        for i, lm in enumerate(lms):
            points3D[i, :] = lm.get_global_pos()
        pose_init = pygeometry.Pose()
        pose_init.set_from_world_to_cam(T_init)
        
        pose, valid_pts = self.bundle_tracking(
            points3D, points2D, curr_shot, camera, config, data)
        #TODO: Remove debug
        # point_ids = [str(lm.id) for lm in lms]
        # n_outliers = reconstruction.remove_outliers(slam_mapper.reconstruction, config, point_ids)
        # points_last = []
        # for lm in lms:
        #     # for obs in lm.get_observations():
        #     obs = lm.get_observation_in_shot(slam_mapper.last_shot)
        #     points_last.append(obs[0:2])


        # points2D = []
        # for fid in valid_ids:
        #     points2D.append(curr_shot.get_observation(fid).point)
    
        # # slam_debug.visualize_matches_pts(np.array(points_last), np.array(points2D),
        # #                                  np.column_stack(
        # #                                      (np.arange(0, len(points_last)), np.arange(0, len(points_last)))),
        # #                                  data.load_image(
        # #                                      slam_mapper.last_shot.id),
        # #                                  data.load_image(curr_shot.id))
        # # #TODO: debug
        # slam_debug.reproject_landmarks(points3D, np.array(points2D), T_init, data.load_image(curr_shot.id), camera, obs_normalized=True, do_show=False, title="curr"+curr_shot.id)
        # slam_debug.reproject_landmarks(points3D, np.array(points2D), pose.get_world_to_cam(), data.load_image(curr_shot.id), camera, obs_normalized=True, do_show=False, title="bundle")
        # slam_debug.reproject_landmarks(points3D, np.array(points2D), slam_mapper.last_shot.pose.get_world_to_cam(), data.load_image(slam_mapper.last_shot.id), camera, obs_normalized=True, do_show=True, title="last")
        # # # TODO: 

        # # points1 = []
        # # points2 = []
        # # points3D = []
        # # for lm in self.reconstruction.points.values(): #curr_shot.get_valid_landmarks():
        # #     points1.append(lm.get_observation_in_shot(kf1)[0:2])
        # #     points2.append(lm.get_observation_in_shot(kf2)[0:2])
        # #     points3D.append(lm.coordinates)

        # # slam_debug.reproject_landmarks(np.array(points3D), None, kf2.pose.get_world_to_cam(),
        # #                                self.data.load_image(kf2.id), self.camera)

        # real_id = valid_ids[175]
        # obs = curr_shot.get_observation(real_id)
        # # 3268
        
        # points2D = np.array(points2D)
        # Set up bundle adjustment problem
        # pose, valid_pts = self.bundle_tracking(
            # points3D, points2D, pose_init, camera, config, data)

        # point_ids = [str(lm.id) for lm in lms]
        # n_outliers = reconstruction.remove_outliers(slam_mapper.reconstruction, config, point_ids)
        # print(n_outliers)
        # # Remove outliers
        assert len(valid_ids) == len(valid_pts)
        n_tracked = 0 #len(lms) - n_outliers
        for idx, is_valid in enumerate(valid_pts):
            if not is_valid:
                curr_shot.remove_observation(valid_ids[idx])
            else:
                n_tracked += 1
        print("Discarded {} landmarks in track_motion".format(
            len(valid_pts) - n_tracked))
        curr_shot.increase_observed_of_landmarks()
        assert(curr_shot.compute_num_valid_pts(1) == np.sum(valid_pts)) # TODO: Remove debug stuff
        assert(curr_shot.compute_num_valid_pts(1) == n_tracked) # TODO: Remove debug stuff
        self.num_tracked_lms = n_tracked
        logger.info("Tracked {} with motion!".format(n_tracked))
        if n_tracked < 10:
            logger.error("Tracking lost!!, Start robust tracking")

            # TODO: ROBUST MATCHING
            return None
            # exit()
        # Save LMS to img
        slam_debug.visualize_tracked_lms(
            points2D[valid_pts, :], curr_shot, data, is_normalized=True)
        return pose

    def bundle_tracking(self, points3D, observations, shot, camera,
                        config, data):
        """Estimates the 6 DOF pose with respect to 3D points

        Reprojects 3D points to the image plane and minimizes the
        reprojection error to the corresponding observations to 
        find the relative motion.

        Args:
            points3D: 3D points to reproject
            observations: their 2D correspondences
            init_pose: initial pose depending on the coord. system of points3D
            camera: intrinsic camera parameters
            config, data

        Returns:
            pose: The estimated (relative) 6 DOF pose
        """
        if len(points3D) != len(observations):
            print("len(points3D) != len(observations): ",
                  len(points3D), len(observations))
            return None, None
        # reproject_landmarks(points3D, observations, init_pose, camera, data)
        # match "last frame" to "current frame"
        # last frame could be reference frame
        # somehow match world points/landmarks seen in last frame
        # to feature matches
        fix_cameras = True
        ba = pybundle.BundleAdjuster()
        ba.add_camera(camera.id, camera, camera, fix_cameras)
        # constant motion velocity -> just say id
        shot_id = str(0)
        camera_id = camera.id
        shot_const = False
        ba.add_shot(shot_id, str(camera_id), shot.pose.rotation,
                    shot.pose.translation, shot_const)
        points_3D_constant = True
        # Add points in world coordinates
        for (pt_id, pt_coord) in enumerate(points3D):
            ba.add_point(str(pt_id), pt_coord, points_3D_constant)
            ft = observations[pt_id, :]
            ba.add_point_projection_observation(shot_id, str(pt_id),
                                                ft[0], ft[1], ft[2])
            # pt_r = init_pose.get_R_world_to_cam().dot(pt_coord)+init_pose.get_t_world_to_cam()                                    
            # pt_r = pt_r[0:2]/pt_r[2]
            # print("error: ", pt_r, ft[0:2], np.linalg.norm(pt_r-ft[0:2]))
        
        if config['bundle_use_gps']:
            g = shot.metadata.gps_position.value
            ba.add_position_prior(shot_id, g[0], g[1], g[2],
                                  shot.metadata.gps_accuracy.value)

        # Assume observations N x 3 (x,y,s)
        ba.add_absolute_up_vector(shot_id, [0, 0, -1], 1e-3)
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
        logger.debug(ba.brief_report())
        s = ba.get_shot(shot_id)
        pose = pygeometry.Pose()
        pose.set_from_world_to_cam([s.r[0], s.r[1], s.r[2]],
                                   [s.t[0], s.t[1], s.t[2]])
        valid_pts = self.discard_outliers(ba, len(points3D), pose, camera)
        return pose, valid_pts

    def discard_outliers(self, ba, n_pts, pose, camera):
        """Remove landmarks with large reprojection error
        or if reprojections are out of bounds
        """
        pts_outside = 0
        pts_inside = 0
        pts_outside_new = 0
        th = 0.006**2
        valid_pts = np.zeros(n_pts, dtype=bool)
        w, h = camera.width, camera.height
        for pt_id in range(0, n_pts):
            p = ba.get_point(str(pt_id))
            error = p.reprojection_errors['0']
            error_sqr = error[0]**2 + error[1]**2
            # Discard if reprojection error too large
            if error_sqr > th:
                pts_outside += 1
            else:
                # check if OOB
                camera_point = pose.transform([p.p[0], p.p[1], p.p[2]])
                if camera_point[2] <= 0.0:
                    pts_outside += 1
                    pts_outside_new += 1
                    continue
                point2D = camera.project(camera_point)
                if slam_utils.in_image(point2D, w, h):
                    pts_inside += 1
                    valid_pts[pt_id] = True
                else:
                    pts_outside += 1
                    pts_outside_new += 1

        # print("pts inside {} and outside {}/ {}".
        #       format(pts_inside, pts_outside, pts_outside_new))
        return valid_pts
