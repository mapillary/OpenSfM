from opensfm import pyslam
from opensfm import pymap
from opensfm import pybundle

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
                            curr_shot.get_pose(), camera, data.config, data)

        n_tracked = 0
        for idx, is_valid in enumerate(valid_pts):
            if not is_valid:
                curr_shot.remove_observation(valid_ids[idx])
            else:
                n_tracked += 1
        assert(curr_shot.compute_num_valid_pts(1) == np.sum(valid_pts)) # TODO: Remove debug stuff

        self.num_tracked_lms = n_tracked
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

        curr_shot.pose.set_from_world_to_cam(T_init)
        # Match landmarks seen in last frame to current one
        n_matches = self.guided_matcher.\
            assign_shot_landmarks_to_kpts_new(slam_mapper.last_shot,
                                              curr_shot, margin)
        if n_matches < 10:  # not enough matches found, increase margin
            n_matches = self.guided_matcher.\
                assign_shot_landmarks_to_kpts(slam_mapper.last_shot, curr_shot, margin * 2)
            if n_matches < 10:
                logger.error("Tracking lost!!, Implement robust matching!")
        lms = curr_shot.get_valid_landmarks()
        points2D = pyslam.SlamUtilities.get_valid_kpts_from_shot(curr_shot)
        valid_ids = curr_shot.get_valid_landmarks_indices()

        points3D = np.zeros((len(lms), 3), dtype=np.float)
        for i, lm in enumerate(lms):
            points3D[i, :] = lm.get_global_pos()
        pose_init = pymap.Pose()
        pose_init.set_from_world_to_cam(T_init)

        # #TODO: debug
        # slam_debug.reproject_landmarks(points3D, None, T_init, data.load_image(curr_shot.id), camera)
        # # TODO: 

        # Set up bundle adjustment problem
        pose, valid_pts = self.bundle_tracking(
            points3D, points2D, pose_init, camera, config, data)

        # Remove outliers
        n_tracked = 0
        for idx, is_valid in enumerate(valid_pts):
            if not is_valid:
                curr_shot.remove_observation(valid_ids[idx])
            else:
                n_tracked += 1
        curr_shot.increase_observed_of_landmarks()
        assert(curr_shot.compute_num_valid_pts(1) == np.sum(valid_pts)) # TODO: Remove debug stuff
        assert(curr_shot.compute_num_valid_pts(1) == n_tracked) # TODO: Remove debug stuff
        self.num_tracked_lms = n_tracked
        if np.sum(valid_pts) < 10:
            logger.error("Tracking lost!!, Start robust tracking")

            # TODO: ROBUST MATCHING
            return None
            # exit()
        # Save LMS to img
        slam_debug.visualize_tracked_lms(
            points2D[valid_pts, :], curr_shot, data, is_normalized=True)
        return pose

    def bundle_tracking(self, points3D, observations, init_pose, camera,
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
            return None
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
        camera_const = False
        ba.add_shot(shot_id, str(camera_id), init_pose.rotation,
                    init_pose.translation, camera_const)
        points_3D_constant = True
        # Add points in world coordinates
        for (pt_id, pt_coord) in enumerate(points3D):
            ba.add_point(str(pt_id), pt_coord, points_3D_constant)
            ft = observations[pt_id, :]
            ba.add_point_projection_observation(shot_id, str(pt_id),
                                                ft[0], ft[1], ft[2])
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
        pose = pymap.Pose()
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
        th = 0.006
        valid_pts = np.zeros(n_pts, dtype=bool)
        w, h = camera.width, camera.height
        for pt_id in range(0, n_pts):
            p = ba.get_point(str(pt_id))
            error = p.reprojection_errors['0']
            # Discard if reprojection error too large
            if np.linalg.norm(error) > th:
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
