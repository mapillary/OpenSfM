#include <slam/slam_utilities.h>
#include <map/landmark.h>
#include <geometry/camera.h>
#include <map/shot.h>
#include <bundle/bundle_adjuster.h>
#include <foundation/types.h>
#include <Eigen/Core>
namespace slam
{
class PySlamUtilities
{
public:
  static MatXd GetUndistortedKeyptsFromShot(const map::Shot &shot)
  {
    return SlamUtilities::ConvertOpenCVKptsToEigen(shot.slam_data_.undist_keypts_);
  }

  static MatXd GetKeyptsFromShot(const map::Shot &shot)
  {
    return SlamUtilities::ConvertOpenCVKptsToEigen(shot.GetKeyPoints());
  }

  static void SetDescriptorFromObservations(map::Landmark &landmark)
  {
    SlamUtilities::SetDescriptorFromObservations(landmark);
  }
  static void SetDescriptorFromObservationsEig(map::Landmark &landmark)
  {
    SlamUtilities::SetDescriptorFromObservationsEig(landmark);
  }

  static void SetNormalAndDepthFromObservations(map::Landmark &landmark, const std::vector<float> &scale_factors)
  {
    SlamUtilities::SetNormalAndDepthFromObservations(landmark, scale_factors);
  }
  
  static MatXf GetValidKeypts(const map::Shot &shot)
  {
    const auto &landmarks = shot.GetLandmarks();
    const auto n_landmarks = landmarks.size();
    const auto& keypts = shot.GetKeyPoints();
    MatXf kpts(n_landmarks, 3);
    // Convert to numpy.
    size_t idx2{0};
    for (size_t i = 0; i < n_landmarks; ++i)
    {
      if (landmarks[i] != nullptr)
      {
        const auto& kpt = keypts[i];
        kpts.row(idx2) = Vec3f(kpt.point[0], kpt.point[1], kpt.scale);
        idx2++;
      }
    }
    return kpts.topRows(idx2);
  }

  // static std::vector<map::Shot*>
  // UpdateLocalKeyframes(const map::Shot &shot)
  // {
  //   return SlamUtilities::update_local_keyframes(shot);
  // }

  // static std::vector<map::Landmark*>
  // UpdateLocalLandmarks(const std::vector<map::Shot *> &local_keyframes)
  // {
  //   return SlamUtilities::update_local_landmarks(local_keyframes);
  // }

  static size_t
  MatchShotToLocalLandmarks(map::Shot &shot, const GuidedMatchingWrapper &matcher)
  {
    return SlamUtilities::MatchShotToLocalMap(shot, matcher.matcher_);
  }

  static std::pair<double, double> ComputeMinMaxDepthInShot(const map::Shot &shot)
  {
    return SlamUtilities::ComputeMinMaxDepthInShot(shot);
  }

  static Mat3d create_E_21(const Mat3d& rot_1w, const Vec3d& trans_1w,
                           const Mat3d& rot_2w, const Vec3d& trans_2w) {
    return SlamUtilities::create_E_21(rot_1w, trans_1w, rot_2w, trans_2w);
  }

  static std::vector<map::Shot*> GetSecondOrderCovisibility(const map::Shot& shot, const size_t first_order_thr, const size_t second_order_thr)
  {
    return SlamUtilities::GetSecondOrderCovisibilityForShot(shot, first_order_thr, second_order_thr);
  }

  static void FuseDuplicatedLandmarks(map::Shot &shot, const std::vector<map::Shot *> &fuse_shots, const slam::GuidedMatchingWrapper &matcher,
                                      const float margin, map::Map &slam_map)
  {
    return SlamUtilities::FuseDuplicatedLandmarks(shot, fuse_shots, matcher.matcher_, margin, slam_map);
  }

  static std::unordered_map<map::ShotId, map::Shot*> ComputeLocalKeyframes(map::Shot& shot)
  {
    return SlamUtilities::ComputeLocalKeyframes(shot);
  }


  // static map::Pose
  // BundleSingleView(const map::Shot& shot)
  // {
  //   BundleAdjuster ba;
  //   //Create the camera
  //   BABrownPerspectiveCamera ba_cam;
  //   const auto& cam = shot.shot_camera_.camera_model_;
  //   constexpr auto cam_name = "cam1";
  //   const map::BrownPerspectiveCamera* const b_cam = dynamic_cast<const map::BrownPerspectiveCamera*>(&cam);
  //   ba_cam.id = cam_name;
  //   ba_cam.SetFocalX(b_cam->fx);
  //   ba_cam.SetFocalY(b_cam->fy);
  //   ba_cam.SetCX(b_cam->cx);
  //   ba_cam.SetCY(b_cam->cy);
  //   ba_cam.SetK1(b_cam->k1);
  //   ba_cam.SetK2(b_cam->k2);
  //   ba_cam.SetP1(b_cam->p1);
  //   ba_cam.SetP2(b_cam->p2);
  //   ba_cam.SetK3(b_cam->k3);
  //   //Priors are the same as the others
  //   ba_cam.focal_x_prior = b_cam->fx;
  //   ba_cam.focal_y_prior = b_cam->fy;
  //   ba_cam.c_x_prior = b_cam->cx;
  //   ba_cam.c_y_prior = b_cam->cy;
  //   ba_cam.k1_prior = b_cam->k1;
  //   ba_cam.k2_prior = b_cam->k2;
  //   ba_cam.p1_prior = b_cam->p1;
  //   ba_cam.p2_prior = b_cam->p2;
  //   ba_cam.k3_prior = b_cam->k3;
  //   ba.AddBrownPerspectiveCamera(ba_cam);
  //   // add the single shot
  //   constexpr auto shot_name = "shot1";
  //   constexpr auto set_cam_const{false};
  //   const auto& shot_pose = shot.GetPose();
  //   ba.AddShot(shot_name, cam_name,
  //              shot_pose.RotationWorldToCameraMin(),
  //              shot_pose.TranslationWorldToCamera(),
  //              set_cam_const);
  //   // add the gps
  //   // TODO: make configurable
  //   const auto& gps_pos = shot.shot_measurements_.gps_position_;
  //   ba.AddPositionPrior(shot_name, gps_pos[0], gps_pos[1], gps_pos[2],
  //                       shot.shot_measurements_.gps_dop_);
                        
  //   size_t n_points{0};
  //   const auto& kpts = shot.GetKeyPoints();
  //   const auto& landmarks = shot.GetLandmarks(); 
  //   const auto n_landmarks = landmarks.size();
  //   constexpr auto set_3D_points_const{true};
  //   for (size_t idx = 0; idx < n_landmarks; ++idx)
  //   {
  //     const auto& lm = landmarks[idx];
  //     if (lm != nullptr)
  //     {
  //       const auto pt_id = std::to_string(n_points);
  //       ba.AddPoint(pt_id, lm->GetGlobalPos(), set_3D_points_const);
  //       const auto& kpt = kpts[idx];
  //       const Vec3d norm_pt_scale = cam.NormalizePointAndScale(kpt.point, kpt.size);
  //       ba.AddPointProjectionObservation(shot_name, pt_id, 
  //                                        norm_pt_scale[0],
  //                                        norm_pt_scale[1],
  //                                        norm_pt_scale[2]);
  //       ++n_points;
  //     }
  //   }
  //   // TODO: BA settings!
  //   ba.SetPointProjectionLossFunction("SoftLOneLoss", 1);
  //   ba.SetInternalParametersPriorSD(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);
  //   ba.SetNumThreads(1);
  //   ba.SetMaxNumIterations(10);
  //   ba.SetLinearSolverType("DENSE_QR");
  //   ba.Run();
  //   const auto& opt_shot = ba.GetShot(shot_name);
  //   map::Pose opt_pose;
  //   opt_pose.SetFromWorldToCamera(opt_shot.GetRotation(), opt_shot.GetTranslation());
  //   return opt_pose;
  // }

  // static auto
  // SetUpBAProblem(const map::Shot& shot)
  // {
  //   BundleAdjuster ba;
  //   const auto& landmarks = shot.GetLandmarks(); 
  //   const auto& kpts = shot.GetKeyPoints();
  //   const auto n_landmarks = landmarks.size();
  //   const auto& shot_pose = shot.GetPose();

  //   const auto cam_name = "cam1";
  //   const auto& cam = shot.shot_camera_.camera_model_;
  //   const map::BrownPerspectiveCamera* const b_cam = dynamic_cast<const map::BrownPerspectiveCamera*>(&cam);

  //   std::cout << "cam: " << cam.height << " w: " << cam.width << std::endl;
  //   std::cout << "b_cam: " << b_cam->height << " w: " << b_cam->width << std::endl;
  //   //add camera
  //   constexpr auto set_cam_const{false};
  //   constexpr auto shot_name = "shot1";
  //   BABrownPerspectiveCamera ba_cam;
  //   ba_cam.id = cam_name;
  //   ba_cam.SetFocalX(b_cam->fx);
  //   ba_cam.SetFocalY(b_cam->fy);
  //   ba_cam.SetCX(b_cam->cx);
  //   ba_cam.SetCY(b_cam->cy);
  //   ba_cam.SetK1(b_cam->k1);
  //   ba_cam.SetK2(b_cam->k2);
  //   ba_cam.SetP1(b_cam->p1);
  //   ba_cam.SetP2(b_cam->p2);
  //   ba_cam.SetK3(b_cam->k3);
  //   //Priors are the same as the others
  //   ba_cam.focal_x_prior = b_cam->fx;
  //   ba_cam.focal_y_prior = b_cam->fy;
  //   ba_cam.c_x_prior = b_cam->cx;
  //   ba_cam.c_y_prior = b_cam->cy;
  //   ba_cam.k1_prior = b_cam->k1;
  //   ba_cam.k2_prior = b_cam->k2;
  //   ba_cam.p1_prior = b_cam->p1;
  //   ba_cam.p2_prior = b_cam->p2;
  //   ba_cam.k3_prior = b_cam->k3;
  //   ba.AddBrownPerspectiveCamera(ba_cam);
  //   // std::cout << "Created camera cam" << std::endl;
  //   //add the shot
  //   ba.AddShot(shot_name, cam_name,
  //              shot_pose.RotationWorldToCameraMin(),
  //              shot_pose.TranslationWorldToCamera(),
  //              set_cam_const);
  //   // std::cout << "Added shot" << std::endl;
  //   //Add points and observations
  //   constexpr auto set_3D_points_const{true};
  //   size_t n_points{0};
  //   for (size_t idx = 0; idx < n_landmarks; ++idx)
  //   {
  //     const auto& lm = landmarks[idx];
  //     if (lm != nullptr)
  //     {
  //       const auto pt_id = std::to_string(n_points);
  //       ba.AddPoint(pt_id, lm->GetGlobalPos(), set_3D_points_const);
  //       const auto& kpt = kpts[idx];
  //       const Vec3d norm_pt_scale = cam.NormalizePointAndScale(kpt.point, kpt.size);
  //       // std::cout << pt_id << ": " << norm_pt_scale.transpose() << std::endl;
  //       ba.AddPointProjectionObservation(shot_name, pt_id, 
  //                                        norm_pt_scale[0],
  //                                        norm_pt_scale[1],
  //                                        norm_pt_scale[2]);
  //       ++n_points;
  //     }
  //   }
  //   // std::cout << "Added " << n_points << " points" << std::endl;

  //   // Additional parameters
  //   constexpr double std_dev{1e-3};
  //   ba.AddAbsoluteUpVector(shot_name, Vec3d(0,0,-1), std_dev);
    
  //   //TODO: Somehow convert the config things
  //   //Use the standard values for now
  //   /*
  //   # Params for bundle adjustment
  //   loss_function: SoftLOneLoss     # Loss function for the ceres problem (see: http://ceres-solver.org/modeling.html#lossfunction)
  //   loss_function_threshold: 1      # Threshold on the squared residuals.  Usually cost is quadratic for smaller residuals and sub-quadratic above.
  //   reprojection_error_sd: 0.004    # The standard deviation of the reprojection error
  //   exif_focal_sd: 0.01             # The standard deviation of the exif focal length in log-scale
  //   principal_point_sd: 0.01        # The standard deviation of the principal point coordinates
  //   radial_distorsion_k1_sd: 0.01   # The standard deviation of the first radial distortion parameter
  //   radial_distorsion_k2_sd: 0.01   # The standard deviation of the second radial distortion parameter
  //   radial_distorsion_k3_sd: 0.01   # The standard deviation of the third radial distortion parameter
  //   radial_distorsion_p1_sd: 0.01   # The standard deviation of the first tangential distortion parameter
  //   radial_distorsion_p2_sd: 0.01   # The standard deviation of the second tangential distortion parameter
  //   */
  //   ba.SetPointProjectionLossFunction("SoftLOneLoss", 1);
  //   ba.SetInternalParametersPriorSD(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01);
  //   ba.SetNumThreads(1);
  //   ba.SetMaxNumIterations(50);
  //   ba.SetLinearSolverType("SPARSE_SCHUR");
  //   // std::cout << "Trying to run BA" << std::endl;
  //   ba.Run();
  //   // std::cout << "Finished BA" << std::endl;
  //   std::cout << "new bundle: " << ba.FullReport() << std::endl;
  //   const auto& opt_shot = ba.GetShot(shot_name);
    
  //   map::Pose opt_pose;
  //   opt_pose.SetFromWorldToCamera(opt_shot.GetRotation(), opt_shot.GetTranslation());

  //   // TODO: Remove and count outliers
  //   size_t n_outliers{0};
  //   constexpr double outlier_th{0.006};
  //   const Mat3d opt_Rcw = opt_pose.RotationWorldToCamera();
  //   const Vec3d opt_tcw = opt_pose.TranslationWorldToCamera();
  //   //check outliers
  //   for (size_t idx = 0; idx < n_points; ++idx)
  //   {
  //     // std::cout << "getting pt: " << idx << std::endl;
  //     const auto& pt = ba.GetPoint(std::to_string(idx));
  //     // std::cout << "shot_name: " << shot_name << std::endl;
  //     // std::cout << "pt: " << pt.id << std::endl;
  //     // std::cout << "err: " << pt.reprojection_errors.size() << std::endl;
  //     // for (const auto& p : pt.reprojection_errors)
  //     // {
  //     //   std::cout << "p: " << p.first << "/" << p.second << std::endl;
  //     // }
  //     const Eigen::VectorXd error = pt.reprojection_errors.at(shot_name);
  //     // std::cout << "got shot_name: " << shot_name << std::endl;
  //     if (error.norm() > outlier_th)
  //     {
  //       ++n_outliers;
  //     }
  //     else
  //     {
  //       Eigen::Vector2d pt2D;
  //       if (!cam.ReprojectToImage(opt_Rcw, opt_tcw,pt.parameters,pt2D))
  //       {
  //         ++n_outliers;
  //       }
  //       else
  //       {
  //         if (!cam.InImage(pt2D))
  //         {
  //           ++n_outliers;
  //         }
  //         // TODO: Keep the valid points
  //       }
  //     }
  //   }
  //   return std::make_pair(opt_pose, n_points - n_outliers);
  // }

  static void TriangulateShotFeatures(const TracksManager& tracks_manager, map::Map& map, map::Shot* shot, const double reproj_threshold, const double min_reproj_angle)
  {
    SlamUtilities::TriangulateShotFeatures(tracks_manager, map, shot, reproj_threshold, min_reproj_angle);
  }

  static void Retriangulate(const TracksManager& tracks_manager, map::Map& map, const double reproj_threshold, const double min_reproj_angle,
                            const bool full_triangulation = true)
  {
    SlamUtilities::Retriangulate(tracks_manager, map, reproj_threshold, min_reproj_angle, full_triangulation);
  }
};
} // namespace slam