#include <bundle/bundle_adjuster.h>
#include <bundle/data/rig.h>
#include <bundle/error/absolute_motion_errors.h>
#include <bundle/error/motion_prior_errors.h>
#include <bundle/error/parameters_errors.h>
#include <bundle/error/position_functors.h>
#include <bundle/error/projection_errors.h>
#include <bundle/error/relative_motion_errors.h>
#include <foundation/types.h>

#include <stdexcept>
#include <string>

namespace bundle {
BundleAdjuster::BundleAdjuster() {
  SetPointProjectionLossFunction("CauchyLoss", 1.0);
  SetRelativeMotionLossFunction("CauchyLoss", 1.0);
  unit_translation_shot_ = NULL;
  focal_prior_sd_ = 1;
  c_prior_sd_ = 1;
  k1_sd_ = 1;
  k2_sd_ = 1;
  p1_sd_ = 1;
  p2_sd_ = 1;
  k3_sd_ = 1;
  k4_sd_ = 1;
  compute_covariances_ = false;
  covariance_estimation_valid_ = false;
  compute_reprojection_errors_ = true;
  adjust_absolute_position_std_ = false;
  max_num_iterations_ = 500;
  num_threads_ = 1;
  linear_solver_type_ = "SPARSE_SCHUR";
  covariance_algorithm_type_ = "SPARSE_QR";
}

geometry::Camera BundleAdjuster::GetDefaultCameraSigma(
    const geometry::Camera &camera) const {
  std::unordered_map<int, double> std_dev_map;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::Focal)] =
      focal_prior_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::AspectRatio)] =
      focal_prior_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::Cx)] = c_prior_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::Cy)] = c_prior_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::K1)] = k1_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::K2)] = k2_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::K3)] = k3_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::P1)] = p1_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::P2)] = p2_sd_;
  std_dev_map[static_cast<int>(geometry::Camera::Parameters::Transition)] = 1.0;

  geometry::Camera sigma_camera = camera;
  for (const auto type : sigma_camera.GetParametersTypes()) {
    sigma_camera.SetParameterValue(type, std_dev_map[static_cast<int>(type)]);
  }
  return sigma_camera;
}

geometry::Pose BundleAdjuster::GetDefaultRigPoseSigma() const {
  geometry::Pose sigma;
  sigma.SetFromCameraToWorld(Vec3d::Constant(rig_rotation_sd_).eval(),
                             Vec3d::Constant(rig_translation_sd_).eval());
  return sigma;
}

void BundleAdjuster::AddCamera(const std::string &id,
                               const geometry::Camera &camera,
                               const geometry::Camera &prior, bool constant) {
  auto &camera_data =
      cameras_
          .emplace(std::piecewise_construct, std::forward_as_tuple(id),
                   std::forward_as_tuple(id, camera, prior,
                                         GetDefaultCameraSigma(camera)))
          .first->second;
  if (constant) {
    camera_data.SetParametersToOptimize({});
  }

  // identity bias by default
  auto &bias_data =
      bias_
          .emplace(
              std::piecewise_construct, std::forward_as_tuple(id),
              std::forward_as_tuple(
                  id, geometry::Similarity(Vec3d::Zero(), Vec3d::Zero(), 1.0)))
          .first->second;
  bias_data.SetParametersToOptimize({});
}

void BundleAdjuster::SetCameraBias(const std::string &camera_id,
                                   const geometry::Similarity &bias) {
  auto bias_exists = bias_.find(camera_id);
  if (bias_exists == bias_.end()) {
    throw std::runtime_error("Camera " + camera_id + " doesn't exist.");
  }
  bias_exists->second = Bias(camera_id, bias);
}

void BundleAdjuster::AddShot(const std::string &id, const std::string &camera,
                             const Vec3d &rotation, const Vec3d &translation,
                             bool constant) {
  const auto camera_exists = cameras_.find(camera);

  Shot *shot = nullptr;
  geometry::Pose pose(rotation, translation);
  if (camera_exists == cameras_.end()) {
    throw std::runtime_error("Camera " + camera_exists->first +
                             " doesn't exist.");
  } else {
    shot =
        &shots_
             .emplace(std::piecewise_construct, std::forward_as_tuple(id),
                      std::forward_as_tuple(id, &camera_exists->second, pose))
             .first->second;
  }
  if (constant) {
    shot->GetPose()->SetParametersToOptimize({});
  }
}

void BundleAdjuster::AddRigInstance(
    const std::string &rig_instance_id, const geometry::Pose &rig_instance_pose,
    const std::unordered_map<std::string, std::string> &shot_cameras,
    const std::unordered_map<std::string, std::string> &shot_rig_cameras,
    bool fixed) {
  auto &rig_instance =
      rig_instances_
          .emplace(std::piecewise_construct,
                   std::forward_as_tuple(rig_instance_id),
                   std::forward_as_tuple(rig_instance_id, rig_instance_pose))
          .first->second;
  if (fixed) {
    rig_instance.SetParametersToOptimize({});
  }

  for (const auto &shot_camera : shot_cameras) {
    const auto shot_id = shot_camera.first;
    const auto camera_id = shot_camera.second;
    const auto rig_camera_id = shot_rig_cameras.at(shot_id);

    const auto camera_exists = cameras_.find(camera_id);
    if (camera_exists == cameras_.end()) {
      throw std::runtime_error("Camera " + camera_id + " doesn't exist.");
    }
    const auto rig_camera_exists = rig_cameras_.find(rig_camera_id);
    if (rig_camera_exists == rig_cameras_.end()) {
      throw std::runtime_error("Rig camera " + rig_camera_id +
                               " doesn't exist.");
    }
    rig_shots_.emplace(
        std::piecewise_construct, std::forward_as_tuple(shot_id),
        std::forward_as_tuple(shot_id, &camera_exists->second,
                              &rig_camera_exists->second,
                              &rig_instances_.at(rig_instance_id)));
  }
};

void BundleAdjuster::AddRigCamera(const std::string &rig_camera_id,
                                  const geometry::Pose &pose,
                                  const geometry::Pose &pose_prior,
                                  bool fixed) {
  const auto rig_camera_exists = rig_cameras_.find(rig_camera_id);
  if (rig_camera_exists != rig_cameras_.end()) {
    throw std::runtime_error("Rig model " + rig_camera_id + " already exist.");
  }

  auto &rig_camera =
      rig_cameras_
          .emplace(std::piecewise_construct,
                   std::forward_as_tuple(rig_camera_id),
                   std::forward_as_tuple(rig_camera_id, pose, pose_prior,
                                         GetDefaultRigPoseSigma()))
          .first->second;
  if (fixed) {
    rig_camera.SetParametersToOptimize({});
  }
};

void BundleAdjuster::AddRigPositionPrior(const std::string &instance_id,
                                         const Vec3d &position,
                                         double std_deviation) {
  auto rig_instance_exists = rig_instances_.find(instance_id);
  if (rig_instance_exists == rig_instances_.end()) {
    throw std::runtime_error("Rig instance " + instance_id + " doesn't exist.");
  }

  geometry::Pose prior_pose;
  prior_pose.SetOrigin(position);
  rig_instance_exists->second.SetPrior(prior_pose);

  geometry::Pose sigma_pose;
  sigma_pose.SetOrigin(Vec3d::Constant(std_deviation));
  rig_instance_exists->second.SetSigma(sigma_pose);
}

void BundleAdjuster::SetScaleSharing(const std::string &id, bool share) {
  const auto find = reconstructions_.find(id);
  if (find == reconstructions_.end()) {
    return;
  }
  find->second.shared = share;
}

void BundleAdjuster::AddReconstruction(const std::string &id, bool constant) {
  Reconstruction r;
  r.id = id;
  r.constant = constant;
  r.shared = true;
  reconstructions_[id] = r;
}

void BundleAdjuster::AddReconstructionShot(const std::string &reconstruction_id,
                                           double scale,
                                           const std::string &shot_id) {
  const auto find = reconstructions_.find(reconstruction_id);
  if (find == reconstructions_.end()) {
    return;
  }
  find->second.scales[shot_id] = scale;
}

void BundleAdjuster::AddPoint(const std::string &id, const Vec3d &position,
                              bool constant) {
  Point p;
  p.id = id;
  p.parameters = position;
  p.constant = constant;
  points_[id] = p;
}

void BundleAdjuster::AddPointProjectionObservation(const std::string &shot,
                                                   const std::string &point,
                                                   const Vec2d &observation,
                                                   double std_deviation) {
  const auto find_rig_shot = rig_shots_.find(shot);
  if (find_rig_shot == rig_shots_.end()) {
    PointProjectionObservation o;
    o.shot = &shots_.at(shot);
    o.camera = &cameras_.at(o.shot->GetCamera()->GetID());
    o.point = &points_[point];
    o.coordinates = observation;
    o.std_deviation = std_deviation;
    point_projection_observations_.push_back(o);
  } else {
    PointRigProjectionObservation o;
    o.rig_shot = &find_rig_shot->second;
    o.camera = &cameras_.at(o.rig_shot->GetCamera()->GetID());
    o.point = &points_[point];
    o.coordinates = observation;
    o.std_deviation = std_deviation;
    point_rig_projection_observations_.push_back(o);
  }
}

void BundleAdjuster::AddRotationPrior(const std::string &shot_id, double rx,
                                      double ry, double rz,
                                      double std_deviation) {
  RotationPrior p;
  p.shot = &shots_.at(shot_id);
  p.rotation[0] = rx;
  p.rotation[1] = ry;
  p.rotation[2] = rz;
  p.std_deviation = std_deviation;
  rotation_priors_.push_back(p);
}

void BundleAdjuster::AddTranslationPrior(const std::string &shot_id, double tx,
                                         double ty, double tz,
                                         double std_deviation) {
  TranslationPrior p;
  p.shot = &shots_.at(shot_id);
  p.translation[0] = tx;
  p.translation[1] = ty;
  p.translation[2] = tz;
  p.std_deviation = std_deviation;
  translation_priors_.push_back(p);
}

void BundleAdjuster::AddPositionPrior(const std::string &shot_id, double x,
                                      double y, double z,
                                      double std_deviation) {
  PositionPrior p;
  p.shot = &shots_.at(shot_id);
  p.position[0] = x;
  p.position[1] = y;
  p.position[2] = z;
  p.std_deviation = std_deviation;
  p.bias = &bias_.at(p.shot->GetCamera()->GetID());
  position_priors_.push_back(p);
}

void BundleAdjuster::AddPointPositionPrior(const std::string &point_id,
                                           double x, double y, double z,
                                           double std_deviation) {
  PointPositionPrior p;
  p.point = &points_[point_id];
  p.position[0] = x;
  p.position[1] = y;
  p.position[2] = z;
  p.std_deviation = std_deviation;
  point_position_priors_.push_back(p);
}

void BundleAdjuster::SetOriginShot(const std::string &shot_id) {
  Shot *shot = &shots_.at(shot_id);
  shot->GetPose()->GetValueData().setZero();
  shot->GetPose()->SetParametersToOptimize({});
}

void BundleAdjuster::SetUnitTranslationShot(const std::string &shot_id) {
  unit_translation_shot_ = &shots_.at(shot_id);
}

void BundleAdjuster::AddRelativeMotion(const RelativeMotion &rm) {
  relative_motions_.push_back(rm);
}

void BundleAdjuster::AddRelativeSimilarity(const RelativeSimilarity &rm) {
  relative_similarity_.push_back(rm);
}

void BundleAdjuster::AddRelativeRotation(const RelativeRotation &rr) {
  relative_rotations_.push_back(rr);
}

void BundleAdjuster::AddCommonPosition(const std::string &shot_id1,
                                       const std::string &shot_id2,
                                       double margin, double std_deviation) {
  CommonPosition a;
  a.shot1 = &shots_.at(shot_id1);
  a.shot2 = &shots_.at(shot_id2);
  a.margin = margin;
  a.std_deviation = std_deviation;
  common_positions_.push_back(a);
}

HeatmapInterpolator::HeatmapInterpolator(const std::vector<double> &in_heatmap,
                                         size_t in_width, double in_resolution)
    : heatmap(in_heatmap),
      width(in_width),
      height(heatmap.size() / width),
      grid(heatmap.data(), 0, height, 0, width),
      interpolator(grid),
      resolution(in_resolution) {}

void BundleAdjuster::AddHeatmap(const std::string &heatmap_id,
                                const std::vector<double> &in_heatmap,
                                size_t in_width, double resolution) {
  heatmaps_[heatmap_id] =
      std::make_shared<HeatmapInterpolator>(in_heatmap, in_width, resolution);
}

void BundleAdjuster::AddAbsolutePositionHeatmap(const std::string &shot_id,
                                                const std::string &heatmap_id,
                                                double x_offset,
                                                double y_offset,
                                                double std_deviation) {
  AbsolutePositionHeatmap a;
  a.shot = &shots_.at(shot_id);
  a.heatmap = heatmaps_[heatmap_id];
  a.x_offset = x_offset;
  a.y_offset = y_offset;
  a.std_deviation = std_deviation;
  absolute_positions_heatmaps_.push_back(a);
}

void BundleAdjuster::AddAbsolutePosition(
    const std::string &shot_id, const Vec3d &position, double std_deviation,
    const std::string &std_deviation_group) {
  AbsolutePosition a;
  a.shot = &shots_.at(shot_id);
  a.position = position;
  a.std_deviation = std_deviation;
  a.std_deviation_group = std_deviation_group;
  absolute_positions_.push_back(a);
}

void BundleAdjuster::AddAbsoluteUpVector(const std::string &shot_id,
                                         const Vec3d &up_vector,
                                         double std_deviation) {
  AbsoluteUpVector a;
  a.shot = &shots_.at(shot_id);
  a.up_vector = up_vector;
  a.std_deviation = std_deviation;
  absolute_up_vectors_.push_back(a);
}

void BundleAdjuster::AddAbsolutePan(const std::string &shot_id, double angle,
                                    double std_deviation) {
  AbsoluteAngle a;
  a.shot = &shots_.at(shot_id);
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_pans_.push_back(a);
}

void BundleAdjuster::AddAbsoluteTilt(const std::string &shot_id, double angle,
                                     double std_deviation) {
  AbsoluteAngle a;
  a.shot = &shots_.at(shot_id);
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_tilts_.push_back(a);
}

void BundleAdjuster::AddAbsoluteRoll(const std::string &shot_id, double angle,
                                     double std_deviation) {
  AbsoluteAngle a;
  a.shot = &shots_.at(shot_id);
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_rolls_.push_back(a);
}

void BundleAdjuster::AddPointPositionShot(const std::string &point_id,
                                          const std::string &shot_id,
                                          const std::string &reconstruction_id,
                                          const Vec3d &position,
                                          double std_deviation,
                                          const PositionConstraintType &type) {
  PointPositionShot a;
  a.point_id = point_id;
  a.shot_id = shot_id;
  a.reconstruction_id = reconstruction_id;
  a.position = position;
  a.std_deviation = std_deviation;
  a.type = type;
  point_positions_shot_.push_back(a);
}

void BundleAdjuster::AddPointPositionWorld(const std::string &point_id,
                                           const Vec3d &position,
                                           double std_deviation,
                                           const PositionConstraintType &type) {
  PointPositionWorld a;
  a.point_id = point_id;
  a.position = position;
  a.std_deviation = std_deviation;
  a.type = type;
  point_positions_world_.push_back(a);
}

void BundleAdjuster::SetPointProjectionLossFunction(std::string name,
                                                    double threshold) {
  point_projection_loss_name_ = name;
  point_projection_loss_threshold_ = threshold;
}

void BundleAdjuster::SetRelativeMotionLossFunction(std::string name,
                                                   double threshold) {
  relative_motion_loss_name_ = name;
  relative_motion_loss_threshold_ = threshold;
}

void BundleAdjuster::SetAdjustAbsolutePositionStd(bool adjust) {
  adjust_absolute_position_std_ = adjust;
}

void BundleAdjuster::SetMaxNumIterations(int miter) {
  max_num_iterations_ = miter;
}

void BundleAdjuster::SetNumThreads(int n) { num_threads_ = n; }

void BundleAdjuster::SetUseAnalyticDerivatives(bool use) {
  use_analytic_ = use;
}

void BundleAdjuster::SetLinearSolverType(std::string t) {
  linear_solver_type_ = t;
}

void BundleAdjuster::SetCovarianceAlgorithmType(std::string t) {
  covariance_algorithm_type_ = t;
}

void BundleAdjuster::SetInternalParametersPriorSD(double focal_sd, double c_sd,
                                                  double k1_sd, double k2_sd,
                                                  double p1_sd, double p2_sd,
                                                  double k3_sd, double k4_sd) {
  focal_prior_sd_ = focal_sd;
  c_prior_sd_ = c_sd;
  k1_sd_ = k1_sd;
  k2_sd_ = k2_sd;
  p1_sd_ = p1_sd;
  p2_sd_ = p2_sd;
  k3_sd_ = k3_sd;
  k4_sd_ = k4_sd;
  for (auto &camera : cameras_) {
    camera.second.SetSigma(GetDefaultCameraSigma(camera.second.GetValue()));
  }
}

void BundleAdjuster::SetRigParametersPriorSD(double rig_translation_sd,
                                             double rig_rotation_sd) {
  rig_translation_sd_ = rig_translation_sd;
  rig_rotation_sd_ = rig_rotation_sd;
  for (auto &rig_camera : rig_cameras_) {
    rig_camera.second.SetSigma(GetDefaultRigPoseSigma());
  }
}

void BundleAdjuster::SetComputeCovariances(bool v) { compute_covariances_ = v; }

bool BundleAdjuster::GetCovarianceEstimationValid() const {
  return covariance_estimation_valid_;
}

void BundleAdjuster::SetComputeReprojectionErrors(bool v) {
  compute_reprojection_errors_ = v;
}

ceres::LossFunction *CreateLossFunction(std::string name, double threshold) {
  if (name.compare("TrivialLoss") == 0) {
    return new ceres::TrivialLoss();
  } else if (name.compare("HuberLoss") == 0) {
    return new ceres::HuberLoss(threshold);
  } else if (name.compare("SoftLOneLoss") == 0) {
    return new ceres::SoftLOneLoss(threshold);
  } else if (name.compare("CauchyLoss") == 0) {
    return new ceres::CauchyLoss(threshold);
  } else if (name.compare("ArctanLoss") == 0) {
    return new ceres::ArctanLoss(threshold);
  }
  return NULL;
}

void BundleAdjuster::AddLinearMotion(const std::string &shot0_id,
                                     const std::string &shot1_id,
                                     const std::string &shot2_id, double alpha,
                                     double position_std_deviation,
                                     double orientation_std_deviation) {
  LinearMotion a;
  a.shot0 = &shots_.at(shot0_id);
  a.shot1 = &shots_.at(shot1_id);
  a.shot2 = &shots_.at(shot2_id);
  a.alpha = alpha;
  a.position_std_deviation = position_std_deviation;
  a.orientation_std_deviation = orientation_std_deviation;
  linear_motion_prior_.push_back(a);
}

template <class T>
struct ErrorTraits {
  using Type = ReprojectionError2D;
};
template <>
struct ErrorTraits<geometry::SphericalCamera> {
  using Type = ReprojectionError3D;
};

template <class T, int C>
struct ErrorTraitsAnalytic {
  using Type = ReprojectionError2DAnalytic<C>;
};

template <>
struct ErrorTraitsAnalytic<geometry::SphericalCamera, 1> {
  using Type = ReprojectionError3DAnalytic;
};

struct AddProjectionError {
  template <class T>
  static void Apply(bool use_analytical, const PointProjectionObservation &obs,
                    ceres::LossFunction *loss, ceres::Problem *problem) {
    constexpr static int ErrorSize = ErrorTraits<T>::Type::Size;
    constexpr static int CameraSize = T::Size;
    constexpr static int ShotSize = 6;

    ceres::CostFunction *cost_function = nullptr;
    if (use_analytical) {
      using ErrorType = typename ErrorTraitsAnalytic<T, CameraSize>::Type;
      cost_function = new ErrorType(obs.camera->GetValue().GetProjectionType(),
                                    obs.coordinates, obs.std_deviation);
    } else {
      using ErrorType = typename ErrorTraits<T>::Type;
      cost_function = new ceres::AutoDiffCostFunction<ErrorType, ErrorSize,
                                                      CameraSize, ShotSize, 3>(
          new ErrorType(obs.camera->GetValue().GetProjectionType(),
                        obs.coordinates, obs.std_deviation));
    }
    problem->AddResidualBlock(cost_function, loss,
                              obs.shot->GetCamera()->GetValueData().data(),
                              obs.shot->GetPose()->GetValueData().data(),
                              obs.point->parameters.data());
  }
};

struct AddRigProjectionError {
  template <class T>
  static void Apply(bool use_analytical,
                    const PointRigProjectionObservation &obs,
                    ceres::LossFunction *loss, ceres::Problem *problem) {
    constexpr static int ErrorSize = RigReprojectionError2D::Size;
    constexpr static int CameraSize = T::Size;
    constexpr static int ShotSize = 6;

    ceres::CostFunction *cost_function = nullptr;
    if (use_analytical) {
      using ErrorType = RigReprojectionError2DAnalytic<CameraSize>;
      cost_function = new ErrorType(obs.camera->GetValue().GetProjectionType(),
                                    obs.coordinates, obs.std_deviation);
    } else {
      using ErrorType = RigReprojectionError2D;
      cost_function =
          new ceres::AutoDiffCostFunction<ErrorType, ErrorSize, CameraSize,
                                          ShotSize, ShotSize, 3>(
              new ErrorType(obs.camera->GetValue().GetProjectionType(),
                            obs.coordinates, obs.std_deviation));
    }
    problem->AddResidualBlock(
        cost_function, loss, obs.camera->GetValueData().data(),
        obs.rig_shot->GetRigInstance()->GetValueData().data(),
        obs.rig_shot->GetRigCamera()->GetValueData().data(),
        obs.point->parameters.data());
  }
};

struct ComputeResidualError {
  template <class T>
  static void Apply(bool use_analytical,
                    const PointProjectionObservation &obs) {
    if (use_analytical) {
      constexpr static int CameraSize = T::Size;
      using ErrorType = typename ErrorTraitsAnalytic<T, CameraSize>::Type;
      constexpr static int ErrorSize =
          ErrorTraitsAnalytic<T, CameraSize>::Type::Size;

      VecNd<ErrorSize> residuals;
      ErrorType error(obs.camera->GetValue().GetProjectionType(),
                      obs.coordinates, 1.0);
      const double *params[] = {obs.shot->GetCamera()->GetValueData().data(),
                                obs.shot->GetPose()->GetValueData().data(),
                                obs.point->parameters.data()};
      error.Evaluate(params, residuals.data(), nullptr);
      obs.point->reprojection_errors[obs.shot->GetID()] = residuals;
    } else {
      using ErrorType = typename ErrorTraits<T>::Type;
      constexpr static int ErrorSize = ErrorTraits<T>::Type::Size;

      VecNd<ErrorSize> residuals;
      ErrorType error(obs.camera->GetValue().GetProjectionType(),
                      obs.coordinates, 1.0);
      error(obs.shot->GetCamera()->GetValueData().data(),
            obs.shot->GetPose()->GetValueData().data(),
            obs.point->parameters.data(), residuals.data());
      obs.point->reprojection_errors[obs.shot->GetID()] = residuals;
    }
  }
};

struct ComputeRigResidualError {
  template <class T>
  static void Apply(bool use_analytical,
                    const PointRigProjectionObservation &obs) {
    if (use_analytical) {
      constexpr static int CameraSize = T::Size;
      using ErrorType = RigReprojectionError2DAnalytic<CameraSize>;
      constexpr static int ErrorSize = ErrorType::Size;

      VecNd<ErrorSize> residuals;
      ErrorType error(obs.camera->GetValue().GetProjectionType(),
                      obs.coordinates, 1.0);
      const double *params[] = {
          obs.camera->GetValueData().data(),
          obs.rig_shot->GetRigInstance()->GetValueData().data(),
          obs.rig_shot->GetRigCamera()->GetValueData().data(),
          obs.point->parameters.data()};
      error.Evaluate(params, residuals.data(), nullptr);
      obs.point->reprojection_errors[obs.rig_shot->GetID()] = residuals;
    } else {
      using ErrorType = RigReprojectionError2D;
      constexpr static int ErrorSize = ErrorType::Size;

      VecNd<ErrorSize> residuals;
      ErrorType error(obs.camera->GetValue().GetProjectionType(),
                      obs.coordinates, 1.0);
      error(obs.camera->GetValueData().data(),
            obs.rig_shot->GetRigInstance()->GetValueData().data(),
            obs.rig_shot->GetRigCamera()->GetValueData().data(),
            obs.point->parameters.data(), residuals.data());
      obs.point->reprojection_errors[obs.rig_shot->GetID()] = residuals;
    }
  }
};

struct AddCameraPriorlError {
  template <class T>
  static void Apply(Camera &camera, ceres::Problem *problem) {
    auto *prior_function = new DataPriorError<geometry::Camera>(&camera);

    // Set some logarithmic prior for Focal and Aspect ratio (if any)
    const auto camera_object = camera.GetValue();
    const auto types = camera_object.GetParametersTypes();
    for (int i = 0; i < types.size(); ++i) {
      const auto t = types[i];
      if (t == geometry::Camera::Parameters::Focal ||
          t == geometry::Camera::Parameters::AspectRatio) {
        prior_function->SetScaleType(
            i, DataPriorError<geometry::Camera>::ScaleType::LOGARITHMIC);
      }
    }

    constexpr static int CameraSize = T::Size;
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<DataPriorError<geometry::Camera>,
                                        CameraSize, CameraSize>(prior_function);
    problem->AddResidualBlock(cost_function, nullptr,
                              camera.GetValueData().data());
  }
};

void BundleAdjuster::Run() {
  ceres::Problem problem;

  // Add shots
  for (auto &i : shots_) {
    auto &data = i.second.GetPose()->GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (i.second.GetPose()->GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }
  }

  // Add cameras
  for (auto &i : cameras_) {
    auto &data = i.second.GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (i.second.GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }

    // Add a barrier for constraining transition of dual to stay in [0, 1]
    const auto camera = i.second.GetValue();
    if (camera.GetProjectionType() == geometry::ProjectionType::DUAL) {
      const auto types = camera.GetParametersTypes();
      int index = -1;
      for (int i = 0; i < types.size() && index < 0; ++i) {
        if (types[i] == geometry::Camera::Parameters::Transition) {
          index = i;
        }
      }
      if (index >= 0) {
        ceres::CostFunction *transition_barrier =
            new ceres::AutoDiffCostFunction<ParameterBarrier, 1,
                                            geometry::DualCamera::Size>(
                new ParameterBarrier(0.0, 1.0, index));
        problem.AddResidualBlock(transition_barrier, nullptr, data.data());
      }
    }
  }

  // Add cameras biases
  for (auto &b : bias_) {
    auto &data = b.second.GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (b.second.GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }
  }

  // Add rig models
  for (auto &rc : rig_cameras_) {
    auto &data = rc.second.GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (rc.second.GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }
  }

  // Add rig instances
  for (auto &ri : rig_instances_) {
    auto &data = ri.second.GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (ri.second.GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }
  }

  // Reconstructions
  for (auto &i : reconstructions_) {
    for (auto &s : i.second.scales) {
      if (i.second.constant) {
        problem.AddParameterBlock(&s.second, 1);
        problem.SetParameterBlockConstant(&s.second);
      } else {
        problem.AddParameterBlock(&s.second, 1);
        problem.SetParameterLowerBound(&s.second, 0, 0.0);
        problem.SetParameterUpperBound(&s.second, 0,
                                       std::numeric_limits<double>::max());
      }
    }
  }

  for (auto &i : points_) {
    if (i.second.constant) {
      problem.AddParameterBlock(i.second.parameters.data(), 3);
      problem.SetParameterBlockConstant(i.second.parameters.data());
    }
  }

  // New generic prior errors (only rig instances + rig modelsfor now)
  for (auto &i : rig_instances_) {
    if (!i.second.HasPrior()) {
      continue;
    }
    auto *position_prior = new DataPriorError<geometry::Pose>(&i.second);
    position_prior->SetConstrainedDataIndexes(
        {Pose::Parameter::TX, Pose::Parameter::TY, Pose::Parameter::TZ});
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<DataPriorError<geometry::Pose>, 3,
                                        Pose::Parameter::NUM_PARAMS>(
            position_prior);
    problem.AddResidualBlock(cost_function, nullptr,
                             i.second.GetValueData().data());
  }
  for (auto &rc : rig_cameras_) {
    if (!rc.second.HasPrior()) {
      continue;
    }
    auto *pose_prior = new DataPriorError<geometry::Pose>(&rc.second);
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<DataPriorError<geometry::Pose>,
                                        Pose::Parameter::NUM_PARAMS,
                                        Pose::Parameter::NUM_PARAMS>(
            pose_prior);
    problem.AddResidualBlock(cost_function, nullptr,
                             rc.second.GetValueData().data());
  }

  // Add reprojection error blocks
  ceres::LossFunction *projection_loss =
      point_projection_observations_.empty() &&
              point_rig_projection_observations_.empty()
          ? nullptr
          : CreateLossFunction(point_projection_loss_name_,
                               point_projection_loss_threshold_);
  for (auto &observation : point_projection_observations_) {
    const auto projection_type =
        observation.camera->GetValue().GetProjectionType();
    geometry::Dispatch<AddProjectionError>(
        projection_type, use_analytic_, observation, projection_loss, &problem);
  }
  for (auto &observation : point_rig_projection_observations_) {
    const auto projection_type =
        observation.camera->GetValue().GetProjectionType();
    geometry::Dispatch<AddRigProjectionError>(
        projection_type, use_analytic_, observation, projection_loss, &problem);
  }

  // Add rotation priors
  for (auto &rp : rotation_priors_) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<RotationPriorError, 3, 6>(
            new RotationPriorError(rp.rotation, rp.std_deviation));

    problem.AddResidualBlock(cost_function, nullptr,
                             rp.shot->GetPose()->GetValueData().data());
  }

  // Add translation priors
  for (auto &tp : translation_priors_) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<TranslationPriorError, 3, 6>(
            new TranslationPriorError(tp.translation, tp.std_deviation));

    problem.AddResidualBlock(cost_function, nullptr,
                             tp.shot->GetPose()->GetValueData().data());
  }

  // Add position priors
  for (auto &pp : position_priors_) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<PositionPriorError, 3, 6, 7>(
            new PositionPriorError(pp.position, pp.std_deviation));

    problem.AddResidualBlock(cost_function, nullptr,
                             pp.shot->GetPose()->GetValueData().data(),
                             pp.bias->GetValueData().data());
  }

  // Add point position priors
  for (auto &pp : point_position_priors_) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<PointPositionPriorError, 3, 3>(
            new PointPositionPriorError(pp.position, pp.std_deviation));

    problem.AddResidualBlock(cost_function, nullptr,
                             pp.point->parameters.data());
  }

  // Add internal parameter priors blocks
  for (auto &i : cameras_) {
    const auto projection_type = i.second.GetValue().GetProjectionType();
    geometry::Dispatch<AddCameraPriorlError>(projection_type, i.second,
                                             &problem);
  }

  // Add unit translation block
  if (unit_translation_shot_) {
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<UnitTranslationPriorError, 1, 6>(
            new UnitTranslationPriorError());

    problem.AddResidualBlock(
        cost_function, nullptr,
        unit_translation_shot_->GetPose()->GetValueData().data());
  }

  // Add relative motion errors
  for (auto &rp : relative_motions_) {
    double robust_threshold =
        relative_motion_loss_threshold_ * rp.robust_multiplier;
    ceres::LossFunction *relative_motion_loss =
        CreateLossFunction(relative_motion_loss_name_, robust_threshold);

    auto *cost_function =
        new ceres::AutoDiffCostFunction<RelativeMotionError, 6, 6, 1, 6>(
            new RelativeMotionError(rp.parameters, rp.scale_matrix));
    double *scale =
        reconstructions_[rp.reconstruction_id_i].GetScalePtr(rp.shot_id_i);
    problem.AddResidualBlock(
        cost_function, relative_motion_loss,
        shots_.at(rp.shot_id_i).GetPose()->GetValueData().data(), scale,
        shots_.at(rp.shot_id_j).GetPose()->GetValueData().data());
  }

  // Add relative similarity errors
  for (auto &rp : relative_similarity_) {
    double robust_threshold =
        relative_motion_loss_threshold_ * rp.robust_multiplier;
    ceres::LossFunction *relative_similarity_loss =
        CreateLossFunction(relative_motion_loss_name_, robust_threshold);

    auto *cost_function =
        new ceres::AutoDiffCostFunction<RelativeSimilarityError, 7, 6, 1, 6, 1>(
            new RelativeSimilarityError(rp.parameters, rp.scale,
                                        rp.scale_matrix));
    double *scale_i =
        reconstructions_[rp.reconstruction_id_i].GetScalePtr(rp.shot_id_i);
    double *scale_j =
        reconstructions_[rp.reconstruction_id_j].GetScalePtr(rp.shot_id_j);
    problem.AddResidualBlock(
        cost_function, relative_similarity_loss,
        shots_.at(rp.shot_id_i).GetPose()->GetValueData().data(), scale_i,
        shots_.at(rp.shot_id_j).GetPose()->GetValueData().data(), scale_j);
  }

  // Add relative rotation errors
  ceres::LossFunction *relative_rotation_loss =
      relative_rotations_.empty()
          ? nullptr
          : CreateLossFunction(relative_motion_loss_name_,
                               relative_motion_loss_threshold_);
  for (auto &rr : relative_rotations_) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<RelativeRotationError, 3, 6, 6>(
            new RelativeRotationError(rr.rotation, rr.scale_matrix));

    problem.AddResidualBlock(
        cost_function, relative_rotation_loss,
        shots_.at(rr.shot_id_i).GetPose()->GetValueData().data(),
        shots_.at(rr.shot_id_j).GetPose()->GetValueData().data());
  }

  // Add common position errors
  for (auto &c : common_positions_) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<CommonPositionError, 3, 6, 6>(
            new CommonPositionError(c.margin, c.std_deviation));

    problem.AddResidualBlock(cost_function, nullptr,
                             c.shot1->GetPose()->GetValueData().data(),
                             c.shot2->GetPose()->GetValueData().data());
  }

  // Add absolute position errors
  std::map<std::string, int> std_dev_group_remap;
  for (const auto &a : absolute_positions_) {
    if (std_dev_group_remap.find(a.std_deviation_group) !=
        std_dev_group_remap.end()) {
      continue;
    }
    const int index = std_dev_group_remap.size();
    std_dev_group_remap[a.std_deviation_group] = index;
  }
  std::vector<double> std_deviations(std_dev_group_remap.size());
  for (const auto &a : absolute_positions_) {
    std_deviations[std_dev_group_remap[a.std_deviation_group]] =
        a.std_deviation;
  }

  // Add heatmap cost
  for (const auto &a : absolute_positions_heatmaps_) {
    auto *cost_function = HeatmapdCostFunctor::Create(
        a.heatmap->interpolator, a.x_offset, a.y_offset, a.heatmap->height,
        a.heatmap->width, a.heatmap->resolution, a.std_deviation);
    problem.AddResidualBlock(cost_function, nullptr,
                             a.shot->GetPose()->GetValueData().data());
  }

  for (auto &a : absolute_positions_) {
    ceres::DynamicCostFunction *cost_function = nullptr;

    // camera parametrization
    ShotPositionShotParam pos_func(0);
    cost_function = new ceres::DynamicAutoDiffCostFunction<
        AbsolutePositionError<ShotPositionShotParam>>(
        new AbsolutePositionError<ShotPositionShotParam>(
            pos_func, a.position, 1.0, true, PositionConstraintType::XYZ));

    // world parametrization
    // ShotPositionWorldParam pos_func(0);
    // cost_function = new ceres::AutoDiffCostFunction<
    //     BAAbsolutePositionError<ShotPositionWorldParam>, 3, 6>(
    //     new BAAbsolutePositionError(pos_func, a.position,
    //     a.std_deviation));

    cost_function->AddParameterBlock(6);
    cost_function->AddParameterBlock(1);
    cost_function->SetNumResiduals(3);
    problem.AddResidualBlock(
        cost_function, nullptr, a.shot->GetPose()->GetValueData().data(),
        &std_deviations[std_dev_group_remap[a.std_deviation_group]]);
  }

  // Add regularizer term if we're adjusting for standart deviation, or lock
  // them up.
  if (adjust_absolute_position_std_) {
    for (int i = 0; i < std_deviations.size(); ++i) {
      ceres::CostFunction *std_dev_cost_function =
          new ceres::AutoDiffCostFunction<StdDeviationConstraint, 1, 1>(
              new StdDeviationConstraint());
      problem.AddResidualBlock(std_dev_cost_function, nullptr,
                               &std_deviations[i]);
    }
  } else {
    for (int i = 0; i < std_deviations.size(); ++i) {
      problem.SetParameterBlockConstant(&std_deviations[i]);
    }
  }

  // Add absolute up vector errors
  ceres::LossFunction *up_vector_loss = nullptr;
  for (auto &a : absolute_up_vectors_) {
    if (a.std_deviation > 0) {
      if (up_vector_loss == nullptr) {
        up_vector_loss = new ceres::CauchyLoss(1);
      }
      auto *up_vector_cost_function =
          new ceres::AutoDiffCostFunction<UpVectorError, 3, 6>(
              new UpVectorError(a.up_vector, a.std_deviation));

      problem.AddResidualBlock(up_vector_cost_function, up_vector_loss,
                               a.shot->GetPose()->GetValueData().data());
    }
  }

  // Add absolute pan (compass) errors
  ceres::LossFunction *pan_loss = nullptr;
  for (auto &a : absolute_pans_) {
    if (a.std_deviation > 0) {
      if (pan_loss == nullptr) {
        pan_loss = new ceres::CauchyLoss(1);
      }
      ceres::CostFunction *pan_cost_function =
          new ceres::AutoDiffCostFunction<PanAngleError, 1, 6>(
              new PanAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(pan_cost_function, pan_loss,
                               a.shot->GetPose()->GetValueData().data());
    }
  }

  // Add absolute tilt errors
  ceres::LossFunction *tilt_loss = nullptr;
  for (auto &a : absolute_tilts_) {
    if (a.std_deviation > 0) {
      if (tilt_loss == nullptr) {
        tilt_loss = new ceres::CauchyLoss(1);
      }
      ceres::CostFunction *tilt_cost_function =
          new ceres::AutoDiffCostFunction<TiltAngleError, 1, 6>(
              new TiltAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(tilt_cost_function, tilt_loss,
                               a.shot->GetPose()->GetValueData().data());
    }
  }

  // Add absolute roll errors
  ceres::LossFunction *roll_loss = nullptr;
  for (auto &a : absolute_rolls_) {
    if (a.std_deviation > 0) {
      if (roll_loss == nullptr) {
        roll_loss = new ceres::CauchyLoss(1);
      }
      ceres::CostFunction *roll_cost_function =
          new ceres::AutoDiffCostFunction<RollAngleError, 1, 6>(
              new RollAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(roll_cost_function, roll_loss,
                               a.shot->GetPose()->GetValueData().data());
    }
  }

  // Add linear motion priors
  ceres::LossFunction *linear_motion_prior_loss_ = nullptr;
  for (auto &a : linear_motion_prior_) {
    if (linear_motion_prior_loss_ == nullptr) {
      linear_motion_prior_loss_ = new ceres::CauchyLoss(1);
    }
    auto *cost_function =
        new ceres::AutoDiffCostFunction<LinearMotionError, 6, 6, 6, 6>(
            new LinearMotionError(a.alpha, a.position_std_deviation,
                                  a.orientation_std_deviation));

    problem.AddResidualBlock(cost_function, linear_motion_prior_loss_,
                             a.shot0->GetPose()->GetValueData().data(),
                             a.shot1->GetPose()->GetValueData().data(),
                             a.shot2->GetPose()->GetValueData().data());
  }

  // Add point positions with shot position priors
  for (auto &p : point_positions_shot_) {
    PointPositionScaledShot pos_func(0, 1, 2);
    auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
        AbsolutePositionError<PointPositionScaledShot>>(
        new AbsolutePositionError<PointPositionScaledShot>(
            pos_func, p.position, p.std_deviation, false, p.type));

    cost_function->AddParameterBlock(6);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    problem.AddResidualBlock(
        cost_function, nullptr,
        shots_.at(p.shot_id).GetPose()->GetValueData().data(),
        reconstructions_[p.reconstruction_id].GetScalePtr(p.shot_id),
        points_[p.point_id].parameters.data());
  }

  // Add point positions with world position priors
  for (auto &p : point_positions_world_) {
    PointPositionWorldFunc pos_func(0);
    auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
        AbsolutePositionError<PointPositionWorldFunc>>(
        new AbsolutePositionError<PointPositionWorldFunc>(
            pos_func, p.position, p.std_deviation, false, p.type));

    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);
    problem.AddResidualBlock(cost_function, nullptr,
                             points_[p.point_id].parameters.data());
  }

  // Solve
  ceres::Solver::Options options;
  if (!ceres::StringToLinearSolverType(linear_solver_type_,
                                       &options.linear_solver_type)) {
    throw std::runtime_error("Linear solver type " + linear_solver_type_ +
                             " doesn't exist.");
  }
  options.num_threads = num_threads_;
  options.max_num_iterations = max_num_iterations_;

  ceres::Solve(options, &problem, &last_run_summary_);

  if (compute_covariances_) {
    ComputeCovariances(&problem);
  }
  if (compute_reprojection_errors_) {
    ComputeReprojectionErrors();
  }
}

void BundleAdjuster::ComputeCovariances(ceres::Problem *problem) {
  bool computed = false;

  if (last_run_summary_.termination_type != ceres::FAILURE) {
    ceres::Covariance::Options options;
    if (!ceres::StringToCovarianceAlgorithmType(covariance_algorithm_type_,
                                                &options.algorithm_type)) {
      throw std::runtime_error("Covariance algorithm type " +
                               covariance_algorithm_type_ + " doesn't exist.");
    }
    ceres::Covariance covariance(options);

    std::vector<std::pair<const double *, const double *>> covariance_blocks;
    for (auto &i : shots_) {
      covariance_blocks.push_back(
          std::make_pair(i.second.GetPose()->GetValueData().data(),
                         i.second.GetPose()->GetValueData().data()));
    }

    bool worked = covariance.Compute(covariance_blocks, problem);

    if (worked) {
      for (auto &i : shots_) {
        covariance_estimation_valid_ = true;

        MatXd covariance_matrix(6, 6);
        if (covariance.GetCovarianceBlock(
                i.second.GetPose()->GetValueData().data(),
                i.second.GetPose()->GetValueData().data(),
                covariance_matrix.data())) {
          i.second.GetPose()->SetCovariance(covariance_matrix);
        }
      }
      computed = true;
    }
  }

  // TODO: It might be slow to check everything for NaNs
  //       So maybe we can find a better solution
  if (computed) {
    // Check for NaNs
    for (auto &i : shots_) {
      if (!i.second.GetPose()->HasCovariance() ||
          !i.second.GetPose()->GetCovariance().allFinite()) {
        covariance_estimation_valid_ = false;
        computed = false;
        break;
      }
      // stop after first Nan value
      if (!computed) break;
    }
  }

  // If covariance estimation failed, use a default value
  if (!computed) {
    covariance_estimation_valid_ = false;

    MatXd default_covariance_matrix = MatXd::Zero(6, 6);
    double default_rotation_variance = 1e-5;
    double default_translation_variance = 1e-2;
    default_covariance_matrix.diagonal().segment<3>(0).setConstant(
        default_rotation_variance);
    default_covariance_matrix.diagonal().segment<3>(3).setConstant(
        default_translation_variance);
    for (auto &i : shots_) {
      i.second.GetPose()->SetCovariance(default_covariance_matrix);
    }
  }
}

void BundleAdjuster::ComputeReprojectionErrors() {
  // Init errors
  for (auto &i : points_) {
    i.second.reprojection_errors.clear();
  }

  for (auto &observation : point_projection_observations_) {
    const auto projection_type =
        observation.camera->GetValue().GetProjectionType();
    geometry::Dispatch<ComputeResidualError>(projection_type, use_analytic_,
                                             observation);
  }
  for (auto &observation : point_rig_projection_observations_) {
    const auto projection_type =
        observation.camera->GetValue().GetProjectionType();
    geometry::Dispatch<ComputeRigResidualError>(projection_type, use_analytic_,
                                                observation);
  }
}

geometry::Camera BundleAdjuster::GetCamera(const std::string &id) const {
  if (cameras_.find(id) == cameras_.end()) {
    throw std::runtime_error("Camera " + id + " doesn't exists");
  }
  return cameras_.at(id).GetValue();
}

geometry::Similarity BundleAdjuster::GetBias(const std::string &id) const {
  if (bias_.find(id) == bias_.end()) {
    throw std::runtime_error("Camera " + id + " doesn't exists");
  }
  return bias_.at(id).GetValue();
}

Shot BundleAdjuster::GetShot(const std::string &id) const {
  if (shots_.find(id) == shots_.end()) {
    throw std::runtime_error("Shot " + id + " doesn't exists");
  }
  return shots_.at(id);
}

Point BundleAdjuster::GetPoint(const std::string &id) const {
  if (points_.find(id) == points_.end()) {
    throw std::runtime_error("Point " + id + " doesn't exists");
  }
  return points_.at(id);
}

Reconstruction BundleAdjuster::GetReconstruction(const std::string &id) const {
  if (reconstructions_.find(id) == reconstructions_.end()) {
    throw std::runtime_error("Reconstruction " + id + " doesn't exists");
  }
  return reconstructions_.at(id);
}

RigCamera BundleAdjuster::GetRigCamera(const std::string &rig_camera_id) const {
  if (rig_cameras_.find(rig_camera_id) == rig_cameras_.end()) {
    throw std::runtime_error("Rig camera " + rig_camera_id + " doesn't exists");
  }
  return rig_cameras_.at(rig_camera_id);
}

RigInstance BundleAdjuster::GetRigInstance(
    const std::string &instance_id) const {
  if (rig_instances_.find(instance_id) == rig_instances_.end()) {
    throw std::runtime_error("Rig instance " + instance_id + " doesn't exists");
  }
  return rig_instances_.at(instance_id);
}

std::string BundleAdjuster::BriefReport() const {
  return last_run_summary_.BriefReport();
}

std::string BundleAdjuster::FullReport() const {
  return last_run_summary_.FullReport();
}
}  // namespace bundle
