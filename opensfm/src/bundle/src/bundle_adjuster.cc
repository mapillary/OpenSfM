#include "../bundle_adjuster.h"
#include "absolute_motion_terms.h"
#include "position_functors.h"
#include "motion_prior_terms.h"
#include "relative_motion_terms.h"
#include "projection_errors.h"

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
  compute_covariances_ = false;
  covariance_estimation_valid_ = false;
  compute_reprojection_errors_ = true;
  adjust_absolute_position_std_ = false;
  max_num_iterations_ = 500;
  num_threads_ = 1;
  linear_solver_type_ = "SPARSE_NORMAL_CHOLESKY";
}

void BundleAdjuster::UpdateSigmas(){
  for (auto &camera : cameras_) {
    Camera sigma_camera = camera.second.GetValue();
    const auto parameters_types = sigma_camera.GetParametersTypes();
    std::unordered_map<int, double> std_dev_map;
    std_dev_map[static_cast<int>(Camera::Parameters::Focal)] = focal_prior_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::AspectRatio)] = focal_prior_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::Cx)] = c_prior_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::Cy)] = c_prior_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::K1)] = k1_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::K2)] = k2_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::K3)] = k3_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::P1)] = p1_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::P2)] = p2_sd_;
    std_dev_map[static_cast<int>(Camera::Parameters::Transition)] = 1.0;
    for(const auto type : parameters_types){
      sigma_camera.SetParameterValue(type, std_dev_map[static_cast<int>(type)]);
    }
    camera.second.SetSigma(sigma_camera);
  }
}
void BundleAdjuster::AddCamera(const std::string &id, const Camera& camera, const Camera& prior, bool constant){  
  Camera sigma = camera;
  cameras_.emplace(id, BACamera(camera, prior, sigma));
  UpdateSigmas();

  if (constant) {
    cameras_.at(id).SetParametersToOptimize({});
  }
}

void BundleAdjuster::AddShot(const std::string &id, 
                                 const std::string &camera,
                                 const Eigen::Vector3d& rotation,
                                 const Eigen::Vector3d& translation,
                                 bool constant) {
  BAShot s;
  s.id = id;
  s.camera = camera;
  s.SetRotationAndTranslation(rotation, translation);
  s.constant = constant;
  shots_[id] = s;
}

void BundleAdjuster::SetScaleSharing(const std::string &id, bool share) {
  const auto find = reconstructions_.find(id);
  if (find == reconstructions_.end()) {
    return;
  }
  find->second.shared = share;
}

void BundleAdjuster::AddReconstruction(const std::string &id,
                                           bool constant) {
  BAReconstruction r;
  r.id = id;
  r.constant = constant;
  r.shared = true;
  reconstructions_[id] = r;
}

void BundleAdjuster::AddReconstructionShot(
    const std::string &reconstruction_id, double scale,
    const std::string &shot_id) {
  const auto find = reconstructions_.find(reconstruction_id);
  if (find == reconstructions_.end()) {
    return;
  }
  find->second.scales[shot_id] = scale;
  find->second.shots[shot_id] = &shots_[shot_id];
}

void BundleAdjuster::AddPoint(const std::string &id, 
                                  const Eigen::Vector3d& position,
                                  bool constant) {
  BAPoint p;
  p.id = id;
  p.parameters = position;
  p.constant = constant;
  points_[id] = p;
}

void BundleAdjuster::AddPointProjectionObservation(const std::string &shot,
                                                   const std::string &point,
                                                   double x, double y,
                                                   double std_deviation) {
  BAPointProjectionObservation o;
  o.shot = &shots_[shot];
  o.camera = &cameras_.at(o.shot->camera);
  o.point = &points_[point];
  o.coordinates[0] = x;
  o.coordinates[1] = y;
  o.std_deviation = std_deviation;
  point_projection_observations_.push_back(o);
}

void BundleAdjuster::AddRotationPrior(
    const std::string &shot_id,
    double rx,
    double ry,
    double rz,
    double std_deviation) {
  BARotationPrior p;
  p.shot = &shots_[shot_id];
  p.rotation[0] = rx;
  p.rotation[1] = ry;
  p.rotation[2] = rz;
  p.std_deviation = std_deviation;
  rotation_priors_.push_back(p);
}

void BundleAdjuster::AddTranslationPrior(
    const std::string &shot_id,
    double tx,
    double ty,
    double tz,
    double std_deviation) {
  BATranslationPrior p;
  p.shot = &shots_[shot_id];
  p.translation[0] = tx;
  p.translation[1] = ty;
  p.translation[2] = tz;
  p.std_deviation = std_deviation;
  translation_priors_.push_back(p);
}

void BundleAdjuster::AddPositionPrior(
    const std::string &shot_id,
    double x,
    double y,
    double z,
    double std_deviation) {
  BAPositionPrior p;
  p.shot = &shots_[shot_id];
  p.position[0] = x;
  p.position[1] = y;
  p.position[2] = z;
  p.std_deviation = std_deviation;
  position_priors_.push_back(p);
}

void BundleAdjuster::AddPointPositionPrior(
    const std::string &point_id,
    double x,
    double y,
    double z,
    double std_deviation) {
  BAPointPositionPrior p;
  p.point = &points_[point_id];
  p.position[0] = x;
  p.position[1] = y;
  p.position[2] = z;
  p.std_deviation = std_deviation;
  point_position_priors_.push_back(p);
}

void BundleAdjuster::SetOriginShot(const std::string &shot_id) {
  BAShot *shot = &shots_[shot_id];
  for (int i = 0; i < 6; ++i) shot->parameters[0] = 0;
  shot->constant = true;
}

void BundleAdjuster::SetUnitTranslationShot(const std::string &shot_id) {
  unit_translation_shot_ = &shots_[shot_id];
}

void BundleAdjuster::AddRelativeMotion(const BARelativeMotion &rm) {
  relative_motions_.push_back(rm);
}

void BundleAdjuster::AddRelativeSimilarity(const BARelativeSimilarity &rm) {
  relative_similarity_.push_back(rm);
}

void BundleAdjuster::AddRelativeRotation(const BARelativeRotation &rr) {
  relative_rotations_.push_back(rr);
}

void BundleAdjuster::AddCommonPosition(const std::string &shot_id1,
                                           const std::string &shot_id2,
                                           double margin,
                                           double std_deviation) {
  BACommonPosition a;
  a.shot1 = &shots_[shot_id1];
  a.shot2 = &shots_[shot_id2];
  a.margin = margin;
  a.std_deviation = std_deviation;
  common_positions_.push_back(a);
}

void BundleAdjuster::AddAbsolutePosition(const std::string &shot_id,
                                             const Eigen::Vector3d& position,
                                             double std_deviation,
                                             const std::string& std_deviation_group) {
  BAAbsolutePosition a;
  a.shot = &shots_[shot_id];
  a.position = position;
  a.std_deviation = std_deviation;
  a.std_deviation_group = std_deviation_group;
  absolute_positions_.push_back(a);
}

void BundleAdjuster::AddAbsoluteUpVector(
    const std::string &shot_id, 
    const Eigen::Vector3d& up_vector,
    double std_deviation) {
  BAAbsoluteUpVector a;
  a.shot = &shots_[shot_id];
  a.up_vector = up_vector;
  a.std_deviation = std_deviation;
  absolute_up_vectors_.push_back(a);
}

void BundleAdjuster::AddAbsolutePan(
    const std::string &shot_id,
    double angle,
    double std_deviation) {
  BAAbsoluteAngle a;
  a.shot = &shots_[shot_id];
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_pans_.push_back(a);
}

void BundleAdjuster::AddAbsoluteTilt(
    const std::string &shot_id,
    double angle,
    double std_deviation) {
  BAAbsoluteAngle a;
  a.shot = &shots_[shot_id];
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_tilts_.push_back(a);
}

void BundleAdjuster::AddAbsoluteRoll(
    const std::string &shot_id,
    double angle,
    double std_deviation) {
  BAAbsoluteAngle a;
  a.shot = &shots_[shot_id];
  a.angle = angle;
  a.std_deviation = std_deviation;
  absolute_rolls_.push_back(a);
}

void BundleAdjuster::AddPointPositionShot(const std::string &point_id,
                                              const std::string &shot_id,
                                              const std::string &reconstruction_id,
                                              const Eigen::Vector3d& position,
                                              double std_deviation,
                                              const PositionConstraintType& type) {
  BAPointPositionShot a;
  a.point_id = point_id;
  a.shot_id = shot_id;
  a.reconstruction_id = reconstruction_id;
  a.position = position;
  a.std_deviation = std_deviation;
  a.type = type;
  point_positions_shot_.push_back(a);
}

void BundleAdjuster::AddPointPositionWorld(const std::string &point_id,
                                               const Eigen::Vector3d& position,
                                               double std_deviation,
                                               const PositionConstraintType& type) {
  BAPointPositionWorld a;
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

void BundleAdjuster::SetAdjustAbsolutePositionStd(bool adjust){
  adjust_absolute_position_std_ = adjust;
}

void BundleAdjuster::SetMaxNumIterations(int miter) {
  max_num_iterations_ = miter;
}

void BundleAdjuster::SetNumThreads(int n) {
  num_threads_ = n;
}

void BundleAdjuster::SetLinearSolverType(std::string t) {
  linear_solver_type_ = t;
}

void BundleAdjuster::SetInternalParametersPriorSD(
    double focal_sd,
    double c_sd,
    double k1_sd,
    double k2_sd,
    double p1_sd,
    double p2_sd,
    double k3_sd) {
  focal_prior_sd_ = focal_sd;
  c_prior_sd_ = c_sd;
  k1_sd_ = k1_sd;
  k2_sd_ = k2_sd;
  p1_sd_ = p1_sd;
  p2_sd_ = p2_sd;
  k3_sd_ = k3_sd;
  UpdateSigmas();
}

void BundleAdjuster::SetComputeCovariances(bool v) {
  compute_covariances_ = v;
}

bool BundleAdjuster::GetCovarianceEstimationValid() {
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

ceres::LinearSolverType LinearSolverTypeFromNamae(std::string name) {
  if (name.compare("DENSE_QR") == 0) {
    return ceres::DENSE_QR;
  } else if (name.compare("DENSE_NORMAL_CHOLESKY") == 0) {
    return ceres::DENSE_NORMAL_CHOLESKY;
  } else if (name.compare("SPARSE_NORMAL_CHOLESKY") == 0) {
    return ceres::SPARSE_NORMAL_CHOLESKY;
  } else if (name.compare("CGNR") == 0) {
    return ceres::CGNR;
  } else if (name.compare("DENSE_SCHUR") == 0) {
    return ceres::DENSE_SCHUR;
  } else if (name.compare("SPARSE_SCHUR") == 0) {
    return ceres::SPARSE_SCHUR;
  } else if (name.compare("ITERATIVE_SCHUR") == 0) {
    return ceres::ITERATIVE_SCHUR;
  }
  return ceres::SPARSE_SCHUR;
}

void BundleAdjuster::AddLinearMotion(const std::string &shot0_id,
                                         const std::string &shot1_id,
                                         const std::string &shot2_id,
                                         double alpha,
                                         double position_std_deviation,
                                         double orientation_std_deviation) {
  BALinearMotion a;
  a.shot0 = &shots_[shot0_id];
  a.shot1 = &shots_[shot1_id];
  a.shot2 = &shots_[shot2_id];
  a.alpha = alpha;
  a.position_std_deviation = position_std_deviation;
  a.orientation_std_deviation = orientation_std_deviation;
  linear_motion_prior_.push_back(a);
}

struct BAStdDeviationConstraint {
  BAStdDeviationConstraint() = default;

  template <typename T>
  bool operator()(const T* const std_deviation, T* residuals) const {
    T std = std_deviation[0];
    residuals[0] = ceres::log(T(1.0)/ceres::sqrt(T(2.0*M_PI)*std*std));
    return true;
  }
};

struct BAParameterBarrier {
  BAParameterBarrier(double lower_bound, double upper_bound, int index)
      : lower_bound_(lower_bound), upper_bound_(upper_bound), index_(index) {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    T eps = T(1e-10);
    T value = parameters[index_];
    T zero = 2.0*ceres::log((T(upper_bound_)-T(lower_bound_))*0.5);
    T penalty = ceres::log(value-T(lower_bound_)+eps)+ceres::log(T(upper_bound_)-value+eps);
    residuals[0] = penalty + zero;
    return true;
  }

  double lower_bound_;
  double upper_bound_;
  int index_;
};

template <class T>
struct ErrorTraits {
  using Type = ReprojectionError2D;
  static constexpr int Size = 2;
};
// template <>
// struct ErrorTraits<SphericalCamera> {
//   using Type = ReprojectionError3D;
//   static constexpr int Size = 3;
// };

struct AddProjectionError {
  template <class T>
  static void Apply(const BAPointProjectionObservation &obs,
                    ceres::LossFunction *loss, ceres::Problem *problem) {
    using ErrorType = typename ErrorTraits<T>::Type;
    constexpr static int ErrorSize = ErrorTraits<T>::Size;
    constexpr static int CameraSize = T::Size;
    constexpr static int ShotSize = 6;

    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<ErrorType, ErrorSize, CameraSize,
                                        ShotSize, 3>(
            new ErrorType(obs.camera->GetValue().GetProjectionType(),
                          obs.coordinates, obs.std_deviation));
    problem->AddResidualBlock(
        cost_function, loss, obs.camera->GetValueData().data(),
        obs.shot->parameters.data(), obs.point->parameters.data());
  }
};

struct ComputeResidualError {
  template <class T>
  static void Apply(const BAPointProjectionObservation &obs) {
    using ErrorType = typename ErrorTraits<T>::Type;
    constexpr static int ErrorSize = ErrorTraits<T>::Size;

    VecNd<ErrorSize> residuals;
    ErrorType error(obs.camera->GetValue().GetProjectionType(), obs.coordinates,
                    1.0);
    error(obs.camera->GetValueData().data(), obs.shot->parameters.data(),
          obs.point->parameters.data(), residuals.data());
    obs.point->reprojection_errors[obs.shot->id] = residuals;
  }
};

struct AddCameraPriorlError {
  template <class T>
  static void Apply(BACamera &camera, ceres::Problem *problem) {
    auto *prior_function = new BADataPriorError<Camera>(&camera);

    // Set some logarithmic prior for Focal and Aspect ratio (if any)
    const auto camera_object = camera.GetValue();
    const auto types = camera_object.GetParametersTypes();
    for (int i = 0; i < types.size(); ++i) {
      const auto t = types[i];
      if (t == Camera::Parameters::Focal ||
          t == Camera::Parameters::AspectRatio) {
        prior_function->SetScaleType(
            i, BADataPriorError<Camera>::ScaleType::LOGARITHMIC);
      }
    }

    constexpr static int CameraSize = T::Size;
    ceres::CostFunction *cost_function =
        new ceres::AutoDiffCostFunction<BADataPriorError<Camera>, CameraSize,
                                        CameraSize>(prior_function);
    problem->AddResidualBlock(cost_function, NULL,
                              camera.GetValueData().data());
  }
};

void BundleAdjuster::Run() {
  ceres::Problem problem;

  // Init paramater blocks.
  for (auto &i : shots_) {
    if (i.second.constant) {
      problem.AddParameterBlock(i.second.parameters.data(), BA_SHOT_NUM_PARAMS);
      problem.SetParameterBlockConstant(i.second.parameters.data());
    } else {
      problem.AddParameterBlock(i.second.parameters.data(), BA_SHOT_NUM_PARAMS);
    }
  }

  for (auto &i : cameras_) {
    auto& data = i.second.GetValueData();
    problem.AddParameterBlock(data.data(), data.size());

    // Lock parameters based on bitmask of parameters : only constant for now
    if (i.second.GetParametersToOptimize().empty()) {
      problem.SetParameterBlockConstant(data.data());
    }

    // Add a barrier for constraining transition of dual to stay in [0, 1]
    const auto camera = i.second.GetValue();
    if (camera.GetProjectionType() == ProjectionType::DUAL) {
      const auto types = camera.GetParametersTypes();
      int index = -1;
      for(int i = 0; i < types.size() && index < 0; ++i){
        if(types[i] == Camera::Parameters::Transition){
          index = i;
        }
      }
      // if(index >= 0){
      //   ceres::CostFunction *transition_barrier =
      //       new ceres::AutoDiffCostFunction<BAParameterBarrier, 1,
                                            //SizeTraits<DualCamera>::Size>(
      //           new BAParameterBarrier(0.0, 1.0, index));
      //   problem.AddResidualBlock(transition_barrier, NULL, data.data());
      // }
    }
  }
  
  
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

  // Add reprojection error blocks
  ceres::LossFunction *projection_loss = CreateLossFunction(
      point_projection_loss_name_, point_projection_loss_threshold_);
  for (auto &observation : point_projection_observations_) {
    const auto projection_type =
        observation.camera->GetValue().GetProjectionType();
    Dispatch<AddProjectionError>(projection_type, observation, projection_loss,
                                 &problem);
  }

  // Add rotation priors
  for (auto &rp : rotation_priors_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<RotationPriorError, 3, 6>(
            new RotationPriorError(rp.rotation, rp.std_deviation));

    problem.AddResidualBlock(cost_function,
                             NULL,
                             rp.shot->parameters.data());
  }

  // Add translation priors
  for (auto &tp : translation_priors_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<TranslationPriorError, 3, 6>(
            new TranslationPriorError(tp.translation, tp.std_deviation));

    problem.AddResidualBlock(cost_function,
                             NULL,
                             tp.shot->parameters.data());
  }

  // Add position priors
  for (auto &pp : position_priors_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PositionPriorError, 3, 6>(
            new PositionPriorError(pp.position, pp.std_deviation));

    problem.AddResidualBlock(cost_function,
                             NULL,
                             pp.shot->parameters.data());
  }

  // Add point position priors
  for (auto &pp : point_position_priors_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<PointPositionPriorError, 3, 3>(
            new PointPositionPriorError(pp.position, pp.std_deviation));

    problem.AddResidualBlock(cost_function,
                             NULL,
                             pp.point->parameters.data());
  }

  // Add internal parameter priors blocks
  for (auto &i : cameras_) {
    const auto projection_type =
        i.second.GetValue().GetProjectionType();
    Dispatch<AddCameraPriorlError>(projection_type, i.second, &problem);
  }

  // Add unit translation block
  if (unit_translation_shot_) {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<UnitTranslationPriorError, 1, 6>(
            new UnitTranslationPriorError());

    problem.AddResidualBlock(cost_function,
                             NULL,
                             unit_translation_shot_->parameters.data());
  }

  // Add relative motion errors
  for (auto &rp : relative_motions_) {
    double robust_threshold = relative_motion_loss_threshold_*rp.robust_multiplier;
    ceres::LossFunction *relative_motion_loss = CreateLossFunction(
      relative_motion_loss_name_, robust_threshold);

    auto *cost_function =
        new ceres::AutoDiffCostFunction<BARelativeMotionError, 6, 6, 1, 6>(
            new BARelativeMotionError(rp.parameters,
                                      rp.scale_matrix));
    double *scale =
        reconstructions_[rp.reconstruction_id_i].GetScalePtr(rp.shot_id_i);
    problem.AddResidualBlock(cost_function, relative_motion_loss,
                             shots_[rp.shot_id_i].parameters.data(), scale,
                             shots_[rp.shot_id_j].parameters.data());
  }

  // Add relative similarity errors
  for (auto &rp : relative_similarity_) {
    double robust_threshold = relative_motion_loss_threshold_*rp.robust_multiplier;
    ceres::LossFunction *relative_similarity_loss = CreateLossFunction(
      relative_motion_loss_name_, robust_threshold);

    auto *cost_function =
        new ceres::AutoDiffCostFunction<BARelativeSimilarityError, 7, 6, 1, 6,
                                        1>(new BARelativeSimilarityError(
            rp.parameters, rp.scale, rp.scale_matrix));
    double *scale_i =
        reconstructions_[rp.reconstruction_id_i].GetScalePtr(rp.shot_id_i);
    double *scale_j =
        reconstructions_[rp.reconstruction_id_j].GetScalePtr(rp.shot_id_j);
    problem.AddResidualBlock(cost_function, relative_similarity_loss,
                             shots_[rp.shot_id_i].parameters.data(), scale_i,
                             shots_[rp.shot_id_j].parameters.data(), scale_j);
  }

  // Add relative rotation errors
    ceres::LossFunction *relative_rotation_loss = CreateLossFunction(
      relative_motion_loss_name_, relative_motion_loss_threshold_);
  for (auto &rr : relative_rotations_) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<BARelativeRotationError, 3, 6, 6>(
            new BARelativeRotationError(rr.rotation, rr.scale_matrix));

    problem.AddResidualBlock(cost_function, relative_rotation_loss,
                             shots_[rr.shot_id_i].parameters.data(),
                             shots_[rr.shot_id_j].parameters.data());
  }

  // Add common position errors
  for (auto &c : common_positions_) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<BACommonPositionError, 3, 6, 6>(
            new BACommonPositionError(c.margin, c.std_deviation));

    problem.AddResidualBlock(cost_function, NULL, c.shot1->parameters.data(),
                             c.shot2->parameters.data());
  }

  // Add absolute position errors
  std::map<std::string,int> std_dev_group_remap;
  for (const auto& a : absolute_positions_){
    if(std_dev_group_remap.find(a.std_deviation_group) != std_dev_group_remap.end()){
      continue;
    }
    const int index = std_dev_group_remap.size();
    std_dev_group_remap[a.std_deviation_group] = index;
  }
  std::vector<double> std_deviations(std_dev_group_remap.size());
  for (const auto& a : absolute_positions_){
    std_deviations[std_dev_group_remap[a.std_deviation_group]] = a.std_deviation;
  }

  for (auto &a : absolute_positions_) {

    ceres::DynamicCostFunction *cost_function = nullptr;

    // camera parametrization
    ShotPositionShotParam pos_func(0);
    cost_function = new ceres::DynamicAutoDiffCostFunction<
        BAAbsolutePositionError<ShotPositionShotParam>>(
        new BAAbsolutePositionError<ShotPositionShotParam>(
            pos_func, a.position, 1.0, true,
            PositionConstraintType::XYZ));

    // world parametrization
    // ShotPositionWorldParam pos_func(0);
    // cost_function = new ceres::AutoDiffCostFunction<
    //     BAAbsolutePositionError<ShotPositionWorldParam>, 3, 6>(
    //     new BAAbsolutePositionError(pos_func, a.position, a.std_deviation));

    cost_function->AddParameterBlock(6);
    cost_function->AddParameterBlock(1);
    cost_function->SetNumResiduals(3);
    problem.AddResidualBlock(cost_function, NULL, a.shot->parameters.data(),
                             &std_deviations[std_dev_group_remap[a.std_deviation_group]]);
  }

  // Add regularizer term if we're adjusting for standart deviation, or lock them up.
  if(adjust_absolute_position_std_){
    for (int i = 0; i < std_deviations.size(); ++i) {
      ceres::CostFunction* std_dev_cost_function =
            new ceres::AutoDiffCostFunction<BAStdDeviationConstraint, 1, 1>(
                new BAStdDeviationConstraint());
      problem.AddResidualBlock(std_dev_cost_function, NULL, &std_deviations[i]);
    }
  }
  else{
    for (int i = 0; i < std_deviations.size(); ++i) {
      problem.SetParameterBlockConstant(&std_deviations[i]);
    }
  }

  // Add absolute up vector errors
  ceres::LossFunction *up_vector_loss = new ceres::CauchyLoss(1);
  for (auto &a : absolute_up_vectors_) {
    if (a.std_deviation > 0) {
      auto *up_vector_cost_function =
          new ceres::AutoDiffCostFunction<BAUpVectorError, 3, 6>(
              new BAUpVectorError(a.up_vector, a.std_deviation));

      problem.AddResidualBlock(up_vector_cost_function, up_vector_loss,
                               a.shot->parameters.data());
    }
  }

  // Add absolute pan (compass) errors
  ceres::LossFunction *pan_loss = new ceres::CauchyLoss(1);
  for (auto &a: absolute_pans_) {
    if (a.std_deviation > 0) {
      ceres::CostFunction* pan_cost_function =
          new ceres::AutoDiffCostFunction<BAPanAngleError, 1, 6>(
              new BAPanAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(pan_cost_function, pan_loss,
                               a.shot->parameters.data());
    }
  }

  // Add absolute tilt errors
  ceres::LossFunction *tilt_loss = new ceres::CauchyLoss(1);
  for (auto &a: absolute_tilts_) {
    if (a.std_deviation > 0) {
      ceres::CostFunction* tilt_cost_function =
          new ceres::AutoDiffCostFunction<BATiltAngleError, 1, 6>(
              new BATiltAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(tilt_cost_function, tilt_loss,
                               a.shot->parameters.data());
    }
  }

  // Add absolute roll errors
  ceres::LossFunction *roll_loss = new ceres::CauchyLoss(1);
  for (auto &a: absolute_rolls_) {
    if (a.std_deviation > 0) {
      ceres::CostFunction* roll_cost_function =
          new ceres::AutoDiffCostFunction<BARollAngleError, 1, 6>(
              new BARollAngleError(a.angle, a.std_deviation));
      problem.AddResidualBlock(roll_cost_function, roll_loss,
                               a.shot->parameters.data());
    }
  }

  // Add linear motion priors
  ceres::LossFunction *linear_motion_prior_loss_ = new ceres::CauchyLoss(1);
  for (auto &a : linear_motion_prior_) {
    auto *cost_function =
        new ceres::AutoDiffCostFunction<BALinearMotionError, 6, 6, 6, 6>(
            new BALinearMotionError(a.alpha, a.position_std_deviation,
                                    a.orientation_std_deviation));

    problem.AddResidualBlock(cost_function, linear_motion_prior_loss_,
                             a.shot0->parameters.data(), 
                             a.shot1->parameters.data(),
                             a.shot2->parameters.data());
  }

  // Add point positions with shot position priors
  for (auto &p : point_positions_shot_) {
    PointPositionScaledShot pos_func(0, 1, 2);
    auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
        BAAbsolutePositionError<PointPositionScaledShot>>(
        new BAAbsolutePositionError<PointPositionScaledShot>(
            pos_func, p.position, p.std_deviation, false, p.type));

    cost_function->AddParameterBlock(6);
    cost_function->AddParameterBlock(1);
    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);

    problem.AddResidualBlock(cost_function, NULL, 
                             shots_[p.shot_id].parameters.data(),
                             reconstructions_[p.reconstruction_id].GetScalePtr(p.shot_id), 
                             points_[p.point_id].parameters.data());
  }

  // Add point positions with world position priors
  for (auto &p : point_positions_world_) {
    PointPositionWorld pos_func(0);
    auto *cost_function = new ceres::DynamicAutoDiffCostFunction<
        BAAbsolutePositionError<PointPositionWorld>>(
        new BAAbsolutePositionError<PointPositionWorld>(pos_func, p.position, p.std_deviation, false, p.type));

    cost_function->AddParameterBlock(3);
    cost_function->SetNumResiduals(3);
    problem.AddResidualBlock(cost_function, NULL, 
                             points_[p.point_id].parameters.data());
  }

  // Solve
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.linear_solver_type = LinearSolverTypeFromNamae(linear_solver_type_);
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
    ceres::Covariance covariance(options);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    for (auto &i : shots_) {
      covariance_blocks.push_back(std::make_pair(i.second.parameters.data(),
                                                 i.second.parameters.data()));
    }

    bool worked = covariance.Compute(covariance_blocks, problem);

    if (worked) {
      for (auto &i : shots_) {
        covariance_estimation_valid_ = true;
        covariance.GetCovarianceBlock(i.second.parameters.data(),
                                      i.second.parameters.data(),
                                      i.second.covariance);
      }
      computed = true;
    }
  }

  if (!computed) { // If covariance estimation failed, use a default value
    for (auto &i : shots_) {
      covariance_estimation_valid_ = false;
      for (int k = 0; k < 6 * 6; ++k) {
        i.second.covariance[k] = 0.0;
      }
      double default_rotation_variance = 1e-5;
      double default_translation_variance = 1e-2;
      i.second.covariance[6 * 0 + 0] = default_rotation_variance;
      i.second.covariance[6 * 1 + 1] = default_rotation_variance;
      i.second.covariance[6 * 2 + 2] = default_rotation_variance;
      i.second.covariance[6 * 3 + 3] = default_translation_variance;
      i.second.covariance[6 * 4 + 4] = default_translation_variance;
      i.second.covariance[6 * 5 + 5] = default_translation_variance;
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
    Dispatch<ComputeResidualError>(projection_type, observation);
  }
}

Camera BundleAdjuster::GetCamera(const std::string &id){
  if(cameras_.find(id) == cameras_.end()){
    throw std::runtime_error("Camera doesn't exists");
  }
  return cameras_.at(id).GetValue();
}

BAShot BundleAdjuster::GetShot(const std::string &id) {
  return shots_[id];
}

BAPoint BundleAdjuster::GetPoint(const std::string &id) {
  return points_[id];
}

BAReconstruction BundleAdjuster::GetReconstruction(const std::string &id) {
  return reconstructions_[id];
}

std::string BundleAdjuster::BriefReport() {
  return last_run_summary_.BriefReport();
}

std::string BundleAdjuster::FullReport() {
  return last_run_summary_.FullReport();
}
