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

void BundleAdjuster::AddCameraNew(const std::string &id, const Camera& camera, const Camera& prior, bool constant){  
  Camera sigma = camera;
  VecXd disto_prior_sd(GetParamsCount(BACameraParameters::DISTO));
  disto_prior_sd << k1_sd_, k2_sd_, k3_sd_, p1_sd_, p2_sd_;
  sigma.SetDistortion(disto_prior_sd);
  Vec2d principal_point_prior_sd;
  principal_point_prior_sd << c_prior_sd_, c_prior_sd_;
  sigma.SetPrincipalPoint(principal_point_prior_sd);
  sigma.SetFocal(focal_prior_sd_);
  sigma.SetAspectRatio(focal_prior_sd_);

  cameras_new_.emplace(id, BACameraNew(camera, prior, sigma));

  if (constant) {
    cameras_new_.at(id).parameters = BACameraParameters(0);
  } else {
    switch (camera.GetProjectionType()) {
      case ProjectionType::PERSPECTIVE:
      case ProjectionType::FISHEYE:
        cameras_new_.at(id).parameters =
            BACameraParameters(static_cast<int>(BACameraParameters::FOCAL) |
                               static_cast<int>(BACameraParameters::K1) |
                               static_cast<int>(BACameraParameters::K2));
        break;
      case ProjectionType::BROWN:
        cameras_new_.at(id).parameters = BACameraParameters(
            static_cast<int>(BACameraParameters::FOCAL) |
            static_cast<int>(BACameraParameters::PRINCIPAL_POINT) |
            static_cast<int>(BACameraParameters::ASPECT_RATIO) |
            static_cast<int>(BACameraParameters::DISTO));
        break;
      case ProjectionType::DUAL:
        cameras_new_.at(id).parameters = BACameraParameters(
            static_cast<int>(BACameraParameters::FOCAL) |
            static_cast<int>(BACameraParameters::K1) |
            static_cast<int>(BACameraParameters::K2) |
            static_cast<int>(BACameraParameters::TRANSITION));
        break;
      default:
      case ProjectionType::SPHERICAL:
        cameras_new_.at(id).parameters = BACameraParameters(0);
        break;
    }
  }
}
void BundleAdjuster::AddPerspectiveCamera(const std::string &id, double focal,
                                          double k1, double k2,
                                          double focal_prior, double k1_prior,
                                          double k2_prior, bool constant) {
  if (use_new_) {
    AddCameraNew(
        id, Camera::CreatePerspectiveCamera(focal, k1, k2),
        Camera::CreatePerspectiveCamera(focal_prior, k1_prior, k2_prior),
        constant);
  } else {
    cameras_[id] =
        std::unique_ptr<BAPerspectiveCamera>(new BAPerspectiveCamera());
    BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*cameras_[id]);
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.constant = constant;
    c.focal_prior = focal_prior;
    c.k1_prior = k1_prior;
    c.k2_prior = k2_prior;
  }
}

void BundleAdjuster::AddBrownPerspectiveCamera(
    const BABrownPerspectiveCamera &c) {
  if (use_new_) {
    Vec2d principal_point;
    principal_point << c.GetCX(), c.GetCY();
    VecXd distortion(static_cast<int>(Disto::COUNT));
    distortion << c.GetK1(), c.GetK2(), c.GetK3(), c.GetP1(), c.GetP2();
    Vec2d principal_point_prior;
    principal_point_prior << c.c_x_prior, c.c_y_prior;
    VecXd distortion_prior(static_cast<int>(Disto::COUNT));
    distortion_prior << c.k1_prior, c.k2_prior, c.k3_prior, c.p1_prior, c.p2_prior;
    AddCameraNew(
        c.id,
        Camera::CreateBrownCamera(c.GetFocalX(), c.GetFocalY() / c.GetFocalX(),
                                  principal_point, distortion),
        Camera::CreateBrownCamera(c.focal_x_prior,
                                  c.focal_y_prior / c.focal_x_prior,
                                  principal_point_prior, distortion_prior),
        c.constant);
  } else {
    cameras_[c.id] = std::unique_ptr<BABrownPerspectiveCamera>(
        new BABrownPerspectiveCamera(c));
  }
}

void BundleAdjuster::AddFisheyeCamera(const std::string &id, double focal,
                                      double k1, double k2, double focal_prior,
                                      double k1_prior, double k2_prior,
                                      bool constant) {
  if (use_new_) {
    AddCameraNew(id, Camera::CreateFisheyeCamera(focal, k1, k2),
                 Camera::CreateFisheyeCamera(focal_prior, k1_prior, k2_prior),
                 constant);
  } else {
    cameras_[id] = std::unique_ptr<BAFisheyeCamera>(new BAFisheyeCamera());
    BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*cameras_[id]);
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.constant = constant;
    c.focal_prior = focal_prior;
    c.k1_prior = k1_prior;
    c.k2_prior = k2_prior;
  }
}

void BundleAdjuster::AddDualCamera(const std::string &id, double focal,
                                   double k1, double k2, double focal_prior,
                                   double k1_prior, double k2_prior,
                                   double transition, bool constant) {
  if (use_new_) {
    AddCameraNew(
        id, Camera::CreateDualCamera(transition, focal, k1, k2),
        Camera::CreateDualCamera(transition, focal_prior, k1_prior, k2_prior),
        constant);
  } else {
    cameras_[id] = std::unique_ptr<BADualCamera>(new BADualCamera());
    BADualCamera &c = static_cast<BADualCamera &>(*cameras_[id]);
    c.id = id;
    c.parameters[BA_DUAL_CAMERA_FOCAL] = focal;
    c.parameters[BA_DUAL_CAMERA_K1] = k1;
    c.parameters[BA_DUAL_CAMERA_K2] = k2;
    c.parameters[BA_DUAL_CAMERA_TRANSITION] = transition;
    c.constant = constant;
    c.focal_prior = focal_prior;
    c.k1_prior = k1_prior;
    c.k2_prior = k2_prior;
  }
}

void BundleAdjuster::AddEquirectangularCamera(const std::string &id) {
  if (use_new_) {
    AddCameraNew(id, Camera::CreateSphericalCamera(),
                 Camera::CreateSphericalCamera(), true);
  } else {
    cameras_[id] =
        std::unique_ptr<BAEquirectangularCamera>(new BAEquirectangularCamera());
    BAEquirectangularCamera &c =
        static_cast<BAEquirectangularCamera &>(*cameras_[id]);
    c.id = id;
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
  if (use_new_) {
    BAPointProjectionObservationNew o;
    o.shot = &shots_[shot];
    o.camera = &cameras_new_.at(o.shot->camera);
    o.point = &points_[point];
    o.coordinates[0] = x;
    o.coordinates[1] = y;
    o.std_deviation = std_deviation;
    point_projection_observations_new_.push_back(o);
  } else {
    BAPointProjectionObservation o;
    o.shot = &shots_[shot];
    o.camera = cameras_[o.shot->camera].get();
    o.point = &points_[point];
    o.coordinates[0] = x;
    o.coordinates[1] = y;
    o.std_deviation = std_deviation;
    point_projection_observations_.push_back(o);
  }
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

void BundleAdjuster::SetUseNew(bool b){
  use_new_ = b;
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

  if(use_new_){
    for (auto &i : cameras_new_) {
      auto data = i.second.GetData();

      // Push data block
      const auto data_size = static_cast<int>(BACameraParameters::COUNT);
      problem.AddParameterBlock(data, data_size);

      // Lock parameters based on bitmask of parameters :
      const auto parameters = static_cast<int>(i.second.parameters);

      // Set it as fully constant (no bits)
      if (!parameters) {
        problem.SetParameterBlockConstant(data);
      // Get relevant bits as a vector of integers
      } else {
        std::vector<int> subset;
        for(int i = 0; i < data_size; ++i ){
          if(!((1 << i) & parameters)){
            subset.push_back(i);
            std::cout << i << " ";
          }
        }
        std::cout << std::endl;
        problem.SetParameterization(data, new ceres::SubsetParameterization(data_size, subset));
      }
    }
  }
  else{
    for (auto &i : cameras_) {
      switch (i.second->type()) {
        case BA_PERSPECTIVE_CAMERA: {
          BAPerspectiveCamera &c =
              static_cast<BAPerspectiveCamera &>(*i.second);
          problem.AddParameterBlock(c.parameters, BA_CAMERA_NUM_PARAMS);
          if (i.second->constant) {
            problem.SetParameterBlockConstant(c.parameters);
          }
          break;
        }
        case BA_BROWN_PERSPECTIVE_CAMERA: {
          BABrownPerspectiveCamera &c =
              static_cast<BABrownPerspectiveCamera &>(*i.second);
          problem.AddParameterBlock(c.parameters, BA_BROWN_CAMERA_NUM_PARAMS);
          if (i.second->constant) {
            problem.SetParameterBlockConstant(c.parameters);
          }
          break;
        }
        case BA_FISHEYE_CAMERA: {
          BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*i.second);
          problem.AddParameterBlock(c.parameters, BA_CAMERA_NUM_PARAMS);
          if (i.second->constant) {
            problem.SetParameterBlockConstant(c.parameters);
          }
          break;
        }
        case BA_DUAL_CAMERA: {
          BADualCamera &c = static_cast<BADualCamera &>(*i.second);
          problem.AddParameterBlock(c.parameters, BA_DUAL_CAMERA_NUM_PARAMS);
          ceres::CostFunction* transition_barrier =
              new ceres::AutoDiffCostFunction<BAParameterBarrier, 1, BA_DUAL_CAMERA_NUM_PARAMS>(
                  new BAParameterBarrier(0.0, 1.0, BA_DUAL_CAMERA_TRANSITION));
          problem.AddResidualBlock(transition_barrier, NULL, c.parameters);
          if (i.second->constant) {
            problem.SetParameterBlockConstant(c.parameters);
          }
          break;
        }
        case BA_EQUIRECTANGULAR_CAMERA:
          // No parameters for now
          break;
      }
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
  if (use_new_) {
    for (auto &observation : point_projection_observations_new_) {
      AddObservationResidualBlockNew(observation, projection_loss, &problem);
    }
  } else {
    for (auto &observation : point_projection_observations_) {
      AddObservationResidualBlock(observation, projection_loss, &problem);
    }
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
  if(use_new_){
    for (auto &i : cameras_new_) {
      auto data = i.second.GetData();

      // Push data block
      const auto data_size = static_cast<int>(BACameraParameters::COUNT);
      problem.AddParameterBlock(data, data_size);

      // Lock parameters based on bitmask of parameters :
      const auto parameters = static_cast<int>(i.second.parameters);

      // Set it as fully constant (no bits)

    BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*i.second);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BrownInternalParametersPriorError,
              BA_BROWN_CAMERA_NUM_PARAMS, BA_BROWN_CAMERA_NUM_PARAMS>(
                new BrownInternalParametersPriorError(c.focal_x_prior, focal_prior_sd_,
                                                      c.focal_y_prior, focal_prior_sd_,
                                                      c.c_x_prior, c_prior_sd_,
                                                      c.c_y_prior, c_prior_sd_,
                                                      c.k1_prior, k1_sd_,
                                                      c.k2_prior, k2_sd_,
                                                      c.p1_prior, p1_sd_,
                                                      c.p2_prior, p2_sd_,
                                                      c.k3_prior, k3_sd_));
  }
  else{
  for (auto &i : cameras_) {
    switch (i.second->type()) {
      case BA_PERSPECTIVE_CAMERA:
      {
        BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*i.second);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BasicRadialInternalParametersPriorError, 
              BA_CAMERA_NUM_PARAMS, BA_CAMERA_NUM_PARAMS>(
                new BasicRadialInternalParametersPriorError(c.focal_prior, focal_prior_sd_,
                                                            c.k1_prior, k1_sd_,
                                                            c.k2_prior, k2_sd_));

        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  c.parameters);
        break;
      }
      case BA_BROWN_PERSPECTIVE_CAMERA:
      {
        BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*i.second);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BrownInternalParametersPriorError,
              BA_BROWN_CAMERA_NUM_PARAMS, BA_BROWN_CAMERA_NUM_PARAMS>(
                new BrownInternalParametersPriorError(c.focal_x_prior, focal_prior_sd_,
                                                      c.focal_y_prior, focal_prior_sd_,
                                                      c.c_x_prior, c_prior_sd_,
                                                      c.c_y_prior, c_prior_sd_,
                                                      c.k1_prior, k1_sd_,
                                                      c.k2_prior, k2_sd_,
                                                      c.p1_prior, p1_sd_,
                                                      c.p2_prior, p2_sd_,
                                                      c.k3_prior, k3_sd_));

        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  c.parameters);
        break;
      }
      case BA_FISHEYE_CAMERA:
      {
        BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*i.second);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BasicRadialInternalParametersPriorError,
              BA_CAMERA_NUM_PARAMS, BA_CAMERA_NUM_PARAMS>(
                new BasicRadialInternalParametersPriorError(c.focal_prior, focal_prior_sd_,
                                                            c.k1_prior, k1_sd_,
                                                            c.k2_prior, k2_sd_));

        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  c.parameters);
        break;
      }
      case BA_DUAL_CAMERA:
      {
        BADualCamera &c = static_cast<BADualCamera &>(*i.second);

        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<BasicRadialInternalParametersPriorError,
              BA_CAMERA_NUM_PARAMS, BA_DUAL_CAMERA_NUM_PARAMS>(
                new BasicRadialInternalParametersPriorError(c.focal_prior, focal_prior_sd_,
                                                            c.k1_prior, k1_sd_,
                                                            c.k2_prior, k2_sd_));

        problem.AddResidualBlock(cost_function,
                                  NULL,
                                  c.parameters);
        break;
      }
      case BA_EQUIRECTANGULAR_CAMERA:
        break;
    }
  }
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

void BundleAdjuster::AddObservationResidualBlockNew(
    const BAPointProjectionObservationNew &observation, ceres::LossFunction *loss,
    ceres::Problem *problem) {
  ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<
      ReprojectionError, 2, GetEnumAsConstExpr(BACameraParameters::COUNT),
      GetEnumAsConstExpr(BAShotParameters::COUNT), 3>(new ReprojectionError(
      observation.camera->GetValue().GetProjectionType(), observation.coordinates,
      observation.std_deviation));
  problem->AddResidualBlock(cost_function, loss, observation.camera->GetData(),
                            observation.shot->parameters.data(),
                            observation.point->parameters.data());
}

void BundleAdjuster::AddObservationResidualBlock(
    const BAPointProjectionObservation &observation,
    ceres::LossFunction *loss,
    ceres::Problem *problem) {
  switch (observation.camera->type()) {
    case BA_PERSPECTIVE_CAMERA:
    {
      BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*observation.camera);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<PerspectiveReprojectionError, 2, 3, 6, 3>(
              new PerspectiveReprojectionError(observation.coordinates[0],
                                               observation.coordinates[1],
                                               observation.std_deviation));

      problem->AddResidualBlock(cost_function,
                                loss,
                                c.parameters,
                                observation.shot->parameters.data(),
                                observation.point->parameters.data());
      break;
    }
    case BA_BROWN_PERSPECTIVE_CAMERA:
    {
      BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*observation.camera);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<BrownPerspectiveReprojectionError, 2, 9, 6, 3>(
              new BrownPerspectiveReprojectionError(observation.coordinates[0],
                                                    observation.coordinates[1],
                                                    observation.std_deviation));

      problem->AddResidualBlock(cost_function,
                                loss,
                                c.parameters,
                                observation.shot->parameters.data(),
                                observation.point->parameters.data());
      break;
    }
    case BA_FISHEYE_CAMERA:
    {
      BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*observation.camera);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<FisheyeReprojectionError, 2, 3, 6, 3>(
              new FisheyeReprojectionError(observation.coordinates[0],
                                           observation.coordinates[1],
                                           observation.std_deviation));

      problem->AddResidualBlock(cost_function,
                                loss,
                                c.parameters,
                                observation.shot->parameters.data(),
                                observation.point->parameters.data());
      break;
    }
    case BA_DUAL_CAMERA:
    {
      BADualCamera &c = static_cast<BADualCamera &>(*observation.camera);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<DualReprojectionError, 2, 4, 6, 3>(
              new DualReprojectionError(observation.coordinates[0],
                                           observation.coordinates[1],
                                           observation.std_deviation));

      problem->AddResidualBlock(cost_function,
                                loss,
                                c.parameters,
                                observation.shot->parameters.data(),
                                observation.point->parameters.data());
      break;
    }
    case BA_EQUIRECTANGULAR_CAMERA:
    {
      BAEquirectangularCamera &c = static_cast<BAEquirectangularCamera &>(*observation.camera);
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<EquirectangularReprojectionError, 3, 6, 3>(
              new EquirectangularReprojectionError(observation.coordinates[0],
                                                   observation.coordinates[1],
                                                   observation.std_deviation));

      problem->AddResidualBlock(cost_function,
                                loss,
                                observation.shot->parameters.data(),
                                observation.point->parameters.data());
      break;
    }
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

  // Sum over all observations
  for (int i = 0; i < point_projection_observations_.size(); ++i) {
    auto& projection = point_projection_observations_[i];
    switch (projection.camera->type()) {
      case BA_PERSPECTIVE_CAMERA:
      {
        BAPerspectiveCamera &c = static_cast<BAPerspectiveCamera &>(*projection.camera);

        PerspectiveReprojectionError pre(projection.coordinates[0],
                                          projection.coordinates[1],
                                          1.0);
        double residuals[2];
        pre(c.parameters,
            projection.shot->parameters.data(),
            projection.point->parameters.data(),
            residuals);
        projection.point->reprojection_errors[projection.shot->id] = Eigen::Vector2d(residuals[0], residuals[1]);
        break;
      }
      case BA_BROWN_PERSPECTIVE_CAMERA:
      {
        BABrownPerspectiveCamera &c = static_cast<BABrownPerspectiveCamera &>(*projection.camera);

        BrownPerspectiveReprojectionError bpre(projection.coordinates[0],
                                                projection.coordinates[1],
                                                1.0);
        double residuals[2];
        bpre(c.parameters,
              projection.shot->parameters.data(),
              projection.point->parameters.data(),
              residuals);
        projection.point->reprojection_errors[projection.shot->id] = Eigen::Vector2d(residuals[0], residuals[1]);
        break;
      }
      case BA_FISHEYE_CAMERA:
      {
        BAFisheyeCamera &c = static_cast<BAFisheyeCamera &>(*projection.camera);

        FisheyeReprojectionError pre(projection.coordinates[0],
                                      projection.coordinates[1],
                                      1.0);
        double residuals[2];
        pre(c.parameters,
            projection.shot->parameters.data(),
            projection.point->parameters.data(),
            residuals);
        projection.point->reprojection_errors[projection.shot->id] = Eigen::Vector2d(residuals[0], residuals[1]);
        break;
      }
      case BA_DUAL_CAMERA:
      {
        BADualCamera &c = static_cast<BADualCamera &>(*projection.camera);

        DualReprojectionError pre(projection.coordinates[0],
                                  projection.coordinates[1],
                                  1.0);

        double residuals[2];
        pre(c.parameters,
            projection.shot->parameters.data(),
            projection.point->parameters.data(),
            residuals);
        projection.point->reprojection_errors[projection.shot->id] = Eigen::Vector2d(residuals[0], residuals[1]);
        break;
      }
      case BA_EQUIRECTANGULAR_CAMERA:
      {
        BAEquirectangularCamera &c = static_cast<BAEquirectangularCamera &>(*projection.camera);

        EquirectangularReprojectionError ere(projection.coordinates[0],
                                              projection.coordinates[1],
                                              1.0);
        double residuals[3];
        ere(projection.shot->parameters.data(),
            projection.point->parameters.data(),
            residuals);
        projection.point->reprojection_errors[projection.shot->id] = Eigen::Vector3d(residuals[0], residuals[1], residuals[2]);
        break;
      }
    }
  }
}

Camera BundleAdjuster::GetCamera(const std::string &id){
  if(cameras_new_.find(id) == cameras_new_.end()){
    throw std::runtime_error("Camera doesn't exists");
  }
  auto c = cameras_new_.at(id).GetValue();
  std::cout << c.GetFocal() << std::endl;
  return cameras_new_.at(id).GetValue();
}
BAPerspectiveCamera BundleAdjuster::GetPerspectiveCamera(
    const std::string &id) {
  return *(BAPerspectiveCamera *)cameras_[id].get();
}

BABrownPerspectiveCamera BundleAdjuster::GetBrownPerspectiveCamera(
    const std::string &id) {
  return *(BABrownPerspectiveCamera *)cameras_[id].get();
}

BAFisheyeCamera BundleAdjuster::GetFisheyeCamera(
    const std::string &id) {
  return *(BAFisheyeCamera *)cameras_[id].get();
}

BADualCamera BundleAdjuster::GetDualCamera(
    const std::string &id) {
  return *(BADualCamera *)cameras_[id].get();
}

BAEquirectangularCamera BundleAdjuster::GetEquirectangularCamera(
    const std::string &id) {
  return *(BAEquirectangularCamera *)cameras_[id].get();
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
