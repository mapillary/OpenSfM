#pragma once

#include <foundation/optional.h>
#include <geometry/camera.h>
#include <geometry/pose.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"
#include "ceres/rotation.h"

extern "C" {
#include <string.h>
}

enum {
  BA_SHOT_RX,
  BA_SHOT_RY,
  BA_SHOT_RZ,
  BA_SHOT_TX,
  BA_SHOT_TY,
  BA_SHOT_TZ,
  BA_SHOT_NUM_PARAMS
};

enum PositionConstraintType {
  X = 0x1,
  Y = 0x2,
  Z = 0x4,
  XY = X | Y,
  XYZ = XY | Z
};

struct BAPoint {
  std::string id;
  Eigen::Matrix<double, 3, 1> parameters;
  bool constant;
  std::map<std::string, VecXd> reprojection_errors;

  Vec3d GetPoint() const { return parameters; }
  void SetPoint(const Vec3d &p) { parameters = p; }
};

struct BADataNode {
  explicit BADataNode(const std::string &id) : id_(id) {}

  std::string GetID() const { return id_; }

 protected:
  std::string id_;
};
template <class T>
struct BAData : public BADataNode {
 public:
  using ValueType = T;

  BAData(const std::string &id, const T &value, const T &prior, const T &sigma)
      : BADataNode(id), value_(value), prior_(prior), sigma_(sigma) {}

  BAData(const std::string &id, const T &value)
      : BADataNode(id), value_(value) {}

  virtual ~BAData() {}

  VecXd &GetValueData() { return value_data_; }

  T GetValue() const {
    T v = value_;
    DataToValue(value_data_, v);
    return v;
  }

  bool HasPrior() const { return prior_.HasValue(); }
  VecXd GetPriorData() const {
    if (!prior_.HasValue()) {
      throw std::runtime_error(GetID() + " hasn't any prior value");
    }
    VecXd prior_data;
    ValueToData(prior_.Value(), prior_data);
    return prior_data;
  }
  void SetPrior(const T &prior) { prior_.SetValue(prior); }

  VecXd GetSigmaData() const {
    if (!sigma_.HasValue()) {
      throw std::runtime_error(GetID() + " hasn't any sigma value");
    }
    VecXd sigma_data;
    ValueToData(sigma_.Value(), sigma_data);
    return sigma_data;
  }
  void SetSigma(const T &sigma) { sigma_.SetValue(sigma); }

  void SetCovariance(const MatXd &covariance) {
    covariance_.SetValue(covariance);
  }
  bool HasCovariance() const { return covariance_.HasValue(); }
  MatXd GetCovariance() const {
    if (!HasCovariance()) {
      throw std::runtime_error(GetID() + " hasn't any covariance");
    }
    return covariance_.Value();
  }

  const std::vector<int> &GetParametersToOptimize() const {
    return parameters_to_optimize_;
  }
  void SetParametersToOptimize(const std::vector<int> &parameters) {
    parameters_to_optimize_ = parameters;
  }

  virtual void ValueToData(const T &value, VecXd &data) const = 0;
  virtual void DataToValue(const VecXd &data, T &value) const = 0;

 protected:
  VecXd value_data_;
  std::vector<int> parameters_to_optimize_;

  T value_;
  foundation::OptionalValue<T> prior_;
  foundation::OptionalValue<T> sigma_;
  foundation::OptionalValue<MatXd> covariance_;

  void Init() {
    ValueToData(value_, value_data_);
    parameters_to_optimize_.resize(value_data_.size());
    std::iota(parameters_to_optimize_.begin(), parameters_to_optimize_.end(),
              0);
  }
};

struct BACamera : public BAData<Camera> {
  BACamera(const std::string &id, const Camera &value, const Camera &prior,
           const Camera &sigma)
      : BAData<Camera>(id, value, prior, sigma) {
    Init();
  }

 private:
  void ValueToData(const Camera &value, VecXd &data) const final {
    data = value.GetParametersValues();
  }

  void DataToValue(const VecXd &data, Camera &value) const final {
    value.SetParametersValues(data);
  }
};

struct BAPose : public BAData<geometry::Pose> {
  // Outputted parametrization : CAM_TO_WORLD is preferred
  enum Parametrization {
    CAM_TO_WORLD = 0,  // x(cam) = Rt*(x(world) - t))
    WORLD_TO_CAM = 1,  // x(cam) = R*x(world) + t
  };

  BAPose(const std::string &id, const geometry::Pose &value,
         const Parametrization &parametrization = Parametrization::CAM_TO_WORLD)
      : BAData<geometry::Pose>(id, value), parametrization_(parametrization) {
    Init();
  }

  BAPose(const std::string &id, const geometry::Pose &value,
         const geometry::Pose &prior, const geometry::Pose &sigma,
         const Parametrization &parametrization = Parametrization::CAM_TO_WORLD)
      : BAData<geometry::Pose>(id, value, prior, sigma),
        parametrization_(parametrization) {
    Init();
  }

 private:
  void ValueToData(const geometry::Pose &value, VecXd &data) const final {
    data.resize(BA_SHOT_NUM_PARAMS);
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      data.segment<3>(BA_SHOT_TX) = value.TranslationCameraToWorld();
      data.segment<3>(BA_SHOT_RX) = value.RotationCameraToWorldMin();
    } else {
      data.segment<3>(BA_SHOT_TX) = value.TranslationWorldToCamera();
      data.segment<3>(BA_SHOT_RX) = value.RotationWorldToCameraMin();
    }
  }

  void DataToValue(const VecXd &data, geometry::Pose &value) const final {
    if (parametrization_ == Parametrization::CAM_TO_WORLD) {
      value.SetFromCameraToWorld(Vec3d(data.segment<3>(BA_SHOT_RX)),
                                 data.segment<3>(BA_SHOT_TX));
    } else {
      value.SetFromWorldToCamera(Vec3d(data.segment<3>(BA_SHOT_RX)),
                                 data.segment<3>(BA_SHOT_TX));
    }
  }

  Parametrization parametrization_;
};

using BARigCamera = BAPose;

struct BADataContainer : public BADataNode {
  explicit BADataContainer(const std::string &id) : BADataNode(id) {}

 protected:
  void RegisterData(const std::string &id, BADataNode *data) {
    ba_nodes_[id] = data;
  }
  BADataNode *GetData(const std::string &id) {
    const auto find_data = ba_nodes_.find(id);
    if (find_data == ba_nodes_.end()) {
      throw std::runtime_error("Data " + id +
                               " doesn't exist in BADataContainer");
    }
    return find_data->second;
  }
  const BADataNode *GetData(const std::string &id) const {
    const auto find_data = ba_nodes_.find(id);
    if (find_data == ba_nodes_.end()) {
      throw std::runtime_error("Data " + id +
                               " doesn't exist in BADataContainer");
    }
    return find_data->second;
  }

 private:
  std::unordered_map<std::string, BADataNode *> ba_nodes_;
};

struct BAShot : public BADataContainer {
  BAShot(const std::string &id, BACamera *camera, const geometry::Pose &pose)
      : BADataContainer(id),
        pose_(id, pose, geometry::Pose(), geometry::Pose()) {
    RegisterData("camera", camera);
    RegisterData("pose", &pose_);
  }
  BAShot(const std::string &id, const geometry::Pose &pose)
      : BADataContainer(id),
        pose_(id, pose, geometry::Pose(), geometry::Pose()) {
    RegisterData("pose", &pose_);
  }

  BAPose *GetPose() { return static_cast<BAPose *>(GetData("pose")); }
  const BAPose *GetPose() const {
    return static_cast<const BAPose *>(GetData("pose"));
  }
  BACamera *GetCamera() { return static_cast<BACamera *>(GetData("camera")); }
  const BACamera *GetCamera() const {
    return static_cast<const BACamera *>(GetData("camera"));
  }

 private:
  BAPose pose_;
};

struct BARigModel : public BADataContainer {
  BARigModel(
      const std::string &id,
      const std::unordered_map<std::string, geometry::Pose> &rig_cameras_poses,
      const std::unordered_map<std::string, geometry::Pose>
          &rig_cameras_poses_prior,
      const geometry::Pose &sigma)
      : BADataContainer(id) {
    for (const auto &rig_camera : rig_cameras_poses) {
      const auto rig_camera_id = rig_camera.first;
      const auto &prior = rig_cameras_poses_prior.at(rig_camera_id);
      auto &rig_camera_data =
          rig_cameras_
              .emplace(std::piecewise_construct,
                       std::forward_as_tuple(rig_camera_id),
                       std::forward_as_tuple(rig_camera_id, rig_camera.second,
                                             prior, sigma))
              .first->second;
      RegisterData(rig_camera_id, &rig_camera_data);
    }
  }
  BARigCamera *GetRigCamera(const std::string &rig_camera_id) {
    return static_cast<BARigCamera *>(GetData(rig_camera_id));
  }

  std::unordered_map<std::string, BARigCamera *> GetRigCameras() {
    std::unordered_map<std::string, BARigCamera *> cameras;
    for (const auto &c : rig_cameras_) {
      cameras[c.first] = GetRigCamera(c.first);
    }
    return cameras;
  }

 private:
  std::unordered_map<std::string, BARigCamera> rig_cameras_;
};

using BARigInstance = BAPose;

struct BARigShot : public BADataContainer {
  BARigShot(const std::string &id, BACamera *camera, BARigCamera *rig_camera,
            BARigInstance *rig_instance)
      : BADataContainer(id) {
    RegisterData("camera", camera);
    RegisterData("rig_camera", rig_camera);
    RegisterData("rig_instance", rig_instance);
  }

  BACamera *GetCamera() { return static_cast<BACamera *>(GetData("camera")); }
  BARigCamera *GetRigCamera() {
    return static_cast<BARigCamera *>(GetData("rig_camera"));
  }
  BARigInstance *GetRigInstance() {
    return static_cast<BARigInstance *>(GetData("rig_instance"));
  }
};

struct BAReconstruction {
  std::string id;
  std::map<std::string, double> scales;
  bool constant;
  bool shared;

  double *GetScalePtr(const std::string &shot) {
    if (shared) {
      return &(scales.begin()->second);
    }
    return &(scales[shot]);
  }

  double GetScale(const std::string &shot) {
    if (shared) {
      return scales.begin()->second;
    }
    return scales[shot];
  }
  void SetScale(const std::string &shot, double v) {
    if (shared) {
      scales.begin()->second = v;
    }
    scales[shot] = v;
  }
};

struct BAPointProjectionObservation {
  Vec2d coordinates;
  BAPoint *point;
  BAShot *shot;
  BACamera *camera;
  double std_deviation;
};

struct BAPointRigProjectionObservation {
  Vec2d coordinates;
  BAPoint *point;
  BARigShot *rig_shot;
  BACamera *camera;
  double std_deviation;
};

struct BARotationPrior {
  BAShot *shot;
  double rotation[3];
  double std_deviation;
};

struct BATranslationPrior {
  BAShot *shot;
  double translation[3];
  double std_deviation;
};

struct BAPositionPrior {
  BAShot *shot;
  double position[3];
  double std_deviation;
};

struct BAPointPositionPrior {
  BAPoint *point;
  double position[3];
  double std_deviation;
};

struct BARelativeMotion {
  BARelativeMotion(const std::string &reconstruction_i,
                   const std::string &shot_i,
                   const std::string &reconstruction_j,
                   const std::string &shot_j, const Vec3d &rotation,
                   const Vec3d &translation, double robust_multiplier) {
    reconstruction_id_i = reconstruction_i;
    shot_id_i = shot_i;
    reconstruction_id_j = reconstruction_j;
    shot_id_j = shot_j;
    parameters.resize(BA_SHOT_NUM_PARAMS);
    parameters.segment(BA_SHOT_RX, 3) = rotation;
    parameters.segment(BA_SHOT_TX, 3) = translation;
    scale_matrix.resize(BA_SHOT_NUM_PARAMS, BA_SHOT_NUM_PARAMS);
    scale_matrix.setIdentity();
    this->robust_multiplier = robust_multiplier;
  }

  Vec3d GetRotation() const { return parameters.segment(BA_SHOT_RX, 3); }
  Vec3d GetTranslation() const { return parameters.segment(BA_SHOT_TX, 3); }
  void SetRotation(const Vec3d &r) { parameters.segment(BA_SHOT_RX, 3) = r; }
  void SetTranslation(const Vec3d &t) { parameters.segment(BA_SHOT_TX, 3) = t; }
  void SetScaleMatrix(const MatXd &s) { scale_matrix = s; }

  std::string reconstruction_id_i;
  std::string shot_id_i;
  std::string reconstruction_id_j;
  std::string shot_id_j;
  VecXd parameters;
  MatXd scale_matrix;
  double robust_multiplier;
};

struct BARelativeSimilarity : public BARelativeMotion {
  BARelativeSimilarity(const std::string &reconstruction_i,
                       const std::string &shot_i,
                       const std::string &reconstruction_j,
                       const std::string &shot_j, const Vec3d &rotation,
                       const Vec3d &translation, double s,
                       double robust_multiplier)
      : BARelativeMotion(reconstruction_i, shot_i, reconstruction_j, shot_j,
                         rotation, translation, robust_multiplier),
        scale(s) {
    scale_matrix.resize(BA_SHOT_NUM_PARAMS + 1, BA_SHOT_NUM_PARAMS + 1);
    scale_matrix.setIdentity();
  }
  double scale;
};

struct BARelativeSimilarityCovariance {
  static const int Size = BA_SHOT_NUM_PARAMS + 1;
  std::vector<Vec3d> points;
  Eigen::Matrix<double, Size, Size> covariance;

  void AddPoint(const Vec3d &v) { points.push_back(v); }

  void Compute() {
    covariance.setZero();
    for (const auto &p : points) {
      const auto &x = p[0];
      const auto &y = p[1];
      const auto &z = p[2];
      Eigen::Matrix<double, 3, BA_SHOT_NUM_PARAMS + 1> local_jacobian;
      local_jacobian.block(0, BA_SHOT_TX, 3, 3) =
          Eigen::Matrix<double, 3, 3>::Identity();
      local_jacobian.block(0, BA_SHOT_RX, 3, 3) << 0, z, -y, -z, 0, x, y, -x, 0;
      local_jacobian.block(0, BA_SHOT_NUM_PARAMS, 3, 1) << x, y, z;
      covariance += local_jacobian.transpose() * local_jacobian;
    }
    if (covariance.determinant() < 1e-20) {
      covariance.setIdentity();
    } else {
      covariance = covariance.inverse();
    }
  }

  Eigen::Matrix<double, Size, Size> GetCovariance() const { return covariance; }
};

struct BARelativeRotation {
  BARelativeRotation(const std::string &shot_i, const std::string &shot_j,
                     const Vec3d &r) {
    shot_id_i = shot_i;
    shot_id_j = shot_j;
    rotation = r;
    scale_matrix.setIdentity();
  }
  Vec3d GetRotation() const { return rotation; }
  void SetRotation(const Vec3d &r) { rotation = r; }
  void SetScaleMatrix(const Mat3d &s) { scale_matrix = s; }

  std::string shot_id_i;
  std::string shot_id_j;
  Vec3d rotation;
  Mat3d scale_matrix;
};

struct BACommonPosition {
  BAShot *shot1;
  BAShot *shot2;
  double margin;
  double std_deviation;
};

struct BAAbsolutePosition {
  BAShot *shot;
  Vec3d position;
  double std_deviation;
  std::string std_deviation_group;
};

struct HeatmapInterpolator {
  HeatmapInterpolator(const std::vector<double> &flatten_heatmap, size_t width,
                      double resolution);
  std::vector<double> heatmap;
  size_t width;
  size_t height;
  ceres::Grid2D<double> grid;
  ceres::BiCubicInterpolator<ceres::Grid2D<double>> interpolator;
  double resolution;
};

struct BAAbsolutePositionHeatmap {
  BAShot *shot;
  std::shared_ptr<HeatmapInterpolator> heatmap;
  double x_offset;
  double y_offset;
  double height;
  double width;
  double std_deviation;
};

struct BAAbsoluteUpVector {
  BAShot *shot;
  Vec3d up_vector;
  double std_deviation;
};

struct BAAbsoluteAngle {
  BAShot *shot;
  double angle;
  double std_deviation;
};

struct BALinearMotion {
  BAShot *shot0;
  BAShot *shot1;
  BAShot *shot2;
  double alpha;
  double position_std_deviation;
  double orientation_std_deviation;
};

struct BAPointPositionShot {
  std::string shot_id;
  std::string reconstruction_id;
  std::string point_id;
  Vec3d position;
  double std_deviation;
  PositionConstraintType type;
};

struct BAPointPositionWorld {
  std::string point_id;
  Vec3d position;
  double std_deviation;
  PositionConstraintType type;
};

class BundleAdjuster {
 public:
  BundleAdjuster();
  virtual ~BundleAdjuster() = default;

  // Bundle variables

  // Basic
  void AddCamera(const std::string &id, const Camera &camera,
                 const Camera &prior, bool constant);
  void AddShot(const std::string &id, const std::string &camera,
               const Vec3d &rotation, const Vec3d &translation, bool constant);
  void AddPoint(const std::string &id, const Vec3d &position, bool constant);

  // Rigs
  void AddRigInstance(
      const std::string &rig_instance_id,
      const geometry::Pose &rig_instance_pose, const std::string &rig_model,
      const std::unordered_map<std::string, std::string> &shot_cameras,
      const std::unordered_map<std::string, std::string> &shot_rig_cameras,
      bool fixed);
  void AddRigModel(
      const std::string &rig_model,
      const std::unordered_map<std::string, geometry::Pose> &cameras_poses,
      const std::unordered_map<std::string, geometry::Pose>
          &cameras_poses_prior,
      bool fixed);
  void AddRigPositionPrior(const std::string &instance_id,
                           const Vec3d &position, double std_deviation);

  // Cluster-based
  void AddReconstruction(const std::string &id, bool constant);
  void AddReconstructionShot(const std::string &reconstruction_id, double scale,
                             const std::string &shot_id);
  void SetScaleSharing(const std::string &id, bool share);

  // Averaging constraints

  // Point projection
  void AddPointProjectionObservation(const std::string &shot,
                                     const std::string &point,
                                     const Vec2d &observation,
                                     double std_deviation);
  void AddRotationPrior(const std::string &shot_id, double rx, double ry,
                        double rz, double std_deviation);
  void AddTranslationPrior(const std::string &shot_id, double tx, double ty,
                           double tz, double std_deviation);
  void AddPositionPrior(const std::string &shot_id, double x, double y,
                        double z, double std_deviation);
  void AddPointPositionPrior(const std::string &point_id, double x, double y,
                             double z, double std_deviation);

  void SetOriginShot(const std::string &shot_id);
  void SetUnitTranslationShot(const std::string &shot_id);

  // Relative motion ones
  void AddRelativeMotion(const BARelativeMotion &rm);
  void AddRelativeSimilarity(const BARelativeSimilarity &rm);
  void AddRelativeRotation(const BARelativeRotation &rr);

  // Absolute motion ones
  void AddCommonPosition(const std::string &shot_id1,
                         const std::string &shot_id2, double margin,
                         double std_deviation);
  void AddAbsolutePosition(const std::string &shot_id, const Vec3d &position,
                           double std_deviation,
                           const std::string &std_deviation_group);

  void AddHeatmap(const std::string &heatmap_id,
                  const std::vector<double> &in_heatmap, size_t in_width,
                  double resolution);

  void AddAbsolutePositionHeatmap(const std::string &shot_id,
                                  const std::string &heatmap_id,
                                  double x_offset, double y_offset,
                                  double std_deviation);

  void AddAbsoluteUpVector(const std::string &shot_id, const Vec3d &up_vector,
                           double std_deviation);
  void AddAbsolutePan(const std::string &shot_id, double angle,
                      double std_deviation);
  void AddAbsoluteTilt(const std::string &shot_id, double angle,
                       double std_deviation);
  void AddAbsoluteRoll(const std::string &shot_id, double angle,
                       double std_deviation);

  // Motion priors
  void AddLinearMotion(const std::string &shot0_id, const std::string &shot1_id,
                       const std::string &shot2_id, double alpha,
                       double position_std_deviation,
                       double orientation_std_deviation);

  // Point positions
  void AddPointPositionShot(const std::string &point_id,
                            const std::string &shot_id,
                            const std::string &reconstruction_id,
                            const Vec3d &position, double std_deviation,
                            const PositionConstraintType &type);
  void AddPointPositionWorld(const std::string &point_id, const Vec3d &position,
                             double std_deviation,
                             const PositionConstraintType &type);

  // Minimization setup
  void SetPointProjectionLossFunction(std::string name, double threshold);
  void SetRelativeMotionLossFunction(std::string name, double threshold);
  void SetAdjustAbsolutePositionStd(bool adjust);

  void SetMaxNumIterations(int miter);
  void SetNumThreads(int n);
  void SetUseAnalyticDerivatives(bool use);
  void SetLinearSolverType(std::string t);

  void SetInternalParametersPriorSD(double focal_sd, double c_sd, double k1_sd,
                                    double k2_sd, double p1_sd, double p2_sd,
                                    double k3_sd, double k4_sd);
  void SetRigParametersPriorSD(double rig_translation_sd,
                               double rig_rotation_sd);

  void SetComputeCovariances(bool v);
  bool GetCovarianceEstimationValid();
  void SetComputeReprojectionErrors(bool v);

  // Minimization
  void Run();
  void ComputeCovariances(ceres::Problem *problem);
  void ComputeReprojectionErrors();

  // Getters
  Camera GetCamera(const std::string &id);
  BAShot GetShot(const std::string &id);
  BAReconstruction GetReconstruction(const std::string &id);
  BAPoint GetPoint(const std::string &id);
  BARigModel GetRigModel(const std::string &model_id);
  BARigInstance GetRigInstance(const std::string &instance_id);

  // Minimization details
  std::string BriefReport();
  std::string FullReport();

 private:
  // default sigmas
  Camera GetDefaultCameraSigma(const Camera &camera) const;
  geometry::Pose GetDefaultRigPoseSigma() const;

  // minimized data
  std::map<std::string, BACamera> cameras_;
  std::map<std::string, BAShot> shots_;
  std::map<std::string, BARigShot> rig_shots_;
  std::map<std::string, BAReconstruction> reconstructions_;
  std::map<std::string, BAPoint> points_;
  std::map<std::string, BARigModel> rig_models_;
  std::map<std::string, BARigInstance> rig_instances_;

  bool use_analytic_{false};

  // minimization constraints

  // reprojection observation
  std::vector<BAPointProjectionObservation> point_projection_observations_;
  std::vector<BAPointRigProjectionObservation>
      point_rig_projection_observations_;
  std::map<std::string, std::shared_ptr<HeatmapInterpolator>> heatmaps_;

  // relative motion between shots
  std::vector<BARelativeMotion> relative_motions_;
  std::vector<BARelativeSimilarity> relative_similarity_;
  std::vector<BARelativeRotation> relative_rotations_;
  std::vector<BACommonPosition> common_positions_;

  // shots absolute positions
  std::vector<BAAbsolutePositionHeatmap> absolute_positions_heatmaps_;
  std::vector<BAAbsolutePosition> absolute_positions_;
  std::vector<BAAbsoluteUpVector> absolute_up_vectors_;
  std::vector<BAAbsoluteAngle> absolute_pans_;
  std::vector<BAAbsoluteAngle> absolute_tilts_;
  std::vector<BAAbsoluteAngle> absolute_rolls_;

  std::vector<BARotationPrior> rotation_priors_;
  std::vector<BATranslationPrior> translation_priors_;
  std::vector<BAPositionPrior> position_priors_;
  std::vector<BAPointPositionPrior> point_position_priors_;

  BAShot *unit_translation_shot_;

  // motion priors
  std::vector<BALinearMotion> linear_motion_prior_;

  // points absolute constraints
  std::vector<BAPointPositionShot> point_positions_shot_;
  std::vector<BAPointPositionWorld> point_positions_world_;

  // Camera parameters prior
  double focal_prior_sd_;
  double c_prior_sd_;
  double k1_sd_;
  double k2_sd_;
  double p1_sd_;
  double p2_sd_;
  double k3_sd_;
  double k4_sd_;

  // Rig model extrinsics prior
  double rig_translation_sd_{1.0};
  double rig_rotation_sd_{1.0};

  // minimization setup
  std::string point_projection_loss_name_;
  double point_projection_loss_threshold_;
  std::string relative_motion_loss_name_;
  double relative_motion_loss_threshold_;
  bool adjust_absolute_position_std_;

  bool compute_covariances_;
  bool covariance_estimation_valid_;
  bool compute_reprojection_errors_;

  int max_num_iterations_;
  int num_threads_;
  std::string linear_solver_type_;

  // internal
  ceres::Solver::Summary last_run_summary_;
};
