#pragma once

#include <bundle/data/bias.h>
#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/pose.h>
#include <bundle/data/rig.h>
#include <bundle/data/shot.h>
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

namespace bundle {
enum PositionConstraintType {
  X = 0x1,
  Y = 0x2,
  Z = 0x4,
  XY = X | Y,
  XYZ = XY | Z
};

struct Point {
  std::string id;
  Eigen::Matrix<double, 3, 1> parameters;
  bool constant;
  std::map<std::string, VecXd> reprojection_errors;

  Vec3d GetPoint() const { return parameters; }
  void SetPoint(const Vec3d &p) { parameters = p; }
};

struct Reconstruction {
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

  double GetScale(const std::string &shot) const {
    if (shared) {
      return scales.begin()->second;
    }
    return scales.at(shot);
  }
  void SetScale(const std::string &shot, double v) {
    if (shared) {
      scales.begin()->second = v;
    }
    scales[shot] = v;
  }
};

struct PointProjectionObservation {
  Vec2d coordinates;
  Point *point;
  Shot *shot;
  Camera *camera;
  double std_deviation;
};

struct PointRigProjectionObservation {
  Vec2d coordinates;
  Point *point;
  RigShot *rig_shot;
  Camera *camera;
  double std_deviation;
};

struct RotationPrior {
  Shot *shot;
  double rotation[3];
  double std_deviation;
};

struct TranslationPrior {
  Shot *shot;
  double translation[3];
  double std_deviation;
};

struct PositionPrior {
  Shot *shot;
  Bias *bias;
  double position[3];
  double std_deviation;
};

struct PointPositionPrior {
  Point *point;
  double position[3];
  double std_deviation;
};

struct RelativeMotion {
  RelativeMotion(const std::string &reconstruction_i, const std::string &shot_i,
                 const std::string &reconstruction_j, const std::string &shot_j,
                 const Vec3d &rotation, const Vec3d &translation,
                 double robust_multiplier) {
    reconstruction_id_i = reconstruction_i;
    shot_id_i = shot_i;
    reconstruction_id_j = reconstruction_j;
    shot_id_j = shot_j;
    parameters.resize(Pose::Parameter::NUM_PARAMS);
    parameters.segment(Pose::Parameter::RX, 3) = rotation;
    parameters.segment(Pose::Parameter::TX, 3) = translation;
    scale_matrix.resize(Pose::Parameter::NUM_PARAMS,
                        Pose::Parameter::NUM_PARAMS);
    scale_matrix.setIdentity();
    this->robust_multiplier = robust_multiplier;
  }

  Vec3d GetRotation() const {
    return parameters.segment(Pose::Parameter::RX, 3);
  }
  Vec3d GetTranslation() const {
    return parameters.segment(Pose::Parameter::TX, 3);
  }
  void SetRotation(const Vec3d &r) {
    parameters.segment(Pose::Parameter::RX, 3) = r;
  }
  void SetTranslation(const Vec3d &t) {
    parameters.segment(Pose::Parameter::TX, 3) = t;
  }
  void SetScaleMatrix(const MatXd &s) { scale_matrix = s; }

  std::string reconstruction_id_i;
  std::string shot_id_i;
  std::string reconstruction_id_j;
  std::string shot_id_j;
  VecXd parameters;
  MatXd scale_matrix;
  double robust_multiplier;
};

struct RelativeSimilarity : public RelativeMotion {
  RelativeSimilarity(const std::string &reconstruction_i,
                     const std::string &shot_i,
                     const std::string &reconstruction_j,
                     const std::string &shot_j, const Vec3d &rotation,
                     const Vec3d &translation, double s,
                     double robust_multiplier)
      : RelativeMotion(reconstruction_i, shot_i, reconstruction_j, shot_j,
                       rotation, translation, robust_multiplier),
        scale(s) {
    scale_matrix.resize(Pose::Parameter::NUM_PARAMS + 1,
                        Pose::Parameter::NUM_PARAMS + 1);
    scale_matrix.setIdentity();
  }
  double scale;
};

struct RelativeSimilarityCovariance {
  static const int Size = Pose::Parameter::NUM_PARAMS + 1;
  std::vector<Vec3d> points;
  Eigen::Matrix<double, Size, Size> covariance;

  void AddPoint(const Vec3d &v) { points.push_back(v); }

  void Compute() {
    covariance.setZero();
    for (const auto &p : points) {
      const auto &x = p[0];
      const auto &y = p[1];
      const auto &z = p[2];
      Eigen::Matrix<double, 3, Pose::Parameter::NUM_PARAMS + 1> local_jacobian;
      local_jacobian.block(0, Pose::Parameter::TX, 3, 3) =
          Eigen::Matrix<double, 3, 3>::Identity();
      local_jacobian.block(0, Pose::Parameter::RX, 3, 3) << 0, z, -y, -z, 0, x,
          y, -x, 0;
      local_jacobian.block(0, Pose::Parameter::NUM_PARAMS, 3, 1) << x, y, z;
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

struct RelativeRotation {
  RelativeRotation(const std::string &shot_i, const std::string &shot_j,
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

struct CommonPosition {
  Shot *shot1;
  Shot *shot2;
  double margin;
  double std_deviation;
};

struct AbsolutePosition {
  Shot *shot;
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

struct AbsolutePositionHeatmap {
  Shot *shot;
  std::shared_ptr<HeatmapInterpolator> heatmap;
  double x_offset;
  double y_offset;
  double height;
  double width;
  double std_deviation;
};

struct AbsoluteUpVector {
  Shot *shot;
  Vec3d up_vector;
  double std_deviation;
};

struct AbsoluteAngle {
  Shot *shot;
  double angle;
  double std_deviation;
};

struct LinearMotion {
  Shot *shot0;
  Shot *shot1;
  Shot *shot2;
  double alpha;
  double position_std_deviation;
  double orientation_std_deviation;
};

struct PointPositionShot {
  std::string shot_id;
  std::string reconstruction_id;
  std::string point_id;
  Vec3d position;
  double std_deviation;
  PositionConstraintType type;
};

struct PointPositionWorld {
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
  void AddCamera(const std::string &id, const geometry::Camera &camera,
                 const geometry::Camera &prior, bool constant);
  void AddShot(const std::string &id, const std::string &camera,
               const Vec3d &rotation, const Vec3d &translation, bool constant);
  void AddPoint(const std::string &id, const Vec3d &position, bool constant);
  void SetCameraBias(const std::string &id, const geometry::Similarity &bias);

  // Rigs
  void AddRigInstance(
      const std::string &rig_instance_id,
      const geometry::Pose &rig_instance_pose,
      const std::unordered_map<std::string, std::string> &shot_cameras,
      const std::unordered_map<std::string, std::string> &shot_rig_cameras,
      bool fixed);
  void AddRigCamera(const std::string &rig_camera, const geometry::Pose &pose,
                    const geometry::Pose &pose_prior, bool fixed);
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
  void AddRelativeMotion(const RelativeMotion &rm);
  void AddRelativeSimilarity(const RelativeSimilarity &rm);
  void AddRelativeRotation(const RelativeRotation &rr);

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
  void SetCovarianceAlgorithmType(std::string t);

  void SetInternalParametersPriorSD(double focal_sd, double c_sd, double k1_sd,
                                    double k2_sd, double p1_sd, double p2_sd,
                                    double k3_sd, double k4_sd);
  void SetRigParametersPriorSD(double rig_translation_sd,
                               double rig_rotation_sd);

  void SetComputeCovariances(bool v);
  bool GetCovarianceEstimationValid() const;
  void SetComputeReprojectionErrors(bool v);

  // Minimization
  void Run();
  void ComputeCovariances(ceres::Problem *problem);
  void ComputeReprojectionErrors();

  // Getters
  geometry::Camera GetCamera(const std::string &id) const;
  geometry::Similarity GetBias(const std::string &id) const;
  Shot GetShot(const std::string &id) const;
  Reconstruction GetReconstruction(const std::string &id) const;
  Point GetPoint(const std::string &id) const;
  RigCamera GetRigCamera(const std::string &rig_camera_id) const;
  RigInstance GetRigInstance(const std::string &instance_id) const;

  // Minimization details
  std::string BriefReport() const;
  std::string FullReport() const;

 private:
  // default sigmas
  geometry::Camera GetDefaultCameraSigma(const geometry::Camera &camera) const;
  geometry::Pose GetDefaultRigPoseSigma() const;

  // minimized data
  std::map<std::string, Camera> cameras_;
  std::map<std::string, Bias> bias_;
  std::map<std::string, Shot> shots_;
  std::map<std::string, RigShot> rig_shots_;
  std::map<std::string, Reconstruction> reconstructions_;
  std::map<std::string, Point> points_;
  std::map<std::string, RigCamera> rig_cameras_;
  std::map<std::string, RigInstance> rig_instances_;

  bool use_analytic_{false};

  // minimization constraints

  // reprojection observation
  std::vector<PointProjectionObservation> point_projection_observations_;
  std::vector<PointRigProjectionObservation> point_rig_projection_observations_;
  std::map<std::string, std::shared_ptr<HeatmapInterpolator>> heatmaps_;

  // relative motion between shots
  std::vector<RelativeMotion> relative_motions_;
  std::vector<RelativeSimilarity> relative_similarity_;
  std::vector<RelativeRotation> relative_rotations_;
  std::vector<CommonPosition> common_positions_;

  // shots absolute positions
  std::vector<AbsolutePositionHeatmap> absolute_positions_heatmaps_;
  std::vector<AbsolutePosition> absolute_positions_;
  std::vector<AbsoluteUpVector> absolute_up_vectors_;
  std::vector<AbsoluteAngle> absolute_pans_;
  std::vector<AbsoluteAngle> absolute_tilts_;
  std::vector<AbsoluteAngle> absolute_rolls_;

  std::vector<RotationPrior> rotation_priors_;
  std::vector<TranslationPrior> translation_priors_;
  std::vector<PositionPrior> position_priors_;
  std::vector<PointPositionPrior> point_position_priors_;

  Shot *unit_translation_shot_;

  // motion priors
  std::vector<LinearMotion> linear_motion_prior_;

  // points absolute constraints
  std::vector<PointPositionShot> point_positions_shot_;
  std::vector<PointPositionWorld> point_positions_world_;

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
  std::string covariance_algorithm_type_;

  // internal
  ceres::Solver::Summary last_run_summary_;
};
}  // namespace bundle
