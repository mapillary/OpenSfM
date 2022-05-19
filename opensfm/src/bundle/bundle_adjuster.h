#pragma once

#include <bundle/data/bias.h>
#include <bundle/data/camera.h>
#include <bundle/data/data.h>
#include <bundle/data/point.h>
#include <bundle/data/pose.h>
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

struct Reconstruction {
  std::string id;
  std::map<std::string, double> scales;
  bool constant;
  bool shared;

  double *GetScalePtr(const std::string &shot) {
    if (shared) {
      return &(scales.begin()->second);
    }
    return &(scales.at(shot));
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

struct RelativeMotion {
  RelativeMotion(const std::string &rig_instance_i,
                 const std::string &rig_instance_j, const Vec3d &rotation,
                 const Vec3d &translation, double scale,
                 double robust_multiplier, bool observed_scale) {
    rig_instance_id_i = rig_instance_i;
    rig_instance_id_j = rig_instance_j;

    const int num_parameters = Similarity::Parameter::NUM_PARAMS;
    parameters.resize(num_parameters);
    scale_matrix.resize(num_parameters, num_parameters);

    parameters.segment(Similarity::Parameter::RX, 3) = rotation;
    parameters.segment(Similarity::Parameter::TX, 3) = translation;
    parameters(Similarity::Parameter::SCALE) = scale;
    scale_matrix.setIdentity();
    this->robust_multiplier = robust_multiplier;
    this->observed_scale = observed_scale;
  }

  void SetScaleMatrix(const MatXd &s) { scale_matrix = s; }

  std::string rig_instance_id_i;
  std::string rig_instance_id_j;

  VecXd parameters;
  MatXd scale_matrix;
  double robust_multiplier;
  bool observed_scale;
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
  std::string shot_i;
  std::string shot_j;
  double margin;
  double std_deviation;
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
  std::string shot_id;
  std::shared_ptr<HeatmapInterpolator> heatmap;
  double x_offset;
  double y_offset;
  double height;
  double width;
  double std_deviation;
};

struct AbsoluteUpVector {
  std::string shot_id;
  Vec3d up_vector;
  double std_deviation;
};

struct AbsoluteAngle {
  std::string shot_id;
  double angle;
  double std_deviation;
};

struct LinearMotion {
  std::string shot0;
  std::string shot1;
  std::string shot2;
  double alpha;
  double position_std_deviation;
  double orientation_std_deviation;
};

class BundleAdjuster {
 public:
  BundleAdjuster();
  virtual ~BundleAdjuster() = default;

  // Bundle variables

  // Basic
  void AddCamera(const std::string &id, const geometry::Camera &camera,
                 const geometry::Camera &prior, bool constant);
  void AddPoint(const std::string &id, const Vec3d &position, bool constant);
  void AddPointPrior(const std::string &id, const Vec3d &position,
                     const Vec3d &std_deviation, bool has_altitude_prior);
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
  void AddRigInstancePositionPrior(const std::string &instance_id,
                                   const Vec3d &position,
                                   const Vec3d &std_deviation,
                                   const std::string &scale_group);

  // Cluster-SfM related
  void AddReconstruction(const std::string &id, bool constant);
  void AddReconstructionInstance(const std::string &reconstruction_id,
                                 double scale, const std::string &instance_id);
  void SetScaleSharing(const std::string &id, bool share);

  // Real bundle adjustment : point projections
  void AddPointProjectionObservation(const std::string &shot,
                                     const std::string &point,
                                     const Vec2d &observation,
                                     double std_deviation);

  // Relative motion constraints
  void AddRelativeMotion(const RelativeMotion &rm);
  void AddRelativeRotation(const RelativeRotation &rr);

  // Absolute motion constraints
  void AddCommonPosition(const std::string &shot_id1,
                         const std::string &shot_id2, double margin,
                         double std_deviation);
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

  // Gauge fixing
  void SetGaugeFixShots(const std::string &shot_origin,
                        const std::string &shot_scale);

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
  int GetProjectionsCount() const;
  int GetRelativeMotionsCount() const;
  geometry::Camera GetCamera(const std::string &id) const;
  geometry::Similarity GetBias(const std::string &id) const;
  Reconstruction GetReconstruction(const std::string &reconstruction_id) const;
  Point GetPoint(const std::string &id) const;
  bool HasPoint(const std::string &id) const;
  RigCamera GetRigCamera(const std::string &rig_camera_id) const;
  RigInstance GetRigInstance(const std::string &instance_id) const;
  std::map<std::string, RigCamera> GetRigCameras() const;
  std::map<std::string, RigInstance> GetRigInstances() const;

  // Minimization details
  std::string BriefReport() const;
  std::string FullReport() const;

 private:
  // default sigmas
  geometry::Camera GetDefaultCameraSigma(const geometry::Camera &camera) const;
  geometry::Pose GetDefaultRigPoseSigma() const;

  // minimized data
  std::map<std::string, Camera> cameras_;
  std::map<std::string, Similarity> bias_;
  std::map<std::string, Shot> shots_;
  std::map<std::string, Reconstruction> reconstructions_;
  std::map<std::string, std::string> reconstructions_assignments_;
  std::map<std::string, Point> points_;
  std::map<std::string, RigCamera> rig_cameras_;
  std::map<std::string, RigInstance> rig_instances_;

  bool use_analytic_{false};

  // minimization constraints

  // reprojection observation
  std::vector<PointProjectionObservation> point_projection_observations_;
  std::map<std::string, std::shared_ptr<HeatmapInterpolator>> heatmaps_;

  // relative motion between shots
  std::vector<RelativeMotion> relative_motions_;
  std::vector<RelativeRotation> relative_rotations_;
  std::vector<CommonPosition> common_positions_;

  // shots absolute positions
  std::vector<AbsolutePositionHeatmap> absolute_positions_heatmaps_;
  std::vector<AbsoluteUpVector> absolute_up_vectors_;
  std::vector<AbsoluteAngle> absolute_pans_;
  std::vector<AbsoluteAngle> absolute_tilts_;
  std::vector<AbsoluteAngle> absolute_rolls_;

  // motion priors
  std::vector<LinearMotion> linear_motion_prior_;

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
  foundation::OptionalValue<std::pair<std::string, std::string>>
      gauge_fix_shots_;

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
