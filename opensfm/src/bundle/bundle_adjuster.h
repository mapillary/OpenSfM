#pragma once

#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>

#include <geometry/camera.h>

#include "ceres/ceres.h"
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

struct BAShot {
  std::string id;
  std::string camera;
  Eigen::Matrix<double, BA_SHOT_NUM_PARAMS, 1> parameters;
  double covariance[BA_SHOT_NUM_PARAMS * BA_SHOT_NUM_PARAMS];
  bool constant;

  Vec3d GetRotation() const {
    Vec3d r;
    Vec3d t;
    InvertTransform_(&parameters[BA_SHOT_RX], &parameters[BA_SHOT_TX], &r[0], &t[0]);
    return r;
  }
  Vec3d GetTranslation() const {
    Vec3d r;
    Vec3d t;
    InvertTransform_(&parameters[BA_SHOT_RX], &parameters[BA_SHOT_TX], &r[0], &t[0]);
    return t;
  }
  double GetCovarianceInvParam(int i, int j) { return covariance[i * BA_SHOT_NUM_PARAMS + j]; }

  void SetRotationAndTranslation(const Vec3d &r, const Vec3d &t) {
    InvertTransform_(&r[0], &t[0], &parameters[BA_SHOT_RX], &parameters[BA_SHOT_TX]);
  }

  void InvertTransform_(const double *r, const double *t, double *rinv, double *tinv) const {
    // Rinv = R^t  tinv = -R^t * t
    rinv[0] = -r[0];
    rinv[1] = -r[1];
    rinv[2] = -r[2];
    ceres::AngleAxisRotatePoint(rinv, t, tinv);
    tinv[0] = -tinv[0];
    tinv[1] = -tinv[1];
    tinv[2] = -tinv[2];
  }
};

struct BAPoint {
  std::string id;
  Eigen::Matrix<double, 3, 1> parameters;
  bool constant;
  std::map<std::string, VecXd> reprojection_errors;

  Vec3d GetPoint() const {return parameters;}
  void SetPoint(const Vec3d &p) {parameters = p;}
};

struct BAReconstruction {
  std::string id;
  std::map<std::string, double > scales;
  std::map<std::string, BAShot *> shots;
  bool constant;
  bool shared;

  double* GetScalePtr(const std::string& shot) {
    if (shared) {
      return &(scales.begin()->second);
    }
    return &(scales[shot]);
  }

  double GetScale(const std::string& shot) {
    if (shared) {
      return scales.begin()->second;
    }
    return scales[shot];
  }
  void SetScale(const std::string& shot, double v) 
  { 
    if (shared) {
      scales.begin()->second = v;
    }
    scales[shot] = v; 
  }
};

template <class T>
struct BAData {
 public:
  using ValueType = T;

  BAData(const T &value, const T &prior, const T &sigma)
      : value_(value), prior_(prior), sigma_(sigma) {}

  VecXd &GetValueData() {
    ValueToData(value_, value_data_);
    return value_data_;
  }

  Camera GetValue() {
    Camera v = value_;
    DataToValue(value_data_, v);
    return v;
  }

  VecXd GetPriorData() const {
    VecXd prior_data;
    ValueToData(prior_, prior_data);
    return prior_data;
  }

  VecXd GetSigmaData() const {
    VecXd sigma_data;
    ValueToData(sigma_, sigma_data);
    return sigma_data;
  }
  void SetSigma(const T &sigma) { sigma_ = sigma; }

  virtual void ValueToData(const T &value, VecXd &data) const = 0;
  virtual void DataToValue(const VecXd &data, T &value) const = 0;

 protected:
  VecXd value_data_;

  T value_;
  T prior_;
  T sigma_;
};

struct BACamera : public BAData<Camera> {
  BACamera(const Camera &value, const Camera &prior, const Camera &sigma)
      : BAData<Camera>(value, prior, sigma),
        all_parameters_(value.GetParametersTypes()),
        parameters_to_optimize_(value.GetParametersTypes()) {}

  std::vector<Camera::Parameters> GetParametersToOptimize() {
    return parameters_to_optimize_;
  }

  void SetParametersToOptimize(const std::vector<Camera::Parameters> &p) {
    parameters_to_optimize_ = p;
  }

 private:
  void ValueToData(const Camera &value, VecXd &data) const final {
    if (data.size() == 0) {
      data = value.GetParametersValues();
    }
  }

  void DataToValue(const VecXd &data, Camera &value) const final {
    if (data.size() > 0) {
      int count = 0;
      for (const auto t : all_parameters_) {
        value.SetParameterValue(t, data(count++));
      }
    }
  }

  std::vector<Camera::Parameters> parameters_to_optimize_;
  std::vector<Camera::Parameters> all_parameters_;
};

struct BAPointProjectionObservation {
  Vec2d coordinates;
  BACamera *camera;
  BAShot *shot;
  BAPoint *point;
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
                   const std::string &shot_j,
                   const Vec3d &rotation,
                   const Vec3d &translation,
                   double robust_multiplier) {
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

  Vec3d GetRotation() const {return parameters.segment(BA_SHOT_RX, 3);}
  Vec3d GetTranslation() const {return parameters.segment(BA_SHOT_TX, 3);}
  void SetRotation(const Vec3d &r) {parameters.segment(BA_SHOT_RX, 3) = r;}
  void SetTranslation(const Vec3d &t) {parameters.segment(BA_SHOT_TX, 3) = t;}
  void SetScaleMatrix(const MatXd& s) {scale_matrix = s;}
  
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
                       const std::string &shot_j,
                       const Vec3d &rotation,
                       const Vec3d &translation,
                       double s, double robust_multiplier)
      : BARelativeMotion(reconstruction_i, shot_i, 
                         reconstruction_j, shot_j,
                         rotation, translation,
                         robust_multiplier),
        scale(s) {
    scale_matrix.resize(BA_SHOT_NUM_PARAMS + 1, BA_SHOT_NUM_PARAMS + 1);
    scale_matrix.setIdentity();
  }
  double scale;
};

struct BARelativeSimilarityCovariance
{
  static const int Size = BA_SHOT_NUM_PARAMS+1;
  std::vector<Vec3d> points;
  Eigen::Matrix<double, Size, Size> covariance;

  void AddPoint(const Vec3d& v){points.push_back(v);}

  void Compute(){
    covariance.setZero();
    for(const auto& p : points){
      const auto& x = p[0];
      const auto& y = p[1];
      const auto& z = p[2];
      Eigen::Matrix<double, 3, BA_SHOT_NUM_PARAMS+1> local_jacobian;
      local_jacobian.block(0, BA_SHOT_TX, 3, 3) = Eigen::Matrix<double,3,3>::Identity();
      local_jacobian.block(0, BA_SHOT_RX, 3, 3) <<  0, z, -y, 
                                                    -z, 0, x,
                                                     y, -x, 0;
      local_jacobian.block(0, BA_SHOT_NUM_PARAMS, 3, 1) << x, y, z;
      covariance += local_jacobian.transpose()*local_jacobian;
    }
    if(covariance.determinant() < 1e-20){
      covariance.setIdentity();
    }
    else{
      covariance = covariance.inverse();
    }    
  }

  Eigen::Matrix<double, Size, Size> GetCovariance()const{
    return covariance;
  }
};

struct BARelativeRotation {
  BARelativeRotation(const std::string &shot_i,
                     const std::string &shot_j,
                     const Vec3d &r) {
    shot_id_i = shot_i;
    shot_id_j = shot_j;
    rotation = r;
    scale_matrix.setIdentity();
  }
  Vec3d GetRotation() const {return rotation;}
  void SetRotation(const Vec3d& r) {rotation = r;}
  void SetScaleMatrix(const Mat3d& s) {scale_matrix = s;}

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

struct BAAbsoluteUpVector {
  BAShot *shot;
  Vec3d up_vector;
  double std_deviation;
};

struct BAAbsoluteAngle{
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

  void AddCamera(const std::string &id, const Camera& camera, const Camera& prior, bool constant);
  void UpdateSigmas();
  void AddShot(
      const std::string &id,
      const std::string &camera,
      const Vec3d& rotation,
      const Vec3d& translation,
      bool constant);
  void AddReconstruction(
      const std::string &id,
      bool constant);
  void AddReconstructionShot(const std::string& reconstruction_id, double scale,
                             const std::string& shot_id);
  void SetScaleSharing(const std::string &id, bool share);
  void AddPoint(const std::string &id, 
                const Vec3d& position,
                bool constant);

  // averaging constraints

  // point projection
  void AddPointProjectionObservation(
      const std::string &shot,
      const std::string &point,
      double x,
      double y,
      double std_deviation);
  void AddRotationPrior(
      const std::string &shot_id,
      double rx,
      double ry,
      double rz,
      double std_deviation);
  void AddTranslationPrior(
      const std::string &shot_id,
      double tx,
      double ty,
      double tz,
      double std_deviation);
  void AddPositionPrior(
      const std::string &shot_id,
      double x,
      double y,
      double z,
      double std_deviation);
  void AddPointPositionPrior(
      const std::string &point_id,
      double x,
      double y,
      double z,
      double std_deviation);

  void SetOriginShot(const std::string &shot_id);
  void SetUnitTranslationShot(const std::string &shot_id);

  // relative motion ones
  void AddRelativeMotion(const BARelativeMotion& rm);
  void AddRelativeSimilarity(const BARelativeSimilarity& rm);
  void AddRelativeRotation(const BARelativeRotation &rr);

  // absolute motion ones
  void AddCommonPosition(
      const std::string &shot_id1,
      const std::string &shot_id2,
      double margin,
      double std_deviation);
  void AddAbsolutePosition(
      const std::string &shot_id,
      const Vec3d& position,
      double std_deviation,
      const std::string& std_deviation_group);
  void AddAbsoluteUpVector(
      const std::string &shot_id,
      const Vec3d& up_vector,
      double std_deviation);
  void AddAbsolutePan(
      const std::string &shot_id,
      double angle,
      double std_deviation);
  void AddAbsoluteTilt(
      const std::string &shot_id,
      double angle,
      double std_deviation);
  void AddAbsoluteRoll(
      const std::string &shot_id,
      double angle,
      double std_deviation);

  // motion priors
  void AddLinearMotion(
      const std::string &shot0_id,
      const std::string &shot1_id,
      const std::string &shot2_id,
      double alpha,
      double position_std_deviation,
      double orientation_std_deviation);

  // point positions
  void AddPointPositionShot(const std::string &point_id,
                            const std::string &shot_id,
                            const std::string &reconstruction_id,
                            const Vec3d& position,
                            double std_deviation,
                            const PositionConstraintType& type);
  void AddPointPositionWorld(const std::string &point_id,
                             const Vec3d& position,
                             double std_deviation,
                             const PositionConstraintType& type);

  // minimization setup
  void SetPointProjectionLossFunction(std::string name, double threshold);
  void SetRelativeMotionLossFunction(std::string name, double threshold);
  void SetAdjustAbsolutePositionStd(bool adjust);

  void SetMaxNumIterations(int miter);
  void SetNumThreads(int n);
  void SetUseAnalyticDerivatives(bool use);
  void SetLinearSolverType(std::string t);

  void SetInternalParametersPriorSD(
      double focal_sd,
      double c_sd,
      double k1_sd,
      double k2_sd,
      double p1_sd,
      double p2_sd,
      double k3_sd,
      double k4_sd);

  void SetComputeCovariances(bool v);
  bool GetCovarianceEstimationValid();
  void SetComputeReprojectionErrors(bool v);

  // minimization
  void Run();
  void AddObservationResidualBlock(
      const BAPointProjectionObservation &observation,
      ceres::LossFunction *loss,
      ceres::Problem *problem);
  void ComputeCovariances(ceres::Problem *problem);
  void ComputeReprojectionErrors();

  // getters
  Camera GetCamera(const std::string &id);
  BAShot GetShot(const std::string &id);
  BAReconstruction GetReconstruction(const std::string &id);
  BAPoint GetPoint(const std::string &id);

  // minimization details
  std::string BriefReport();
  std::string FullReport();

 private:
  // minimized data
  std::map<std::string, BACamera> cameras_;
  std::map<std::string, BAShot> shots_;
  std::map<std::string, BAReconstruction> reconstructions_;
  std::map<std::string, BAPoint> points_;

  
  bool use_analytic_{false};

  // minimization constraints

  // reprojection observation
  std::vector<BAPointProjectionObservation> point_projection_observations_;

  // relative motion between shots
  std::vector<BARelativeMotion> relative_motions_;
  std::vector<BARelativeSimilarity> relative_similarity_;
  std::vector<BARelativeRotation> relative_rotations_;
  std::vector<BACommonPosition> common_positions_;

  // shots absolute positions
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


