#include <cmath>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <map>
#include <string>

extern "C" {
#include <string.h>
}

#include "ceres/ceres.h"
#include "ceres/rotation.h"
#include "ceres/loss_function.h"
#include "ceres/covariance.h"



enum {
  BA_CAMERA_FOCAL,
  BA_CAMERA_K1,
  BA_CAMERA_K2,
  BA_CAMERA_NUM_PARAMS
};

struct BACamera {
  double parameters[BA_CAMERA_NUM_PARAMS];
  bool constant;
  double focal_prior;
  std::string id;

  double GetFocal() { return parameters[BA_CAMERA_FOCAL]; }
  double GetK1() { return parameters[BA_CAMERA_K1]; }
  double GetK2() { return parameters[BA_CAMERA_K2]; }
  void SetFocal(double v) { parameters[BA_CAMERA_FOCAL] = v; }
  void SetK1(double v) { parameters[BA_CAMERA_K1] = v; }
  void SetK2(double v) { parameters[BA_CAMERA_K2] = v; }
};

enum {
  BA_SHOT_RX,
  BA_SHOT_RY,
  BA_SHOT_RZ,
  BA_SHOT_TX,
  BA_SHOT_TY,
  BA_SHOT_TZ,
  BA_SHOT_NUM_PARAMS
};

struct BAShot {
  double parameters[BA_SHOT_NUM_PARAMS];
  double covariance[BA_SHOT_NUM_PARAMS * BA_SHOT_NUM_PARAMS];
  bool constant;
  double gps_x, gps_y, gps_z;
  double gps_dop;
  int exif_orientation;
  std::string camera;
  std::string id;

  double GetRX() { return parameters[BA_SHOT_RX]; }
  double GetRY() { return parameters[BA_SHOT_RY]; }
  double GetRZ() { return parameters[BA_SHOT_RZ]; }
  double GetTX() { return parameters[BA_SHOT_TX]; }
  double GetTY() { return parameters[BA_SHOT_TY]; }
  double GetTZ() { return parameters[BA_SHOT_TZ]; }
  void SetRX(double v) { parameters[BA_SHOT_RX] = v; }
  void SetRY(double v) { parameters[BA_SHOT_RY] = v; }
  void SetRZ(double v) { parameters[BA_SHOT_RZ] = v; }
  void SetTX(double v) { parameters[BA_SHOT_TX] = v; }
  void SetTY(double v) { parameters[BA_SHOT_TY] = v; }
  void SetTZ(double v) { parameters[BA_SHOT_TZ] = v; }
  double GetCovariance(int i, int j) { return covariance[i * BA_SHOT_NUM_PARAMS + j]; }
};

struct BAPoint {
  double coordinates[3];
  bool constant;
  double reprojection_error;
  std::string id;

  double GetX() { return coordinates[0]; }
  double GetY() { return coordinates[1]; }
  double GetZ() { return coordinates[2]; }
  void SetX(double v) { coordinates[0] = v; }
  void SetY(double v) { coordinates[1] = v; }
  void SetZ(double v) { coordinates[2] = v; }
};

struct BAObservation {
  double coordinates[2];
  BACamera *camera;
  BAShot *shot;
  BAPoint *point;
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

struct BAPointPositionPrior {
  BAPoint *point;
  double position[3];
  double std_deviation;
};

class TruncatedLoss : public ceres::LossFunction {
 public:
  explicit TruncatedLoss(double t)
    : t2_(t*t) {
    CHECK_GT(t, 0.0);
  }

  virtual void Evaluate(double s, double rho[3]) const {
    if (s >= t2_) {
      // Outlier.
      rho[0] = t2_;
      rho[1] = std::numeric_limits<double>::min();
      rho[2] = 0.0;
    } else {
      // Inlier.
      rho[0] = s;
      rho[1] = 1.0;
      rho[2] = 0.0;
    }
  }

 private:
  const double t2_;
};


// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed to be located at the image center).
struct SnavelyReprojectionError {
  SnavelyReprojectionError(double observed_x, double observed_y, double std_deviation)
      : observed_x_(observed_x)
      , observed_y_(observed_y)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const shot,
                  const T* const point,
                  T* residuals) const {
    // shot[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(shot, point, p);

    // shot[3,4,5] are the translation.
    p[0] += shot[3];
    p[1] += shot[4];
    p[2] += shot[5];

    // Project.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[1];
    const T& l2 = camera[2];
    T r2 = xp * xp + yp * yp;
    T distortion = T(1.0) + r2  * (l1 + l2  * r2);

    // Compute final projected point position.
    const T& focal = camera[0];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;
    // The error is the difference between the predicted and observed position.
    residuals[0] = T(scale_) * (predicted_x - T(observed_x_));
    residuals[1] = T(scale_) * (predicted_y - T(observed_y_));

    return true;
  }

  double observed_x_;
  double observed_y_;
  double scale_;
};


struct InternalParametersPriorError {
  InternalParametersPriorError(double focal_estimate,
                               double focal_std_deviation,
                               double k1_std_deviation,
                               double k2_std_deviation)
      : log_focal_estimate_(log(focal_estimate))
      , focal_scale_(1.0 / focal_std_deviation)
      , k1_scale_(1.0 / k1_std_deviation)
      , k2_scale_(1.0 / k2_std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    residuals[0] = T(focal_scale_) * (log(parameters[BA_CAMERA_FOCAL]) - T(log_focal_estimate_));
    residuals[1] = T(k1_scale_) * parameters[BA_CAMERA_K1];
    residuals[2] = T(k2_scale_) * parameters[BA_CAMERA_K2];
    return true;
  }

  double log_focal_estimate_;
  double focal_scale_;
  double k1_scale_;
  double k2_scale_;
};


struct RotationPriorError {
  RotationPriorError(double *R_prior, double std_deviation)
      : R_prior_(R_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    // Get rotation and translation values.
    const T* const R = parameters + BA_SHOT_RX;
    T Rpt[3] = { -T(R_prior_[0]),
                 -T(R_prior_[1]),
                 -T(R_prior_[2]) };

    // Compute rotation residual: log( R Rp^t )
    T qR[4], qRpt[4], qR_Rpt[4];
    ceres::AngleAxisToQuaternion(R, qR);
    ceres::AngleAxisToQuaternion(Rpt, qRpt);
    ceres::QuaternionProduct(qR, qRpt, qR_Rpt);
    T R_Rpt[3];
    ceres::QuaternionToAngleAxis(qR_Rpt, R_Rpt);

    residuals[0] = T(scale_) * R_Rpt[0];
    residuals[1] = T(scale_) * R_Rpt[1];
    residuals[2] = T(scale_) * R_Rpt[2];

    return true;
  }

  double *R_prior_;
  double scale_;
};

struct TranslationPriorError {
  TranslationPriorError(double *translation_prior, double std_deviation)
      : translation_prior_(translation_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters, T* residuals) const {
    residuals[0] = T(scale_) * (T(translation_prior_[0]) - parameters[BA_SHOT_TX]);
    residuals[1] = T(scale_) * (T(translation_prior_[1]) - parameters[BA_SHOT_TY]);
    residuals[2] = T(scale_) * (T(translation_prior_[2]) - parameters[BA_SHOT_TZ]);
    return true;
  }

  double *translation_prior_;
  double scale_;
};

struct UnitTranslationPriorError {
  UnitTranslationPriorError() {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T* const t = shot + 3;
    residuals[0] = log(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    return true;
  }
};


struct PointPositionPriorError {
  PointPositionPriorError(double *position, double std_deviation)
      : position_(position)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const p, T* residuals) const {
    residuals[0] = T(scale_) * (p[0] - T(position_[0]));
    residuals[1] = T(scale_) * (p[1] - T(position_[1]));
    residuals[2] = T(scale_) * (p[2] - T(position_[2]));
    return true;
  }

  double *position_;
  double scale_;
};


struct GPSPriorError {
  GPSPriorError(double x, double y, double z, double std_deviation)
      : x_(x), y_(y), z_(z)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    // shot[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(shot, shot + 3, p);

    residuals[0] = T(scale_) * (-p[0] - T(x_));
    residuals[1] = T(scale_) * (-p[1] - T(y_));
    residuals[2] = T(scale_) * (-p[2] - T(z_));
    return true;
  }

  double x_, y_, z_;
  double scale_;
};


// A bundle adjustment class for optimizing the problem
//
//    sum_p ( reprojection_error(p) / reprojection_error_sd )^2
//  + sum_c ( (focal - focal_prior) / focal_prior_sd )^2
//
class BundleAdjuster {
 public:
  BundleAdjuster() {
    unit_translation_shot_ = NULL;
    loss_function_ = "TrivialLoss";
    loss_function_threshold_ = 1;
    reprojection_error_sd_ = 1;
    focal_prior_sd_ = 1;
    k1_sd_ = 1;
    k2_sd_ = 1;
    compute_covariances_ = false;
    covariance_estimation_valid_ = false;
    compute_reprojection_errors_ = true;
    max_num_iterations_ = 50;
    num_threads_ = 1;
  }

  virtual ~BundleAdjuster() {}

  BACamera GetCamera(const std::string &id) {
    return cameras_[id];
  }

  BAShot GetShot(const std::string &id) {
    return shots_[id];
  }

  BAPoint GetPoint(const std::string &id) {
    return points_[id];
  }

  void AddCamera(
      const std::string &id,
      double focal,
      double k1,
      double k2,
      double focal_prior,
      bool constant) {
    BACamera c;
    c.id = id;
    c.parameters[BA_CAMERA_FOCAL] = focal;
    c.parameters[BA_CAMERA_K1] = k1;
    c.parameters[BA_CAMERA_K2] = k2;
    c.constant = constant;
    c.focal_prior = focal_prior;
    cameras_[id] = c;
  }

  void AddShot(
      const std::string &id,
      const std::string &camera,
      double rx,
      double ry,
      double rz,
      double tx,
      double ty,
      double tz,
      double gpsx,
      double gpsy,
      double gpsz,
      double gps_dop,
      bool constant) {
    BAShot s;
    s.id = id;
    s.camera = camera;
    s.parameters[BA_SHOT_RX] = rx;
    s.parameters[BA_SHOT_RY] = ry;
    s.parameters[BA_SHOT_RZ] = rz;
    s.parameters[BA_SHOT_TX] = tx;
    s.parameters[BA_SHOT_TY] = ty;
    s.parameters[BA_SHOT_TZ] = tz;
    s.constant = constant;
    s.gps_x = gpsx;
    s.gps_y = gpsy;
    s.gps_z = gpsz;
    s.gps_dop = gps_dop;
    shots_[id] = s;
  }

  void AddPoint(
      const std::string &id,
      double x,
      double y,
      double z,
      bool constant) {
    BAPoint p;
    p.id = id;
    p.coordinates[0] = x;
    p.coordinates[1] = y;
    p.coordinates[2] = z;
    p.constant = constant;
    p.reprojection_error = -1;
    points_[id] = p;
  }

  void AddObservation(
      const std::string &shot,
      const std::string &point,
      double x,
      double y) {
    BAObservation o;
    o.shot = &shots_[shot];
    o.camera = &cameras_[o.shot->camera];
    o.point = &points_[point];
    o.coordinates[0] = x;
    o.coordinates[1] = y;
    observations_.push_back(o);
  }

  void AddRotationPrior(
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

  void AddTranslationPrior(
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

  void AddPointPositionPrior(
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


  void SetOriginShot(const std::string &shot_id) {
    BAShot *shot = &shots_[shot_id];
    for (int i = 0; i < 6; ++i) shot->parameters[0] = 0;
    shot->constant = true;
  }

  void SetUnitTranslationShot(const std::string &shot_id) {
    unit_translation_shot_ = &shots_[shot_id];
  }

  void SetLossFunction(const std::string &function_name,
                       double threshold) {
    loss_function_ = function_name;
    loss_function_threshold_ = threshold;
  }

  void SetReprojectionErrorSD(double sd) {
    reprojection_error_sd_ = sd;
  }

  void SetMaxNumIterations(int miter) {
    max_num_iterations_ = miter;
  }

  void SetNumThreads(int n) {
    num_threads_ = n;
  }

  void SetInternalParametersPriorSD(double focal_sd, double k1_sd, double k2_sd) {
    focal_prior_sd_ = focal_sd;
    k1_sd_ = k1_sd;
    k2_sd_ = k2_sd;
  }

  void SetComputeCovariances(bool v) {
    compute_covariances_ = v;
  }

  bool GetCovarianceEstimationValid() {
    return covariance_estimation_valid_;
  }

  void SetComputeReprojectionErrors(bool v) {
    compute_reprojection_errors_ = v;
  }

  void Run() {
    ceres::LossFunction *loss;
    if (loss_function_.compare("TruncatedLoss") == 0) {
      loss = new TruncatedLoss(loss_function_threshold_);
    } else if (loss_function_.compare("TrivialLoss") == 0) {
      loss = new ceres::TrivialLoss();
    } else if (loss_function_.compare("HuberLoss") == 0) {
      loss = new ceres::HuberLoss(loss_function_threshold_);
    } else if (loss_function_.compare("SoftLOneLoss") == 0) {
      loss = new ceres::SoftLOneLoss(loss_function_threshold_);
    } else if (loss_function_.compare("CauchyLoss") == 0) {
      loss = new ceres::CauchyLoss(loss_function_threshold_);
    } else if (loss_function_.compare("ArctanLoss") == 0) {
      loss = new ceres::ArctanLoss(loss_function_threshold_);
    }

    ceres::Problem problem;

    // Init parameter blocks.
    for (auto &i : cameras_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.parameters, BA_CAMERA_NUM_PARAMS);
        problem.SetParameterBlockConstant(i.second.parameters);
      }
    }
    for (auto &i : shots_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.parameters, BA_SHOT_NUM_PARAMS);
        problem.SetParameterBlockConstant(i.second.parameters);
      }
    }
    for (auto &i : points_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.coordinates, 3);
        problem.SetParameterBlockConstant(i.second.coordinates);
      }
    }

    // Add reprojection error blocks
    for (int i = 0; i < observations_.size(); ++i) {
      // Each Residual block takes a point and a camera as input and outputs a 2
      // dimensional residual. Internally, the cost function stores the observed
      // image location and compares the reprojection against the observation.
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 3, 6, 3>(
              new SnavelyReprojectionError(observations_[i].coordinates[0],
                                           observations_[i].coordinates[1],
                                           reprojection_error_sd_));

      problem.AddResidualBlock(cost_function,
                               loss,
                               observations_[i].camera->parameters,
                               observations_[i].shot->parameters,
                               observations_[i].point->coordinates);
    }

    // Add rotation priors
    for (int i = 0; i < rotation_priors_.size(); ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RotationPriorError, 3, 6>(
              new RotationPriorError(rotation_priors_[i].rotation,
                                     rotation_priors_[i].std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               rotation_priors_[i].shot->parameters);
    }

    // Add translation priors
    for (int i = 0; i < translation_priors_.size(); ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<TranslationPriorError, 3, 6>(
              new TranslationPriorError(translation_priors_[i].translation,
                                        translation_priors_[i].std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               translation_priors_[i].shot->parameters);
    }

    // Add point position priors
    for (int i = 0; i < point_position_priors_.size(); ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<PointPositionPriorError, 3, 3>(
              new PointPositionPriorError(point_position_priors_[i].position,
                                          point_position_priors_[i].std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               point_position_priors_[i].point->coordinates);
    }


    // Add internal parameter priors blocks
    for (auto &i : cameras_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<InternalParametersPriorError, 3, 3>(
              new InternalParametersPriorError(i.second.focal_prior, focal_prior_sd_, k1_sd_, k2_sd_));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               i.second.parameters);
    }

    // Add unit translation block
    if (unit_translation_shot_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<UnitTranslationPriorError, 1, 6>(
              new UnitTranslationPriorError());

      problem.AddResidualBlock(cost_function,
                               NULL,
                               unit_translation_shot_->parameters);
    }

    // for (auto &i : shots_) {
    //   ceres::CostFunction* cost_function =
    //       new ceres::AutoDiffCostFunction<GPSPriorError, 3, 6>(
    //           new GPSPriorError(i.second.gps_position[0],
    //                             i.second.gps_position[1],
    //                             i.second.gps_position[2],
    //                             i.second.gps_dop));

    //   problem.AddResidualBlock(cost_function,
    //                            NULL,
    //                            i.second.parameters);
    // }


    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = num_threads_;
    options.num_linear_solver_threads = num_threads_;
    options.max_num_iterations = max_num_iterations_;

    ceres::Solve(options, &problem, &last_run_summary_);

    if (compute_covariances_) {
      ComputeCovariances(&problem);
    }
    if (compute_reprojection_errors_) {
      ComputeReprojectionErrors();
    }
  }

  void ComputeCovariances(ceres::Problem *problem) {
    ceres::Covariance::Options options;
    ceres::Covariance covariance(options);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    for (auto &i : shots_) {
      covariance_blocks.push_back(std::make_pair(i.second.parameters, i.second.parameters));
    }

    bool worked = covariance.Compute(covariance_blocks, problem);

    if (worked) {
      for (auto &i : shots_) {
        covariance_estimation_valid_ = true;
        covariance.GetCovarianceBlock(i.second.parameters, i.second.parameters, i.second.covariance);
      }
    } else { // If covariance estimation failed, use a default value
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

  void ComputeReprojectionErrors() {
    // Init errors
    for (auto &i : points_) {
      i.second.reprojection_error = 0;
    }

    // Sum over all observations
    for (int i = 0; i < observations_.size(); ++i) {
      SnavelyReprojectionError sre(observations_[i].coordinates[0],
                                   observations_[i].coordinates[1],
                                   1.0);
      double residuals[2];
      sre(observations_[i].camera->parameters,
          observations_[i].shot->parameters,
          observations_[i].point->coordinates,
          residuals);
      double error = sqrt(residuals[0] * residuals[0] + residuals[1] * residuals[1]);
      observations_[i].point->reprojection_error =
          std::max(observations_[i].point->reprojection_error, error);
    }
  }


  std::string BriefReport() {
    return last_run_summary_.BriefReport();
  }
  std::string FullReport() {
    return last_run_summary_.FullReport();
  }

 private:
  std::map<std::string, BACamera> cameras_;
  std::map<std::string, BAShot> shots_;
  std::map<std::string, BAPoint> points_;

  std::vector<BAObservation> observations_;
  std::vector<BARotationPrior> rotation_priors_;
  std::vector<BATranslationPrior> translation_priors_;
  std::vector<BAPointPositionPrior> point_position_priors_;

  BAShot *unit_translation_shot_;

  std::string loss_function_;
  double loss_function_threshold_;
  double reprojection_error_sd_;
  double focal_prior_sd_;
  double k1_sd_;
  double k2_sd_;
  bool compute_covariances_;
  bool covariance_estimation_valid_;
  bool compute_reprojection_errors_;
  int max_num_iterations_;
  int num_threads_;


  ceres::Solver::Summary last_run_summary_;
};


