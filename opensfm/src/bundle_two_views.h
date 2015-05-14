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
  TVBA_PARAMS_RX,
  TVBA_PARAMS_RY,
  TVBA_PARAMS_RZ,
  TVBA_PARAMS_TX,
  TVBA_PARAMS_TY,
  TVBA_PARAMS_TZ,
  TVBA_PARAMS_NUM_PARAMS
};

struct SampsonError {
  SampsonError(double x1, double y1, double f1,
               double x2, double y2, double f2,
               double std_deviation)
      : x1_(x1), y1_(y1), f1_(f1)
      , x2_(x2), y2_(y2), f2_(f2)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const parameters,
                  T* residuals) const {
    // F = K2^{-T} [t]_x R K1^{-1}

    const T *R = parameters + TVBA_PARAMS_RX;
    const T *t = parameters + TVBA_PARAMS_TX;
    const T Rt[3] = { -R[0], -R[1], -R[2] };
    const T mt[3] = { -t[0], -t[1], -t[2] };

    const T x1[3] = { T(x1_), T(y1_), T(f1_) };
    const T x2[3] = { T(x2_), T(y2_), T(f2_) };

    T R_x1[3], F_x1[3];
    ceres::AngleAxisRotatePoint(R, x1, R_x1);
    ceres::CrossProduct(t, R_x1, F_x1);

    T x2_t[3], x2_F[3];
    ceres::CrossProduct(mt, x2, x2_t);
    ceres::AngleAxisRotatePoint(Rt, x2_t, x2_F);

    const T x2_F_x1 = ceres::DotProduct(x2, F_x1);

    residuals[0] = T(scale_) * T(0.5) * x2_F_x1
        / ceres::sqrt(F_x1[0] * F_x1[0] + F_x1[1] * F_x1[1] +
                      x2_F[0] * x2_F[0] + x2_F[1] * x2_F[1] );

    return true;
  }

  double x1_, y1_, f1_;
  double x2_, y2_, f2_;
  double scale_;
};


struct TVBAUnitTranslationPriorError {
  TVBAUnitTranslationPriorError(double std_deviation)
      : scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T* const t = shot + TVBA_PARAMS_TX;
    residuals[0] = T(scale_) * log(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
    return true;
  }
  double scale_;
};

struct TVBAParams {
  double parameters[TVBA_PARAMS_NUM_PARAMS];
  double covariance[TVBA_PARAMS_NUM_PARAMS * TVBA_PARAMS_NUM_PARAMS];
  double focal1;
  double focal2;

  double GetRX() { return parameters[TVBA_PARAMS_RX]; }
  double GetRY() { return parameters[TVBA_PARAMS_RY]; }
  double GetRZ() { return parameters[TVBA_PARAMS_RZ]; }
  double GetTX() { return parameters[TVBA_PARAMS_TX]; }
  double GetTY() { return parameters[TVBA_PARAMS_TY]; }
  double GetTZ() { return parameters[TVBA_PARAMS_TZ]; }
  void SetRX(double v) { parameters[TVBA_PARAMS_RX] = v; }
  void SetRY(double v) { parameters[TVBA_PARAMS_RY] = v; }
  void SetRZ(double v) { parameters[TVBA_PARAMS_RZ] = v; }
  void SetTX(double v) { parameters[TVBA_PARAMS_TX] = v; }
  void SetTY(double v) { parameters[TVBA_PARAMS_TY] = v; }
  void SetTZ(double v) { parameters[TVBA_PARAMS_TZ] = v; }
  double GetCovariance(int i, int j) { return covariance[i * TVBA_PARAMS_NUM_PARAMS + j]; }
};

struct TVBAObservation {
  double x1, y1, x2, y2;
};

class TwoViewBundleAdjuster {
 public:
  TwoViewBundleAdjuster() {
    loss_function_ = "CauchyLoss";
    loss_function_threshold_ = 1;
    reprojection_error_sd_ = 1;
    compute_covariance_ = false;
    covariance_estimation_valid_ = false;
    max_num_iterations_ = 50;
  }

  virtual ~TwoViewBundleAdjuster() {}


  void InitParams(double rx,
                  double ry,
                  double rz,
                  double tx,
                  double ty,
                  double tz,
                  double focal1,
                  double focal2) {
    params_.parameters[TVBA_PARAMS_RX] = rx;
    params_.parameters[TVBA_PARAMS_RY] = ry;
    params_.parameters[TVBA_PARAMS_RZ] = rz;
    params_.parameters[TVBA_PARAMS_TX] = tx;
    params_.parameters[TVBA_PARAMS_TY] = ty;
    params_.parameters[TVBA_PARAMS_TZ] = tz;
    params_.focal1 = focal1;
    params_.focal2 = focal2;
  }

  void AddObservation(double x1,
                      double y1,
                      double x2,
                      double y2) {
    TVBAObservation o;
    o.x1 = x1;
    o.y1 = y1;
    o.x2 = x2;
    o.y2 = y2;
    observations_.push_back(o);
  }

  TVBAParams GetParams() {
    return params_;
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

  void SetComputeCovariance(bool v) {
    compute_covariance_ = v;
  }

  bool GetCovarianceEstimationValid() {
    return covariance_estimation_valid_;
  }

  void Run() {
    ceres::LossFunction *loss;
    if (loss_function_.compare("TrivialLoss") == 0) {
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

    // Add reprojection error cost
    for (int i = 0; i < observations_.size(); ++i) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<SampsonError, 1, 6>(
              new SampsonError(observations_[i].x1, observations_[i].y1, params_.focal1,
                               observations_[i].x2, observations_[i].y2, params_.focal2,
                               reprojection_error_sd_));

      problem.AddResidualBlock(cost_function,
                               loss,
                               params_.parameters);
    }

    // Add unit translation cost
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<TVBAUnitTranslationPriorError, 1, 6>(
            new TVBAUnitTranslationPriorError(reprojection_error_sd_));  // Error will count as much as one match

    problem.AddResidualBlock(cost_function,
                             NULL,
                             params_.parameters);

    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;
    options.max_num_iterations = max_num_iterations_;

    ceres::Solve(options, &problem, &last_run_summary_);

    if (compute_covariance_) {
      ComputeCovariance(&problem);
    }
  }

  void ComputeCovariance(ceres::Problem *problem) {
    ceres::Covariance::Options options;
    ceres::Covariance covariance(options);

    std::vector<std::pair<const double*, const double*> > covariance_blocks;
    covariance_blocks.push_back(std::make_pair(params_.parameters, params_.parameters));

    bool worked = covariance.Compute(covariance_blocks, problem);

    if (worked) {
      covariance_estimation_valid_ = true;
      covariance.GetCovarianceBlock(params_.parameters, params_.parameters, params_.covariance);
    } else { // If covariance estimation failed, use a default value
      covariance_estimation_valid_ = false;
      for (int k = 0; k < 6 * 6; ++k) {
        params_.covariance[k] = 0.0;
      }
      double default_rotation_variance = 1e-5;
      double default_translation_variance = 1e-2;
      params_.covariance[6 * 0 + 0] = default_rotation_variance;
      params_.covariance[6 * 1 + 1] = default_rotation_variance;
      params_.covariance[6 * 2 + 2] = default_rotation_variance;
      params_.covariance[6 * 3 + 3] = default_translation_variance;
      params_.covariance[6 * 4 + 4] = default_translation_variance;
      params_.covariance[6 * 5 + 5] = default_translation_variance;
    }
  }

  std::string BriefReport() {
    return last_run_summary_.BriefReport();
  }
  std::string FullReport() {
    return last_run_summary_.FullReport();
  }

 private:
  TVBAParams params_;
  std::vector<TVBAObservation> observations_;

  std::string loss_function_;
  double loss_function_threshold_;
  double reprojection_error_sd_;
  bool compute_covariance_;
  bool covariance_estimation_valid_;
  int max_num_iterations_;

  ceres::Solver::Summary last_run_summary_;
};


