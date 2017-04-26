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


enum {
  RA_SHOT_RX,
  RA_SHOT_RY,
  RA_SHOT_RZ,
  RA_SHOT_TX,
  RA_SHOT_TY,
  RA_SHOT_TZ,
  RA_SHOT_NUM_PARAMS
};

enum {
  RA_RECONSTRUCTION_RX,
  RA_RECONSTRUCTION_RY,
  RA_RECONSTRUCTION_RZ,
  RA_RECONSTRUCTION_TX,
  RA_RECONSTRUCTION_TY,
  RA_RECONSTRUCTION_TZ,
  RA_RECONSTRUCTION_SCALE,
  RA_RECONSTRUCTION_NUM_PARAMS
};

struct RAShot {
  std::string id;
  double parameters[RA_SHOT_NUM_PARAMS];
  bool constant;

  double GetRX() { return parameters[RA_SHOT_RX]; }
  double GetRY() { return parameters[RA_SHOT_RY]; }
  double GetRZ() { return parameters[RA_SHOT_RZ]; }
  double GetTX() { return parameters[RA_SHOT_TX]; }
  double GetTY() { return parameters[RA_SHOT_TY]; }
  double GetTZ() { return parameters[RA_SHOT_TZ]; }
  void SetRX(double v) { parameters[RA_SHOT_RX] = v; }
  void SetRY(double v) { parameters[RA_SHOT_RY] = v; }
  void SetRZ(double v) { parameters[RA_SHOT_RZ] = v; }
  void SetTX(double v) { parameters[RA_SHOT_TX] = v; }
  void SetTY(double v) { parameters[RA_SHOT_TY] = v; }
  void SetTZ(double v) { parameters[RA_SHOT_TZ] = v; }
};

struct RAReconstruction {
  std::string id;
  double parameters[RA_RECONSTRUCTION_NUM_PARAMS];
  bool constant;

  double GetRX() { return parameters[RA_RECONSTRUCTION_RX]; }
  double GetRY() { return parameters[RA_RECONSTRUCTION_RY]; }
  double GetRZ() { return parameters[RA_RECONSTRUCTION_RZ]; }
  double GetTX() { return parameters[RA_RECONSTRUCTION_TX]; }
  double GetTY() { return parameters[RA_RECONSTRUCTION_TY]; }
  double GetTZ() { return parameters[RA_RECONSTRUCTION_TZ]; }
  double GetScale() { return parameters[RA_RECONSTRUCTION_SCALE]; }
  void SetRX(double v) { parameters[RA_RECONSTRUCTION_RX] = v; }
  void SetRY(double v) { parameters[RA_RECONSTRUCTION_RY] = v; }
  void SetRZ(double v) { parameters[RA_RECONSTRUCTION_RZ] = v; }
  void SetTX(double v) { parameters[RA_RECONSTRUCTION_TX] = v; }
  void SetTY(double v) { parameters[RA_RECONSTRUCTION_TY] = v; }
  void SetTZ(double v) { parameters[RA_RECONSTRUCTION_TZ] = v; }
  void SetScale(double v) { parameters[RA_RECONSTRUCTION_SCALE] = v; }
};

struct RARelativeMotionConstraint {
  RARelativeMotionConstraint(const std::string &reconstruction,
                   const std::string &shot,
                   double rx,
                   double ry,
                   double rz,
                   double tx,
                   double ty,
                   double tz) {
    reconstruction_id = reconstruction;
    shot_id = shot;
    parameters[RA_SHOT_RX] = rx;
    parameters[RA_SHOT_RY] = ry;
    parameters[RA_SHOT_RZ] = rz;
    parameters[RA_SHOT_TX] = tx;
    parameters[RA_SHOT_TY] = ty;
    parameters[RA_SHOT_TZ] = tz;
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        scale_matrix[6 * i + j] = (i == j) ? 1.0 : 0.0;
      }
    }
  }
  double GetRX() { return parameters[RA_SHOT_RX]; }
  double GetRY() { return parameters[RA_SHOT_RY]; }
  double GetRZ() { return parameters[RA_SHOT_RZ]; }
  double GetTX() { return parameters[RA_SHOT_TX]; }
  double GetTY() { return parameters[RA_SHOT_TY]; }
  double GetTZ() { return parameters[RA_SHOT_TZ]; }
  void SetRX(double v) { parameters[RA_SHOT_RX] = v; }
  void SetRY(double v) { parameters[RA_SHOT_RY] = v; }
  void SetRZ(double v) { parameters[RA_SHOT_RZ] = v; }
  void SetTX(double v) { parameters[RA_SHOT_TX] = v; }
  void SetTY(double v) { parameters[RA_SHOT_TY] = v; }
  void SetTZ(double v) { parameters[RA_SHOT_TZ] = v; }
  void SetScaleMatrix(int i, int j, double value) {
    scale_matrix[i * RA_SHOT_NUM_PARAMS + j] = value;
  }

  std::string reconstruction_id;
  std::string shot_id;
  double parameters[RA_SHOT_NUM_PARAMS];
  double scale_matrix[RA_SHOT_NUM_PARAMS * RA_SHOT_NUM_PARAMS];
};

struct RAAbsolutePositionConstraint {
  RAShot *shot;
  double position[3];
  double std_deviation;
};


struct RARelativeMotionError {
  RARelativeMotionError(double *Rtai, double *scale_matrix)
      : Rtai_(Rtai)
      , scale_matrix_(scale_matrix)
  {}

  template <typename T>
  bool operator()(const T* const reconstruction_a,
  				  const T* const shot_i,
                  T* residuals) const {
    T r[6];

    // Get rotation and translation values.
    const T* const Ri = shot_i + RA_SHOT_RX;
    const T* const Ra = reconstruction_a + RA_RECONSTRUCTION_RX;
    T Rit[3] = { -Ri[0], -Ri[1], -Ri[2] };
    T Rat[3] = { -Ra[0], -Ra[1], -Ra[2] };
    const T* const ti = shot_i + RA_SHOT_TX;
    const T* const ta = reconstruction_a + RA_RECONSTRUCTION_TX;
    const T* const scale_a = reconstruction_a + RA_RECONSTRUCTION_SCALE;
    T Rai[3] = { T(Rtai_[RA_SHOT_RX]),
                 T(Rtai_[RA_SHOT_RY]),
                 T(Rtai_[RA_SHOT_RZ]) };
    T tai[3] = { T(Rtai_[RA_SHOT_TX]),
                 T(Rtai_[RA_SHOT_TY]),
                 T(Rtai_[RA_SHOT_TZ]) };

    // Compute rotation residual: log( Rai Ra Ri^t )
    T qRai[4], qRa[4], qRit[4], qRa_Rit[4], qRai_Ra_Rit[4];
    ceres::AngleAxisToQuaternion(Rai, qRai);
    ceres::AngleAxisToQuaternion(Ra, qRa);
    ceres::AngleAxisToQuaternion(Rit, qRit);
    ceres::QuaternionProduct(qRa, qRit, qRa_Rit);
    ceres::QuaternionProduct(qRai, qRa_Rit, qRai_Ra_Rit);

    T Rai_Ra_Rit[3];
    ceres::QuaternionToAngleAxis(qRai_Ra_Rit, Rai_Ra_Rit);
    r[0] = Rai_Ra_Rit[0];
    r[1] = Rai_Ra_Rit[1];
    r[2] = Rai_Ra_Rit[2];

    // Compute translation residual: tai - sa * ti - Ri Ra^t ta
    T Rat_ta[3], Ri_Rat_ta[3];
    ceres::AngleAxisRotatePoint(Rat, ta, Rat_ta);
    ceres::AngleAxisRotatePoint(Ri, Rat_ta, Ri_Rat_ta);

    r[3] = tai[0] - scale_a[0] * ti[0] - Ri_Rat_ta[0];
    r[4] = tai[1] - scale_a[0] * ti[1] - Ri_Rat_ta[1];
    r[5] = tai[2] - scale_a[0] * ti[2] - Ri_Rat_ta[2];

    for (int i = 0; i < 6; ++i) {
      residuals[i] = T(0);
      for (int j = 0; j < 6; ++j) {
        residuals[i] += T(scale_matrix_[6 * i + j]) * r[j];
      }
    }
    return true;
  }

  double *Rtai_;
  double *scale_matrix_;
};


struct RAAbsolutePositionError {
  RAAbsolutePositionError(double *c_prior, double std_deviation)
      : c_prior_(c_prior)
      , scale_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const shot_i,
                  T* residuals) const {
    // error: c + R^t t
    const T* const Ri = shot_i + RA_SHOT_RX;
    const T* const ti = shot_i + RA_SHOT_TX;
    T Rit[3] = { -Ri[0], -Ri[1], -Ri[2] };

    T Rit_ti[3];
    ceres::AngleAxisRotatePoint(Rit, ti, Rit_ti);

    residuals[0] = T(scale_) * (T(c_prior_[0]) + Rit_ti[0]);
    residuals[1] = T(scale_) * (T(c_prior_[1]) + Rit_ti[1]);
    residuals[2] = T(scale_) * (T(c_prior_[2]) + Rit_ti[2]);

    return true;
  }

  double *c_prior_;
  double scale_;
};


class ReconstructionAlignment {
 public:
  ReconstructionAlignment() {}
  virtual ~ReconstructionAlignment() {}

  RAShot GetShot(const std::string &id) {
    return shots_[id];
  }

  RAReconstruction GetReconstruction(const std::string &id) {
    return reconstructions_[id];
  }

  void AddShot(
      const std::string &id,
      double rx,
      double ry,
      double rz,
      double tx,
      double ty,
      double tz,
      bool constant) {
    RAShot s;
    s.id = id;
    s.parameters[RA_SHOT_RX] = rx;
    s.parameters[RA_SHOT_RY] = ry;
    s.parameters[RA_SHOT_RZ] = rz;
    s.parameters[RA_SHOT_TX] = tx;
    s.parameters[RA_SHOT_TY] = ty;
    s.parameters[RA_SHOT_TZ] = tz;
    s.constant = constant;
    shots_[id] = s;
  }

  void AddReconstruction(
      const std::string &id,
      double rx,
      double ry,
      double rz,
      double tx,
      double ty,
      double tz,
      double scale,
      bool constant) {
    RAReconstruction r;
    r.id = id;
    r.parameters[RA_RECONSTRUCTION_RX] = rx;
    r.parameters[RA_RECONSTRUCTION_RY] = ry;
    r.parameters[RA_RECONSTRUCTION_RZ] = rz;
    r.parameters[RA_RECONSTRUCTION_TX] = tx;
    r.parameters[RA_RECONSTRUCTION_TY] = ty;
    r.parameters[RA_RECONSTRUCTION_TZ] = tz;
    r.parameters[RA_RECONSTRUCTION_SCALE] = scale;
    r.constant = constant;
    reconstructions_[id] = r;
  }

  void AddRelativeMotionConstraint(const RARelativeMotionConstraint &rm) {
    relative_motions_.push_back(rm);
  }

  void AddAbsolutePositionConstraint(
      const std::string &shot_id,
      double x,
      double y,
      double z,
      double std_deviation) {
    RAAbsolutePositionConstraint a;
    a.shot = &shots_[shot_id];
    a.position[0] = x;
    a.position[1] = y;
    a.position[2] = z;
    a.std_deviation = std_deviation;
    absolute_positions_.push_back(a);
  }

  void Run() {

    ceres::Problem problem;

    // Init paramater blocks.
    for (auto &i : shots_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.parameters, RA_SHOT_NUM_PARAMS);
        problem.SetParameterBlockConstant(i.second.parameters);
      }
    }

    for (auto &i : reconstructions_) {
      if (i.second.constant) {
        problem.AddParameterBlock(i.second.parameters, RA_RECONSTRUCTION_NUM_PARAMS);
        problem.SetParameterBlockConstant(i.second.parameters);
      } else {
        problem.AddParameterBlock(i.second.parameters, RA_RECONSTRUCTION_NUM_PARAMS);
        double *scale = i.second.parameters + RA_RECONSTRUCTION_SCALE;
        problem.SetParameterLowerBound(scale, 0, 0.0);
        problem.SetParameterUpperBound(scale, 0, std::numeric_limits<double>::max());
      }
    }

    // Add relative motion errors
    ceres::LossFunction *loss = new ceres::CauchyLoss(1.0);
    for (auto &rp: relative_motions_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RARelativeMotionError, 7, 6, 6>(
              new RARelativeMotionError(rp.parameters, rp.scale_matrix));

      problem.AddResidualBlock(cost_function,
                               loss,
                               reconstructions_[rp.reconstruction_id].parameters,
                               shots_[rp.shot_id].parameters);
    }

    // Add absolute position errors
    for (auto &a: absolute_positions_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RAAbsolutePositionError, 3, 6>(
              new RAAbsolutePositionError(a.position, a.std_deviation));

      problem.AddResidualBlock(cost_function,
                               NULL,
                               a.shot->parameters);
    }

    // Solve
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 8;
    options.num_linear_solver_threads = 8;
    options.max_num_iterations = 500;

    ceres::Solve(options, &problem, &last_run_summary_);
  }

  std::string BriefReport() {
    return last_run_summary_.BriefReport();
  }

  std::string FullReport() {
    return last_run_summary_.FullReport();
  }

 private:
  std::map<std::string, RAReconstruction> reconstructions_;
  std::map<std::string, RAShot> shots_;
  std::vector<RARelativeMotionConstraint> relative_motions_;
  std::vector<RAAbsolutePositionConstraint> absolute_positions_;

  ceres::Solver::Summary last_run_summary_;
};


