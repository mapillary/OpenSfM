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

struct RACommonPointConstraint {
  RAReconstruction *reconstruction_a;
  double point_a[3];
  RAReconstruction *reconstruction_b;
  double point_b[3];
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
    const T* const ti = shot_i + RA_SHOT_TX;
    const T* const ta = reconstruction_a + RA_RECONSTRUCTION_TX;
    const T* const scale_a = reconstruction_a + RA_RECONSTRUCTION_SCALE;
    T Rai[3] = { T(Rtai_[RA_SHOT_RX]),
                 T(Rtai_[RA_SHOT_RY]),
                 T(Rtai_[RA_SHOT_RZ]) };
    T tai[3] = { T(Rtai_[RA_SHOT_TX]),
                 T(Rtai_[RA_SHOT_TY]),
                 T(Rtai_[RA_SHOT_TZ]) };
    T Rait[3] = { -Rai[0], -Rai[1], -Rai[2] };

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

    // Compute optical center residual: Rai^t tai - sa Ra Ri^t ti + ta
    T Rait_tai[3], Rit_ti[3], Ra_Rit_ti[3];
    ceres::AngleAxisRotatePoint(Rait, tai, Rait_tai);
    ceres::AngleAxisRotatePoint(Rit, ti, Rit_ti);
    ceres::AngleAxisRotatePoint(Ra, Rit_ti, Ra_Rit_ti);

    r[3] = Rait_tai[0] - scale_a[0] * Ra_Rit_ti[0] + ta[0];
    r[4] = Rait_tai[1] - scale_a[0] * Ra_Rit_ti[1] + ta[1];
    r[5] = Rait_tai[2] - scale_a[0] * Ra_Rit_ti[2] + ta[2];

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

struct RACommonPointError {
  RACommonPointError(double *pai, double *pbi, double std_deviation)
      : pai_(pai)
      , pbi_(pbi)
      , inv_std_(1.0 / std_deviation)
  {}

  template <typename T>
  bool operator()(const T* const reconstruction_a,
                  const T* const reconstruction_b,
                  T* residuals) const {
    T transformed_pai[3];
    transform_point(reconstruction_a, pai_, transformed_pai);

    T transformed_pbi[3];
    transform_point(reconstruction_b, pbi_, transformed_pbi);

    T scale_factor = reconstruction_a[RA_RECONSTRUCTION_SCALE] + \
                     reconstruction_b[RA_RECONSTRUCTION_SCALE];

    residuals[0] = T(inv_std_) * scale_factor * (transformed_pai[0] - transformed_pbi[0]);
    residuals[1] = T(inv_std_) * scale_factor * (transformed_pai[1] - transformed_pbi[1]);
    residuals[2] = T(inv_std_) * scale_factor * (transformed_pai[2] - transformed_pbi[2]);

    return true;
  }

  template <typename T>
  void transform_point(const T* const reconstruction,
                       double *point,
                       T *transformed) const {
    const T* const R = reconstruction + RA_RECONSTRUCTION_RX;
    const T* const t = reconstruction + RA_RECONSTRUCTION_TX;
    const T &scale = reconstruction[RA_RECONSTRUCTION_SCALE];
    T p_t_s[3] = {
        (T(point[0]) - t[0]) / scale,
        (T(point[1]) - t[1]) / scale,
        (T(point[2]) - t[2]) / scale
    };
    T Rt[3] = { -R[0], -R[1], -R[2] };
    ceres::AngleAxisRotatePoint(Rt, p_t_s, transformed);
  }

  double *pai_;
  double *pbi_;
  double inv_std_;
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

  void AddCommonPointConstraint(
    const std::string &reconstruction_a_id,
    double xa, double ya, double za,
    const std::string &reconstruction_b_id,
    double xb, double yb, double zb,
    double std_deviation) {
    RACommonPointConstraint c;
    c.reconstruction_a = &reconstructions_[reconstruction_a_id];
    c.point_a[0] = xa;
    c.point_a[1] = ya;
    c.point_a[2] = za;
    c.reconstruction_b = &reconstructions_[reconstruction_b_id];
    c.point_b[0] = xb;
    c.point_b[1] = yb;
    c.point_b[2] = zb;
    c.std_deviation = std_deviation;
    common_points_.push_back(c);
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
      problem.AddParameterBlock(i.second.parameters, RA_RECONSTRUCTION_NUM_PARAMS);
      if (i.second.constant) {
        problem.SetParameterBlockConstant(i.second.parameters);
      } else {
        problem.SetParameterLowerBound(i.second.parameters, RA_RECONSTRUCTION_SCALE, 0.1);
        problem.SetParameterUpperBound(i.second.parameters, RA_RECONSTRUCTION_SCALE, std::numeric_limits<double>::max());
      }
    }

    // Add relative motion errors
    ceres::LossFunction *loss = new ceres::CauchyLoss(1.0);
    for (auto &rp: relative_motions_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RARelativeMotionError, 6, 7, 6>(
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

    // Add common point errors
    ceres::LossFunction *l1_loss = new ceres::SoftLOneLoss(1.0);
    for (auto &a: common_points_) {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<RACommonPointError, 3, 7, 7>(
              new RACommonPointError(a.point_a, a.point_b, a.std_deviation));

      problem.AddResidualBlock(cost_function,
                               l1_loss,
                               a.reconstruction_a->parameters,
                               a.reconstruction_b->parameters);
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
  std::vector<RACommonPointConstraint> common_points_;

  ceres::Solver::Summary last_run_summary_;
};


