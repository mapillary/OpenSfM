#pragma once

#include <bundle/bundle_adjuster.h>
#include <bundle/error/error_utils.h>

#include <Eigen/Eigen>

namespace bundle {

template <class PosFunc>
struct AbsolutePositionError {
  AbsolutePositionError(const PosFunc& pos_func, const Vec3d& pos_prior,
                        double std_deviation, bool has_std_deviation_param,
                        const PositionConstraintType& type)
      : pos_func_(pos_func),
        pos_prior_(pos_prior),
        scale_(1.0 / std_deviation),
        has_std_deviation_param_(has_std_deviation_param),
        type_(type) {}

  template <typename T>
  bool operator()(T const* const* p, T* r) const {
    Eigen::Map<Vec3<T>> residual(r);

    // error is : position_prior - adjusted_position
    residual = pos_prior_.cast<T>() - pos_func_(p);
    if (has_std_deviation_param_) {
      residual /= p[1][0];
    } else {
      residual *= T(scale_);
    }

    // filter axises to use
    const std::vector<PositionConstraintType> axises = {
        PositionConstraintType::X, PositionConstraintType::Y,
        PositionConstraintType::Z};
    for (int i = 0; i < axises.size(); ++i) {
      if (!HasFlag(axises[i])) {
        residual(i) *= T(0.);
      }
    }
    return true;
  }

  inline bool HasFlag(const PositionConstraintType& flag) const {
    return (int(type_) & int(flag)) == int(flag);
  }

  PosFunc pos_func_;
  Vec3d pos_prior_;
  double scale_;
  bool has_std_deviation_param_;
  PositionConstraintType type_;
};

template <typename T>
T diff_between_angles(T a, T b) {
  T d = a - b;
  if (d > T(M_PI)) {
    return d - T(2 * M_PI);
  } else if (d < -T(M_PI)) {
    return d + T(2 * M_PI);
  } else {
    return d;
  }
}

struct UpVectorError {
  UpVectorError(const Vec3d& acceleration, double std_deviation)
      : scale_(1.0 / std_deviation) {
    acceleration_ = acceleration.normalized();
  }

  template <typename T>
  bool operator()(const T* const shot, T* r) const {
    Eigen::Map<const Vec3<T>> R(shot + Pose::Parameter::RX);
    Eigen::Map<Vec3<T>> residual(r);

    const Vec3<T> acceleration = acceleration_.cast<T>();
    const Vec3<T> z_world = RotatePoint(R.eval(), acceleration);
    const Vec3<T> z_axis = Vec3d(0, 0, 1).cast<T>();
    residual = T(scale_) * (z_world - z_axis);
    return true;
  }

  Vec3d acceleration_;
  double scale_;
};

struct PanAngleError {
  PanAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    Eigen::Map<const Vec3<T>> R(shot + Pose::Parameter::RX);

    const Vec3<T> z_axis = Vec3d(0, 0, 1).cast<T>();
    const auto z_world = RotatePoint(R.eval(), z_axis);

    if (ceres::abs(z_world(0)) < T(1e-8) && ceres::abs(z_world(1)) < T(1e-8)) {
      residuals[0] = T(0.0);
    } else {
      const T predicted_angle = atan2(z_world(0), z_world(1));
      residuals[0] =
          T(scale_) * diff_between_angles(predicted_angle, T(angle_));
    }
    return true;
  }

  double angle_;
  double scale_;
};

struct TiltAngleError {
  TiltAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T* const R = shot + Pose::Parameter::RX;
    T ez[3] = {T(0), T(0), T(1)};  // ez: A point in front of the camera (z=1)
    T Rt_ez[3];
    ceres::AngleAxisRotatePoint(R, ez, Rt_ez);

    T l = sqrt(Rt_ez[0] * Rt_ez[0] + Rt_ez[1] * Rt_ez[1]);
    T predicted_angle = -atan2(Rt_ez[2], l);

    residuals[0] = T(scale_) * diff_between_angles(predicted_angle, T(angle_));
    return true;
  }

  double angle_;
  double scale_;
};

struct RollAngleError {
  RollAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T* const R = shot + Pose::Parameter::RX;
    T ex[3] = {T(1), T(0), T(0)};  // A point to the right of the camera (x=1)
    T ez[3] = {T(0), T(0), T(1)};  // A point in front of the camera (z=1)
    T Rt_ex[3], Rt_ez[3];
    T tangle_ = T(angle_);
    ceres::AngleAxisRotatePoint(R, ex, Rt_ex);
    ceres::AngleAxisRotatePoint(R, ez, Rt_ez);

    T a[3] = {Rt_ez[1], -Rt_ez[0], T(0)};
    T la = sqrt(a[0] * a[0] + a[1] * a[1]);

    if (la < 1e-5) {
      residuals[0] = T(0.0);
      return true;
    }

    a[0] /= la;
    a[1] /= la;
    T b[3];
    ceres::CrossProduct(Rt_ex, a, b);
    T sin_roll = Rt_ez[0] * b[0] + Rt_ez[1] * b[1] + Rt_ez[2] * b[2];
    T predicted_angle = asin(sin_roll);
    residuals[0] = T(scale_) * diff_between_angles(predicted_angle, T(angle_));

    return true;
  }

  double angle_;
  double scale_;
};

struct RotationPriorError {
  RotationPriorError(double* R_prior, double std_deviation)
      : R_prior_(R_prior), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    // Get rotation and translation values.
    const T R[3] = {-shot[Pose::Parameter::RX], -shot[Pose::Parameter::RY],
                    -shot[Pose::Parameter::RZ]};
    T Rpt[3] = {-T(R_prior_[0]), -T(R_prior_[1]), -T(R_prior_[2])};

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

  double* R_prior_;
  double scale_;
};

struct TranslationPriorError {
  TranslationPriorError(double* translation_prior, double std_deviation)
      : translation_prior_(translation_prior), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, T* residuals) const {
    const T mt[3] = {-shot[Pose::Parameter::TX], -shot[Pose::Parameter::TY],
                     -shot[Pose::Parameter::TZ]};
    const T Rt[3] = {-shot[Pose::Parameter::RX], -shot[Pose::Parameter::RY],
                     -shot[Pose::Parameter::RZ]};
    T translation[3];
    ceres::AngleAxisRotatePoint(Rt, mt, translation);

    residuals[0] = T(scale_) * (T(translation_prior_[0]) - translation[0]);
    residuals[1] = T(scale_) * (T(translation_prior_[1]) - translation[1]);
    residuals[2] = T(scale_) * (T(translation_prior_[2]) - translation[2]);
    return true;
  }

  double* translation_prior_;
  double scale_;
};

struct PositionPriorError {
  PositionPriorError(double* position_prior, double std_deviation)
      : position_prior_(position_prior), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const shot, const T* const bias,
                  T* residuals) const {
    Eigen::Map<const Vec3<T>> R(bias + Bias::Parameter::RX);
    Eigen::Map<const Vec3<T>> t(bias + Bias::Parameter::TX);
    const T* const scale = bias + Bias::Parameter::SCALE;

    Eigen::Map<Vec3d> prior(position_prior_);
    Eigen::Map<Vec3<T>> res(residuals);
    Eigen::Map<const Vec3<T>> optical_center(shot + Pose::Parameter::TX);

    res = T(scale_) *
          (optical_center -
           (scale[0] * RotatePoint(R.eval(), prior.cast<T>().eval()) + t));

    return true;
  }

  double* position_prior_;
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
  PointPositionPriorError(double* position, double std_deviation)
      : position_(position), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const p, T* residuals) const {
    residuals[0] = T(scale_) * (p[0] - T(position_[0]));
    residuals[1] = T(scale_) * (p[1] - T(position_[1]));
    residuals[2] = T(scale_) * (p[2] - T(position_[2]));
    return true;
  }

  double* position_;
  double scale_;
};

struct HeatmapdCostFunctor {
  explicit HeatmapdCostFunctor(
      const ceres::BiCubicInterpolator<ceres::Grid2D<double>>& interpolator,
      double x_offset, double y_offset, double height, double width,
      double resolution, double std_deviation)
      : interpolator_(interpolator),
        x_offset_(x_offset),
        y_offset_(y_offset),
        height_(height),
        width_(width),
        resolution_(resolution),
        scale_(1. / std_deviation) {}

  template <typename T>
  bool operator()(T const* p, T* residuals) const {
    const T x_coor = p[Pose::Parameter::TX] - x_offset_;
    const T y_coor = p[Pose::Parameter::TY] - y_offset_;
    // const T z_coor = x[2]; - Z goes brrrrr
    const T row = height_ / 2. - (y_coor / resolution_);
    const T col = width_ / 2. + (x_coor / resolution_);
    interpolator_.Evaluate(row, col, residuals);
    residuals[0] *= scale_;
    return true;
  }

  static ceres::CostFunction* Create(
      const ceres::BiCubicInterpolator<ceres::Grid2D<double>>& interpolator,
      double x_offset, double y_offset, double height, double width,
      double heatmap_resolution, double std_deviation) {
    return new ceres::AutoDiffCostFunction<HeatmapdCostFunctor, 1, 6>(
        new HeatmapdCostFunctor(interpolator, x_offset, y_offset, height, width,
                                heatmap_resolution, std_deviation));
  }

 private:
  const ceres::BiCubicInterpolator<ceres::Grid2D<double>>& interpolator_;
  const double x_offset_;
  const double y_offset_;
  const double height_;
  const double width_;
  const double resolution_;
  const double scale_;
};
}  // namespace bundle
