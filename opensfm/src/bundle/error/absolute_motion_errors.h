#pragma once

#include <bundle/bundle_adjuster.h>
#include <bundle/error/error_utils.h>
#include <bundle/error/position_functors.h>

#include <Eigen/Eigen>

namespace bundle {
struct UpVectorError {
  UpVectorError(const Vec3d& acceleration, double std_deviation)
      : scale_(1.0 / std_deviation) {
    acceleration_ = acceleration.normalized();
  }

  template <typename T>
  bool operator()(const T* const rig_instance, const T* const rig_camera,
                  T* residuals) const {
    T const* const params[] = {rig_instance, rig_camera};
    Vec3<T> R = ShotRotationFunctor(0, 1)(params);
    Eigen::Map<Vec3<T>> residual(residuals);

    const Vec3<T> acceleration = acceleration_.cast<T>();
    const Vec3<T> z_world = RotatePoint(R, acceleration);
    const Vec3<T> z_axis = Vec3d(0, 0, 1).cast<T>();
    residual = T(scale_) * (z_world - z_axis);
    return true;
  }

  Vec3d acceleration_;
  const double scale_;
};

struct PanAngleError {
  PanAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const rig_instance, const T* const rig_camera,
                  T* residuals) const {
    T const* const params[] = {rig_instance, rig_camera};
    Vec3<T> R = ShotRotationFunctor(0, 1)(params);

    const Vec3<T> z_axis = Vec3d(0, 0, 1).cast<T>();
    const auto z_world = RotatePoint(R, z_axis);

    if (ceres::abs(z_world(0)) < T(1e-8) && ceres::abs(z_world(1)) < T(1e-8)) {
      residuals[0] = T(0.0);
    } else {
      const T predicted_angle = atan2(z_world(0), z_world(1));
      residuals[0] = T(scale_) * DiffBetweenAngles(predicted_angle, T(angle_));
    }
    return true;
  }

  const double angle_;
  const double scale_;
};

struct TiltAngleError {
  TiltAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const rig_instance, const T* const rig_camera,
                  T* residuals) const {
    T const* const params[] = {rig_instance, rig_camera};
    Vec3<T> R = ShotRotationFunctor(0, 1)(params);
    T ez[3] = {T(0), T(0), T(1)};  // ez: A point in front of the camera (z=1)
    T Rt_ez[3];
    ceres::AngleAxisRotatePoint(R.data(), ez, Rt_ez);

    T l = sqrt(Rt_ez[0] * Rt_ez[0] + Rt_ez[1] * Rt_ez[1]);
    T predicted_angle = -atan2(Rt_ez[2], l);

    residuals[0] = T(scale_) * DiffBetweenAngles(predicted_angle, T(angle_));
    return true;
  }

  const double angle_;
  const double scale_;
};

struct RollAngleError {
  RollAngleError(double angle, double std_deviation)
      : angle_(angle), scale_(1.0 / std_deviation) {}

  template <typename T>
  bool operator()(const T* const rig_instance, const T* const rig_camera,
                  T* residuals) const {
    T const* const params[] = {rig_instance, rig_camera};
    Vec3<T> R = ShotRotationFunctor(0, 1)(params);
    T ex[3] = {T(1), T(0), T(0)};  // A point to the right of the camera (x=1)
    T ez[3] = {T(0), T(0), T(1)};  // A point in front of the camera (z=1)
    T Rt_ex[3], Rt_ez[3];
    T tangle_ = T(angle_);
    ceres::AngleAxisRotatePoint(R.data(), ex, Rt_ex);
    ceres::AngleAxisRotatePoint(R.data(), ez, Rt_ez);

    T a[3] = {Rt_ez[1], -Rt_ez[0], T(0)};
    T la = sqrt(a[0] * a[0] + a[1] * a[1]);

    const double eps = 1e-5;
    if (la < eps) {
      residuals[0] = T(0.0);
      return true;
    }

    a[0] /= la;
    a[1] /= la;
    T b[3];
    ceres::CrossProduct(Rt_ex, a, b);
    T sin_roll = Rt_ez[0] * b[0] + Rt_ez[1] * b[1] + Rt_ez[2] * b[2];
    if (sin_roll <= -(1.0 - eps)) {
      residuals[0] = T(0.0);
      return true;
    }

    T predicted_angle = asin(sin_roll);
    residuals[0] = T(scale_) * DiffBetweenAngles(predicted_angle, T(angle_));

    return true;
  }

  const double angle_;
  const double scale_;
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
  bool operator()(const T* const rig_instance, const T* const rig_camera,
                  T* residuals) const {
    T const* const params[] = {rig_instance, rig_camera};
    Vec3<T> position = ShotPositionFunctor(0, 1)(params);
    const T x_coor = position[0] - x_offset_;
    const T y_coor = position[1] - y_offset_;
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
    return new ceres::AutoDiffCostFunction<HeatmapdCostFunctor, 1, 6, 6>(
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

struct TranslationPriorError {
  explicit TranslationPriorError(const double prior_norm)
      : prior_norm_(prior_norm) {}

  template <typename T>
  bool operator()(const T* const rig_instance1, const T* const rig_instance2,
                  T* residuals) const {
    const auto t1 = Eigen::Map<const Vec3<T>>(rig_instance1 + Pose::Parameter::TX);
    const auto t2 = Eigen::Map<const Vec3<T>>(rig_instance2 + Pose::Parameter::TX);
    residuals[0] = log((t1 - t2).norm() / T(prior_norm_));
    return true;
  }

  const double prior_norm_;
};
}  // namespace bundle
