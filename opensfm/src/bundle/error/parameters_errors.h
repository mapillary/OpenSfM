#pragma once

#include "ceres/ceres.h"

namespace bundle {

struct StdDeviationConstraint {
  StdDeviationConstraint() = default;

  template <typename T>
  bool operator()(const T *const std_deviation, T *residuals) const {
    T std = std_deviation[0];
    residuals[0] = ceres::log(T(1.0) / ceres::sqrt(T(2.0 * M_PI) * std * std));
    return true;
  }
};

struct ParameterBarrier {
  ParameterBarrier(double lower_bound, double upper_bound, int index)
      : lower_bound_(lower_bound), upper_bound_(upper_bound), index_(index) {}

  template <typename T>
  bool operator()(const T *const parameters, T *residuals) const {
    T eps = T(1e-10);
    T value = parameters[index_];
    T zero = 2.0 * ceres::log((T(upper_bound_) - T(lower_bound_)) * 0.5);
    T penalty = ceres::log(value - T(lower_bound_) + eps) +
                ceres::log(T(upper_bound_) - value + eps);
    residuals[0] = penalty + zero;
    return true;
  }

  double lower_bound_;
  double upper_bound_;
  int index_;
};
}  // namespace bundle
