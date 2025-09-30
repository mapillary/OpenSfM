#pragma once

#include <Eigen/Core>
#include <unsupported/Eigen/AutoDiff>
#include <cmath>

// Compatibility overloads for Eigen::AutoDiffScalar with Ceres 2.2+
// These are needed because Ceres rotation.h uses std::hypot and std::fpclassify
// which don't have overloads for AutoDiffScalar types

namespace Eigen {
// 3-argument hypot for AutoDiffScalar (used in ceres/rotation.h)
template<typename DerType>
inline AutoDiffScalar<DerType> hypot(const AutoDiffScalar<DerType>& x,
                                      const AutoDiffScalar<DerType>& y,
                                      const AutoDiffScalar<DerType>& z) {
  return sqrt(x*x + y*y + z*z);
}
}

namespace std {
// fpclassify for AutoDiffScalar (used in ceres/rotation.h)
template<typename DerType>
inline int fpclassify(const Eigen::AutoDiffScalar<DerType>& x) {
  return std::fpclassify(x.value());
}
}
