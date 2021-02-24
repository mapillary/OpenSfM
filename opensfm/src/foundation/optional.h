#pragma once

#include <Eigen/Eigen>

namespace foundation {
template <typename T>
class OptionalValue {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OptionalValue() = default;
  explicit OptionalValue(const T& value) { SetValue(value); }
  bool HasValue() const { return has_value_; }
  const T& Value() const { return value_; }
  T& Value() { return value_; }
  void SetValue(const T& v) {
    value_ = v;
    has_value_ = true;
  }
  void Reset() { has_value_ = false; }

 private:
  bool has_value_{false};
  T value_;
};
}  // namespace foundation
