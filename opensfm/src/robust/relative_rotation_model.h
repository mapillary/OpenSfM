#pragma once

#include <foundation/numeric.h>
#include <geometry/transform.h>
#include "model.h"

class RelativeRotation : public Model<RelativeRotation, 1, 1> {
 public:
  using Error = typename Model<RelativeRotation, 1, 1>::Error;
  using Type = Eigen::Matrix3d;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 3;

  static double ThresholdAdapter(const double threshold_angle) {
    return 1.0 - std::cos(threshold_angle);
  }

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models) {
    models[0] = RotationBetweenPoints(begin, end).transpose();
    return 1;
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models) {
    models[0] = RotationBetweenPoints(begin, end).transpose();
    return 1;
  }

  static Error Evaluate(const Type& model, const Data& d) {
    Error e;
    e[0] = 1.0 - (model * d.first).dot(d.second);
    return e;
  }
};
