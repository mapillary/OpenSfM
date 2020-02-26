#pragma once


#include "model.h"
#include <foundation/numeric.h>
#include <geometry/absolute_pose.h>


class AbsolutePoseKnownRotation : public Model<AbsolutePoseKnownRotation, 1, 1> {
 public:
  using Error = typename Model<AbsolutePoseKnownRotation, 1, 1>::Error;
  using Type = Eigen::Vector3d;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 2;

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models){
    models[0] = AbsolutePoseNPointsKnownRotation(begin, end);
    return 1;
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models){
    models[0] = AbsolutePoseNPointsKnownRotation(begin, end);
    return 1;
  }

  static Error Evaluate(const Type& model, const Data& d){
    const auto translation = model;
    const auto bearing = d.first.normalized();
    const auto point = d.second;
    const auto projected = (point+translation).normalized();

    Error e;
    e[0] = std::acos(bearing.dot(projected));
    return e;
  }
};