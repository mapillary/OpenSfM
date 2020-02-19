#pragma once


#include "model.h"
#include <foundation/numeric.h>
#include <geometry/absolute_pose.h>


class AbsolutePose : public Model<AbsolutePose, 1, 4> {
 public:
  using Error = typename Model<AbsolutePose, 1, 4>::Error;
  using Type = Eigen::Matrix<double, 3, 4>;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 3;

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models){
    const auto poses = AbsolutePoseThreePoints(begin, end);
    for(int i = 0; i < poses.size(); ++i){
      models[i] = poses[i];
    }
    return poses.size();
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models){
    const auto poses = AbsolutePoseThreePoints(begin, end);
    for(int i = 0; i < poses.size(); ++i){
      models[i] = poses[i];
    }
    return poses.size();
  }

  static Error Evaluate(const Type& model, const Data& d){
    const auto rotation = model.block<3, 3>(0, 0);
    const auto translation = model.block<3, 1>(0, 3);
    const auto bearing = d.first.normalized();
    const auto point = d.second;
    const auto projected = (rotation*point+translation).normalized();

    Error e;
    e[0] = std::acos(bearing.dot(projected));
    return e;
  }
};