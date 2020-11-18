#pragma once

#include <foundation/numeric.h>
#include <geometry/relative_pose.h>
#include <geometry/triangulation.h>
#include "essential_model.h"
#include "model.h"

class RelativePose : public Model<RelativePose, 1, 10> {
 public:
  using Error = typename Model<RelativePose, 1, 10>::Error;
  using Type = Eigen::Matrix<double, 3, 4>;
  using Data = std::pair<Eigen::Vector3d, Eigen::Vector3d>;
  static const int MINIMAL_SAMPLES = 5;

  static double ThresholdAdapter(const double threshold_angle) {
    return 1.0 - std::cos(threshold_angle);
  }

  template <class IT>
  static int Estimate(IT begin, IT end, Type* models) {
    const auto essentials = EssentialFivePoints(begin, end);
    for (int i = 0; i < essentials.size(); ++i) {
      models[i] = RelativePoseFromEssential(essentials[i], begin, end);
    }
    return essentials.size();
  }

  template <class IT>
  static int EstimateNonMinimal(IT begin, IT end, Type* models) {
    const auto essentials = EssentialNPoints(begin, end);
    for (int i = 0; i < essentials.size(); ++i) {
      models[i] = RelativePoseFromEssential(essentials[i], begin, end);
    }
    return essentials.size();
  }

  static Error Evaluate(const Type& model, const Data& d) {
    const auto rotation = model.block<3, 3>(0, 0);
    const auto translation = model.block<3, 1>(0, 3);
    const auto x = d.first.normalized();
    const auto y = d.second.normalized();

    Eigen::Matrix<double, 2, 3> bearings;
    Eigen::Matrix<double, 2, 3> centers;
    centers.row(0) = Eigen::Vector3d::Zero();
    centers.row(1) = -rotation.transpose() * translation;
    bearings.row(0) = x;
    bearings.row(1) = rotation.transpose() * y;
    const auto point =
        geometry::TriangulateTwoBearingsMidpointSolve(centers, bearings);

    Error e;
    if (!point.first) {
      e[0] = 1.0;
    } else {
      const auto projected_x = point.second.normalized();
      const auto projected_y =
          (rotation * point.second + translation).normalized();
      e[0] = 1.0 - ((projected_x.dot(x) + projected_y.dot(y)) * 0.5);
    }
    return e;
  }
};
