#pragma once

#include <map/defines.h>

#include <Eigen/Dense>

namespace map {
struct Observation {
  Observation() = default;
  Observation(double x, double y, double s, int r, int g, int b, int feature,
              int segmentation = NO_SEMANTIC_VALUE,
              int instance = NO_SEMANTIC_VALUE)
      : point(x, y),
        scale(s),
        color(r, g, b),
        feature_id(feature),
        segmentation_id(segmentation),
        instance_id(instance) {}
  bool operator==(const Observation& k) const {
    return point == k.point && scale == k.scale && color == k.color &&
           feature_id == k.feature_id && segmentation_id == k.segmentation_id &&
           instance_id == k.instance_id;
  }

  // Mandatory data
  Eigen::Vector2d point;
  double scale{1.};
  Eigen::Vector3i color;
  int feature_id{0};

  // Optional data : semantics
  int segmentation_id;
  int instance_id;
  static constexpr int NO_SEMANTIC_VALUE = -1;
};
}  // namespace map
