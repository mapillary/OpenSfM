#pragma once

#include <sfm/types.h>
#include <Eigen/Dense>

struct Keypoint {
  Keypoint() = default;
  Keypoint(double x, double y, double s) : point(x, y), scale(s), color(0, 0, 0), id(0) {}
  bool operator==(const Keypoint& k) const {
    return point == k.point && scale == k.scale && color == k.color &&
           id == k.id;
  }
  Eigen::Vector2d point;
  double scale{1.};
  Eigen::Vector3i color;
  int id{0};
};