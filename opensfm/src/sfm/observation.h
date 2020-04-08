#pragma once

#include <sfm/types.h>
#include <Eigen/Dense>

struct Observation {
  Observation() = default;
  Observation(double x, double y, double s, int r, int g, int b, int id)
      : point(x, y), scale(s), color(r, g, b), id(id) {}
  bool operator==(const Observation& k) const {
    return point == k.point && scale == k.scale && color == k.color &&
           id == k.id;
  }
  Eigen::Vector2d point;
  double scale{1.};
  Eigen::Vector3i color;
  int id{0};
};