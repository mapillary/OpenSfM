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

  Observation(double x, double y, double s, uint8_t r, uint8_t g, uint8_t b, size_t id,
            float _angle, float _resp, float _size, int _class_id)
  : point(x, y), scale(s), color(r, g, b), id(id),
    angle(_angle), response(_resp), size(_size), class_id(_class_id)
    { }
  
  float angle;
  float response;
  // int octave; //this is probably the scale
  float size;
  int class_id;
};