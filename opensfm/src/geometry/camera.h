#pragma once

#include <Eigen/Eigen>

class Camera {
 public:
  enum Type { PERSPECTIVE, BROWN, FISHEYE, SPHERICAL, DUAL };

  static Camera CreatePerspective(double focal, double k1, double k2);
  static Camera CreateBrownCamera(double focal_x, double focal_y,
                                  const Eigen::Vector2d& principal_point,
                                  const Eigen::VectorXd& distortion);
  static Camera CreateFisheyeCamera(double focal, double k1, double k2);
  static Camera CreateDualCamera(double transition, double focal, double k1, double k2);
  static Camera CreateSphericalCamera();

  Eigen::Vector2d Project(const Eigen::Vector3d& point) const;
  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) const;

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) const;
  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) const;

 private:
  Camera();

  Type type_;
  Eigen::VectorXd projection_;      // dual transition (1 = perspective, 0 = fisheye)
  Eigen::Matrix2d affine_;          // fx 0, 0, fy
  Eigen::Vector2d principal_point_; // cx, cy
  Eigen::VectorXd distortion_;      // r^2, r^4, r^6, p1, p2
};