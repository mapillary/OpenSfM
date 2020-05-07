#pragma once

#include <geometry/camera_functions.h>

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

  // This is where the pseudo-strategy pattern takes place.
  template <class OUT, class FUNC, class... IN>
  OUT Dispatch(IN... args) const;

  struct ProjectT {
    template <class PROJ, class DISTO, class AFF>
    static Eigen::Vector2d Apply(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& projection,
                                 const Eigen::Matrix2d& affine,
                                 const Eigen::Vector2d& principal_point,
                                 const Eigen::VectorXd& distortion) {
      return AFF::Forward(
          DISTO::Forward(PROJ::Forward(point, projection), distortion), affine,
          principal_point);
    }
  };

  struct BearingT {
    template <class PROJ, class DISTO, class AFF>
    static Eigen::Vector3d Apply(const Eigen::Vector2d& point,
                                 const Eigen::VectorXd& projection,
                                 const Eigen::Matrix2d& affine,
                                 const Eigen::Vector2d& principal_point,
                                 const Eigen::VectorXd& distortion) {
      return PROJ::Backward(
          DISTO::Backward(AFF::Backward(point, affine, principal_point),
                          distortion),
          projection);
    }
  };

  Type type_;
  Eigen::VectorXd projection_;      // dual transition (1 = perspective, 0 = fisheye)
  Eigen::Matrix2d affine_;          // fx 0, 0, fy
  Eigen::Vector2d principal_point_; // cx, cy
  Eigen::VectorXd distortion_;      // r^2, r^4, r^6, p1, p2
};