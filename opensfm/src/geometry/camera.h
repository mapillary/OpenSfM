#pragma once

#include <foundation/newton_raphson.h>

#include <Eigen/Eigen>
#include <iostream>

template <class PROJ, class DISTO, class AFF>
struct CameraT {
  CameraT() {
    affine_.setIdentity();
    principal_point_.setZero();
    distorsion_.setZero();
  }
  CameraT(double focal_x, double focal_y,
          const Eigen::Vector2d& principal_point,
          const Eigen::Vector3d& radial_distorsion,
          const Eigen::Vector2d& tangential_distorsion)
      : principal_point_(principal_point) {
    affine_ << focal_x, 0, 0, focal_y;
    distorsion_.resize(5);
    distorsion_.head<3>() = radial_distorsion;
    distorsion_.tail<2>() = tangential_distorsion;
  }

  CameraT(double focal, double k1, double k2) {
    affine_ << focal, 0, 0, focal;
    distorsion_.resize(2);
    distorsion_ << k1, k2;
    principal_point_ << 0., 0.;
  }

  Eigen::Vector2d Project(const Eigen::Vector3d& point) const {
    return AFF::Forward(DISTO::Forward(PROJ::Forward(point), distorsion_),
                        affine_, principal_point_);
  }

  Eigen::MatrixX2d ProjectMany(const Eigen::MatrixX3d& points) const {
    Eigen::MatrixX2d projected(points.rows(), 2);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Project(points.row(i));
    }
    return projected;
  }

  Eigen::Vector3d Bearing(const Eigen::Vector2d& point) const {
    return PROJ::Backward(DISTO::Backward(
        AFF::Backward(point, affine_, principal_point_), distorsion_));
  }

  Eigen::MatrixX3d BearingsMany(const Eigen::MatrixX2d& points) const {
    Eigen::MatrixX3d projected(points.rows(), 3);
    for (int i = 0; i < points.rows(); ++i) {
      projected.row(i) = Bearing(points.row(i));
    }
    return projected;
  }

  Eigen::Matrix2d affine_;
  Eigen::Vector2d principal_point_;
  Eigen::VectorXd distorsion_;  // r^2, r^4, r^6, p1, p2
};

struct FisheyeProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point) {
    const auto r = point.head<2>().norm();
    const auto theta = std::atan2(r, point[2]);
    return Eigen::Vector2d(theta / r * point[0], theta / r * point[1]);
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point) {
    const auto r = point.norm();
    const auto s = std::tan(r) / r;
    return Eigen::Vector3d(point[0] * s, point[1] * s, 1.0).normalized();
  }
};

struct PerspectiveProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point) {
    return Eigen::Vector2d(point[0] / point[2], point[1] / point[2]);
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point) {
    return Eigen::Vector3d(point[0], point[1], 1.0).normalized();
  }
};

struct SphericalProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point) {
    const auto lon = std::atan2(point[0], point[2]);
    const auto lat = std::atan2(-point[1], std::hypot(point[0], point[2]));
    return Eigen::Vector2d(lon / (2 * M_PI), -lat / (2 * M_PI));
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point) {
    const auto lon = point[0] * 2 * M_PI;
    const auto lat = -point[1] * 2 * M_PI;
    return Eigen::Vector3d(std::cos(lat) * std::sin(lon), std::sin(lat),
                           std::cos(lat) * std::cos(lon));
  }
};

struct Disto24{
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point, const Eigen::VectorXd& k) {
    const auto r2 = point.dot(point);
    const auto distortion = Distorsion(r2, k[0], k[1]);
    return point * distortion;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& k) {
    // Compute undistorted radius
    auto rd = point.norm();
    DistoEval eval_function{rd, k[0], k[1]};
    const auto ru_refined =
        NewtonRaphson<DistoEval, 1, 1, ManualDiff<DistoEval, 1, 1>>(
            eval_function, rd, iterations);

    // Ccompute distorsion factor from undistorted radius
    const auto r2 = ru_refined * ru_refined;
    const auto distorsion = Distorsion(r2, k[0], k[1]);

    // Unapply undistorsion
    return point/distorsion;
  }

  static const int iterations = 20;
  struct DistoEval {
    const double& rd;
    const double& k1;
    const double& k2;
    double operator()(const double& x) const {
      const auto r = x;
      const auto r2 = r * r;
      return r * Disto24::Distorsion(r2, k1, k2) - rd;
    }
    double derivative(const double& x) const {
      const auto r = x;
      const auto r2 = r * r;
      return Disto24::DistorsionDerivative(r2, k1, k2);
    }
  };

  static double Distorsion(double r2, double k1, double k2) {
    return 1.0 + r2 * (k1 + k2 * r2);
  }
  static double DistorsionDerivative(double r2, double k1, double k2) {
    return 1.0 + r2 * 2.0 * (k1 + 2.0 * k2 * r2);
  }
};

struct DistoBrown{

  static Eigen::Vector2d Forward(const Eigen::Vector2d& point, const Eigen::VectorXd& k) {
    const auto r2 = point.dot(point);
    const auto distortion_radial = RadialDistorsion(r2, k[0], k[1], k[2]);
    const auto distortion_tangential =
        TangentialDistorsion(r2, point[0], point[1], k[3], k[4]);
    return point * distortion_radial + distortion_tangential;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point, const Eigen::VectorXd& k) {
    // Undistort using Newton iterations.
    // Sorry for the analytical derivatives,
    // there no real alternative. Jet/Dual number
    // would kill the performance and finite
    // differencing is so inaccurate.
    const int iterations = 20;
    struct DistoEval {
      const Eigen::Vector2d& point_distorted;
      const double& k1;
      const double& k2;
      const double& k3;
      const double& p1;
      const double& p2;
      Eigen::Vector2d operator()(const Eigen::Vector2d& point) const {
        const auto r2 = point.dot(point);
        const auto distortion_radial = RadialDistorsion(r2, k1, k2, k3);
        const auto distortion_tangential =
            TangentialDistorsion(r2, point[0], point[1], p1, p2);
        return point * distortion_radial + distortion_tangential -
               point_distorted;
      }
      Eigen::Matrix2d derivative(const Eigen::Vector2d& point) const {
        const auto x = point[0];
        const auto y = point[1];
        const auto r2 = point.squaredNorm();
        const double r4 = r2 * r2;
        const double r6 = r4 * r2;
        const double x2 = x * x;
        const double x4 = x2 * x2;
        const double y2 = y * y;
        const double y4 = y2 * y2;

        const auto dxx = 5 * k2 * x4 + 3 * k1 * x2 + 6 * k3 * x2 * r4 +
                         6 * k2 * x2 * y2 + k3 * r6 + k2 * y4 + k1 * y2 + 1 +
                         2 * p1 * y + 6 * p2 * x;
        const auto dxy = x * (2 * k1 * y + 4 * k2 * y * r2 + 6 * k3 * y * r4) +
                         2 * p1 * x + 2 * p2 * y;

        const auto dyy = 5 * k2 * y4 + 3 * k1 * y2 + 6 * k3 * y2 * r4 +
                         6 * k2 * x2 * y2 + k3 * r6 + k2 * x4 + k1 * x2 + 1 +
                         2 * p2 * x + 6 * p1 * y;
        const auto dyx = y * (2 * k1 * x + 4 * k2 * x * r2 + 6 * k3 * x * r4) +
                         2 * p2 * y + 2 * p1 * x;
        Eigen::Matrix2d jacobian;
        jacobian << dxx, dxy, dyx, dyy;
        return jacobian;
      }
    };
    DistoEval eval_function{point, k[0], k[1], k[2], k[3], k[4]};
    return  NewtonRaphson<DistoEval, 2, 2, ManualDiff<DistoEval, 2, 2>>(
            eval_function, point, iterations);
  }

  static double RadialDistorsion(double r2, double k1, double k2, double k3) {
    return 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
  }
  static Eigen::Vector2d TangentialDistorsion(double r2, double x, double y,
                                              double p1, double p2) {
    return Eigen::Vector2d(2.0 * p1 * x * y + p2 * (r2 + 2 * x * x),
                           2.0 * p2 * x * y + p1 * (r2 + 2 * y * y));
  }
};

struct Affine{
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point,
                                 const Eigen::Matrix2d& affine,
                                 const Eigen::Vector2d& shift) {
    return affine * point + shift;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point,
                                  const Eigen::Matrix2d& affine,
                                  const Eigen::Vector2d& shift) {
    return affine.inverse()*(point-shift);
  }
};

struct Identity {
  template <class... Types>
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point, Types... args) {
    return point;
  }

  template <class... Types>
  static Eigen::Vector2d Backward(const Eigen::Vector2d& point, Types... args) {
    return point;
  }
};

using FisheyeCamera = CameraT<FisheyeProjection, Disto24, Affine>;
using PerspectiveCamera = CameraT< PerspectiveProjection, Disto24, Affine>;
using BrownCamera = CameraT< PerspectiveProjection, DistoBrown, Affine>;
using SphericalCamera = CameraT< SphericalProjection, Identity, Identity>;