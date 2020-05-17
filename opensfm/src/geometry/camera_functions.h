#pragma once

#include <foundation/newton_raphson.h>

#include <Eigen/Eigen>
#include <iostream>

enum ProjectionType { PERSPECTIVE, BROWN, FISHEYE, SPHERICAL, DUAL };
enum Disto { K1 = 0, K2 = 1, K3 = 2, P1 = 3, P2 = 4, COUNT = 5 };

struct FisheyeProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& p) {
    const auto r = point.head<2>().norm();
    const auto theta = std::atan2(r, point[2]);
    return Eigen::Vector2d(theta / r * point[0], theta / r * point[1]);
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& p) {
    const auto theta = point.norm();
    const auto s = std::tan(theta) / theta;
    return Eigen::Vector3d(point[0] * s, point[1] * s, 1.0).normalized();
  }
};

struct PerspectiveProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& p) {
    return Eigen::Vector2d(point[0] / point[2], point[1] / point[2]);
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& p) {
    return Eigen::Vector3d(point[0], point[1], 1.0).normalized();
  }
};

struct DualProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& p) {
    const auto p_persp = PerspectiveProjection::Forward(point, p);
    const auto p_fish = FisheyeProjection::Forward(point, p);
    return p[0] * p_persp + (1.0 - p[0]) * p_fish;
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& p) {
    // Perform a bit iterations for finding theta from r
    const auto r = point.norm();
    ThetaEval eval_function{0, r, p[0]};
    const auto theta_refined =
        NewtonRaphson<ThetaEval, 1, 1, ManualDiff<ThetaEval, 1, 1>>(
            eval_function, 0, iterations);

    const auto s = std::tan(theta_refined) / (p[0] * std::tan(theta_refined) +
                                              (1.0 - p[0]) * theta_refined);
    return Eigen::Vector3d(point[0] * s, point[1] * s, 1.0).normalized();
  }

  static constexpr int iterations = 5;
  struct ThetaEval {
    mutable int count;
    const double& r;
    const double& transition;
    double operator()(double x) const {
      return transition * std::tan(x) + (1.0 - transition) * x - r;
    }
    double derivative(double x) const {
      /* Here's some trick : use a half shorter step to prevent gross
       * overfitting on tan(x) */
      const double mult = count++ == 0 ? 2.0 : 1.0;
      const auto secant = 1.0 / std::cos(x);
      return mult * (transition * secant * secant - transition + 1);
    }
  };
};

struct SphericalProjection {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& p) {
    const auto lon = std::atan2(point[0], point[2]);
    const auto lat = std::atan2(-point[1], std::hypot(point[0], point[2]));
    return Eigen::Vector2d(lon / (2 * M_PI), -lat / (2 * M_PI));
  }

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& p) {
    const auto lon = point[0] * 2 * M_PI;
    const auto lat = -point[1] * 2 * M_PI;
    return Eigen::Vector3d(std::cos(lat) * std::sin(lon), -std::sin(lat),
                           std::cos(lat) * std::cos(lon));
  }
};

struct Disto24 {
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point,
                                 const Eigen::VectorXd& k) {
    if(k.norm() < std::numeric_limits<double>::epsilon()){
      return point;
    }
    const auto r2 = point.dot(point);
    const auto distortion = Distortion(r2, k[Disto::K1], k[Disto::K2]);
    return point * distortion;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& k) {
    if(k.norm() < std::numeric_limits<double>::epsilon()){
      return point;
    }

    // Compute undistorted radius
    auto rd = point.norm();
    DistoEval eval_function{rd, k[Disto::K1], k[Disto::K2]};
    const auto ru_refined =
        NewtonRaphson<DistoEval, 1, 1, ManualDiff<DistoEval, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const auto r2 = ru_refined * ru_refined;
    const auto distortion = Distortion(r2, k[Disto::K1], k[Disto::K2]);

    // Unapply undistortion
    return point / distortion;
  }

  static constexpr int iterations = 10;
  struct DistoEval {
    const double& rd;
    const double& k1;
    const double& k2;
    double operator()(double x) const {
      const auto r = x;
      const auto r2 = r * r;
      return r * Disto24::Distortion(r2, k1, k2) - rd;
    }
    double derivative(double x) const {
      const auto r = x;
      const auto r2 = r * r;
      return Disto24::DistortionDerivative(r2, k1, k2);
    }
  };

  static double Distortion(double r2, double k1, double k2) {
    return 1.0 + r2 * (k1 + k2 * r2);
  }
  static double DistortionDerivative(double r2, double k1, double k2) {
    return 1.0 + r2 * 2.0 * (k1 + 2.0 * k2 * r2);
  }
};

struct DistoBrown {
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point,
                                 const Eigen::VectorXd& k) {
    if(k.norm() < std::numeric_limits<double>::epsilon()){
      return point;
    }

    const auto r2 = point.dot(point);
    const auto distortion_radial = RadialDistortion(r2, k[Disto::K1], k[Disto::K2], k[Disto::K3]);
    const auto distortion_tangential =
        TangentialDistortion(r2, point[0], point[1], k[Disto::P1], k[Disto::P2]);
    return point * distortion_radial + distortion_tangential;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point,
                                  const Eigen::VectorXd& k) {
    if(k.norm() < std::numeric_limits<double>::epsilon()){
      return point;
    }

    DistoEval eval_function{point, k[Disto::K1], k[Disto::K2], k[Disto::K3], k[Disto::P1], k[Disto::P2]};
    return NewtonRaphson<DistoEval, 2, 2, ManualDiff<DistoEval, 2, 2>>(
        eval_function, point, iterations);
  }

  /* Undistort using Newton iterations. Sorry for the analytical derivatives,
   * there no real alternative. Jet/Dual number would kill the performance and
   * finite differencing is so inaccurate. */
  static constexpr int iterations = 10;
  struct DistoEval {
    const Eigen::Vector2d& point_distorted;
    const double& k1;
    const double& k2;
    const double& k3;
    const double& p1;
    const double& p2;
    Eigen::Vector2d operator()(const Eigen::Vector2d& point) const {
      const auto r2 = point.dot(point);
      const auto distortion_radial = RadialDistortion(r2, k1, k2, k3);
      const auto distortion_tangential =
          TangentialDistortion(r2, point[0], point[1], p1, p2);
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

  static double RadialDistortion(double r2, double k1, double k2, double k3) {
    return 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3));
  }
  static Eigen::Vector2d TangentialDistortion(double r2, double x, double y,
                                              double p1, double p2) {
    return Eigen::Vector2d(2.0 * p1 * x * y + p2 * (r2 + 2 * x * x),
                           2.0 * p2 * x * y + p1 * (r2 + 2 * y * y));
  }
};

struct Affine {
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point,
                                 const Eigen::Matrix2d& affine,
                                 const Eigen::Vector2d& shift) {
    return affine * point + shift;
  }

  static Eigen::Vector2d Backward(const Eigen::Vector2d& point,
                                  const Eigen::Matrix2d& affine,
                                  const Eigen::Vector2d& shift) {
    return affine.inverse() * (point - shift);
  }
};

struct Identity {
  template <class... Types>
  static Eigen::Vector2d Forward(const Eigen::Vector2d& point, Types&&... args) {
    return point;
  }

  template <class... Types>
  static Eigen::Vector2d Backward(const Eigen::Vector2d& point, Types&&... args) {
    return point;
  }
};

struct ProjectT {
  template <class TYPE>
  static Eigen::Vector2d Apply(const Eigen::Vector3d& point,
                               const Eigen::VectorXd& projection,
                               const Eigen::Matrix2d& affine,
                               const Eigen::Vector2d& principal_point,
                               const Eigen::VectorXd& distortion) {
    return TYPE::Forward(point, projection, affine, principal_point,
                         distortion);
  }
};

struct BearingT {
  template <class TYPE>
  static Eigen::Vector3d Apply(const Eigen::Vector2d& point,
                               const Eigen::VectorXd& projection,
                               const Eigen::Matrix2d& affine,
                               const Eigen::Vector2d& principal_point,
                               const Eigen::VectorXd& distortion) {
    return TYPE::Backward(point, projection, affine, principal_point,
                          distortion);
  }
};

/* This struct helps define most cameras models as they tend to follow the
 * pattern PROJ - > DISTO -> AFFINE. However, its is not mandatory for any
 * camera model to follow it. You can add any new camera models as long as it
 * implements the Forward and Backward functions. */
template <class PROJ, class DISTO, class AFF>
struct ProjectGeneric {
  static Eigen::Vector2d Forward(const Eigen::Vector3d& point,
                                 const Eigen::VectorXd& projection,
                                 const Eigen::Matrix2d& affine,
                                 const Eigen::Vector2d& principal_point,
                                 const Eigen::VectorXd& distortion) {
    return AFF::Forward(
        DISTO::Forward(PROJ::Forward(point, projection), distortion), affine,
        principal_point);
  };

  static Eigen::Vector3d Backward(const Eigen::Vector2d& point,
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

using PerspectiveCameraT = ProjectGeneric<PerspectiveProjection, Disto24, Affine>;
using BrownCameraT = ProjectGeneric<PerspectiveProjection, DistoBrown, Affine>;
using FisheyeCameraT = ProjectGeneric<FisheyeProjection, Disto24, Affine>;
using DualCameraT = ProjectGeneric<DualProjection, Disto24, Affine>;
using SphericalCameraT = ProjectGeneric<SphericalProjection, Identity, Identity>;

/* This is where the pseudo-strategy pattern takes place. If you want to add
 * your own new camera model, just add a new enum value, the corresponding
 * case below and the implementation (see above). */
template <class OUT, class FUNC, class... IN>
OUT Dispatch(const ProjectionType& type, IN&&... args) {
  switch (type) {
    case PERSPECTIVE:
      return FUNC::template Apply<PerspectiveCameraT>(std::forward<IN>(args)...);
    case BROWN:
      return FUNC::template Apply<BrownCameraT>(std::forward<IN>(args)...);
    case FISHEYE:
      return FUNC::template Apply<FisheyeCameraT>(std::forward<IN>(args)...);
    case DUAL:
      return FUNC::template Apply<DualCameraT>(std::forward<IN>(args)...);
    case SPHERICAL:
      return FUNC::template Apply<SphericalCameraT>(std::forward<IN>(args)...);
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};