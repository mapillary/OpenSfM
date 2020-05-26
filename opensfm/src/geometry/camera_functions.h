#pragma once

#include <foundation/newton_raphson.h>
#include <foundation/types.h>

#include <iostream>

enum class ProjectionType { PERSPECTIVE, BROWN, FISHEYE, SPHERICAL, DUAL };
enum class Disto { K1 = 0, K2 = 1, K3 = 2, P1 = 3, P2 = 4, COUNT = 5 };

struct FisheyeProjection {
  template <class T>
  static Vec2<T> Forward(const Vec3<T>& point, const VecX<T>& p) {
    const auto r = point.template head<2>().norm();
    const auto theta = atan2(r, point[2]);
    return Vec2<T>(theta / r * point[0], theta / r * point[1]);
  }

  template <class T>
  static Vec3<T> Backward(const Vec2<T>& point, const VecX<T>& p) {
    const auto theta = point.norm();
    const auto s = tan(theta) / theta;
    return Vec3<T>(point[0] * s, point[1] * s, 1.0).normalized();
  }
};

struct PerspectiveProjection {
  template <class T>
  static Vec2<T> Forward(const Vec3<T>& point, const VecX<T>& p) {
    return Vec2<T>(point[0] / point[2], point[1] / point[2]);
  }

  template <class T>
  static Vec3<T> Backward(const Vec2<T>& point, const VecX<T>& p) {
    return Vec3<T>(point[0], point[1], 1.0).normalized();
  }
};

struct DualProjection {
  template <class T>
  static Vec2<T> Forward(const Vec3<T>& point, const VecX<T>& p) {
    const auto p_persp = PerspectiveProjection::Forward(point, p);
    const auto p_fish = FisheyeProjection::Forward(point, p);
    return p[0] * p_persp + (1.0 - p[0]) * p_fish;
  }

  template <class T>
  static Vec3<T> Backward(const Vec2<T>& point, const VecX<T>& p) {
    // Perform a bit iterations for finding theta from r
    const auto r = point.norm();
    ThetaEval<T> eval_function{0, r, p[0]};
    const auto theta_refined =
        NewtonRaphson<ThetaEval<T>, 1, 1, ManualDiff<ThetaEval<T>, 1, 1>>(
            eval_function, 0, iterations);

    const auto s = tan(theta_refined) / (p[0] * tan(theta_refined) +
                                              (1.0 - p[0]) * theta_refined);
    return Vec3<T>(point[0] * s, point[1] * s, 1.0).normalized();
  }

  static constexpr int iterations = 5;
  template <class T>
  struct ThetaEval {
    mutable int count;
    const T& r;
    const T& transition;
    T operator()(const T& x) const {
      return transition * tan(x) + (1.0 - transition) * x - r;
    }
    T derivative(const T& x) const {
      /* Here's some trick : use a half shorter step to prevent gross
       * overfitting on tan(x) */
      const T mult = count++ == 0 ? T(2.0) : T(1.0);
      const auto secant = 1.0 / cos(x);
      return mult * (transition * secant * secant - transition + 1);
    }
  };
};

struct SphericalProjection {
  template <class T>
  static Vec2<T> Forward(const Vec3<T>& point, const VecX<T>& p) {
    const auto lon = atan2(point[0], point[2]);
    const auto lat = atan2(-point[1], hypot(point[0], point[2]));
    return Vec2<T>(lon / (2 * M_PI), -lat / (2 * M_PI));
  }

  template <class T>
  static Vec3<T> Backward(const Vec2<T>& point, const VecX<T>& p) {
    const auto lon = point[0] * 2 * M_PI;
    const auto lat = -point[1] * 2 * M_PI;
    return Vec3<T>(cos(lat) * sin(lon), -sin(lat),
                    cos(lat) * cos(lon));
  }
};

struct Disto24 {
  template <class T>
  static Vec2<T> Forward(const Vec2<T>& point, const VecX<T>& k) {
    const auto r2 = point.dot(point);
    const auto distortion = Distortion(r2, k[static_cast<int>(Disto::K1)],
                                       k[static_cast<int>(Disto::K2)]);
    return point * distortion;
  }

  template <class T>
  static Vec2<T> Backward(const Vec2<T>& point, const VecX<T>& k) {
    /* Beware if you use Backward together with autodiff. You'll need to remove
     * the line below, otherwise, derivatives won't be propagated */
    if (k.norm() < T(std::numeric_limits<double>::epsilon())) {
      return point;
    }

    // Compute undistorted radius
    auto rd = point.norm();
    DistoEval<T> eval_function{rd, k[static_cast<int>(Disto::K1)],
                               k[static_cast<int>(Disto::K2)]};
    const auto ru_refined =
        NewtonRaphson<DistoEval<T>, 1, 1, ManualDiff<DistoEval<T>, 1, 1>>(
            eval_function, rd, iterations);

    // Compute distortion factor from undistorted radius
    const auto r2 = ru_refined * ru_refined;
    const auto distortion = Distortion(r2, k[static_cast<int>(Disto::K1)],
                                       k[static_cast<int>(Disto::K2)]);

    // Unapply undistortion
    return point / distortion;
  }

  static constexpr int iterations = 10;
  template <class T>
  struct DistoEval {
    const T& rd;
    const T& k1;
    const T& k2;
    T operator()(const T& x) const {
      const auto r = x;
      const auto r2 = r * r;
      return r * Disto24::Distortion(r2, k1, k2) - rd;
    }
    T derivative(const T& x) const {
      const auto r = x;
      const auto r2 = r * r;
      return Disto24::DistortionDerivative(r2, k1, k2);
    }
  };

  template <class T>
  static T Distortion(const T& r2, const T& k1, const T& k2) {
    return T(1.0) + r2 * (k1 + k2 * r2);
  }

  template <class T>
  static T DistortionDerivative(const T& r2, const T& k1, const T& k2) {
    return T(1.0) + r2 * T(2.0) * (k1 + T(2.0) * k2 * r2);
  }
};

struct DistoBrown {
  template <class T>
  static Vec2<T> Forward(const Vec2<T>& point, const VecX<T>& k) {
    const auto r2 = point.dot(point);
    const auto distortion_radial = RadialDistortion(
        r2, k[static_cast<int>(Disto::K1)], k[static_cast<int>(Disto::K2)],
        k[static_cast<int>(Disto::K3)]);
    const auto distortion_tangential = TangentialDistortion(
        r2, point[0], point[1], k[static_cast<int>(Disto::P1)],
        k[static_cast<int>(Disto::P2)]);
    return point * distortion_radial + distortion_tangential;
  }

  template <class T>
  static Vec2<T> Backward(const Vec2<T>& point, const VecX<T>& k) {
    /* Beware if you use Backward together with autodiff. You'll need to remove
     * the line below, otherwise, derivatives won't be propagated */
    if (k.norm() < T(std::numeric_limits<double>::epsilon())) {
      return point;
    }

    DistoEval<T> eval_function{point,
                               k[static_cast<int>(Disto::K1)],
                               k[static_cast<int>(Disto::K2)],
                               k[static_cast<int>(Disto::K3)],
                               k[static_cast<int>(Disto::P1)],
                               k[static_cast<int>(Disto::P2)]};
    return NewtonRaphson<DistoEval<T>, 2, 2, ManualDiff<DistoEval<T>, 2, 2>>(
        eval_function, point, iterations);
  }

  /* Undistort using Newton iterations. Sorry for the analytical derivatives,
   * there no real alternative. Jet/Dual number would kill the performance and
   * finite differencing is so inaccurate. */
  static constexpr int iterations = 10;

  template <class T>
  struct DistoEval {
    const Vec2<T>& point_distorted;
    const T& k1;
    const T& k2;
    const T& k3;
    const T& p1;
    const T& p2;

    Vec2<T> operator()(const Vec2<T>& point) const {
      const auto r2 = point.dot(point);
      const auto distortion_radial = RadialDistortion(r2, k1, k2, k3);
      const auto distortion_tangential =
          TangentialDistortion(r2, point[0], point[1], p1, p2);
      return point * distortion_radial + distortion_tangential -
             point_distorted;
    }

    Mat2<T> derivative(const Vec2<T>& point) const {
      const auto x = point[0];
      const auto y = point[1];
      const auto r2 = point.squaredNorm();
      const T r4 = r2 * r2;
      const T r6 = r4 * r2;
      const T x2 = x * x;
      const T x4 = x2 * x2;
      const T y2 = y * y;
      const T y4 = y2 * y2;

      const auto dxx = T(5.0) * k2 * x4 + T(3.0) * k1 * x2 +
                       T(6.0) * k3 * x2 * r4 + T(6.0) * k2 * x2 * y2 + k3 * r6 +
                       k2 * y4 + k1 * y2 + T(1.0) + T(2.0) * p1 * y +
                       T(6.0) * p2 * x;
      const auto dxy =
          x * (T(2.0) * k1 * y + T(4.0) * k2 * y * r2 + T(6.0) * k3 * y * r4) +
          T(2.0) * p1 * x + T(2.0) * p2 * y;

      const auto dyy = T(5.0) * k2 * y4 + T(3.0) * k1 * y2 +
                       T(6.0) * k3 * y2 * r4 + T(6.0) * k2 * x2 * y2 + k3 * r6 +
                       k2 * x4 + k1 * x2 + T(1.0) + T(2.0) * p2 * x +
                       T(6.0) * p1 * y;
      const auto dyx =
          y * (T(2.0) * k1 * x + T(4.0) * k2 * x * r2 + T(6.0) * k3 * x * r4) +
          T(2.0) * p2 * y + T(2.0) * p1 * x;

      Mat2<T> jacobian;
      jacobian << dxx, dxy, dyx, dyy;
      return jacobian;
    }
  };

  template <class T>
  static T RadialDistortion(const T& r2, const T& k1, const T& k2,
                            const T& k3) {
    return T(1.0) + r2 * (k1 + r2 * (k2 + r2 * k3));
  }
  template <class T>
  static Vec2<T> TangentialDistortion(const T& r2, const T& x, const T& y,
                                       const T& p1, const T& p2) {
    return Vec2<T>(T(2.0) * p1 * x * y + p2 * (r2 + T(2.0) * x * x),
                    T(2.0) * p2 * x * y + p1 * (r2 + T(2.0) * y * y));
  }
};

struct Affine {
  template <class T>
  static Vec2<T> Forward(const Vec2<T>& point, const Mat2<T>& affine,
                          const Vec2<T>& shift) {
    return affine * point + shift;
  }

  template <class T>
  static Vec2<T> Backward(const Vec2<T>& point, const Mat2<T>& affine,
                           const Vec2<T>& shift) {
    return affine.inverse() * (point - shift);
  }
};

struct Identity {
  template <class T, class... Types>
  static Vec2<T> Forward(const Vec2<T>& point, Types&&... args) {
    return point;
  }

  template <class T, class... Types>
  static Vec2<T> Backward(const Vec2<T>& point, Types&&... args) {
    return point;
  }
};

struct ProjectFunction {
  template <class TYPE, class T>
  static Vec2<T> Apply(const Vec3<T>& point, const VecX<T>& projection,
                        const Mat2<T>& affine, const Vec2<T>& principal_point,
                        const VecX<T>& distortion) {
    return TYPE::Forward(point, projection, affine, principal_point,
                         distortion);
  }
};

struct BearingFunction {
  template <class TYPE, class T>
  static Vec3<T> Apply(const Vec2<T>& point, const VecX<T>& projection,
                        const Mat2<T>& affine, const Vec2<T>& principal_point,
                        const VecX<T>& distortion) {
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
  template <class T>
  static Vec2<T> Forward(const Vec3<T>& point, const VecX<T>& projection,
                          const Mat2<T>& affine,
                          const Vec2<T>& principal_point,
                          const VecX<T>& distortion) {
    return AFF::Forward(
        DISTO::Forward(PROJ::Forward(point, projection), distortion), affine,
        principal_point);
  };

  template <class T>
  static Vec3<T> Backward(const Vec2<T>& point, const VecX<T>& projection,
                           const Mat2<T>& affine,
                           const Vec2<T>& principal_point,
                           const VecX<T>& distortion) {
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
    case ProjectionType::PERSPECTIVE:
      return FUNC::template Apply<PerspectiveCameraT>(std::forward<IN>(args)...);
    case ProjectionType::BROWN:
      return FUNC::template Apply<BrownCameraT>(std::forward<IN>(args)...);
    case ProjectionType::FISHEYE:
      return FUNC::template Apply<FisheyeCameraT>(std::forward<IN>(args)...);
    case ProjectionType::DUAL:
      return FUNC::template Apply<DualCameraT>(std::forward<IN>(args)...);
    case ProjectionType::SPHERICAL:
      return FUNC::template Apply<SphericalCameraT>(std::forward<IN>(args)...);
    default:
      throw std::runtime_error("Invalid ProjectionType");
  }
};